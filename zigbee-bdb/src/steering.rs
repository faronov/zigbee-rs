//! Network Steering commissioning (BDB v3.0.1 spec §8.3).
//!
//! Network Steering has two operating modes depending on whether the
//! device is already on a network:
//!
//! ## Not on a network
//! 1. Scan primary channels for open networks (`NLME-NETWORK-DISCOVERY`)
//! 2. Filter by extended PAN ID if `bdbUseExtendedPanId` is configured
//! 3. Attempt to join the best-LQI network (`NLME-JOIN`)
//! 4. On join success: broadcast `Device_annce`
//! 5. Request Trust Center link key (APSME-REQUEST-KEY)
//! 6. If primary channels fail, retry on secondary channels
//!
//! ## Already on a network
//! 1. Open local permit joining for `bdbcMinCommissioningTime`
//! 2. Broadcast `Mgmt_Permit_Joining_req` to the network

use zigbee_aps::security::ApsKeyType;
use zigbee_mac::MacDriver;
use zigbee_nwk::DeviceType;
use zigbee_types::ShortAddress;
use zigbee_zdo::ZdpStatus;
use zigbee_zdo::discovery::NodeDescRsp;

use crate::attributes::BDB_MIN_COMMISSIONING_TIME;
use crate::{
    BdbLayer, BdbStatus, KeyFrameResult, NetworkSecurityState, SecurityPersistence,
    SteeringDiagnostics, SteeringStage, TrustCenterLinkKeyState,
};

#[cfg(feature = "efr32-trace")]
macro_rules! bdb_diag {
    ($($arg:tt)*) => {
        rtt_target::rprintln!($($arg)*);
    };
}

#[cfg(not(feature = "efr32-trace"))]
macro_rules! bdb_diag {
    ($($arg:tt)*) => {
        ()
    };
}

// ---------------------------------------------------------------------------
// Telink TLSR8258 debug markers.
//
// When the `telink-debug` feature is enabled, the steering layer writes
// counters and captures into a fixed SRAM window (0x0084F450..0x0084F500) so
// a host-side flash dump can decode them.  This works around the fact that
// the joining device on the TLSR8258 has neither RTT nor a UART available
// while commissioning is running.
//
// Layout (relative to TLNK_DBG_BASE = 0x0084F450):
//   +0x00  try_process_frame entries
//   +0x04  NWK header parse OK
//   +0x08  process_key_wait_frame entries
//   +0x0C  NWK frames with security=1
//   +0x10  NWK frames with security=0 (Transport-Key candidate)
//   +0x14  active-key seq matched count
//   +0x18  active-key decrypt SUCCESS
//   +0x1C  TC-link-key try-1 (patched AAD) SUCCESS
//   +0x20  TC-link-key try-2 (original AAD) SUCCESS
//   +0x24  Key-transport key (try-3) SUCCESS
//   +0x28  all-decrypt-FAIL count
//   +0x2C  process_incoming_aps_frame invocations
//   +0x30  key installed (returned true) count
//   +0x34  APS first byte capture (set once, bit31 = "set")
//   +0x38  last NWK sec_hdr key_seq_number (low 8b) | (sec_consumed<<16)
//   +0x3C  last frame: NWK src | (NWK dst <<16)
//   +0x40  Phase-0 passive_rx attempts entered
//   +0x44  Phase-1 parent_poll attempts entered
//   +0x48  Phase-2 coord_poll attempts entered
//   +0x4C  passive_rx got-frame count
//   +0x50  parent_poll got-frame count
//   +0x54  coord_poll got-frame count
//   ---- NOTE: offsets 0x60..0x9C collide with the example main.rs
//   ---- DBG_JOIN_BASE + 0x160..0x19C (drop-frame capture + Step 2 queue
//   ---- counters). Phase 0/1/2 markers therefore live at +0xA0..+0xBC.
//   +0xA0  Phase-0 reached marker (set_once, 0x5030_0000 | bit31)
//   +0xA4  Phase-1+2 reached marker (set_once, 0x5031_0000 | bit31)
//   +0xA8  Phase-0 passive_rx Err/timeout count
//   +0xAC  parent_addr at Phase-1 entry (set_once, low16 = NWK short)
//   +0xB0  Phase-1 first frame: payload bytes [0..4] LE (first-frame-wins via tdbg_set on the first capture; overwritten by later rounds — use 0xC0/0xC4 for LAST)
//   +0xB4  Phase-1 first frame: payload bytes [4..8] LE
//   +0xB8  Phase-2 first frame: payload bytes [0..4] LE  (Phase 2 currently DISABLED — will stay 0)
//   +0xBC  Phase-2 first frame: payload bytes [4..8] LE  (DISABLED)
//   +0xC0  Phase-1 LAST frame: payload bytes [0..4] LE (overwritten every iter)
//   +0xC4  Phase-1 LAST frame: payload bytes [4..8] LE
//   +0xC8  Phase-1 sec=0 frames seen (NWK FCF bit9: payload[1] & 0x02 == 0)
//   +0xCC  Phase-1 sec=1 frames seen (NWK FCF bit9: payload[1] & 0x02 != 0)
// ---------------------------------------------------------------------------
#[cfg(feature = "telink-debug")]
mod tlnk_dbg {
    pub const BASE: u32 = 0x0084_F450;

    #[inline(always)]
    pub fn bump(off: u32) {
        unsafe {
            let p = (BASE + off) as *mut u32;
            core::ptr::write_volatile(p, core::ptr::read_volatile(p).wrapping_add(1));
        }
    }
    #[inline(always)]
    pub fn set(off: u32, val: u32) {
        unsafe {
            core::ptr::write_volatile((BASE + off) as *mut u32, val);
        }
    }
    #[inline(always)]
    pub fn set_once(off: u32, val: u32) {
        unsafe {
            let p = (BASE + off) as *mut u32;
            if core::ptr::read_volatile(p) == 0 {
                core::ptr::write_volatile(p, val | 0x8000_0000);
            }
        }
    }
}

#[cfg(feature = "telink-debug")]
macro_rules! tdbg_bump {
    ($off:expr) => {
        tlnk_dbg::bump($off)
    };
}
#[cfg(not(feature = "telink-debug"))]
macro_rules! tdbg_bump {
    ($off:expr) => {
        ()
    };
}

#[cfg(feature = "telink-debug")]
macro_rules! tdbg_set {
    ($off:expr, $val:expr) => {
        tlnk_dbg::set($off, $val)
    };
}
#[cfg(not(feature = "telink-debug"))]
macro_rules! tdbg_set {
    ($off:expr, $val:expr) => {
        ()
    };
}

#[cfg(feature = "telink-debug")]
macro_rules! tdbg_set_once {
    ($off:expr, $val:expr) => {
        tlnk_dbg::set_once($off, $val)
    };
}
#[cfg(not(feature = "telink-debug"))]
macro_rules! tdbg_set_once {
    ($off:expr, $val:expr) => {
        ()
    };
}

/// Default scan duration exponent for active scan (2^n + 1 superframes).
/// Exponent 3 ≈ 138 ms per channel — good balance of speed vs. reliability.
const SCAN_DURATION: u8 = 3;
// Official Telink BDB sends one initial Node_Desc request plus three retries.
const TCLK_EXCHANGE_ATTEMPTS: u8 = 4;
const TCLK_EXCHANGE_START_DELAY_US: u32 = 1_200_000;
const TCLK_EXCHANGE_TIMEOUT_US: u32 = 5_000_000;
const TCLK_EXCHANGE_POLL_INTERVAL_US: u32 = 50_000;
const TCLK_MIN_STACK_REVISION: u8 = 21;

impl<M: MacDriver> BdbLayer<M> {
    fn security_exchange_timed_out(&self, started: u32) -> bool {
        self.zdo
            .aps()
            .nwk()
            .mac()
            .monotonic_micros()
            .wrapping_sub(started)
            >= TCLK_EXCHANGE_TIMEOUT_US
    }

    async fn receive_security_exchange_frame(&mut self) -> bool {
        if self.zdo.nwk().rx_on_when_idle() {
            match self
                .zdo
                .aps_mut()
                .nwk_mut()
                .mac_mut()
                .mcps_data_indication()
                .await
            {
                Ok(frame) => {
                    self.try_process_frame(frame.payload.as_slice());
                    true
                }
                Err(_) => false,
            }
        } else {
            self.steering_diagnostics.poll_attempts =
                self.steering_diagnostics.poll_attempts.saturating_add(1);
            match self.zdo.aps_mut().nwk_mut().mac_mut().mlme_poll().await {
                Ok(Some(frame)) => {
                    self.steering_diagnostics.poll_data_frames =
                        self.steering_diagnostics.poll_data_frames.saturating_add(1);
                    self.try_process_frame(frame.as_slice());
                    true
                }
                Ok(None) => false,
                Err(_) => {
                    self.steering_diagnostics.poll_errors =
                        self.steering_diagnostics.poll_errors.saturating_add(1);
                    false
                }
            }
        }
    }

    async fn wait_for_security_condition(
        &mut self,
        started: u32,
        rounds: &mut u16,
        mut ready: impl FnMut(&Self) -> bool,
    ) -> bool {
        loop {
            if ready(self) {
                return true;
            }
            if self.security_exchange_timed_out(started) {
                return false;
            }

            self.receive_security_exchange_frame().await;
            *rounds = rounds.saturating_add(1);

            if ready(self) {
                return true;
            }
            if self.security_exchange_timed_out(started) {
                return false;
            }

            self.zdo
                .aps_mut()
                .nwk_mut()
                .mac_mut()
                .delay_micros(TCLK_EXCHANGE_POLL_INTERVAL_US)
                .await;
        }
    }

    async fn wait_for_zdp_response(
        &mut self,
        slot: usize,
        started: u32,
        rounds: &mut u16,
    ) -> Option<heapless::Vec<u8, 128>> {
        loop {
            if let Some(response) = self.zdo.take_response(slot) {
                return Some(response);
            }
            if self.security_exchange_timed_out(started) {
                self.zdo.cancel_pending(slot);
                return None;
            }

            self.receive_security_exchange_frame().await;
            *rounds = rounds.saturating_add(1);

            if let Some(response) = self.zdo.take_response(slot) {
                return Some(response);
            }
            if self.security_exchange_timed_out(started) {
                self.zdo.cancel_pending(slot);
                return None;
            }

            self.zdo
                .aps_mut()
                .nwk_mut()
                .mac_mut()
                .delay_micros(TCLK_EXCHANGE_POLL_INTERVAL_US)
                .await;
        }
    }

    /// Execute the Network Steering procedure (BDB spec §8.3).
    ///
    /// Behaviour depends on `bdbNodeIsOnANetwork`:
    /// - **Not on network**: scan → join → announce → TC key exchange
    /// - **On network**: open permit joining → broadcast Mgmt_Permit_Joining_req
    pub async fn network_steering(&mut self) -> Result<(), BdbStatus> {
        self.network_steering_inner(None).await
    }

    /// Execute Network Steering with synchronous security persistence.
    pub async fn network_steering_with_persistence(
        &mut self,
        persistence: &mut dyn SecurityPersistence,
    ) -> Result<(), BdbStatus> {
        self.network_steering_inner(Some(persistence)).await
    }

    async fn network_steering_inner(
        &mut self,
        persistence: Option<&mut dyn SecurityPersistence>,
    ) -> Result<(), BdbStatus> {
        self.steering_diagnostics = SteeringDiagnostics::default();
        // BDB+0x1FC: dispatcher entry + on/off branch witness.
        //   0x5EE0_0010 = took steer_on_network (node_is_on_a_network was true)
        //   0x5EE0_0011 = took steer_off_network
        if self.attributes.node_is_on_a_network {
            tdbg_set!(0x1FC, 0x5EE0_0010);
            self.steer_on_network().await
        } else {
            tdbg_set!(0x1FC, 0x5EE0_0011);
            self.steer_off_network(persistence).await
        }
    }

    /// Steering when the device is NOT on a network — join an existing PAN.
    async fn steer_off_network(
        &mut self,
        mut persistence: Option<&mut dyn SecurityPersistence>,
    ) -> Result<(), BdbStatus> {
        self.steering_diagnostics.stage = SteeringStage::Scanning;
        let mut discovered_any = false;
        let mut discovered_networks_total: u16 = 0;
        let mut attempted_joins: u16 = 0;

        // Reset retry budget at the start of each commissioning attempt
        if self.attributes.steering_attempts_remaining == 0 {
            self.attributes.steering_attempts_remaining = 5;
        }
        self.attributes.steering_attempts_remaining = self
            .attributes
            .steering_attempts_remaining
            .saturating_sub(1);

        log::info!(
            "[BDB:Steering] Scanning for open networks… (attempts left: {})",
            self.attributes.steering_attempts_remaining,
        );

        // Try primary channels first, then secondary
        let channel_sets = [
            self.attributes.primary_channel_set,
            self.attributes.secondary_channel_set,
        ];

        for (idx, &channel_mask) in channel_sets.iter().enumerate() {
            if channel_mask.0 == 0 {
                continue;
            }

            let set_name = if idx == 0 { "primary" } else { "secondary" };
            log::debug!(
                "[BDB:Steering] Scanning {} channels: 0x{:08X}",
                set_name,
                channel_mask.0
            );

            // Step 1: Network discovery
            self.steering_diagnostics.scan_requests =
                self.steering_diagnostics.scan_requests.saturating_add(1);
            let networks = match self
                .zdo
                .nlme_network_discovery(channel_mask, SCAN_DURATION)
                .await
            {
                Ok(n) => n,
                Err(_) => {
                    log::debug!("[BDB:Steering] No networks on {} channels", set_name);
                    continue;
                }
            };

            log::info!("[BDB:Steering] Found {} network(s)", networks.len());
            discovered_any = discovered_any || !networks.is_empty();
            discovered_networks_total = discovered_networks_total
                .saturating_add(networks.len().min(u16::MAX as usize) as u16);
            self.steering_diagnostics.networks_discovered = self
                .steering_diagnostics
                .networks_discovered
                .saturating_add(networks.len().min(u16::MAX as usize) as u16);

            // Step 2: Filter by extended PAN ID if configured
            let use_epid = self.zdo.aps().aib().aps_use_extended_pan_id;
            let has_epid_filter = use_epid != [0u8; 8];
            let mut epid_rejects: u16 = 0;
            let mut permit_closed_rejects: u16 = 0;
            let mut pass_skips: u16 = 0;
            let mut set_attempted_joins: u16 = 0;

            // Debug: show all discovered networks
            for (i, network) in networks.iter().enumerate() {
                log::info!(
                    "[BDB:Steering] net[{}] PAN=0x{:04X} ch={} d={} permit={} LQI={} via 0x{:04X}",
                    i,
                    network.pan_id.0,
                    network.logical_channel,
                    network.depth,
                    network.permit_joining,
                    network.lqi,
                    network.router_address.0,
                );
            }

            // Step 3: Try joining networks — prefer coordinator (depth=0) for
            // reliable Transport-Key delivery. Some routers don't properly relay
            // the "Update Device" APS command to the TC, so direct coordinator
            // association is more reliable.
            // First pass: coordinator beacons only (depth == 0)
            // Second pass: all other routers
            for prefer_coordinator in [true, false] {
                for network in &networks {
                    // Apply extended PAN ID filter
                    if has_epid_filter && network.extended_pan_id != use_epid {
                        epid_rejects = epid_rejects.saturating_add(1);
                        log::debug!(
                            "[BDB:Steering] Skipping PAN 0x{:04X} — EPID mismatch",
                            network.pan_id.0,
                        );
                        continue;
                    }

                    // Must have permit joining enabled
                    if !network.permit_joining {
                        permit_closed_rejects = permit_closed_rejects.saturating_add(1);
                        self.steering_diagnostics.permit_closed_rejects = self
                            .steering_diagnostics
                            .permit_closed_rejects
                            .saturating_add(1);
                        continue;
                    }

                    // Two-pass: coordinators first, then routers
                    let is_coordinator = network.depth == 0;
                    if prefer_coordinator && !is_coordinator {
                        pass_skips = pass_skips.saturating_add(1);
                        continue;
                    }
                    if !prefer_coordinator && is_coordinator {
                        pass_skips = pass_skips.saturating_add(1);
                        continue; // already tried
                    }

                    set_attempted_joins = set_attempted_joins.saturating_add(1);
                    attempted_joins = attempted_joins.saturating_add(1);
                    self.steering_diagnostics.stage = SteeringStage::Joining;
                    self.steering_diagnostics.join_attempts =
                        self.steering_diagnostics.join_attempts.saturating_add(1);
                    self.steering_diagnostics.channel = network.logical_channel;
                    self.steering_diagnostics.pan_id = network.pan_id.0;
                    self.steering_diagnostics.parent_address = network.router_address.0;
                    self.steering_diagnostics.parent_lqi = network.lqi;
                    self.steering_diagnostics.parent_depth = network.depth;
                    // One attempt per parent — avoid polluting TC state with repeated join/leave
                    let max_tries = 1u8;
                    let mut joined_addr = None;
                    for try_num in 0..max_tries {
                        if try_num > 0 {
                            log::info!(
                                "[BDB:Steering] Retrying coordinator join (attempt {}/{})",
                                try_num + 1,
                                max_tries,
                            );
                        }

                        log::info!(
                            "[BDB:Steering] Joining PAN 0x{:04X} ch {} LQI {} depth {} via 0x{:04X}",
                            network.pan_id.0,
                            network.logical_channel,
                            network.lqi,
                            network.depth,
                            network.router_address.0,
                        );

                        // Step 3: Attempt join
                        match self.zdo.nlme_join(network).await {
                            Ok(addr) => {
                                bdb_diag!("[BDB][EFR32] nlme_join=ok addr=0x{:04X}", addr.0);
                                self.steering_diagnostics.join_successes =
                                    self.steering_diagnostics.join_successes.saturating_add(1);
                                self.steering_diagnostics.last_join_status = 0;
                                self.steering_diagnostics.assigned_address = addr.0;
                                joined_addr = Some(addr);
                                break;
                            }
                            Err(e) => {
                                self.steering_diagnostics.last_join_status = e as u8;
                                bdb_diag!("[BDB][EFR32] nlme_join=err {:?}", e);
                                log::warn!("[BDB:Steering] Join failed: {:?}", e);
                                continue;
                            }
                        }
                    }
                    let nwk_addr = match joined_addr {
                        Some(a) => a,
                        None => continue,
                    };

                    let ieee = self.zdo.nwk().nib().ieee_address;

                    // Step 5: Start router if we are a router
                    if self.zdo.nwk().device_type() == DeviceType::Router {
                        let _ = self.zdo.nlme_start_router().await;
                    }

                    // Step 5b: TC link key exchange
                    // After joining, the coordinator sends Transport-Key (with NWK key)
                    // encrypted with the well-known TC link key (ZigBeeAlliance09).
                    // We must receive and process it before declaring success.
                    // Then send APSME-REQUEST-KEY(0x04) so Z2M establishes a unique TC link key.
                    log::info!("[BDB:Steering] Waiting for Transport-Key from TC...");
                    self.steering_diagnostics.stage = SteeringStage::WaitingForTransportKey;

                    let mut key_received = false;
                    let rx_on = self.zdo.nwk().rx_on_when_idle();

                    // Phase 0: Passive RX listen — only useful when rx_on_when_idle=true
                    // because the TC sends Transport-Key as a DIRECT unicast. When the
                    // device is sleepy (rx_on_when_idle=false), the TC buffers the TK at
                    // the parent as an indirect frame — passive RX will never see it and
                    // the ~3 s timeout delays the first poll, risking indirect-frame expiry.
                    tdbg_set_once!(0xA0, 0x5030_0000); // phase-0 reached
                    if rx_on {
                        log::info!(
                            "[BDB:Steering] Phase 0: passive RX for direct Transport-Key..."
                        );
                        for rx_attempt in 0..4u8 {
                            tdbg_bump!(0x40); // passive_rx attempts
                            match self
                                .zdo
                                .aps_mut()
                                .nwk_mut()
                                .mac_mut()
                                .mcps_data_indication()
                                .await
                            {
                                Ok(mac_frame) => {
                                    tdbg_bump!(0x4C); // passive_rx got frame
                                    self.steering_diagnostics.passive_rx_frames = self
                                        .steering_diagnostics
                                        .passive_rx_frames
                                        .saturating_add(1);
                                    self.steering_diagnostics.last_frame_len =
                                        mac_frame.payload.len().min(u8::MAX as usize) as u8;
                                    let prefix_len = mac_frame
                                        .payload
                                        .len()
                                        .min(self.steering_diagnostics.last_frame_prefix.len());
                                    self.steering_diagnostics.last_frame_prefix[..prefix_len]
                                        .copy_from_slice(
                                            &mac_frame.payload.as_slice()[..prefix_len],
                                        );
                                    let mac_payload = mac_frame.payload.as_slice();
                                    bdb_diag!(
                                        "[BDB][EFR32] passive_rx[{}] {} bytes",
                                        rx_attempt,
                                        mac_payload.len()
                                    );
                                    log::info!(
                                        "[BDB:Steering] RX {}: {} bytes",
                                        rx_attempt,
                                        mac_payload.len(),
                                    );
                                    if let Some(true) = self.try_process_frame(mac_payload) {
                                        key_received = true;
                                        break;
                                    }
                                }
                                Err(_) => {
                                    tdbg_bump!(0xA8); // passive_rx err/timeout
                                    bdb_diag!("[BDB][EFR32] passive_rx[{}] none", rx_attempt);
                                }
                            }
                        }
                        if key_received {
                            log::info!("[BDB:Steering] Transport-Key received during passive RX!");
                        }
                    } else {
                        log::info!(
                            "[BDB:Steering] Phase 0: skipped (sleepy device, TK via indirect poll)"
                        );
                    }
                    tdbg_set_once!(0xF0, 0x9000_0001); // phase-0 completed/skipped
                    tdbg_set_once!(0xF4, 0x9000_0002); // post key_received check

                    // Phase 1+2: Poll parent and coordinator if passive RX didn't work
                    let _parent_addr = self.zdo.nwk().nib().parent_address;
                    tdbg_set_once!(0xF8, 0x9000_0003); // got parent_addr
                    tdbg_set_once!(0xA4, 0x5031_0000); // phase-1+2 reached
                    tdbg_set_once!(0xAC, _parent_addr.0 as u32); // parent NWK short

                    // Raised from 6→20 to give the TC enough time to send TK
                    // after the parent relays Update-Device. The EFR32 reference
                    // shows TK arriving within ~200 ms, but slower coordinators or
                    // multi-hop relays may need more rounds.
                    const MAX_TOTAL_ROUNDS: usize = 128;
                    const MAX_EMPTY_ROUNDS: u16 = 128;
                    let mut empty_count: u16 = 0;
                    let mut total_rounds: usize = 0;
                    let mut data_frames: usize = 0;

                    while !key_received
                        && total_rounds < MAX_TOTAL_ROUNDS
                        && empty_count < MAX_EMPTY_ROUNDS
                    {
                        total_rounds += 1;
                        let mut got_data_this_round = false;

                        // Poll parent for indirect frames
                        tdbg_bump!(0x44); // parent_poll attempts
                        self.steering_diagnostics.poll_attempts =
                            self.steering_diagnostics.poll_attempts.saturating_add(1);
                        match self.zdo.aps_mut().nwk_mut().mac_mut().mlme_poll().await {
                            Ok(Some(mac_frame)) => {
                                self.steering_diagnostics.poll_data_frames =
                                    self.steering_diagnostics.poll_data_frames.saturating_add(1);
                                self.steering_diagnostics.last_frame_len =
                                    mac_frame.len().min(u8::MAX as usize) as u8;
                                let prefix_len = mac_frame
                                    .len()
                                    .min(self.steering_diagnostics.last_frame_prefix.len());
                                self.steering_diagnostics.last_frame_prefix[..prefix_len]
                                    .copy_from_slice(&mac_frame.as_slice()[..prefix_len]);
                                tdbg_bump!(0x50); // parent_poll got frame
                                got_data_this_round = true;
                                data_frames += 1;
                                let mac_payload = mac_frame.as_slice();
                                // Capture first 8 bytes of NWK payload (Phase 1)
                                if mac_payload.len() >= 4 {
                                    let _w0 = u32::from_le_bytes([
                                        mac_payload[0],
                                        mac_payload[1],
                                        mac_payload[2],
                                        mac_payload[3],
                                    ]);
                                    tdbg_set!(0xB0, _w0);
                                    tdbg_set!(0xC0, _w0); // LAST-frame-wins
                                }
                                if mac_payload.len() >= 8 {
                                    let _w1 = u32::from_le_bytes([
                                        mac_payload[4],
                                        mac_payload[5],
                                        mac_payload[6],
                                        mac_payload[7],
                                    ]);
                                    tdbg_set!(0xB4, _w1);
                                    tdbg_set!(0xC4, _w1); // LAST-frame-wins
                                }
                                // Per-iter NWK FCF security-bit counters (FCF is LE in payload[0..2]; sec bit = bit9 = payload[1] & 0x02)
                                if mac_payload.len() >= 2 {
                                    if mac_payload[1] & 0x02 != 0 {
                                        tdbg_bump!(0xCC); // sec=1
                                    } else {
                                        tdbg_bump!(0xC8); // sec=0 (Transport-Key candidate)
                                    }
                                }
                                bdb_diag!(
                                    "[BDB][EFR32] parent_poll[{}] {} bytes total={}",
                                    total_rounds,
                                    mac_payload.len(),
                                    data_frames
                                );
                                log::info!(
                                    "[BDB:Steering] P-Poll {}: {} bytes (total={})",
                                    total_rounds,
                                    mac_payload.len(),
                                    data_frames,
                                );
                                if let Some(true) = self.try_process_frame(mac_payload) {
                                    bdb_diag!("[BDB][EFR32] transport_key=ok via parent_poll");
                                    key_received = true;
                                    break;
                                }
                                tdbg_bump!(0xD0); // post-try_process_frame returned (sec=1 broadcast, etc)
                            }
                            Ok(None) => {
                                bdb_diag!("[BDB][EFR32] parent_poll[{}] none", total_rounds);
                            }
                            Err(e) => {
                                self.steering_diagnostics.poll_errors =
                                    self.steering_diagnostics.poll_errors.saturating_add(1);
                                bdb_diag!("[BDB][EFR32] parent_poll[{}] err {:?}", total_rounds, e);
                                log::warn!("[BDB:Steering] P-Poll {}: err {:?}", total_rounds, e);
                            }
                        }

                        if key_received {
                            break;
                        }

                        // ----------------------------------------------------------------
                        // PHASE 2 DISABLED (diagnostic): the inner mlme_set(MacCoordShortAddress, 0)
                        // followed by mlme_poll() empirically hangs the steering loop after
                        // the first iteration on the TLSR8258 backend — the receive window
                        // appears to block on filter mismatch when the destination changes
                        // mid-flight, so the `while` retry loop never iterates more than once.
                        //
                        // We rely entirely on Phase 1 (parent poll) for Transport-Key delivery
                        // since the parent is the actual forwarder for indirect APS commands
                        // when join goes through a router. If TK arrives via Phase 1 in this
                        // configuration, the Phase 2 path can be removed or kept behind a
                        // feature flag. If it does not, we know Phase 2 needs a real fix
                        // (not just sidestepping).
                        //
                        // To re-enable: restore the block below verbatim — its markers at
                        // BDB +0x48/+0x54/+0xB8/+0xBC remain reserved for it.
                        // ----------------------------------------------------------------
                        /*
                        {
                            let mac = self.zdo.aps_mut().nwk_mut().mac_mut();
                            let _ = mac
                                .mlme_set(
                                    PibAttribute::MacCoordShortAddress,
                                    PibValue::ShortAddress(ShortAddress(0x0000)),
                                )
                                .await;
                            tdbg_bump!(0x48); // coord_poll attempts
                            match mac.mlme_poll().await {
                                Ok(Some(mac_frame)) => {
                                    tdbg_bump!(0x54); // coord_poll got frame
                                    got_data_this_round = true;
                                    data_frames += 1;
                                    let mac_payload = mac_frame.as_slice();
                                    // Capture first 8 bytes of NWK payload (Phase 2)
                                    if mac_payload.len() >= 4 {
                                        let w0 = u32::from_le_bytes([
                                            mac_payload[0], mac_payload[1],
                                            mac_payload[2], mac_payload[3],
                                        ]);
                                        tdbg_set!(0xB8, w0);
                                    }
                                    if mac_payload.len() >= 8 {
                                        let w1 = u32::from_le_bytes([
                                            mac_payload[4], mac_payload[5],
                                            mac_payload[6], mac_payload[7],
                                        ]);
                                        tdbg_set!(0xBC, w1);
                                    }
                                    bdb_diag!(
                                        "[BDB][EFR32] coord_poll[{}] {} bytes total={}",
                                        total_rounds,
                                        mac_payload.len(),
                                        data_frames
                                    );
                                    log::info!(
                                        "[BDB:Steering] C-Poll {}: {} bytes (total={})",
                                        total_rounds,
                                        mac_payload.len(),
                                        data_frames,
                                    );
                                    if let Some(true) = self.try_process_frame(mac_payload) {
                                        bdb_diag!("[BDB][EFR32] transport_key=ok via coord_poll");
                                        key_received = true;
                                    }
                                }
                                Ok(None) => {
                                    bdb_diag!("[BDB][EFR32] coord_poll[{}] none", total_rounds);
                                }
                                Err(e) => {
                                    bdb_diag!(
                                        "[BDB][EFR32] coord_poll[{}] err {:?}",
                                        total_rounds,
                                        e
                                    );
                                    log::debug!(
                                        "[BDB:Steering] C-Poll {}: err {:?}",
                                        total_rounds,
                                        e
                                    );
                                }
                            }
                            // Restore parent address
                            let mac = self.zdo.aps_mut().nwk_mut().mac_mut();
                            let _ = mac
                                .mlme_set(
                                    PibAttribute::MacCoordShortAddress,
                                    PibValue::ShortAddress(parent_addr),
                                )
                                .await;
                        }
                        */

                        if key_received {
                            break;
                        }

                        if got_data_this_round {
                            empty_count = 0;
                        } else {
                            empty_count += 1;
                            log::debug!(
                                "[BDB:Steering] Round {}: no data ({}/{})",
                                total_rounds,
                                empty_count,
                                MAX_EMPTY_ROUNDS,
                            );
                        }
                        tdbg_bump!(0xD4); // end-of-round body reached (next iter should follow)
                    }

                    log::info!(
                        "[BDB:Steering] Transport-Key wait done: passive_rx={} rounds={} frames={} empty={}",
                        if key_received { "hit" } else { "miss" },
                        total_rounds,
                        data_frames,
                        empty_count
                    );

                    if !key_received {
                        self.steering_diagnostics.stage = SteeringStage::TransportKeyMissing;
                        bdb_diag!(
                            "[BDB][EFR32] transport_key=missing rounds={} frames={} empty={}",
                            total_rounds,
                            data_frames,
                            empty_count
                        );
                        log::warn!(
                            "[BDB:Steering] Transport-Key NOT received after {} rounds ({} data frames, {} consecutive empty)",
                            total_rounds,
                            data_frames,
                            empty_count,
                        );
                    }

                    if !key_received {
                        bdb_diag!(
                            "[BDB][EFR32] reset pan=0x{:04X} reason=no_transport_key",
                            network.pan_id.0
                        );
                        log::warn!(
                            "[BDB:Steering] Transport-Key not received — resetting and trying next parent on PAN 0x{:04X}",
                            network.pan_id.0,
                        );
                        // We cannot send a proper encrypted leave without the
                        // network key. Clear local NWK/MAC state and try the
                        // next beacon candidate; declaring success here leaves
                        // us unable to decrypt ZHA interview traffic.
                        let _ = self.zdo.nlme_reset(false).await;
                        continue;
                    }

                    self.steering_diagnostics.stage = SteeringStage::TransportKeyReceived;
                    self.steering_diagnostics.transport_key_received = true;

                    if let Some(persistence) = persistence.as_deref_mut()
                        && let Err(error) = self.reserve_network_security(persistence)
                    {
                        self.steering_diagnostics.stage = SteeringStage::PersistenceFailed;
                        log::error!(
                            "[BDB:Steering] Failed to persist network security: {:?}",
                            error
                        );
                        let _ = self.zdo.nlme_reset(false).await;
                        return Err(BdbStatus::PersistenceFailure);
                    }

                    // Step 5c: Send Device_annce now that we have the NWK key
                    self.steering_diagnostics.stage = SteeringStage::Announcing;
                    self.zdo.set_local_nwk_addr(nwk_addr);
                    self.zdo.set_local_ieee_addr(ieee);
                    bdb_diag!(
                        "[BDB][EFR32] zdo_local addr=0x{:04X} ieee={:02X?}",
                        nwk_addr.0,
                        ieee
                    );
                    if let Err(e) = self.zdo.device_annce(nwk_addr, ieee).await {
                        log::warn!("[BDB:Steering] Device_annce failed: {:?}", e);
                        let _ = self.zdo.nlme_reset(false).await;
                        continue;
                    }

                    // Step 5d: Retrieve a unique Trust Center link key, then
                    // prove possession and wait for a successful Confirm-Key.
                    // The official Telink BDB gives the complete exchange
                    // five seconds and retries it up to three times.
                    let tc_addr = ShortAddress::COORDINATOR;
                    let tc_ieee = self.zdo.aps().aib().aps_trust_center_address;
                    if tc_ieee == [0u8; 8] {
                        self.steering_diagnostics.stage =
                            SteeringStage::TrustCenterLinkKeyExchangeFailed;
                        self.attributes.commissioning_status =
                            crate::attributes::BdbCommissioningStatus::TcLinkKeyExchangeFailure;
                        let _ = self.zdo.nlme_reset(false).await;
                        return Err(BdbStatus::TrustCenterLinkKeyExchangeFailure);
                    }

                    self.zdo
                        .aps_mut()
                        .nwk_mut()
                        .mac_mut()
                        .delay_micros(TCLK_EXCHANGE_START_DELAY_US)
                        .await;

                    let mut tclk_exchange_complete = false;
                    for _ in 0..TCLK_EXCHANGE_ATTEMPTS {
                        self.zdo
                            .aps_mut()
                            .security_mut()
                            .remove_key(&tc_ieee, ApsKeyType::TrustCenterLinkKey);

                        let exchange_started = self.zdo.aps().nwk().mac().monotonic_micros();
                        let mut exchange_rounds = 0u16;
                        self.steering_diagnostics.stage =
                            SteeringStage::QueryingTrustCenterNodeDescriptor;
                        self.steering_diagnostics.node_desc_requests = self
                            .steering_diagnostics
                            .node_desc_requests
                            .saturating_add(1);
                        let node_desc_slot = match self.zdo.start_node_desc_req(tc_addr).await {
                            Ok(slot) => slot,
                            Err(e) => {
                                self.steering_diagnostics.node_desc_send_failures = self
                                    .steering_diagnostics
                                    .node_desc_send_failures
                                    .saturating_add(1);
                                self.steering_diagnostics.last_node_desc_status = e as u8;
                                log::warn!("[BDB:Steering] Node_Desc_req failed: {:?}", e);
                                let _ = self
                                    .wait_for_security_condition(
                                        exchange_started,
                                        &mut exchange_rounds,
                                        |_| false,
                                    )
                                    .await;
                                continue;
                            }
                        };

                        let node_desc_payload = match self
                            .wait_for_zdp_response(
                                node_desc_slot,
                                exchange_started,
                                &mut exchange_rounds,
                            )
                            .await
                        {
                            Some(payload) => payload,
                            None => {
                                self.steering_diagnostics.node_desc_timeouts = self
                                    .steering_diagnostics
                                    .node_desc_timeouts
                                    .saturating_add(1);
                                log::warn!("[BDB:Steering] Node_Desc_rsp timed out");
                                continue;
                            }
                        };
                        self.steering_diagnostics.node_desc_responses = self
                            .steering_diagnostics
                            .node_desc_responses
                            .saturating_add(1);

                        let node_desc = match NodeDescRsp::parse(node_desc_payload.as_slice()) {
                            Ok(response) => response,
                            Err(e) => {
                                self.steering_diagnostics.node_desc_parse_failures = self
                                    .steering_diagnostics
                                    .node_desc_parse_failures
                                    .saturating_add(1);
                                log::warn!("[BDB:Steering] Invalid Node_Desc_rsp: {:?}", e);
                                let _ = self
                                    .wait_for_security_condition(
                                        exchange_started,
                                        &mut exchange_rounds,
                                        |_| false,
                                    )
                                    .await;
                                continue;
                            }
                        };
                        self.steering_diagnostics.last_node_desc_status = node_desc.status as u8;
                        if node_desc.status != ZdpStatus::Success
                            || node_desc.nwk_addr_of_interest != tc_addr
                        {
                            log::warn!(
                                "[BDB:Steering] Node_Desc_rsp rejected: status={:?} addr=0x{:04X}",
                                node_desc.status,
                                node_desc.nwk_addr_of_interest.0,
                            );
                            let _ = self
                                .wait_for_security_condition(
                                    exchange_started,
                                    &mut exchange_rounds,
                                    |_| false,
                                )
                                .await;
                            continue;
                        }

                        let Some(node_descriptor) = node_desc.node_descriptor else {
                            self.steering_diagnostics.node_desc_parse_failures = self
                                .steering_diagnostics
                                .node_desc_parse_failures
                                .saturating_add(1);
                            let _ = self
                                .wait_for_security_condition(
                                    exchange_started,
                                    &mut exchange_rounds,
                                    |_| false,
                                )
                                .await;
                            continue;
                        };
                        let stack_revision = node_descriptor.stack_revision();
                        self.steering_diagnostics.trust_center_server_mask =
                            node_descriptor.server_mask;
                        self.steering_diagnostics.trust_center_stack_revision = stack_revision;
                        log::info!(
                            "[BDB:Steering] Trust Center stack revision {} (server mask 0x{:04X})",
                            stack_revision,
                            node_descriptor.server_mask,
                        );

                        if stack_revision < TCLK_MIN_STACK_REVISION {
                            log::info!(
                                "[BDB:Steering] Pre-R21 Trust Center; unique link-key exchange not required"
                            );
                            tclk_exchange_complete = true;
                            break;
                        }

                        self.steering_diagnostics.stage =
                            SteeringStage::RequestingTrustCenterLinkKey;
                        self.steering_diagnostics.request_key_attempts = self
                            .steering_diagnostics
                            .request_key_attempts
                            .saturating_add(1);
                        match self.zdo.aps_mut().send_request_key(tc_addr).await {
                            Ok(()) => {
                                self.steering_diagnostics.request_key_send_successes = self
                                    .steering_diagnostics
                                    .request_key_send_successes
                                    .saturating_add(1);
                                self.steering_diagnostics.request_key_error = 0;
                            }
                            Err(e) => {
                                self.steering_diagnostics.request_key_send_failures = self
                                    .steering_diagnostics
                                    .request_key_send_failures
                                    .saturating_add(1);
                                self.steering_diagnostics.request_key_error = e as u8;
                                log::warn!("[BDB:Steering] Request-Key failed: {:?}", e);
                                let _ = self
                                    .wait_for_security_condition(
                                        exchange_started,
                                        &mut exchange_rounds,
                                        |_| false,
                                    )
                                    .await;
                                continue;
                            }
                        }

                        self.steering_diagnostics.stage =
                            SteeringStage::WaitingForTrustCenterLinkKey;
                        let tclk_installed = self
                            .wait_for_security_condition(
                                exchange_started,
                                &mut exchange_rounds,
                                |bdb| {
                                    bdb.zdo
                                        .aps()
                                        .security()
                                        .find_key(&tc_ieee, ApsKeyType::TrustCenterLinkKey)
                                        .is_some()
                                },
                            )
                            .await;
                        if !tclk_installed {
                            log::warn!("[BDB:Steering] Unique TC link key was not received");
                            continue;
                        }
                        self.steering_diagnostics.tclk_installations = self
                            .steering_diagnostics
                            .tclk_installations
                            .saturating_add(1);

                        if let Some(persistence) = persistence.as_deref_mut()
                            && let Err(error) =
                                self.reserve_trust_center_link_key(persistence, &tc_ieee)
                        {
                            self.steering_diagnostics.stage = SteeringStage::PersistenceFailed;
                            log::error!(
                                "[BDB:Steering] Failed to persist Trust Center link key: {:?}",
                                error
                            );
                            self.zdo
                                .aps_mut()
                                .security_mut()
                                .remove_key(&tc_ieee, ApsKeyType::TrustCenterLinkKey);
                            let _ = self.zdo.nlme_reset(false).await;
                            return Err(BdbStatus::PersistenceFailure);
                        }

                        let handshake_before = self.zdo.aps().security_handshake_stats();
                        self.steering_diagnostics.stage = SteeringStage::VerifyingLinkKey;
                        tdbg_bump!(0x1E0);
                        self.steering_diagnostics.verify_key_attempts = self
                            .steering_diagnostics
                            .verify_key_attempts
                            .saturating_add(1);
                        match self.zdo.aps_mut().send_tc_verify_key(tc_addr).await {
                            Ok(()) => {
                                self.steering_diagnostics.verify_key_successes = self
                                    .steering_diagnostics
                                    .verify_key_successes
                                    .saturating_add(1);
                                self.steering_diagnostics.verify_key_error = 0;
                                tdbg_bump!(0x1E4);
                            }
                            Err(e) => {
                                self.steering_diagnostics.verify_key_error = e as u8;
                                tdbg_set!(0x1E8, 0xE1AD_0000);
                                log::warn!("[BDB:Steering] Verify-Key failed: {:?}", e);
                                continue;
                            }
                        }

                        self.steering_diagnostics.stage = SteeringStage::WaitingForConfirmKey;
                        let confirm_seen = self
                            .wait_for_security_condition(
                                exchange_started,
                                &mut exchange_rounds,
                                |bdb| {
                                    let stats = bdb.zdo.aps().security_handshake_stats();
                                    stats.confirm_key_successes
                                        > handshake_before.confirm_key_successes
                                        || stats.confirm_key_rejections
                                            > handshake_before.confirm_key_rejections
                                },
                            )
                            .await;
                        let handshake_after = self.zdo.aps().security_handshake_stats();
                        self.steering_diagnostics.confirm_key_frames =
                            handshake_after.confirm_key_received;
                        self.steering_diagnostics.confirm_key_successes =
                            handshake_after.confirm_key_successes;
                        self.steering_diagnostics.confirm_key_rejections =
                            handshake_after.confirm_key_rejections;
                        self.steering_diagnostics.last_confirm_key_status =
                            handshake_after.last_confirm_key_status;

                        if confirm_seen
                            && handshake_after.confirm_key_successes
                                > handshake_before.confirm_key_successes
                        {
                            tclk_exchange_complete = true;
                            break;
                        }
                    }

                    if !tclk_exchange_complete {
                        self.steering_diagnostics.stage =
                            SteeringStage::TrustCenterLinkKeyExchangeFailed;
                        self.attributes.commissioning_status =
                            crate::attributes::BdbCommissioningStatus::TcLinkKeyExchangeFailure;
                        self.zdo
                            .aps_mut()
                            .security_mut()
                            .remove_key(&tc_ieee, ApsKeyType::TrustCenterLinkKey);
                        let _ = self.zdo.nlme_reset(false).await;
                        return Err(BdbStatus::TrustCenterLinkKeyExchangeFailure);
                    }

                    if let Some(persistence) = persistence.as_deref_mut()
                        && let Err(error) = self.commit_persisted_network(persistence, &tc_ieee)
                    {
                        self.steering_diagnostics.stage = SteeringStage::PersistenceFailed;
                        log::error!(
                            "[BDB:Steering] Failed to commit commissioned network: {:?}",
                            error
                        );
                        return Err(BdbStatus::PersistenceFailure);
                    }

                    // Success!
                    self.attributes.node_is_on_a_network = true;
                    self.attributes.commissioning_status =
                        crate::attributes::BdbCommissioningStatus::Success;
                    self.steering_diagnostics.stage = SteeringStage::Complete;

                    bdb_diag!("[BDB][EFR32] steering=ok addr=0x{:04X}", nwk_addr.0);
                    log::info!("[BDB:Steering] Joined successfully as 0x{:04X}", nwk_addr.0,);
                    // BDB+0x1F0: full off-network success path
                    // (Device_annce + Verify-Key sent)
                    tdbg_set!(0x1F0, 0x5EE0_0001);
                    return Ok(());
                }
            } // end prefer_coordinator pass

            if !networks.is_empty() {
                log::info!(
                    "[BDB:Steering] {} summary: total={} attempted={} reject_epid={} reject_permit_closed={} pass_skips={}",
                    set_name,
                    networks.len(),
                    set_attempted_joins,
                    epid_rejects,
                    permit_closed_rejects,
                    pass_skips,
                );
                if set_attempted_joins == 0 {
                    log::warn!(
                        "[BDB:Steering] {}: discovered networks but none were join candidates (all filtered)",
                        set_name
                    );
                }
            }
        }

        // All attempts exhausted
        if discovered_any {
            log::warn!(
                "[BDB:Steering] Exhausted steering with {} discovered network(s) but {} join attempt(s)",
                discovered_networks_total,
                attempted_joins
            );
        }
        if self.steering_diagnostics.join_successes != 0 {
            self.steering_diagnostics.stage = SteeringStage::TransportKeyMissing;
        } else if attempted_joins != 0 {
            self.steering_diagnostics.stage = SteeringStage::JoinFailed;
        } else if discovered_any {
            self.steering_diagnostics.stage = SteeringStage::NoJoinCandidate;
        } else {
            self.steering_diagnostics.stage = SteeringStage::NoNetworks;
        }
        self.attributes.commissioning_status =
            crate::attributes::BdbCommissioningStatus::NoScanResponse;
        Err(BdbStatus::NoScanResponse)
    }

    fn reserve_network_security(
        &mut self,
        persistence: &mut dyn SecurityPersistence,
    ) -> Result<(), crate::SecurityPersistenceError> {
        let (network_key, key_sequence) = self
            .zdo
            .nwk()
            .security()
            .active_key()
            .map(|entry| (entry.key, entry.seq_number))
            .ok_or(crate::SecurityPersistenceError::InvalidState)?;
        let nib = self.zdo.nwk().nib();
        let state = NetworkSecurityState {
            extended_pan_id: nib.extended_pan_id,
            pan_id: nib.pan_id.0,
            short_address: nib.network_address.0,
            ieee_address: nib.ieee_address,
            channel: nib.logical_channel,
            depth: nib.depth,
            parent_address: nib.parent_address.0,
            update_id: nib.update_id,
            network_key,
            key_sequence,
            outgoing_frame_counter: nib.outgoing_frame_counter,
        };
        let reservation = persistence.reserve_network_security(&state)?;
        if !reservation.is_valid() || reservation.current < state.outgoing_frame_counter {
            return Err(crate::SecurityPersistenceError::InvalidState);
        }
        if !self
            .zdo
            .nwk_mut()
            .nib_mut()
            .set_frame_counter_reservation(reservation.current, reservation.limit)
        {
            return Err(crate::SecurityPersistenceError::InvalidState);
        }
        Ok(())
    }

    fn reserve_trust_center_link_key(
        &mut self,
        persistence: &mut dyn SecurityPersistence,
        trust_center: &zigbee_types::IeeeAddress,
    ) -> Result<(), crate::SecurityPersistenceError> {
        let state = self
            .zdo
            .aps()
            .security()
            .find_key(trust_center, ApsKeyType::TrustCenterLinkKey)
            .map(|entry| TrustCenterLinkKeyState {
                partner_address: entry.partner_address,
                key: entry.key,
                key_type: entry.key_type,
                outgoing_frame_counter: entry.outgoing_frame_counter,
                incoming_frame_counter: entry.incoming_frame_counter,
                incoming_frame_counter_valid: entry.incoming_frame_counter_valid,
            })
            .ok_or(crate::SecurityPersistenceError::InvalidState)?;
        let reservation = persistence.reserve_trust_center_link_key(&state)?;
        if !reservation.is_valid() || reservation.current < state.outgoing_frame_counter {
            return Err(crate::SecurityPersistenceError::InvalidState);
        }
        let entry = self
            .zdo
            .aps_mut()
            .security_mut()
            .find_key_mut(trust_center, ApsKeyType::TrustCenterLinkKey)
            .ok_or(crate::SecurityPersistenceError::InvalidState)?;
        entry.outgoing_frame_counter = reservation.current;
        entry.outgoing_frame_counter_limit = reservation.limit;
        Ok(())
    }

    fn commit_persisted_network(
        &self,
        persistence: &mut dyn SecurityPersistence,
        trust_center: &zigbee_types::IeeeAddress,
    ) -> Result<(), crate::SecurityPersistenceError> {
        let state = self
            .zdo
            .aps()
            .security()
            .find_key(trust_center, ApsKeyType::TrustCenterLinkKey)
            .map(|entry| TrustCenterLinkKeyState {
                partner_address: entry.partner_address,
                key: entry.key,
                key_type: entry.key_type,
                outgoing_frame_counter: entry.outgoing_frame_counter,
                incoming_frame_counter: entry.incoming_frame_counter,
                incoming_frame_counter_valid: entry.incoming_frame_counter_valid,
            })
            .ok_or(crate::SecurityPersistenceError::InvalidState)?;
        persistence.commit_network(&state)
    }

    /// Steering when the device IS already on a network.
    ///
    /// Opens the network for joining and broadcasts Mgmt_Permit_Joining_req
    /// so that routers in the network also open their permit joining.
    async fn steer_on_network(&mut self) -> Result<(), BdbStatus> {
        log::info!("[BDB:Steering] Already on network — opening permit joining");

        // Can only permit joining on coordinator / router
        if self.zdo.nwk().device_type() == DeviceType::EndDevice {
            log::debug!("[BDB:Steering] End device — skipping permit joining");
            // End devices can only trigger steering-on-network by sending
            // Mgmt_Permit_Joining_req to their parent / coordinator.
            let _ = self
                .zdo
                .mgmt_permit_joining_req(
                    ShortAddress::COORDINATOR,
                    BDB_MIN_COMMISSIONING_TIME as u8,
                    true,
                )
                .await;
            // BDB+0x1F4: steer_on_network EndDevice path returns Ok without Device_annce
            tdbg_set!(0x1F4, 0x5EE0_0002);
            return Ok(());
        }

        // Open local permit joining (duration = bdbcMinCommissioningTime)
        // Duration is capped at 254 (0xFE) seconds per Zigbee spec.
        let duration = core::cmp::min(BDB_MIN_COMMISSIONING_TIME, 254) as u8;

        self.zdo
            .nlme_permit_joining(duration)
            .await
            .map_err(|_| BdbStatus::SteeringFailure)?;

        // Broadcast Mgmt_Permit_Joining_req to all routers
        self.zdo
            .mgmt_permit_joining_req(ShortAddress::BROADCAST, duration, true)
            .await
            .map_err(|_| BdbStatus::SteeringFailure)?;

        self.attributes.commissioning_status = crate::attributes::BdbCommissioningStatus::Success;

        log::info!("[BDB:Steering] Permit joining opened for {}s", duration,);
        // BDB+0x1F8: steer_on_network coord/router path returns Ok without Device_annce
        tdbg_set!(0x1F8, 0x5EE0_0003);
        Ok(())
    }

    /// Parse a MAC payload, log diagnostics, and attempt Transport-Key extraction.
    /// Returns `Some(true)` if the NWK key was installed.
    fn try_process_frame(&mut self, mac_payload: &[u8]) -> Option<bool> {
        tdbg_bump!(0x00); // try_process_frame entries
        if let Some((nwk_hdr, nwk_consumed)) = zigbee_nwk::frames::NwkHeader::parse(mac_payload) {
            self.steering_diagnostics.nwk_header_len = nwk_consumed.min(u8::MAX as usize) as u8;
            self.steering_diagnostics.nwk_security = nwk_hdr.frame_control.security;
            tdbg_bump!(0x04); // NWK header parse OK
            tdbg_set!(
                0x3C,
                (nwk_hdr.src_addr.0 as u32) | ((nwk_hdr.dst_addr.0 as u32) << 16)
            );
            bdb_diag!(
                "[BDB][EFR32] nwk type={} src=0x{:04X} dst=0x{:04X} sec={} used={}",
                nwk_hdr.frame_control.frame_type,
                nwk_hdr.src_addr.0,
                nwk_hdr.dst_addr.0,
                nwk_hdr.frame_control.security as u8,
                nwk_consumed
            );
            log::info!(
                "[BDB:Steering] NWK: type={} src=0x{:04X} dst=0x{:04X} sec={}",
                nwk_hdr.frame_control.frame_type,
                nwk_hdr.src_addr.0,
                nwk_hdr.dst_addr.0,
                nwk_hdr.frame_control.security,
            );
            // Hex dump coordinator frames for debugging
            if nwk_hdr.src_addr.0 == 0x0000 {
                let dump_len = mac_payload.len().min(32);
                let hex: heapless::String<96> =
                    mac_payload[..dump_len]
                        .iter()
                        .fold(heapless::String::new(), |mut s, b| {
                            let _ = core::fmt::Write::write_fmt(&mut s, format_args!("{:02X}", b));
                            s
                        });
                log::info!("[BDB:Steering] COORD hex: {}", hex);
            }
            self.process_key_wait_frame(mac_payload, &nwk_hdr, nwk_consumed, 0)
        } else if mac_payload.len() > 2 {
            self.steering_diagnostics.key_frame_result = KeyFrameResult::NwkParseFailed;
            bdb_diag!("[BDB][EFR32] nwk_parse=fail len={}", mac_payload.len());
            let dump_len = mac_payload.len().min(20);
            let hex: heapless::String<60> =
                mac_payload[..dump_len]
                    .iter()
                    .fold(heapless::String::new(), |mut s, b| {
                        let _ = core::fmt::Write::write_fmt(&mut s, format_args!("{:02X}", b));
                        s
                    });
            log::warn!(
                "[BDB:Steering] NWK parse FAIL: len={} {}",
                mac_payload.len(),
                hex
            );
            None
        } else {
            None
        }
    }

    /// Process a received MAC frame during Transport-Key wait.
    ///
    /// Parses NWK header and security, attempts decrypt if needed, then processes
    /// via APS layer. Returns `Some(true)` if NWK key was installed (Transport-Key
    /// received), `Some(false)` if frame was processed but no key, `None` if
    /// parsing/decrypt failed.
    fn process_key_wait_frame(
        &mut self,
        mac_payload: &[u8],
        nwk_hdr: &zigbee_nwk::frames::NwkHeader,
        nwk_consumed: usize,
        lqi: u8,
    ) -> Option<bool> {
        tdbg_bump!(0x08); // process_key_wait_frame entries
        let after_nwk = &mac_payload[nwk_consumed..];
        let mut buf = [0u8; 128];
        let payload_data: Option<([u8; 128], usize)>;

        if nwk_hdr.frame_control.security {
            tdbg_bump!(0x0C); // sec=1
            tdbg_bump!(0x58); // pre-NwkSecurityHeader::parse
            let parse_result = zigbee_nwk::security::NwkSecurityHeader::parse(after_nwk);
            tdbg_bump!(0x5C); // post-parse (regardless of Option result)
            if let Some((sec_hdr, sec_consumed)) = parse_result {
                tdbg_set!(
                    0x38,
                    (sec_hdr.key_seq_number as u32) | ((sec_consumed as u32) << 16)
                );
                bdb_diag!(
                    "[BDB][EFR32] nwk_sec key_seq={} sec_used={} cipher_len={}",
                    sec_hdr.key_seq_number,
                    sec_consumed,
                    after_nwk.len().saturating_sub(sec_consumed)
                );
                if let Some(key_entry) = self
                    .zdo
                    .aps()
                    .nwk()
                    .security()
                    .key_by_seq(sec_hdr.key_seq_number)
                {
                    tdbg_bump!(0x14); // active key seq matched
                    let key = key_entry.key;
                    let aad_len = nwk_consumed + sec_consumed;
                    // AAD must use ACTUAL security level (5), not OTA value (0).
                    let mut aad_buf = [0u8; 64];
                    let aad_copy_len = aad_len.min(aad_buf.len());
                    aad_buf[..aad_copy_len].copy_from_slice(&mac_payload[..aad_copy_len]);
                    aad_buf[nwk_consumed] = (aad_buf[nwk_consumed] & !0x07) | 0x05;
                    tdbg_bump!(0x90); // pre-decrypt(active_key)
                    let active_pt = self.zdo.aps().nwk().security().decrypt(
                        &aad_buf[..aad_copy_len],
                        &after_nwk[sec_consumed..],
                        &key,
                        &sec_hdr,
                    );
                    tdbg_bump!(0x94); // post-decrypt(active_key)
                    if let Some(pt) = active_pt {
                        tdbg_bump!(0x18); // active-key decrypt success
                        bdb_diag!("[BDB][EFR32] nwk_decrypt=ok active_key len={}", pt.len());
                        let len = pt.len().min(128);
                        buf[..len].copy_from_slice(&pt[..len]);
                        payload_data = Some((buf, len));
                    } else {
                        self.steering_diagnostics.key_frame_result =
                            KeyFrameResult::ActiveKeyDecryptFailed;
                        bdb_diag!("[BDB][EFR32] nwk_decrypt=fail active_key");
                        log::warn!("[BDB:Steering] NWK decrypt failed");
                        payload_data = None;
                    }
                } else {
                    // No active NWK key yet. Per Zigbee Pro spec §4.5.3 the
                    // Transport-Key arrives as a **sec=0 NWK** frame carrying
                    // an APS Transport-Key command encrypted with the *APS-
                    // layer* KT key (HMAC-derived from the TC link key). The
                    // KT key is NOT a NWK-layer key — attempting to use it
                    // here against sec=1 broadcasts only burns cycles and,
                    // worse, can hang RustCrypto `ccm` on certain inputs
                    // (observed on TLSR8258 after 3 successful failures on
                    // the 4th call → stalls the steering loop indefinitely).
                    //
                    // The correct sec=0 path lives in the `else` branch
                    // below (`if nwk_hdr.frame_control.security` false →
                    // pass after_nwk to `process_incoming_aps_frame`, which
                    // applies the KT key at the APS layer where it belongs).
                    //
                    // So: when sec=1 and we have no NWK key, simply drop.
                    tdbg_bump!(0x28); // counted as "undecryptable sec=1" (was MIC-fail)
                    self.steering_diagnostics.key_frame_result = KeyFrameResult::SecuredNoActiveKey;
                    bdb_diag!("[BDB][EFR32] sec=1 no_active_key — drop (KT is APS-layer, not NWK)");
                    payload_data = None;
                }
            } else {
                self.steering_diagnostics.key_frame_result =
                    KeyFrameResult::SecurityHeaderParseFailed;
                bdb_diag!("[BDB][EFR32] nwk_sec=parse_fail len={}", after_nwk.len());
                log::warn!("[BDB:Steering] NWK security header parse failed");
                payload_data = None;
            }
        } else {
            // NWK security OFF — this is what Transport-Key looks like
            tdbg_bump!(0x10); // sec=0 (unsecured)
            bdb_diag!("[BDB][EFR32] nwk_unsecured after_nwk={}", after_nwk.len());
            log::info!(
                "[BDB:Steering] NWK unsecured frame! {} bytes — possible Transport-Key",
                after_nwk.len()
            );
            let len = after_nwk.len().min(128);
            buf[..len].copy_from_slice(&after_nwk[..len]);
            payload_data = Some((buf, len));
            self.steering_diagnostics.key_frame_result = KeyFrameResult::UnsecuredAps;
        }

        if let Some((data, len)) = payload_data {
            tdbg_bump!(0x2C); // process_incoming_aps_frame invocations
            if len >= 1 {
                tdbg_set_once!(0x34, data[0] as u32); // first APS byte
            }
            let mut aps_buf = zigbee_aps::apsde::ApsFrameBuffer::new();
            // Log first 20 bytes hex for debugging APS parsing
            if len >= 4 {
                bdb_diag!(
                    "[BDB][EFR32] aps first={:02X} {:02X} {:02X} {:02X} len={}",
                    data[0],
                    data[1],
                    data[2],
                    data[3],
                    len
                );
                log::info!(
                    "[BDB:Steering] APS payload hex: {:02X} {:02X} {:02X} {:02X} (len={})",
                    data[0],
                    data[1],
                    data[2],
                    data[3],
                    len,
                );
            }

            let indication = {
                self.zdo.aps_mut().process_incoming_aps_frame(
                    &data[..len],
                    nwk_hdr.src_addr,
                    nwk_hdr.dst_addr,
                    lqi,
                    nwk_hdr.frame_control.security,
                    &mut aps_buf,
                )
            };
            if let Some(indication) = indication {
                let _ = self.zdo.deliver_client_response(&indication);
            }

            if self.zdo.aps().nwk().security().active_key().is_some() {
                self.steering_diagnostics.key_frame_result = KeyFrameResult::KeyInstalled;
                tdbg_bump!(0x30); // key installed
                bdb_diag!("[BDB][EFR32] aps_process=key_installed");
                log::info!("[BDB:Steering] NWK key received from TC!");
                return Some(true);
            }
            self.steering_diagnostics.key_frame_result = KeyFrameResult::ApsProcessedNoKey;
            bdb_diag!("[BDB][EFR32] aps_process=no_key");
            log::info!("[BDB:Steering] APS processed but no key installed yet");
            Some(false)
        } else {
            bdb_diag!("[BDB][EFR32] payload_data=none");
            None
        }
    }
}
