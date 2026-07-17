//! TLSR8258 802.15.4 radio: PHY/channel/DMA bring-up (`phy`, hardware-only)
//! plus pure, host-testable framing (`frame`). This module is the only
//! place `mac_test` needs to touch for radio I/O.

pub mod frame;
#[cfg(target_arch = "tc32")]
pub mod phy;

/// Total DMA buffer size in bytes: 5-byte header + up to 127-byte MAC PSDU +
/// HW trailer (CRC/RSSI/status), rounded up to a 16-byte DMA granule and
/// kept 4-byte aligned. Matches the sensor lab's proven `DmaBuf` size.
pub const DMA_BUF_LEN: usize = 144;
/// Maximum MAC frame length passed between the driver and upper MAC layer.
/// The IEEE 802.15.4 PSDU limit is 127 bytes including the two-byte FCS,
/// which TLSR8258 appends/removes in hardware.
pub const MAX_MAC_FRAME_LEN: usize = 125;

/// RF DMA buffer wrapper. `repr(align(4))` is required by the TLSR8258 DMA
/// engine (see `memory.x`'s `.rf_dma` section, which is checked post-link
/// for 4-byte alignment by `scripts/tlsr8258.sh verify_layout`).
#[repr(align(4))]
pub struct DmaBuf(pub [u8; DMA_BUF_LEN]);

#[derive(Clone, Copy, Debug)]
pub struct ReceivedFrame {
    data: [u8; MAX_MAC_FRAME_LEN],
    len: u8,
    pub lqi: u8,
    pub rssi: i8,
}

impl ReceivedFrame {
    pub fn as_slice(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    pub fn len(&self) -> usize {
        self.len as usize
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

#[derive(Clone, Copy, Debug)]
pub enum RawRxOutcome {
    Frame(ReceivedFrame),
    InvalidLength,
    InvalidCrc,
}

/// Exclusive handle to the TLSR8258 radio/DMA engine.
///
/// TLSR8258 has one RF block and this HAL uses fixed application-linked DMA
/// storage, so production code should obtain the handle with [`Radio::take`].
pub struct Radio {
    _private: (),
}

#[cfg(target_arch = "tc32")]
static mut RADIO_TAKEN: u8 = 0;

impl Radio {
    #[cfg(target_arch = "tc32")]
    pub fn take() -> Option<Self> {
        unsafe {
            let ptr = core::ptr::addr_of_mut!(RADIO_TAKEN);
            if core::ptr::read_volatile(ptr) != 0 {
                return None;
            }
            core::ptr::write_volatile(ptr, 1);
        }
        Some(Self { _private: () })
    }

    /// Bypass singleton acquisition. The caller must guarantee that no other
    /// radio handle or legacy free-function user can access the RF block.
    #[cfg(target_arch = "tc32")]
    pub unsafe fn steal() -> Self {
        unsafe {
            core::ptr::write_volatile(core::ptr::addr_of_mut!(RADIO_TAKEN), 1);
        }
        Self { _private: () }
    }

    #[cfg(target_arch = "tc32")]
    pub fn init(&mut self) {
        hw::init();
    }

    #[cfg(target_arch = "tc32")]
    pub fn set_channel(&mut self, channel: u8) {
        hw::set_channel(channel);
    }

    #[cfg(target_arch = "tc32")]
    pub fn set_ack_filter(&mut self, pan_id: u16, short_address: u16, extended_address: [u8; 8]) {
        hw::set_ack_filter(pan_id, short_address, extended_address);
    }

    #[cfg(target_arch = "tc32")]
    pub fn transmit(&mut self, frame: &[u8]) -> TxOutcome {
        hw::send_mac_frame(frame)
    }

    #[cfg(target_arch = "tc32")]
    pub fn receive_raw_for(
        &mut self,
        timeout_ticks: u32,
        max_frames: u16,
        on_frame: impl FnMut(RawRxOutcome),
    ) -> u32 {
        hw::rx_raw_window_for(timeout_ticks, max_frames, on_frame)
    }

    #[cfg(target_arch = "tc32")]
    pub fn receive_raw_until(
        &mut self,
        timeout_ticks: u32,
        max_frames: u16,
        on_frame: impl FnMut(RawRxOutcome) -> bool,
    ) -> u32 {
        hw::rx_raw_window_until(timeout_ticks, max_frames, on_frame)
    }

    #[cfg(target_arch = "tc32")]
    pub fn measure_energy(&mut self) -> u8 {
        hw::measure_energy()
    }
}

#[cfg(target_arch = "tc32")]
mod hw {
    use core::sync::atomic::{Ordering, compiler_fence};

    use super::frame::{self, BeaconInfo};
    use super::{DMA_BUF_LEN, DmaBuf, MAX_MAC_FRAME_LEN, RawRxOutcome, ReceivedFrame, phy};
    use crate::timer;

    #[unsafe(link_section = ".rf_dma")]
    static mut RF_RX_BUF: [DmaBuf; 2] = [DmaBuf([0u8; DMA_BUF_LEN]), DmaBuf([0u8; DMA_BUF_LEN])];
    #[unsafe(link_section = ".rf_dma")]
    static mut RF_TX_BUF: DmaBuf = DmaBuf([0u8; DMA_BUF_LEN]);

    #[repr(C)]
    #[derive(Clone, Copy)]
    struct AckFilter {
        pan_id: u16,
        short_address: u16,
        extended_address: [u8; 8],
        enabled: u8,
    }

    static mut ACK_FILTER: AckFilter = AckFilter {
        pan_id: 0xFFFF,
        short_address: 0xFFFF,
        extended_address: [0; 8],
        enabled: 0,
    };
    static mut SOFTWARE_ACK_COUNT: u32 = 0;
    static mut SOFTWARE_ACK_TIMEOUT_COUNT: u32 = 0;
    static mut ACTIVE_RX_INDEX: u8 = 0;
    static mut RX_ARMED_AFTER_TX: u8 = 0;
    static mut CSMA_RNG_STATE: u32 = 0;
    static mut CCA_ATTEMPT_COUNT: u32 = 0;
    static mut CCA_BUSY_COUNT: u32 = 0;
    static mut CHANNEL_ACCESS_FAILURE_COUNT: u32 = 0;

    const CCA_THRESHOLD_DBM: i8 = -70;
    const CCA_RX_SETTLE_TICKS: u32 = timer::TICKS_PER_MS * 128 / 1_000;
    const CCA_SAMPLE_TICKS: u32 = timer::TICKS_PER_MS * 128 / 1_000;
    const UNIT_BACKOFF_TICKS: u32 = timer::TICKS_PER_MS * 320 / 1_000;
    const MAC_MIN_BE: u8 = 3;
    const MAC_MAX_BE: u8 = 5;
    const MAC_MAX_CSMA_BACKOFFS: u8 = 4;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum TxOutcome {
        Sent,
        InvalidFrame,
        ChannelAccessFailure,
        Timeout,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct CsmaStats {
        pub cca_attempts: u32,
        pub cca_busy: u32,
        pub channel_access_failures: u32,
    }

    /// Compile-time-ish alignment/placement facts the post-link script
    /// re-verifies from the linked ELF (see `verify_layout` in
    /// `scripts/tlsr8258.sh`): all DMA buffers must be 4-byte aligned and
    /// must live inside the `.rf_dma` section, i.e. outside the I-cache
    /// tag/data reservation.
    pub fn dma_buffers_aligned() -> bool {
        let rx0 = rx_buffer_ptr(0) as u32;
        let rx1 = rx_buffer_ptr(1) as u32;
        let tx = core::ptr::addr_of!(RF_TX_BUF) as u32;
        rx0 % 4 == 0 && rx1 % 4 == 0 && tx % 4 == 0
    }

    /// Bring up Timer0 + the RF PHY/DMA, and program channel 11 as the
    /// initial channel. Must run once, after `.data`/`.bss` init.
    pub fn init() {
        crate::mmio::disable_all_irqs();
        timer::init();
        set_active_rx_index(0);
        let rx_ptr = active_rx_ptr();
        phy::init(rx_ptr);
    }

    pub fn set_channel(channel: u8) {
        phy::set_channel(channel);
    }

    pub fn set_ack_filter(pan_id: u16, short_address: u16, extended_address: [u8; 8]) {
        unsafe {
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(ACK_FILTER),
                AckFilter {
                    pan_id,
                    short_address,
                    extended_address,
                    enabled: 1,
                },
            );
        }
    }

    pub fn software_ack_stats() -> (u32, u32) {
        unsafe {
            (
                core::ptr::read_volatile(core::ptr::addr_of!(SOFTWARE_ACK_COUNT)),
                core::ptr::read_volatile(core::ptr::addr_of!(SOFTWARE_ACK_TIMEOUT_COUNT)),
            )
        }
    }

    pub fn csma_stats() -> CsmaStats {
        unsafe {
            CsmaStats {
                cca_attempts: core::ptr::read_volatile(core::ptr::addr_of!(CCA_ATTEMPT_COUNT)),
                cca_busy: core::ptr::read_volatile(core::ptr::addr_of!(CCA_BUSY_COUNT)),
                channel_access_failures: core::ptr::read_volatile(core::ptr::addr_of!(
                    CHANNEL_ACCESS_FAILURE_COUNT
                )),
            }
        }
    }

    pub fn measure_energy() -> u8 {
        let rx_ptr = active_rx_ptr();
        phy::set_trx_off();
        phy::rx_done_clear();
        prepare_rx_dma(rx_ptr);
        phy::set_rx_mode();
        timer::sleep_ticks(CCA_RX_SETTLE_TICKS);

        let start = timer::now_ticks();
        let mut sum = 0i32;
        let mut samples = 0i32;
        loop {
            sum += phy::rssi_dbm() as i32;
            samples += 1;
            if timer::now_ticks().wrapping_sub(start) >= CCA_SAMPLE_TICKS {
                break;
            }
            unsafe { core::arch::asm!("nop") };
        }
        phy::set_trx_off();

        let rssi = (sum / samples).clamp(-99, -15);
        (255 * (rssi + 99) / 84) as u8
    }

    /// Fixed bound for one Beacon Request TX: settle + on-air time at 250
    /// kb/s for a short command frame is well under 1 ms; 5 ms leaves
    /// generous margin without ever blocking indefinitely.
    pub const TX_TIMEOUT_TICKS: u32 = timer::TICKS_PER_MS * 5;

    pub fn send_mac_frame(mac_frame: &[u8]) -> TxOutcome {
        if mac_frame.len() > MAX_MAC_FRAME_LEN {
            return TxOutcome::InvalidFrame;
        }
        let tx_ptr = core::ptr::addr_of_mut!(RF_TX_BUF) as *mut u8;
        let tx_slice = unsafe { core::slice::from_raw_parts_mut(tx_ptr, DMA_BUF_LEN) };
        // This call must execute in release builds: putting it inside
        // debug_assert! would remove the DMA-buffer write entirely.
        if frame::encode_tx_dma(tx_slice, mac_frame).is_err() {
            return TxOutcome::InvalidFrame;
        }
        compiler_fence(Ordering::Release);

        let rx_ptr = active_rx_ptr();
        set_rx_armed_after_tx(false);
        if !perform_csma_ca(rx_ptr) {
            return TxOutcome::ChannelAccessFailure;
        }

        phy::rx_done_clear();
        prepare_rx_dma(rx_ptr);

        phy::set_tx_dma_config(DMA_BUF_LEN as u16);
        phy::tx_done_clear();
        phy::set_tx_mode();
        // Settle delay before triggering DMA, matching the proven sensor-lab
        // sequence (PLL/analog settle after the mode-register write).
        timer::sleep_ticks(timer::ms(1) / 4); // ~0.25 ms fixed pause
        phy::tx_pkt(tx_ptr);

        let ok = timer::wait_until(TX_TIMEOUT_TICKS, phy::tx_done);
        if ok {
            phy::tx_done_clear();
            // An 802.15.4 ACK starts only 12 symbols after the transmitted
            // frame. Enter RX here, before returning to layout-sensitive
            // caller code, and leave the already-armed DMA buffer intact.
            phy::set_rx_mode();
            set_rx_armed_after_tx(true);
        } else {
            phy::set_trx_off();
        }
        if ok {
            TxOutcome::Sent
        } else {
            TxOutcome::Timeout
        }
    }

    /// Encode and transmit a Beacon Request with the given sequence number.
    pub fn send_beacon_request(seq: u8) -> TxOutcome {
        let mac_frame = frame::beacon_request_mac_frame(seq);
        send_mac_frame(&mac_frame)
    }

    /// One received-frame classification, produced by [`rx_window`].
    #[derive(Debug, Clone, Copy)]
    pub enum RxOutcome {
        /// Length-valid, CRC-valid, and MAC-parseable as a Beacon frame.
        Beacon {
            info: BeaconInfo,
            len: u8,
            lqi: u8,
            rssi: i8,
        },
        /// A valid ACK, including the Frame Pending bit needed by polling.
        Ack { sequence: u8, frame_pending: bool },
        /// A valid MAC Association Response command.
        AssociationResponse(frame::AssociationResponse),
        /// Length check (`RF_ZIGBEE_PACKET_LENGTH_OK`) failed.
        InvalidLength,
        /// Length was valid but the CRC/status check failed.
        InvalidCrc,
        /// Length- and CRC-valid, but not parseable as a Beacon (e.g. an ACK
        /// or a different command/data frame received during the window).
        NotABeacon { len: u8, lqi: u8, rssi: i8 },
    }

    /// Fixed bound for one RX window: long enough to catch a coordinator's
    /// beacon response after a Beacon Request, short enough that the
    /// channel-cycle loop in `mac_test` makes visible progress. Tuned to
    /// the same ~10 ms window used by the sensor lab's proven `scan_one`.
    pub const RX_WINDOW_TICKS: u32 = timer::TICKS_PER_MS * 10;

    /// Enter RX and poll for up to [`RX_WINDOW_TICKS`], classifying up to
    /// `max_frames` received frames via `on_frame`. Always returns after the
    /// fixed deadline (or after `max_frames` frames, whichever is first) —
    /// "no infinite wait for radio status".
    pub fn rx_window(max_frames: u16, on_frame: impl FnMut(RxOutcome)) -> u32 {
        rx_window_for(RX_WINDOW_TICKS, max_frames, on_frame)
    }

    pub fn rx_window_for(
        timeout_ticks: u32,
        max_frames: u16,
        mut on_frame: impl FnMut(RxOutcome),
    ) -> u32 {
        rx_raw_window_for(timeout_ticks, max_frames, |outcome| match outcome {
            RawRxOutcome::Frame(frame) => classify_and_report(&frame, &mut on_frame),
            RawRxOutcome::InvalidLength => on_frame(RxOutcome::InvalidLength),
            RawRxOutcome::InvalidCrc => on_frame(RxOutcome::InvalidCrc),
        })
    }

    /// Receive validated MAC frames without classifying their frame type.
    /// FCS bytes are removed because TLSR8258 validates them in hardware.
    pub fn rx_raw_window_for(
        timeout_ticks: u32,
        max_frames: u16,
        mut on_frame: impl FnMut(RawRxOutcome),
    ) -> u32 {
        rx_raw_window_until(timeout_ticks, max_frames, |outcome| {
            on_frame(outcome);
            false
        })
    }

    /// Receive validated MAC frames until the deadline, frame limit, or the
    /// callback reports that the caller has obtained the frame it needs.
    pub fn rx_raw_window_until(
        timeout_ticks: u32,
        max_frames: u16,
        mut on_frame: impl FnMut(RawRxOutcome) -> bool,
    ) -> u32 {
        let mut rx_ptr = active_rx_ptr();

        if !take_rx_armed_after_tx() {
            phy::set_trx_off();
            phy::rx_done_clear();
            rearm_rx(rx_ptr);
        }

        let start = timer::now_ticks();
        let mut frames_seen: u16 = 0;
        loop {
            if timer::now_ticks().wrapping_sub(start) >= timeout_ticks {
                break;
            }
            if frames_seen >= max_frames {
                break;
            }
            if phy::rx_done() {
                phy::rx_done_clear();
                compiler_fence(Ordering::Acquire);
                maybe_send_software_ack(rx_ptr);
                let completed_rx_ptr = rx_ptr;
                rx_ptr = rotate_rx_buffer();
                rearm_rx(rx_ptr);
                let mut snapshot = [0u8; DMA_BUF_LEN];
                for (i, byte) in snapshot.iter_mut().enumerate() {
                    *byte = unsafe { core::ptr::read_volatile(completed_rx_ptr.add(i)) };
                }
                frames_seen += 1;
                if on_frame(decode_received_frame(&snapshot)) {
                    break;
                }
            }
            unsafe { core::arch::asm!("nop") };
        }
        let elapsed = timer::now_ticks().wrapping_sub(start);
        phy::set_trx_off();
        elapsed
    }

    fn set_rx_armed_after_tx(armed: bool) {
        unsafe {
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(RX_ARMED_AFTER_TX),
                if armed { 1 } else { 0 },
            );
        }
    }

    fn rx_buffer_ptr(index: u8) -> *mut u8 {
        debug_assert!(index < 2);
        unsafe {
            core::ptr::addr_of_mut!(RF_RX_BUF)
                .cast::<DmaBuf>()
                .add(index as usize)
                .cast::<u8>()
        }
    }

    fn active_rx_index() -> u8 {
        unsafe { core::ptr::read_volatile(core::ptr::addr_of!(ACTIVE_RX_INDEX)) & 1 }
    }

    fn set_active_rx_index(index: u8) {
        unsafe {
            core::ptr::write_volatile(core::ptr::addr_of_mut!(ACTIVE_RX_INDEX), index & 1);
        }
    }

    fn active_rx_ptr() -> *mut u8 {
        rx_buffer_ptr(active_rx_index())
    }

    fn rotate_rx_buffer() -> *mut u8 {
        let next = active_rx_index() ^ 1;
        set_active_rx_index(next);
        rx_buffer_ptr(next)
    }

    fn take_rx_armed_after_tx() -> bool {
        unsafe {
            let ptr = core::ptr::addr_of_mut!(RX_ARMED_AFTER_TX);
            let armed = core::ptr::read_volatile(ptr) != 0;
            core::ptr::write_volatile(ptr, 0);
            armed
        }
    }

    fn prepare_rx_dma(rx_ptr: *mut u8) {
        unsafe {
            core::ptr::write_volatile(rx_ptr, 0);
            core::ptr::write_volatile(rx_ptr.add(4), 0);
        }
        phy::set_rx_buffer(rx_ptr);
        phy::enable_dma_rx();
    }

    fn rearm_rx(rx_ptr: *mut u8) {
        prepare_rx_dma(rx_ptr);
        phy::set_rx_mode();
    }

    fn perform_csma_ca(rx_ptr: *mut u8) -> bool {
        let mut backoffs = 0u8;
        let mut backoff_exponent = MAC_MIN_BE;

        loop {
            let slots = next_random() & ((1u32 << backoff_exponent) - 1);

            phy::set_trx_off();
            phy::rx_done_clear();
            prepare_rx_dma(rx_ptr);
            phy::set_rx_mode();
            timer::sleep_ticks(CCA_RX_SETTLE_TICKS);
            if slots != 0 {
                timer::sleep_ticks(slots * UNIT_BACKOFF_TICKS);
            }

            increment_counter(core::ptr::addr_of_mut!(CCA_ATTEMPT_COUNT));
            if channel_is_clear() {
                phy::set_trx_off();
                phy::rx_done_clear();
                return true;
            }

            increment_counter(core::ptr::addr_of_mut!(CCA_BUSY_COUNT));
            backoffs = backoffs.saturating_add(1);
            if backoffs > MAC_MAX_CSMA_BACKOFFS {
                increment_counter(core::ptr::addr_of_mut!(CHANNEL_ACCESS_FAILURE_COUNT));
                phy::set_trx_off();
                phy::rx_done_clear();
                return false;
            }
            backoff_exponent = backoff_exponent.saturating_add(1).min(MAC_MAX_BE);
        }
    }

    fn channel_is_clear() -> bool {
        let start = timer::now_ticks();
        let mut sum = 0i32;
        let mut samples = 0i32;
        let mut received_frame = false;
        loop {
            sum += phy::rssi_dbm() as i32;
            samples += 1;
            received_frame |= phy::rx_done();
            if timer::now_ticks().wrapping_sub(start) >= CCA_SAMPLE_TICKS {
                break;
            }
            unsafe { core::arch::asm!("nop") };
        }
        !received_frame && sum / samples <= CCA_THRESHOLD_DBM as i32
    }

    fn next_random() -> u32 {
        unsafe {
            let ptr = core::ptr::addr_of_mut!(CSMA_RNG_STATE);
            let mut value = core::ptr::read_volatile(ptr)
                ^ timer::now_ticks().rotate_left(11)
                ^ ((phy::rssi_dbm() as u8 as u32) << 24);
            if value == 0 {
                value = 0xA536_6B4D;
            }
            value ^= value << 13;
            value ^= value >> 17;
            value ^= value << 5;
            core::ptr::write_volatile(ptr, value);
            value
        }
    }

    fn increment_counter(ptr: *mut u32) {
        unsafe {
            let value = core::ptr::read_volatile(ptr);
            core::ptr::write_volatile(ptr, value.wrapping_add(1));
        }
    }

    fn maybe_send_software_ack(rx_ptr: *mut u8) {
        let filter = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(ACK_FILTER)) };
        if filter.enabled == 0 {
            return;
        }

        let total_len = unsafe { core::ptr::read_volatile(rx_ptr) } as usize;
        let payload_len = unsafe { core::ptr::read_volatile(rx_ptr.add(4)) } as usize;
        if total_len == 0 || total_len > 136 || total_len != payload_len + 9 || payload_len < 7 {
            return;
        }
        let status = unsafe { core::ptr::read_volatile(rx_ptr.add(total_len + 3)) };
        if status & 0x51 != 0x10 {
            return;
        }

        let frame_control =
            u16::from_le_bytes(
                [unsafe { core::ptr::read_volatile(rx_ptr.add(5)) }, unsafe {
                    core::ptr::read_volatile(rx_ptr.add(6))
                }],
            );
        if frame_control & (1 << 5) == 0 {
            return;
        }

        let destination_pan =
            u16::from_le_bytes(
                [unsafe { core::ptr::read_volatile(rx_ptr.add(8)) }, unsafe {
                    core::ptr::read_volatile(rx_ptr.add(9))
                }],
            );
        if destination_pan != filter.pan_id && destination_pan != 0xFFFF {
            return;
        }

        let destination_mode = (frame_control >> 10) & 0x03;
        let addressed_to_us = match destination_mode {
            0x02 => {
                let destination = u16::from_le_bytes([
                    unsafe { core::ptr::read_volatile(rx_ptr.add(10)) },
                    unsafe { core::ptr::read_volatile(rx_ptr.add(11)) },
                ]);
                filter.short_address != 0xFFFF && destination == filter.short_address
            }
            0x03 if payload_len >= 13 => {
                let mut matches = true;
                let mut index = 0;
                while index < 8 {
                    let byte = unsafe { core::ptr::read_volatile(rx_ptr.add(10 + index)) };
                    matches &= byte == filter.extended_address[index];
                    index += 1;
                }
                matches
            }
            _ => false,
        };
        if !addressed_to_us {
            return;
        }

        let sequence = unsafe { core::ptr::read_volatile(rx_ptr.add(7)) };
        if send_ack_fast(sequence) {
            unsafe {
                let count = core::ptr::read_volatile(core::ptr::addr_of!(SOFTWARE_ACK_COUNT));
                core::ptr::write_volatile(
                    core::ptr::addr_of_mut!(SOFTWARE_ACK_COUNT),
                    count.wrapping_add(1),
                );
            }
        } else {
            unsafe {
                let count =
                    core::ptr::read_volatile(core::ptr::addr_of!(SOFTWARE_ACK_TIMEOUT_COUNT));
                core::ptr::write_volatile(
                    core::ptr::addr_of_mut!(SOFTWARE_ACK_TIMEOUT_COUNT),
                    count.wrapping_add(1),
                );
            }
        }
    }

    fn send_ack_fast(sequence: u8) -> bool {
        let tx_ptr = core::ptr::addr_of_mut!(RF_TX_BUF) as *mut u8;
        unsafe {
            core::ptr::write_volatile(tx_ptr, 4);
            core::ptr::write_volatile(tx_ptr.add(1), 0);
            core::ptr::write_volatile(tx_ptr.add(2), 0);
            core::ptr::write_volatile(tx_ptr.add(3), 0);
            core::ptr::write_volatile(tx_ptr.add(4), 5);
            core::ptr::write_volatile(tx_ptr.add(5), 0x02);
            core::ptr::write_volatile(tx_ptr.add(6), 0);
            core::ptr::write_volatile(tx_ptr.add(7), sequence);
        }
        compiler_fence(Ordering::Release);
        phy::disable_dma_rx();
        phy::disable_rx_mode();
        phy::set_tx_dma_config(DMA_BUF_LEN as u16);
        phy::tx_done_clear();
        phy::set_tx_mode();

        // The TLSR8258 radio needs the same RX->TX settle used by the
        // official Zigbee SDK before transmitting a MAC ACK.
        timer::sleep_ticks(timer::us(120));
        phy::tx_pkt(tx_ptr);
        let sent = timer::wait_until(timer::TICKS_PER_MS, phy::tx_done);
        if sent {
            phy::tx_done_clear();
        }
        sent
    }

    fn decode_received_frame(buf: &[u8]) -> RawRxOutcome {
        if !frame::packet_length_ok(buf) {
            return RawRxOutcome::InvalidLength;
        }
        if !frame::packet_crc_ok(buf) {
            return RawRxOutcome::InvalidCrc;
        }
        let dma_len = frame::payload_len(buf) as usize;
        if dma_len < 2 || dma_len - 2 > MAX_MAC_FRAME_LEN {
            return RawRxOutcome::InvalidLength;
        }
        let rssi = frame::packet_rssi(buf);
        let lqi = frame::rssi_to_lqi(rssi);
        let Some(psdu) = frame::mac_psdu(buf) else {
            return RawRxOutcome::InvalidLength;
        };
        let frame_len = dma_len - 2;
        let mut data = [0u8; MAX_MAC_FRAME_LEN];
        data[..frame_len].copy_from_slice(&psdu[..frame_len]);
        RawRxOutcome::Frame(ReceivedFrame {
            data,
            len: frame_len as u8,
            lqi,
            rssi,
        })
    }

    fn classify_and_report(received: &ReceivedFrame, on_frame: &mut impl FnMut(RxOutcome)) {
        let psdu = received.as_slice();
        if let Some((sequence, frame_pending)) = frame::ack_info(psdu) {
            on_frame(RxOutcome::Ack {
                sequence,
                frame_pending,
            });
            return;
        }
        if let Some(response) = frame::parse_association_response(psdu) {
            on_frame(RxOutcome::AssociationResponse(response));
            return;
        }
        match frame::parse_beacon(psdu) {
            Some(info) => on_frame(RxOutcome::Beacon {
                info,
                len: received.len() as u8,
                lqi: received.lqi,
                rssi: received.rssi,
            }),
            None => on_frame(RxOutcome::NotABeacon {
                len: received.len() as u8,
                lqi: received.lqi,
                rssi: received.rssi,
            }),
        }
    }
}

#[cfg(target_arch = "tc32")]
pub use hw::{
    CsmaStats, RX_WINDOW_TICKS, RxOutcome, TX_TIMEOUT_TICKS, TxOutcome, csma_stats,
    dma_buffers_aligned, init, rx_raw_window_for, rx_window, rx_window_for, send_beacon_request,
    send_mac_frame, set_ack_filter, set_channel, software_ack_stats,
};
