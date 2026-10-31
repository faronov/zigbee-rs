//! CC2340R5 [`RadioPhy`] backend for the shared [`SoftMacCore`].
//!
//! The host-side radio driver is Rust and accesses the LRFD registers
//! directly. TI's BSD-licensed PBE/MCE/RFE firmware images remain embedded as
//! radio microcode data; no RCL, ZBOSS, or MAC shim library is linked.
//!
//! # Architecture
//! ```text
//! SoftMacCore (scan, associate, poll, CSMA-CA, retries, PIB)
//!        │ RadioPhy
//!        ▼
//! Cc2340RadioPhy (this module)
//!   ├── RX FIFO hygiene (stale ACKs, sleepy-device broadcasts), dedup
//!   └── Cc2340Driver (driver.rs)
//!          ├── generated TI PHY settings and PA table
//!          ├── direct LRFD trim, synthesizer, and TOPsm setup
//!          ├── official PBE/MCE/RFE firmware loaded as data
//!          └── polling FIFO TX/RX, timer-paced so the CPU sleeps between polls
//! ```
//!
//! # Hardware filtering and acknowledgement
//!
//! The PBE does the time-critical parts of IEEE 802.15.4 itself, configured
//! as TI's RCL `RCL_CMD_IEEE_RX_TX` handler does, so the PHY uses
//! [`AckWaitTx`](crate::AckWaitTx):
//!
//! * **Frame filtering:** only beacons, data and MAC commands addressed to our
//!   PAN/short/extended address (or broadcast) reach the RX FIFO. While the
//!   MAC disables the address filter (scan) the PAN is `0xFFFF`, so every
//!   beacon passes. There is no raw sniffer mode.
//! * **Incoming:** AR frames that pass the filter are acknowledged by the PBE
//!   192 µs after the frame, always with frame pending = 0 (no source-match
//!   table), so [`RadioPhy::set_ack_frame_pending`] stays unsupported and a
//!   parent role is not available.
//! * **Outgoing:** [`RadioPhy::try_transmit_acknowledged`] performs one host
//!   CCA on the live RX operation and the TX, then the PBE listens for the
//!   immediate ACK itself (`ACKSTATUS`). CSMA-CA backoff and
//!   `macMaxFrameRetries` belong to [`SoftMacCore`].
//!
//! # RX FIFO as the receive buffer
//!
//! The RX operation keeps running between frames, so the PBE commits PAN
//! traffic and the ACKs of our own transmissions to the 288-byte RX FIFO even
//! when no MAC receive is pending. A full FIFO stops RSSI updates (CCA then
//! reports busy forever) and drops the frames our TX solicits. Instead of a
//! second RAM queue the PHY keeps the FIFO itself clean before every TX,
//! sleep and wait: stale ACKs (and MAC broadcasts on a sleepy device) are
//! released from its head without being read, and a frame that still has to
//! be delivered is kept there. After a Data Request whose ACK has frame
//! pending = 0, such a held frame is reported as frame pending so that the
//! MAC's poll reads it.

mod config;
pub mod driver;
mod fifo;
mod firmware;
mod hardware;

use embassy_time::{Duration, Instant, Timer};
use zigbee_types::{IeeeAddress, MacAddress};

use crate::frames;
use crate::pib::{MacPib, PibAttribute, PibValue};
use crate::{
    AckResult, EndDevice, MacError, PhyAddressFilter, PhyCapabilities, PhyError, PhyRxFrame,
    PlatformServices, RadioPhy, SoftMacCore,
};
use driver::Cc2340Driver;
pub use driver::{DriverStats, RadioConfig, RadioError, RxInfo};
pub use fifo::PbeCounters;

/// Retained-RX depth of the CC2340 end-device MAC; the depth of the former
/// bespoke `Cc2340Mac` queue. Each slot is one [`PhyRxFrame`].
pub const CC2340_PENDING_RX: usize = 8;

/// Shared software MAC (end-device role) on the CC2340 radio.
///
/// Construct with [`Cc2340RadioPhy::into_soft_mac`].
pub type Cc2340SoftMac = SoftMacCore<Cc2340RadioPhy, EndDevice, CC2340_PENDING_RX>;

/// Lowest and highest TX power of the CC2340R5 PA table, in dBm.
const TX_POWER_MIN_DBM: i8 = -20;
const TX_POWER_MAX_DBM: i8 = 8;
/// Default TX power, kept from the hardware-tested bespoke MAC.
const TX_POWER_DEFAULT_DBM: i8 = 5;

const FRAME_TYPE_DATA: u8 = 0x01;
const FRAME_TYPE_ACK: u8 = 0x02;
const FRAME_TYPE_COMMAND: u8 = 0x03;
const FRAME_CONTROL_ACK_REQUEST: u8 = 0x20;
const COMMAND_DATA_REQUEST: u8 = 0x04;

/// PHY-level counters, cumulative since boot.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PhyStats {
    /// Repeated frames (same source and DSN) acknowledged but not delivered.
    pub duplicates: u32,
    /// Undelivered frames released from the RX FIFO to keep room for one
    /// maximum-size frame.
    pub fifo_evictions: u32,
}

/// Source of the last accepted frame and its DSN, for duplicate rejection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct LastRx {
    source: MacAddress,
    sequence: u8,
}

/// Invalid factory or configured IEEE identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IdentityError {
    UnprogrammedExtendedAddress,
}

/// The CC2340 backend has no independently verified entropy peripheral.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntropyDisposition {
    /// `PlatformServices::fill_random` returns `MacError::Unsupported` and
    /// never substitutes deterministic bytes for cryptographic entropy.
    UnavailableFailClosed,
}

/// Radio power states exposed by the current direct-register backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadioSleepDisposition {
    /// [`RadioPhy::sleep`] stops the PBE operation, powers down the TOPsm
    /// cores and synthesizer reference and gates the LRFD clocks; the LRFD
    /// stays powered, so firmware images, PHY registers, trims and the RX
    /// FIFO are retained. The 48 MHz crystal (HFXT) is also stopped (the CPU
    /// keeps running from HFOSC); [`RadioPhy::wake`] restarts HFXT (busy-wait
    /// for amplitude settle) and re-enables the clocks. Device standby (LRFD
    /// register loss, LF time base) is not claimed.
    ClockGatedIdle,
    /// [`Self::ClockGatedIdle`], after which the caller may put the whole
    /// device into standby (LRFD power domain off) and report it with
    /// [`Cc2340RadioPhy::note_standby`]; the next wake then re-runs the full
    /// radio bring-up (firmware images, trims, PHY, synthesizer, PA) from the
    /// retained software configuration, as RCL does after
    /// `PowerLPF3_AWAKE_STANDBY`. Standby loses the RX FIFO, so it should
    /// not be entered while [`Cc2340RadioPhy::holds_received_frame`].
    /// Standby entry itself (LF clock, RTC wake, `HapiEnterStandby`) belongs
    /// to the board.
    StandbyCapable,
}

/// CC2340R5 IEEE 802.15.4 radio exposed through the portable PHY contract.
pub struct Cc2340RadioPhy {
    driver: Cc2340Driver,
    /// The MAC programmed an address filter (`false` while scanning).
    filtered: bool,
    /// Deliver MAC broadcasts while filtered (receiver on when idle).
    deliver_broadcasts: bool,
    /// The last Data Request was reported as frame pending only for frames
    /// already in the RX FIFO, so the poll's receive ends once it is empty.
    fifo_only_poll: bool,
    /// [`RadioPhy::sleep`] left an undelivered frame in the RX FIFO.
    holds_frame: bool,
    last_rx: Option<LastRx>,
    stats: PhyStats,
}

impl Cc2340RadioPhy {
    /// Create the PHY with a validated factory EUI-64.
    ///
    /// Erased all-zero and all-`0xFF` identities are rejected before any
    /// radio hardware is touched.
    pub fn new(extended_address: IeeeAddress) -> Result<Self, IdentityError> {
        if !valid_ieee_address(extended_address) {
            return Err(IdentityError::UnprogrammedExtendedAddress);
        }
        Ok(Self {
            driver: Cc2340Driver::new(RadioConfig {
                ieee_addr: extended_address,
                tx_power_dbm: TX_POWER_DEFAULT_DBM,
                ..RadioConfig::default()
            }),
            filtered: true,
            deliver_broadcasts: false,
            fifo_only_poll: false,
            holds_frame: false,
            last_rx: None,
            stats: PhyStats {
                duplicates: 0,
                fifo_evictions: 0,
            },
        })
    }

    /// Build the end-device [`Cc2340SoftMac`] on top of this PHY.
    ///
    /// Call [`Self::initialize_radio`] first. `macDSN`/`macBSN` start at 0
    /// (there is no verified entropy source); `MLME-RESET` re-seeds them.
    #[inline(never)]
    pub fn into_soft_mac(self) -> Result<Cc2340SoftMac, MacError> {
        let mut pib = MacPib::new(self.extended_address(), 0, 0);
        pib.set(
            PibAttribute::PhyTransmitPower,
            PibValue::I8(TX_POWER_DEFAULT_DBM),
        )
        .map_err(|_| MacError::InvalidParameter)?;
        SoftMacCore::new_custom(self, pib)
    }

    /// Return the EUI-64 selected before PHY construction.
    pub const fn extended_address(&self) -> IeeeAddress {
        self.driver.config().ieee_addr
    }

    /// Initialize the radio while preserving the low-level availability error.
    ///
    /// Compile-only images therefore distinguish `FirmwareUnavailable` from
    /// `RadioConfigUnavailable` without collapsing either into `PhyError`.
    pub fn initialize_radio(&mut self) -> Result<(), RadioError> {
        // Checked before the driver's runtime `configured` state (which can
        // never be set without firmware) so a fallback build folds this to a
        // constant error and drops the unreachable device storage.
        if firmware::images().is_none() {
            return Err(RadioError::FirmwareUnavailable);
        }
        self.driver.init()
    }

    pub const fn entropy_disposition(&self) -> EntropyDisposition {
        EntropyDisposition::UnavailableFailClosed
    }

    pub const fn radio_sleep_disposition(&self) -> RadioSleepDisposition {
        RadioSleepDisposition::StandbyCapable
    }

    /// Deliver MAC broadcast frames while the address filter is programmed.
    ///
    /// Off by default: a sleepy end device's parent relays broadcasts to it
    /// as indirect unicast, and on a busy PAN the broadcasts heard while
    /// awake would otherwise crowd the polled frames out of the RX FIFO and
    /// the retained-RX queue. Enable it for a receiver-on-when-idle device.
    pub fn set_broadcast_delivery(&mut self, enabled: bool) {
        self.deliver_broadcasts = enabled;
    }

    /// Quiesce any in-flight operation before an active-clock wait, after
    /// releasing stale RX FIFO entries.
    ///
    /// This deliberately does not power the radio down; use
    /// [`RadioPhy::sleep`] for that.
    pub fn prepare_active_wait(&mut self) -> Result<(), RadioError> {
        self.release_stale_rx();
        self.driver.prepare_active_wait()
    }

    /// Whether the radio is currently powered down by [`RadioPhy::sleep`].
    pub const fn is_asleep(&self) -> bool {
        self.driver.is_asleep()
    }

    /// Whether [`RadioPhy::sleep`] left an undelivered frame in the RX FIFO;
    /// device standby would lose it.
    pub const fn holds_received_frame(&self) -> bool {
        self.holds_frame
    }

    /// Report that the device was in standby since [`RadioPhy::sleep`] (see
    /// [`RadioSleepDisposition::StandbyCapable`]). Returns `false` if the
    /// radio was not asleep, i.e. standby was entered out of sequence.
    pub fn note_standby(&mut self) -> bool {
        let noted = self.driver.note_standby();
        if noted {
            self.holds_frame = false;
        }
        noted
    }

    /// PHY and driver counters.
    pub fn stats(&self) -> (PhyStats, DriverStats, PbeCounters) {
        (self.stats, self.driver.stats(), self.driver.pbe_counters())
    }

    /// Release stale entries from the head of the RX FIFO and keep room for
    /// one maximum-size frame.
    ///
    /// Stale entries (see [`discard_before_mac`]) are dropped unread. The
    /// first entry the MAC still has to see stays in place; only if the FIFO
    /// is too full to receive a maximum-size frame behind it is it evicted.
    fn release_stale_rx(&mut self) {
        let drop_broadcasts = self.drops_broadcasts();
        while let Some(head) = self.driver.peek_rx_head() {
            if !discard_before_mac(head.prefix(), drop_broadcasts) {
                let free = self.driver.rx_free_bytes().unwrap_or(0);
                if free >= fifo::MAX_FIFO_ENTRY_BYTES {
                    return;
                }
                self.stats.fifo_evictions = self.stats.fifo_evictions.wrapping_add(1);
                log::warn!("[CC2340] RX FIFO full: undelivered frame evicted");
            }
            self.driver.discard_rx_head(&head);
        }
    }

    const fn drops_broadcasts(&self) -> bool {
        self.filtered && !self.deliver_broadcasts
    }

    /// Classify the frame the driver wrote to `frame`'s buffer and commit it;
    /// `false` if the MAC must not see it.
    fn accept(&mut self, frame: &mut PhyRxFrame, rx: RxInfo) -> bool {
        let data = &frame.buffer_mut()[..rx.len];
        let prefix = &data[..data.len().min(fifo::RX_HEAD_PREFIX_LEN)];
        if discard_before_mac(prefix, self.drops_broadcasts()) {
            return false;
        }
        if let Some(key) = duplicate_key(data) {
            if self.last_rx == Some(key) {
                // The PBE acknowledged the retransmission (the sender missed
                // our first ACK); deliver the frame only once.
                self.stats.duplicates = self.stats.duplicates.wrapping_add(1);
                return false;
            }
            self.last_rx = Some(key);
        }
        frame.set_len(rx.len, rx.lqi).is_ok()
    }

    /// Before a TX: release stale entries or, while scanning, everything
    /// received on the previous channel.
    fn prepare_transmit(&mut self) {
        self.fifo_only_poll = false;
        if self.filtered {
            self.release_stale_rx();
        } else {
            self.driver.flush_rx_fifo();
        }
    }
}

impl RadioPhy for Cc2340RadioPhy {
    type Tx = crate::AckWaitTx;
    /// macMaxFrameTotalWaitTime plus parent software latency; the value of
    /// the hardware-tested bespoke MAC.
    const POLL_RESPONSE_WAIT_US: u32 = 100_000;
    const DEFAULT_TX_POWER_DBM: i8 = TX_POWER_DEFAULT_DBM;

    fn capabilities(&self) -> PhyCapabilities {
        PhyCapabilities {
            hardware_auto_ack: true,
            hardware_address_filter: true,
            tx_power_min: TX_POWER_MIN_DBM,
            tx_power_max: TX_POWER_MAX_DBM,
        }
    }

    async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError> {
        self.prepare_transmit();
        self.driver.transmit(frame).await.map_err(map_tx_error)
    }

    async fn send_ack(&mut self, _sequence: u8, _frame_pending: bool) -> Result<(), PhyError> {
        // The PBE acknowledges in hardware; the MAC never asks for this.
        Err(PhyError::Unsupported)
    }

    async fn receive(&mut self, timeout_us: u32, frame: &mut PhyRxFrame) -> Result<bool, PhyError> {
        if self.fifo_only_poll {
            while let Some(rx) = self.driver.try_receive(frame.buffer_mut()) {
                if self.accept(frame, rx) {
                    return Ok(true);
                }
            }
            self.fifo_only_poll = false;
            return Ok(false);
        }

        let deadline = Instant::now() + Duration::from_micros(u64::from(timeout_us));
        loop {
            match self.driver.receive(deadline, frame.buffer_mut()).await {
                Ok(Some(rx)) => {
                    if self.accept(frame, rx) {
                        return Ok(true);
                    }
                }
                Ok(None) => return Ok(false),
                Err(error) => receive_error(error)?,
            }
            if Instant::now() >= deadline {
                return Ok(false);
            }
        }
    }

    fn set_channel(&mut self, channel: u8) -> Result<(), PhyError> {
        if !(11..=26).contains(&channel) {
            return Err(PhyError::InvalidChannel);
        }
        self.driver.set_channel(channel);
        Ok(())
    }

    fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
        if !(TX_POWER_MIN_DBM..=TX_POWER_MAX_DBM).contains(&dbm) {
            return Err(PhyError::Unsupported);
        }
        self.driver.set_tx_power(dbm);
        Ok(())
    }

    async fn energy_detect(&mut self, duration_us: u32) -> Result<u8, PhyError> {
        let deadline = Instant::now() + Duration::from_micros(u64::from(duration_us));
        let mut strongest = i8::MIN;
        loop {
            strongest = strongest.max(self.driver.read_rssi());
            if Instant::now() >= deadline {
                return Ok(rssi_to_ed(strongest));
            }
            Timer::after_millis(1).await;
        }
    }

    fn set_address_filter(&mut self, filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
        self.filtered = filter.is_some();
        self.driver.update_config(|config| {
            config.promiscuous = false;
            match filter {
                Some(filter) => {
                    config.pan_id = filter.pan_id.0;
                    config.short_addr = filter.short_address.0;
                    config.ieee_addr = filter.extended_address;
                }
                // Scan: accept the beacons and broadcasts of every PAN. ACK
                // matching needs the PBE filter, so this is not the PBE's
                // promiscuous mode.
                None => config.pan_id = 0xFFFF,
            }
        });
        Ok(())
    }

    async fn try_transmit_acknowledged(
        &mut self,
        frame: &[u8],
    ) -> Result<Option<AckResult>, PhyError> {
        self.prepare_transmit();
        let frame_pending = match self.driver.transmit_with_ack(frame).await {
            Ok(frame_pending) => frame_pending,
            Err(RadioError::NoAck) => return Ok(None),
            Err(error) => return Err(map_tx_error(error)),
        };
        if !frame_pending && is_data_request(frame) {
            // A frame that arrived before the poll is still in the FIFO; let
            // the MAC's poll read it instead of waiting for the next poll.
            self.release_stale_rx();
            if self.driver.peek_rx_head().is_some() {
                self.fifo_only_poll = true;
                return Ok(Some(AckResult {
                    frame_pending: true,
                }));
            }
        }
        Ok(Some(AckResult { frame_pending }))
    }

    fn sleep(&mut self) -> Result<(), PhyError> {
        self.release_stale_rx();
        self.holds_frame = self.driver.peek_rx_head().is_some();
        self.driver.sleep().map_err(|error| {
            log::warn!("[CC2340] radio sleep failed: {error:?}");
            PhyError::Hardware
        })
    }

    fn wake(&mut self) -> Result<(), PhyError> {
        self.driver.wake().map_err(|error| {
            log::warn!("[CC2340] radio wake failed: {error:?}");
            PhyError::Hardware
        })
    }
}

impl zigbee_crypto::ForwardAesProvider for Cc2340RadioPhy {}
impl PlatformServices for Cc2340RadioPhy {
    fn monotonic_micros(&self) -> u32 {
        Instant::now().as_micros() as u32
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        Timer::after_micros(u64::from(duration_us)).await;
    }

    fn fill_random(&mut self, _output: &mut [u8]) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }
}

/// Whether a received frame, given by its first bytes, is dropped before the
/// MAC sees it: ACKs (the PBE already matched ours) and, with
/// `drop_broadcasts`, data and command frames to the broadcast short
/// address.
fn discard_before_mac(prefix: &[u8], drop_broadcasts: bool) -> bool {
    let Some(&frame_control) = prefix.first() else {
        return true;
    };
    match frame_control & 0x07 {
        FRAME_TYPE_ACK => true,
        FRAME_TYPE_DATA | FRAME_TYPE_COMMAND => drop_broadcasts && is_short_broadcast(prefix),
        _ => false,
    }
}

/// Destination is the short broadcast address `0xFFFF`.
fn is_short_broadcast(prefix: &[u8]) -> bool {
    matches!(prefix, [_, fc_high, _, _, _, 0xFF, 0xFF, ..] if (fc_high >> 2) & 0x03 == 0x02)
}

/// MAC command frame carrying a Data Request (IEEE 802.15.4-2006 frame
/// layout, no auxiliary security header: the MAC never secures it).
fn is_data_request(frame: &[u8]) -> bool {
    let [frame_control, frame_control_high, ..] = *frame else {
        return false;
    };
    if frame_control & 0x07 != FRAME_TYPE_COMMAND {
        return false;
    }
    let address_len = |mode: u8| match mode & 0x03 {
        0x02 => 2,
        0x03 => 8,
        _ => 0,
    };
    let destination = address_len(frame_control_high >> 2);
    let source = address_len(frame_control_high >> 6);
    let destination_pan = if destination != 0 { 2 } else { 0 };
    let pan_compressed = frame_control & 0x40 != 0;
    let source_pan = if source != 0 && !pan_compressed { 2 } else { 0 };
    let offset = 3 + destination_pan + destination + source_pan + source;
    frame.get(offset) == Some(&COMMAND_DATA_REQUEST)
}

fn duplicate_key(data: &[u8]) -> Option<LastRx> {
    if data.len() < 3 || data[0] & FRAME_CONTROL_ACK_REQUEST == 0 {
        return None;
    }
    let frame_type = data[0] & 0x07;
    if frame_type != FRAME_TYPE_DATA && frame_type != FRAME_TYPE_COMMAND {
        return None;
    }
    let frame_control = u16::from_le_bytes([data[0], data[1]]);
    Some(LastRx {
        source: frames::parse_source_address(data, frame_control)?,
        sequence: data[2],
    })
}

/// Map the strongest RSSI of an ED measurement to the IEEE ED scale
/// (computed in `i32`: the former `i16` product overflowed above 0 dBm).
fn rssi_to_ed(rssi: i8) -> u8 {
    ((i32::from(rssi) + 128) * 255 / 256) as u8
}

fn map_tx_error(error: RadioError) -> PhyError {
    match error {
        RadioError::InvalidFrame => PhyError::FrameTooLong,
        RadioError::InvalidChannel => PhyError::InvalidChannel,
        // A configuration that can never transmit fails the request.
        RadioError::RadioConfigUnavailable
        | RadioError::FirmwareUnavailable
        | RadioError::InvalidTxPower
        | RadioError::FirmwareImageTooLarge
        | RadioError::FactoryTrimUnavailable
        | RadioError::SynthConfigInvalid => {
            log::warn!("[CC2340] TX impossible: {error:?}");
            PhyError::Hardware
        }
        // A busy CCA or an aborted transmission consumes one CSMA-CA attempt,
        // as in the bespoke MAC this backend replaces.
        _ => {
            log::debug!("[CC2340] TX attempt failed: {error:?}");
            PhyError::ChannelBusy
        }
    }
}

/// A receive error either fails the MAC request (radio unusable) or is
/// transient: the driver has restarted RX and the wait continues, as in the
/// bespoke MAC this backend replaces.
fn receive_error(error: RadioError) -> Result<(), PhyError> {
    match error {
        RadioError::RadioConfigUnavailable | RadioError::FirmwareUnavailable => {
            Err(PhyError::Hardware)
        }
        _ => {
            log::debug!("[CC2340] RX error, continuing: {error:?}");
            Ok(())
        }
    }
}

/// Validate an EUI-64 without imposing a byte-order interpretation.
pub const fn valid_ieee_address(address: IeeeAddress) -> bool {
    let mut index = 0;
    let mut all_zero = true;
    let mut all_erased = true;
    while index < address.len() {
        all_zero &= address[index] == 0;
        all_erased &= address[index] == 0xFF;
        index += 1;
    }
    !all_zero && !all_erased
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_types::{PanId, ShortAddress};

    const VALID_IEEE: IeeeAddress = [0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE];

    /// Data frame, PAN-compressed short → short, AR set.
    fn data_frame(seq: u8, dst: u16, src: u16) -> heapless::Vec<u8, 127> {
        let mut frame = heapless::Vec::new();
        frame
            .extend_from_slice(&[0x61, 0x88, seq, 0x34, 0x12])
            .unwrap();
        frame.extend_from_slice(&dst.to_le_bytes()).unwrap();
        frame.extend_from_slice(&src.to_le_bytes()).unwrap();
        frame.push(0xAA).unwrap();
        frame
    }

    /// Run `accept` on `data` as if the driver had written it to the slot.
    fn accept(phy: &mut Cc2340RadioPhy, data: &[u8]) -> Option<PhyRxFrame> {
        let mut frame = PhyRxFrame::EMPTY;
        frame.buffer_mut()[..data.len()].copy_from_slice(data);
        let info = RxInfo {
            len: data.len(),
            rssi: -40,
            lqi: 200,
            modem_lqi: None,
        };
        phy.accept(&mut frame, info).then_some(frame)
    }

    #[test]
    fn identity_constructor_rejects_unprogrammed_values() {
        assert!(matches!(
            Cc2340RadioPhy::new([0; 8]),
            Err(IdentityError::UnprogrammedExtendedAddress)
        ));
        assert!(matches!(
            Cc2340RadioPhy::new([0xFF; 8]),
            Err(IdentityError::UnprogrammedExtendedAddress)
        ));

        let phy = Cc2340RadioPhy::new(VALID_IEEE).unwrap();
        assert_eq!(phy.extended_address(), VALID_IEEE);
    }

    #[test]
    fn entropy_is_explicitly_fail_closed() {
        let mut phy = Cc2340RadioPhy::new(VALID_IEEE).unwrap();
        let mut output = [0xA5; 8];
        assert_eq!(
            phy.entropy_disposition(),
            EntropyDisposition::UnavailableFailClosed
        );
        assert_eq!(phy.fill_random(&mut output), Err(MacError::Unsupported));
        assert_eq!(output, [0xA5; 8]);
    }

    #[test]
    fn standby_capable_is_advertised_and_harmless_before_radio_init() {
        let mut phy = Cc2340RadioPhy::new(VALID_IEEE).unwrap();
        assert_eq!(
            phy.radio_sleep_disposition(),
            RadioSleepDisposition::StandbyCapable
        );
        assert_eq!(phy.prepare_active_wait(), Ok(()));
        // Before `initialize_radio` there is nothing to power down: no MMIO
        // is touched and the driver never reports itself asleep, so a later
        // init performs the full bring-up.
        assert_eq!(RadioPhy::sleep(&mut phy), Ok(()));
        assert!(!phy.is_asleep());
        assert!(!phy.holds_received_frame());
        // Standby without a sleeping radio is reported as out of sequence and
        // does not arm a re-init.
        assert!(!phy.note_standby());
        assert_eq!(RadioPhy::wake(&mut phy), Ok(()));
        assert_eq!(phy.stats().1.sleeps, 0);
        assert_eq!(phy.driver.read_rssi(), i8::MIN);
    }

    #[test]
    fn compile_only_availability_error_is_preserved_before_mmio() {
        let expected = if firmware::images().is_none() {
            RadioError::FirmwareUnavailable
        } else if config::ieee_802154_phy_writes().is_none() {
            RadioError::RadioConfigUnavailable
        } else {
            // A complete SDK would continue into real MMIO and is therefore a
            // hardware-only test.
            return;
        };

        let mut phy = Cc2340RadioPhy::new(VALID_IEEE).unwrap();
        assert_eq!(phy.initialize_radio(), Err(expected));
    }

    /// The MAC is built without touching the radio and keeps the 5 dBm
    /// product default.
    #[test]
    fn soft_mac_keeps_identity_and_default_tx_power() {
        let mac = Cc2340RadioPhy::new(VALID_IEEE)
            .unwrap()
            .into_soft_mac()
            .unwrap();
        assert_eq!(
            mac.get_pib(PibAttribute::MacExtendedAddress),
            PibValue::ExtendedAddress(VALID_IEEE)
        );
        assert_eq!(
            mac.get_pib(PibAttribute::PhyTransmitPower),
            PibValue::I8(TX_POWER_DEFAULT_DBM)
        );
        assert_eq!(mac.phy().driver.config().tx_power_dbm, 5);
        assert_eq!(mac.phy().driver.config().ieee_addr, VALID_IEEE);
    }

    #[test]
    fn channel_and_tx_power_are_range_checked() {
        let mut phy = Cc2340RadioPhy::new(VALID_IEEE).unwrap();
        assert_eq!(phy.set_channel(10), Err(PhyError::InvalidChannel));
        assert_eq!(phy.set_channel(27), Err(PhyError::InvalidChannel));
        assert_eq!(phy.set_channel(26), Ok(()));
        assert_eq!(phy.driver.config().channel, 26);
        assert_eq!(phy.set_tx_power(9), Err(PhyError::Unsupported));
        assert_eq!(phy.set_tx_power(-21), Err(PhyError::Unsupported));
        assert_eq!(phy.set_tx_power(-20), Ok(()));
        assert_eq!(phy.driver.config().tx_power_dbm, -20);
    }

    #[test]
    fn address_filter_maps_to_pbe_filter_and_scan_pan() {
        let mut phy = Cc2340RadioPhy::new(VALID_IEEE).unwrap();
        let filter = PhyAddressFilter {
            pan_id: PanId(0x1234),
            short_address: ShortAddress(0x5678),
            extended_address: VALID_IEEE,
        };
        phy.set_address_filter(Some(filter)).unwrap();
        assert!(phy.filtered);
        assert_eq!(phy.driver.config().pan_id, 0x1234);
        assert_eq!(phy.driver.config().short_addr, 0x5678);
        assert!(!phy.driver.config().promiscuous);

        phy.set_address_filter(None).unwrap();
        assert!(!phy.filtered);
        assert_eq!(phy.driver.config().pan_id, 0xFFFF);
        assert_eq!(phy.driver.config().short_addr, 0x5678);
        // ACK matching needs the PBE filter on, even while scanning.
        assert!(!phy.driver.config().promiscuous);
    }

    #[test]
    fn acks_and_sleepy_broadcasts_are_stale() {
        assert!(discard_before_mac(&[0x02, 0x00, 0x42], false));
        assert!(discard_before_mac(&[], false));

        let broadcast = data_frame(1, 0xFFFF, 0x0000);
        assert!(discard_before_mac(&broadcast[..7], true));
        assert!(!discard_before_mac(&broadcast[..7], false));

        let unicast = data_frame(1, 0x5678, 0x0000);
        assert!(!discard_before_mac(&unicast[..7], true));

        // Beacon Request: command frame to the short broadcast address.
        let beacon_request = [0x03, 0x08, 0x10, 0xFF, 0xFF, 0xFF, 0xFF];
        assert!(discard_before_mac(&beacon_request, true));
        // Beacons are never stale.
        assert!(!discard_before_mac(
            &[0x00, 0x80, 0x01, 0x34, 0x12, 0x00, 0x00],
            true
        ));
    }

    #[test]
    fn repeated_frame_from_same_source_and_dsn_is_delivered_once() {
        let mut phy = Cc2340RadioPhy::new(VALID_IEEE).unwrap();
        let frame = data_frame(7, 0x5678, 0x0000);
        assert!(accept(&mut phy, &frame).is_some());
        assert!(accept(&mut phy, &frame).is_none());
        assert_eq!(phy.stats().0.duplicates, 1);
        // Another DSN or another sender is new.
        assert!(accept(&mut phy, &data_frame(8, 0x5678, 0x0000)).is_some());
        assert!(accept(&mut phy, &data_frame(8, 0x5678, 0x1111)).is_some());
    }

    #[test]
    fn broadcasts_follow_the_delivery_knob_and_are_never_duplicates() {
        let mut phy = Cc2340RadioPhy::new(VALID_IEEE).unwrap();
        let mut frame = data_frame(3, 0xFFFF, 0x0000);
        frame[0] &= !FRAME_CONTROL_ACK_REQUEST;
        assert!(accept(&mut phy, &frame).is_none());
        phy.set_broadcast_delivery(true);
        assert!(accept(&mut phy, &frame).is_some());
        assert!(accept(&mut phy, &frame).is_some());
        // Scanning (no filter) always delivers.
        phy.set_broadcast_delivery(false);
        phy.set_address_filter(None).unwrap();
        assert!(accept(&mut phy, &frame).is_some());
    }

    #[test]
    fn accepted_frame_keeps_bytes_and_lqi() {
        let mut phy = Cc2340RadioPhy::new(VALID_IEEE).unwrap();
        let frame = data_frame(9, 0x5678, 0x0000);
        let accepted = accept(&mut phy, &frame).unwrap();
        assert_eq!(accepted.as_slice(), frame.as_slice());
        assert_eq!(accepted.lqi, 200);
        assert!(accept(&mut phy, &[0x02, 0x00, 0x09]).is_none());
    }

    #[test]
    fn data_request_is_recognised_for_both_source_modes() {
        let parent = MacAddress::Short(PanId(0x1234), ShortAddress(0x0000));
        let short = frames::build_data_request_short(1, &parent, ShortAddress(0x5678));
        let extended = frames::build_data_request(2, &parent, &VALID_IEEE);
        assert!(is_data_request(&short));
        assert!(is_data_request(&extended));
        assert!(!is_data_request(&data_frame(3, 0x0000, 0x5678)));
        assert!(!is_data_request(&[0x03, 0x08]));
    }

    #[test]
    fn error_mapping_keeps_csma_and_fatal_cases_apart() {
        assert_eq!(map_tx_error(RadioError::ChannelBusy), PhyError::ChannelBusy);
        assert_eq!(
            map_tx_error(RadioError::OperationTimeout),
            PhyError::ChannelBusy
        );
        assert_eq!(
            map_tx_error(RadioError::InvalidFrame),
            PhyError::FrameTooLong
        );
        assert_eq!(
            map_tx_error(RadioError::FirmwareUnavailable),
            PhyError::Hardware
        );
        assert_eq!(receive_error(RadioError::CrcError), Ok(()));
        assert_eq!(receive_error(RadioError::FifoOverflow), Ok(()));
        assert_eq!(
            receive_error(RadioError::RadioConfigUnavailable),
            Err(PhyError::Hardware)
        );
    }

    #[test]
    fn energy_scale_is_monotonic() {
        assert_eq!(rssi_to_ed(i8::MIN), 0);
        assert!(rssi_to_ed(-90) < rssi_to_ed(-40));
        assert_eq!(rssi_to_ed(i8::MAX), 254);
    }
}
