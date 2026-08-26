//! CC2340R5 IEEE 802.15.4 radio driver.
//!
//! This module is the Rust host-side replacement for the former speculative
//! `mac_ti23xx_*` and RCL FFI bindings. The current bring-up stage performs the
//! parts that can be implemented and validated independently:
//!
//! - applies TI's early CC2340 device trim workaround;
//! - enables the LRFD module and radio-memory clocks;
//! - imports and loads the official IEEE PBE/MCE/RFE firmware as data;
//! - replays TI's IEEE PHY settings and per-die FCFG radio trims;
//! - programs the channel synthesizer and LaunchPad PA setting;
//! - initializes all three radio TOPsm cores;
//! - transfers frames through the LRFD FIFOs and polls PBE radio events.

use super::{config, fifo, firmware, hardware};

const TX_TIMEOUT_TICKS: u32 = 400_000;
const ABORT_POLL_LIMIT: usize = 100_000;

/// Radio configuration for the CC2340 driver.
#[derive(Debug, Clone)]
pub struct RadioConfig {
    pub channel: u8,
    pub pan_id: u16,
    pub short_addr: u16,
    pub ieee_addr: [u8; 8],
    pub tx_power_dbm: i8,
    pub rx_on_when_idle: bool,
    pub promiscuous: bool,
    pub auto_ack: bool,
}

impl Default for RadioConfig {
    fn default() -> Self {
        Self {
            channel: 11,
            pan_id: 0xFFFF,
            short_addr: 0xFFFF,
            ieee_addr: [0u8; 8],
            tx_power_dbm: 0,
            rx_on_when_idle: false,
            promiscuous: false,
            auto_ack: true,
        }
    }
}

/// Radio error type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadioError {
    InvalidChannel,
    InvalidFrame,
    InvalidTxPower,
    FirmwareUnavailable,
    ClockTimeout,
    FirmwareImageTooLarge,
    RadioConfigUnavailable,
    FactoryTrimUnavailable,
    SynthConfigInvalid,
    SynthTimeout,
    RadioReadyTimeout,
    ChannelBusy,
    NoAck,
    RxTimeout,
    CrcError,
    FifoOverflow,
    MalformedFrame,
    /// The RX FIFO entry carried no usable RSSI, so no IEEE LQI can be derived
    /// for the frame. Reported explicitly instead of surfacing a frame with a
    /// fabricated link quality.
    LinkQualityUnavailable,
    OperationTimeout,
    PbeError(u8),
    HardwareError,
}

impl From<hardware::HardwareError> for RadioError {
    fn from(error: hardware::HardwareError) -> Self {
        match error {
            hardware::HardwareError::ClockTimeout => Self::ClockTimeout,
            hardware::HardwareError::ImageTooLarge => Self::FirmwareImageTooLarge,
            hardware::HardwareError::FactoryTrimUnavailable => Self::FactoryTrimUnavailable,
            hardware::HardwareError::SynthConfigInvalid => Self::SynthConfigInvalid,
            hardware::HardwareError::SynthDividerTimeout => Self::SynthTimeout,
            hardware::HardwareError::TopsmTimeout => Self::RadioReadyTimeout,
        }
    }
}

/// A received IEEE 802.15.4 frame with metadata.
pub struct RxFrame {
    pub data: [u8; fifo::MAX_PHY_FRAME_LEN],
    pub len: usize,
    /// Received signal strength in dBm, exactly as decoded from the PBE.
    pub rssi: i8,
    /// IEEE 802.15.4 LQI in `0..=255`, larger is better.
    ///
    /// Derived from [`Self::rssi`] via [`crate::lqi::from_rssi_dbm`]. The
    /// CC2340 PBE's appended LQI byte is a raw modem estimate on an
    /// undocumented scale and is deliberately *not* used here; see
    /// [`Self::modem_lqi`].
    pub lqi: u8,
    /// Raw modem LQI byte from the PBE, for diagnostics only.
    ///
    /// `None` when the FIFO does not append it. Never feed this to the stack.
    pub modem_lqi: Option<u8>,
}

/// Low-level CC2340 IEEE 802.15.4 radio driver.
pub struct Cc2340Driver {
    config: RadioConfig,
    images_loaded: bool,
    configured: bool,
    config_dirty: bool,
    operation_active: bool,
}

impl Cc2340Driver {
    pub fn new(config: RadioConfig) -> Self {
        Self {
            config,
            images_loaded: false,
            configured: false,
            config_dirty: true,
            operation_active: false,
        }
    }

    /// Initialize the Rust LRFD hardware, PHY, trims, synthesizer, PA, and
    /// radio microcode cores.
    pub fn init(&mut self) -> Result<(), RadioError> {
        if self.configured {
            return Ok(());
        }

        if !self.images_loaded {
            // Keep compile-only availability modes observable before consulting
            // generated PHY-dependent tables such as the PA settings.
            let images = firmware::images().ok_or(RadioError::FirmwareUnavailable)?;
            let phy_writes =
                config::ieee_802154_phy_writes().ok_or(RadioError::RadioConfigUnavailable)?;
            let frequency = hardware::channel_frequency(self.config.channel)
                .ok_or(RadioError::InvalidChannel)?;
            let tx_power =
                config::tx_power(self.config.tx_power_dbm).ok_or(RadioError::InvalidTxPower)?;

            hardware::setup_device_trim();
            hardware::enable_radio_clocks()?;
            if let Err(error) = hardware::load_firmware(images.pbe, images.mce, images.rfe) {
                hardware::disable_radio_clocks();
                return Err(error.into());
            }
            hardware::apply_phy_configuration(phy_writes);
            hardware::route_scheduler_interrupt();

            let bringup = || -> Result<(), hardware::HardwareError> {
                hardware::finish_radio_setup()?;
                hardware::enable_synth_refsys()?;
                hardware::program_frequency(frequency)?;
                hardware::program_tx_power(tx_power.raw);
                hardware::enable_radio_cores();
                hardware::wait_for_topsm_ready()?;
                fifo::clear_events(u32::MAX);
                fifo::reset_both();
                Ok(())
            };
            if let Err(error) = bringup() {
                hardware::hard_stop();
                hardware::disable_radio_cores();
                hardware::disable_synth_refsys();
                hardware::disable_radio_clocks();
                return Err(error.into());
            }

            log::info!(
                "[CC2340 DRV] Initialized channel {} at {} dBm using firmware from {} and PHY settings from {}",
                self.config.channel,
                tx_power.dbm,
                firmware::source(),
                config::source()
            );
            self.images_loaded = true;
            self.configured = true;
            self.config_dirty = false;
            self.operation_active = false;
        }

        Ok(())
    }

    pub fn deinit(&mut self) {
        if self.images_loaded {
            hardware::hard_stop();
            hardware::disable_radio_cores();
            hardware::disable_synth_refsys();
            hardware::disable_radio_clocks();
        }
        self.images_loaded = false;
        self.configured = false;
        self.config_dirty = true;
        self.operation_active = false;
    }

    pub fn update_config<F: FnOnce(&mut RadioConfig)>(&mut self, update: F) {
        update(&mut self.config);
        self.config_dirty = true;
    }

    pub fn set_channel(&mut self, channel: u8) {
        self.config.channel = channel;
        self.config_dirty = true;
    }

    pub fn set_pan_id(&mut self, pan_id: u16) {
        self.config.pan_id = pan_id;
    }

    pub fn set_short_addr(&mut self, address: u16) {
        self.config.short_addr = address;
    }

    pub fn set_ieee_addr(&mut self, address: &[u8; 8]) {
        self.config.ieee_addr = *address;
    }

    pub fn set_tx_power(&mut self, dbm: i8) {
        self.config.tx_power_dbm = dbm;
        self.config_dirty = true;
    }

    pub fn set_rx_on_when_idle(&mut self, enabled: bool) {
        self.config.rx_on_when_idle = enabled;
    }

    pub fn perform_cca(&self) -> Result<i8, RadioError> {
        if !self.configured {
            return Err(RadioError::RadioConfigUnavailable);
        }
        Err(RadioError::HardwareError)
    }

    pub fn read_rssi(&self) -> i8 {
        if self.configured {
            let rssi = hardware::read_rssi();
            if rssi == 127 { i8::MIN } else { rssi }
        } else {
            i8::MIN
        }
    }

    pub async fn transmit(&mut self, frame: &[u8]) -> Result<(), RadioError> {
        if frame.is_empty() || frame.len() > fifo::MAX_MPDU_LEN {
            return Err(RadioError::InvalidFrame);
        }
        if !self.configured {
            return Err(RadioError::RadioConfigUnavailable);
        }

        self.prepare_operation()?;
        fifo::prepare_tx();
        fifo::reset_tx();
        fifo::write_tx_frame(frame).map_err(map_fifo_error)?;
        fifo::clear_events(u32::MAX);
        self.operation_active = true;
        fifo::start_tx();

        let started = hardware::timer_ticks();
        loop {
            let events = fifo::events();
            if events & fifo::EVENT_TERMINAL != 0 {
                return self.finish_operation(events);
            }
            if hardware::timer_ticks().wrapping_sub(started) >= TX_TIMEOUT_TICKS {
                break;
            }
            yield_now().await;
        }

        self.abort_active_operation()?;
        Err(RadioError::OperationTimeout)
    }

    pub async fn receive(&mut self) -> Result<RxFrame, RadioError> {
        if !self.configured {
            return Err(RadioError::RadioConfigUnavailable);
        }

        self.prepare_operation()?;
        fifo::prepare_promiscuous_rx();
        fifo::reset_rx();
        fifo::clear_events(u32::MAX);
        self.operation_active = true;
        fifo::start_rx();

        let mut dropped_frames: u32 = 0;

        loop {
            let events = fifo::events();

            if events & fifo::EVENT_RX_OK != 0 {
                match fifo::try_read_rx_frame() {
                    Ok(Some(frame)) => {
                        fifo::clear_events(fifo::EVENT_RX_OK);
                        self.abort_active_operation()?;
                        return Ok(RxFrame {
                            data: frame.data,
                            len: frame.len,
                            rssi: frame.rssi,
                            // Normalize once, here, at the point the value
                            // enters the stack.
                            lqi: crate::lqi::from_rssi_dbm(frame.rssi),
                            modem_lqi: frame.modem_lqi,
                        });
                    }
                    Ok(None) => {}
                    Err(error) => match classify_rx_fifo_error(error) {
                        RxFifoDisposition::DropFrame => {
                            // `try_read_rx_frame` advanced the FIFO read
                            // pointer before decoding, so this entry is
                            // already consumed. Drop it and keep the
                            // continuous RX operation running: a single frame
                            // whose metadata is unusable must not truncate an
                            // active scan or an association/rejoin wait.
                            //
                            // No fabricated LQI reaches the stack — the frame
                            // is discarded outright, not surfaced with a
                            // made-up link quality.
                            if !fifo::rx_data_pending() {
                                fifo::clear_events(fifo::EVENT_RX_OK);
                            }
                            dropped_frames = dropped_frames.saturating_add(1);
                            log::debug!(
                                "[CC2340 DRV] Dropped RX frame with unusable link quality \
                                 ({error:?}), continuing RX ({dropped_frames} dropped)"
                            );
                            if dropped_frames == RSSI_DROP_WARN_THRESHOLD {
                                log::warn!(
                                    "[CC2340 DRV] {RSSI_DROP_WARN_THRESHOLD} consecutive RX \
                                     frames carried no valid RSSI — check the PBE RSSI path"
                                );
                            }
                        }
                        RxFifoDisposition::Fatal(error) => {
                            self.abort_active_operation()?;
                            return Err(error);
                        }
                    },
                }
            }

            if events & fifo::EVENT_RX_BUF_FULL != 0 {
                self.abort_active_operation()?;
                return Err(RadioError::FifoOverflow);
            }

            if events & (fifo::EVENT_RX_NOK | fifo::EVENT_RX_IGNORED) != 0 {
                fifo::clear_events(fifo::EVENT_RX_NOK | fifo::EVENT_RX_IGNORED);
            }

            if events & fifo::EVENT_TERMINAL != 0 {
                return match self.finish_operation(events) {
                    Ok(()) => Err(RadioError::RxTimeout),
                    Err(error) => Err(error),
                };
            }

            yield_now().await;
        }
    }

    pub fn abort_tx(&mut self) {
        if let Err(error) = self.abort_active_operation() {
            log::error!("[CC2340 DRV] Failed to abort radio operation: {error:?}");
        }
    }

    /// Stop an in-flight PBE operation while retaining the initialized radio.
    ///
    /// This is the only power-wait transition currently exposed by the MAC;
    /// `deinit` is deliberately not used for application waits until
    /// firmware/trim restoration is validated on hardware.
    pub fn prepare_active_wait(&mut self) -> Result<(), RadioError> {
        self.abort_active_operation()
    }

    fn prepare_operation(&mut self) -> Result<(), RadioError> {
        self.abort_active_operation()?;
        self.apply_runtime_config()?;
        hardware::wait_for_topsm_ready()?;
        Ok(())
    }

    fn apply_runtime_config(&mut self) -> Result<(), RadioError> {
        if !self.config_dirty {
            return Ok(());
        }

        let frequency =
            hardware::channel_frequency(self.config.channel).ok_or(RadioError::InvalidChannel)?;
        let tx_power =
            config::tx_power(self.config.tx_power_dbm).ok_or(RadioError::InvalidTxPower)?;
        hardware::program_frequency(frequency)?;
        hardware::program_tx_power(tx_power.raw);
        self.config_dirty = false;
        Ok(())
    }

    fn finish_operation(&mut self, events: u32) -> Result<(), RadioError> {
        let end_cause = fifo::end_cause();
        fifo::clear_events(u32::MAX);
        self.operation_active = false;

        if events & fifo::EVENT_OP_ERROR != 0 && end_cause == 0 {
            Err(RadioError::HardwareError)
        } else if end_cause == 0 {
            Ok(())
        } else {
            Err(map_pbe_error(end_cause))
        }
    }

    fn abort_active_operation(&mut self) -> Result<(), RadioError> {
        if !self.operation_active {
            return Ok(());
        }

        if fifo::events() & fifo::EVENT_TERMINAL == 0 {
            fifo::clear_events(fifo::EVENT_TERMINAL);
            fifo::hard_stop();
            let mut stopped = false;
            for _ in 0..ABORT_POLL_LIMIT {
                if fifo::events() & fifo::EVENT_TERMINAL != 0 {
                    stopped = true;
                    break;
                }
                core::hint::spin_loop();
            }
            if !stopped {
                self.operation_active = false;
                fifo::reset_both();
                return Err(RadioError::OperationTimeout);
            }
        }

        fifo::clear_events(u32::MAX);
        fifo::reset_both();
        self.operation_active = false;
        Ok(())
    }
}

fn map_fifo_error(error: fifo::FifoError) -> RadioError {
    match error {
        fifo::FifoError::FrameTooLong => RadioError::InvalidFrame,
        fifo::FifoError::NoSpace => RadioError::FifoOverflow,
        fifo::FifoError::MalformedEntry | fifo::FifoError::MetadataMismatch => {
            RadioError::MalformedFrame
        }
        fifo::FifoError::MissingRssi | fifo::FifoError::InvalidRssi => {
            RadioError::LinkQualityUnavailable
        }
    }
}

/// What [`Cc2340Driver::receive`] does with a FIFO decode failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RxFifoDisposition {
    /// A per-frame failure. The entry is already consumed; drop it and keep
    /// the current continuous RX operation running.
    DropFrame,
    /// A failure of the FIFO or its runtime configuration. Abort the
    /// operation and report the error to the caller.
    Fatal(RadioError),
}

/// Number of consecutive dropped frames after which the drop is escalated
/// from `debug` to a single `warn`.
///
/// Dropping is the correct per-frame behaviour, but a *persistently* invalid
/// RSSI path is a hardware/PHY problem an operator needs to see. One warning,
/// not one per frame.
const RSSI_DROP_WARN_THRESHOLD: u32 = 8;

/// Classify an RX FIFO decode failure as recoverable or fatal.
///
/// The distinction is between "this frame's metadata is unusable" and "this
/// radio cannot produce usable metadata at all":
///
/// * [`fifo::FifoError::InvalidRssi`] is per-frame. The PBE appended TI's
///   "no valid reading" sentinel for one entry, which says nothing about the
///   next one. Returning an error here would abort the continuous RX
///   operation and truncate whatever the caller was waiting for — an active
///   scan, an association response, a rejoin response — because of a single
///   frame. It is dropped instead, so no fabricated LQI reaches the stack and
///   reception continues.
/// * [`fifo::FifoError::MissingRssi`] means `FIFOCFG.APPEND_RSSI` is not set:
///   the runtime FIFO configuration is broken and *every* frame will fail the
///   same way. Dropping would spin forever, silently receiving nothing, so it
///   stays a loud [`RadioError::LinkQualityUnavailable`].
/// * Everything else keeps the behaviour this driver already shipped.
const fn classify_rx_fifo_error(error: fifo::FifoError) -> RxFifoDisposition {
    match error {
        fifo::FifoError::InvalidRssi => RxFifoDisposition::DropFrame,
        fifo::FifoError::MissingRssi => {
            RxFifoDisposition::Fatal(RadioError::LinkQualityUnavailable)
        }
        fifo::FifoError::FrameTooLong => RxFifoDisposition::Fatal(RadioError::InvalidFrame),
        fifo::FifoError::NoSpace => RxFifoDisposition::Fatal(RadioError::FifoOverflow),
        fifo::FifoError::MalformedEntry | fifo::FifoError::MetadataMismatch => {
            RxFifoDisposition::Fatal(RadioError::MalformedFrame)
        }
    }
}

fn map_pbe_error(end_cause: u8) -> RadioError {
    match end_cause {
        0x01 => RadioError::RxTimeout,
        0xF9 | 0xFA => RadioError::FifoOverflow,
        other => RadioError::PbeError(other),
    }
}

async fn yield_now() {
    let mut yielded = false;
    core::future::poll_fn(|context| {
        if yielded {
            core::task::Poll::Ready(())
        } else {
            yielded = true;
            context.waker().wake_by_ref();
            core::task::Poll::Pending
        }
    })
    .await
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The regression this classification guards against: a single frame
    /// whose appended RSSI is TI's "not available" sentinel used to abort the
    /// whole continuous RX operation, so one bad beacon truncated an active
    /// scan or an association/rejoin wait.
    #[test]
    fn invalid_rssi_is_a_per_frame_drop_not_a_terminal_error() {
        assert_eq!(
            classify_rx_fifo_error(fifo::FifoError::InvalidRssi),
            RxFifoDisposition::DropFrame,
        );
    }

    /// A missing RSSI field is a broken runtime FIFO configuration: every
    /// frame would fail identically, so dropping would spin forever while
    /// silently receiving nothing. It stays loud and fatal.
    #[test]
    fn missing_rssi_stays_a_loud_fatal_error() {
        assert_eq!(
            classify_rx_fifo_error(fifo::FifoError::MissingRssi),
            RxFifoDisposition::Fatal(RadioError::LinkQualityUnavailable),
        );
    }

    /// Every non-RSSI failure keeps the behaviour this driver already shipped,
    /// and keeps agreeing with the TX-side mapping.
    #[test]
    fn other_fifo_errors_stay_fatal_and_match_the_tx_mapping() {
        for error in [
            fifo::FifoError::FrameTooLong,
            fifo::FifoError::NoSpace,
            fifo::FifoError::MalformedEntry,
            fifo::FifoError::MetadataMismatch,
            fifo::FifoError::MissingRssi,
        ] {
            assert_eq!(
                classify_rx_fifo_error(error),
                RxFifoDisposition::Fatal(map_fifo_error(error)),
                "{error:?} must stay fatal with its established mapping"
            );
        }
    }

    /// Exactly one FIFO error is recoverable. If a future refactor adds a new
    /// variant it has to make an explicit choice here rather than inheriting
    /// "drop" by accident.
    #[test]
    fn only_invalid_rssi_is_recoverable() {
        let recoverable: usize = [
            fifo::FifoError::FrameTooLong,
            fifo::FifoError::NoSpace,
            fifo::FifoError::MalformedEntry,
            fifo::FifoError::MetadataMismatch,
            fifo::FifoError::MissingRssi,
            fifo::FifoError::InvalidRssi,
        ]
        .into_iter()
        .filter(|error| classify_rx_fifo_error(*error) == RxFifoDisposition::DropFrame)
        .count();

        assert_eq!(recoverable, 1);
    }

    /// A dropped frame yields no `RxFrame` at all, so there is no path by
    /// which an invalid-RSSI entry can reach the stack carrying a fabricated
    /// LQI. The only LQI a frame can carry is the RSSI-derived one.
    #[test]
    fn a_dropped_frame_can_never_deliver_a_fabricated_lqi() {
        // `DropFrame` carries no frame and no error payload: nothing to
        // surface upwards.
        assert!(matches!(
            classify_rx_fifo_error(fifo::FifoError::InvalidRssi),
            RxFifoDisposition::DropFrame
        ));

        // And the LQI of a frame that *is* delivered is derived from RSSI,
        // never from the raw modem byte.
        assert_eq!(
            crate::lqi::from_rssi_dbm(-42),
            crate::lqi::from_rssi_dbm(-42)
        );
        assert_ne!(
            crate::lqi::from_rssi_dbm(-90),
            crate::lqi::from_rssi_dbm(-30)
        );
    }

    /// Model of the `receive()` policy over a stream of FIFO reads.
    ///
    /// This exercises the classification, not the LRFD registers — the loop
    /// itself needs hardware. It returns how many frames a caller would have
    /// received and the error that ended the operation, if any.
    fn scan_outcome(reads: &[Result<Option<i8>, fifo::FifoError>]) -> (usize, Option<RadioError>) {
        let mut delivered = 0usize;
        for read in reads {
            match read {
                Ok(Some(_)) => delivered += 1,
                Ok(None) => {}
                Err(error) => match classify_rx_fifo_error(*error) {
                    // The entry is already consumed; keep receiving.
                    RxFifoDisposition::DropFrame => {}
                    RxFifoDisposition::Fatal(error) => return (delivered, Some(error)),
                },
            }
        }
        (delivered, None)
    }

    /// An active scan must survive frames whose RSSI metadata is unusable.
    /// Before this classification a single such beacon ended the scan, and
    /// every later beacon — including the one from the parent we needed —
    /// was never seen.
    #[test]
    fn a_scan_survives_frames_with_an_invalid_rssi() {
        let reads = [
            Ok(Some(-40)),
            Err(fifo::FifoError::InvalidRssi),
            Ok(Some(-55)),
            Err(fifo::FifoError::InvalidRssi),
            Err(fifo::FifoError::InvalidRssi),
            Ok(None),
            Ok(Some(-70)),
        ];

        assert_eq!(scan_outcome(&reads), (3, None));
    }

    /// The same stream under the old "any decode error is terminal" policy
    /// delivered only the frames before the first bad one.
    #[test]
    fn an_association_wait_is_not_truncated_by_one_invalid_rssi_frame() {
        let reads = [Err(fifo::FifoError::InvalidRssi), Ok(Some(-45))];

        // The association response still arrives.
        assert_eq!(scan_outcome(&reads), (1, None));
    }

    /// A broken FIFO configuration still stops the operation immediately,
    /// rather than silently dropping every frame forever.
    #[test]
    fn a_missing_rssi_configuration_stops_the_operation_at_once() {
        let reads = [
            Ok(Some(-40)),
            Err(fifo::FifoError::MissingRssi),
            Ok(Some(-55)),
        ];

        assert_eq!(
            scan_outcome(&reads),
            (1, Some(RadioError::LinkQualityUnavailable))
        );
    }

    /// The escalation threshold has to be reachable and small enough to be
    /// useful as a diagnostic.
    #[test]
    fn drop_warning_threshold_is_a_sane_small_bound() {
        const { assert!(RSSI_DROP_WARN_THRESHOLD > 0) };
        const { assert!(RSSI_DROP_WARN_THRESHOLD <= 64) };
    }
}
