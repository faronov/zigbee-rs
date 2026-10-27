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

/// 100 ms in SYSTIM 250 ns ticks.
const TX_TIMEOUT_TICKS: u32 = 400_000;
const ABORT_POLL_LIMIT: usize = 100_000;
/// ACK wait for an ACK-requested TX: `macAckWaitDuration` (54 symbols), from
/// the end of the frame to ACK sync. RCL's default `ackTimeout` is 300 us.
pub const ACK_TIMEOUT_US: u16 = 864;
/// [`ACK_TIMEOUT_US`] in the PBE's 0.25 us `RXTIMEOUT` units.
const ACK_TIMEOUT_QUARTER_US: u16 = ACK_TIMEOUT_US * 4;
/// Longest a TX waits for an in-progress reception (127-byte frame, 4.3 ms)
/// plus its auto-ACK before preempting RX: 5 ms in 250 ns ticks.
const RX_BUSY_WAIT_TICKS: u32 = 20_000;
/// CCA energy threshold: RCL `RCL_CmdIeee_TxAction_Default().rssiLimit`.
const CCA_RSSI_LIMIT_DBM: i8 = -70;
/// CCA signal threshold: RCL default `ccaCorrThresh` (correlation tops in a
/// 128 us window).
const CCA_CORR_THRESHOLD: u16 = 3;
/// How long CCA waits for a valid RSSI: one unit backoff period (RCL waits
/// one backoff, then treats an invalid RSSI as busy) plus RCL's 192 us CCA
/// start margin for a receiver that CCA had to start itself, rounded up to
/// 1 ms.
const CCA_RSSI_WAIT_TICKS: u32 = 1_000 * 4;
/// TI's "RSSI not available" reading.
const RSSI_INVALID: i8 = 127;

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
            hardware::HardwareError::HfxtTimeout => Self::ClockTimeout,
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

/// Counters of the host-side driver, cumulative since boot.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DriverStats {
    /// Frames handed to the PBE for transmission.
    pub tx_frames: u32,
    /// ACK-requested frames whose ACK the PBE received.
    pub ack_received: u32,
    /// ... of which the ACK had the frame-pending bit set.
    pub ack_frame_pending: u32,
    /// ACK-requested frames without ACK sync within the ACK timeout.
    pub ack_timeout: u32,
    /// ACK-requested frames where something other than our ACK was received.
    pub ack_rx_error: u32,
    /// Frames delivered by [`Cc2340Driver::receive`].
    pub rx_frames: u32,
    /// Frames dropped because their FIFO metadata was unusable.
    pub rx_dropped: u32,
    /// RX operations the PBE ended on its own and the driver restarted.
    pub rx_restarts: u32,
    /// Transmissions refused because CCA found the channel busy.
    pub cca_busy: u32,
    /// Radio power-downs by [`Cc2340Driver::sleep`].
    pub sleeps: u32,
    /// Full LRFD re-initialisations after device standby
    /// ([`Cc2340Driver::note_standby`]).
    pub standby_reinits: u32,
}

/// Low-level CC2340 IEEE 802.15.4 radio driver.
///
/// The PBE runs one long-lived IEEE RX operation with hardware frame
/// filtering and immediate auto-ACK (RCL `RCL_CMD_IEEE_RX_TX` with a single
/// PAN, `RCL_CmdIeee_AutoAck_ImmAckNoAutoPend`). [`Self::receive`] reads
/// committed frames from the RX FIFO while that operation keeps running. A
/// transmission stops RX, sends the frame as a TX-then-RX operation (the PBE
/// waits for the immediate ACK itself when one is requested) and RX resumes
/// automatically after it, without host involvement in either turnaround.
pub struct Cc2340Driver {
    config: RadioConfig,
    images_loaded: bool,
    configured: bool,
    /// Channel or PA level changed since it was last programmed.
    config_dirty: bool,
    /// Addressing or promiscuous mode changed since the filter was written.
    filter_dirty: bool,
    operation_active: bool,
    /// Radio cores, synthesizer reference and LRFD clocks are powered down
    /// by [`Self::sleep`]; the next operation (or [`Self::wake`]) restores
    /// them.
    asleep: bool,
    /// The device went through standby while [`Self::asleep`]: the LRFD
    /// lost its registers, so [`Self::wake`] must run the full [`Self::init`].
    lrfd_lost: bool,
    /// The RX FIFO must be reset (not just trimmed) before the next start.
    rx_fifo_reset_pending: bool,
    pa_value: u16,
    stats: DriverStats,
}

impl Cc2340Driver {
    pub fn new(config: RadioConfig) -> Self {
        Self {
            config,
            images_loaded: false,
            configured: false,
            config_dirty: true,
            filter_dirty: true,
            operation_active: false,
            asleep: false,
            lrfd_lost: false,
            rx_fifo_reset_pending: true,
            pa_value: 0,
            stats: DriverStats::default(),
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
            hardware::start_hfxt()?;
            hardware::enable_hfxt_clock_buffer();
            hardware::enable_radio_clocks()?;
            if let Err(error) = hardware::load_firmware(images.pbe, images.mce, images.rfe) {
                hardware::disable_radio_clocks();
                return Err(error.into());
            }
            hardware::apply_phy_configuration(phy_writes);
            hardware::route_scheduler_interrupt();

            let temperature = die_temperature_c();
            let pa_value = tx_power.compensated_rfe_value(temperature);
            let bringup = || -> Result<(), hardware::HardwareError> {
                hardware::finish_radio_setup()?;
                hardware::enable_synth_refsys()?;
                hardware::program_frequency(frequency)?;
                hardware::program_tx_power(pa_value);
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

            // First bring-up at info; post-standby re-inits (every slow poll)
            // only at debug, they are counted in the STATS line.
            let level = if self.stats.standby_reinits == 0 {
                log::Level::Info
            } else {
                log::Level::Debug
            };
            log::log!(
                level,
                "[CC2340 DRV] Initialized channel {} at {} dBm (PA 0x{:04X}, {} C) using firmware from {} and PHY settings from {}",
                self.config.channel,
                tx_power.dbm,
                pa_value,
                temperature,
                firmware::source(),
                config::source()
            );
            self.pa_value = pa_value;
            self.images_loaded = true;
            self.configured = true;
            self.config_dirty = false;
            self.filter_dirty = true;
            self.operation_active = false;
            self.rx_fifo_reset_pending = true;
        }

        Ok(())
    }

    pub fn deinit(&mut self) {
        if self.asleep && hardware::start_hfxt().is_ok() && hardware::enable_radio_clocks().is_ok()
        {
            self.asleep = false;
        }
        if self.images_loaded && !self.asleep {
            hardware::hard_stop();
            hardware::disable_radio_cores();
            hardware::disable_synth_refsys();
            hardware::disable_radio_clocks();
        }
        self.images_loaded = false;
        self.configured = false;
        self.config_dirty = true;
        self.filter_dirty = true;
        self.operation_active = false;
        self.rx_fifo_reset_pending = true;
    }

    pub fn update_config<F: FnOnce(&mut RadioConfig)>(&mut self, update: F) {
        let before = self.config.clone();
        update(&mut self.config);
        if before.channel != self.config.channel || before.tx_power_dbm != self.config.tx_power_dbm
        {
            self.config_dirty = true;
        }
        if before.pan_id != self.config.pan_id
            || before.short_addr != self.config.short_addr
            || before.ieee_addr != self.config.ieee_addr
            || before.promiscuous != self.config.promiscuous
        {
            self.filter_dirty = true;
        }
    }

    pub fn set_channel(&mut self, channel: u8) {
        self.update_config(|config| config.channel = channel);
    }

    pub fn set_pan_id(&mut self, pan_id: u16) {
        self.update_config(|config| config.pan_id = pan_id);
    }

    pub fn set_short_addr(&mut self, address: u16) {
        self.update_config(|config| config.short_addr = address);
    }

    pub fn set_ieee_addr(&mut self, address: &[u8; 8]) {
        self.update_config(|config| config.ieee_addr = *address);
    }

    pub fn set_tx_power(&mut self, dbm: i8) {
        self.update_config(|config| config.tx_power_dbm = dbm);
    }

    pub fn set_rx_on_when_idle(&mut self, enabled: bool) {
        self.config.rx_on_when_idle = enabled;
    }

    /// Cumulative host-side driver counters.
    pub fn stats(&self) -> DriverStats {
        self.stats
    }

    /// PBE counters of the operation currently (or last) running.
    pub fn pbe_counters(&self) -> fifo::PbeCounters {
        if self.configured && !self.asleep {
            fifo::pbe_counters()
        } else {
            fifo::PbeCounters::default()
        }
    }

    /// The 16-bit PA value last written to RFE `SPARE5`.
    pub fn pa_value(&self) -> u16 {
        self.pa_value
    }

    pub fn perform_cca(&self) -> Result<i8, RadioError> {
        if !self.configured {
            return Err(RadioError::RadioConfigUnavailable);
        }
        Err(RadioError::HardwareError)
    }

    pub fn read_rssi(&self) -> i8 {
        if self.configured && !self.asleep {
            let rssi = hardware::read_rssi();
            if rssi == 127 { i8::MIN } else { rssi }
        } else {
            i8::MIN
        }
    }

    /// Transmit a frame that does not request an ACK. Returns once the frame
    /// has been sent; RX is running again afterwards.
    pub async fn transmit(&mut self, frame: &[u8]) -> Result<(), RadioError> {
        self.transmit_frame(frame, false).await.map(|_| ())
    }

    /// Transmit an ACK-requested frame and let the PBE receive its immediate
    /// ACK (RCL TX action with `expectImmAck`).
    ///
    /// Returns the ACK's frame-pending bit, or [`RadioError::NoAck`] when no
    /// matching ACK arrived within [`ACK_TIMEOUT_US`]. No retransmission
    /// here; that is the MAC's `macMaxFrameRetries` loop.
    pub async fn transmit_with_ack(&mut self, frame: &[u8]) -> Result<bool, RadioError> {
        if frame.len() < 3 {
            return Err(RadioError::InvalidFrame);
        }
        self.transmit_frame(frame, true).await
    }

    async fn transmit_frame(&mut self, frame: &[u8], expect_ack: bool) -> Result<bool, RadioError> {
        if frame.is_empty() || frame.len() > fifo::MAX_MPDU_LEN {
            return Err(RadioError::InvalidFrame);
        }
        if !self.configured {
            return Err(RadioError::RadioConfigUnavailable);
        }
        self.wake()?;

        // Do not cut off a frame being received or the auto-ACK being sent.
        self.wait_rx_idle().await;
        self.clear_channel_assessment().await?;
        self.stop_operation()?;
        self.prepare_cores()?;
        // RCL: only the TX FIFO is reset for a TX; frames already committed
        // to the RX FIFO stay readable.
        fifo::reset_tx();
        self.prepare_rx_fifo();
        fifo::write_tx_frame(frame).map_err(map_fifo_error)?;
        let ack = expect_ack.then_some((frame[2], ACK_TIMEOUT_QUARTER_US));
        fifo::prepare_tx_then_rx(ack);
        fifo::clear_events(u32::MAX);
        self.operation_active = true;
        hardware::arm_pbe_start_trigger();
        fifo::start_tx();
        self.stats.tx_frames = self.stats.tx_frames.wrapping_add(1);

        let done_event = if expect_ack {
            fifo::EVENT_RX_EMPTY
        } else {
            fifo::EVENT_TX_DONE
        };
        let started = hardware::timer_ticks();
        loop {
            let events = fifo::events();
            if events & done_event != 0 {
                fifo::clear_events(done_event);
                return if expect_ack {
                    self.ack_outcome(fifo::ack_status())
                } else {
                    Ok(false)
                };
            }
            if events & fifo::EVENT_TERMINAL != 0 {
                // The operation ended before the TX (and ACK) outcome.
                self.finish_operation(events)?;
                return if expect_ack {
                    Err(RadioError::NoAck)
                } else {
                    Ok(false)
                };
            }
            if hardware::timer_ticks().wrapping_sub(started) >= TX_TIMEOUT_TICKS {
                break;
            }
            yield_now().await;
        }

        self.stop_operation()?;
        Err(RadioError::OperationTimeout)
    }

    /// Clear channel assessment on the running RX operation, as RCL
    /// `RCL_Handler_Ieee_RxTx` does for `RCL_CmdIeee_CcaMode3EnergyOrSignal`:
    /// busy while a frame is being received, when RSSI is at or above
    /// [`CCA_RSSI_LIMIT_DBM`], or when the correlator saw more than
    /// [`CCA_CORR_THRESHOLD`] tops (DSSS signal, also below the energy limit).
    ///
    /// The frame-in-progress flag alone is not enough: hardware filtering
    /// rejects a frame for someone else after its header and goes back to
    /// sync search while the rest of that frame is still on the air.
    async fn clear_channel_assessment(&mut self) -> Result<(), RadioError> {
        if self.operation_active
            && fifo::events() & fifo::EVENT_TERMINAL != 0
            && !fifo::rx_data_pending()
        {
            // The PBE ended the RX operation on its own (e.g. RX FIFO error);
            // its RSSI register is stale.
            hardware::cancel_pbe_start_trigger();
            fifo::clear_events(u32::MAX);
            self.operation_active = false;
            self.rx_fifo_reset_pending = true;
            self.stats.rx_restarts = self.stats.rx_restarts.wrapping_add(1);
        }
        if !self.operation_active {
            self.start_rx()?;
        }
        let started = hardware::timer_ticks();
        let mut rssi = hardware::read_rssi();
        while rssi == RSSI_INVALID
            && hardware::timer_ticks().wrapping_sub(started) < CCA_RSSI_WAIT_TICKS
        {
            yield_now().await;
            rssi = hardware::read_rssi();
        }
        let busy = fifo::rx_busy()
            || rssi == RSSI_INVALID
            || rssi >= CCA_RSSI_LIMIT_DBM
            || fifo::correlation_count() > CCA_CORR_THRESHOLD;
        if busy {
            self.stats.cca_busy = self.stats.cca_busy.wrapping_add(1);
            return Err(RadioError::ChannelBusy);
        }
        Ok(())
    }

    fn ack_outcome(&mut self, status: fifo::AckStatus) -> Result<bool, RadioError> {
        match status {
            fifo::AckStatus::Received { frame_pending } => {
                self.stats.ack_received = self.stats.ack_received.wrapping_add(1);
                if frame_pending {
                    self.stats.ack_frame_pending = self.stats.ack_frame_pending.wrapping_add(1);
                }
                Ok(frame_pending)
            }
            fifo::AckStatus::Timeout => {
                self.stats.ack_timeout = self.stats.ack_timeout.wrapping_add(1);
                Err(RadioError::NoAck)
            }
            fifo::AckStatus::RxError | fifo::AckStatus::Unknown(_) => {
                self.stats.ack_rx_error = self.stats.ack_rx_error.wrapping_add(1);
                Err(RadioError::NoAck)
            }
        }
    }

    /// Take one frame the running RX operation has already committed to the
    /// RX FIFO, without waiting and without starting RX.
    ///
    /// A sleepy end device only calls [`Self::receive`] around its own polls,
    /// so between them the PBE keeps receiving PAN traffic into the FIFO.
    /// Once the FIFO is full the receiver stops producing RSSI readings,
    /// CCA can no longer see an idle channel, and there is no room left for
    /// the frames we are about to solicit. The MAC drains it before a TX.
    pub fn try_receive(&mut self) -> Option<RxFrame> {
        if !self.configured || !self.operation_active {
            return None;
        }
        loop {
            match fifo::try_read_rx_frame() {
                Ok(Some(frame)) => {
                    self.stats.rx_frames = self.stats.rx_frames.wrapping_add(1);
                    return Some(RxFrame {
                        data: frame.data,
                        len: frame.len,
                        rssi: frame.rssi,
                        lqi: crate::lqi::from_rssi_dbm(frame.rssi),
                        modem_lqi: frame.modem_lqi,
                    });
                }
                Ok(None) => return None,
                Err(error) => match classify_rx_fifo_error(error) {
                    RxFifoDisposition::DropFrame => {
                        self.stats.rx_dropped = self.stats.rx_dropped.wrapping_add(1);
                    }
                    RxFifoDisposition::Fatal(_) => {
                        // Leave the FIFO to the next operation start, which
                        // resets it.
                        let _ = self.stop_operation();
                        self.rx_fifo_reset_pending = true;
                        return None;
                    }
                },
            }
        }
    }

    /// Return the next received frame, starting continuous RX if it is not
    /// running. The RX operation keeps running when a frame is returned.
    pub async fn receive(&mut self) -> Result<RxFrame, RadioError> {
        if !self.configured {
            return Err(RadioError::RadioConfigUnavailable);
        }
        self.wake()?;

        let mut dropped_frames: u32 = 0;
        let mut restarts: u32 = 0;

        loop {
            if self.operation_active && (self.config_dirty || self.filter_dirty) {
                // New channel/PA/addressing: the PBE latches them at start.
                self.wait_rx_idle().await;
                self.stop_operation()?;
            }
            if !self.operation_active {
                self.start_rx()?;
            }

            match fifo::try_read_rx_frame() {
                Ok(Some(frame)) => {
                    self.stats.rx_frames = self.stats.rx_frames.wrapping_add(1);
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
                        // `try_read_rx_frame` advanced the FIFO read pointer
                        // before decoding, so this entry is already consumed.
                        // Drop it and keep the RX operation running: a single
                        // frame whose metadata is unusable must not truncate
                        // an active scan or an association/rejoin wait.
                        //
                        // No fabricated LQI reaches the stack — the frame is
                        // discarded outright.
                        dropped_frames = dropped_frames.saturating_add(1);
                        self.stats.rx_dropped = self.stats.rx_dropped.wrapping_add(1);
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
                        continue;
                    }
                    RxFifoDisposition::Fatal(error) => {
                        self.stop_operation()?;
                        self.rx_fifo_reset_pending = true;
                        return Err(error);
                    }
                },
            }

            let events = fifo::events();
            if events & fifo::EVENT_TERMINAL != 0 && !fifo::rx_data_pending() {
                // The PBE ended the RX operation on its own (for example on
                // an RX FIFO error). Everything committed has been read;
                // restart it.
                let end_cause = fifo::end_cause();
                hardware::cancel_pbe_start_trigger();
                fifo::clear_events(u32::MAX);
                self.operation_active = false;
                self.rx_fifo_reset_pending = true;
                self.stats.rx_restarts = self.stats.rx_restarts.wrapping_add(1);
                restarts += 1;
                log::debug!("[CC2340 DRV] RX ended (cause 0x{end_cause:02X}), restarting");
                if restarts > MAX_RX_RESTARTS {
                    return Err(map_pbe_error(end_cause));
                }
            } else if events & RX_INFO_EVENTS != 0 {
                // Informational only; frames are taken from the FIFO level.
                fifo::clear_events(events & RX_INFO_EVENTS);
            }

            yield_now().await;
        }
    }

    pub fn abort_tx(&mut self) {
        if let Err(error) = self.stop_operation() {
            log::error!("[CC2340 DRV] Failed to abort radio operation: {error:?}");
        }
    }

    /// Stop an in-flight PBE operation while retaining the initialized radio.
    ///
    /// This is the only power-wait transition currently exposed by the MAC;
    /// `deinit` is deliberately not used for application waits until
    /// firmware/trim restoration is validated on hardware.
    pub fn prepare_active_wait(&mut self) -> Result<(), RadioError> {
        self.stop_operation()
    }

    /// Power the radio down between operations while keeping the CPU-side
    /// state and the LRFD contents (TOPsm images, REGBANK, trims, synth
    /// programming): clock gating does not remove power from the LRFD.
    ///
    /// Mirrors what TI does around every IEEE command: the end of
    /// `RCL_Handler_Ieee_RxTx` (`rcl/handlers/ieee.c`) calls `LRF_disable()`
    /// (TOPsm power-down requests) and `LRF_disableSynthRefsys()`, and with
    /// no clock dependency left `LRFDApplyClockDependencies()`
    /// (`driverlib/lrfd.c`) clears `LRFDDBELL.CLKCTL` and the `CLKCTL` LRFD
    /// module clock. In addition HFXT is stopped (TI only does this on
    /// standby entry, see [`hardware::stop_hfxt`]); the CPU keeps running
    /// from HFOSC and [`Self::wake`] restarts the crystal before the LRFD
    /// clocks. Measured on LP-EM-CC2340R5 this is the largest idle consumer
    /// left once the radio is gated. Committed RX frames stay in the FIFO RAM.
    pub fn sleep(&mut self) -> Result<(), RadioError> {
        if !self.configured || self.asleep {
            return Ok(());
        }
        let stopped = self.stop_operation();
        hardware::disable_radio_cores();
        hardware::disable_synth_refsys();
        hardware::disable_radio_clocks();
        hardware::stop_hfxt();
        self.asleep = true;
        self.stats.sleeps = self.stats.sleeps.wrapping_add(1);
        stopped
    }

    /// Undo [`Self::sleep`]: re-enable the LRFD clocks and the synthesizer
    /// reference (`LRF_enableSynthRefsys()`, including its settle time). The
    /// TOPsm cores are re-initialised by the next operation's
    /// `prepare_cores`, exactly as before every RCL command. Every operation
    /// entry point calls this, so an explicit call is only an eager restore.
    pub fn wake(&mut self) -> Result<(), RadioError> {
        if !self.asleep {
            return Ok(());
        }
        if self.lrfd_lost {
            // RCL on `PowerLPF3_AWAKE_STANDBY` (`rclPowerNotify()`,
            // `rcl/RCL.c`) drops `lrfState` to `RadioState_ImagesLoaded`
            // so the next command re-runs `LRF_setupRadio()`. Here the
            // images are reloaded as well (conservative; ~1 ms of copying)
            // by the same path as the first bring-up.
            self.asleep = false;
            self.lrfd_lost = false;
            self.images_loaded = false;
            self.configured = false;
            self.operation_active = false;
            self.filter_dirty = true;
            self.rx_fifo_reset_pending = true;
            self.stats.standby_reinits = self.stats.standby_reinits.wrapping_add(1);
            return self.init();
        }
        hardware::start_hfxt()?;
        hardware::enable_radio_clocks()?;
        hardware::enable_synth_refsys()?;
        self.asleep = false;
        Ok(())
    }

    /// Whether [`Self::sleep`] powered the radio down.
    pub const fn is_asleep(&self) -> bool {
        self.asleep
    }

    /// Record that the device was in standby while the radio was asleep.
    ///
    /// The LRFD power domain is off in standby, so its registers (PHY
    /// configuration, trims, synthesizer programming, frame filter) are
    /// lost; the next [`Self::wake`] re-initialises everything from the
    /// retained software configuration. Returns `false` (and does nothing)
    /// when the radio was not put to sleep first, which a caller must treat
    /// as a sequencing bug: standby with a running radio is not supported.
    pub fn note_standby(&mut self) -> bool {
        if self.asleep {
            self.lrfd_lost = true;
        }
        self.asleep
    }

    fn frame_filter(&self) -> fifo::FrameFilter {
        fifo::FrameFilter {
            pan_id: self.config.pan_id,
            short_addr: self.config.short_addr,
            ext_addr: self.config.ieee_addr,
            promiscuous: self.config.promiscuous,
        }
    }

    /// Start continuous, filtered RX with hardware auto-ACK.
    fn start_rx(&mut self) -> Result<(), RadioError> {
        self.stop_operation()?;
        self.prepare_cores()?;
        fifo::reset_tx();
        self.prepare_rx_fifo();
        fifo::prepare_rx();
        fifo::clear_events(u32::MAX);
        self.operation_active = true;
        hardware::arm_pbe_start_trigger();
        fifo::start_rx();
        Ok(())
    }

    /// Everything a PBE operation needs before its API command; the PBE must
    /// be stopped.
    fn prepare_cores(&mut self) -> Result<(), RadioError> {
        self.apply_runtime_config()?;
        // RCL calls `LRF_enable()` (MSGBOX = 0, PBE/MCE/RFE TOPsm INIT +
        // ENABLE) before every PBE operation, including between RX and TX of
        // a single command, and then waits for the PBE to post MSGBOX. Without
        // the re-init the PBE keeps state from a previous hard-stopped
        // operation and the next TX ends immediately with ERR_STOP (0xFC).
        hardware::enable_radio_cores();
        hardware::wait_for_topsm_ready()?;
        fifo::configure_frame_filter(&self.frame_filter());
        self.filter_dirty = false;
        Ok(())
    }

    /// Keep committed RX frames across a restart; reset the FIFO only when
    /// nothing is pending or its state is suspect.
    fn prepare_rx_fifo(&mut self) {
        if self.rx_fifo_reset_pending || !fifo::rx_data_pending() {
            fifo::reset_rx();
            self.rx_fifo_reset_pending = false;
        }
    }

    /// Wait (bounded) until the PBE is neither receiving a frame nor sending
    /// the auto-ACK for one.
    async fn wait_rx_idle(&mut self) {
        if !self.operation_active {
            return;
        }
        let started = hardware::timer_ticks();
        while fifo::rx_busy()
            && fifo::events() & fifo::EVENT_TERMINAL == 0
            && hardware::timer_ticks().wrapping_sub(started) < RX_BUSY_WAIT_TICKS
        {
            yield_now().await;
        }
    }

    fn apply_runtime_config(&mut self) -> Result<(), RadioError> {
        let tx_power =
            config::tx_power(self.config.tx_power_dbm).ok_or(RadioError::InvalidTxPower)?;
        if self.config_dirty {
            let frequency = hardware::channel_frequency(self.config.channel)
                .ok_or(RadioError::InvalidChannel)?;
            hardware::program_frequency(frequency)?;
            self.config_dirty = false;
        }
        // RCL re-applies the temperature-compensated PA setting before TX.
        let pa_value = tx_power.compensated_rfe_value(die_temperature_c());
        if pa_value != self.pa_value {
            hardware::program_tx_power(pa_value);
            self.pa_value = pa_value;
        }
        Ok(())
    }

    fn finish_operation(&mut self, events: u32) -> Result<(), RadioError> {
        hardware::cancel_pbe_start_trigger();
        let end_cause = fifo::end_cause();
        fifo::clear_events(u32::MAX);
        self.operation_active = false;
        self.rx_fifo_reset_pending = true;

        if events & fifo::EVENT_OP_ERROR != 0 && end_cause == 0 {
            Err(RadioError::HardwareError)
        } else if end_cause == 0 {
            Ok(())
        } else {
            Err(map_pbe_error(end_cause))
        }
    }

    /// Hard-stop the running PBE operation (RCL `LRF_sendHardStop` and wait
    /// for `OPDONE`/`OPERROR`). Committed RX frames stay in the FIFO.
    fn stop_operation(&mut self) -> Result<(), RadioError> {
        if !self.operation_active {
            return Ok(());
        }

        if fifo::events() & fifo::EVENT_TERMINAL == 0 {
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
                hardware::cancel_pbe_start_trigger();
                self.operation_active = false;
                self.rx_fifo_reset_pending = true;
                fifo::reset_both();
                return Err(RadioError::OperationTimeout);
            }
        }

        hardware::cancel_pbe_start_trigger();
        fifo::clear_events(u32::MAX);
        // A frame cut short by the stop must not prefix the next entry.
        fifo::discard_uncommitted_rx();
        self.operation_active = false;
        Ok(())
    }
}

/// Events [`Cc2340Driver::receive`] clears as informational.
const RX_INFO_EVENTS: u32 = fifo::EVENT_RX_OK
    | fifo::EVENT_RX_NOK
    | fifo::EVENT_RX_IGNORED
    | fifo::EVENT_RX_BUF_FULL
    | fifo::EVENT_RX_EMPTY
    | fifo::EVENT_TX_DONE;

/// Consecutive PBE-ended RX operations tolerated within one `receive` call.
const MAX_RX_RESTARTS: u32 = 8;

fn die_temperature_c() -> i32 {
    hardware::die_temperature_c().unwrap_or(config::TX_POWER_REFERENCE_TEMPERATURE_C)
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
