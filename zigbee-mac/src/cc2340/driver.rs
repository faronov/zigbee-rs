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
    pub rssi: i8,
    pub lqi: u8,
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
        let frequency =
            hardware::channel_frequency(self.config.channel).ok_or(RadioError::InvalidChannel)?;
        let tx_power =
            config::tx_power(self.config.tx_power_dbm).ok_or(RadioError::InvalidTxPower)?;

        if !self.images_loaded {
            let images = firmware::images().ok_or(RadioError::FirmwareUnavailable)?;
            let phy_writes =
                config::ieee_802154_phy_writes().ok_or(RadioError::RadioConfigUnavailable)?;

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
                            lqi: frame.lqi,
                        });
                    }
                    Ok(None) => {}
                    Err(error) => {
                        let error = map_fifo_error(error);
                        self.abort_active_operation()?;
                        return Err(error);
                    }
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
