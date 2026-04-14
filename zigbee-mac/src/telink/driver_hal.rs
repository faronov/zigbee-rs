//! Telink TLSR8258 radio driver using `tlsr82xx-hal` crate.
//!
//! Wraps the HAL's `Radio` struct with async TX/RX via Embassy signals
//! and DMA buffer management for IEEE 802.15.4 Zigbee operation.
//!
//! This replaces the direct-register `driver.rs` with a cleaner
//! HAL-based implementation that leverages the community crate.

use core::sync::atomic::{AtomicBool, AtomicI8, AtomicU8, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use tlsr82xx_hal::radio::{IrqFlags, Radio, RadioMode, RadioPower, ZigbeeConfig};

// ── Constants ───────────────────────────────────────────────────

const RF_PKT_BUFF_LEN: usize = 144;
const ZB_RADIO_RX_HDR_LEN: usize = 13;
const MAX_FRAME_LEN: usize = 127;
const CCA_RSSI_THRESHOLD: i8 = -70;

// ── Async completion signals ────────────────────────────────────

static TX_DONE: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static RX_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

static RX_LEN: AtomicU8 = AtomicU8::new(0);
static RX_CRC_OK: AtomicBool = AtomicBool::new(false);
static RX_RSSI: AtomicI8 = AtomicI8::new(-128);

// ── DMA buffers (must be 4-byte aligned, in static RAM) ─────────

#[repr(C, align(4))]
struct DmaBuf([u8; RF_PKT_BUFF_LEN]);

static mut RX_DMA_BUF: DmaBuf = DmaBuf([0u8; RF_PKT_BUFF_LEN]);
static mut TX_DMA_BUF: DmaBuf = DmaBuf([0u8; RF_PKT_BUFF_LEN]);

// ── Received frame ──────────────────────────────────────────────

#[derive(Debug)]
pub struct ReceivedFrame {
    pub data: [u8; MAX_FRAME_LEN],
    pub len: usize,
    pub rssi: i8,
    pub lqi: u8,
}

// ── Radio configuration ─────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct RadioConfig {
    pub channel: u8,
    pub tx_power: RadioPower,
}

impl Default for RadioConfig {
    fn default() -> Self {
        Self {
            channel: 11,
            tx_power: RadioPower::PLUS_3P23_DBM,
        }
    }
}

// ── Error type ──────────────────────────────────────────────────

#[derive(Debug)]
pub enum DriverError {
    NotInitialized,
    InvalidFrame,
    TxTimeout,
    RxTimeout,
    CrcError,
    HardwareError,
}

// ── HAL-based driver ────────────────────────────────────────────

pub struct TelinkHalDriver {
    radio: Radio,
    config: RadioConfig,
    initialized: bool,
}

impl TelinkHalDriver {
    pub fn new(config: RadioConfig) -> Self {
        Self {
            radio: Radio::new(),
            config,
            initialized: false,
        }
    }

    /// Initialize the radio hardware for IEEE 802.15.4 Zigbee.
    pub fn init(&mut self) {
        // Initialize radio in Zigbee 250K mode
        self.radio.init_mode(RadioMode::Zigbee250K).ok();

        // Set channel and power
        self.radio.set_zigbee_channel(self.config.channel).ok();
        self.radio.set_power(self.config.tx_power);

        // Configure RX DMA buffer
        unsafe {
            self.radio
                .configure_rx_buffer(&mut (*core::ptr::addr_of_mut!(RX_DMA_BUF)).0)
                .ok();
        }

        // Enable TX DMA channel
        self.radio.enable_dma_tx_channel();

        // Enable radio IRQs for TX and RX
        self.radio.clear_all_irq_status();
        self.radio.set_irq_mask(IrqFlags::TX);
        self.radio.set_irq_mask(IrqFlags::RX);

        self.initialized = true;
    }

    /// Set the radio channel (11-26 for 802.15.4).
    pub fn set_channel(&mut self, channel: u8) {
        self.config.channel = channel;
        if self.initialized {
            self.radio.set_zigbee_channel(channel).ok();
        }
    }

    /// Set TX power level.
    pub fn set_tx_power(&mut self, power: RadioPower) {
        self.config.tx_power = power;
        if self.initialized {
            self.radio.set_power(power);
        }
    }

    /// Transmit an 802.15.4 frame (async, waits for TX IRQ).
    pub async fn transmit(&mut self, frame: &[u8]) -> Result<(), DriverError> {
        if !self.initialized {
            return Err(DriverError::NotInitialized);
        }
        if frame.is_empty() || frame.len() > MAX_FRAME_LEN {
            return Err(DriverError::InvalidFrame);
        }

        TX_DONE.reset();

        // Build DMA TX packet: [dmaLen(4)] [rfLen(1)] [payload...]
        let rf_len = (frame.len() + 2) as u8; // +2 for CRC
        unsafe {
            let buf = &mut (*core::ptr::addr_of_mut!(TX_DMA_BUF)).0;
            let dma_len = (frame.len() + 1) as u32; // rfLen + payload
            buf[0] = dma_len as u8;
            buf[1] = (dma_len >> 8) as u8;
            buf[2] = (dma_len >> 16) as u8;
            buf[3] = (dma_len >> 24) as u8;
            buf[4] = rf_len;
            buf[5..5 + frame.len()].copy_from_slice(frame);
        }

        // Start TX
        unsafe {
            self.radio
                .tx_packet_now(&(*core::ptr::addr_of!(TX_DMA_BUF)).0)
                .map_err(|_| DriverError::HardwareError)?;
        }

        // Wait for TX completion via IRQ
        let result = embassy_futures::select::select(
            TX_DONE.wait(),
            embassy_time::Timer::after(embassy_time::Duration::from_millis(10)),
        )
        .await;

        match result {
            embassy_futures::select::Either::First(ok) => {
                if ok {
                    Ok(())
                } else {
                    Err(DriverError::HardwareError)
                }
            }
            embassy_futures::select::Either::Second(_) => {
                self.radio.stop_trx();
                Err(DriverError::TxTimeout)
            }
        }
    }

    /// Receive the next 802.15.4 frame (async, waits for RX IRQ).
    pub async fn receive(&mut self) -> Result<ReceivedFrame, DriverError> {
        if !self.initialized {
            return Err(DriverError::NotInitialized);
        }

        RX_DONE.reset();

        // Re-configure RX buffer and start RX
        unsafe {
            (*core::ptr::addr_of_mut!(RX_DMA_BUF)).0.fill(0);
            self.radio
                .configure_rx_buffer(&mut (*core::ptr::addr_of_mut!(RX_DMA_BUF)).0)
                .ok();
        }
        self.radio.set_rx_mode();

        // Wait for RX via IRQ
        RX_DONE.wait().await;

        if !RX_CRC_OK.load(Ordering::Acquire) {
            return Err(DriverError::CrcError);
        }

        let len = RX_LEN.load(Ordering::Acquire) as usize;
        if len == 0 {
            return Err(DriverError::RxTimeout);
        }

        let rssi = RX_RSSI.load(Ordering::Acquire);
        let lqi = rssi_to_lqi(rssi);

        let mut frame = ReceivedFrame {
            data: [0u8; MAX_FRAME_LEN],
            len,
            rssi,
            lqi,
        };

        unsafe {
            let payload_start = ZB_RADIO_RX_HDR_LEN;
            frame.data[..len].copy_from_slice(
                &(&(*core::ptr::addr_of!(RX_DMA_BUF)).0)[payload_start..payload_start + len],
            );
        }

        Ok(frame)
    }

    /// Perform CCA (Clear Channel Assessment).
    pub fn cca(&self) -> bool {
        if !self.initialized {
            return false;
        }
        self.radio.rssi_dbm_154() < CCA_RSSI_THRESHOLD
    }

    /// Read current RSSI in dBm.
    pub fn rssi(&self) -> i8 {
        self.radio.rssi_dbm_154()
    }

    /// Get current config.
    pub fn config(&self) -> &RadioConfig {
        &self.config
    }

    /// Enable RX mode.
    pub fn enable_rx(&mut self) {
        self.radio.set_rx_mode();
    }

    /// Disable radio (idle).
    pub fn disable_radio(&mut self) {
        self.radio.set_tx_rx_off();
    }
}

// ── IRQ handlers (called from platform interrupt) ───────────────

/// Call from the RF TX IRQ handler.
pub fn rf_tx_irq_handler() {
    let mut radio = Radio::new();
    if radio.tx_finished() {
        radio.clear_irq_status(IrqFlags::TX);
        TX_DONE.signal(true);
    }
}

/// Call from the RF RX IRQ handler.
pub fn rf_rx_irq_handler() {
    let mut radio = Radio::new();
    if radio.rx_finished() {
        radio.clear_irq_status(IrqFlags::RX);

        // TLSR8258 RX DMA buffer layout:
        //   [0..3]  dmaLen (u32)
        //   [4]     per-packet RSSI (raw, subtract 110 for dBm)
        //   [5..11] reserved (7 bytes)
        //   [12]    payloadLen (802.15.4 PHR = PSDU length including FCS)
        //   [13..]  payload (PSDU)
        //   After payload: CRC status byte — bit 4 = CRC OK
        unsafe {
            let buf = &(*core::ptr::addr_of!(RX_DMA_BUF)).0;
            let payload_len = buf[12] as usize; // PHR byte

            if payload_len >= 5 && payload_len <= MAX_FRAME_LEN + 2 {
                // Per-packet RSSI from DMA buffer (not live register)
                let rssi_raw = buf[4] as i8;
                let rssi = rssi_raw.saturating_sub(110); // Convert to dBm

                // CRC status byte is after the payload data
                // Position: 13 (header) + payload_len (includes 2 FCS bytes)
                // The status byte is at buf[13 + payload_len - 2] typically
                // On TLSR8258, CRC status is in the DMA length field or
                // at a fixed offset. Check bit 4 of the byte after payload.
                let crc_offset = ZB_RADIO_RX_HDR_LEN + payload_len;
                let crc_ok = if crc_offset < RF_PKT_BUFF_LEN {
                    (buf[crc_offset] & 0x10) != 0 // Bit 4 = CRC OK
                } else {
                    false
                };

                let frame_len = payload_len - 2; // Subtract 2-byte FCS
                RX_LEN.store(frame_len as u8, Ordering::Release);
                RX_RSSI.store(rssi, Ordering::Release);
                RX_CRC_OK.store(crc_ok, Ordering::Release);
            } else {
                RX_LEN.store(0, Ordering::Release);
                RX_CRC_OK.store(false, Ordering::Release);
            }
        }

        RX_DONE.signal(());
    }
}

// ── Utility ─────────────────────────────────────────────────────

fn rssi_to_lqi(rssi: i8) -> u8 {
    let clamped = (rssi as i16).clamp(-100, -20);
    (((clamped + 100) as u16) * 255 / 80) as u8
}
