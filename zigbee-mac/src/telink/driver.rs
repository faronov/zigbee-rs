//! Low-level Telink TLSR8258 802.15.4 radio driver using direct register access.
//!
//! Provides async TX/RX on top of the TLSR8258 IEEE 802.15.4 radio peripheral
//! using Embassy signals for interrupt-driven completion notification.
//!
//! All radio control is done via volatile memory-mapped register access,
//! eliminating the need for the Telink `tl_zigbee_sdk` C library
//! (`libdrivers_8258.a`).
//!
//! # Radio peripheral overview
//! - 2.4 GHz IEEE 802.15.4 compliant
//! - Hardware CRC generation and checking
//! - Configurable TX power (chip-dependent power table indices)
//! - RSSI / LQI measurement
//! - Energy Detection (ED) scan
//! - CCA (Clear Channel Assessment) with configurable threshold
//! - DMA-based TX/RX with hardware packet format
//!
//! # Architecture
//! ```text
//! TelinkDriver (Rust, async)
//!   ├── Direct register access (volatile MMIO at 0x800000+)
//!   │     ├── set_channel / set_tx_power / set_trx_state / ...
//!   │     ├── tx_ready + tx_trigger / set_rx_buf
//!   │     └── perform_cca / start_ed_scan / stop_ed_scan / get_lqi
//!   ├── TX completion: rf_tx_irq_handler() → TX_SIGNAL
//!   └── RX completion: rf_rx_irq_handler() → RX_SIGNAL
//! ```
//!
//! # Packet format
//! TX buffer layout (rf_sendPkt_t):
//! ```text
//! [0..3]  dmaLen  (u32, little-endian — set by DMA header build)
//! [4]     rfLen   (payload length + 2 for CRC)
//! [5..]   payload (MAC frame)
//! ```
//!
//! RX buffer layout (rf_recvPkt_t):
//! ```text
//! [0..3]  dmaLen      (u32, DMA transfer length)
//! [4]     rssi        (raw RSSI byte)
//! [5..11] reserved    (7 bytes)
//! [12]    payloadLen  (802.15.4 PSDU length)
//! [13..]  payload     (MAC frame)
//! ```
//!
//! # Build requirements
//! No external C libraries are required. The RF interrupt must be registered
//! in platform startup code so that `rf_rx_irq_handler` and
//! `rf_tx_irq_handler` are called from the ISR.

use core::sync::atomic::{AtomicBool, AtomicI8, AtomicU8, AtomicU32, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

// ── Constants ───────────────────────────────────────────────────

/// Total DMA buffer length for radio packets.
const RF_PKT_BUFF_LEN: usize = 144;

/// RX buffer header length before the 802.15.4 payload.
/// dmaLen(4) + rssi(1) + reserved(7) + payloadLen(1) = 13 bytes.
const ZB_RADIO_RX_HDR_LEN: usize = 13;

/// Maximum 802.15.4 PHY frame size (PSDU including FCS).
const MAX_FRAME_LEN: usize = 127;

// ── Radio states ───────────────────────────────────────────────

const RF_STATE_RX: u8 = 1;
const RF_STATE_OFF: u8 = 3;

// ── CCA result codes ───────────────────────────────────────────

const PHY_CCA_IDLE: u8 = 0x04;
#[allow(dead_code)]
const PHY_CCA_BUSY: u8 = 0x00;

/// CCA RSSI threshold in raw register units.
/// -70 dBm + 110 offset = 40.
const CCA_RSSI_THRESHOLD: u8 = 40;

// ── TLSR8258 Register Map ──────────────────────────────────────
// All registers are memory-mapped at 0x800000 + offset.
// Offsets from Telink SDK register_8258.h.

const REG_BASE: u32 = 0x800000;

// RF core registers
const REG_RF_MODE_CFG: u32 = REG_BASE + 0x400; // RF mode configuration
const REG_RF_PA_POWER: u32 = REG_BASE + 0x404; // PA power setting
const REG_RF_ACC_LEN: u32 = REG_BASE + 0x405; // Access code length
const REG_RF_CHN: u32 = REG_BASE + 0x408; // Channel frequency
const REG_RF_RSSI: u32 = REG_BASE + 0x441; // Instantaneous RSSI

// DMA registers
const REG_DMA2_ADDR: u32 = REG_BASE + 0xC08; // RF RX DMA buffer (low 16 bits)
const REG_DMA3_ADDR: u32 = REG_BASE + 0xC0C; // RF TX DMA buffer (low 16 bits)
const REG_DMA_CHN_EN: u32 = REG_BASE + 0xC20; // DMA channel enable
const REG_DMA_CHN_IRQ_MSK: u32 = REG_BASE + 0xC21; // DMA IRQ mask

// RF link-layer control
const REG_RF_LL_CTRL_0: u32 = REG_BASE + 0xF02; // LL control 0
const REG_RF_LL_CTRL_1: u32 = REG_BASE + 0xF03; // LL control 1
#[allow(dead_code)]
const REG_RF_RX_TIMEOUT: u32 = REG_BASE + 0xF0A; // RX timeout
#[allow(dead_code)]
const REG_RF_LL_CTRL_2: u32 = REG_BASE + 0xF15; // LL control 2
const REG_RF_LL_CTRL_3: u32 = REG_BASE + 0xF16; // LL control 3
const REG_RF_IRQ_MASK: u32 = REG_BASE + 0xF1C; // RF IRQ mask (u16)
const REG_RF_IRQ_STATUS: u32 = REG_BASE + 0xF20; // RF IRQ status (u16)

// System timer (32-bit, 16 MHz)
const REG_SYSTEM_TIMER: u32 = REG_BASE + 0x740;
/// System timer wake compare register — CPU wakes from suspend when timer reaches this value.
const REG_SYSTEM_WAKEUP_TICK: u32 = REG_BASE + 0x748;

// Power Management registers
/// Wake-up source enable register.
/// BIT(4)=PAD, BIT(5)=CORE, BIT(6)=TIMER, BIT(7)=COMPARATOR
const REG_WAKEUP_EN: u32 = REG_BASE + 0x6E;
/// Power-down control. BIT(7) = enter suspend/deep-sleep.
const REG_PWDN_CTRL: u32 = REG_BASE + 0x6F;

/// System timer clock: 16 MHz = 16 ticks per microsecond.
const SYSTEM_TIMER_TICKS_PER_US: u32 = 16;

/// Wake source: timer
const PM_WAKEUP_TIMER: u8 = 1 << 6;
/// Sleep trigger bit
const FLD_PWDN_CTRL_SLEEP: u8 = 1 << 7;

// RF IRQ bit masks (for REG_RF_IRQ_MASK / REG_RF_IRQ_STATUS)
const FLD_RF_IRQ_TX: u16 = 1 << 1; // TX done
const FLD_RF_IRQ_RX: u16 = 1 << 0; // RX done
#[allow(dead_code)]
const FLD_RF_IRQ_RX_TO: u16 = 1 << 2; // RX timeout
#[allow(dead_code)]
const FLD_RF_IRQ_TX_DS: u16 = 1 << 8; // TX DMA done
#[allow(dead_code)]
const FLD_RF_IRQ_RX_DR: u16 = 1 << 9; // RX DMA ready

// RF mode values (low 2 bits of REG_RF_LL_CTRL_3)
#[allow(dead_code)]
const RF_MODE_TX: u8 = 0;
const RF_MODE_RX: u8 = 1;
#[allow(dead_code)]
const RF_MODE_AUTO: u8 = 2;
const RF_MODE_OFF: u8 = 3;

// DMA channel bits (for REG_DMA_CHN_EN / REG_DMA_CHN_IRQ_MSK)
const DMA_CHN_RF_RX: u8 = 1 << 2; // Channel 2 — RF RX
const DMA_CHN_RF_TX: u8 = 1 << 3; // Channel 3 — RF TX

// LL ctrl_1 bit for timestamp capture
const FLD_LL_CTRL1_TIMESTAMP_EN: u8 = 1 << 5;

/// TX power lookup table: index → PA register value.
/// Entries map power levels 0–10 to TLSR8258 PA register values
/// (from Telink SDK rf_power_level_e).
const TX_POWER_TABLE: [u8; 11] = [
    0x06, // 0: -20 dBm
    0x06, // 1: -15 dBm (same PA setting, attenuator varies)
    0x06, // 2: -10 dBm
    0x25, // 3:  -5 dBm
    0x2C, // 4:  -2 dBm
    0x30, // 5:   0 dBm
    0x36, // 6:  +1 dBm
    0x41, // 7:  +3 dBm
    0x52, // 8:  +5 dBm
    0x61, // 9:  +7 dBm
    0xBF, // 10: +10 dBm
];

// ── Register access helpers ────────────────────────────────────

#[inline(always)]
fn reg_write_u8(addr: u32, val: u8) {
    unsafe { core::ptr::write_volatile(addr as *mut u8, val) }
}

#[inline(always)]
fn reg_read_u8(addr: u32) -> u8 {
    unsafe { core::ptr::read_volatile(addr as *const u8) }
}

#[inline(always)]
fn reg_write_u16(addr: u32, val: u16) {
    unsafe { core::ptr::write_volatile(addr as *mut u16, val) }
}

#[allow(dead_code)]
#[inline(always)]
fn reg_read_u16(addr: u32) -> u16 {
    unsafe { core::ptr::read_volatile(addr as *const u16) }
}

#[allow(dead_code)]
#[inline(always)]
fn reg_write_u32(addr: u32, val: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
}

#[allow(dead_code)]
#[inline(always)]
fn reg_read_u32(addr: u32) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

// ── Pure-Rust radio functions (replace tl_zigbee_sdk FFI) ──────

/// Initialize the radio hardware for IEEE 802.15.4 operation.
/// Replaces `rf_init()` + `mac_trxInit()`.
fn radio_init() {
    // Enable DMA channels 2 (RF RX) and 3 (RF TX)
    let dma_en = reg_read_u8(REG_DMA_CHN_EN);
    reg_write_u8(REG_DMA_CHN_EN, dma_en | DMA_CHN_RF_RX | DMA_CHN_RF_TX);

    // Enable DMA IRQs for RF RX/TX channels
    let dma_irq = reg_read_u8(REG_DMA_CHN_IRQ_MSK);
    reg_write_u8(REG_DMA_CHN_IRQ_MSK, dma_irq | DMA_CHN_RF_RX | DMA_CHN_RF_TX);

    // Set access code length to 4 bytes (802.15.4 preamble + SFD)
    reg_write_u8(REG_RF_ACC_LEN, 4);

    // Set 802.15.4 mode in RF mode configuration register
    // Bit pattern: 802.15.4 packet format, CRC-16 enabled
    reg_write_u8(REG_RF_MODE_CFG, 0x20);

    // Configure LL control 0 for 802.15.4 mode
    reg_write_u8(REG_RF_LL_CTRL_0, 0x45);

    // Enable timestamp capture in LL control 1
    let ctrl1 = reg_read_u8(REG_RF_LL_CTRL_1);
    reg_write_u8(REG_RF_LL_CTRL_1, ctrl1 | FLD_LL_CTRL1_TIMESTAMP_EN);

    // Enable RF IRQs for TX and RX completion
    reg_write_u16(REG_RF_IRQ_MASK, FLD_RF_IRQ_TX | FLD_RF_IRQ_RX);

    // Clear any pending IRQ status
    reg_write_u16(REG_RF_IRQ_STATUS, 0xFFFF);

    // Start in OFF state
    set_trx_state(RF_STATE_OFF);
}

/// Set the IEEE 802.15.4 channel (11–26).
/// Replaces `rf_setChannel()`.
fn set_channel(channel: u8) {
    // 802.15.4 channel frequency: 2405 + 5*(channel-11) MHz
    // Physical frequency offset from 2400 MHz: (channel - 10) * 5
    let freq = ((channel as u16).wrapping_sub(10)) * 5;
    reg_write_u16(REG_RF_CHN, freq);
}

/// Set the TX power level.
/// Replaces `rf_setTxPower()`.
fn set_tx_power(power: u8) {
    let idx = (power as usize).min(TX_POWER_TABLE.len() - 1);
    reg_write_u8(REG_RF_PA_POWER, TX_POWER_TABLE[idx]);
}

/// Set the RX DMA buffer address.
/// Replaces `rf_setRxBuf()`.
fn set_rx_buf(buf: *mut u8) {
    // TLSR8258 DMA address registers hold the low 16 bits of the pointer.
    // RAM is at 0x840000+, so the low 16 bits uniquely identify the buffer.
    let addr_lo = buf as u32 as u16;
    reg_write_u16(REG_DMA2_ADDR, addr_lo);
}

/// Set the radio transceiver state (RX / OFF).
/// Replaces `rf_setTrxState()`.
fn set_trx_state(state: u8) {
    let mode = match state {
        RF_STATE_RX => RF_MODE_RX,
        RF_STATE_OFF => RF_MODE_OFF,
        _ => RF_MODE_OFF,
    };
    // Write mode to low 2 bits of LL ctrl_3, preserving other bits
    let ctrl = reg_read_u8(REG_RF_LL_CTRL_3);
    reg_write_u8(REG_RF_LL_CTRL_3, (ctrl & 0xFC) | mode);
}

/// Prepare the TX DMA buffer with the 802.15.4 frame.
/// Replaces `rf802154_tx_ready()`.
///
/// Builds the DMA header in `RF_TX_DMA_BUF` and copies the payload.
fn tx_ready(payload: *const u8, len: u8) {
    unsafe {
        let buf = core::ptr::addr_of_mut!(RF_TX_DMA_BUF) as *mut u8;
        // DMA length = rfLen field (1 byte) + payload (len) + CRC (2 bytes)
        let dma_len = (len as u32) + 3;
        core::ptr::write_volatile(buf as *mut u32, dma_len);
        // rfLen = payload length + 2 (hardware appends 2-byte CRC)
        core::ptr::write_volatile(buf.add(4), len + 2);
        // Copy payload after the header
        core::ptr::copy_nonoverlapping(payload, buf.add(5), len as usize);
    }
}

/// Trigger transmission of the prepared TX buffer.
/// Replaces `rf802154_tx()`.
fn tx_trigger() {
    // Set TX DMA buffer address (low 16 bits)
    let buf_addr = core::ptr::addr_of!(RF_TX_DMA_BUF) as u32 as u16;
    reg_write_u16(REG_DMA3_ADDR, buf_addr);

    // Trigger TX: set mode to TX in LL ctrl_3
    let ctrl = reg_read_u8(REG_RF_LL_CTRL_3);
    reg_write_u8(REG_RF_LL_CTRL_3, (ctrl & 0xFC) | RF_MODE_TX);
}

/// Perform Clear Channel Assessment.
/// Replaces `rf_performCCA()`.
///
/// Returns `PHY_CCA_IDLE` (0x04) if channel is idle, `PHY_CCA_BUSY` (0x00) if busy.
fn perform_cca() -> u8 {
    let rssi_raw = reg_read_u8(REG_RF_RSSI);
    if rssi_raw < CCA_RSSI_THRESHOLD {
        PHY_CCA_IDLE
    } else {
        PHY_CCA_BUSY
    }
}

/// Start an Energy Detection scan (switch to RX mode to sample RSSI).
/// Replaces `rf_startEDScan()`.
fn start_ed_scan() {
    set_trx_state(RF_STATE_RX);
}

/// Stop ED scan and return the energy level (0–255).
/// Replaces `rf_stopEDScan()`.
///
/// Reads the current RSSI and converts to an LQI-scaled ED value.
fn stop_ed_scan() -> u8 {
    let rssi_raw = reg_read_u8(REG_RF_RSSI);
    let rssi_dbm = (rssi_raw as i16) - 110;
    get_lqi(rssi_dbm as i8)
}

/// Convert RSSI (dBm) to Link Quality Indicator (0–255).
/// Replaces `rf_getLqi()`.
///
/// Linear mapping: 0 at ≤ -106 dBm, 255 at ≥ -6 dBm.
fn get_lqi(rssi: i8) -> u8 {
    let rssi_i16 = rssi as i16;
    let lqi = (255 * (rssi_i16 + 106)) / 100;
    lqi.clamp(0, 255) as u8
}

// ── Async completion signals ────────────────────────────────────
// Set from IRQ handler functions (ISR context), awaited by driver.

static TX_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static RX_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();

// ── RX frame buffer ─────────────────────────────────────────────
// Filled by rf_rx_irq_handler, read by receive() after awaiting RX_SIGNAL.
// Protected by the single-consumer pattern: ISR writes before signaling,
// async code reads after awaiting the signal, so no concurrent access.

struct SyncUnsafeCell<T>(core::cell::UnsafeCell<T>);

// SAFETY: RX_FRAME_BUF is only written in the ISR callback and read after
// the RX_SIGNAL is received, ensuring no concurrent access.
unsafe impl<T> Sync for SyncUnsafeCell<T> {}

impl<T> SyncUnsafeCell<T> {
    const fn new(value: T) -> Self {
        Self(core::cell::UnsafeCell::new(value))
    }
    fn get(&self) -> *mut T {
        self.0.get()
    }
}

static RX_FRAME_BUF: SyncUnsafeCell<[u8; MAX_FRAME_LEN]> =
    SyncUnsafeCell::new([0u8; MAX_FRAME_LEN]);
static RX_FRAME_LEN: AtomicU8 = AtomicU8::new(0);
static RX_FRAME_RSSI: AtomicI8 = AtomicI8::new(-127);
static RX_FRAME_TIMESTAMP: AtomicU32 = AtomicU32::new(0);
static RX_CRC_FAIL: AtomicBool = AtomicBool::new(false);
/// Monotonic counter incremented on each received frame (for relative ordering).
static RX_COUNTER: AtomicU8 = AtomicU8::new(0);

// DMA RX buffer given to the radio hardware (must be aligned and static).
static mut RF_RX_DMA_BUF: [u8; RF_PKT_BUFF_LEN] = [0u8; RF_PKT_BUFF_LEN];

// DMA TX buffer with room for header (5 bytes) + payload + CRC.
static mut RF_TX_DMA_BUF: [u8; RF_PKT_BUFF_LEN] = [0u8; RF_PKT_BUFF_LEN];

/// Received 802.15.4 frame with metadata.
pub struct ReceivedFrame {
    pub data: [u8; 128],
    pub len: usize,
    pub rssi: i8,
    pub lqi: u8,
    pub timestamp: u32,
}

/// Radio driver error.
#[derive(Debug, Clone, Copy)]
pub enum RadioError {
    /// CCA failure — channel is busy
    CcaFailure,
    /// TX was aborted
    TxAborted,
    /// Radio hardware error
    HardwareError,
    /// Invalid frame (too long or too short)
    InvalidFrame,
    /// Received frame failed CRC check
    CrcError,
    /// Radio not initialized
    NotInitialized,
}

/// Radio configuration for the Telink 802.15.4 radio.
pub struct RadioConfig {
    /// 802.15.4 channel (11–26)
    pub channel: u8,
    /// PAN ID for hardware address filtering
    pub pan_id: u16,
    /// Short address for hardware address filtering
    pub short_address: u16,
    /// Extended (IEEE) address
    pub extended_address: [u8; 8],
    /// Transmit power in dBm (-20 to +10 typical for Telink)
    pub tx_power: i8,
    /// Enable promiscuous mode (receive all frames)
    pub promiscuous: bool,
}

impl Default for RadioConfig {
    fn default() -> Self {
        Self {
            channel: 11,
            pan_id: 0xFFFF,
            short_address: 0xFFFF,
            extended_address: [0u8; 8],
            tx_power: 0,
            promiscuous: false,
        }
    }
}

/// Async driver for the Telink TLSR8258 IEEE 802.15.4 radio.
///
/// Uses direct memory-mapped register access for hardware control, with
/// Embassy signals for interrupt-driven async TX/RX.
pub struct TelinkDriver {
    config: RadioConfig,
    initialized: bool,
}

impl TelinkDriver {
    /// Create a new Telink radio driver with the given configuration.
    ///
    /// Initializes the radio hardware, configures the DMA RX buffer.
    ///
    /// # Safety
    /// The caller must ensure that:
    /// - Platform clocks and GPIOs are initialized
    /// - The RF interrupt is registered so that `rf_rx_irq_handler` and
    ///   `rf_tx_irq_handler` are called from the ISR
    pub fn new(config: RadioConfig) -> Self {
        let mut driver = Self {
            config,
            initialized: false,
        };
        driver.init_hardware();
        driver
    }

    /// Initialize the radio hardware.
    fn init_hardware(&mut self) {
        radio_init();

        // Point the radio DMA engine at our static RX buffer
        let rx_buf_ptr = core::ptr::addr_of_mut!(RF_RX_DMA_BUF) as *mut u8;
        set_rx_buf(rx_buf_ptr);

        // Apply initial config
        set_channel(self.config.channel);
        set_tx_power(self.config.tx_power as u8);

        // Start in RX state
        set_trx_state(RF_STATE_RX);

        self.initialized = true;
        log::info!("telink: radio initialized (pure-Rust register access)");
    }

    /// Apply the current configuration to the radio hardware.
    fn apply_config(&mut self) {
        if !self.initialized {
            return;
        }
        set_channel(self.config.channel);
        set_tx_power(self.config.tx_power as u8);
    }

    /// Update radio configuration and re-apply to hardware.
    pub fn update_config(&mut self, update_fn: impl FnOnce(&mut RadioConfig)) {
        update_fn(&mut self.config);
        self.apply_config();
    }

    /// Transmit a raw 802.15.4 frame (async). Waits for TX completion interrupt.
    ///
    /// The frame should include the full MAC header and payload. The radio
    /// hardware appends the FCS (CRC-16) automatically.
    pub async fn transmit(&mut self, data: &[u8]) -> Result<(), RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }
        if data.is_empty() || data.len() > MAX_FRAME_LEN {
            return Err(RadioError::InvalidFrame);
        }

        TX_SIGNAL.reset();

        // Build the DMA TX packet directly from the payload
        tx_ready(data.as_ptr(), data.len() as u8);

        // Switch to TX state and start transmission
        tx_trigger();

        log::trace!("telink: tx {} bytes", data.len());

        // Wait for TX completion from rf_tx_irq_handler
        let success = TX_SIGNAL.wait().await;

        if success {
            Ok(())
        } else {
            Err(RadioError::TxAborted)
        }
    }

    /// Receive the next 802.15.4 frame (async). Waits for RX interrupt.
    ///
    /// The radio must be in RX state (the default after `init()`). Returns
    /// the received frame data with RSSI, LQI, and timestamp. Frames failing
    /// CRC are rejected.
    pub async fn receive(&mut self) -> Result<ReceivedFrame, RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }

        RX_SIGNAL.reset();

        // Ensure radio is in RX state
        set_trx_state(RF_STATE_RX);

        // Wait for frame reception from rf_rx_irq_handler
        RX_SIGNAL.wait().await;

        // Check CRC
        if RX_CRC_FAIL.load(Ordering::Acquire) {
            return Err(RadioError::CrcError);
        }

        // Read frame metadata from atomics (written by ISR before signaling)
        let len = RX_FRAME_LEN.load(Ordering::Acquire) as usize;
        let rssi = RX_FRAME_RSSI.load(Ordering::Acquire);
        let timestamp = RX_FRAME_TIMESTAMP.load(Ordering::Acquire);
        let lqi = get_lqi(rssi);

        let mut frame = ReceivedFrame {
            data: [0u8; 128],
            len,
            rssi,
            lqi,
            timestamp,
        };

        // SAFETY: ISR has finished writing to RX_FRAME_BUF before signaling RX_SIGNAL
        unsafe {
            let buf = &*RX_FRAME_BUF.get();
            frame.data[..len].copy_from_slice(&buf[..len]);
        }

        log::trace!(
            "telink: rx {} bytes rssi={}dBm lqi={} ts={}",
            len,
            rssi,
            lqi,
            timestamp,
        );
        Ok(frame)
    }

    /// Perform a Clear Channel Assessment on the current channel.
    ///
    /// Returns `true` if the channel is idle, `false` if busy.
    pub fn cca(&self) -> Result<bool, RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }

        let result = perform_cca();
        Ok(result == PHY_CCA_IDLE)
    }

    /// Perform an Energy Detection scan.
    ///
    /// Starts the ED scan, waits briefly for samples to accumulate, then
    /// stops and returns the ED value (0–255, derived from averaged RSSI
    /// converted to LQI by the SDK).
    pub fn energy_detect(&self) -> Result<u8, RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }

        start_ed_scan();
        // IEEE 802.15.4 ED measurement duration = 8 symbol periods = 128µs.
        // Use a calibrated busy-wait (~128µs at typical Telink clock rates).
        // Each iteration takes ~4 cycles; at 32 MHz → ~4000 iterations ≈ 500µs
        // (overshoot is fine — gives better RSSI averaging).
        for _ in 0..4000 {
            core::hint::spin_loop();
        }
        let ed = stop_ed_scan();
        Ok(ed)
    }

    /// Switch radio to RX state.
    pub fn enable_rx(&self) {
        set_trx_state(RF_STATE_RX);
    }

    /// Switch radio off.
    pub fn disable_rx(&self) {
        set_trx_state(RF_STATE_OFF);
    }

    /// Power down the radio to save battery between poll cycles.
    ///
    /// Disables RF, DMA channels, and RF IRQs. Saves ~5-8 mA.
    /// Call `radio_wake()` before next TX/RX.
    pub fn radio_sleep(&self) {
        // Disable radio
        set_trx_state(RF_STATE_OFF);
        // Disable RF DMA channels
        let dma_en = reg_read_u8(REG_DMA_CHN_EN);
        reg_write_u8(REG_DMA_CHN_EN, dma_en & !(DMA_CHN_RF_RX | DMA_CHN_RF_TX));
        // Disable RF IRQs
        reg_write_u16(REG_RF_IRQ_MASK, 0);
    }

    /// Re-enable the radio after `radio_sleep()`.
    ///
    /// Restores DMA channels, IRQ mask, and re-applies channel.
    pub fn radio_wake(&mut self) {
        // Re-enable RF DMA channels
        let dma_en = reg_read_u8(REG_DMA_CHN_EN);
        reg_write_u8(REG_DMA_CHN_EN, dma_en | DMA_CHN_RF_RX | DMA_CHN_RF_TX);
        // Re-enable TX+RX IRQs
        reg_write_u16(REG_RF_IRQ_MASK, FLD_RF_IRQ_TX | FLD_RF_IRQ_RX);
        // Re-apply channel
        set_channel(self.config.channel);
        // Set RX buffer
        set_rx_buf(core::ptr::addr_of!(RF_RX_DMA_BUF) as *mut u8);
        // Back to RX mode
        set_trx_state(RF_STATE_RX);
    }

    /// Enter CPU suspend mode for `duration_ms` milliseconds.
    ///
    /// The CPU halts, SRAM is retained, and the system timer continues
    /// running. The CPU resumes execution after the timer fires.
    /// Radio must be powered down before calling this.
    ///
    /// Current draw in suspend: ~3 µA (vs ~1.5 mA in WFI idle).
    ///
    /// Unlike PHY6222's system sleep (which reboots), TLSR8258 suspend
    /// resumes execution right after the sleep trigger write.
    pub fn cpu_suspend_ms(duration_ms: u32) {
        // Read current system timer (32-bit, 16 MHz)
        let now = reg_read_u32(REG_SYSTEM_TIMER);
        let wake_tick = now.wrapping_add(duration_ms * SYSTEM_TIMER_TICKS_PER_US * 1000);

        // Set wake-up tick
        reg_write_u32(REG_SYSTEM_WAKEUP_TICK, wake_tick);

        // Enable timer as wake source
        reg_write_u8(REG_WAKEUP_EN, PM_WAKEUP_TIMER);

        // Enter suspend — CPU halts, resumes when timer fires
        let ctrl = reg_read_u8(REG_PWDN_CTRL);
        reg_write_u8(REG_PWDN_CTRL, ctrl | FLD_PWDN_CTRL_SLEEP);

        // Execution resumes here after wake-up
        // Small delay for clock stabilization
        for _ in 0..1000u32 {
            core::hint::spin_loop();
        }
    }
}

// ── IRQ handler implementations ─────────────────────────────────
// These are called from the platform RF ISR. They copy received data
// from the radio DMA buffer and signal the async waiting tasks.

/// RX interrupt handler — called from the platform RF ISR when a frame
/// has been received into the DMA buffer.
///
/// Extracts the payload, RSSI, and timestamp from the Telink DMA packet
/// format and signals `RX_SIGNAL` to wake the async `receive()` call.
#[unsafe(no_mangle)]
pub extern "C" fn rf_rx_irq_handler() {
    // Read the DMA RX buffer pointer
    let p = core::ptr::addr_of!(RF_RX_DMA_BUF) as *const u8;

    // Parse the rf_recvPkt_t header:
    //   [0..3]  dmaLen (u32 LE)
    //   [4]     rssi
    //   [5..11] reserved (7 bytes)
    //   [12]    payloadLen (802.15.4 PSDU length including FCS)
    //   [13..]  payload
    let payload_len = unsafe { *p.add(12) } as usize;

    // Sanity check — reject obviously invalid lengths
    if !(3..=MAX_FRAME_LEN).contains(&payload_len) {
        RX_CRC_FAIL.store(true, Ordering::Release);
        RX_FRAME_LEN.store(0, Ordering::Release);
        RX_SIGNAL.signal(true);
        return;
    }

    // RSSI is at offset 4, needs -110 offset for Telink chips to get dBm
    let rssi_raw = unsafe { *p.add(4) } as i8;
    let rssi_dbm = (rssi_raw as i16 - 110) as i8;

    // Use a monotonic counter as a timestamp proxy — the actual hardware
    // timer location varies by chip family (TLSR8258 vs B91) and isn't
    // available portably. This provides frame ordering at minimum.
    // Monotonic counter — safe to do load+store since we're in ISR context
    let count = RX_COUNTER.load(Ordering::Relaxed);
    RX_COUNTER.store(count.wrapping_add(1), Ordering::Relaxed);
    let timestamp = count as u32;

    // The PSDU length from the radio includes the 2-byte FCS.
    // Strip it here so upper layers receive only the MAC frame.
    let frame_len = if payload_len >= 2 {
        (payload_len - 2).min(MAX_FRAME_LEN)
    } else {
        0
    };

    RX_CRC_FAIL.store(false, Ordering::Release);
    RX_FRAME_LEN.store(frame_len as u8, Ordering::Release);
    RX_FRAME_RSSI.store(rssi_dbm, Ordering::Release);
    RX_FRAME_TIMESTAMP.store(timestamp, Ordering::Release);

    // Copy payload from DMA buffer into our frame buffer
    // SAFETY: DMA buffer is valid and we're in ISR context (radio is paused)
    unsafe {
        let payload_ptr = p.add(ZB_RADIO_RX_HDR_LEN);
        let buf = &mut *RX_FRAME_BUF.get();
        core::ptr::copy_nonoverlapping(payload_ptr, buf.as_mut_ptr(), frame_len);
    }

    // Re-arm the RX buffer for the next packet
    let rx_buf_ptr = core::ptr::addr_of_mut!(RF_RX_DMA_BUF) as *mut u8;
    set_rx_buf(rx_buf_ptr);

    RX_SIGNAL.signal(true);
}

/// TX interrupt handler — called from the platform RF ISR when frame
/// transmission is complete.
///
/// Signals `TX_SIGNAL` to wake the async `transmit()` call and switches
/// the radio back to RX state.
#[unsafe(no_mangle)]
pub extern "C" fn rf_tx_irq_handler() {
    // Switch back to RX mode after transmission
    set_trx_state(RF_STATE_RX);

    TX_SIGNAL.signal(true);
}
