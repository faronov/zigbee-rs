//! Low-level Telink 802.15.4 radio driver via FFI to the tl_zigbee_sdk PHY layer.
//!
//! Provides async TX/RX on top of Telink's IEEE 802.15.4 radio peripheral
//! (TLSR825x and B91 families) using Embassy signals for interrupt-driven
//! completion notification.
//!
//! Telink chips (TLSR8258, TLSR8278, B91/TLSR9518) have a built-in 2.4 GHz
//! multi-protocol radio supporting BLE 5.x and IEEE 802.15.4. Radio access is
//! provided through the `tl_zigbee_sdk` C library. This driver creates Rust FFI
//! bindings to that library's MAC PHY layer and implements the IRQ handler
//! callbacks to bridge into Embassy async signals.
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
//!   ├── FFI calls → tl_zigbee_sdk MAC PHY (Telink C library)
//!   │     ├── rf_setChannel / rf_setTxPower / rf_setTrxState / ...
//!   │     ├── rf802154_tx_ready + rf802154_tx / rf_setRxBuf
//!   │     └── rf_performCCA / rf_startEDScan / rf_stopEDScan / rf_getLqi
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
//! The downstream firmware crate must link the Telink Zigbee SDK libraries.
//! The RF interrupt must be registered in platform startup code so that
//! `rf_rx_irq_handler` and `rf_tx_irq_handler` are called from the ISR.

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

// ── Radio states (from mac_phy.h) ──────────────────────────────

const RF_STATE_RX: u8 = 1;
const RF_STATE_OFF: u8 = 3;

// ── CCA result codes (from mac_phy.h) ──────────────────────────

const PHY_CCA_IDLE: u8 = 0x04;
#[allow(dead_code)]
const PHY_CCA_BUSY: u8 = 0x00;

// ── FFI bindings to Telink tl_zigbee_sdk MAC PHY layer ─────────
// These map to functions in the Telink Zigbee SDK. The SDK is compiled
// and linked by the downstream firmware crate.

unsafe extern "C" {
    // ── Initialization ──
    fn rf_init();
    fn mac_trxInit();

    // ── Configuration ──
    fn rf_setChannel(chn: u8);
    fn rf_setTxPower(power: u8);
    fn rf_setRxBuf(buf: *mut u8);
    fn rf_setTrxState(state: u8);

    // ── TX ──
    fn rf802154_tx_ready(buf: *mut u8, len: u8);
    fn rf802154_tx();

    // ── CCA / Energy Detection ──
    fn rf_performCCA() -> u8;
    fn rf_startEDScan();
    fn rf_stopEDScan() -> u8;

    // ── LQI ──
    fn rf_getLqi(rssi: i8) -> u8;
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

// DMA TX buffer used by rf802154_tx_ready (must be static for DMA access).
// The SDK's rf802154_tx_ready writes into its own static `rf_tx_buf`, so we
// only need a staging buffer to pass payload data to it.
static mut TX_PAYLOAD_BUF: [u8; MAX_FRAME_LEN] = [0u8; MAX_FRAME_LEN];

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

/// Async driver for the Telink TLSR825x / B91 IEEE 802.15.4 radio.
///
/// Uses FFI calls to Telink's `tl_zigbee_sdk` MAC PHY layer for hardware
/// access, with Embassy signals for interrupt-driven async TX/RX.
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
        unsafe {
            rf_init();
            mac_trxInit();

            // Point the radio DMA engine at our static RX buffer
            let rx_buf_ptr = core::ptr::addr_of_mut!(RF_RX_DMA_BUF) as *mut u8;
            rf_setRxBuf(rx_buf_ptr);

            // Apply initial config
            rf_setChannel(self.config.channel);
            rf_setTxPower(self.config.tx_power as u8);

            // Start in RX state
            rf_setTrxState(RF_STATE_RX);
        }

        self.initialized = true;
        log::info!("telink: radio initialized (rf_init + mac_trxInit)");
    }

    /// Apply the current configuration to the radio hardware.
    fn apply_config(&mut self) {
        if !self.initialized {
            return;
        }
        unsafe {
            rf_setChannel(self.config.channel);
            rf_setTxPower(self.config.tx_power as u8);
        }
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

        unsafe {
            // Copy payload into our static staging buffer for the C SDK to read
            let tx_buf_ptr = core::ptr::addr_of_mut!(TX_PAYLOAD_BUF) as *mut u8;
            core::ptr::copy_nonoverlapping(data.as_ptr(), tx_buf_ptr, data.len());

            // rf802154_tx_ready builds the DMA header in the SDK's internal
            // rf_tx_buf and copies the payload after it.
            rf802154_tx_ready(tx_buf_ptr, data.len() as u8);

            // Switch to TX state and start transmission
            rf802154_tx();
        }

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
        unsafe { rf_setTrxState(RF_STATE_RX) };

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
        let lqi = unsafe { rf_getLqi(rssi) };

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

        let result = unsafe { rf_performCCA() };
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

        unsafe {
            rf_startEDScan();
            // The SDK accumulates RSSI samples via a poll callback.
            // In our bare-metal context we do a brief busy-wait to let
            // the hardware gather a few samples.
            for _ in 0..1000 {
                core::hint::spin_loop();
            }
            let ed = rf_stopEDScan();
            Ok(ed)
        }
    }

    /// Switch radio to RX state.
    pub fn enable_rx(&self) {
        unsafe { rf_setTrxState(RF_STATE_RX) };
    }

    /// Switch radio off.
    pub fn disable_rx(&self) {
        unsafe { rf_setTrxState(RF_STATE_OFF) };
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
    if payload_len < 3 || payload_len > MAX_FRAME_LEN {
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
    unsafe {
        let rx_buf_ptr = core::ptr::addr_of_mut!(RF_RX_DMA_BUF) as *mut u8;
        rf_setRxBuf(rx_buf_ptr);
    }

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
    unsafe { rf_setTrxState(RF_STATE_RX) };

    TX_SIGNAL.signal(true);
}
