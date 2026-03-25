//! Low-level BL702 802.15.4 radio driver via FFI to Bouffalo's `lmac154` library.
//!
//! Provides async TX/RX on top of the BL702's IEEE 802.15.4 radio peripheral
//! using Embassy signals for interrupt-driven completion notification.
//!
//! The BL702 (Bouffalo Lab, RISC-V) has a built-in multi-protocol radio
//! supporting BLE 5.0 and IEEE 802.15.4. The 802.15.4 radio registers are
//! not publicly documented — Bouffalo provides radio access through the
//! `lmac154` C library (`liblmac154.a`). This driver creates Rust FFI
//! bindings to that library and implements the callback functions to bridge
//! into Embassy async signals.
//!
//! # Radio peripheral overview
//! - 2.4 GHz IEEE 802.15.4 compliant
//! - Hardware CRC generation and checking
//! - Configurable TX power: 0 dBm to +14 dBm
//! - RSSI / LQI measurement
//! - Hardware auto-ACK with configurable retransmission
//! - Hardware address filtering (PAN ID, short addr, long addr)
//! - CSMA-CA support
//! - AES-128 CCM hardware acceleration
//!
//! # Architecture
//! ```text
//! Bl702Driver (Rust, async)
//!   ├── FFI calls → liblmac154.a (Bouffalo C library)
//!   │     ├── lmac154_setChannel / setPanId / setShortAddr / ...
//!   │     ├── lmac154_triggerTx / enableRx / readRxData
//!   │     └── lmac154_runCCA / getRSSI / getLQI
//!   ├── TX completion: lmac154_txDoneEvent callback → TX_SIGNAL
//!   └── RX completion: lmac154_rxDoneEvent callback → RX_SIGNAL
//! ```
//!
//! # Build requirements
//! The downstream firmware crate must link `liblmac154.a` from Bouffalo's SDK.
//! Add to your `build.rs`:
//! ```rust,ignore
//! println!("cargo:rustc-link-search=path/to/bl_iot_sdk/components/network/lmac154/lib");
//! println!("cargo:rustc-link-lib=static=lmac154");
//! ```
//!
//! The M154 interrupt must be registered in the firmware startup:
//! ```rust,ignore
//! // After lmac154 init, register the interrupt handler
//! bl_irq_register(M154_IRQn, lmac154_getInterruptHandler());
//! bl_irq_enable(M154_IRQn);
//! ```

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

// ── FFI bindings to Bouffalo lmac154 C library ──────────────────
// These map to functions in liblmac154.a. The library is provided as a
// pre-compiled static archive by Bouffalo Lab and must be linked by the
// downstream firmware crate.

/// Channel indices used by lmac154 (0 = channel 11, 15 = channel 26).
#[repr(u32)]
#[derive(Debug, Clone, Copy)]
pub enum Lmac154Channel {
    Channel11 = 0,
    Channel12 = 1,
    Channel13 = 2,
    Channel14 = 3,
    Channel15 = 4,
    Channel16 = 5,
    Channel17 = 6,
    Channel18 = 7,
    Channel19 = 8,
    Channel20 = 9,
    Channel21 = 10,
    Channel22 = 11,
    Channel23 = 12,
    Channel24 = 13,
    Channel25 = 14,
    Channel26 = 15,
}

impl Lmac154Channel {
    /// Convert 802.15.4 channel number (11–26) to lmac154 enum.
    pub fn from_channel(ch: u8) -> Self {
        match ch {
            11 => Self::Channel11,
            12 => Self::Channel12,
            13 => Self::Channel13,
            14 => Self::Channel14,
            15 => Self::Channel15,
            16 => Self::Channel16,
            17 => Self::Channel17,
            18 => Self::Channel18,
            19 => Self::Channel19,
            20 => Self::Channel20,
            21 => Self::Channel21,
            22 => Self::Channel22,
            23 => Self::Channel23,
            24 => Self::Channel24,
            25 => Self::Channel25,
            26 => Self::Channel26,
            _ => Self::Channel11,
        }
    }
}

/// TX power levels supported by lmac154 (0–14 dBm).
#[repr(u32)]
#[derive(Debug, Clone, Copy)]
pub enum Lmac154TxPower {
    P0dBm = 0,
    P1dBm = 1,
    P2dBm = 2,
    P3dBm = 3,
    P4dBm = 4,
    P5dBm = 5,
    P6dBm = 6,
    P7dBm = 7,
    P8dBm = 8,
    P9dBm = 9,
    P10dBm = 10,
    P11dBm = 11,
    P12dBm = 12,
    P13dBm = 13,
    P14dBm = 14,
}

impl Lmac154TxPower {
    /// Convert dBm value to lmac154 TX power enum. Clamps to 0–14 range.
    pub fn from_dbm(dbm: i8) -> Self {
        match dbm.clamp(0, 14) as u8 {
            0 => Self::P0dBm,
            1 => Self::P1dBm,
            2 => Self::P2dBm,
            3 => Self::P3dBm,
            4 => Self::P4dBm,
            5 => Self::P5dBm,
            6 => Self::P6dBm,
            7 => Self::P7dBm,
            8 => Self::P8dBm,
            9 => Self::P9dBm,
            10 => Self::P10dBm,
            11 => Self::P11dBm,
            12 => Self::P12dBm,
            13 => Self::P13dBm,
            _ => Self::P14dBm,
        }
    }
}

/// TX completion status from lmac154.
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Lmac154TxStatus {
    TxFinished = 0,
    CsmaFailed = 1,
    TxAborted = 2,
    HwError = 3,
}

/// Frame type flags for hardware filtering.
#[repr(u8)]
pub enum Lmac154FrameType {
    Beacon = 0x01,
    Data = 0x02,
    Ack = 0x04,
    Command = 0x08,
}

extern "C" {
    // ── Initialization ──
    fn lmac154_init();
    fn lmac154_getInterruptHandler() -> Option<unsafe extern "C" fn()>;

    // ── Configuration ──
    fn lmac154_setChannel(ch_ind: Lmac154Channel);
    fn lmac154_getChannel() -> Lmac154Channel;
    fn lmac154_setPanId(pid: u16);
    fn lmac154_getPanId() -> u16;
    fn lmac154_setShortAddr(sadr: u16);
    fn lmac154_getShortAddr() -> u16;
    fn lmac154_setLongAddr(ladr: *const u8);
    fn lmac154_getLongAddr(ladr: *mut u8);
    fn lmac154_setTxPower(power: Lmac154TxPower);

    // ── TX ──
    fn lmac154_triggerTx(data_ptr: *const u8, length: u8, csma: u8);
    fn lmac154_setTxRetry(num: u32);
    fn lmac154_resetTx();

    // ── RX ──
    fn lmac154_enableRx();
    fn lmac154_disableRx();
    fn lmac154_getRxLength() -> u8;
    fn lmac154_readRxData(buf: *mut u8, offset: u8, len: u8);

    // ── Promiscuous mode ──
    fn lmac154_enableRxPromiscuousMode(enhanced_mode: u8, ignore_mpdu: u8);
    fn lmac154_disableRxPromiscuousMode();

    // ── Frame filtering ──
    fn lmac154_enableFrameTypeFiltering(frame_types: u8);
    fn lmac154_disableFrameTypeFiltering();

    // ── CCA / Energy Detection ──
    fn lmac154_runCCA(rssi: *mut i32) -> u8;

    // ── Status ──
    fn lmac154_getRSSI() -> i32;
    fn lmac154_getLQI() -> u8;

    // ── Auto-ACK ──
    fn lmac154_enableHwAutoTxAck();
    fn lmac154_disableHwAutoTxAck();

    // ── Coexistence ──
    fn lmac154_enableCoex();
}

// ── Async completion signals ────────────────────────────────────
// Set from lmac154 callback functions (ISR context), awaited by driver.

static TX_SIGNAL: Signal<CriticalSectionRawMutex, Lmac154TxStatus> = Signal::new();
static RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// RX frame buffer — filled by rxDoneEvent callback, read by receive().
// Protected by the single-consumer pattern: ISR writes before signaling,
// async code reads after awaiting the signal, so no concurrent access.
struct SyncUnsafeCell<T>(core::cell::UnsafeCell<T>);

// SAFETY: RX_BUF is only written in the ISR callback and read after
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

static RX_BUF: SyncUnsafeCell<[u8; MAX_FRAME_LEN]> = SyncUnsafeCell::new([0u8; MAX_FRAME_LEN]);
static RX_LEN: AtomicU8 = AtomicU8::new(0);
static RX_CRC_FAIL: AtomicBool = AtomicBool::new(false);

// RSSI from last received frame
static RX_RSSI: AtomicI32 = AtomicI32::new(-127);

/// Maximum 802.15.4 PHY frame size (including FCS).
const MAX_FRAME_LEN: usize = 127;

/// Received frame data (copied out of radio buffer in ISR).
pub struct RxFrame {
    pub data: [u8; MAX_FRAME_LEN],
    pub len: usize,
    pub lqi: u8,
    pub rssi: i8,
}

/// BL702 radio configuration.
#[derive(Debug, Clone, Copy)]
pub struct RadioConfig {
    /// 802.15.4 channel (11–26)
    pub channel: u8,
    /// PAN ID for hardware address filtering
    pub pan_id: u16,
    /// Short address for hardware address filtering
    pub short_address: u16,
    /// Extended (IEEE) address
    pub extended_address: [u8; 8],
    /// Transmit power in dBm (0 to +14)
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

/// Async wrapper around the BL702 IEEE 802.15.4 radio peripheral.
///
/// Uses FFI calls to Bouffalo's `lmac154` C library for hardware access,
/// with Embassy signals for interrupt-driven async TX/RX.
pub struct Bl702Driver {
    config: RadioConfig,
    initialized: bool,
}

/// Radio driver error.
#[derive(Debug, Clone, Copy)]
pub enum RadioError {
    /// CCA (Clear Channel Assessment) failure — channel is busy
    CcaFailure,
    /// TX was aborted by the hardware
    TxAborted,
    /// Radio hardware error during TX
    HardwareError,
    /// Invalid frame (too long or too short)
    InvalidFrame,
    /// Received frame failed CRC check
    CrcError,
    /// Radio not initialized
    NotInitialized,
}

impl Bl702Driver {
    /// Create a new BL702 radio driver with the given configuration.
    ///
    /// This initializes the lmac154 library, configures the radio, and
    /// enables BLE/Zigbee coexistence.
    ///
    /// # Safety
    /// The caller must ensure that:
    /// - The radio peripheral clocks are enabled (via `bl702-hal` or GLB registers)
    /// - The M154 interrupt is registered after this call:
    ///   ```rust,ignore
    ///   let handler = lmac154_getInterruptHandler();
    ///   bl_irq_register(M154_IRQn, handler);
    ///   bl_irq_enable(M154_IRQn);
    ///   ```
    pub fn new(config: RadioConfig) -> Self {
        let mut driver = Self {
            config,
            initialized: false,
        };
        driver.init_hardware();
        driver
    }

    /// Initialize the lmac154 library and configure the radio.
    fn init_hardware(&mut self) {
        unsafe {
            // Initialize the lmac154 library (configures radio hardware)
            lmac154_init();

            // Enable BLE/Zigbee coexistence (BL702 shares the radio)
            lmac154_enableCoex();

            // Enable hardware auto-ACK for received frames requesting it
            lmac154_enableHwAutoTxAck();

            // Accept standard 802.15.4 frame types
            lmac154_enableFrameTypeFiltering(
                Lmac154FrameType::Beacon as u8
                    | Lmac154FrameType::Data as u8
                    | Lmac154FrameType::Ack as u8
                    | Lmac154FrameType::Command as u8,
            );
        }

        self.initialized = true;
        self.apply_config();

        log::info!("bl702: lmac154 initialized, radio configured");
    }

    /// Apply the current configuration to radio hardware via lmac154 FFI.
    fn apply_config(&mut self) {
        if !self.initialized {
            return;
        }

        unsafe {
            // Set 802.15.4 channel (11–26)
            lmac154_setChannel(Lmac154Channel::from_channel(self.config.channel));

            // Set PAN ID for hardware address filtering
            lmac154_setPanId(self.config.pan_id);

            // Set short address for hardware address filtering
            lmac154_setShortAddr(self.config.short_address);

            // Set extended (IEEE) address
            lmac154_setLongAddr(self.config.extended_address.as_ptr());

            // Set TX power (0–14 dBm)
            lmac154_setTxPower(Lmac154TxPower::from_dbm(self.config.tx_power));

            // Set retransmission count for frames requesting ACK
            lmac154_setTxRetry(3);

            // Promiscuous mode
            if self.config.promiscuous {
                lmac154_enableRxPromiscuousMode(0, 0);
            } else {
                lmac154_disableRxPromiscuousMode();
            }
        }

        log::debug!(
            "bl702: config applied ch={} pan=0x{:04X} addr=0x{:04X} tx_pwr={}dBm promisc={}",
            self.config.channel,
            self.config.pan_id,
            self.config.short_address,
            self.config.tx_power,
            self.config.promiscuous,
        );
    }

    /// Update radio configuration and re-apply to hardware.
    pub fn update_config(&mut self, update_fn: impl FnOnce(&mut RadioConfig)) {
        update_fn(&mut self.config);
        self.apply_config();
    }

    /// Enable the receiver. Frames matching the address filter will trigger
    /// `lmac154_rxDoneEvent` which signals `RX_SIGNAL`.
    pub fn enable_rx(&self) {
        unsafe { lmac154_enableRx() };
    }

    /// Disable the receiver.
    pub fn disable_rx(&self) {
        unsafe { lmac154_disableRx() };
    }

    /// Transmit a raw 802.15.4 frame (async). Waits for TX completion interrupt.
    ///
    /// The frame should include the full MAC header and payload. The radio
    /// hardware appends the FCS (CRC-16) automatically.
    ///
    /// Uses CSMA-CA for channel access.
    pub async fn transmit(&mut self, frame: &[u8]) -> Result<(), RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }
        if frame.is_empty() || frame.len() > MAX_FRAME_LEN {
            return Err(RadioError::InvalidFrame);
        }

        TX_SIGNAL.reset();

        // Trigger transmission with CSMA-CA enabled
        unsafe {
            lmac154_triggerTx(frame.as_ptr(), frame.len() as u8, 1);
        }
        log::trace!("bl702: tx {} bytes", frame.len());

        // Wait for TX completion from lmac154_txDoneEvent callback
        let status = TX_SIGNAL.wait().await;

        match status {
            Lmac154TxStatus::TxFinished => Ok(()),
            Lmac154TxStatus::CsmaFailed => Err(RadioError::CcaFailure),
            Lmac154TxStatus::TxAborted => Err(RadioError::TxAborted),
            Lmac154TxStatus::HwError => Err(RadioError::HardwareError),
        }
    }

    /// Receive the next 802.15.4 frame (async). Waits for RX interrupt.
    ///
    /// The receiver must be enabled (call `enable_rx()` first). Returns
    /// the received frame data with LQI and RSSI. Frames failing CRC
    /// are rejected.
    pub async fn receive(&mut self) -> Result<RxFrame, RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }

        RX_SIGNAL.reset();

        // Ensure receiver is enabled
        self.enable_rx();

        // Wait for frame reception from lmac154_rxDoneEvent callback
        RX_SIGNAL.wait().await;

        // Check CRC
        if RX_CRC_FAIL.load(Ordering::Acquire) {
            return Err(RadioError::CrcError);
        }

        // Read frame data from the atomic buffer (written by ISR callback)
        let len = RX_LEN.load(Ordering::Acquire) as usize;
        let rssi = RX_RSSI.load(Ordering::Acquire) as i8;

        let mut frame = RxFrame {
            data: [0u8; MAX_FRAME_LEN],
            len,
            lqi: unsafe { lmac154_getLQI() },
            rssi,
        };

        // SAFETY: ISR has finished writing to RX_BUF before signaling RX_SIGNAL
        unsafe {
            let buf = &*RX_BUF.get();
            frame.data[..len].copy_from_slice(&buf[..len]);
        }

        log::trace!("bl702: rx {} bytes rssi={}dBm lqi={}", len, rssi, frame.lqi);
        Ok(frame)
    }

    /// Perform an Energy Detection (CCA) measurement on the current channel.
    ///
    /// Returns the RSSI in dBm and whether the channel is busy.
    pub fn energy_detect(&self) -> Result<(i8, bool), RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }

        let mut rssi: i32 = 0;
        let busy = unsafe { lmac154_runCCA(&mut rssi as *mut i32) };

        Ok((rssi as i8, busy != 0))
    }

    /// Get the interrupt handler function pointer for M154_IRQn registration.
    ///
    /// Must be called after `new()` and registered with the interrupt controller:
    /// ```rust,ignore
    /// let handler = driver.get_interrupt_handler();
    /// if let Some(h) = handler {
    ///     bl_irq_register(M154_IRQn, h);
    ///     bl_irq_enable(M154_IRQn);
    /// }
    /// ```
    pub fn get_interrupt_handler(&self) -> Option<unsafe extern "C" fn()> {
        unsafe { lmac154_getInterruptHandler() }
    }
}

// ── lmac154 callback implementations ────────────────────────────
// These are weak symbols in liblmac154.a. Our implementations override
// them and bridge into Embassy async signals.

/// Called from lmac154 ISR when frame transmission completes.
#[no_mangle]
extern "C" fn lmac154_txDoneEvent(tx_status: Lmac154TxStatus) {
    TX_SIGNAL.signal(tx_status);
}

/// Called from lmac154 ISR when a frame has been received.
///
/// Copies the frame data from the radio buffer into our static RX_BUF
/// and signals the async receiver.
#[no_mangle]
extern "C" fn lmac154_rxDoneEvent(rx_buf: *const u8, rx_len: u8, crc_fail: u8) {
    // Guard against null pointer from C library (e.g., on error conditions)
    if rx_buf.is_null() {
        RX_CRC_FAIL.store(true, Ordering::Release);
        RX_LEN.store(0, Ordering::Release);
        RX_SIGNAL.signal(());
        return;
    }

    let len = rx_len.min(MAX_FRAME_LEN as u8) as usize;

    RX_CRC_FAIL.store(crc_fail != 0, Ordering::Release);
    RX_LEN.store(len as u8, Ordering::Release);

    // Read RSSI while still in ISR context (register is valid now)
    let rssi = unsafe { lmac154_getRSSI() };
    RX_RSSI.store(rssi, Ordering::Release);

    // Copy frame data from lmac154's buffer into our static buffer
    // SAFETY: rx_buf is valid for rx_len bytes during this callback
    unsafe {
        let buf = &mut *RX_BUF.get();
        core::ptr::copy_nonoverlapping(rx_buf, buf.as_mut_ptr(), len);
    }

    RX_SIGNAL.signal(());
}

/// Called from lmac154 ISR when an ACK is received (or not) after TX.
///
/// For frames with AR (Ack Request) bit set, this fires instead of /
/// after txDoneEvent. We currently handle TX completion in txDoneEvent
/// and log ACK status here.
#[no_mangle]
extern "C" fn lmac154_ackEvent(ack_received: u8, frame_pending: u8, seq_num: u8) {
    log::trace!(
        "bl702: ack_event received={} pending={} seq=0x{:02X}",
        ack_received,
        frame_pending,
        seq_num,
    );
}
