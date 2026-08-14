//! Pure-Rust IEEE 802.15.4 radio driver for Silicon Labs EFR32MG21.
//!
//! The EFR32MG21 is an ARM Cortex-M33 multi-protocol SoC (Series 2) with an
//! integrated 2.4 GHz radio supporting IEEE 802.15.4, BLE, and proprietary
//! protocols. This driver configures the radio entirely through memory-mapped
//! registers — **no RAIL library, no GSDK binary blobs required**.
//!
//! # Radio hardware blocks
//! - **RAC** (Radio Controller): top-level radio state machine, PA control
//! - **FRC** (Frame Controller): frame formatting, preamble, sync word, CRC
//! - **MODEM**: modulation/demodulation (O-QPSK 250kbps for 802.15.4)
//! - **SYNTH**: frequency synthesizer (PLL) for channel selection
//! - **AGC**: automatic gain control, RSSI measurement
//! - **BUFC**: buffer controller for TX/RX FIFOs
//! - **CRC**: hardware CRC engine (part of FRC block)
//!
//! # Architecture
//! ```text
//! Efr32s2Driver (pure Rust, async)
//!   ├── CMU registers (0x40008000)
//!   │     └── enable_clocks()    → enable RAC, FRC, MODEM, SYNTH, AGC, BUFC
//!   ├── RAC registers (0x40093000)
//!   │     ├── radio_init()       → configure for 802.15.4 mode
//!   │     ├── set_tx_power()     → PA power control
//!   │     └── radio_sleep/wake() → power management
//!   ├── FRC registers (0x40090000)
//!   │     └── configure frame format, CRC-16
//!   ├── MODEM registers (0x40094000)
//!   │     └── O-QPSK 250kbps configuration
//!   ├── SYNTH registers (0x40092000)
//!   │     └── set_channel()      → program PLL for channel frequency
//!   ├── AGC registers (0x40095000)
//!   │     └── energy_detect()    → RSSI measurement
//!   ├── BUFC registers (0x40091000)
//!   │     ├── transmit()         → load TX buffer, trigger TX
//!   │     └── receive()          → configure RX buffer
//!   └── IRQ → Embassy Signal for async TX/RX completion
//! ```
//!
//! # IMPORTANT — Scaffold Driver
//! This is a scaffold implementation. Register values for radio init are
//! simplified approximations. The exact register sequences for 802.15.4 mode
//! need to be verified against the EFR32xG21 Reference Manual or extracted
//! from the RAIL library source. All unverified values are marked with
//! `// TODO: verify against EFR32xG21 RM`.

use core::sync::atomic::{AtomicBool, AtomicI8, AtomicU8, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

// ── EFR32MG21 (Series 2) Peripheral Base Addresses ──────────────

/// Clock Management Unit — controls peripheral clock gating.
const CMU_BASE: u32 = 0x4000_8000;

/// Radio Controller — top-level radio state machine.
const RAC_BASE: u32 = 0x4009_3000;

/// Frame Controller — frame formatting, preamble, sync word, CRC.
const FRC_BASE: u32 = 0x4009_0000;

/// Modem — modulation/demodulation engine.
const MODEM_BASE: u32 = 0x4009_4000;

/// Frequency Synthesizer — PLL for channel selection.
const SYNTH_BASE: u32 = 0x4009_2000;

/// Automatic Gain Control — receiver gain and RSSI.
const AGC_BASE: u32 = 0x4009_5000;

/// Buffer Controller — TX/RX DMA-like FIFO management.
const BUFC_BASE: u32 = 0x4009_1000;

/// GPIO controller base.
const _GPIO_BASE: u32 = 0x4003_C000;

// ── CMU Register Offsets ────────────────────────────────────────

/// High-frequency peripheral clock enable register 0.
const CMU_HFPERCLKEN0: u32 = CMU_BASE + 0x044;
/// High-frequency radio clock enable register.
const CMU_HFRADIOCLKEN0: u32 = CMU_BASE + 0x074; // TODO: verify against EFR32xG21 RM

// ── CMU Clock Enable Bits ───────────────────────────────────────

const CMU_HFRADIOCLKEN0_RAC: u32 = 1 << 0; // TODO: verify against EFR32xG21 RM
const CMU_HFRADIOCLKEN0_FRC: u32 = 1 << 1; // TODO: verify against EFR32xG21 RM
const CMU_HFRADIOCLKEN0_MODEM: u32 = 1 << 2; // TODO: verify against EFR32xG21 RM
const CMU_HFRADIOCLKEN0_SYNTH: u32 = 1 << 3; // TODO: verify against EFR32xG21 RM
const CMU_HFRADIOCLKEN0_AGC: u32 = 1 << 4; // TODO: verify against EFR32xG21 RM
const CMU_HFRADIOCLKEN0_BUFC: u32 = 1 << 5; // TODO: verify against EFR32xG21 RM

// ── RAC Register Offsets ────────────────────────────────────────

/// RAC command register — triggers state transitions.
const RAC_CMD: u32 = RAC_BASE + 0x004;
/// RAC status register — current radio state.
const RAC_STATUS: u32 = RAC_BASE + 0x008;
/// RAC interrupt flag register.
const RAC_IF: u32 = RAC_BASE + 0x010;
/// RAC interrupt flag clear register.
const RAC_IFC: u32 = RAC_BASE + 0x018;
/// RAC interrupt enable register.
const RAC_IEN: u32 = RAC_BASE + 0x01C;
/// RAC PA power control register.
const RAC_PACTRL: u32 = RAC_BASE + 0x100; // TODO: verify against EFR32xG21 RM

// ── RAC Commands ────────────────────────────────────────────────

const RAC_CMD_TXEN: u32 = 1 << 0; // TODO: verify against EFR32xG21 RM
const RAC_CMD_RXEN: u32 = 1 << 1; // TODO: verify against EFR32xG21 RM
const RAC_CMD_TXDIS: u32 = 1 << 2; // TODO: verify against EFR32xG21 RM
const RAC_CMD_RXDIS: u32 = 1 << 3; // TODO: verify against EFR32xG21 RM

// ── RAC Status Bits ─────────────────────────────────────────────

const RAC_STATUS_STATE_MASK: u32 = 0x0F; // TODO: verify against EFR32xG21 RM
const _RAC_STATE_OFF: u32 = 0x00;
const _RAC_STATE_IDLE: u32 = 0x01;
const RAC_STATE_RX: u32 = 0x02;
const _RAC_STATE_TX: u32 = 0x03;

// ── RAC IRQ Bits ────────────────────────────────────────────────

const RAC_IF_TXDONE: u32 = 1 << 0; // TODO: verify against EFR32xG21 RM
const RAC_IF_RXDONE: u32 = 1 << 1; // TODO: verify against EFR32xG21 RM
const RAC_IF_RXOF: u32 = 1 << 2; // TODO: verify against EFR32xG21 RM

// ── FRC Register Offsets ────────────────────────────────────────

/// FRC control register — frame format configuration.
const FRC_CTRL: u32 = FRC_BASE + 0x000;
/// FRC CRC initialization value.
const FRC_CRCINIT: u32 = FRC_BASE + 0x020; // TODO: verify against EFR32xG21 RM
/// FRC CRC polynomial.
const FRC_CRCPOLY: u32 = FRC_BASE + 0x024; // TODO: verify against EFR32xG21 RM
/// FRC interrupt flag register.
const FRC_IF: u32 = FRC_BASE + 0x030; // TODO: verify against EFR32xG21 RM
/// FRC interrupt flag clear register.
const FRC_IFC: u32 = FRC_BASE + 0x038; // TODO: verify against EFR32xG21 RM

// ── MODEM Register Offsets ──────────────────────────────────────

/// MODEM control register 0 — modulation format.
const MODEM_CTRL0: u32 = MODEM_BASE + 0x000;
/// MODEM control register 1 — demodulation parameters.
const MODEM_CTRL1: u32 = MODEM_BASE + 0x004;
/// MODEM control register 2 — additional parameters.
const MODEM_CTRL2: u32 = MODEM_BASE + 0x008;

// ── SYNTH Register Offsets ──────────────────────────────────────

/// SYNTH channel frequency control register.
const SYNTH_FREQ: u32 = SYNTH_BASE + 0x004; // TODO: verify against EFR32xG21 RM
/// SYNTH channel spacing register.
const SYNTH_CHSP: u32 = SYNTH_BASE + 0x008; // TODO: verify against EFR32xG21 RM
/// SYNTH channel number register.
const SYNTH_CHNO: u32 = SYNTH_BASE + 0x00C; // TODO: verify against EFR32xG21 RM
/// SYNTH command register.
const SYNTH_CMD: u32 = SYNTH_BASE + 0x010; // TODO: verify against EFR32xG21 RM

// ── AGC Register Offsets ────────────────────────────────────────

/// AGC control register.
const AGC_CTRL0: u32 = AGC_BASE + 0x000;
/// AGC RSSI register — current received signal strength.
const AGC_RSSI: u32 = AGC_BASE + 0x020; // TODO: verify against EFR32xG21 RM

// ── BUFC Register Offsets ───────────────────────────────────────

/// BUFC TX buffer 0 data register.
const BUFC_BUF0_DATA: u32 = BUFC_BASE + 0x040; // TODO: verify against EFR32xG21 RM
/// BUFC TX buffer 0 write count.
const _BUFC_BUF0_WCNT: u32 = BUFC_BASE + 0x044; // TODO: verify against EFR32xG21 RM
/// BUFC RX buffer 1 data register.
const BUFC_BUF1_DATA: u32 = BUFC_BASE + 0x080; // TODO: verify against EFR32xG21 RM
/// BUFC RX buffer 1 read count.
const BUFC_BUF1_RCNT: u32 = BUFC_BASE + 0x084; // TODO: verify against EFR32xG21 RM
/// BUFC command register.
const BUFC_CMD: u32 = BUFC_BASE + 0x004; // TODO: verify against EFR32xG21 RM

// ── Register access helpers ─────────────────────────────────────

#[inline(always)]
fn reg_write(addr: u32, val: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) };
}

#[inline(always)]
fn reg_read(addr: u32) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

/// Read-modify-write: set bits in a register.
#[inline(always)]
fn reg_set_bits(addr: u32, bits: u32) {
    let old = reg_read(addr);
    reg_write(addr, old | bits);
}

/// Read-modify-write: clear bits in a register.
#[inline(always)]
fn reg_clear_bits(addr: u32, bits: u32) {
    let old = reg_read(addr);
    reg_write(addr, old & !bits);
}

// ── Async completion signals ────────────────────────────────────

/// TX completion signal (set from IRQ handler).
static TX_DONE: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// RX completion signal (set from IRQ handler).
static RX_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// Static RX buffer — written by IRQ handler, read after signal.
struct SyncUnsafeCell<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for SyncUnsafeCell<T> {}

impl<T> SyncUnsafeCell<T> {
    const fn new(value: T) -> Self {
        Self(core::cell::UnsafeCell::new(value))
    }
    fn get(&self) -> *mut T {
        self.0.get()
    }
}

/// Maximum IEEE 802.15.4 frame size (PHY payload = 127 bytes).
const MAX_FRAME_LEN: usize = 127;

static RX_BUF: SyncUnsafeCell<[u8; MAX_FRAME_LEN]> = SyncUnsafeCell::new([0u8; MAX_FRAME_LEN]);
static RX_LEN: AtomicU8 = AtomicU8::new(0);
static RX_CRC_OK: AtomicBool = AtomicBool::new(false);
static RX_RSSI: AtomicI8 = AtomicI8::new(-127);

// ── Public types ────────────────────────────────────────────────

/// Received 802.15.4 frame.
pub struct RxFrame {
    pub data: [u8; MAX_FRAME_LEN],
    pub len: usize,
    pub rssi: i8,
    pub lqi: u8,
}

/// Radio configuration.
#[derive(Debug, Clone, Copy)]
pub struct RadioConfig {
    /// IEEE 802.15.4 channel (11–26).
    pub channel: u8,
    /// PAN ID for address filtering.
    pub pan_id: u16,
    /// Short address for address filtering.
    pub short_address: u16,
    /// Extended (IEEE) 64-bit address.
    pub extended_address: [u8; 8],
    /// TX power in dBm (clamped to -20..+19).
    pub tx_power: i8,
    /// Accept all frames regardless of address.
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

/// Radio driver error.
#[derive(Debug, Clone, Copy)]
pub enum RadioError {
    /// Channel busy (CCA failed).
    CcaFailure,
    /// TX hardware error.
    HardwareError,
    /// Frame too long or too short.
    InvalidFrame,
    /// CRC check failed on received frame.
    CrcError,
    /// Driver not initialized.
    NotInitialized,
    /// RX timeout — no frame received.
    RxTimeout,
}

/// Pure-Rust async radio driver for EFR32MG21 IEEE 802.15.4 mode.
pub struct Efr32s2Driver {
    config: RadioConfig,
    initialized: bool,
}

impl Efr32s2Driver {
    /// Create and initialize a new EFR32 radio driver.
    ///
    /// Configures the radio for IEEE 802.15.4 (Zigbee) mode with the given
    /// settings. After this call the radio is ready for TX/RX.
    pub fn new(config: RadioConfig) -> Self {
        let mut drv = Self {
            config,
            initialized: false,
        };
        drv.init_hardware();
        drv
    }

    // ── Hardware initialization ─────────────────────────────────

    /// Full radio initialization: clocks → RAC → FRC → MODEM → SYNTH → AGC.
    fn init_hardware(&mut self) {
        self.enable_clocks();
        self.configure_rac();
        self.configure_frc();
        self.configure_modem();
        self.configure_synth();
        self.configure_agc();
        self.apply_config();

        self.initialized = true;
        log::info!("efr32s2: radio initialized in IEEE 802.15.4 mode");
    }

    /// Enable peripheral clocks for all radio blocks via CMU.
    fn enable_clocks(&self) {
        // Enable high-frequency peripheral clock (for GPIO, etc.)
        // The exact bits depend on EFR32xG1 revision.
        reg_set_bits(CMU_HFPERCLKEN0, 1 << 0); // TODO: verify against EFR32xG21 RM

        // Enable radio block clocks
        reg_set_bits(
            CMU_HFRADIOCLKEN0,
            CMU_HFRADIOCLKEN0_RAC
                | CMU_HFRADIOCLKEN0_FRC
                | CMU_HFRADIOCLKEN0_MODEM
                | CMU_HFRADIOCLKEN0_SYNTH
                | CMU_HFRADIOCLKEN0_AGC
                | CMU_HFRADIOCLKEN0_BUFC,
        );
    }

    /// Configure RAC (Radio Controller) for 802.15.4 operation.
    fn configure_rac(&self) {
        // Reset RAC to known state
        reg_write(RAC_CMD, RAC_CMD_TXDIS | RAC_CMD_RXDIS);

        // Wait for radio to reach idle state
        for _ in 0..10_000u32 {
            let state = reg_read(RAC_STATUS) & RAC_STATUS_STATE_MASK;
            if state <= 1 {
                break;
            }
            core::hint::spin_loop();
        }

        // Configure PA for 802.15.4: 2.4 GHz PA, output power
        // TODO: verify against EFR32xG21 RM — PA configuration is complex
        // and depends on the specific board layout and matching network.
        reg_write(RAC_PACTRL, 0x0000_0040); // TODO: verify against EFR32xG21 RM

        // Clear and enable relevant IRQs
        reg_write(RAC_IFC, RAC_IF_TXDONE | RAC_IF_RXDONE | RAC_IF_RXOF);
        reg_write(RAC_IEN, RAC_IF_TXDONE | RAC_IF_RXDONE);
    }

    /// Configure FRC (Frame Controller) for IEEE 802.15.4 frame format.
    fn configure_frc(&self) {
        // Set frame format: 802.15.4
        // - Preamble: 4 bytes of 0x00 (32 zero bits)
        // - SFD: 0xA7 (Start of Frame Delimiter)
        // - Length field: 1 byte
        // - PSDU: variable (max 127 bytes)
        // - FCS: 2 bytes (CRC-16/CCITT)
        reg_write(FRC_CTRL, 0x0000_0001); // TODO: verify against EFR32xG21 RM

        // CRC-16/CCITT polynomial: x^16 + x^12 + x^5 + 1
        // Initial value: 0x0000 (per IEEE 802.15.4)
        reg_write(FRC_CRCINIT, 0x0000_0000); // TODO: verify against EFR32xG21 RM
        reg_write(FRC_CRCPOLY, 0x0000_1021); // TODO: verify against EFR32xG21 RM
    }

    /// Configure MODEM for IEEE 802.15.4 O-QPSK modulation at 250 kbps.
    fn configure_modem(&self) {
        // 802.15.4 2.4 GHz PHY: O-QPSK with half-sine pulse shaping
        // - Chip rate: 2 Mchip/s
        // - Symbol rate: 62.5 ksym/s
        // - Bit rate: 250 kbps (4 bits per symbol)
        // - DSSS spreading: 32 chips per symbol

        // CTRL0: modulation format = O-QPSK, 802.15.4 mode
        reg_write(MODEM_CTRL0, 0x0000_0004); // TODO: verify against EFR32xG21 RM

        // CTRL1: demodulator parameters for 802.15.4
        reg_write(MODEM_CTRL1, 0x0000_0000); // TODO: verify against EFR32xG21 RM

        // CTRL2: additional demod settings
        reg_write(MODEM_CTRL2, 0x0000_0000); // TODO: verify against EFR32xG21 RM
    }

    /// Configure SYNTH (Frequency Synthesizer) for 2.4 GHz 802.15.4 channels.
    fn configure_synth(&self) {
        // Base frequency: 2405 MHz (channel 11)
        // Channel spacing: 5 MHz
        // Channels 11-26 → 2405-2480 MHz

        // Set base frequency (the exact encoding depends on the PLL divider config)
        // TODO: verify against EFR32xG21 RM — frequency word calculation
        reg_write(SYNTH_FREQ, 0x0000_0000); // TODO: verify against EFR32xG21 RM

        // Channel spacing: 5 MHz
        reg_write(SYNTH_CHSP, 0x0000_0000); // TODO: verify against EFR32xG21 RM
    }

    /// Configure AGC (Automatic Gain Control) for 802.15.4 reception.
    fn configure_agc(&self) {
        // Set AGC parameters for 802.15.4 signal characteristics
        // - Target RSSI level
        // - Gain step sizes
        // - Settling time
        reg_write(AGC_CTRL0, 0x0000_0000); // TODO: verify against EFR32xG21 RM
    }

    /// Set TX power via RAC PA control register.
    ///
    /// EFR32MG21 supports -20 dBm to +19 dBm output power.
    fn set_tx_power(&self, dbm: i8) {
        let power = dbm.clamp(-20, 19);
        // Map dBm to PA power register value
        // The exact mapping is non-linear and depends on the PA type.
        // For 2.4 GHz PA on EFR32MG21:
        //   -20 dBm → ~0, 0 dBm → ~64, +19 dBm → ~252
        let pa_val = ((power as i16 + 20) * 252 / 39).clamp(0, 252) as u32;
        let old = reg_read(RAC_PACTRL);
        reg_write(RAC_PACTRL, (old & 0xFFFF_FF00) | pa_val); // TODO: verify against EFR32xG21 RM
    }

    /// Set RF channel for IEEE 802.15.4.
    ///
    /// Channels 11–26 map to center frequencies 2405–2480 MHz (5 MHz spacing).
    fn set_channel(&self, channel: u8) {
        let ch = channel.clamp(11, 26);
        let ch_offset = (ch - 11) as u32;

        // Write channel number to SYNTH — the PLL locks to the new frequency.
        reg_write(SYNTH_CHNO, ch_offset);

        // Trigger synthesizer calibration for new channel
        reg_write(SYNTH_CMD, 0x0000_0001); // TODO: verify against EFR32xG21 RM

        // Wait for PLL lock (~50µs typical)
        for _ in 0..5_000u32 {
            core::hint::spin_loop();
        }
    }

    /// Apply current configuration to hardware.
    fn apply_config(&self) {
        self.set_channel(self.config.channel);
        self.set_tx_power(self.config.tx_power);
        // Address filtering is handled in software (same approach as PHY6222).
        // The FRC/RAC hardware can do address filtering, but software filtering
        // is more flexible and works reliably across register configurations.
    }

    /// Get current radio configuration.
    pub fn config(&self) -> &RadioConfig {
        &self.config
    }

    /// Update radio configuration.
    pub fn update_config(&mut self, update_fn: impl FnOnce(&mut RadioConfig)) {
        update_fn(&mut self.config);
        if self.initialized {
            self.apply_config();
        }
    }

    // ── TX / RX operations ──────────────────────────────────────

    /// Transmit an IEEE 802.15.4 frame (async).
    ///
    /// The frame should contain the full MAC header + payload. The radio
    /// hardware appends the 2-byte FCS (CRC-16) automatically.
    pub async fn transmit(&mut self, frame: &[u8]) -> Result<(), RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }
        if frame.is_empty() || frame.len() > MAX_FRAME_LEN {
            return Err(RadioError::InvalidFrame);
        }

        TX_DONE.reset();

        // Ensure radio is idle before TX
        reg_write(RAC_CMD, RAC_CMD_RXDIS);
        for _ in 0..1_000u32 {
            let state = reg_read(RAC_STATUS) & RAC_STATUS_STATE_MASK;
            if state != RAC_STATE_RX {
                break;
            }
            core::hint::spin_loop();
        }

        // Clear TX buffer
        reg_write(BUFC_CMD, 0x0000_0001); // TODO: verify against EFR32xG21 RM — reset TX FIFO

        // Write frame to TX buffer:
        // First byte is the PHR (PHY header = PSDU length including FCS)
        let phr = (frame.len() + 2) as u32; // +2 for CRC appended by hardware
        reg_write(BUFC_BUF0_DATA, phr);
        for &b in frame {
            reg_write(BUFC_BUF0_DATA, b as u32);
        }

        // Clear IRQ flags and enable TX done interrupt
        reg_write(RAC_IFC, RAC_IF_TXDONE);

        // Trigger TX
        reg_write(RAC_CMD, RAC_CMD_TXEN);

        log::trace!(
            "efr32s2: tx {} bytes on ch{}",
            frame.len(),
            self.config.channel
        );

        // Wait for TX completion IRQ
        let ok = TX_DONE.wait().await;
        if ok {
            Ok(())
        } else {
            Err(RadioError::HardwareError)
        }
    }

    /// Receive the next IEEE 802.15.4 frame (async).
    ///
    /// Enables the receiver and waits for a frame. Returns the frame data
    /// with RSSI and LQI. Frames failing CRC are rejected.
    pub async fn receive(&mut self) -> Result<RxFrame, RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }

        RX_DONE.reset();

        // Clear RX buffer
        reg_write(BUFC_CMD, 0x0000_0002); // TODO: verify against EFR32xG21 RM — reset RX FIFO

        // Clear IRQ flags and enable RX interrupts
        reg_write(RAC_IFC, RAC_IF_RXDONE | RAC_IF_RXOF);
        reg_write(FRC_IFC, 0xFFFF_FFFF); // clear all FRC flags

        // Enable RX
        reg_write(RAC_CMD, RAC_CMD_RXEN);

        // Wait for RX completion
        RX_DONE.wait().await;

        if !RX_CRC_OK.load(Ordering::Acquire) {
            return Err(RadioError::CrcError);
        }

        let len = RX_LEN.load(Ordering::Acquire) as usize;
        if len == 0 {
            return Err(RadioError::RxTimeout);
        }

        let rssi = RX_RSSI.load(Ordering::Acquire);

        let mut frame = RxFrame {
            data: [0u8; MAX_FRAME_LEN],
            len,
            rssi,
            lqi: rssi_to_lqi(rssi),
        };

        unsafe {
            let buf = &*RX_BUF.get();
            frame.data[..len].copy_from_slice(&buf[..len]);
        }

        log::trace!("efr32s2: rx {} bytes rssi={}dBm", len, rssi);
        Ok(frame)
    }

    /// Perform energy detection (synchronous).
    ///
    /// Reads the current RSSI from the AGC block. The receiver should
    /// already be enabled, or this returns the last measured value.
    pub fn energy_detect(&self) -> Result<(i8, bool), RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }

        // Read RSSI from AGC register (signed 8-bit value in upper byte)
        let rssi_raw = reg_read(AGC_RSSI);
        let rssi = (rssi_raw & 0xFF) as i8; // TODO: verify against EFR32xG21 RM — RSSI encoding
        let busy = rssi > -60; // CCA threshold: -60 dBm (typical for 802.15.4)

        Ok((rssi, busy))
    }

    /// Perform Clear Channel Assessment (async, IEEE 802.15.4 CCA mode 1).
    ///
    /// Briefly enables the receiver to measure RF energy on the current channel.
    /// Returns `true` if the channel is busy (energy above threshold).
    pub async fn clear_channel_assessment(&mut self) -> Result<bool, RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }

        // Enable RX briefly for RSSI measurement
        reg_write(RAC_CMD, RAC_CMD_RXEN);

        // Wait 128µs (8 symbol periods at 62.5 ksym/s) for RSSI to settle
        embassy_time::Timer::after_micros(128).await;

        // Read RSSI
        let rssi_raw = reg_read(AGC_RSSI);
        let rssi = (rssi_raw & 0xFF) as i8; // TODO: verify against EFR32xG21 RM

        // Disable RX
        reg_write(RAC_CMD, RAC_CMD_RXDIS);

        // CCA threshold: -60 dBm (typical for 802.15.4)
        let busy = rssi > -60;
        Ok(busy)
    }

    /// Enable continuous receive mode.
    pub fn enable_rx(&self) {
        if !self.initialized {
            return;
        }
        reg_write(RAC_IFC, RAC_IF_RXDONE | RAC_IF_RXOF);
        reg_write(RAC_CMD, RAC_CMD_RXEN);
    }

    /// Disable the receiver.
    pub fn disable_rx(&self) {
        reg_write(RAC_CMD, RAC_CMD_RXDIS);
        reg_write(RAC_IFC, RAC_IF_RXDONE | RAC_IF_RXOF);
    }

    /// Power down the radio to save power between TX/RX cycles.
    ///
    /// Disables radio peripheral clocks via CMU. Saves ~5–10 mA on EFR32MG21.
    /// Call `radio_wake()` before the next TX or RX operation.
    pub fn radio_sleep(&self) {
        // Disable RX/TX
        reg_write(RAC_CMD, RAC_CMD_TXDIS | RAC_CMD_RXDIS);

        // Clear pending IRQs
        reg_write(RAC_IFC, RAC_IF_TXDONE | RAC_IF_RXDONE | RAC_IF_RXOF);

        // Disable radio block clocks to save power
        reg_clear_bits(
            CMU_HFRADIOCLKEN0,
            CMU_HFRADIOCLKEN0_RAC
                | CMU_HFRADIOCLKEN0_FRC
                | CMU_HFRADIOCLKEN0_MODEM
                | CMU_HFRADIOCLKEN0_SYNTH
                | CMU_HFRADIOCLKEN0_AGC
                | CMU_HFRADIOCLKEN0_BUFC,
        );
    }

    /// Re-enable radio after `radio_sleep()`.
    ///
    /// Re-enables clocks and re-applies channel configuration.
    /// Must be called before any TX/RX operation after sleep.
    pub fn radio_wake(&mut self) {
        // Re-enable radio block clocks
        reg_set_bits(
            CMU_HFRADIOCLKEN0,
            CMU_HFRADIOCLKEN0_RAC
                | CMU_HFRADIOCLKEN0_FRC
                | CMU_HFRADIOCLKEN0_MODEM
                | CMU_HFRADIOCLKEN0_SYNTH
                | CMU_HFRADIOCLKEN0_AGC
                | CMU_HFRADIOCLKEN0_BUFC,
        );

        // Small settling delay for clocks
        for _ in 0..1_000u32 {
            core::hint::spin_loop();
        }

        // Re-apply channel (re-locks PLL)
        self.set_channel(self.config.channel);
    }
}

// ── IRQ handler ─────────────────────────────────────────────────

/// RAC/FRC interrupt handler for EFR32MG21 radio.
///
/// This must be registered as the interrupt handler for the radio IRQ
/// (varies by device — typically IRQ #30 or FRC_PRI_IRQn on EFR32xG1).
/// The handler reads IRQ status from both RAC and FRC, processes TX/RX
/// completion, and signals the async driver via Embassy Signal.
///
/// The function name `FRC_PRI` matches the `device.x` linker symbol,
/// overriding the weak `DefaultHandler` alias.
#[unsafe(no_mangle)]
pub extern "C" fn FRC_PRI() {
    let rac_flags = reg_read(RAC_IF);
    let frc_flags = reg_read(FRC_IF);

    // Clear all pending IRQs
    reg_write(RAC_IFC, rac_flags);
    reg_write(FRC_IFC, frc_flags);

    // TX completion
    if rac_flags & RAC_IF_TXDONE != 0 {
        TX_DONE.signal(true);
    }

    // RX completion
    if rac_flags & RAC_IF_RXDONE != 0 {
        // Check CRC result from FRC
        let crc_ok = frc_flags & 0x0000_0010 != 0; // TODO: verify against EFR32xG21 RM — CRC OK bit

        RX_CRC_OK.store(crc_ok, Ordering::Release);

        if crc_ok {
            // Read frame length from RX buffer
            let len_word = reg_read(BUFC_BUF1_RCNT);
            let len = (len_word as usize).min(MAX_FRAME_LEN);

            RX_LEN.store(len as u8, Ordering::Release);

            // Read RSSI
            let rssi_raw = reg_read(AGC_RSSI);
            let rssi = (rssi_raw & 0xFF) as i8;
            RX_RSSI.store(rssi, Ordering::Release);

            // Copy frame data from RX FIFO
            unsafe {
                let buf = &mut *RX_BUF.get();
                for i in 0..len {
                    buf[i] = (reg_read(BUFC_BUF1_DATA) & 0xFF) as u8;
                }
            }
        } else {
            RX_LEN.store(0, Ordering::Release);
        }

        RX_DONE.signal(());
    }

    // RX overflow — signal with empty frame
    if rac_flags & RAC_IF_RXOF != 0 {
        RX_CRC_OK.store(false, Ordering::Release);
        RX_LEN.store(0, Ordering::Release);
        RX_DONE.signal(());
    }
}

// ── Utility ─────────────────────────────────────────────────────

/// Convert RSSI to LQI (Link Quality Indicator, 0–255).
///
/// Delegates to the shared stack-wide policy in [`crate::lqi::from_rssi_dbm`]
/// so every backend that has no IEEE-normalized hardware quality byte uses one
/// definition. Behaviour is unchanged from the previous local copy.
fn rssi_to_lqi(rssi: i8) -> u8 {
    crate::lqi::from_rssi_dbm(rssi)
}
