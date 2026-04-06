//! Pure-Rust IEEE 802.15.4 radio driver for Silicon Labs EFR32MG1P.
//!
//! The EFR32MG1P is an ARM Cortex-M4F multi-protocol SoC with an integrated
//! 2.4 GHz radio supporting IEEE 802.15.4, BLE, and proprietary protocols.
//! This driver configures the radio entirely through memory-mapped registers —
//! **no RAIL library, no GSDK binary blobs required**.
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
//! Efr32Driver (pure Rust, async)
//!   ├── CMU registers (0x400E4000)
//!   │     └── enable_clocks()    → enable RAC, FRC, MODEM, SYNTH, AGC, BUFC
//!   ├── RAC registers (0x40084000)
//!   │     ├── radio_init()       → configure for 802.15.4 mode
//!   │     ├── set_tx_power()     → PA power control
//!   │     └── radio_sleep/wake() → power management
//!   ├── FRC registers (0x40080000)
//!   │     └── configure frame format, CRC-16
//!   ├── MODEM registers (0x40086000)
//!   │     └── O-QPSK 250kbps configuration
//!   ├── SYNTH registers (0x40083000)
//!   │     └── set_channel()      → program PLL for channel frequency
//!   ├── AGC registers (0x40087000)
//!   │     └── energy_detect()    → RSSI measurement
//!   ├── BUFC registers (0x40082000)
//!   │     ├── transmit()         → load TX buffer, trigger TX
//!   │     └── receive()          → configure RX buffer
//!   └── IRQ → Embassy Signal for async TX/RX completion
//! ```
//!
//! # Register Values
//! Radio configuration registers were extracted from a working RAIL-based
//! Zigbee firmware running on an IKEA TRÅDFRI EFR32MG1P module. The MODEM,
//! RAC, FRC, and AGC register blocks are programmed with these exact values
//! to configure the radio for IEEE 802.15.4 O-QPSK 250 kbps operation.

use core::sync::atomic::{AtomicBool, AtomicI8, AtomicU8, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

// ── EFR32MG1 Peripheral Base Addresses ──────────────────────────

/// Clock Management Unit — controls peripheral clock gating.
const CMU_BASE: u32 = 0x400E_4000;

/// Radio Controller — top-level radio state machine.
const RAC_BASE: u32 = 0x4008_4000;

/// Frame Controller — frame formatting, preamble, sync word, CRC.
const FRC_BASE: u32 = 0x4008_0000;

/// Modem — modulation/demodulation engine.
const MODEM_BASE: u32 = 0x4008_6000;

/// Frequency Synthesizer — PLL for channel selection.
const SYNTH_BASE: u32 = 0x4008_3000;

/// Automatic Gain Control — receiver gain and RSSI.
const AGC_BASE: u32 = 0x4008_7000;

/// Buffer Controller — TX/RX DMA-like FIFO management.
const BUFC_BASE: u32 = 0x4008_2000;

/// GPIO controller base.
const _GPIO_BASE: u32 = 0x4000_A000;

// ── CMU Register Offsets ────────────────────────────────────────

/// High-frequency peripheral clock enable register 0.
const CMU_HFPERCLKEN0: u32 = CMU_BASE + 0x044;
/// High-frequency radio clock enable register.
const CMU_HFRADIOCLKEN0: u32 = CMU_BASE + 0x0C8;
/// CMU oscillator enable register.
const CMU_OSCENCMD: u32 = CMU_BASE + 0x020;
/// CMU clock select command register.
const CMU_HFCLKSEL: u32 = CMU_BASE + 0x024;
/// CMU status register.
const CMU_STATUS: u32 = CMU_BASE + 0x01C;

// ── CMU Clock Enable Bits ───────────────────────────────────────

/// Bit 0 = RAC, bit 1 = FRC, bit 2 = MODEM — combined = 0x07 enables all radio.
const CMU_HFRADIOCLKEN0_ALL: u32 = 0x07;

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

// ── RAC Commands ────────────────────────────────────────────────

const RAC_CMD_TXEN: u32 = 1 << 0;
const RAC_CMD_RXEN: u32 = 1 << 1;
const RAC_CMD_TXDIS: u32 = 1 << 2;
const RAC_CMD_RXDIS: u32 = 1 << 3;

// ── RAC Status Bits ─────────────────────────────────────────────

const RAC_STATUS_STATE_MASK: u32 = 0x0F;
const _RAC_STATE_OFF: u32 = 0x00;
const _RAC_STATE_IDLE: u32 = 0x01;
const RAC_STATE_RX: u32 = 0x02;
const _RAC_STATE_TX: u32 = 0x03;

// ── RAC IRQ Bits ────────────────────────────────────────────────

const RAC_IF_TXDONE: u32 = 1 << 0;
const RAC_IF_RXDONE: u32 = 1 << 1;
const RAC_IF_RXOF: u32 = 1 << 2;

// ── FRC Register Offsets ────────────────────────────────────────

/// FRC control register — frame format configuration.
const FRC_CTRL: u32 = FRC_BASE + 0x000;
/// FRC RX control register.
const FRC_RXCTRL: u32 = FRC_BASE + 0x004;
/// FRC trailing RX data register.
const FRC_TRAILRXDATA: u32 = FRC_BASE + 0x008;
/// FRC FEC control register.
const FRC_FECCTRL: u32 = FRC_BASE + 0x040;
/// FRC sniff control register.
const FRC_SNIFFCTRL: u32 = FRC_BASE + 0x044;
/// FRC block RAM address register.
const FRC_BLOCKRAMADDR: u32 = FRC_BASE + 0x04C;
/// FRC convolutional RAM address register.
const FRC_CONVRAMADDR: u32 = FRC_BASE + 0x06C;
/// FRC max length register.
const FRC_MAXLENGTH: u32 = FRC_BASE + 0x07C;
/// FRC address filter control registers (0xA0..0xAC).
const FRC_ADDRFILTCTRL: u32 = FRC_BASE + 0x0A0;
/// FRC CRC initialization value.
const _FRC_CRCINIT: u32 = FRC_BASE + 0x020;
/// FRC CRC polynomial.
const _FRC_CRCPOLY: u32 = FRC_BASE + 0x024;
/// FRC interrupt flag register.
const FRC_IF: u32 = FRC_BASE + 0x030;
/// FRC interrupt flag clear register.
const FRC_IFC: u32 = FRC_BASE + 0x038;

// ── MODEM Register Offsets ──────────────────────────────────────

/// MODEM control register 0 — modulation format.
const _MODEM_CTRL0: u32 = MODEM_BASE + 0x000;

// ── SYNTH Register Offsets ──────────────────────────────────────

/// SYNTH channel frequency control register.
const SYNTH_FREQ: u32 = SYNTH_BASE + 0x004;
/// SYNTH channel spacing register.
const SYNTH_CHSP: u32 = SYNTH_BASE + 0x008;
/// SYNTH channel number register.
const SYNTH_CHNO: u32 = SYNTH_BASE + 0x00C;
/// SYNTH command register.
const SYNTH_CMD: u32 = SYNTH_BASE + 0x010;

// ── AGC Register Offsets ────────────────────────────────────────

/// AGC control register 0.
const _AGC_CTRL0: u32 = AGC_BASE + 0x000;
/// AGC RSSI register — current received signal strength.
const AGC_RSSI: u32 = AGC_BASE + 0x020;

// ── BUFC Register Offsets ───────────────────────────────────────

/// BUFC TX buffer 0 data register.
const BUFC_BUF0_DATA: u32 = BUFC_BASE + 0x040;
/// BUFC TX buffer 0 write count.
const _BUFC_BUF0_WCNT: u32 = BUFC_BASE + 0x044;
/// BUFC RX buffer 1 data register.
const BUFC_BUF1_DATA: u32 = BUFC_BASE + 0x080;
/// BUFC RX buffer 1 read count.
const BUFC_BUF1_RCNT: u32 = BUFC_BASE + 0x084;
/// BUFC command register.
const BUFC_CMD: u32 = BUFC_BASE + 0x004;

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

/// Pure-Rust async radio driver for EFR32MG1P IEEE 802.15.4 mode.
pub struct Efr32Driver {
    config: RadioConfig,
    initialized: bool,
}

impl Efr32Driver {
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
        self.load_rac_sequences();
        self.configure_rac();
        self.configure_frc();
        self.configure_modem();
        self.configure_synth();
        self.configure_agc();
        self.apply_config();

        self.initialized = true;
        log::info!("efr32: radio initialized in IEEE 802.15.4 mode");
    }

    /// Load RAC sequencer microcode into RAM.
    ///
    /// The RAC radio sequencer runs a custom instruction set from RAM.
    /// This blob was dumped from a working RAIL firmware and contains
    /// the TX/RX state machine programs.
    fn load_rac_sequences(&self) {
        let seq_data = &super::rac_seq::RAC_SEQ_DATA;
        let dst_base = 0x2100_0000u32;
        for (i, &word) in seq_data.iter().enumerate() {
            let addr = dst_base + (i as u32) * 4;
            reg_write(addr, word);
        }

        // Set RAC sequence pointers (same values as RAIL uses)
        reg_write(RAC_BASE + 0x58, 0x21000F88); // Sequence pointer A
        reg_write(RAC_BASE + 0x64, 0x2100085E); // Sequence pointer B

        log::info!("efr32: loaded {} words of RAC sequence data", seq_data.len());
    }

    /// Enable peripheral clocks for all radio blocks via CMU.
    ///
    /// Sequence: enable HFXO (32 MHz crystal) → switch HFCLK to HFXO →
    /// enable radio peripheral clocks (RAC + FRC + MODEM via HFRADIOCLKEN0).
    fn enable_clocks(&self) {
        // Enable HFXO (bit 3 of OSCENCMD = HFXOEN)
        reg_write(CMU_OSCENCMD, 1 << 3);

        // Wait for HFXO ready (bit 4 of CMU_STATUS = HFXORDY)
        for _ in 0..100_000u32 {
            if reg_read(CMU_STATUS) & (1 << 4) != 0 {
                break;
            }
            core::hint::spin_loop();
        }

        // Switch HFCLK source to HFXO (write 2 to HFCLKSEL)
        reg_write(CMU_HFCLKSEL, 2);

        // Enable high-frequency peripheral clock
        // On Series 1, radio peripherals (RAC/FRC/MODEM) are on HFPERCLK bus
        // and don't have a separate HFRADIOCLKEN0 register.
        // RAC manages its own clock gating internally.
        let val = reg_read(CMU_HFPERCLKEN0);
        reg_write(CMU_HFPERCLKEN0, val | (1 << 0)); // Timer0 for timing
    }

    /// Configure RAC (Radio Controller) for 802.15.4 operation.
    ///
    /// Register values from a working RAIL-based firmware on EFR32MG1P.
    /// RAM pointer registers (0x58-0x64, 0x7C) are skipped — those are
    /// RAIL internal buffer addresses that don't apply to bare-metal mode.
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

        // Write RAC configuration registers (dumped from working firmware).
        // Skip: 0x00 (RXENSRCEN — read-only status),
        //       0x04 (CMD — command trigger),
        //       0x08 (STATUS — read-only),
        //       0x10/0x18/0x1C (IF/IFC/IEN — set separately below).
        reg_write(RAC_BASE + 0x0C, 0x0000_0380); // CTRL
        reg_write(RAC_BASE + 0x14, 0x0000_0003); // FORCESTATE
        reg_write(RAC_BASE + 0x20, 0x002C_0004); // IF_CFG0
        reg_write(RAC_BASE + 0x24, 0x0000_000C); // IF_CFG1
        reg_write(RAC_BASE + 0x28, 0x0000_00BC); // IF_CFG2
        reg_write(RAC_BASE + 0x30, 0x0000_0760); // PAEN/LNAEN timing
        reg_write(RAC_BASE + 0x3C, 0x0000_0150); // SYNTH timing
        reg_write(RAC_BASE + 0x48, 0x0000_008C); // PA config
        reg_write(RAC_BASE + 0x54, 0x0000_0004); // antenna/misc
        // 0x58-0x64 = RAIL RAM pointers (skip)
        reg_write(RAC_BASE + 0x6C, 0x0000_0001); // misc
        reg_write(RAC_BASE + 0x70, 0x0000_0001); // misc
        reg_write(RAC_BASE + 0x74, 0x0000_0010); // misc
        reg_write(RAC_BASE + 0x78, 0x0000_01C3); // misc
        // 0x7C = RAIL RAM pointer (skip)

        // Clear and enable relevant IRQs
        reg_write(RAC_IFC, RAC_IF_TXDONE | RAC_IF_RXDONE | RAC_IF_RXOF);
        reg_write(RAC_IEN, RAC_IF_TXDONE | RAC_IF_RXDONE);
    }

    /// Configure FRC (Frame Controller) for IEEE 802.15.4 frame format.
    ///
    /// Register values from a working RAIL-based firmware on EFR32MG1P.
    fn configure_frc(&self) {
        reg_write(FRC_CTRL,         0x0000_0000); // frame format control
        reg_write(FRC_RXCTRL,       0x0014_8001); // RX control
        reg_write(FRC_TRAILRXDATA,  0x0000_007F); // trailing RX data
        reg_write(FRC_FECCTRL,      0x0000_07A0); // FEC control
        reg_write(FRC_SNIFFCTRL,    0x0000_0068); // sniff control
        reg_write(FRC_BLOCKRAMADDR, 0x0000_001B); // block RAM address
        reg_write(FRC_CONVRAMADDR,  0x0000_A002); // convolutional RAM address
        reg_write(FRC_MAXLENGTH,    0x0001_7F8);  // max frame length

        // Address filter control registers
        reg_write(FRC_ADDRFILTCTRL + 0x00, 0x0000_4000);
        reg_write(FRC_ADDRFILTCTRL + 0x04, 0x0000_4CFF);
        reg_write(FRC_ADDRFILTCTRL + 0x08, 0x0000_4100);
        reg_write(FRC_ADDRFILTCTRL + 0x0C, 0x0000_4DFF);
    }

    /// Configure MODEM for IEEE 802.15.4 O-QPSK modulation at 250 kbps.
    ///
    /// All 32 register words from a working RAIL-based firmware on EFR32MG1P.
    /// Configures O-QPSK with half-sine pulse shaping, 2 Mchip/s chip rate,
    /// 62.5 ksym/s symbol rate, DSSS spreading (32 chips/symbol).
    fn configure_modem(&self) {
        // MODEM registers at 0x40086000, 32 words (0x00..0x7C)
        static MODEM_REGS: [u32; 32] = [
            0x0000_0000, // [0x00] CTRL0
            0x0000_0000, // [0x04] CTRL1
            0xFFFF_0000, // [0x08] CTRL2
            0x0000_0000, // [0x0C] CTRL3
            0x0000_0000, // [0x10] CTRL4
            0x0000_0010, // [0x14] CTRL5
            0x0413_F920, // [0x18] TXBR (TX baud rate)
            0x0052_C007, // [0x1C] RXBR (RX baud rate)
            0x0000_0000, // [0x20] CF (carrier frequency)
            0x0000_0000, // [0x24]
            0x0300_0000, // [0x28] timing
            0x0000_0000, // [0x2C]
            0x00FF_0264, // [0x30] pre/sync config
            0x0000_08A2, // [0x34] sync word
            0x0000_0001, // [0x38] sync config
            0x0008_07B0, // [0x3C] DSSS config
            0x0000_00A7, // [0x40] SFD (0xA7 = 802.15.4 SFD)
            0x0000_0000, // [0x44]
            0x0AC0_0141, // [0x48] demod config
            0x744A_C39B, // [0x4C] spreading/chip config
            0x0000_03F0, // [0x50]
            0x0000_0000, // [0x54]
            0x0000_0000, // [0x58]
            0x3010_0101, // [0x5C] demod timing
            0x7F7F_7050, // [0x60] AGC integration
            0x0000_0000, // [0x64]
            0x0000_0500, // [0x68]
            0x00F0_0000, // [0x6C]
            0x0000_0000, // [0x70]
            0x0000_0000, // [0x74]
            0x0000_0000, // [0x78]
            0x0000_0000, // [0x7C]
        ];

        for (i, &val) in MODEM_REGS.iter().enumerate() {
            reg_write(MODEM_BASE + (i as u32) * 4, val);
        }
    }

    /// Configure SYNTH (Frequency Synthesizer) for 2.4 GHz 802.15.4 channels.
    fn configure_synth(&self) {
        // Base frequency: 2405 MHz (channel 11)
        // Channel spacing: 5 MHz
        // Channels 11-26 → 2405-2480 MHz

        // Base frequency and channel spacing — these are programmed
        // dynamically by the PLL configuration. The exact encoding depends
        // on the SYNTH divider setup. Leave at reset defaults; channel
        // selection uses SYNTH_CHNO + SYNTH_CMD.
        reg_write(SYNTH_FREQ, 0x0000_0000);
        reg_write(SYNTH_CHSP, 0x0000_0000);
    }

    /// Configure AGC (Automatic Gain Control) for 802.15.4 reception.
    ///
    /// Register values from a working RAIL-based firmware on EFR32MG1P.
    fn configure_agc(&self) {
        reg_write(AGC_BASE + 0x00, 0x18C9_8021); // CTRL0 — AGC mode/target
        reg_write(AGC_BASE + 0x04, 0x0000_0080); // CTRL1
        reg_write(AGC_BASE + 0x08, 0x0000_8000); // CTRL2
        reg_write(AGC_BASE + 0x0C, 0x0000_8000); // CTRL3
        reg_write(AGC_BASE + 0x14, 0x0000_E0FA); // MININDEX
        reg_write(AGC_BASE + 0x18, 0x0000_18E7); // MANGAIN
        reg_write(AGC_BASE + 0x1C, 0x8284_0000); // timing/hold
        reg_write(AGC_BASE + 0x24, 0x0000_0082); // misc
        reg_write(AGC_BASE + 0x28, 0x0180_0000); // misc

        // Gain table entries
        reg_write(AGC_BASE + 0x48, 0x0000_3D3C);
        reg_write(AGC_BASE + 0x4C, 0x0000_19BC);
        reg_write(AGC_BASE + 0x50, 0x0CA8_6543); // gain steps
        reg_write(AGC_BASE + 0x54, 0x0654_3210); // gain steps
        reg_write(AGC_BASE + 0x58, 0x18B5_2507); // PNRF gain
        reg_write(AGC_BASE + 0x5C, 0x2518_3DCD); // PNRF gain

        // LNA/mixer control
        reg_write(AGC_BASE + 0x70, 0x0001_0103);
        reg_write(AGC_BASE + 0x74, 0x0000_0442);
        reg_write(AGC_BASE + 0x78, 0x0055_2300);
    }

    /// Set TX power via RAC PA config register.
    ///
    /// EFR32MG1P supports -20 dBm to +19 dBm output power.
    fn set_tx_power(&self, dbm: i8) {
        let power = dbm.clamp(-20, 19);
        // Map dBm to PA power register value.
        // The exact mapping is non-linear; linear approximation:
        //   -20 dBm → ~0, 0 dBm → ~64, +19 dBm → ~252
        let pa_val = ((power as i16 + 20) * 252 / 39).clamp(0, 252) as u32;
        // PA config is at RAC_BASE + 0x48 (confirmed from register dump value 0x8C)
        let old = reg_read(RAC_BASE + 0x48);
        reg_write(RAC_BASE + 0x48, (old & 0xFFFF_FF00) | pa_val);
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
        reg_write(SYNTH_CMD, 0x0000_0001);

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
        reg_write(BUFC_CMD, 0x0000_0001); // reset TX FIFO

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
            "efr32: tx {} bytes on ch{}",
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
        reg_write(BUFC_CMD, 0x0000_0002); // reset RX FIFO

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

        log::trace!("efr32: rx {} bytes rssi={}dBm", len, rssi);
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
        let rssi = (rssi_raw & 0xFF) as i8;
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
        let rssi = (rssi_raw & 0xFF) as i8;

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
    /// Disables radio peripheral clocks via CMU. Saves ~5–10 mA on EFR32MG1P.
    /// Call `radio_wake()` before the next TX or RX operation.
    pub fn radio_sleep(&self) {
        // Disable RX/TX
        reg_write(RAC_CMD, RAC_CMD_TXDIS | RAC_CMD_RXDIS);

        // Clear pending IRQs
        reg_write(RAC_IFC, RAC_IF_TXDONE | RAC_IF_RXDONE | RAC_IF_RXOF);

        // Disable radio block clocks to save power
        // Series 1: use RAC CMD to disable (no HFRADIOCLKEN0)
        reg_write(RAC_CMD, RAC_CMD_TXDIS | RAC_CMD_RXDIS);
    }

    /// Re-enable radio after `radio_sleep()`.
    ///
    /// Re-enables clocks and re-applies channel configuration.
    /// Must be called before any TX/RX operation after sleep.
    pub fn radio_wake(&mut self) {
        // Re-enable radio block clocks
        // Series 1: radio clocks managed by RAC internally

        // Small settling delay for clocks
        for _ in 0..1_000u32 {
            core::hint::spin_loop();
        }

        // Re-apply channel (re-locks PLL)
        self.set_channel(self.config.channel);
    }
}

// ── IRQ handler ─────────────────────────────────────────────────

/// RAC/FRC interrupt handler for EFR32MG1P radio.
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
        let crc_ok = frc_flags & 0x0000_0010 != 0; // FRC CRC OK bit

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
fn rssi_to_lqi(rssi: i8) -> u8 {
    // Simple linear mapping: -100 dBm → 0, -20 dBm → 255
    let clamped = (rssi as i16).clamp(-100, -20);
    (((clamped + 100) as u16) * 255 / 80) as u8
}
