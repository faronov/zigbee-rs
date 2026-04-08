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
//!   ├── BUFC registers (0x40081000)
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
/// CMSIS TypeDef struct at 0x40081000 (NOT 0x40082000).
const BUFC_BASE: u32 = 0x4008_1000;

/// GPIO controller base.
const _GPIO_BASE: u32 = 0x4000_A000;

// ── CMU Register Offsets (from CMSIS efr32fg1v_cmu.h TypeDef) ──
//
// CRITICAL: these offsets differ significantly from other ARM chips.
// The EFR32 Series 1 CMU layout has many reserved gaps.

/// CMU control register (0x000).
const _CMU_CTRL: u32 = CMU_BASE + 0x000;
/// CMU oscillator enable command register (0x060, NOT 0x020).
const CMU_OSCENCMD: u32 = CMU_BASE + 0x060;
/// CMU HFCLK select command register (0x074, NOT 0x068).
const CMU_HFCLKSEL: u32 = CMU_BASE + 0x074;
/// CMU status register (0x090, NOT 0x01C).
/// Bit 0 = HFRCOENS, Bit 2 = HFXOENS, Bit 3 = HFXORDY.
const CMU_STATUS: u32 = CMU_BASE + 0x090;
/// HFCLK status register (0x094). Bits[2:0] = selected source.
const _CMU_HFCLKSTATUS: u32 = CMU_BASE + 0x094;
/// High-frequency bus clock enable register 0 (0x0B0).
/// Bit 1 = GPIO.
const CMU_HFBUSCLKEN0: u32 = CMU_BASE + 0x0B0;
/// High-frequency peripheral clock enable register 0 (0x0C0).
const CMU_HFPERCLKEN0: u32 = CMU_BASE + 0x0C0;
/// Radio clock enable register 0 (0x0C8).
/// Individual radio peripheral enables: FRC, AGC, MODEM, etc.
const CMU_RADIOCLKEN0: u32 = CMU_BASE + 0x0C8;

// ── RAC Register Offsets (from CMSIS efr32fg1v_rac.h TypeDef) ──
//
// CRITICAL: verified against official CMSIS header.
// RAC has a complex layout with sequencer regs, RF front-end, etc.

const _RAC_RXENSRCEN: u32 = RAC_BASE + 0x000;
const RAC_STATUS: u32 = RAC_BASE + 0x004;
const RAC_CMD: u32 = RAC_BASE + 0x008;
const RAC_CTRL: u32 = RAC_BASE + 0x00C;
const _RAC_FORCESTATE: u32 = RAC_BASE + 0x010;
const RAC_IF: u32 = RAC_BASE + 0x014;
const _RAC_IFS: u32 = RAC_BASE + 0x018;
const RAC_IFC: u32 = RAC_BASE + 0x01C;  // was 0x018!
const _RAC_IEN: u32 = RAC_BASE + 0x020; // was 0x01C!
// Sequencer registers
const RAC_SEQSTATUS: u32 = RAC_BASE + 0x03C;
const RAC_SEQCMD: u32 = RAC_BASE + 0x040;
const _RAC_R0: u32 = RAC_BASE + 0x048;
const _RAC_R6: u32 = RAC_BASE + 0x060;
const RAC_VECTADDR: u32 = RAC_BASE + 0x07C;  // was 0x058 (=R4)!
const RAC_SEQCTRL: u32 = RAC_BASE + 0x080;   // was 0x014!
const _RAC_SR0: u32 = RAC_BASE + 0x088;
const _RAC_SR1: u32 = RAC_BASE + 0x08C;
const _RAC_SR2: u32 = RAC_BASE + 0x090;
const _RAC_SR3: u32 = RAC_BASE + 0x094;
const RAC_SYNTHREGCTRL: u32 = RAC_BASE + 0x09C;
const RAC_VCOCTRL: u32 = RAC_BASE + 0x0A0;
// RF front-end registers
const _RAC_LNAMIXCTRL: u32 = RAC_BASE + 0x0FC;
const _RAC_LNAMIXCTRL1: u32 = RAC_BASE + 0x134;
const RAC_IFPGACTRL: u32 = RAC_BASE + 0x138;
const RAC_IFFILTCTRL: u32 = RAC_BASE + 0x140;
const RAC_IFADCCTRL: u32 = RAC_BASE + 0x144;

// Sequencer commands (RAC_SEQCMD at 0x040)
const RAC_SEQCMD_HALT: u32 = 1 << 0;
const _RAC_SEQCMD_STEP: u32 = 1 << 1;
const RAC_SEQCMD_RESUME: u32 = 1 << 2;

// ── RAC Commands ────────────────────────────────────────────────

const RAC_CMD_TXEN: u32 = 1 << 0;
const _RAC_CMD_FORCETX: u32 = 1 << 1;
const _RAC_CMD_TXONCCA: u32 = 1 << 2;
const _RAC_CMD_CLEARTXEN: u32 = 1 << 3;
const _RAC_CMD_TXAFTERFRAME: u32 = 1 << 4;
const RAC_CMD_TXDIS: u32 = 1 << 5;       // was 1<<2!
const _RAC_CMD_CLEARRXOVERFLOW: u32 = 1 << 6;
const _RAC_CMD_RXCAL: u32 = 1 << 7;
const RAC_CMD_RXDIS: u32 = 1 << 8;       // was 1<<3!
// Note: NO RXEN in RAC_CMD — RX is started via RAC_RXENSRCEN register

// ── RAC Status Bits ─────────────────────────────────────────────

const RAC_STATUS_STATE_SHIFT: u32 = 24;
const RAC_STATUS_STATE_MASK: u32 = 0x0F << RAC_STATUS_STATE_SHIFT;
// RAC hardware state machine values (bits [27:24] of RAC_STATUS).
// These are the 10 hardware states — NOT the same as sequencer states!
const _RAC_STATE_OFF: u32 = 0x00;       // Radio off
const _RAC_STATE_RXWARM: u32 = 0x01;    // RX warming up
const RAC_STATE_RXSEARCH: u32 = 0x02;   // RX active, searching for preamble
const _RAC_STATE_RXFRAME: u32 = 0x03;   // Receiving a frame
const _RAC_STATE_RX2TX: u32 = 0x04;     // RX→TX turnaround
const _RAC_STATE_TXWARM: u32 = 0x05;    // TX warming up
const _RAC_STATE_TX: u32 = 0x06;        // Actively transmitting
const _RAC_STATE_TXPD: u32 = 0x07;      // TX power-down
const _RAC_STATE_TX2RX: u32 = 0x08;     // TX→RX turnaround
const _RAC_STATE_SHUTDOWN: u32 = 0x09;  // Radio shutdown / error state

// ── Sequencer RAM Variables (from VDowbensky/efr32_baremetal) ───
//
// The RAC sequencer uses RAM at 0x21000000 for microcode and
// variables at 0x21000EFC-0x21000FFF for configuration.
// SEQ TypeDef base = 0x21000F00.

/// Sequencer control register (NOT in RAC peripheral — it's in RAM!)
const SEQ_CONTROL_REG: u32 = 0x2100_0EFC;
/// Radio state transitions (SEQ->REG000)
const _SEQ_TRANSITIONS: u32 = 0x2100_0F00;
/// RX warm-up time (SEQ->REG09C)
const SEQ_RX_WARMTIME: u32 = 0x2100_0F9C;
/// RX search time (SEQ->REG0A0)
const SEQ_RX_SEARCHTIME: u32 = 0x2100_0FA0;
/// RX→TX turnaround (SEQ->REG0A4)
const SEQ_RX_TX_TIME: u32 = 0x2100_0FA4;
/// RX frame→TX time (SEQ->REG0A8)
const SEQ_RXFRAME_TX_TIME: u32 = 0x2100_0FA8;
/// TX warm-up time (SEQ->REG0AC)
const SEQ_TX_WARMTIME: u32 = 0x2100_0FAC;
/// TX→RX turnaround (SEQ->REG0B0)
const SEQ_TX_RX_TIME: u32 = 0x2100_0FB0;
/// TX→RX search time (SEQ->REG0B4)
const SEQ_TX_RX_SEARCHTIME: u32 = 0x2100_0FB4;
/// TX→TX time (SEQ->REG0B8)
const SEQ_TX_TX_TIME: u32 = 0x2100_0FB8;
/// SYNTH LPF control RX (SEQ->SYNTHLPFCTRLRX at 0xFF8)
const SEQ_SYNTHLPFCTRLRX: u32 = 0x2100_0FF8;
/// SYNTH LPF control TX (SEQ->SYNTHLPFCTRLTX at 0xFFC)
const SEQ_SYNTHLPFCTRLTX: u32 = 0x2100_0FFC;

// ── RAC IRQ Bits (from GSDK RAC_field.py) ────────────────────
// NOTE: RAC_IF does NOT have TXDONE/RXDONE — those are FRC_IF only.
const _RAC_IF_STATECHANGE: u32 = 1 << 0;
const _RAC_IF_STIMCMPEV: u32 = 1 << 1;
const _RAC_IF_BUSERROR: u32 = 1 << 2;

// ── FRC Register Offsets (from CMSIS TypeDef at 0x40080000) ─────

/// FRC status register (read-only).
const _FRC_STATUS: u32 = FRC_BASE + 0x000;
/// FRC data filter control.
const FRC_DFLCTRL: u32 = FRC_BASE + 0x004;
/// FRC max frame length.
const _FRC_MAXLENGTH: u32 = FRC_BASE + 0x008;
/// FRC address filter control.
const _FRC_ADDRFILTCTRL: u32 = FRC_BASE + 0x00C;
/// FRC data buffer register.
const _FRC_DATABUFFER: u32 = FRC_BASE + 0x010;
/// FRC word counter.
const _FRC_WCNT: u32 = FRC_BASE + 0x014;
/// FRC word count compare 0 — set to (payload_bytes - 1) before TX.
const FRC_WCNTCMP0: u32 = FRC_BASE + 0x018;
/// FRC word count compare 1.
const _FRC_WCNTCMP1: u32 = FRC_BASE + 0x01C;
/// FRC word count compare 2.
const _FRC_WCNTCMP2: u32 = FRC_BASE + 0x020;
/// FRC command register — BIT(0)=RXABORT, BIT(1)=FRAMEDETRESUME.
const FRC_CMD: u32 = FRC_BASE + 0x024;
/// FRC whitening control.
const _FRC_WHITECTRL: u32 = FRC_BASE + 0x028;
/// FRC whitening polynomial.
const _FRC_WHITEPOLY: u32 = FRC_BASE + 0x02C;
/// FRC whitening init value.
const _FRC_WHITEINIT: u32 = FRC_BASE + 0x030;
/// FRC FEC control register.
const FRC_FECCTRL: u32 = FRC_BASE + 0x034;
/// FRC block RAM address.
const FRC_BLOCKRAMADDR: u32 = FRC_BASE + 0x038;
/// FRC convolutional RAM address.
const FRC_CONVRAMADDR: u32 = FRC_BASE + 0x03C;
/// FRC control register — bits[10:8]=BITSPERWORD, bit[5]=BITORDER.
const FRC_CTRL: u32 = FRC_BASE + 0x040;
/// FRC RX control register.
const FRC_RXCTRL: u32 = FRC_BASE + 0x044;
/// FRC trail TX data control.
const _FRC_TRAILTXDATACTRL: u32 = FRC_BASE + 0x048;
/// FRC trailing RX data register.
const FRC_TRAILRXDATA: u32 = FRC_BASE + 0x04C;
/// FRC sequence counter.
const _FRC_SCNT: u32 = FRC_BASE + 0x050;
/// FRC interrupt flag register.
const FRC_IF: u32 = FRC_BASE + 0x060;
/// FRC interrupt flag set register.
const _FRC_IFS: u32 = FRC_BASE + 0x064;
/// FRC interrupt flag clear register.
const FRC_IFC: u32 = FRC_BASE + 0x068;
/// FRC interrupt enable register.
const FRC_IEN: u32 = FRC_BASE + 0x06C;
/// FRC buffer mode register.
const _FRC_BUFFERMODE: u32 = FRC_BASE + 0x070;
/// Frame Control Descriptor 0 (TX descriptor).
const FRC_FCD0: u32 = FRC_BASE + 0x0A0;
/// Frame Control Descriptor 1.
const _FRC_FCD1: u32 = FRC_BASE + 0x0A4;
/// Frame Control Descriptor 2 (RX descriptor).
const FRC_FCD2: u32 = FRC_BASE + 0x0A8;
/// Frame Control Descriptor 3.
const _FRC_FCD3: u32 = FRC_BASE + 0x0AC;

// ── FRC Command Bits (FRC_CMD at 0x024) ─────────────────────────

/// Abort current RX operation.
const FRC_CMD_RXABORT: u32 = 1 << 0;
/// Resume frame detection after abort.
const _FRC_CMD_FRAMEDETRESUME: u32 = 1 << 1;

// ── FRC Interrupt Flag Bits (FRC_IF at 0x060) ───────────────────

const FRC_IF_TXDONE: u32 = 1 << 0;
const _FRC_IF_TXAFTERFRAMEDONE: u32 = 1 << 1;
const _FRC_IF_TXABORTED: u32 = 1 << 2;
const _FRC_IF_TXUF: u32 = 1 << 3;
const FRC_IF_RXDONE: u32 = 1 << 4;
const _FRC_IF_RXABORTED: u32 = 1 << 5;
const FRC_IF_FRAMEERROR: u32 = 1 << 6;
const _FRC_IF_BLOCKERROR: u32 = 1 << 7;
const FRC_IF_RXOF: u32 = 1 << 8;

// ── MODEM Register Offsets (CMSIS: efr32fg1v_modem.h) ──────────
// 0x000–0x010 are read-only status registers; first writable is MIXCTRL at 0x014.

/// MODEM status register (RO).
const _MODEM_STATUS: u32 = MODEM_BASE + 0x000;

// ── SYNTH Register Offsets (CMSIS: efr32fg1v_synth.h) ──────────

/// SYNTH status register (RO).
const _SYNTH_STATUS: u32 = SYNTH_BASE + 0x000;
/// SYNTH command register (WO).
const _SYNTH_CMD: u32 = SYNTH_BASE + 0x004;
/// SYNTH control register.
const SYNTH_CTRL: u32 = SYNTH_BASE + 0x008;
/// SYNTH calibration control register.
const _SYNTH_CALCTRL: u32 = SYNTH_BASE + 0x00C;
/// SYNTH frequency register (base frequency).
const SYNTH_FREQ: u32 = SYNTH_BASE + 0x02C;
/// SYNTH IF frequency register.
const SYNTH_IFFREQ: u32 = SYNTH_BASE + 0x030;
/// SYNTH divider control (LO divider).
const SYNTH_DIVCTRL: u32 = SYNTH_BASE + 0x034;
/// SYNTH channel control register (channel number).
const SYNTH_CHCTRL: u32 = SYNTH_BASE + 0x038;
/// SYNTH channel spacing register.
const SYNTH_CHSP: u32 = SYNTH_BASE + 0x03C;
/// SYNTH calibration offset register.
const _SYNTH_CALOFFSET: u32 = SYNTH_BASE + 0x040;
/// SYNTH VCO tuning register.
const _SYNTH_VCOTUNING: u32 = SYNTH_BASE + 0x044;

// ── AGC Register Offsets (CMSIS: efr32fg1v_agc.h) ──────────────
// 0x000–0x00C are read-only status registers; first writable is CTRL0 at 0x014.

/// AGC control register 0.
const _AGC_CTRL0: u32 = AGC_BASE + 0x014;
/// AGC RSSI register — current received signal strength (RO, 0x008).
const AGC_RSSI: u32 = AGC_BASE + 0x008;

// ── BUFC Register Offsets (from CMSIS TypeDef at 0x40081000) ────

// Buffer 0 — used for TX
const BUFC_BUF0_CTRL: u32 = BUFC_BASE + 0x000;
const BUFC_BUF0_ADDR: u32 = BUFC_BASE + 0x004;
const _BUFC_BUF0_WRITEOFFSET: u32 = BUFC_BASE + 0x008;
const _BUFC_BUF0_READOFFSET: u32 = BUFC_BASE + 0x00C;
const _BUFC_BUF0_READDATA: u32 = BUFC_BASE + 0x014;
const BUFC_BUF0_WRITEDATA: u32 = BUFC_BASE + 0x018;
const _BUFC_BUF0_STATUS: u32 = BUFC_BASE + 0x020;
const BUFC_BUF0_CMD: u32 = BUFC_BASE + 0x028;

// Buffer 1 — used for RX
const BUFC_BUF1_CTRL: u32 = BUFC_BASE + 0x030;
const BUFC_BUF1_ADDR: u32 = BUFC_BASE + 0x034;
const BUFC_BUF1_READDATA: u32 = BUFC_BASE + 0x044;
const BUFC_BUF1_STATUS: u32 = BUFC_BASE + 0x050;
const BUFC_BUF1_CMD: u32 = BUFC_BASE + 0x058;

// Buffer 2 — used for RX length
const _BUFC_BUF2_CMD: u32 = BUFC_BASE + 0x088;

// BUFC interrupt registers
const BUFC_IF: u32 = BUFC_BASE + 0x0E0;
const BUFC_IFC: u32 = BUFC_BASE + 0x0E8;

/// BUFC buffer size code: 2 = 256 bytes.
const BUFC_BUFSIZE_256: u32 = 2;

// ── Static RAM buffers for BUFC DMA ─────────────────────────────

/// Word-aligned buffer wrapper for BUFC DMA.
/// BUFC requires buffer addresses to be word-aligned (4 bytes).
/// Rust `[u8; N]` has alignment 1, which is insufficient.
#[repr(C, align(4))]
struct AlignedBuf<const N: usize>([u8; N]);

/// TX RAM buffer pointed to by BUFC BUF0_ADDR.
static BUFC_TX_RAM: SyncUnsafeCell<AlignedBuf<256>> = SyncUnsafeCell::new(AlignedBuf([0u8; 256]));
/// RX RAM buffer pointed to by BUFC BUF1_ADDR.
static BUFC_RX_RAM: SyncUnsafeCell<AlignedBuf<256>> = SyncUnsafeCell::new(AlignedBuf([0u8; 256]));

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

    /// Full radio initialization — order matches baremetal RADIO_Config():
    /// 1. Enable clocks
    /// 2. SetAndForgetWrite (RAC analog config) ← BEFORE sequencer!
    /// 3. Load sequencer + PROTIMER
    /// 4. FRC, MODEM, SYNTH, AGC, BUFC
    /// 5. Apply config (channel, power)
    fn init_hardware(&mut self) {
        self.enable_clocks();
        self.configure_rac();        // MUST be before sequencer load!
        self.configure_protimer();
        self.load_rac_sequences();   // Now RAC regs are set when seq runs
        self.configure_crc();        // CRC must be configured for FRC frame validation
        self.configure_frc();
        self.configure_modem();
        self.configure_bufc();
        self.configure_synth();
        self.configure_agc();
        self.apply_config();

        self.initialized = true;

        // Start RX NOW — all peripherals (CRC, FRC, MODEM, BUFC, SYNTH, AGC)
        // are fully configured. This is where the baremetal reference calls
        // radio_startrx().
        let ctrl = reg_read(SEQ_CONTROL_REG);
        reg_write(SEQ_CONTROL_REG, ctrl & !0x20); // Clear "radio disabled" flag
        reg_write(RAC_IFPGACTRL, 0x0000_87F6);    // IFPGA for 2.4 GHz RX
        reg_write(_RAC_RXENSRCEN, 0x02);           // Software RX enable

        // Wait for RX to come up
        for _ in 0..50_000u32 { core::hint::spin_loop(); }

        // Debug: dump critical register values after init
        rtt_target::rprintln!("=== POST-INIT DUMP ===");
        rtt_target::rprintln!("FRC: CTRL={:#X} RXCTRL={:#X} FECCTRL={:#X}",
            reg_read(FRC_CTRL), reg_read(FRC_RXCTRL), reg_read(FRC_FECCTRL));
        rtt_target::rprintln!("FRC: FCD0={:#X} FCD1={:#X} FCD2={:#X} FCD3={:#X}",
            reg_read(FRC_FCD0), reg_read(FRC_BASE+0xA4), reg_read(FRC_FCD2), reg_read(FRC_BASE+0xAC));
        rtt_target::rprintln!("RAC: CTRL={:#X} STATUS={:#X} SEQST={:#X}",
            reg_read(RAC_CTRL), reg_read(RAC_STATUS), reg_read(RAC_SEQSTATUS));
        rtt_target::rprintln!("RAC: R4={:#X} R5={:#X} R6={:#X} R7={:#X}",
            reg_read(RAC_BASE+0x58), reg_read(RAC_BASE+0x5C), 
            reg_read(RAC_BASE+0x60), reg_read(RAC_BASE+0x64));
        rtt_target::rprintln!("SYNTH: FREQ={:#X} DIVCTRL={:#X} IFFREQ={:#X}",
            reg_read(SYNTH_FREQ), reg_read(SYNTH_DIVCTRL), reg_read(SYNTH_IFFREQ));
        rtt_target::rprintln!("SEQ: CTRL_REG={:#X} TRANSITIONS={:#X}",
            reg_read(SEQ_CONTROL_REG), reg_read(_SEQ_TRANSITIONS));
        // Dump CRC peripheral to verify register offsets are correct.
        // We expect: +0x000=0x704 (CTRL), +0x008=0x0 (INIT), +0x00C=0x8408 (POLY).
        // If POLY shows 0x0 at +0x00C, the offsets need adjustment.
        rtt_target::rprintln!("CRC @0x40082000: +0={:#X} +4={:#X} +8={:#X} +C={:#X} +10={:#X} +14={:#X} +18={:#X}",
            reg_read(0x4008_2000), reg_read(0x4008_2004),
            reg_read(0x4008_2008), reg_read(0x4008_200C),
            reg_read(0x4008_2010), reg_read(0x4008_2014),
            reg_read(0x4008_2018));
        rtt_target::rprintln!("RAC state={}", (reg_read(RAC_STATUS) >> 24) & 0x0F);
        rtt_target::rprintln!("=== END DUMP ===");
    }

    /// Initialize PROTIMER (Protocol Timer) — required by RAC sequencer.
    ///
    /// The PROTIMER provides timing for the radio state machine.
    /// Without it, the sequencer cannot schedule TX/RX warm-up
    /// and transitions, causing SEQSTATUS to go to DONE immediately.
    ///
    /// PROTIMER base = 0x40085000, clock from RADIOCLKEN0 bit 0.
    fn configure_protimer(&self) {
        const PROTIMER_BASE: u32 = 0x4008_5000;
        const PROTIMER_CTRL: u32 = PROTIMER_BASE + 0x000;
        const PROTIMER_CMD: u32 = PROTIMER_BASE + 0x004;
        const PROTIMER_PRECNTTOP: u32 = PROTIMER_BASE + 0x028;
        const PROTIMER_WRAPCNTTOP: u32 = PROTIMER_BASE + 0x030;
        const PROTIMER_IFC: u32 = PROTIMER_BASE + 0x064;
        const PROTIMER_IEN: u32 = PROTIMER_BASE + 0x068;

        // CTRL = 0x11100 (from baremetal)
        reg_write(PROTIMER_CTRL, 0x0001_1100);

        // PRECNTTOP: derived from system clock
        // precnttop = (HFCLK_Hz / 1000) * 0x200 + 500) / 1000
        // For 38.4 MHz: (38400000/1000) * 512 + 500) / 1000 = (38400*512+500)/1000 = 19661
        // = 0x4CBD → encoded as (0x4CBD & 0xFF) | ((0x4CBD & 0xFFFFFF00) - 0x100)
        let hfclk = 38_400_000u32;
        let precnttop_raw = (hfclk / 1000 * 0x200 + 500) / 1000;
        let precnttop = (precnttop_raw & 0xFF) | (precnttop_raw & 0xFFFFFF00).wrapping_sub(0x100);
        reg_write(PROTIMER_PRECNTTOP, precnttop);

        // WRAPCNTTOP = 0 (from baremetal)
        reg_write(PROTIMER_WRAPCNTTOP, 0);

        // Enable WRAPCNTOF interrupt
        reg_write(PROTIMER_IEN, 1 << 2); // WRAPCNTOF = bit 2

        // Clear pending interrupts
        reg_write(PROTIMER_IFC, 0xFFFF_FFFF);

        // Start the timer (CMD bit 0 = START)
        reg_write(PROTIMER_CMD, 1);
    }

    /// Load RAC sequencer microcode into RAM.
    ///
    /// The RAC radio sequencer runs a custom instruction set from RAM.
    /// This blob was dumped from a working RAIL firmware and contains
    /// the TX/RX state machine programs. The sequencer is essential —
    /// RAC_CMD_TXEN/RXEN trigger the sequencer to coordinate SYNTH
    /// calibration, PA enable, FRC TX/RX, and return to idle.
    fn load_rac_sequences(&self) {
        // Matches VDowbensky/efr32_baremetal RADIO_SeqInit() EXACTLY.
        //
        // VDowbensky genericSeqProg = 3820 bytes (955 words).
        // Code: 0x21000000-0x21000EEB
        // Variables: 0x21000EFC-0x21000FFF (cleared before resume)
        //
        // CRITICAL: Do NOT write variables before RESUME!
        // The sequencer starts with all vars=0 and enters WAITING state.
        // Variables are set AFTER resume by RADIO_Config().

        // 1. Halt sequencer
        reg_write(RAC_SEQCMD, RAC_SEQCMD_HALT);

        // 2. Clear ALL 4KB of sequencer RAM
        for i in 0..1024u32 {
            reg_write(0x2100_0000 + i * 4, 0);
        }

        // Dummy read (baremetal does this)
        let _ = reg_read(RAC_STATUS);

        // 3. Set vector address + compact mode
        reg_write(RAC_VECTADDR, 0x2100_0000);
        reg_write(RAC_SEQCTRL, 1); // COMPACT

        // 4. Load microcode (955 words = 3820 bytes)
        let seq_data = &super::rac_seq::RAC_SEQ_DATA;
        for (i, &word) in seq_data.iter().enumerate() {
            reg_write(0x2100_0000 + (i as u32) * 4, word);
        }

        // 5. Set R6 pointer (baremetal does this BEFORE clearing vars)
        reg_write(_RAC_R6, 0x2100_0FCC);

        // 6. Clear variable areas (baremetal does TWO clears AFTER code load):
        //    a) 0x21000F6C-0x21000FFF (SEQ config area — 148 bytes)
        for addr in (0x2100_0F6Cu32..=0x2100_0FFCu32).step_by(4) {
            reg_write(addr, 0);
        }
        //    b) 0x21000EFC-0x21000F6B (SEQ variables — 112 bytes)
        for addr in (0x2100_0EFCu32..0x2100_0F6Cu32).step_by(4) {
            reg_write(addr, 0);
        }

        // 7. RESUME — sequencer runs init with all vars = 0
        //    It should enter WAITING state if everything is correct.
        reg_write(RAC_SEQCMD, RAC_SEQCMD_RESUME);
        reg_write(_RAC_SR0, 0);
        reg_write(_RAC_SR1, 0);
        reg_write(_RAC_SR2, 0);

        // Short delay for sequencer init
        for _ in 0..50_000u32 { core::hint::spin_loop(); }

        let seqst = reg_read(RAC_SEQSTATUS);
        rtt_target::rprintln!("efr32: SEQ {} words, SEQST={:#X}", seq_data.len(), seqst);

        // 8. NOW set variables (AFTER sequencer is running, same as RADIO_Config)
        reg_write(SEQ_CONTROL_REG, reg_read(SEQ_CONTROL_REG) | 0x08);

        // Warm-up times (µs)
        reg_write(SEQ_RX_WARMTIME, 100);
        reg_write(SEQ_TX_WARMTIME, 100);
        reg_write(SEQ_RX_TX_TIME, 100);
        reg_write(SEQ_TX_RX_TIME, 100);
        reg_write(SEQ_TX_TX_TIME, 100);
        reg_write(SEQ_RXFRAME_TX_TIME, 100);
        reg_write(SEQ_RX_SEARCHTIME, 0);
        reg_write(SEQ_TX_RX_SEARCHTIME, 0);

        // State transitions: all → RX
        // The sequencer uses its OWN state encoding, NOT the RAIL API encoding:
        //   Sequencer: IDLE=0, RX=1, TX=2
        //   RAIL API:  INACTIVE=0, IDLE=1, RX=2, TX=3
        // One-hot bit for sequencer RX (state 1) = 1 << 1 = 0x02
        // Previous value 0x04040404 was bit 2 = TX, causing TX→TX→SHUTDOWN!
        reg_write(_SEQ_TRANSITIONS, 0x0202_0202);

        // SYNTH LPF
        reg_write(SEQ_SYNTHLPFCTRLRX, 0x0003_C002);
        reg_write(SEQ_SYNTHLPFCTRLTX, 0x0003_C002);

        // 9. NOTE: Do NOT start RX here! The baremetal reference calls
        //    radio_startrx() AFTER all config (FRC, MODEM, BUFC, SYNTH, AGC).
        //    Starting RX now would enter RXSEARCH with unconfigured peripherals:
        //    - BUFC BUF1_ADDR = 0 → DMA writes to flash address 0!
        //    - FRC has no frame format → can't delimit frames
        //    - CRC polynomial not set → CRC engine uninitialized
        //    RX is started at the end of init_hardware() instead.

        for _ in 0..50_000u32 { core::hint::spin_loop(); }
        rtt_target::rprintln!("efr32: after seq load: SEQST={:#X} RAC={:#X}",
            reg_read(RAC_SEQSTATUS), reg_read(RAC_STATUS));
    }


    /// Enable peripheral clocks for all radio blocks via CMU.
    ///
    /// CRITICAL: EFR32 CMU register offsets differ from other ARM chips!
    /// From CMSIS efr32fg1v_cmu.h TypeDef struct:
    ///   OSCENCMD   = 0x060 (oscillator enable command)
    ///   HFCLKSEL   = 0x074 (HF clock source select)
    ///   STATUS     = 0x090 (clock status)
    ///   HFBUSCLKEN0 = 0x0B0 (bus clock enables)
    ///   HFPERCLKEN0 = 0x0C0 (peripheral clock enables)
    ///   RADIOCLKEN0 = 0x0C8 (radio peripheral clock enables!)
    fn enable_clocks(&self) {
        // 1. Enable HFXO (38.4 MHz crystal)
        //    OSCENCMD: bit 2 = HFXOEN (from CMSIS: _CMU_OSCENCMD_HFXOEN_SHIFT = 2)
        reg_write(CMU_OSCENCMD, 1 << 2);

        // 2. Wait for HFXO ready
        //    STATUS: bit 3 = HFXORDY (from CMSIS: _CMU_STATUS_HFXORDY_SHIFT = 3)
        let mut ready = false;
        for _ in 0..500_000u32 {
            if reg_read(CMU_STATUS) & (1 << 3) != 0 {
                ready = true;
                break;
            }
            core::hint::spin_loop();
        }
        if !ready {
            log::warn!("efr32: HFXO did not become ready!");
        }

        // 3. Switch HFCLK source to HFXO
        //    HFCLKSEL: write 2 = HFXO
        reg_write(CMU_HFCLKSEL, 2);

        // 4. Enable all radio peripheral clocks via RADIOCLKEN0 (0x0C8)
        //    Bits 0-8: PROTIMER, ?, CRC, FRC, RAC, MODEM, SYNTH, AGC, BUFC
        //    Set all 10 bits (0x3FF) to enable everything
        reg_write(CMU_RADIOCLKEN0, 0x3FF);

        // 5. Enable HFPERCLK for GPIO/Timer
        let val = reg_read(CMU_HFPERCLKEN0);
        reg_write(CMU_HFPERCLKEN0, val | 1);

        // 6. Enable HFBUSCLK for GPIO
        let val = reg_read(CMU_HFBUSCLKEN0);
        reg_write(CMU_HFBUSCLKEN0, val | (1 << 1)); // GPIO

        log::info!(
            "efr32: clocks enabled, STATUS={:#010X}, RADIOCLKEN0={:#010X}",
            reg_read(CMU_STATUS),
            reg_read(CMU_RADIOCLKEN0)
        );
    }

    /// Configure RAC (Radio Controller) for 802.15.4 operation.
    ///
    /// Register values from CMSIS efr32fg1v_rac.h and baremetal reference.
    /// Uses named register constants at verified CMSIS offsets.
    fn configure_rac(&self) {
        // Reset RAC to known state
        reg_write(RAC_CMD, RAC_CMD_TXDIS | RAC_CMD_RXDIS);

        // Wait for radio to reach idle/off state
        for _ in 0..10_000u32 {
            let state = (reg_read(RAC_STATUS) >> RAC_STATUS_STATE_SHIFT) & 0x0F;
            if state <= 1 {
                break;
            }
            core::hint::spin_loop();
        }

        // CTRL (0x00C): Enable active/PA/LNA polarities
        // CTRL (0x00C): ACTIVEPOL | PAENPOL | LNAENPOL
        // Bit 0 = FORCEDISABLE — must be 0! (was 1 = radio force-disabled!)
        reg_write(RAC_CTRL, 0x0000_0380);

        // SYNTHREGCTRL (0x09C): from reference dump = 0x0363_6D80
        reg_write(RAC_SYNTHREGCTRL, 0x0363_6D80);

        // VCOCTRL (0x0A0): from reference dump = 0x0F00_277A
        reg_write(RAC_VCOCTRL, 0x0F00_277A);

        // LNAMIXCTRL (0x0FC): from reference dump = 0x00000000
        reg_write(RAC_BASE + 0x0FC, 0x0000_0000);

        // LNAMIXCTRL1 (0x134): from reference dump = 0x00301F19
        reg_write(RAC_BASE + 0x134, 0x0030_1F19);

        // IFPGACTRL (0x138): from reference dump = 0x000087E6
        reg_write(RAC_IFPGACTRL, 0x0000_87E6);

        // IFPGACAL (0x13C): from reference dump = 0x000008E0
        reg_write(RAC_BASE + 0x13C, 0x0000_08E0);

        // IFFILTCTRL (0x140): from reference dump = 0x0088006D
        reg_write(RAC_IFFILTCTRL, 0x0088_006D);

        // IFADCCTRL (0x144): from reference dump = 0x115E6C0
        reg_write(RAC_IFADCCTRL, 0x1153_E6C0);

        // PA registers (0x100-0x150) — power amplifier configuration
        reg_write(RAC_BASE + 0x100, 0x0000_0004); // PACTRL0
        reg_write(RAC_BASE + 0x104, 0x0104_D700); // PAPKDCTRL
        reg_write(RAC_BASE + 0x108, 0x0040_0484); // PABIASCTRL0
        reg_write(RAC_BASE + 0x10C, 0x0002_4525); // PABIASCTRL1
        reg_write(RAC_BASE + 0x150, 0x001E_0044); // PACTUNECTRL

        // Sub-GHz PA registers (set to reference values)
        reg_write(RAC_BASE + 0x110, 0x0000_0000); // SGRFENCTRL0
        reg_write(RAC_BASE + 0x114, 0x0186_DB00); // SGLNAMIXCTRL
        reg_write(RAC_BASE + 0x118, 0x4000_0008); // SGPACTRL0
        reg_write(RAC_BASE + 0x11C, 0x0108_D700); // SGPAPKDCTRL
        reg_write(RAC_BASE + 0x120, 0x0700_0444); // SGPABIASCTRL0
        reg_write(RAC_BASE + 0x124, 0x0008_4523); // SGPABIASCTRL1

        // RFBIASCAL (0x130): from reference dump
        reg_write(RAC_BASE + 0x130, 0x0025_1504); // RFBIASCAL

        // RAC interrupt enable — the sequencer needs specific RAC interrupts
        reg_write(RAC_BASE + 0x020, 0x002C_0004); // IEN from reference

        // HFXO retiming control
        reg_write(RAC_BASE + 0x030, 0x0000_0760); // HFXORETIMECTRL

        // Additional RAC registers from reference dump
        reg_write(RAC_BASE + 0x048, 0x0000_008C); // R0
        reg_write(RAC_BASE + 0x054, 0x0000_0004); // R3
        reg_write(RAC_BASE + 0x06C, 0x0000_0001); // WAITMASK
        reg_write(RAC_BASE + 0x070, 0x0000_0001); // WAITSNSH

        // Clear force-disable if set
        let ctrl = reg_read(RAC_CTRL);
        if ctrl & (1 << 14) != 0 {
            reg_write(RAC_CTRL, ctrl & !(1 << 14));
        }

        // Clear pending IRQs
        reg_write(RAC_IFC, 0xFFFF_FFFF);
    }

    /// Configure BUFC (Buffer Controller) with RAM buffer addresses.
    ///
    /// BUFC requires static RAM buffers for TX (BUF0) and RX (BUF1).
    /// The buffer addresses and sizes must be set before any TX/RX.
    fn configure_bufc(&self) {
        // Buffer 0 = TX: 256 bytes
        let tx_addr = BUFC_TX_RAM.get() as u32;
        reg_write(BUFC_BUF0_CTRL, BUFC_BUFSIZE_256);
        reg_write(BUFC_BUF0_ADDR, tx_addr);
        reg_write(BUFC_BUF0_CMD, 1); // clear

        // Buffer 1 = RX: 256 bytes
        let rx_addr = BUFC_RX_RAM.get() as u32;
        reg_write(BUFC_BUF1_CTRL, BUFC_BUFSIZE_256);
        reg_write(BUFC_BUF1_ADDR, rx_addr);
        reg_write(BUFC_BUF1_CMD, 1); // clear

        // Buffer 2 = RX length: clear
        reg_write(_BUFC_BUF2_CMD, 1);

        log::info!(
            "efr32: BUFC configured, TX@{:#010X} RX@{:#010X}",
            tx_addr,
            rx_addr
        );
    }

    /// Configure CRC peripheral for IEEE 802.15.4 CRC-16-CCITT.
    ///
    /// CRC base = 0x40082000 (separate from BUFC at 0x40081000).
    /// The FRC uses this CRC engine when CALCCRC/INCLUDECRC are set in FCD.
    /// Without configuring CRC, all received frames fail CRC validation
    /// and are silently dropped by the FRC — no RXDONE interrupt fires.
    fn configure_crc(&self) {
        // Radio CRC peripheral at 0x40082000 — same IP block as GPCRC.
        // Register layout matches CMSIS GPCRC TypeDef (NOT the ad-hoc
        // offsets we had before — +0x010/+0x018 were WRONG and wrote
        // POLY/INIT to DATA/reserved instead of the real registers).
        const CRC_BASE: u32 = 0x4008_2000;
        const CRC_CTRL: u32 = CRC_BASE + 0x000;
        const CRC_CMD: u32  = CRC_BASE + 0x004;
        const CRC_INIT: u32 = CRC_BASE + 0x010; // confirmed from reference dump
        const CRC_POLY: u32 = CRC_BASE + 0x018; // confirmed: reference shows 0x8408 at +0x18

        // IEEE 802.15.4 CRC-16/CCITT:
        //   Polynomial: 0x1021 → bit-reflected for LSB-first = 0x8408
        //   Seed: 0x0000
        //   CTRL = 0x0704 (from reference firmware register dump):
        //     bits[10:8] = 7 → BITSPERWORD = 8 bits/word
        //     bit[2]     = 1 → CRCWIDTH encoding for 16-bit CRC
        reg_write(CRC_CTRL, 0x0000_0704);
        reg_write(CRC_INIT, 0x0000_0000);
        reg_write(CRC_POLY, 0x0000_8408);

        // CMD bit 0 = INIT → load seed value into CRC accumulator
        reg_write(CRC_CMD, 0x0000_0001);
    }

    /// Configure FRC (Frame Controller) for IEEE 802.15.4 frame format.
    ///
    /// Register offsets from CMSIS TypeDef. Values for 802.15.4:
    /// - CTRL: BITSPERWORD=7 (8 bits/word)
    /// - FCD0: TX descriptor — CALCCRC | INCLUDECRC, buffer 0, WORDS=0xFF
    /// - FCD2: RX descriptor — CALCCRC | INCLUDECRC, buffer 1, WORDS=0xFF
    fn configure_frc(&self) {
        // ALL values from the running RAIL reference firmware dump.

        // DFLCTRL at 0x004: Dynamic Frame Length control for RX
        // CRITICAL for RX: tells FRC how to extract frame length from PHR.
        // Reference value = 0x00148001 (SINGLEBYTE mode, 7-bit length field)
        // Without this, FRC can't delimit received frames!
        reg_write(FRC_DFLCTRL, 0x0014_8001);

        // MAXLENGTH at 0x008: max frame length for RX = 127 bytes (802.15.4)
        // Without this, FRC may reject or truncate received frames.
        reg_write(FRC_BASE + 0x008, 0x0000_007F);

        // CTRL at 0x040: reference = 0x7A0
        reg_write(FRC_CTRL, 0x0000_07A0);
        // RXCTRL at 0x044: reference = 0x68
        reg_write(FRC_RXCTRL, 0x0000_0068);
        // TRAILRXDATA at 0x04C: reference = 0x1B
        reg_write(FRC_TRAILRXDATA, 0x0000_001B);
        // FECCTRL at 0x034: reference = 0x0
        reg_write(FRC_FECCTRL, 0x0000_0000);
        // BLOCKRAMADDR at 0x038: clear
        reg_write(FRC_BLOCKRAMADDR, 0x0000_0000);
        // CONVRAMADDR at 0x03C: clear
        reg_write(FRC_CONVRAMADDR, 0x0000_0000);

        // Frame Control Descriptors from reference dump:
        // FCD0 (TX): 0x4000 = buffer=0, words=0, bit14=CALCCRC
        reg_write(FRC_FCD0, 0x0000_4000);
        // FCD1: 0x4CFF = buffer=0, words=0xFF, CALCCRC + INCLUDECRC
        reg_write(FRC_BASE + 0x0A4, 0x0000_4CFF);
        // FCD2 (RX): 0x4100 = buffer=1, words=0, bit14=CALCCRC
        reg_write(FRC_FCD2, 0x0000_4100);
        // FCD3: 0x4DFF = buffer=1, words=0xFF, CALCCRC + INCLUDECRC
        reg_write(FRC_BASE + 0x0AC, 0x0000_4DFF);

        // Clear all pending FRC interrupt flags
        reg_write(FRC_IFC, 0xFFFF_FFFF);

        // Enable FRC interrupts: TXDONE(0) | RXDONE(4) | FRAMEERROR(6) | RXOF(8)
        // RXOF must be enabled — the IRQ handler checks for it to signal
        // RX errors, but it won't fire unless enabled in IEN.
        reg_write(FRC_IEN, FRC_IF_TXDONE | FRC_IF_RXDONE | FRC_IF_FRAMEERROR | FRC_IF_RXOF);
    }

    /// Configure MODEM for IEEE 802.15.4 O-QPSK modulation at 250 kbps.
    ///
    /// Register values from a working RAIL-based firmware on EFR32MG1P.
    /// Configures O-QPSK with half-sine pulse shaping, 2 Mchip/s chip rate,
    /// 62.5 ksym/s symbol rate, DSSS spreading (32 chips/symbol).
    ///
    /// CMSIS: offsets 0x000–0x010 are read-only status registers (STATUS,
    /// TIMDETSTATUS, FREQOFFSET, AFCADJRX, AFCADJTX). We start writing
    /// at MIXCTRL (0x014) — the first read-write register.
    fn configure_modem(&self) {
        // MODEM writable registers from MIXCTRL (0x14) through ROUTELOC1 (0x78)
        // = 26 words. Values from RAIL reference firmware dump at 0x40086014.
        static MODEM_REGS: [u32; 26] = [
            0x0000_0010, // [0x14] MIXCTRL
            0x0413_F920, // [0x18] CTRL0
            0x0052_C007, // [0x1C] CTRL1
            0x0000_0000, // [0x20] CTRL2
            0x0000_0000, // [0x24] CTRL3
            0x0300_0000, // [0x28] CTRL4
            0x0000_0000, // [0x2C] CTRL5
            0x00FF_0264, // [0x30] TXBR
            0x0000_08A2, // [0x34] RXBR
            0x0000_0001, // [0x38] CF
            0x0008_07B0, // [0x3C] PRE
            0x0000_00A7, // [0x40] SYNC0 (802.15.4 SFD = 0xA7)
            0x0000_0000, // [0x44] SYNC1
            0x0AC0_0141, // [0x48] TIMING
            0x744A_C39B, // [0x4C] DSSS0
            0x0000_03F0, // [0x50] MODINDEX
            0x0000_0000, // [0x54] AFC
            0x0000_0000, // [0x58] AFCADJLIM (ref=0x0)
            0x3010_0101, // [0x5C] SHAPING0 (ref=0x30100101)
            0x7F7F_7050, // [0x60] SHAPING1 (ref=0x7F7F7050)
            0x0000_0000, // [0x64] SHAPING2 (ref=0x0)
            0x0000_0500, // [0x68] RAMPCTRL (ref=0x500)
            0x00F0_0000, // [0x6C] RAMPLEV
            0x0000_0000, // [0x70] ROUTEPEN
            0x0000_0000, // [0x74] ROUTELOC0
            0x0000_0000, // [0x78] ROUTELOC1
        ];

        for (i, &val) in MODEM_REGS.iter().enumerate() {
            reg_write(MODEM_BASE + 0x14 + (i as u32) * 4, val);
        }
    }

    /// Configure SYNTH (Frequency Synthesizer) for 2.4 GHz 802.15.4 channels.
    ///
    /// CMSIS layout: STATUS(0x00), CMD(0x04), CTRL(0x08), CALCTRL(0x0C),
    /// reserved(0x10-0x020), VCDACCTRL(0x24), reserved(0x28),
    /// FREQ(0x2C), IFFREQ(0x30), DIVCTRL(0x34), CHCTRL(0x38), CHSP(0x3C).
    ///
    /// Base frequency: 2405 MHz (channel 11), channel spacing: 5 MHz.
    /// Channels 11–26 → 2405–2480 MHz.
    fn configure_synth(&self) {
        // All values from a working RAIL-based 802.15.4 firmware dump.
        //
        // CRITICAL: RAIL programs SYNTH_FREQ directly for each channel,
        // NOT using CHCTRL/CHSP. FREQ encodes the full VCO frequency.
        // Formula: FREQ = freq_hz * lodiv * 524288 / HFXO_hz

        // SYNTH_CTRL: dithering and lock threshold (from reference)
        reg_write(SYNTH_CTRL, 0x0000_AC3F);

        // SYNTH_CALCTRL (0x00C): calibration control
        reg_write(SYNTH_BASE + 0x00C, 0x0004_2801);

        // SYNTH_VCDACCTRL (0x024)
        reg_write(SYNTH_BASE + 0x024, 0x0000_0023);

        // SYNTH_FREQ: initial = channel 11 = 2405 MHz
        // 2405e6 * 1 * 524288 / 38400000 = 32836267 = 0x01F50AAB
        reg_write(SYNTH_FREQ, 0x01F5_0AAB);

        // SYNTH_IFFREQ: IF frequency + LOSIDE bit (from reference dump)
        // Ref value: 0x00104924 (LOSIDE=1, IFFREQ=0x4924)
        reg_write(SYNTH_IFFREQ, 0x0010_4924);

        // SYNTH_DIVCTRL: lodiv=1 for 2.4 GHz
        // Ref value: 0x01 (divC=1, divA=divB=0 → lodiv=1)
        reg_write(SYNTH_DIVCTRL, 0x0000_0001);

        // SYNTH_CHCTRL: channel 0 (will be set by set_channel)
        reg_write(SYNTH_CHCTRL, 0);

        // SYNTH_CHSP: channel spacing (15 from reference — RAIL uses direct FREQ)
        reg_write(SYNTH_CHSP, 0x0000_000F);

        // SYNTH_VCOGAIN (0x050)
        reg_write(SYNTH_BASE + 0x050, 0x0000_0029);
    }

    /// Configure AGC (Automatic Gain Control) for 802.15.4 reception.
    ///
    /// All values from the working RAIL firmware dump.
    /// CMSIS: STATUS0-FRAMERSSI (0x00-0x0C) are RO, CTRL0 starts at 0x14.
    fn configure_agc(&self) {
        // Values directly from reference firmware register dump
        reg_write(AGC_BASE + 0x14, 0x0000_E0FA); // CTRL0
        reg_write(AGC_BASE + 0x18, 0x0000_18E7); // CTRL1
        reg_write(AGC_BASE + 0x1C, 0x8284_0000); // CTRL2
        reg_write(AGC_BASE + 0x20, 0x0000_0000); // RSSISTEPTHR (ref=0)
        reg_write(AGC_BASE + 0x24, 0x0000_0082); // IFPEAKDET
        reg_write(AGC_BASE + 0x28, 0x0180_0000); // MANGAIN

        // Gain table entries (from reference dump at 0x48-0x78)
        reg_write(AGC_BASE + 0x48, 0x0000_3D3C); // GAINRANGE
        reg_write(AGC_BASE + 0x4C, 0x0000_19BC); // GAININDEX
        reg_write(AGC_BASE + 0x50, 0x0CA8_6543); // SLICECODE
        reg_write(AGC_BASE + 0x54, 0x0654_3210); // ATTENCODE1
        reg_write(AGC_BASE + 0x58, 0x18B5_2507); // ATTENCODE2
        reg_write(AGC_BASE + 0x5C, 0x2518_3DCD); // ATTENCODE3
        reg_write(AGC_BASE + 0x60, 0x0000_0000); // GAINERROR1
        reg_write(AGC_BASE + 0x64, 0x0000_0000); // GAINERROR2
        reg_write(AGC_BASE + 0x68, 0x0000_0000); // GAINERROR3
        reg_write(AGC_BASE + 0x70, 0x0001_0103); // GAINSTEPLIM
        reg_write(AGC_BASE + 0x74, 0x0000_0442); // LOOPDEL
        reg_write(AGC_BASE + 0x78, 0x0055_2300); // MININDEX
    }

    /// Set TX power via RAC PA registers.
    ///
    /// Reference firmware values at RAC 0x100-0x15C for PA configuration.
    fn set_tx_power(&self, _dbm: i8) {
        // PA registers from reference dump (RAC_BASE + 0x100+)
        // PACTRL0: configures PA slices and power level
        reg_write(RAC_BASE + 0x100, 0x0000_0004); // PACTRL0
        reg_write(RAC_BASE + 0x104, 0x0104_D700); // PAPKDCTRL
        reg_write(RAC_BASE + 0x108, 0x0040_0484); // PABIASCTRL0
        reg_write(RAC_BASE + 0x10C, 0x0002_4525); // PABIASCTRL1
        // PACTUNECTRL (0x150): PA tuning
        reg_write(RAC_BASE + 0x150, 0x001E_0044); // CTune TX/RX
    }

    /// Set RF channel for IEEE 802.15.4.
    ///
    /// Channels 11–26 map to center frequencies 2405–2480 MHz (5 MHz spacing).
    /// Programs SYNTH_FREQ directly (same approach as RAIL library).
    fn set_channel(&self, channel: u8) {
        let ch = channel.clamp(11, 26);
        let freq_mhz = 2405u32 + (ch as u32 - 11) * 5;

        // FREQ = freq_hz * lodiv * 524288 / HFXO_hz
        // lodiv=1, HFXO=38400000
        // FREQ = freq_mhz * 1000000 * 524288 / 38400000
        //      = freq_mhz * 524288000 / 38400
        //      = freq_mhz * 13652 + freq_mhz * 8 / 38400 * 1000000
        // Simplified: FREQ = freq_mhz * 13653 + (freq_mhz * 2048 / 75)
        // Actually just use 64-bit math:
        let freq_reg = (freq_mhz as u64 * 1_000_000 * 524_288 / 38_400_000) as u32;
        reg_write(SYNTH_FREQ, freq_reg);

        // Short delay for PLL lock (~50µs)
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
    ///
    /// TX flow (from working baremetal code):
    ///   1. Clear BUFC buffer 0 (TX)
    ///   2. Write frame bytes to BUF0_WRITEDATA (0x018)
    ///   3. Set FRC_WCNTCMP0 = payload_bytes - 1
    ///   4. RAC_CMD = RAC_CMD_TXEN → RAC sequencer starts TX
    ///   5. Wait for FRC_IF_TXDONE (bit 0 at FRC+0x060)
    pub async fn transmit(&mut self, frame: &[u8]) -> Result<(), RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }
        if frame.is_empty() || frame.len() > MAX_FRAME_LEN {
            return Err(RadioError::InvalidFrame);
        }

        TX_DONE.reset();

        // Debug: increment atomic TX call counter
        static TX_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
        TX_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);

        // NOTE: Do NOT send RXDIS before TXEN — the baremetal reference
        // writes TXEN directly while RX is active. The sequencer handles
        // the RX→TX transition internally.

        // Clear TX buffer (BUF0_CMD bit 0 = CLEAR)
        reg_write(BUFC_BUF0_CMD, 1);

        // IEEE 802.15.4 PPDU format:
        //   Preamble (4 bytes) + SFD (1 byte) — handled by MODEM
        //   PHR (1 byte) = PSDU length (frame + 2-byte FCS)
        //   PSDU = frame bytes
        //   FCS (2 bytes) — appended by FRC hardware (INCLUDECRC in FCD1)
        //
        // We write: PHR + frame bytes to BUFC.
        // FRC appends the 2-byte CRC automatically.
        // PHR value = frame.len() + 2 (includes the FCS that FRC will add)
        let phr = (frame.len() + 2) as u8;
        reg_write(BUFC_BUF0_WRITEDATA, phr as u32);
        for &b in frame {
            reg_write(BUFC_BUF0_WRITEDATA, b as u32);
        }

        // FRC_WCNTCMP0 = number of PAYLOAD bytes - 1 (NOT including PHR)
        // The baremetal code: FRC->WCNTCMP0 = BUFC_TxBufferBytesAvailable() - 1
        // which counts all bytes in the buffer (PHR + frame) minus 1.
        // With two-subframe FCD: FCD0 handles first subframe (PHR, WORDS=0),
        // FCD1 handles second subframe (payload, WORDS=0xFF).
        // WCNTCMP0 counts TOTAL written bytes - 1.
        reg_write(FRC_WCNTCMP0, (frame.len()) as u32); // PHR(1) + frame - 1

        // Clear pending FRC TX-done flags before starting
        reg_write(FRC_IFC, 0xFFFF_FFFF);

        // Critical sequencer steps (from baremetal reference):
        // 1. Clear "radio disabled" flag in sequencer control
        let seq_ctrl = reg_read(SEQ_CONTROL_REG);
        reg_write(SEQ_CONTROL_REG, seq_ctrl & !0x20);
        // 2. Set IFPGA band select for 2.4 GHz
        let ifpga = reg_read(RAC_IFPGACTRL);
        reg_write(RAC_IFPGACTRL, ifpga | (1 << 16)); // BANDSEL

        // Start TX via RAC — the sequencer handles SYNTH cal, PA, FRC
        reg_write(RAC_CMD, RAC_CMD_TXEN);

        // Debug: wait a bit longer and check if TXDONE fires
        for _ in 0..10_000u32 { core::hint::spin_loop(); }
        let rac_st = reg_read(RAC_STATUS);
        let frc_if = reg_read(FRC_IF);
        let rac_state = (rac_st >> 24) & 0x0F;
        rtt_target::rprintln!("  RAC_st={} FRC_IF={:#X} FREQ={:#X}",
            rac_state, frc_if, reg_read(SYNTH_FREQ));

        log::trace!(
            "efr32: tx {} bytes on ch{}",
            frame.len(),
            self.config.channel
        );

        // Wait for TX completion with async timeout (10ms should be plenty)
        let result = embassy_futures::select::select(
            TX_DONE.wait(),
            embassy_time::Timer::after(embassy_time::Duration::from_millis(10)),
        ).await;

        match result {
            embassy_futures::select::Either::First(ok) => {
                if ok { Ok(()) } else { Err(RadioError::HardwareError) }
            }
            embassy_futures::select::Either::Second(_) => {
                reg_write(RAC_CMD, RAC_CMD_TXDIS);
                Err(RadioError::HardwareError)
            }
        }
    }

    /// Receive the next IEEE 802.15.4 frame (async).
    ///
    /// Enables the receiver and waits for a frame. Returns the frame data
    /// with RSSI and LQI. Frames failing CRC are rejected.
    ///
    /// RX flow (from working baremetal code):
    ///   1. Clear BUFC buffer 1 (RX) and buffer 2 (RX length)
    ///   2. RXENSRCEN = 0x02 → software RX enable
    ///   3. FRC captures frame, checks CRC
    ///   4. Data appears in BUF1, length in BUF1_STATUS
    ///   5. FRC_IF_RXDONE (bit 4) signals completion
    pub async fn receive(&mut self) -> Result<RxFrame, RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }

        RX_DONE.reset();

        // Abort any in-progress RX before clearing buffers.
        // After TX, the sequencer auto-transitions to RX (SEQ_TRANSITIONS=0x0202).
        // If we clear BUF1 while FRC is writing to it, we corrupt state.
        // FRC_CMD bit 0 = RXABORT.
        reg_write(FRC_CMD, FRC_CMD_RXABORT);

        // Clear RX buffer (BUF1) and length buffer (BUF2)
        reg_write(BUFC_BUF1_CMD, 1);
        reg_write(_BUFC_BUF2_CMD, 1);

        // Clear pending FRC and RAC RX flags
        reg_write(FRC_IFC, FRC_IF_RXDONE | FRC_IF_RXOF | FRC_IF_FRAMEERROR);
        reg_write(RAC_IFC, 0xFFFF_FFFF);

        // Clear sequencer disable flag and set band select (same as TX)
        let seq_ctrl = reg_read(SEQ_CONTROL_REG);
        reg_write(SEQ_CONTROL_REG, seq_ctrl & !0x20);
        reg_write(RAC_IFPGACTRL, 0x0000_87F6);

        // Start RX via RXENSRCEN (software RX enable, as the baremetal does)
        reg_write(_RAC_RXENSRCEN, 0x02);

        // Debug: verify radio enters RX state
        for _ in 0..5_000u32 { core::hint::spin_loop(); }
        let rac_state = (reg_read(RAC_STATUS) >> 24) & 0x0F;
        if rac_state != 2 {
            rtt_target::rprintln!("  RX: state={} (expected 2)", rac_state);
        }

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
        // Start RX via RXENSRCEN (RAC_CMD has no RXEN bit)
        reg_write(_RAC_RXENSRCEN, 0x02);

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
        // Clear RX buffer (BUF1) and length buffer (BUF2)
        reg_write(BUFC_BUF1_CMD, 1);
        reg_write(_BUFC_BUF2_CMD, 1);
        reg_write(FRC_IFC, FRC_IF_RXDONE | FRC_IF_RXOF | FRC_IF_FRAMEERROR);
        reg_write(RAC_IFC, 0xFFFF_FFFF);
        // Start RX via RXENSRCEN (RAC_CMD has no RXEN bit)
        reg_write(_RAC_RXENSRCEN, 0x02);
    }

    /// Disable the receiver.
    pub fn disable_rx(&self) {
        reg_write(RAC_CMD, RAC_CMD_RXDIS);
        reg_write(FRC_IFC, FRC_IF_RXDONE | FRC_IF_RXOF | FRC_IF_FRAMEERROR);
        reg_write(RAC_IFC, 0xFFFF_FFFF);
    }

    /// Power down the radio to save power between TX/RX cycles.
    ///
    /// Disables radio peripheral clocks via CMU. Saves ~5–10 mA on EFR32MG1P.
    /// Call `radio_wake()` before the next TX or RX operation.
    pub fn radio_sleep(&self) {
        reg_write(RAC_CMD, RAC_CMD_TXDIS | RAC_CMD_RXDIS);
        reg_write(RAC_IFC, 0xFFFF_FFFF);
        // Disable all radio peripheral clocks via RADIOCLKEN0
        reg_write(CMU_RADIOCLKEN0, 0);
    }

    /// Re-enable radio after `radio_sleep()`.
    pub fn radio_wake(&mut self) {
        // Re-enable all radio peripheral clocks
        reg_write(CMU_RADIOCLKEN0, 0x3FF);
        for _ in 0..1_000u32 { core::hint::spin_loop(); }
        self.set_channel(self.config.channel);
    }
}

// ── IRQ handler ─────────────────────────────────────────────────

/// RAC/FRC interrupt handler for EFR32MG1P radio.
///
/// Reads IRQ status from both RAC and FRC, processes TX/RX completion,
/// and signals the async driver via Embassy Signal.
///
/// TX data was written to BUFC BUF0; RX data arrives in BUFC BUF1.
/// FRC_IF_TXDONE = bit 0, FRC_IF_RXDONE = bit 4.
#[unsafe(no_mangle)]
pub extern "C" fn FRC_PRI() {
    let rac_flags = reg_read(RAC_IF);
    let frc_flags = reg_read(FRC_IF);

    // Clear all pending IRQs
    reg_write(RAC_IFC, rac_flags);
    reg_write(FRC_IFC, frc_flags);

    // TX completion (FRC_IF_TXDONE = bit 0)
    if frc_flags & FRC_IF_TXDONE != 0 {
        TX_DONE.signal(true);
    }

    // RX completion (FRC_IF_RXDONE = bit 4)
    let frc_rx_done = frc_flags & FRC_IF_RXDONE != 0;
    let rac_rx_done = false; // RAC has no RXDONE flag

    if frc_rx_done || rac_rx_done {
        // FRC_IF_RXDONE signals good CRC
        let crc_ok = frc_rx_done;
        RX_CRC_OK.store(crc_ok, Ordering::Release);

        if crc_ok {
            // Read byte count from BUFC BUF1 (RX) status, bits [12:0]
            let status = reg_read(BUFC_BUF1_STATUS);
            let len = ((status & 0x1FFF) as usize).min(MAX_FRAME_LEN);

            RX_LEN.store(len as u8, Ordering::Release);

            // Read RSSI from AGC
            let rssi_raw = reg_read(AGC_RSSI);
            let rssi = (rssi_raw & 0xFF) as i8;
            RX_RSSI.store(rssi, Ordering::Release);

            // Read frame data byte-by-byte from BUFC BUF1 (RX buffer)
            unsafe {
                let buf = &mut *RX_BUF.get();
                for i in 0..len {
                    buf[i] = (reg_read(BUFC_BUF1_READDATA) & 0xFF) as u8;
                }
            }
        } else {
            RX_LEN.store(0, Ordering::Release);
        }

        RX_DONE.signal(());
    }

    // RX overflow (FRC_IF_RXOF = bit 8) or frame error (bit 6)
    if frc_flags & (FRC_IF_RXOF | FRC_IF_FRAMEERROR) != 0 {
        RX_CRC_OK.store(false, Ordering::Release);
        RX_LEN.store(0, Ordering::Release);
        RX_DONE.signal(());
    }
}

// ── Additional IRQ Handlers ─────────────────────────────────────

/// Regular FRC interrupt handler (IRQ 3) — same logic as FRC_PRI.
#[unsafe(no_mangle)]
pub extern "C" fn FRC() {
    FRC_PRI();
}

/// RAC sequencer interrupt handler (IRQ 5).
#[unsafe(no_mangle)]
pub extern "C" fn RAC_SEQ() {
    let flags = reg_read(RAC_IF);
    reg_write(RAC_IFC, flags);
}

/// RAC state machine interrupt handler (IRQ 6).
#[unsafe(no_mangle)]
pub extern "C" fn RAC_RSM() {
    let flags = reg_read(RAC_IF);
    reg_write(RAC_IFC, flags);
}

/// BUFC interrupt handler (IRQ 7).
#[unsafe(no_mangle)]
pub extern "C" fn BUFC() {
    let flags = reg_read(BUFC_IF);
    reg_write(BUFC_IFC, flags);
}

// ── Utility ─────────────────────────────────────────────────────

/// Convert RSSI to LQI (Link Quality Indicator, 0–255).
fn rssi_to_lqi(rssi: i8) -> u8 {
    // Simple linear mapping: -100 dBm → 0, -20 dBm → 255
    let clamped = (rssi as i16).clamp(-100, -20);
    (((clamped + 100) as u16) * 255 / 80) as u8
}
