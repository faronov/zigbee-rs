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
const RAC_CMD_RXEN: u32 = 1 << 1;
const RAC_CMD_TXDIS: u32 = 1 << 2;
const RAC_CMD_RXDIS: u32 = 1 << 3;
const _RAC_CMD_CLEARRXOVERFLOW: u32 = 1 << 6;

// ── RAC Status Bits ─────────────────────────────────────────────

const RAC_STATUS_STATE_MASK: u32 = 0x0F;
const _RAC_STATE_OFF: u32 = 0x00;
const _RAC_STATE_IDLE: u32 = 0x01;
const RAC_STATE_RX: u32 = 0x02;
const _RAC_STATE_TX: u32 = 0x03;

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

// ── RAC IRQ Bits ────────────────────────────────────────────────

const RAC_IF_TXDONE: u32 = 1 << 0;
const RAC_IF_RXDONE: u32 = 1 << 1;
const RAC_IF_RXOF: u32 = 1 << 2;

// ── FRC Register Offsets (from CMSIS TypeDef at 0x40080000) ─────

/// FRC status register (read-only).
const _FRC_STATUS: u32 = FRC_BASE + 0x000;
/// FRC data filter control.
const _FRC_DFLCTRL: u32 = FRC_BASE + 0x004;
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
const _FRC_CMD: u32 = FRC_BASE + 0x024;
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
const _FRC_CMD_RXABORT: u32 = 1 << 0;
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

/// TX RAM buffer pointed to by BUFC BUF0_ADDR.
static BUFC_TX_RAM: SyncUnsafeCell<[u8; 256]> = SyncUnsafeCell::new([0u8; 256]);
/// RX RAM buffer pointed to by BUFC BUF1_ADDR.
static BUFC_RX_RAM: SyncUnsafeCell<[u8; 256]> = SyncUnsafeCell::new([0u8; 256]);

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

    /// Full radio initialization: clocks → RAC → BUFC → FRC → MODEM → SYNTH → AGC.
    fn init_hardware(&mut self) {
        self.enable_clocks();
        self.load_rac_sequences();
        self.configure_rac();
        self.configure_bufc();
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
    /// the TX/RX state machine programs. The sequencer is essential —
    /// RAC_CMD_TXEN/RXEN trigger the sequencer to coordinate SYNTH
    /// calibration, PA enable, FRC TX/RX, and return to idle.
    fn load_rac_sequences(&self) {
        // 1. Halt the sequencer before loading
        reg_write(RAC_SEQCMD, RAC_SEQCMD_HALT);

        // 2. Clear sequencer RAM (4KB at 0x21000000)
        let dst_base = 0x2100_0000u32;
        for i in 0..1024u32 {
            reg_write(dst_base + i * 4, 0);
        }

        // 3. Set vector address BEFORE loading code
        reg_write(RAC_VECTADDR, dst_base);

        // 4. Enable compact mode
        reg_write(RAC_SEQCTRL, 1); // COMPACT = bit 0

        // 5. Load microcode
        let seq_data = &super::rac_seq::RAC_SEQ_DATA;
        for (i, &word) in seq_data.iter().enumerate() {
            let addr = dst_base + (i as u32) * 4;
            reg_write(addr, word);
        }

        // 6. Initialize sequencer variables in RAM
        // From VDowbensky/efr32_baremetal generic_seq.h:
        //   SEQ_CONTROL_REG    = 0x21000EFC
        //   RADIO_TRANSITIONS  = 0x21000F00 (SEQ->REG000)
        //   RX_WARMTIME        = 0x21000F9C (SEQ->REG09C)
        //   TX_WARMTIME        = 0x21000FAC (SEQ->REG0AC)
        //   TX_RX_TIME etc.    = various
        //   PHYINFO            = 0x21000FF0
        //   SYNTHLPFCTRLRX     = 0x21000FF8
        //   SYNTHLPFCTRLTX     = 0x21000FFC

        // SEQ_CONTROL_REG: bit 3 = enable, bit 5 = disable flag
        reg_write(SEQ_CONTROL_REG, 0x08); // Enable sequencer

        // Warm-up times (in µs, from baremetal: all set to 100)
        reg_write(SEQ_RX_WARMTIME, 100);
        reg_write(SEQ_TX_WARMTIME, 100);
        reg_write(SEQ_RX_TX_TIME, 100);
        reg_write(SEQ_TX_RX_TIME, 100);
        reg_write(SEQ_TX_TX_TIME, 100);
        reg_write(SEQ_RXFRAME_TX_TIME, 100);
        reg_write(SEQ_RX_SEARCHTIME, 0);
        reg_write(SEQ_TX_RX_SEARCHTIME, 0);

        // SYNTH LPF control for RX and TX
        reg_write(SEQ_SYNTHLPFCTRLRX, 0x0003_C002);
        reg_write(SEQ_SYNTHLPFCTRLTX, 0x0003_C002);

        // R6 pointer (used by sequencer for state management)
        reg_write(_RAC_R6, 0x21000FCC);

        // Clear sequencer scratch registers
        reg_write(_RAC_SR0, 0);
        reg_write(_RAC_SR1, 0);
        reg_write(_RAC_SR2, 0);
        reg_write(_RAC_SR3, 0);

        // 7. Resume sequencer
        reg_write(RAC_SEQCMD, RAC_SEQCMD_RESUME);

        // Enable RXENSRCEN software RX enable
        reg_write(_RAC_RXENSRCEN, 0x02);

        log::info!(
            "efr32: RAC seq loaded {} words, SEQSTATUS={:#X}",
            seq_data.len(),
            reg_read(RAC_SEQSTATUS)
        );
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
            let state = reg_read(RAC_STATUS) & RAC_STATUS_STATE_MASK;
            if state <= 1 {
                break;
            }
            core::hint::spin_loop();
        }

        // CTRL (0x00C): Enable active/PA/LNA polarities
        // From baremetal: ACTIVEPOL | PAENPOL | LNAENPOL = 0x381
        reg_write(RAC_CTRL, 0x0000_0381);

        // SYNTHREGCTRL (0x09C): voltage regulator trims for synth
        // From baremetal: CHPLDOVREFTRIM=3, CHPLDOAMPCURR=3, etc.
        reg_write(RAC_SYNTHREGCTRL, 0x0000_0FFF);

        // VCOCTRL (0x0A0): VCO control
        reg_write(RAC_VCOCTRL, 0x0F00_277A);

        // IFPGACTRL (0x138): IF PGA control — band select + gain
        reg_write(RAC_IFPGACTRL, 0x0000_87F6);

        // IFFILTCTRL (0x140): IF filter control
        reg_write(RAC_IFFILTCTRL, 0x0088_00E0);

        // IFADCCTRL (0x144): IF ADC control
        reg_write(RAC_IFADCCTRL, 0x1153_E6C0);

        // Clear force-disable if set
        let ctrl = reg_read(RAC_CTRL);
        if ctrl & (1 << 14) != 0 { // FORCEDISABLE bit
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

    /// Configure FRC (Frame Controller) for IEEE 802.15.4 frame format.
    ///
    /// Register offsets from CMSIS TypeDef. Values for 802.15.4:
    /// - CTRL: BITSPERWORD=7 (8 bits/word)
    /// - FCD0: TX descriptor — CALCCRC | INCLUDECRC, buffer 0, WORDS=0xFF
    /// - FCD2: RX descriptor — CALCCRC | INCLUDECRC, buffer 1, WORDS=0xFF
    fn configure_frc(&self) {
        // CTRL at 0x040: bits[10:8]=BITSPERWORD=7 → 0x700
        reg_write(FRC_CTRL, 0x0000_0700);
        // RXCTRL at 0x044
        reg_write(FRC_RXCTRL, 0x0014_8001);
        // TRAILRXDATA at 0x04C
        reg_write(FRC_TRAILRXDATA, 0x0000_001B);
        // FECCTRL at 0x034: BLOCKWHITEMODE = 1
        reg_write(FRC_FECCTRL, 0x0000_0001);
        // BLOCKRAMADDR at 0x038
        reg_write(FRC_BLOCKRAMADDR, 0x0000_001B);
        // CONVRAMADDR at 0x03C
        reg_write(FRC_CONVRAMADDR, 0x0000_A002);

        // Frame Control Descriptors:
        // FCD0 (TX): CALCCRC=1 | INCLUDECRC=1 | buffer=0 | WORDS=0xFF → 0x0CFF
        reg_write(FRC_FCD0, 0x0000_0CFF);
        // FCD2 (RX): CALCCRC=1 | INCLUDECRC=1 | buffer=1 | WORDS=0xFF → 0x0DFF
        reg_write(FRC_FCD2, 0x0000_0DFF);

        // Clear all pending FRC interrupt flags
        reg_write(FRC_IFC, 0xFFFF_FFFF);

        // Enable FRC interrupts: TXDONE(0) | RXDONE(4) | FRAMEERROR(6) = 0x51
        reg_write(FRC_IEN, FRC_IF_TXDONE | FRC_IF_RXDONE | FRC_IF_FRAMEERROR);
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

        // Ensure radio is idle before TX
        reg_write(RAC_CMD, RAC_CMD_RXDIS);
        for _ in 0..1_000u32 {
            let state = reg_read(RAC_STATUS) & RAC_STATUS_STATE_MASK;
            if state != RAC_STATE_RX {
                break;
            }
            core::hint::spin_loop();
        }

        // Clear TX buffer (BUF0_CMD bit 0 = CLEAR)
        reg_write(BUFC_BUF0_CMD, 1);

        // Write frame bytes to TX buffer one by one via BUF0_WRITEDATA.
        // The PHR (length byte) is written first, then the PSDU payload.
        let phr = (frame.len() + 2) as u8; // +2 for FCS appended by hardware
        reg_write(BUFC_BUF0_WRITEDATA, phr as u32);
        for &b in frame {
            reg_write(BUFC_BUF0_WRITEDATA, b as u32);
        }

        // Tell FRC how many payload bytes to transmit (PHR + frame bytes)
        let total_bytes = 1 + frame.len(); // 1 for PHR + payload
        reg_write(FRC_WCNTCMP0, (total_bytes - 1) as u32);

        // Clear pending FRC and RAC TX-done flags before starting
        reg_write(FRC_IFC, FRC_IF_TXDONE);
        reg_write(RAC_IFC, RAC_IF_TXDONE);

        // Critical sequencer steps (from baremetal reference):
        // 1. Clear "radio disabled" flag in sequencer control
        let seq_ctrl = reg_read(SEQ_CONTROL_REG);
        reg_write(SEQ_CONTROL_REG, seq_ctrl & !0x20);
        // 2. Set IFPGA band select for 2.4 GHz
        let ifpga = reg_read(RAC_IFPGACTRL);
        reg_write(RAC_IFPGACTRL, ifpga | (1 << 16)); // BANDSEL

        // Start TX via RAC — the sequencer handles SYNTH cal, PA, FRC
        reg_write(RAC_CMD, RAC_CMD_TXEN);

        // Debug: capture RAC STATUS and FRC IF right after TXEN
        static TX_RAC_STATUS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
        static TX_FRC_IF: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
        TX_RAC_STATUS.store(reg_read(RAC_STATUS), core::sync::atomic::Ordering::Relaxed);
        TX_FRC_IF.store(reg_read(FRC_IF), core::sync::atomic::Ordering::Relaxed);

        log::trace!(
            "efr32: tx {} bytes on ch{}",
            frame.len(),
            self.config.channel
        );

        // Wait for TX completion with timeout (poll-based, ~100ms max)
        let mut timeout_loops = 0u32;
        loop {
            // Check if TX_DONE was signaled
            if let Some(ok) = TX_DONE.try_take() {
                return if ok { Ok(()) } else { Err(RadioError::HardwareError) };
            }
            timeout_loops += 1;
            if timeout_loops > 1_000_000 {
                // TX timed out — abort and return error
                reg_write(RAC_CMD, RAC_CMD_TXDIS);
                log::warn!("efr32: TX timeout, RAC={:#X}", reg_read(RAC_STATUS));
                return Err(RadioError::HardwareError);
            }
            core::hint::spin_loop();
        }
    }

    /// Receive the next IEEE 802.15.4 frame (async).
    ///
    /// Enables the receiver and waits for a frame. Returns the frame data
    /// with RSSI and LQI. Frames failing CRC are rejected.
    ///
    /// RX flow (from working baremetal code):
    ///   1. Clear BUFC buffer 1 (RX) and buffer 2 (RX length)
    ///   2. RAC_CMD = RAC_CMD_RXEN → sequencer starts RX
    ///   3. FRC captures frame, checks CRC
    ///   4. Data appears in BUF1, length in BUF1_STATUS
    ///   5. FRC_IF_RXDONE (bit 4) signals completion
    pub async fn receive(&mut self) -> Result<RxFrame, RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }

        RX_DONE.reset();

        // Clear RX buffer (BUF1) and length buffer (BUF2)
        reg_write(BUFC_BUF1_CMD, 1);
        reg_write(_BUFC_BUF2_CMD, 1);

        // Clear pending FRC and RAC RX flags
        reg_write(FRC_IFC, FRC_IF_RXDONE | FRC_IF_RXOF | FRC_IF_FRAMEERROR);
        reg_write(RAC_IFC, RAC_IF_RXDONE | RAC_IF_RXOF);

        // Clear sequencer disable flag and set band select (same as TX)
        let seq_ctrl = reg_read(SEQ_CONTROL_REG);
        reg_write(SEQ_CONTROL_REG, seq_ctrl & !0x20);
        reg_write(RAC_IFPGACTRL, 0x0000_87F6);

        // Start RX via RXENSRCEN (software RX enable, as the baremetal does)
        reg_write(_RAC_RXENSRCEN, 0x02);

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
        // Clear RX buffer (BUF1) and length buffer (BUF2)
        reg_write(BUFC_BUF1_CMD, 1);
        reg_write(_BUFC_BUF2_CMD, 1);
        reg_write(FRC_IFC, FRC_IF_RXDONE | FRC_IF_RXOF | FRC_IF_FRAMEERROR);
        reg_write(RAC_IFC, RAC_IF_RXDONE | RAC_IF_RXOF);
        reg_write(RAC_CMD, RAC_CMD_RXEN);
    }

    /// Disable the receiver.
    pub fn disable_rx(&self) {
        reg_write(RAC_CMD, RAC_CMD_RXDIS);
        reg_write(FRC_IFC, FRC_IF_RXDONE | FRC_IF_RXOF | FRC_IF_FRAMEERROR);
        reg_write(RAC_IFC, RAC_IF_RXDONE | RAC_IF_RXOF);
    }

    /// Power down the radio to save power between TX/RX cycles.
    ///
    /// Disables radio peripheral clocks via CMU. Saves ~5–10 mA on EFR32MG1P.
    /// Call `radio_wake()` before the next TX or RX operation.
    pub fn radio_sleep(&self) {
        reg_write(RAC_CMD, RAC_CMD_TXDIS | RAC_CMD_RXDIS);
        reg_write(RAC_IFC, RAC_IF_TXDONE | RAC_IF_RXDONE | RAC_IF_RXOF);
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
    if frc_flags & FRC_IF_TXDONE != 0 || rac_flags & RAC_IF_TXDONE != 0 {
        TX_DONE.signal(true);
    }

    // RX completion (FRC_IF_RXDONE = bit 4)
    let frc_rx_done = frc_flags & FRC_IF_RXDONE != 0;
    let rac_rx_done = rac_flags & RAC_IF_RXDONE != 0;

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
    if frc_flags & (FRC_IF_RXOF | FRC_IF_FRAMEERROR) != 0 || rac_flags & RAC_IF_RXOF != 0 {
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
