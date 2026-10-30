//! Pure-Rust IEEE 802.15.4 radio driver for Phyplus PHY6222/PHY6252.
//!
//! The PHY62x2 is an ARM Cortex-M0 BLE SoC whose radio also supports IEEE
//! 802.15.4 O-QPSK (250 kbps, `PKT_FMT_ZIGBEE`). This driver configures the
//! radio entirely through memory-mapped registers; no vendor library or ROM
//! radio routine is called.
//!
//! # Register-level provenance
//! Analog/baseband register values are transcribed from the PHYPLUS PHY62xx
//! SDK `rf_phy_driver.c` as redistributed in pvvx/THB2
//! (`bthome_phy6222/SDK/lib/ble_controller/rf_phy_driver.c`) and
//! pvvx/phy62x2-SDK (`lib/rf/rf_phy_driver.c`); LL-HW engine access follows
//! `ll_hw_drv.c` from the same trees. Where the two SDK revisions differ the
//! newer phy62x2-SDK value is used and the difference is noted inline. The
//! SDK only exercises the Zigbee packet format from its direct-test-mode
//! (DTM) code, so the MAC-level use here (single TX / single RX with
//! software ACKs) is this crate's own composition of those primitives.
//!
//! # Hardware validation
//! The radio path is validated on a PB-03F (PHY6252) against an independent
//! 802.15.4 sniffer; see `docs/book/src/platform-guides/phy6222.md` for the
//! current validation level. Low-power operation and CCA/ED accuracy are not
//! hardware-proven.
//!
//! # Architecture
//! ```text
//! Phy6222Driver (pure Rust, async)
//!   ├── RF PHY / BB registers (0x4003_0000..0x4003_0130)
//!   │     ├── ana_cfg()      → clocks, PLL, PA ramp, RX front end
//!   │     ├── tp_cal()       → two-point modulation calibration
//!   │     ├── rx_dcoc_cal()  → RX DC-offset calibration
//!   │     └── bb_cfg()       → PKT_FMT_ZIGBEE baseband + AGC table
//!   ├── LL HW engine (0x4003_1000..0x4003_1080)
//!   │     ├── STX / SRX single-shot modes, MODE_DONE interrupt
//!   │     └── word-wide TX FIFO 0x4003_1400 / RX FIFO 0x4003_1C00
//!   └── LL_IRQ (BB_IRQn = 4) → RX Signal / TX op-idle flag
//! ```
//!
//! # Requirements on the caller
//! * HCLK must be the 48 MHz DLL clock (`phy6222_hal::clock::init_dll48m`) or
//!   [`RadioConfig::hclk_mhz`] must describe the actual HCLK; the LL timing
//!   registers are programmed in HCLK cycles.
//! * An `embassy-time` driver must be running before [`Phy6222Driver::take`]:
//!   calibration waits are bounded by wall-clock time.
//! * `BB_IRQn` (4) must be unmasked in the NVIC after `take()`.

// `LL_HW_BASE + 0x00` (the trigger register) is written with an explicit
// `+ 0x00` offset to keep every write visually aligned with the SDK
// register map.
#![allow(clippy::identity_op)]

use core::cell::{Cell, UnsafeCell};
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant};
use zigbee_types::{PanId, ShortAddress};

// ── Register blocks ─────────────────────────────────────────────

/// RF PHY analog + baseband register block (`PHY_REG_WT(0x4003xxxx)`).
const RF: u32 = 0x4003_0000;
/// Link-layer hardware engine (`LL_HW_BASE`).
const LL: u32 = 0x4003_1000;
/// TX FIFO write port (32-bit words, `LL_HW_TFIFO`).
const LL_HW_TFIFO: u32 = 0x4003_1400;
/// RX FIFO read port (32-bit words, `LL_HW_RFIFO`).
const LL_HW_RFIFO: u32 = 0x4003_1C00;
/// `0x4000_F040`: xtal-to-digital enable lives in bit 18.
const AON_CLK_CTRL: u32 = 0x4000_F040;
/// `0x4000_F044`: RF/RX-ADC clock selection and enables.
const AON_CLK_CFG: u32 = 0x4000_F044;
/// `0x4000_F014`: DCDC / digital LDO trim (`DCDC_CONFIG_SETTING`).
const AON_PMU_CTRL: u32 = 0x4000_F014;
/// `AP_PCR->SW_RESET2`; bit 10 is `BBLL_RST` (MOD_CP_CPU + 10).
const PCR_SW_RESET2: u32 = 0x4000_000C;
const BBLL_RST_BIT: u8 = 10;

// ── LL HW interrupt bits (`ll_hw_drv.h` LIRQ_*) ─────────────────

/// Mode done: the STX/SRX operation finished (the only unmasked source).
const LIRQ_MD: u32 = 0x0001;
/// CRC error on a received packet.
const LIRQ_CERR: u32 = 0x0002;
/// RX timeout: the RX window elapsed without a sync.
const LIRQ_RTO: u32 = 0x0004;
/// RX done: a packet was written into the RX FIFO.
const LIRQ_RD: u32 = 0x0200;
/// CRC OK on a received packet.
const LIRQ_COK: u32 = 0x0400;
/// Second CRC-error flag (TRLP/RTLP chains).
const LIRQ_CERR2: u32 = 0x0800;
/// All LL HW interrupt status bits (`LL_HW_IRQ_MASK`).
const LL_HW_IRQ_MASK: u32 = 0x3FFF;

/// `LL_HW_CRC_ZB_FMT`: IEEE 802.15.4 CRC-16 for `ll_hw_set_crc_fmt`.
const LL_HW_CRC_ZB_FMT: u32 = 0x03;

/// `ll_hw_ign_rfifo` value: bit 1 drops CRC-error packets, bit 0 drops
/// empty packets, so neither consumes RX FIFO space.
const LL_HW_IGN_CRC_AND_EMPTY: u32 = 0x03;

/// RX timeout written to `LL+0x24`/`LL+0x28`, in microseconds.
///
/// The register is 16 bits wide (`& 0xffff`), so long receives re-arm the
/// engine on every timeout inside [`Phy6222Driver::receive`]; the re-arm gap
/// is a few microseconds of interrupt latency.
const RX_TIMEOUT_US: u32 = 0xFFFF;

/// Bound on a single STX operation (127-byte PSDU is 4.3 ms on air).
const TX_TIMEOUT: Duration = Duration::from_millis(10);

/// RX window of a TRX operation (`LL+0x24`/`LL+0x28`), in microseconds:
/// `macAckWaitDuration` (54 symbols) after the end of the transmission.
const ACK_WAIT_US: u32 = 864;

/// `ll_hw_set_stx` / `ll_hw_set_srx` / `ll_hw_set_trx` mode words (`LL+0x04`).
const LL_MODE_STX: u32 = 0x0000_0100;
const LL_MODE_SRX: u32 = 0x0001_0001;
const LL_MODE_TRX: u32 = 0x0001_0102;

// ── Register access helpers ─────────────────────────────────────

#[inline(always)]
fn reg_write(addr: u32, val: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) };
}

#[inline(always)]
fn reg_read(addr: u32) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

/// `subWriteReg(addr, hi, lo, value)`: `reg[hi:lo] = value`.
#[inline(always)]
fn sub_write_reg(addr: u32, hi: u8, lo: u8, value: u32) {
    let width = hi - lo + 1;
    let field = if width >= 32 {
        u32::MAX
    } else {
        (1u32 << width) - 1
    };
    let mask = field << lo;
    let old = reg_read(addr);
    reg_write(addr, (old & !mask) | ((value << lo) & mask));
}

/// Busy-wait for `us` microseconds of wall-clock time.
fn busy_wait_us(us: u64) {
    let deadline = Instant::now() + Duration::from_micros(us);
    while Instant::now() < deadline {
        core::hint::spin_loop();
    }
}

// ── Frame sizes ─────────────────────────────────────────────────

/// Maximum IEEE 802.15.4 PSDU (`aMaxPHYPacketSize`), including the FCS.
const MAX_FRAME_LEN: usize = 127;

/// Maximum MPDU (MHR + payload, without FCS) accepted by `transmit` and
/// returned by `receive`.
const MAX_MPDU_LEN: usize = MAX_FRAME_LEN - 2;

/// Words needed for `[len][PSDU]` (128 bytes) in the word-wide FIFOs.
const FIFO_WORDS: usize = (MAX_FRAME_LEN + 1).div_ceil(4);

/// Smallest PSDU the receiver hands up (an ACK: FCF + DSN + FCS).
const MIN_RX_PSDU_LEN: usize = 5;

/// CCA busy threshold, in dBm (CCA mode 1, energy above threshold).
const CCA_THRESHOLD_DBM: i8 = -60;

// ── Public types ────────────────────────────────────────────────

/// Received 802.15.4 frame: MPDU without FCS.
pub struct RxFrame {
    pub data: [u8; MAX_FRAME_LEN],
    pub len: usize,
    /// RSSI in dBm from the packet foot word (`-(foot1 >> 24)`).
    pub rssi: i8,
    pub lqi: u8,
}

/// Who produces the 2-byte FCS of transmitted frames.
///
/// The SDK's only Zigbee transmitter (`rf_phy_dtm_zigbee_pkt_gen`) writes
/// `[len incl. FCS][payload][zigbee_crc16 LSB first]` into the TX FIFO, i.e.
/// it computes the FCS in software. Whether the LL engine appends an FCS by
/// itself in `LL_HW_CRC_ZB_FMT` is not documented, so the choice is
/// switchable and was decided on hardware with an independent sniffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FcsMode {
    /// Driver appends `zigbee_crc16` to the FIFO payload (SDK DTM behavior).
    Software,
    /// Driver writes only the MPDU; the LL engine is expected to append FCS.
    /// Lab-only: ignored unless the `phy6222-diag` feature is enabled.
    Hardware,
}

/// TX power code table `(dBm, PA code)`, highest power first; the PA code
/// to dBm mapping differs per package (`rf_phy_driver.h`
/// `RF_PHY_TX_POWER_*`). Only the selected table is linked.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PaTable(&'static [(i8, u8)]);

impl PaTable {
    /// QFN32 parts (PHY6222QC, PHY6252 modules such as Ai-Thinker PB-03F).
    pub const QFN32: Self = Self(&[(5, 0x3f), (0, 0x1f), (-2, 0x0f), (-5, 0x0a), (-20, 0x01)]);
    /// TSOP16 parts (PHY6252 TSOP16 sensor boards).
    pub const TSOP16: Self = Self(&[
        (5, 0x1d),
        (4, 0x17),
        (3, 0x15),
        (0, 0x0d),
        (-2, 0x0a),
        (-5, 0x06),
        (-6, 0x05),
        (-10, 0x03),
        (-15, 0x02),
        (-20, 0x01),
    ]);
}

/// Radio configuration.
#[derive(Debug, Clone, Copy)]
pub struct RadioConfig {
    /// IEEE 802.15.4 channel (11–26).
    pub channel: u8,
    /// PAN ID (address filtering is done in software by the MAC).
    pub pan_id: u16,
    /// Short address (address filtering is done in software by the MAC).
    pub short_address: u16,
    /// Extended (IEEE) 64-bit address.
    pub extended_address: [u8; 8],
    /// Requested TX power in dBm; the nearest table entry not above it is used.
    pub tx_power: i8,
    /// Accept all frames regardless of address.
    pub promiscuous: bool,
    /// Actual HCLK in MHz; LL timing registers are in HCLK cycles.
    pub hclk_mhz: u8,
    /// Package-specific PA code table.
    pub pa_table: PaTable,
    /// FCS generation strategy for transmitted frames.
    pub fcs: FcsMode,
}

/// Default FCS mode, selected from sniffer evidence on PB-03F (PHY6252).
pub const DEFAULT_FCS_MODE: FcsMode = FcsMode::Software;

impl Default for RadioConfig {
    fn default() -> Self {
        Self {
            channel: 11,
            pan_id: 0xFFFF,
            short_address: 0xFFFF,
            extended_address: [0u8; 8],
            tx_power: 0,
            promiscuous: false,
            hclk_mhz: 48,
            pa_table: PaTable::QFN32,
            fcs: DEFAULT_FCS_MODE,
        }
    }
}

/// Radio driver error.
#[derive(Debug, Clone, Copy)]
pub enum RadioError {
    /// Channel busy (CCA failed).
    CcaFailure,
    /// TX/RX engine did not complete or reported an inconsistent state.
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

/// Radio counters and calibration results, for bring-up diagnostics.
///
/// The event counters are maintained only with the `phy6222-diag` feature
/// (otherwise they read zero); calibration and `hw_pkt_stats` are always
/// reported.
#[derive(Debug, Clone, Copy, Default)]
pub struct RadioStats {
    /// STX operations completed.
    pub tx_done: u32,
    /// STX operations that timed out.
    pub tx_timeout: u32,
    /// Frames delivered by `receive` (hardware CRC OK).
    pub rx_ok: u32,
    /// MODE_DONE without CRC OK (CRC error or empty FIFO).
    pub rx_crc_err: u32,
    /// RX windows that expired without a sync (re-armed).
    pub rx_timeout: u32,
    /// CRC-OK frames whose software CRC-16 disagreed with the FIFO FCS.
    pub rx_sw_fcs_mismatch: u32,
    /// Length bytes outside 5..=127.
    pub rx_bad_len: u32,
    /// LL engine aborts (cancelled receive or TX timeout).
    pub aborts: u32,
    /// Raw `LL+0x30` packet statistics `[23:16]` crcErr `[15:8]` total `[7:0]` ok.
    pub hw_pkt_stats: u32,
    /// Two-point calibration results (low/high), already offset.
    pub tp_cal: (u8, u8),
    /// RX DC-offset calibration word (`0x400300EC & 0x3f3fffff`), or the
    /// SDK default `0x2020_0000` when calibration did not converge.
    pub dcoc: u32,
    /// Whether the DC-offset calibration converged.
    pub dcoc_ok: bool,
}

// ── Pure helpers (host-tested) ──────────────────────────────────

/// IEEE 802.15.4 FCS (ITU-T CRC-16, init 0, LSB first), returned in
/// on-air byte order (CRC-16/KERMIT). Intended to match the ROM's
/// `zigbee_crc16_gen(data, len, seed = {0}, crc)` used by the SDK DTM
/// transmitter; the ROM equivalence is checked by the PB-03F radio lab.
pub fn zigbee_crc16(data: &[u8]) -> [u8; 2] {
    let mut crc: u16 = 0;
    for &b in data {
        crc ^= u16::from(b);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0x8408
            } else {
                crc >> 1
            };
        }
    }
    crc.to_le_bytes()
}

/// IEEE 802.15.4 channel (11–26) to the PHY62x2 RF channel index
/// (MHz above 2400): `5 + 5 * (ch - 11)`.
fn rf_channel_index(channel: u8) -> u8 {
    5 + 5 * (channel.clamp(11, 26) - 11)
}

/// `RF_PHY_TPCAL_CALC(tp0, tp1, chn)` with `chn = (rfChn - 2) >> 1` and the
/// `ll_hw_go` clamp of `rfChn` to `2..=80`.
fn tpcal_for_rf_channel(tp0: u8, tp1: u8, rf_chn: u8) -> u8 {
    let idx = u32::from((rf_chn.clamp(2, 80) - 2) >> 1);
    let (tp0, tp1) = (u32::from(tp0), u32::from(tp1));
    if tp0 > tp1 {
        (((tp0 << 5) - (tp0 - tp1) * idx + 16) >> 5) as u8
    } else {
        tp0 as u8
    }
}

/// PA code for the requested power: `(code, extra_max_path)`.
///
/// Picks the highest table entry not above `dbm` (lowest entry if `dbm` is
/// below the table). `extra_max_path` selects the SDK
/// `RF_PHY_TX_POWER_EXTRA_MAX` supply settings.
fn tx_power_code(table: PaTable, dbm: i8) -> (u8, bool) {
    let entries = table.0;
    let code = entries
        .iter()
        .find(|(level, _)| *level <= dbm)
        .or(entries.last())
        .map(|&(_, code)| code)
        .unwrap_or(0x01);
    (code, code == 0x3f)
}

/// Bytes of `[len][PSDU]` in the word-wide FIFOs.
const FIFO_BYTES: usize = FIFO_WORDS * 4;

// The FIFO byte views below reinterpret little-endian words as bytes.
const _: () = assert!(cfg!(target_endian = "little"));

/// Lay out `[len][frame][fcs]` (`ll_hw_write_tfifo`) in a zeroed buffer
/// and return the number of little-endian FIFO words to write.
fn pack_tx_fifo(buf: &mut [u8; FIFO_BYTES], len_byte: u8, frame: &[u8], fcs: &[u8]) -> usize {
    let end = 1 + frame.len();
    buf[0] = len_byte;
    buf[1..end].copy_from_slice(frame);
    buf[end..end + fcs.len()].copy_from_slice(fcs);
    (end + fcs.len()).div_ceil(4)
}

/// Words occupied by an RX FIFO packet including its first word
/// (`ll_hw_read_rfifo_zb`: `wlen = 1 + (blen >> 2)`, `blen` incl. FCS).
fn rfifo_packet_words(len_byte: u8) -> usize {
    1 + (usize::from(len_byte) >> 2)
}

/// RSSI in dBm from packet foot word 1 (`rf_phy_get_pktFoot`).
fn rssi_from_foot1(foot1: u32) -> i8 {
    -((foot1 >> 24).min(127) as i16) as i8
}

/// Outcome of one SRX operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum RxOutcome {
    Pending = 0,
    FrameOk = 1,
    CrcError = 2,
    Timeout = 3,
}

impl RxOutcome {
    fn from_u8(v: u8) -> Self {
        match v {
            1 => RxOutcome::FrameOk,
            2 => RxOutcome::CrcError,
            3 => RxOutcome::Timeout,
            _ => RxOutcome::Pending,
        }
    }
}

/// Classify an SRX `LL+0x10` status word. Only MODE_DONE is unmasked, and
/// the SDK accepts a packet only when `RD` and `COK` are both set.
fn classify_rx_irq(irq_status: u32) -> RxOutcome {
    if irq_status & LIRQ_MD == 0 {
        RxOutcome::Pending
    } else if irq_status & (LIRQ_RD | LIRQ_COK) == (LIRQ_RD | LIRQ_COK) {
        RxOutcome::FrameOk
    } else if irq_status & (LIRQ_CERR | LIRQ_CERR2) != 0 {
        RxOutcome::CrcError
    } else if irq_status & LIRQ_RTO != 0 {
        RxOutcome::Timeout
    } else {
        RxOutcome::CrcError
    }
}

/// MPDU length (FCS stripped) for an RX FIFO length byte, or `None` if the
/// PSDU length is outside `5..=127`.
fn zb_rfifo_mpdu_len(len_byte: u8) -> Option<usize> {
    let psdu = usize::from(len_byte);
    (MIN_RX_PSDU_LEN..=MAX_FRAME_LEN)
        .contains(&psdu)
        .then(|| psdu - 2)
}

// ── Shared state between the driver and LL_IRQ ──────────────────

static RX_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static DRIVER_TAKEN: Mutex<CriticalSectionRawMutex, Cell<bool>> = Mutex::new(Cell::new(false));

/// Operation the LL engine is currently running (written before trigger).
static LL_OP: AtomicU8 = AtomicU8::new(OP_IDLE);
const OP_IDLE: u8 = 0;
const OP_TX: u8 = 1;
/// SRX, or TRX (TX then the ACK window): the RX part ends the operation.
const OP_RX: u8 = 2;
/// STX of an acknowledgement started by `LL_IRQ` itself.
const OP_ACK: u8 = 3;
/// An RX being stopped by `abort_if_busy`: `LL_IRQ` discards the result.
const OP_STOP: u8 = 4;

/// `phy_hw_stop` RX timeout (µs): ends an unsynced RX with RTO.
const STOP_RX_TIMEOUT_US: u32 = 33;
/// Bound for a stopped RX that already synced to finish its frame
/// (127-byte PSDU: 4.3 ms).
const STOP_TIMEOUT: Duration = Duration::from_millis(5);

/// Bound for an `LL_IRQ`-started ACK still on air when the driver starts
/// its next operation (5-byte PSDU: 352 µs plus TX settle).
const ACK_TX_TIMEOUT: Duration = Duration::from_millis(2);

/// Addressing snapshot `LL_IRQ` uses to acknowledge received frames.
#[derive(Clone, Copy)]
struct AckFilter {
    pan_id: u16,
    short_address: u16,
    extended_address: [u8; 8],
    hw_fcs: bool,
}

/// Critical section shared with `LL_IRQ` (blocking mutex over no data).
static CS: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());

static ACK_FILTER: Mutex<CriticalSectionRawMutex, Cell<AckFilter>> =
    Mutex::new(Cell::new(AckFilter {
        pan_id: 0xFFFF,
        short_address: 0xFFFF,
        extended_address: [0; 8],
        hw_fcs: false,
    }));

static RX_OUTCOME: AtomicU8 = AtomicU8::new(RxOutcome::Pending as u8);
static RX_LEN: AtomicU8 = AtomicU8::new(0);
static RX_FOOT1: AtomicU32 = AtomicU32::new(0);

struct SyncUnsafeCell<T>(UnsafeCell<T>);
// SAFETY: RX_WORDS is written by LL_IRQ only while LL_OP == OP_RX and read
// by the task only after RX_DONE was signalled for that operation.
unsafe impl<T> Sync for SyncUnsafeCell<T> {}
static RX_WORDS: SyncUnsafeCell<[u32; FIFO_WORDS]> =
    SyncUnsafeCell(UnsafeCell::new([0; FIFO_WORDS]));

/// Single-writer counters. Cortex-M0 has no atomic read-modify-write, so
/// each counter is updated with load+store from exactly one context.
/// Counting is compiled in only with the `phy6222-diag` feature.
struct Counter(AtomicU32);
impl Counter {
    const fn new() -> Self {
        Self(AtomicU32::new(0))
    }
    fn inc(&self) {
        if cfg!(feature = "phy6222-diag") {
            self.0.store(
                self.0.load(Ordering::Relaxed).wrapping_add(1),
                Ordering::Relaxed,
            );
        }
    }
    fn get(&self) -> u32 {
        self.0.load(Ordering::Relaxed)
    }
}
// Task-context counters.
static TX_DONE_CNT: Counter = Counter::new();
static TX_TIMEOUT_CNT: Counter = Counter::new();
static RX_OK_CNT: Counter = Counter::new();
static RX_SW_FCS_MISMATCH_CNT: Counter = Counter::new();
static ABORT_CNT: Counter = Counter::new();
static RX_CRC_ERR_CNT: Counter = Counter::new();
static RX_TIMEOUT_CNT: Counter = Counter::new();
// ISR-context counter.
static RX_BAD_LEN_CNT: Counter = Counter::new();

/// Pure-Rust async radio driver for PHY62x2 IEEE 802.15.4 mode.
pub struct Phy6222Driver {
    config: RadioConfig,
    initialized: bool,
    /// Two-point calibration at RF index 2 and 66 (`g_rfPhyTpCal0/1`).
    tp_cal0: u8,
    tp_cal1: u8,
    dcoc: u32,
    dcoc_ok: bool,
}

/// Offset added to raw two-point calibration results
/// (`rf_tpCal_gen_cap_arrary`: `rf_tp_cal(...) + 2`).
const TP_CAL_OFFSET: u8 = 2;

impl Phy6222Driver {
    /// Acquire and initialize the process-wide radio driver.
    ///
    /// IRQ signals and FIFO state are global hardware resources, so a second
    /// driver instance would alias them unsafely.
    pub fn take(config: RadioConfig) -> Option<Self> {
        let acquired = DRIVER_TAKEN.lock(|taken| {
            if taken.get() {
                false
            } else {
                taken.set(true);
                true
            }
        });
        acquired.then(|| Self::new(config))
    }

    fn new(config: RadioConfig) -> Self {
        let mut drv = Self {
            config,
            initialized: false,
            tp_cal0: 0x20,
            tp_cal1: 0x20,
            dcoc: 0x2020_0000,
            dcoc_ok: false,
        };
        drv.init_hardware();
        drv
    }

    // ── Initialization (rf_phy_ini + rf_calibrate + ll_hw_set_timing) ──

    fn init_hardware(&mut self) {
        self.analog_up();

        // rf_calibrate: two-point modulation calibration at RF index 2 and
        // 66 using the 500 kHz deviation path (fDev = 1, as Zigbee's
        // O-QPSK/half-sine shares BLE 2M's deviation), then RX DCOC.
        self.tp_cal0 = self.tp_cal(2).saturating_add(TP_CAL_OFFSET);
        self.tp_cal1 = self.tp_cal(66).saturating_add(TP_CAL_OFFSET);
        let (dcoc, ok) = Self::rx_dcoc_cal(88);
        self.dcoc = dcoc;
        self.dcoc_ok = ok;

        // Calibration overwrites 0x40/0x50/0xE0, so the baseband is
        // configured afterwards.
        self.digital_up();

        self.initialized = true;
        log::info!(
            "phy6222: 802.15.4 radio up, tp_cal={:#04x}/{:#04x} dcoc={:#010x}{}",
            self.tp_cal0,
            self.tp_cal1,
            self.dcoc,
            if self.dcoc_ok { "" } else { " (default)" }
        );
    }

    fn is_48mhz(&self) -> bool {
        self.config.hclk_mhz == 48
    }

    /// `rf_phy_ana_cfg` with `RF_PHY_CLK_SEL_16M_XTAL` / `RX_ADC_CLK_SEL_32M_DBL`.
    fn ana_cfg(&self) {
        sub_write_reg(AON_CLK_CTRL, 18, 18, 1); // xtal output to digital
        sub_write_reg(AON_CLK_CFG, 25, 24, 2); // rxAdcClkSel = 32M DBL
        sub_write_reg(AON_CLK_CFG, 23, 22, 0); // rfPhyClkSel = 16M XTAL
        sub_write_reg(AON_CLK_CFG, 6, 5, 3); // trim dll/dbl ldo vout
        sub_write_reg(AON_CLK_CFG, 8, 8, 1); // DBL enable (RX ADC 32M DBL)
        sub_write_reg(AON_CLK_CFG, 19, 18, 3); // RX ADC clk en, RF PHY clk en
        sub_write_reg(RF + 0x8C, 23, 23, u32::from(self.is_48mhz()));

        // PLL. phy62x2-SDK writes ctrl1 = 0x160 then 0x180; THB2's SDK also
        // programs the RX PLL copies (0x104..0x10C), which are kept here.
        reg_write(RF + 0xCC, 0x2000_0BC0);
        reg_write(RF + 0xD0, 0x0000_0160);
        reg_write(RF + 0xD4, 0x076A_3E7A);
        reg_write(RF + 0xD8, 0x0489_0000);
        reg_write(RF + 0xD0, 0x0000_0180);
        reg_write(RF + 0xD4, 0x076A_3E7A);
        reg_write(RF + 0xD8, 0x0489_0000);
        reg_write(RF + 0x104, 0x0000_0180);
        reg_write(RF + 0x108, 0x076A_3E7A);
        reg_write(RF + 0x10C, 0x0489_8000);

        // VCO coarse tune, then flag the 16 MHz reference to the RF PLL.
        reg_write(RF + 0x80, 0x0000_24CC);
        sub_write_reg(RF + 0x80, 0, 0, 1);

        // PA ramp; the PA code in [16:12] is set by `set_tx_power`, which
        // always follows.
        reg_write(RF + 0xB8, 0x0825);
        reg_write(RF + 0xDC, 0x01A6_FC2F); // RX front end, boost TIA current
    }

    /// `rf_phy_set_txPower`: supply trims plus the PA code in `0xB8[16:12]`.
    fn set_tx_power(&self, dbm: i8) {
        let (code, extra) = tx_power_code(self.config.pa_table, dbm);
        // (DCDC trim, LO LDO, RF+0xDC[9:5] = PA VTRIM[9:7] << 2 | LNA LDO[6:5])
        let (dcdc, lo_ldo, vtrim_lna) = if extra {
            (0x08, 0, 1 << 2) // DCDC highest, VTRIM 1, LNA LDO 0
        } else {
            (0x0A, 2, 1) // VTRIM 0, LNA LDO 1
        };
        sub_write_reg(AON_PMU_CTRL, 18, 15, dcdc);
        sub_write_reg(RF + 0xCC, 11, 10, lo_ldo);
        sub_write_reg(RF + 0xDC, 9, 5, vtrim_lna);
        let b8 = reg_read(RF + 0xB8);
        reg_write(RF + 0xB8, (b8 & 0x0FFF) | (u32::from(code & 0x1F) << 12));
    }

    /// `rf_tp_cal(rfChn, fDev = 1)`: raw two-point calibration code.
    fn tp_cal(&self, rf_chn: u8) -> u8 {
        let bit23 = u32::from(self.is_48mhz());
        reg_write(RF + 0x40, 0x0003_0010); // test mode, no tx-done
        sub_write_reg(RF + 0xD8, 20, 18, 0x02); // tpm dac var
        reg_write(RF + 0x8C, 0x0053_407F);
        sub_write_reg(RF + 0x8C, 23, 23, bit23);
        reg_write(RF + 0xB4, u32::from(rf_chn));
        reg_write(RF + 0xA0, 0x0000_000E); // pll_auto override
        reg_write(RF + 0xA4, 0x0000_0000);
        reg_write(RF + 0xA4, 0x0000_0114); // trigger
        // SDK: 3 * 10000 busy-loop iterations at 48 MHz (~2-4 ms).
        busy_wait_us(3_000);
        let k = ((reg_read(RF + 0xF4) >> 16) & 0xFF) as u8;
        reg_write(RF + 0xA4, 0x0000_0000);
        reg_write(RF + 0xA0, 0x0000_0000);
        reg_write(RF + 0x8C, 0x0010_4040); // clr tp_cal_en
        reg_write(RF + 0xA4, 0x0000_0140);
        reg_write(RF + 0x40, 0x0003_2800);
        sub_write_reg(RF + 0x8C, 23, 23, bit23);
        k
    }

    /// `rf_rxDcoc_cfg(rfChn, bwSet = 1)`: RX DC-offset calibration.
    ///
    /// Only run from `init_hardware`, before `ll_cfg` reprograms the LL RX
    /// timeouts, so the SDK's save/restore of `LL+0x24/0x28` is omitted.
    fn rx_dcoc_cal(rf_chn: u8) -> (u32, bool) {
        reg_write(LL + 0x24, 0);
        reg_write(LL + 0x28, 0);

        reg_write(RF + 0xB4, u32::from(rf_chn));
        reg_write(RF + 0xA0, 0x0000_000E); // pll_auto override
        reg_write(RF + 0xA4, 0x0000_0100); // clr trx_auto
        reg_write(RF + 0xA8, 0x0000_0040); // lo override
        reg_write(RF + 0xAC, 0x0000_0050); // rx lo and pll buff
        reg_write(RF + 0xA4, 0x0000_012A); // rx close

        let flt_phy: u32 = 0x034D;
        reg_write(RF + 0xE0, 0x0000_0100); // pga bw 1 MHz
        reg_write(RF + 0xC8, 0x0000_01A4); // rx max gain
        reg_write(RF + 0xC4, 0x0000_2020); // dcoc dac from mix_sig_top
        reg_write(RF + 0x50, 0x200C_5680); // longer dcoc averaging
        busy_wait_us(100);

        reg_write(RF + 0x78, (flt_phy << 22) | 0x0021_6564);
        reg_write(RF + 0x78, (flt_phy << 22) | 0x0021_6565); // start

        let mut result = (0x2020_0000, false);
        let deadline = Instant::now() + Duration::from_millis(4);
        while Instant::now() < deadline {
            let ec = reg_read(RF + 0xEC);
            if (ec >> 30) & 0x3 == 0x3 {
                let dc = ec & 0x3F3F_FFFF;
                reg_write(RF + 0xC4, 0x0001_0000 | (dc >> 16));
                result = (dc, true);
                break;
            }
        }

        reg_write(RF + 0x78, (flt_phy << 22) | 0x0021_6564);
        reg_write(RF + 0xA8, 0);
        reg_write(RF + 0xAC, 0);
        reg_write(RF + 0xC8, 0);
        reg_write(RF + 0xA4, 0x0000_0100);
        reg_write(RF + 0xA0, 0);
        reg_write(RF + 0xA4, 0x0000_0140);
        result
    }

    /// `rf_phy_bb_cfg(PKT_FMT_ZIGBEE)`.
    fn bb_cfg(&self) {
        reg_write(RF + 0xE0, 0x0000_0080); // small PGA bw (Zigbee, BLE 2M)
        sub_write_reg(RF + 0xD8, 20, 18, 0x02); // tpm dac var
        // reg_dc for the 16 MHz XTAL RF clock: 0x00087000 in phy62x2-SDK
        // (0x00080000 in the older THB2 copy).
        reg_write(RF + 0x90, 0x0008_7000);
        // dac delay: 0x01000002 in phy62x2-SDK (0x01000003 in THB2).
        reg_write(RF + 0xB0, 0x0100_0002);
        reg_write(RF + 0x94, 0x0000_1000 + u32::from(self.tp_cal0));

        reg_write(RF + 0x00, 0x7806_8000); // PKT_FMT_ZIGBEE, sync threshold
        reg_write(RF + 0x48, 0x0000_0000); // no CRC seed / whitening
        reg_write(RF + 0x40, 0x000B_2800); // gauss off
        reg_write(RF + 0x4C, 0x3675_EE07); // 802.15.4 SHR correlator
        reg_write(RF + 0x124, (LL_HW_CRC_ZB_FMT << 8) | LL_HW_CRC_ZB_FMT);

        reg_write(RF + 0x50, 0x2208_6680); // AGC control (Zigbee)
        sub_write_reg(RF + 0x0C, 7, 0, 0xFF); // max RX length
        const AGC_TAB: [u32; 9] = [
            0x545C_9CA4,
            0x4243_444C,
            0x464C_5241,
            0x2E34_3A40,
            0x557F_0028,
            0x3D43_494F,
            0x4C2B_3137,
            0x343A_4046,
            0x1C22_282E,
        ];
        for (i, v) in AGC_TAB.iter().enumerate() {
            reg_write(RF + 0x54 + 4 * i as u32, *v);
        }
    }

    /// LL engine configuration: `ll_hw_set_timing(PKT_FMT_ZIGBEE)`,
    /// `ll_hw_ign_rfifo`, RX timeouts. Re-applied after every BBLL reset.
    fn ll_cfg(&self) {
        let h = u32::from(self.config.hclk_mhz);
        reg_write(LL + 0x20, ((10 * h) << 16) | h); // tx/rx release 10/1 us
        reg_write(LL + 0x1C, 62 * h); // rx->tx interval
        reg_write(LL + 0x18, 72 * h); // tx->rx interval
        reg_write(RF + 0xBC, (57 << 16) | (8 << 8) | 52); // TxBB, RxAFE, PLL settle
        reg_write(LL + 0x68, LL_HW_IGN_CRC_AND_EMPTY);
        reg_write(LL + 0x24, RX_TIMEOUT_US & 0xFFFF);
        reg_write(LL + 0x28, RX_TIMEOUT_US & 0xFFFF);
    }

    /// Reset the LL engine (`BBLL_RST`), discarding any running operation,
    /// then restore its configuration and clear FIFOs/IRQs.
    fn ll_reset(&self) {
        sub_write_reg(PCR_SW_RESET2, BBLL_RST_BIT, BBLL_RST_BIT, 0);
        sub_write_reg(PCR_SW_RESET2, BBLL_RST_BIT, BBLL_RST_BIT, 1);
        LL_OP.store(OP_IDLE, Ordering::Release);
        reg_write(LL + 0x0C, 0);
        reg_write(LL + 0x14, LL_HW_IRQ_MASK);
        reg_write(LL + 0x58, 0x0100); // rst rfifo
        reg_write(LL + 0x58, 0x0001); // rst tfifo
    }

    /// Stop a still-running LL operation (e.g. a cancelled `receive` or
    /// the CCA window). An acknowledgement started by `LL_IRQ` is allowed
    /// to finish. A receive is stopped like `phy_hw_stop`: the RX timeout
    /// is cut to [`STOP_RX_TIMEOUT_US`] so the engine ends with RTO, or
    /// completes a frame it already synced to (discarded, never ACKed).
    /// Resetting BB/LL mid-frame instead left the next TRX unable to
    /// transmit (hardware: data requests never on air, 10 ms deadline), so
    /// `BBLL_RST` is only the bounded fallback.
    fn abort_if_busy(&self) {
        if LL_OP.load(Ordering::Acquire) == OP_ACK {
            wait_while_op(OP_ACK, ACK_TX_TIMEOUT);
        }
        // thumbv6m has no CAS: take the RX→STOP transition with LL_IRQ
        // masked so a completion in between is not overwritten.
        let stopping = CS.lock(|()| {
            let rx = LL_OP.load(Ordering::Acquire) == OP_RX;
            if rx {
                LL_OP.store(OP_STOP, Ordering::Release);
            }
            rx
        });
        if stopping {
            reg_write(LL + 0x24, STOP_RX_TIMEOUT_US);
            reg_write(LL + 0x28, STOP_RX_TIMEOUT_US);
            wait_while_op(OP_STOP, STOP_TIMEOUT);
            reg_write(LL + 0x24, RX_TIMEOUT_US);
            reg_write(LL + 0x28, RX_TIMEOUT_US);
        }
        if LL_OP.load(Ordering::Acquire) != OP_IDLE {
            self.ll_reset();
            self.ll_cfg();
            ABORT_CNT.inc();
        }
    }

    fn set_channel(&self, channel: u8) {
        reg_write(RF + 0xB4, u32::from(rf_channel_index(channel)));
    }

    /// `ll_hw_go`: clear IRQ, unmask MODE_DONE, trigger, then load the
    /// per-channel TP calibration (the SDK sets it after the trigger).
    fn ll_go(&self, op: u8) {
        LL_OP.store(op, Ordering::Release);
        reg_write(LL + 0x14, LL_HW_IRQ_MASK);
        reg_write(LL + 0x0C, LIRQ_MD);
        reg_write(LL + 0x00, 0x0001);
        // RF+0x94 is only ever `0x1000 | tpcal` (see `bb_cfg`), so write it
        // whole instead of read-modify-write of bits [7:0].
        let rf_chn = rf_channel_index(self.config.channel);
        reg_write(
            RF + 0x94,
            0x0000_1000 | u32::from(tpcal_for_rf_channel(self.tp_cal0, self.tp_cal1, rf_chn)),
        );
    }

    fn apply_config(&self) {
        self.set_channel(self.config.channel);
        self.set_tx_power(self.config.tx_power);
        let filter = AckFilter {
            pan_id: self.config.pan_id,
            short_address: self.config.short_address,
            extended_address: self.config.extended_address,
            hw_fcs: self.hw_fcs(),
        };
        ACK_FILTER.lock(|f| f.set(filter));
    }

    /// `Hardware` FCS is a lab A/B option; production builds (without
    /// `phy6222-diag`) always use the SDK software FCS.
    fn hw_fcs(&self) -> bool {
        cfg!(feature = "phy6222-diag") && self.config.fcs == FcsMode::Hardware
    }

    /// Current radio configuration.
    pub fn config(&self) -> &RadioConfig {
        &self.config
    }

    /// Update the radio configuration and apply it to hardware.
    ///
    /// `hclk_mhz` is fixed when the driver is taken (it sets the LL timing
    /// and the RF clock path); a changed value is ignored.
    pub fn update_config(&mut self, update_fn: impl FnOnce(&mut RadioConfig)) {
        let hclk_mhz = self.config.hclk_mhz;
        update_fn(&mut self.config);
        self.commit_config(hclk_mhz);
    }

    /// Non-generic tail of [`Self::update_config`], shared by every caller
    /// so each closure instantiation stays a few instructions (fixed XIP).
    #[inline(never)]
    fn commit_config(&mut self, hclk_mhz: u8) {
        self.config.hclk_mhz = hclk_mhz;
        if self.initialized {
            self.abort_if_busy();
            self.apply_config();
        }
    }

    /// Diagnostic counters and calibration results.
    pub fn stats(&self) -> RadioStats {
        RadioStats {
            tx_done: TX_DONE_CNT.get(),
            tx_timeout: TX_TIMEOUT_CNT.get(),
            rx_ok: RX_OK_CNT.get(),
            rx_crc_err: RX_CRC_ERR_CNT.get(),
            rx_timeout: RX_TIMEOUT_CNT.get(),
            rx_sw_fcs_mismatch: RX_SW_FCS_MISMATCH_CNT.get(),
            rx_bad_len: RX_BAD_LEN_CNT.get(),
            aborts: ABORT_CNT.get(),
            hw_pkt_stats: reg_read(LL + 0x30),
            tp_cal: (self.tp_cal0, self.tp_cal1),
            dcoc: self.dcoc,
            dcoc_ok: self.dcoc_ok,
        }
    }

    // ── TX / RX ─────────────────────────────────────────────────

    /// Transmit an MPDU (MAC header + payload, **without** FCS).
    ///
    /// The FCS is produced according to [`RadioConfig::fcs`] (honoured only
    /// with the `phy6222-diag` feature; otherwise software FCS); the on-air
    /// PSDU is `frame.len() + 2` bytes, so `frame` is bounded by 125 bytes.
    ///
    /// Completes without yielding; see [`Self::transmit_blocking`].
    pub async fn transmit(&mut self, frame: &[u8]) -> Result<(), RadioError> {
        self.transmit_blocking(frame)
    }

    /// Transmit and spin until `LL_IRQ` reports MODE_DONE (returning
    /// `LL_OP` to idle), bounded by the 10 ms TX timeout; interrupts must
    /// be enabled.
    ///
    /// A PSDU is at most 4.3 ms on air (a data request ~0.8 ms, a software
    /// ACK ~0.35 ms), so the CPU spins instead of yielding: TX is
    /// latency-critical (the ACK wait follows immediately) and not
    /// embedding a TX future keeps every MAC state machine smaller. No
    /// `wfi` is used because a hung LL engine would then have no
    /// guaranteed wake source before the deadline.
    pub fn transmit_blocking(&mut self, frame: &[u8]) -> Result<(), RadioError> {
        self.start_tx(frame, LL_MODE_STX, OP_TX)?;
        let deadline = Instant::now() + TX_TIMEOUT;
        while LL_OP.load(Ordering::Acquire) == OP_TX {
            if Instant::now() >= deadline {
                return self.tx_done(false);
            }
        }
        self.tx_done(true)
    }

    fn tx_done(&self, ok: bool) -> Result<(), RadioError> {
        if ok {
            TX_DONE_CNT.inc();
            Ok(())
        } else {
            TX_TIMEOUT_CNT.inc();
            self.abort_if_busy();
            Err(RadioError::HardwareError)
        }
    }

    /// Transmit a frame that requests an acknowledgement and receive the
    /// ACK in the same hardware operation (`ll_hw_set_trx`): the LL engine
    /// turns the radio around after the `LL+0x18` TX→RX interval and
    /// listens for `ACK_WAIT_US` (864 µs, macAckWaitDuration). A software re-arm after the TX-done
    /// interrupt is too slow for the 192 µs turnaround (verified with a
    /// sniffer: parents ACKed every association request, none was seen).
    ///
    /// Returns `Ok(Some(frame_pending))` for an ACK carrying `frame[2]` as
    /// its sequence number and `Ok(None)` otherwise. Spins like
    /// [`Self::transmit_blocking`].
    pub fn transmit_wait_ack(&mut self, frame: &[u8]) -> Result<Option<bool>, RadioError> {
        RX_OUTCOME.store(RxOutcome::Pending as u8, Ordering::Release);
        reg_write(LL + 0x24, ACK_WAIT_US);
        reg_write(LL + 0x28, ACK_WAIT_US);
        reg_write(LL + 0x58, 0x0100); // rst rfifo
        self.start_tx(frame, LL_MODE_TRX, OP_RX)?;
        let deadline = Instant::now() + TX_TIMEOUT;
        while LL_OP.load(Ordering::Acquire) == OP_RX {
            if Instant::now() >= deadline {
                return self.tx_done(false).map(|()| None);
            }
        }
        self.tx_done(true)?;
        if RxOutcome::from_u8(RX_OUTCOME.load(Ordering::Acquire)) != RxOutcome::FrameOk {
            return Ok(None);
        }
        // SAFETY: LL_OP is idle, so LL_IRQ no longer writes RX_WORDS.
        let w0 = unsafe { (*RX_WORDS.0.get())[0] };
        // FIFO byte 0 = PSDU length, then FCF (2 bytes) and sequence number.
        let [_, fcf0, _, seq] = w0.to_le_bytes();
        let is_ack = RX_LEN.load(Ordering::Acquire) == 3 && fcf0 & 0x07 == 0x02;
        Ok((is_ack && seq == frame[2]).then_some(fcf0 & 0x10 != 0))
    }

    /// Load the TX FIFO and start one STX or TRX operation; `LL_IRQ`
    /// returns `LL_OP` to idle on completion.
    fn start_tx(&mut self, frame: &[u8], mode: u32, op: u8) -> Result<(), RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }
        if frame.is_empty() || frame.len() > MAX_MPDU_LEN {
            return Err(RadioError::InvalidFrame);
        }
        self.abort_if_busy();

        let sw_fcs = zigbee_crc16(frame);
        let fcs: &[u8] = if self.hw_fcs() { &[] } else { &sw_fcs };

        let mut buf = [0u8; FIFO_BYTES];
        let words = pack_tx_fifo(&mut buf, (frame.len() + 2) as u8, frame, fcs);
        reg_write(LL + 0x58, 0x0001); // rst tfifo
        for w in buf.chunks_exact(4).take(words) {
            reg_write(LL_HW_TFIFO, u32::from_le_bytes([w[0], w[1], w[2], w[3]]));
        }
        reg_write(LL + 0x04, mode);
        reg_write(LL + 0x38, 0x07); // bypass SN/NESN/MD
        self.ll_go(op);
        Ok(())
    }

    /// Start one SRX window.
    fn start_rx(&self) {
        RX_OUTCOME.store(RxOutcome::Pending as u8, Ordering::Release);
        RX_DONE.reset();
        reg_write(LL + 0x58, 0x0100); // rst rfifo
        reg_write(LL + 0x24, RX_TIMEOUT_US); // undo a TRX ACK window
        reg_write(LL + 0x28, RX_TIMEOUT_US);
        reg_write(LL + 0x04, LL_MODE_SRX);
        reg_write(LL + 0x38, 0x07);
        self.ll_go(OP_RX);
    }

    /// Receive the next frame with a good CRC (MPDU, FCS stripped).
    ///
    /// RX timeouts and CRC errors re-arm the receiver internally and are
    /// only counted (see [`Self::stats`]); the caller bounds the wait by
    /// racing this future against a timer.
    ///
    /// Dropping the future leaves the receive window armed (at most one
    /// `RX_TIMEOUT_US` window, it is not re-armed); every later operation,
    /// including [`Self::radio_sleep`], first aborts it in hardware. This
    /// avoids a `Drop` guard, whose drop glue would propagate into every
    /// stack future that awaits the radio and cost ~1 KiB of XIP.
    pub async fn receive(&mut self) -> Result<RxFrame, RadioError> {
        self.rx_begin()?;
        loop {
            RX_DONE.wait().await;
            if let Some(result) = self.rx_complete() {
                return result;
            }
        }
    }

    /// Synchronous head of [`Self::receive`]: abort any running operation
    /// and arm the first window. Kept out of the async state machine.
    #[inline(never)]
    fn rx_begin(&mut self) -> Result<(), RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }
        self.abort_if_busy();
        self.start_rx();
        Ok(())
    }

    /// Synchronous tail of one [`Self::receive`] window: count timeouts and
    /// CRC errors and re-arm (`None`), or return the received frame.
    #[inline(never)]
    fn rx_complete(&mut self) -> Option<Result<RxFrame, RadioError>> {
        match RxOutcome::from_u8(RX_OUTCOME.load(Ordering::Acquire)) {
            RxOutcome::FrameOk => {}
            RxOutcome::CrcError => {
                RX_CRC_ERR_CNT.inc();
                self.start_rx();
                return None;
            }
            RxOutcome::Timeout => {
                RX_TIMEOUT_CNT.inc();
                self.start_rx();
                return None;
            }
            RxOutcome::Pending => {
                self.abort_if_busy();
                return Some(Err(RadioError::HardwareError));
            }
        }

        RX_OK_CNT.inc();
        let len = usize::from(RX_LEN.load(Ordering::Acquire));
        let rssi = rssi_from_foot1(RX_FOOT1.load(Ordering::Acquire));
        let mut frame = RxFrame {
            data: [0u8; MAX_FRAME_LEN],
            len,
            rssi,
            lqi: rssi_to_lqi(rssi),
        };
        // SAFETY: LL_IRQ finished writing RX_WORDS before signalling RX_DONE
        // and only writes them again in a later OP_RX (an OP_ACK in flight
        // touches the TX FIFO only), so no concurrent writer exists. The words are
        // little-endian (asserted above), so the byte view is the FIFO byte
        // order: byte 0 is the length byte and the MPDU starts at byte 1.
        let bytes = unsafe { &*(RX_WORDS.0.get() as *const [u8; FIFO_BYTES]) };
        frame.data[..len].copy_from_slice(&bytes[1..=len]);
        // Cross-check the hardware CRC verdict in software (diagnostic).
        if cfg!(feature = "phy6222-diag")
            && zigbee_crc16(&frame.data[..len]) != [bytes[len + 1], bytes[len + 2]]
        {
            RX_SW_FCS_MISMATCH_CNT.inc();
        }
        log::trace!("phy6222: rx {} bytes rssi={}dBm", len, rssi);
        Some(Ok(frame))
    }

    /// Enable the receiver briefly and sample the RSSI register.
    ///
    /// The SDK has no live-RSSI API; `0x400300E8[31:24]` (packet foot 1) is
    /// sampled after 8 symbol periods. This is not hardware-validated as a
    /// live energy measurement. The 128 µs window is spun rather than
    /// awaited: it precedes a TX that spins anyway, and keeping CCA
    /// synchronous removes an await point from every CSMA state machine.
    #[inline(never)]
    fn measure_channel_rssi(&mut self) -> Result<i8, RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }
        self.abort_if_busy();
        self.start_rx();
        busy_wait_us(128);
        let rssi = rssi_from_foot1(reg_read(RF + 0xE8));
        self.abort_if_busy();
        Ok(rssi)
    }

    /// Energy detect on the current channel: (RSSI dBm, busy). Spins for
    /// 128 µs like [`Self::channel_busy`].
    pub fn energy_detect_blocking(&mut self) -> Result<(i8, bool), RadioError> {
        let rssi = self.measure_channel_rssi()?;
        Ok((rssi, rssi > CCA_THRESHOLD_DBM))
    }

    /// CCA mode 1 on the current channel; `true` if busy. Spins for 128 µs.
    pub fn channel_busy(&mut self) -> Result<bool, RadioError> {
        Ok(self.measure_channel_rssi()? > CCA_THRESHOLD_DBM)
    }

    /// Energy detect on the current channel: (RSSI dBm, busy).
    pub async fn energy_detect(&mut self) -> Result<(i8, bool), RadioError> {
        self.energy_detect_blocking()
    }

    /// CCA mode 1 on the current channel; `true` if busy.
    pub async fn clear_channel_assessment(&mut self) -> Result<bool, RadioError> {
        self.channel_busy()
    }

    /// Stop the LL engine and gate the RF/RX-ADC clocks.
    ///
    /// Call [`Self::radio_wake`] before the next operation. The DBL stays
    /// enabled because system clock generation may depend on it.
    pub fn radio_sleep(&self) {
        self.abort_if_busy();
        sub_write_reg(AON_CLK_CFG, 19, 18, 0);
    }

    /// Re-enable clocks and re-run the full analog/baseband/LL
    /// configuration, reusing the stored calibration results.
    pub fn radio_wake(&mut self) {
        self.analog_up();
        if self.dcoc_ok {
            reg_write(RF + 0xC4, 0x0001_0000 | (self.dcoc >> 16));
        }
        self.digital_up();
    }

    /// LL reset, analog/clock configuration and PA (before calibration).
    #[inline(never)]
    fn analog_up(&self) {
        self.ll_reset();
        self.ana_cfg();
        self.set_tx_power(self.config.tx_power);
    }

    /// Baseband, LL timing, channel and power (after calibration).
    #[inline(never)]
    fn digital_up(&self) {
        self.bb_cfg();
        self.ll_cfg();
        self.apply_config();
    }
}

// ── IRQ handler ─────────────────────────────────────────────────

/// LL HW interrupt handler for `BB_IRQn` (4), dispatched through the ROM
/// jump table entry emitted by the product linker script.
#[unsafe(no_mangle)]
pub extern "C" fn LL_IRQ() {
    let status = reg_read(LL + 0x10) & LL_HW_IRQ_MASK;
    reg_write(LL + 0x14, LL_HW_IRQ_MASK);
    if status & LIRQ_MD == 0 {
        return;
    }
    match LL_OP.load(Ordering::Acquire) {
        OP_RX => {
            let mut outcome = classify_rx_irq(status);
            if outcome == RxOutcome::FrameOk {
                outcome = read_rx_fifo();
            }
            reg_write(LL + 0x58, 0x0100); // drop anything left in the RX FIFO
            RX_OUTCOME.store(outcome as u8, Ordering::Release);
            if outcome == RxOutcome::FrameOk
                && let Some(seq) = ack_sequence_for_rx()
            {
                start_ack_tx(seq);
            } else {
                LL_OP.store(OP_IDLE, Ordering::Release);
            }
            RX_DONE.signal(());
        }
        // OP_TX / OP_ACK done, or a stopped RX (result discarded).
        _ => {
            reg_write(LL + 0x58, 0x0100); // rst rfifo
            LL_OP.store(OP_IDLE, Ordering::Release);
        }
    }
}

/// Spin while `LL_OP == op`, for at most `timeout`.
fn wait_while_op(op: u8, timeout: Duration) {
    let deadline = Instant::now() + timeout;
    while LL_OP.load(Ordering::Acquire) == op && Instant::now() < deadline {
        core::hint::spin_loop();
    }
}

/// Sequence number to acknowledge for the frame `read_rx_fifo` just stored
/// (shared exact-destination policy), or `None`.
fn ack_sequence_for_rx() -> Option<u8> {
    let filter = ACK_FILTER.lock(Cell::get);
    let len = usize::from(RX_LEN.load(Ordering::Relaxed));
    // SAFETY: only LL_IRQ (this context) writes RX_WORDS; the driver reads
    // them only after RX_DONE.
    let bytes = unsafe { &*(RX_WORDS.0.get() as *const [u8; FIFO_BYTES]) };
    crate::frames::software_ack_sequence(
        &bytes[1..=len],
        PanId(filter.pan_id),
        ShortAddress(filter.short_address),
        &filter.extended_address,
    )
}

/// Start the STX of an ACK (`frame pending` clear: end devices hold no
/// indirect data) on the channel and TP calibration of the RX that just
/// completed. Sent from the interrupt because the 802.15.4 turnaround
/// (`macAckWaitDuration`, 864 µs) is shorter than a task wake-up: software
/// ACKs were measured on air 1.7–3.7 ms after the frame and never accepted.
fn start_ack_tx(seq: u8) {
    let hw_fcs = ACK_FILTER.lock(Cell::get).hw_fcs;
    let ack = [0x02, 0x00, seq];
    let crc = zigbee_crc16(&ack);
    reg_write(LL + 0x58, 0x0001); // rst tfifo
    reg_write(LL_HW_TFIFO, u32::from_le_bytes([5, ack[0], ack[1], ack[2]]));
    if !hw_fcs {
        reg_write(LL_HW_TFIFO, u32::from_le_bytes([crc[0], crc[1], 0, 0]));
    }
    reg_write(LL + 0x04, LL_MODE_STX);
    reg_write(LL + 0x38, 0x07);
    LL_OP.store(OP_ACK, Ordering::Release);
    reg_write(LL + 0x14, LL_HW_IRQ_MASK);
    reg_write(LL + 0x0C, LIRQ_MD);
    reg_write(LL + 0x00, 0x0001);
}

/// Pop one packet (`ll_hw_read_rfifo_zb`) into `RX_WORDS`.
fn read_rx_fifo() -> RxOutcome {
    let info = reg_read(LL + 0x54);
    let depth = ((info >> 16) & 0x7FF).wrapping_sub(info & 0x7FF) & 0x7FF;
    if depth == 0 {
        return RxOutcome::CrcError;
    }
    // SAFETY: only LL_IRQ writes RX_WORDS, while LL_OP == OP_RX.
    let words = unsafe { &mut *RX_WORDS.0.get() };
    words[0] = reg_read(LL_HW_RFIFO);
    let len_byte = (words[0] & 0xFF) as u8;
    let Some(mpdu_len) = zb_rfifo_mpdu_len(len_byte) else {
        RX_BAD_LEN_CNT.inc();
        return RxOutcome::CrcError;
    };
    let wlen = rfifo_packet_words(len_byte);
    for w in words.iter_mut().take(wlen).skip(1) {
        *w = reg_read(LL_HW_RFIFO);
    }
    let _foot0 = reg_read(LL_HW_RFIFO);
    let foot1 = reg_read(LL_HW_RFIFO);
    RX_LEN.store(mpdu_len as u8, Ordering::Release);
    RX_FOOT1.store(foot1, Ordering::Release);
    RxOutcome::FrameOk
}

// ── Utility ─────────────────────────────────────────────────────

/// RSSI-derived IEEE 802.15.4 LQI (shared stack-wide policy).
fn rssi_to_lqi(rssi: i8) -> u8 {
    crate::lqi::from_rssi_dbm(rssi)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Only pure helpers are tested: constructing the driver touches MMIO.

    #[test]
    fn frame_size_constants() {
        assert_eq!(MAX_FRAME_LEN, 127);
        assert_eq!(MAX_MPDU_LEN + 2, MAX_FRAME_LEN);
        assert_eq!(FIFO_WORDS, 32);
    }

    #[test]
    fn irq_bits_match_ll_hw_drv_h() {
        assert_eq!(LIRQ_MD, 1 << 0);
        assert_eq!(LIRQ_CERR, 1 << 1);
        assert_eq!(LIRQ_RTO, 1 << 2);
        assert_eq!(LIRQ_RD, 1 << 9);
        assert_eq!(LIRQ_COK, 1 << 10);
        assert_eq!(LIRQ_CERR2, 1 << 11);
        assert_eq!(LL_HW_IRQ_MASK, 0x3FFF);
    }

    #[test]
    fn classify_requires_mode_done() {
        assert_eq!(classify_rx_irq(LIRQ_RD | LIRQ_COK), RxOutcome::Pending);
        assert_eq!(classify_rx_irq(0), RxOutcome::Pending);
    }

    #[test]
    fn classify_ok_needs_rd_and_cok() {
        assert_eq!(
            classify_rx_irq(LIRQ_MD | LIRQ_RD | LIRQ_COK),
            RxOutcome::FrameOk
        );
        assert_eq!(classify_rx_irq(LIRQ_MD | LIRQ_COK), RxOutcome::CrcError);
    }

    #[test]
    fn classify_crc_error_and_timeout() {
        assert_eq!(
            classify_rx_irq(LIRQ_MD | LIRQ_RD | LIRQ_CERR),
            RxOutcome::CrcError
        );
        assert_eq!(classify_rx_irq(LIRQ_MD | LIRQ_CERR2), RxOutcome::CrcError);
        assert_eq!(classify_rx_irq(LIRQ_MD | LIRQ_RTO), RxOutcome::Timeout);
        // A CRC error wins over a simultaneous timeout flag.
        assert_eq!(
            classify_rx_irq(LIRQ_MD | LIRQ_CERR | LIRQ_RTO),
            RxOutcome::CrcError
        );
    }

    #[test]
    fn rx_outcome_round_trips() {
        for o in [
            RxOutcome::Pending,
            RxOutcome::FrameOk,
            RxOutcome::CrcError,
            RxOutcome::Timeout,
        ] {
            assert_eq!(RxOutcome::from_u8(o as u8), o);
        }
        assert_eq!(RxOutcome::from_u8(0xFF), RxOutcome::Pending);
    }

    /// Independent bit-serial LFSR formulation of the 802.15.4 FCS (a 16-bit
    /// register as a bit array, like the `uint8_t seed[16]` argument of the
    /// SDK's ROM `zigbee_crc16_gen`, whose source is not published).
    fn bit_serial_crc16(data: &[u8]) -> [u8; 2] {
        let mut reg = [0u8; 16];
        for &byte in data {
            for j in 0..8 {
                let bit = (byte >> j) & 1;
                let fb = reg[0] ^ bit;
                for k in 0..15 {
                    reg[k] = reg[k + 1];
                }
                reg[3] ^= fb;
                reg[10] ^= fb;
                reg[15] = fb;
            }
        }
        let to_byte = |bits: &[u8]| bits.iter().enumerate().fold(0u8, |a, (i, b)| a | (b << i));
        [to_byte(&reg[..8]), to_byte(&reg[8..])]
    }

    #[test]
    fn crc16_matches_kermit_check_value() {
        assert_eq!(zigbee_crc16(b"123456789"), [0x89, 0x21]);
        assert_eq!(zigbee_crc16(&[]), [0, 0]);
    }

    #[test]
    fn crc16_matches_bit_serial_lfsr() {
        let mut seed = 0x1234_5678u32;
        for len in 0..64 {
            let mut buf = [0u8; 64];
            for b in buf.iter_mut().take(len) {
                seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
                *b = (seed >> 16) as u8;
            }
            assert_eq!(zigbee_crc16(&buf[..len]), bit_serial_crc16(&buf[..len]));
        }
    }

    #[test]
    fn crc16_of_frame_with_fcs_is_zero_residue() {
        // Beacon request: FCF 0x0803, DSN 0x2a, dst PAN/addr 0xffff/0xffff, cmd 7.
        let mut f = [0x03, 0x08, 0x2a, 0xff, 0xff, 0xff, 0xff, 0x07, 0, 0];
        let fcs = zigbee_crc16(&f[..8]);
        f[8..].copy_from_slice(&fcs);
        assert_eq!(zigbee_crc16(&f), [0, 0]);
    }

    #[test]
    fn channel_index_matches_sdk_formula() {
        assert_eq!(rf_channel_index(11), 5);
        assert_eq!(rf_channel_index(15), 25);
        assert_eq!(rf_channel_index(26), 80);
        assert_eq!(rf_channel_index(5), 5);
        assert_eq!(rf_channel_index(30), 80);
    }

    #[test]
    fn tpcal_calc_matches_sdk_macro() {
        // tp0 > tp1: linear interpolation with rounding.
        assert_eq!(tpcal_for_rf_channel(0x30, 0x20, 2), 0x30);
        // idx = (66-2)>>1 = 32 → tp1 exactly.
        assert_eq!(tpcal_for_rf_channel(0x30, 0x20, 66), 0x20);
        // idx = (34-2)>>1 = 16 → midpoint.
        assert_eq!(tpcal_for_rf_channel(0x30, 0x20, 34), 0x28);
        // tp0 <= tp1: SDK uses tp0 for every channel.
        assert_eq!(tpcal_for_rf_channel(0x20, 0x30, 80), 0x20);
        // rfChn clamped to 2..=80.
        assert_eq!(
            tpcal_for_rf_channel(0x30, 0x20, 0),
            tpcal_for_rf_channel(0x30, 0x20, 2)
        );
        assert_eq!(
            tpcal_for_rf_channel(0x30, 0x20, 200),
            tpcal_for_rf_channel(0x30, 0x20, 80)
        );
    }

    #[test]
    fn tx_power_tables() {
        assert_eq!(tx_power_code(PaTable::QFN32, 0), (0x1f, false));
        assert_eq!(tx_power_code(PaTable::QFN32, 10), (0x3f, true));
        assert_eq!(tx_power_code(PaTable::QFN32, -1), (0x0f, false));
        assert_eq!(tx_power_code(PaTable::QFN32, -40), (0x01, false));
        assert_eq!(tx_power_code(PaTable::TSOP16, 0), (0x0d, false));
        assert_eq!(tx_power_code(PaTable::TSOP16, 5), (0x1d, false));
        assert_eq!(tx_power_code(PaTable::TSOP16, -12), (0x02, false));
    }

    #[test]
    fn fifo_words_are_little_endian_with_residue() {
        let collect = |len: u8, frame: &[u8], fcs: &[u8]| {
            let mut buf = [0u8; FIFO_BYTES];
            let words = pack_tx_fifo(&mut buf, len, frame, fcs);
            buf.chunks_exact(4)
                .take(words)
                .map(|w| u32::from_le_bytes([w[0], w[1], w[2], w[3]]))
                .collect::<heapless::Vec<u32, FIFO_WORDS>>()
        };
        assert_eq!(collect(1, &[2, 3], &[4, 5])[..], [0x0403_0201, 0x0000_0005]);
        assert_eq!(collect(9, &[8, 7, 6], &[])[..], [0x0607_0809]);
        assert_eq!(collect(7, &[], &[])[..], [0x0000_0007]);
        let full = [0xAAu8; MAX_MPDU_LEN];
        assert_eq!(collect(127, &full, &[0xBB, 0xCC]).len(), FIFO_WORDS);
        assert_eq!(collect(127, &full, &[]).len(), 32);
    }

    #[test]
    fn rfifo_word_count_covers_len_byte_and_psdu() {
        for blen in 0u8..=127 {
            let words = rfifo_packet_words(blen);
            assert!(words * 4 > usize::from(blen));
            assert!(words <= FIFO_WORDS);
        }
        assert_eq!(rfifo_packet_words(5), 2);
        assert_eq!(rfifo_packet_words(127), 32);
    }

    #[test]
    fn mpdu_len_strips_fcs_and_rejects_out_of_range() {
        assert_eq!(zb_rfifo_mpdu_len(5), Some(3));
        assert_eq!(zb_rfifo_mpdu_len(127), Some(125));
        assert_eq!(zb_rfifo_mpdu_len(4), None);
        assert_eq!(zb_rfifo_mpdu_len(0), None);
        assert_eq!(zb_rfifo_mpdu_len(128), None);
    }

    #[test]
    fn rssi_is_negated_foot1_top_byte() {
        assert_eq!(rssi_from_foot1(0x4B00_0000), -75);
        assert_eq!(rssi_from_foot1(0), 0);
        assert_eq!(rssi_from_foot1(0xFF00_0000), -127);
    }

    #[test]
    fn rssi_to_lqi_is_monotonic_and_bounded() {
        assert_eq!(rssi_to_lqi(-100), 0);
        assert_eq!(rssi_to_lqi(-20), 255);
        assert!(rssi_to_lqi(-60) > rssi_to_lqi(-90));
    }

    #[test]
    fn cca_threshold_is_minus_60_dbm() {
        assert_eq!(CCA_THRESHOLD_DBM, -60);
    }
}
