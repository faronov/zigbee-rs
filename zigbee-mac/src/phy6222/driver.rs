//! Pure-Rust IEEE 802.15.4 radio driver for Phyplus PHY6222/6252.
//!
//! The PHY6222 is an ARM Cortex-M0 BLE SoC whose radio hardware also supports
//! IEEE 802.15.4 (Zigbee) modulation. This driver configures the radio entirely
//! through memory-mapped registers — **no vendor binary blobs required**.
//!
//! # Radio hardware
//! - 2.4 GHz multi-protocol: BLE 1M/2M/LR + IEEE 802.15.4 (O-QPSK, 250 kbps)
//! - TX power: -20 dBm to +10 dBm (5-bit DAC, register `0x400300b8`)
//! - CRC-16 in hardware (Zigbee FCS polynomial)
//! - Hardware whitening bypass for 802.15.4
//! - RSSI per-packet via `0x400300e4` foot word
//!
//! # Architecture
//! ```text
//! Phy6222Driver (pure Rust, async)
//!   ├── RF PHY registers (0x40030000..0x40030110)
//!   │     ├── rf_phy_bb_cfg()   → baseband for Zigbee mode
//!   │     ├── rf_phy_ana_cfg()  → PLL, LNA, PA configuration
//!   │     └── set_channel()     → frequency synthesis
//!   ├── LL HW registers (0x40031000..0x40031060)
//!   │     ├── ll_hw_set_stx()   → single TX mode
//!   │     ├── ll_hw_set_srx()   → single RX mode
//!   │     └── ll_hw_trigger()   → start operation
//!   ├── TX FIFO (0x40031400)    → write frame data
//!   ├── RX FIFO (0x40031C00)    → read received frames
//!   └── IRQ → Embassy Signal for async completion
//! ```
//!
//! # Register map source
//! Derived from the open-source PHY6222 SDK (`rf_phy_driver.c`, `ll_hw_drv.c`)
//! available at: <https://github.com/pvvx/THB2>

// `LL_HW_BASE + 0x00` (the trigger register) is written out with an
// explicit `+ 0x00` offset throughout this file, matching every other
// `reg_write(BASE + offset, ...)` call site and the register map comments
// above each block. This is a deliberate readability/consistency choice
// for a file that is effectively a register map transcription — silence
// clippy's `identity_op` for it rather than making the trigger register
// the one visual outlier among dozens of sibling register writes.
#![allow(clippy::identity_op)]

use core::cell::Cell;
use core::sync::atomic::{AtomicI8, AtomicU8, Ordering};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

// ── PHY6222 Register Addresses ──────────────────────────────────

/// RF PHY analog/baseband register block.
const RF_PHY_BASE: u32 = 0x4003_0000;

/// Link-Layer hardware engine base.
const LL_HW_BASE: u32 = 0x4003_1000;

/// TX FIFO base address (write frame data here).
const LL_HW_TFIFO: u32 = 0x4003_1400;

/// RX FIFO base address (read received frames here).
const LL_HW_RFIFO: u32 = 0x4003_1C00;

/// BB HW base (alias used for channel register).
const BB_HW_BASE: u32 = 0x4003_0000;

/// Global clock control register (xtal output enable, clock mux).
const CLK_CTRL_REG: u32 = 0x4000_F040;
const CLK_CFG_REG: u32 = 0x4000_F044;

// ── LL HW IRQ bits ──────────────────────────────────────────────

/// Mode-done interrupt (TX or RX operation complete).
const LIRQ_MD: u32 = 0x01;
/// CRC OK on received packet.
const LIRQ_COK: u32 = 0x02;
/// CRC error on received packet.
const LIRQ_CERR: u32 = 0x04;
/// RX timeout (no packet received within window).
const LIRQ_RTO: u32 = 0x08;

/// Mask covering all LL HW interrupt sources.
const LL_HW_IRQ_MASK: u32 = 0x1F;

// ── CRC format constants ────────────────────────────────────────

/// Zigbee CRC-16 format (ITU-T polynomial for 802.15.4).
const LL_HW_CRC_ZB_FMT: u32 = 0x03;

// ── Packet format ───────────────────────────────────────────────

/// Zigbee / IEEE 802.15.4 packet format selector.
const _PKT_FMT_ZIGBEE: u8 = 0;

// ── Register access helpers ─────────────────────────────────────

#[inline(always)]
fn reg_write(addr: u32, val: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) };
}

#[inline(always)]
fn reg_read(addr: u32) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

/// Write a bitfield: reg[hi:lo] = value.
#[inline(always)]
fn sub_write_reg(addr: u32, hi: u8, lo: u8, value: u32) {
    let mask = ((1u32 << (hi - lo + 1)) - 1) << lo;
    let old = reg_read(addr);
    reg_write(addr, (old & !mask) | ((value << lo) & mask));
}

// ── Async completion signals ────────────────────────────────────

/// TX completion signal (set from IRQ handler).
static TX_DONE: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// RX completion signal (set from IRQ handler).
static RX_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static DRIVER_TAKEN: Mutex<CriticalSectionRawMutex, Cell<bool>> = Mutex::new(Cell::new(false));

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

/// Maximum IEEE 802.15.4 on-air PSDU size (`aMaxPHYPacketSize` = 127 bytes,
/// including the 2-byte FCS/CRC-16).
const MAX_FRAME_LEN: usize = 127;

/// Maximum MPDU length (MAC header + payload) that may be handed to
/// [`Phy6222Driver::transmit`] or read back from [`Phy6222Driver::receive`].
///
/// The hardware appends (TX) / includes (RX) a 2-byte FCS on top of this,
/// so `MAX_MPDU_LEN + 2 == MAX_FRAME_LEN` keeps the on-air PSDU within
/// `aMaxPHYPacketSize`. See `transmit()` and `zb_rfifo_mpdu_len()`.
const MAX_MPDU_LEN: usize = MAX_FRAME_LEN - 2;

/// CCA / energy-detect busy threshold, in dBm. Matches the typical
/// IEEE 802.15.4 CCA Mode 1 (energy above threshold) default.
const CCA_THRESHOLD_DBM: i8 = -60;

static RX_BUF: SyncUnsafeCell<[u8; MAX_FRAME_LEN]> = SyncUnsafeCell::new([0u8; MAX_FRAME_LEN]);
static RX_LEN: AtomicU8 = AtomicU8::new(0);
static RX_RSSI: AtomicI8 = AtomicI8::new(-127);

/// Outcome of the most recently completed SRX (single-RX) hardware
/// operation. Replaces a bare "CRC ok?" boolean so that a genuine RX
/// timeout (`LIRQ_RTO`, no frame received at all) can be told apart from
/// a frame that *was* received but failed its CRC check — both used to
/// collapse into the same "not CRC-OK" state and were reported to callers
/// as [`RadioError::CrcError`], hiding real timeouts. See `receive()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum RxOutcome {
    /// No SRX has completed since the last reset — `RX_DONE` should not
    /// yet be signalled while this is the value; treated defensively as a
    /// hardware error if ever observed after a signal.
    Pending = 0,
    /// Mode-done with good CRC — a frame is available in `RX_BUF`/`RX_LEN`.
    FrameOk = 1,
    /// Mode-done but the CRC check failed.
    CrcError = 2,
    /// The RX window elapsed with no frame received at all.
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

static RX_OUTCOME: AtomicU8 = AtomicU8::new(RxOutcome::Pending as u8);

/// Classify a raw LL HW IRQ status word for an SRX (single-RX) mode-done or
/// timeout event.
///
/// Kept as a small, pure, hardware-independent function (no register
/// access) precisely so the mode-done/CRC/timeout precedence can be
/// exercised with host tests — see `driver::tests`. `LL_IRQ` calls this
/// only for the RX-relevant events (mode-done while in SRX mode, or an
/// `LIRQ_RTO` with no mode-done) it has already identified via the mode
/// register, matching this function's precedence: mode-done takes priority
/// over a (mutually exclusive, per the PHY6222 LL HW) timeout bit.
fn classify_rx_irq(irq_status: u32) -> RxOutcome {
    if irq_status & LIRQ_MD != 0 {
        if irq_status & LIRQ_COK != 0 {
            RxOutcome::FrameOk
        } else {
            RxOutcome::CrcError
        }
    } else if irq_status & LIRQ_RTO != 0 {
        RxOutcome::Timeout
    } else {
        RxOutcome::Pending
    }
}

/// Convert the PHY6222 Zigbee RFIFO length byte into the MPDU length
/// (MAC header + payload, FCS stripped) used by the rest of this driver.
///
/// Per the public pvvx/THB2 PHY6222 SDK (`ll_hw_drv.c`,
/// `ll_hw_read_rfifo_zb()`): the length byte popped from the head of the
/// RX FIFO is the over-the-air PSDU length and "blen included the 2 byte
/// crc" — i.e. it counts the trailing 2-byte FCS/CRC-16, exactly like the
/// length byte `transmit()` writes to the TX FIFO (`frame.len() + 2`).
/// Both driver.rs FIFO consumers must agree on this convention: after
/// popping the length byte, the following `len_byte` bytes in the FIFO are
/// `[MHR][payload][fcs_lo][fcs_hi]`. This function strips the trailing 2
/// FCS bytes so `RX_LEN`/`RX_BUF` (and therefore `RxFrame`) hold only the
/// MPDU, matching what `transmit()` sends and what the MAC layer parses
/// (FCF, addressing, payload — no FCS).
fn zb_rfifo_mpdu_len(len_byte: u8) -> usize {
    (len_byte as usize).saturating_sub(2).min(MAX_MPDU_LEN)
}

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
    /// TX power in dBm (clamped to 0–10).
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

/// Pure-Rust async radio driver for PHY6222 IEEE 802.15.4 mode.
pub struct Phy6222Driver {
    config: RadioConfig,
    initialized: bool,
    /// TP calibration values (computed during init).
    tp_cal0: u8,
    tp_cal1: u8,
}

impl Phy6222Driver {
    /// Acquire and initialize the process-wide PHY6222 radio driver.
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
            tp_cal0: 0x23,
            tp_cal1: 0x23,
        };
        drv.init_hardware();
        drv
    }

    // ── Hardware initialization ─────────────────────────────────

    /// Full radio initialization: clocks → analog → baseband → Zigbee CRC.
    fn init_hardware(&mut self) {
        // Enable crystal output to digital
        sub_write_reg(CLK_CTRL_REG, 18, 18, 1);

        // Clock source: 16 MHz XTAL for RF PHY, 32 MHz doubled for RX ADC
        sub_write_reg(CLK_CFG_REG, 25, 24, 0x00); // rxAdcClkSel = 32M DBL
        sub_write_reg(CLK_CFG_REG, 23, 22, 0x00); // rfPhyClkSel = 16M XTAL
        sub_write_reg(CLK_CFG_REG, 6, 5, 0x03); // trim DLL/DBL LDO vout
        sub_write_reg(CLK_CFG_REG, 8, 8, 1); // DBL enable
        sub_write_reg(CLK_CFG_REG, 19, 18, 0x03); // RX ADC clk en, RF PHY clk en

        // Indicate 16 MHz reference clock to RFPLL
        sub_write_reg(RF_PHY_BASE + 0x80, 0, 0, 1);

        // Configure analog blocks: PLL, RX front-end, PA
        self.rf_phy_ana_cfg();

        // Set TX power
        self.rf_phy_set_tx_power(self.config.tx_power);

        // Configure baseband for Zigbee (802.15.4) mode
        self.rf_phy_bb_cfg();

        // Configure LL HW timing for Zigbee packet format
        self.ll_hw_set_timing();

        // Set Zigbee CRC format (CRC-16 ITU-T)
        self.ll_hw_set_crc_fmt();

        // Run two-point calibration
        self.rf_tp_calibrate();

        // Apply channel/address config
        self.apply_config();

        self.initialized = true;
        log::info!("phy6222: radio initialized in IEEE 802.15.4 mode");
    }

    /// Configure RF analog blocks: PLL, LNA, PA.
    fn rf_phy_ana_cfg(&self) {
        // PLL control
        reg_write(RF_PHY_BASE + 0xCC, 0x2000_0BC0); // i_pll_ctrl0
        reg_write(RF_PHY_BASE + 0xD0, 0x0000_0180); // i_pll_ctrl1 (TX PLL BW)
        reg_write(RF_PHY_BASE + 0xD4, 0x076A_3E7A); // i_pll_ctrl2 (lpf, boost vco)
        reg_write(RF_PHY_BASE + 0xD8, 0x0489_0000); // i_pll_ctrl3 (vco/tp varactor)

        // RX PLL BW (active when rx_en)
        reg_write(RF_PHY_BASE + 0x104, 0x0000_0180); // i_pll_ctrl5
        reg_write(RF_PHY_BASE + 0x108, 0x076A_3E7A); // i_pll_ctrl6
        reg_write(RF_PHY_BASE + 0x10C, 0x0489_8000); // i_pll_ctrl7

        // VCO coarse tuning
        reg_write(
            RF_PHY_BASE + 0x80,
            reg_read(RF_PHY_BASE + 0x80) | 0x0000_24CC,
        );

        // RX front-end (boost TIA current)
        reg_write(RF_PHY_BASE + 0xDC, 0x01A6_FC2F);

        // DAC delay
        reg_write(RF_PHY_BASE + 0xB0, 0x0100_0003);
    }

    /// Configure baseband for IEEE 802.15.4 (Zigbee) packet format.
    ///
    /// Sets PGA bandwidth, modulation, sync word, preamble, and disables
    /// Gaussian shaping (802.15.4 uses O-QPSK, not GFSK).
    fn rf_phy_bb_cfg(&self) {
        // Zigbee shares PGA BW setting with BLE 2M (smaller bandwidth)
        reg_write(RF_PHY_BASE + 0xE0, 0x0000_0080); // PGA BW: small
        sub_write_reg(RF_PHY_BASE + 0xD8, 20, 18, 0x02); // TPM DAC varactor

        // DC offset for 16 MHz reference clock
        reg_write(RF_PHY_BASE + 0x90, 0x0008_0000);

        // Zigbee-specific baseband configuration
        reg_write(RF_PHY_BASE + 0x00, 0x7806_8000); // Zigbee sync/pkt format
        reg_write(RF_PHY_BASE + 0x48, 0x0000_0000); // CRC/whitening seed = 0
        reg_write(RF_PHY_BASE + 0x40, 0x000B_2800); // Disable Gaussian shaping
        reg_write(RF_PHY_BASE + 0x4C, 0x3675_EE07); // Zigbee sync word / access addr

        // Max packet length for 802.15.4 (127 + length byte)
        reg_write(RF_PHY_BASE + 0x0C, 128);
    }

    /// Configure LL HW timing parameters for Zigbee packet format.
    fn ll_hw_set_timing(&self) {
        // TX-to-RX turnaround: 192 µs (802.15.4 spec: aTurnaroundTime = 12 symbols = 192 µs)
        let hclk_per_us: u32 = 16; // 16 MHz HCLK
        reg_write(LL_HW_BASE + 0x18, 192 * hclk_per_us);
        // RX-to-TX turnaround
        reg_write(LL_HW_BASE + 0x1C, 192 * hclk_per_us);
        // TX/RX settle time
        reg_write(
            LL_HW_BASE + 0x08,
            (80 * hclk_per_us) << 16 | (80 * hclk_per_us),
        );
    }

    /// Set Zigbee CRC format on both TX and RX paths.
    fn ll_hw_set_crc_fmt(&self) {
        // Register at LL_HW_BASE + 0x34: [3:0] = TX CRC fmt, [7:4] = RX CRC fmt
        let val = LL_HW_CRC_ZB_FMT | (LL_HW_CRC_ZB_FMT << 4);
        reg_write(LL_HW_BASE + 0x34, val);
    }

    /// Run two-point RF calibration for Zigbee mode.
    fn rf_tp_calibrate(&mut self) {
        // Zigbee uses the same calibration path as BLE 2M
        // Calibrate at channel 2 (low) and channel 66 (high) of the RF index
        self.tp_cal0 = self.rf_tp_cal(2) + 4;
        self.tp_cal1 = self.rf_tp_cal(66) + 4;
    }

    /// Single-point TP calibration at a given RF channel index.
    ///
    /// Triggers VCO coarse tuning at the given RF channel and reads back
    /// the calibration capacitor value. Falls back to 0x20 if the read
    /// returns an out-of-range value.
    fn rf_tp_cal(&self, rf_chn: u8) -> u8 {
        // Set the target RF channel for calibration
        reg_write(BB_HW_BASE + 0xB4, rf_chn as u32);

        // Trigger VCO coarse calibration
        let old = reg_read(RF_PHY_BASE + 0x80);
        reg_write(RF_PHY_BASE + 0x80, old | 0x0000_0400); // cal_en bit

        // Wait for calibration to settle (~50µs per PHY6222 SDK)
        for _ in 0..800 {
            core::hint::spin_loop();
        }

        // Read back calibration cap value from VCO tune register
        let cal_reg = reg_read(RF_PHY_BASE + 0x94);
        let cap = (cal_reg & 0xFF) as u8;

        // Clear cal_en
        reg_write(RF_PHY_BASE + 0x80, old);

        // Sanity check: valid cap range is roughly 0x05..0x3F
        if (0x05..=0x3F).contains(&cap) {
            cap
        } else {
            0x20 // fallback default
        }
    }

    /// Set TX power (0–10 dBm, 5-bit DAC).
    fn rf_phy_set_tx_power(&self, dbm: i8) {
        let power = (dbm.clamp(0, 10) as u32) & 0x1F;
        let old = reg_read(RF_PHY_BASE + 0xB8);
        reg_write(RF_PHY_BASE + 0xB8, (old & 0x0FFF) | (power << 12));
    }

    /// Set RF channel for IEEE 802.15.4.
    ///
    /// 802.15.4 channels 11–26 map to center frequencies 2405–2480 MHz.
    /// The radio uses an RF channel index: `(freq_mhz - 2400) = rf_chn`.
    /// Channel 11 → rf_chn=5, channel 26 → rf_chn=80.
    fn set_channel(&self, channel: u8) {
        let ch = channel.clamp(11, 26);
        let rf_chn = (ch - 11) * 5 + 5; // ch11→5, ch12→10, ... ch26→80
        reg_write(BB_HW_BASE + 0xB4, rf_chn as u32);

        // Apply TP calibration for this channel
        let cap = self.tp_cal_for_channel(rf_chn);
        sub_write_reg(RF_PHY_BASE + 0x94, 7, 0, cap as u32);
    }

    /// Interpolate TP calibration cap value for a given RF channel index.
    fn tp_cal_for_channel(&self, rf_chn: u8) -> u8 {
        // Interpolate only inside the measured 2..=66 range. Zigbee
        // channels 25-26 map above the high point and must use that endpoint
        // rather than extrapolating an unmeasured capacitor value.
        let calibrated_channel = rf_chn.clamp(2, 66);
        let idx = ((calibrated_channel - 2) >> 1) as u16;
        let cal0 = self.tp_cal0 as u16;
        let cal1 = self.tp_cal1 as u16;
        let result = if cal1 >= cal0 {
            cal0 + (idx * (cal1 - cal0)) / 32
        } else {
            cal0 - (idx * (cal0 - cal1)) / 32
        };
        result as u8
    }

    /// Apply current configuration to hardware.
    fn apply_config(&self) {
        self.set_channel(self.config.channel);
        self.rf_phy_set_tx_power(self.config.tx_power);
        // Address filtering is handled in software for this driver
        // (the LL HW doesn't have hardware address matching for Zigbee mode)
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
    /// `frame` must be the MAC header + payload **without** an FCS — the
    /// radio hardware appends the 2-byte FCS (CRC-16) automatically. Since
    /// the on-air PSDU is `frame.len() + 2` and must stay within
    /// `aMaxPHYPacketSize` (127 bytes), `frame` itself is bounded by
    /// [`MAX_MPDU_LEN`] (125 bytes), not [`MAX_FRAME_LEN`].
    pub async fn transmit(&mut self, frame: &[u8]) -> Result<(), RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }
        if frame.is_empty() || frame.len() > MAX_MPDU_LEN {
            return Err(RadioError::InvalidFrame);
        }

        // Defensively abort any SRX left running by a `receive()` future
        // that was cancelled (e.g. by `embassy_futures::select` racing a
        // timeout) before it reached `RX_DONE.wait().await`'s completion,
        // and clear any IRQ status latched by it, so this TX starts from a
        // clean hardware state. See `receive()` for the matching rationale.
        reg_write(LL_HW_BASE + 0x00, 0x0000);
        reg_write(LL_HW_BASE + 0x14, LL_HW_IRQ_MASK);

        TX_DONE.reset();

        // Set single-TX mode
        reg_write(LL_HW_BASE + 0x04, 0x00_01_00); // txNum=1, rxNum=0, mode=STX
        reg_write(LL_HW_BASE + 0x38, 0x07); // bypass SN/NESN/MD

        // Reset TX FIFO
        reg_write(LL_HW_BASE + 0x40, 0x01); // TFIFO reset

        // Write frame to TX FIFO: [length_byte, frame_data...]
        // The LL HW expects a length byte followed by the PSDU
        let tfifo = LL_HW_TFIFO as *mut u8;
        unsafe {
            // Length byte (PSDU length = frame.len() + 2 for FCS appended by HW)
            core::ptr::write_volatile(tfifo, (frame.len() + 2) as u8);
            // Frame data
            for (i, &b) in frame.iter().enumerate() {
                core::ptr::write_volatile(tfifo.add(1 + i), b);
            }
        }

        // Clear IRQ status and enable mode-done IRQ
        reg_write(LL_HW_BASE + 0x14, LL_HW_IRQ_MASK);
        reg_write(LL_HW_BASE + 0x0C, LIRQ_MD);

        // Trigger!
        reg_write(LL_HW_BASE + 0x00, 0x0001);

        log::trace!(
            "phy6222: tx {} bytes on ch{}",
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
    /// (FCS stripped, see [`zb_rfifo_mpdu_len`]) with RSSI and LQI.
    /// Returns [`RadioError::CrcError`] for a frame that failed its CRC
    /// check, or [`RadioError::RxTimeout`] if the RX window elapsed with
    /// no frame at all — the two are distinguished via [`RxOutcome`]
    /// rather than collapsed into one "not ok" state.
    ///
    /// # Cancellation
    /// Callers (the MAC layer) routinely race this future against a timer
    /// with `embassy_futures::select` and drop it if the timer wins. When
    /// that happens the SRX operation this call started may still be
    /// running in hardware. The next call to `receive()` (or `transmit()`)
    /// defensively aborts that stale SRX and clears any IRQ/status it may
    /// have latched *before* starting its own operation, so a leftover
    /// operation from a cancelled `receive()` can't corrupt or pre-empt
    /// the next one.
    pub async fn receive(&mut self) -> Result<RxFrame, RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }

        // Abort any previous SRX left running by a cancelled `receive()`
        // future and clear stale IRQ/status latched from it (see doc
        // comment above) before touching `RX_DONE`/`RX_OUTCOME`, so a
        // late spurious IRQ from that stale operation can't race us.
        reg_write(LL_HW_BASE + 0x00, 0x0000);
        reg_write(LL_HW_BASE + 0x14, LL_HW_IRQ_MASK);
        RX_OUTCOME.store(RxOutcome::Pending as u8, Ordering::Release);
        RX_DONE.reset();

        // Set single-RX mode
        reg_write(LL_HW_BASE + 0x04, 0x01_00_01); // txNum=0, rxNum=1, mode=SRX
        reg_write(LL_HW_BASE + 0x38, 0x07); // bypass SN/NESN/MD

        // Set RX timeout (10 seconds in HCLK ticks — long window for Zigbee)
        reg_write(LL_HW_BASE + 0x28, 0xFFFF); // max timeout

        // Clear IRQ and enable mode-done + CRC OK/ERR + RX timeout
        reg_write(LL_HW_BASE + 0x14, LL_HW_IRQ_MASK);
        reg_write(LL_HW_BASE + 0x0C, LIRQ_MD | LIRQ_COK | LIRQ_CERR | LIRQ_RTO);

        // Trigger RX
        reg_write(LL_HW_BASE + 0x00, 0x0001);

        // Wait for RX completion
        RX_DONE.wait().await;

        match RxOutcome::from_u8(RX_OUTCOME.load(Ordering::Acquire)) {
            RxOutcome::CrcError => return Err(RadioError::CrcError),
            RxOutcome::Timeout => return Err(RadioError::RxTimeout),
            // `RX_DONE` fired without `LL_IRQ` recording an outcome — should
            // not happen, but fail safe rather than read stale RX_LEN/RX_BUF.
            RxOutcome::Pending => return Err(RadioError::HardwareError),
            RxOutcome::FrameOk => {}
        }

        let len = RX_LEN.load(Ordering::Acquire) as usize;
        if len == 0 {
            // Defensive: `FrameOk` should always carry a non-zero MPDU.
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

        log::trace!("phy6222: rx {} bytes rssi={}dBm", len, rssi);
        Ok(frame)
    }

    /// Briefly enable the receiver on the current channel and measure RSSI.
    ///
    /// Shared by [`Self::energy_detect`] and [`Self::clear_channel_assessment`]
    /// — both need to actually turn the receiver on for a short window and
    /// let the RSSI front-end settle before reading the foot-word RSSI
    /// register; reading that register without first enabling SRX would
    /// return whatever value happened to be latched from an earlier,
    /// unrelated RX (or nothing, on a cold radio).
    async fn measure_channel_rssi(&mut self) -> Result<i8, RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }

        // Defensively abort any stale SRX (e.g. a cancelled `receive()`)
        // and clear stale IRQ status first — see `receive()`.
        reg_write(LL_HW_BASE + 0x00, 0x0000);
        reg_write(LL_HW_BASE + 0x14, LL_HW_IRQ_MASK);

        // Set SRX mode with a short timeout, just to energize the receiver
        // and sample RF energy — we don't care about demodulating a frame.
        reg_write(LL_HW_BASE + 0x04, 0x01_00_01);
        reg_write(LL_HW_BASE + 0x38, 0x07);
        reg_write(LL_HW_BASE + 0x28, 0x0800); // short RX window

        reg_write(LL_HW_BASE + 0x14, LL_HW_IRQ_MASK);
        reg_write(LL_HW_BASE + 0x0C, LIRQ_MD | LIRQ_RTO);

        // Trigger RX
        reg_write(LL_HW_BASE + 0x00, 0x0001);

        // Wait 128µs (8 symbol periods, the minimum 802.15.4 ED duration)
        // for RSSI to settle.
        embassy_time::Timer::after_micros(128).await;

        // Read RSSI from foot word
        let foot0 = reg_read(RF_PHY_BASE + 0xE4);
        let rssi = ((foot0 >> 24) & 0xFF) as i8;

        // Abort RX and clear IRQ status so the next operation starts clean.
        reg_write(LL_HW_BASE + 0x00, 0x0000);
        reg_write(LL_HW_BASE + 0x14, LL_HW_IRQ_MASK);

        Ok(rssi)
    }

    /// Perform an IEEE 802.15.4 Energy Detect scan on the current channel.
    ///
    /// Actually enables the receiver for a short window and measures RF
    /// energy (via [`Self::measure_channel_rssi`]) rather than reading a
    /// possibly-stale RSSI register — the channel to measure is whatever
    /// `RadioConfig::channel` is currently applied to hardware (callers
    /// select the channel via `update_config` before calling this, e.g.
    /// during `MLME-SCAN.request` with `ScanType::Ed`).
    ///
    /// Returns (RSSI in dBm, channel_busy).
    pub async fn energy_detect(&mut self) -> Result<(i8, bool), RadioError> {
        let rssi = self.measure_channel_rssi().await?;
        Ok((rssi, rssi > CCA_THRESHOLD_DBM))
    }

    /// Perform Clear Channel Assessment (async, IEEE 802.15.4 CCA mode 1).
    ///
    /// Briefly enables the receiver to measure RF energy on the current channel.
    /// Returns `true` if the channel is busy (energy above threshold).
    pub async fn clear_channel_assessment(&mut self) -> Result<bool, RadioError> {
        let rssi = self.measure_channel_rssi().await?;
        Ok(rssi > CCA_THRESHOLD_DBM)
    }

    /// Enable continuous receive mode.
    pub fn enable_rx(&self) {
        if !self.initialized {
            return;
        }
        // Set SRX mode and trigger
        reg_write(LL_HW_BASE + 0x04, 0x01_00_01);
        reg_write(LL_HW_BASE + 0x38, 0x07);
        reg_write(LL_HW_BASE + 0x28, 0xFFFF);
        reg_write(LL_HW_BASE + 0x14, LL_HW_IRQ_MASK);
        reg_write(LL_HW_BASE + 0x0C, LIRQ_MD | LIRQ_COK | LIRQ_CERR | LIRQ_RTO);
        reg_write(LL_HW_BASE + 0x00, 0x0001);
    }

    /// Disable the receiver.
    pub fn disable_rx(&self) {
        // Abort current operation by writing 0 to trigger
        reg_write(LL_HW_BASE + 0x00, 0x0000);
        reg_write(LL_HW_BASE + 0x14, LL_HW_IRQ_MASK); // clear pending IRQs
    }

    /// Power down the radio analog blocks to save power between TX/RX.
    ///
    /// Shuts off PLL, LNA, PA, ADC clocks. Call `radio_wake()` before
    /// the next TX or RX operation. Saves ~5–8 mA.
    pub fn radio_sleep(&self) {
        // Disable LL HW engine
        reg_write(LL_HW_BASE + 0x00, 0x0000);
        reg_write(LL_HW_BASE + 0x14, LL_HW_IRQ_MASK);

        // Disable RF PHY clocks: RX ADC clk, RF PHY clk
        sub_write_reg(CLK_CFG_REG, 19, 18, 0x00);
        // Disable DBL (clock doubler)
        sub_write_reg(CLK_CFG_REG, 8, 8, 0);
    }

    /// Re-enable radio analog blocks after `radio_sleep()`.
    ///
    /// Re-enables clocks and reconfigures the analog chain.
    /// Must be called before any TX/RX operation after sleep.
    pub fn radio_wake(&mut self) {
        // Re-enable DBL
        sub_write_reg(CLK_CFG_REG, 8, 8, 1);
        // Re-enable RF PHY clocks
        sub_write_reg(CLK_CFG_REG, 19, 18, 0x03);

        // Small settling delay for PLL
        for _ in 0..1000u32 {
            core::hint::spin_loop();
        }

        // Re-apply channel (re-locks PLL)
        self.set_channel(self.config.channel);
    }
}

// ── IRQ handler ─────────────────────────────────────────────────

/// LL HW interrupt handler — called from the BB (Baseband) IRQ vector.
///
/// This must be registered as the interrupt handler for BB_IRQn
/// (IRQ #4 on PHY6222, verified from SDK bus_dev.h). The handler reads the
/// IRQ status, processes TX/RX completion, and signals the async driver.
///
/// The function name `LL_IRQ` matches the `device.x` linker symbol at
/// vector position 4, overriding the weak `DefaultHandler` alias.
#[unsafe(no_mangle)]
pub extern "C" fn LL_IRQ() {
    let irq_status = reg_read(LL_HW_BASE + 0x10); // IRQ status register

    // Clear all pending IRQs
    reg_write(LL_HW_BASE + 0x14, LL_HW_IRQ_MASK);

    if irq_status & LIRQ_MD != 0 {
        // Mode done — check if this was TX or RX by reading mode register
        let mode = reg_read(LL_HW_BASE + 0x04) & 0x07;

        if mode == 0x00 {
            // STX mode done → TX complete
            TX_DONE.signal(true);
        } else if mode == 0x01 {
            // SRX mode done — `classify_rx_irq` distinguishes CRC-ok from
            // CRC-error for a mode-done event (RTO is not set here, since
            // mode-done and RX-timeout are mutually exclusive per the LL
            // HW; see `classify_rx_irq`'s doc comment).
            let outcome = classify_rx_irq(irq_status);
            RX_OUTCOME.store(outcome as u8, Ordering::Release);

            if outcome == RxOutcome::FrameOk {
                // Read frame from RX FIFO. `len_byte` is the PSDU length
                // as encoded by the PHY6222 Zigbee RFIFO framing and
                // includes the 2-byte FCS; `zb_rfifo_mpdu_len` strips it
                // so `RX_LEN`/`RX_BUF` hold only the MPDU (MHR + payload),
                // matching what `transmit()` sends and what the MAC layer
                // parses. See `zb_rfifo_mpdu_len`'s doc comment.
                let rfifo = LL_HW_RFIFO as *const u8;
                let len_byte = unsafe { core::ptr::read_volatile(rfifo) };
                let len = zb_rfifo_mpdu_len(len_byte);

                RX_LEN.store(len as u8, Ordering::Release);

                // Read RSSI from foot word
                let foot0 = reg_read(RF_PHY_BASE + 0xE4);
                let rssi = ((foot0 >> 24) & 0xFF) as i8;
                RX_RSSI.store(rssi, Ordering::Release);

                // Copy frame data (MPDU only — the 2 FCS bytes still
                // sitting in the FIFO after it are left unread here).
                unsafe {
                    let buf = &mut *RX_BUF.get();
                    for (i, byte) in buf.iter_mut().enumerate().take(len) {
                        *byte = core::ptr::read_volatile(rfifo.add(1 + i));
                    }
                }
            } else {
                RX_LEN.store(0, Ordering::Release);
            }

            RX_DONE.signal(());
        }
    } else if irq_status & LIRQ_RTO != 0 {
        // RX timeout — no frame received at all, distinct from a received
        // frame that failed its CRC check (see `RxOutcome`).
        RX_OUTCOME.store(RxOutcome::Timeout as u8, Ordering::Release);
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

#[cfg(test)]
mod tests {
    use super::*;

    // These tests exercise only pure, hardware-independent helper
    // functions. `Phy6222Driver`/`Phy6222Mac` cannot be constructed on a
    // host test target: `Phy6222Driver::new()` performs unconditional,
    // unsafe MMIO register access (`init_hardware()`), which would fault
    // on any machine other than a real PHY6222 SoC — an existing, pre-dating
    // limitation of this backend, not something introduced or worked
    // around here. See item 6/7/8/9 helpers below.

    #[test]
    fn max_frame_and_mpdu_len_leave_room_for_two_byte_fcs() {
        assert_eq!(MAX_FRAME_LEN, 127); // aMaxPHYPacketSize
        assert_eq!(MAX_MPDU_LEN, 125);
        assert_eq!(MAX_MPDU_LEN + 2, MAX_FRAME_LEN);
    }

    // ── classify_rx_irq (RxTimeout vs CrcError precedence, item 9) ──

    #[test]
    fn classify_rx_irq_mode_done_with_good_crc_is_frame_ok() {
        assert_eq!(classify_rx_irq(LIRQ_MD | LIRQ_COK), RxOutcome::FrameOk);
    }

    #[test]
    fn classify_rx_irq_mode_done_with_bad_crc_is_crc_error() {
        assert_eq!(classify_rx_irq(LIRQ_MD | LIRQ_CERR), RxOutcome::CrcError);
        // Mode-done without the COK bit set at all is also a CRC error,
        // regardless of whether CERR happens to be latched too.
        assert_eq!(classify_rx_irq(LIRQ_MD), RxOutcome::CrcError);
    }

    #[test]
    fn classify_rx_irq_timeout_without_mode_done_is_timeout() {
        assert_eq!(classify_rx_irq(LIRQ_RTO), RxOutcome::Timeout);
    }

    #[test]
    fn classify_rx_irq_neither_bit_is_pending() {
        assert_eq!(classify_rx_irq(0), RxOutcome::Pending);
    }

    #[test]
    fn classify_rx_irq_mode_done_takes_precedence_over_timeout_bit() {
        // The LL HW should never raise MD and RTO together for the same
        // SRX operation, but if it did, a completed frame must win over a
        // stray timeout bit rather than being discarded as a timeout.
        assert_eq!(
            classify_rx_irq(LIRQ_MD | LIRQ_COK | LIRQ_RTO),
            RxOutcome::FrameOk
        );
    }

    #[test]
    fn rx_outcome_round_trips_through_u8() {
        for outcome in [
            RxOutcome::Pending,
            RxOutcome::FrameOk,
            RxOutcome::CrcError,
            RxOutcome::Timeout,
        ] {
            assert_eq!(RxOutcome::from_u8(outcome as u8), outcome);
        }
        // Any other encoded value defensively decodes to `Pending` rather
        // than panicking or aliasing a real outcome.
        assert_eq!(RxOutcome::from_u8(0xFF), RxOutcome::Pending);
    }

    // ── zb_rfifo_mpdu_len (FCS stripping, item 6) ───────────────────

    #[test]
    fn zb_rfifo_mpdu_len_strips_two_byte_fcs() {
        // A 10-byte over-the-air PSDU (8-byte MPDU + 2-byte FCS).
        assert_eq!(zb_rfifo_mpdu_len(10), 8);
    }

    #[test]
    fn zb_rfifo_mpdu_len_at_max_psdu_matches_max_mpdu_len() {
        assert_eq!(zb_rfifo_mpdu_len(MAX_FRAME_LEN as u8), MAX_MPDU_LEN);
    }

    #[test]
    fn zb_rfifo_mpdu_len_saturates_instead_of_underflowing() {
        // A length byte shorter than the FCS itself (corrupt/impossible in
        // practice) must not panic or wrap around `usize`.
        assert_eq!(zb_rfifo_mpdu_len(0), 0);
        assert_eq!(zb_rfifo_mpdu_len(1), 0);
        assert_eq!(zb_rfifo_mpdu_len(2), 0);
    }

    #[test]
    fn zb_rfifo_mpdu_len_caps_at_max_mpdu_len() {
        // Defensive cap even if the length byte somehow exceeded the PHY's
        // own 127-byte maximum.
        assert_eq!(zb_rfifo_mpdu_len(u8::MAX), MAX_MPDU_LEN);
    }

    // ── rssi_to_lqi sanity ───────────────────────────────────────────

    #[test]
    fn rssi_to_lqi_is_monotonic_and_bounded() {
        assert_eq!(rssi_to_lqi(-100), 0);
        assert_eq!(rssi_to_lqi(-20), 255);
        assert!(rssi_to_lqi(-60) > rssi_to_lqi(-90));
        // Out-of-range RSSI values are clamped, not wrapped.
        assert_eq!(rssi_to_lqi(-128), rssi_to_lqi(-100));
        assert_eq!(rssi_to_lqi(0), rssi_to_lqi(-20));
    }

    // ── CCA / ED threshold (item 8 support) ─────────────────────────

    #[test]
    fn cca_threshold_matches_ieee_802_15_4_cca_mode_1_default() {
        assert_eq!(CCA_THRESHOLD_DBM, -60);
    }

    #[test]
    fn tp_calibration_clamps_above_measured_range() {
        let driver = Phy6222Driver {
            config: RadioConfig::default(),
            initialized: false,
            tp_cal0: 0x10,
            tp_cal1: 0x30,
        };
        assert_eq!(driver.tp_cal_for_channel(66), 0x30);
        assert_eq!(driver.tp_cal_for_channel(75), 0x30);
        assert_eq!(driver.tp_cal_for_channel(80), 0x30);
    }
}
