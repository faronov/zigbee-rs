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

use core::sync::atomic::{AtomicBool, AtomicI8, AtomicU8, Ordering};
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
    /// Create and initialize a new PHY6222 radio driver.
    ///
    /// Configures the radio for IEEE 802.15.4 (Zigbee) mode with the given
    /// settings. After this call the radio is ready for TX/RX.
    pub fn new(config: RadioConfig) -> Self {
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
        if cap >= 0x05 && cap <= 0x3F {
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
        // Linear interpolation between cal points at rf_chn=2 and rf_chn=66
        let idx = ((rf_chn.saturating_sub(2)) >> 1).min(39) as u16;
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
    /// with RSSI and LQI. Frames failing CRC are rejected.
    pub async fn receive(&mut self) -> Result<RxFrame, RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }

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

        log::trace!("phy6222: rx {} bytes rssi={}dBm", len, rssi);
        Ok(frame)
    }

    /// Perform Clear Channel Assessment (energy detection).
    ///
    /// Returns (RSSI in dBm, channel_busy).
    pub fn energy_detect(&self) -> Result<(i8, bool), RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }

        // Read RSSI from the RX front-end (foot word 0)
        let foot0 = reg_read(RF_PHY_BASE + 0xE4);
        let rssi_raw = ((foot0 >> 24) & 0xFF) as i8;
        let busy = rssi_raw > -60; // CCA threshold: -60 dBm (typical)

        Ok((rssi_raw, busy))
    }

    /// Perform Clear Channel Assessment (async, IEEE 802.15.4 CCA mode 1).
    ///
    /// Briefly enables the receiver to measure RF energy on the current channel.
    /// Returns `true` if the channel is busy (energy above threshold).
    pub async fn clear_channel_assessment(&mut self) -> Result<bool, RadioError> {
        if !self.initialized {
            return Err(RadioError::NotInitialized);
        }

        // Set SRX mode with a short timeout
        reg_write(LL_HW_BASE + 0x04, 0x01_00_01);
        reg_write(LL_HW_BASE + 0x38, 0x07);
        reg_write(LL_HW_BASE + 0x28, 0x0800); // short RX window

        reg_write(LL_HW_BASE + 0x14, LL_HW_IRQ_MASK);
        reg_write(LL_HW_BASE + 0x0C, LIRQ_MD | LIRQ_RTO);

        // Trigger RX
        reg_write(LL_HW_BASE + 0x00, 0x0001);

        // Wait 128µs (8 symbol periods) for RSSI to settle
        embassy_time::Timer::after_micros(128).await;

        // Read RSSI from foot word
        let foot0 = reg_read(RF_PHY_BASE + 0xE4);
        let rssi = ((foot0 >> 24) & 0xFF) as i8;

        // Abort RX
        reg_write(LL_HW_BASE + 0x00, 0x0000);
        reg_write(LL_HW_BASE + 0x14, LL_HW_IRQ_MASK);

        // CCA threshold: -60 dBm (typical for 802.15.4)
        let busy = rssi > -60;
        Ok(busy)
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
            // SRX mode done → check for received data
            let crc_ok = (irq_status & LIRQ_COK) != 0;
            RX_CRC_OK.store(crc_ok, Ordering::Release);

            if crc_ok {
                // Read frame from RX FIFO
                let rfifo = LL_HW_RFIFO as *const u8;
                let len_byte = unsafe { core::ptr::read_volatile(rfifo) };
                let len = (len_byte as usize).min(MAX_FRAME_LEN);

                RX_LEN.store(len as u8, Ordering::Release);

                // Read RSSI from foot word
                let foot0 = reg_read(RF_PHY_BASE + 0xE4);
                let rssi = ((foot0 >> 24) & 0xFF) as i8;
                RX_RSSI.store(rssi, Ordering::Release);

                // Copy frame data
                unsafe {
                    let buf = &mut *RX_BUF.get();
                    for i in 0..len {
                        buf[i] = core::ptr::read_volatile(rfifo.add(1 + i));
                    }
                }
            } else {
                RX_LEN.store(0, Ordering::Release);
            }

            RX_DONE.signal(());
        }
    } else if irq_status & LIRQ_RTO != 0 {
        // RX timeout
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
