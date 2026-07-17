//! TLSR8258 802.15.4 PHY/DMA register access, transcribed from the official
//! SDK and proven on hardware with tc32-43 and tc32-45.

#![cfg(target_arch = "tc32")]

use crate::mmio::{REG_CLK_EN0, REG_CLK_EN1, REG_CLK_EN2, REG_RST0, REG_RST1, REG_RST2, r8, w8};

// RF control registers
const REG_RF_MODE_CTRL: u32 = 0x800F00; // auto mode control
const REG_RF_SN: u32 = 0x800F01; // SN/NESN reset
const REG_RF_LL_CTRL_0: u32 = 0x800F02; // TX/RX enable
const REG_RF_LL_CTRL_1: u32 = 0x800F03; // timestamp / misc
const REG_RF_TX_SETTLE: u32 = 0x800F04; // TX settle time (u16)
const REG_RF_LL_CTRL_2: u32 = 0x800F15; // TX pipe
const REG_RF_LL_CTRL_3: u32 = 0x800F16; // TRX off/scheduled
const REG_RF_IRQ_MASK: u32 = 0x800F1C; // RF IRQ mask (u16)
const REG_RF_IRQ_STATUS: u32 = 0x800F20; // RF IRQ status (u16)

// RF analog registers
const REG_RF_RX_MODE: u32 = 0x800428; // RX mode enable
const REG_RF_CHANNEL: u32 = 0x80040D; // physical channel
const REG_RF_RSSI: u32 = 0x800441; // RSSI readback
const REG_PLL_FINE_TUNE: u32 = 0x8004D6; // PLL fine divider (u16)

// Modem channel registers (from SDK rf_set_channel disassembly)
const REG_MODEM_CHN_L: u32 = 0x801244;
const REG_MODEM_CHN_H: u32 = 0x801245;
const REG_MODEM_BAND: u32 = 0x801229;

// DMA registers
const REG_DMA2_ADDR: u32 = 0x800C08; // RX DMA addr low (u16)
const REG_DMA2_ADDR_HI: u32 = 0x800C42; // RX DMA addr high
const REG_DMA2_SIZE: u32 = 0x800C0A;
const REG_DMA2_MODE: u32 = 0x800C0B;
const REG_DMA3_ADDR: u32 = 0x800C0C; // TX DMA addr low (u16)
const REG_DMA3_ADDR_HI: u32 = 0x800C43;
const REG_DMA3_SIZE: u32 = 0x800C0E;
const REG_DMA3_MODE: u32 = 0x800C0F;
const REG_DMA_CHN_EN: u32 = 0x800C20;
const REG_DMA_CHN_IRQ_MSK: u32 = 0x800C21;
const REG_DMA_TX_READY: u32 = 0x800C24;
const REG_DMA_IRQ_STATUS: u32 = 0x800C26;
const REG_IRQ_SRC: u32 = 0x800648;
const REG_RF_TX_TRIGGER: u32 = 0x800C5B;

const RF_TRX_MODE: u8 = 0xE0;
const RF_TRX_OFF: u8 = 0x45;
const DEFAULT_TX_SETTLE_US: u16 = 150;

#[inline(always)]
unsafe fn r16(addr: u32) -> u16 {
    unsafe { core::ptr::read_volatile(addr as *const u16) }
}
#[inline(always)]
unsafe fn w16(addr: u32, val: u16) {
    unsafe { core::ptr::write_volatile(addr as *mut u16, val) }
}

pub fn reset_baseband() {
    unsafe {
        w8(REG_RST1, 0x01);
        w8(REG_RST1, 0x00);
    }
}

pub fn set_trx_off() {
    unsafe {
        w8(REG_RF_LL_CTRL_3, 0x29);
        w8(REG_RF_RX_MODE, RF_TRX_MODE);
        w8(REG_RF_LL_CTRL_0, RF_TRX_OFF);
    }
}

pub fn set_auto_mode() {
    unsafe { w8(REG_RF_MODE_CTRL, 0x80) };
}

pub fn reset_sn() {
    unsafe {
        w8(REG_RF_SN, 0x3F);
        w8(REG_RF_SN, 0x00);
    }
}

pub fn clear_irq_status() {
    unsafe { w16(REG_RF_IRQ_STATUS, 0xFFFF) };
}

pub fn clear_irq_mask() {
    unsafe { w16(REG_RF_IRQ_MASK, 0) };
}

pub fn set_tx_pipe(pipe: u8) {
    unsafe { w8(REG_RF_LL_CTRL_2, 0x10 | (pipe & 0x07)) };
}

pub fn set_tx_settle(us: u16) {
    unsafe { w16(REG_RF_TX_SETTLE, us.saturating_sub(1) & 0x0FFF) };
}

/// Write PHY registers transcribed from
/// `examples/telink-tlsr8258-sensor::rf_phy_init_zigbee`, itself extracted
/// from the Telink SDK's `libdrivers_8258.a rf_drv_init()`.
///
/// Note on table sizes: this transcribes the common RF-init table (6
/// register writes: `0x8012D2/D3`, `0x80127B`, `0x801276/77`, `0x800430`)
/// followed by the Zigbee-250K PHY table (28 register writes). Some prior
/// bring-up notes describe this as "5+28"; the sensor lab's own source
/// comment says "(6 entries)" for the first table, and that is what is
/// transcribed here verbatim. If a hardware discrepancy is ever found
/// between "5" and "6" for the first table, treat the sensor lab's
/// `rf_phy_init_zigbee` (the hardware-proven source) as authoritative, not
/// this comment.
fn rf_phy_init_zigbee() {
    unsafe {
        w8(REG_CLK_EN2, 0xFF);
        w8(REG_RST0, 0x00);
        w8(REG_RST1, 0x00);
        w8(REG_RST2, 0x00);
        w8(REG_CLK_EN0, 0xFF);
        w8(REG_CLK_EN1, 0xFF);

        // tbl_rf_init — common RF PHY init (6 entries).
        w8(0x8012D2, 0x9B);
        w8(0x8012D3, 0x19);
        w8(0x80127B, 0x0E);
        w8(0x801276, 0x50);
        w8(0x801277, 0x73);
        w8(0x800430, 0x3E);

        // tbl_rf_zigbee_250k — Zigbee 250K mode PHY (28 entries).
        w8(0x801220, 0x04);
        w8(0x801221, 0x2B);
        w8(0x801222, 0x43);
        w8(0x801223, 0x86);
        w8(0x80122A, 0x90);
        w8(0x801254, 0x0E);
        w8(0x801255, 0x09);
        w8(0x801256, 0x0C);
        w8(0x801257, 0x08);
        w8(0x801258, 0x09);
        w8(0x801259, 0x0F);
        w8(0x800400, 0x13);
        w8(0x800420, 0x18);
        w8(0x800402, 0x46);
        w8(0x800404, 0xC0);
        w8(0x800405, 0x04);
        w8(0x800421, 0x23);
        w8(0x800422, 0x04);
        w8(0x800408, 0xA7);
        w8(0x800409, 0x00);
        w8(0x80040A, 0x00);
        w8(0x80040B, 0x00);
        w8(0x800460, 0x36);
        w8(0x800461, 0x46);
        w8(0x800462, 0x51);
        w8(0x800463, 0x61);
        w8(0x800464, 0x6D);
        w8(0x800465, 0x78);

        // Enable DMA channels 2+3 (RF RX/TX) — from rf_drv_init.
        let dma_en = r8(REG_DMA_CHN_EN);
        w8(REG_DMA_CHN_EN, dma_en | 0x0C);
    }
}

/// Set the 802.15.4 channel (11..=26). Writes the physical channel, PLL fine
/// divider, and modem channel/power-band registers, matching the SDK
/// `rf_set_channel` disassembly. Silently ignores out-of-range channels
/// (defensive: `mac_test` only ever calls this with 11/18/26).
#[inline(never)]
#[unsafe(link_section = ".ram_code")]
pub fn set_channel(channel: u8) {
    if !(11..=26).contains(&channel) {
        return;
    }
    let physical = (channel as u16 - 10) * 5;
    let freq_mhz: u16 = 2400 + physical;
    let band: u8 = if freq_mhz > 2464 {
        0x0C
    } else if freq_mhz > 2434 {
        0x10
    } else {
        0x14
    };

    set_trx_off();
    unsafe {
        w8(REG_RF_CHANNEL, physical as u8);
        w16(REG_PLL_FINE_TUNE, freq_mhz);
        let modem_val: u16 = (freq_mhz << 2) | 1;
        w8(REG_MODEM_CHN_L, modem_val as u8);
        let existing_h = r8(REG_MODEM_CHN_H);
        w8(
            REG_MODEM_CHN_H,
            (existing_h & 0xC0) | ((modem_val >> 8) as u8 & 0x3F),
        );
        let existing_band = r8(REG_MODEM_BAND);
        w8(REG_MODEM_BAND, (existing_band & 0xC3) | (band & 0x3C));
    }

    // Match the hardware-proven sensor path: do not arm TX while the
    // channel PLL is still settling after the frequency-register writes.
    for _ in 0..2_000u32 {
        unsafe { core::arch::asm!("nop") };
    }
}

pub fn set_rx_mode() {
    unsafe {
        w8(REG_RF_RX_MODE, RF_TRX_MODE | 0x01);
        w8(REG_RF_LL_CTRL_0, RF_TRX_OFF | (1 << 5));
    }
}

pub fn disable_rx_mode() {
    unsafe {
        w8(REG_RF_RX_MODE, RF_TRX_MODE);
    }
}

pub fn rssi_dbm() -> i8 {
    unsafe { r8(REG_RF_RSSI) as i8 - 110 }
}

pub fn set_rx_buffer(addr: *mut u8) {
    let a = addr as usize;
    unsafe {
        w16(REG_DMA2_ADDR, a as u16);
        w8(REG_DMA2_ADDR_HI, ((a >> 16) as u8) & 0x0F);
    }
}

pub fn set_rx_dma_config(buf_size: u16) {
    unsafe {
        w8(REG_DMA2_SIZE, ((buf_size >> 4) & 0xFF) as u8);
        w8(REG_DMA2_MODE, 0x01);
    }
}

pub fn enable_dma_rx() {
    unsafe {
        let v = r8(REG_DMA_CHN_EN);
        w8(REG_DMA_CHN_EN, v | 0x04);
    }
}

pub fn disable_dma_rx() {
    unsafe {
        let v = r8(REG_DMA_CHN_EN);
        w8(REG_DMA_CHN_EN, v & !0x04);
    }
}

pub fn enable_dma_tx() {
    unsafe {
        let v = r8(REG_DMA_CHN_EN);
        w8(REG_DMA_CHN_EN, v | 0x08);
    }
}

pub fn rx_done() -> bool {
    unsafe { r16(REG_RF_IRQ_STATUS) & 0x01 != 0 }
}

pub fn rx_done_clear() {
    unsafe {
        w8(REG_RF_IRQ_STATUS, 0x01);
        w8(REG_DMA_IRQ_STATUS, 0x04);
        w8(REG_IRQ_SRC, 1 << 4); // NOTE: byte write into a u32 register is
        // intentional here — it mirrors the proven
        // sensor-lab sequence, which only needs the
        // low byte (bit 4) cleared.
    }
}

pub fn tx_done_clear() {
    unsafe {
        w8(REG_RF_IRQ_STATUS, 0x02);
        w8(REG_DMA_IRQ_STATUS, 0x08);
        w8(REG_IRQ_SRC, 1 << 4);
    }
}

pub fn tx_done() -> bool {
    unsafe { r16(REG_RF_IRQ_STATUS) & 0x02 != 0 }
}

pub fn tx_pkt(addr: *const u8) {
    let a = addr as usize;
    unsafe {
        w8(REG_DMA3_ADDR_HI, 0x04);
        w16(REG_DMA3_ADDR, (a & 0xFFFF) as u16);
        let v = r8(REG_RF_TX_TRIGGER);
        w8(REG_RF_TX_TRIGGER, v | 0x08);
        let dma = r8(REG_DMA_TX_READY);
        w8(REG_DMA_TX_READY, dma | 0x08);
    }
}

pub fn set_tx_dma_config(buf_size: u16) {
    unsafe {
        w8(REG_DMA3_SIZE, ((buf_size >> 4) & 0xFF) as u8);
        w8(REG_DMA3_MODE, 0x00);
    }
}

pub fn set_tx_mode() {
    unsafe { w8(REG_RF_LL_CTRL_0, RF_TRX_OFF | (1 << 4)) };
}

pub fn set_irq_mask_rx_only() {
    unsafe {
        let v = r16(REG_RF_IRQ_MASK);
        w16(REG_RF_IRQ_MASK, v | 0x01);
    }
}

/// Full radio init: reset, PHY config, DMA, channel 11. Mirrors
/// `examples/telink-tlsr8258-sensor::radio::init`, minus the CPU-side
/// `REG_IRQ_EN`/`REG_IRQ_MASK` enable at the end — this firmware runs fully
/// polled, so the CPU IRQ line for RF/DMA is never unmasked (see
/// `platform::vectors` and `mac_test`).
#[inline(never)]
#[unsafe(link_section = ".ram_code")]
pub fn init(rx_buf: *mut u8) {
    set_auto_mode();
    set_trx_off();
    reset_sn();
    clear_irq_status();
    clear_irq_mask();
    set_tx_pipe(0);
    set_tx_settle(DEFAULT_TX_SETTLE_US);
    rf_phy_init_zigbee();
    set_channel(11);
    set_rx_buffer(rx_buf);
    set_rx_dma_config(144);
    unsafe {
        core::ptr::write_volatile(rx_buf, 0);
        core::ptr::write_volatile(rx_buf.add(4), 0);
    }
    enable_dma_rx();
    enable_dma_tx();
    unsafe {
        w8(REG_DMA_IRQ_STATUS, 0x0C);
        w8(REG_IRQ_SRC, 1 << 4);
        w8(REG_DMA_CHN_IRQ_MSK, 0x04);
        let ctrl1 = r8(REG_RF_LL_CTRL_1);
        w8(REG_RF_LL_CTRL_1, ctrl1 | (1 << 5));
    }
    set_irq_mask_rx_only();
}
