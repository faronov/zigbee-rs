//! BL702 radio and IEEE 802.15.4 MMIO definitions.

pub(super) const GLB_BASE: u32 = 0x4000_0000;
pub(super) const PHY_BASE: u32 = 0x4000_1800;
pub(super) const RF_FSM: u32 = 0x4000_1510;
pub(super) const M154_BASE: u32 = 0x4c00_0000;

pub(super) const GLB_CLK_CFG1: u32 = GLB_BASE + 0x004;
pub(super) const GLB_MIX_RESET: u32 = GLB_BASE + 0x010;
pub(super) const GLB_CGEN_CFG2: u32 = GLB_BASE + 0x028;
pub(super) const GLB_COEX_CTRL: u32 = GLB_BASE + 0x0b0;

pub(super) const PHY_CCA_CTRL: u32 = PHY_BASE + 0x034;
pub(super) const PHY_CCA_ENABLE: u32 = PHY_BASE + 0x02c;
pub(super) const PHY_RSSI: u32 = PHY_BASE + 0x060;

pub(super) const M154_CONTROL: u32 = M154_BASE;
pub(super) const M154_RESET: u32 = M154_BASE + 0x008;
pub(super) const M154_IRQ_ENABLE: u32 = M154_BASE + 0x020;
pub(super) const M154_IRQ_CLEAR: u32 = M154_BASE + 0x024;
pub(super) const M154_IRQ_STATUS: u32 = M154_BASE + 0x030;
pub(super) const M154_TIMER: u32 = M154_BASE + 0x040;
pub(super) const M154_RECOVERY_TIME: u32 = M154_BASE + 0x068;
pub(super) const M154_CSMA_CONTROL: u32 = M154_BASE + 0x090;
pub(super) const M154_BUSY_STATUS: u32 = M154_BASE + 0x0b0;
pub(super) const M154_TX_CONTROL: u32 = M154_BASE + 0x0b4;
pub(super) const M154_STATE: u32 = M154_BASE + 0x0bc;
pub(super) const M154_RX_STATUS: u32 = M154_BASE + 0x0c4;
pub(super) const M154_FILTER_CONTROL: u32 = M154_BASE + 0x0c8;
pub(super) const M154_RX_CONTROL: u32 = M154_BASE + 0x0e4;
pub(super) const M154_COEX_CONTROL: u32 = M154_BASE + 0x180;
pub(super) const M154_TX_BUFFER: u32 = M154_BASE + 0x200;
pub(super) const M154_TX_LENGTH: u32 = M154_BASE + 0x2fc;
pub(super) const M154_RX_BUFFER: u32 = M154_BASE + 0x400;
pub(super) const M154_RX_METADATA: u32 = M154_BASE + 0x4fc;

pub(super) const IRQ_RX_CRC: u32 = 0x0008_0000;
pub(super) const IRQ_RX_DONE: u32 = 0x0100_0000;
pub(super) const IRQ_RX_AUX: u32 = 0x00b0_0000;
pub(super) const IRQ_RX_ACK: u32 = 0x0040_0000;
pub(super) const IRQ_RX_MASK: u32 = IRQ_RX_CRC | IRQ_RX_DONE | IRQ_RX_AUX | IRQ_RX_ACK;

pub(super) const IRQ_TX_FINISHED: u32 = 0x1000_0000;
pub(super) const IRQ_TX_CSMA_FAILED: u32 = 0x0c00_0000;
pub(super) const IRQ_TX_ABORTED: u32 = 0x8000_0000;
pub(super) const IRQ_TX_HW_ERROR: u32 = 0x0000_0002;
pub(super) const IRQ_TX_MASK: u32 =
    IRQ_TX_FINISHED | IRQ_TX_CSMA_FAILED | IRQ_TX_ABORTED | IRQ_TX_HW_ERROR;

#[inline(always)]
pub(super) fn read32(address: u32) -> u32 {
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

#[inline(always)]
pub(super) fn write32(address: u32, value: u32) {
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

#[inline(always)]
pub(super) fn update32(address: u32, mask: u32, value: u32) {
    write32(address, (read32(address) & !mask) | (value & mask));
}

#[inline(always)]
pub(super) fn set32(address: u32, bits: u32) {
    write32(address, read32(address) | bits);
}

#[inline(always)]
pub(super) fn clear32(address: u32, bits: u32) {
    write32(address, read32(address) & !bits);
}
