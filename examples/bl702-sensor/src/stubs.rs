//! CI/test stub implementations of the `lmac154` FFI functions.
//!
//! When the `stubs` feature is enabled these `#[unsafe(no_mangle)]` symbols satisfy
//! the linker without requiring the real `liblmac154.a` static library,
//! allowing `cargo build --release --features stubs` to produce a valid ELF
//! for CI checks and static analysis.

#![allow(non_snake_case)]

use core::ptr;

// ── Initialization ──────────────────────────────────────────────

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_init() {}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_getInterruptHandler() -> Option<unsafe extern "C" fn()> {
    None
}

// ── Configuration ───────────────────────────────────────────────

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_setChannel(_ch_ind: u32) {}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_getChannel() -> u32 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_setPanId(_pid: u16) {}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_getPanId() -> u16 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_setShortAddr(_sadr: u16) {}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_getShortAddr() -> u16 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_setLongAddr(_ladr: *const u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_getLongAddr(_ladr: *mut u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_setTxPower(_power: u32) {}

// ── TX ──────────────────────────────────────────────────────────

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_triggerTx(_data_ptr: *const u8, _length: u8, _csma: u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_setTxRetry(_num: u32) {}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_resetTx() {}

// ── RX ──────────────────────────────────────────────────────────

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_enableRx() {}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_disableRx() {}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_getRxLength() -> u8 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_readRxData(_buf: *mut u8, _offset: u8, _len: u8) {}

// ── Promiscuous mode ────────────────────────────────────────────

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_enableRxPromiscuousMode(_enhanced_mode: u8, _ignore_mpdu: u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_disableRxPromiscuousMode() {}

// ── Frame filtering ─────────────────────────────────────────────

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_enableFrameTypeFiltering(_frame_types: u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_disableFrameTypeFiltering() {}

// ── CCA / Energy Detection ──────────────────────────────────────

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_runCCA(rssi: *mut i32) -> u8 {
    if !rssi.is_null() {
        unsafe { ptr::write(rssi, 0) };
    }
    0
}

// ── Status ──────────────────────────────────────────────────────

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_getRSSI() -> i32 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_getLQI() -> u8 {
    0
}

// ── Auto-ACK ────────────────────────────────────────────────────

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_enableHwAutoTxAck() {}

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_disableHwAutoTxAck() {}

// ── Coexistence ─────────────────────────────────────────────────

#[unsafe(no_mangle)]
pub extern "C" fn lmac154_enableCoex() {}
