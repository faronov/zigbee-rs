//! CI link stubs for Telink tl_zigbee_sdk FFI functions.
//!
//! Provides no-op implementations of all extern functions imported by the
//! `zigbee-mac` Telink radio driver so that `cargo build --release --features stubs`
//! can produce a valid ELF without the real Telink SDK libraries.

#![allow(non_snake_case)]

#[unsafe(no_mangle)]
pub extern "C" fn rf_init() {}

#[unsafe(no_mangle)]
pub extern "C" fn mac_trxInit() {}

#[unsafe(no_mangle)]
pub extern "C" fn rf_setChannel(_chn: u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn rf_setTxPower(_power: u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn rf_setRxBuf(_buf: *mut u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn rf_setTrxState(_state: u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn rf802154_tx_ready(_buf: *mut u8, _len: u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn rf802154_tx() {}

#[unsafe(no_mangle)]
pub extern "C" fn rf_performCCA() -> u8 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn rf_startEDScan() {}

#[unsafe(no_mangle)]
pub extern "C" fn rf_stopEDScan() -> u8 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn rf_getLqi(_rssi: i8) -> u8 {
    0
}

// Critical-section stubs for thumbv6m CI builds (cortex-m's implementation
// is stripped by LTO since nothing in the same codegen unit references it).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn _critical_section_1_0_acquire() -> bool {
    false
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn _critical_section_1_0_release(_restore_state: bool) {}
