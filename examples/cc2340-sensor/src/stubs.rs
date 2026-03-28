//! CI/build stubs for TI CC2340 RCL and MAC platform FFI functions.
//!
//! Provides no-op implementations of every symbol the CC2340 MAC driver
//! imports via `unsafe extern "C"`, so that `cargo build --release --features stubs`
//! can produce a valid ELF without linking the real TI SDK libraries.
//!
//! These stubs are gated behind the `stubs` cargo feature and must never
//! be included in a production firmware build.

#![allow(non_snake_case)]

use core::ptr;

// ── RCL core API stubs ──────────────────────────────────────────

#[unsafe(no_mangle)]
pub extern "C" fn RCL_init() {}

#[unsafe(no_mangle)]
pub extern "C" fn RCL_open(_client: *mut u8, _config: *const u8) -> *mut u8 {
    ptr::null_mut()
}

#[unsafe(no_mangle)]
pub extern "C" fn RCL_close(_handle: *mut u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn RCL_Command_submit(_handle: *mut u8, _cmd: *mut u8) -> u16 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn RCL_Command_pend(_cmd: *mut u8) -> u16 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn RCL_Command_stop(_cmd: *mut u8, _stop_type: u32) -> u16 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn RCL_readRssi(_handle: *mut u8) -> i8 {
    0
}

// ── MAC platform shim stubs ─────────────────────────────────────

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_radio_init(_enable: u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_set_channel(_page: u8, _channel_num: u8) -> i32 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_24_set_tx_power(_tx_power_dbm: u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_trans_set_rx_on_off(_enable: u32) {}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_set_ieee_addr(_addr: *const u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_send_packet(_mhr_len: u8, _buf: u8, _wait_type: u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_perform_cca(_rssi: *mut i8) -> i32 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_set_promiscuous_mode(_mode: u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_src_match_add_short_addr(_index: u8, _short_addr: u16) -> u8 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_src_match_delete_short_addr(_index: u8) -> u8 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_src_match_tbl_drop() {}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_trans_rec_pkt(_buf: *mut u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_get_radio_data_status(_status_type: u8) -> u8 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_clear_radio_data_status(_status_type: u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_abort_tx() {}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_enable_rx() {}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_get_sync_rssi() -> i8 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn mac_ti23xx_set_cca_rssi_threshold(_rssi: i8) {}
