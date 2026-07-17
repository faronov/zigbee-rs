//! # Telink TLSR8258 raw-radio bring-up — pure Rust, zero Zigbee-stack deps
//!
//! Standalone `no_std` firmware that proves out TLSR8258 802.15.4 PHY/DMA
//! bring-up. Its default modes stay below the Zigbee stack; the optional
//! `mac-driver` mode validates the reusable `zigbee-mac` backend.
//!
//! ## What it does
//!
//! Forever: cycle channels 11 -> 18 -> 26; on each, transmit an exact IEEE
//! 802.15.4 Beacon Request, switch to RX, accept only length-valid and
//! CRC-valid Beacon frames, and record everything (per-channel beacon
//! counts, TX success/timeout, invalid-length/CRC counts, last frame
//! metadata) in a fixed-address, checksum-protected SRAM diagnostic record.
//! See `diag` for the record layout and `mac_test` for the state machine.
//!
//! ## Module map
//!
//! - [`platform`]: boot vectors/startup, MMIO, clocks/timer, GPIO LEDs,
//!   linker-symbol-derived layout checks.
//! - [`radio`]: PHY init (from the proven official register tables), channel
//!   set, DMA TX/RX, status/CRC/RSSI helpers, bounded waits.
//! - [`mac_test`]: the channel-cycle/Beacon-Request/RX-classify state
//!   machine.
//! - [`diag`]: the fixed-address SRAM diagnostic record.
//!
//! ## Host tests
//!
//! `platform`/`radio`'s hardware-facing code is gated on
//! `#[cfg(target_arch = "tc32")]` (true only for the real `tc32-unknown-none-elf`
//! target); the pure logic in `radio::frame` and `diag`'s
//! checksum/canary/record logic is compiled and unit-tested on the host via
//! `#![cfg_attr(not(test), no_std)]` / `#![cfg_attr(not(test), no_main)]` —
//! see the README for the exact `cargo test` invocation.

#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

#[cfg(all(target_arch = "tc32", feature = "association"))]
mod association;
pub mod diag;
#[cfg(all(
    target_arch = "tc32",
    feature = "mac-driver",
    not(feature = "runtime-join")
))]
mod mac_driver_test;
#[cfg(all(target_arch = "tc32", not(feature = "mac-driver")))]
pub mod mac_test;
pub mod platform;
pub mod radio;
#[cfg(all(target_arch = "tc32", feature = "runtime-join"))]
mod runtime_join_test;

#[cfg(all(feature = "mac-driver", feature = "association"))]
compile_error!("`mac-driver` and `association*` modes are mutually exclusive");

/// Panic handler. Only compiled for the real target and outside `cfg(test)`
/// (the host test harness supplies its own via `std`). Records the return
/// address into the reserved tail of the diagnostic region so a panic is
/// visible via `scripts/tlsr8258.sh dump 0x0084FFF8 2` even without RTT/UART, then
/// spins forever — there is nowhere safe to jump back to.
#[cfg(all(target_arch = "tc32", not(test)))]
#[panic_handler]
fn panic_handler(_info: &core::panic::PanicInfo) -> ! {
    let lr: u32;
    unsafe {
        core::arch::asm!("mov {0}, lr", out(reg) lr);
        // These words are inside the linker-reserved `.diag` tail, but
        // outside DiagRecord itself and therefore outside its checksum.
        core::ptr::write_volatile(diag::PANIC_MAGIC_ADDR as *mut u32, 0xDEAD_BEEF);
        core::ptr::write_volatile(diag::PANIC_LR_ADDR as *mut u32, lr);
    }
    loop {
        unsafe { core::arch::asm!("nop") };
    }
}

/// Rust entry point, called from `platform::vectors::_start` after the
/// assembly `.data`-copy/`.bss`-zero loops complete.
#[cfg(target_arch = "tc32")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn _rust_entry() -> ! {
    platform::clocks::init();
    #[cfg(feature = "runtime-join")]
    {
        runtime_join_test::run()
    }
    #[cfg(all(feature = "mac-driver", not(feature = "runtime-join")))]
    {
        mac_driver_test::run()
    }
    #[cfg(all(not(feature = "mac-driver"), feature = "association"))]
    {
        association::run()
    }
    #[cfg(all(not(feature = "mac-driver"), not(feature = "association")))]
    {
        mac_test::run()
    }
}
