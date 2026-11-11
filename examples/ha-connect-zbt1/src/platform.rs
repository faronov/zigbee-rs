//! Board start-up shared by the role bins (`router.rs`, `sed.rs`).
//!
//! The original end-device image (`main.rs`) keeps its own copy of this
//! sequence so its bytes stay identical to the hardware-validated build.

#[cfg(any(
    all(feature = "ed", feature = "router"),
    all(feature = "ed", feature = "sed"),
    all(feature = "router", feature = "sed"),
))]
compile_error!(
    "enable exactly one role feature (ed, router or sed); use --no-default-features for router/sed"
);

#[cfg(all(feature = "zrs-boot", not(feature = "router")))]
compile_error!("feature `zrs-boot` is the loader-linked router: combine it with `router`");

use ha_connect_zbt1::resources::BoardResources;
use ha_connect_zbt1_product as product;

/// Resources a role composition needs after clocks and time are running.
pub struct Started {
    pub radio: efr32s2_hal::radio::RadioToken,
    pub flash: efr32s2_hal::flash::Efr32s2Flash,
}

/// RTT log, HFXO, RTCC time driver and the 1 MiB flash-size guard.
pub fn start(model: &str) -> Started {
    crate::rtt_log::init();
    let BoardResources {
        clocks,
        host_uart: _,
        radio,
        rtcc,
        flash,
    } = BoardResources::take().unwrap_or_else(|| halt());

    if let Err(error) = ha_connect_zbt1::init_clocks(clocks) {
        log::error!("[ZBT1] HFXO start failed: {error:?}");
        halt()
    }
    let rtcc = rtcc.start().unwrap_or_else(|error| {
        log::error!("[ZBT1] RTCC start failed: {error:?}");
        halt()
    });
    efr32s2_hal::time_driver::init(rtcc);

    log::info!(
        "[ZBT1] {} on {} ({}), CTUNE {}, TX {} dBm",
        model,
        ha_connect_zbt1::BOARD_NAME,
        ha_connect_zbt1::MCU_PART,
        ha_connect_zbt1::hfxo_ctune(),
        product::TX_POWER_DBM
    );
    // The product layout puts persistence at the top of 1 MiB; refuse to
    // run on a part whose DEVINFO reports less.
    let capacity = efr32s2_hal::flash::device_flash_capacity();
    if capacity != ha_connect_zbt1::INTERNAL_FLASH_CAPACITY {
        log::error!("[ZBT1] DEVINFO flash size {capacity} B, product needs 1 MiB");
        halt()
    }
    Started { radio, flash }
}

/// Reject an all-zero/all-ones DEVINFO EUI-64 before it reaches the stack.
pub fn check_ieee(ieee: [u8; 8]) {
    if ieee == [0; 8] || ieee == [0xFF; 8] {
        log::error!("[ZBT1] no valid DEVINFO EUI-64");
        halt()
    }
    log::info!("[ZBT1] EUI-64 {:02X?}", ieee);
}

/// Open the shared security journal and log its disposition.
pub fn open_security_store(
    flash: efr32s2_hal::flash::Efr32s2Flash,
) -> product::storage::SecurityStore {
    let (store, migration) = product::storage::security_store(flash).unwrap_or_else(|error| {
        log::error!("[ZBT1] security journal open failed: {error:?}");
        halt()
    });
    match migration {
        product::storage::MigrationDisposition::ExistingJournal => {
            log::info!("[ZBT1] valid security journal preserved")
        }
        product::storage::MigrationDisposition::FreshReset => {
            log::warn!("[ZBT1] foreign/blank persistence erased; fresh join required")
        }
    }
    store
}

/// Log the outcome of the identity and role guards; halt on a store error.
pub fn guard_outcome<E: core::fmt::Debug>(what: &str, result: Result<bool, E>) {
    match result {
        Ok(true) => log::warn!("[ZBT1] cleared network state: {what} changed"),
        Ok(false) => {}
        Err(error) => {
            log::error!("[ZBT1] {what} guard failed: {error:?}");
            halt()
        }
    }
}

/// Stop with interrupts masked; the RTT log keeps the reason.
pub fn halt() -> ! {
    cortex_m::interrupt::disable();
    loop {
        cortex_m::asm::wfi();
    }
}
