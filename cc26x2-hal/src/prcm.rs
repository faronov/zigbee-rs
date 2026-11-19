//! Power domains and clock gates (SDK `driverlib/prcm.[ch]`, `inc/hw_prcm.h`).

use crate::mmio::{self, PRCM_BASE};

const CLKLOADCTL: u32 = PRCM_BASE + 0x28;
const RFCCLKG: u32 = PRCM_BASE + 0x2C;
const SECDMACLKGR: u32 = PRCM_BASE + 0x3C;
const GPIOCLKGR: u32 = PRCM_BASE + 0x48;
const UARTCLKGR: u32 = PRCM_BASE + 0x6C;
const PDCTL0RFC: u32 = PRCM_BASE + 0x130;
const PDCTL0SERIAL: u32 = PRCM_BASE + 0x134;
const PDCTL0PERIPH: u32 = PRCM_BASE + 0x138;
const PDSTAT0: u32 = PRCM_BASE + 0x140;
const PDCTL1RFC: u32 = PRCM_BASE + 0x188;
const PDSTAT1: u32 = PRCM_BASE + 0x194;
const RFCMODESEL: u32 = PRCM_BASE + 0x1D0;

const LOAD: u32 = 1 << 0;
const LOAD_DONE: u32 = 1 << 1;

/// Switchable power domains.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Domain {
    Rfc,
    Serial,
    Periph,
}

/// Peripheral run-mode clock gates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Gate {
    Gpio,
    Uart0,
    Trng,
    Crypto,
}

/// Bounded spin on a hardware condition; `false` on timeout.
pub(crate) fn wait(mut done: impl FnMut() -> bool) -> bool {
    for _ in 0..2_000_000u32 {
        if done() {
            return true;
        }
        core::hint::spin_loop();
    }
    done()
}

pub fn power_on(domain: Domain) -> bool {
    match domain {
        Domain::Rfc => mmio::write(PDCTL0RFC, 1),
        Domain::Serial => mmio::write(PDCTL0SERIAL, 1),
        Domain::Periph => mmio::write(PDCTL0PERIPH, 1),
    }
    wait(|| is_on(domain))
}

pub fn power_off(domain: Domain) -> bool {
    match domain {
        Domain::Rfc => {
            mmio::write(PDCTL0RFC, 0);
            mmio::write(PDCTL1RFC, 0);
        }
        Domain::Serial => mmio::write(PDCTL0SERIAL, 0),
        Domain::Periph => mmio::write(PDCTL0PERIPH, 0),
    }
    wait(|| !is_on(domain))
}

pub fn is_on(domain: Domain) -> bool {
    let s0 = mmio::read(PDSTAT0);
    match domain {
        Domain::Rfc => s0 & 1 != 0 || mmio::read(PDSTAT1) & (1 << 2) != 0,
        Domain::Serial => s0 & (1 << 1) != 0,
        Domain::Periph => s0 & (1 << 2) != 0,
    }
}

/// Enable a peripheral clock in run and sleep mode (takes effect after
/// [`load`]). Sleep mode is what `WFI`/`WFE` enter when the executor idles:
/// without the sleep gate a UART stops shifting out mid-frame.
pub fn enable(gate: Gate) {
    let (run, bit) = match gate {
        Gate::Gpio => (GPIOCLKGR, 1),
        Gate::Uart0 => (UARTCLKGR, 1),
        Gate::Trng => (SECDMACLKGR, 1 << 1),
        Gate::Crypto => (SECDMACLKGR, 1 << 0),
    };
    mmio::modify(run, 0, bit);
    // `xCLKGS` follows `xCLKGR` at +4 for every gate.
    mmio::modify(run + 4, 0, bit);
}

/// RF core clock domain on/off (takes effect after [`load`]).
pub fn rfc_clock(on: bool) {
    mmio::write(RFCCLKG, on as u32);
}

/// Select the RF core mode (`RF_MODE_AUTO` = 0 on this family).
pub fn rfc_mode_auto() {
    mmio::write(RFCMODESEL, 0);
}

/// Apply the clock gate settings; `false` if the load never completed.
pub fn load() -> bool {
    mmio::write(CLKLOADCTL, LOAD);
    wait(|| mmio::read(CLKLOADCTL) & LOAD_DONE != 0)
}
