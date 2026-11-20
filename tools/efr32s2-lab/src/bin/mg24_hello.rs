//! EFR32MG24 gate 1 (RF-BM-MG24B1 dev board, feature `rfbm`): HFXO,
//! SYSRTC, USART0 host link, DEVINFO, reset cause and the SE TRNG.
//!
//! Flash at 0x0800_0000 over SWD on a module **without** a bootloader
//! (back the flash up first). Host side: the ESP32-S3 (or any 3.3 V USB-UART)
//! on PA05 (MG24 TX) / PA06 (MG24 RX), 115 200 8N1. Expected output:
//!
//! ```text
//! mg24-hello: reset <cause> (0x...)
//! DEVINFO part 0x... flash <KiB> KiB sram <KiB> KiB EUI-64 ..
//! HFXO 39000000 Hz ok, CTUNE <n> (token|board default)
//! SYSRTC 32768 ticks = <n> TIMER0 cycles (LFRCO vs HFXO <ppm> ppm)
//! TRNG <16 bytes> | TRNG error <e>
//! tick <n> rx <bytes> overruns <n> line-errors <n>
//! ```
//!
//! Every byte received is echoed back, so a host can check the link at
//! 115 200 without the ZRHP framing.

#![no_std]
#![no_main]

use core::fmt::Write;

use efr32s2_hal::{delay, devinfo, reset, rtcc};
use rf_bm_mg24b1::resources::BoardResources;

#[allow(unused_imports)]
use efr32s2_hal::vectors::__INTERRUPTS;

const BAUD: u32 = 115_200;

struct Out<'a>(&'a mut efr32s2_hal::usart::host::HostUart);

impl Write for Out<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let mut bytes = s.as_bytes();
        while !bytes.is_empty() {
            let n = self.0.try_write(bytes);
            bytes = &bytes[n..];
        }
        Ok(())
    }
}

/// embassy-executor's pender (pulled in by the HAL time driver); the lab
/// has no executor, so wakes are no-ops.
#[unsafe(no_mangle)]
fn __pender(_context: *mut ()) {}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let (cause, raw_cause) = reset::take_reset_cause();
    let BoardResources {
        clocks,
        host_uart,
        rtcc: rtcc_token,
        ..
    } = BoardResources::take().unwrap();
    // A failed HFXO start leaves the core on FSRCO: halt where a debugger
    // can read the error (no UART baud rate is valid without the HFXO).
    let hfxo = rf_bm_mg24b1::init_clocks(clocks);
    if hfxo.is_err() {
        loop {
            cortex_m::asm::bkpt();
        }
    }
    delay::enable_cycle_counter();
    let mut uart = host_uart.open(BAUD).unwrap();
    let mut out = Out(&mut uart);
    let _ = writeln!(out, "\r\nmg24-hello: reset {cause:?} ({raw_cause:#x})\r");
    // SAFETY: DEVINFO is a read-only information page.
    let (part, msize) = unsafe {
        (
            core::ptr::read_volatile(0x0FE0_8004 as *const u32),
            core::ptr::read_volatile(0x0FE0_800C as *const u32),
        )
    };
    let _ = writeln!(
        out,
        "DEVINFO part {part:#010x} flash {} KiB sram {} KiB EUI-64 {:02X?}\r",
        msize & 0xFFFF,
        (msize >> 16) & 0x7FF,
        devinfo::eui64()
    );
    let token = devinfo::mfg_ctune();
    let _ = writeln!(
        out,
        "HFXO {} Hz ok, CTUNE {} ({})\r",
        rf_bm_mg24b1::HCLK_HZ,
        rf_bm_mg24b1::hfxo_ctune(),
        if token.is_some() {
            "token"
        } else {
            "board default, unmeasured"
        }
    );
    match rtcc_token.start() {
        Ok(_rtc) => {
            // One second of LFRCO ticks measured in HFXO cycles.
            let start = rtcc::now();
            while rtcc::now() == start {}
            let t0 = delay::cycles();
            let from = rtcc::now();
            while rtcc::now().wrapping_sub(from) < rtcc::RTCC_HZ {}
            let cycles = delay::cycles().wrapping_sub(t0);
            let ppm = (cycles as i64 - delay::HCLK_HZ as i64) * 1_000_000 / delay::HCLK_HZ as i64;
            let _ = writeln!(
                out,
                "SYSRTC {} ticks = {cycles} TIMER0 cycles (LFRCO vs HFXO {ppm} ppm)\r",
                rtcc::RTCC_HZ
            );
        }
        Err(error) => {
            let _ = writeln!(out, "SYSRTC start failed: {error:?}\r");
        }
    }
    let mut random = [0u8; 16];
    match efr32s2_hal::se::fill_random(&mut random) {
        Ok(()) => {
            let _ = writeln!(out, "TRNG {random:02X?}\r");
        }
        Err(error) => {
            let _ = writeln!(out, "TRNG error {error:?}\r");
        }
    }
    let mut received = 0u32;
    let mut tick = 0u32;
    let mut last = delay::cycles();
    let mut buf = [0u8; 64];
    loop {
        let n = uart.read(&mut buf);
        if n != 0 {
            received = received.wrapping_add(n as u32);
            let mut echo = &buf[..n];
            while !echo.is_empty() {
                let sent = uart.try_write(echo);
                echo = &echo[sent..];
            }
        }
        if delay::cycles().wrapping_sub(last) >= delay::HCLK_HZ {
            last = last.wrapping_add(delay::HCLK_HZ);
            let (overruns, errors) = (uart.rx_overruns(), uart.rx_line_errors());
            let mut out = Out(&mut uart);
            let _ = writeln!(
                out,
                "tick {tick} rx {received} overruns {overruns} line-errors {errors}\r"
            );
            tick += 1;
        }
    }
}
