//! PB-03F-Kit hardware lab: proves ROM boot, the 48 MHz clock switch, the
//! CH340 UART console, the five LEDs, the user button and internal-flash
//! erase/program/read on a spare sector, independently of the Zigbee stack.
//!
//! Results are printed on the console and mirrored into `LAB_RESULT` so they
//! can also be read over SWD.

#![no_std]
#![no_main]

use core::fmt::Write;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use panic_halt as _;
use pb03f_kit::{Resources, time};
use phy6222_hal::clock;

/// Spare sector below the product security journal (0x7e000..0x80000).
const FLASH_TEST_OFFSET: u32 = 0x0007_d000;

#[repr(C)]
pub struct LabResult {
    magic: u32,
    clksel_at_reset: u32,
    clkhf_ctl1_at_reset: u32,
    sw_clk_at_reset: u32,
    sw_clk1_at_reset: u32,
    apb_clk_at_reset: u32,
    hclk_hz: u32,
    flash_status: u32,
    loops: u32,
}

#[unsafe(no_mangle)]
pub static mut LAB_RESULT: LabResult = LabResult {
    magic: 0,
    clksel_at_reset: 0,
    clkhf_ctl1_at_reset: 0,
    sw_clk_at_reset: 0,
    sw_clk1_at_reset: 0,
    apb_clk_at_reset: 0,
    hclk_hz: 0,
    flash_status: 0,
    loops: 0,
};

fn read(addr: u32) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

fn delay_ms(ms: u32) {
    let end = time::now_us() + u64::from(ms) * 1000;
    while time::now_us() < end {}
}

/// 0 = pass, 1 = erase error, 2 = write error, 3 = read error,
/// 4 = not erased after erase, 5 = readback mismatch.
fn flash_test(flash: &mut pb03f_kit::InternalFlash) -> u32 {
    let mut pattern = [0u8; 40];
    for (i, b) in pattern.iter_mut().enumerate() {
        *b = (i as u8).wrapping_mul(37).wrapping_add(0x5a);
    }
    if flash
        .erase(FLASH_TEST_OFFSET, FLASH_TEST_OFFSET + 4096)
        .is_err()
    {
        return 1;
    }
    let mut buf = [0u8; 40];
    if flash.read(FLASH_TEST_OFFSET, &mut buf).is_err() {
        return 3;
    }
    if buf.iter().any(|&b| b != 0xff) {
        return 4;
    }
    // Unaligned length and offset exercise the <=8-byte STIG chunking.
    if flash.write(FLASH_TEST_OFFSET + 4, &pattern[..36]).is_err() {
        return 2;
    }
    if flash.read(FLASH_TEST_OFFSET, &mut buf).is_err() {
        return 3;
    }
    if buf[..4] != [0xff; 4] || buf[4..] != pattern[..36] {
        return 5;
    }
    0
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let r = unsafe { &mut *core::ptr::addr_of_mut!(LAB_RESULT) };
    r.clksel_at_reset = read(0x4000_f03c);
    r.clkhf_ctl1_at_reset = read(0x4000_f044);
    r.sw_clk_at_reset = read(0x4000_0008);
    r.sw_clk1_at_reset = read(0x4000_0014);
    r.apb_clk_at_reset = read(0x4000_0018);
    r.magic = 0x1ab0_0001;

    time::init();
    r.hclk_hz = clock::hclk_hz();
    r.magic = 0x1ab0_0002;

    let Ok(Resources {
        mut flash,
        mut leds,
        user_button,
        console,
    }) = Resources::take()
    else {
        loop {
            cortex_m::asm::nop();
        }
    };
    let mut uart = console.into_uart();
    let _ = write!(
        uart,
        "\npb03f-lab: boot ok\nreset clksel={:#010x} ctl1={:#010x} sw_clk={:#010x} sw_clk1={:#010x} apb={:#010x}\nnow hclk={} Hz clksel={:#010x} ctl1={:#010x}\n",
        r.clksel_at_reset,
        r.clkhf_ctl1_at_reset,
        r.sw_clk_at_reset,
        r.sw_clk1_at_reset,
        r.apb_clk_at_reset,
        r.hclk_hz,
        read(0x4000_f03c),
        read(0x4000_f044),
    );
    r.magic = 0x1ab0_0003;

    r.flash_status = flash_test(&mut flash);
    let _ = writeln!(
        uart,
        "flash test @{:#x}: {}",
        FLASH_TEST_OFFSET, r.flash_status
    );
    r.magic = 0x1ab0_0004;

    let t0 = time::now_us();
    delay_ms(1000);
    let _ = writeln!(uart, "1000 ms delay took {} us", time::now_us() - t0);

    let mut step = 0u32;
    loop {
        leds.all_off();
        match step % 5 {
            0 => leds.red.set_on(true),
            1 => leds.green.set_on(true),
            2 => leds.blue.set_on(true),
            3 => leds.cool_white.set_on(true),
            _ => leds.warm_white.set_on(true),
        }
        let _ = writeln!(
            uart,
            "t={} ms led={} button={}",
            time::now_us() / 1000,
            ["red", "green", "blue", "cool", "warm"][(step % 5) as usize],
            if user_button.is_pressed() {
                "pressed"
            } else {
                "released"
            }
        );
        r.loops = step;
        step = step.wrapping_add(1);
        delay_ms(500);
    }
}
