//! PB-03F-Kit hardware lab: proves ROM boot, the 48 MHz clock switch, the
//! CH340 UART console, the five LEDs, the user button, internal-flash
//! erase/program/read on a spare sector and the ADC supply monitor (P14
//! driven high), independently of the Zigbee stack. It also runs the
//! AES-128 block's known-answer test and samples its status register around
//! one single-mode block (`zigbee_mac::phy6222::Phy6222Aes`), to prove the
//! completion semantics the production engine does not rely on yet.
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
use zigbee_mac::phy6222::{Phy6222Aes, Phy6222AesError};

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
    /// Last supply reading in mV (`u32::MAX` = ADC error).
    supply_mv: u32,
    /// 0 = AES-128 KAT passed, 1 = engine already taken, 2 = clock
    /// unavailable, 0x10 + n = vector n mismatched.
    aes_kat: u32,
    /// Status register (`0x4004_0014`) samples around one block: before
    /// start, right after start, after the fixed wait, after stop.
    aes_status: [u32; 4],
    /// Status reads until the single-mode done bit (`u32::MAX`: never).
    aes_polls_until_done: u32,
    /// Smallest completion wait (CPU cycles) that passed the KAT in the
    /// sweep (`u32::MAX`: none).
    aes_min_wait_cycles: u32,
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
    supply_mv: 0,
    aes_kat: u32::MAX,
    aes_status: [0; 4],
    aes_polls_until_done: 0,
    aes_min_wait_cycles: u32::MAX,
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

fn aes_code(result: Result<(), Phy6222AesError>) -> u32 {
    match result {
        Ok(()) => 0,
        Err(Phy6222AesError::ClockUnavailable) => 2,
        Err(Phy6222AesError::KnownAnswerTestFailed(vector)) => 0x10 + u32::from(vector),
        Err(_) => 3,
    }
}

/// AES-128 block: production KAT, status probe, and a completion-wait
/// sweep (smallest passing wait), ending with the production wait and a
/// passing KAT again.
fn aes_test(r: &mut LabResult, uart: &mut impl Write) {
    let Some(mut aes) = Phy6222Aes::take() else {
        r.aes_kat = 1;
        return;
    };
    r.aes_kat = aes_code(aes.self_test());
    if let Ok(probe) = aes.completion_probe() {
        r.aes_status = [
            probe.status_before_start,
            probe.status_after_start,
            probe.status_after_wait,
            probe.status_after_stop,
        ];
        r.aes_polls_until_done = probe.polls_until_done.unwrap_or(u32::MAX);
        let _ = writeln!(uart, "AES probe: {:?}", probe);
    }
    for cycles in [8, 16, 32, 64, 128, 256, 512] {
        if aes.set_completion_cycles(cycles).is_ok() && aes.self_test().is_ok() {
            r.aes_min_wait_cycles = cycles;
            break;
        }
    }
    let restored = aes
        .set_completion_cycles(zigbee_mac::phy6222::aes::COMPLETION_WAIT_CYCLES)
        .and_then(|()| aes.self_test());
    if restored.is_err() {
        r.aes_kat = aes_code(restored);
    }
    let _ = writeln!(
        uart,
        "AES-128 KAT: {} (0 = pass), min passing wait {} cycles",
        r.aes_kat, r.aes_min_wait_cycles
    );
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
        supply_monitor,
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

    aes_test(r, &mut uart);
    r.magic = 0x1ab0_0005;

    let mut supply = supply_monitor.into_monitor();

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
        r.supply_mv = match supply.read_millivolts() {
            Ok(mv) => mv,
            Err(e) => {
                let _ = writeln!(uart, "supply ADC error: {:?}", e);
                u32::MAX
            }
        };
        let _ = writeln!(
            uart,
            "t={} ms supply={} mV led={} button={}",
            time::now_us() / 1000,
            r.supply_mv,
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
