//! BRD4001A + BRD4183A lab plumbing: HFXO, LED0, VCOM logger, panic output.
//!
//! Board facts (GSDK 4.5.0 `hardware/board/config/brd4183a_brd4001a`):
//! LED0 PB1, BTN0 PB0, VCOM USART TX PA5 / RX PA6, VCOM enable PA4,
//! HFXO 38.4 MHz CTUNE 120, PTI DOUT PC4 / DFRAME PC5.

#![allow(dead_code)]

use core::cell::RefCell;
use core::fmt::Write;

use cortex_m::interrupt::Mutex;
use efr32s2_hal::{
    clock::HfxoConfig,
    gpio::{Gpio, Level, Pin, Port, PushPull},
    i2c::I2c0Token,
    peripherals::Peripherals,
    radio::RadioToken,
    usart::{UartTx, Usart},
};

pub const HFXO: HfxoConfig = HfxoConfig {
    frequency_hz: 38_400_000,
    ctune: 120,
};

static LOG: Mutex<RefCell<Option<UartTx>>> = Mutex::new(RefCell::new(None));

/// Write formatted text to the VCOM UART (no-op before [`init`]).
pub fn log_fmt(args: core::fmt::Arguments<'_>) {
    cortex_m::interrupt::free(|cs| {
        if let Some(uart) = LOG.borrow(cs).borrow_mut().as_mut() {
            let _ = uart.write_fmt(args);
            let _ = uart.write_str("\r\n");
        }
    });
}

#[macro_export]
macro_rules! log {
    ($($arg:tt)*) => { $crate::shared::log_fmt(format_args!($($arg)*)) };
}

pub struct Lab {
    pub gpio: Gpio,
    pub i2c0: I2c0Token,
    pub radio: RadioToken,
    pub usart0: Usart,
    pub led: Pin<PushPull>,
    pub vcom_enable: Pin<PushPull>,
}

/// Bring up HFXO, LED0 and the VCOM logger; returns the remaining resources.
pub fn init(name: &str) -> Lab {
    let Peripherals {
        mut clocks,
        mut gpio,
        usart0,
        usart1,
        i2c0,
        radio,
        ..
    } = Peripherals::take().expect("peripherals taken once");
    let hfxo = clocks.configure_hfxo(HFXO);
    // SAFETY: each pin is claimed exactly once here.
    let (led, vcom_enable, tx) = unsafe {
        (
            gpio.claim_pin(Port::B, 1).unwrap(),
            gpio.claim_pin(Port::A, 4).unwrap(),
            gpio.claim_pin(Port::A, 5).unwrap(),
        )
    };
    let led = led.into_push_pull(Level::Low);
    let vcom_enable = vcom_enable.into_push_pull(Level::High);
    let uart = usart1.into_uart_tx(tx.into_push_pull(Level::High), 115_200);
    cortex_m::interrupt::free(|cs| *LOG.borrow(cs).borrow_mut() = Some(uart));
    efr32s2_hal::delay::enable_cycle_counter();
    log!("");
    log!("== efr32s2-lab {} ({:?}) ==", name, efr32s2_hal::chip::CHIP);
    match hfxo {
        Ok(c) => log!("HFXO ok, HCLK {} Hz", c.hclk_hz()),
        Err(e) => log!("HFXO FAILED: {:?}", e),
    }
    log!("EUI64 {:016X}", eui64());
    Lab {
        gpio,
        i2c0,
        radio,
        usart0,
        led,
        vcom_enable,
    }
}

/// Factory EUI-64 from DEVINFO (`EUI64L` 0x0FE08048, `EUI64H` 0x0FE0804C).
pub fn eui64() -> u64 {
    unsafe {
        let lo = core::ptr::read_volatile(0x0FE0_8048 as *const u32) as u64;
        let hi = core::ptr::read_volatile(0x0FE0_804C as *const u32) as u64;
        (hi << 32) | lo
    }
}

pub fn delay_ms(ms: u32) {
    for _ in 0..ms {
        efr32s2_hal::delay::spin_cycles(efr32s2_hal::delay::us_to_cycles(1000));
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    log!("PANIC: {}", info);
    loop {
        cortex_m::asm::bkpt();
    }
}

#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    log!("HARDFAULT pc={:08X} lr={:08X}", frame.pc(), frame.lr());
    loop {
        cortex_m::asm::bkpt();
    }
}
