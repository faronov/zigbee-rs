//! TLC59731 RGB LED: red, green, blue, white, off, two seconds each, then
//! a slow hue ramp. Pass = the colours appear in that order.

#![no_std]
#![no_main]

#[path = "../shared/lab.rs"]
#[macro_use]
mod lab;

use embassy_executor::Spawner;
use embassy_time::Timer;
use stm32wb5mm_dk::spi1::{Spi1Bus, Spi1Pins};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = lab::init();
    let mut vcom = lab::Vcom::new(p.USART1, p.PB6);
    say!(vcom, "[DK][rgb] READY sdi=PA7 select=PH1");
    let mut bus = Spi1Bus::new(Spi1Pins {
        spi1: p.SPI1,
        sck: p.PA1,
        mosi: p.PA7,
        lcd_cs: p.PH0,
        lcd_dc: p.PC9,
        lcd_rst: p.PC8,
        led_select: p.PH1,
    });
    let steps: [(&str, [u8; 3]); 5] = [
        ("red", [0x40, 0, 0]),
        ("green", [0, 0x40, 0]),
        ("blue", [0, 0, 0x40]),
        ("white", [0x30, 0x30, 0x30]),
        ("off", [0, 0, 0]),
    ];
    for round in 0..3u32 {
        for (name, rgb) in steps {
            match bus.set_rgb(rgb) {
                Ok(()) => say!(vcom, "[DK][rgb] round={round} SET {name} {rgb:?}"),
                Err(error) => say!(vcom, "[DK][rgb] FAIL {name} {error:?}"),
            }
            Timer::after_secs(2).await;
        }
    }
    say!(vcom, "[DK][rgb] DONE");
    lab::halt()
}
