//! SSD1315 OLED: init, a border with text and the counter, the RGB LED
//! write in between (shared PA7), and a full-frame refresh rate.
//! Pass = the text is readable and upright, the counter advances, and the
//! LED colour changes do not corrupt the display.

#![no_std]
#![no_main]

#[path = "../shared/lab.rs"]
#[macro_use]
mod lab;

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use stm32wb5mm_dk::CycleDelay;
use stm32wb5mm_dk::spi1::{Spi1Bus, Spi1Pins};
use zigbee_display::PageFrame;

struct Line<const N: usize>([u8; N], usize);

impl<const N: usize> core::fmt::Write for Line<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &b in s.as_bytes() {
            if self.1 < N {
                self.0[self.1] = b;
                self.1 += 1;
            }
        }
        Ok(())
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = lab::init();
    let mut vcom = lab::Vcom::new(p.USART1, p.PB6);
    say!(
        vcom,
        "[DK][lcd] READY spi1 sck=PA1 mosi=PA7 cs=PH0 dc=PC9 rst=PC8"
    );
    let mut bus = Spi1Bus::new(Spi1Pins {
        spi1: p.SPI1,
        sck: p.PA1,
        mosi: p.PA7,
        lcd_cs: p.PH0,
        lcd_dc: p.PC9,
        lcd_rst: p.PC8,
        led_select: p.PH1,
    });
    let started = Instant::now();
    match bus.lcd().init(&mut CycleDelay) {
        Ok(()) => say!(
            vcom,
            "[DK][lcd] INIT ok in {} us",
            started.elapsed().as_micros()
        ),
        Err(error) => {
            say!(vcom, "[DK][lcd] INIT_FAIL {error:?}");
            lab::halt()
        }
    }
    let Some(mut frame) = PageFrame::new([0u8; 1024], 128, 64) else {
        say!(vcom, "[DK][lcd] FRAME_FAIL");
        lab::halt()
    };
    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let colours = [[0x20, 0, 0], [0, 0x20, 0], [0, 0, 0x20], [0, 0, 0]];
    for n in 0..30u32 {
        let _ = frame.clear(BinaryColor::Off);
        let _ = Rectangle::new(Point::zero(), Size::new(128, 64))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut frame);
        let _ = Text::with_baseline("zigbee-rs DK lab", Point::new(4, 3), style, Baseline::Top)
            .draw(&mut frame);
        let _ = Text::with_baseline("TOP LEFT", Point::new(4, 15), style, Baseline::Top)
            .draw(&mut frame);
        let mut line = Line([0u8; 21], 0);
        let _ = write!(line, "count {n}");
        let text = core::str::from_utf8(&line.0[..line.1]).unwrap_or("?");
        let _ = Text::with_baseline(text, Point::new(4, 50), style, Baseline::Top).draw(&mut frame);
        let t0 = Instant::now();
        let result = bus.lcd().flush(&frame);
        let us = t0.elapsed().as_micros();
        let led = bus.set_rgb(colours[n as usize % colours.len()]);
        say!(vcom, "[DK][lcd] FLUSH n={n} {result:?} {us} us led={led:?}");
        Timer::after_millis(500).await;
    }
    say!(vcom, "[DK][lcd] DONE");
    lab::halt()
}
