//! TSC touch key and buttons: 1 ms discharge, one acquisition, log the
//! count every 100 ms for 60 s together with B1/B2. Pass = a stable
//! baseline that drops clearly while the key is touched, and both buttons
//! read low while pressed.

#![no_std]
#![no_main]

#[path = "../shared/lab.rs"]
#[macro_use]
mod lab;

use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use stm32wb5mm_dk::touch::{self, Acquisition, TouchKey, TouchPins};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = lab::init();
    let mut vcom = lab::Vcom::new(p.USART1, p.PB6);
    say!(
        vcom,
        "[DK][touch] READY tsc g6=PD10/PD11 g4=PC6/PC7 b1=PC12 b2=PC13"
    );
    let b1 = stm32wb5mm_dk::button_b1(p.PC12);
    let b2 = stm32wb5mm_dk::button_b2(p.PC13);
    let mut key = match TouchKey::new(TouchPins {
        tsc: p.TSC,
        key_sample: p.PD10,
        key: p.PD11,
        shield_sample: p.PC6,
        shield: p.PC7,
    }) {
        Ok(key) => key,
        Err(error) => {
            say!(vcom, "[DK][touch] INIT_FAIL {error:?}");
            lab::halt()
        }
    };
    let (mut min, mut max) = (u16::MAX, 0u16);
    for n in 0..600u32 {
        key.discharge();
        Timer::after_micros(u64::from(touch::DISCHARGE_US)).await;
        let t0 = Instant::now();
        key.start();
        let result = loop {
            match key.poll() {
                Acquisition::Busy if t0.elapsed().as_millis() < 20 => {}
                Acquisition::Busy => break None,
                other => break Some(other),
            }
        };
        let us = t0.elapsed().as_micros();
        if let Some(Acquisition::Count(count)) = result {
            min = min.min(count);
            max = max.max(count);
        }
        if n % 5 == 0 {
            say!(
                vcom,
                "[DK][touch] n={n} {result:?} {us} us min={min} max={max} b1={} b2={}",
                u8::from(b1.is_low()),
                u8::from(b2.is_low())
            );
        }
        Timer::after_millis(100).await;
    }
    say!(vcom, "[DK][touch] DONE min={min} max={max}");
    lab::halt()
}
