#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

#[path = "../shared/fault.rs"]
mod fault;
#[path = "../shared/platform.rs"]
mod platform;
#[path = "../shared/time_driver.rs"]
mod time_driver;
#[path = "../shared/vectors.rs"]
mod vectors;

use cortex_m as _;
use embassy_time::{Duration, Timer};
use embedded_hal::pwm::SetDutyCycle;

#[embassy_executor::task]
async fn run(mut pwm: efr32mg1_tradfri::LedPwm) -> ! {
    rtt_target::rprintln!(
        "[EFR32][diag-pwm] READY controller=TIMER0 channel=0 loc=0 pin=PA0 hz={}",
        pwm.actual_frequency_hz()
    );
    let max = pwm.max_duty_cycle();
    let levels = [0, max / 4, max / 2, (u32::from(max) * 3 / 4) as u16, max];
    loop {
        for duty in levels {
            if let Err(error) = pwm.set_duty_cycle(duty) {
                rtt_target::rprintln!("[EFR32][diag-pwm] DUTY_FAIL error={:?}", error);
                platform::halt()
            }
            rtt_target::rprintln!("[EFR32][diag-pwm] DUTY value={} max={}", duty, max);
            Timer::after(Duration::from_millis(500)).await;
        }
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    platform::init_small!("diag-pwm");
    time_driver::init();
    let pwm = match efr32mg1_tradfri::led_pwm() {
        Ok(pwm) => pwm,
        Err(error) => {
            rtt_target::rprintln!("[EFR32][diag-pwm] INIT_FAIL error={:?}", error);
            platform::halt()
        }
    };
    static EXECUTOR: static_cell::StaticCell<embassy_executor::Executor> =
        static_cell::StaticCell::new();
    EXECUTOR
        .init(embassy_executor::Executor::new())
        .run(|spawner| spawner.must_spawn(run(pwm)))
}
