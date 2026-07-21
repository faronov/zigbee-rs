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
use efr32mg1_hal::pm;
use embassy_time::{Duration, Timer};
use zigbee_mac::MacDriver;
use zigbee_mac::efr32::Efr32Mac;
use zigbee_mac::frames::build_beacon_request;
use zigbee_mac::pib::{PibAttribute, PibValue};

const ITERATIONS: u8 = 5;
const SLEEP_MS: u32 = 500;
const CHANNEL: u8 = 15;
const TEST_SEQ: u8 = 0xA5;

async fn transmit(mac: &mut Efr32Mac, sequence: u8) -> bool {
    if mac
        .mlme_set(PibAttribute::PhyCurrentChannel, PibValue::U8(CHANNEL))
        .await
        .is_err()
    {
        return false;
    }
    mac.debug_transmit_raw(&build_beacon_request(sequence))
        .await
        .is_ok()
}

#[embassy_executor::task]
async fn run(mut mac: Efr32Mac) -> ! {
    let sleep_ticks = pm::ms_to_ticks(SLEEP_MS, pm::LFRCO_HZ);
    let mut failures = 0u8;
    rtt_target::rprintln!(
        "[EFR32][diag-radio-em2] BOOT iterations={} sleep_ms={} channel={} \
         nv=off i2c=off zigbee=off",
        ITERATIONS,
        SLEEP_MS,
        CHANNEL
    );

    if transmit(&mut mac, TEST_SEQ).await {
        rtt_target::rprintln!("[EFR32][diag-radio-em2] initial_tx=PASS");
    } else {
        failures += 1;
    }

    for iteration in 1..=ITERATIONS {
        mac.radio_sleep();
        cortex_m::peripheral::NVIC::unpend(vectors::Interrupt::FrcPri);
        let before = pm::now();
        let sleep = pm::sleep_for_ticks_polled(sleep_ticks);
        let elapsed = pm::elapsed_ticks(before, pm::now());
        let clock_ready = efr32mg1_tradfri::init_clocks().is_ok();
        mac.radio_wake();
        let tx_ok = transmit(&mut mac, TEST_SEQ.wrapping_add(iteration)).await;
        let progressed = pm::progressed_within_tolerance(elapsed, sleep_ticks, 10);
        let sleepdeep_cleared = !pm::sleepdeep_is_set();
        let passed = sleep.is_ok() && clock_ready && tx_ok && progressed && sleepdeep_cleared;
        failures += (!passed) as u8;
        rtt_target::rprintln!(
            "[EFR32][diag-radio-em2] iter={} elapsed_ticks={} elapsed_ms={} sleep_ok={} \
             clock_ready={} tx_ok={} progressed={} sleepdeep_cleared={} => {}",
            iteration,
            elapsed,
            pm::ticks_to_ms(elapsed, pm::LFRCO_HZ),
            sleep.is_ok(),
            clock_ready,
            tx_ok,
            progressed,
            sleepdeep_cleared,
            if passed { "PASS" } else { "FAIL" }
        );
    }

    if failures == 0 {
        rtt_target::rprintln!(
            "[EFR32][diag-radio-em2] PASS iterations={} failures=0",
            ITERATIONS
        );
        platform::led_on();
    } else {
        rtt_target::rprintln!(
            "[EFR32][diag-radio-em2] FAIL iterations={} failures={}",
            ITERATIONS,
            failures
        );
    }
    loop {
        Timer::after(Duration::from_secs(3_600)).await;
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    platform::init_large!("diag-radio-em2");
    time_driver::init();
    static EXECUTOR: static_cell::StaticCell<embassy_executor::Executor> =
        static_cell::StaticCell::new();
    EXECUTOR
        .init(embassy_executor::Executor::new())
        .run(|spawner| spawner.must_spawn(run(Efr32Mac::new())))
}
