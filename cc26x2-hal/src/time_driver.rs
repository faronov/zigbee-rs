//! Embassy time driver on the AON RTC (feature `time-driver`).
//!
//! 32,768 Hz ticks from [`crate::aon_rtc`]; one compare channel (CH0)
//! multiplexed by `embassy-time-queue-utils`. Defines the `AON_RTC_COMB`
//! interrupt handler. The application selects
//! `embassy-time-driver/tick-hz-32_768` and calls [`init`] after
//! [`crate::setup::trim_device`] started the RTC.

use core::cell::RefCell;
use core::task::Waker;

use cortex_m::interrupt::Mutex;
use embassy_time_queue_utils::Queue;

use crate::aon_rtc::{self, RtcToken};
use crate::vectors::Interrupt;

const _: () = assert!(
    embassy_time_driver::TICK_HZ == aon_rtc::TICK_HZ as u64,
    "cc26x2-hal time-driver needs embassy-time-driver/tick-hz-32_768"
);

struct RtcDriver {
    queue: Mutex<RefCell<Queue>>,
}

impl RtcDriver {
    fn rearm(&self) {
        cortex_m::interrupt::free(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            loop {
                let now = aon_rtc::now_ticks();
                let next = queue.next_expiration(now);
                if next == u64::MAX {
                    aon_rtc::disarm_ch0();
                    return;
                }
                if next > now {
                    aon_rtc::arm_ch0(next);
                    // A deadline that passed while arming is serviced now.
                    if aon_rtc::now_ticks() < next {
                        return;
                    }
                }
            }
        });
    }
}

impl embassy_time_driver::Driver for RtcDriver {
    fn now(&self) -> u64 {
        aon_rtc::now_ticks()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        let changed = cortex_m::interrupt::free(|cs| {
            self.queue.borrow(cs).borrow_mut().schedule_wake(at, waker)
        });
        if changed {
            self.rearm();
        }
    }
}

embassy_time_driver::time_driver_impl!(
    static DRIVER: RtcDriver = RtcDriver {
        queue: Mutex::new(RefCell::new(Queue::new())),
    }
);

/// Hand the running RTC to Embassy and unmask its interrupt.
pub fn init(rtc: RtcToken) {
    let _owned = rtc;
    aon_rtc::disarm_ch0();
    cortex_m::peripheral::NVIC::unpend(Interrupt::AON_RTC_COMB);
    unsafe { cortex_m::peripheral::NVIC::unmask(Interrupt::AON_RTC_COMB) };
}

/// AON RTC combined event: Embassy CH0 alarm.
#[unsafe(no_mangle)]
extern "C" fn AON_RTC_COMB() {
    if aon_rtc::take_ch0_event() {
        DRIVER.rearm();
    }
}
