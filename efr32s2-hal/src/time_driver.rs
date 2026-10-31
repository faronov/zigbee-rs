//! Embassy time driver on the RTCC (feature `time-driver`).
//!
//! Runs at 32,768 Hz from the LFRCO and keeps counting in EM2, so Embassy
//! timers stay valid across [`crate::power`] sleeps. A software queue
//! multiplexes timers onto RTCC CC0; the overflow interrupt extends the
//! counter to 64 bits. This module defines the `RTCC` interrupt handler,
//! which also clears the CC1 flag used by [`crate::power::sleep_em2`].
//!
//! The application must select `embassy-time-driver/tick-hz-32_768` and
//! call [`init`] with the started [`Rtcc`] before using Embassy time.

use core::cell::RefCell;
use core::task::Waker;

use cortex_m::interrupt::Mutex;
use embassy_time_queue_utils::Queue;

use crate::rtcc::{self, Channel, Rtcc};
use crate::vectors::Interrupt;

const _: () = assert!(
    embassy_time_driver::TICK_HZ == rtcc::RTCC_HZ as u64,
    "efr32s2-hal time-driver needs embassy-time-driver/tick-hz-32_768"
);

struct RtccDriver {
    queue: Mutex<RefCell<Queue>>,
}

impl RtccDriver {
    /// Arm CC0 for `deadline`; `false` if it already passed.
    fn set_alarm(deadline: u64) -> bool {
        if deadline == u64::MAX {
            rtcc::disarm(Channel::Cc0);
            return true;
        }
        match rtcc::ticks_until(rtcc::now64(), deadline) {
            None => false,
            Some(ticks) => {
                rtcc::arm(Channel::Cc0, ticks);
                true
            }
        }
    }

    fn rearm(&self) {
        cortex_m::interrupt::free(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            let mut next = queue.next_expiration(rtcc::now64());
            while !Self::set_alarm(next) {
                next = queue.next_expiration(rtcc::now64());
            }
        });
    }
}

impl embassy_time_driver::Driver for RtccDriver {
    fn now(&self) -> u64 {
        rtcc::now64()
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
    static DRIVER: RtccDriver = RtccDriver {
        queue: Mutex::new(RefCell::new(Queue::new())),
    }
);

/// Hand the running RTCC to Embassy and unmask its interrupt.
pub fn init(rtcc: Rtcc) {
    // The handle only proves initialization; the driver uses the
    // module-level register functions from interrupt context.
    let _started = rtcc;
    rtcc::enable_overflow_interrupt();
    cortex_m::peripheral::NVIC::unpend(Interrupt::RTCC);
    unsafe { cortex_m::peripheral::NVIC::unmask(Interrupt::RTCC) };
}

/// RTCC interrupt: overflow epoch, Embassy CC0 alarm, CC1 sleep wake.
#[unsafe(no_mangle)]
extern "C" fn RTCC() {
    let flags = rtcc::take_flags();
    if flags.overflow {
        rtcc::note_overflow();
    }
    if flags.cc0 || flags.overflow {
        DRIVER.rearm();
    }
}
