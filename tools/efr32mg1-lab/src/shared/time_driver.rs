use core::cell::RefCell;
use core::task::Waker;

use cortex_m::interrupt::Mutex;
use efr32mg1_hal::pm;
use embassy_time_queue_utils::Queue;

struct Efr32RtccTimeDriver {
    queue: Mutex<RefCell<Queue>>,
}

impl Efr32RtccTimeDriver {
    const fn new() -> Self {
        Self {
            queue: Mutex::new(RefCell::new(Queue::new())),
        }
    }

    fn init(&self) {
        if pm::init().is_err() {
            rtt_target::rprintln!("[EFR32][time_driver] RTCC_INIT_FAIL");
            loop {
                cortex_m::asm::nop();
            }
        }
        pm::enable_overflow_interrupt();
        cortex_m::peripheral::NVIC::unpend(crate::vectors::Interrupt::Rtcc);
        unsafe { cortex_m::peripheral::NVIC::unmask(crate::vectors::Interrupt::Rtcc) };
    }

    fn set_alarm(&self, deadline: u64) -> bool {
        if deadline == u64::MAX {
            pm::disarm_wake();
            return true;
        }
        let now = pm::now64();
        match pm::ticks_from_now_clamped(now, deadline) {
            None => false,
            Some(ticks_from_now) => {
                pm::arm_wake(ticks_from_now);
                true
            }
        }
    }

    #[inline(never)]
    fn rearm(&self) {
        cortex_m::interrupt::free(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            let mut next = queue.next_expiration(pm::now64());
            while !self.set_alarm(next) {
                next = queue.next_expiration(pm::now64());
            }
        });
    }
}

impl embassy_time_driver::Driver for Efr32RtccTimeDriver {
    fn now(&self) -> u64 {
        pm::now64()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        let changed = cortex_m::interrupt::free(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            queue.schedule_wake(at, waker)
        });
        if changed {
            self.rearm();
        }
    }
}

embassy_time_driver::time_driver_impl!(
    static TIME_DRIVER: Efr32RtccTimeDriver = Efr32RtccTimeDriver::new()
);

pub fn init() {
    TIME_DRIVER.init();
}

#[unsafe(no_mangle)]
pub extern "C" fn RTCC() {
    let flags = pm::take_pending_flags();
    if flags.overflow {
        pm::bump_wrap_count();
    }
    TIME_DRIVER.rearm();
}
