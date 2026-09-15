//! Single-future executor with interrupt-driven CPU idle, not SoC/radio sleep.

use core::future::Future;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

struct WakeSignal {
    ready: AtomicBool,
}

impl WakeSignal {
    const fn new() -> Self {
        Self {
            ready: AtomicBool::new(false),
        }
    }

    fn wake(&self) {
        self.ready.store(true, Ordering::SeqCst);
    }

    fn should_idle(&self) -> bool {
        !self.ready.load(Ordering::SeqCst)
    }

    fn waker(&'static self) -> Waker {
        // Every clone refers to static storage; queued timer wakers may outlive a run.
        unsafe { Waker::from_raw(RawWaker::new(core::ptr::from_ref(self).cast(), &VTABLE)) }
    }
}

unsafe fn clone_waker(data: *const ()) -> RawWaker {
    RawWaker::new(data, &VTABLE)
}

unsafe fn wake(data: *const ()) {
    // Only WakeSignal::waker creates these pointers, from a static WakeSignal.
    unsafe { &*data.cast::<WakeSignal>() }.wake();
}

unsafe fn drop_waker(_: *const ()) {}

static VTABLE: RawWakerVTable = RawWakerVTable::new(clone_waker, wake, wake, drop_waker);

fn run<F: Future>(
    future: F,
    signal: &'static WakeSignal,
    mut idle: impl FnMut(&WakeSignal),
) -> F::Output {
    let mut future = core::pin::pin!(future);
    let waker = signal.waker();
    let mut context = Context::from_waker(&waker);
    loop {
        signal.ready.store(false, Ordering::SeqCst);
        if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
            return output;
        }
        idle(signal);
    }
}

/// Run the firmware's sole root future, idling the CPU between interrupts.
///
/// Initialize the product time driver first. This does not turn off the radio,
/// stop clocks, or enter ESP light/deep sleep.
#[cfg(target_os = "none")]
pub fn block_on<F: Future>(future: F) -> F::Output {
    static SIGNAL: WakeSignal = WakeSignal::new();
    static RUNNING: AtomicBool = AtomicBool::new(false);
    assert!(
        !RUNNING.swap(true, Ordering::SeqCst),
        "ESP executor cannot be nested"
    );
    struct RunningGuard;
    impl Drop for RunningGuard {
        fn drop(&mut self) {
            RUNNING.store(false, Ordering::SeqCst);
        }
    }
    let _running = RunningGuard;
    run(future, &SIGNAL, |signal| {
        critical_section::with(|_| {
            if signal.should_idle() {
                // esp-hal masks mstatus.MIE, not individual interrupt sources.
                // RISC-V WFI resumes for a pending enabled source even with MIE
                // clear, closing the check/idle race; the ISR runs on CS exit.
                unsafe { core::arch::asm!("wfi", options(nomem, nostack)) };
            }
        });
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::{Cell, RefCell};
    use std::boxed::Box;

    fn signal() -> &'static WakeSignal {
        Box::leak(Box::new(WakeSignal::new()))
    }

    #[test]
    fn ready_future_never_idles() {
        let result = run(async { 42 }, signal(), |_| panic!("unexpected idle"));
        assert_eq!(result, 42);
    }

    #[test]
    fn self_wake_does_not_enter_idle() {
        let polls = Cell::new(0);
        let future = core::future::poll_fn(|cx| {
            polls.set(polls.get() + 1);
            if polls.get() == 3 {
                Poll::Ready(())
            } else {
                cx.waker().wake_by_ref();
                Poll::Pending
            }
        });
        run(future, signal(), |signal| assert!(!signal.should_idle()));
        assert_eq!(polls.get(), 3);
    }

    #[test]
    fn pending_future_is_polled_again_after_external_wake() {
        let ready = Cell::new(false);
        let polls = Cell::new(0);
        let waits = Cell::new(0);
        let registered = RefCell::new(None);
        let future = core::future::poll_fn(|cx| {
            polls.set(polls.get() + 1);
            if ready.get() {
                Poll::Ready(7)
            } else {
                *registered.borrow_mut() = Some(cx.waker().clone());
                Poll::Pending
            }
        });
        let output = run(future, signal(), |signal| {
            assert!(signal.should_idle());
            waits.set(waits.get() + 1);
            ready.set(true);
            registered.borrow_mut().take().unwrap().wake();
            assert!(!signal.should_idle());
        });
        assert_eq!((output, polls.get(), waits.get()), (7, 2, 1));
    }

    #[test]
    fn wake_between_poll_and_idle_check_prevents_sleep() {
        let ready = Cell::new(false);
        let future = core::future::poll_fn(|_| {
            if ready.get() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        });
        run(future, signal(), |signal| {
            ready.set(true);
            signal.wake();
            assert!(!signal.should_idle());
        });
    }

    #[test]
    fn cloned_waker_remains_valid_after_root_future_completes() {
        let signal = signal();
        let retained = RefCell::new(None);
        run(
            core::future::poll_fn(|cx| {
                *retained.borrow_mut() = Some(cx.waker().clone());
                Poll::Ready(())
            }),
            signal,
            |_| panic!("unexpected idle"),
        );
        retained.borrow_mut().take().unwrap().wake();
        assert!(!signal.should_idle());
    }
}
