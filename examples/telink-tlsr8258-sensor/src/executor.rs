use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

fn noop_waker() -> Waker {
    const VTABLE: RawWakerVTable = RawWakerVTable::new(
        |pointer| RawWaker::new(pointer, &VTABLE),
        |_| {},
        |_| {},
        |_| {},
    );
    unsafe { Waker::new(core::ptr::null(), &VTABLE) }
}

pub fn block_on<F: Future>(future: F) -> F::Output {
    let mut future = future;
    let mut future = unsafe { Pin::new_unchecked(&mut future) };
    let waker = noop_waker();
    let mut context = Context::from_waker(&waker);

    loop {
        if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
            return output;
        }
        for _ in 0..100u32 {
            unsafe {
                core::arch::asm!("nop");
            }
        }
    }
}
