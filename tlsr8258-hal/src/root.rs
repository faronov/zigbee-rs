//! One-shot static storage for long-lived firmware root objects.
//!
//! TLSR8258 applications build their stack, stores and root future once and
//! then run forever. Keeping those objects in the entry function's frame makes
//! them permanent SVC-stack occupants that the linker cannot see. A
//! [`RootSlot`] moves such an object into linker-accounted `.bss` exactly once
//! and hands back the only reference to it.
//!
//! The stored value is never dropped: root objects live until reset.

use core::cell::UnsafeCell;
use core::mem::{MaybeUninit, align_of, size_of};
use core::pin::Pin;

/// Maximum alignment a [`RootSlot`] provides.
pub const ROOT_SLOT_ALIGN: usize = 8;

#[repr(C, align(8))]
struct Bytes<const N: usize>([MaybeUninit<u8>; N]);

/// `N` bytes of 8-byte-aligned static storage that can be claimed once.
pub struct RootSlot<const N: usize> {
    #[cfg(target_arch = "tc32")]
    claimed: UnsafeCell<bool>,
    #[cfg(not(target_arch = "tc32"))]
    claimed: core::sync::atomic::AtomicBool,
    bytes: UnsafeCell<Bytes<N>>,
}

// SAFETY: `bytes` is only reachable through the single successful `claim`,
// which is serialized by the one-shot `claimed` flag below.
unsafe impl<const N: usize> Sync for RootSlot<N> {}

impl<const N: usize> RootSlot<N> {
    /// An unclaimed slot.
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            #[cfg(target_arch = "tc32")]
            claimed: UnsafeCell::new(false),
            #[cfg(not(target_arch = "tc32"))]
            claimed: core::sync::atomic::AtomicBool::new(false),
            bytes: UnsafeCell::new(Bytes([MaybeUninit::uninit(); N])),
        }
    }

    /// Move `value` into the slot and return the only reference to it.
    ///
    /// Returns `value` unchanged if the slot was already claimed. The size and
    /// alignment of `T` are checked at compile time.
    ///
    /// `T: 'static` is required: the returned reference is `'static`, so a
    /// value borrowing shorter-lived data must not be stored.
    ///
    /// ```compile_fail,E0597
    /// use tlsr8258_hal::root::RootSlot;
    ///
    /// static SLOT: RootSlot<8> = RootSlot::new();
    /// let local = 7u32;
    /// let _stored = SLOT.claim(&local);
    /// ```
    // The one-shot flag makes the returned `&mut` unique, so handing it out
    // from `&'static self` (interior mutability) is sound.
    #[allow(clippy::mut_from_ref)]
    pub fn claim<T: 'static>(&'static self, value: T) -> Result<&'static mut T, T> {
        const {
            assert!(
                size_of::<T>() <= N,
                "RootSlot is smaller than the stored type"
            );
            assert!(
                align_of::<T>() <= ROOT_SLOT_ALIGN,
                "RootSlot alignment too small"
            );
        }
        if !self.take_flag() {
            return Err(value);
        }
        let pointer = self.bytes.get().cast::<T>();
        // SAFETY: the flag was clear, so no reference into `bytes` exists;
        // size and alignment were checked above; the storage is `'static`
        // and is never reused because the flag is never cleared.
        unsafe {
            pointer.write(value);
            Ok(&mut *pointer)
        }
    }

    /// [`claim`](Self::claim) and pin: the value never moves again.
    ///
    /// ```compile_fail,E0597
    /// use tlsr8258_hal::root::RootSlot;
    ///
    /// static SLOT: RootSlot<8> = RootSlot::new();
    /// let local = 7u32;
    /// let _stored = SLOT.claim_pinned(&local);
    /// ```
    pub fn claim_pinned<T: 'static>(&'static self, value: T) -> Result<Pin<&'static mut T>, T> {
        self.claim(value).map(Pin::static_mut)
    }

    #[cfg(target_arch = "tc32")]
    fn take_flag(&self) -> bool {
        // TC32 has no atomic read-modify-write; mask IRQs like
        // `Peripherals::take` so the check-and-set is indivisible.
        crate::mmio::with_irqs_disabled(|| unsafe {
            let claimed = self.claimed.get();
            let was_claimed = core::ptr::read_volatile(claimed);
            if !was_claimed {
                core::ptr::write_volatile(claimed, true);
            }
            !was_claimed
        })
    }

    #[cfg(not(target_arch = "tc32"))]
    fn take_flag(&self) -> bool {
        use core::sync::atomic::Ordering;
        self.claimed
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slot_is_claimed_exactly_once() {
        static SLOT: RootSlot<16> = RootSlot::new();
        let first = SLOT.claim([7u32; 4]).unwrap();
        first[1] = 9;
        assert_eq!(*first, [7, 9, 7, 7]);
        assert_eq!(SLOT.claim(5u8), Err(5));
    }

    #[test]
    fn pinned_claim_keeps_the_address() {
        static SLOT: RootSlot<8> = RootSlot::new();
        let pinned = SLOT.claim_pinned(0x1122_3344_5566_7788u64).unwrap();
        assert_eq!(
            &*pinned as *const u64 as usize % ROOT_SLOT_ALIGN,
            0,
            "slot storage must stay 8-byte aligned"
        );
        assert_eq!(*pinned, 0x1122_3344_5566_7788);
    }
}
