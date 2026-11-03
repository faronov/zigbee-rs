//! Retained-RAM primitives for the reset-on-wake (LOW32K) image.
//!
//! A LOW32K wake re-enters `_start`, keeps `.data`/`.bss`/`.retained` intact
//! and calls `_rust_retention_entry` on fresh stacks (see the crate-level
//! startup). Composition roots place their long-lived objects in
//! [`RetainedCell`]s inside the product's NOLOAD `.retained` section and
//! publish them with one [`RetainedRoot`] header. A wake is accepted only if
//! that header still names the same cells and the stack guard below the fresh
//! SVC stack is intact; anything else must reset rather than resume.

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{Ordering, compiler_fence};

/// Marker that selects the product linker script's retention layout
/// (`DEFINED(TELINK_RETENTION_IMAGE)`). It exists only with this feature.
#[cfg(target_arch = "tc32")]
#[unsafe(no_mangle)]
#[used]
#[unsafe(link_section = ".retention_image_marker")]
pub static TELINK_RETENTION_IMAGE: u32 = 0x4c33_324b;

/// Statically placed storage that is initialized explicitly on cold boot and
/// reused untouched after a retention wake.
pub struct RetainedCell<T>(UnsafeCell<MaybeUninit<T>>);

// SAFETY: the TLSR8258 runtime is single-threaded; callers uphold the
// exclusive-access contracts of the `unsafe` accessors below.
unsafe impl<T> Sync for RetainedCell<T> {}

// The `unsafe` accessors hand out `&'static mut` from a shared static by
// design: exclusivity is the caller's documented safety contract.
#[allow(clippy::mut_from_ref)]
impl<T> RetainedCell<T> {
    pub const fn new() -> Self {
        Self(UnsafeCell::new(MaybeUninit::uninit()))
    }

    /// Uninitialized slot for in-place construction (`build_into`).
    ///
    /// # Safety
    /// No other reference to this cell may be live.
    pub unsafe fn slot(&'static self) -> &'static mut MaybeUninit<T> {
        unsafe { &mut *self.0.get() }
    }

    /// Move `value` into the cell.
    ///
    /// # Safety
    /// No other reference to this cell may be live.
    pub unsafe fn initialize(&'static self, value: T) -> &'static mut T {
        unsafe { self.slot() }.write(value)
    }

    /// # Safety
    /// The cell must have been initialized and no other reference may be live.
    pub unsafe fn get(&'static self) -> &'static mut T {
        unsafe { (&mut *self.0.get()).assume_init_mut() }
    }

    pub fn address(&'static self) -> u32 {
        self.0.get() as u32
    }
}

impl<T> Default for RetainedCell<T> {
    fn default() -> Self {
        Self::new()
    }
}

const ROOT_MAGIC: u32 = 0x5345_4431; // "SED1"
const ROOT_VERSION: u32 = 1;
#[cfg(target_arch = "tc32")]
const STACK_GUARD_BYTE: u8 = 0xa5;

#[repr(C)]
#[derive(Clone, Copy)]
struct RootHeader<const N: usize> {
    magic: u32,
    magic_inverse: u32,
    version: u32,
    addresses: [u32; N],
    checksum: u32,
}

impl<const N: usize> RootHeader<N> {
    #[cfg(any(target_arch = "tc32", test))]
    const fn invalid() -> Self {
        Self {
            magic: 0,
            magic_inverse: !0,
            version: 0,
            addresses: [0; N],
            checksum: 0,
        }
    }

    fn checksum(&self) -> u32 {
        let mut value = 0x6d5a_56a9u32;
        let mut mix = |word: u32| {
            value = (value.rotate_left(7) ^ word).wrapping_mul(0x9e37_79b1);
        };
        mix(ROOT_MAGIC);
        mix(!ROOT_MAGIC);
        mix(self.version);
        for &address in &self.addresses {
            mix(address);
        }
        value
    }

    #[cfg(any(target_arch = "tc32", test))]
    fn valid(&self, addresses: &[u32; N]) -> bool {
        self.magic == ROOT_MAGIC
            && self.magic_inverse == !ROOT_MAGIC
            && self.version == ROOT_VERSION
            && self.addresses == *addresses
            && self.checksum == self.checksum()
    }
}

/// Commit record naming the `N` retained cells of one composition.
///
/// The cold path calls [`begin_cold`](Self::begin_cold) before touching any
/// cell and [`commit`](Self::commit) after all of them are initialized; the
/// magic word is published last, so an interrupted cold boot never validates.
pub struct RetainedRoot<const N: usize>(RetainedCell<RootHeader<N>>);

impl<const N: usize> RetainedRoot<N> {
    pub const fn new() -> Self {
        Self(RetainedCell::new())
    }

    /// Invalidate any previous root and paint the retention stack guard.
    #[cfg(target_arch = "tc32")]
    pub fn begin_cold(&'static self) {
        self.invalidate();
        let (mut cursor, end) = stack_guard_range();
        while cursor < end {
            unsafe { core::ptr::write_volatile(cursor, STACK_GUARD_BYTE) };
            cursor = unsafe { cursor.add(1) };
        }
    }

    #[cfg(any(target_arch = "tc32", test))]
    fn invalidate(&'static self) {
        unsafe {
            core::ptr::write(self.0.0.get(), MaybeUninit::new(RootHeader::invalid()));
        }
    }

    /// Publish `addresses` (each a [`RetainedCell::address`]).
    pub fn commit(&'static self, addresses: [u32; N]) {
        let mut header = RootHeader {
            magic: 0,
            magic_inverse: !ROOT_MAGIC,
            version: ROOT_VERSION,
            addresses,
            checksum: 0,
        };
        header.checksum = header.checksum();
        let stored = unsafe { self.0.initialize(header) };
        compiler_fence(Ordering::SeqCst);
        unsafe { core::ptr::write_volatile(&mut stored.magic, ROOT_MAGIC) };
        compiler_fence(Ordering::SeqCst);
    }

    #[cfg(any(target_arch = "tc32", test))]
    fn header_valid(&'static self, addresses: &[u32; N]) -> bool {
        unsafe { &*self.0.0.get().cast::<RootHeader<N>>() }.valid(addresses)
    }

    /// True only if this root was committed for exactly `addresses` and the
    /// retention stack guard is intact.
    #[cfg(target_arch = "tc32")]
    pub fn is_valid(&'static self, addresses: [u32; N]) -> bool {
        self.header_valid(&addresses) && stack_guard_intact()
    }
}

impl<const N: usize> Default for RetainedRoot<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(target_arch = "tc32")]
unsafe extern "C" {
    static _retention_stack_guard_start_: u8;
    static _retention_stack_guard_end_: u8;
}

#[cfg(target_arch = "tc32")]
fn stack_guard_range() -> (*mut u8, *mut u8) {
    (
        core::ptr::addr_of!(_retention_stack_guard_start_) as *mut u8,
        core::ptr::addr_of!(_retention_stack_guard_end_) as *mut u8,
    )
}

/// True if the linker-placed guard below the retention SVC stack still holds
/// the pattern painted by [`RetainedRoot::begin_cold`].
#[cfg(target_arch = "tc32")]
pub fn stack_guard_intact() -> bool {
    let (mut cursor, end) = stack_guard_range();
    while cursor < end {
        if unsafe { core::ptr::read_volatile(cursor) } != STACK_GUARD_BYTE {
            return false;
        }
        cursor = unsafe { cursor.add(1) };
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The checksum is the pre-extraction algorithm, word for word.
    fn reference_checksum(version: u32, addresses: [u32; 4]) -> u32 {
        let mut value = 0x6d5a_56a9u32;
        for word in [
            0x5345_4431,
            !0x5345_4431,
            version,
            addresses[0],
            addresses[1],
            addresses[2],
            addresses[3],
        ] {
            value = value.rotate_left(7) ^ word;
            value = value.wrapping_mul(0x9e37_79b1);
        }
        value
    }

    #[test]
    fn header_layout_and_checksum_match_the_proven_image() {
        assert_eq!(core::mem::size_of::<RootHeader<4>>(), 32);
        let header = RootHeader {
            magic: ROOT_MAGIC,
            magic_inverse: !ROOT_MAGIC,
            version: ROOT_VERSION,
            addresses: [0x841000, 0x841400, 0x841800, 0x841c00],
            checksum: 0,
        };
        assert_eq!(
            header.checksum(),
            reference_checksum(1, [0x841000, 0x841400, 0x841800, 0x841c00])
        );
    }

    #[test]
    fn root_validates_only_committed_addresses() {
        static ROOT: RetainedRoot<2> = RetainedRoot::new();
        ROOT.invalidate();
        assert!(!ROOT.header_valid(&[1, 2]));
        ROOT.commit([1, 2]);
        assert!(ROOT.header_valid(&[1, 2]));
        assert!(!ROOT.header_valid(&[1, 3]));
        ROOT.invalidate();
        assert!(!ROOT.header_valid(&[1, 2]));
    }

    #[test]
    fn corrupted_header_is_rejected() {
        static ROOT: RetainedRoot<1> = RetainedRoot::new();
        ROOT.commit([7]);
        unsafe { (*ROOT.0.0.get()).assume_init_mut().checksum ^= 1 };
        assert!(!ROOT.header_valid(&[7]));
    }

    #[test]
    fn cell_round_trips_a_value() {
        static CELL: RetainedCell<u32> = RetainedCell::new();
        unsafe {
            *CELL.initialize(5) += 1;
            assert_eq!(*CELL.get(), 6);
        }
        assert_ne!(CELL.address(), 0);
    }
}
