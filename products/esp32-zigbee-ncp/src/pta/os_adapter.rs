//! Bookkeeping of the OS adapter that `libcoexist.a` calls
//! (`coex_adapter_funcs_t`), host-testable and failing closed.
//!
//! The NCP is bare metal on one core without a scheduler, so the adapter
//! cannot provide everything a FreeRTOS adapter does:
//!
//! * a semaphore that is not available can never become available while
//!   the caller waits (the holder cannot run), so a take that would have to
//!   wait fails at once;
//! * memory comes from a fixed arena and is never reused;
//! * software timers have no callbacks (nothing would ever run them);
//! * the pool has a fixed number of semaphores.
//!
//! Each request the adapter cannot honour is recorded as a latched
//! [`fault`] bit and counted. The coexistence library does not always
//! check what the adapter returns (`ic_set_extern_coex` ignores the results
//! of `coex_core_enable` and `coex_schm_status_bit_set` and enables the
//! hardware anyway), so the owner of the arbiter reads
//! [`OsAdapter::faults`] and keeps coexistence off once any bit is set
//! (`super::Follower`).
//!
//! What the H2 blob requests (static analysis of `libcoexist.a` SHA-1
//! `d67b0a16d30eb2d876b6a717a84840ea14f18b82`, calls through
//! `g_coa_funcs_p`): `coex_core_pre_init` and `coex_schm_init` (from
//! `coex_pre_init`) create two semaphores and set a timer function;
//! `coex_core_lock` and `coex_schm_lock` take with `portMAX_DELAY`;
//! `coex_schm_change_phase` arms and disarms the phase timer (reachable
//! from `esp_coex_external_set` through `coex_schm_status_bit_set`, when a
//! coexistence scheme allows the phase loop); `malloc_internal` and `free`
//! are not called. Whether the external-coexistence path arms the timer at
//! run time is a hardware question; if it does, PTA stays off with
//! [`fault::TIMER_ARMED`] instead of running without the phase timer.

use core::ffi::c_void;
use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

/// Bits of [`OsAdapter::faults`]: requests the adapter could not honour.
pub mod fault {
    /// `semphr_create` while every semaphore slot is in use.
    pub const SEMAPHORES_EXHAUSTED: u32 = 1 << 0;
    /// `malloc_internal` larger than what is left of the arena.
    pub const ALLOC_FAILED: u32 = 1 << 1;
    /// `semphr_take` with a block time on a semaphore that is not
    /// available: waiting could never end, so the take failed.
    pub const BLOCKING_TAKE: u32 = 1 << 2;
    /// `timer_arm_us`: the timer callback would never run.
    pub const TIMER_ARMED: u32 = 1 << 3;
    /// A semaphore handle or arena pointer the adapter never handed out (or
    /// already deleted), or an impossible semaphore (maximum 0, initial
    /// count above the maximum).
    pub const INVALID_REQUEST: u32 = 1 << 4;
}

/// `portMAX_DELAY` (`OSI_FUNCS_TIME_BLOCKING`): the blob's locks wait
/// forever.
pub const WAIT_FOREVER: u32 = u32::MAX;

/// Semaphore slot state: in use, and the count in the low bits.
const LIVE: u32 = 1 << 31;
const COUNT: u32 = !LIVE;

/// The adapter's state: `SEMAPHORES` counting semaphores and an arena of
/// `ARENA_BYTES` (the memory itself belongs to the caller; this hands out
/// offsets). Every operation is lock-free (one compare-and-swap), so it may
/// run in an interrupt or inside a critical section.
pub struct OsAdapter<const SEMAPHORES: usize, const ARENA_BYTES: usize> {
    /// Per slot: [`LIVE`] | count. A slot's address is its handle.
    semaphores: [AtomicU32; SEMAPHORES],
    max: [AtomicU32; SEMAPHORES],
    created: AtomicU32,
    arena_used: AtomicU32,
    frees: AtomicU32,
    timer_calls: AtomicU32,
    faults: AtomicU32,
    fault_events: AtomicU32,
}

/// A snapshot for the lab ([`OsAdapter::stats`]).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct AdapterStats {
    /// Semaphores currently in use.
    pub semaphores_live: u32,
    /// Semaphores created since boot.
    pub semaphores_created: u32,
    /// Arena bytes handed out (never reused).
    pub arena_used: u32,
    /// `free` calls on arena memory.
    pub frees: u32,
    /// Timer calls of any kind (`setfn`, `arm_us`, `disarm`, `done`).
    pub timer_calls: u32,
    /// Latched [`fault`] bits.
    pub faults: u32,
    /// Requests not honoured since boot or [`OsAdapter::reset_fault_events`].
    pub fault_events: u32,
}

impl<const SEMAPHORES: usize, const ARENA_BYTES: usize> Default
    for OsAdapter<SEMAPHORES, ARENA_BYTES>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const SEMAPHORES: usize, const ARENA_BYTES: usize> OsAdapter<SEMAPHORES, ARENA_BYTES> {
    const ARENA_FITS: () = assert!(ARENA_BYTES <= u32::MAX as usize);

    pub const fn new() -> Self {
        let () = Self::ARENA_FITS;
        Self {
            semaphores: [const { AtomicU32::new(0) }; SEMAPHORES],
            max: [const { AtomicU32::new(0) }; SEMAPHORES],
            created: AtomicU32::new(0),
            arena_used: AtomicU32::new(0),
            frees: AtomicU32::new(0),
            timer_calls: AtomicU32::new(0),
            faults: AtomicU32::new(0),
            fault_events: AtomicU32::new(0),
        }
    }

    /// `semphr_create(max, init)`: a handle, or null (and a fault) when the
    /// pool is full or the request is impossible.
    pub fn semaphore_create(&self, max: u32, init: u32) -> *mut c_void {
        if max == 0 || max > COUNT || init > max {
            self.fault(fault::INVALID_REQUEST);
            return core::ptr::null_mut();
        }
        for (slot, state) in self.semaphores.iter().enumerate() {
            if state
                .compare_exchange(0, LIVE | init, Relaxed, Relaxed)
                .is_ok()
            {
                // The handle is not out yet: nothing gives before this.
                self.max[slot].store(max, Relaxed);
                self.created.fetch_add(1, Relaxed);
                return self.handle(slot);
            }
        }
        self.fault(fault::SEMAPHORES_EXHAUSTED);
        core::ptr::null_mut()
    }

    /// `semphr_delete`: the slot is free again. Null is ignored.
    pub fn semaphore_delete(&self, handle: *mut c_void) {
        if handle.is_null() {
            return;
        }
        match self.slot(handle) {
            Some(slot) if self.semaphores[slot].swap(0, Relaxed) & LIVE != 0 => {}
            _ => self.fault(fault::INVALID_REQUEST),
        }
    }

    /// `semphr_take(handle, block_ticks)` (`_from_isr`: `block_ticks` 0).
    /// An unavailable semaphore fails at once; with a block time that is a
    /// [`fault::BLOCKING_TAKE`], without one an ordinary failed poll.
    pub fn semaphore_take(&self, handle: *mut c_void, block_ticks: u32) -> bool {
        let Some(slot) = self.slot(handle) else {
            self.fault(fault::INVALID_REQUEST);
            return false;
        };
        let taken = update(&self.semaphores[slot], |state| {
            (state & LIVE != 0 && state & COUNT != 0).then(|| state - 1)
        });
        match taken {
            Ok(_) => true,
            Err(state) if state & LIVE == 0 => {
                self.fault(fault::INVALID_REQUEST);
                false
            }
            Err(_) => {
                if block_ticks != 0 {
                    self.fault(fault::BLOCKING_TAKE);
                }
                false
            }
        }
    }

    /// `semphr_give`; false when the count is already at its maximum (as
    /// FreeRTOS, not a fault).
    pub fn semaphore_give(&self, handle: *mut c_void) -> bool {
        let Some(slot) = self.slot(handle) else {
            self.fault(fault::INVALID_REQUEST);
            return false;
        };
        let max = self.max[slot].load(Relaxed);
        let given = update(&self.semaphores[slot], |state| {
            (state & LIVE != 0 && state & COUNT < max).then(|| state + 1)
        });
        match given {
            Ok(_) => true,
            Err(state) if state & LIVE == 0 => {
                self.fault(fault::INVALID_REQUEST);
                false
            }
            Err(_) => false,
        }
    }

    /// `malloc_internal(size)`: the 8-aligned offset of `size` bytes in the
    /// arena, or `None` (and [`fault::ALLOC_FAILED`]) when they do not fit.
    pub fn alloc(&self, size: usize) -> Option<usize> {
        let size = size.max(1);
        let reserved = update(&self.arena_used, |used| {
            let end = (used as usize).checked_add(size)?.checked_add(7)? & !7;
            (end <= ARENA_BYTES).then_some(end as u32)
        });
        match reserved {
            Ok(start) => Some(start as usize),
            Err(_) => {
                self.fault(fault::ALLOC_FAILED);
                None
            }
        }
    }

    /// `free` of arena `offset` (pointer minus arena base; the caller
    /// ignores null). The bytes are not reused; an offset outside what was
    /// handed out is a [`fault::INVALID_REQUEST`].
    pub fn free(&self, offset: usize) {
        if offset < self.arena_used.load(Relaxed) as usize {
            self.frees.fetch_add(1, Relaxed);
        } else {
            self.fault(fault::INVALID_REQUEST);
        }
    }

    /// `timer_setfn`, `timer_disarm`, `timer_done`: harmless while no
    /// timer can be armed; counted.
    pub fn timer_call(&self) {
        self.timer_calls.fetch_add(1, Relaxed);
    }

    /// `timer_arm_us`: cannot be honoured ([`fault::TIMER_ARMED`]).
    pub fn timer_arm(&self) {
        self.timer_call();
        self.fault(fault::TIMER_ARMED);
    }

    /// Latched [`fault`] bits (cleared only by a reboot).
    pub fn faults(&self) -> u32 {
        self.faults.load(Relaxed)
    }

    /// Requests not honoured (wrapping): the `coex_adapter_faults` counter.
    pub fn fault_events(&self) -> u32 {
        self.fault_events.load(Relaxed)
    }

    /// Zero [`Self::fault_events`] (`DIAG_RESET_COUNTERS`); the latched
    /// [`Self::faults`] stay.
    pub fn reset_fault_events(&self) {
        self.fault_events.store(0, Relaxed);
    }

    pub fn stats(&self) -> AdapterStats {
        AdapterStats {
            semaphores_live: self
                .semaphores
                .iter()
                .filter(|state| state.load(Relaxed) & LIVE != 0)
                .count() as u32,
            semaphores_created: self.created.load(Relaxed),
            arena_used: self.arena_used.load(Relaxed),
            frees: self.frees.load(Relaxed),
            timer_calls: self.timer_calls.load(Relaxed),
            faults: self.faults(),
            fault_events: self.fault_events(),
        }
    }

    fn fault(&self, kind: u32) {
        self.faults.fetch_or(kind, Relaxed);
        self.fault_events.fetch_add(1, Relaxed);
    }

    fn handle(&self, slot: usize) -> *mut c_void {
        core::ptr::from_ref(&self.semaphores[slot])
            .cast_mut()
            .cast()
    }

    fn slot(&self, handle: *mut c_void) -> Option<usize> {
        let offset = handle.addr().checked_sub(self.semaphores.as_ptr().addr())?;
        let size = core::mem::size_of::<AtomicU32>();
        let slot = offset / size;
        (offset % size == 0 && slot < SEMAPHORES).then_some(slot)
    }
}

/// `f` applied atomically (a compare-and-swap loop): `Ok(previous)`, or
/// `Err(current)` when `f` refuses. (`AtomicU32::fetch_update` is
/// deprecated on the ESP toolchain, its replacement unstable on the host
/// one.)
fn update(atomic: &AtomicU32, mut f: impl FnMut(u32) -> Option<u32>) -> Result<u32, u32> {
    let mut current = atomic.load(Relaxed);
    loop {
        let Some(new) = f(current) else {
            return Err(current);
        };
        match atomic.compare_exchange_weak(current, new, Relaxed, Relaxed) {
            Ok(previous) => return Ok(previous),
            Err(actual) => current = actual,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::fault::*;
    use super::*;

    type Adapter = OsAdapter<2, 64>;

    #[test]
    fn locks_as_the_blob_uses_them_are_honoured_without_faults() {
        let a = Adapter::new();
        // coex_core_pre_init / coex_schm_init: binary semaphores, given.
        let core = a.semaphore_create(1, 1);
        let schm = a.semaphore_create(1, 1);
        assert!(!core.is_null() && !schm.is_null() && core != schm);
        for _ in 0..3 {
            assert!(a.semaphore_take(core, WAIT_FOREVER));
            assert!(a.semaphore_take(schm, WAIT_FOREVER));
            assert!(a.semaphore_give(schm));
            assert!(a.semaphore_give(core));
        }
        a.timer_call(); // timer_setfn in coex_schm_init
        assert_eq!(a.faults(), 0);
        assert_eq!(a.fault_events(), 0);
        let stats = a.stats();
        assert_eq!((stats.semaphores_live, stats.semaphores_created), (2, 2));
        assert_eq!(stats.timer_calls, 1);
    }

    #[test]
    fn a_take_that_would_wait_fails_and_latches_a_fault() {
        let a = Adapter::new();
        let mutex = a.semaphore_create(1, 1);
        assert!(a.semaphore_take(mutex, WAIT_FOREVER));
        // A poll (no block time, or from an ISR) on a held mutex is no fault.
        assert!(!a.semaphore_take(mutex, 0));
        assert_eq!(a.faults(), 0);
        // Waiting could never end: the holder cannot run.
        assert!(!a.semaphore_take(mutex, WAIT_FOREVER));
        assert!(!a.semaphore_take(mutex, 10));
        assert_eq!(a.faults(), BLOCKING_TAKE);
        assert_eq!(a.fault_events(), 2);
        // Giving it back does not clear the latch.
        assert!(a.semaphore_give(mutex));
        assert!(a.semaphore_take(mutex, WAIT_FOREVER));
        assert_eq!(a.faults(), BLOCKING_TAKE);
    }

    #[test]
    fn counting_semaphores_stay_within_their_maximum() {
        let a = Adapter::new();
        let sem = a.semaphore_create(2, 0);
        assert!(!a.semaphore_take(sem, 0));
        assert!(a.semaphore_give(sem));
        assert!(a.semaphore_give(sem));
        // Full: FreeRTOS errQUEUE_FULL, not a fault.
        assert!(!a.semaphore_give(sem));
        assert!(a.semaphore_take(sem, 0));
        assert!(a.semaphore_take(sem, 0));
        assert!(!a.semaphore_take(sem, 0));
        assert_eq!(a.faults(), 0);
    }

    #[test]
    fn more_semaphores_than_slots_is_a_fault_and_delete_frees_a_slot() {
        let a = Adapter::new();
        let first = a.semaphore_create(1, 1);
        let second = a.semaphore_create(1, 0);
        assert!(!second.is_null());
        assert!(a.semaphore_create(1, 1).is_null());
        assert_eq!(a.faults(), SEMAPHORES_EXHAUSTED);
        a.semaphore_delete(first);
        let third = a.semaphore_create(1, 1);
        assert_eq!(third, first, "the freed slot is reused");
        assert!(a.semaphore_take(third, 0));
        assert_eq!(a.stats().semaphores_live, 2);
        assert_eq!(a.stats().semaphores_created, 3);
        // The latch stays.
        assert_eq!(a.faults(), SEMAPHORES_EXHAUSTED);
    }

    #[test]
    fn unknown_handles_and_impossible_semaphores_are_faults() {
        let a = Adapter::new();
        assert!(a.semaphore_create(0, 0).is_null());
        assert!(a.semaphore_create(1, 2).is_null());
        assert_eq!(a.faults(), INVALID_REQUEST);
        assert_eq!(a.stats().semaphores_live, 0);

        let b = Adapter::new();
        let sem = b.semaphore_create(1, 1);
        let foreign = core::ptr::from_ref(&b.created).cast_mut().cast();
        let misaligned = sem.cast::<u8>().wrapping_add(1).cast();
        let past_the_end = sem.cast::<AtomicU32>().wrapping_add(2).cast();
        for handle in [core::ptr::null_mut(), foreign, misaligned, past_the_end] {
            assert!(!b.semaphore_take(handle, WAIT_FOREVER));
            assert!(!b.semaphore_give(handle));
        }
        b.semaphore_delete(core::ptr::null_mut());
        assert_eq!(b.fault_events(), 8, "null delete is ignored");
        // A deleted semaphore is gone.
        b.semaphore_delete(sem);
        assert!(!b.semaphore_take(sem, 0));
        assert!(!b.semaphore_give(sem));
        b.semaphore_delete(sem);
        assert_eq!(b.faults(), INVALID_REQUEST);
        assert_eq!(b.fault_events(), 11);
    }

    #[test]
    fn the_arena_hands_out_aligned_memory_until_it_is_full() {
        let a = Adapter::new();
        assert_eq!(a.alloc(5), Some(0));
        assert_eq!(a.alloc(0), Some(8));
        assert_eq!(a.alloc(16), Some(16));
        assert_eq!(a.alloc(32), Some(32));
        assert_eq!(a.stats().arena_used, 64);
        assert_eq!(a.faults(), 0);
        assert_eq!(a.alloc(1), None);
        assert_eq!(a.alloc(usize::MAX), None);
        assert_eq!(a.faults(), ALLOC_FAILED);
        assert_eq!(a.fault_events(), 2);
        assert_eq!(a.stats().arena_used, 64);
    }

    #[test]
    fn freed_memory_is_counted_not_reused_and_foreign_frees_are_faults() {
        let a = Adapter::new();
        let offset = a.alloc(24).unwrap();
        a.free(offset);
        a.free(offset + 8);
        assert_eq!(a.alloc(8), Some(24));
        assert_eq!((a.stats().frees, a.faults()), (2, 0));
        a.free(32);
        a.free(usize::MAX);
        assert_eq!(a.faults(), INVALID_REQUEST);
        assert_eq!(a.stats().frees, 2);
    }

    #[test]
    fn arming_a_timer_is_a_fault_other_timer_calls_are_not() {
        let a = Adapter::new();
        a.timer_call();
        a.timer_call();
        assert_eq!(a.faults(), 0);
        a.timer_arm();
        assert_eq!(a.faults(), TIMER_ARMED);
        assert_eq!(a.stats().timer_calls, 3);
    }

    #[test]
    fn resetting_the_counter_keeps_the_latch() {
        let a = Adapter::new();
        a.timer_arm();
        assert!(a.alloc(65).is_none());
        assert_eq!(a.fault_events(), 2);
        a.reset_fault_events();
        assert_eq!(a.fault_events(), 0);
        assert_eq!(a.faults(), TIMER_ARMED | ALLOC_FAILED);
        a.timer_arm();
        assert_eq!(a.fault_events(), 1);
    }
}
