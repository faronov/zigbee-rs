//! Hardware semaphores shared with CPU2 (RM0434 §6, AN5289 "semaphores").
//!
//! Semaphore numbers follow ST's wireless firmware convention
//! (`hw_conf.h`); the coprocessor firmware takes the same ones.

use embassy_stm32::pac;

/// RNG ownership (CPU1 and CPU2 both draw from the one RNG).
pub const RNG: u8 = 0;
/// Flash controller ownership for program and erase.
pub const FLASH: u8 = 2;
/// Held by CPU2 while it must not be disturbed by a CPU1 flash operation;
/// CPU1 takes it around each single program/erase step.
pub const BLOCK_FLASH_REQ_BY_CPU2: u8 = 7;

/// CPU1 (Cortex-M4) HSEM core id.
const CPU1_COREID: u8 = 4;

/// Enable the HSEM clock. Idempotent.
pub fn enable() {
    pac::RCC.ahb3enr().modify(|w| w.set_hsemen(true));
}

/// One-step lock (read of `HSEM_RLRx`). Returns `true` if CPU1 now holds
/// the semaphore with process id 0.
pub fn try_lock(semaphore: u8) -> bool {
    let reg = pac::HSEM.rlr(semaphore as usize).read();
    reg.lock() && reg.coreid() == CPU1_COREID && reg.procid() == 0
}

/// Release a semaphore taken with [`try_lock`].
pub fn unlock(semaphore: u8) {
    pac::HSEM.r(semaphore as usize).write(|w| {
        w.set_procid(0);
        w.set_coreid(CPU1_COREID);
        w.set_lock(false);
    });
}

/// RAII ownership of one semaphore.
pub struct Guard {
    semaphore: u8,
}

impl Guard {
    /// Spin on [`try_lock`] for at most `attempts` tries.
    pub fn acquire(semaphore: u8, attempts: u32) -> Option<Self> {
        for _ in 0..attempts {
            if try_lock(semaphore) {
                return Some(Self { semaphore });
            }
            core::hint::spin_loop();
        }
        None
    }
}

impl Drop for Guard {
    fn drop(&mut self) {
        unlock(self.semaphore);
    }
}
