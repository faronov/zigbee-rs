//! XT-ZB1 composition adapter for the reusable BL702 HAL.

use core::cell::UnsafeCell;

use bl702_hal::efuse::EfuseReader;
use bl702_hal::timer::Monotonic;
use bl702_hal::uart::{Uart0Tx, WriteError};
use bl702_xt_zb1::Resources;

pub use bl702_xt_zb1::ApplicationResources;

struct Global<T> {
    value: UnsafeCell<Option<T>>,
    borrowed: UnsafeCell<bool>,
}

// SAFETY: All access is serialized with the BL702 single-hart critical
// section. Values are initialized once before interrupts are enabled.
unsafe impl<T> Sync for Global<T> {}

impl<T> Global<T> {
    const fn empty() -> Self {
        Self {
            value: UnsafeCell::new(None),
            borrowed: UnsafeCell::new(false),
        }
    }

    fn set(&self, value: T) -> Result<(), T> {
        riscv::interrupt::free(|| {
            // SAFETY: The single-hart critical section excludes all other
            // accesses to this cell.
            let slot = unsafe { &mut *self.value.get() };
            if slot.is_some() {
                Err(value)
            } else {
                *slot = Some(value);
                Ok(())
            }
        })
    }

    fn with<R>(&self, operation: impl FnOnce(&T) -> R) -> Option<R> {
        riscv::interrupt::free(|| {
            // SAFETY: The single-hart critical section excludes mutation.
            if unsafe { *self.borrowed.get() } {
                None
            } else {
                unsafe { (&*self.value.get()).as_ref().map(operation) }
            }
        })
    }

    fn with_mut<R>(&self, operation: impl FnOnce(&mut T) -> R) -> Option<R> {
        let pointer = riscv::interrupt::free(|| {
            // SAFETY: The single-hart critical section serializes the borrow
            // flag and value access.
            unsafe {
                if *self.borrowed.get() {
                    None
                } else {
                    let pointer = (&mut *self.value.get())
                        .as_mut()
                        .map(|value| value as *mut T);
                    if pointer.is_some() {
                        *self.borrowed.get() = true;
                    }
                    pointer
                }
            }
        })?;

        // SAFETY: The borrow flag excludes every other access while the
        // operation runs, but interrupts remain enabled during bounded UART
        // polling.
        let result = operation(unsafe { &mut *pointer });
        riscv::interrupt::free(|| {
            // SAFETY: This is the matching release for the borrow above.
            unsafe { *self.borrowed.get() = false };
        });
        Some(result)
    }
}

static UART: Global<Uart0Tx<14, 15>> = Global::empty();
static TIMER: Global<Monotonic> = Global::empty();
static CHIP_ID: Global<[u8; 8]> = Global::empty();

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitError {
    ResourcesAlreadyTaken,
    Uart,
    AlreadyInitialized,
}

pub fn init() -> Result<ApplicationResources, InitError> {
    let resources = Resources::take().ok_or(InitError::ResourcesAlreadyTaken)?;
    let (runtime, application) = resources.split();
    let clocks = runtime.clocks;
    let uart = runtime
        .monitor
        .into_uart(clocks)
        .map_err(|_| InitError::Uart)?;
    let timer = Monotonic::new_1mhz(runtime.timer0, clocks);
    let chip_id = EfuseReader::new(runtime.efuse).chip_id();

    UART.set(uart).map_err(|_| InitError::AlreadyInitialized)?;
    TIMER
        .set(timer)
        .map_err(|_| InitError::AlreadyInitialized)?;
    CHIP_ID
        .set(chip_id)
        .map_err(|_| InitError::AlreadyInitialized)?;
    Ok(application)
}

pub fn delay_us(duration_us: u32) {
    bl702_hal::timer::delay_us(duration_us);
}

pub fn timer_ticks() -> u64 {
    TIMER
        .with_mut(Monotonic::ticks)
        .unwrap_or_else(|| panic!("BL702 timer is unavailable"))
}

pub fn chip_id() -> [u8; 8] {
    CHIP_ID.with(|id| *id).unwrap_or([0; 8])
}

pub fn uart_write(byte: u8) -> Result<(), WriteError> {
    UART.with_mut(|uart| uart.write_byte(byte))
        .unwrap_or(Err(WriteError::Timeout))
}
