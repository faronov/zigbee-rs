//! Exclusive BL702 peripheral ownership.
//!
//! BL702 is a single-hart RV32IMC device and has no RV32A extension. Target
//! singleton acquisition therefore uses a single-hart critical section rather
//! than atomic read-modify-write instructions.

use crate::gpio::Pins;

macro_rules! token {
    ($name:ident) => {
        #[derive(Debug)]
        pub struct $name {
            _private: (),
        }

        impl $name {
            const fn new() -> Self {
                Self { _private: () }
            }
        }
    };
}

token!(Adc);
token!(Aes);
token!(Efuse);
token!(Flash);
token!(I2c0);
token!(Power);
token!(Pwm);
token!(Spi0);
token!(Timer0);
token!(Uart0);
token!(Uart1);

/// All uniquely owned chip resources.
pub struct Peripherals {
    pub adc: Adc,
    /// SEC_ENG AES-128 accelerator (see [`crate::aes`]).
    pub aes: Aes,
    pub efuse: Efuse,
    pub flash: Flash,
    pub i2c0: I2c0,
    pub pins: Pins,
    pub power: Power,
    pub pwm: Pwm,
    pub spi0: Spi0,
    pub timer0: Timer0,
    pub uart0: Uart0,
    pub uart1: Uart1,
}

#[cfg(target_arch = "riscv32")]
static mut TAKEN: bool = false;

#[cfg(not(target_arch = "riscv32"))]
static TAKEN: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

impl Peripherals {
    /// Acquire all chip peripherals once.
    pub fn take() -> Option<Self> {
        #[cfg(target_arch = "riscv32")]
        {
            riscv::interrupt::free(|| {
                // SAFETY: Access is serialized by the single-hart critical
                // section. No RV32A atomic instructions are required.
                unsafe {
                    if TAKEN {
                        None
                    } else {
                        TAKEN = true;
                        Some(Self::steal())
                    }
                }
            })
        }

        #[cfg(not(target_arch = "riscv32"))]
        {
            use core::sync::atomic::Ordering;
            TAKEN
                .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                .ok()
                .map(|_| {
                    // SAFETY: The compare-exchange above established unique
                    // ownership for host checks and tests.
                    unsafe { Self::steal() }
                })
        }
    }

    /// Construct peripheral tokens without checking singleton ownership.
    ///
    /// # Safety
    ///
    /// The caller must ensure that no other token or driver accesses the same
    /// BL702 peripherals or pins.
    unsafe fn steal() -> Self {
        Self {
            adc: Adc::new(),
            aes: Aes::new(),
            efuse: Efuse::new(),
            flash: Flash::new(),
            i2c0: I2c0::new(),
            pins: Pins::new(),
            power: Power::new(),
            pwm: Pwm::new(),
            spi0: Spi0::new(),
            timer0: Timer0::new(),
            uart0: Uart0::new(),
            uart1: Uart1::new(),
        }
    }
}
