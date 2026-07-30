//! Exclusive ownership tokens for singleton TLSR8258 peripherals.

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

token!(Pwm);
token!(SerialController);

macro_rules! pins {
    ($($field:ident: ($port:ident, $bit:literal)),+ $(,)?) => {
        /// All GPIO pads as uniquely owned tokens.
        pub struct Pins {
            $(pub $field: crate::gpio::Pin,)+
        }

        impl Pins {
            const fn new() -> Self {
                Self {
                    $($field: crate::gpio::Pin::new(crate::gpio::Port::$port, $bit),)+
                }
            }
        }
    };
}

pins!(
    pa0: (A, 0), pa1: (A, 1), pa2: (A, 2), pa3: (A, 3),
    pa4: (A, 4), pa5: (A, 5), pa6: (A, 6), pa7: (A, 7),
    pb0: (B, 0), pb1: (B, 1), pb2: (B, 2), pb3: (B, 3),
    pb4: (B, 4), pb5: (B, 5), pb6: (B, 6), pb7: (B, 7),
    pc0: (C, 0), pc1: (C, 1), pc2: (C, 2), pc3: (C, 3),
    pc4: (C, 4), pc5: (C, 5), pc6: (C, 6), pc7: (C, 7),
    pd0: (D, 0), pd1: (D, 1), pd2: (D, 2), pd3: (D, 3),
    pd4: (D, 4), pd5: (D, 5), pd6: (D, 6), pd7: (D, 7),
    pe0: (E, 0), pe1: (E, 1), pe2: (E, 2), pe3: (E, 3),
    pe4: (E, 4), pe5: (E, 5), pe6: (E, 6), pe7: (E, 7),
);

/// Uniquely owned chip peripherals with stateful safe driver constructors.
pub struct Peripherals {
    /// TLSR8258's mutually exclusive I2C/SPI controller.
    pub serial: SerialController,
    pub pwm: Pwm,
    pub pins: Pins,
}

#[cfg(target_arch = "tc32")]
static mut TAKEN: bool = false;

#[cfg(not(target_arch = "tc32"))]
static TAKEN: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

impl Peripherals {
    /// Acquire the singleton peripheral tokens once.
    pub fn take() -> Option<Self> {
        #[cfg(target_arch = "tc32")]
        {
            // TC32 is single-core and lacks atomic read-modify-write
            // instructions, so serialize acquisition by masking interrupts.
            let previous_irq = unsafe { crate::mmio::r8(crate::mmio::REG_IRQ_EN) };
            unsafe { crate::mmio::w8(crate::mmio::REG_IRQ_EN, 0) };
            let taken = core::ptr::addr_of_mut!(TAKEN);
            let was_taken = unsafe { core::ptr::read_volatile(taken) };
            if !was_taken {
                unsafe { core::ptr::write_volatile(taken, true) };
            }
            unsafe { crate::mmio::w8(crate::mmio::REG_IRQ_EN, previous_irq) };

            if was_taken {
                None
            } else {
                // SAFETY: The interrupt-masked check established unique
                // ownership on this single-core target.
                Some(unsafe { Self::steal() })
            }
        }

        #[cfg(not(target_arch = "tc32"))]
        {
            use core::sync::atomic::Ordering;
            TAKEN
                .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                .ok()
                .map(|_| {
                    // SAFETY: The successful compare-exchange established
                    // unique ownership for host checks and tests.
                    unsafe { Self::steal() }
                })
        }
    }

    /// Construct tokens after singleton ownership has been established.
    ///
    /// # Safety
    ///
    /// No other token or driver may access these peripheral instances.
    unsafe fn steal() -> Self {
        Self {
            serial: SerialController::new(),
            pwm: Pwm::new(),
            pins: Pins::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::size_of;

    #[test]
    fn peripherals_are_single_take_and_zero_sized() {
        assert!(Peripherals::take().is_some());
        assert!(Peripherals::take().is_none());
        assert_eq!(size_of::<SerialController>(), 0);
        assert_eq!(size_of::<Pwm>(), 0);
        assert!(size_of::<Pins>() > 0);
    }
}
