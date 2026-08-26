//! Exclusive ownership tokens for PHY62x2 stateful peripherals.

/// Exclusive ADC ownership token.
#[derive(Debug)]
pub struct AdcToken {
    _private: (),
}

/// Exclusive SPIF flash-controller ownership token.
#[derive(Debug)]
pub struct FlashToken {
    _private: (),
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum I2cInstance {
    I2c0,
    I2c1,
}

/// Exclusive ownership token for one DesignWare I2C instance.
#[derive(Debug)]
pub struct I2cToken {
    pub(crate) instance: I2cInstance,
}

/// All stateful PHY62x2 peripherals implemented by this HAL.
///
/// The token bundle may be acquired only once. Board crates split it into
/// physically wired resources; products then consume only the resources they
/// select.
pub struct Peripherals {
    pub adc: AdcToken,
    pub flash: FlashToken,
    pub i2c0: I2cToken,
    pub i2c1: I2cToken,
}

static mut TAKEN: bool = false;

impl Peripherals {
    /// Acquire the HAL peripheral tokens exactly once.
    pub fn take() -> Option<Self> {
        cortex_m::interrupt::free(|_| {
            // SAFETY: PRIMASK excludes every other accessor while the flag is
            // checked and updated. The tokens are never recreated or cloned.
            unsafe {
                if TAKEN {
                    None
                } else {
                    TAKEN = true;
                    Some(Self {
                        adc: AdcToken { _private: () },
                        flash: FlashToken { _private: () },
                        i2c0: I2cToken {
                            instance: I2cInstance::I2c0,
                        },
                        i2c1: I2cToken {
                            instance: I2cInstance::I2c1,
                        },
                    })
                }
            }
        })
    }
}
