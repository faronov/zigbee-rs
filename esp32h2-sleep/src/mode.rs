//! Request admission: which power domains, wake sources and reject sources a
//! light or deep sleep uses. Pure, so every refusal is host-tested.

use crate::error::SleepError;
use crate::power::{PD_CPU, PD_DEEP, PD_MODEM, PD_RC_FAST, PD_TOP, PD_XTAL, PowerDown};

/// PMU wake/reject source bits (`PMU_SLP_WAKEUP_CNTL2`, IDF
/// `soc/esp32h2/include/soc/pmu_reg.h` / `sleep_modes.c`).
pub(crate) const WAKE_EXT1: u32 = 1 << 1;
pub(crate) const WAKE_GPIO: u32 = 1 << 2;
pub(crate) const WAKE_TIMER: u32 = 1 << 4;
/// Sources IDF lets reject a sleep request that races a pending wake
/// (`RTC_SLEEP_REJECT_MASK`: EXT1, GPIO, timer, BT/802.15.4, UART0/1, USB).
pub(crate) const REJECT_MASK: u32 =
    (1 << 1) | (1 << 2) | (1 << 4) | (1 << 6) | (1 << 7) | (1 << 10);

/// LP IO pads usable as EXT1 wake on H2 (GPIO7..=14 = RTC IO 0..=7).
pub(crate) const EXT1_GPIO_MASK: u32 = 0xFF << 7;

/// How much of the digital system a light sleep powers down.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum SleepMode {
    /// Every digital domain stays powered (clock-gated, 0.6 V retention).
    /// This is the proven mode the vendor esp-hal path implemented.
    #[default]
    Retained,
    /// TOP, CPU and modem domains are powered off; this crate retains their
    /// registers by a CPU copy into RAM. Requires the `top-pd` feature, the
    /// XTAL power-down and timer/EXT1 wake only.
    TopPowerDown,
}

/// EXT1 (LP IO) wake on any selected pad at the given level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ext1Wake {
    /// GPIO number bit mask; only GPIO7..=14 are LP IO pads.
    pub gpio_mask: u32,
    /// Wake on high level (`true`) or low level (`false`).
    pub high: bool,
}

/// One light-sleep request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Request {
    /// LP-timer wake after this many microseconds (always armed).
    pub duration_us: u64,
    /// Oscillators stopped during sleep.
    pub power_down: PowerDown,
    /// Digital GPIO wake (pads armed by the caller); TOP must stay powered.
    pub gpio_wake: bool,
    /// LP IO wake; the only pad wake usable with TOP powered down.
    pub ext1: Option<Ext1Wake>,
    /// Domains powered down.
    pub mode: SleepMode,
}

/// Admitted light sleep.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Plan {
    /// `PMU_SLEEP_PD_*` flags.
    pub pd: u32,
    /// PMU wake enable bits.
    pub wake: u32,
    /// PMU reject enable bits.
    pub reject: u32,
    /// EXT1 RTC-pin selection (`ext_wakeup_sel`, bit n = GPIO 7 + n).
    pub ext1_sel: u8,
    /// EXT1 level (`ext_wakeup_lv`): 1 = high for every selected pin.
    pub ext1_high: bool,
}

impl Plan {
    pub(crate) fn top_pd(&self) -> bool {
        self.pd & PD_TOP != 0
    }
}

/// Whether [`SleepMode::TopPowerDown`] is compiled in.
pub const TOP_PD_SUPPORTED: bool = cfg!(feature = "top-pd");

/// The vendor-equivalent admission of retained light sleep (Step A): only
/// the oscillators may stop, wake is LP timer and/or digital GPIO.
pub(crate) fn validate_retained(
    deep: bool,
    power_down: u32,
    allowed_power_down: u32,
    wake: u32,
) -> Result<(), SleepError> {
    if deep || power_down & !allowed_power_down != 0 {
        return Err(SleepError::UnsupportedPowerConfiguration);
    }
    if wake == 0 || wake & !(WAKE_TIMER | WAKE_GPIO) != 0 {
        return Err(SleepError::UnsupportedWakeSource);
    }
    Ok(())
}

/// H2 deep sleep accepts only the LP timer wake (PMU wake bit 4).
pub(crate) fn validate_deep(wake: u32) -> Result<(), SleepError> {
    if wake != WAKE_TIMER {
        return Err(SleepError::UnsupportedWakeSource);
    }
    Ok(())
}

/// Deep sleep powers down every domain and oscillator and wakes by timer.
pub(crate) const DEEP_PD: u32 = PD_DEEP;

/// Admit a light-sleep request.
pub(crate) fn plan_light(request: &Request, top_pd_supported: bool) -> Result<Plan, SleepError> {
    let osc = request.power_down.flags();
    let mut wake = WAKE_TIMER;
    match request.mode {
        SleepMode::Retained => {
            if request.ext1.is_some() {
                return Err(SleepError::UnsupportedWakeSource);
            }
            if request.gpio_wake {
                wake |= WAKE_GPIO;
            }
            validate_retained(false, osc, PD_XTAL | PD_RC_FAST, wake)?;
            Ok(Plan {
                pd: osc,
                wake,
                reject: wake & REJECT_MASK,
                ext1_sel: 0,
                ext1_high: false,
            })
        }
        SleepMode::TopPowerDown => {
            if !top_pd_supported {
                return Err(SleepError::UnsupportedPowerConfiguration);
            }
            // SYSTIMER and the restored clock tree assume XTAL stops; a TOP
            // power-down with the crystal kept would only waste power.
            if !request.power_down.xtal {
                return Err(SleepError::UnsupportedPowerConfiguration);
            }
            // Digital GPIO wake lives in the powered-down TOP domain.
            if request.gpio_wake {
                return Err(SleepError::UnsupportedWakeSource);
            }
            let (ext1_sel, ext1_high) = match request.ext1 {
                None => (0, false),
                Some(ext1) => {
                    if ext1.gpio_mask == 0 || ext1.gpio_mask & !EXT1_GPIO_MASK != 0 {
                        return Err(SleepError::UnsupportedWakeSource);
                    }
                    wake |= WAKE_EXT1;
                    ((ext1.gpio_mask >> 7) as u8, ext1.high)
                }
            };
            Ok(Plan {
                pd: PD_TOP | PD_MODEM | PD_CPU | osc,
                wake,
                reject: wake & REJECT_MASK,
                ext1_sel,
                ext1_high,
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn entry_rejects_every_power_down_bit_and_unhandled_wake_source() {
        for bit in 0..32 {
            assert_eq!(
                validate_retained(false, 1 << bit, 0, 1 << 4),
                Err(SleepError::UnsupportedPowerConfiguration)
            );
        }
        assert_eq!(
            validate_retained(true, 0, 0, 1 << 4),
            Err(SleepError::UnsupportedPowerConfiguration)
        );
        for wake in [0, 1, 1 << 1, 1 << 6, (1 << 4) | (1 << 10)] {
            assert_eq!(
                validate_retained(false, 0, 0, wake),
                Err(SleepError::UnsupportedWakeSource)
            );
        }
        for wake in [1 << 4, 1 << 2, (1 << 4) | (1 << 2)] {
            assert_eq!(validate_retained(false, 0, 0, wake), Ok(()));
        }
    }

    #[test]
    fn only_the_allowed_oscillators_may_stop() {
        let allowed = PowerDown::OSCILLATORS.flags();
        for flags in [PD_XTAL, PD_RC_FAST, allowed] {
            assert_eq!(validate_retained(false, flags, allowed, 1 << 4), Ok(()));
        }
        // Top, VDD_SDIO (flash), modem, CPU and XTAL32K stay powered.
        for bit in [0, 1, 2, 3, 6, 7, 31] {
            assert_eq!(
                validate_retained(false, (1 << bit) | allowed, allowed, 1 << 4),
                Err(SleepError::UnsupportedPowerConfiguration)
            );
        }
        assert_eq!(
            validate_retained(true, PD_XTAL, allowed, 1 << 4),
            Err(SleepError::UnsupportedPowerConfiguration)
        );
    }

    #[test]
    fn deep_sleep_accepts_only_the_timer_wake() {
        assert_eq!(validate_deep(1 << 4), Ok(()));
        for wake in [0, 1 << 2, (1 << 4) | (1 << 2), 1 << 6] {
            assert_eq!(validate_deep(wake), Err(SleepError::UnsupportedWakeSource));
        }
    }

    fn request(mode: SleepMode) -> Request {
        Request {
            duration_us: 1_000,
            power_down: PowerDown::OSCILLATORS,
            gpio_wake: false,
            ext1: None,
            mode,
        }
    }

    #[test]
    fn retained_plan_matches_the_vendor_wake_and_reject_sets() {
        let plan = plan_light(&request(SleepMode::Retained), true).unwrap();
        assert_eq!((plan.pd, plan.wake, plan.reject), (0x30, 1 << 4, 1 << 4));
        assert!(!plan.top_pd());
        let gpio = Request {
            gpio_wake: true,
            ..request(SleepMode::Retained)
        };
        let plan = plan_light(&gpio, true).unwrap();
        assert_eq!((plan.wake, plan.reject), (0x14, 0x14));
        let none = Request {
            power_down: PowerDown::NONE,
            ..request(SleepMode::Retained)
        };
        assert_eq!(plan_light(&none, false).unwrap().pd, 0);
        let ext1 = Request {
            ext1: Some(Ext1Wake {
                gpio_mask: 1 << 9,
                high: false,
            }),
            ..request(SleepMode::Retained)
        };
        assert_eq!(
            plan_light(&ext1, true),
            Err(SleepError::UnsupportedWakeSource)
        );
    }

    #[test]
    fn top_power_down_requires_the_feature_and_the_xtal_power_down() {
        assert_eq!(
            plan_light(&request(SleepMode::TopPowerDown), false),
            Err(SleepError::UnsupportedPowerConfiguration)
        );
        for power_down in [
            PowerDown::NONE,
            PowerDown {
                xtal: false,
                rc_fast: true,
            },
        ] {
            let req = Request {
                power_down,
                ..request(SleepMode::TopPowerDown)
            };
            assert_eq!(
                plan_light(&req, true),
                Err(SleepError::UnsupportedPowerConfiguration)
            );
        }
        let plan = plan_light(&request(SleepMode::TopPowerDown), true).unwrap();
        assert_eq!(plan.pd, PD_TOP | PD_MODEM | PD_CPU | PD_XTAL | PD_RC_FAST);
        // VDD_SDIO (flash supply) stays on: the flash is not re-initialised.
        assert_eq!(plan.pd & crate::power::PD_VDDSDIO, 0);
        assert!(plan.top_pd());
        assert_eq!((plan.wake, plan.reject), (1 << 4, 1 << 4));
        let xtal_only = Request {
            power_down: PowerDown {
                xtal: true,
                rc_fast: false,
            },
            ..request(SleepMode::TopPowerDown)
        };
        assert_eq!(plan_light(&xtal_only, true).unwrap().pd & PD_RC_FAST, 0);
    }

    #[test]
    fn top_power_down_wakes_only_by_timer_and_lp_io() {
        let gpio = Request {
            gpio_wake: true,
            ..request(SleepMode::TopPowerDown)
        };
        assert_eq!(
            plan_light(&gpio, true),
            Err(SleepError::UnsupportedWakeSource)
        );
        for gpio_mask in [0, 1 << 6, 1 << 15, (1 << 9) | (1 << 2)] {
            let req = Request {
                ext1: Some(Ext1Wake {
                    gpio_mask,
                    high: false,
                }),
                ..request(SleepMode::TopPowerDown)
            };
            assert_eq!(
                plan_light(&req, true),
                Err(SleepError::UnsupportedWakeSource)
            );
        }
        let button = Request {
            ext1: Some(Ext1Wake {
                gpio_mask: 1 << 9,
                high: false,
            }),
            ..request(SleepMode::TopPowerDown)
        };
        let plan = plan_light(&button, true).unwrap();
        assert_eq!((plan.wake, plan.reject), (0x12, 0x12));
        // GPIO9 is RTC IO 2.
        assert_eq!((plan.ext1_sel, plan.ext1_high), (1 << 2, false));
        let all = Request {
            ext1: Some(Ext1Wake {
                gpio_mask: EXT1_GPIO_MASK,
                high: true,
            }),
            ..request(SleepMode::TopPowerDown)
        };
        let plan = plan_light(&all, true).unwrap();
        assert_eq!((plan.ext1_sel, plan.ext1_high), (0xFF, true));
    }

    #[test]
    fn reject_mask_is_the_idf_set() {
        assert_eq!(REJECT_MASK, 0x4D6);
        assert_eq!(DEEP_PD, 0x7F);
        assert_eq!(TOP_PD_SUPPORTED, cfg!(feature = "top-pd"));
    }
}
