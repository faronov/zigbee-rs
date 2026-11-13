//! Touch key on the TSC (ST "TouchSensing_1touchkey" example, UM2825):
//! group 6 senses the key (IO1 PD10 sampling capacitor, IO2 PD11
//! electrode), group 4 drives the active shield (IO1 PC6 sampling
//! capacitor, IO2 PC7 shield).
//!
//! [`TouchKey`] exposes the raw charge-transfer cycle: discharge, start,
//! read the count. A touch adds capacitance and lowers the count; the
//! baseline and threshold logic belongs to the product.

use stm32wb_hal::embassy_stm32::mode::Blocking;
use stm32wb_hal::embassy_stm32::tsc::{
    self, ChargeTransferPulseCycle, Group, GroupStatus, IOPin, MaxCount, PGPrescalerDivider,
    PinGroupWithRoles, PinGroups, State, Tsc, pin_roles,
};
use stm32wb_hal::embassy_stm32::{Peri, peripherals};

/// Time the sampling capacitors need to discharge before an acquisition
/// (ST TSL `TSLPRM_DELAY_DISCHARGE_ALL`: 1000 µs).
pub const DISCHARGE_US: u32 = 1_000;
/// Count of an acquisition that hit the max-count limit (no electrode or
/// shorted sampling capacitor).
pub const MAX_COUNT: u16 = 8_191;

/// The TSC pins of the touch key and its shield.
pub struct TouchPins {
    pub tsc: Peri<'static, peripherals::TSC>,
    pub key_sample: Peri<'static, peripherals::PD10>,
    pub key: Peri<'static, peripherals::PD11>,
    pub shield_sample: Peri<'static, peripherals::PC6>,
    pub shield: Peri<'static, peripherals::PC7>,
}

/// One charge-transfer acquisition step result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Acquisition {
    /// Still counting.
    Busy,
    /// Charge-transfer count of the key electrode.
    Count(u16),
    /// The count reached [`MAX_COUNT`] (max-count error).
    MaxCount,
}

/// TSC touch key error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TouchError {
    /// The pin groups were rejected by the TSC driver.
    Groups,
}

/// The touch key: TSC with group 6 (key) and group 4 (shield) enabled.
pub struct TouchKey {
    tsc: Tsc<'static, peripherals::TSC, Blocking>,
    key: IOPin,
}

impl TouchKey {
    /// Configure the TSC like the ST example: 2-cycle charge-transfer
    /// pulses, pulse generator HCLK/4, max count 8191, idle IOs driven low.
    pub fn new(pins: TouchPins) -> Result<Self, TouchError> {
        let mut key_group: PinGroupWithRoles<peripherals::TSC, tsc::G6> =
            PinGroupWithRoles::default();
        key_group.set_io1::<pin_roles::Sample>(pins.key_sample);
        let key = key_group.set_io2::<pin_roles::Channel>(pins.key).pin;
        let mut shield_group: PinGroupWithRoles<peripherals::TSC, tsc::G4> =
            PinGroupWithRoles::default();
        shield_group.set_io1::<pin_roles::Sample>(pins.shield_sample);
        shield_group.set_io2::<pin_roles::Shield>(pins.shield);

        let groups = PinGroups {
            g4: Some(shield_group.pin_group),
            g6: Some(key_group.pin_group),
            ..Default::default()
        };
        let config = tsc::Config {
            ct_pulse_high_length: ChargeTransferPulseCycle::_2,
            ct_pulse_low_length: ChargeTransferPulseCycle::_2,
            pulse_generator_prescaler: PGPrescalerDivider::_4,
            max_count_value: MaxCount::_8191,
            ..Default::default()
        };
        let tsc = Tsc::new_blocking(pins.tsc, groups, config).map_err(|_| TouchError::Groups)?;
        Ok(Self { tsc, key })
    }

    /// Ground the sampling capacitors. Wait [`DISCHARGE_US`] before
    /// [`Self::start`].
    pub fn discharge(&mut self) {
        self.tsc.discharge_io(true);
    }

    /// Start one acquisition of the key (the shield is driven alongside).
    pub fn start(&mut self) {
        self.tsc.discharge_io(false);
        // Only fails for a pin outside the configured groups.
        let _ = self.tsc.set_active_channels(&[self.key]);
        self.tsc.start();
    }

    /// Poll the acquisition started by [`Self::start`].
    pub fn poll(&mut self) -> Acquisition {
        match self.tsc.get_state() {
            State::Busy => Acquisition::Busy,
            State::Error => Acquisition::MaxCount,
            State::Ready | State::Reset => {
                if self.tsc.group_get_status(Group::Six) == GroupStatus::Complete {
                    Acquisition::Count(self.tsc.group_get_value(Group::Six))
                } else {
                    Acquisition::Busy
                }
            }
        }
    }
}
