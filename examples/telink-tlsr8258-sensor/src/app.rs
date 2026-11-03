//! Composition of the TB-04 sensor: the product profile and policy, the
//! shared sleepy-sensor lifecycle and the TLSR8258 adapters.

use core::mem::MaybeUninit;

use sensor_sed_app::{FixedBattery, NoOta, NoUserAction, SensorSedParts};
use tlsr8258_sensor_app::{TelinkNoDiagnostics, TelinkSupervisor, TelinkWake};
use tlsr8258_tb04::leds::StatusLeds;
use tlsr8258_tb04_product::boot::failure;
use tlsr8258_tb04_product::sensor::{
    DATE_CODE, MANUFACTURER, MODEL, SENSOR_POLICY, SW_BUILD, SensorProfile, SensorRgbStatus,
    SyntheticEnvironment, USER_ACTIONS, fixed_battery,
};
use zigbee_mac::telink::TelinkMac;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_zcl::clusters::basic::PowerSource;

pub type Device = ZigbeeDevice<TelinkMac>;
pub type Parts = SensorSedParts<
    TelinkWake,
    SensorRgbStatus,
    SyntheticEnvironment,
    FixedBattery,
    NoOta,
    NoUserAction,
    TelinkSupervisor,
    TelinkNoDiagnostics,
>;

pub fn parts(leds: StatusLeds) -> Parts {
    SensorSedParts {
        wake: TelinkWake,
        status: SensorRgbStatus::new(leds),
        environment: SyntheticEnvironment::new(),
        battery: fixed_battery(),
        ota: NoOta,
        actions: USER_ACTIONS,
        supervisor: TelinkSupervisor,
        diagnostics: TelinkNoDiagnostics,
    }
}

/// Build the MAC and the sensor device into `slot`.
///
/// Kept out of the composition roots so the MAC, its constructor `Result`
/// and the device builder are temporaries of this short-lived frame. The
/// caller's frame stays live under the whole `block_on` call tree.
#[inline(never)]
pub fn build_device(
    ieee_address: zigbee_types::IeeeAddress,
    aes: tlsr8258_hal::peripherals::Aes,
    leds: &StatusLeds,
    profile: &mut SensorProfile,
    slot: &'static mut MaybeUninit<Device>,
) -> &'static mut Device {
    let Ok(mut mac) = TelinkMac::with_extended_address(ieee_address) else {
        failure(leds);
    };
    if mac.install_aes_engine(aes).is_err() {
        failure(leds);
    }
    ZigbeeDevice::builder(mac)
        .power_mode(SENSOR_POLICY.power_mode())
        // SensorApp is the sole owner of all four-round parent poll windows.
        .automatic_polling(false)
        .manufacturer(MANUFACTURER)
        .model(MODEL)
        .date_code(DATE_CODE)
        .sw_build(SW_BUILD)
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask(1 << 15))
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_into(slot)
}
