//! Always-on TLSR8258 join/relay router application.
//!
//! Child admission is not implemented; see the package README for the exact
//! capability boundary.

use core::mem::MaybeUninit;

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::{MacError, telink::TelinkMac};
use zigbee_nwk::DeviceType;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::event_loop::{StackEvent, StartError};
use zigbee_runtime::power::PowerMode;
use zigbee_zcl::clusters::basic::PowerSource;
use zigbee_zcl::{ClusterId, DeviceId};

use tlsr8258_tb04::{leds as board, storage};

// Distinct from the sensor runtime's `DEVICE_EUI_OFFSET` (0x33) so a router
// and a sensor built from the same factory-programmed part never collide on
// IEEE address if someone reflashes one board with both images over time.
const DEVICE_EUI_OFFSET: u8 = 0x52; // 'R' for Router

fn failure() -> ! {
    board::LED_GREEN.write(false);
    board::LED_BLUE.write(false);
    board::LED_RED.write(true);
    loop {
        tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(1_000));
    }
}

pub fn run() -> ! {
    type Device = ZigbeeDevice<TelinkMac>;

    if board::configure_status_leds().is_err() {
        failure();
    }

    let mut ieee_address = [0u8; 8];
    tlsr8258_hal::flash::factory_ieee(&mut ieee_address);
    ieee_address[0] = ieee_address[0].wrapping_add(DEVICE_EUI_OFFSET);
    let mac = TelinkMac::with_extended_address(ieee_address);

    static mut DEVICE_STORAGE: MaybeUninit<Device> = MaybeUninit::uninit();

    let device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::Router)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer("Zigbee-RS")
        .model("TLSR8258-Router")
        .date_code("20260718")
        .sw_build("0.1.0")
        .power_source(PowerSource::MainsSinglePhase)
        .channels(zigbee_types::ChannelMask(1 << 15))
        .endpoint(
            1,
            PROFILE_HOME_AUTOMATION,
            DeviceId::RANGE_EXTENDER,
            |endpoint| {
                endpoint
                    .cluster_server(ClusterId::BASIC)
                    .cluster_server(ClusterId::IDENTIFY)
            },
        )
        .build_into(unsafe { &mut *core::ptr::addr_of_mut!(DEVICE_STORAGE) });

    let mut security_store = storage::security_store();
    if device
        .reset_security_state_if_identity_changed(&mut security_store)
        .is_err()
    {
        failure();
    }

    'commission: loop {
        let mut attempts = 0u8;
        loop {
            attempts = attempts.saturating_add(1);
            match tlsr8258_rt::block_on(
                device.start_or_resume_with_security_store(&mut security_store),
            ) {
                Ok(_) => break,
                Err(StartError::CommissioningFailed(_)) if attempts < 10 => {
                    tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(5_000));
                }
                Err(_) => failure(),
            }
        }

        // Solid green = joined and relaying. Unlike the sensor runtime,
        // there is no battery/sensor state to report — this LED state is
        // the entire "am I alive and joined" signal for the router.
        board::LED_RED.write(false);
        board::LED_GREEN.write(true);
        board::LED_BLUE.write(false);

        let mut identify_elapsed = 0u32;
        let one_second = tlsr8258_hal::timer::ms(1_000);
        let mut tick_anchor = tlsr8258_hal::timer::now_ticks();

        loop {
            // Continuous RX: block (with an internal MAC-level timeout) for
            // the next inbound frame, then relay/process it through the
            // full stack. `device.receive()` never sleeps the radio, which
            // is required for a rx_on_when_idle router — see
            // `TelinkMac::mlme_start` for where that PIB state is set.
            match tlsr8258_rt::block_on(device.receive()) {
                Ok(indication) => {
                    let event = tlsr8258_rt::block_on(device.process_incoming_with_security_store(
                        &indication,
                        &mut [],
                        &mut security_store,
                    ));
                    match event {
                        Ok(Some(StackEvent::RejoinRequested)) => {
                            let _ = tlsr8258_rt::block_on(
                                device.secure_rejoin_with_security_store(&mut security_store),
                            );
                        }
                        Ok(Some(StackEvent::LeaveRequested)) => {
                            if tlsr8258_rt::block_on(
                                device.factory_reset_with_security_store(&mut security_store),
                            )
                            .is_err()
                            {
                                failure();
                            }
                            board::LED_GREEN.write(false);
                            board::LED_RED.write(true);
                            continue 'commission;
                        }
                        Ok(_) => {}
                        Err(_) => failure(),
                    }

                    if tlsr8258_rt::block_on(device.tick_with_security_store(
                        0,
                        &mut [],
                        &mut security_store,
                    ))
                    .is_err()
                    {
                        failure();
                    }
                }
                Err(MacError::NoData) => {}
                Err(_) => failure(),
            }

            let now = tlsr8258_hal::timer::now_ticks();
            let elapsed = now.wrapping_sub(tick_anchor);
            if elapsed >= one_second {
                let elapsed_secs = (elapsed / one_second).min(u16::MAX as u32) as u16;
                tick_anchor = tick_anchor.wrapping_add(u32::from(elapsed_secs) * one_second);
                identify_elapsed = identify_elapsed.wrapping_add(u32::from(elapsed_secs));
                if tlsr8258_rt::block_on(device.tick_with_security_store(
                    elapsed_secs,
                    &mut [],
                    &mut security_store,
                ))
                .is_err()
                {
                    failure();
                }
                if device.is_identifying(1) {
                    board::LED_BLUE.write((identify_elapsed & 1) == 0);
                } else {
                    board::LED_BLUE.write(false);
                }
            }
        }
    }
}
