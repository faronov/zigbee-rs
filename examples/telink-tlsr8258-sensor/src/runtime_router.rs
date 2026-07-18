//! Production TLSR8258 Zigbee router application — EXPERIMENTAL,
//! join/relay-only.
//!
//! This mirrors `runtime_sensor.rs` for the join/commissioning, security
//! journal, and reset-resume plumbing, which is proven end-device behavior
//! shared unchanged with this router build. The only new piece is the
//! device role: `DeviceType::Router` + `PowerMode::AlwaysOn` with a
//! continuous-receive main loop instead of sleepy polling.
//!
//! # Scope (read this before treating this as a real router)
//!
//! What this firmware DOES:
//! - Joins an existing Zigbee network with the router capability bit set.
//! - Calls `zigbee_nwk`'s `Nlme::nlme_start_router()` after joining, which
//!   drives `zigbee_mac::telink::TelinkMac::mlme_start` into non-beacon
//!   (BO=SO=15), non-PAN-coordinator, continuous-RX mode.
//! - Relays unicast NWK frames not addressed to itself and rebroadcasts
//!   broadcast/route-request traffic (existing `zigbee-nwk` forwarding —
//!   unchanged by this example).
//! - Sends periodic NWK Link Status broadcasts (existing generic
//!   `zigbee-nwk` behavior, not router-specific code added here).
//! - Persists join/security state across resets via the same flash journal
//!   used by the sensor runtime, and secure-rejoins on request.
//!
//! What this firmware explicitly DOES NOT do (and must not silently grow):
//! - **No child association.** `TelinkMac` never implements
//!   `MLME-ASSOCIATE.response`; nothing here fakes it.
//! - **No beacons.** `macBeaconOrder`/`macSuperframeOrder` are fixed at 15
//!   (non-beacon); this device never transmits an 802.15.4 beacon.
//! - **No permit-joining.** `macAssociationPermit` is never set to `true`.
//! - **No indirect transmission / pending-frame queue** for sleepy
//!   children — `TelinkMac::mcps_data` rejects `tx_options.indirect`.
//!
//! This is the direct TLSR8258 analogue of `examples/nrf52840-router`: an
//! always-on FFD join/relay target, not a complete Zigbee router.

use core::mem::MaybeUninit;

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::telink::TelinkMac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, StartError};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::security_journal::{SecurityJournalStorage, SecurityStateJournal};
use zigbee_runtime::security_store::SecurityStoreError;
use zigbee_runtime::{ClusterRef, ZigbeeDevice};
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::identify::IdentifyCluster;

use crate::{board, executor};

const SECURITY_SECTOR_A: u32 = 0x0007_4000;
const SECURITY_SECTOR_B: u32 = 0x0007_5000;

// Distinct from the sensor runtime's `DEVICE_EUI_OFFSET` (0x33) so a router
// and a sensor built from the same factory-programmed part never collide on
// IEEE address if someone reflashes one board with both images over time.
const DEVICE_EUI_OFFSET: u8 = 0x52; // 'R' for Router

/// Router maintenance tick period (link status, route table aging, etc. —
/// all handled generically by `zigbee-nwk`/`zigbee-runtime`; this constant
/// only paces how often we call `tick_with_security_store`).
const MAINTENANCE_TICK_SECS_MAX: u16 = 60;

struct Tlsr8258SecurityFlash;

impl SecurityJournalStorage for Tlsr8258SecurityFlash {
    fn read(&self, address: u32, output: &mut [u8]) -> Result<(), SecurityStoreError> {
        if tlsr8258_hal::flash::read_bytes(address, output) {
            Ok(())
        } else {
            Err(SecurityStoreError::Hardware)
        }
    }

    fn program(&mut self, address: u32, data: &[u8]) -> Result<(), SecurityStoreError> {
        tlsr8258_hal::flash::program(address, data).map_err(|_| SecurityStoreError::Hardware)
    }

    fn erase_sector(&mut self, address: u32) -> Result<(), SecurityStoreError> {
        tlsr8258_hal::flash::erase_sector(address).map_err(|_| SecurityStoreError::Hardware)
    }
}

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
    static mut BASIC_STORAGE: MaybeUninit<BasicCluster> = MaybeUninit::uninit();
    static mut IDENTIFY_STORAGE: MaybeUninit<IdentifyCluster> = MaybeUninit::uninit();

    let basic_cluster = unsafe {
        let ptr = core::ptr::addr_of_mut!(BASIC_STORAGE).cast::<BasicCluster>();
        ptr.write(BasicCluster::new(
            b"Zigbee-RS",
            b"TLSR8258-Router",
            b"20260718",
            b"0.1.0",
        ));
        &mut *ptr
    };
    basic_cluster.set_power_source(0x01); // Mains powered — router never sleeps

    let identify_cluster = unsafe {
        let ptr = core::ptr::addr_of_mut!(IDENTIFY_STORAGE).cast::<IdentifyCluster>();
        ptr.write(IdentifyCluster::new());
        &mut *ptr
    };

    let device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::Router)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer("Zigbee-RS")
        .model("TLSR8258-Router")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask(1 << 15))
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0007, |endpoint| {
            // 0x0007 = Home Gateway, matching examples/nrf52840-router.
            endpoint.cluster_server(0x0000).cluster_server(0x0003)
        })
        .build_into(unsafe { &mut *core::ptr::addr_of_mut!(DEVICE_STORAGE) });

    let mut security_store = SecurityStateJournal::new(
        Tlsr8258SecurityFlash,
        SECURITY_SECTOR_A,
        SECURITY_SECTOR_B,
    );
    if crate::security_identity::prepare(device, &mut security_store, ieee_address).is_err() {
        failure();
    }

    'commission: loop {
        let mut attempts = 0u8;
        loop {
            attempts = attempts.saturating_add(1);
            match executor::block_on(
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

        let mut maintenance_elapsed = 0u16;
        let one_second = tlsr8258_hal::timer::ms(1_000);
        let mut tick_anchor = tlsr8258_hal::timer::now_ticks();

        loop {
            // Continuous RX: block (with an internal MAC-level timeout) for
            // the next inbound frame, then relay/process it through the
            // full stack. `device.receive()` never sleeps the radio, which
            // is required for a rx_on_when_idle router — see
            // `TelinkMac::mlme_start` for where that PIB state is set.
            //
            // `clusters` is rebuilt (cheaply — two `ClusterRef`s) on every
            // use rather than held across the loop, so `identify_cluster`
            // remains individually accessible for its own `tick()` below
            // without a persistent borrow conflict.
            match executor::block_on(device.receive()) {
                Ok(indication) => {
                    let mut clusters = [
                        ClusterRef {
                            endpoint: 1,
                            cluster: basic_cluster,
                        },
                        ClusterRef {
                            endpoint: 1,
                            cluster: identify_cluster,
                        },
                    ];
                    let event = executor::block_on(device.process_incoming_with_security_store(
                        &indication,
                        &mut clusters,
                        &mut security_store,
                    ));
                    match event {
                        Ok(Some(StackEvent::RejoinRequested)) => {
                            let _ = executor::block_on(
                                device.secure_rejoin_with_security_store(&mut security_store),
                            );
                        }
                        Ok(Some(StackEvent::LeaveRequested)) => {
                            if executor::block_on(
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

                    let mut clusters2 = [
                        ClusterRef {
                            endpoint: 1,
                            cluster: basic_cluster,
                        },
                        ClusterRef {
                            endpoint: 1,
                            cluster: identify_cluster,
                        },
                    ];
                    if executor::block_on(device.tick_with_security_store(
                        0,
                        &mut clusters2,
                        &mut security_store,
                    ))
                    .is_err()
                    {
                        failure();
                    }

                    identify_cluster.tick(0);
                }
                Err(_) => {
                    // MAC-level RX timeout or transient radio error — fall
                    // through to the maintenance tick below rather than
                    // treating this as fatal. A router must keep listening.
                }
            }

            let now = tlsr8258_hal::timer::now_ticks();
            let elapsed = now.wrapping_sub(tick_anchor);
            if elapsed >= one_second {
                let elapsed_secs = (elapsed / one_second).min(u16::MAX as u32) as u16;
                tick_anchor = tick_anchor.wrapping_add(u32::from(elapsed_secs) * one_second);
                maintenance_elapsed =
                    maintenance_elapsed.saturating_add(elapsed_secs).min(MAINTENANCE_TICK_SECS_MAX);
                let mut clusters = [
                    ClusterRef {
                        endpoint: 1,
                        cluster: basic_cluster,
                    },
                    ClusterRef {
                        endpoint: 1,
                        cluster: identify_cluster,
                    },
                ];
                if executor::block_on(device.tick_with_security_store(
                    elapsed_secs,
                    &mut clusters,
                    &mut security_store,
                ))
                .is_err()
                {
                    failure();
                }
                identify_cluster.tick(elapsed_secs);
                if identify_cluster.is_identifying() {
                    board::LED_BLUE.write((maintenance_elapsed & 1) == 0);
                } else {
                    board::LED_BLUE.write(false);
                }
            }
        }
    }
}
