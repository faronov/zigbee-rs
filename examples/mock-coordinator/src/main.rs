//! Finite host composition of a persisted Zigbee coordinator.

use router_app::{
    CoordinatorApp, NoDiagnostics, NoStatus, NoSupervisor, PersistentChildren, RouterParts,
    RouterPolicy,
};
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::EdValue;
use zigbee_mac::mock::MockMac;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::child_store::RamChildTableStore;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::{ApplicationProfile, DeviceProfile, RangeExtender};
use zigbee_runtime::role::Router;
use zigbee_runtime::security_store::{RamSecurityStateStore, SecurityStateStore};
use zigbee_types::ChannelMask;
use zigbee_zcl::DeviceId;
use zigbee_zcl::clusters::basic::PowerSource;

const COORDINATOR_IEEE: [u8; 8] = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77];

static POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 1_000,
    join_retry_initial_ms: 10,
    join_retry_max_ms: 100,
    secure_rejoin_failure_limit: 2,
};

type CoordinatorProfile = DeviceProfile<RangeExtender>;

fn coordinator_profile() -> CoordinatorProfile {
    DeviceProfile::new(
        1,
        PROFILE_HOME_AUTOMATION,
        DeviceId::COMBINED_INTERFACE,
        RangeExtender,
    )
}

fn coordinator_device(
    profile: &CoordinatorProfile,
    include_energy_scan: bool,
) -> ZigbeeDevice<MockMac, Router> {
    let mut mac = MockMac::new(COORDINATOR_IEEE);
    mac.set_rx_delay_us(u32::MAX);
    if include_energy_scan {
        mac.add_energy(EdValue {
            channel: 11,
            energy: 90,
        });
        mac.add_energy(EdValue {
            channel: 15,
            energy: 20,
        });
    }
    ZigbeeDevice::builder(mac)
        .manufacturer("Zigbee-RS")
        .model("MockCoordinator-01")
        .power_source(PowerSource::MainsSinglePhase)
        .power_mode(PowerMode::AlwaysOn)
        .channels(ChannelMask((1 << 11) | (1 << 15)))
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_coordinator()
}

fn live_network_key(device: &ZigbeeDevice<MockMac, Router>) -> [u8; 16] {
    device
        .bdb()
        .zdo()
        .nwk()
        .security()
        .active_key()
        .expect("coordinator has an active network key")
        .key
}

fn main() {
    pollster::block_on(async {
        println!("zigbee-rs finite CoordinatorApp persistence demo");

        let mut security_store = RamSecurityStateStore::new();
        let (pan_id, channel, network_key) = {
            let mut profile = coordinator_profile();
            let mut device = coordinator_device(&profile, true);
            let node = ZigbeeNode::new(&mut device, &mut security_store, &mut profile);
            let mut app = CoordinatorApp::new(
                node,
                PersistentChildren::new(RamChildTableStore::new()),
                &POLICY,
                RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
            )
            .expect("valid coordinator composition");

            app.initialize().await.expect("form coordinator PAN");
            let pan_id = app.node().device().pan_id();
            let channel = app.node().device().channel();
            let network_key = live_network_key(app.node().device());
            println!(
                "  formed PAN 0x{pan_id:04X} on channel {channel} as 0x{:04X}",
                app.node().device().short_address()
            );
            let events = app.step().await.expect("finite coordinator step");
            println!(
                "  completed formation step with {} event(s)",
                events.iter().count()
            );
            (pan_id, channel, network_key)
        };

        let formed_state = security_store
            .load()
            .expect("load formed coordinator state")
            .expect("formed coordinator state exists");
        assert!(formed_state.commissioned);
        assert_eq!(formed_state.short_address, 0x0000);
        assert_eq!(formed_state.depth, 0);
        assert_eq!(formed_state.parent_address, 0xFFFF);

        {
            let mut profile = coordinator_profile();
            let mut device = coordinator_device(&profile, false);
            let node = ZigbeeNode::new(&mut device, &mut security_store, &mut profile);
            let mut app = CoordinatorApp::new(
                node,
                PersistentChildren::new(RamChildTableStore::new()),
                &POLICY,
                RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
            )
            .expect("valid restarted coordinator composition");

            app.initialize()
                .await
                .expect("restart persisted coordinator PAN");
            assert_eq!(app.node().device().pan_id(), pan_id);
            assert_eq!(app.node().device().channel(), channel);
            assert_eq!(live_network_key(app.node().device()), network_key);
            assert!(
                app.node().device().mac().tx_history().is_empty(),
                "persisted coordinator restart must not re-form or associate"
            );
            println!(
                "  restarted the same PAN 0x{pan_id:04X} on channel {channel} without re-formation"
            );
            let events = app.step().await.expect("finite restarted step");
            println!(
                "  completed restarted step with {} event(s)",
                events.iter().count()
            );
        }

        println!("CoordinatorApp persistence demo complete");
    });
}
