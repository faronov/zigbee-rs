//! Finite host composition of a forwarding dimmable-light router.

use router_app::{
    DiagnosticEvent, Diagnostics, NoChildren, NoStatus, NoSupervisor, RelayRouterApp, RouterParts,
    RouterPolicy,
};
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::mock::MockMac;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::{ApplicationProfile, DeviceProfile, DimmableLight};
use zigbee_runtime::security_store::{
    PersistentSecurityState, RamSecurityStateStore, SecurityStateStore,
};
use zigbee_runtime::{UserAction, ZigbeeDevice};
use zigbee_types::ShortAddress;
use zigbee_zcl::DeviceId;
use zigbee_zcl::clusters::Cluster;
use zigbee_zcl::clusters::basic::PowerSource;
use zigbee_zcl::clusters::level_control::{CMD_MOVE_TO_LEVEL, CMD_MOVE_TO_LEVEL_WITH_ON_OFF};
use zigbee_zcl::clusters::on_off::CMD_TOGGLE;

const LIGHT_IEEE: [u8; 8] = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80];
const COORDINATOR_IEEE: [u8; 8] = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77];
const PAN_ID: u16 = 0x1A62;
const CHANNEL: u8 = 15;

static POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 1_000,
    join_retry_initial_ms: 10,
    join_retry_max_ms: 100,
    secure_rejoin_failure_limit: 2,
};

fn commissioned_router_state() -> PersistentSecurityState {
    let mut state = PersistentSecurityState::empty();
    state.commissioned = true;
    state.extended_pan_id = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
    state.pan_id = PAN_ID;
    state.short_address = 0x5E3D;
    state.ieee_address = LIGHT_IEEE;
    state.channel = CHANNEL;
    state.depth = 1;
    state.parent_address = ShortAddress::COORDINATOR.0;
    state.update_id = 0;
    state.update_id_valid = true;
    state.network_key = [0x42; 16];
    state.key_sequence = 0;
    state.global_counter_limit = 0x400;
    state.tclk_present = true;
    state.trust_center_address = COORDINATOR_IEEE;
    state.trust_center_link_key = [0x5A; 16];
    state.tclk_counter_limit = 0x400;
    state.validate().expect("valid router security state");
    state
}

/// Counts commissioning attempts so the demo can show what a button press did.
#[derive(Default)]
struct AttemptCounter {
    attempts: u32,
}

impl Diagnostics for AttemptCounter {
    fn record(&mut self, event: DiagnosticEvent) {
        if matches!(event, DiagnosticEvent::CommissioningAttempt { .. }) {
            self.attempts += 1;
        }
    }
}

fn print_light_state(label: &str, light: &DimmableLight) {
    println!(
        "  {label}: on={}, level={}",
        light.is_on(),
        light.current_level()
    );
}

fn main() {
    pollster::block_on(async {
        println!("zigbee-rs finite RelayRouterApp dimmable-light demo");

        let mut profile = DeviceProfile::new(
            1,
            PROFILE_HOME_AUTOMATION,
            DeviceId::DIMMABLE_LIGHT,
            DimmableLight::default(),
        );
        let mut mac = MockMac::new(LIGHT_IEEE);
        mac.set_rx_delay_us(u32::MAX);
        let mut device = ZigbeeDevice::builder(mac)
            .manufacturer("Zigbee-RS")
            .model("MockDimLight-01")
            .power_source(PowerSource::MainsSinglePhase)
            .power_mode(PowerMode::AlwaysOn)
            .endpoint(
                profile.endpoint(),
                profile.profile_id(),
                profile.device_id(),
                |endpoint| profile.configure_endpoint(endpoint),
            )
            .build_relay();
        let mut store = RamSecurityStateStore::new();
        store
            .store(&commissioned_router_state())
            .expect("seed persisted router network");
        let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
        let mut app = RelayRouterApp::new(
            node,
            NoChildren,
            &POLICY,
            RouterParts::new(NoStatus, NoSupervisor, AttemptCounter::default()),
        )
        .expect("valid forwarding-light composition");
        #[cfg(feature = "green-power")]
        let mut gp_store = zigbee_runtime::gp_store::RamGpProxyStore::new();
        #[cfg(feature = "green-power")]
        app.set_green_power_store(&mut gp_store);

        app.initialize().await.expect("resume light router");
        println!(
            "  resumed router 0x{:04X} on PAN 0x{:04X}, channel {}",
            app.node().device().short_address(),
            app.node().device().pan_id(),
            app.node().device().channel()
        );
        print_light_state("initial", app.node().profile().component());

        app.node_mut()
            .profile_mut()
            .component_mut()
            .level_control_mut()
            .handle_command(CMD_MOVE_TO_LEVEL_WITH_ON_OFF, &[200, 0, 0])
            .expect("MoveToLevelWithOnOff");
        print_light_state(
            "MoveToLevelWithOnOff(200)",
            app.node().profile().component(),
        );

        app.node_mut()
            .profile_mut()
            .component_mut()
            .on_off_mut()
            .handle_command(CMD_TOGGLE, &[])
            .expect("Toggle");
        print_light_state("Toggle", app.node().profile().component());

        app.node_mut()
            .profile_mut()
            .component_mut()
            .level_control_mut()
            .handle_command(CMD_MOVE_TO_LEVEL, &[32, 0, 0])
            .expect("standalone MoveToLevel");
        print_light_state(
            "standalone MoveToLevel(32)",
            app.node().profile().component(),
        );

        for step in 1..=2 {
            let events = app.step().await.expect("finite router step");
            println!(
                "  completed finite router step {step}/2 with {} event(s)",
                events.iter().count()
            );
        }

        // A light or plug has a physical button; here it is simulated. The
        // gesture maps to `request_commissioning()`, which the app ignores
        // while joined, so a joined product keeps its normal button action.
        assert!(
            !app.request_commissioning(),
            "a joined light ignores a commissioning request"
        );
        println!("  simulated button while joined: ignored by the router app");

        // After a Leave without rejoin (coordinator Mgmt_Leave, Trust Center
        // Remove-Device, or this local Leave) the router stays factory-new
        // and does not search for a network on its own.
        app.node_mut().device_mut().user_action(UserAction::Leave);
        app.step().await.expect("leave step");
        assert!(!app.node().device().is_joined());
        assert!(app.awaiting_commissioning_request());
        let attempts_before = app.parts().diagnostics.attempts;
        for _ in 0..3 {
            app.step().await.expect("idle factory-new step");
        }
        assert!(app.awaiting_commissioning_request());
        assert_eq!(app.parts().diagnostics.attempts, attempts_before);
        println!("  left network: factory-new, waiting for the button");

        assert!(
            app.request_commissioning(),
            "a factory-new light accepts the button request"
        );
        assert!(!app.awaiting_commissioning_request());
        app.step()
            .await
            .expect("button-requested commissioning step");
        assert_eq!(app.parts().diagnostics.attempts, attempts_before + 1);
        // No coordinator is scripted, so the attempt finds no network and the
        // app falls back to its bounded retry backoff.
        println!(
            "  simulated button: one Network Steering attempt (joined={})",
            app.node().device().is_joined()
        );

        println!("RelayRouterApp dimmable-light demo complete");
    });
}

#[cfg(test)]
mod tests {
    /// Run the finite demo, including its post-Leave button assertions, under
    /// `cargo test --workspace`.
    #[test]
    fn finite_demo_waits_for_button_after_leave() {
        super::main();
    }
}
