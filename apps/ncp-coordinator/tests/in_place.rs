//! In-place construction used by the ESP32 NCP image (`Ncp::new_into`, a
//! Trust Center app built in its own storage and borrowed by `TcPort`) must
//! behave exactly like the by-value forms on the host link.

use std::collections::VecDeque;
use std::mem::MaybeUninit;

use ncp_coordinator_app::tc::{TcApp, TcPort};
use ncp_coordinator_app::transport::{HostTransport, TransportInfo};
use ncp_coordinator_app::{LogRing, Ncp, NcpIdentity};
use router_app::{
    NoApsTables, NoDiagnostics, NoStatus, NoSupervisor, PersistentChildren, RouterParts,
    RouterPolicy, TrustCenterCoordinatorApp,
};
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::mock::MockMac;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::child_store::RamChildTableStore;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::{ApplicationProfile, DeviceProfile, RangeExtender};
use zigbee_runtime::role::Router;
use zigbee_runtime::security_store::RamSecurityStateStore;
use zigbee_runtime::trust_center_store::RamTrustCenterDeviceStore;
use zigbee_types::ChannelMask;
use zigbee_zcl::DeviceId;
use zigbee_zrhp::codec::Bytes8;
use zigbee_zrhp::frame::TX_BUFFER_LEN;
use zigbee_zrhp::link::{Link, LinkConfig};
use zigbee_zrhp::msg::system::SysNegotiateReq;
use zigbee_zrhp::queue::EndpointQueues;

const IEEE: [u8; 8] = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80];

static POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 1_000,
    join_retry_initial_ms: 10,
    join_retry_max_ms: 100,
    secure_rejoin_failure_limit: 2,
};

#[derive(Default)]
struct Pipe {
    to_ncp: VecDeque<u8>,
    to_host: Vec<u8>,
}

impl HostTransport for Pipe {
    fn read(&mut self, buf: &mut [u8]) -> usize {
        let n = buf.len().min(self.to_ncp.len());
        for slot in &mut buf[..n] {
            *slot = self.to_ncp.pop_front().unwrap();
        }
        n
    }
    fn write(&mut self, data: &[u8]) -> usize {
        self.to_host.extend_from_slice(data);
        data.len()
    }
    fn info(&self) -> TransportInfo {
        TransportInfo {
            kind: 1,
            baud: 0,
            flow_control: false,
        }
    }
}

type Profile = DeviceProfile<RangeExtender>;

fn profile() -> Profile {
    DeviceProfile::new(
        1,
        PROFILE_HOME_AUTOMATION,
        DeviceId::COMBINED_INTERFACE,
        RangeExtender,
    )
}

fn device(profile: &Profile) -> ZigbeeDevice<MockMac, Router> {
    let mut mac = MockMac::new(IEEE);
    mac.set_rx_delay_us(u32::MAX);
    ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::AlwaysOn)
        .channels(ChannelMask(1 << 15))
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_coordinator()
}

fn identity() -> NcpIdentity {
    NcpIdentity {
        fw_version: "test",
        build_id: [0; 8],
        chip: 1,
        chip_rev: 0,
        flash_size: 4 * 1024 * 1024,
        board: "pipe",
        product: "test",
        reset_reason: 3,
        key_export: true,
        max_children: 64,
        max_tc_devices: 256,
        stack_allowed: true,
    }
}

fn clock_ms() -> u32 {
    0
}

fn poisoned<T>(slot: &mut MaybeUninit<T>) -> &mut MaybeUninit<T> {
    // Nothing may be read before the constructor writes it.
    unsafe { slot.as_mut_ptr().write_bytes(0xA5, 1) };
    slot
}

/// The bytes an NCP emits over a scripted host session: the NCP's reset,
/// the host's handshake and one negotiated request.
fn session<A: TcApp>(ncp: &mut Ncp<8>, port: &mut TcPort<A>) -> Vec<u8> {
    let log = LogRing::<8>::new();
    let mut pipe = Pipe::default();
    let mut host = Link::<32, 32>::new(LinkConfig::desktop_host(), 0x1A2B_3C4D);
    let mut queues = EndpointQueues::<4096>::new();
    let mut now = 1_000u64;
    host.open(now, zigbee_zrhp::frame::ResetReason::HostOpen);
    let mut emitted = Vec::new();
    for round in 0..12 {
        now += 1_000;
        if round == 4 {
            queues
                .request(
                    1,
                    &SysNegotiateReq {
                        major_min: 1,
                        major_max: 1,
                        host_caps: 0,
                        name: Bytes8(b"pipe-host"),
                    },
                )
                .unwrap();
        }
        let bytes = std::mem::take(&mut pipe.to_host);
        emitted.extend_from_slice(&bytes);
        let mut offset = 0;
        while offset < bytes.len() {
            offset += host.push_rx(&bytes[offset..], now);
            while host.poll_event().is_some() {}
        }
        let mut tx = [0u8; TX_BUFFER_LEN];
        while let Some(n) = host.poll_transmit(now, &mut queues, &mut tx) {
            pipe.to_ncp.extend(&tx[..n]);
        }
        assert!(ncp.service(now, &mut pipe, port, &log).is_none());
        while pollster::block_on(ncp.drive(port, || 0)) {}
        ncp.service(now, &mut pipe, port, &log);
    }
    emitted.extend_from_slice(&pipe.to_host);
    emitted
}

#[test]
fn in_place_ncp_and_borrowed_app_behave_like_the_by_value_forms() {
    // By value: `Ncp::new` and a port that owns its app.
    let mut security = RamSecurityStateStore::new();
    let mut profile_a = profile();
    let mut device_a = device(&profile_a);
    let app = TrustCenterCoordinatorApp::new(
        ZigbeeNode::new(&mut device_a, &mut security, &mut profile_a),
        PersistentChildren::new(RamChildTableStore::new()),
        RamTrustCenterDeviceStore::new(),
        &POLICY,
        RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
    )
    .unwrap();
    let mut port = TcPort::new(app, IEEE, clock_ms);
    let mut ncp = Ncp::<8>::new(identity(), 0xC0FF_EE01, 1_000);
    let by_value = session(&mut ncp, &mut port);

    // In place: `Ncp::new_into` and a port that borrows an app constructed
    // in its own storage.
    let mut security = RamSecurityStateStore::new();
    let mut profile_b = profile();
    let mut device_b = device(&profile_b);
    let mut app_storage = Box::new(MaybeUninit::uninit());
    let app = TrustCenterCoordinatorApp::new_with_aps_tables_into(
        poisoned(&mut app_storage),
        ZigbeeNode::new(&mut device_b, &mut security, &mut profile_b),
        PersistentChildren::new(RamChildTableStore::new()),
        NoApsTables,
        RamTrustCenterDeviceStore::new(),
        &POLICY,
        RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
    )
    .unwrap();
    let mut port = TcPort::new(app, IEEE, clock_ms);
    let mut ncp_storage = Box::new(MaybeUninit::uninit());
    let ncp = Ncp::<8>::new_into(poisoned(&mut ncp_storage), identity(), 0xC0FF_EE01, 1_000);
    let in_place = session(ncp, &mut port);

    assert!(by_value.len() > 64, "the session produced NCP output");
    assert_eq!(in_place, by_value);
    assert!(ncp.host_ready());
    drop(port);
    unsafe {
        ncp_storage.assume_init_drop();
        app_storage.assume_init_drop();
    }
}
