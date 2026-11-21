//! The NCP on the host: the real Trust Center coordinator on a mock MAC,
//! speaking ZRHP over stdin/stdout.
//!
//! Host libraries (the zigpy library `zigpy-zrhp`, the Go adapter, the
//! zigbee-herdsman adapter) run conformance tests against this process: the
//! link layer, the message dispatch, formation, backup export and restore
//! are the NCP's own code; only the radio is simulated (nothing joins).
//! Bytes are raw ZRHP wire bytes in both directions; diagnostics go to
//! stderr.
//!
//! ```text
//! cargo run --manifest-path apps/ncp-coordinator/Cargo.toml --example ncp_sim
//! ```
//!
//! `SYS_RESET` ends the process with exit code 0 (the RAM stores are lost,
//! like a factory-new NCP); a host test restarts it when it needs a reboot.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::sync::OnceLock;
use std::sync::mpsc;
use std::time::{Duration, Instant};

use ncp_coordinator_app::tc::TcPort;
use ncp_coordinator_app::transport::{HostTransport, TransportInfo, UartChangeError};
use ncp_coordinator_app::{LogRing, Ncp, NcpIdentity};
use router_app::{
    NoDiagnostics, NoStatus, NoSupervisor, PersistentChildren, RouterParts, RouterPolicy,
    TrustCenterCoordinatorApp,
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

const IEEE: [u8; 8] = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80];

static POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 1_000,
    join_retry_initial_ms: 10,
    join_retry_max_ms: 100,
    secure_rejoin_failure_limit: 2,
};

static START: OnceLock<Instant> = OnceLock::new();

fn now_us() -> u64 {
    START.get_or_init(Instant::now).elapsed().as_micros() as u64
}

fn clock_ms() -> u32 {
    (now_us() / 1000) as u32
}

/// stdin -> NCP, NCP -> stdout.
struct StdioPipe {
    rx: mpsc::Receiver<Vec<u8>>,
    to_ncp: VecDeque<u8>,
    to_host: Vec<u8>,
    closed: bool,
}

impl StdioPipe {
    fn pump_in(&mut self) {
        loop {
            match self.rx.try_recv() {
                Ok(chunk) => self.to_ncp.extend(chunk),
                Err(mpsc::TryRecvError::Empty) => return,
                Err(mpsc::TryRecvError::Disconnected) => {
                    self.closed = self.to_ncp.is_empty();
                    return;
                }
            }
        }
    }

    fn flush_out(&mut self) {
        if self.to_host.is_empty() {
            return;
        }
        let mut out = std::io::stdout().lock();
        if out
            .write_all(&self.to_host)
            .and_then(|()| out.flush())
            .is_err()
        {
            self.closed = true;
        }
        self.to_host.clear();
    }
}

impl HostTransport for StdioPipe {
    fn read(&mut self, buf: &mut [u8]) -> usize {
        let n = buf.len().min(self.to_ncp.len());
        for (slot, byte) in buf[..n].iter_mut().zip(self.to_ncp.drain(..n)) {
            *slot = byte;
        }
        n
    }
    fn write(&mut self, data: &[u8]) -> usize {
        self.to_host.extend_from_slice(data);
        data.len()
    }
    fn info(&self) -> TransportInfo {
        TransportInfo {
            kind: 2,
            baud: 921_600,
            flow_control: true,
        }
    }
    fn check_uart(&self, _baud: u32, _flow: bool) -> Result<(), UartChangeError> {
        Err(UartChangeError::Unsupported)
    }
    fn apply_uart(&mut self, _baud: u32, _flow: bool) -> Result<(), UartChangeError> {
        Err(UartChangeError::Unsupported)
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
    for channel in 11..=26 {
        mac.add_energy(zigbee_mac::EdValue {
            channel,
            energy: if channel == 15 { 10 } else { 120 },
        });
    }
    // A neighbouring Zigbee PRO network on channel 25 (coordinator and one
    // router) for `NWK_ACTIVE_SCAN`.
    for src in [0x0000, 0x5A17] {
        use zigbee_mac::primitives::{PanDescriptor, SuperframeSpec, ZigbeeBeaconPayload};
        use zigbee_types::{MacAddress, PanId, ShortAddress};
        mac.add_beacon(PanDescriptor {
            channel: 25,
            coord_address: MacAddress::Short(PanId(0x4D2A), ShortAddress(src)),
            superframe_spec: SuperframeSpec {
                pan_coordinator: src == 0x0000,
                ..Default::default()
            },
            lqi: 180,
            security_use: false,
            zigbee_beacon: ZigbeeBeaconPayload {
                protocol_id: 0,
                stack_profile: 2,
                protocol_version: 2,
                router_capacity: true,
                device_depth: u8::from(src != 0x0000),
                end_device_capacity: true,
                extended_pan_id: [0xDD, 0xCC, 0xBB, 0xAA, 0x44, 0x33, 0x22, 0x11],
                tx_offset: [0xFF; 3],
                update_id: 1,
            },
        });
    }
    ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::AlwaysOn)
        .channels(ChannelMask::ALL_2_4GHZ)
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
        fw_version: concat!("ncp-sim/", env!("CARGO_PKG_VERSION")),
        build_id: [0x51, 0x4D, 0, 0, 0, 0, 0, 1],
        chip: 0,
        chip_rev: 0,
        flash_size: 4 * 1024 * 1024,
        board: "host",
        product: "zigbee-rs NCP simulator",
        reset_reason: 1,
        key_export: true,
        max_children: 64,
        max_tc_devices: 256,
        stack_allowed: true,
    }
}

fn main() {
    let _ = now_us();
    let (tx, rx) = mpsc::channel::<Vec<u8>>();
    std::thread::spawn(move || {
        let mut stdin = std::io::stdin().lock();
        let mut buf = [0u8; 4096];
        loop {
            match stdin.read(&mut buf) {
                Ok(0) | Err(_) => return,
                Ok(n) => {
                    if tx.send(buf[..n].to_vec()).is_err() {
                        return;
                    }
                }
            }
        }
    });

    let mut security = RamSecurityStateStore::new();
    let mut gp_store = zigbee_runtime::gp_store::RamGpProxyStore::new();
    let mut profile = profile();
    let mut device = device(&profile);
    let node = ZigbeeNode::new(&mut device, &mut security, &mut profile);
    let mut app = match TrustCenterCoordinatorApp::new(
        node,
        PersistentChildren::new(RamChildTableStore::new()),
        RamTrustCenterDeviceStore::new(),
        &POLICY,
        RouterParts::new(NoStatus, NoSupervisor, NoDiagnostics),
    ) {
        Ok(app) => app,
        Err(error) => {
            eprintln!("ncp_sim: coordinator setup failed: {error:?}");
            std::process::exit(2);
        }
    };
    app.set_green_power_store(&mut gp_store);
    let mut port = TcPort::new(app, IEEE, clock_ms);
    let log = LogRing::<8>::new();
    let cookie = (std::process::id() ^ (now_us() as u32).rotate_left(13)) | 1;
    let mut ncp = Ncp::<8>::new(identity(), cookie, now_us());
    let mut pipe = StdioPipe {
        rx,
        to_ncp: VecDeque::new(),
        to_host: Vec::new(),
        closed: false,
    };
    eprintln!("ncp_sim: up, cookie 0x{cookie:08X}");
    loop {
        pipe.pump_in();
        if let Some(request) = ncp.service(now_us(), &mut pipe, &mut port, &log) {
            pipe.flush_out();
            eprintln!("ncp_sim: {request:?}, exiting");
            std::process::exit(0);
        }
        let mut ran = false;
        while pollster::block_on(ncp.drive(&mut port, now_us)) {
            ran = true;
        }
        if ran {
            let _ = ncp.service(now_us(), &mut pipe, &mut port, &log);
        }
        pipe.flush_out();
        if pipe.closed {
            return;
        }
        std::thread::sleep(Duration::from_micros(500));
    }
}
