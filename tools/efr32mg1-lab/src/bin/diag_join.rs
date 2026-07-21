#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

#[path = "../shared/fault.rs"]
mod fault;
#[path = "../shared/platform.rs"]
mod platform;
#[path = "../shared/time_driver.rs"]
mod time_driver;
#[path = "../shared/vectors.rs"]
mod vectors;

use cortex_m as _;
use embassy_futures::select;
use embassy_time::{Duration, Instant, Timer};
use static_cell::StaticCell;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::MacDriver;
use zigbee_mac::efr32::Efr32Mac;
use zigbee_mac::primitives::{MlmeScanRequest, ScanType};
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, TickResult};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::{ClusterRef, ZigbeeDevice};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;
use zigbee_zcl::{ClusterId, DeviceId};

const JOIN_RETRY_SECS: u64 = 15;
const FAST_POLL_MS: u64 = 250;
const STEADY_POLL_MS: u64 = 1_000;
const FAST_POLL_SECS: u64 = 90;
const ANNCE_RETRY_SECS: u64 = 2;
const ANNCE_RETRIES: u8 = 5;
const SCAN_DURATION: u8 = 4;

fn clusters() -> [ClusterRef<'static>; 0] {
    []
}

#[derive(Copy, Clone)]
struct Handles {
    device: *mut ZigbeeDevice<Efr32Mac>,
}

async fn tick(handles: Handles, elapsed: u16) -> TickResult {
    let mut clusters = clusters();
    unsafe { (&mut *handles.device).tick(elapsed, &mut clusters).await }
}

async fn process(
    handles: Handles,
    indication: &zigbee_mac::McpsDataIndication,
) -> Option<StackEvent> {
    let mut clusters = clusters();
    unsafe {
        (&mut *handles.device)
            .process_incoming(indication, &mut clusters)
            .await
    }
}

struct JoinApp {
    device: &'static mut ZigbeeDevice<Efr32Mac>,
    last_join_attempt: Instant,
    join_attempts: u8,
    joined_at: Option<Instant>,
    last_annce_retry: Instant,
    annce_retries: u8,
    rx_frames: u32,
}

impl JoinApp {
    fn new(device: &'static mut ZigbeeDevice<Efr32Mac>) -> Self {
        Self {
            device,
            last_join_attempt: Instant::now(),
            join_attempts: 0,
            joined_at: None,
            last_annce_retry: Instant::now(),
            annce_retries: 0,
            rx_frames: 0,
        }
    }

    fn handles(&mut self) -> Handles {
        Handles {
            device: self.device as *mut _,
        }
    }

    fn handle_event(&mut self, event: &StackEvent) {
        match event {
            StackEvent::Joined {
                short_address,
                channel,
                pan_id,
            } => {
                self.joined_at = Some(Instant::now());
                self.last_annce_retry = Instant::now();
                self.annce_retries = 0;
                platform::led_on();
                rtt_target::rprintln!(
                    "[EFR32][diag-join] Joined addr=0x{:04X} ch={} pan=0x{:04X}",
                    short_address,
                    channel,
                    pan_id
                );
            }
            StackEvent::Left | StackEvent::LeaveRequested | StackEvent::RejoinRequested => {
                self.joined_at = None;
                self.annce_retries = 0;
                platform::led_off();
                rtt_target::rprintln!("[EFR32][diag-join] Left network");
            }
            StackEvent::CommissioningComplete { success } => rtt_target::rprintln!(
                "[EFR32][diag-join] Commissioning {}",
                if *success { "ok" } else { "failed" }
            ),
            StackEvent::ReportSent => {
                rtt_target::rprintln!("[EFR32][diag-join] Report sent")
            }
            _ => rtt_target::rprintln!("[EFR32][diag-join] Stack event"),
        }
    }

    fn poll_interval_ms(&self, now: Instant) -> u64 {
        match self.joined_at {
            Some(joined) if now.duration_since(joined).as_secs() < FAST_POLL_SECS => FAST_POLL_MS,
            Some(_) => STEADY_POLL_MS,
            None => 500,
        }
    }

    async fn request_join(&mut self, reason: &str) {
        let channel_mask = ChannelMask::ALL_2_4GHZ;
        self.last_join_attempt = Instant::now();
        self.join_attempts = self.join_attempts.wrapping_add(1);
        rtt_target::rprintln!(
            "[EFR32][diag-join] {} join attempt {}",
            reason,
            self.join_attempts
        );

        if self.device.bdb_mut().initialize().is_err() {
            rtt_target::rprintln!("[EFR32][diag-join] bdb_init=err");
            return;
        }
        match self
            .device
            .mac_mut()
            .mlme_scan(MlmeScanRequest {
                scan_type: ScanType::Active,
                channel_mask,
                scan_duration: SCAN_DURATION,
            })
            .await
        {
            Ok(confirm) => rtt_target::rprintln!(
                "[EFR32][diag-join] raw_scan={} beacon(s)",
                confirm.pan_descriptors.len()
            ),
            Err(_) => rtt_target::rprintln!("[EFR32][diag-join] raw_scan=err"),
        }

        match self
            .device
            .bdb_mut()
            .zdo_mut()
            .nlme_network_discovery(channel_mask, SCAN_DURATION)
            .await
        {
            Ok(networks) => {
                rtt_target::rprintln!("[EFR32][diag-join] pre_scan={} network(s)", networks.len());
                for (index, network) in networks.iter().take(4).enumerate() {
                    rtt_target::rprintln!(
                        "[EFR32][diag-join] net[{}] pan=0x{:04X} ch={} permit={} \
                         depth={} lqi={} via=0x{:04X}",
                        index,
                        network.pan_id.0,
                        network.logical_channel,
                        network.permit_joining as u8,
                        network.depth,
                        network.lqi,
                        network.router_address.0
                    );
                }
            }
            Err(_) => rtt_target::rprintln!("[EFR32][diag-join] pre_scan=err"),
        }

        self.device.bdb_mut().attributes_mut().primary_channel_set = channel_mask;
        self.device.bdb_mut().attributes_mut().secondary_channel_set = ChannelMask(0);
        if self.device.bdb_mut().network_steering().await.is_err() {
            rtt_target::rprintln!("[EFR32][diag-join] network_steering=err");
            return;
        }

        let nib = self.device.bdb().zdo().nwk().nib();
        self.joined_at = Some(Instant::now());
        self.last_annce_retry = Instant::now();
        self.annce_retries = 0;
        rtt_target::rprintln!(
            "[EFR32][diag-join] network_steering=ok addr=0x{:04X} ch={} pan=0x{:04X}",
            nib.network_address.0,
            nib.logical_channel,
            nib.pan_id.0
        );
        let _ = self.device.send_device_annce().await;
    }

    async fn retry_announce(&mut self, now: Instant) {
        let Some(joined) = self.joined_at else {
            return;
        };
        if now.duration_since(joined).as_secs() >= FAST_POLL_SECS
            || self.annce_retries >= ANNCE_RETRIES
            || now.duration_since(self.last_annce_retry).as_secs() < ANNCE_RETRY_SECS
        {
            return;
        }
        self.last_annce_retry = now;
        self.annce_retries += 1;
        let passed = self.device.send_device_annce().await.is_ok();
        rtt_target::rprintln!(
            "[EFR32][diag-join] periodic device_annce {}={}",
            self.annce_retries,
            if passed { "ok" } else { "err" }
        );
    }

    async fn service_polls(&mut self) {
        for _ in 0..4 {
            match self.device.poll().await {
                Ok(Some(indication)) => {
                    self.rx_frames = self.rx_frames.wrapping_add(1);
                    rtt_target::rprintln!(
                        "[EFR32][diag-join] Poll RX {} bytes (frame #{})",
                        indication.payload.len(),
                        self.rx_frames
                    );
                    if let Some(event) = process(self.handles(), &indication).await {
                        self.handle_event(&event);
                    }
                    if let TickResult::Event(ref event) = tick(self.handles(), 0).await {
                        self.handle_event(event);
                    }
                }
                Ok(None) => break,
                Err(_) => {
                    rtt_target::rprintln!("[EFR32][diag-join] Poll failed");
                    break;
                }
            }
        }
    }

    async fn direct_rx(&mut self, window_ms: u64) {
        let deadline = Instant::now() + Duration::from_millis(window_ms);
        loop {
            let now = Instant::now();
            if now >= deadline {
                break;
            }
            match select::select(self.device.receive(), Timer::after(deadline - now)).await {
                select::Either::First(Ok(indication)) => {
                    self.rx_frames = self.rx_frames.wrapping_add(1);
                    rtt_target::rprintln!(
                        "[EFR32][diag-join] Direct RX {} bytes (frame #{})",
                        indication.payload.len(),
                        self.rx_frames
                    );
                    if let Some(event) = process(self.handles(), &indication).await {
                        self.handle_event(&event);
                    }
                    if let TickResult::Event(ref event) = tick(self.handles(), 0).await {
                        self.handle_event(event);
                    }
                }
                select::Either::First(Err(_)) | select::Either::Second(_) => break,
            }
        }
    }

    async fn run(&mut self) -> ! {
        self.request_join("startup").await;
        loop {
            let now = Instant::now();
            if !self.device.is_joined()
                && now.duration_since(self.last_join_attempt).as_secs() >= JOIN_RETRY_SECS
            {
                self.request_join("retry").await;
            }
            let poll_ms = self.poll_interval_ms(now);
            if self.device.is_joined() {
                self.device.mac_mut().radio_wake();
                self.direct_rx(poll_ms).await;
                self.service_polls().await;
                self.retry_announce(Instant::now()).await;
                if let TickResult::Event(ref event) = tick(self.handles(), 1).await {
                    self.handle_event(event);
                }
            } else {
                self.device.mac_mut().radio_sleep();
                Timer::after(Duration::from_millis(poll_ms)).await;
                self.device.mac_mut().radio_wake();
                platform::led_on();
                Timer::after(Duration::from_millis(80)).await;
                platform::led_off();
            }
        }
    }
}

static DEVICE: StaticCell<ZigbeeDevice<Efr32Mac>> = StaticCell::new();
static APP: StaticCell<JoinApp> = StaticCell::new();

fn build_app() -> &'static mut JoinApp {
    let mac = Efr32Mac::new();
    let mut ieee = mac.extended_address();
    ieee[0] |= 0x02;
    ieee[7] ^= 0xD1;
    let device = ZigbeeDevice::builder(mac.with_extended_address(ieee))
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 1_000,
            wake_duration_ms: 300,
        })
        .manufacturer("Zigbee-RS")
        .model("EFR32MG1-JoinDiag")
        .date_code("20260422")
        .sw_build("0.1.0")
        .power_source(PowerSource::Battery)
        .channels(ChannelMask::ALL_2_4GHZ)
        .endpoint(
            1,
            PROFILE_HOME_AUTOMATION,
            DeviceId::TEMPERATURE_SENSOR,
            |endpoint| {
                endpoint
                    .cluster_server(ClusterId::BASIC)
                    .cluster_server(ClusterId::IDENTIFY)
            },
        )
        .build_into(DEVICE.uninit());
    APP.init_with(|| JoinApp::new(device))
}

#[embassy_executor::task]
async fn run(app: &'static mut JoinApp) -> ! {
    for _ in 0..3 {
        platform::led_on();
        Timer::after(Duration::from_millis(100)).await;
        platform::led_off();
        Timer::after(Duration::from_millis(100)).await;
    }
    Timer::after(Duration::from_millis(500)).await;
    app.run().await
}

#[cortex_m_rt::entry]
fn main() -> ! {
    platform::init_small!("diag-join");
    time_driver::init();
    let app = build_app();
    static EXECUTOR: StaticCell<embassy_executor::Executor> = StaticCell::new();
    EXECUTOR
        .init(embassy_executor::Executor::new())
        .run(|spawner| spawner.must_spawn(run(app)))
}
