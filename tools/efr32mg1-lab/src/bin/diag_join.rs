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

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m as _;
use efr32mg1_hal::pm;
use embassy_futures::select;
use embassy_time::{Duration, Instant, Timer};
use static_cell::StaticCell;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_bdb::attributes::{BDB_POPULAR_CHANNEL_FALLBACK_SET, BDB_POPULAR_CHANNEL_SET};
use zigbee_mac::efr32::Efr32Mac;
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

#[repr(C)]
struct JoinMetrics {
    magic: AtomicU32,
    network_up_attempt_duration_ms: AtomicU32,
    network_up_since_boot_ms: AtomicU32,
    join_attempt_duration_ms: AtomicU32,
    join_complete_since_boot_ms: AtomicU32,
    first_sleep_since_boot_ms: AtomicU32,
    join_attempts: AtomicU32,
    em2_sleep_count: AtomicU32,
    em2_wake_count: AtomicU32,
    em2_error_count: AtomicU32,
    rx_frames_at_first_sleep: AtomicU32,
    rx_frames: AtomicU32,
}

impl JoinMetrics {
    const fn new() -> Self {
        Self {
            magic: AtomicU32::new(0x4A4F_494E),
            network_up_attempt_duration_ms: AtomicU32::new(0),
            network_up_since_boot_ms: AtomicU32::new(0),
            join_attempt_duration_ms: AtomicU32::new(0),
            join_complete_since_boot_ms: AtomicU32::new(0),
            first_sleep_since_boot_ms: AtomicU32::new(0),
            join_attempts: AtomicU32::new(0),
            em2_sleep_count: AtomicU32::new(0),
            em2_wake_count: AtomicU32::new(0),
            em2_error_count: AtomicU32::new(0),
            rx_frames_at_first_sleep: AtomicU32::new(0),
            rx_frames: AtomicU32::new(0),
        }
    }
}

#[unsafe(no_mangle)]
static JOIN_METRICS: JoinMetrics = JoinMetrics::new();

#[used]
#[unsafe(no_mangle)]
static JOIN_EUI_VARIANT: u8 = 0;

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
    boot_at: Instant,
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
            boot_at: Instant::now(),
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
                let now = Instant::now();
                self.joined_at = Some(now);
                self.last_annce_retry = now;
                self.annce_retries = 0;
                JOIN_METRICS.network_up_attempt_duration_ms.store(
                    now.duration_since(self.last_join_attempt)
                        .as_millis()
                        .min(u32::MAX as u64) as u32,
                    Ordering::Release,
                );
                JOIN_METRICS.network_up_since_boot_ms.store(
                    now.duration_since(self.boot_at)
                        .as_millis()
                        .min(u32::MAX as u64) as u32,
                    Ordering::Release,
                );
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
            StackEvent::CommissioningComplete { success } => {
                if *success {
                    let now = Instant::now();
                    JOIN_METRICS.join_attempt_duration_ms.store(
                        now.duration_since(self.last_join_attempt)
                            .as_millis()
                            .min(u32::MAX as u64) as u32,
                        Ordering::Release,
                    );
                    JOIN_METRICS.join_complete_since_boot_ms.store(
                        now.duration_since(self.boot_at)
                            .as_millis()
                            .min(u32::MAX as u64) as u32,
                        Ordering::Release,
                    );
                } else {
                    self.joined_at = None;
                    self.annce_retries = 0;
                    platform::led_off();
                }
                rtt_target::rprintln!(
                    "[EFR32][diag-join] Commissioning {}",
                    if *success { "ok" } else { "failed" }
                );
            }
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
        self.last_join_attempt = Instant::now();
        self.join_attempts = self.join_attempts.wrapping_add(1);
        JOIN_METRICS
            .join_attempts
            .store(self.join_attempts as u32, Ordering::Release);
        rtt_target::rprintln!(
            "[EFR32][diag-join] {} join attempt {}",
            reason,
            self.join_attempts
        );

        if self.device.bdb_mut().initialize().is_err() {
            rtt_target::rprintln!("[EFR32][diag-join] bdb_init=err");
            return;
        }
        self.device.bdb_mut().attributes_mut().primary_channel_set = BDB_POPULAR_CHANNEL_SET;
        self.device.bdb_mut().attributes_mut().secondary_channel_set =
            BDB_POPULAR_CHANNEL_FALLBACK_SET;
        if self.device.bdb_mut().network_steering().await.is_err() {
            let (started, completed, failed, dsn, state) =
                self.device.mac_mut().software_ack_snapshot();
            rtt_target::rprintln!("[EFR32][diag-join] network_steering=err");
            rtt_target::rprintln!(
                "[EFR32][diag-join] sw_ack {}/{}/{} dsn={} state={}",
                started,
                completed,
                failed,
                dsn,
                state,
            );
            return;
        }

        let now = Instant::now();
        let nib = self.device.bdb().zdo().nwk().nib();
        self.joined_at = Some(now);
        self.last_annce_retry = now;
        self.annce_retries = 0;
        JOIN_METRICS.network_up_attempt_duration_ms.store(
            now.duration_since(self.last_join_attempt)
                .as_millis()
                .min(u32::MAX as u64) as u32,
            Ordering::Release,
        );
        JOIN_METRICS.network_up_since_boot_ms.store(
            now.duration_since(self.boot_at)
                .as_millis()
                .min(u32::MAX as u64) as u32,
            Ordering::Release,
        );
        rtt_target::rprintln!(
            "[EFR32][diag-join] network_up addr=0x{:04X} ch={} pan=0x{:04X}",
            nib.network_address.0,
            nib.logical_channel,
            nib.pan_id.0
        );
        let (started, completed, failed, dsn, state) =
            self.device.mac_mut().software_ack_snapshot();
        rtt_target::rprintln!(
            "[EFR32][diag-join] sw_ack={}/{}/{} dsn={} state={}",
            started,
            completed,
            failed,
            dsn,
            state
        );
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
                    JOIN_METRICS
                        .rx_frames
                        .store(self.rx_frames, Ordering::Release);
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
                    JOIN_METRICS
                        .rx_frames
                        .store(self.rx_frames, Ordering::Release);
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

    fn sleep_joined_until_next_poll(&mut self) {
        self.device.mac_mut().radio_sleep();
        cortex_m::peripheral::NVIC::unpend(vectors::Interrupt::FrcPri);

        if JOIN_METRICS
            .first_sleep_since_boot_ms
            .load(Ordering::Acquire)
            == 0
        {
            let elapsed = Instant::now()
                .duration_since(self.boot_at)
                .as_millis()
                .min(u32::MAX as u64) as u32;
            JOIN_METRICS
                .first_sleep_since_boot_ms
                .store(elapsed, Ordering::Release);
            JOIN_METRICS
                .rx_frames_at_first_sleep
                .store(self.rx_frames, Ordering::Release);
        }
        JOIN_METRICS.em2_sleep_count.fetch_add(1, Ordering::AcqRel);

        let ticks = pm::ms_to_ticks(STEADY_POLL_MS as u32, pm::LFRCO_HZ);
        if pm::sleep_for_ticks_polled(ticks).is_err() {
            JOIN_METRICS.em2_error_count.fetch_add(1, Ordering::AcqRel);
            platform::halt();
        }
        JOIN_METRICS.em2_wake_count.fetch_add(1, Ordering::AcqRel);

        if efr32mg1_tradfri::init_clocks().is_err() {
            JOIN_METRICS.em2_error_count.fetch_add(1, Ordering::AcqRel);
            platform::halt();
        }
    }

    async fn run(&mut self) -> ! {
        self.request_join("startup").await;
        loop {
            if !self.device.is_joined()
                && Instant::now()
                    .duration_since(self.last_join_attempt)
                    .as_secs()
                    >= JOIN_RETRY_SECS
            {
                self.request_join("retry").await;
            }
            let now = Instant::now();
            let poll_ms = self.poll_interval_ms(now);
            if self.device.is_joined() {
                self.device.mac_mut().radio_wake();
                if poll_ms == FAST_POLL_MS {
                    self.direct_rx(poll_ms).await;
                }
                self.service_polls().await;
                self.retry_announce(Instant::now()).await;
                if let TickResult::Event(ref event) = tick(self.handles(), 1).await {
                    self.handle_event(event);
                }
                if poll_ms == STEADY_POLL_MS {
                    self.sleep_joined_until_next_poll();
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
    let eui_variant = unsafe { core::ptr::read_volatile(&JOIN_EUI_VARIANT) };
    ieee[0] ^= 0x01;
    ieee[1] ^= eui_variant;
    ieee[7] = (ieee[7] | 0x02) & 0xFE;
    let device = ZigbeeDevice::builder(mac.with_extended_address(ieee))
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 1_000,
            wake_duration_ms: 300,
        })
        .automatic_polling(false)
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
    fault::clear();
    time_driver::init();
    let app = build_app();
    static EXECUTOR: StaticCell<embassy_executor::Executor> = StaticCell::new();
    EXECUTOR
        .init(embassy_executor::Executor::new())
        .run(|spawner| spawner.must_spawn(run(app)))
}
