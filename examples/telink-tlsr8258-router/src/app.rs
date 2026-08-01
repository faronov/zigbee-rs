//! Always-on TLSR8258 parent-router application.
//!
//! Child state is RAM-only and is rebuilt by re-association after reboot.
//! See the package README for the remaining hardware-validation boundary.

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU32, Ordering};

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::{MacDriver, MacError, PlatformServices, telink::TelinkMac};
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::{ApplicationProfile, DeviceProfile, RangeExtender};
use zigbee_runtime::role::Router;
use zigbee_runtime::security_store::SecurityStateStore;
use zigbee_zcl::DeviceId;
use zigbee_zcl::clusters::basic::PowerSource;

use tlsr8258_tb04::{leds::StatusLeds, resources::BoardResources};

// Distinct from the sensor runtime's `DEVICE_EUI_OFFSET` (0x33) so a router
// and a sensor built from the same factory-programmed part never collide on
// IEEE address if someone reflashes one board with both images over time.
const DEVICE_EUI_OFFSET: u8 = 0x52; // 'R' for Router
const ENDPOINT: u8 = 1;
const MAX_RX_SLICE_US: u32 = 20_000;
const JOIN_RETRY_MIN_MS: u32 = 5_000;
const JOIN_RETRY_MAX_MS: u32 = 60_000;

#[repr(C)]
struct JoinMetrics {
    magic: AtomicU32,
    join_attempts: AtomicU32,
    attempt_started_us: AtomicU32,
    association_ms: AtomicU32,
    transport_key_ms: AtomicU32,
    security_reserved_ms: AtomicU32,
    device_annce_ms: AtomicU32,
    network_up_ms: AtomicU32,
    tclk_complete_ms: AtomicU32,
    first_node_desc_ms: AtomicU32,
    first_active_ep_ms: AtomicU32,
    first_simple_desc_ms: AtomicU32,
    commissioning_complete_ms: AtomicU32,
    rx_frames: AtomicU32,
    last_steering_stage: AtomicU32,
    device_annce_attempts: AtomicU32,
    device_annce_failures: AtomicU32,
    request_key_attempts: AtomicU32,
    request_key_successes: AtomicU32,
    request_key_failures: AtomicU32,
    request_key_error: AtomicU32,
    tclk_installations: AtomicU32,
    verify_key_attempts: AtomicU32,
    verify_key_successes: AtomicU32,
    verify_key_error: AtomicU32,
    verify_key_sent: AtomicU32,
    verify_key_frame_counter: AtomicU32,
    confirm_key_frames: AtomicU32,
    confirm_key_successes: AtomicU32,
    confirm_key_rejections: AtomicU32,
    last_confirm_key_status: AtomicU32,
    last_confirm_key_type: AtomicU32,
    last_confirm_key_key_identifier: AtomicU32,
    last_confirm_key_security: AtomicU32,
    last_confirm_key_source: AtomicU32,
    last_confirm_key_source_ieee_low: AtomicU32,
    last_confirm_key_source_ieee_high: AtomicU32,
    last_confirm_key_destination_low: AtomicU32,
    last_confirm_key_destination_high: AtomicU32,
    zdo_response_attempts: AtomicU32,
    zdo_response_successes: AtomicU32,
    zdo_response_failures: AtomicU32,
}

impl JoinMetrics {
    const fn new() -> Self {
        Self {
            magic: AtomicU32::new(0x544A_4F49),
            join_attempts: AtomicU32::new(0),
            attempt_started_us: AtomicU32::new(0),
            association_ms: AtomicU32::new(0),
            transport_key_ms: AtomicU32::new(0),
            security_reserved_ms: AtomicU32::new(0),
            device_annce_ms: AtomicU32::new(0),
            network_up_ms: AtomicU32::new(0),
            tclk_complete_ms: AtomicU32::new(0),
            first_node_desc_ms: AtomicU32::new(0),
            first_active_ep_ms: AtomicU32::new(0),
            first_simple_desc_ms: AtomicU32::new(0),
            commissioning_complete_ms: AtomicU32::new(0),
            rx_frames: AtomicU32::new(0),
            last_steering_stage: AtomicU32::new(0),
            device_annce_attempts: AtomicU32::new(0),
            device_annce_failures: AtomicU32::new(0),
            request_key_attempts: AtomicU32::new(0),
            request_key_successes: AtomicU32::new(0),
            request_key_failures: AtomicU32::new(0),
            request_key_error: AtomicU32::new(0),
            tclk_installations: AtomicU32::new(0),
            verify_key_attempts: AtomicU32::new(0),
            verify_key_successes: AtomicU32::new(0),
            verify_key_error: AtomicU32::new(0),
            verify_key_sent: AtomicU32::new(0),
            verify_key_frame_counter: AtomicU32::new(0),
            confirm_key_frames: AtomicU32::new(0),
            confirm_key_successes: AtomicU32::new(0),
            confirm_key_rejections: AtomicU32::new(0),
            last_confirm_key_status: AtomicU32::new(0xFF),
            last_confirm_key_type: AtomicU32::new(0xFF),
            last_confirm_key_key_identifier: AtomicU32::new(0xFF),
            last_confirm_key_security: AtomicU32::new(0),
            last_confirm_key_source: AtomicU32::new(0xFFFF),
            last_confirm_key_source_ieee_low: AtomicU32::new(0),
            last_confirm_key_source_ieee_high: AtomicU32::new(0),
            last_confirm_key_destination_low: AtomicU32::new(0),
            last_confirm_key_destination_high: AtomicU32::new(0),
            zdo_response_attempts: AtomicU32::new(0),
            zdo_response_successes: AtomicU32::new(0),
            zdo_response_failures: AtomicU32::new(0),
        }
    }

    fn begin_attempt(&self, started_us: u32) {
        let attempts = self.join_attempts.load(Ordering::Relaxed);
        self.join_attempts
            .store(attempts.wrapping_add(1), Ordering::Relaxed);
        self.attempt_started_us.store(started_us, Ordering::Relaxed);
        self.association_ms.store(0, Ordering::Relaxed);
        self.transport_key_ms.store(0, Ordering::Relaxed);
        self.security_reserved_ms.store(0, Ordering::Relaxed);
        self.device_annce_ms.store(0, Ordering::Relaxed);
        self.network_up_ms.store(0, Ordering::Relaxed);
        self.tclk_complete_ms.store(0, Ordering::Relaxed);
        self.first_node_desc_ms.store(0, Ordering::Relaxed);
        self.first_active_ep_ms.store(0, Ordering::Relaxed);
        self.first_simple_desc_ms.store(0, Ordering::Relaxed);
        self.commissioning_complete_ms.store(0, Ordering::Relaxed);
        self.rx_frames.store(0, Ordering::Relaxed);
        self.last_steering_stage.store(0, Ordering::Relaxed);
        self.device_annce_attempts.store(0, Ordering::Relaxed);
        self.device_annce_failures.store(0, Ordering::Relaxed);
        self.request_key_attempts.store(0, Ordering::Relaxed);
        self.request_key_successes.store(0, Ordering::Relaxed);
        self.request_key_failures.store(0, Ordering::Relaxed);
        self.request_key_error.store(0, Ordering::Relaxed);
        self.tclk_installations.store(0, Ordering::Relaxed);
        self.verify_key_attempts.store(0, Ordering::Relaxed);
        self.verify_key_successes.store(0, Ordering::Relaxed);
        self.verify_key_error.store(0, Ordering::Relaxed);
        self.verify_key_sent.store(0, Ordering::Relaxed);
        self.verify_key_frame_counter.store(0, Ordering::Relaxed);
        self.confirm_key_frames.store(0, Ordering::Relaxed);
        self.confirm_key_successes.store(0, Ordering::Relaxed);
        self.confirm_key_rejections.store(0, Ordering::Relaxed);
        self.last_confirm_key_status.store(0xFF, Ordering::Relaxed);
        self.last_confirm_key_type.store(0xFF, Ordering::Relaxed);
        self.last_confirm_key_key_identifier
            .store(0xFF, Ordering::Relaxed);
        self.last_confirm_key_security.store(0, Ordering::Relaxed);
        self.last_confirm_key_source
            .store(0xFFFF, Ordering::Relaxed);
        self.last_confirm_key_source_ieee_low
            .store(0, Ordering::Relaxed);
        self.last_confirm_key_source_ieee_high
            .store(0, Ordering::Relaxed);
        self.last_confirm_key_destination_low
            .store(0, Ordering::Relaxed);
        self.last_confirm_key_destination_high
            .store(0, Ordering::Relaxed);
        self.zdo_response_attempts.store(0, Ordering::Relaxed);
        self.zdo_response_successes.store(0, Ordering::Relaxed);
        self.zdo_response_failures.store(0, Ordering::Relaxed);
    }

    fn capture_steering(&self, device: &ZigbeeDevice<TelinkMac, Router>) {
        let diagnostics = device.steering_diagnostics();
        let started_us = if diagnostics.attempt_started_us != 0 {
            self.attempt_started_us
                .store(diagnostics.attempt_started_us, Ordering::Relaxed);
            diagnostics.attempt_started_us
        } else {
            self.attempt_started_us.load(Ordering::Relaxed)
        };

        self.store_milestone(
            &self.association_ms,
            started_us,
            diagnostics.association_complete_us,
        );
        self.store_milestone(
            &self.transport_key_ms,
            started_us,
            diagnostics.transport_key_received_us,
        );
        self.store_milestone(
            &self.security_reserved_ms,
            started_us,
            diagnostics.security_reserved_us,
        );
        self.store_milestone(
            &self.device_annce_ms,
            started_us,
            diagnostics.device_annce_sent_us,
        );
        self.store_milestone(&self.network_up_ms, started_us, diagnostics.network_up_us);
        self.store_milestone(
            &self.tclk_complete_ms,
            started_us,
            diagnostics.tclk_complete_us,
        );
        self.last_steering_stage
            .store(diagnostics.stage as u32, Ordering::Relaxed);
        self.device_annce_attempts.store(
            u32::from(diagnostics.device_annce_attempts),
            Ordering::Relaxed,
        );
        self.device_annce_failures.store(
            u32::from(diagnostics.device_annce_failures),
            Ordering::Relaxed,
        );
        self.request_key_attempts.store(
            u32::from(diagnostics.request_key_attempts),
            Ordering::Relaxed,
        );
        self.request_key_successes.store(
            u32::from(diagnostics.request_key_send_successes),
            Ordering::Relaxed,
        );
        self.request_key_failures.store(
            u32::from(diagnostics.request_key_send_failures),
            Ordering::Relaxed,
        );
        self.request_key_error
            .store(u32::from(diagnostics.request_key_error), Ordering::Relaxed);
        self.tclk_installations
            .store(u32::from(diagnostics.tclk_installations), Ordering::Relaxed);
        self.verify_key_attempts.store(
            u32::from(diagnostics.verify_key_attempts),
            Ordering::Relaxed,
        );
        self.verify_key_successes.store(
            u32::from(diagnostics.verify_key_successes),
            Ordering::Relaxed,
        );
        self.verify_key_error
            .store(u32::from(diagnostics.verify_key_error), Ordering::Relaxed);

        let security = device.aps_security_handshake_stats();
        self.verify_key_sent
            .store(security.verify_key_sent, Ordering::Relaxed);
        self.verify_key_frame_counter
            .store(security.last_verify_key_frame_counter, Ordering::Relaxed);
        self.confirm_key_frames
            .store(security.confirm_key_received, Ordering::Relaxed);
        self.confirm_key_successes
            .store(security.confirm_key_successes, Ordering::Relaxed);
        self.confirm_key_rejections
            .store(security.confirm_key_rejections, Ordering::Relaxed);
        self.last_confirm_key_status.store(
            u32::from(security.last_confirm_key_status),
            Ordering::Relaxed,
        );
        self.last_confirm_key_type
            .store(u32::from(security.last_confirm_key_type), Ordering::Relaxed);
        self.last_confirm_key_key_identifier.store(
            u32::from(security.last_confirm_key_key_identifier),
            Ordering::Relaxed,
        );
        let security_flags = u32::from(security.last_confirm_key_aps_secured)
            | (u32::from(security.last_confirm_key_nwk_secured) << 1);
        self.last_confirm_key_security
            .store(security_flags, Ordering::Relaxed);
        self.last_confirm_key_source.store(
            u32::from(security.last_confirm_key_source),
            Ordering::Relaxed,
        );
        self.store_ieee(
            &self.last_confirm_key_source_ieee_low,
            &self.last_confirm_key_source_ieee_high,
            security.last_confirm_key_source_ieee,
        );
        self.store_ieee(
            &self.last_confirm_key_destination_low,
            &self.last_confirm_key_destination_high,
            security.last_confirm_key_destination,
        );
    }

    fn capture_interview(&self, device: &ZigbeeDevice<TelinkMac, Router>) {
        let diagnostics = device.zdo_diagnostics();
        let elapsed_ms = self.elapsed_ms(device.mac().monotonic_micros());
        if diagnostics.node_desc_requests != 0 {
            self.store_first(&self.first_node_desc_ms, elapsed_ms);
        }
        if diagnostics.active_ep_requests != 0 {
            self.store_first(&self.first_active_ep_ms, elapsed_ms);
        }
        if diagnostics.simple_desc_requests != 0 {
            self.store_first(&self.first_simple_desc_ms, elapsed_ms);
        }
        self.zdo_response_attempts
            .store(diagnostics.response_attempts, Ordering::Relaxed);
        self.zdo_response_successes
            .store(diagnostics.response_successes, Ordering::Relaxed);
        self.zdo_response_failures
            .store(diagnostics.response_failures, Ordering::Relaxed);
    }

    fn mark_commissioning_complete(&self, device: &ZigbeeDevice<TelinkMac, Router>) {
        let elapsed_ms = self.elapsed_ms(device.mac().monotonic_micros());
        self.store_first(&self.commissioning_complete_ms, elapsed_ms);
    }

    fn record_rx(&self) {
        let frames = self.rx_frames.load(Ordering::Relaxed);
        self.rx_frames
            .store(frames.wrapping_add(1), Ordering::Relaxed);
    }

    fn elapsed_ms(&self, now_us: u32) -> u32 {
        now_us.wrapping_sub(self.attempt_started_us.load(Ordering::Relaxed)) / 1_000
    }

    fn store_milestone(&self, field: &AtomicU32, started_us: u32, milestone_us: u32) {
        if milestone_us != 0 {
            field.store(
                milestone_us.wrapping_sub(started_us) / 1_000,
                Ordering::Relaxed,
            );
        }
    }

    fn store_first(&self, field: &AtomicU32, elapsed_ms: u32) {
        if field.load(Ordering::Relaxed) == 0 {
            field.store(elapsed_ms.max(1), Ordering::Relaxed);
        }
    }

    fn store_ieee(&self, low: &AtomicU32, high: &AtomicU32, address: [u8; 8]) {
        low.store(
            u32::from_le_bytes([address[0], address[1], address[2], address[3]]),
            Ordering::Relaxed,
        );
        high.store(
            u32::from_le_bytes([address[4], address[5], address[6], address[7]]),
            Ordering::Relaxed,
        );
    }
}

#[used]
#[unsafe(no_mangle)]
static TELINK_JOIN_METRICS: JoinMetrics = JoinMetrics::new();

enum LoopControl {
    Continue,
    Recommission,
    Fatal,
}

async fn apply_stack_event<M, S, P, R>(
    node: &mut ZigbeeNode<'_, M, S, P, R>,
    event: StackEvent,
) -> LoopControl
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    R: zigbee_runtime::role::DeviceRole,
{
    match event {
        StackEvent::RejoinRequested => match node.secure_rejoin().await {
            Ok(_) => LoopControl::Continue,
            Err(StartError::PersistenceFailed(_)) => LoopControl::Fatal,
            Err(_) => LoopControl::Recommission,
        },
        StackEvent::LeaveRequested | StackEvent::FactoryResetRequested => {
            match node.factory_reset().await {
                Ok(()) => LoopControl::Recommission,
                Err(_) => LoopControl::Fatal,
            }
        }
        StackEvent::Left | StackEvent::CommissioningComplete { success: false } => {
            LoopControl::Recommission
        }
        _ => LoopControl::Continue,
    }
}

fn failure(leds: &StatusLeds) -> ! {
    leds.green.write(false);
    leds.blue.write(false);
    leds.red.write(true);
    loop {
        tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(1_000));
    }
}

pub fn run() -> ! {
    type Device = ZigbeeDevice<TelinkMac, Router>;

    tlsr8258_hal::timer::init();
    let resources = match BoardResources::take() {
        Some(resources) => resources,
        None => loop {
            core::hint::spin_loop();
        },
    };
    let leds = resources.lighting.into_status_leds();
    if leds.init().is_err() {
        failure(&leds);
    }
    let adc = match tlsr8258_hal::adc::Adc::new(
        resources.adc,
        tlsr8258_hal::flash::FlashGeometry::KiB512,
    ) {
        Ok(adc) => adc,
        Err(_) => failure(&leds),
    };
    if adc.install_flash_voltage_guard(resources.adc_pc5).is_err() {
        failure(&leds);
    }

    let mut ieee_address = [0u8; 8];
    tlsr8258_hal::flash::factory_ieee(&mut ieee_address);
    ieee_address[0] = ieee_address[0].wrapping_add(DEVICE_EUI_OFFSET);
    let mac = TelinkMac::with_extended_address(ieee_address);

    static mut DEVICE_STORAGE: MaybeUninit<Device> = MaybeUninit::uninit();
    let mut profile = DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::RANGE_EXTENDER,
        RangeExtender,
    );

    let device = ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer("Zigbee-RS")
        .model("TLSR8258-Router")
        .date_code("20260718")
        .sw_build("0.1.0")
        .power_source(PowerSource::MainsSinglePhase)
        .channels(zigbee_types::ChannelMask(1 << 15))
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_router_into(unsafe { &mut *core::ptr::addr_of_mut!(DEVICE_STORAGE) });

    let mut security_store = tlsr8258_tb04_product::storage::security_store(resources.flash);
    if device
        .reset_security_state_if_identity_changed(&mut security_store, ieee_address)
        .is_err()
    {
        failure(&leds);
    }
    let mut node = ZigbeeNode::new(device, &mut security_store, &mut profile);

    // Single root future for the whole firmware: start/resume with join-retry
    // backoff, MAX_RX_SLICE_US-bounded receive windows, `process_incoming`,
    // parent command servicing, tick/`RunAgain` scheduling, metrics timestamps,
    // LED states, and factory-reset/rejoin all run inside this one future so
    // `tlsr8258_rt::block_on` is monomorphized exactly once. Synchronous chip,
    // ADC, LED, MAC, device, and node initialization above stays outside the
    // future. This never returns (`Output = !`).
    let app = async move {
        'commission: loop {
            leds.green.write(false);
            leds.blue.write(false);
            leds.red.write(true);

            let mut retry_delay_ms = JOIN_RETRY_MIN_MS;
            loop {
                TELINK_JOIN_METRICS.begin_attempt(node.device().mac().monotonic_micros());
                match node.start_or_resume().await {
                    Ok(_) => break,
                    Err(StartError::CommissioningFailed(_)) => {
                        let diagnostics = node.device().steering_diagnostics();
                        let announce_exhausted = diagnostics.device_annce_exhausted();
                        TELINK_JOIN_METRICS.capture_steering(node.device());
                        if announce_exhausted && node.factory_reset().await.is_err() {
                            failure(&leds);
                        }
                        tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(retry_delay_ms));
                        retry_delay_ms = retry_delay_ms.saturating_mul(2).min(JOIN_RETRY_MAX_MS);
                    }
                    Err(_) => failure(&leds),
                }
            }
            TELINK_JOIN_METRICS.capture_steering(node.device());

            // Solid green = joined and relaying. Unlike the sensor runtime,
            // there is no battery/sensor state to report — this LED state is
            // the entire "am I alive and joined" signal for the router.
            leds.red.write(false);
            leds.green.write(true);
            leds.blue.write(false);

            let mut identify_elapsed = 0u32;
            let one_second = tlsr8258_hal::timer::ms(1_000);
            let mut tick_anchor = tlsr8258_hal::timer::now_ticks();
            let mut rx_slice_us = MAX_RX_SLICE_US;

            loop {
                let mut event = None;
                match node.device_mut().receive_timeout(rx_slice_us).await {
                    Ok(indication) => {
                        TELINK_JOIN_METRICS.record_rx();
                        match node.process_incoming(&indication).await {
                            Ok(stack_event) => event = stack_event,
                            Err(_) => failure(&leds),
                        }
                    }
                    Err(MacError::NoData) => {}
                    Err(_) => failure(&leds),
                }
                TELINK_JOIN_METRICS.capture_interview(node.device());

                if let Some(stack_event) = event {
                    if matches!(
                        stack_event,
                        StackEvent::CommissioningComplete { success: true }
                    ) {
                        TELINK_JOIN_METRICS.mark_commissioning_complete(node.device());
                    }
                    match apply_stack_event(&mut node, stack_event).await {
                        LoopControl::Continue => {}
                        LoopControl::Recommission => continue 'commission,
                        LoopControl::Fatal => failure(&leds),
                    }
                }

                let now = tlsr8258_hal::timer::now_ticks();
                let elapsed = now.wrapping_sub(tick_anchor);
                let elapsed_secs = if elapsed >= one_second {
                    let elapsed_secs = (elapsed / one_second).min(u16::MAX as u32) as u16;
                    tick_anchor = tick_anchor.wrapping_add(u32::from(elapsed_secs) * one_second);
                    identify_elapsed = identify_elapsed.wrapping_add(u32::from(elapsed_secs));
                    elapsed_secs
                } else {
                    0
                };

                match node.tick(elapsed_secs).await {
                    Ok(TickResult::Idle) => rx_slice_us = MAX_RX_SLICE_US,
                    Ok(TickResult::RunAgain(delay_ms)) => {
                        rx_slice_us = delay_ms.max(1).saturating_mul(1_000).min(MAX_RX_SLICE_US);
                    }
                    Ok(TickResult::Event(stack_event)) => {
                        rx_slice_us = MAX_RX_SLICE_US;
                        if matches!(
                            stack_event,
                            StackEvent::CommissioningComplete { success: true }
                        ) {
                            TELINK_JOIN_METRICS.mark_commissioning_complete(node.device());
                        }
                        match apply_stack_event(&mut node, stack_event).await {
                            LoopControl::Continue => {}
                            LoopControl::Recommission => continue 'commission,
                            LoopControl::Fatal => failure(&leds),
                        }
                    }
                    Err(_) => failure(&leds),
                }
                TELINK_JOIN_METRICS.capture_steering(node.device());

                if node.device().is_identifying(ENDPOINT) {
                    leds.blue.write((identify_elapsed & 1) == 0);
                } else {
                    leds.blue.write(false);
                }
            }
        }
    };
    tlsr8258_rt::block_on(app)
}
