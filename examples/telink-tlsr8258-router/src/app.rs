//! Always-on TLSR8258 parent-router application.
//!
//! Child state is restored from a product-owned crash-safe journal before the
//! router answers orphan notifications or schedules Parent Announce.

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU32, Ordering};

use router_app::{NoDiagnostics, ParentRouterApp, PersistentChildren, RouterObserver, RouterParts};
use zigbee_mac::{PlatformServices, telink::TelinkMac};
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_runtime::role::Router;
use zigbee_zcl::clusters::basic::PowerSource;

use tlsr8258_tb04::{leds::StatusLeds, resources::BoardResources};
use tlsr8258_tb04_product::router::{ROUTER_POLICY, led_adapters, range_extender_profile};

// Distinct from the sensor runtime's `DEVICE_EUI_OFFSET` (0x33) so a router
// and a sensor built from the same factory-programmed part never collide on
// IEEE address if someone reflashes one board with both images over time.
const DEVICE_EUI_OFFSET: u8 = 0x52; // 'R' for Router

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
    // Radio receive/acknowledgement diagnostics. Appended at the end so
    // the offsets of every field above stay stable for existing RAM dumps.
    // Snapshotted once per second, never per frame, so reading them cannot
    // perturb the turnaround-critical radio path.
    radio_rx_frames_valid: AtomicU32,
    radio_rx_invalid_length: AtomicU32,
    radio_rx_invalid_crc: AtomicU32,
    radio_rx_dma_incomplete: AtomicU32,
    radio_rx_queue_overflow: AtomicU32,
    radio_rx_serviced_irq: AtomicU32,
    radio_rx_serviced_polled: AtomicU32,
    mac_tx_attempts: AtomicU32,
    mac_ack_windows: AtomicU32,
    mac_ack_matched: AtomicU32,
    mac_ack_windows_expired: AtomicU32,
    mac_ack_frames_seen: AtomicU32,
    mac_foreign_acks: AtomicU32,
    mac_ack_window_frames_seen: AtomicU32,
    /// `0x100 | dsn` when an unmatched ACK has been seen, else `0`.
    mac_last_foreign_ack_sequence: AtomicU32,
    /// `0x100 | dsn` for the most recent expired ACK window, else `0`.
    mac_last_expired_sequence: AtomicU32,
    // Receive-queue overload accounting. Appended after the existing radio
    // block so every offset above stays stable for RAM dumps taken from the
    // previous firmware.
    //
    // `radio_rx_queue_overflow` keeps its original meaning (total frames
    // lost at the HAL interrupt queue). `radio_rx_queue_evicted` is the
    // subset of those losses where a lower-priority queued frame was
    // sacrificed to keep a more valuable newly arrived frame, so
    // `overflow - evicted` is the count of arrivals dropped outright and is
    // the number that has to stay at zero. `radio_rx_queue_high_water` is
    // what makes the queue capacity a measurement rather than a guess.
    radio_rx_queue_evicted: AtomicU32,
    radio_rx_queue_high_water: AtomicU32,
    // The MAC's own bounded queues, counted separately from the HAL's so a
    // loss is attributable to exactly one stage.
    mac_data_queue_overflow: AtomicU32,
    mac_data_queue_evicted: AtomicU32,
    mac_data_queue_high_water: AtomicU32,
    mac_command_queue_overflow: AtomicU32,
    mac_command_queue_high_water: AtomicU32,
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
            radio_rx_frames_valid: AtomicU32::new(0),
            radio_rx_invalid_length: AtomicU32::new(0),
            radio_rx_invalid_crc: AtomicU32::new(0),
            radio_rx_dma_incomplete: AtomicU32::new(0),
            radio_rx_queue_overflow: AtomicU32::new(0),
            radio_rx_serviced_irq: AtomicU32::new(0),
            radio_rx_serviced_polled: AtomicU32::new(0),
            mac_tx_attempts: AtomicU32::new(0),
            mac_ack_windows: AtomicU32::new(0),
            mac_ack_matched: AtomicU32::new(0),
            mac_ack_windows_expired: AtomicU32::new(0),
            mac_ack_frames_seen: AtomicU32::new(0),
            mac_foreign_acks: AtomicU32::new(0),
            mac_ack_window_frames_seen: AtomicU32::new(0),
            mac_last_foreign_ack_sequence: AtomicU32::new(0),
            mac_last_expired_sequence: AtomicU32::new(0),
            radio_rx_queue_evicted: AtomicU32::new(0),
            radio_rx_queue_high_water: AtomicU32::new(0),
            mac_data_queue_overflow: AtomicU32::new(0),
            mac_data_queue_evicted: AtomicU32::new(0),
            mac_data_queue_high_water: AtomicU32::new(0),
            mac_command_queue_overflow: AtomicU32::new(0),
            mac_command_queue_high_water: AtomicU32::new(0),
        }
    }

    /// Snapshot the HAL receive-path and MAC acknowledgement counters.
    ///
    /// These are cumulative since boot and are deliberately *not* cleared by
    /// `begin_attempt`: a retransmission defect has to be attributable
    /// across rejoins, and the whole point is to compare the totals against
    /// one sniffer capture covering the same interval.
    fn capture_radio(&self, mac: &TelinkMac) {
        let rx = mac.rx_diagnostics();
        self.radio_rx_frames_valid
            .store(rx.frames_valid, Ordering::Relaxed);
        self.radio_rx_invalid_length
            .store(rx.invalid_length, Ordering::Relaxed);
        self.radio_rx_invalid_crc
            .store(rx.invalid_crc, Ordering::Relaxed);
        self.radio_rx_dma_incomplete
            .store(rx.dma_incomplete, Ordering::Relaxed);
        self.radio_rx_queue_overflow
            .store(rx.queue_overflow, Ordering::Relaxed);
        self.radio_rx_queue_evicted
            .store(rx.queue_evicted, Ordering::Relaxed);
        self.radio_rx_queue_high_water
            .store(u32::from(rx.queue_high_water), Ordering::Relaxed);
        self.radio_rx_serviced_irq
            .store(rx.serviced_irq, Ordering::Relaxed);
        self.radio_rx_serviced_polled
            .store(rx.serviced_polled, Ordering::Relaxed);

        let queues = mac.queue_diagnostics();
        self.mac_data_queue_overflow
            .store(queues.data_indication_overflow, Ordering::Relaxed);
        self.mac_data_queue_evicted
            .store(queues.data_indication_evicted, Ordering::Relaxed);
        self.mac_data_queue_high_water.store(
            u32::from(queues.data_indication_high_water),
            Ordering::Relaxed,
        );
        self.mac_command_queue_overflow
            .store(queues.command_event_overflow, Ordering::Relaxed);
        self.mac_command_queue_high_water.store(
            u32::from(queues.command_event_high_water),
            Ordering::Relaxed,
        );

        let ack = mac.ack_diagnostics();
        self.mac_tx_attempts
            .store(ack.tx_attempts, Ordering::Relaxed);
        self.mac_ack_windows
            .store(ack.ack_windows, Ordering::Relaxed);
        self.mac_ack_matched
            .store(ack.ack_matched, Ordering::Relaxed);
        self.mac_ack_windows_expired
            .store(ack.ack_windows_expired, Ordering::Relaxed);
        self.mac_ack_frames_seen
            .store(ack.ack_frames_seen, Ordering::Relaxed);
        self.mac_foreign_acks
            .store(ack.foreign_acks, Ordering::Relaxed);
        self.mac_ack_window_frames_seen
            .store(ack.window_frames_seen, Ordering::Relaxed);
        self.mac_last_foreign_ack_sequence.store(
            ack.last_foreign_ack_sequence
                .map_or(0, |dsn| 0x100 | u32::from(dsn)),
            Ordering::Relaxed,
        );
        self.mac_last_expired_sequence.store(
            ack.last_expired_sequence
                .map_or(0, |dsn| 0x100 | u32::from(dsn)),
            Ordering::Relaxed,
        );
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

/// Zero-storage static observer that preserves the debugger ABI of
/// `TELINK_JOIN_METRICS` while the shared app owns the lifecycle.
struct TelinkJoinObserver;

impl RouterObserver<TelinkMac, Router> for TelinkJoinObserver {
    fn on_commissioning_attempt(
        _device: &ZigbeeDevice<TelinkMac, Router>,
        _attempt: u32,
        started_us: u32,
    ) {
        TELINK_JOIN_METRICS.begin_attempt(started_us);
    }

    fn on_start_result(device: &ZigbeeDevice<TelinkMac, Router>, result: Result<u16, StartError>) {
        if matches!(result, Ok(_) | Err(StartError::CommissioningFailed(_))) {
            TELINK_JOIN_METRICS.capture_steering(device);
        }
    }

    fn on_secure_rejoin_result(
        device: &ZigbeeDevice<TelinkMac, Router>,
        _result: Result<u16, StartError>,
    ) {
        TELINK_JOIN_METRICS.capture_steering(device);
    }

    fn on_network_ready(device: &ZigbeeDevice<TelinkMac, Router>) {
        // ParentRouterApp calls this only after the durable child table has
        // been restored, so no observer work can open parent traffic early.
        TELINK_JOIN_METRICS.capture_steering(device);
    }

    fn on_frame_received(_device: &ZigbeeDevice<TelinkMac, Router>, _receive_elapsed_us: u32) {
        TELINK_JOIN_METRICS.record_rx();
    }

    fn on_frame_processed(
        device: &ZigbeeDevice<TelinkMac, Router>,
        _event: Option<&StackEvent>,
        _elapsed_us: u32,
    ) {
        TELINK_JOIN_METRICS.capture_interview(device);
    }

    fn on_stack_event(device: &ZigbeeDevice<TelinkMac, Router>, event: &StackEvent) {
        if matches!(event, StackEvent::CommissioningComplete { success: true }) {
            TELINK_JOIN_METRICS.mark_commissioning_complete(device);
        }
    }

    fn on_tick(device: &ZigbeeDevice<TelinkMac, Router>, elapsed_secs: u16, _result: &TickResult) {
        TELINK_JOIN_METRICS.capture_interview(device);
        TELINK_JOIN_METRICS.capture_steering(device);
        if elapsed_secs != 0 {
            // Bounded snapshot only on the existing one-second stack tick,
            // never in the turnaround-critical IRQ or per-frame path.
            TELINK_JOIN_METRICS.capture_radio(device.mac());
        }
    }
}

fn failure(leds: &StatusLeds) -> ! {
    leds.green.write(false);
    leds.blue.write(false);
    leds.red.write(true);
    halted()
}

fn halted() -> ! {
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
    let mut mac = TelinkMac::with_extended_address(ieee_address);
    if mac.install_aes_engine(resources.aes).is_err() {
        failure(&leds);
    }

    static mut DEVICE_STORAGE: MaybeUninit<Device> = MaybeUninit::uninit();
    let mut profile = range_extender_profile();

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

    let (security_partition, child_partition) =
        tlsr8258_tb04_product::storage::split_flash(resources.flash);
    let mut security_store = tlsr8258_tb04_product::storage::security_store(security_partition);
    // Product-owned durable child table, on its own two flash sectors. The
    // runtime owns the record format and restore semantics; the product owns
    // where the bytes live and when they are written.
    let child_store = tlsr8258_tb04_product::storage::child_table_store(child_partition);
    if device
        .reset_security_state_if_identity_changed(&mut security_store, ieee_address)
        .is_err()
    {
        failure(&leds);
    }
    let node = ZigbeeNode::new(device, &mut security_store, &mut profile);
    let children = PersistentChildren::new(child_store);
    let (status, supervisor) = led_adapters(leds);
    let parts = RouterParts::new(status, supervisor, NoDiagnostics);
    let mut app = match ParentRouterApp::<_, _, _, _, _, _, _, TelinkJoinObserver>::new_observed(
        node,
        children,
        &ROUTER_POLICY,
        parts,
    ) {
        Ok(app) => app,
        // The LEDs were initialized red before ownership moved into the
        // adapters. Constructor failure is therefore still fail-closed.
        Err(_) => halted(),
    };

    // The shared parent app now owns commissioning, child restore/persist/
    // clear, bounded receive, tick, rejoin, and retry behavior. The device
    // itself remains in caller-owned static storage, and this is the only
    // root future/`block_on` monomorphization in the firmware.
    tlsr8258_rt::block_on(app.run())
}
