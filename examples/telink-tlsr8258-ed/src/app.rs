//! Always-on TLSR8258 End Device application.
//!
//! The shared `router_app::AlwaysOnEndDeviceApp` owns commissioning, secured
//! rejoin, bounded receive/tick processing, security checkpoints and factory
//! reset. This file only composes the product identity, the end-device MAC,
//! the product-owned security journal and the TB-04 LEDs, and publishes a
//! small retained counter block for SWire memory reads.

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU32, Ordering};

use router_app::{
    AlwaysOnEndDeviceApp, NoDiagnostics, RouterAppError, RouterObserver, RouterParts,
};
use zigbee_mac::telink::TelinkMac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_runtime::role::EndDevice;
use zigbee_zcl::clusters::basic::PowerSource;

use tlsr8258_tb04::leds::StatusLeds;
use tlsr8258_tb04_product::boot::{Boot, boot, failure, halted};
use tlsr8258_tb04_product::end_device::{
    DATE_CODE, DEVICE_EUI_OFFSET, END_DEVICE_MODEL, END_DEVICE_POLICY, EndDeviceProfile,
    MANUFACTURER, SW_BUILD, end_device_profile,
};
use tlsr8258_tb04_product::status_leds::led_adapters;

/// Debugger ABI of `TELINK_ED_METRICS` (all fields little-endian `u32`,
/// offsets fixed by declaration order). Written from task context only: on
/// stack callbacks and, for the radio block, once per one-second stack tick.
#[repr(C)]
struct EdMetrics {
    /// `0x4445_4A54` ("TJED").
    magic: AtomicU32,
    commissioning_attempts: AtomicU32,
    /// `0x1_0000 | short` on success, `0xE00k_00ss` on failure (see
    /// [`encode_start`]); 0 before the first result.
    last_start_result: AtomicU32,
    secure_rejoin_results: AtomicU32,
    last_secure_rejoin_result: AtomicU32,
    network_ready_count: AtomicU32,
    /// `pan_id << 16 | short_address` at the last network-ready callback.
    network_identity: AtomicU32,
    commissioning_complete_count: AtomicU32,
    rx_frames: AtomicU32,
    faults: AtomicU32,
    /// Seconds of stack time accumulated from the one-second ticks.
    uptime_secs: AtomicU32,
    radio_rx_frames_valid: AtomicU32,
    radio_rx_invalid_length: AtomicU32,
    radio_rx_invalid_crc: AtomicU32,
    radio_rx_dma_incomplete: AtomicU32,
    radio_rx_queue_overflow: AtomicU32,
    radio_rx_queue_evicted: AtomicU32,
    radio_rx_queue_high_water: AtomicU32,
    radio_rx_serviced_irq: AtomicU32,
    radio_rx_serviced_polled: AtomicU32,
}

impl EdMetrics {
    const fn new() -> Self {
        Self {
            magic: AtomicU32::new(0x4445_4A54),
            commissioning_attempts: AtomicU32::new(0),
            last_start_result: AtomicU32::new(0),
            secure_rejoin_results: AtomicU32::new(0),
            last_secure_rejoin_result: AtomicU32::new(0),
            network_ready_count: AtomicU32::new(0),
            network_identity: AtomicU32::new(0),
            commissioning_complete_count: AtomicU32::new(0),
            rx_frames: AtomicU32::new(0),
            faults: AtomicU32::new(0),
            uptime_secs: AtomicU32::new(0),
            radio_rx_frames_valid: AtomicU32::new(0),
            radio_rx_invalid_length: AtomicU32::new(0),
            radio_rx_invalid_crc: AtomicU32::new(0),
            radio_rx_dma_incomplete: AtomicU32::new(0),
            radio_rx_queue_overflow: AtomicU32::new(0),
            radio_rx_queue_evicted: AtomicU32::new(0),
            radio_rx_queue_high_water: AtomicU32::new(0),
            radio_rx_serviced_irq: AtomicU32::new(0),
            radio_rx_serviced_polled: AtomicU32::new(0),
        }
    }

    fn bump(field: &AtomicU32) {
        field.store(
            field.load(Ordering::Relaxed).wrapping_add(1),
            Ordering::Relaxed,
        );
    }

    fn capture_radio(&self) {
        let rx = tlsr8258_hal::radio::rx_diagnostics();
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
    }
}

fn encode_start(result: Result<u16, StartError>) -> u32 {
    match result {
        Ok(short) => 0x1_0000 | u32::from(short),
        Err(StartError::InitFailed) => 0xE001_0000,
        Err(StartError::CommissioningFailed(status)) => 0xE002_0000 | status as u32,
        Err(StartError::PersistenceFailed(_)) => 0xE003_0000,
    }
}

#[used]
#[unsafe(no_mangle)]
static TELINK_ED_METRICS: EdMetrics = EdMetrics::new();

type Device = ZigbeeDevice<TelinkMac, EndDevice>;

/// Zero-storage static observer feeding `TELINK_ED_METRICS`.
struct EdObserver;

impl RouterObserver<TelinkMac, EndDevice> for EdObserver {
    fn on_commissioning_attempt(_device: &Device, _attempt: u32, _started_us: u32) {
        EdMetrics::bump(&TELINK_ED_METRICS.commissioning_attempts);
    }

    fn on_start_result(_device: &Device, result: Result<u16, StartError>) {
        TELINK_ED_METRICS
            .last_start_result
            .store(encode_start(result), Ordering::Relaxed);
    }

    fn on_secure_rejoin_result(_device: &Device, result: Result<u16, StartError>) {
        EdMetrics::bump(&TELINK_ED_METRICS.secure_rejoin_results);
        TELINK_ED_METRICS
            .last_secure_rejoin_result
            .store(encode_start(result), Ordering::Relaxed);
    }

    fn on_network_ready(device: &Device) {
        EdMetrics::bump(&TELINK_ED_METRICS.network_ready_count);
        TELINK_ED_METRICS.network_identity.store(
            (u32::from(device.pan_id()) << 16) | u32::from(device.short_address()),
            Ordering::Relaxed,
        );
    }

    fn on_frame_received(_device: &Device, _receive_elapsed_us: u32) {
        EdMetrics::bump(&TELINK_ED_METRICS.rx_frames);
    }

    fn on_stack_event(_device: &Device, event: &StackEvent) {
        if matches!(event, StackEvent::CommissioningComplete { success: true }) {
            EdMetrics::bump(&TELINK_ED_METRICS.commissioning_complete_count);
        }
    }

    fn on_tick(_device: &Device, elapsed_secs: u16, _result: &TickResult) {
        if elapsed_secs != 0 {
            let uptime = TELINK_ED_METRICS.uptime_secs.load(Ordering::Relaxed);
            TELINK_ED_METRICS.uptime_secs.store(
                uptime.wrapping_add(u32::from(elapsed_secs)),
                Ordering::Relaxed,
            );
            // Bounded snapshot on the one-second tick, never per frame.
            TELINK_ED_METRICS.capture_radio();
        }
    }

    fn on_fault(_device: &Device, _error: RouterAppError) {
        EdMetrics::bump(&TELINK_ED_METRICS.faults);
    }
}

/// Build the MAC and the End Device straight into static storage.
///
/// Kept out of [`run`] so the by-value MAC and the builder are temporaries
/// of this short-lived frame rather than of the frame that stays live under
/// `block_on`.
#[inline(never)]
fn build_device(
    ieee_address: zigbee_types::IeeeAddress,
    aes: tlsr8258_hal::peripherals::Aes,
    leds: &StatusLeds,
    profile: &mut EndDeviceProfile,
) -> &'static mut Device {
    let Ok(mut mac) = TelinkMac::with_extended_address(ieee_address) else {
        failure(leds);
    };
    if mac.install_aes_engine(aes).is_err() {
        failure(leds);
    }

    static mut DEVICE_STORAGE: MaybeUninit<Device> = MaybeUninit::uninit();
    ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer(MANUFACTURER)
        .model(END_DEVICE_MODEL)
        .date_code(DATE_CODE)
        .sw_build(SW_BUILD)
        .power_source(PowerSource::MainsSinglePhase)
        .channels(zigbee_types::ChannelMask(1 << 15))
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_into(unsafe { &mut *core::ptr::addr_of_mut!(DEVICE_STORAGE) })
}

/// Board bring-up in its own short frame; measured smaller in flash and
/// stack for this image than inlining it into [`run`].
#[inline(never)]
fn bring_up() -> Boot {
    boot(DEVICE_EUI_OFFSET)
}

pub fn run() -> ! {
    let board = bring_up();
    let mut profile = end_device_profile();
    let device = build_device(board.ieee_address, board.aes, &board.leds, &mut profile);

    // The End Device owns only the security journal; the APS and child
    // partitions stay reserved (and untouched) for the router image.
    let (security_partition, _child_partition, _aps_partition) =
        tlsr8258_tb04_product::storage::split_flash(board.flash);
    let mut security_store = tlsr8258_tb04_product::storage::security_store(security_partition);
    // Before ZigbeeNode construction: persisted keys/counters may never be
    // resumed under a different EUI-64, nor by a different role image.
    if device
        .reset_security_state_if_identity_changed(&mut security_store, board.ieee_address)
        .is_err()
    {
        failure(&board.leds);
    }
    if security_store
        .select_role_tag(tlsr8258_tb04_product::role::END_DEVICE)
        .is_err()
    {
        failure(&board.leds);
    }
    let node = ZigbeeNode::new(device, &mut security_store, &mut profile);
    let (status, supervisor) = led_adapters(board.leds);
    let parts = RouterParts::new(status, supervisor, NoDiagnostics);
    let mut app = match AlwaysOnEndDeviceApp::<_, _, _, _, _, _, EdObserver>::new_observed(
        node,
        &END_DEVICE_POLICY,
        parts,
    ) {
        Ok(app) => app,
        // The LEDs were initialized red before ownership moved into the
        // adapters, so constructor failure is still fail-closed.
        Err(_) => halted(),
    };

    // The only root future and `block_on` monomorphization in the image.
    tlsr8258_rt::block_on(app.run())
}
