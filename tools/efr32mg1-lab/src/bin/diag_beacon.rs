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
use embassy_time::{Duration, Timer};
use zigbee_mac::MacDriver;
use zigbee_mac::efr32::Efr32Mac;
use zigbee_mac::frames::build_beacon_request;
use zigbee_mac::pib::{PibAttribute, PibValue};
use zigbee_mac::primitives::{MlmeScanRequest, ScanType};
use zigbee_types::ChannelMask;

const CHANNEL: u8 = 15;
const DURATION: u8 = 4;
const TEST_SEQ: u8 = 0xA5;

struct RttLogger;
static LOGGER: RttLogger = RttLogger;

impl log::Log for RttLogger {
    fn enabled(&self, _metadata: &log::Metadata<'_>) -> bool {
        true
    }

    fn log(&self, record: &log::Record<'_>) {
        rtt_target::rprintln!("[{}] {}", record.level(), record.args());
    }

    fn flush(&self) {}
}

async fn known_tx(mac: &mut Efr32Mac) -> bool {
    if let Err(error) = mac
        .mlme_set(PibAttribute::PhyCurrentChannel, PibValue::U8(CHANNEL))
        .await
    {
        rtt_target::rprintln!("[EFR32][diag-beacon] tx-only channel-set=err {:?}", error);
        return false;
    }
    let request = build_beacon_request(TEST_SEQ);
    rtt_target::rprintln!(
        "[EFR32][diag-beacon] TX_ONLY_BEGIN raw-beacon-req ch={} len={} seq={}",
        CHANNEL,
        request.len(),
        TEST_SEQ
    );
    match mac.debug_transmit_raw(&request).await {
        Ok(()) => {
            rtt_target::rprintln!("[EFR32][diag-beacon] TX_ONLY_PASS scan_gate=open");
            true
        }
        Err(error) => {
            rtt_target::rprintln!(
                "[EFR32][diag-beacon] TX_ONLY_FAIL {:?} scan_gate=closed",
                error
            );
            false
        }
    }
}

#[embassy_executor::task]
async fn run(mut mac: Efr32Mac) -> ! {
    for _ in 0..3 {
        platform::led_on();
        Timer::after(Duration::from_millis(100)).await;
        platform::led_off();
        Timer::after(Duration::from_millis(100)).await;
    }
    Timer::after(Duration::from_secs(5)).await;
    mac.debug_radio_snapshot("init");

    if !known_tx(&mut mac).await {
        platform::led_on();
        loop {
            rtt_target::rprintln!(
                "[EFR32][diag-beacon] ACTIVE_SCAN_BLOCKED tx-only did not complete"
            );
            Timer::after(Duration::from_secs(5)).await;
        }
    }

    let channel_mask = ChannelMask(1 << CHANNEL);
    rtt_target::rprintln!(
        "[EFR32][diag-beacon] ACTIVE_SCAN_ENABLED mask={:#010X}",
        channel_mask.0
    );
    loop {
        mac.debug_radio_snapshot("before-scan");
        match mac
            .mlme_scan(MlmeScanRequest {
                scan_type: ScanType::Active,
                channel_mask,
                scan_duration: DURATION,
            })
            .await
        {
            Ok(confirm) => {
                mac.debug_radio_snapshot("after-scan");
                rtt_target::rprintln!(
                    "[EFR32][diag-beacon] Scan done: {} beacon(s)",
                    confirm.pan_descriptors.len()
                );
                platform::led_off();
                for pan in confirm.pan_descriptors.iter() {
                    rtt_target::rprintln!(
                        "[EFR32][diag-beacon] beacon: ch={} coord={:?} lqi={} permit={} \
                         depth={} router_cap={} enddev_cap={}",
                        pan.channel,
                        pan.coord_address,
                        pan.lqi,
                        pan.superframe_spec.association_permit,
                        pan.zigbee_beacon.device_depth,
                        pan.zigbee_beacon.router_capacity,
                        pan.zigbee_beacon.end_device_capacity
                    );
                    platform::led_on();
                }
            }
            Err(error) => {
                mac.debug_radio_snapshot("scan-error");
                rtt_target::rprintln!("[EFR32][diag-beacon] Scan error: {:?}", error);
            }
        }
        Timer::after(Duration::from_secs(2)).await;
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    platform::init_large!("diag-beacon");
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Trace);
    time_driver::init();
    static EXECUTOR: static_cell::StaticCell<embassy_executor::Executor> =
        static_cell::StaticCell::new();
    EXECUTOR
        .init(embassy_executor::Executor::new())
        .run(|spawner| spawner.must_spawn(run(Efr32Mac::new())))
}
