//! Minimal 802.15.4 packet sniffer for nRF52840-DK.
//! Listens on all channels and prints received frames via defmt/RTT.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_nrf as _;
use embassy_time::{Duration, Timer};

use zigbee_mac::nrf::NrfMac;
use zigbee_mac::MacDriver;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    defmt::info!("nRF52840 Sniffer starting...");

    let mut mac = NrfMac::new(p);

    // Scan each channel for energy and beacons
    for ch in 11u8..=26 {
        mac.set_channel(ch);
        
        // Listen for 500ms on this channel
        defmt::info!("Listening on channel {}...", ch);
        
        let deadline = embassy_time::Instant::now() + Duration::from_millis(500);
        loop {
            let now = embassy_time::Instant::now();
            if now >= deadline { break; }
            let remaining = deadline - now;

            let result = embassy_futures::select::select(
                mac.receive_raw(),
                Timer::after(remaining),
            ).await;

            match result {
                embassy_futures::select::Either::First(Ok(frame)) => {
                    defmt::info!("CH{} RX {} bytes: {:02X}", 
                        ch, frame.len, &frame.data[..frame.len.min(32)]);
                }
                embassy_futures::select::Either::First(Err(_)) => {
                    // CRC error or other — still interesting
                    defmt::warn!("CH{} RX error", ch);
                }
                embassy_futures::select::Either::Second(_) => break,
            }
        }
    }

    defmt::info!("Scan complete. Entering continuous listen on ch15...");
    mac.set_channel(15);
    
    loop {
        match mac.receive_raw().await {
            Ok(frame) => {
                defmt::info!("RX {} bytes rssi={}: {:02X}", 
                    frame.len, frame.rssi, &frame.data[..frame.len.min(40)]);
            }
            Err(_) => {}
        }
    }
}
