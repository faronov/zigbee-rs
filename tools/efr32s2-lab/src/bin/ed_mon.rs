//! RF presence monitor: peak energy on `CHANNEL` per 100 ms window plus the
//! number of frames received in it, so a second node can tell "no carrier
//! at all" from "carrier but undecodable" while a device under test
//! transmits. Frames are drained continuously so the receiver never stalls
//! on a full buffer.

#![no_std]
#![no_main]

#[path = "../shared/mod.rs"]
mod shared;

use efr32s2_hal::radio::MAX_FRAME;

const CHANNEL: u8 = 15;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut lab = shared::init("ed-mon");
    let mut radio = lab.radio.init().expect("radio init");
    radio.set_channel(CHANNEL).unwrap();
    let mut buf = [0u8; MAX_FRAME];
    let mut n = 0u32;
    loop {
        let mut peaks = [-128i8; 10];
        let mut frames = 0u32;
        let mut ours = 0u32;
        let mut last_rssi = 0i8;
        for p in peaks.iter_mut() {
            for _ in 0..100 {
                if let Ok(Some(dbm)) = radio.energy_detect(1000) {
                    *p = (*p).max(dbm);
                }
                while let Some(rx) = radio.receive(&mut buf) {
                    frames += 1;
                    let len = rx.len;
                    if len >= 5 && buf[3] == 0x5A && buf[4] == 0x5A {
                        ours += 1;
                        last_rssi = rx.rssi_dbm;
                    }
                }
            }
        }
        log!(
            "ed #{} {:?} frames {} dut {} rssi {}",
            n,
            peaks,
            frames,
            ours,
            last_rssi
        );
        lab.led.toggle();
        n = n.wrapping_add(1);
    }
}
