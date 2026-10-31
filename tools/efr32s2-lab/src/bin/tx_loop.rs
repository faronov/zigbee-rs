//! TX diagnostic: back-to-back maximum-length data frames on `CHANNEL`
//! (no CSMA) so a probe can halt the core mid-transmission and compare the
//! radio registers with a reference TX snapshot, and a second node can
//! verify the frames on air (source PAN 0x5A5A, payload = counter).

#![no_std]
#![no_main]

#[path = "../shared/mod.rs"]
mod shared;

use efr32s2_hal::delay::{cycles, cycles_to_us};

const CHANNEL: u8 = 15;
const FRAME_LEN: usize = 20;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut lab = shared::init("tx-loop");
    let mut radio = lab.radio.init().expect("radio init");
    radio.set_channel(CHANNEL).unwrap();
    let mut f = [0u8; 120];
    // Data frame, no ACK request, PAN-ID compression, short dst/src.
    f[..9].copy_from_slice(&[0x41, 0x88, 0, 0x5A, 0x5A, 0xFF, 0xFF, 0x34, 0x12]);
    let mut n = 0u32;
    loop {
        f[2] = n as u8;
        f[9..13].copy_from_slice(&n.to_le_bytes());
        let t = cycles();
        let r = radio.transmit(&f[..FRAME_LEN]);
        if n.is_multiple_of(256) {
            log!(
                "tx #{} {:?} {} us state {:?}",
                n,
                r,
                cycles_to_us(cycles().wrapping_sub(t)),
                radio.rac_state()
            );
            lab.led.toggle();
        }
        n = n.wrapping_add(1);
    }
}
