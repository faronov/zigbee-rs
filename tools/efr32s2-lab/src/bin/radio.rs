//! Gates 2-4: raw IEEE 802.15.4 on the CPU-driven Series 2 radio.
//!
//! 1. init + per-channel energy detect (11..26);
//! 2. passive RX dump on `CHANNEL` (expects coordinator/router traffic);
//! 3. beacon request TX -> coordinator beacon RX (proves TX without a sniffer);
//! 4. MAC data request to the coordinator with ACK request -> hardware ACK
//!    from the coordinator (ACK reception + ACK-wait timing);
//! 5. continuous RX dump with counters.

#![no_std]
#![no_main]

#[path = "../shared/mod.rs"]
mod shared;

use efr32s2_hal::{
    delay::{Deadline, cycles, cycles_to_us},
    radio::{CsmaConfig, MAX_PSDU, Radio, RadioError, synth_diag},
};

const CHANNEL: u8 = 15;
const DEFAULT_PAN: u16 = 0xDFE9;

fn hex(buf: &[u8]) -> Hex<'_> {
    Hex(buf)
}
struct Hex<'a>(&'a [u8]);
impl core::fmt::Display for Hex<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for b in self.0 {
            write!(f, "{:02X}", b)?;
        }
        Ok(())
    }
}

/// Drain received frames for `ms`, logging each; returns a beacon's PAN.
fn listen(radio: &mut Radio, ms: u32, buf: &mut [u8; MAX_PSDU]) -> Option<u16> {
    let mut pan = None;
    let deadline = Deadline::after_us(ms * 1000);
    while !deadline.expired() {
        if let Some(f) = radio.receive(buf) {
            let data = &buf[..f.len];
            let kind = data[0] & 7;
            log!(
                "rx len={} rssi={} lqi={} type={} {}",
                f.len,
                f.rssi_dbm,
                f.lqi,
                kind,
                hex(&data[..f.len.min(24)])
            );
            if kind == 0 && f.len >= 7 {
                // Beacon: FCF(2) seq(1) srcPAN(2) srcAddr(2|8) superframe(2)...
                let p = u16::from_le_bytes([data[3], data[4]]);
                log!("  BEACON pan={:04X} src={}", p, hex(&data[5..7]));
                pan.get_or_insert(p);
            }
        }
    }
    pan
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut lab = shared::init("radio");
    let mut buf = [0u8; MAX_PSDU];
    let t0 = cycles();
    let mut radio = match lab.radio.init() {
        Ok(r) => r,
        Err(e) => {
            log!("radio init failed: {:?}", e);
            loop {
                cortex_m::asm::wfi();
            }
        }
    };
    log!(
        "radio init {} us, state {:?}",
        cycles_to_us(cycles().wrapping_sub(t0)),
        radio.rac_state()
    );

    // 1. Energy detect.
    for ch in 11..=26u8 {
        radio.set_channel(ch).unwrap();
        let t = cycles();
        let ed = radio.energy_detect(5000);
        log!(
            "ED ch{} {:?} dBm ({} us) synth={:08X} {:?}",
            ch,
            ed,
            cycles_to_us(cycles().wrapping_sub(t)),
            radio.synth_status(),
            synth_diag()
        );
    }

    // 2. Passive RX.
    radio.set_channel(CHANNEL).unwrap();
    let t = cycles();
    let rx = radio.rx_on();
    log!(
        "rx_on {:?} in {} us, state {:?} frc={:08X}",
        rx,
        cycles_to_us(cycles().wrapping_sub(t)),
        radio.rac_state(),
        radio.frc_status()
    );
    log!(
        "RSSI now {:?}, CCA clear {:?}",
        radio.rssi_dbm(),
        radio.cca()
    );
    log!("-- passive RX 5 s on ch{} --", CHANNEL);
    listen(&mut radio, 5000, &mut buf);

    // 3. Beacon request.
    let mut pan = None;
    for seq in 0..3u8 {
        let req = [0x03, 0x08, seq, 0xFF, 0xFF, 0xFF, 0xFF, 0x07];
        let t = cycles();
        let r = radio.transmit_csma(&req, CsmaConfig::default(), &mut buf);
        log!(
            "beacon req #{} -> {:?} ({} us)",
            seq,
            r,
            cycles_to_us(cycles().wrapping_sub(t))
        );
        if let Some(p) = listen(&mut radio, 300, &mut buf) {
            pan.get_or_insert(p);
        }
    }
    let pan = pan.unwrap_or(DEFAULT_PAN);
    log!("using PAN {:04X}", pan);

    // 4. Data request to coordinator 0x0000 with ACK request.
    let eui = shared::eui64().to_le_bytes();
    let (mut acked, mut pending, mut noack, mut other) = (0u32, 0u32, 0u32, 0u32);
    for i in 0..20u8 {
        let seq = 0x40 + i;
        let mut f = [0u8; 16];
        f[..3].copy_from_slice(&[0x63, 0xC8, seq]);
        f[3..5].copy_from_slice(&pan.to_le_bytes());
        f[5..7].copy_from_slice(&0u16.to_le_bytes());
        f[7..15].copy_from_slice(&eui);
        f[15] = 0x04;
        let t = cycles();
        match radio.transmit_csma(&f, CsmaConfig::default(), &mut buf) {
            Ok(o) => {
                acked += 1;
                if o.ack_frame_pending == Some(true) {
                    pending += 1;
                }
                log!(
                    "data req seq={:02X} ACK pending={:?} ({} us)",
                    seq,
                    o.ack_frame_pending,
                    cycles_to_us(cycles().wrapping_sub(t))
                );
            }
            Err(RadioError::NoAck) => {
                noack += 1;
                log!("data req seq={:02X} no ACK", seq);
            }
            Err(e) => {
                other += 1;
                log!("data req seq={:02X} err {:?}", seq, e);
            }
        }
        listen(&mut radio, 50, &mut buf);
    }
    log!(
        "ACK summary: {} acked ({} pending), {} no-ack, {} errors",
        acked,
        pending,
        noack,
        other
    );

    // 5. Continuous RX.
    log!("-- continuous RX --");
    loop {
        listen(&mut radio, 10_000, &mut buf);
        let s = radio.stats();
        log!(
            "stats rx={} ovf={} tx={} txfail={} acks={} state {:?}",
            s.rx_frames,
            s.rx_overflows,
            s.tx_frames,
            s.tx_failures,
            s.acks_sent,
            radio.rac_state()
        );
        lab.led.toggle();
    }
}
