//! EFR32MG24 over-the-air radio gates (BRD4187 on a BRD4001A, feature
//! `brd4187`), driven by line commands on the WSTK VCOM so an hwlab
//! experiment can line them up with the AEM, the nRF sniffer and frames
//! injected by the nRF radio lab.
//!
//! Commands (one per line, 115 200 8N1):
//!
//! ```text
//! ch <11..26>            select the channel (RX restarts if it was on)
//! pw <deci-dBm>          TX power; logs the applied table entry
//! ed                     energy detect on every channel (5 ms each)
//! rx                     receive and acknowledge: frames to PAN 0x1234 /
//!                        short 0x5678 (or our EUI-64) with AR get a
//!                        software Imm-Ack; every frame is logged
//! tx <n> <len> <gap_ms>  n data frames (PAN 0x1234, src 0x5678, broadcast,
//!                        no AR) of <len> MAC bytes, <gap_ms> apart
//! txa <n> <pan> <dst>    n MAC data requests with AR to <dst> on <pan>
//!                        (hex); reports ACK / no ACK per frame
//! breq                   3 beacon requests (CSMA), listen 300 ms after each
//! idle                   radio off
//! regs                   RAC state and PA registers
//! stat                   radio counters
//! ```

#![no_std]
#![no_main]

#[path = "../shared/mod.rs"]
mod shared;

use efr32s2_hal::{
    delay::{Deadline, cycles, cycles_to_us},
    radio::{AckFilter, CsmaConfig, MAX_FRAME, Radio, RadioError, synth_diag},
};

const LAB_PAN: u16 = 0x1234;
const LAB_SHORT: u16 = 0x5678;

struct Hex<'a>(&'a [u8]);
impl core::fmt::Display for Hex<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for b in self.0 {
            write!(f, "{:02X}", b)?;
        }
        Ok(())
    }
}

fn parse_u32(s: Option<&str>, radix: u32) -> Option<u32> {
    s.and_then(|s| u32::from_str_radix(s.trim_start_matches("0x"), radix).ok())
}

fn parse_i32(s: Option<&str>) -> Option<i32> {
    s.and_then(|s| s.parse().ok())
}

fn read32(address: u32) -> u32 {
    // SAFETY: lab diagnostics read radio/DEVINFO registers only.
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

/// Drain received frames for `ms`, logging each; true if a beacon came.
fn listen(radio: &mut Radio, ms: u32, buf: &mut [u8; MAX_FRAME]) -> u32 {
    let mut beacons = 0;
    let deadline = Deadline::after_us(ms * 1000);
    while !deadline.expired() {
        if let Some(f) = radio.receive(buf) {
            log_frame(&buf[..f.len], f.rssi_dbm, f.lqi);
            if buf[0] & 7 == 0 {
                beacons += 1;
            }
        }
    }
    beacons
}

fn log_frame(data: &[u8], rssi: i8, lqi: u8) {
    log!(
        "rx len={} rssi={} lqi={} {}",
        data.len(),
        rssi,
        lqi,
        Hex(&data[..data.len().min(24)])
    );
    if data.len() >= 7 && data[0] & 7 == 0 {
        log!(
            "  BEACON pan={:04X} src={}",
            u16::from_le_bytes([data[3], data[4]]),
            Hex(&data[5..7])
        );
    }
}

fn regs(radio: &Radio) {
    let rssi = read32(0xB800_C018);
    let frame = read32(0xB800_C01C);
    log!(
        "AGC.RSSI={:08X} ({} q-dBm raw) FRAMERSSI={:08X} ({} q-dBm raw) CTRL0={:08X}",
        rssi,
        quarter_dbm(rssi),
        frame,
        quarter_dbm(frame),
        read32(0xB800_C020)
    );
    log!(
        "state {:?} RAC.TX={:08X} PAENCTRL={:08X} PACTRL={:08X} RFSTATUS={:08X} RAMPLEV={:08X} PADEBUG={:08X} RFLOCK0={:08X} synth {:?}",
        radio.rac_state(),
        read32(0xB802_016C),
        read32(0xB802_00B8),
        read32(0xB802_01C8),
        read32(0xB802_00AC),
        read32(0xB801_40DC),
        read32(0xB801_41B0),
        read32(0xB802_07F8),
        synth_diag()
    );
}

/// `AGC.RSSI`/`FRAMERSSI` `[15:6]`: signed quarter-dBm before the chip
/// adjustment.
fn quarter_dbm(raw: u32) -> i32 {
    (((raw >> 6) & 0x3FF) as i32) << 22 >> 22
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Idle,
    Rx,
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut lab = shared::init("mg24-air");
    let t0 = cycles();
    let mut radio = match lab.radio.init() {
        Ok(radio) => radio,
        Err(error) => {
            log!("radio init failed: {:?}", error);
            loop {
                cortex_m::asm::wfi();
            }
        }
    };
    log!(
        "radio init {} us, state {:?}, power range {:?} dBm",
        cycles_to_us(cycles().wrapping_sub(t0)),
        radio.rac_state(),
        efr32s2_hal::radio::TX_POWER_RANGE_DBM
    );
    let eui = shared::eui64();
    radio.set_ack_filter(AckFilter {
        pan_id: Some(LAB_PAN),
        short_address: Some(LAB_SHORT),
        extended_address: Some(eui),
    });
    regs(&radio);
    log!(
        "ready (ch {}, pw {})",
        radio.channel(),
        radio.tx_power_ddbm()
    );

    let mut line = [0u8; 64];
    let mut len = 0usize;
    let mut buf = [0u8; MAX_FRAME];
    let mut mode = Mode::Idle;
    let mut seq = 0u8;
    loop {
        if mode == Mode::Rx {
            while let Some(f) = radio.receive(&mut buf) {
                log_frame(&buf[..f.len], f.rssi_dbm, f.lqi);
                let frame = read32(0xB800_C01C);
                log!("  FRAMERSSI {} q-dBm raw", quarter_dbm(frame));
                lab.led.toggle();
            }
        }
        let Some(cmd) = shared::read_line(&mut line, &mut len) else {
            continue;
        };
        let mut words = cmd.split_ascii_whitespace();
        let Some(verb) = words.next() else {
            continue;
        };
        match verb {
            "ch" => match parse_u32(words.next(), 10) {
                Some(ch) if (11..=26).contains(&ch) => {
                    let r = radio.set_channel(ch as u8);
                    log!("ch {} -> {:?} {:?}", ch, r, synth_diag());
                }
                _ => log!("usage: ch <11..26>"),
            },
            "pw" => match parse_i32(words.next()) {
                Some(ddbm) => {
                    let applied = radio.set_tx_power_ddbm(ddbm as i16);
                    log!("pw {} -> {} ddBm", ddbm, applied);
                    regs(&radio);
                }
                None => log!("usage: pw <deci-dBm>"),
            },
            "ed" => {
                let ch = radio.channel();
                for c in 11..=26u8 {
                    let _ = radio.set_channel(c);
                    let ed = radio.energy_detect(5000);
                    log!("ED ch{} {:?} dBm {:?}", c, ed, synth_diag());
                }
                let _ = radio.set_channel(ch);
                mode = Mode::Idle;
                radio.idle();
            }
            "rx" => {
                let r = radio.rx_on();
                log!(
                    "rx on ch{} {:?} rssi {:?}",
                    radio.channel(),
                    r,
                    radio.rssi_dbm()
                );
                mode = Mode::Rx;
            }
            "idle" => {
                radio.idle();
                mode = Mode::Idle;
                log!("idle");
            }
            "tx" => {
                let n = parse_u32(words.next(), 10).unwrap_or(10);
                let flen =
                    (parse_u32(words.next(), 10).unwrap_or(20) as usize).clamp(12, MAX_FRAME);
                let gap = parse_u32(words.next(), 10).unwrap_or(50);
                let mut f = [0u8; MAX_FRAME];
                // Data, no AR, PAN-ID compression, short dst/src.
                f[..9].copy_from_slice(&[0x41, 0x88, 0, 0x34, 0x12, 0xFF, 0xFF, 0x78, 0x56]);
                let (mut ok, mut fail, mut worst) = (0u32, 0u32, 0u32);
                for i in 0..n {
                    seq = seq.wrapping_add(1);
                    f[2] = seq;
                    f[9..13].copy_from_slice(&i.to_le_bytes());
                    let t = cycles();
                    match radio.transmit(&f[..flen]) {
                        Ok(()) => ok += 1,
                        Err(e) => {
                            fail += 1;
                            if fail <= 3 {
                                log!("tx #{} {:?} {:?}", i, e, synth_diag());
                            }
                        }
                    }
                    worst = worst.max(cycles_to_us(cycles().wrapping_sub(t)));
                    if gap > 0 {
                        shared::delay_ms(gap);
                    }
                }
                log!(
                    "tx done {} ok {} fail, len {}, worst {} us, pw {}",
                    ok,
                    fail,
                    flen,
                    worst,
                    radio.tx_power_ddbm()
                );
                if mode == Mode::Rx {
                    let _ = radio.rx_on();
                }
            }
            "txa" => {
                let n = parse_u32(words.next(), 10).unwrap_or(10);
                let pan = parse_u32(words.next(), 16).unwrap_or(u32::from(LAB_PAN)) as u16;
                let dst = parse_u32(words.next(), 16).unwrap_or(0) as u16;
                let (mut acked, mut noack, mut other) = (0u32, 0u32, 0u32);
                for _ in 0..n {
                    seq = seq.wrapping_add(1);
                    // MAC command, AR, PAN-ID compression, short dst, ext src.
                    let mut f = [0u8; 16];
                    f[..3].copy_from_slice(&[0x63, 0xC8, seq]);
                    f[3..5].copy_from_slice(&pan.to_le_bytes());
                    f[5..7].copy_from_slice(&dst.to_le_bytes());
                    f[7..15].copy_from_slice(&eui.to_le_bytes());
                    f[15] = 0x04; // Data Request
                    let t = cycles();
                    match radio.transmit_csma(&f, CsmaConfig::default()) {
                        Ok(o) => {
                            acked += 1;
                            log!(
                                "txa seq={:02X} ACK fp={:?} ({} us)",
                                seq,
                                o.ack_frame_pending,
                                cycles_to_us(cycles().wrapping_sub(t))
                            );
                        }
                        Err(RadioError::NoAck) => {
                            noack += 1;
                            log!("txa seq={:02X} no ACK", seq);
                        }
                        Err(e) => {
                            other += 1;
                            log!("txa seq={:02X} {:?}", seq, e);
                        }
                    }
                    listen(&mut radio, 100, &mut buf);
                }
                log!(
                    "txa done: {} acked, {} no-ack, {} other",
                    acked,
                    noack,
                    other
                );
            }
            "breq" => {
                let mut beacons = 0;
                for _ in 0..3 {
                    seq = seq.wrapping_add(1);
                    let req = [0x03, 0x08, seq, 0xFF, 0xFF, 0xFF, 0xFF, 0x07];
                    let r = radio.transmit_csma(&req, CsmaConfig::default());
                    log!("beacon req ch{} -> {:?}", radio.channel(), r);
                    beacons += listen(&mut radio, 300, &mut buf);
                }
                log!("breq done: {} beacons on ch{}", beacons, radio.channel());
                mode = Mode::Rx;
            }
            "regs" => regs(&radio),
            "stat" => {
                let s = radio.stats;
                log!(
                    "stats rx={} ovf={} resync={} tx={} txfail={} acks={} synthfail={}",
                    s.rx_frames,
                    s.rx_overflows,
                    s.rx_resyncs,
                    s.tx_frames,
                    s.tx_failures,
                    s.acks_sent,
                    s.synth_failures
                );
            }
            other => log!("? {}", other),
        }
    }
}
