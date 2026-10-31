//! Gate 9a: EM2 on the RTCC, then radio state after wake.
//!
//! 1. RTCC from LFRCO + Embassy RTCC driver (provides the RTCC handler);
//! 2. radio init, RX on `CHANNEL`, count frames;
//! 3. snapshot of every PHY configuration register;
//! 4. two 2 s EM2 sleeps: RTCC elapsed time, TIMER0 progress during
//!    sleep, HCLK after wake, register diff, RX and beacon
//!    request/response after wake. Round 1 (EM2DBGEN on) uses the radio
//!    as is; round 2 (EM2DBGEN off) calls `Radio::resume` first;
//! 5. endless 5 s EM2 / 300 ms RX duty cycle (with `resume`) for AEM
//!    current capture.

#![no_std]
#![no_main]

#[path = "../shared/mod.rs"]
mod shared;

use efr32s2_hal::{
    delay::{Deadline, cycles},
    power::{self, SleepOutcome},
    radio::{self, CsmaConfig, MAX_FRAME, Radio},
    rtcc,
};

const CHANNEL: u8 = 15;
const MAX_REGS: usize = 256;

/// Frames received in `ms`; returns (all frames, beacons).
fn listen(radio: &mut Radio, ms: u32, buf: &mut [u8; MAX_FRAME]) -> (u32, u32) {
    let (mut frames, mut beacons) = (0, 0);
    let deadline = Deadline::after_us(ms * 1000);
    while !deadline.expired() {
        if let Some(f) = radio.receive(buf) {
            frames += 1;
            if buf[0] & 7 == 0 && f.len >= 7 {
                beacons += 1;
                log!(
                    "  beacon pan={:04X} rssi={}",
                    u16::from_le_bytes([buf[3], buf[4]]),
                    f.rssi_dbm
                );
            }
        }
    }
    (frames, beacons)
}

/// HCLK estimate: TIMER0 cycles over 100 ms of RTCC time.
fn hclk_hz() -> u32 {
    let start = rtcc::now();
    while rtcc::now() == start {}
    let t0 = rtcc::now();
    let c0 = cycles();
    while rtcc::now().wrapping_sub(t0) < 3277 {}
    (cycles().wrapping_sub(c0) as u64 * 10) as u32
}

fn snapshot(regs: &mut [u32; MAX_REGS]) -> usize {
    let n = radio::config_register_count().min(MAX_REGS);
    for (i, slot) in regs.iter_mut().enumerate().take(n) {
        *slot = radio::config_register(i).map_or(0, |(_, v)| v);
    }
    n
}

fn diff(before: &[u32; MAX_REGS], n: usize) -> usize {
    let mut changed = 0;
    for (i, &old) in before.iter().enumerate().take(n) {
        if let Some((address, now)) = radio::config_register(i)
            && now != old
        {
            changed += 1;
            if changed <= 12 {
                log!("  reg {:08X}: {:08X} -> {:08X}", address, old, now);
            }
        }
    }
    changed
}

/// Radio-adjacent registers outside the PHY table: CMU CLKEN1,
/// RADIOCLKCTRL, BUFC EN, BUF0/1 CTRL+ADDR, RAC IEN, FRC IEN.
const EXTRA: [u32; 9] = [
    0x4000_8068,
    0x4000_8280,
    0xBA00_0004,
    0xBA00_000C,
    0xBA00_0010,
    0xBA00_004C,
    0xBA00_0050,
    0xB802_0020,
    0xB800_4078,
];

fn read_extra() -> [u32; 9] {
    EXTRA.map(|a| unsafe { core::ptr::read_volatile(a as *const u32) })
}

fn beacon_request(radio: &mut Radio, seq: u8, buf: &mut [u8; MAX_FRAME]) -> u32 {
    let req = [0x03, 0x08, seq, 0xFF, 0xFF, 0xFF, 0xFF, 0x07];
    let r = radio.transmit_csma(&req, CsmaConfig::default());
    let (_, beacons) = listen(radio, 300, buf);
    log!("  beacon req -> {:?}, {} beacon(s)", r, beacons);
    beacons
}

fn sleep(ms: u32) -> (Result<SleepOutcome, power::PowerError>, u32, u32) {
    shared::log_flush();
    let r0 = rtcc::now();
    let c0 = cycles();
    let outcome = power::sleep_em2(rtcc::ms_to_ticks(ms), || false);
    let ticks = rtcc::now().wrapping_sub(r0);
    let timer = cycles().wrapping_sub(c0);
    (outcome, ticks, timer)
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut lab = shared::init("em2");
    let rtcc = match lab.rtcc.start() {
        Ok(r) => r,
        Err(e) => {
            log!("RTCC start failed: {:?}", e);
            loop {
                cortex_m::asm::wfi();
            }
        }
    };
    efr32s2_hal::time_driver::init(rtcc);
    let t = rtcc::now();
    shared::delay_ms(100);
    log!(
        "RTCC running: {} ticks in 100 ms busy wait, HCLK {} Hz",
        rtcc::now().wrapping_sub(t),
        hclk_hz()
    );

    let mut buf = [0u8; MAX_FRAME];
    let mut radio = match lab.radio.init() {
        Ok(r) => r,
        Err(e) => {
            log!("radio init failed: {:?}", e);
            loop {
                cortex_m::asm::wfi();
            }
        }
    };
    radio.set_channel(CHANNEL).unwrap();
    radio.rx_on().unwrap();
    let (frames, _) = listen(&mut radio, 2000, &mut buf);
    log!("before sleep: {} frames in 2 s", frames);
    beacon_request(&mut radio, 1, &mut buf);
    radio.idle();

    let mut regs = [0u32; MAX_REGS];
    let n = snapshot(&mut regs);
    log!("snapshot of {} PHY registers", n);

    let extra = read_extra();
    for (round, debug) in [(1u8, true), (2, false)] {
        power::set_em2_debug(debug);
        log!("-- round {}: EM2 2 s, EM2DBGEN={} --", round, debug);
        let (outcome, ticks, timer) = sleep(2000);
        for (i, (now, old)) in read_extra().iter().zip(extra.iter()).enumerate() {
            if now != old {
                log!("  extra {:08X}: {:08X} -> {:08X}", EXTRA[i], old, now);
            }
        }
        log!(
            "woke {:?}: RTCC {} ticks ({} ms), TIMER0 +{} cycles, HCLK {} Hz",
            outcome,
            ticks,
            rtcc::ticks_to_ms(ticks as u64),
            timer,
            hclk_hz()
        );
        let changed = diff(&regs, n);
        log!("PHY registers changed: {}/{}", changed, n);
        if round == 2 {
            let c = cycles();
            let r = radio.resume();
            log!(
                "resume {:?} in {} us",
                r,
                efr32s2_hal::delay::cycles_to_us(cycles().wrapping_sub(c))
            );
        }
        let rx = radio.rx_on();
        let (frames, _) = listen(&mut radio, 2000, &mut buf);
        log!(
            "after wake: rx_on {:?}, {} frames in 2 s, state {:?}",
            rx,
            frames,
            radio.rac_state()
        );
        beacon_request(&mut radio, 0x10 + round, &mut buf);
        radio.idle();
    }

    log!("-- duty cycle: 5 s EM2 (EM2DBGEN off) / 300 ms RX --");
    power::set_em2_debug(false);
    let mut cycle = 0u32;
    loop {
        let entries = power::em2_entries();
        // Odd cycles: VCOM level shifter off during sleep (current A/B).
        let vcom_off = cycle % 2 == 1;
        if vcom_off {
            shared::log_flush();
            lab.vcom_enable.set_low();
        }
        let (outcome, ticks, _) = sleep(5000);
        lab.vcom_enable.set_high();
        let entries = power::em2_entries().wrapping_sub(entries);
        lab.led.set_high();
        let resumed = radio.resume();
        let rx = resumed.and_then(|()| radio.rx_on());
        let (frames, _) = listen(&mut radio, 300, &mut buf);
        radio.idle();
        lab.led.set_low();
        cycle += 1;
        log!(
            "cycle {} vcom_off={} {:?} {} ms ({} EM2 entries), rx_on {:?}, {} frames",
            cycle,
            vcom_off,
            outcome,
            rtcc::ticks_to_ms(ticks as u64),
            entries,
            rx,
            frames
        );
    }
}
