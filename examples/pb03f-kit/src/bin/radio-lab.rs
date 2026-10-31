//! PB-03F-Kit radio lab: brings up the pure-Rust PHY62x2 802.15.4 driver
//! without the Zigbee stack and exercises it against real networks.
//!
//! * prints calibration results and the LL/BB register state, including
//!   after an aborted receive (BBLL reset);
//! * compares [`zigbee_crc16`] with the ROM `zigbee_crc16_gen`;
//! * transmits beacon requests on `LAB_CHANNEL`, alternating the FCS mode
//!   (software DSN 0x10.., hardware DSN 0x90..) so an external sniffer can
//!   tell which mode produced a valid FCS;
//! * receives for 300 ms after each request and prints every frame with
//!   RSSI, plus the driver counters and an energy-detect sample.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use panic_halt as _;
use pb03f_kit::{Resources, console_log, time, vectors};
use zigbee_mac::phy6222::driver::{FcsMode, PaTable, Phy6222Driver, RadioConfig, zigbee_crc16};

const LAB_CHANNEL: u8 = 15;

macro_rules! out {
    ($($arg:tt)*) => { console_log::write_fmt(format_args!($($arg)*)) };
}

fn read(addr: u32) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

fn dump_regs(tag: &str) {
    out!(
        "[{}] LL 18={:#x} 1c={:#x} 20={:#x} 24={:#x} 28={:#x} 68={:#x} 74={:#x} | BB 00={:#x} 124={:#x} bc={:#x} b4={} b8={:#x} 94={:#x}\n",
        tag,
        read(0x4003_1018),
        read(0x4003_101c),
        read(0x4003_1020),
        read(0x4003_1024),
        read(0x4003_1028),
        read(0x4003_1068),
        read(0x4003_1074),
        read(0x4003_0000),
        read(0x4003_0124),
        read(0x4003_00bc),
        read(0x4003_00b4),
        read(0x4003_00b8),
        read(0x4003_0094),
    );
}

/// ROM `void zigbee_crc16_gen(uint8_t* dataIn, int length, uint8_t* seed,
/// uint8_t* crcCode)` (`bb_rom_sym_m0.txt`: 0x0001798d).
fn rom_crc16(data: &[u8]) -> [u8; 2] {
    type RomCrc = unsafe extern "C" fn(*const u8, i32, *mut u8, *mut u8);
    let f: RomCrc = unsafe { core::mem::transmute(0x0001_798d_usize) };
    let mut seed = [0u8; 16];
    let mut crc = [0xffu8; 2];
    unsafe {
        f(
            data.as_ptr(),
            data.len() as i32,
            seed.as_mut_ptr(),
            crc.as_mut_ptr(),
        )
    };
    crc
}

fn crc_self_test() {
    let mut buf = [0u8; 64];
    let mut seed = 0x1234_5678u32;
    let mut mismatches = 0;
    for len in 0..buf.len() {
        for b in buf.iter_mut().take(len) {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            *b = (seed >> 16) as u8;
        }
        if rom_crc16(&buf[..len]) != zigbee_crc16(&buf[..len]) {
            mismatches += 1;
        }
    }
    let rom = rom_crc16(b"123456789");
    out!(
        "crc: rom(\"123456789\")={:02x}{:02x} rust={:02x?} mismatches={}/64\n",
        rom[0],
        rom[1],
        zigbee_crc16(b"123456789"),
        mismatches
    );
}

/// Raw STX with the LL interrupt masked: time from trigger to MODE_DONE.
fn raw_tx_probe(tag: &str, bytes: &[u8]) {
    let w = |a: u32, v: u32| unsafe { core::ptr::write_volatile(a as *mut u32, v) };
    cortex_m::peripheral::NVIC::mask(vectors::Interrupt::LlIrq);
    w(0x4003_1058, 0x0001);
    for chunk in bytes.chunks(4) {
        let mut b = [0u8; 4];
        b[..chunk.len()].copy_from_slice(chunk);
        w(0x4003_1400, u32::from_le_bytes(b));
    }
    let tfifo = read(0x4003_1050);
    w(0x4003_1004, 0x0000_0100);
    w(0x4003_1038, 0x07);
    w(0x4003_1014, 0x3fff);
    w(0x4003_100c, 0x1);
    let t0 = time::now_us();
    w(0x4003_1000, 0x1);
    let mut first_status = 0;
    let mut t_first = 0;
    let status = loop {
        let st = read(0x4003_1010);
        if st != 0 && first_status == 0 {
            first_status = st;
            t_first = time::now_us() - t0;
        }
        if st & 1 != 0 || time::now_us() - t0 > 50_000 {
            break st;
        }
    };
    let dt = time::now_us() - t0;
    w(0x4003_1014, 0x3fff);
    cortex_m::peripheral::NVIC::unpend(vectors::Interrupt::LlIrq);
    unsafe { cortex_m::peripheral::NVIC::unmask(vectors::Interrupt::LlIrq) };
    out!(
        "raw-tx {}: len={} words tfifo_before={:#x} first={:#x}@{}us done={:#x}@{}us tfifo_after={:#x}\n",
        tag,
        bytes[0],
        tfifo,
        first_status,
        t_first,
        status,
        dt,
        read(0x4003_1050)
    );
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    time::init();
    let Ok(Resources {
        console, mut leds, ..
    }) = Resources::take()
    else {
        loop {
            cortex_m::asm::wfi();
        }
    };
    console_log::init(console.into_uart(), log::LevelFilter::Info);
    out!("\npb03f-radio-lab: boot ok\n");
    dump_regs("reset");
    crc_self_test();

    unsafe { cortex_m::peripheral::NVIC::unmask(vectors::Interrupt::LlIrq) };
    let config = RadioConfig {
        channel: LAB_CHANNEL,
        pa_table: PaTable::QFN32,
        tx_power: 0,
        hclk_mhz: (phy6222_hal::clock::hclk_hz() / 1_000_000) as u8,
        ..RadioConfig::default()
    };
    let Some(mut radio) = Phy6222Driver::take(config) else {
        out!("radio already taken\n");
        loop {
            cortex_m::asm::wfi();
        }
    };
    let s = radio.stats();
    out!(
        "radio: hclk={} MHz tp_cal={:#04x}/{:#04x} dcoc={:#010x} ok={}\n",
        config.hclk_mhz,
        s.tp_cal.0,
        s.tp_cal.1,
        s.dcoc,
        s.dcoc_ok
    );
    dump_regs("init");

    let mut req = [0x0a, 0x03, 0x08, 0x01, 0xff, 0xff, 0xff, 0xff, 0x07, 0, 0];
    let fcs = zigbee_crc16(&req[1..9]);
    req[9..].copy_from_slice(&fcs);
    raw_tx_probe("sw-fcs", &req);
    raw_tx_probe("hw-fcs", &req[..9]);
    let mut long = [0u8; 128];
    long[0] = 127;
    long[1] = 0x01; // data frame, no addressing (sniffer-visible filler)
    raw_tx_probe("len127", &long);
    let mut short = [0u8; 6];
    short[0] = 5;
    short[1] = 0x02; // ACK frame
    raw_tx_probe("ack", &short);

    // Abort a receive mid-flight and check what the BBLL reset preserved.
    let _ = select(radio.receive(), Timer::after_millis(5)).await;
    dump_regs("after-abort");

    let mut dsn_sw: u8 = 0x10;
    let mut dsn_hw: u8 = 0x90;
    let mut round: u32 = 0;
    loop {
        let mode = if round % 2 == 0 {
            FcsMode::Software
        } else {
            FcsMode::Hardware
        };
        radio.update_config(|c| c.fcs = mode);
        let dsn = match mode {
            FcsMode::Software => {
                dsn_sw = if dsn_sw >= 0x7f { 0x10 } else { dsn_sw + 1 };
                dsn_sw
            }
            FcsMode::Hardware => {
                dsn_hw = if dsn_hw >= 0xef { 0x90 } else { dsn_hw + 1 };
                dsn_hw
            }
        };
        leds.all_off();
        leds.blue.set_on(true);
        // Beacon request: FCF 0x0803, DSN, dst PAN/addr broadcast, cmd 0x07.
        let beacon_req = [0x03, 0x08, dsn, 0xff, 0xff, 0xff, 0xff, 0x07];
        let t0 = Instant::now();
        let tx = radio.transmit(&beacon_req).await;
        let tx_us = t0.elapsed().as_micros();
        leds.blue.set_on(false);
        out!(
            "--- round {} ch{} fcs={:?} dsn={:#04x} tx={:?} ({} us)\n",
            round,
            LAB_CHANNEL,
            mode,
            dsn,
            tx.is_ok(),
            tx_us
        );

        let deadline = Instant::now() + Duration::from_millis(300);
        let mut frames = 0u32;
        loop {
            let now = Instant::now();
            if now >= deadline {
                break;
            }
            match select(radio.receive(), Timer::after(deadline - now)).await {
                Either::First(Ok(f)) => {
                    frames += 1;
                    leds.green.set_on(true);
                    let data = f.as_slice();
                    let fcf = u16::from_le_bytes([data[0], data[1]]);
                    if frames <= 8 {
                        out!(
                            "  rx len={} rssi={} lqi={} fcf={:#06x} seq={:#04x} hdr={:02x?}\n",
                            data.len(),
                            radio.last_rx_rssi(),
                            f.lqi,
                            fcf,
                            data[2],
                            &data[3..data.len().min(11)]
                        );
                    }
                    leds.green.set_on(false);
                }
                Either::First(Err(e)) => {
                    out!("  rx err {:?}\n", e);
                    break;
                }
                Either::Second(()) => break,
            }
        }
        let ed = radio.energy_detect().await;
        let s = radio.stats();
        out!(
            "  frames={} | tx_done={} tx_to={} rx_ok={} crc_err={} rx_to={} sw_fcs_mm={} bad_len={} aborts={} hw={:#08x} ed={:?}\n",
            frames,
            s.tx_done,
            s.tx_timeout,
            s.rx_ok,
            s.rx_crc_err,
            s.rx_timeout,
            s.rx_sw_fcs_mismatch,
            s.rx_bad_len,
            s.aborts,
            s.hw_pkt_stats,
            ed.map(|(rssi, _)| rssi).ok()
        );
        round = round.wrapping_add(1);
        Timer::after_millis(700).await;
    }
}
