//! CC26x2 dongle bring-up gates (`cargo build --release --bin cc26x2-lab`).
//!
//! Never part of the product image; proves each primitive on its own:
//!
//! 1. start-up: trim, chip identity, XOSC_HF, LF crystal, RTC time base,
//!    TRNG, UART log;
//! 2. flash: erase/program/verify/erase the top sector of the application
//!    window above this image (never the journals or the CCFG sector);
//! 3. radio: RF core power-up, CPE patch and IEEE setup; energy detect on
//!    every channel; an active scan (MAC Beacon Request on channels 11..26,
//!    beacons logged with PAN, short address and association permit); a
//!    promiscuous receive window on the busiest beaconing channel.
//!
//! Gate 3 transmits only Beacon Requests, which every Zigbee device may
//! send at any time; it never joins or opens a network.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod platform;

use cc26x2_hal::rf::{Csma, MAX_FRAME, Radio, TxOutcome};
use cc26x2_hal::trng::Trng;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};

#[allow(unused_imports)]
use cc26x2_hal::vectors::__INTERRUPTS;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Keep the product CCFG in the image.
    let ccfg = unsafe { core::ptr::read_volatile(&cc26x2_dongle_product::ccfg::CCFG[12]) };
    let started = platform::start("lab");
    log::info!(
        "[LAB] EUI-64 {:02X?} CCFG BL_CONFIG 0x{:08X}",
        started.identity.eui64,
        ccfg
    );

    // Gate 1: time base and entropy.
    let t0 = Instant::now();
    Timer::after(Duration::from_millis(1000)).await;
    log::info!("[LAB] 1000 ms timer took {} ms", t0.elapsed().as_millis());
    match Trng::new(started.trng) {
        Ok(mut trng) => match (trng.next_u64(), trng.next_u64()) {
            (Ok(a), Ok(b)) => log::info!("[LAB] TRNG {a:016X} {b:016X}"),
            other => log::error!("[LAB] TRNG read failed: {other:?}"),
        },
        Err(e) => log::error!("[LAB] TRNG start failed: {e:?}"),
    }

    // Gate 1b: CRYPTO engine, FIPS-197 C.1 known answer.
    aes_gate(started.crypto);

    // Gate 2: flash on the reserved GP window.
    flash_gate(started.flash);

    // Gate 3: radio.
    let mut radio = match started.radio.init(cc26x2_dongle_product::TX_POWER_DBM) {
        Ok(radio) => radio,
        Err(e) => {
            log::error!("[LAB] RF core init failed: {e:?}");
            platform::halt()
        }
    };
    log::info!("[LAB] RF core up, TX power {:?}", radio.tx_power());
    let mut busiest = (0u8, 0usize);
    loop {
        for channel in 11..=26u8 {
            let ed = energy(&mut radio, channel).await;
            let beacons = scan_channel(&mut radio, channel).await;
            log::info!("[LAB] ch {channel}: ED max {ed:?} dBm, {beacons} beacon(s)");
            if beacons > busiest.1 {
                busiest = (channel, beacons);
            }
        }
        if busiest.1 > 0 {
            sniff(&mut radio, busiest.0, Duration::from_secs(60)).await;
        }
        log::info!("[LAB] stats {:?}", radio.stats());
        Timer::after(Duration::from_secs(10)).await;
    }
}

fn flash_gate(mut flash: cc26x2_hal::flash::Flash) {
    // The top sector of the application window: above this small lab image,
    // below every journal.
    let sector = cc26x2_dongle_product::APPLICATION_END - cc26x2_hal::chip::SECTOR_SIZE as u32;
    unsafe extern "C" {
        static __sidata: u8;
        static __sdata: u8;
        static __edata: u8;
    }
    let lma = core::ptr::addr_of!(__sidata) as u32;
    let image_end =
        lma + (core::ptr::addr_of!(__edata) as u32 - core::ptr::addr_of!(__sdata) as u32);
    if image_end > sector {
        log::error!("[LAB] flash gate skipped: image ends at 0x{image_end:05X}");
        return;
    }
    let pattern: [u8; 16] = *b"zigbee-rs cc26x2";
    let result = flash
        .erase_sector(sector)
        .and_then(|()| flash.program(sector + 64, &pattern))
        .and_then(|()| {
            let mut back = [0u8; 16];
            for (i, b) in back.iter_mut().enumerate() {
                *b = unsafe { core::ptr::read_volatile((sector + 64 + i as u32) as *const u8) };
            }
            if back == pattern {
                Ok(())
            } else {
                Err(cc26x2_hal::flash::FlashError::Controller(0xDEAD))
            }
        })
        .and_then(|()| flash.erase_sector(sector));
    let ccfg = flash.erase_sector(cc26x2_hal::flash::CCFG_SECTOR);
    log::info!(
        "[LAB] flash gate at 0x{sector:05X}: {result:?}; CCFG sector erase refused: {ccfg:?}"
    );
}

async fn energy(radio: &mut Radio, channel: u8) -> Option<i8> {
    radio.start_energy_detect(channel, 5_000).ok()?;
    Timer::after(Duration::from_millis(6)).await;
    for _ in 0..100 {
        if let Some(result) = radio.poll_energy_detect() {
            return result.ok();
        }
        Timer::after(Duration::from_millis(1)).await;
    }
    None
}

/// MAC Beacon Request on `channel`, then collect beacons for 400 ms.
async fn scan_channel(radio: &mut Radio, channel: u8) -> usize {
    if radio.set_filter(None).is_err() || radio.set_channel(channel).is_err() {
        return 0;
    }
    if radio.rx_on().is_err() {
        return 0;
    }
    static mut SEQ: u8 = 0x40;
    let seq = unsafe {
        SEQ = SEQ.wrapping_add(1);
        SEQ
    };
    // FCF 0x0803: MAC command, dst short, no src; PAN 0xFFFF, dst 0xFFFF.
    let request = [0x03, 0x08, seq, 0xFF, 0xFF, 0xFF, 0xFF, 0x07];
    let csma = Csma {
        min_be: 3,
        max_be: 5,
        max_backoffs: 4,
    };
    if let Err(e) = radio.start_tx(&request, csma) {
        log::warn!("[LAB] ch {channel}: TX submit failed: {e:?}");
        return 0;
    }
    let deadline = Instant::now() + Duration::from_millis(100);
    let outcome = loop {
        if let Some(r) = radio.poll_tx() {
            break Some(r);
        }
        if Instant::now() > deadline {
            radio.abort_tx();
            break None;
        }
        Timer::after(Duration::from_micros(500)).await;
    };
    if !matches!(outcome, Some(Ok(TxOutcome::Sent))) {
        log::warn!("[LAB] ch {channel}: beacon request {outcome:?}");
    }
    let mut frame = [0u8; MAX_FRAME];
    let mut beacons = 0;
    let end = Instant::now() + Duration::from_millis(400);
    while Instant::now() < end {
        while let Some(info) = radio.receive(&mut frame) {
            let f = &frame[..info.len];
            if f.len() >= 9 && f[0] & 0x07 == 0 {
                beacons += 1;
                let pan = u16::from_le_bytes([f[3], f[4]]);
                let src = u16::from_le_bytes([f[5], f[6]]);
                // Superframe spec bytes 7..9: association permit is bit 15.
                let permit = f[8] & 0x80 != 0;
                let zigbee = f.get(11..).map(|p| (p.first().copied(), p.get(2).copied()));
                log::info!(
                    "[LAB]   beacon PAN 0x{pan:04X} from 0x{src:04X} rssi {} corr {} permit {permit} nwk {:02X?}",
                    info.rssi_dbm,
                    info.correlation,
                    zigbee
                );
            }
        }
        Timer::after(Duration::from_millis(2)).await;
    }
    beacons
}

/// Promiscuous receive on `channel`: frame type, length, RSSI, and the
/// RSSI/correlation distribution of FCS-valid versus FCS-failed frames
/// (CRC-error frames are kept for this gate only).
async fn sniff(radio: &mut Radio, channel: u8, window: Duration) {
    if radio.set_filter(None).is_err()
        || radio.set_channel(channel).is_err()
        || radio.set_keep_crc_errors(true).is_err()
        || radio.rx_on().is_err()
    {
        return;
    }
    log::info!(
        "[LAB] sniffing channel {channel} for {} s",
        window.as_secs()
    );
    // RSSI buckets: < -90, -90..-81, -80..-71, -70..-61, >= -60 dBm.
    let mut ok = [0u32; 5];
    let mut bad = [0u32; 5];
    let mut corr_ok = 0u32;
    let mut corr_bad = 0u32;
    let mut frame = [0u8; MAX_FRAME];
    let mut count = 0u32;
    let end = Instant::now() + window;
    while Instant::now() < end {
        while let Some(info) = radio.receive_any(&mut frame) {
            let bucket = match info.rssi_dbm {
                ..=-91 => 0,
                -90..=-81 => 1,
                -80..=-71 => 2,
                -70..=-61 => 3,
                _ => 4,
            };
            if !info.crc_ok {
                bad[bucket] += 1;
                corr_bad += u32::from(info.correlation);
                continue;
            }
            ok[bucket] += 1;
            corr_ok += u32::from(info.correlation);
            count += 1;
            if count <= 20 {
                let f = &frame[..info.len];
                log::info!(
                    "[LAB]   rx type {} len {} seq {} rssi {} {:02X?}",
                    f[0] & 0x07,
                    info.len,
                    f.get(2).copied().unwrap_or(0),
                    info.rssi_dbm,
                    &f[..f.len().min(12)]
                );
            }
        }
        Timer::after(Duration::from_millis(2)).await;
    }
    let _ = radio.set_keep_crc_errors(false);
    let n_bad: u32 = bad.iter().sum();
    log::info!("[LAB] {count} frame(s) on channel {channel}, {n_bad} CRC error(s)");
    log::info!("[LAB]   rssi <-90/-90/-80/-70/>=-60 ok {ok:?} crc-error {bad:?}");
    log::info!(
        "[LAB]   mean correlation ok {} crc-error {}",
        corr_ok.checked_div(count).unwrap_or(0),
        corr_bad.checked_div(n_bad).unwrap_or(0)
    );
}

/// AES-128 on the CRYPTO engine against FIPS-197 appendix C.1.
fn aes_gate(token: cc26x2_hal::crypto::CryptoToken) {
    const KEY: [u8; 16] = [
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
        0x0f,
    ];
    const PLAIN: [u8; 16] = [
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee,
        0xff,
    ];
    const CIPHER: [u8; 16] = [
        0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5,
        0x5a,
    ];
    let result = cc26x2_hal::crypto::AesEngine::new(token).and_then(|mut engine| {
        let mut block = PLAIN;
        engine.encrypt_block(&KEY, &mut block).map(|()| block)
    });
    match result {
        Ok(block) if block == CIPHER => log::info!("[LAB] AES-128 CRYPTO engine: FIPS-197 ok"),
        other => log::error!("[LAB] AES-128 CRYPTO engine: {other:02X?}"),
    }
}
