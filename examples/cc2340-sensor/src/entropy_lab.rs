//! LP-EM-CC2340R5 entropy-source lab image (`--features entropy-lab --bin
//! cc2340-entropy-lab`). Not a product image: it exposes raw radio noise.
//!
//! Proves the radio ADC-noise entropy source independently of Zigbee and
//! characterises it on silicon:
//!
//! 1. installs the `LAES` engine (FIPS-197 self-test);
//! 2. runs [`STATS_CAPTURES`] raw 1024-word captures through on-device
//!    statistics: per-stream code histograms (most-common-value estimate),
//!    the longest repetition run and the largest one-code / top-two counts
//!    per 512-code window (the quantities the health-test cutoffs bound);
//! 3. runs [`LAG_CAPTURES`] captures per lag 1..=3 through a lag-k predictor
//!    (best next code for each previous code, SP 800-90B 6.3.8 style), whose
//!    hit rate bounds the min-entropy of a correlated source;
//! 4. seeds the CTR_DRBG with the production path (`seed_entropy`: DRBG
//!    known-answer test, health tests, CBC-MAC conditioning), streams
//!    [`DRBG_BYTES`] of output through monobit / byte chi-square / runs
//!    statistics, reseeds, and records the timing and the first output bytes
//!    (compared across reboots by the operator).
//!
//! Results are read with the debug probe from the `LAB` symbol
//! (`LAB.magic == 0x454E_5452` when finished); RTT prints a summary.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::ptr::addr_of_mut;

use cortex_m as _;
use embassy_executor::Spawner;
use embassy_time::Instant;
use lp_em_cc2340r5 as board;
use rtt_target::rprintln;
use zigbee_mac::PlatformServices;
use zigbee_mac::cc2340::Cc2340RadioPhy;
use zigbee_mac::cc2340::entropy::{APT_WINDOW, NOISE_WORDS};

const STATS_CAPTURES: u32 = 256;
const LAG_CAPTURES: u32 = 64;
const DRBG_CHUNK: usize = 4096;
const DRBG_BYTES: usize = 16 * DRBG_CHUNK;

/// On-device statistics, read by the debug probe.
#[repr(C)]
pub struct LabResults {
    pub magic: u32,
    pub stage: u32,
    pub stats_ok: u32,
    pub stats_failed: u32,
    /// `[I, Q]` histograms of every 5-bit code of the statistics captures.
    pub histogram: [[u32; 32]; 2],
    /// Longest run of one code value per stream.
    pub longest_run: [u32; 2],
    /// Largest count of one value in a 512-code window, per stream.
    pub max_window_count: [u32; 2],
    /// Largest top-two count sum in a 512-code window, per stream.
    pub max_window_pair: [u32; 2],
    /// Lag-k predictor hits per `[lag - 1][stream]` and the samples scored
    /// per stream and lag.
    pub lag_hits: [[u32; 2]; 3],
    pub lag_samples: [u32; 2],
    /// DRBG output statistics over `DRBG_BYTES`: one bits, byte chi-square
    /// (x1000, 255 degrees of freedom), bit runs, longest bit run.
    pub drbg_ones: u32,
    pub drbg_chi2_milli: u32,
    pub drbg_runs: u32,
    pub drbg_longest_run: u32,
    pub drbg_first: [u8; 16],
    pub drbg_after_reseed: [u8; 16],
    /// `EntropyDisposition` after the first seed: 1 = seeded.
    pub seeded: u32,
    pub seed_us: u32,
    pub reseed_us: u32,
    pub fill_us: u32,
    pub reseed_ok: u32,
    pub fill_ok: u32,
}

#[unsafe(no_mangle)]
pub static mut LAB: LabResults = LabResults {
    magic: 0,
    stage: 0,
    stats_ok: 0,
    stats_failed: 0,
    histogram: [[0; 32]; 2],
    longest_run: [0; 2],
    max_window_count: [0; 2],
    max_window_pair: [0; 2],
    lag_hits: [[0; 2]; 3],
    lag_samples: [0; 2],
    drbg_ones: 0,
    drbg_chi2_milli: 0,
    drbg_runs: 0,
    drbg_longest_run: 0,
    drbg_first: [0; 16],
    drbg_after_reseed: [0; 16],
    seeded: 0,
    seed_us: 0,
    reseed_us: 0,
    fill_us: 0,
    reseed_ok: 0,
    fill_ok: 0,
};
static mut SCRATCH: [u32; NOISE_WORDS] = [0; NOISE_WORDS];
static mut TRANSITIONS: [[[u32; 32]; 32]; 2] = [[[0; 32]; 32]; 2];
static mut DRBG: [u8; DRBG_CHUNK] = [0; DRBG_CHUNK];

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    halt()
}

fn halt() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

struct StreamStats {
    last: u8,
    run: u32,
    window: u32,
    counts: [u32; 32],
}

impl StreamStats {
    const fn new() -> Self {
        Self {
            last: u8::MAX,
            run: 0,
            window: 0,
            counts: [0; 32],
        }
    }

    fn feed(&mut self, code: u8, lab: &mut LabResults, stream: usize) {
        lab.histogram[stream][usize::from(code)] += 1;
        if code == self.last {
            self.run += 1;
        } else {
            self.last = code;
            self.run = 1;
        }
        lab.longest_run[stream] = lab.longest_run[stream].max(self.run);
        self.counts[usize::from(code)] += 1;
        self.window += 1;
        if self.window == u32::from(APT_WINDOW) {
            let mut sorted = self.counts;
            sorted.sort_unstable();
            lab.max_window_count[stream] = lab.max_window_count[stream].max(sorted[31]);
            lab.max_window_pair[stream] = lab.max_window_pair[stream].max(sorted[31] + sorted[30]);
            self.counts = [0; 32];
            self.window = 0;
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 1024,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "Terminal"
            }
        }
    };
    rtt_target::set_print_channel(channels.up.0);
    // SAFETY: single-threaded lab image; these statics are only touched here
    // and read by the debug probe.
    let lab = unsafe { &mut *addr_of_mut!(LAB) };
    let transitions = unsafe { &mut *addr_of_mut!(TRANSITIONS) };
    let scratch = unsafe { &mut *addr_of_mut!(SCRATCH) };
    let drbg = unsafe { &mut *addr_of_mut!(DRBG) };

    let Some(resources) = board::take() else {
        halt()
    };
    let Ok(ieee) = board::identity::factory_ieee_address() else {
        halt()
    };
    rprintln!("[LAB] CC2340 entropy lab, EUI-64 {:02X?}", ieee);
    let Ok(mut phy) = Cc2340RadioPhy::new(ieee) else {
        halt()
    };
    if let Err(error) = phy.install_aes_engine(resources.aes) {
        rprintln!("[LAB] LAES self-test failed: {:?}", error);
        halt()
    }
    rprintln!("[LAB] LAES AES-128 self-test passed");

    lab.stage = 2;
    for _ in 0..STATS_CAPTURES {
        if phy.lab_capture_raw_noise(scratch).is_err() {
            lab.stats_failed += 1;
            continue;
        }
        lab.stats_ok += 1;
        // A new capture is not contiguous with the previous one.
        let mut streams = [StreamStats::new(), StreamStats::new()];
        for &word in scratch.iter() {
            for shift in [0, 5, 10] {
                streams[1].feed(((word >> shift) & 0x1F) as u8, lab, 1);
                streams[0].feed(((word >> (16 + shift)) & 0x1F) as u8, lab, 0);
            }
        }
    }
    rprintln!(
        "[LAB] stats captures ok={} failed={} longest run I/Q {:?} max window count {:?} pair {:?}",
        lab.stats_ok,
        lab.stats_failed,
        lab.longest_run,
        lab.max_window_count,
        lab.max_window_pair
    );

    for lag in 1..=3usize {
        *transitions = [[[0; 32]; 32]; 2];
        let mut scored = [0u32; 2];
        for _ in 0..LAG_CAPTURES {
            if phy.lab_capture_raw_noise(scratch).is_err() {
                lab.stats_failed += 1;
                continue;
            }
            let mut history = [[0u8; 3]; 2];
            let mut seen = 0usize;
            for &word in scratch.iter() {
                for shift in [0, 5, 10] {
                    let codes = [
                        ((word >> (16 + shift)) & 0x1F) as u8,
                        ((word >> shift) & 0x1F) as u8,
                    ];
                    for stream in 0..2 {
                        if seen >= lag {
                            let previous = history[stream][lag - 1];
                            transitions[stream][usize::from(previous)]
                                [usize::from(codes[stream])] += 1;
                            scored[stream] += 1;
                        }
                        history[stream] = [codes[stream], history[stream][0], history[stream][1]];
                    }
                    seen += 1;
                }
            }
        }
        for stream in 0..2 {
            lab.lag_hits[lag - 1][stream] = transitions[stream]
                .iter()
                .map(|row| row.iter().copied().max().unwrap_or(0))
                .sum();
            lab.lag_samples[stream] = scored[stream];
        }
    }
    rprintln!(
        "[LAB] lag predictor hits {:?} of {:?} samples",
        lab.lag_hits,
        lab.lag_samples
    );

    lab.stage = 3;
    let started = Instant::now();
    let seeded = phy.seed_entropy();
    lab.seed_us = started.elapsed().as_micros() as u32;
    rprintln!(
        "[LAB] seed_entropy {:?} in {} us -> {:?}",
        seeded,
        lab.seed_us,
        phy.entropy_disposition()
    );
    lab.seeded = u32::from(seeded.is_ok());

    let mut byte_counts = [0u32; 256];
    let mut last_bit = 2u8;
    let mut run = 0u32;
    let mut fill_ok = true;
    for chunk in 0..DRBG_BYTES / DRBG_CHUNK {
        let started = Instant::now();
        fill_ok &= phy.fill_random(drbg).is_ok();
        if chunk == 0 {
            lab.fill_us = started.elapsed().as_micros() as u32;
            lab.drbg_first.copy_from_slice(&drbg[..16]);
        }
        for &byte in drbg.iter() {
            byte_counts[usize::from(byte)] += 1;
            lab.drbg_ones += byte.count_ones();
            for bit in (0..8).rev().map(|shift| (byte >> shift) & 1) {
                if bit == last_bit {
                    run += 1;
                } else {
                    lab.drbg_runs += 1;
                    last_bit = bit;
                    run = 1;
                }
                lab.drbg_longest_run = lab.drbg_longest_run.max(run);
            }
        }
    }
    // chi2 = sum((n - E)^2 / E), E = DRBG_BYTES / 256, in milli-units.
    let expected = (DRBG_BYTES / 256) as u64;
    lab.drbg_chi2_milli = (byte_counts
        .iter()
        .map(|&count| {
            let delta = u64::from(count).abs_diff(expected);
            delta * delta * 1000 / expected
        })
        .sum::<u64>()) as u32;

    let started = Instant::now();
    let reseeded = phy.seed_entropy();
    lab.reseed_us = started.elapsed().as_micros() as u32;
    lab.reseed_ok = u32::from(reseeded.is_ok());
    fill_ok &= phy.fill_random(&mut lab.drbg_after_reseed).is_ok();
    lab.fill_ok = u32::from(fill_ok);
    rprintln!(
        "[LAB] {} B: ones {} chi2 {}.{:03} runs {} longest {}; first {:02x?}; reseed {:?} in {} us; fill {} us/4 KiB",
        DRBG_BYTES,
        lab.drbg_ones,
        lab.drbg_chi2_milli / 1000,
        lab.drbg_chi2_milli % 1000,
        lab.drbg_runs,
        lab.drbg_longest_run,
        lab.drbg_first,
        reseeded,
        lab.reseed_us,
        lab.fill_us
    );

    // The production radio path still comes up after the captures.
    match phy.initialize_radio() {
        Ok(()) => rprintln!("[LAB] IEEE radio initialised after the noise captures"),
        Err(error) => rprintln!("[LAB] IEEE radio init failed: {:?}", error),
    }

    lab.stage = 4;
    lab.magic = 0x454E_5452;
    rprintln!("[LAB] done");
    halt()
}
