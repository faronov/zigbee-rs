//! MP23DB01HP PDM microphone on SAI1 A with circular DMA.
//!
//! 1. Lane check: ones-density of each byte lane (the microphone's lanes
//!    sit near 500 ‰, unused lanes at 0 or 1000) and the word rate.
//! 2. Level: bursts of 8 × 1024 words (41 ms) through one CIC(64) in the
//!    configured bit order, reported as the block level (burst mean
//!    removed) and the streaming [`LevelMeter`] result, with the CPU time
//!    per chunk (must stay below the 5.1 ms a half ring takes to fill).
//!    Pass = no overruns and the level rises with sound near the board.

#![no_std]
#![no_main]

#[path = "../shared/lab.rs"]
#[macro_use]
mod lab;

use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use stm32wb_hal::embassy_stm32::{bind_interrupts, dma, peripherals};
use stm32wb5mm_dk::mic::{self, MicPins, Microphone};
use zigbee_pdm::{BitOrder, Cic, LevelMeter};

bind_interrupts!(struct Irqs {
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
});

const CHUNK: usize = 1024;
const CHUNKS_PER_BURST: usize = 8;
/// CIC output samples per burst (8 PDM bits per word, R = 64).
const BURST_SAMPLES: usize = CHUNK * CHUNKS_PER_BURST * 8 / 64;

static mut RING: [u32; 2 * CHUNK] = [0; 2 * CHUNK];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = lab::init();
    let mut vcom = lab::Vcom::new(p.USART1, p.PB6);
    say!(
        vcom,
        "[DK][mic] READY sai1a ck2=PA8 d2=PA9 sck={} Hz pdm={} Hz words/s={} lane={} msb={}",
        mic::SAI_SCK_HZ,
        mic::PDM_CLOCK_HZ,
        mic::WORDS_PER_SECOND,
        mic::MIC_BYTE_LANE,
        mic::MIC_BIT_ORDER_MSB
    );
    // SAFETY: the only reference to RING, handed to the microphone once.
    let ring = unsafe { &mut *core::ptr::addr_of_mut!(RING) };
    let mut microphone = Microphone::new(
        MicPins {
            sai1: p.SAI1,
            clock: p.PA8,
            data: p.PA9,
            dma: p.DMA1_CH2,
        },
        Irqs,
        ring,
    );
    microphone.start();
    let mut chunk = [0u32; CHUNK];

    for n in 0..6u32 {
        microphone.clear();
        let t0 = Instant::now();
        let mut ones = [0u32; 4];
        let mut ok = true;
        for _ in 0..4 {
            if microphone.read(&mut chunk).await.is_err() {
                ok = false;
            }
            for word in &chunk {
                for (lane, count) in ones.iter_mut().enumerate() {
                    *count += ((word >> (8 * lane)) as u8).count_ones();
                }
            }
        }
        let us = t0.elapsed().as_micros();
        let bits = (4 * CHUNK * 8) as u32;
        say!(
            vcom,
            "[DK][mic] lanes n={n} {} words in {us} us ok={ok} density‰={:?}",
            4 * CHUNK,
            ones.map(|o| o * 1000 / bits)
        );
        Timer::after_millis(500).await;
    }

    let order = if mic::MIC_BIT_ORDER_MSB {
        BitOrder::Msb
    } else {
        BitOrder::Lsb
    };
    let Some(mut cic) = Cic::new(64) else {
        lab::halt()
    };
    let mut meter = LevelMeter::new(&cic);
    let mut samples = [0i32; BURST_SAMPLES];
    for n in 0..40u32 {
        microphone.clear();
        cic.reset();
        let mut len = 0usize;
        let mut overruns = 0u32;
        let mut busy_us = 0u64;
        for _ in 0..CHUNKS_PER_BURST {
            if microphone.read(&mut chunk).await.is_err() {
                overruns += 1;
                microphone.clear();
                continue;
            }
            let t0 = Instant::now();
            for word in &chunk {
                let byte = [(word >> (8 * mic::MIC_BYTE_LANE)) as u8];
                cic.push_bytes(&byte, order, |s| {
                    meter.push(s);
                    if len < samples.len() {
                        samples[len] = s;
                        len += 1;
                    }
                });
            }
            busy_us += t0.elapsed().as_micros();
        }
        let block = block_centi_dbfs(&samples[..len], cic.full_scale() as f32);
        let streaming = meter.take_centi_dbfs().unwrap_or(i16::MIN);
        say!(
            vcom,
            "[DK][mic] burst n={n} overruns={overruns} cpu={}us/chunk block={block} meter={streaming} cdBFS",
            busy_us / CHUNKS_PER_BURST as u64
        );
        Timer::after_millis(500).await;
    }
    microphone.stop();
    say!(vcom, "[DK][mic] DONE");
    lab::halt()
}

/// Level of one block after removing its own mean, in centi-dBFS.
fn block_centi_dbfs(samples: &[i32], full_scale: f32) -> i16 {
    if samples.is_empty() {
        return i16::MIN;
    }
    let n = samples.len() as f32;
    let mean = samples.iter().map(|&s| s as f32).sum::<f32>() / n;
    let var = samples
        .iter()
        .map(|&s| {
            let d = (s as f32 - mean) / full_scale;
            d * d
        })
        .sum::<f32>()
        / n;
    if var <= 0.0 {
        return -12_000;
    }
    (libm::log10f(var) * 1000.0) as i16
}
