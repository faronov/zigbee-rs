//! Microphone task: a sound level once per [`PERIOD_MS`], published to
//! [`shared::SENSORS`] in 0.01 dB SPL.
//!
//! The SAI DMA runs continuously into a ring (the board's rule: the
//! MP23DB01HP needs its clock running to stay settled, and an SAI overrun
//! with the DMA idle stops reception). Each burst drops the stale ring,
//! decimates [`CHUNKS_PER_BURST`] chunks (~41 ms of audio) with a 64x CIC
//! filter and converts the RMS level with the microphone's nominal
//! sensitivity. The level is uncalibrated and unweighted (no A-weighting).

use embassy_time::Timer;
use static_cell::ConstStaticCell;
use stm32wb5mm_dk::mic::{self, MicPins, Microphone};
use zigbee_pdm::{BitOrder, Cic, LevelMeter};

use crate::platform::Irqs;
use crate::shared;

const PERIOD_MS: u64 = 1_000;
const CHUNK: usize = 1_024;
const CHUNKS_PER_BURST: usize = 8;
const DECIMATION: u16 = 64;
const ORDER: BitOrder = if mic::MIC_BIT_ORDER_MSB {
    BitOrder::Msb
} else {
    BitOrder::Lsb
};

static RING: ConstStaticCell<[u32; 2 * CHUNK]> = ConstStaticCell::new([0; 2 * CHUNK]);

#[embassy_executor::task]
pub async fn run(pins: MicPins) -> ! {
    let mut microphone = Microphone::new(pins, Irqs, RING.take());
    let Some(mut cic) = Cic::new(DECIMATION) else {
        log::error!("[WB] microphone: invalid CIC decimation");
        crate::platform::halt()
    };
    let mut meter = LevelMeter::new(&cic);
    let mut chunk = [0u32; CHUNK];
    microphone.start();
    Timer::after_millis(mic::SETTLE_MS).await;

    loop {
        microphone.clear();
        cic.reset();
        meter.reset();
        for _ in 0..CHUNKS_PER_BURST {
            if microphone.read(&mut chunk).await.is_err() {
                microphone.clear();
                continue;
            }
            for word in &chunk {
                let byte = [(word >> (8 * mic::MIC_BYTE_LANE)) as u8];
                cic.push_bytes(&byte, ORDER, |sample| meter.push(sample));
            }
        }
        let level = meter
            .take_centi_dbfs()
            .map(|dbfs| zigbee_pdm::spl_centi_db(dbfs, mic::SENSITIVITY_CENTI_DBFS));
        shared::SENSORS.update(|s| s.sound_centi_db = level);
        Timer::after_millis(PERIOD_MS).await;
    }
}
