//! MP23DB01HP digital MEMS microphone on SAI1 block A in PDM mode
//! (ST BSP `stm32wb5mm_dk_audio.c`, UM2825): PDM clock CK2 on PA8 and
//! data D2 on PA9, both AF3. Register setup matches the BSP (32-bit
//! frames, 4 slots, slot 1 active, 2 microphone pairs, CK2).
//!
//! One 32-bit word arrives per SAI frame; the microphone's 8 PDM bits sit
//! in byte lane [`MIC_BYTE_LANE`], earliest bit first in the order given
//! by [`MIC_BIT_ORDER_MSB`]. The product runs its own CIC decimator.
//!
//! The DMA runs in circular mode into a caller-provided ring while the
//! microphone is started, so the SAI FIFO never overruns (an overrun with
//! the DMA idle stops SAI reception until the block is re-enabled,
//! observed on the DK). The MP23DB01HP output needs more than 80 ms after
//! its clock starts to settle, so start it once and read bursts with
//! [`Microphone::clear`] + [`Microphone::read`]; measured with
//! `stm32wb5mm-dk-diag-mic`.

use stm32wb_hal::embassy_stm32::dma::{self, ReadableRingBuffer, TransferOptions};
use stm32wb_hal::embassy_stm32::gpio::{AfType, Flex, OutputType, Pull, Speed};
use stm32wb_hal::embassy_stm32::interrupt::typelevel::{Binding, DMA1_CHANNEL2};
use stm32wb_hal::embassy_stm32::pac;
use stm32wb_hal::embassy_stm32::pac::sai::vals;
use stm32wb_hal::embassy_stm32::{Peri, peripherals, rcc, sai};

/// SAI kernel clock: PLL "P" output selected by [`crate::rcc_config`].
pub const SAI_KERNEL_HZ: u32 = 64_000_000;
/// SAI bit-clock divider (NODIV = 1: SCK = kernel / MCKDIV).
const MCKDIV: vals::Mckdiv = vals::Mckdiv::Div10;
/// SAI bit clock: 6.4 MHz.
pub const SAI_SCK_HZ: u32 = SAI_KERNEL_HZ / 10;
/// PDM clock on CK2 = SCK / (2 × microphone pairs): 1.6 MHz (the
/// MP23DB01HP normal mode accepts 1.2–3.25 MHz; the BSP uses 1.536 MHz).
pub const PDM_CLOCK_HZ: u32 = SAI_SCK_HZ / 4;
/// 32-bit words per second delivered to the DMA (one per frame).
pub const WORDS_PER_SECOND: u32 = SAI_SCK_HZ / 32;
/// Byte lane of each word that carries this board's microphone (D2). Lanes
/// 0 and 1 read identical bytes on the DK (the second edge samples the
/// held data line); lanes 2 and 3 read 0.
pub const MIC_BYTE_LANE: usize = 1;
/// The earliest PDM sample is the most significant bit of each byte lane
/// (32-bit MSB-first frames). Measured: MSB order gives the noise-shaped,
/// ≥10 dB quieter stream; LSB order leaks modulator noise in band.
pub const MIC_BIT_ORDER_MSB: bool = true;
/// Microphone turn-on time to discard after [`Microphone::start`]
/// (measured: the level is still falling 80 ms after the clock starts).
pub const SETTLE_MS: u64 = 250;
/// MP23DB01HP sensitivity: −41 dBFS at 94 dB SPL (datasheet typical,
/// ±1 dB), in 0.01 dB, relative to the full scale of a 1-bit PDM stream.
/// Nominal: the board is not acoustically calibrated.
pub const SENSITIVITY_CENTI_DBFS: i16 = -4_100;

/// SAI1 resources of the microphone.
pub struct MicPins {
    pub sai1: Peri<'static, peripherals::SAI1>,
    pub clock: Peri<'static, peripherals::PA8>,
    pub data: Peri<'static, peripherals::PA9>,
    pub dma: Peri<'static, peripherals::DMA1_CH2>,
}

/// The microphone: SAI1 block A configured for PDM, plus its DMA channel.
pub struct Microphone {
    _clock: Flex<'static>,
    _data: Flex<'static>,
    _sai1: Peri<'static, peripherals::SAI1>,
    ring: ReadableRingBuffer<'static, u32>,
    running: bool,
}

/// The DMA overwrote words before they were read: the reader fell behind
/// by more than half the ring. Call [`Microphone::clear`] and read again.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Overrun;

fn block() -> pac::sai::Ch {
    pac::SAI1.ch(0)
}

impl Microphone {
    /// Enable SAI1 and configure block A like the ST BSP. `ring` receives
    /// one word per SAI frame in circular DMA mode; its length must be
    /// even and is best twice the chunk passed to [`Self::read`]. No clock
    /// is driven until [`Self::start`].
    pub fn new(
        pins: MicPins,
        irq: impl Binding<DMA1_CHANNEL2, dma::InterruptHandler<peripherals::DMA1_CH2>> + 'static,
        ring: &'static mut [u32],
    ) -> Self {
        rcc::enable_and_reset::<peripherals::SAI1>();

        let mut clock = Flex::new(pins.clock);
        clock.set_as_af_unchecked(3, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        let mut data = Flex::new(pins.data);
        data.set_as_af_unchecked(3, AfType::input(Pull::Up));

        <peripherals::DMA1_CH2 as sai::Dma<peripherals::SAI1, sai::A>>::remap(&pins.dma);
        let request =
            <peripherals::DMA1_CH2 as sai::Dma<peripherals::SAI1, sai::A>>::request(&pins.dma);
        let channel = dma::Channel::new(pins.dma, irq);

        let ch = block();
        ch.cr1().write(|w| {
            w.set_mode(vals::Mode::MasterRx);
            w.set_prtcfg(vals::Prtcfg::Free);
            w.set_ds(vals::Ds::Bit32);
            w.set_lsbfirst(vals::Lsbfirst::MsbFirst);
            w.set_ckstr(vals::Ckstr::FallingEdge);
            w.set_syncen(vals::Syncen::Asynchronous);
            w.set_mono(vals::Mono::Stereo);
            w.set_outdriv(vals::Outdriv::OnStart);
            w.set_nodiv(true);
            w.set_mckdiv(MCKDIV);
        });
        ch.cr2().write(|w| w.set_fth(vals::Fth::Empty));
        ch.frcr().write(|w| {
            w.set_frl(31);
            w.set_fsall(0);
            w.set_fsdef(false);
            w.set_fspol(vals::Fspol::RisingEdge);
            w.set_fsoff(vals::Fsoff::OnFirst);
        });
        ch.slotr().write(|w| {
            w.set_fboff(0);
            w.set_slotsz(vals::Slotsz::DataSize);
            w.set_nbslot(3);
            w.set_sloten(vals::Sloten::from_bits(0x0002));
        });
        pac::SAI1.pdmcr().write(|w| {
            w.set_micnbr(1);
            w.set_cken(1, true);
            w.set_pdmen(true);
        });

        // SAFETY: DR is the SAI1 A data register and `ring` is 'static;
        // the ring buffer stops the channel when dropped.
        let ring = unsafe {
            ReadableRingBuffer::new(
                channel,
                request,
                ch.dr().as_ptr() as *mut u32,
                ring,
                TransferOptions::default(),
            )
        };

        Self {
            _clock: clock,
            _data: data,
            _sai1: pins.sai1,
            ring,
            running: false,
        }
    }

    /// Start the PDM clock and the circular DMA. The microphone needs
    /// [`SETTLE_MS`] before its level is meaningful.
    pub fn start(&mut self) {
        if self.running {
            return;
        }
        let ch = block();
        ch.cr2().modify(|w| w.set_fflush(true));
        ch.clrfr().write(|w| w.0 = 0x77);
        self.ring.start();
        ch.cr1().modify(|w| w.set_dmaen(true));
        ch.cr1().modify(|w| w.set_saien(true));
        self.running = true;
    }

    /// Stop the PDM clock (the microphone powers down) and the DMA.
    pub fn stop(&mut self) {
        if !self.running {
            return;
        }
        let ch = block();
        ch.cr1().modify(|w| {
            w.set_saien(false);
            w.set_dmaen(false);
        });
        // SAIEN reads back 1 until the current frame is complete.
        while ch.cr1().read().saien() {}
        self.ring.stop();
        while self.ring.is_running() {}
        ch.cr2().modify(|w| w.set_fflush(true));
        self.running = false;
    }

    /// Whether the microphone is clocked.
    pub const fn is_running(&self) -> bool {
        self.running
    }

    /// Drop everything buffered so the next [`Self::read`] returns fresh
    /// words.
    pub fn clear(&mut self) {
        self.ring.clear();
    }

    /// Wait for and copy the next `words.len()` words (one SAI frame
    /// each). Returns [`Overrun`] if the reader fell behind the DMA.
    pub async fn read(&mut self, words: &mut [u32]) -> Result<(), Overrun> {
        self.ring
            .read_exact(words)
            .await
            .map(|_| ())
            .map_err(|_| Overrun)
    }
}

impl Drop for Microphone {
    fn drop(&mut self) {
        self.stop();
    }
}
