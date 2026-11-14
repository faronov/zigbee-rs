//! DK start-up: clocks, interrupts, the VCOM log and the board resources.
//! The capability adapters are shared ([`stm32wb_router_app::platform`]).

use embassy_executor::Spawner;
use stm32wb_hal::embassy_stm32::gpio::Input;
use stm32wb_hal::embassy_stm32::{self, bind_interrupts, dma, ipcc, peripherals};
pub use stm32wb_router_app::platform::halt;

bind_interrupts!(pub struct Irqs {
    IPCC_C1_RX => ipcc::ReceiveInterruptHandler;
    IPCC_C1_TX => ipcc::TransmitInterruptHandler;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    #[cfg(feature = "mic")]
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
});

/// Resources a composition needs once clocks, time and the log run.
pub struct Started {
    pub ipcc: embassy_stm32::Peri<'static, peripherals::IPCC>,
    pub rng: embassy_stm32::Peri<'static, peripherals::RNG>,
    pub button_b1: Input<'static>,
    pub button_b2: Input<'static>,
    #[cfg(any(feature = "rgb-led", feature = "lcd"))]
    pub spi1: stm32wb5mm_dk::spi1::Spi1Pins,
    #[cfg(feature = "touch")]
    pub touch: stm32wb5mm_dk::touch::TouchPins,
    #[cfg(feature = "sensors")]
    pub sensor_bus: stm32wb5mm_dk::sensors::SensorBus,
    #[cfg(feature = "mic")]
    pub mic: stm32wb5mm_dk::mic::MicPins,
}

/// HSE 32 MHz / PLL 64 MHz (the clock tree CPU2's radio requires), LSE for
/// the radio wake-up timer (plus the SAI1 clock with `mic`), and the VCOM
/// log.
pub fn start(spawner: Spawner, model: &str) -> Started {
    let mut config = embassy_stm32::Config::default();
    config.rcc = stm32wb5mm_dk::rcc_config();
    let p = embassy_stm32::init(config);

    let vcom = stm32wb5mm_dk::vcom_tx(p.USART1, p.PB6, p.DMA1_CH1, Irqs);
    stm32wb_router_app::log::init();
    spawner.spawn(drain_log(vcom).unwrap_or_else(|_| halt()));
    log::info!("[WB] {model} on STM32WB5MM-DK");

    Started {
        ipcc: p.IPCC,
        rng: p.RNG,
        button_b1: stm32wb5mm_dk::button_b1(p.PC12),
        button_b2: stm32wb5mm_dk::button_b2(p.PC13),
        #[cfg(any(feature = "rgb-led", feature = "lcd"))]
        spi1: stm32wb5mm_dk::spi1::Spi1Pins {
            spi1: p.SPI1,
            sck: p.PA1,
            mosi: p.PA7,
            lcd_cs: p.PH0,
            lcd_dc: p.PC9,
            lcd_rst: p.PC8,
            led_select: p.PH1,
        },
        #[cfg(feature = "touch")]
        touch: stm32wb5mm_dk::touch::TouchPins {
            tsc: p.TSC,
            key_sample: p.PD10,
            key: p.PD11,
            shield_sample: p.PC6,
            shield: p.PC7,
        },
        #[cfg(feature = "sensors")]
        sensor_bus: stm32wb5mm_dk::sensors::i2c3(p.I2C3, p.PB13, p.PB11),
        #[cfg(feature = "mic")]
        mic: stm32wb5mm_dk::mic::MicPins {
            sai1: p.SAI1,
            clock: p.PA8,
            data: p.PA9,
            dma: p.DMA1_CH2,
        },
    }
}

/// Move queued log bytes to VCOM.
#[embassy_executor::task]
async fn drain_log(tx: stm32wb5mm_dk::VcomTx) {
    stm32wb_router_app::log::drain_uart(tx).await
}
