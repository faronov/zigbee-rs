//! Nucleo-WB55 Zigbee Router.
//!
//! A plain mains-powered router (Range Extender: Basic, Identify on
//! endpoint 1) over CPU2's IEEE 802.15.4 MAC; the composition, the button
//! actions and the run loop are shared with the other STM32WB routers
//! ([`stm32wb_router_app`]).
//!
//! Buttons ([`product`]): SW1 short press commissions, or opens permit-join
//! for 180 s once joined; a 3 s hold commits a journal-aware factory reset.
//! SW2 short press identifies for 10 s; a hold closes permit-join. SW3
//! restarts (the router resumes from its journals). LED1-LED3 show the
//! router status. Log: ST-LINK VCP (USART1, 115200 8N1).

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use nucleo_wb55_router_product::{self as product, policy};
use stm32wb_hal::embassy_stm32::{self, bind_interrupts, dma, ipcc, peripherals};
use stm32wb_router_app::leds::{self, StatusLeds};
use stm32wb_router_app::platform::{self, halt};
use stm32wb_router_app::router::{self, Button, Identity, RouterCells};

bind_interrupts!(struct Irqs {
    IPCC_C1_RX => ipcc::ReceiveInterruptHandler;
    IPCC_C1_TX => ipcc::TransmitInterruptHandler;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
});

static CELLS: RouterCells<policy::RangeExtenderProfile> = RouterCells::new();

const IDENTITY: Identity = Identity {
    manufacturer: product::MANUFACTURER,
    model: product::ROUTER_MODEL,
    date_code: product::DATE_CODE,
    sw_build: product::SW_BUILD,
    role: product::role::ROUTER,
};

#[embassy_executor::task]
async fn drain_log(tx: nucleo_wb55::VcomTx) {
    stm32wb_router_app::log::drain_uart(tx).await
}

#[embassy_executor::task]
async fn status_leds(leds: StatusLeds) {
    leds::run(leds).await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc = nucleo_wb55::rcc_config();
    let p = embassy_stm32::init(config);

    let vcom = nucleo_wb55::vcom_tx(p.USART1, p.PB6, p.DMA1_CH1, Irqs);
    stm32wb_router_app::log::init();
    spawner.spawn(drain_log(vcom).unwrap_or_else(|_| halt()));
    log::info!("[WB] {} on Nucleo-WB55", product::ROUTER_MODEL);

    spawner.spawn(
        status_leds(StatusLeds {
            red: nucleo_wb55::led3_red(p.PB1),
            green: nucleo_wb55::led2_green(p.PB0),
            blue: nucleo_wb55::led1_blue(p.PB5),
            active_high: nucleo_wb55::LED_ACTIVE_HIGH,
        })
        .unwrap_or_else(|_| halt()),
    );
    let mut buttons = [
        Button::new(
            nucleo_wb55::button_sw1(p.PC4),
            nucleo_wb55::BUTTON_ACTIVE_LOW,
            &product::BUTTONS[0],
        ),
        Button::new(
            nucleo_wb55::button_sw2(p.PD0),
            nucleo_wb55::BUTTON_ACTIVE_LOW,
            &product::BUTTONS[1],
        ),
        Button::new(
            nucleo_wb55::button_sw3(p.PD1),
            nucleo_wb55::BUTTON_ACTIVE_LOW,
            &product::BUTTONS[2],
        ),
    ];

    let transport = stm32wb_router_app::mac_started(
        spawner,
        stm32wb_hal::mac::start(p.IPCC, Irqs, p.RNG).await,
    );
    let (app, _settings) = router::compose(
        &CELLS,
        transport,
        &IDENTITY,
        policy::range_extender_profile,
        policy::declare_range_extender,
        platform::parts(leds::publish_status, product::SW1.name),
    );
    router::run(app, &mut buttons).await
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    stm32wb_router_app::log::panic(info)
}

#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    stm32wb_router_app::log::hard_fault(frame)
}
