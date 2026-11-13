//! P-NUCLEO-WB55 USB dongle Zigbee Router.
//!
//! A plain mains-powered router (Range Extender: Basic, Identify on
//! endpoint 1) over CPU2's IEEE 802.15.4 MAC; the composition, the button
//! actions and the run loop are shared with the other STM32WB routers
//! ([`stm32wb_router_app`]).
//!
//! Button ([`product`]): SW1 short press commissions, or opens permit-join
//! for 180 s once joined; a 3 s hold commits a journal-aware factory reset.
//! LED1-LED3 show the router status. Log (feature `usb-log`, default): the
//! dongle's own USB port as a CDC ACM serial port.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use stm32wb_hal::embassy_stm32::{self, bind_interrupts, ipcc};
use stm32wb_router_app::leds::{self, StatusLeds};
use stm32wb_router_app::platform::{self, halt};
use stm32wb_router_app::router::{self, Button, Identity, RouterCells};
use wb55_dongle_router_product::{self as product, policy};

#[cfg(feature = "usb-log")]
bind_interrupts!(struct Irqs {
    IPCC_C1_RX => ipcc::ReceiveInterruptHandler;
    IPCC_C1_TX => ipcc::TransmitInterruptHandler;
    USB_LP => embassy_stm32::usb::InterruptHandler<embassy_stm32::peripherals::USB>;
});

#[cfg(not(feature = "usb-log"))]
bind_interrupts!(struct Irqs {
    IPCC_C1_RX => ipcc::ReceiveInterruptHandler;
    IPCC_C1_TX => ipcc::TransmitInterruptHandler;
});

static CELLS: RouterCells<policy::RangeExtenderProfile> = RouterCells::new();

const IDENTITY: Identity = Identity {
    manufacturer: product::MANUFACTURER,
    model: product::ROUTER_MODEL,
    date_code: product::DATE_CODE,
    sw_build: product::SW_BUILD,
    role: product::role::ROUTER,
};

#[cfg(feature = "usb-log")]
mod usb {
    use stm32wb_router_app::usb_log::{self, LogClass, LogDevice};

    #[embassy_executor::task]
    pub async fn device(mut device: LogDevice) {
        device.run().await
    }

    #[embassy_executor::task]
    pub async fn drain(class: LogClass) {
        usb_log::drain(class).await
    }
}

#[embassy_executor::task]
async fn status_leds(leds: StatusLeds) {
    leds::run(leds).await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc = wb55_usb_dongle::rcc_config();
    let p = embassy_stm32::init(config);

    #[cfg(feature = "usb-log")]
    {
        let driver = wb55_usb_dongle::usb(p.USB, p.PA12, p.PA11, Irqs);
        let (device, class) = stm32wb_router_app::usb_log::build(
            driver,
            &stm32wb_router_app::usb_log::Descriptor {
                vid: product::USB_VID,
                pid: product::USB_PID,
                manufacturer: product::MANUFACTURER,
                product: product::ROUTER_MODEL,
            },
        );
        stm32wb_router_app::log::init();
        spawner.spawn(usb::device(device).unwrap_or_else(|_| halt()));
        spawner.spawn(usb::drain(class).unwrap_or_else(|_| halt()));
    }
    log::info!(
        "[WB] {} on the P-NUCLEO-WB55 USB dongle",
        product::ROUTER_MODEL
    );

    spawner.spawn(
        status_leds(StatusLeds {
            red: wb55_usb_dongle::led3_red(p.PB1),
            green: wb55_usb_dongle::led2_green(p.PB0),
            blue: wb55_usb_dongle::led1_blue(p.PA4),
            active_high: wb55_usb_dongle::LED_ACTIVE_HIGH,
        })
        .unwrap_or_else(|_| halt()),
    );
    let mut buttons = [Button::new(
        wb55_usb_dongle::button_sw1(p.PA10),
        wb55_usb_dongle::BUTTON_ACTIVE_LOW,
        &product::BUTTONS[0],
    )];

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
