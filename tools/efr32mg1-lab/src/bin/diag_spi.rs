#![no_std]
#![no_main]

#[path = "../shared/fault.rs"]
mod fault;
#[path = "../shared/platform.rs"]
mod platform;
#[path = "../shared/vectors.rs"]
mod vectors;

use cortex_m as _;
use embedded_hal::spi::SpiBus;

#[cortex_m_rt::entry]
fn main() -> ! {
    platform::init_small!("diag-spi");
    let resources = match efr32mg1_tradfri::flash_spi() {
        Ok(resources) => resources,
        Err(error) => {
            rtt_target::rprintln!("[EFR32][diag-spi] INIT_FAIL error={:?}", error);
            platform::halt()
        }
    };
    let mut bus = resources.bus;
    let chip_select = resources.chip_select;
    let mut jedec_id = [0u8; 3];

    chip_select.set_low();
    let result = bus
        .write(&[0x9F])
        .and_then(|()| bus.read(&mut jedec_id))
        .and_then(|()| bus.flush());
    chip_select.set_high();

    match result {
        Ok(()) if jedec_id != [0; 3] && jedec_id != [0xFF; 3] => {
            rtt_target::rprintln!(
                "[EFR32][diag-spi] PASS controller=USART0 hz={} jedec={:02X?}",
                bus.actual_bus_hz(),
                jedec_id
            );
            platform::led_on();
        }
        Ok(()) => rtt_target::rprintln!(
            "[EFR32][diag-spi] NO_DEVICE controller=USART0 hz={} jedec={:02X?}",
            bus.actual_bus_hz(),
            jedec_id
        ),
        Err(error) => rtt_target::rprintln!("[EFR32][diag-spi] TRANSFER_FAIL error={:?}", error),
    }
    platform::halt()
}
