//! TB-04 cold bring-up shared by the role images.

use tlsr8258_hal::peripherals::Aes;
use tlsr8258_tb04::leds::StatusLeds;
use tlsr8258_tb04::resources::{BoardResources, OnboardFlash};

/// Board resources a role image still needs after [`boot`].
pub struct Boot {
    /// Initialized, showing red.
    pub leds: StatusLeds,
    pub aes: Aes,
    pub flash: OnboardFlash,
    /// Factory EUI-64 with the role's first-byte offset applied.
    pub ieee_address: [u8; 8],
}

/// Start Timer0, take the board, light red, and install the PC5/Zbit flash
/// voltage guard before any persistent journal can be touched.
///
/// Each role image derives a distinct IEEE address from the factory EUI-64
/// with its own `eui_offset`, so their journals never share an identity.
///
/// Always inlined: as its own frame, it shifts LLVM's inlining of the shared
/// helpers and grows the sensor stacks. An image that profits from an
/// out-of-line bring-up (the End Device) wraps it in its own
/// `#[inline(never)]` function.
#[inline(always)]
pub fn boot(eui_offset: u8) -> Boot {
    tlsr8258_hal::timer::init();
    let resources = match BoardResources::take() {
        Some(resources) => resources,
        None => loop {
            core::hint::spin_loop();
        },
    };
    let leds = resources.lighting.into_status_leds();
    if leds.init().is_err() {
        failure(&leds);
    }
    let adc = match tlsr8258_hal::adc::Adc::new(
        resources.adc,
        tlsr8258_hal::flash::FlashGeometry::KiB512,
    ) {
        Ok(adc) => adc,
        Err(_) => failure(&leds),
    };
    if adc.install_flash_voltage_guard(resources.adc_pc5).is_err() {
        failure(&leds);
    }

    let mut ieee_address = [0u8; 8];
    tlsr8258_hal::flash::factory_ieee(&mut ieee_address);
    ieee_address[0] = ieee_address[0].wrapping_add(eui_offset);
    Boot {
        leds,
        aes: resources.aes,
        flash: resources.flash,
        ieee_address,
    }
}

/// Fail closed with a steady red LED.
pub fn failure(leds: &StatusLeds) -> ! {
    leds.green.write(false);
    leds.blue.write(false);
    leds.red.write(true);
    halted()
}

/// Park the CPU, leaving the LEDs as they are.
pub fn halted() -> ! {
    loop {
        tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(1_000));
    }
}
