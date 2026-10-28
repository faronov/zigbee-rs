//! Typed LP-EM-CC2340R5 LED and button wiring.
//!
//! The generic GPIO types come from [`cc2340r5_hal::gpio`] and are
//! re-exported here.

pub use cc2340r5_hal::gpio::*;

pub const LED1_DIO: u8 = 7;
pub const LED2_DIO: u8 = 6;
pub const BUTTON1_DIO: u8 = 13;
pub const BUTTON2_DIO: u8 = 14;

/// Physical LED1 resource on DIO7.
pub type Led1 = Led<LED1_DIO>;
/// Physical LED2 resource on DIO6.
pub type Led2 = Led<LED2_DIO>;
/// Physical BTN1 resource on DIO13.
pub type Button1 = Button<BUTTON1_DIO>;
/// Physical BTN2 resource on DIO14.
pub type Button2 = Button<BUTTON2_DIO>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn documented_board_pins_are_unique() {
        let pins = [LED1_DIO, LED2_DIO, BUTTON1_DIO, BUTTON2_DIO];
        for (index, pin) in pins.iter().enumerate() {
            assert!(*pin < DIO_COUNT);
            assert!(!SWD_DIOS.contains(pin));
            assert!(!pins[..index].contains(pin));
        }
    }
}
