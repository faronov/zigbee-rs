//! Host transport selection and UART policy of the NCP product.
//!
//! Exactly one ZRHP transport is active per image (spec 11.4):
//!
//! * `transport-usj` — USB-Serial-JTAG CDC-ACM (ESP32-H2-DevKitM-1's USB
//!   port, Waveshare ESP32-H2-Zero). No line rate, no RTS/CTS; USB NAK and
//!   ZRHP credits are the flow control. The chip's DTR/RTS reset stays on.
//! * `transport-uart` — a UART with UHCI DMA receive. On the Thread Border
//!   Router board: UART0 to the ESP32-S3 at 460 800 without flow control
//!   (RTS/CTS only with the reserved-line board mod). Elsewhere: 921 600
//!   with RTS/CTS behind a USB-UART bridge.

#[cfg(all(feature = "transport-usj", feature = "transport-uart"))]
compile_error!("select exactly one ZRHP transport: transport-usj or transport-uart");
#[cfg(not(any(feature = "transport-usj", feature = "transport-uart")))]
compile_error!("select exactly one ZRHP transport: transport-usj or transport-uart");
#[cfg(all(feature = "transport-usj", feature = "thread-br-h2"))]
compile_error!("the Thread BR board's H2 talks to its host over UART0: use transport-uart");
#[cfg(all(feature = "transport-uart", not(feature = "thread-br-h2")))]
compile_error!(
    "transport-uart needs a board with a described host UART (thread-br-h2); \
     a devkit UART behind a USB-UART bridge with RTS/CTS has no pin map yet"
);

/// The two ZRHP transports (`SYS_GET_INFO.transport` codes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Transport {
    UsbSerialJtag = 1,
    Uart = 2,
}

/// The transport this image is built for.
#[cfg(feature = "transport-usj")]
pub const SELECTED: Transport = Transport::UsbSerialJtag;
#[cfg(feature = "transport-uart")]
pub const SELECTED: Transport = Transport::Uart;

/// Byte appended before `WR_DONE` when a USJ burst ends on a full 64-byte
/// packet: an empty COBS frame (`0x00`), which every ZRHP receiver ignores.
pub const USJ_BURST_FILLER: Option<u8> = Some(0x00);

/// Whether the board wires RTS/CTS on the host UART (capability bit 22 and
/// `SYS_GET_INFO.uart_flow` describe the *active* setting).
pub const UART_FLOW_WIRED: bool = cfg!(feature = "rtscts-reserved-lines");

/// Product default line rate (volatile `SYS_SET_UART` changes revert to it
/// on reboot).
pub const UART_DEFAULT_BAUD: u32 = 460_800;
/// Product default flow control: off on the Thread BR board even with the
/// board mod, until gate H8 has checked R19/R20 (the host enables it).
pub const UART_DEFAULT_FLOW: bool = false;

/// Rates `SYS_SET_UART` accepts on the ESP32-H2 (spec 4.1/4.4): 2 000 000
/// only with RTS/CTS; 3 000 000 stays off until gate H2 measures it.
pub const fn uart_rate_supported(baud: u32, flow_control: bool) -> bool {
    match baud {
        460_800 | 921_600 => true,
        2_000_000 => flow_control,
        _ => false,
    }
}

/// `SYS_SET_UART` admission (spec 7.1): `Ok` or the ZRHP refusal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartChangeRefusal {
    /// Not a UART image, or RTS/CTS requested without the wiring
    /// (`UNSUPPORTED_COMMAND`).
    Unsupported,
    /// Rate outside the supported set (`INVALID_PARAMETER`).
    InvalidRate,
}

pub const fn check_uart_change(baud: u32, flow_control: bool) -> Result<(), UartChangeRefusal> {
    if !matches!(SELECTED, Transport::Uart) || (flow_control && !UART_FLOW_WIRED) {
        return Err(UartChangeRefusal::Unsupported);
    }
    if !uart_rate_supported(baud, flow_control) {
        return Err(UartChangeRefusal::InvalidRate);
    }
    Ok(())
}

/// No valid frame for this long after a rate switch ⇒ revert (spec 7.1).
pub const UART_SWITCH_REVERT_MS: u32 = 3_000;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rates_follow_the_spec_table() {
        assert!(uart_rate_supported(460_800, false));
        assert!(uart_rate_supported(921_600, false));
        assert!(!uart_rate_supported(2_000_000, false));
        assert!(uart_rate_supported(2_000_000, true));
        assert!(!uart_rate_supported(3_000_000, true));
        assert!(!uart_rate_supported(115_200, false));
    }

    #[test]
    fn default_rate_is_admissible() {
        assert!(uart_rate_supported(UART_DEFAULT_BAUD, UART_DEFAULT_FLOW));
    }

    #[cfg(feature = "transport-usj")]
    #[test]
    fn usj_images_refuse_uart_changes() {
        assert_eq!(
            check_uart_change(921_600, false),
            Err(UartChangeRefusal::Unsupported)
        );
    }

    #[cfg(all(feature = "transport-uart", not(feature = "rtscts-reserved-lines")))]
    #[test]
    fn flow_control_needs_the_wiring() {
        assert_eq!(check_uart_change(921_600, false), Ok(()));
        assert_eq!(
            check_uart_change(921_600, true),
            Err(UartChangeRefusal::Unsupported)
        );
        assert_eq!(
            check_uart_change(1_000_000, false),
            Err(UartChangeRefusal::InvalidRate)
        );
    }
}
