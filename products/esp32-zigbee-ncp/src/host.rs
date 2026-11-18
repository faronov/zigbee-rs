//! The product's ZRHP host transport: `esp-serial-stream` behind the
//! application's [`HostTransport`] trait, selected at build time
//! ([`crate::transport::SELECTED`]).

#[cfg(feature = "transport-usj")]
use esp_serial_stream::usb_burst::USB_PACKET;
use ncp_coordinator_app::transport::{HostTransport, LineFaults, TransportInfo, UartChangeError};

use crate::transport::UartChangeRefusal;
#[cfg(feature = "transport-uart")]
use crate::transport::check_uart_change;

/// USB-Serial-JTAG CDC-ACM.
#[cfg(feature = "transport-usj")]
pub struct Host {
    stream: esp_serial_stream::usj::UsjStream,
}

#[cfg(feature = "transport-usj")]
impl Host {
    /// Take over the USB-Serial-JTAG peripheral (once).
    pub fn new(usb: esp_hal::peripherals::USB_DEVICE<'static>) -> Option<Self> {
        let stream =
            esp_serial_stream::usj::UsjStream::new(usb, crate::transport::USJ_BURST_FILLER)?;
        Some(Self { stream })
    }

    /// A host opened/closed the port or the bus was reset since the last
    /// call (a hint only; the link layer decides).
    pub fn take_line_activity(&mut self) -> bool {
        let events = self.stream.take_line_events();
        events.control_lines_changed || events.bus_reset
    }
}

#[cfg(feature = "transport-usj")]
impl HostTransport for Host {
    fn read(&mut self, buf: &mut [u8]) -> usize {
        self.stream.read(buf)
    }

    fn write(&mut self, data: &[u8]) -> usize {
        // Stay within one USB packet per call so `end_burst` can still add
        // its filler when needed.
        let chunk = data.len().min(USB_PACKET as usize);
        self.stream.write(&data[..chunk])
    }

    fn end_burst(&mut self) -> bool {
        self.stream.end_burst()
    }

    fn info(&self) -> TransportInfo {
        TransportInfo {
            kind: 1,
            baud: 0,
            flow_control: false,
        }
    }
}

/// UART (UHCI GDMA in both directions).
#[cfg(feature = "transport-uart")]
pub struct Host {
    stream: esp_serial_stream::uart::UartDmaStream,
}

#[cfg(feature = "transport-uart")]
impl Host {
    pub fn new(stream: esp_serial_stream::uart::UartDmaStream) -> Self {
        Self { stream }
    }
}

#[cfg(feature = "transport-uart")]
impl HostTransport for Host {
    fn read(&mut self, buf: &mut [u8]) -> usize {
        self.stream.read(buf)
    }

    fn write(&mut self, data: &[u8]) -> usize {
        self.stream.write(data)
    }

    fn take_line_faults(&mut self) -> LineFaults {
        let errors = self.stream.take_line_faults();
        LineFaults {
            errors,
            overruns: 0,
        }
    }

    fn info(&self) -> TransportInfo {
        let line = self.stream.line();
        TransportInfo {
            kind: 2,
            baud: line.baud,
            flow_control: line.flow_control,
        }
    }

    fn check_uart(&self, baud: u32, flow_control: bool) -> Result<(), UartChangeError> {
        check_uart_change(baud, flow_control).map_err(refusal)
    }

    fn apply_uart(&mut self, baud: u32, flow_control: bool) -> Result<(), UartChangeError> {
        check_uart_change(baud, flow_control).map_err(refusal)?;
        self.stream
            .set_line(esp_serial_stream::uart::LineConfig { baud, flow_control })
            .map_err(|_| UartChangeError::Busy)
    }
}

#[cfg_attr(feature = "transport-usj", allow(dead_code))]
fn refusal(refusal: UartChangeRefusal) -> UartChangeError {
    match refusal {
        UartChangeRefusal::Unsupported => UartChangeError::Unsupported,
        UartChangeRefusal::InvalidRate => UartChangeError::InvalidRate,
    }
}

#[cfg(feature = "transport-usj")]
const _: fn() -> LineFaults = LineFaults::default;
