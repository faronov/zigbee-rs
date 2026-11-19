//! The host byte stream as the NCP needs it: non-blocking in both
//! directions. Implemented by the platform (ESP32-H2: USB-Serial-JTAG or
//! UART with DMA receive, `esp-serial-stream`).

/// Line faults since the last poll (each one counts as a corrupted frame).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LineFaults {
    pub errors: u32,
    /// Of `errors`, receive FIFO/ring overruns.
    pub overruns: u32,
}

/// `SYS_GET_INFO` transport description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransportInfo {
    /// 1 USB-Serial-JTAG, 2 UART.
    pub kind: u8,
    /// 0 for USB.
    pub baud: u32,
    /// RTS/CTS active.
    pub flow_control: bool,
}

/// Why `SYS_SET_UART` was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartChangeError {
    /// Not a UART link, or flow control without the wiring.
    Unsupported,
    /// Rate outside the product's supported set.
    InvalidRate,
    /// The transport cannot switch now; retry.
    Busy,
}

pub trait HostTransport {
    /// Move received bytes into `buf`; 0 when none. Never blocks.
    fn read(&mut self, buf: &mut [u8]) -> usize;
    /// Accept as many bytes as fit now; 0 when the output is full. Never
    /// blocks.
    fn write(&mut self, data: &[u8]) -> usize;
    /// A frame boundary with nothing more to send right now (USB: issue a
    /// short packet). `false` = call again later.
    fn end_burst(&mut self) -> bool {
        true
    }
    fn take_line_faults(&mut self) -> LineFaults {
        LineFaults::default()
    }
    fn info(&self) -> TransportInfo;
    /// Check a `SYS_SET_UART` request without applying it.
    fn check_uart(&self, _baud: u32, _flow_control: bool) -> Result<(), UartChangeError> {
        Err(UartChangeError::Unsupported)
    }
    /// Apply a rate/flow change (after `apply_delay_ms`).
    fn apply_uart(&mut self, _baud: u32, _flow_control: bool) -> Result<(), UartChangeError> {
        Err(UartChangeError::Unsupported)
    }
}
