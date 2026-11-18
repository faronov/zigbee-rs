//! Non-blocking serial byte streams for the ESP32-H2/C6 host links.
//!
//! Two transports carry a host protocol (ZRHP) between a Zigbee NCP and its
//! host, and both must keep the Zigbee loop running whatever the host does:
//!
//! * [`usj::UsjStream`] — USB-Serial-JTAG CDC-ACM. Interrupt-driven receive
//!   ring with USB NAK back-pressure, non-blocking IN FIFO writes, short
//!   packet at every burst end ([`usb_burst`]).
//! * [`uart::UartDmaStream`] — UART with UHCI GDMA receive (keeps receiving
//!   through flash-erase stalls with interrupts masked) and DMA transmit,
//!   optional RTS/CTS, runtime rate changes.
//!
//! Nothing here knows about frames: the streams move bytes, count line
//! faults and report USB line activity. Pin routing belongs to the board
//! crate; which transport an image uses is a product decision.
//!
//! The pure parts ([`ring`], [`usb_burst`]) are host-tested. The drivers
//! exist only for the chip target and are build-checked; their behaviour on
//! hardware (USJ throughput and host-absent handling, UART DMA receive
//! under flash load, rate switching) is still an open hardware gate.

#![cfg_attr(not(test), no_std)]

pub mod ring;
pub mod usb_burst;

#[cfg(all(target_os = "none", any(feature = "esp32h2", feature = "esp32c6")))]
pub mod uart;
#[cfg(all(target_os = "none", any(feature = "esp32h2", feature = "esp32c6")))]
pub mod usj;

#[cfg(all(feature = "esp32h2", feature = "esp32c6"))]
compile_error!("select at most one of the esp32h2 or esp32c6 features");
