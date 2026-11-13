//! Firmware composition shared by the STM32WB Zigbee routers
//! (STM32WB5MM-DK, Nucleo-WB55, USB dongle).
//!
//! CPU2 runs ST's IEEE 802.15.4 MAC firmware: it sends Imm-Acks, answers
//! data polls with Frame Pending, holds indirect frames for sleepy children
//! and transmits beacons. [`zigbee_mac::stm32wb::Stm32wbMac`] maps that MAC
//! onto zigbee-rs's parent MAC contract, and [`router_app::ParentRouterApp`]
//! runs the Zigbee router on top.
//!
//! - [`log`]: non-blocking `log` backend, drained to a UART or USB CDC;
//! - [`platform`]: status, diagnostics, supervisor and step-error policy;
//! - [`router`]: composition, shared button actions and the run loop;
//! - [`leds`]: status on three discrete LEDs;
//! - [`usb_log`] (feature `usb-log`): the log on a USB CDC ACM port.
//!
//! Each example's `main.rs` stays the composition root: it initializes the
//! chip with its board's clock tree, binds interrupts, constructs the
//! board resources and spawns the tasks. The chip package comes from the
//! example (`stm32wb-hal/stm32wb55xx`), through feature unification.

#![no_std]

pub mod leds;
pub mod log;
pub mod platform;
pub mod router;
#[cfg(feature = "usb-log")]
pub mod usb_log;

use embassy_executor::Spawner;
use stm32wb_hal::embassy_stm32_wpan::sub::mm::MemoryManager;
use stm32wb_hal::mac::WbTransport;

/// Returns mailbox buffers to CPU2; must run for the life of the MAC.
#[embassy_executor::task]
async fn run_memory_manager(mut memory_manager: MemoryManager<'static>) {
    memory_manager.run_queue().await
}

/// Hand back CPU2's started MAC transport after spawning its memory
/// manager; halts (with the reason on the log) when the coprocessor did not
/// come up. Call it on the result of [`stm32wb_hal::mac::start`].
pub fn mac_started(
    spawner: Spawner,
    started: Result<(WbTransport, MemoryManager<'static>), stm32wb_hal::mac::StartError>,
) -> WbTransport {
    let (transport, memory_manager) = match started {
        Ok(parts) => parts,
        Err(error) => {
            ::log::error!("[WB] coprocessor MAC start failed: {error:?}");
            platform::halt()
        }
    };
    spawner.spawn(run_memory_manager(memory_manager).unwrap_or_else(|_| platform::halt()));
    ::log::info!("[WB] coprocessor MAC running");
    transport
}
