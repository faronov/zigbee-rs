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
use stm32wb_hal::aes::AesEngine;
use stm32wb_hal::embassy_stm32_wpan::sub::mm::MemoryManager;
use stm32wb_hal::hse::{HseTuneError, HseTuning};
use stm32wb_hal::mac::WbTransport;

/// Returns mailbox buffers to CPU2; must run for the life of the MAC.
#[embassy_executor::task]
async fn run_memory_manager(mut memory_manager: MemoryManager<'static>) {
    memory_manager.run_queue().await
}

/// CPU1's AES1 engine for the MAC transport (Zigbee CCM*/AES-MMO); halts
/// when it cannot be claimed. [`stm32wb_hal::mac::start`] runs its
/// known-answer test before CPU2 boots.
pub fn aes_engine() -> AesEngine {
    AesEngine::take().unwrap_or_else(|error| {
        ::log::error!("[WB] AES1 unavailable: {error:?}");
        platform::halt()
    })
}

/// Log the outcome of [`stm32wb_hal::hse::apply_otp_tuning`], which the
/// composition root runs before `embassy_stm32::init` (before the logger
/// exists). A missing or rejected OTP value is not fatal: HSE then keeps
/// its reset load capacitance, as before the tuning was applied.
pub fn log_hse_tuning(tuning: &Result<HseTuning, HseTuneError>) {
    match tuning {
        Ok(HseTuning::Applied {
            tuning,
            previous,
            hse_was_running,
            unlock_reads,
            verify_reads,
        }) => ::log::info!(
            "[WB] HSE tuning {tuning} from OTP (was {previous}, reads {unlock_reads}/{verify_reads}{})",
            if *hse_was_running { ", HSE on" } else { "" }
        ),
        Ok(HseTuning::NoOtpRecord { current }) => {
            ::log::warn!("[WB] no OTP HSE tuning; HSETUNE {current}")
        }
        Err(HseTuneError::OutOfRange { offset, value }) => {
            ::log::warn!("[WB] OTP HSE tuning {value} at +{offset} invalid")
        }
        Err(HseTuneError::WriteRejected { wanted, read_back }) => {
            ::log::warn!("[WB] HSECR rejected tuning {wanted} (reads {read_back})")
        }
    }
}

/// Claim CLK48 (HSEM 5) for `usage` before CPU2 boots; halts with the
/// reason on the log when the clock tree does not match. Pass the claim to
/// [`stm32wb_hal::mac::start`].
pub fn claim_clk48(usage: stm32wb_hal::clk48::Clk48Use) -> stm32wb_hal::clk48::Clk48Claim {
    stm32wb_hal::clk48::claim(usage).unwrap_or_else(|error| {
        ::log::error!("[WB] CLK48 unavailable: {error:?}");
        platform::halt()
    })
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
    ::log::info!("[WB] AES1 known-answer test passed; coprocessor MAC running");
    transport
}
