//! STM32WB CPU1 support for zigbee-rs.
//!
//! On STM32WB the 2.4 GHz radio belongs to CPU2, which runs ST's IEEE
//! 802.15.4 MAC firmware. This crate supplies everything the CPU1
//! application needs around it, without product or board policy:
//!
//! - [`mac`]: the [`zigbee_mac::stm32wb::WbMacTransport`] over the IPCC
//!   mailbox (`embassy-stm32-wpan`), coprocessor start-up, entropy from the
//!   shared RNG and the monotonic clock;
//! - [`identity`]: the factory EUI-64;
//! - [`flash`]: dual-core-safe internal flash windows below the CPU2 secure
//!   area, presented through [`word_flash::WordPerDoubleWord`] so the
//!   zigbee-runtime journals work on 64-bit-programmed flash;
//! - [`hsem`]: the hardware semaphores shared with CPU2;
//! - [`hse`]: the factory HSE load-capacitor tuning from OTP, applied
//!   before the clock tree starts HSE;
//! - [`aes`]: the CPU1 AES1 engine behind Zigbee CCM* and AES-MMO (the
//!   MAC transport owns it; no software AES is linked).
//!
//! Only [`word_flash`], the AES1 register protocol and the OTP/`HSECR`
//! logic of [`hse`] (host-tested against models) are hardware
//! independent; the rest requires one chip feature (`stm32wb55vg`,
//! `stm32wb55rg`, `stm32wb55cg`).

#![no_std]

pub mod word_flash;

#[cfg(any(feature = "_chip", test))]
pub mod aes;

#[cfg(any(feature = "_chip", test))]
pub mod clk48;

#[cfg(feature = "_chip")]
pub mod flash;
pub mod hse;
#[cfg(feature = "_chip")]
pub mod hsem;
#[cfg(feature = "_chip")]
pub mod identity;
#[cfg(feature = "_chip")]
pub mod mac;
#[cfg(feature = "_chip")]
mod st_abi;

#[cfg(feature = "_chip")]
pub use embassy_stm32;
#[cfg(feature = "_chip")]
pub use embassy_stm32_wpan;
