//! Board and platform support for PHY62x2 development boards.

#![no_std]

#[cfg(all(feature = "phy6222", feature = "phy6252"))]
compile_error!("select exactly one of the phy6222 or phy6252 features");
#[cfg(not(any(feature = "phy6222", feature = "phy6252")))]
compile_error!("select exactly one of the phy6222 or phy6252 features");

pub mod pins;
pub mod storage;
pub mod time;
pub mod vectors;
