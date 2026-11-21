//! CLK48, the 48 MHz kernel clock of USB and RNG, shared with CPU2.
//!
//! RM0434 (RCC): CLK48 is selected by `RCC_CCIPR.CLK48SEL`; for USB it
//! must stay within ±0.25 % of 48 MHz. ST's recommended USB source on
//! STM32WB is HSI48 trimmed by the CRS from the USB SOF packets
//! (`CRS_CFGR.SYNCSRC = USB`, `CRS_CR.AUTOTRIMEN | CEN`).
//!
//! AN5289 (hardware semaphores): semaphore 5 guards the CLK48
//! configuration (`RCC_CRRCR`, `CLK48SEL`). CPU2's wireless firmware also
//! draws from the RNG, whose kernel clock is CLK48; it may change or switch
//! off that clock when it is done unless CPU1 holds semaphore 5. A CPU1
//! that runs USB therefore takes semaphore 5 before CPU2 boots and keeps it
//! for as long as USB runs (for these images: until reset).
//!
//! CPU1's RNG (`mac::start`, `fill_random`) uses CLK48 too, on every board:
//! if CPU2 switched HSI48 off, CPU1's RNG would stop delivering. So every
//! STM32WB image claims CLK48 before CPU2 boots, and `mac::start` takes the
//! [`Clk48Claim`] as proof.
//!
//! [`claim`] takes the semaphore (bounded) and checks the clock tree the
//! board's RCC configuration set up: HSI48 ready and CLK48SEL = HSI48, plus
//! (for [`Clk48Use::Usb`]) the CRS trimming from the USB SOF. Any mismatch is
//! an error; the claim is never silently accepted.

#[cfg(feature = "_chip")]
use embassy_stm32::pac;

#[cfg(feature = "_chip")]
use crate::hsem;

/// Bounded attempts to take semaphore 5. CPU2 is not running yet when the
/// composition root claims CLK48, so the first attempt normally succeeds.
#[cfg(feature = "_chip")]
pub const CLAIM_ATTEMPTS: u32 = 10_000;

/// Why CLK48 cannot be used for USB.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Clk48Error {
    /// Semaphore 5 is held by another core/process.
    SemaphoreBusy,
    /// HSI48 is not running.
    Hsi48NotReady,
    /// CLK48SEL does not select HSI48.
    NotHsi48,
    /// The CRS is not enabled to trim HSI48 from the USB SOF.
    CrsNotSyncedToUsb,
}

/// Proof that CPU1 owns the CLK48 configuration. It cannot be dropped:
/// semaphore 5 stays taken until reset.
#[cfg(feature = "_chip")]
pub struct Clk48Claim {
    usage: Clk48Use,
}

#[cfg(feature = "_chip")]
impl Clk48Claim {
    /// What the claim was validated for.
    pub fn usage(&self) -> Clk48Use {
        self.usage
    }
}

/// What CPU1 runs on CLK48.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Clk48Use {
    /// The RNG only (boards that log through the ST-LINK VCP).
    Rng,
    /// USB and the RNG: HSI48 must also be trimmed by the CRS from USB.
    Usb,
}

/// Snapshot of the registers [`check`] validates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Clk48State {
    pub hsi48_ready: bool,
    pub clk48_is_hsi48: bool,
    pub crs_enabled: bool,
    pub crs_autotrim: bool,
    pub crs_sync_usb: bool,
}

/// Validate a CLK48 snapshot for `usage`.
pub fn check(state: &Clk48State, usage: Clk48Use) -> Result<(), Clk48Error> {
    if !state.hsi48_ready {
        return Err(Clk48Error::Hsi48NotReady);
    }
    if !state.clk48_is_hsi48 {
        return Err(Clk48Error::NotHsi48);
    }
    if usage == Clk48Use::Usb && !(state.crs_enabled && state.crs_autotrim && state.crs_sync_usb) {
        return Err(Clk48Error::CrsNotSyncedToUsb);
    }
    Ok(())
}

#[cfg(feature = "_chip")]
fn read_state() -> Clk48State {
    let crs_cr = pac::CRS.cr().read();
    Clk48State {
        hsi48_ready: pac::RCC.crrcr().read().hsi48rdy(),
        clk48_is_hsi48: pac::RCC.ccipr().read().clk48sel() == pac::rcc::vals::Clk48sel::Hsi48,
        crs_enabled: crs_cr.cen(),
        crs_autotrim: crs_cr.autotrimen(),
        crs_sync_usb: pac::CRS.cfgr().read().syncsrc() == pac::crs::vals::Syncsrc::Usb,
    }
}

/// Take semaphore 5 for good and check that CLK48 is HSI48 (trimmed by the
/// CRS from USB for [`Clk48Use::Usb`]). Call after `embassy_stm32::init`
/// and before CPU2 boots (`mac::start`). On error the semaphore is released
/// again.
#[cfg(feature = "_chip")]
pub fn claim(usage: Clk48Use) -> Result<Clk48Claim, Clk48Error> {
    hsem::enable();
    let guard = hsem::Guard::acquire(hsem::CLK48_CONFIG, CLAIM_ATTEMPTS)
        .ok_or(Clk48Error::SemaphoreBusy)?;
    check(&read_state(), usage)?;
    // Keep semaphore 5 until reset: CPU2 must not change or stop CLK48.
    core::mem::forget(guard);
    Ok(Clk48Claim { usage })
}

#[cfg(test)]
mod tests {
    use super::*;

    const GOOD: Clk48State = Clk48State {
        hsi48_ready: true,
        clk48_is_hsi48: true,
        crs_enabled: true,
        crs_autotrim: true,
        crs_sync_usb: true,
    };

    #[test]
    fn hsi48_with_crs_from_usb_is_accepted() {
        assert_eq!(check(&GOOD, Clk48Use::Usb), Ok(()));
        assert_eq!(check(&GOOD, Clk48Use::Rng), Ok(()));
    }

    #[test]
    fn untrimmed_or_wrong_sources_are_rejected_per_use() {
        assert_eq!(
            check(
                &Clk48State {
                    hsi48_ready: false,
                    ..GOOD
                },
                Clk48Use::Rng,
            ),
            Err(Clk48Error::Hsi48NotReady)
        );
        assert_eq!(
            check(
                &Clk48State {
                    clk48_is_hsi48: false,
                    ..GOOD
                },
                Clk48Use::Rng,
            ),
            Err(Clk48Error::NotHsi48)
        );
        for state in [
            Clk48State {
                crs_enabled: false,
                ..GOOD
            },
            Clk48State {
                crs_autotrim: false,
                ..GOOD
            },
            Clk48State {
                crs_sync_usb: false,
                ..GOOD
            },
        ] {
            assert_eq!(
                check(&state, Clk48Use::Usb),
                Err(Clk48Error::CrsNotSyncedToUsb)
            );
            // The RNG does not need the CRS (no USB SOF to trim from).
            assert_eq!(check(&state, Clk48Use::Rng), Ok(()));
        }
    }
}
