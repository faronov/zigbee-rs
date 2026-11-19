//! True random number generator (`inc/hw_trng.h`, driverlib `trng.c`).
//!
//! Configured like TI `TRNGCC26XX` (256 minimum / 240,000 maximum samples per
//! 64-bit cycle, sample divider 0). The module is in the PERIPH domain; each
//! 64-bit result is read from `OUT0/OUT1` once `IRQFLAGSTAT.RDY` is set.

use crate::mmio::{self, TRNG_BASE};
use crate::prcm::{self, Domain, Gate};

const OUT0: u32 = TRNG_BASE;
const OUT1: u32 = TRNG_BASE + 0x04;
const IRQFLAGSTAT: u32 = TRNG_BASE + 0x08;
const IRQFLAGCLR: u32 = TRNG_BASE + 0x10;
const CTL: u32 = TRNG_BASE + 0x14;
const CFG0: u32 = TRNG_BASE + 0x18;
const SWRESET: u32 = TRNG_BASE + 0x1FF0;

const CTL_EN: u32 = 1 << 10;
const MIN_SAMPLES: u32 = 256;
const MAX_SAMPLES: u32 = 240_000;

/// Ownership token.
pub struct TrngToken(pub(crate) ());

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrngError {
    Power,
    Timeout,
}

pub struct Trng(());

impl Trng {
    pub fn new(_token: TrngToken) -> Result<Self, TrngError> {
        if !prcm::is_on(Domain::Periph) && !prcm::power_on(Domain::Periph) {
            return Err(TrngError::Power);
        }
        prcm::enable(Gate::Trng);
        prcm::load();
        mmio::write(SWRESET, 1);
        prcm::wait(|| mmio::read(SWRESET) == 0);
        let ctl = mmio::read(CTL) & !CTL_EN & !0xFFFF_0000;
        mmio::write(CTL, ctl | ((MAX_SAMPLES >> 8) << 16));
        mmio::write(
            CFG0,
            ((MAX_SAMPLES >> 8) << 16) | ((MIN_SAMPLES >> 6) & 0xFF),
        );
        mmio::modify(CTL, 0, CTL_EN);
        Ok(Self(()))
    }

    /// Next 64 random bits.
    pub fn next_u64(&mut self) -> Result<u64, TrngError> {
        if !prcm::wait(|| mmio::read(IRQFLAGSTAT) & 1 != 0) {
            return Err(TrngError::Timeout);
        }
        let lo = mmio::read(OUT0);
        let hi = mmio::read(OUT1);
        mmio::write(IRQFLAGCLR, 1);
        Ok(u64::from(hi) << 32 | u64::from(lo))
    }

    pub fn fill(&mut self, out: &mut [u8]) -> Result<(), TrngError> {
        for chunk in out.chunks_mut(8) {
            let v = self.next_u64()?.to_le_bytes();
            chunk.copy_from_slice(&v[..chunk.len()]);
        }
        Ok(())
    }
}
