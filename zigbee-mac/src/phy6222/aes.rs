//! PHY62x2 AES-128 block owner: the hardware binding of
//! [`crate::phy6222_aes`].
//!
//! # Ownership
//! The AES block at `0x4004_0000` is the BLE link-layer crypto engine. The
//! zigbee-rs PHY62x2 stack does not run the BLE link layer and calls no ROM
//! `LL_ENC_*` routine; the 802.15.4 radio driver ([`super::driver`]) uses
//! only the RF PHY/baseband (`0x4003_0000`) and LL HW engine
//! (`0x4003_1000`), never the AES block, and the zigbee-rs loader
//! (`bootloaders/phy62x2-boot`) does not touch it. [`Phy6222Aes::take`] is
//! the single owner; after
//! [`Phy6222RadioPhy::install_aes_engine`](super::Phy6222RadioPhy::install_aes_engine)
//! the radio PHY owns it and lends it to NWK/APS CCM*, AES-MMO and the
//! Trust Center through `ForwardAesProvider`.
//!
//! The module clock bit (`PCR SW_CLK`, `0x4000_0008` bit 4) shares its
//! register with the other peripheral clocks; it is changed with a
//! read-modify-write inside a critical section. No interrupt handler in
//! this stack writes `SW_CLK`.

use core::cell::Cell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use crate::phy6222_aes::{AesCore, AesRegisters};
pub use crate::phy6222_aes::{
    COMPLETION_WAIT_CYCLES, CompletionProbe, PROBE_STATUS_POLLS, Phy6222AesError,
    STATUS_SINGLE_MODE_DONE,
};

/// AES block base address (`AES_BASE_ADDR`, vendor `LL_ENC_BASE`).
const AES_BASE: usize = 0x4004_0000;
/// `AP_PCR->SW_CLK` (`AP_PCR_BASE` = `0x4000_0000`).
const PCR_SW_CLK: usize = 0x4000_0008;
/// `MOD_AES` clock gate.
const SW_CLK_AES: u32 = 1 << 4;

static AES_TAKEN: Mutex<CriticalSectionRawMutex, Cell<bool>> = Mutex::new(Cell::new(false));
/// Serializes the `SW_CLK` read-modify-write against interrupts.
static SW_CLK_LOCK: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());

struct HwRegisters;

impl HwRegisters {
    #[inline(always)]
    fn read_address(address: usize) -> u32 {
        // SAFETY: fixed, always-mapped PHY62x2 peripheral registers.
        unsafe { core::ptr::read_volatile(address as *const u32) }
    }

    #[inline(always)]
    fn write_address(address: usize, value: u32) {
        // SAFETY: as above; the AES block is owned by this engine.
        unsafe { core::ptr::write_volatile(address as *mut u32, value) }
    }
}

impl AesRegisters for HwRegisters {
    fn clock_enabled(&mut self) -> bool {
        Self::read_address(PCR_SW_CLK) & SW_CLK_AES != 0
    }

    fn set_clock(&mut self, enabled: bool) {
        SW_CLK_LOCK.lock(|_| {
            let value = Self::read_address(PCR_SW_CLK);
            let value = if enabled {
                value | SW_CLK_AES
            } else {
                value & !SW_CLK_AES
            };
            Self::write_address(PCR_SW_CLK, value);
        });
    }

    fn write(&mut self, offset: usize, value: u32) {
        Self::write_address(AES_BASE + offset, value);
    }

    fn read(&mut self, offset: usize) -> u32 {
        Self::read_address(AES_BASE + offset)
    }

    fn wait_cycles(&mut self, cycles: u32) {
        delay_cycles(cycles);
    }
}

/// Busy-wait for at least `cycles` CPU cycles. Each iteration is a `subs`
/// and a taken `bne` (4 cycles on a Cortex-M0, more with XIP wait states);
/// counting 2 per iteration keeps the bound on any core. Interrupts only
/// lengthen it.
#[inline(always)]
fn delay_cycles(cycles: u32) {
    #[cfg(target_arch = "arm")]
    // SAFETY: register-only countdown loop; no memory or stack access. The
    // asm block is not `nomem`, so the surrounding volatile register
    // accesses are not moved across it.
    unsafe {
        core::arch::asm!(
            "2:",
            "subs {0}, #1",
            "bne 2b",
            inout(reg) cycles / 2 + 1 => _,
            options(nostack),
        );
    }
    #[cfg(not(target_arch = "arm"))]
    for _ in 0..cycles {
        core::hint::spin_loop();
    }
}

/// Exclusive owner of the PHY62x2 AES-128 block.
///
/// Starts unvalidated: [`Self::encrypt_block`] returns
/// [`Phy6222AesError::NotValidated`] until [`Self::self_test`] passes.
pub struct Phy6222Aes {
    core: AesCore<HwRegisters>,
}

impl Phy6222Aes {
    /// Acquire the AES block once. Does not touch the hardware.
    pub fn take() -> Option<Self> {
        let acquired = AES_TAKEN.lock(|taken| !taken.replace(true));
        acquired.then(|| Self {
            core: AesCore::new(HwRegisters),
        })
    }

    /// Run the two-vector AES-128 known-answer test on the block; required
    /// before any encryption.
    pub fn self_test(&mut self) -> Result<(), Phy6222AesError> {
        self.core.self_test()
    }

    /// Encrypt one AES-128 block in place. `block` is unchanged on error.
    #[inline(never)]
    pub fn encrypt_block(
        &mut self,
        key: &[u8; 16],
        block: &mut [u8; 16],
    ) -> Result<(), Phy6222AesError> {
        let input = *block;
        self.core.encrypt(key, &input, block)
    }

    /// Change the fixed completion wait (lab tuning; the default is
    /// [`COMPLETION_WAIT_CYCLES`]). Rerun [`Self::self_test`] afterwards.
    pub fn set_completion_cycles(&mut self, cycles: u32) -> Result<(), Phy6222AesError> {
        self.core.set_completion_cycles(cycles)
    }

    /// Sample the status register around one block (hardware lab only).
    /// Rerun [`Self::self_test`] afterwards.
    pub fn completion_probe(&mut self) -> Result<CompletionProbe, Phy6222AesError> {
        self.core.completion_probe()
    }
}

/// Keyed AES-128 forward permutation lent to the protocol layers.
pub(super) struct Phy6222HardwareAes128<'engine> {
    pub(super) engine: Option<&'engine mut Phy6222Aes>,
    pub(super) key: zigbee_crypto::AesKey,
}

impl zigbee_crypto::Aes128Forward for Phy6222HardwareAes128<'_> {
    type Error = Phy6222AesError;

    fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), Self::Error> {
        self.engine
            .as_deref_mut()
            .ok_or(Phy6222AesError::NotValidated)?
            .encrypt_block(&self.key, block)
    }
}
