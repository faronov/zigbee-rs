//! BL702 SEC_ENG hardware AES-128 ECB single-block accelerator.
//!
//! # Register evidence
//!
//! Unlike the TLSR8258 accelerator (whose protocol had to be recovered by
//! disassembling a vendor object), the BL702 Security Engine ("SEC_ENG")
//! AES block is fully described by *open-source* Bouffalo Lab SDK sources,
//! so this is a **high-confidence** register contract:
//!
//! * `bl_iot_sdk` `.../bl702_std/BSP_Driver/regs/sec_eng_reg.h` gives the
//!   register offsets and bitfields transcribed below.
//! * `.../bl702_std/BSP_Driver/regs/bl702.h` defines
//!   `SEC_ENG_BASE = 0x4000_4000`.
//! * `.../std_drv/src/bl702_sec_eng.c` (`Sec_Eng_AES_Init`,
//!   `Sec_Eng_AES_Set_Key_IV_BE`, `Sec_Eng_AES_Crypt`) gives the exact
//!   direct-mode (non-DMA-link) register *sequence* this module reproduces.
//! * `.../hosal/bl702_hal/bl_sec_aes.c` demonstrates a big-endian AES-128
//!   ECB known-answer test using the classic FIPS-197 key
//!   `2b 7e 15 16 28 ae d2 a6 ab f7 15 88 09 cf 4f 3c` and confirms that
//!   `Enable_BE` (`ENDIAN = 0x0f`) plus `KEY_0 = first four key bytes` plus
//!   natural byte-order source/destination buffers yields *standard* AES.
//!
//! # Register map (all relative to `SEC_ENG_BASE + SEC_ENG_AES_OFFSET`,
//! # where `SEC_ENG_AES_OFFSET = 0x100`)
//!
//! ```text
//! +0x00 CTRL    (see bitfields below)
//! +0x04 MSA     message source address       (engine reads 16*N bytes)
//! +0x08 MDA     message destination address  (engine writes 16*N bytes)
//! +0x0C STATUS
//! +0x10 IV_0 .. +0x1C IV_3
//! +0x20 KEY_0 .. +0x3C KEY_7   (only KEY_0..3 used for AES-128)
//! +0x40 KEY_SEL_0  +0x44 KEY_SEL_1
//! +0x48 ENDIAN  (DOUT b0, DIN b1, KEY b2, IV b3; 0x0f == all big-endian)
//! ```
//! and, at `SEC_ENG_BASE + 0x1FC`, `CTRL_PROT` (write `0x03` to grant ID0
//! access, as `Sec_Eng_AES_Init` does).
//!
//! CTRL bitfields (offset `0x100`):
//! ```text
//!  b0     BUSY        (read-only status: 1 while a block is in flight)
//!  b1     TRIG_1T     (write 1 to start; one-shot)
//!  b2     EN          (engine enable)
//!  b3..4  MODE        (key length: 0 = 128-bit)
//!  b5     DEC_EN      (0 = encrypt, 1 = decrypt)
//!  b6     DEC_KEY_SEL (0 for new key; CTR uses 1)
//!  b7     HW_KEY_EN   (0 = software key in KEY_x registers)
//!  b9     INT_CLR_1T  (clear the finished interrupt; one-shot)
//!  b12..13 BLOCK_MODE (0 = ECB)
//!  b14    IV_SEL      (0 = use freshly written IV)
//!  b15    LINK_MODE   (0 = direct register mode; not the DMA-link path)
//!  b16..31 MSG_LEN    (number of 16-byte blocks)
//! ```
//!
//! # Protocol this module implements (one 16-byte block, ECB, software key)
//!
//! Following `Sec_Eng_AES_Init` + `Sec_Eng_AES_Set_Key_IV_BE` +
//! `Sec_Eng_AES_Crypt`, collapsed to a single self-contained block so every
//! call re-keys the engine (AES-MMO re-keys every block):
//!
//! 1. Bounded-wait until CTRL `BUSY` clears.
//! 2. Write CTRL configuring `EN=1`, `BLOCK_MODE=ECB`, `MODE=128`,
//!    `DEC_EN`, `INT_CLR_1T=1`, and every other managed field cleared
//!    (`TRIG=0`, `MSG_LEN=0`, `HW_KEY_EN=0`, `DEC_KEY_SEL=0`, `IV_SEL=0`,
//!    `LINK_MODE=0`).
//! 3. Write `CTRL_PROT = 0x03`, `ENDIAN = 0x0f`, `KEY_0..3` (little-endian
//!    word view of the caller's key bytes, `KEY_0` = bytes `0..4`), and
//!    `IV_0..3 = 0`.
//! 4. Bounded-wait `BUSY`.
//! 5. Point `MSA`/`MDA` at word-aligned in/out block buffers, set
//!    `MSG_LEN = 1`, write CTRL, then set `TRIG_1T` and write CTRL again.
//! 6. Bounded-wait `BUSY`; the destination buffer now holds the result.
//!
//! # Deliberate deviations from the vendor object
//!
//! * **Bounded busy waits.** The vendor's `Sec_Eng_AES_*` do bound their
//!   `BUSY` polls (`SEC_ENG_AES_BUSY_TIMEOUT_COUNT`), but this module uses
//!   a caller-supplied `timeout_iterations` and returns
//!   [`AesError::Timeout`] rather than a vendor-fixed count — matching this
//!   crate's existing "no unbounded wait" rule (`i2c.rs`, `spi.rs`).
//! * **Word-aligned DMA buffers.** The engine reads/writes `MSA`/`MDA` over
//!   the bus. This module copies the caller's block into a 4-byte-aligned
//!   local buffer and drains from another, so an unaligned caller slice can
//!   never be handed to the engine (the DMA-link path explicitly rejects
//!   unaligned addresses; the direct path is given aligned buffers here).
//! * **Explicit CPU/device ordering.** A full RISC-V memory fence immediately
//!   before `TRIG_1T` makes the source-buffer writes and DMA-address writes
//!   visible before the engine starts. A second fence after `BUSY` clears,
//!   followed by volatile destination reads, prevents either the CPU or the
//!   compiler from treating the zero-initialized destination as unchanged by
//!   the autonomous engine.
//! * **Startup known-answer gate.** [`AesEngine::self_test`] runs two
//!   back-to-back AES-128 vectors (different keys) against the real
//!   peripheral before Zigbee security may use it; a mismatch returns
//!   [`AesError::KnownAnswerMismatch`] and the engine must not be used
//!   (never a silent software fallback).
//!
//! # Ownership, clocks, and non-aliasing
//!
//! Exclusive access is established by the zero-sized
//! [`crate::peripherals::Aes`] token, handed out at most once by
//! [`crate::peripherals::Peripherals::take`]. The SEC_ENG AES sub-block is
//! clocked from the bus clock and, unlike the AHB peripherals
//! (`i2c`/`spi`/`pwm`), has **no** `GLB` `CGEN`/`SWRST` gate — the vendor
//! `Sec_Eng_AES_Init` touches no clock register, and neither does
//! [`AesEngine::new`]. This is a *different* peripheral from the Serial
//! Flash Controller inline AES driven through the boot ROM in `flash.rs`
//! (`SF_CTRL_AES_*`): they do not share registers and are not alternative
//! owners of the same path.
//!
//! # Hardware validation caveat
//!
//! This module has **not** been exercised against real BL702 silicon. The
//! register contract is transcribed from open-source vendor drivers and the
//! byte/endianness mapping is cross-checked against a published FIPS-197
//! known-answer vector, but the on-chip DMA timing, any cache-coherency
//! requirement on the `MSA`/`MDA` SRAM buffers, and
//! [`AesEngine::DEFAULT_TIMEOUT_ITERATIONS`] (a conservative placeholder,
//! not a measured bound) all still require bench validation. Because
//! [`AesEngine::self_test`] fails closed, a wrong assumption here rejects
//! the engine rather than corrupting Zigbee security.
//!
//! # No DMA-link, no CBC/CTR
//!
//! Only the polled, single-block, direct-register ECB path is implemented
//! (CCM* needs only the forward permutation, one 16-byte block at a time).
//! The engine's `LINK_MODE` descriptor-DMA path and its CBC/CTR block modes
//! are intentionally out of scope.

#[cfg(target_arch = "riscv32")]
use crate::mmio::{read32, write32};

// ── Absolute register addresses (only the real-MMIO `HwRegisters` needs
//    these; the host mock addresses memory directly). ──────────────────
/// `SEC_ENG_BASE` (`bl702.h`).
#[cfg(target_arch = "riscv32")]
const SEC_ENG_BASE: u32 = 0x4000_4000;
/// `SEC_ENG_AES_OFFSET` — direct-mode AES register window base
/// (`sec_eng_reg.h`).
#[cfg(target_arch = "riscv32")]
const SEC_ENG_AES_OFFSET: u32 = 0x100;
/// Base of the direct-mode AES registers.
#[cfg(target_arch = "riscv32")]
const AES_BASE: u32 = SEC_ENG_BASE + SEC_ENG_AES_OFFSET;

#[cfg(target_arch = "riscv32")]
const REG_AES_CTRL: u32 = AES_BASE;
#[cfg(target_arch = "riscv32")]
const REG_AES_MSA: u32 = AES_BASE + 0x04;
#[cfg(target_arch = "riscv32")]
const REG_AES_MDA: u32 = AES_BASE + 0x08;
#[cfg(target_arch = "riscv32")]
const REG_AES_IV_0: u32 = AES_BASE + 0x10;
#[cfg(target_arch = "riscv32")]
const REG_AES_KEY_0: u32 = AES_BASE + 0x20;
#[cfg(target_arch = "riscv32")]
const REG_AES_ENDIAN: u32 = AES_BASE + 0x48;
/// `SEC_ENG_SE_AES_0_CTRL_PROT` — grants ID0 access; write `0x03`.
#[cfg(target_arch = "riscv32")]
const REG_AES_CTRL_PROT: u32 = SEC_ENG_BASE + 0x1FC;

// ── CTRL bitfields and endian/prot values, shared by `run_block` and the
//    host mock tests. ─────────────────────────────────────────────────
#[cfg(any(target_arch = "riscv32", test))]
const CTRL_BUSY: u32 = 1; // b0 (read-only status)
#[cfg(any(target_arch = "riscv32", test))]
const CTRL_TRIG_1T: u32 = 1 << 1;
#[cfg(any(target_arch = "riscv32", test))]
const CTRL_EN: u32 = 1 << 2;
#[cfg(any(target_arch = "riscv32", test))]
const CTRL_DEC_EN: u32 = 1 << 5;
#[cfg(any(target_arch = "riscv32", test))]
const CTRL_INT_CLR_1T: u32 = 1 << 9;
#[cfg(any(target_arch = "riscv32", test))]
const CTRL_MSG_LEN_SHIFT: u32 = 16;
// `MODE` (b3..4) value `0` selects a 128-bit key and `BLOCK_MODE` (b12..13)
// value `0` selects ECB — both contribute no set bits, so `ctrl_base` names
// them only in this comment rather than OR-ing zero constants.

/// `ENDIAN = 0x0f`: DOUT/DIN/KEY/IV all big-endian, matching
/// `Sec_Eng_AES_Enable_BE`, so natural byte-order buffers give standard AES.
#[cfg(any(target_arch = "riscv32", test))]
const ENDIAN_ALL_BE: u32 = 0x0f;
/// `CTRL_PROT` value that grants ID0 access (`Sec_Eng_AES_Init`).
#[cfg(any(target_arch = "riscv32", test))]
const CTRL_PROT_ID0_ACCESS: u32 = 0x03;

/// 4-byte-aligned scratch block so a `MSA`/`MDA` DMA address is always word
/// aligned regardless of the caller's slice alignment.
#[cfg(any(target_arch = "riscv32", test))]
#[repr(C, align(4))]
struct AlignedBlock([u8; 16]);

/// Errors from [`AesEngine::new`]/[`AesEngine::encrypt_block`]/
/// [`AesEngine::self_test`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AesError {
    /// `timeout_iterations == 0` was passed to [`AesEngine::new`] — that
    /// would make every subsequent `BUSY` poll fail immediately (mirrors
    /// `i2c::I2cError`'s "reject rather than silently misbehave").
    ZeroIterations,
    /// CTRL `BUSY` did not clear within the configured
    /// `timeout_iterations`. A hard failure, not an "assume done and read
    /// garbage" fallback.
    Timeout,
    /// The on-silicon AES result did not match an AES-128 known-answer
    /// vector. Callers must not use the accelerator after this error
    /// because byte/word ordering, DMA, clocking, or the peripheral itself
    /// is not behaving as required.
    KnownAnswerMismatch,
}

/// Abstraction over the SEC_ENG AES register file plus its `MSA`/`MDA` DMA
/// pointers, so the exact polled protocol in [`run_block`] runs unchanged
/// against either real BL702 MMIO ([`HwRegisters`], riscv32-only) or the
/// in-memory mock this module's host tests drive against NIST AES-128
/// known-answer vectors. Not part of the public API.
#[cfg(any(target_arch = "riscv32", test))]
trait AesRegisters {
    fn read_ctrl(&mut self) -> u32;
    fn write_ctrl(&mut self, value: u32);
    fn write_ctrl_prot(&mut self, value: u32);
    fn write_endian(&mut self, value: u32);
    /// Write `KEY_index` (`index` in `0..4` for AES-128).
    fn write_key(&mut self, index: usize, word: u32);
    /// Write `IV_index` (`index` in `0..4`).
    fn write_iv(&mut self, index: usize, word: u32);
    /// Set the message source DMA address (`MSA`).
    fn set_source(&mut self, ptr: *const u8);
    /// Set the message destination DMA address (`MDA`).
    fn set_dest(&mut self, ptr: *mut u8);
}

#[cfg(target_arch = "riscv32")]
struct HwRegisters;

#[cfg(target_arch = "riscv32")]
impl AesRegisters for HwRegisters {
    fn read_ctrl(&mut self) -> u32 {
        read32(REG_AES_CTRL)
    }
    fn write_ctrl(&mut self, value: u32) {
        write32(REG_AES_CTRL, value)
    }
    fn write_ctrl_prot(&mut self, value: u32) {
        write32(REG_AES_CTRL_PROT, value)
    }
    fn write_endian(&mut self, value: u32) {
        write32(REG_AES_ENDIAN, value)
    }
    fn write_key(&mut self, index: usize, word: u32) {
        write32(REG_AES_KEY_0 + (index as u32) * 4, word)
    }
    fn write_iv(&mut self, index: usize, word: u32) {
        write32(REG_AES_IV_0 + (index as u32) * 4, word)
    }
    fn set_source(&mut self, ptr: *const u8) {
        write32(REG_AES_MSA, ptr as u32)
    }
    fn set_dest(&mut self, ptr: *mut u8) {
        write32(REG_AES_MDA, ptr as u32)
    }
}

/// Reject a zero iteration bound up front — see [`AesError::ZeroIterations`].
#[cfg(any(target_arch = "riscv32", test))]
const fn validate_timeout_iterations(timeout_iterations: u32) -> Result<u32, AesError> {
    if timeout_iterations == 0 {
        Err(AesError::ZeroIterations)
    } else {
        Ok(timeout_iterations)
    }
}

/// CTRL configuration common to the pre-key setup write and the run write:
/// enable + ECB + 128-bit + encrypt/decrypt, every managed field explicit.
#[cfg(any(target_arch = "riscv32", test))]
#[inline]
fn ctrl_base(decrypt: bool) -> u32 {
    // ECB + 128-bit key contribute no set bits (both selector fields are 0).
    let mut ctrl = CTRL_EN | CTRL_INT_CLR_1T;
    if decrypt {
        ctrl |= CTRL_DEC_EN;
    }
    ctrl
}

/// Poll CTRL `BUSY` until clear, bounded by `timeout_iterations`.
#[cfg(any(target_arch = "riscv32", test))]
fn wait_not_busy<R: AesRegisters>(regs: &mut R, timeout_iterations: u32) -> Result<(), AesError> {
    for _ in 0..timeout_iterations {
        if regs.read_ctrl() & CTRL_BUSY == 0 {
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(AesError::Timeout)
}

/// Order CPU SRAM accesses against the SEC_ENG bus master.
#[cfg(any(target_arch = "riscv32", test))]
#[inline(always)]
fn dma_memory_fence() {
    #[cfg(target_arch = "riscv32")]
    // SAFETY: `fence rw, rw` has no memory operands or stack effect. Omitting
    // `nomem` intentionally gives the compiler a memory clobber as well as
    // emitting the architectural RISC-V fence.
    unsafe {
        core::arch::asm!("fence rw, rw", options(nostack));
    }

    #[cfg(not(target_arch = "riscv32"))]
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
}

/// Run one ECB-128 block against `regs`. Pure register-order/state-machine
/// logic, independent of whether `regs` is real MMIO or a host test mock.
#[cfg(any(target_arch = "riscv32", test))]
fn run_block<R: AesRegisters>(
    regs: &mut R,
    key: &[u8; 16],
    input: &[u8; 16],
    output: &mut [u8; 16],
    decrypt: bool,
    timeout_iterations: u32,
) -> Result<(), AesError> {
    let source = AlignedBlock(*input);
    let mut dest = AlignedBlock([0u8; 16]);

    // 1. Wait for any previous operation to drain.
    wait_not_busy(regs, timeout_iterations)?;

    // 2. Configure mode/key-length/direction, TRIG and MSG_LEN cleared.
    regs.write_ctrl(ctrl_base(decrypt));

    // 3. Grant ID0 access, select big-endian, load key and a zero IV.
    regs.write_ctrl_prot(CTRL_PROT_ID0_ACCESS);
    regs.write_endian(ENDIAN_ALL_BE);
    for index in 0..4 {
        let start = index * 4;
        let word = u32::from_le_bytes([key[start], key[start + 1], key[start + 2], key[start + 3]]);
        regs.write_key(index, word);
    }
    for index in 0..4 {
        regs.write_iv(index, 0);
    }

    // 4. Wait before programming the DMA descriptor.
    wait_not_busy(regs, timeout_iterations)?;

    // 5. Point the engine at the aligned block buffers and run one block.
    regs.set_source(source.0.as_ptr());
    regs.set_dest(dest.0.as_mut_ptr());
    let ctrl_run = ctrl_base(decrypt) | (1u32 << CTRL_MSG_LEN_SHIFT);
    regs.write_ctrl(ctrl_run);
    dma_memory_fence();
    regs.write_ctrl(ctrl_run | CTRL_TRIG_1T);

    // 6. Wait for completion; the destination buffer holds the result.
    wait_not_busy(regs, timeout_iterations)?;
    dma_memory_fence();
    for (output_byte, dest_byte) in output.iter_mut().zip(dest.0.iter()) {
        // SAFETY: `dest_byte` points inside the live 16-byte destination
        // scratch block. Volatile access is required because SEC_ENG, not
        // Rust code, writes this memory.
        *output_byte = unsafe { core::ptr::read_volatile(dest_byte) };
    }
    Ok(())
}

/// Owns the BL702 SEC_ENG AES-128 ECB hardware accelerator.
///
/// Constructed from the exclusive [`crate::peripherals::Aes`] token, so at
/// most one [`AesEngine`] can exist at a time. Only available on the
/// `riscv32` target — the register-driving methods are meaningless off
/// silicon, and host tests exercise the register state machine through
/// [`run_block`] and a mock instead.
#[cfg(target_arch = "riscv32")]
pub struct AesEngine {
    timeout_iterations: u32,
}

#[cfg(target_arch = "riscv32")]
impl AesEngine {
    /// A conservative placeholder bound for the `BUSY` poll — a single
    /// 16-byte block completes in far fewer bus cycles, but this is not a
    /// measured figure (see the module docs' "Hardware validation caveat").
    /// Mirrors `i2c.rs`'s `TIMEOUT_ITERATIONS`.
    pub const DEFAULT_TIMEOUT_ITERATIONS: u32 = 1_000_000;

    /// Take ownership of the SEC_ENG AES accelerator.
    ///
    /// `timeout_iterations` bounds every subsequent block's wait for the
    /// CTRL `BUSY` bit to clear. No clock/reset register is touched (the
    /// SEC_ENG AES sub-block has no `GLB` gate — see module docs).
    pub fn new(
        _peripheral: crate::peripherals::Aes,
        timeout_iterations: u32,
    ) -> Result<Self, AesError> {
        let timeout_iterations = validate_timeout_iterations(timeout_iterations)?;
        Ok(Self { timeout_iterations })
    }

    /// Encrypt one 16-byte block in place with `key`.
    pub fn encrypt_block(&mut self, key: &[u8; 16], block: &mut [u8; 16]) -> Result<(), AesError> {
        let input = *block;
        run_block(
            &mut HwRegisters,
            key,
            &input,
            block,
            false,
            self.timeout_iterations,
        )
    }

    /// Decrypt one 16-byte block in place with `key`.
    ///
    /// CCM* (this workspace's only current AES consumer on this platform)
    /// never calls this — CCM* uses only the forward permutation. Provided
    /// because the register evidence is exactly as clear as for
    /// [`Self::encrypt_block`], not because anything here needs it.
    pub fn decrypt_block(&mut self, key: &[u8; 16], block: &mut [u8; 16]) -> Result<(), AesError> {
        let input = *block;
        run_block(
            &mut HwRegisters,
            key,
            &input,
            block,
            true,
            self.timeout_iterations,
        )
    }

    /// Run AES-128 known-answer tests on the real accelerator.
    ///
    /// Checks the complete key/data/result byte ordering and DMA handshake
    /// before Zigbee security uses the engine. Two back-to-back vectors with
    /// different keys also verify the engine can be re-keyed and reused, as
    /// AES-MMO requires.
    pub fn self_test(&mut self) -> Result<(), AesError> {
        for (key, plaintext, ciphertext) in SELF_TEST_VECTORS.iter() {
            let mut block = *plaintext;
            self.encrypt_block(key, &mut block)?;
            if &block != ciphertext {
                return Err(AesError::KnownAnswerMismatch);
            }
        }
        Ok(())
    }
}

/// Two independent FIPS-197/NIST AES-128 ECB known-answer vectors used by
/// [`AesEngine::self_test`] (and the host mock tests below). The two
/// different keys exercise a re-key/reuse cycle back to back.
#[cfg(any(target_arch = "riscv32", test))]
const SELF_TEST_VECTORS: [([u8; 16], [u8; 16], [u8; 16]); 2] = [
    (
        [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
            0x0e, 0x0f,
        ],
        [
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd,
            0xee, 0xff,
        ],
        [
            0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4,
            0xc5, 0x5a,
        ],
    ),
    (
        [0u8; 16],
        [0u8; 16],
        [
            0x66, 0xe9, 0x4b, 0xd4, 0xef, 0x8a, 0x2c, 0x3b, 0x88, 0x4c, 0xfa, 0x59, 0xca, 0x34,
            0x2b, 0x2e,
        ],
    ),
];

#[cfg(test)]
mod tests {
    use super::*;
    use aes::Aes128;
    use aes::cipher::generic_array::GenericArray;
    use aes::cipher::{BlockDecrypt, BlockEncrypt, KeyInit};

    /// Host-only mock of the SEC_ENG AES register file plus its `MSA`/`MDA`
    /// DMA. Reproduces the documented handshake (mode select, big-endian
    /// key words, `MSG_LEN`, `TRIG`, `BUSY`) while performing the *actual*
    /// AES-128 permutation with the independently-vetted RustCrypto `aes`
    /// crate as an oracle, reading from the stored `MSA` pointer and writing
    /// to the stored `MDA` pointer exactly like the silicon.
    ///
    /// This validates [`run_block`]'s register **ordering**/bit
    /// **sequencing** and the big-endian key mapping end-to-end against real
    /// NIST vectors. It does **not** and cannot validate the real silicon's
    /// own AES/DMA implementation; see the module-level caveat.
    struct MockRegisters {
        ctrl: u32,
        endian: u32,
        ctrl_prot: u32,
        key: [u32; 4],
        source: *const u8,
        dest: *mut u8,
        /// Busy polls returned as "still busy" after each trigger before
        /// completion, to exercise the bounded wait.
        busy_polls_after_trig: u32,
        busy_remaining: u32,
    }

    impl MockRegisters {
        fn new(busy_polls_after_trig: u32) -> Self {
            Self {
                ctrl: 0,
                endian: 0,
                ctrl_prot: 0,
                key: [0; 4],
                source: core::ptr::null(),
                dest: core::ptr::null_mut(),
                busy_polls_after_trig,
                busy_remaining: 0,
            }
        }

        fn execute(&mut self) {
            assert_eq!(self.endian, ENDIAN_ALL_BE, "engine run without BE endian");
            assert_eq!(
                self.ctrl_prot, CTRL_PROT_ID0_ACCESS,
                "engine run without ID0 access grant"
            );
            // Reconstruct the key bytes from the big-endian key words.
            let mut key = [0u8; 16];
            for (index, word) in self.key.iter().enumerate() {
                key[index * 4..index * 4 + 4].copy_from_slice(&word.to_le_bytes());
            }
            let decrypt = self.ctrl & CTRL_DEC_EN != 0;
            // SAFETY: `source`/`dest` point at the 16-byte aligned block
            // buffers owned by `run_block` for the duration of this call.
            let input = unsafe { core::slice::from_raw_parts(self.source, 16) };
            let mut block = GenericArray::clone_from_slice(input);
            let cipher = Aes128::new(GenericArray::from_slice(&key));
            if decrypt {
                cipher.decrypt_block(&mut block);
            } else {
                cipher.encrypt_block(&mut block);
            }
            // SAFETY: as above; `dest` is writable for 16 bytes.
            unsafe {
                core::ptr::copy_nonoverlapping(block.as_ptr(), self.dest, 16);
            }
        }
    }

    impl AesRegisters for MockRegisters {
        fn read_ctrl(&mut self) -> u32 {
            if self.busy_remaining > 0 {
                self.busy_remaining -= 1;
                self.ctrl | CTRL_BUSY
            } else {
                self.ctrl & !CTRL_BUSY
            }
        }
        fn write_ctrl(&mut self, value: u32) {
            let rising_trig = value & CTRL_TRIG_1T != 0 && self.ctrl & CTRL_TRIG_1T == 0;
            self.ctrl = value & !CTRL_BUSY;
            if rising_trig && value & CTRL_EN != 0 && (value >> CTRL_MSG_LEN_SHIFT) != 0 {
                self.execute();
                self.busy_remaining = self.busy_polls_after_trig;
            }
        }
        fn write_ctrl_prot(&mut self, value: u32) {
            self.ctrl_prot = value;
        }
        fn write_endian(&mut self, value: u32) {
            self.endian = value;
        }
        fn write_key(&mut self, index: usize, word: u32) {
            self.key[index] = word;
        }
        fn write_iv(&mut self, _index: usize, _word: u32) {}
        fn set_source(&mut self, ptr: *const u8) {
            self.source = ptr;
        }
        fn set_dest(&mut self, ptr: *mut u8) {
            self.dest = ptr;
        }
    }

    fn run(mock: &mut MockRegisters, key: &[u8; 16], input: &[u8; 16], decrypt: bool) -> [u8; 16] {
        let mut output = [0u8; 16];
        run_block(mock, key, input, &mut output, decrypt, 1_000)
            .expect("mock block should complete");
        output
    }

    #[test]
    fn encrypts_fips197_known_answer() {
        let (key, plaintext, ciphertext) = &SELF_TEST_VECTORS[0];
        let mut mock = MockRegisters::new(0);
        assert_eq!(&run(&mut mock, key, plaintext, false), ciphertext);
    }

    #[test]
    fn encrypts_zero_key_known_answer() {
        let (key, plaintext, ciphertext) = &SELF_TEST_VECTORS[1];
        let mut mock = MockRegisters::new(0);
        assert_eq!(&run(&mut mock, key, plaintext, false), ciphertext);
    }

    #[test]
    fn decrypt_is_inverse_of_encrypt() {
        let (key, plaintext, ciphertext) = &SELF_TEST_VECTORS[0];
        let mut mock = MockRegisters::new(0);
        assert_eq!(&run(&mut mock, key, ciphertext, true), plaintext);
    }

    #[test]
    fn rekey_reuse_across_two_blocks() {
        // Same engine value, two different keys back to back, mirroring the
        // AES-MMO re-key-every-block requirement and the on-silicon
        // self-test's two-vector shape.
        let mut mock = MockRegisters::new(2);
        let first = &SELF_TEST_VECTORS[0];
        let second = &SELF_TEST_VECTORS[1];
        assert_eq!(&run(&mut mock, &first.0, &first.1, false), &first.2);
        assert_eq!(&run(&mut mock, &second.0, &second.1, false), &second.2);
    }

    #[test]
    fn bounded_wait_times_out_when_busy_never_clears() {
        struct AlwaysBusy;
        impl AesRegisters for AlwaysBusy {
            fn read_ctrl(&mut self) -> u32 {
                CTRL_BUSY
            }
            fn write_ctrl(&mut self, _value: u32) {}
            fn write_ctrl_prot(&mut self, _value: u32) {}
            fn write_endian(&mut self, _value: u32) {}
            fn write_key(&mut self, _index: usize, _word: u32) {}
            fn write_iv(&mut self, _index: usize, _word: u32) {}
            fn set_source(&mut self, _ptr: *const u8) {}
            fn set_dest(&mut self, _ptr: *mut u8) {}
        }
        let mut output = [0u8; 16];
        assert_eq!(
            run_block(
                &mut AlwaysBusy,
                &[0u8; 16],
                &[0u8; 16],
                &mut output,
                false,
                16
            ),
            Err(AesError::Timeout)
        );
    }

    #[test]
    fn zero_timeout_iterations_is_rejected() {
        assert_eq!(
            validate_timeout_iterations(0),
            Err(AesError::ZeroIterations)
        );
        assert_eq!(validate_timeout_iterations(1), Ok(1));
    }

    #[test]
    fn self_test_vectors_are_standard_aes() {
        // Independent confirmation that the baked-in KAT vectors are correct
        // AES-128 (guards against a typo silently weakening the on-silicon
        // gate), computed straight from RustCrypto without the register path.
        for (key, plaintext, ciphertext) in SELF_TEST_VECTORS.iter() {
            let mut block = GenericArray::clone_from_slice(plaintext);
            Aes128::new(GenericArray::from_slice(key)).encrypt_block(&mut block);
            assert_eq!(block.as_slice(), ciphertext);
        }
    }
}
