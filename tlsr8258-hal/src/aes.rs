//! TLSR8258 hardware AES-128 ECB single-block accelerator.
//!
//! # Register evidence
//!
//! `platform/chip_8258/register.h` (open header, full confidence):
//! ```c
//! /*******************************      aes registers: 0x540      ******************************/
//! #define reg_aes_ctrl            REG_ADDR8(0x540)
//! enum {
//!     FLD_AES_CTRL_CODEC_TRIG = BIT(0),
//!     FLD_AES_CTRL_DATA_FEED = BIT(1),
//!     FLD_AES_CTRL_CODEC_FINISHED = BIT(2),
//! };
//! #define reg_aes_data            REG_ADDR32(0x548)
//! #define reg_aes_key(v)          REG_ADDR8(0x550+v)
//! ```
//! and, in the same file's reset/clock blocks:
//! ```c
//! #define reg_rst1  REG_ADDR8(0x61)
//! enum{ ... FLD_RST1_AES = BIT(4), ... };
//! #define reg_clk_en1 REG_ADDR8(0x64)
//! enum{ ... FLD_CLK1_AES_EN = BIT(4), ... };
//! ```
//! `platform/chip_8258/aes.h` only *declares* `aes_encrypt`/`aes_decrypt`
//! as `extern` — unlike `gpio.h`/`pwm.h`/`watchdog.h`, this chip ships no
//! open-source body for this peripheral, so (matching this crate's
//! existing tiering for `mmio.rs`'s `analog_read`/`analog_write`, sourced
//! the same way) the actual register *protocol* below was recovered by
//! disassembling the compiled `aes_encrypt`/`aes_decrypt` objects out of
//! `platform/lib/libdrivers_8258.a:aes.o` with the tc32 `llvm-objdump`
//! shipped in this workspace's own tc32 toolchain — no vendor code was
//! linked into this crate or copied byte-for-byte; only the *protocol* it
//! implements was transcribed by reading the disassembly. This is a
//! **medium-high, not full**, confidence tier: lower than the open
//! `static inline` headers this crate otherwise prefers, but grounded in
//! the vendor's own compiled implementation rather than guesswork, and
//! cross-checked internally (see below) rather than taken on faith from a
//! single disassembly reading.
//!
//! `aes_encrypt(key: r0, data: r1, result: r2)` disassembles to (tc32,
//! `--print-imm-hex`, addresses relative to function start):
//! ```text
//!  0: tpush   {r4,r5,r6,lr}
//!  2: tloadr  r3, [pc, #0x6c]      @ r3 = 0x00800540 (reg_aes_ctrl)
//!  4: tloadrb r4, [r3]             @ r4 = *reg_aes_ctrl
//!  6: tmov    r5, #0x1
//!  8: tbclr   r4, r5               @ r4 &= ~r5   (clears bit0, mask=r5=1)
//!  a: tstorerb r4, [r3]            @ *reg_aes_ctrl = r4   (bit0 forced 0)
//!  c: tadd    r3, #0x10            @ r3 = 0x800550 (reg_aes_key(0))
//!  e: tloadr  r5, [pc, #0x64]      @ r5 = 0x00800560 (reg_aes_key(16), loop end)
//! 10: tloadrb r4, [r0]             @ copy Key[i] -> *r3, i = 0..16,
//! 12: tstorerb r4, [r3]            @   byte order unchanged (not reversed)
//! 14: tadd    r0, #0x1
//! 16: tadd    r3, #0x1
//! 18: tcmp    r3, r5
//! 1a: tjne    0x10                 @ loop while r3 != 0x800560
//! 1c: tloadr  r5, [pc, #0x50]      @ r5 = 0x00800540 (reg_aes_ctrl, reused)
//! 1e: tloadrb r3, [r5]             @ r3 = *reg_aes_ctrl (post key-load)
//! 20: tshftl  r0, r3, #0x1e        @ r0 = r3 << 30   (isolates bit1 into bit31)
//! 22: tjpl    0x46                 @ if bit1 clear, skip the feed loop entirely
//! 24: tloadr  r6, [pc, #0x50]      @ r6 = 0x00800548 (reg_aes_data)
//! 26..3c:                          @ assemble Data[4i..4i+4] little-endian,
//!                                  @   *reg_aes_data (32-bit) = word
//! 3e: tadd    r1, #0x4
//! 40: tloadrb r3, [r5]             @ re-read reg_aes_ctrl
//! 42: tshftl  r4, r3, #0x8         @ (sets Z iff r3 == 0)
//! 44: tjne    0x28                 @ loop feeding words while ctrl != 0
//! 46: tloadr  r0, [pc, #0x28]      @ r0 = 0x00800540 (reg_aes_ctrl)
//! 4a: tloadrb r3, [r0]
//! 4c: tshftl  r1, r3, #0x8         @ (sets Z iff r3 == 0)
//! 4e: tjeq    0x4a                 @ spin **unconditionally** while ctrl == 0
//! 50..6a: read *reg_aes_data (32-bit) x4, little-endian byte order,
//!         into Result[0..16]
//! 6c: tmov    r0, #0x0             @ return 0
//! 6e: tpop    {r4,r5,r6,pc}
//! ```
//! `aes_decrypt` disassembles identically except its opening sequence is
//! `tmov r4,#1; tor r4,r5; <truncate to byte>; tstorerb r4,[r3]` — i.e. it
//! **sets** bit0 instead of clearing it, everything else (key load, data
//! feed handshake, finished poll, result read) is byte-for-byte the same
//! object code. This cross-comparison is what pins down
//! `FLD_AES_CTRL_CODEC_TRIG` (bit0) as an encrypt(0)/decrypt(1) *mode
//! select* bit that is read-modify-written once up front (not a one-shot
//! "start" pulse — nothing in either function ever sets bit0 for encrypt,
//! and nothing explicitly triggers a "start"; the block evidently begins
//! running as soon as its key and data are fully fed), rather than reading
//! either disassembly in isolation.
//!
//! # Protocol this module implements
//!
//! 1. Read-modify-write `reg_aes_ctrl` bit0 (`FLD_AES_CTRL_CODEC_TRIG`) to
//!    select encrypt (`0`) or decrypt (`1`), preserving all other bits.
//! 2. Write the 16 key bytes one at a time, in the caller's array order, to
//!    `reg_aes_key(0..16)` (`0x550..0x560`).
//! 3. If `reg_aes_ctrl` bit1 (`FLD_AES_CTRL_DATA_FEED`) is now set, feed the
//!    16-byte block as four little-endian 32-bit words to `reg_aes_data`
//!    (`0x548`). Bit1 must remain set through the first three writes and
//!    clear after the fourth; any other transition is rejected.
//! 4. Poll `reg_aes_ctrl` bit2 (`FLD_AES_CTRL_CODEC_FINISHED`) until set.
//! 5. Read the 128-bit result as four little-endian 32-bit words from the
//!    same `reg_aes_data` port.
//!
//! # Deliberate deviations from the vendor object
//!
//! * **Bounded, bit-specific feed handshake.** After each data write the
//!   vendor object loops while the *whole* `reg_aes_ctrl` byte is nonzero
//!   (`tshftl ..., #8; tjne`), without bounding how many words it reads
//!   from the caller. This module deliberately tests the header-defined
//!   `FLD_AES_CTRL_DATA_FEED` bit instead: mode/finished bits must not
//!   request a fifth word. It requires that bit to stay asserted through
//!   the first three words and clear after the fourth, returning
//!   [`AesError::UnexpectedDataFeed`] for either an early de-assertion or
//!   a request beyond the 16-byte block. This fail-closed interpretation
//!   still requires silicon validation.
//! * **Bounded finished-poll.** The vendor object spins on step 4
//!   *unconditionally* (`tjeq` back to itself with no counter or
//!   timeout) — see the disassembly transcript above. This module bounds
//!   that wait to a caller-supplied `timeout_iterations` and returns
//!   [`AesError::Timeout`] instead of hanging forever if the accelerator
//!   never finishes, matching this crate's existing rule (`mmio.rs`,
//!   `i2c.rs`, `spi.rs`) that no wait may be unbounded.
//! * **No status-bit write-back.** Neither vendor function ever writes
//!   `FLD_AES_CTRL_CODEC_FINISHED`/`FLD_AES_CTRL_DATA_FEED` back to `0`
//!   after use (both bits are presumably hardware-owned status, not
//!   software-writable); this module does not attempt to clear them
//!   either, and the mock in this module's tests enforces that only bit0
//!   is ever software-writable so a future edit can't silently start
//!   relying on writing status bits.
//!
//! # Clock/reset ownership
//!
//! [`AesEngine::new`] enables the AES clock-gate and pulses the AES reset
//! bit through `crate::reset`'s shared per-peripheral facade
//! ([`crate::reset::enable_clock`]/[`crate::reset::pulse_reset`] with
//! [`crate::reset::Peripheral::Aes`]) rather than poking
//! `reg_clk_en1`/`reg_rst1` directly — that facade's own docs already
//! reserve the `Aes` variant "for `aes.rs` ... to adopt later", which this
//! module now does. Both facade calls are themselves read-modify-writes
//! that preserve unrelated bits (other peripherals' clock-enables/
//! resets), and ownership of *this* peripheral's registers is established
//! by the caller handing over the zero-sized [`crate::peripherals::Aes`]
//! token, which [`crate::peripherals::Peripherals::take`] hands out at
//! most once.
//!
//! This module previously (before that facade existed) hand-rolled its
//! own local read-enable-bit/OR/write and assert-reset/deassert-reset
//! sequence, matching the copies `i2c.rs`/`spi.rs`/`pwm.rs` had at the
//! time this module was written. All of those drivers now consume the
//! shared facade instead of keeping local reset/clock sequences.
//!
//! # Hardware validation caveat
//!
//! This module has **not** been exercised against real TLSR8258 silicon.
//! Every claim above about the polled protocol comes from (a) the open
//! `register.h` bit/address definitions and (b) disassembling the
//! vendor's own compiled `aes_encrypt`/`aes_decrypt` objects — cross
//! checked against each other, not from a datasheet timing diagram or an
//! oscilloscope. In particular [`AesEngine::DEFAULT_TIMEOUT_ITERATIONS`]
//! is a conservative placeholder, not a measured bound (the vendor object
//! itself never bounds this wait at all, so there is no vendor timing
//! figure to anchor to either) — bench validation on real hardware before
//! relying on this timeout in production is required.
//!
//! # No DMA
//!
//! This module only implements the polled, non-DMA single-block path
//! (`reg_aes_data` fed/drained one 32-bit word at a time by the CPU).
//! `register.h`'s `FLD_DMA_CHN_AES_IN`/`FLD_DMA_CHN_AES_OUT` /
//! `FLD_DMA_IRQ_AES_DECO`/`FLD_DMA_IRQ_AES_CODE` bits exist for a
//! DMA-driven path, but this crate has no DMA-channel-ownership facility
//! yet (see `pwm.rs`'s own "Explicitly unsupported" section, which
//! documents the same gap) and CCM* only ever needs one 16-byte block at a
//! time, so the added complexity is not justified here.

use crate::mmio::REG_BASE;
#[cfg(target_arch = "tc32")]
use crate::mmio::{r8, r32, w8, w32};

/// `reg_aes_ctrl` (`register.h`, `aes registers: 0x540` block).
const REG_AES_CTRL: u32 = REG_BASE + 0x540;
/// `reg_aes_data` (32-bit; shared write-in/read-out port register).
const REG_AES_DATA: u32 = REG_BASE + 0x548;
/// `reg_aes_key(0)`; `reg_aes_key(v) == REG_AES_KEY_BASE + v` for `v` in
/// `0..16`.
const REG_AES_KEY_BASE: u32 = REG_BASE + 0x550;

/// `FLD_AES_CTRL_CODEC_TRIG` — encrypt(0)/decrypt(1) mode select, written
/// once up front (see module docs for why this is a mode bit, not a start
/// pulse).
const FLD_AES_CTRL_CODEC_TRIG: u8 = 1 << 0;
/// `FLD_AES_CTRL_DATA_FEED` — hardware-owned status: set once the
/// accelerator wants its 16-byte data block, clears once fed.
const FLD_AES_CTRL_DATA_FEED: u8 = 1 << 1;
/// `FLD_AES_CTRL_CODEC_FINISHED` — hardware-owned status: set once the
/// block operation has completed and the result is ready to read.
const FLD_AES_CTRL_CODEC_FINISHED: u8 = 1 << 2;

/// Errors returned by [`AesEngine::encrypt_block`]/[`AesEngine::decrypt_block`]
/// and by [`AesEngine::new`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AesError {
    /// `timeout_iterations == 0` was passed to [`AesEngine::new`] — that
    /// would make every subsequent finished-poll fail immediately, which
    /// is almost certainly not what the caller intended (mirrors
    /// `i2c::I2cError::InvalidTimeout`/`watchdog::WatchdogError::ZeroPeriod`'s
    /// existing "reject rather than silently misbehave" convention).
    ZeroIterations,
    /// The accelerator de-asserted `FLD_AES_CTRL_DATA_FEED` before all four
    /// words were accepted, or still asserted it after the complete
    /// 16-byte block. Both states violate this module's bounded handshake;
    /// the vendor object can instead continue into an out-of-bounds fifth
    /// read when another control bit remains set.
    UnexpectedDataFeed,
    /// `FLD_AES_CTRL_CODEC_FINISHED` did not become set within the
    /// configured `timeout_iterations` bounded-wait budget. The vendor
    /// object spins here unconditionally instead (see module docs) — this
    /// is a hard failure, not a "assume done and read garbage" fallback.
    Timeout,
}

/// Abstraction over the raw AES-128 ECB accelerator registers, so the
/// exact polled protocol in [`run_block`] can run unchanged against either
/// the real TLSR8258 MMIO ([`HwRegisters`], tc32-only) or the in-memory
/// mock this module's host tests drive against NIST AES-128 known-answer
/// vectors (see `tests::MockRegisters`). Not part of this module's public
/// API: callers only ever see [`AesEngine`].
trait AesRegisters {
    fn read_ctrl(&mut self) -> u8;
    fn write_ctrl(&mut self, value: u8);
    /// Write one byte of `reg_aes_key(index)`; `index` is always `0..16`.
    fn write_key_byte(&mut self, index: usize, value: u8);
    /// Write one little-endian 32-bit word to `reg_aes_data`.
    fn write_data_word(&mut self, value: u32);
    /// Read one little-endian 32-bit word from `reg_aes_data`.
    fn read_data_word(&mut self) -> u32;
}

#[cfg(target_arch = "tc32")]
struct HwRegisters;

#[cfg(target_arch = "tc32")]
impl AesRegisters for HwRegisters {
    fn read_ctrl(&mut self) -> u8 {
        unsafe { r8(REG_AES_CTRL) }
    }
    fn write_ctrl(&mut self, value: u8) {
        unsafe { w8(REG_AES_CTRL, value) }
    }
    fn write_key_byte(&mut self, index: usize, value: u8) {
        unsafe { w8(REG_AES_KEY_BASE + index as u32, value) }
    }
    fn write_data_word(&mut self, value: u32) {
        unsafe { w32(REG_AES_DATA, value) }
    }
    fn read_data_word(&mut self) -> u32 {
        unsafe { r32(REG_AES_DATA) }
    }
}

/// Reject a zero iteration bound up front — see [`AesError::ZeroIterations`].
const fn validate_timeout_iterations(timeout_iterations: u32) -> Result<u32, AesError> {
    if timeout_iterations == 0 {
        Err(AesError::ZeroIterations)
    } else {
        Ok(timeout_iterations)
    }
}

/// Run the full encrypt/decrypt protocol described in the module docs
/// against `regs`. Pure register-order/state-machine logic, independent of
/// whether `regs` is real MMIO or a host test mock.
fn run_block<R: AesRegisters>(
    regs: &mut R,
    key: &[u8; 16],
    input: &[u8; 16],
    output: &mut [u8; 16],
    decrypt: bool,
    timeout_iterations: u32,
) -> Result<(), AesError> {
    // 1. Mode select: read-modify-write bit0, preserving every other bit.
    let ctrl = regs.read_ctrl();
    let ctrl = if decrypt {
        ctrl | FLD_AES_CTRL_CODEC_TRIG
    } else {
        ctrl & !FLD_AES_CTRL_CODEC_TRIG
    };
    regs.write_ctrl(ctrl);

    // 2. Load the key, byte for byte, in the caller's array order.
    for (index, &byte) in key.iter().enumerate() {
        regs.write_key_byte(index, byte);
    }

    // 3. Feed the data block only if DATA_FEED is asserted, bounded to
    //    exactly 4 words (16 bytes) — see module docs' "Deliberate
    //    deviations" section.
    if regs.read_ctrl() & FLD_AES_CTRL_DATA_FEED != 0 {
        for word_index in 0..4usize {
            let start = word_index * 4;
            let word = u32::from_le_bytes([
                input[start],
                input[start + 1],
                input[start + 2],
                input[start + 3],
            ]);
            regs.write_data_word(word);
            let still_requested = regs.read_ctrl() & FLD_AES_CTRL_DATA_FEED != 0;
            let expect_another_word = word_index < 3;
            if still_requested != expect_another_word {
                return Err(AesError::UnexpectedDataFeed);
            }
        }
    }

    // 4. Bounded finished-poll (vendor object spins here unconditionally).
    let mut finished = false;
    for _ in 0..timeout_iterations {
        if regs.read_ctrl() & FLD_AES_CTRL_CODEC_FINISHED != 0 {
            finished = true;
            break;
        }
        core::hint::spin_loop();
    }
    if !finished {
        return Err(AesError::Timeout);
    }

    // 5. Drain the 128-bit result, little-endian, from the same port.
    for word_index in 0..4usize {
        let word = regs.read_data_word();
        let start = word_index * 4;
        output[start..start + 4].copy_from_slice(&word.to_le_bytes());
    }
    Ok(())
}

/// Owns the TLSR8258 AES-128 ECB hardware accelerator.
///
/// Constructed from the exclusive [`crate::peripherals::Aes`] token, so at
/// most one [`AesEngine`] can exist at a time — see the module docs'
/// "Clock/reset ownership" section for exactly which registers this
/// establishes ownership of and how.
pub struct AesEngine {
    timeout_iterations: u32,
}

impl AesEngine {
    /// A conservative placeholder bound for the `FLD_AES_CTRL_CODEC_FINISHED`
    /// poll — see the module docs' "Hardware validation caveat": the vendor
    /// object this was derived from never bounds this wait at all, so there
    /// is no vendor timing figure to anchor to. Callers with real hardware
    /// timing data should measure and pass their own bound to [`Self::new`]
    /// instead of relying on this constant.
    pub const DEFAULT_TIMEOUT_ITERATIONS: u32 = 10_000;

    /// Take ownership of the AES accelerator, enabling its clock and
    /// pulsing its reset (see module docs). `timeout_iterations` bounds
    /// every subsequent [`Self::encrypt_block`]/[`Self::decrypt_block`]
    /// call's wait for `FLD_AES_CTRL_CODEC_FINISHED`.
    #[cfg(target_arch = "tc32")]
    pub fn new(
        _peripheral: crate::peripherals::Aes,
        timeout_iterations: u32,
    ) -> Result<Self, AesError> {
        let timeout_iterations = validate_timeout_iterations(timeout_iterations)?;
        // Clock-gate and reset the AES block via the generic
        // reg_clk_en1/reg_rst1 facade (see `crate::reset::Peripheral::Aes`),
        // matching `i2c.rs`'s/`spi.rs`'s own migration to the same facade.
        crate::reset::enable_clock(crate::reset::Peripheral::Aes)
            .expect("AES has a documented reg_clk_en1 bit");
        crate::reset::pulse_reset(crate::reset::Peripheral::Aes);
        Ok(Self { timeout_iterations })
    }

    /// Encrypt one 16-byte block in place with `key`.
    #[cfg(target_arch = "tc32")]
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
    /// CCM* (this workspace's only current consumer of AES on this
    /// platform) never calls this — CCM* uses only the forward/encrypt
    /// permutation for both its CBC-MAC and CTR-mode keystream, by
    /// design. This is provided because the hardware evidence for it is
    /// exactly as clear as for [`Self::encrypt_block`] (see module docs)
    /// and it is essentially free to offer alongside it, not because
    /// anything in this workspace currently needs it.
    #[cfg(target_arch = "tc32")]
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use aes::Aes128;
    use aes::cipher::generic_array::GenericArray;
    use aes::cipher::{BlockDecrypt, BlockEncrypt, KeyInit};

    /// Host-only mock of the register interface driven by [`run_block`].
    /// Reproduces the exact hand-shake this module documents (mode-select
    /// bit, `DATA_FEED` asserted once the key is fully loaded, cleared
    /// once the block is fully fed, `CODEC_FINISHED` set after a
    /// caller-chosen number of polls) while performing the *actual*
    /// AES-128 permutation with the independently-vetted RustCrypto `aes`
    /// crate (already used elsewhere in this workspace, see `rng.rs`) as
    /// the oracle.
    ///
    /// This validates [`run_block`]'s register **ordering**/bit
    /// **sequencing** end-to-end against real NIST known-answer vectors —
    /// a byte or word placed in the wrong slot or order would produce the
    /// wrong ciphertext/plaintext even though the underlying math is
    /// correct. It does **not** and cannot validate the real silicon's own
    /// AES implementation; see the module-level "Hardware validation
    /// caveat".
    struct MockRegisters {
        ctrl: u8,
        key: [u8; 16],
        key_bytes_written: usize,
        input: [u8; 16],
        input_words_written: usize,
        output: [u8; 16],
        output_words_read: usize,
        result_ready: bool,
        finished_polls_remaining: u32,
    }

    impl MockRegisters {
        fn new(finished_polls_remaining: u32) -> Self {
            Self {
                ctrl: 0,
                key: [0; 16],
                key_bytes_written: 0,
                input: [0; 16],
                input_words_written: 0,
                output: [0; 16],
                output_words_read: 0,
                result_ready: false,
                finished_polls_remaining,
            }
        }

        fn run_cipher(&mut self) {
            let decrypt = self.ctrl & FLD_AES_CTRL_CODEC_TRIG != 0;
            let cipher = Aes128::new(GenericArray::from_slice(&self.key));
            let mut block = GenericArray::clone_from_slice(&self.input);
            if decrypt {
                cipher.decrypt_block(&mut block);
            } else {
                cipher.encrypt_block(&mut block);
            }
            self.output.copy_from_slice(block.as_slice());
            self.result_ready = true;
        }
    }

    impl AesRegisters for MockRegisters {
        fn read_ctrl(&mut self) -> u8 {
            if self.result_ready && self.ctrl & FLD_AES_CTRL_CODEC_FINISHED == 0 {
                if self.finished_polls_remaining == 0 {
                    self.ctrl |= FLD_AES_CTRL_CODEC_FINISHED;
                } else {
                    self.finished_polls_remaining -= 1;
                }
            }
            self.ctrl
        }

        fn write_ctrl(&mut self, value: u8) {
            // Only bit0 (mode select) is ever software-writable in this
            // module's protocol; the status bits are hardware-owned (see
            // module docs' "No status-bit write-back").
            self.ctrl = (self.ctrl & !FLD_AES_CTRL_CODEC_TRIG) | (value & FLD_AES_CTRL_CODEC_TRIG);
        }

        fn write_key_byte(&mut self, index: usize, value: u8) {
            self.key[index] = value;
            self.key_bytes_written += 1;
            if self.key_bytes_written == 16 {
                self.ctrl |= FLD_AES_CTRL_DATA_FEED;
            }
        }

        fn write_data_word(&mut self, value: u32) {
            let start = self.input_words_written * 4;
            self.input[start..start + 4].copy_from_slice(&value.to_le_bytes());
            self.input_words_written += 1;
            if self.input_words_written == 4 {
                self.ctrl &= !FLD_AES_CTRL_DATA_FEED;
                self.run_cipher();
            }
        }

        fn read_data_word(&mut self) -> u32 {
            let start = self.output_words_read * 4;
            let mut bytes = [0u8; 4];
            bytes.copy_from_slice(&self.output[start..start + 4]);
            self.output_words_read += 1;
            u32::from_le_bytes(bytes)
        }
    }

    // FIPS-197 Appendix B / C.1's AES-128 known-answer vector.
    const KAT_KEY: [u8; 16] = [
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
        0x0f,
    ];
    const KAT_PLAINTEXT: [u8; 16] = [
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee,
        0xff,
    ];
    const KAT_CIPHERTEXT: [u8; 16] = [
        0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5,
        0x5a,
    ];

    #[test]
    fn encrypt_matches_fips197_known_answer_vector() {
        let mut regs = MockRegisters::new(0);
        let mut output = [0u8; 16];
        run_block(&mut regs, &KAT_KEY, &KAT_PLAINTEXT, &mut output, false, 10)
            .expect("encrypt should succeed");
        assert_eq!(output, KAT_CIPHERTEXT);
        // Confirms the key/data byte order the disassembly evidence
        // documents: neither is reversed on the way into the mock.
        assert_eq!(regs.key, KAT_KEY);
        assert_eq!(regs.input, KAT_PLAINTEXT);
    }

    #[test]
    fn decrypt_matches_fips197_known_answer_vector() {
        let mut regs = MockRegisters::new(0);
        let mut output = [0u8; 16];
        run_block(&mut regs, &KAT_KEY, &KAT_CIPHERTEXT, &mut output, true, 10)
            .expect("decrypt should succeed");
        assert_eq!(output, KAT_PLAINTEXT);
    }

    #[test]
    fn bounded_finished_poll_succeeds_before_the_configured_timeout() {
        let mut regs = MockRegisters::new(5);
        let mut output = [0u8; 16];
        run_block(&mut regs, &KAT_KEY, &KAT_PLAINTEXT, &mut output, false, 10)
            .expect("5 polls to finish should fit inside a 10-iteration budget");
        assert_eq!(output, KAT_CIPHERTEXT);
    }

    #[test]
    fn finished_poll_times_out_instead_of_hanging() {
        let mut regs = MockRegisters::new(1_000);
        let mut output = [0u8; 16];
        let result = run_block(&mut regs, &KAT_KEY, &KAT_PLAINTEXT, &mut output, false, 5);
        assert_eq!(result, Err(AesError::Timeout));
    }

    /// A register mock that never de-asserts `DATA_FEED`, simulating a
    /// misbehaving/faulty accelerator asking for a 5th word of a 16-byte
    /// block.
    struct StuckDataFeedRegisters;

    impl AesRegisters for StuckDataFeedRegisters {
        fn read_ctrl(&mut self) -> u8 {
            FLD_AES_CTRL_DATA_FEED
        }
        fn write_ctrl(&mut self, _value: u8) {}
        fn write_key_byte(&mut self, _index: usize, _value: u8) {}
        fn write_data_word(&mut self, _value: u32) {}
        fn read_data_word(&mut self) -> u32 {
            0
        }
    }

    #[test]
    fn stuck_data_feed_is_a_hard_error_not_an_out_of_bounds_read() {
        let mut regs = StuckDataFeedRegisters;
        let mut output = [0u8; 16];
        let result = run_block(&mut regs, &KAT_KEY, &KAT_PLAINTEXT, &mut output, false, 10);
        assert_eq!(result, Err(AesError::UnexpectedDataFeed));
    }

    struct EarlyDataFeedDropRegisters {
        ctrl: u8,
    }

    impl AesRegisters for EarlyDataFeedDropRegisters {
        fn read_ctrl(&mut self) -> u8 {
            self.ctrl
        }
        fn write_ctrl(&mut self, value: u8) {
            self.ctrl = value & FLD_AES_CTRL_CODEC_TRIG;
        }
        fn write_key_byte(&mut self, index: usize, _value: u8) {
            if index == 15 {
                self.ctrl |= FLD_AES_CTRL_DATA_FEED;
            }
        }
        fn write_data_word(&mut self, _value: u32) {
            self.ctrl &= !FLD_AES_CTRL_DATA_FEED;
            self.ctrl |= FLD_AES_CTRL_CODEC_FINISHED;
        }
        fn read_data_word(&mut self) -> u32 {
            0
        }
    }

    #[test]
    fn early_data_feed_drop_cannot_return_a_partial_block() {
        let mut regs = EarlyDataFeedDropRegisters { ctrl: 0 };
        let mut output = [0u8; 16];
        let result = run_block(&mut regs, &KAT_KEY, &KAT_PLAINTEXT, &mut output, false, 10);
        assert_eq!(result, Err(AesError::UnexpectedDataFeed));
    }

    /// A register mock where `DATA_FEED` never asserts and
    /// `CODEC_FINISHED` never sets, simulating hardware that never reacts
    /// (e.g. a clock/reset ownership bug upstream of this module).
    struct NeverReadyRegisters;

    impl AesRegisters for NeverReadyRegisters {
        fn read_ctrl(&mut self) -> u8 {
            0
        }
        fn write_ctrl(&mut self, _value: u8) {}
        fn write_key_byte(&mut self, _index: usize, _value: u8) {}
        fn write_data_word(&mut self, _value: u32) {}
        fn read_data_word(&mut self) -> u32 {
            0
        }
    }

    #[test]
    fn never_ready_hardware_times_out_rather_than_hanging() {
        let mut regs = NeverReadyRegisters;
        let mut output = [0u8; 16];
        let result = run_block(&mut regs, &KAT_KEY, &KAT_PLAINTEXT, &mut output, false, 10);
        assert_eq!(result, Err(AesError::Timeout));
    }

    #[test]
    fn zero_timeout_iterations_are_rejected() {
        assert_eq!(
            validate_timeout_iterations(0),
            Err(AesError::ZeroIterations)
        );
        assert_eq!(validate_timeout_iterations(1), Ok(1));
    }

    #[test]
    fn register_addresses_and_bits_match_register_h() {
        assert_eq!(REG_AES_CTRL, 0x800540);
        assert_eq!(REG_AES_DATA, 0x800548);
        assert_eq!(REG_AES_KEY_BASE, 0x800550);
        assert_eq!(FLD_AES_CTRL_CODEC_TRIG, 0x01);
        assert_eq!(FLD_AES_CTRL_DATA_FEED, 0x02);
        assert_eq!(FLD_AES_CTRL_CODEC_FINISHED, 0x04);
    }

    #[test]
    fn reset_facade_aes_bits_match_register_h() {
        // Cross-check that `crate::reset::Peripheral::Aes` (owned by
        // another module) still carries the same bit4/bit4
        // reg_clk_en1/reg_rst1 values this module's own disassembly-based
        // evidence independently derived, since `AesEngine::new` now
        // depends on that facade instead of hand-rolling the pokes here.
        assert_eq!(crate::reset::Peripheral::Aes.clock_bit(), Some(0x10));
        assert_eq!(crate::reset::Peripheral::Aes.reset_bit(), 0x10);
    }
}
