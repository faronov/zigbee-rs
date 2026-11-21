//! `LAES` AES-128 forward block engine of the CC2340R52.
//!
//! The CC23x0 carries one 128-bit-key AES core (`AES_BASE` 0x400C_0000,
//! clock gate `CLKCTL.CLKCFG0.LAES`). TI's drivers use it for ECB, CBC,
//! CBC-MAC, CTR and CCM by driving single forward-cipher operations from the
//! CPU (or the uDMA) through `AUTOCFG` triggers; there is no decrypt
//! direction. Zigbee CCM* and AES-MMO only need the forward permutation, so
//! this module exposes exactly one primitive: encrypt one block.
//!
//! # Register sequence
//!
//! Mirrors the CPU (polling, non-DMA) single-block path of TI's
//! `AESECBLPF3.c` / `AESCommonLPF3.c` and the `driverlib/aes.h` helpers of
//! the SimpleLink Low Power F3 SDK 8.40 (`hw_aes.h` for the offsets):
//!
//! 1. `CLKCTL.CLKENSET0 = LAES` (`Power_setDependency(PowerLPF3_PERIPH_AES)`),
//!    confirmed through `CLKCTL.CLKCFG0` with a bounded poll;
//! 2. the engine must be idle (`STA.STATE`); stale `RIS` flags are cleared
//!    (`ICLR`);
//! 3. `KEY0..KEY3` (`AESWriteKEY`), then
//!    `AUTOCFG = AESSRC_BUF | TRGAES_WRBUF3S`
//!    (`AESECBLPF3_SINGLE_BLOCK_AUTOCFG` without `BUSHALT`, see below);
//! 4. `BUF0..BUF3` (`AESWriteBUF`); the `BUF3` write schedules exactly one
//!    `TXT = AES(KEY, BUF)` operation;
//! 5. bounded poll until `RIS.AESDONE` is set and `STA.STATE` is idle;
//! 6. read `TXT0..TXT3` (`AESReadTXT`);
//! 7. `ABORT` (clears `TXT`, `BUF`, `DMA`, `AUTOCFG`) and `ICLR`, as
//!    `AESCommonLPF3_cleanup()` does after every operation.
//!
//! Byte order: `AESCopyBlock` copies word-aligned blocks with `LDM`/`STM`
//! on the little-endian Cortex-M0+, so register word *n* holds bytes
//! `4n..4n+4` of the standard FIPS-197 block in little-endian order (byte
//! `4n` in bits 7:0). `KEY`, `BUF` and `TXT` all use that mapping.
//!
//! `AUTOCFG.BUSHALT` is left disabled, unlike TI's default: with it, an
//! access to `KEY`/`TXT`/`BUF` while the core is busy stalls the bus until
//! the core is idle, which is an unbounded wait if the core ever wedges.
//! Without it such accesses are ignored or read as zero, so the explicit
//! bounded completion poll above is the only synchronisation and a wedged
//! core turns into [`AesError::Timeout`] (TI's `AES_BUSHALT_DISABLED` build
//! variant polls `STA.STATE` the same way).
//!
//! # Ownership
//!
//! [`Aes`] is the singleton token from [`crate::Core`]; [`AesEngine`]
//! consumes it, so nothing else in a firmware image can program the core.
//! The CC2340 radio (LRFD/PBE driven by `zigbee-mac`, and TI's RCL alike)
//! never touches `LAES`: TI's Zigbee stack performs MAC/NWK/APS security on
//! the CPU through the same `AESECB` driver this module mirrors. The uDMA
//! channels 4/5 that TI can route to `LAES` (`EVTSVT.DMACH4SEL/DMACH5SEL`)
//! are not used by this HAL. The bootloader (`take_boot`) does not get the
//! token.
//!
//! # Power
//!
//! The `LAES` clock stays enabled for the engine's lifetime, as TI keeps its
//! power dependency for the lifetime of an open AES handle. Peripheral
//! registers must be assumed lost in standby: TI's I2C, UART2, SPI, LGPT and
//! uDMA drivers re-initialise their hardware on `PowerLPF3_AWAKE_STANDBY`.
//! TI's AES drivers register no such hook because they keep no register
//! state between operations (polling operations cannot be interrupted by
//! standby; DMA operations forbid it). This engine follows the same rule:
//! every block reprograms `KEY`, `AUTOCFG` and `BUF` and ends with `ABORT`,
//! and standby is only entered from the idle loop, never inside a block.
//! TI's power driver does not re-set `CLKCTL` gates after standby; should
//! the `LAES` clock nevertheless be found disabled (after standby, or gated
//! by foreign code) it is re-enabled and the known-answer self-test is
//! re-run before any output is published.
//!
//! Errors never fall back to software AES and never publish partial output.

use crate::clkctl::PeripheralClock;

/// `AES_BASE` (`hw_memmap.h`).
pub const AES_BASE: u32 = 0x400C_0000;
/// `AES_O_ABORT`.
pub const AES_ABORT: u32 = AES_BASE + 0x14;
/// `AES_O_STA`.
pub const AES_STA: u32 = AES_BASE + 0x1C;
/// `AES_O_AUTOCFG`.
pub const AES_AUTOCFG: u32 = AES_BASE + 0x2C;
/// `AES_O_KEY0` (`KEY1..KEY3` follow at +4).
pub const AES_KEY0: u32 = AES_BASE + 0x50;
/// `AES_O_TXT0` (`TXT1..TXT3` follow at +4).
pub const AES_TXT0: u32 = AES_BASE + 0x70;
/// `AES_O_BUF0` (`BUF1..BUF3` follow at +4).
pub const AES_BUF0: u32 = AES_BASE + 0x90;
/// `AES_O_RIS`.
pub const AES_RIS: u32 = AES_BASE + 0x108;
/// `AES_O_ICLR`.
pub const AES_ICLR: u32 = AES_BASE + 0x114;

/// `AES_STA_STATE_BUSY`.
pub const STA_STATE_BUSY: u32 = 0x0000_0001;
/// `AES_AUTOCFG_AESSRC_BUF`: hardware-triggered operations compute
/// `TXT = AES(KEY, BUF)`.
pub const AUTOCFG_AESSRC_BUF: u32 = 0x0000_0020;
/// `AES_AUTOCFG_TRGAES_WRBUF3S`: the next `BUF3` write schedules a single
/// operation.
pub const AUTOCFG_TRGAES_WRBUF3S: u32 = 0x0000_0008;
/// Single-block `AUTOCFG` (TI's `AESECBLPF3_SINGLE_BLOCK_AUTOCFG` with
/// `BUSHALT` disabled, see the module documentation).
pub const AUTOCFG_SINGLE_BLOCK: u32 = AUTOCFG_AESSRC_BUF | AUTOCFG_TRGAES_WRBUF3S;
/// `AES_RIS_AESDONE`.
pub const RIS_AESDONE: u32 = 0x0000_0001;
/// `AES_ICLR_ALL` (`driverlib/aes.h`): `AESDONE | AESSTART | CHADONE |
/// CHBDONE`.
pub const ICLR_ALL: u32 = 0x0000_000F;
/// `AES_ABORT_ABORTAES_SET`.
pub const ABORT_ABORTAES: u32 = 0x0000_0001;

/// Errors from the `LAES` engine. None of them publishes output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AesError {
    /// A zero poll budget would make every operation fail.
    ZeroIterations,
    /// `CLKCFG0.LAES` did not report the clock running within the budget.
    ClockUnavailable,
    /// The core was busy before this engine programmed it (a foreign owner
    /// or a wedged operation); nothing was written.
    Busy,
    /// The scheduled operation did not complete within the budget; the
    /// core was aborted and cleared.
    Timeout,
    /// The on-silicon known-answer test produced a wrong ciphertext.
    KnownAnswerMismatch,
}

/// Register access used by the engine; the target implementation is
/// volatile MMIO, the host tests use an in-memory model of the core.
pub(crate) trait AesRegisters {
    fn clock_enabled(&mut self) -> bool;
    fn request_clock(&mut self);
    fn read_sta(&mut self) -> u32;
    fn read_ris(&mut self) -> u32;
    fn write_iclr(&mut self, value: u32);
    fn write_abort(&mut self, value: u32);
    fn write_autocfg(&mut self, value: u32);
    fn write_key(&mut self, index: usize, value: u32);
    fn write_buf(&mut self, index: usize, value: u32);
    fn read_txt(&mut self, index: usize) -> u32;
}

pub(crate) const fn validate_timeout_iterations(iterations: u32) -> Result<u32, AesError> {
    if iterations == 0 {
        Err(AesError::ZeroIterations)
    } else {
        Ok(iterations)
    }
}

/// Make sure the `LAES` clock runs. Returns `true` when it had to be
/// (re-)enabled, i.e. the core state must be re-validated.
pub(crate) fn restore_clock<R: AesRegisters>(
    registers: &mut R,
    iterations: u32,
) -> Result<bool, AesError> {
    if registers.clock_enabled() {
        return Ok(false);
    }
    registers.request_clock();
    for _ in 0..iterations {
        if registers.clock_enabled() {
            return Ok(true);
        }
    }
    Err(AesError::ClockUnavailable)
}

fn wait_idle<R: AesRegisters>(registers: &mut R, iterations: u32) -> bool {
    for _ in 0..iterations {
        if registers.read_sta() & STA_STATE_BUSY == 0 {
            return true;
        }
    }
    false
}

/// `AESCommonLPF3_cleanup()`: abort (clears `TXT`, `BUF`, `DMA`, `AUTOCFG`)
/// and clear every interrupt flag.
fn cleanup<R: AesRegisters>(registers: &mut R) {
    registers.write_abort(ABORT_ABORTAES);
    registers.write_iclr(ICLR_ALL);
}

/// Encrypt one block with the clock known to be running.
pub(crate) fn encrypt_clocked<R: AesRegisters>(
    registers: &mut R,
    key: &[u8; 16],
    input: &[u8; 16],
    iterations: u32,
) -> Result<[u8; 16], AesError> {
    if !wait_idle(registers, iterations) {
        return Err(AesError::Busy);
    }

    registers.write_iclr(ICLR_ALL);
    for (index, word) in key.chunks_exact(4).enumerate() {
        registers.write_key(
            index,
            u32::from_le_bytes([word[0], word[1], word[2], word[3]]),
        );
    }
    registers.write_autocfg(AUTOCFG_SINGLE_BLOCK);
    // BUF3 last: it schedules the operation.
    for (index, word) in input.chunks_exact(4).enumerate() {
        registers.write_buf(
            index,
            u32::from_le_bytes([word[0], word[1], word[2], word[3]]),
        );
    }

    let mut done = false;
    for _ in 0..iterations {
        if registers.read_ris() & RIS_AESDONE != 0 && registers.read_sta() & STA_STATE_BUSY == 0 {
            done = true;
            break;
        }
    }
    if !done {
        cleanup(registers);
        return Err(AesError::Timeout);
    }

    let mut output = [0u8; 16];
    for (index, word) in output.chunks_exact_mut(4).enumerate() {
        word.copy_from_slice(&registers.read_txt(index).to_le_bytes());
    }
    cleanup(registers);
    Ok(output)
}

/// Run the two-key known-answer self-test with the clock known to be
/// running.
pub(crate) fn self_test_clocked<R: AesRegisters>(
    registers: &mut R,
    iterations: u32,
) -> Result<(), AesError> {
    for (key, plaintext, ciphertext) in SELF_TEST_VECTORS.iter() {
        if encrypt_clocked(registers, key, plaintext, iterations)? != *ciphertext {
            return Err(AesError::KnownAnswerMismatch);
        }
    }
    Ok(())
}

/// Encrypt one block, re-validating the core first if its clock had to be
/// restored.
pub(crate) fn encrypt<R: AesRegisters>(
    registers: &mut R,
    key: &[u8; 16],
    input: &[u8; 16],
    iterations: u32,
) -> Result<[u8; 16], AesError> {
    if restore_clock(registers, iterations)? {
        self_test_clocked(registers, iterations)?;
    }
    encrypt_clocked(registers, key, input, iterations)
}

/// Enable the clock (if needed) and run the known-answer self-test.
pub(crate) fn self_test<R: AesRegisters>(
    registers: &mut R,
    iterations: u32,
) -> Result<(), AesError> {
    restore_clock(registers, iterations)?;
    self_test_clocked(registers, iterations)
}

/// FIPS-197 Appendix C.1 and Appendix B: two different keys, so the
/// self-test also proves the key is reloaded per block.
pub(crate) const SELF_TEST_VECTORS: [([u8; 16], [u8; 16], [u8; 16]); 2] = [
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
        [
            0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf,
            0x4f, 0x3c,
        ],
        [
            0x32, 0x43, 0xf6, 0xa8, 0x88, 0x5a, 0x30, 0x8d, 0x31, 0x31, 0x98, 0xa2, 0xe0, 0x37,
            0x07, 0x34,
        ],
        [
            0x39, 0x25, 0x84, 0x1d, 0x02, 0xdc, 0x09, 0xfb, 0xdc, 0x11, 0x85, 0x97, 0x19, 0x6a,
            0x0b, 0x32,
        ],
    ),
];

/// The `LAES` clock gate used by this engine.
pub const CLOCK: PeripheralClock = PeripheralClock::Laes;

/// Ownership token for the `LAES` core (from [`crate::Core`]).
pub struct Aes {
    _private: (),
}

impl Aes {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }
}

#[cfg(target_os = "none")]
struct HwRegisters;

#[cfg(target_os = "none")]
impl HwRegisters {
    #[inline(always)]
    fn read(address: u32) -> u32 {
        // SAFETY: fixed CC2340R52 LAES/CLKCTL registers owned through `Aes`.
        unsafe { (address as *const u32).read_volatile() }
    }

    #[inline(always)]
    fn write(address: u32, value: u32) {
        // SAFETY: fixed CC2340R52 LAES/CLKCTL registers owned through `Aes`.
        unsafe { (address as *mut u32).write_volatile(value) }
    }
}

#[cfg(target_os = "none")]
impl AesRegisters for HwRegisters {
    fn clock_enabled(&mut self) -> bool {
        Self::read(crate::clkctl::CLKCTL_CLKCFG0) & CLOCK.mask() != 0
    }

    fn request_clock(&mut self) {
        // Write-one-to-set; other gates are unaffected.
        Self::write(crate::clkctl::CLKCTL_CLKENSET0, CLOCK.mask());
    }

    fn read_sta(&mut self) -> u32 {
        Self::read(AES_STA)
    }

    fn read_ris(&mut self) -> u32 {
        Self::read(AES_RIS)
    }

    fn write_iclr(&mut self, value: u32) {
        Self::write(AES_ICLR, value);
    }

    fn write_abort(&mut self, value: u32) {
        Self::write(AES_ABORT, value);
    }

    fn write_autocfg(&mut self, value: u32) {
        Self::write(AES_AUTOCFG, value);
    }

    fn write_key(&mut self, index: usize, value: u32) {
        Self::write(AES_KEY0 + 4 * index as u32, value);
    }

    fn write_buf(&mut self, index: usize, value: u32) {
        Self::write(AES_BUF0 + 4 * index as u32, value);
    }

    fn read_txt(&mut self, index: usize) -> u32 {
        Self::read(AES_TXT0 + 4 * index as u32)
    }
}

/// Exclusive owner of the `LAES` AES-128 core.
#[cfg(target_os = "none")]
pub struct AesEngine {
    _token: Aes,
    iterations: u32,
}

#[cfg(target_os = "none")]
impl AesEngine {
    /// Poll budget for the clock gate, the idle check and one block
    /// (the core needs 23 cycles per block; each poll iteration is several
    /// bus reads at 48 MHz).
    pub const DEFAULT_TIMEOUT_ITERATIONS: u32 = 10_000;

    /// Consume the token, enable the `LAES` clock and check the core is
    /// idle. Call [`Self::self_test`] before using the engine for security.
    pub fn new(token: Aes, timeout_iterations: u32) -> Result<Self, AesError> {
        let iterations = validate_timeout_iterations(timeout_iterations)?;
        let mut registers = HwRegisters;
        restore_clock(&mut registers, iterations)?;
        if !wait_idle(&mut registers, iterations) {
            return Err(AesError::Busy);
        }
        Ok(Self {
            _token: token,
            iterations,
        })
    }

    /// Encrypt one standard AES-128 block in place. On error `block` is
    /// unchanged.
    pub fn encrypt_block(&mut self, key: &[u8; 16], block: &mut [u8; 16]) -> Result<(), AesError> {
        *block = encrypt(&mut HwRegisters, key, block, self.iterations)?;
        Ok(())
    }

    /// Two back-to-back FIPS-197 known answers under different keys.
    pub fn self_test(&mut self) -> Result<(), AesError> {
        self_test(&mut HwRegisters, self.iterations)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use aes::Aes128;
    use aes::cipher::generic_array::GenericArray;
    use aes::cipher::{BlockEncrypt, KeyInit};
    use std::vec::Vec;
    use zigbee_crypto::{Aes128Forward, AesKey, ForwardAesProvider};

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Event {
        Iclr(u32),
        Abort,
        Autocfg(u32),
        Key(usize, u32),
        Buf(usize, u32),
        Txt(usize),
    }

    /// In-memory model of the LAES core as `hw_aes.h` documents it
    /// (BUSHALT disabled: writes to `KEY`/`BUF` while busy are ignored,
    /// `TXT` reads while busy return zero).
    struct Model {
        events: Vec<Event>,
        key: [u32; 4],
        buf: [u32; 4],
        txt: [u32; 4],
        autocfg: u32,
        ris: u32,
        busy: bool,
        /// Status reads the next operation stays busy for; `None` never
        /// completes.
        latency: Option<u32>,
        remaining: Option<u32>,
        busy_at_entry: bool,
        clock_enabled: bool,
        clock_enable_works: bool,
        clock_requests: u32,
        operations: u32,
        corrupt_output: bool,
    }

    impl Model {
        fn new() -> Self {
            Self {
                events: Vec::new(),
                key: [0; 4],
                buf: [0; 4],
                txt: [0; 4],
                autocfg: 0,
                ris: 0,
                busy: false,
                latency: Some(2),
                remaining: None,
                busy_at_entry: false,
                clock_enabled: true,
                clock_enable_works: true,
                clock_requests: 0,
                operations: 0,
                corrupt_output: false,
            }
        }

        fn words_to_bytes(words: &[u32; 4]) -> [u8; 16] {
            let mut bytes = [0u8; 16];
            for (chunk, word) in bytes.chunks_exact_mut(4).zip(words) {
                chunk.copy_from_slice(&word.to_le_bytes());
            }
            bytes
        }

        fn start(&mut self) {
            self.operations += 1;
            self.busy = true;
            self.ris |= 0x2; // AESSTART
            self.remaining = self.latency;
        }

        fn tick(&mut self) {
            if !self.busy {
                return;
            }
            match self.remaining.as_mut() {
                None => {}
                Some(0) => self.finish(),
                Some(remaining) => *remaining -= 1,
            }
        }

        fn finish(&mut self) {
            let key = Self::words_to_bytes(&self.key);
            let mut block = GenericArray::clone_from_slice(&Self::words_to_bytes(&self.buf));
            Aes128::new(GenericArray::from_slice(&key)).encrypt_block(&mut block);
            for (word, chunk) in self.txt.iter_mut().zip(block.chunks_exact(4)) {
                *word = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            }
            if self.corrupt_output {
                self.txt = [0; 4];
            }
            self.busy = false;
            self.ris |= RIS_AESDONE;
        }
    }

    impl AesRegisters for Model {
        fn clock_enabled(&mut self) -> bool {
            self.clock_enabled
        }

        fn request_clock(&mut self) {
            self.clock_requests += 1;
            if self.clock_enable_works {
                self.clock_enabled = true;
            }
        }

        fn read_sta(&mut self) -> u32 {
            assert!(self.clock_enabled, "LAES accessed with its clock gated");
            if self.busy_at_entry {
                return STA_STATE_BUSY;
            }
            self.tick();
            u32::from(self.busy)
        }

        fn read_ris(&mut self) -> u32 {
            self.tick();
            self.ris
        }

        fn write_iclr(&mut self, value: u32) {
            self.events.push(Event::Iclr(value));
            self.ris &= !value;
        }

        fn write_abort(&mut self, value: u32) {
            assert_eq!(value, ABORT_ABORTAES);
            self.events.push(Event::Abort);
            self.busy = false;
            self.txt = [0; 4];
            self.buf = [0; 4];
            self.autocfg = 0;
        }

        fn write_autocfg(&mut self, value: u32) {
            self.events.push(Event::Autocfg(value));
            self.autocfg = value;
        }

        fn write_key(&mut self, index: usize, value: u32) {
            self.events.push(Event::Key(index, value));
            if !self.busy {
                self.key[index] = value;
            }
        }

        fn write_buf(&mut self, index: usize, value: u32) {
            self.events.push(Event::Buf(index, value));
            if self.busy {
                return;
            }
            self.buf[index] = value;
            if index == 3 && self.autocfg & AUTOCFG_TRGAES_WRBUF3S != 0 {
                assert_eq!(self.autocfg & 0x30, AUTOCFG_AESSRC_BUF);
                // Single-shot trigger: consumed by this write.
                self.autocfg &= !AUTOCFG_TRGAES_WRBUF3S;
                self.start();
            }
        }

        fn read_txt(&mut self, index: usize) -> u32 {
            self.events.push(Event::Txt(index));
            if self.busy { 0 } else { self.txt[index] }
        }
    }

    /// Host stand-in for `AesEngine` + the MAC's provider: the register
    /// model behind the production engine functions.
    struct ModelProvider {
        model: Model,
        iterations: u32,
    }

    struct ModelCipher<'a> {
        provider: &'a mut ModelProvider,
        key: AesKey,
    }

    impl Aes128Forward for ModelCipher<'_> {
        type Error = AesError;

        fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), AesError> {
            let provider = &mut *self.provider;
            *block = encrypt(&mut provider.model, &self.key, block, provider.iterations)?;
            Ok(())
        }
    }

    impl ForwardAesProvider for ModelProvider {
        fn forward_cipher(&mut self, key: &AesKey) -> impl Aes128Forward + '_ {
            ModelCipher {
                provider: self,
                key: *key,
            }
        }
    }

    fn count_operations(events: &[Event]) -> usize {
        events
            .iter()
            .filter(|event| matches!(event, Event::Buf(3, _)))
            .count()
    }

    #[test]
    fn register_map_matches_hw_aes() {
        assert_eq!(AES_ABORT, 0x400C_0014);
        assert_eq!(AES_STA, 0x400C_001C);
        assert_eq!(AES_AUTOCFG, 0x400C_002C);
        assert_eq!(AES_KEY0, 0x400C_0050);
        assert_eq!(AES_TXT0, 0x400C_0070);
        assert_eq!(AES_BUF0, 0x400C_0090);
        assert_eq!(AES_RIS, 0x400C_0108);
        assert_eq!(AES_ICLR, 0x400C_0114);
        assert_eq!(AUTOCFG_SINGLE_BLOCK, 0x28);
        assert_eq!(CLOCK.mask(), 1 << 16);
    }

    #[test]
    fn self_test_vectors_are_standard_aes() {
        for (key, plaintext, ciphertext) in SELF_TEST_VECTORS {
            let mut block = GenericArray::clone_from_slice(&plaintext);
            Aes128::new(GenericArray::from_slice(&key)).encrypt_block(&mut block);
            assert_eq!(block.as_slice(), ciphertext);
        }
    }

    #[test]
    fn register_sequence_and_byte_order_match_driverlib() {
        let (key, plaintext, ciphertext) = SELF_TEST_VECTORS[0];
        let mut model = Model::new();
        let output = encrypt(&mut model, &key, &plaintext, 8).unwrap();
        assert_eq!(output, ciphertext);
        assert_eq!(
            model.events,
            std::vec![
                Event::Iclr(ICLR_ALL),
                Event::Key(0, 0x0302_0100),
                Event::Key(1, 0x0706_0504),
                Event::Key(2, 0x0b0a_0908),
                Event::Key(3, 0x0f0e_0d0c),
                Event::Autocfg(0x28),
                Event::Buf(0, 0x3322_1100),
                Event::Buf(1, 0x7766_5544),
                Event::Buf(2, 0xbbaa_9988),
                Event::Buf(3, 0xffee_ddcc),
                Event::Txt(0),
                Event::Txt(1),
                Event::Txt(2),
                Event::Txt(3),
                Event::Abort,
                Event::Iclr(ICLR_ALL),
            ]
        );
        // ABORT leaves no plaintext or ciphertext in the core.
        assert_eq!((model.buf, model.txt, model.autocfg), ([0; 4], [0; 4], 0));
    }

    #[test]
    fn model_engine_passes_every_zigbee_known_answer() {
        let mut provider = ModelProvider {
            model: Model::new(),
            iterations: 8,
        };
        zigbee_crypto::kat::run_all(&mut provider).unwrap();
        assert!(provider.model.operations > 50);
        assert_eq!(provider.model.clock_requests, 0);
    }

    #[test]
    fn self_test_rekeys_between_vectors() {
        let mut model = Model::new();
        self_test(&mut model, 8).unwrap();
        assert_eq!(count_operations(&model.events), 2);
    }

    #[test]
    fn zero_poll_budget_is_rejected() {
        assert_eq!(
            validate_timeout_iterations(0),
            Err(AesError::ZeroIterations)
        );
        assert_eq!(validate_timeout_iterations(1), Ok(1));
    }

    #[test]
    fn regated_clock_is_restored_and_revalidated_before_output() {
        let (key, plaintext, ciphertext) = SELF_TEST_VECTORS[1];
        let mut model = Model::new();
        model.clock_enabled = false;
        assert_eq!(encrypt(&mut model, &key, &plaintext, 8), Ok(ciphertext));
        assert_eq!(model.clock_requests, 1);
        // Two self-test blocks, then the caller's block.
        assert_eq!(count_operations(&model.events), 3);
    }

    #[test]
    fn unavailable_clock_fails_closed_without_touching_the_core() {
        let mut model = Model::new();
        model.clock_enabled = false;
        model.clock_enable_works = false;
        assert_eq!(
            encrypt(&mut model, &[0; 16], &[0; 16], 4),
            Err(AesError::ClockUnavailable)
        );
        assert!(model.events.is_empty());
    }

    #[test]
    fn wrong_silicon_output_fails_the_revalidation() {
        let mut model = Model::new();
        model.clock_enabled = false;
        model.corrupt_output = true;
        assert_eq!(
            encrypt(&mut model, &[0; 16], &[0; 16], 8),
            Err(AesError::KnownAnswerMismatch)
        );
        let mut model = Model::new();
        model.corrupt_output = true;
        assert_eq!(self_test(&mut model, 8), Err(AesError::KnownAnswerMismatch));
    }

    #[test]
    fn busy_core_is_not_programmed() {
        let mut model = Model::new();
        model.busy_at_entry = true;
        assert_eq!(
            encrypt(&mut model, &[0; 16], &[0; 16], 4),
            Err(AesError::Busy)
        );
        assert!(model.events.is_empty());
    }

    #[test]
    fn timeout_aborts_and_publishes_nothing() {
        let mut model = Model::new();
        model.latency = None;
        assert_eq!(
            encrypt(&mut model, &[0x11; 16], &[0x22; 16], 4),
            Err(AesError::Timeout)
        );
        assert!(
            !model
                .events
                .iter()
                .any(|event| matches!(event, Event::Txt(_)))
        );
        assert_eq!(
            &model.events[model.events.len() - 2..],
            &[Event::Abort, Event::Iclr(ICLR_ALL)]
        );
        assert_eq!((model.buf, model.txt, model.busy), ([0; 4], [0; 4], false));
    }

    #[test]
    fn slow_completion_within_budget_succeeds_and_too_slow_fails() {
        let (key, plaintext, ciphertext) = SELF_TEST_VECTORS[0];
        let mut model = Model::new();
        model.latency = Some(5);
        assert_eq!(encrypt(&mut model, &key, &plaintext, 8), Ok(ciphertext));
        let mut model = Model::new();
        model.latency = Some(50);
        assert_eq!(
            encrypt(&mut model, &key, &plaintext, 8),
            Err(AesError::Timeout)
        );
    }

    #[test]
    fn provider_errors_leave_the_caller_block_unchanged() {
        let mut provider = ModelProvider {
            model: Model::new(),
            iterations: 4,
        };
        provider.model.latency = None;
        let mut block = [0xA5; 16];
        let failed = provider
            .forward_cipher(&[0; 16])
            .encrypt_block(&mut block)
            .is_err();
        assert!(failed);
        assert_eq!(block, [0xA5; 16]);
        assert!(zigbee_crypto::kat::run_all(&mut provider).is_err());
    }

    #[test]
    fn stale_done_flag_cannot_complete_a_new_operation() {
        // A completion flag left over from an earlier operation is cleared
        // before the new one is programmed.
        let mut model = Model::new();
        model.ris = RIS_AESDONE;
        model.latency = None;
        assert_eq!(
            encrypt(&mut model, &[0; 16], &[0; 16], 4),
            Err(AesError::Timeout)
        );
    }
}
