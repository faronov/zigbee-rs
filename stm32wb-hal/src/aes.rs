//! STM32WB AES1 forward AES-128 block engine for CPU1.
//!
//! # Which engine
//!
//! The STM32WB55 has two instances of the same AES peripheral (RM0434,
//! "AES hardware accelerator (AES)" chapter and the memory map):
//!
//! - **AES1** at `0x5006_0000` on AHB2, clocked for CPU1 by
//!   `RCC_AHB2ENR.AES1EN`, is the application (CPU1) engine. CPU2 only
//!   touches it on an explicit CPU1 request through the FUS customer key
//!   storage commands (`SHCI_C2_FUS_LoadUsrKey`/`UnloadUsrKey`), which load
//!   a stored key into AES1 in secure mode; zigbee-rs never issues them.
//! - **AES2** at `0x5800_1800` on AHB3 belongs to CPU2's wireless firmware
//!   (BLE / 802.15.4 link-layer security) and may become CPU2-secure. CPU1
//!   never touches it; this module has no path to it.
//!
//! ST's semaphore table (`hw_conf.h`: RNG 0, PKA 1, FLASH 2, RCC 3, stop
//! mode 4, CLK48 5, flash blocking 6/7, NVM SRAM 8/9) has no AES entry, so
//! AES1 needs no HSEM against the 802.15.4 MAC firmware. MAC-level security
//! on CPU2 is not used either: every MLME/MCPS request in [`crate::mac`]
//! carries `SecurityLevel::Unsecure`; zigbee-rs secures NWK/APS on CPU1.
//!
//! # Register sequence (ECB encrypt, 128-bit key)
//!
//! `CR` = `ERRC | CCFC` (EN = 0, clear stale flags); `CR` = configuration
//! (`MODE = 00` encryption, `CHMOD = 000` ECB, `KEYSIZE = 0` 128-bit,
//! `DATATYPE = 10` byte swap); `KEYR3..KEYR0` = key (big-endian words,
//! `KEYR3` holds key bytes 0..3, as in ST's HAL `CRYP_SetKey`); `EN = 1`;
//! four `DINR` writes; bounded poll of `SR.CCF` with `RDERR`/`WRERR`
//! checks; four `DOUTR` reads; `CCFC`; `EN = 0`.
//!
//! `DATATYPE = 10` swaps the bytes of each 32-bit word between the bus and
//! the core, so the little-endian Cortex-M word view of the standard AES
//! byte string (`u32::from_le_bytes`) reaches the core in standard order.
//! The key registers are not affected by `DATATYPE` and take big-endian
//! words. The host model below implements those RM semantics and must pass
//! `zigbee_crypto::kat::run_all`.
//!
//! # Failure policy
//!
//! Every wait is bounded; engine error flags, timeouts, an unavailable
//! clock and a known-answer mismatch return [`AesError`]; output is only
//! published after the engine reported a clean completion. There is no
//! software fall-back.
//!
//! If a customer key was ever loaded into AES1 in secure mode (FUS CKS),
//! CPU1 key writes do not take effect and the start-up known-answer test
//! fails, which stops Zigbee before any frame is secured.

/// AES1 register offsets (RM0434 AES register map).
#[cfg_attr(not(feature = "_chip"), allow(dead_code))]
mod reg {
    pub const CR: usize = 0x00;
    pub const SR: usize = 0x04;
    pub const DINR: usize = 0x08;
    pub const DOUTR: usize = 0x0C;
    /// `KEYR0`; `KEYRn` is at `KEYR0 + 4 * n` for n = 0..=3.
    pub const KEYR0: usize = 0x10;
}

const CR_EN: u32 = 1 << 0;
const CR_DATATYPE_SHIFT: u32 = 1;
const CR_DATATYPE_MASK: u32 = 0b11 << CR_DATATYPE_SHIFT;
/// `DATATYPE = 10`: byte swapping.
const CR_DATATYPE_BYTE: u32 = 0b10 << CR_DATATYPE_SHIFT;
const CR_MODE_MASK: u32 = 0b11 << 3;
/// `CHMOD[1:0]` (bits 5..6) and `CHMOD[2]` (bit 16).
const CR_CHMOD_MASK: u32 = (0b11 << 5) | (1 << 16);
const CR_CCFC: u32 = 1 << 7;
const CR_ERRC: u32 = 1 << 8;
const CR_KEYSIZE: u32 = 1 << 18;

/// ECB, encryption, 128-bit key, byte-swapped data; `EN` clear.
const CR_ECB_ENCRYPT_128: u32 = CR_DATATYPE_BYTE;
const _: () = assert!(CR_ECB_ENCRYPT_128 & (CR_MODE_MASK | CR_CHMOD_MASK | CR_KEYSIZE) == 0);
const _: () = assert!(CR_ECB_ENCRYPT_128 & CR_DATATYPE_MASK == CR_DATATYPE_BYTE);

const SR_CCF: u32 = 1 << 0;
const SR_RDERR: u32 = 1 << 1;
const SR_WRERR: u32 = 1 << 2;

/// Errors from the AES1 engine. Every error leaves the output untouched.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AesError {
    /// A zero poll budget would make every operation fail.
    ZeroIterations,
    /// The AES1 token was already taken.
    AlreadyTaken,
    /// `RCC_AHB2ENR.AES1EN` did not read back as set.
    ClockUnavailable,
    /// `SR.CCF` did not rise within the poll budget.
    Timeout,
    /// The engine reported a read (`RDERR`) or write (`WRERR`) error.
    EngineError,
    /// An on-silicon known answer did not match standard AES-128.
    KnownAnswerMismatch,
}

/// Register access used by the block sequence: the AES1 peripheral on the
/// chip, an RM-faithful model in host tests.
pub(crate) trait Aes1Registers {
    fn clock_enabled(&mut self) -> bool;
    fn enable_clock(&mut self);
    fn write_cr(&mut self, value: u32);
    fn read_sr(&mut self) -> u32;
    /// `index` 0..=3 selects `KEYR0..KEYR3`.
    fn write_keyr(&mut self, index: usize, value: u32);
    fn write_dinr(&mut self, value: u32);
    fn read_doutr(&mut self) -> u32;
}

const fn validate_timeout_iterations(timeout_iterations: u32) -> Result<u32, AesError> {
    if timeout_iterations == 0 {
        Err(AesError::ZeroIterations)
    } else {
        Ok(timeout_iterations)
    }
}

/// Enable the AES1 clock. Returns `true` if it had been off (the engine
/// state is then unknown and must be re-validated).
fn restore_clock<R: Aes1Registers>(registers: &mut R) -> Result<bool, AesError> {
    if registers.clock_enabled() {
        return Ok(false);
    }
    registers.enable_clock();
    if !registers.clock_enabled() {
        return Err(AesError::ClockUnavailable);
    }
    Ok(true)
}

/// Disable the engine and clear its flags.
fn abort<R: Aes1Registers>(registers: &mut R) {
    registers.write_cr(CR_ECB_ENCRYPT_128 | CR_CCFC | CR_ERRC);
    registers.write_cr(CR_ECB_ENCRYPT_128);
}

fn run_block_clocked<R: Aes1Registers>(
    registers: &mut R,
    key: &[u8; 16],
    input: &[u8; 16],
    output: &mut [u8; 16],
    timeout_iterations: u32,
) -> Result<(), AesError> {
    // Disabled engine, stale CCF/RDERR/WRERR cleared, then configuration.
    registers.write_cr(CR_CCFC | CR_ERRC);
    registers.write_cr(CR_ECB_ENCRYPT_128);
    // Key registers ignore DATATYPE: KEYR3 holds key bytes 0..3.
    for (index, chunk) in key.chunks_exact(4).enumerate() {
        registers.write_keyr(
            3 - index,
            u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]),
        );
    }
    registers.write_cr(CR_ECB_ENCRYPT_128 | CR_EN);
    for chunk in input.chunks_exact(4) {
        registers.write_dinr(u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]));
    }

    let mut complete = false;
    for _ in 0..timeout_iterations {
        let status = registers.read_sr();
        if status & (SR_RDERR | SR_WRERR) != 0 {
            abort(registers);
            return Err(AesError::EngineError);
        }
        if status & SR_CCF != 0 {
            complete = true;
            break;
        }
    }
    if !complete {
        abort(registers);
        return Err(AesError::Timeout);
    }

    let mut words = [0u32; 4];
    for word in &mut words {
        *word = registers.read_doutr();
    }
    registers.write_cr(CR_ECB_ENCRYPT_128 | CR_EN | CR_CCFC);
    let status = registers.read_sr();
    registers.write_cr(CR_ECB_ENCRYPT_128);
    if status & (SR_RDERR | SR_WRERR) != 0 {
        abort(registers);
        return Err(AesError::EngineError);
    }
    for (chunk, word) in output.chunks_exact_mut(4).zip(words) {
        chunk.copy_from_slice(&word.to_le_bytes());
    }
    Ok(())
}

fn self_test_clocked<R: Aes1Registers>(
    registers: &mut R,
    timeout_iterations: u32,
) -> Result<(), AesError> {
    for (key, plaintext, ciphertext) in SELF_TEST_VECTORS.iter() {
        let mut output = [0u8; 16];
        run_block_clocked(registers, key, plaintext, &mut output, timeout_iterations)?;
        if &output != ciphertext {
            return Err(AesError::KnownAnswerMismatch);
        }
    }
    Ok(())
}

fn run_block<R: Aes1Registers>(
    registers: &mut R,
    key: &[u8; 16],
    input: &[u8; 16],
    output: &mut [u8; 16],
    timeout_iterations: u32,
) -> Result<(), AesError> {
    if restore_clock(registers)? {
        // The clock was gated behind the stack's back: re-validate the
        // engine before it secures another frame.
        self_test_clocked(registers, timeout_iterations)?;
    }
    run_block_clocked(registers, key, input, output, timeout_iterations)
}

fn self_test_with<R: Aes1Registers>(
    registers: &mut R,
    timeout_iterations: u32,
) -> Result<(), AesError> {
    restore_clock(registers)?;
    self_test_clocked(registers, timeout_iterations)
}

/// FIPS-197 C.1 and the all-zero key/plaintext block (two different keys,
/// so a stuck key register fails the test).
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

#[cfg(feature = "_chip")]
pub use chip::{Aes1, AesEngine, HardwareAes128};

#[cfg(feature = "_chip")]
mod chip {
    use core::sync::atomic::{AtomicBool, Ordering};

    use embassy_stm32::pac;
    use zigbee_crypto::{Aes128Forward, AesKey};

    use super::{AesError, reg};

    /// AES1 base address (RM0434 memory map; CMSIS `AES1_BASE`).
    const AES1_BASE: usize = 0x5006_0000;

    static AES1_TAKEN: AtomicBool = AtomicBool::new(false);

    /// Ownership token for the AES1 peripheral. `embassy-stm32` has no
    /// singleton for it on STM32WB (no register block in its metadata), so
    /// this crate hands out exactly one.
    pub struct Aes1 {
        _private: (),
    }

    impl Aes1 {
        /// The AES1 token, once.
        pub fn take() -> Option<Self> {
            if AES1_TAKEN.swap(true, Ordering::AcqRel) {
                None
            } else {
                Some(Self { _private: () })
            }
        }
    }

    struct HwRegisters;

    impl HwRegisters {
        #[inline(always)]
        fn read(offset: usize) -> u32 {
            // SAFETY: AES1 is owned through the `Aes1` token; offsets are
            // registers of RM0434's AES register map.
            unsafe { core::ptr::read_volatile((AES1_BASE + offset) as *const u32) }
        }

        #[inline(always)]
        fn write(offset: usize, value: u32) {
            // SAFETY: as in `read`.
            unsafe { core::ptr::write_volatile((AES1_BASE + offset) as *mut u32, value) }
        }
    }

    impl super::Aes1Registers for HwRegisters {
        fn clock_enabled(&mut self) -> bool {
            pac::RCC.ahb2enr().read().aes1en()
        }

        fn enable_clock(&mut self) {
            // AHB2ENR is CPU1's enable register (CPU2 uses C2AHB2ENR).
            pac::RCC.ahb2enr().modify(|w| w.set_aes1en(true));
        }

        fn write_cr(&mut self, value: u32) {
            Self::write(reg::CR, value)
        }

        fn read_sr(&mut self) -> u32 {
            Self::read(reg::SR)
        }

        fn write_keyr(&mut self, index: usize, value: u32) {
            Self::write(reg::KEYR0 + 4 * index, value)
        }

        fn write_dinr(&mut self, value: u32) {
            Self::write(reg::DINR, value)
        }

        fn read_doutr(&mut self) -> u32 {
            Self::read(reg::DOUTR)
        }
    }

    /// The CPU1 AES1 engine: standard AES-128 forward blocks for Zigbee
    /// CCM* and AES-MMO.
    pub struct AesEngine {
        _token: Aes1,
        timeout_iterations: u32,
    }

    impl AesEngine {
        /// Bound on one `SR.CCF` poll. An ECB block completes within a few
        /// hundred AES clock cycles; 10 000 status reads take on the order
        /// of a millisecond at the routers' 64 MHz HCLK, so only a dead
        /// engine reaches the bound.
        pub const DEFAULT_TIMEOUT_ITERATIONS: u32 = 10_000;

        /// Take ownership of AES1 and enable its clock. Run
        /// [`self_test`](Self::self_test) before relying on it.
        pub fn new(token: Aes1, timeout_iterations: u32) -> Result<Self, AesError> {
            let timeout_iterations = super::validate_timeout_iterations(timeout_iterations)?;
            super::restore_clock(&mut HwRegisters)?;
            Ok(Self {
                _token: token,
                timeout_iterations,
            })
        }

        /// [`Aes1::take`] and [`new`](Self::new) with the default bound.
        pub fn take() -> Result<Self, AesError> {
            let token = Aes1::take().ok_or(AesError::AlreadyTaken)?;
            Self::new(token, Self::DEFAULT_TIMEOUT_ITERATIONS)
        }

        /// Encrypt one AES-128 block in place. On error `block` is
        /// unchanged.
        #[inline(never)]
        pub fn encrypt_block(
            &mut self,
            key: &[u8; 16],
            block: &mut [u8; 16],
        ) -> Result<(), AesError> {
            let input = *block;
            super::run_block(
                &mut HwRegisters,
                key,
                &input,
                block,
                self.timeout_iterations,
            )
        }

        /// Two back-to-back FIPS-197 known answers with different keys.
        pub fn self_test(&mut self) -> Result<(), AesError> {
            super::self_test_with(&mut HwRegisters, self.timeout_iterations)
        }

        /// The full `zigbee_crypto::kat` suite (AES blocks, IEEE 802.15.4
        /// and Zigbee CCM*, install-code AES-MMO) on the silicon. Lab
        /// builds only: it links the vectors and a reference CCM*.
        #[cfg(feature = "kat")]
        pub fn known_answer_suite(&mut self) -> Result<(), zigbee_crypto::kat::KatFailure> {
            zigbee_crypto::kat::run_all(&mut EngineProvider(self))
        }
    }

    #[cfg(feature = "kat")]
    struct EngineProvider<'a>(&'a mut AesEngine);

    #[cfg(feature = "kat")]
    impl zigbee_crypto::ForwardAesProvider for EngineProvider<'_> {
        fn forward_cipher(&mut self, key: &AesKey) -> impl Aes128Forward + '_ {
            HardwareAes128::new(self.0, *key)
        }
    }

    /// Keyed AES-128 forward permutation borrowing the AES1 engine; what
    /// the STM32WB MAC transport hands to CCM* and AES-MMO.
    pub struct HardwareAes128<'engine> {
        engine: &'engine mut AesEngine,
        key: AesKey,
    }

    impl<'engine> HardwareAes128<'engine> {
        pub fn new(engine: &'engine mut AesEngine, key: AesKey) -> Self {
            Self { engine, key }
        }
    }

    impl Aes128Forward for HardwareAes128<'_> {
        type Error = AesError;

        #[inline(always)]
        fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), Self::Error> {
            self.engine.encrypt_block(&self.key, block)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::vec::Vec;

    use zigbee_crypto::{Aes128Forward, AesKey, ForwardAesProvider, SoftwareAes128};

    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Event {
        Cr(u32),
        Key(usize, u32),
        Din(u32),
        Dout,
    }

    /// AES1 as RM0434 describes it: keys only writable with `EN = 0`,
    /// `DATATYPE` swapping on `DINR`/`DOUTR` only, `KEYR3` the most
    /// significant key word, `CCF` after four `DINR` writes with `EN = 1`,
    /// `WRERR` on a `DINR` write during computation, `RDERR` on a `DOUTR`
    /// read before `CCF`, `CCFC`/`ERRC` write-1-to-clear.
    struct Model {
        events: Vec<Event>,
        cr: u32,
        keyr: [u32; 4],
        din: Vec<u32>,
        dout: [u32; 4],
        dout_index: usize,
        ccf: bool,
        rderr: bool,
        wrerr: bool,
        /// Status reads before `CCF` rises; `None` = never.
        busy_reads: Option<u32>,
        busy_left: u32,
        clock_enabled: bool,
        clock_enable_succeeds: bool,
        clock_enable_calls: u32,
        /// A dead core: produces zeros.
        dead: bool,
        /// Raise `WRERR` when the computation starts.
        inject_wrerr: bool,
        blocks: u32,
    }

    impl Model {
        fn new() -> Self {
            Self {
                events: Vec::new(),
                cr: 0,
                keyr: [0; 4],
                din: Vec::new(),
                dout: [0; 4],
                dout_index: 0,
                ccf: false,
                rderr: false,
                wrerr: false,
                busy_reads: Some(2),
                busy_left: 0,
                clock_enabled: true,
                clock_enable_succeeds: true,
                clock_enable_calls: 0,
                dead: false,
                inject_wrerr: false,
                blocks: 0,
            }
        }

        fn swap(&self, word: u32) -> u32 {
            match (self.cr & CR_DATATYPE_MASK) >> CR_DATATYPE_SHIFT {
                0b00 => word,
                0b01 => word.rotate_left(16),
                0b10 => word.swap_bytes(),
                _ => word.reverse_bits(),
            }
        }

        fn compute(&mut self) {
            assert_eq!(self.cr & CR_MODE_MASK, 0, "MODE must be encryption");
            assert_eq!(self.cr & CR_CHMOD_MASK, 0, "CHMOD must be ECB");
            assert_eq!(self.cr & CR_KEYSIZE, 0, "KEYSIZE must be 128-bit");
            let mut key = [0u8; 16];
            for (index, word) in self.keyr.iter().rev().enumerate() {
                key[index * 4..index * 4 + 4].copy_from_slice(&word.to_be_bytes());
            }
            let mut block = [0u8; 16];
            for (index, word) in self.din.iter().enumerate() {
                let core = self.swap(*word);
                block[index * 4..index * 4 + 4].copy_from_slice(&core.to_be_bytes());
            }
            SoftwareAes128::new(&key).encrypt_block(&mut block).unwrap();
            if self.dead {
                block = [0; 16];
            }
            for (index, chunk) in block.chunks_exact(4).enumerate() {
                let core = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                self.dout[index] = self.swap(core);
            }
            self.dout_index = 0;
            self.din.clear();
            self.busy_left = self.busy_reads.unwrap_or(u32::MAX);
            self.blocks += 1;
            if self.inject_wrerr {
                self.wrerr = true;
            }
        }

        fn computing(&self) -> bool {
            self.din.is_empty() && self.busy_left > 0 && !self.ccf && self.blocks > 0
        }

        fn cmd_count(&self) -> usize {
            self.blocks as usize
        }
    }

    impl Aes1Registers for Model {
        fn clock_enabled(&mut self) -> bool {
            self.clock_enabled
        }

        fn enable_clock(&mut self) {
            self.clock_enable_calls += 1;
            if self.clock_enable_succeeds {
                self.clock_enabled = true;
            }
        }

        fn write_cr(&mut self, value: u32) {
            assert!(self.clock_enabled, "register access with the clock off");
            self.events.push(Event::Cr(value));
            if value & CR_CCFC != 0 {
                self.ccf = false;
            }
            if value & CR_ERRC != 0 {
                self.rderr = false;
                self.wrerr = false;
            }
            let stored = value & !(CR_CCFC | CR_ERRC);
            if stored & CR_EN == 0 {
                self.din.clear();
                self.busy_left = 0;
            }
            self.cr = stored;
        }

        fn read_sr(&mut self) -> u32 {
            assert!(self.clock_enabled);
            if self.cr & CR_EN != 0 && !self.ccf && self.busy_left > 0 && self.din.is_empty() {
                if self.busy_left == u32::MAX {
                    // Never completes.
                } else {
                    self.busy_left -= 1;
                    if self.busy_left == 0 {
                        self.ccf = true;
                    }
                }
            }
            (u32::from(self.ccf) * SR_CCF)
                | (u32::from(self.rderr) * SR_RDERR)
                | (u32::from(self.wrerr) * SR_WRERR)
        }

        fn write_keyr(&mut self, index: usize, value: u32) {
            assert!(self.clock_enabled);
            assert_eq!(self.cr & CR_EN, 0, "key written with EN = 1");
            self.events.push(Event::Key(index, value));
            self.keyr[index] = value;
        }

        fn write_dinr(&mut self, value: u32) {
            assert!(self.clock_enabled);
            self.events.push(Event::Din(value));
            if self.cr & CR_EN == 0 || self.computing() {
                self.wrerr = true;
                return;
            }
            self.din.push(value);
            if self.din.len() == 4 {
                self.compute();
            }
        }

        fn read_doutr(&mut self) -> u32 {
            assert!(self.clock_enabled);
            self.events.push(Event::Dout);
            if !self.ccf {
                self.rderr = true;
                return 0;
            }
            let value = self.dout[self.dout_index % 4];
            self.dout_index += 1;
            value
        }
    }

    /// The register model behind the generic provider interface, as the
    /// MAC transport exposes the engine on the chip.
    struct ModelProvider {
        model: Model,
        timeout: u32,
    }

    struct ModelCipher<'a> {
        provider: &'a mut ModelProvider,
        key: AesKey,
    }

    impl Aes128Forward for ModelCipher<'_> {
        type Error = AesError;

        fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), AesError> {
            let input = *block;
            run_block(
                &mut self.provider.model,
                &self.key,
                &input,
                block,
                self.provider.timeout,
            )
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

    const FIPS_KEY: [u8; 16] = SELF_TEST_VECTORS[0].0;
    const FIPS_PT: [u8; 16] = SELF_TEST_VECTORS[0].1;
    const FIPS_CT: [u8; 16] = SELF_TEST_VECTORS[0].2;

    #[test]
    fn model_passes_the_full_kat_suite() {
        let mut provider = ModelProvider {
            model: Model::new(),
            timeout: 8,
        };
        zigbee_crypto::kat::run_all(&mut provider).unwrap();
        assert!(provider.model.blocks > 10);
    }

    #[test]
    fn register_sequence_matches_rm0434_ecb_encrypt() {
        let mut model = Model::new();
        let mut output = [0u8; 16];
        run_block(&mut model, &FIPS_KEY, &FIPS_PT, &mut output, 8).unwrap();
        assert_eq!(output, FIPS_CT);
        assert_eq!(
            model.events,
            std::vec![
                Event::Cr(CR_CCFC | CR_ERRC),
                Event::Cr(CR_DATATYPE_BYTE),
                Event::Key(3, 0x0001_0203),
                Event::Key(2, 0x0405_0607),
                Event::Key(1, 0x0809_0a0b),
                Event::Key(0, 0x0c0d_0e0f),
                Event::Cr(CR_DATATYPE_BYTE | CR_EN),
                Event::Din(0x3322_1100),
                Event::Din(0x7766_5544),
                Event::Din(0xbbaa_9988),
                Event::Din(0xffee_ddcc),
                Event::Dout,
                Event::Dout,
                Event::Dout,
                Event::Dout,
                Event::Cr(CR_DATATYPE_BYTE | CR_EN | CR_CCFC),
                Event::Cr(CR_DATATYPE_BYTE),
            ]
        );
        assert_eq!(model.cr & CR_EN, 0, "engine left enabled");
        assert!(!model.ccf);
    }

    /// Why `DATATYPE = 10`: with no swap the same little-endian word view
    /// reaches the core byte-reversed per word and the result is not AES.
    #[test]
    fn standard_byte_order_needs_the_byte_swap() {
        for datatype in [0b00u32, 0b01, 0b11] {
            let mut model = Model::new();
            model.write_cr(datatype << CR_DATATYPE_SHIFT);
            for (index, chunk) in FIPS_KEY.chunks_exact(4).enumerate() {
                model.write_keyr(3 - index, u32::from_be_bytes(chunk.try_into().unwrap()));
            }
            model.write_cr((datatype << CR_DATATYPE_SHIFT) | CR_EN);
            for chunk in FIPS_PT.chunks_exact(4) {
                model.write_dinr(u32::from_le_bytes(chunk.try_into().unwrap()));
            }
            while model.read_sr() & SR_CCF == 0 {}
            let mut output = [0u8; 16];
            for chunk in output.chunks_exact_mut(4) {
                chunk.copy_from_slice(&model.read_doutr().to_le_bytes());
            }
            assert_ne!(output, FIPS_CT, "DATATYPE {datatype:#04b}");
        }
    }

    #[test]
    fn self_test_uses_two_keys() {
        let mut model = Model::new();
        self_test_with(&mut model, 8).unwrap();
        assert_eq!(model.cmd_count(), 2);
    }

    #[test]
    fn rejects_zero_timeout() {
        assert_eq!(
            validate_timeout_iterations(0),
            Err(AesError::ZeroIterations)
        );
        assert_eq!(validate_timeout_iterations(1), Ok(1));
    }

    #[test]
    fn zero_poll_budget_never_publishes() {
        let mut model = Model::new();
        let mut output = [0xA5; 16];
        assert_eq!(
            run_block(&mut model, &FIPS_KEY, &FIPS_PT, &mut output, 0),
            Err(AesError::Timeout)
        );
        assert_eq!(output, [0xA5; 16]);
    }

    #[test]
    fn timeout_is_fail_closed_and_disables_the_engine() {
        let mut model = Model::new();
        model.busy_reads = None;
        let mut output = [0x5A; 16];
        assert_eq!(
            run_block(&mut model, &FIPS_KEY, &FIPS_PT, &mut output, 4),
            Err(AesError::Timeout)
        );
        assert_eq!(output, [0x5A; 16]);
        assert!(!model.events.contains(&Event::Dout));
        assert_eq!(model.cr & CR_EN, 0);
    }

    #[test]
    fn completion_on_the_last_poll_is_accepted() {
        let mut model = Model::new();
        model.busy_reads = Some(4);
        let mut output = [0u8; 16];
        run_block(&mut model, &FIPS_KEY, &FIPS_PT, &mut output, 4).unwrap();
        assert_eq!(output, FIPS_CT);
    }

    #[test]
    fn write_error_flag_is_fail_closed() {
        let mut model = Model::new();
        model.inject_wrerr = true;
        let mut output = [0x5A; 16];
        assert_eq!(
            run_block(&mut model, &FIPS_KEY, &FIPS_PT, &mut output, 8),
            Err(AesError::EngineError)
        );
        assert_eq!(output, [0x5A; 16]);
        assert_eq!(model.cr & CR_EN, 0);
        assert!(!model.wrerr, "error flag left set");
    }

    #[test]
    fn read_error_after_completion_is_fail_closed() {
        struct LateReadError(Model);
        impl Aes1Registers for LateReadError {
            fn clock_enabled(&mut self) -> bool {
                self.0.clock_enabled()
            }
            fn enable_clock(&mut self) {
                self.0.enable_clock()
            }
            fn write_cr(&mut self, value: u32) {
                self.0.write_cr(value)
            }
            fn read_sr(&mut self) -> u32 {
                self.0.read_sr()
            }
            fn write_keyr(&mut self, index: usize, value: u32) {
                self.0.write_keyr(index, value)
            }
            fn write_dinr(&mut self, value: u32) {
                self.0.write_dinr(value)
            }
            fn read_doutr(&mut self) -> u32 {
                // A DOUTR read the engine flagged (RDERR).
                self.0.rderr = true;
                self.0.read_doutr()
            }
        }
        let mut registers = LateReadError(Model::new());
        let mut output = [0x77; 16];
        assert_eq!(
            run_block(&mut registers, &FIPS_KEY, &FIPS_PT, &mut output, 8),
            Err(AesError::EngineError)
        );
        assert_eq!(output, [0x77; 16]);
        assert_eq!(registers.0.cr & CR_EN, 0);
    }

    #[test]
    fn stale_flags_are_cleared_before_a_block() {
        let mut model = Model::new();
        model.rderr = true;
        model.wrerr = true;
        model.ccf = true;
        let mut output = [0u8; 16];
        run_block(&mut model, &FIPS_KEY, &FIPS_PT, &mut output, 8).unwrap();
        assert_eq!(output, FIPS_CT);
    }

    #[test]
    fn unavailable_clock_is_fail_closed() {
        let mut model = Model::new();
        model.clock_enabled = false;
        model.clock_enable_succeeds = false;
        let mut output = [0xA5; 16];
        assert_eq!(
            run_block(&mut model, &FIPS_KEY, &FIPS_PT, &mut output, 8),
            Err(AesError::ClockUnavailable)
        );
        assert_eq!(output, [0xA5; 16]);
        assert!(model.events.is_empty());
    }

    #[test]
    fn clock_restore_revalidates_before_encrypting() {
        let mut model = Model::new();
        model.clock_enabled = false;
        let mut output = [0u8; 16];
        run_block(&mut model, &FIPS_KEY, &FIPS_PT, &mut output, 8).unwrap();
        assert_eq!(output, FIPS_CT);
        assert_eq!(model.clock_enable_calls, 1);
        assert_eq!(model.cmd_count(), 3);
    }

    #[test]
    fn restored_dead_engine_is_fail_closed() {
        let mut model = Model::new();
        model.clock_enabled = false;
        model.dead = true;
        let mut output = [0x5A; 16];
        assert_eq!(
            run_block(&mut model, &FIPS_KEY, &FIPS_PT, &mut output, 8),
            Err(AesError::KnownAnswerMismatch)
        );
        assert_eq!(output, [0x5A; 16]);
    }

    #[test]
    fn dead_engine_fails_the_self_test() {
        let mut model = Model::new();
        model.dead = true;
        assert_eq!(
            self_test_with(&mut model, 8),
            Err(AesError::KnownAnswerMismatch)
        );
    }

    /// A key that never reaches the core (AES1 key registers held by a FUS
    /// customer key in secure mode) fails the two-key self-test.
    #[test]
    fn ignored_key_writes_fail_the_self_test() {
        struct LockedKey(Model);
        impl Aes1Registers for LockedKey {
            fn clock_enabled(&mut self) -> bool {
                self.0.clock_enabled()
            }
            fn enable_clock(&mut self) {
                self.0.enable_clock()
            }
            fn write_cr(&mut self, value: u32) {
                self.0.write_cr(value)
            }
            fn read_sr(&mut self) -> u32 {
                self.0.read_sr()
            }
            fn write_keyr(&mut self, _index: usize, _value: u32) {}
            fn write_dinr(&mut self, value: u32) {
                self.0.write_dinr(value)
            }
            fn read_doutr(&mut self) -> u32 {
                self.0.read_doutr()
            }
        }
        let mut model = Model::new();
        model.keyr = [0x1111_1111, 0x2222_2222, 0x3333_3333, 0x4444_4444];
        assert_eq!(
            self_test_with(&mut LockedKey(model), 8),
            Err(AesError::KnownAnswerMismatch)
        );
    }
}
