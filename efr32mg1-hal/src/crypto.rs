//! EFR32MG1 CRYPTO AES-128 forward block accelerator.
//!
//! The register sequence follows GSDK 4.5 `CRYPTO_AES_ECB128`: enable the
//! CRYPTO HFBUS clock, select AES-128, load `KEYBUF`, load the
//! `AESENC; DATA0TODATA1` sequence, write one DATA0 block, start the sequence,
//! wait with a bounded poll, and read DATA1. Words use the Cortex-M
//! little-endian memory view used by EMLIB's unaligned helpers.
//!
//! Only the forward AES permutation is exposed because Zigbee CCM* and
//! AES-MMO require encryption, not ECB decryption. Failures never fall back to
//! software AES.

#[cfg(target_arch = "arm")]
use crate::clock;

#[cfg(target_arch = "arm")]
const CRYPTO_BASE: usize = 0x400F_0000;
#[cfg(target_arch = "arm")]
const REG_CTRL: usize = CRYPTO_BASE;
#[cfg(target_arch = "arm")]
const REG_WAC: usize = CRYPTO_BASE + 0x04;
#[cfg(target_arch = "arm")]
const REG_CMD: usize = CRYPTO_BASE + 0x08;
#[cfg(target_arch = "arm")]
const REG_STATUS: usize = CRYPTO_BASE + 0x10;
#[cfg(target_arch = "arm")]
const REG_KEYBUF: usize = CRYPTO_BASE + 0x24;
#[cfg(target_arch = "arm")]
const REG_SEQCTRL: usize = CRYPTO_BASE + 0x30;
#[cfg(target_arch = "arm")]
const REG_SEQ0: usize = CRYPTO_BASE + 0x50;
#[cfg(target_arch = "arm")]
const REG_DATA0: usize = CRYPTO_BASE + 0x80;
#[cfg(target_arch = "arm")]
const REG_DATA1: usize = CRYPTO_BASE + 0x84;

#[cfg(any(target_arch = "arm", test))]
const STATUS_BUSY: u32 = (1 << 0) | (1 << 1);
#[cfg(any(target_arch = "arm", test))]
const CTRL_AES128: u32 = 0;
#[cfg(any(target_arch = "arm", test))]
const WAC_DEFAULT: u32 = 0;
#[cfg(any(target_arch = "arm", test))]
const SEQCTRL_BLOCK_BYTES: u32 = 16;
#[cfg(any(target_arch = "arm", test))]
const SEQ0_AESENC_DATA0_TO_DATA1: u32 = 0x0000_4405;
#[cfg(any(target_arch = "arm", test))]
const CMD_SEQSTART: u32 = 1 << 9;

/// Errors from the hardware AES-128 accelerator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AesError {
    /// A zero poll budget would make every hardware operation fail
    /// immediately.
    ZeroIterations,
    /// The CRYPTO HFBUS clock could not be enabled.
    ClockUnavailable,
    /// CRYPTO remained busy beyond the configured bounded wait.
    Timeout,
    /// An on-silicon known-answer result did not match standard AES-128.
    KnownAnswerMismatch,
}

#[cfg(any(target_arch = "arm", test))]
trait CryptoRegisters {
    fn crypto_clock_enabled(&mut self) -> bool {
        true
    }
    fn enable_crypto_clock(&mut self) {}
    fn read_status(&mut self) -> u32;
    fn write_ctrl(&mut self, value: u32);
    fn write_wac(&mut self, value: u32);
    fn write_keybuf(&mut self, value: u32);
    fn write_seq0(&mut self, value: u32);
    fn write_seqctrl(&mut self, value: u32);
    fn write_data0(&mut self, value: u32);
    fn write_cmd(&mut self, value: u32);
    fn read_data1(&mut self) -> u32;
}

#[cfg(target_arch = "arm")]
struct HwRegisters;

#[cfg(target_arch = "arm")]
impl HwRegisters {
    #[inline]
    unsafe fn read(address: usize) -> u32 {
        unsafe { core::ptr::read_volatile(address as *const u32) }
    }

    #[inline]
    unsafe fn write(address: usize, value: u32) {
        unsafe { core::ptr::write_volatile(address as *mut u32, value) }
    }
}

#[cfg(target_arch = "arm")]
impl CryptoRegisters for HwRegisters {
    fn crypto_clock_enabled(&mut self) -> bool {
        clock::crypto_clock_enabled()
    }

    fn enable_crypto_clock(&mut self) {
        clock::enable_crypto_clock();
    }

    fn read_status(&mut self) -> u32 {
        unsafe { Self::read(REG_STATUS) }
    }

    fn write_ctrl(&mut self, value: u32) {
        unsafe { Self::write(REG_CTRL, value) }
    }

    fn write_wac(&mut self, value: u32) {
        unsafe { Self::write(REG_WAC, value) }
    }

    fn write_keybuf(&mut self, value: u32) {
        unsafe { Self::write(REG_KEYBUF, value) }
    }

    fn write_seq0(&mut self, value: u32) {
        unsafe { Self::write(REG_SEQ0, value) }
    }

    fn write_seqctrl(&mut self, value: u32) {
        unsafe { Self::write(REG_SEQCTRL, value) }
    }

    fn write_data0(&mut self, value: u32) {
        unsafe { Self::write(REG_DATA0, value) }
    }

    fn write_cmd(&mut self, value: u32) {
        unsafe { Self::write(REG_CMD, value) }
    }

    fn read_data1(&mut self) -> u32 {
        unsafe { Self::read(REG_DATA1) }
    }
}

#[cfg(any(target_arch = "arm", test))]
const fn validate_timeout_iterations(timeout_iterations: u32) -> Result<u32, AesError> {
    if timeout_iterations == 0 {
        Err(AesError::ZeroIterations)
    } else {
        Ok(timeout_iterations)
    }
}

#[cfg(any(target_arch = "arm", test))]
fn wait_idle<R: CryptoRegisters>(
    registers: &mut R,
    timeout_iterations: u32,
) -> Result<(), AesError> {
    for _ in 0..timeout_iterations {
        if registers.read_status() & STATUS_BUSY == 0 {
            return Ok(());
        }
    }
    Err(AesError::Timeout)
}

#[cfg(any(target_arch = "arm", test))]
fn write_block_words(mut write_word: impl FnMut(u32), block: &[u8; 16]) {
    for chunk in block.chunks_exact(4) {
        write_word(u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]));
    }
}

#[cfg(any(target_arch = "arm", test))]
fn restore_clock<R: CryptoRegisters>(registers: &mut R) -> Result<bool, AesError> {
    let was_enabled = registers.crypto_clock_enabled();
    registers.enable_crypto_clock();
    if !registers.crypto_clock_enabled() {
        return Err(AesError::ClockUnavailable);
    }
    Ok(!was_enabled)
}

#[cfg(any(target_arch = "arm", test))]
fn run_block_clocked<R: CryptoRegisters>(
    registers: &mut R,
    key: &[u8; 16],
    input: &[u8; 16],
    output: &mut [u8; 16],
    timeout_iterations: u32,
) -> Result<(), AesError> {
    wait_idle(registers, timeout_iterations)?;

    registers.write_ctrl(CTRL_AES128);
    registers.write_wac(WAC_DEFAULT);
    write_block_words(|word| registers.write_keybuf(word), key);
    registers.write_seq0(SEQ0_AESENC_DATA0_TO_DATA1);
    registers.write_seqctrl(SEQCTRL_BLOCK_BYTES);
    write_block_words(|word| registers.write_data0(word), input);
    registers.write_cmd(CMD_SEQSTART);

    wait_idle(registers, timeout_iterations)?;
    for chunk in output.chunks_exact_mut(4) {
        chunk.copy_from_slice(&registers.read_data1().to_le_bytes());
    }
    Ok(())
}

#[cfg(any(target_arch = "arm", test))]
fn self_test_clocked<R: CryptoRegisters>(
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

#[cfg(any(target_arch = "arm", test))]
fn run_block<R: CryptoRegisters>(
    registers: &mut R,
    key: &[u8; 16],
    input: &[u8; 16],
    output: &mut [u8; 16],
    timeout_iterations: u32,
) -> Result<(), AesError> {
    if restore_clock(registers)? {
        // Resident bootloader services may gate or repurpose CRYPTO. Validate
        // the accelerator before publishing security output after a restore.
        self_test_clocked(registers, timeout_iterations)?;
    }
    run_block_clocked(registers, key, input, output, timeout_iterations)
}

#[cfg(any(target_arch = "arm", test))]
fn self_test_with<R: CryptoRegisters>(
    registers: &mut R,
    timeout_iterations: u32,
) -> Result<(), AesError> {
    restore_clock(registers)?;
    self_test_clocked(registers, timeout_iterations)
}

/// Owns the EFR32MG1 CRYPTO AES-128 accelerator.
///
/// The hardware-driving API is only available on ARM targets. Host tests
/// execute the same register protocol against an in-memory peripheral model.
#[cfg(target_arch = "arm")]
pub struct AesEngine {
    _peripheral: crate::peripherals::Crypto,
    timeout_iterations: u32,
}

#[cfg(target_arch = "arm")]
impl AesEngine {
    /// Conservative bound for one CRYPTO sequence completion poll.
    pub const DEFAULT_TIMEOUT_ITERATIONS: u32 = 1_000_000;

    /// Consume the singleton CRYPTO token and create the owned AES engine.
    pub fn new(
        peripheral: crate::peripherals::Crypto,
        timeout_iterations: u32,
    ) -> Result<Self, AesError> {
        let timeout_iterations = validate_timeout_iterations(timeout_iterations)?;
        let mut registers = HwRegisters;
        restore_clock(&mut registers)?;
        wait_idle(&mut registers, timeout_iterations)?;
        Ok(Self {
            _peripheral: peripheral,
            timeout_iterations,
        })
    }

    /// Encrypt one standard AES-128 block in place.
    pub fn encrypt_block(&mut self, key: &[u8; 16], block: &mut [u8; 16]) -> Result<(), AesError> {
        let input = *block;
        run_block(
            &mut HwRegisters,
            key,
            &input,
            block,
            self.timeout_iterations,
        )
    }

    /// Run two back-to-back AES-128 known-answer vectors with different keys.
    pub fn self_test(&mut self) -> Result<(), AesError> {
        self_test_with(&mut HwRegisters, self.timeout_iterations)
    }
}

#[cfg(any(target_arch = "arm", test))]
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
    use aes::cipher::{BlockEncrypt, KeyInit};
    use std::vec::Vec;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Event {
        Ctrl(u32),
        Wac(u32),
        Key(u32),
        Seq0(u32),
        Seqctrl(u32),
        Data0(u32),
        Cmd(u32),
        Data1,
    }

    struct MockRegisters {
        events: Vec<Event>,
        key_words: [u32; 4],
        key_index: usize,
        input_words: [u32; 4],
        input_index: usize,
        output_words: [u32; 4],
        output_index: usize,
        started: bool,
        busy_before_start: bool,
        busy_reads_after_start: Option<u32>,
        clock_enabled: bool,
        clock_enable_succeeds: bool,
        clock_enable_calls: u32,
        force_zero_output: bool,
    }

    impl MockRegisters {
        fn completing() -> Self {
            Self {
                events: Vec::new(),
                key_words: [0; 4],
                key_index: 0,
                input_words: [0; 4],
                input_index: 0,
                output_words: [0; 4],
                output_index: 0,
                started: false,
                busy_before_start: false,
                busy_reads_after_start: Some(1),
                clock_enabled: true,
                clock_enable_succeeds: true,
                clock_enable_calls: 0,
                force_zero_output: false,
            }
        }

        fn encrypt_loaded_block(&mut self) {
            if self.force_zero_output {
                self.output_words = [0; 4];
                self.output_index = 0;
                return;
            }

            let mut key = [0u8; 16];
            let mut input = [0u8; 16];
            for (index, word) in self.key_words.iter().enumerate() {
                key[index * 4..index * 4 + 4].copy_from_slice(&word.to_le_bytes());
            }
            for (index, word) in self.input_words.iter().enumerate() {
                input[index * 4..index * 4 + 4].copy_from_slice(&word.to_le_bytes());
            }

            let cipher = Aes128::new(GenericArray::from_slice(&key));
            let mut block = GenericArray::clone_from_slice(&input);
            cipher.encrypt_block(&mut block);
            for (index, chunk) in block.chunks_exact(4).enumerate() {
                self.output_words[index] =
                    u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            }
            self.output_index = 0;
        }
    }

    impl CryptoRegisters for MockRegisters {
        fn crypto_clock_enabled(&mut self) -> bool {
            self.clock_enabled
        }

        fn enable_crypto_clock(&mut self) {
            self.clock_enable_calls += 1;
            if self.clock_enable_succeeds {
                self.clock_enabled = true;
            }
        }

        fn read_status(&mut self) -> u32 {
            if !self.started {
                return if self.busy_before_start {
                    STATUS_BUSY
                } else {
                    0
                };
            }
            match self.busy_reads_after_start.as_mut() {
                None => STATUS_BUSY,
                Some(0) => 0,
                Some(remaining) => {
                    *remaining -= 1;
                    STATUS_BUSY
                }
            }
        }

        fn write_ctrl(&mut self, value: u32) {
            self.events.push(Event::Ctrl(value));
            self.key_index = 0;
            self.input_index = 0;
            self.output_index = 0;
            self.started = false;
        }

        fn write_wac(&mut self, value: u32) {
            self.events.push(Event::Wac(value));
        }

        fn write_keybuf(&mut self, value: u32) {
            assert!(self.key_index < self.key_words.len());
            self.key_words[self.key_index] = value;
            self.key_index += 1;
            self.events.push(Event::Key(value));
        }

        fn write_seq0(&mut self, value: u32) {
            self.events.push(Event::Seq0(value));
        }

        fn write_seqctrl(&mut self, value: u32) {
            self.events.push(Event::Seqctrl(value));
        }

        fn write_data0(&mut self, value: u32) {
            assert!(self.input_index < self.input_words.len());
            self.input_words[self.input_index] = value;
            self.input_index += 1;
            self.events.push(Event::Data0(value));
        }

        fn write_cmd(&mut self, value: u32) {
            assert_eq!(self.key_index, 4);
            assert_eq!(self.input_index, 4);
            self.events.push(Event::Cmd(value));
            self.encrypt_loaded_block();
            self.started = true;
        }

        fn read_data1(&mut self) -> u32 {
            assert_eq!(self.read_status(), 0);
            assert!(self.output_index < self.output_words.len());
            let value = self.output_words[self.output_index];
            self.output_index += 1;
            self.events.push(Event::Data1);
            value
        }
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
    fn register_protocol_matches_emlib_and_standard_byte_order() {
        let (key, plaintext, ciphertext) = SELF_TEST_VECTORS[0];
        let mut registers = MockRegisters::completing();
        let mut output = [0u8; 16];
        run_block(&mut registers, &key, &plaintext, &mut output, 4).unwrap();

        assert_eq!(output, ciphertext);
        assert_eq!(
            registers.events,
            std::vec![
                Event::Ctrl(0),
                Event::Wac(0),
                Event::Key(0x0302_0100),
                Event::Key(0x0706_0504),
                Event::Key(0x0b0a_0908),
                Event::Key(0x0f0e_0d0c),
                Event::Seq0(0x0000_4405),
                Event::Seqctrl(16),
                Event::Data0(0x3322_1100),
                Event::Data0(0x7766_5544),
                Event::Data0(0xbbaa_9988),
                Event::Data0(0xffee_ddcc),
                Event::Cmd(0x200),
                Event::Data1,
                Event::Data1,
                Event::Data1,
                Event::Data1,
            ]
        );
    }

    #[test]
    fn two_vector_self_test_rekeys_and_reuses_engine() {
        let mut registers = MockRegisters::completing();
        self_test_with(&mut registers, 4).unwrap();
        assert_eq!(
            registers
                .events
                .iter()
                .filter(|event| matches!(event, Event::Cmd(_)))
                .count(),
            2
        );
    }

    #[test]
    fn clock_restore_revalidates_engine_before_encrypting() {
        let (key, plaintext, ciphertext) = SELF_TEST_VECTORS[0];
        let mut registers = MockRegisters::completing();
        registers.clock_enabled = false;
        let mut output = [0u8; 16];

        run_block(&mut registers, &key, &plaintext, &mut output, 4).unwrap();

        assert_eq!(output, ciphertext);
        assert_eq!(registers.clock_enable_calls, 1);
        assert_eq!(
            registers
                .events
                .iter()
                .filter(|event| matches!(event, Event::Cmd(_)))
                .count(),
            3
        );
    }

    #[test]
    fn unavailable_clock_is_fail_closed() {
        let mut registers = MockRegisters::completing();
        registers.clock_enabled = false;
        registers.clock_enable_succeeds = false;
        let mut output = [0xA5; 16];

        assert_eq!(
            run_block(&mut registers, &[0; 16], &[0; 16], &mut output, 4),
            Err(AesError::ClockUnavailable)
        );
        assert_eq!(output, [0xA5; 16]);
        assert!(registers.events.is_empty());
    }

    #[test]
    fn restored_dead_peripheral_is_fail_closed() {
        let mut registers = MockRegisters::completing();
        registers.clock_enabled = false;
        registers.force_zero_output = true;
        let mut output = [0x5A; 16];

        assert_eq!(
            run_block(&mut registers, &[0; 16], &[0; 16], &mut output, 4),
            Err(AesError::KnownAnswerMismatch)
        );
        assert_eq!(output, [0x5A; 16]);
    }

    #[test]
    fn timeout_before_programming_is_fail_closed() {
        let mut registers = MockRegisters::completing();
        registers.busy_before_start = true;
        let mut output = [0xA5; 16];
        assert_eq!(
            run_block(&mut registers, &[0; 16], &[0; 16], &mut output, 2),
            Err(AesError::Timeout)
        );
        assert_eq!(output, [0xA5; 16]);
        assert!(registers.events.is_empty());
    }

    #[test]
    fn timeout_after_start_does_not_publish_partial_output() {
        let mut registers = MockRegisters::completing();
        registers.busy_reads_after_start = None;
        let mut output = [0x5A; 16];
        assert_eq!(
            run_block(&mut registers, &[0; 16], &[0; 16], &mut output, 2),
            Err(AesError::Timeout)
        );
        assert_eq!(output, [0x5A; 16]);
        assert!(!registers.events.contains(&Event::Data1));
    }
}
