//! Register-independent driver for the PHY6222/PHY6252 AES-128 ECB block.
//!
//! The datasheet (PHY6222 DS v1.5, section 4.4 "AES-ECB Encryption") only
//! states that the block supports AES-128 encryption; the register contract
//! below follows the vendor ROM routine `LL_ENC_AES128_Encrypt0` and its
//! clock-gating wrapper `LL_ENC_AES128_Encrypt1`, as reconstructed in pvvx
//! THB2 (`SDK/lib/ble_controller/ll_enc.c`, `SDK/lib/rf/patch.c`). No ROM
//! function or vendor library is called; the registers are driven here.
//!
//! | Offset | Use |
//! |--------|-----|
//! | `0x00` | enable: `0` stop, `1` start |
//! | `0x04` | control: `0xF10` = single mode, encrypt, byte-reverse in words |
//! | `0x10` | interrupt enable (only touched by the lab probe) |
//! | `0x14` | interrupt status (only read by the lab probe) |
//! | `0x20..=0x2C` | key, `key[0..4]` big-endian in `0x2C` ... `key[12..16]` in `0x20` |
//! | `0x30..=0x3C` | plaintext, same packing (`0x3C` first) |
//! | `0x50..=0x5C` | ciphertext, same packing (`0x5C` first) |
//!
//! Every block reloads key and plaintext (the vendor notes that the block
//! loses its key in sleep) and runs with the module clock (`PCR SW_CLK`
//! bit 4) enabled only for that block, like `LL_ENC_AES128_Encrypt1`.
//!
//! # Completion
//! The vendor single-mode path does not poll the status register: it waits a
//! fixed 20-iteration `volatile` countdown ("need delay 10-20 cycle for single
//! mode encrypt"), about 180 CPU cycles on a Cortex-M0. Only the CCM paths
//! poll `0x14`. The status semantics in single mode are not proven on
//! silicon, so completion here is a fixed wait of
//! [`COMPLETION_WAIT_CYCLES`] CPU cycles (more than five times the vendor
//! delay) and the startup known-answer test is the gate that proves the
//! result is ready. [`AesCore::completion_probe`] records the status register
//! for the hardware lab.
//!
//! # Revalidation
//! The engine gates the module clock after every block, so finding it enabled
//! on entry means another agent used or reconfigured the block. The next
//! block then reruns the known-answer test before any output is published.
//! The AES block is not in the AON domain; PHY62x2 system sleep is a warm
//! reboot, so a lost power domain always reruns the composition root and its
//! startup known-answer test.
//!
//! Failures never fall back to software AES and never publish output.

use crate::aes_kat::AES_128_KATS;

/// Enable register: `0` stops the engine, `1` starts one operation.
pub(crate) const REG_ENABLE: usize = 0x00;
/// Control register.
pub(crate) const REG_CTRL: usize = 0x04;
/// Interrupt enable register (lab probe only).
pub(crate) const REG_INT_ENABLE: usize = 0x10;
/// Interrupt status register (lab probe only).
pub(crate) const REG_INT_STATUS: usize = 0x14;
/// Lowest key word (`key[12..16]`).
pub(crate) const REG_KEY: usize = 0x20;
/// Lowest plaintext word (`plaintext[12..16]`).
pub(crate) const REG_DATA_IN: usize = 0x30;
/// Lowest ciphertext word (`ciphertext[12..16]`).
pub(crate) const REG_DATA_OUT: usize = 0x50;

const ENABLE_STOP: u32 = 0;
const ENABLE_START: u32 = 1;
/// Single mode, encrypt, byte-reverse within words (vendor `0xf10`).
pub(crate) const CTRL_SINGLE_ENCRYPT: u32 = 0xF10;
/// Interrupt enables written by the vendor CCM paths before polling `0x14`.
const INT_ENABLE_ALL: u32 = 0xF;
/// Vendor `LL_ENC_SINGLE_MODE_DONE_MASK`.
pub const STATUS_SINGLE_MODE_DONE: u32 = 0x8;

/// Fixed completion wait after start, in CPU cycles.
///
/// Vendor: 20 iterations of a `volatile` countdown (~180 Cortex-M0 cycles)
/// for an engine documented to need 10-20 cycles. 1024 cycles is 21 µs at
/// 48 MHz HCLK and 64 µs at 16 MHz.
pub const COMPLETION_WAIT_CYCLES: u32 = 1024;

/// Status register reads made by [`AesCore::completion_probe`] while
/// waiting for the single-mode done bit.
pub const PROBE_STATUS_POLLS: u32 = 256;

/// Failure from the fail-closed PHY62x2 AES-128 engine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phy6222AesError {
    /// The AES module clock (`PCR SW_CLK` bit 4) did not read back enabled.
    ClockUnavailable,
    /// A zero completion wait was requested.
    ZeroCompletionWait,
    /// The known-answer vector with this 1-based index produced a wrong
    /// result (the engine is unvalidated until a later KAT passes).
    KnownAnswerTestFailed(u8),
    /// No engine is installed, or its last known-answer test did not pass.
    NotValidated,
}

/// Register access used by [`AesCore`]; offsets are relative to the AES
/// base address.
pub(crate) trait AesRegisters {
    fn clock_enabled(&mut self) -> bool;
    fn set_clock(&mut self, enabled: bool);
    fn write(&mut self, offset: usize, value: u32);
    fn read(&mut self, offset: usize) -> u32;
    /// Busy-wait for at least `cycles` CPU cycles.
    fn wait_cycles(&mut self, cycles: u32);
}

/// Status register samples around one single-mode block (hardware lab).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompletionProbe {
    /// `0x14` after the stop write, before the block is started.
    pub status_before_start: u32,
    /// `0x14` on the first read after the start write.
    pub status_after_start: u32,
    /// Reads of `0x14` until [`STATUS_SINGLE_MODE_DONE`] was seen
    /// (`None`: not within [`PROBE_STATUS_POLLS`]).
    pub polls_until_done: Option<u32>,
    /// `0x14` after the fixed completion wait.
    pub status_after_wait: u32,
    /// `0x14` after the stop write that ends the block.
    pub status_after_stop: u32,
    /// The block produced the FIPS-197 C.1 ciphertext.
    pub output_matches: bool,
}

/// Fail-closed engine state machine over a register implementation.
pub(crate) struct AesCore<R> {
    registers: R,
    completion_cycles: u32,
    validated: bool,
}

impl<R: AesRegisters> AesCore<R> {
    /// The engine starts unvalidated: [`Self::encrypt`] refuses until
    /// [`Self::self_test`] passes.
    pub(crate) const fn new(registers: R) -> Self {
        Self {
            registers,
            completion_cycles: COMPLETION_WAIT_CYCLES,
            validated: false,
        }
    }

    /// Change the completion wait (lab tuning). Invalidates the engine until
    /// the next passing [`Self::self_test`].
    pub(crate) fn set_completion_cycles(&mut self, cycles: u32) -> Result<(), Phy6222AesError> {
        if cycles == 0 {
            return Err(Phy6222AesError::ZeroCompletionWait);
        }
        self.completion_cycles = cycles;
        self.validated = false;
        Ok(())
    }

    /// Run both known-answer vectors (different keys, back to back).
    pub(crate) fn self_test(&mut self) -> Result<(), Phy6222AesError> {
        self.validated = false;
        for (index, (key, input, expected)) in AES_128_KATS.iter().enumerate() {
            let mut output = [0u8; 16];
            self.run_block(key, input, &mut output)?;
            if output != *expected {
                return Err(Phy6222AesError::KnownAnswerTestFailed(index as u8 + 1));
            }
        }
        self.validated = true;
        Ok(())
    }

    /// Encrypt one block; `output` is written only on success.
    pub(crate) fn encrypt(
        &mut self,
        key: &[u8; 16],
        input: &[u8; 16],
        output: &mut [u8; 16],
    ) -> Result<(), Phy6222AesError> {
        if !self.validated {
            return Err(Phy6222AesError::NotValidated);
        }
        if self.registers.clock_enabled() {
            // Gated after every block by this engine: someone else ran or
            // reconfigured the block since. Prove it again first.
            self.self_test()?;
        }
        self.run_block(key, input, output)
    }

    /// One block in the `LL_ENC_AES128_Encrypt1` order, clock enabled only
    /// for its duration.
    fn run_block(
        &mut self,
        key: &[u8; 16],
        input: &[u8; 16],
        output: &mut [u8; 16],
    ) -> Result<(), Phy6222AesError> {
        self.clock_on()?;
        let registers = &mut self.registers;
        registers.write(REG_ENABLE, ENABLE_STOP);
        write_block(registers, REG_KEY, key);
        write_block(registers, REG_DATA_IN, input);
        registers.write(REG_CTRL, CTRL_SINGLE_ENCRYPT);
        registers.write(REG_ENABLE, ENABLE_START);
        registers.wait_cycles(self.completion_cycles);
        let mut completed = [0u8; 16];
        read_block(registers, REG_DATA_OUT, &mut completed);
        registers.write(REG_ENABLE, ENABLE_STOP);
        registers.set_clock(false);
        *output = completed;
        Ok(())
    }

    /// Run FIPS-197 C.1 once with the interrupt enables the vendor CCM paths
    /// use, sampling the status register (hardware lab). Leaves the engine
    /// unvalidated: rerun [`Self::self_test`] before encrypting.
    pub(crate) fn completion_probe(&mut self) -> Result<CompletionProbe, Phy6222AesError> {
        self.validated = false;
        let (key, input, expected) = &AES_128_KATS[0];
        self.clock_on()?;
        let registers = &mut self.registers;
        registers.write(REG_ENABLE, ENABLE_STOP);
        registers.write(REG_INT_ENABLE, INT_ENABLE_ALL);
        let status_before_start = registers.read(REG_INT_STATUS);
        write_block(registers, REG_KEY, key);
        write_block(registers, REG_DATA_IN, input);
        registers.write(REG_CTRL, CTRL_SINGLE_ENCRYPT);
        registers.write(REG_ENABLE, ENABLE_START);
        let status_after_start = registers.read(REG_INT_STATUS);
        let mut polls_until_done = None;
        for poll in 1..=PROBE_STATUS_POLLS {
            if registers.read(REG_INT_STATUS) & STATUS_SINGLE_MODE_DONE != 0 {
                polls_until_done = Some(poll);
                break;
            }
        }
        registers.wait_cycles(self.completion_cycles);
        let status_after_wait = registers.read(REG_INT_STATUS);
        let mut output = [0u8; 16];
        read_block(registers, REG_DATA_OUT, &mut output);
        registers.write(REG_ENABLE, ENABLE_STOP);
        let status_after_stop = registers.read(REG_INT_STATUS);
        registers.write(REG_INT_ENABLE, 0);
        registers.set_clock(false);
        Ok(CompletionProbe {
            status_before_start,
            status_after_start,
            polls_until_done,
            status_after_wait,
            status_after_stop,
            output_matches: output == *expected,
        })
    }

    fn clock_on(&mut self) -> Result<(), Phy6222AesError> {
        self.registers.set_clock(true);
        if self.registers.clock_enabled() {
            Ok(())
        } else {
            self.registers.set_clock(false);
            Err(Phy6222AesError::ClockUnavailable)
        }
    }
}

/// Write 16 bytes as four big-endian words, highest offset (`bytes[0..4]`)
/// first, as the vendor routine does.
fn write_block<R: AesRegisters>(registers: &mut R, base: usize, bytes: &[u8; 16]) {
    for (index, chunk) in bytes.as_chunks::<4>().0.iter().enumerate() {
        registers.write(base + 12 - 4 * index, u32::from_be_bytes(*chunk));
    }
}

fn read_block<R: AesRegisters>(registers: &mut R, base: usize, bytes: &mut [u8; 16]) {
    for (index, chunk) in bytes.as_chunks_mut::<4>().0.iter_mut().enumerate() {
        *chunk = registers.read(base + 12 - 4 * index).to_be_bytes();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use std::vec::Vec;
    use zigbee_crypto::{Aes128Forward, AesKey, ForwardAesProvider, SoftwareAes128, kat};

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Event {
        Clock(bool),
        Write(usize, u32),
        Read(usize),
        Wait(u32),
    }

    /// In-memory model of the vendor register contract.
    struct Model {
        events: Vec<Event>,
        clock: bool,
        clock_enable_works: bool,
        key: [u32; 4],
        data: [u32; 4],
        ctrl: u32,
        output: [u32; 4],
        status: u32,
        /// Cycles the engine needs after start.
        latency: u32,
        remaining: u32,
        corrupt: bool,
        writes_while_gated: u32,
    }

    impl Model {
        fn new() -> Self {
            Self {
                events: Vec::new(),
                clock: false,
                clock_enable_works: true,
                key: [0; 4],
                data: [0; 4],
                ctrl: 0,
                output: [0; 4],
                status: 0,
                latency: 20,
                remaining: 0,
                corrupt: false,
                writes_while_gated: 0,
            }
        }

        fn words_to_bytes(words: &[u32; 4]) -> [u8; 16] {
            let mut bytes = [0u8; 16];
            for index in 0..4 {
                // `words[3]` is offset 0x0C (first bytes).
                bytes[index * 4..index * 4 + 4].copy_from_slice(&words[3 - index].to_be_bytes());
            }
            bytes
        }

        fn start(&mut self) {
            assert_eq!(self.ctrl, CTRL_SINGLE_ENCRYPT);
            let key = Self::words_to_bytes(&self.key);
            let mut block = Self::words_to_bytes(&self.data);
            SoftwareAes128::new(&key).encrypt_block(&mut block).unwrap();
            if self.corrupt {
                block[0] ^= 1;
            }
            for index in 0..4 {
                self.output[3 - index] = u32::from_be_bytes([
                    block[index * 4],
                    block[index * 4 + 1],
                    block[index * 4 + 2],
                    block[index * 4 + 3],
                ]);
            }
            self.remaining = self.latency;
            self.status = 0;
            if self.remaining == 0 {
                self.status = STATUS_SINGLE_MODE_DONE;
            }
        }

        fn advance(&mut self, cycles: u32) {
            self.remaining = self.remaining.saturating_sub(cycles);
            if self.remaining == 0 {
                self.status |= STATUS_SINGLE_MODE_DONE;
            }
        }

        fn starts(&self) -> usize {
            self.events
                .iter()
                .filter(|event| **event == Event::Write(REG_ENABLE, ENABLE_START))
                .count()
        }
    }

    impl AesRegisters for Model {
        fn clock_enabled(&mut self) -> bool {
            self.clock
        }

        fn set_clock(&mut self, enabled: bool) {
            self.events.push(Event::Clock(enabled));
            self.clock = enabled && self.clock_enable_works;
        }

        fn write(&mut self, offset: usize, value: u32) {
            self.events.push(Event::Write(offset, value));
            if !self.clock {
                self.writes_while_gated += 1;
                return;
            }
            match offset {
                REG_ENABLE if value == ENABLE_START => self.start(),
                REG_ENABLE | REG_INT_ENABLE => {}
                REG_CTRL => self.ctrl = value,
                0x20..=0x2C => self.key[(offset - REG_KEY) / 4] = value,
                0x30..=0x3C => self.data[(offset - REG_DATA_IN) / 4] = value,
                _ => panic!("unexpected write {offset:#x}"),
            }
        }

        fn read(&mut self, offset: usize) -> u32 {
            self.events.push(Event::Read(offset));
            if !self.clock {
                return 0;
            }
            match offset {
                REG_INT_STATUS => {
                    // Each status read takes a few cycles of wall time.
                    self.advance(4);
                    self.status
                }
                // Before completion the output words still hold zeros.
                0x50..=0x5C if self.remaining > 0 => 0,
                0x50..=0x5C => self.output[(offset - REG_DATA_OUT) / 4],
                _ => panic!("unexpected read {offset:#x}"),
            }
        }

        fn wait_cycles(&mut self, cycles: u32) {
            self.events.push(Event::Wait(cycles));
            self.advance(cycles);
        }
    }

    fn validated() -> AesCore<Model> {
        let mut core = AesCore::new(Model::new());
        core.self_test().unwrap();
        core.registers.events.clear();
        core
    }

    struct ModelProvider(AesCore<Model>);

    struct ModelCipher<'a> {
        core: &'a mut AesCore<Model>,
        key: AesKey,
    }

    impl Aes128Forward for ModelCipher<'_> {
        type Error = Phy6222AesError;

        fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), Self::Error> {
            let input = *block;
            self.core.encrypt(&self.key, &input, block)
        }
    }

    impl ForwardAesProvider for ModelProvider {
        fn forward_cipher(&mut self, key: &AesKey) -> impl Aes128Forward + '_ {
            ModelCipher {
                core: &mut self.0,
                key: *key,
            }
        }
    }

    #[test]
    fn register_sequence_matches_vendor_single_mode_routine() {
        let (key, input, expected) = AES_128_KATS[0];
        let mut core = validated();
        let mut output = [0u8; 16];

        core.encrypt(&key, &input, &mut output).unwrap();

        assert_eq!(output, expected);
        assert_eq!(
            core.registers.events,
            [
                Event::Clock(true),
                Event::Write(REG_ENABLE, 0),
                Event::Write(0x2C, 0x0001_0203),
                Event::Write(0x28, 0x0405_0607),
                Event::Write(0x24, 0x0809_0A0B),
                Event::Write(0x20, 0x0C0D_0E0F),
                Event::Write(0x3C, 0x0011_2233),
                Event::Write(0x38, 0x4455_6677),
                Event::Write(0x34, 0x8899_AABB),
                Event::Write(0x30, 0xCCDD_EEFF),
                Event::Write(REG_CTRL, 0xF10),
                Event::Write(REG_ENABLE, 1),
                Event::Wait(COMPLETION_WAIT_CYCLES),
                Event::Read(0x5C),
                Event::Read(0x58),
                Event::Read(0x54),
                Event::Read(0x50),
                Event::Write(REG_ENABLE, 0),
                Event::Clock(false),
            ]
        );
        assert_eq!(core.registers.writes_while_gated, 0);
    }

    #[test]
    fn model_provider_passes_every_zigbee_crypto_kat() {
        let mut provider = ModelProvider(validated());
        assert_eq!(kat::run_all(&mut provider), Ok(()));
        let model = &provider.0.registers;
        assert!(!model.clock, "clock left running");
        assert_eq!(model.writes_while_gated, 0);
        // Every block reloads the key: one key load per start.
        let key_loads = model
            .events
            .iter()
            .filter(|event| matches!(event, Event::Write(0x2C, _)))
            .count();
        assert_eq!(key_loads, model.starts());
    }

    #[test]
    fn self_test_runs_both_vectors_with_different_keys() {
        let mut core = AesCore::new(Model::new());
        core.self_test().unwrap();
        let keys: Vec<u32> = core
            .registers
            .events
            .iter()
            .filter_map(|event| match event {
                Event::Write(0x2C, word) => Some(*word),
                _ => None,
            })
            .collect();
        assert_eq!(keys, [0x0001_0203, 0x2B7E_1516]);
    }

    #[test]
    fn unvalidated_engine_refuses_without_touching_registers() {
        let mut core = AesCore::new(Model::new());
        let mut output = [0x5A; 16];
        assert_eq!(
            core.encrypt(&[0; 16], &[0; 16], &mut output),
            Err(Phy6222AesError::NotValidated)
        );
        assert_eq!(output, [0x5A; 16]);
        assert!(core.registers.events.is_empty());
    }

    #[test]
    fn zero_completion_wait_is_rejected() {
        let mut core = validated();
        assert_eq!(
            core.set_completion_cycles(0),
            Err(Phy6222AesError::ZeroCompletionWait)
        );
        // Still usable with the previous wait.
        let mut output = [0u8; 16];
        assert_eq!(core.encrypt(&[0; 16], &[0; 16], &mut output), Ok(()));
    }

    #[test]
    fn changing_the_wait_requires_a_new_known_answer_test() {
        let mut core = validated();
        core.set_completion_cycles(2048).unwrap();
        let mut output = [0x5A; 16];
        assert_eq!(
            core.encrypt(&[0; 16], &[0; 16], &mut output),
            Err(Phy6222AesError::NotValidated)
        );
        core.self_test().unwrap();
        assert_eq!(core.encrypt(&[0; 16], &[0; 16], &mut output), Ok(()));
    }

    #[test]
    fn too_short_completion_wait_fails_the_known_answer_test() {
        let mut model = Model::new();
        model.latency = COMPLETION_WAIT_CYCLES + 1;
        let mut core = AesCore::new(model);
        assert_eq!(
            core.self_test(),
            Err(Phy6222AesError::KnownAnswerTestFailed(1))
        );
        assert!(!core.registers.clock);
        let mut output = [0x5A; 16];
        assert_eq!(
            core.encrypt(&[0; 16], &[0; 16], &mut output),
            Err(Phy6222AesError::NotValidated)
        );
        assert_eq!(output, [0x5A; 16]);
    }

    #[test]
    fn wrong_ciphertext_fails_the_known_answer_test() {
        let mut model = Model::new();
        model.corrupt = true;
        let mut core = AesCore::new(model);
        assert_eq!(
            core.self_test(),
            Err(Phy6222AesError::KnownAnswerTestFailed(1))
        );
    }

    #[test]
    fn unavailable_clock_is_fail_closed() {
        let mut model = Model::new();
        model.clock_enable_works = false;
        let mut core = AesCore::new(model);
        assert_eq!(core.self_test(), Err(Phy6222AesError::ClockUnavailable));
        assert_eq!(
            core.registers.events,
            [Event::Clock(true), Event::Clock(false)]
        );
    }

    #[test]
    fn clock_lost_after_validation_publishes_nothing() {
        let mut core = validated();
        core.registers.clock_enable_works = false;
        let mut output = [0x5A; 16];
        assert_eq!(
            core.encrypt(&[1; 16], &[2; 16], &mut output),
            Err(Phy6222AesError::ClockUnavailable)
        );
        assert_eq!(output, [0x5A; 16]);
        assert_eq!(core.registers.writes_while_gated, 0);
    }

    #[test]
    fn foreign_clock_use_reruns_the_known_answer_test_first() {
        let (key, input, expected) = AES_128_KATS[1];
        let mut core = validated();
        core.registers.clock = true;
        let mut output = [0u8; 16];

        core.encrypt(&key, &input, &mut output).unwrap();

        assert_eq!(output, expected);
        assert_eq!(core.registers.starts(), 3);
        assert!(!core.registers.clock);
    }

    #[test]
    fn foreign_use_of_a_broken_engine_is_sticky_and_publishes_nothing() {
        let mut core = validated();
        core.registers.clock = true;
        core.registers.corrupt = true;
        let mut output = [0x5A; 16];

        assert_eq!(
            core.encrypt(&[1; 16], &[2; 16], &mut output),
            Err(Phy6222AesError::KnownAnswerTestFailed(1))
        );
        assert_eq!(output, [0x5A; 16]);
        assert_eq!(core.registers.starts(), 1);

        core.registers.corrupt = false;
        assert_eq!(
            core.encrypt(&[1; 16], &[2; 16], &mut output),
            Err(Phy6222AesError::NotValidated)
        );
        assert_eq!(output, [0x5A; 16]);
    }

    #[test]
    fn completion_probe_samples_status_and_invalidates() {
        let mut core = validated();
        let probe = core.completion_probe().unwrap();
        // The model's done bit is sticky from the validation blocks and
        // cleared by the next start; silicon semantics are what the lab
        // probe measures.
        assert_eq!(probe.status_after_start, 0);
        assert!(probe.polls_until_done.is_some());
        assert_eq!(
            probe.status_after_wait & STATUS_SINGLE_MODE_DONE,
            STATUS_SINGLE_MODE_DONE
        );
        assert!(probe.output_matches);
        assert!(!core.registers.clock);
        assert_eq!(
            core.registers.events.last(),
            Some(&Event::Clock(false)),
            "probe must gate the clock"
        );
        let mut output = [0u8; 16];
        assert_eq!(
            core.encrypt(&[0; 16], &[0; 16], &mut output),
            Err(Phy6222AesError::NotValidated)
        );
    }
}
