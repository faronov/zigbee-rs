use core::hint::spin_loop;

use crate::aes_kat::AES_128_KATS;

const AES_IDLE: u32 = 0;
#[cfg(test)]
const AES_BUSY: u32 = 1;

const KAT_KEY_1: [u8; 16] = AES_128_KATS[0].0;
const KAT_INPUT_1: [u8; 16] = AES_128_KATS[0].1;
const KAT_OUTPUT_1: [u8; 16] = AES_128_KATS[0].2;
const KAT_KEY_2: [u8; 16] = AES_128_KATS[1].0;
const KAT_INPUT_2: [u8; 16] = AES_128_KATS[1].1;
const KAT_OUTPUT_2: [u8; 16] = AES_128_KATS[1].2;

pub(crate) trait AesRegisters {
    fn state(&mut self) -> u32;
    fn configure_aes128_encrypt(&mut self);
    fn write_key(&mut self, key: &[u8; 16]);
    fn write_input(&mut self, input: &[u8; 16]);
    fn trigger(&mut self);
    fn read_output(&mut self, output: &mut [u8; 16]);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AesEngineError {
    BusyTimeout,
    CompletionTimeout,
    KnownAnswerTestFailed(u8),
}

pub(crate) struct AesDriver<R> {
    registers: R,
    wait_limit: u32,
}

impl<R: AesRegisters> AesDriver<R> {
    pub(crate) const fn new(registers: R, wait_limit: u32) -> Self {
        Self {
            registers,
            wait_limit,
        }
    }

    pub(crate) fn self_test(&mut self) -> Result<(), AesEngineError> {
        let mut output = [0u8; 16];
        self.encrypt(&KAT_KEY_1, &KAT_INPUT_1, &mut output)?;
        if output != KAT_OUTPUT_1 {
            return Err(AesEngineError::KnownAnswerTestFailed(1));
        }

        self.encrypt(&KAT_KEY_2, &KAT_INPUT_2, &mut output)?;
        if output != KAT_OUTPUT_2 {
            return Err(AesEngineError::KnownAnswerTestFailed(2));
        }

        Ok(())
    }

    pub(crate) fn encrypt(
        &mut self,
        key: &[u8; 16],
        input: &[u8; 16],
        output: &mut [u8; 16],
    ) -> Result<(), AesEngineError> {
        self.wait_for_state(AES_IDLE, AesEngineError::BusyTimeout)?;

        self.registers.configure_aes128_encrypt();
        self.registers.write_key(key);
        self.registers.write_input(input);
        self.registers.trigger();

        // Typical-mode AES can complete before software samples the transient
        // BUSY state. ESP-IDF and esp-hal therefore wait only for IDLE after
        // triggering; requiring BUSY to be observed rejects valid fast
        // completions on ESP32-C6.
        self.wait_for_state(AES_IDLE, AesEngineError::CompletionTimeout)?;

        let mut completed = [0u8; 16];
        self.registers.read_output(&mut completed);
        *output = completed;
        Ok(())
    }

    fn wait_for_state(
        &mut self,
        expected: u32,
        error: AesEngineError,
    ) -> Result<(), AesEngineError> {
        for _ in 0..self.wait_limit {
            if self.registers.state() == expected {
                return Ok(());
            }
            spin_loop();
        }
        Err(error)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Copy)]
    enum Behavior {
        Normal,
        CompletesBeforePolling,
        CorruptOutput,
        InitialBusy,
        NeverStarts,
        NeverCompletes,
    }

    struct MockRegisters {
        behavior: Behavior,
        started: bool,
        busy_observed: bool,
        key: [u8; 16],
        input: [u8; 16],
        output: [u8; 16],
        trigger_count: u8,
    }

    impl MockRegisters {
        fn new(behavior: Behavior) -> Self {
            Self {
                behavior,
                started: false,
                busy_observed: false,
                key: [0; 16],
                input: [0; 16],
                output: [0; 16],
                trigger_count: 0,
            }
        }
    }

    impl AesRegisters for MockRegisters {
        fn state(&mut self) -> u32 {
            match self.behavior {
                Behavior::InitialBusy if !self.started => AES_BUSY,
                Behavior::NeverStarts if self.started => AES_IDLE,
                Behavior::NeverCompletes if self.started => AES_BUSY,
                Behavior::Normal | Behavior::CorruptOutput
                    if self.started && !self.busy_observed =>
                {
                    self.busy_observed = true;
                    AES_BUSY
                }
                _ => AES_IDLE,
            }
        }

        fn configure_aes128_encrypt(&mut self) {}

        fn write_key(&mut self, key: &[u8; 16]) {
            self.key = *key;
        }

        fn write_input(&mut self, input: &[u8; 16]) {
            self.input = *input;
        }

        fn trigger(&mut self) {
            self.started = true;
            self.busy_observed = false;
            self.trigger_count += 1;
            if matches!(self.behavior, Behavior::NeverStarts) {
                return;
            }
            self.output = if matches!(self.behavior, Behavior::CorruptOutput) {
                [0; 16]
            } else {
                match (self.key, self.input) {
                    (KAT_KEY_1, KAT_INPUT_1) => KAT_OUTPUT_1,
                    (KAT_KEY_2, KAT_INPUT_2) => KAT_OUTPUT_2,
                    _ => [0x5a; 16],
                }
            };
        }

        fn read_output(&mut self, output: &mut [u8; 16]) {
            *output = self.output;
            self.started = false;
        }
    }

    #[test]
    fn self_test_covers_reuse_and_rekeying() {
        let mut driver = AesDriver::new(MockRegisters::new(Behavior::Normal), 4);

        assert_eq!(driver.self_test(), Ok(()));
        assert_eq!(driver.registers.trigger_count, 2);
    }

    #[test]
    fn self_test_accepts_completion_before_busy_can_be_sampled() {
        let mut driver = AesDriver::new(MockRegisters::new(Behavior::CompletesBeforePolling), 4);

        assert_eq!(driver.self_test(), Ok(()));
        assert_eq!(driver.registers.trigger_count, 2);
    }

    #[test]
    fn self_test_rejects_incorrect_ciphertext() {
        let mut driver = AesDriver::new(MockRegisters::new(Behavior::CorruptOutput), 4);

        assert_eq!(
            driver.self_test(),
            Err(AesEngineError::KnownAnswerTestFailed(1))
        );
    }

    #[test]
    fn busy_peripheral_times_out() {
        let mut driver = AesDriver::new(MockRegisters::new(Behavior::InitialBusy), 3);
        let mut output = [0xa5; 16];

        assert_eq!(
            driver.encrypt(&[0; 16], &[0; 16], &mut output),
            Err(AesEngineError::BusyTimeout)
        );
        assert_eq!(output, [0xa5; 16]);
    }

    #[test]
    fn ignored_trigger_fails_startup_known_answer_test() {
        let mut driver = AesDriver::new(MockRegisters::new(Behavior::NeverStarts), 3);

        assert_eq!(
            driver.self_test(),
            Err(AesEngineError::KnownAnswerTestFailed(1))
        );
    }

    #[test]
    fn stalled_operation_times_out_without_publishing_output() {
        let mut driver = AesDriver::new(MockRegisters::new(Behavior::NeverCompletes), 3);
        let mut output = [0xa5; 16];

        assert_eq!(
            driver.encrypt(&[0; 16], &[0; 16], &mut output),
            Err(AesEngineError::CompletionTimeout)
        );
        assert_eq!(output, [0xa5; 16]);
    }
}
