//! Register-independent Nordic ECB transaction driver.

use core::hint::spin_loop;

use crate::aes_kat::AES_128_KATS;

#[cfg(all(target_arch = "arm", feature = "nrf52840"))]
const NRF_RAM_END: usize = 0x2004_0000;
#[cfg(all(target_arch = "arm", feature = "nrf52833", not(feature = "nrf52840")))]
const NRF_RAM_END: usize = 0x2002_0000;

/// Failure from the fail-closed Nordic ECB AES-128 backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NrfAesError {
    /// A transaction did not complete before its finite polling bound.
    Timeout,
    /// `STOPECB` was issued, but neither completion event acknowledged it.
    AbortTimeout,
    /// `ERRORECB` reported an AES-core conflict or another hardware failure.
    Hardware,
    /// The EasyDMA block is not aligned or resident in the selected chip's RAM.
    InvalidDmaBuffer,
    /// A timed-out operation could not be aborted safely.
    Poisoned,
    /// The composition root did not install and validate an ECB engine.
    #[cfg(any(feature = "nrf52840", feature = "nrf52833"))]
    NotInstalled,
    /// One of the two startup known-answer tests produced wrong ciphertext.
    KnownAnswerTestFailed(u8),
    /// The unique ECB peripheral has already been installed or rejected.
    #[cfg(any(feature = "nrf52840", feature = "nrf52833"))]
    AlreadyInstalled,
}

/// Exact 48-byte layout consumed and produced by Nordic ECB EasyDMA.
#[repr(C, align(4))]
pub(crate) struct EcbDataBlock {
    pub(crate) key: [u8; 16],
    pub(crate) cleartext: [u8; 16],
    pub(crate) ciphertext: [u8; 16],
}

const _: () = assert!(core::mem::size_of::<EcbDataBlock>() == 48);
const _: () = assert!(core::mem::align_of::<EcbDataBlock>() >= 4);

pub(crate) trait EcbRegisters {
    fn clear_end_event(&mut self);
    fn clear_error_event(&mut self);
    fn set_data_ptr(&mut self, data: *mut EcbDataBlock);
    fn start(&mut self);
    fn end_event(&mut self) -> bool;
    fn error_event(&mut self) -> bool;
    fn stop(&mut self);
}

pub(crate) struct NrfAesDriver<R> {
    registers: R,
    data: EcbDataBlock,
    wait_limit: u32,
    poisoned: bool,
}

impl<R: EcbRegisters> NrfAesDriver<R> {
    pub(crate) const fn new(registers: R, wait_limit: u32) -> Self {
        Self {
            registers,
            data: EcbDataBlock {
                key: [0; 16],
                cleartext: [0; 16],
                ciphertext: [0; 16],
            },
            wait_limit,
            poisoned: false,
        }
    }

    pub(crate) fn self_test(&mut self) -> Result<(), NrfAesError> {
        for (index, (key, input, expected)) in AES_128_KATS.iter().enumerate() {
            let mut output = [0u8; 16];
            self.encrypt(key, input, &mut output)?;
            if output != *expected {
                return Err(NrfAesError::KnownAnswerTestFailed(index as u8 + 1));
            }
        }
        Ok(())
    }

    pub(crate) fn encrypt(
        &mut self,
        key: &[u8; 16],
        input: &[u8; 16],
        output: &mut [u8; 16],
    ) -> Result<(), NrfAesError> {
        if self.poisoned {
            return Err(NrfAesError::Poisoned);
        }

        self.data.key = *key;
        self.data.cleartext = *input;
        self.data.ciphertext = [0; 16];

        let data_ptr = core::ptr::addr_of_mut!(self.data);
        if !valid_dma_address(data_ptr) {
            self.scrub();
            return Err(NrfAesError::InvalidDmaBuffer);
        }

        // Events are sticky. Both must be cleared before the pointer and task
        // are programmed so stale completion can never publish old output.
        self.registers.clear_end_event();
        self.registers.clear_error_event();
        self.registers.set_data_ptr(data_ptr);
        dma_memory_fence();
        self.registers.start();

        for _ in 0..self.wait_limit {
            // Treat ERRORECB as authoritative if both events are observed.
            if self.registers.error_event() {
                dma_memory_fence();
                self.clear_events();
                self.scrub();
                return Err(NrfAesError::Hardware);
            }
            if self.registers.end_event() {
                dma_memory_fence();
                let mut completed = [0u8; 16];
                for (destination, source) in completed.iter_mut().zip(self.data.ciphertext.iter()) {
                    // SAFETY: `source` is inside the live EasyDMA block. The
                    // ENDECB event plus the fence above guarantees DMA is done.
                    *destination = unsafe { core::ptr::read_volatile(source) };
                }
                self.clear_events();
                self.scrub();
                *output = completed;
                return Ok(());
            }
            spin_loop();
        }

        // Abort before returning. Nordic guarantees EasyDMA is finished once
        // ENDECB or ERRORECB is generated, so keep the buffer intact and poison
        // the engine if the bounded abort acknowledgement itself times out.
        self.registers.stop();
        for _ in 0..self.wait_limit {
            if self.registers.error_event() || self.registers.end_event() {
                dma_memory_fence();
                self.clear_events();
                self.scrub();
                return Err(NrfAesError::Timeout);
            }
            spin_loop();
        }
        self.poisoned = true;
        Err(NrfAesError::AbortTimeout)
    }

    fn clear_events(&mut self) {
        self.registers.clear_end_event();
        self.registers.clear_error_event();
    }

    fn scrub(&mut self) {
        self.data.key = [0; 16];
        self.data.cleartext = [0; 16];
        self.data.ciphertext = [0; 16];
    }
}

#[inline(always)]
fn dma_memory_fence() {
    #[cfg(target_arch = "arm")]
    // SAFETY: `dmb sy` only orders memory accesses. Omitting `nomem` gives
    // the compiler a memory clobber as well as emitting the CPU barrier.
    unsafe {
        core::arch::asm!("dmb sy", options(nostack, preserves_flags));
    }

    #[cfg(not(target_arch = "arm"))]
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
}

#[inline]
fn valid_dma_address(data: *mut EcbDataBlock) -> bool {
    if !(data as usize).is_multiple_of(core::mem::align_of::<EcbDataBlock>()) {
        return false;
    }

    #[cfg(target_arch = "arm")]
    {
        let start = data as usize;
        start >= 0x2000_0000
            && start
                .checked_add(core::mem::size_of::<EcbDataBlock>())
                .is_some_and(|end| end <= NRF_RAM_END)
    }

    #[cfg(not(target_arch = "arm"))]
    {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use self::std::vec;
    use self::std::vec::Vec;

    #[derive(Clone, Copy)]
    enum Behavior {
        CompleteAfter(u8),
        HardwareError,
        Timeout,
        AbortTimeout,
        CorruptOutput,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Call {
        ClearEnd,
        ClearError,
        SetPointer,
        Start,
        Stop,
    }

    struct MockRegisters {
        behavior: Behavior,
        data: *mut EcbDataBlock,
        end: bool,
        error: bool,
        polls: u8,
        starts: u8,
        keys: Vec<[u8; 16]>,
        calls: Vec<Call>,
    }

    impl MockRegisters {
        fn new(behavior: Behavior) -> Self {
            Self {
                behavior,
                data: core::ptr::null_mut(),
                // Begin with both sticky events set to exercise stale clearing.
                end: true,
                error: true,
                polls: 0,
                starts: 0,
                keys: Vec::new(),
                calls: Vec::new(),
            }
        }

        fn finish_if_due(&mut self) {
            if let Behavior::CompleteAfter(delay) = self.behavior
                && self.polls >= delay
            {
                self.end = true;
            }
        }
    }

    impl EcbRegisters for MockRegisters {
        fn clear_end_event(&mut self) {
            self.calls.push(Call::ClearEnd);
            self.end = false;
        }

        fn clear_error_event(&mut self) {
            self.calls.push(Call::ClearError);
            self.error = false;
        }

        fn set_data_ptr(&mut self, data: *mut EcbDataBlock) {
            self.calls.push(Call::SetPointer);
            self.data = data;
        }

        fn start(&mut self) {
            self.calls.push(Call::Start);
            self.starts += 1;
            self.polls = 0;
            // SAFETY: the driver passes its live, aligned `data` field and
            // does not move it while this synchronous transaction runs.
            let data = unsafe { &mut *self.data };
            self.keys.push(data.key);
            data.ciphertext = if matches!(self.behavior, Behavior::CorruptOutput) {
                [0x5a; 16]
            } else {
                AES_128_KATS
                    .iter()
                    .find_map(|(key, input, output)| {
                        (data.key == *key && data.cleartext == *input).then_some(*output)
                    })
                    .unwrap_or([0xa5; 16])
            };
            if matches!(self.behavior, Behavior::HardwareError) {
                self.error = true;
            }
            if matches!(self.behavior, Behavior::CorruptOutput) {
                self.end = true;
            }
            self.finish_if_due();
        }

        fn end_event(&mut self) -> bool {
            self.polls = self.polls.saturating_add(1);
            self.finish_if_due();
            self.end
        }

        fn error_event(&mut self) -> bool {
            self.error
        }

        fn stop(&mut self) {
            self.calls.push(Call::Stop);
            if matches!(self.behavior, Behavior::Timeout) {
                self.error = true;
            }
        }
    }

    #[test]
    fn succeeds_after_bounded_polling_without_publishing_stale_output() {
        let mut driver = NrfAesDriver::new(MockRegisters::new(Behavior::CompleteAfter(2)), 4);
        let mut output = [0x55; 16];

        assert_eq!(
            driver.encrypt(&[0x11; 16], &[0x22; 16], &mut output),
            Ok(())
        );
        assert_eq!(output, [0xa5; 16]);
    }

    #[test]
    fn accepts_immediate_completion() {
        let mut driver = NrfAesDriver::new(MockRegisters::new(Behavior::CompleteAfter(0)), 2);
        let mut output = [0; 16];

        assert_eq!(driver.encrypt(&[1; 16], &[2; 16], &mut output), Ok(()));
        assert_eq!(output, [0xa5; 16]);
    }

    #[test]
    fn hardware_error_does_not_publish_dma_output() {
        let mut driver = NrfAesDriver::new(MockRegisters::new(Behavior::HardwareError), 3);
        let mut output = [0x55; 16];

        assert_eq!(
            driver.encrypt(&[1; 16], &[2; 16], &mut output),
            Err(NrfAesError::Hardware)
        );
        assert_eq!(output, [0x55; 16]);
    }

    #[test]
    fn timeout_stops_engine_and_does_not_publish_dma_output() {
        let mut driver = NrfAesDriver::new(MockRegisters::new(Behavior::Timeout), 3);
        let mut output = [0x55; 16];

        assert_eq!(
            driver.encrypt(&[1; 16], &[2; 16], &mut output),
            Err(NrfAesError::Timeout)
        );
        assert_eq!(output, [0x55; 16]);
        assert!(driver.registers.calls.contains(&Call::Stop));
    }

    #[test]
    fn unacknowledged_abort_poisons_engine() {
        let mut driver = NrfAesDriver::new(MockRegisters::new(Behavior::AbortTimeout), 2);
        let mut output = [0x55; 16];

        assert_eq!(
            driver.encrypt(&[1; 16], &[2; 16], &mut output),
            Err(NrfAesError::AbortTimeout)
        );
        assert_eq!(
            driver.encrypt(&[1; 16], &[2; 16], &mut output),
            Err(NrfAesError::Poisoned)
        );
    }

    #[test]
    fn stale_events_are_cleared_before_pointer_and_start() {
        let mut driver = NrfAesDriver::new(MockRegisters::new(Behavior::CompleteAfter(0)), 2);
        let mut output = [0; 16];

        assert_eq!(driver.encrypt(&[1; 16], &[2; 16], &mut output), Ok(()));
        assert_eq!(
            &driver.registers.calls[..4],
            &[
                Call::ClearEnd,
                Call::ClearError,
                Call::SetPointer,
                Call::Start
            ]
        );
    }

    #[test]
    fn startup_kats_reuse_and_rekey_one_driver() {
        let mut driver = NrfAesDriver::new(MockRegisters::new(Behavior::CompleteAfter(0)), 2);

        assert_eq!(driver.self_test(), Ok(()));
        assert_eq!(driver.registers.starts, 2);
        assert_eq!(
            driver.registers.keys,
            vec![AES_128_KATS[0].0, AES_128_KATS[1].0]
        );
    }

    #[test]
    fn startup_kat_rejects_wrong_ciphertext() {
        let mut driver = NrfAesDriver::new(MockRegisters::new(Behavior::CorruptOutput), 2);

        assert_eq!(
            driver.self_test(),
            Err(NrfAesError::KnownAnswerTestFailed(1))
        );
    }
}
