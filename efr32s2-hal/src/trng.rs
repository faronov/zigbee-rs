//! CRYPTOACC true random number generator (Silex BA431) on EFR32xG22.
//!
//! The init sequence, register map and parameters follow GSDK 4.5.0
//! `sl_cryptoacc_library` (`sx_trng.c`, `ba431_config.[ch]`,
//! `cryptolib_def.h`) and `sli_cryptoacc_driver_trng.c`:
//!
//! 1. conditioning-function known-answer test in test mode;
//! 2. soft reset, FIFO wake-up level 8, switch-off timer 0, clock divider 7,
//!    init wait 512, 8 × 128-bit conditioning blocks, NDRNG enable;
//! 3. wait for the NIST start-up health test and check `STARTUP_FAIL`;
//! 4. load four FIFO words as the conditioning key, soft reset to flush;
//! 5. read words from the FIFO, checking the repetition-count and
//!    adaptive-proportion health tests before every word.
//!
//! Every hardware wait is bounded and reports [`TrngError::Timeout`]. The
//! output is the conditioned (AES-CBC-MAC) NDRNG stream, which is what the
//! GSDK PSA driver returns for `psa_generate_random`.
//!
//! xG21 has no CRYPTOACC; its TRNG sits behind the Secure Engine mailbox
//! and is not implemented here.

/// Errors reported by the TRNG.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrngError {
    /// The conditioning known-answer test returned a wrong result.
    ConditioningTest,
    /// The NIST start-up health test failed.
    StartupTest,
    /// A continuous health test (repetition count / adaptive proportion /
    /// AIS31 alarm) failed; the generator must be re-initialized.
    HealthTest,
    /// The generator entered the error state.
    ErrorState,
    /// A bounded hardware wait expired.
    Timeout,
}

/// BA431 register offsets from the register block base.
pub(crate) mod reg {
    pub const CTRL: u32 = 0x00;
    pub const FIFO_LEVEL: u32 = 0x04;
    pub const FIFO_THRESHOLD: u32 = 0x08;
    pub const KEY0: u32 = 0x10;
    pub const TEST_DATA: u32 = 0x20;
    pub const STATUS: u32 = 0x30;
    pub const INIT_WAIT: u32 = 0x34;
    pub const SW_OFF_TIMER: u32 = 0x40;
    pub const CLK_DIV: u32 = 0x44;
}

const CTRL_NDRNG_ENABLE: u32 = 1 << 0;
const CTRL_TEST_ENABLE: u32 = 1 << 2;
const CTRL_INT_REP_EN: u32 = 1 << 4;
const CTRL_INT_PROP_EN: u32 = 1 << 5;
const CTRL_SOFTRESET: u32 = 1 << 8;
const CTRL_INT_ALM_EN: u32 = 1 << 10;
const CTRL_BYPASS_HEALTH_TEST: u32 = 1 << 12;
const CTRL_BYPASS_AIS31_TEST: u32 = 1 << 13;
const CTRL_NB_128BIT_BLOCKS_SHIFT: u32 = 16;
const CTRL_NB_128BIT_BLOCKS_MASK: u32 = 0xF << CTRL_NB_128BIT_BLOCKS_SHIFT;

const STAT_TESTDATA_BUSY: u32 = 1 << 0;
const STAT_STATE_MASK: u32 = 0xE;
const STAT_REP_FAIL: u32 = 1 << 4;
const STAT_PROP_FAIL: u32 = 1 << 5;
const STAT_PREALM_INT: u32 = 1 << 8;
const STAT_ALM_INT: u32 = 1 << 9;
const STAT_STARTUP_FAIL: u32 = 1 << 10;

const STATE_RESET: u32 = 0x0;
const STATE_STARTUP: u32 = 0x2;
const STATE_ERROR: u32 = 0xA;

/// `cryptolib_def.h` defaults.
const RNG_FIFO_WAKEUP_LVL: u32 = 8;
const RNG_OFF_TIMER_VAL: u32 = 0;
const RNG_CLKDIV: u32 = 7;
const RNG_INIT_WAIT_VAL: u32 = 512;
const RNG_NB_128BIT_BLOCKS: u32 = 8;

/// `sx_trng_conditioning_test` vectors.
const KAT_TEST_DATA: [u32; 16] = [
    0xE1BC_C06B,
    0x9199_452A,
    0x1A74_34E1,
    0x2519_9E7F,
    0x578A_2DAE,
    0x9CAC_031E,
    0xAC6F_B79E,
    0x518E_AF45,
    0x461C_C830,
    0x11E4_5CA3,
    0x19C1_FBE5,
    0xEF52_0A1A,
    0x4524_9FF6,
    0x179B_4FDF,
    0x7B41_2BAD,
    0x1037_6CE6,
];
const KAT_KEY: [u32; 4] = [0x1615_7E2B, 0xA6D2_AE28, 0x8815_F7AB, 0x3C4F_CF09];
const KAT_ANSWER: [u32; 4] = [0xA1CA_F13F, 0x09AC_1F68, 0x30CA_0E12, 0xA7E1_8675];

/// Poll budget for one hardware wait. The start-up test needs well under a
/// millisecond at the GSDK clock divider; this bounds a dead block to a few
/// hundred milliseconds at 38.4 MHz.
const WAIT_LIMIT: u32 = 2_000_000;

/// Register access used by [`Ba431`]; the target implementation is MMIO,
/// host tests substitute a model.
pub(crate) trait Ba431Bus {
    fn read(&mut self, offset: u32) -> u32;
    fn write(&mut self, offset: u32, value: u32);
    fn read_fifo(&mut self) -> u32;
}

/// BA431 driver logic, independent of the register transport.
pub(crate) struct Ba431<B: Ba431Bus> {
    bus: B,
    ready: bool,
}

impl<B: Ba431Bus> Ba431<B> {
    pub(crate) fn new(bus: B) -> Self {
        Self { bus, ready: false }
    }

    #[cfg(test)]
    pub(crate) fn bus(&self) -> &B {
        &self.bus
    }

    fn ctrl_set(&mut self, bits: u32) {
        let v = self.bus.read(reg::CTRL);
        self.bus.write(reg::CTRL, v | bits);
    }

    fn soft_reset(&mut self) {
        let v = self.bus.read(reg::CTRL);
        self.bus.write(reg::CTRL, v | CTRL_SOFTRESET);
        let v = self.bus.read(reg::CTRL);
        self.bus.write(reg::CTRL, v & !CTRL_SOFTRESET);
    }

    fn set_blocks(&mut self, blocks: u32) {
        let v = self.bus.read(reg::CTRL);
        self.bus.write(
            reg::CTRL,
            (v & !CTRL_NB_128BIT_BLOCKS_MASK) | (blocks << CTRL_NB_128BIT_BLOCKS_SHIFT),
        );
    }

    fn write_key(&mut self, key: &[u32; 4]) {
        for (i, k) in key.iter().enumerate() {
            self.bus.write(reg::KEY0 + 4 * i as u32, *k);
        }
    }

    fn wait(&mut self, mut done: impl FnMut(&mut B) -> bool) -> Result<(), TrngError> {
        for _ in 0..WAIT_LIMIT {
            if done(&mut self.bus) {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(TrngError::Timeout)
    }

    fn conditioning_test(&mut self) -> Result<(), TrngError> {
        self.soft_reset();
        self.set_blocks(4);
        self.bus.write(
            reg::CTRL,
            CTRL_TEST_ENABLE | CTRL_BYPASS_AIS31_TEST | CTRL_BYPASS_HEALTH_TEST,
        );
        self.write_key(&KAT_KEY);
        for word in KAT_TEST_DATA {
            self.wait(|b| b.read(reg::STATUS) & STAT_TESTDATA_BUSY == 0)?;
            self.bus.write(reg::TEST_DATA, word);
        }
        self.wait(|b| b.read(reg::FIFO_LEVEL) >= 4)?;
        self.bus.write(reg::CTRL, 0);
        let mut diff = 0;
        for expected in KAT_ANSWER {
            diff |= self.bus.read_fifo() ^ expected;
        }
        if diff != 0 {
            return Err(TrngError::ConditioningTest);
        }
        Ok(())
    }

    fn wait_startup(&mut self) -> Result<(), TrngError> {
        let mut status = 0;
        self.wait(|b| {
            status = b.read(reg::STATUS);
            !matches!(status & STAT_STATE_MASK, STATE_RESET | STATE_STARTUP)
        })?;
        if status & STAT_STARTUP_FAIL != 0 {
            return Err(TrngError::StartupTest);
        }
        if status & STAT_STATE_MASK == STATE_ERROR {
            return Err(TrngError::ErrorState);
        }
        Ok(())
    }

    fn next_fifo_word(&mut self) -> Result<u32, TrngError> {
        self.wait(|b| b.read(reg::FIFO_LEVEL) > 0)?;
        Ok(self.bus.read_fifo())
    }

    /// `sx_trng_init(DO_TRNG_COND_TEST)`.
    pub(crate) fn init(&mut self) -> Result<(), TrngError> {
        self.ready = false;
        self.bus.write(reg::CTRL, 0);
        self.conditioning_test()?;
        self.soft_reset();
        self.bus.write(reg::FIFO_THRESHOLD, RNG_FIFO_WAKEUP_LVL);
        self.bus.write(reg::SW_OFF_TIMER, RNG_OFF_TIMER_VAL);
        self.bus.write(reg::CLK_DIV, RNG_CLKDIV);
        self.bus.write(reg::INIT_WAIT, RNG_INIT_WAIT_VAL);
        self.set_blocks(RNG_NB_128BIT_BLOCKS);
        self.ctrl_set(CTRL_NDRNG_ENABLE);
        self.wait_startup()?;
        let mut key = [0u32; 4];
        for k in &mut key {
            *k = self.next_fifo_word()?;
        }
        self.write_key(&key);
        // Flush the FIFO, which still holds output of the KAT-keyed stage.
        self.soft_reset();
        self.ctrl_set(CTRL_INT_REP_EN | CTRL_INT_PROP_EN | CTRL_INT_ALM_EN);
        self.wait_startup()?;
        self.ready = true;
        Ok(())
    }

    fn check_health(&mut self) -> Result<(), TrngError> {
        let status = self.bus.read(reg::STATUS);
        if status & (STAT_REP_FAIL | STAT_PROP_FAIL | STAT_ALM_INT) != 0 {
            self.ready = false;
            return Err(TrngError::HealthTest);
        }
        if status & STAT_STATE_MASK == STATE_ERROR {
            self.ready = false;
            return Err(TrngError::ErrorState);
        }
        if status & STAT_PREALM_INT != 0 {
            // `sx_trng_get_rand_blk`: a preliminary AIS31 alarm restarts the
            // generator and repeats the start-up test.
            self.soft_reset();
            self.wait_startup()?;
        }
        Ok(())
    }

    /// Fill `out` with conditioned random bytes.
    pub(crate) fn fill(&mut self, out: &mut [u8]) -> Result<(), TrngError> {
        if !self.ready {
            self.init()?;
        }
        for chunk in out.chunks_mut(4) {
            self.check_health()?;
            let word = self.next_fifo_word()?.to_le_bytes();
            chunk.copy_from_slice(&word[..chunk.len()]);
        }
        // The health tests run on the noise source; re-check after the last
        // word so a failure during this request is not silently returned.
        self.check_health()
    }

    /// Stop the noise source (`CTRL = 0`).
    pub(crate) fn stop(&mut self) {
        self.bus.write(reg::CTRL, 0);
        self.ready = false;
    }
}

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod hw {
    /// `CRYPTOACC_S_RNGCTRL_BASE` (efr32mg22c224f512im40.h).
    const RNGCTRL_BASE: u32 = 0x4C02_1000;
    /// `CRYPTOACC_RNGOUT_FIFO_S_MEM_BASE`.
    const RNGOUT_FIFO: u32 = 0x4C02_4000;

    pub(crate) struct Mmio;

    impl super::Ba431Bus for Mmio {
        fn read(&mut self, offset: u32) -> u32 {
            unsafe { core::ptr::read_volatile((RNGCTRL_BASE + offset) as *const u32) }
        }
        fn write(&mut self, offset: u32, value: u32) {
            unsafe { core::ptr::write_volatile((RNGCTRL_BASE + offset) as *mut u32, value) }
        }
        fn read_fifo(&mut self) -> u32 {
            unsafe { core::ptr::read_volatile(RNGOUT_FIFO as *const u32) }
        }
    }
}

/// The CRYPTOACC TRNG.
///
/// Owns the CRYPTOACC random-number block; create at most one. Construction
/// opens the CRYPTOACC clocks and runs the full GSDK init sequence including
/// the conditioning known-answer test. [`release`](Self::release) stops the
/// noise source and gates the clocks again, so a sleepy device pays nothing
/// once its key material exists.
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub struct Trng {
    inner: Ba431<hw::Mmio>,
}

#[cfg(all(target_arch = "arm", target_os = "none"))]
impl Trng {
    /// Open the CRYPTOACC clocks and initialize the generator.
    pub fn new() -> Result<Self, TrngError> {
        crate::clock::enable_cryptoacc();
        let mut inner = Ba431::new(hw::Mmio);
        if let Err(e) = inner.init() {
            inner.stop();
            crate::clock::disable_cryptoacc();
            return Err(e);
        }
        Ok(Self { inner })
    }

    /// Fill `out` with random bytes. After a health-test failure the next
    /// call re-runs the full init sequence.
    pub fn fill(&mut self, out: &mut [u8]) -> Result<(), TrngError> {
        self.inner.fill(out)
    }

    /// Stop the generator and gate the CRYPTOACC clocks.
    pub fn release(mut self) {
        self.inner.stop();
        crate::clock::disable_cryptoacc();
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::collections::VecDeque;
    use std::vec::Vec;

    /// Behavioural BA431 model: KAT in test mode, start-up state machine,
    /// FIFO of scripted words, injectable status faults.
    struct Model {
        ctrl: u32,
        key: [u32; 4],
        test_words: u32,
        fifo: VecDeque<u32>,
        status_extra: u32,
        startup_polls: u32,
        state: u32,
        next_word: u32,
        kat_wrong: bool,
        writes: Vec<(u32, u32)>,
    }

    impl Model {
        fn new() -> Self {
            Self {
                ctrl: 0,
                key: [0; 4],
                test_words: 0,
                fifo: VecDeque::new(),
                status_extra: 0,
                startup_polls: 0,
                state: STATE_RESET,
                next_word: 0x1000,
                kat_wrong: false,
                writes: Vec::new(),
            }
        }
        fn refill(&mut self) {
            while self.state != STATE_RESET
                && self.state != STATE_STARTUP
                && self.ctrl & CTRL_TEST_ENABLE == 0
                && self.fifo.len() < 32
            {
                self.next_word = self
                    .next_word
                    .wrapping_mul(1_103_515_245)
                    .wrapping_add(12_345);
                self.fifo.push_back(self.next_word);
            }
        }
    }

    impl Ba431Bus for Model {
        fn read(&mut self, offset: u32) -> u32 {
            match offset {
                reg::CTRL => self.ctrl,
                reg::FIFO_LEVEL => {
                    self.refill();
                    self.fifo.len() as u32
                }
                reg::STATUS => {
                    if self.state == STATE_STARTUP {
                        if self.startup_polls == 0 {
                            self.state = 0x8; // running
                        } else {
                            self.startup_polls -= 1;
                        }
                    }
                    self.state | self.status_extra
                }
                _ => 0,
            }
        }
        fn write(&mut self, offset: u32, value: u32) {
            self.writes.push((offset, value));
            match offset {
                reg::CTRL => {
                    if value & CTRL_SOFTRESET != 0 {
                        self.fifo.clear();
                        if self.ctrl & CTRL_NDRNG_ENABLE != 0 {
                            self.state = STATE_STARTUP;
                            self.startup_polls = 3;
                        }
                    }
                    if value & CTRL_NDRNG_ENABLE != 0 && self.ctrl & CTRL_NDRNG_ENABLE == 0 {
                        self.state = STATE_STARTUP;
                        self.startup_polls = 3;
                    }
                    if value & CTRL_NDRNG_ENABLE == 0 {
                        self.state = STATE_RESET;
                    }
                    self.ctrl = value & !CTRL_SOFTRESET;
                }
                o if (reg::KEY0..reg::KEY0 + 16).contains(&o) => {
                    self.key[((o - reg::KEY0) / 4) as usize] = value;
                }
                reg::TEST_DATA => {
                    self.test_words += 1;
                    if self.test_words.is_multiple_of(16) && self.key == KAT_KEY {
                        for (i, w) in KAT_ANSWER.iter().enumerate() {
                            let w = if self.kat_wrong && i == 2 { w ^ 1 } else { *w };
                            self.fifo.push_back(w);
                        }
                    }
                }
                _ => {}
            }
        }
        fn read_fifo(&mut self) -> u32 {
            self.refill();
            self.fifo.pop_front().unwrap_or(0)
        }
    }

    #[test]
    fn init_runs_kat_then_seeds_conditioning_key() {
        let mut t = Ba431::new(Model::new());
        let mut out = [0u8; 16];
        t.fill(&mut out).unwrap();
        assert_ne!(out, [0u8; 16]);
        let m = t.bus();
        // GSDK parameters were programmed.
        assert!(m.writes.contains(&(reg::CLK_DIV, 7)));
        assert!(m.writes.contains(&(reg::INIT_WAIT, 512)));
        assert!(m.writes.contains(&(reg::FIFO_THRESHOLD, 8)));
        assert!(m.writes.contains(&(reg::SW_OFF_TIMER, 0)));
        assert_eq!(m.ctrl & CTRL_NB_128BIT_BLOCKS_MASK, 8 << 16);
        assert_eq!(
            m.ctrl & (CTRL_NDRNG_ENABLE | CTRL_INT_REP_EN | CTRL_INT_PROP_EN),
            CTRL_NDRNG_ENABLE | CTRL_INT_REP_EN | CTRL_INT_PROP_EN
        );
        // The KAT key was replaced by generator output.
        assert_ne!(m.key, KAT_KEY);
        assert_eq!(m.test_words, 16);
    }

    #[test]
    fn odd_lengths_and_distinct_words() {
        let mut t = Ba431::new(Model::new());
        let mut a = [0u8; 7];
        let mut b = [0u8; 7];
        t.fill(&mut a).unwrap();
        t.fill(&mut b).unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn wrong_kat_answer_fails() {
        let mut m = Model::new();
        m.kat_wrong = true;
        let mut t = Ba431::new(m);
        assert_eq!(t.fill(&mut [0u8; 4]), Err(TrngError::ConditioningTest));
    }

    #[test]
    fn startup_failure_is_reported() {
        let mut m = Model::new();
        m.status_extra = STAT_STARTUP_FAIL;
        let mut t = Ba431::new(m);
        assert_eq!(t.fill(&mut [0u8; 4]), Err(TrngError::StartupTest));
    }

    #[test]
    fn health_failure_forces_reinit() {
        let mut t = Ba431::new(Model::new());
        t.fill(&mut [0u8; 4]).unwrap();
        t.bus.status_extra = STAT_REP_FAIL;
        assert_eq!(t.fill(&mut [0u8; 4]), Err(TrngError::HealthTest));
        assert!(!t.ready);
        t.bus.status_extra = 0;
        let before = t.bus.test_words;
        t.fill(&mut [0u8; 4]).unwrap();
        // Re-initialized, including another KAT.
        assert_eq!(t.bus.test_words, before + 16);
    }

    #[test]
    fn stuck_fifo_times_out() {
        let mut t = Ba431::new(Model::new());
        t.fill(&mut [0u8; 4]).unwrap();
        // Disable the model's generator: FIFO never refills.
        t.bus.ctrl |= CTRL_TEST_ENABLE;
        t.bus.fifo.clear();
        assert_eq!(t.fill(&mut [0u8; 4]), Err(TrngError::Timeout));
    }
}
