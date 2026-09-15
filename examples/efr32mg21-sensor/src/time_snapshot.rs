//! SysTick interpolation while its exception is masked during a clock read.

pub(super) struct Clock {
    last: u64,
}

impl Clock {
    pub const fn new() -> Self {
        Self { last: 0 }
    }

    pub fn now(
        &mut self,
        full_ms: u64,
        reload: u32,
        hclk_hz: u32,
        sample: impl FnMut() -> (bool, u32, bool),
    ) -> u64 {
        let sampled = snapshot_micros(full_ms, reload, hclk_hz, sample);
        // SysTick cannot count multiple coalesced exceptions. Hold the last
        // timestamp rather than regress; lost masked time is not reconstructed.
        self.last = self.last.max(sampled);
        self.last
    }
}

fn snapshot_micros(
    full_ms: u64,
    reload: u32,
    hclk_hz: u32,
    mut sample: impl FnMut() -> (bool, u32, bool),
) -> u64 {
    loop {
        let (pending_before, remaining, pending_after) = sample();
        if pending_before != pending_after {
            continue;
        }
        let elapsed_cycles = if remaining == 0 {
            0
        } else {
            u64::from(reload + 1 - remaining)
        };
        let full_ms = full_ms + u64::from(pending_after);
        return full_ms * 1_000 + elapsed_cycles * 1_000_000 / u64::from(hclk_hz);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HCLK: u32 = 38_400_000;
    const RELOAD: u32 = HCLK / 1_000 - 1;

    #[test]
    fn milliseconds_and_microseconds_use_the_declared_frequency() {
        assert_eq!(
            snapshot_micros(30_000, RELOAD, HCLK, || (false, RELOAD, false)),
            30_000_000
        );
        assert_eq!(
            snapshot_micros(10, RELOAD, HCLK, || (false, 19_200, false)),
            10_500
        );
    }

    #[test]
    fn pending_reload_does_not_move_the_clock_backwards() {
        let before = snapshot_micros(10, RELOAD, HCLK, || (false, 40, false));
        let pending = snapshot_micros(10, RELOAD, HCLK, || (true, RELOAD, true));
        let served = snapshot_micros(11, RELOAD, HCLK, || (false, RELOAD, false));
        assert_eq!((before, pending, served), (10_998, 11_000, 11_000));
    }

    #[test]
    fn reload_during_sampling_retries_with_the_new_period() {
        let mut reads = 0;
        let now = snapshot_micros(10, RELOAD, HCLK, || {
            reads += 1;
            if reads == 1 {
                (false, RELOAD, true)
            } else {
                (true, RELOAD - 100, true)
            }
        });
        assert_eq!(reads, 2);
        assert_eq!(now, 11_002);
    }

    #[test]
    fn zero_counter_is_the_start_of_a_period_not_its_end() {
        assert_eq!(snapshot_micros(0, RELOAD, HCLK, || (false, 0, false)), 0);
        assert_eq!(
            snapshot_micros(10, RELOAD, HCLK, || (true, 0, true)),
            11_000
        );
    }

    #[test]
    fn pending_millisecond_carry_keeps_the_full_epoch() {
        assert_eq!(
            snapshot_micros(u64::from(u32::MAX), RELOAD, HCLK, || (true, RELOAD, true)),
            (1u64 << 32) * 1_000
        );
    }

    #[test]
    fn coalesced_overflows_cannot_regress_an_observed_timestamp() {
        let mut clock = Clock::new();
        assert_eq!(clock.now(10, RELOAD, HCLK, || (true, 40, true)), 11_998);
        assert_eq!(clock.now(10, RELOAD, HCLK, || (true, RELOAD, true)), 11_998);
        assert_eq!(
            clock.now(12, RELOAD, HCLK, || (false, RELOAD, false)),
            12_000
        );
    }
}
