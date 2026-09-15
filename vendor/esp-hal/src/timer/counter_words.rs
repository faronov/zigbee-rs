pub(super) const fn split(value: u64) -> (u32, u32) {
    ((value >> 32) as u32, value as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn high_word_survives_counter_reload() {
        for value in [0, u32::MAX as u64, 1 << 32, (1 << 52) - 1, u64::MAX] {
            let (high, low) = split(value);
            assert_eq!((u64::from(high) << 32) | u64::from(low), value);
        }
        assert_eq!(split(0x000a_bcde_1234_5678), (0x000a_bcde, 0x1234_5678));
    }
}
