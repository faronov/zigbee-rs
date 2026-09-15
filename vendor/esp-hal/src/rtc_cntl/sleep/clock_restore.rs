//! Replay saved divider banks before selecting the saved clock source.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Register {
    Cpu,
    Ahb,
    #[cfg(any(test, esp32c6))]
    Mspi,
    Source,
    #[cfg(any(test, esp32h2))]
    LatchBus,
    RomTicks,
}

#[cfg(any(test, esp32c6))]
#[inline(always)]
pub(crate) fn c6(
    cpu: u32,
    ahb: u32,
    mspi: u32,
    source: u8,
    mhz: u32,
    mut write: impl FnMut(Register, u32),
) {
    write(Register::Cpu, cpu);
    write(Register::Ahb, ahb);
    write(Register::Mspi, mspi);
    write(Register::Source, u32::from(source));
    write(Register::RomTicks, mhz);
}

#[cfg(any(test, esp32h2))]
#[inline(always)]
pub(crate) fn h2(cpu: u8, ahb: u8, source: u8, mhz: u32, mut write: impl FnMut(Register, u32)) {
    write(Register::Cpu, u32::from(cpu));
    write(Register::Ahb, u32::from(ahb));
    write(Register::Source, u32::from(source));
    write(Register::LatchBus, 1);
    write(Register::RomTicks, mhz);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn c6_preserves_inactive_divider_banks_and_flash_timing_before_mux_switch() {
        let mut writes = std::vec::Vec::new();
        c6(0x0102, 0x0503, 0x0402, 1, 80, |reg, value| {
            writes.push((reg, value))
        });
        assert_eq!(
            writes,
            [
                (Register::Cpu, 0x0102),
                (Register::Ahb, 0x0503),
                (Register::Mspi, 0x0402),
                (Register::Source, 1),
                (Register::RomTicks, 80),
            ]
        );
    }

    #[test]
    fn h2_restores_nondefault_bus_divider_and_latches_before_updating_rom_delay() {
        let mut writes = std::vec::Vec::new();
        h2(1, 3, 1, 48, |reg, value| writes.push((reg, value)));
        assert_eq!(
            writes,
            [
                (Register::Cpu, 1),
                (Register::Ahb, 3),
                (Register::Source, 1),
                (Register::LatchBus, 1),
                (Register::RomTicks, 48),
            ]
        );
    }
}
