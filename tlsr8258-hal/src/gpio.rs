//! Pure-Rust TLSR8258 GPIO driver.
//!
//! Register layout is transcribed from the open, `static inline` functions in
//! Telink's `platform/chip_8258/gpio.h` (direction/read/write/toggle/
//! interrupt polarity — these ship as source, not as a compiled library, so
//! the addresses below are taken directly from the vendor header rather
//! than reverse engineered).
//!
//! **Ports B and C are a silicon-level exception for input-enable and
//! drive-strength.** Every other per-pin field (`in`/`oen`/`out`/`pol`/
//! `func`/`irq_wakeup_en`) follows one uniform digital-register formula
//! across all five ports (`platform/chip_8258/register.h`'s
//! `reg_gpio_ie(i) = REG_ADDR8(0x581 + (group << 3))` and friends), and
//! that formula is what this module uses for input-enable/drive-strength
//! on Ports A, D, and E too. But Telink's own per-port macros in the same
//! header — `areg_gpio_pb_ie = 0xBD`, `areg_gpio_pb_ds = 0xBF`,
//! `areg_gpio_pc_ie = 0xC0`, `areg_gpio_pc_ds = 0xC2` — show that Ports B
//! and C route *only* those two fields through the analog bus instead,
//! diverging from the header's own generic formula for exactly those two
//! fields on exactly those two ports. This was cross-checked against the
//! compiled `gpio_set_input_en()`/`gpio_set_data_strength()` bodies in
//! `libdrivers_8258.a:gpio.o` (both header and disassembly agree exactly,
//! including which port branches to which analog address), so it is held
//! at full confidence, not the lower "disassembly-only" tier the pull
//! resistor path below gets. See [`Pin::ie_location`]/[`Pin::ds_location`]
//! for the routing table and their host-testable coverage.
//!
//! The pull-up/down resistor path (`set_pull`) is different: the *addresses*
//! of the per-nibble analog registers (`areg_0e_pa0_pa3_pull` ..
//! `areg_15_pd4_pd7_pull`) are likewise taken verbatim from
//! `platform/chip_8258/register.h`, but the *bit-packing* (2 bits per pin,
//! shift = (pin_index_in_nibble) * 2, low nibble first) is not spelled out
//! in any header — `gpio_setup_up_down_resistor()` itself is compiled into
//! `libdrivers_8258.a` (closed source). That packing was cross-checked by
//! disassembling `gpio.o` from the vendor archive (see
//! `tests::pull_register_disassembly_notes` below for the raw evidence) and
//! is therefore held to a lower confidence tier than the rest of this file.
//! Treat `set_pull` as "very likely correct, not vendor-header-verified"
//! until confirmed on hardware.
//!
//! GPIO function-mux (routing a pin to UART/I2C/SPI/PWM instead of plain
//! GPIO) is out of scope for *this* pass, not because it is unrecoverable:
//! `gpio_set_func()` for anything other than `AS_GPIO` ships only as a
//! compiled body in this SDK snapshot, but a clean-room, Apache-2.0
//! reference (`modern-tc32/tlsr82xx`) reportedly documents the TLSR8258
//! pin-mux tables independently, and the two IE/DS registers above are
//! themselves evidence that vendor headers plus disassembly (or an
//! independent clean-room source) can fully recover these tables when the
//! work is done. Extending this module with the remaining per-pin mux
//! routes is left to a future pass. The `AS_GPIO` case itself
//! ([`set_function_gpio`]) was disassembly-confirmed to be a plain
//! `reg |= pin_mask` on the same `func` register documented below, so it
//! is implemented at full (header-equivalent) confidence today.

#[cfg(target_arch = "tc32")]
use super::mmio::{r8, w8};

/// One of the five TLSR8258 GPIO ports, matching Telink's `GPIO_GROUPx`
/// values (`platform/chip_8258/gpio.h`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Port {
    A,
    B,
    C,
    D,
    E,
}

impl Port {
    const fn index(self) -> u8 {
        match self {
            Port::A => 0,
            Port::B => 1,
            Port::C => 2,
            Port::D => 3,
            Port::E => 4,
        }
    }
}

/// A single GPIO pin, e.g. `Pin::new(Port::B, 5)` for `GPIO_PB5`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pin {
    port: Port,
    bit: u8,
}

/// Pull resistor selection, matching Telink's `GPIO_PullTypeDef`
/// (`platform/chip_8258/gpio.h`) both in name and in discriminant value —
/// the raw 2-bit field written to the analog register *is* this value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pull {
    Float = 0,
    PullUp1M = 1,
    PullDown100K = 2,
    PullUp10K = 3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioError {
    /// Port E has no documented pull-resistor analog register.
    PullNotSupportedOnPort,
    /// The analog bus timed out while reading/writing the pull-resistor
    /// register — see [`super::mmio::AnalogError`].
    Analog(super::mmio::AnalogError),
}

impl From<super::mmio::AnalogError> for GpioError {
    fn from(error: super::mmio::AnalogError) -> Self {
        GpioError::Analog(error)
    }
}

const REG_GPIO_BASE: u32 = super::mmio::REG_BASE + 0x580;

// Byte offsets within a port's 8-byte register block, matching
// `platform/chip_8258/register.h`'s `reg_gpio_in/ie/oen/out/pol/ds/func/
// irq_wakeup_en` macros (`REG_ADDR8(0x580 + (group << 3) + offset)`).
const OFFSET_IN: u32 = 0;
const OFFSET_IE: u32 = 1;
const OFFSET_OEN: u32 = 2;
const OFFSET_OUT: u32 = 3;
const OFFSET_POL: u32 = 4;
const OFFSET_DS: u32 = 5;
const OFFSET_FUNC: u32 = 6;
const OFFSET_IRQ_WAKEUP_EN: u32 = 7;

/// Per-nibble pull-resistor analog register addresses
/// (`areg_0e_pa0_pa3_pull` .. `areg_15_pd4_pd7_pull`,
/// `platform/chip_8258/register.h`). Indexed `[port][nibble]`; Port::E has
/// no entry (see module docs).
const PULL_AREG: [[u8; 2]; 4] = [
    [0x0E, 0x0F], // Port A: PA0-3, PA4-7
    [0x10, 0x11], // Port B
    [0x12, 0x13], // Port C
    [0x14, 0x15], // Port D
];

/// Input-enable analog register for Port B (`areg_gpio_pb_ie`,
/// `platform/chip_8258/register.h`) — see module docs for why this
/// diverges from the digital-register formula every other field uses.
const AREG_PB_IE: u8 = 0xBD;
/// Drive-strength analog register for Port B (`areg_gpio_pb_ds`).
const AREG_PB_DS: u8 = 0xBF;
/// Input-enable analog register for Port C (`areg_gpio_pc_ie`).
const AREG_PC_IE: u8 = 0xC0;
/// Drive-strength analog register for Port C (`areg_gpio_pc_ds`).
const AREG_PC_DS: u8 = 0xC2;

/// Where a per-pin, single-bit field's register lives: the uniform
/// digital per-port block (`Digital`), or — only for input-enable/
/// drive-strength on Ports B/C — the analog bus (`Analog`). See the
/// module docs for why this split exists.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FieldLocation {
    Digital(u32),
    Analog(u8),
}

impl Pin {
    /// Construct a pin, panicking (in *every* build profile, not just
    /// debug — see [`Self::try_new`] for a non-panicking alternative) if
    /// `bit` is out of the `0..8` range a real TLSR8258 GPIO port has.
    ///
    /// A previous version of this constructor used `debug_assert!`, which
    /// is compiled out in release builds: an out-of-range `bit` would
    /// silently produce a `Pin` whose `mask()` shifts by an out-of-range
    /// amount. `assert!` always checks, so invalid `Pin`s can no longer be
    /// constructed through this path in any build profile — for a
    /// compile-time-constant `bit` this constructor is itself `const`, so
    /// an invalid literal is rejected at compile time.
    pub const fn new(port: Port, bit: u8) -> Self {
        assert!(bit < 8, "GPIO bit index must be 0..8");
        Self { port, bit }
    }

    /// Non-panicking, validated alternative to [`Self::new`] for
    /// runtime-computed bit indices (e.g. iterating over a caller-supplied
    /// pin list). Returns `None` if `bit >= 8`.
    pub const fn try_new(port: Port, bit: u8) -> Option<Self> {
        if bit < 8 {
            Some(Self { port, bit })
        } else {
            None
        }
    }

    const fn mask(self) -> u8 {
        1u8 << self.bit
    }

    /// Expose the pin's port and bit index (e.g. for peripherals like the
    /// ADC that need to look pins up in their own channel tables).
    pub const fn port_and_bit(self) -> (Port, u8) {
        (self.port, self.bit)
    }

    const fn reg(self, offset: u32) -> u32 {
        REG_GPIO_BASE + ((self.port.index() as u32) << 3) + offset
    }

    /// Register location for this pin's input-enable bit. Ports B and C
    /// route this through the analog bus instead of the digital register
    /// the generic formula would otherwise compute — see module docs.
    pub const fn ie_location(self) -> FieldLocation {
        match self.port {
            Port::B => FieldLocation::Analog(AREG_PB_IE),
            Port::C => FieldLocation::Analog(AREG_PC_IE),
            _ => FieldLocation::Digital(self.reg(OFFSET_IE)),
        }
    }

    /// Register location for this pin's drive-strength bit. See
    /// [`Self::ie_location`] for why Ports B/C differ.
    pub const fn ds_location(self) -> FieldLocation {
        match self.port {
            Port::B => FieldLocation::Analog(AREG_PB_DS),
            Port::C => FieldLocation::Analog(AREG_PC_DS),
            _ => FieldLocation::Digital(self.reg(OFFSET_DS)),
        }
    }

    /// Analog register address and 2-bit shift for this pin's pull
    /// resistor, or `None` for Port E (unsupported, see module docs).
    const fn pull_location(self) -> Option<(u8, u8)> {
        let port = self.port.index();
        if port >= 4 {
            return None;
        }
        let nibble = (self.bit / 4) as usize;
        let shift = (self.bit % 4) * 2;
        Some((PULL_AREG[port as usize][nibble], shift))
    }
}

/// Enable (`true`) or disable (Hi-Z, `false`) a pin's output driver.
///
/// Mirrors `gpio_set_output_en()`: the hardware's `oen` bit is
/// active-*low* for "output enabled", so this function inverts `enable`
/// before writing, matching the vendor source exactly.
#[cfg(target_arch = "tc32")]
pub fn set_output_enable(pin: Pin, enable: bool) {
    let addr = pin.reg(OFFSET_OEN);
    let mask = pin.mask();
    unsafe {
        let value = r8(addr);
        w8(addr, if enable { value & !mask } else { value | mask });
    }
}

/// Enable or disable a pin's input buffer/read-back path.
///
/// Ports B/C route this bit through the analog bus instead of the digital
/// per-port block every other port (and every other field) uses — see
/// module docs and [`Pin::ie_location`].
#[cfg(target_arch = "tc32")]
pub fn set_input_enable(pin: Pin, enable: bool) -> Result<(), GpioError> {
    set_field_bit(pin.ie_location(), pin.mask(), enable)
}

/// Drive a pin high (`true`) or low (`false`). Has no effect unless the
/// pin's output is also enabled via [`set_output_enable`].
#[cfg(target_arch = "tc32")]
pub fn write(pin: Pin, high: bool) {
    let addr = pin.reg(OFFSET_OUT);
    let mask = pin.mask();
    unsafe {
        let value = r8(addr);
        w8(addr, if high { value | mask } else { value & !mask });
    }
}

/// Flip a pin's output latch (independent of the input read-back value).
#[cfg(target_arch = "tc32")]
pub fn toggle(pin: Pin) {
    let addr = pin.reg(OFFSET_OUT);
    let mask = pin.mask();
    unsafe {
        w8(addr, r8(addr) ^ mask);
    }
}

/// Read a pin's current input level.
#[cfg(target_arch = "tc32")]
pub fn read(pin: Pin) -> bool {
    let addr = pin.reg(OFFSET_IN);
    unsafe { r8(addr) & pin.mask() != 0 }
}

/// Select strong (`true`) or weak (`false`) output drive strength.
///
/// Ports B/C route this bit through the analog bus instead of the digital
/// per-port block every other port uses — see module docs and
/// [`Pin::ds_location`]. This was previously mischaracterized in this
/// module as "a handful of special-function pins poke undeciphered analog
/// registers"; disassembly plus the vendor header's own per-port macros
/// now confirm it is simply *every* pin on Ports B and C, not a special
/// case, and the registers are fully identified (see [`FieldLocation`]).
#[cfg(target_arch = "tc32")]
pub fn set_drive_strength(pin: Pin, strong: bool) -> Result<(), GpioError> {
    set_field_bit(pin.ds_location(), pin.mask(), strong)
}

/// Shared read-modify-write for a single-bit field that may live in either
/// the digital or analog register space — see [`FieldLocation`].
#[cfg(target_arch = "tc32")]
fn set_field_bit(location: FieldLocation, mask: u8, set: bool) -> Result<(), GpioError> {
    match location {
        FieldLocation::Digital(addr) => unsafe {
            let value = r8(addr);
            w8(addr, if set { value | mask } else { value & !mask });
        },
        FieldLocation::Analog(addr) => {
            let value = super::mmio::analog_read(addr)?;
            super::mmio::analog_write(addr, if set { value | mask } else { value & !mask })?;
        }
    }
    Ok(())
}

/// Route a pin back to plain GPIO (the only function-mux path this module
/// implements — see module docs for why peripheral muxing is out of scope).
#[cfg(target_arch = "tc32")]
pub fn set_function_gpio(pin: Pin) {
    let addr = pin.reg(OFFSET_FUNC);
    let mask = pin.mask();
    unsafe {
        w8(addr, r8(addr) | mask);
    }
}

/// Set the edge polarity (`true` = falling, `false` = rising) that this
/// pin's interrupt/wakeup logic reacts to. Does not itself enable the
/// interrupt — pair with [`set_wakeup_interrupt_enable`] and the caller's
/// own global IRQ mask policy.
#[cfg(target_arch = "tc32")]
pub fn set_interrupt_polarity(pin: Pin, falling: bool) {
    let addr = pin.reg(OFFSET_POL);
    let mask = pin.mask();
    unsafe {
        let value = r8(addr);
        w8(addr, if falling { value | mask } else { value & !mask });
    }
}

/// Enable or disable this pin as a wakeup/interrupt source
/// (`reg_gpio_irq_wakeup_en`, gated by `FLD_IRQ_GPIO_EN` in the global IRQ
/// mask — left to the caller, matching this crate's "IRQs stay off unless
/// the application opts in" convention).
#[cfg(target_arch = "tc32")]
pub fn set_wakeup_interrupt_enable(pin: Pin, enable: bool) {
    let addr = pin.reg(OFFSET_IRQ_WAKEUP_EN);
    let mask = pin.mask();
    unsafe {
        let value = r8(addr);
        w8(addr, if enable { value | mask } else { value & !mask });
    }
}

/// Configure a pin's internal pull resistor.
///
/// See the module-level docs for the confidence caveat on this specific
/// path (addresses are vendor-header-verified; the 2-bit packing was
/// confirmed by disassembly rather than open source).
#[cfg(target_arch = "tc32")]
pub fn set_pull(pin: Pin, pull: Pull) -> Result<(), GpioError> {
    let (addr, shift) = pin
        .pull_location()
        .ok_or(GpioError::PullNotSupportedOnPort)?;
    let mask = 0x03u8 << shift;
    let value = super::mmio::analog_read(addr)?;
    let updated = (value & !mask) | ((pull as u8) << shift);
    super::mmio::analog_write(addr, updated)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_addresses_match_vendor_macro() {
        // reg_gpio_in(i) = REG_ADDR8(0x580 + ((i>>8)<<3)); i>>8 is the group
        // index for GPIO_GROUPx = 0x000/0x100/0x200/0x300/0x400.
        assert_eq!(Pin::new(Port::A, 0).reg(OFFSET_IN), 0x800580);
        assert_eq!(Pin::new(Port::B, 0).reg(OFFSET_IN), 0x800588);
        assert_eq!(Pin::new(Port::C, 0).reg(OFFSET_IN), 0x800590);
        assert_eq!(Pin::new(Port::D, 0).reg(OFFSET_IN), 0x800598);
        assert_eq!(Pin::new(Port::E, 0).reg(OFFSET_IN), 0x8005A0);

        // Sub-registers within a port's 8-byte block. Note `reg(OFFSET_IE)`/
        // `reg(OFFSET_DS)` here are just the generic digital-offset formula
        // from `platform/chip_8258/register.h` — for Port B specifically
        // that formula is *not* where input-enable/drive-strength actually
        // live (see `ie_location`/`ds_location` tests below); this test
        // only pins down the raw offset arithmetic `reg()` performs.
        let pb = Pin::new(Port::B, 3);
        assert_eq!(pb.reg(OFFSET_IE), 0x800589);
        assert_eq!(pb.reg(OFFSET_OEN), 0x80058A);
        assert_eq!(pb.reg(OFFSET_OUT), 0x80058B);
        assert_eq!(pb.reg(OFFSET_POL), 0x80058C);
        assert_eq!(pb.reg(OFFSET_DS), 0x80058D);
        assert_eq!(pb.reg(OFFSET_FUNC), 0x80058E);
        assert_eq!(pb.reg(OFFSET_IRQ_WAKEUP_EN), 0x80058F);
    }

    #[test]
    fn analog_register_constants_match_vendor_per_port_macros() {
        // `areg_gpio_pb_ie`/`areg_gpio_pb_ds`/`areg_gpio_pc_ie`/
        // `areg_gpio_pc_ds` in `platform/chip_8258/register.h`, confirmed
        // against the compiled `gpio_set_input_en()`/
        // `gpio_set_data_strength()` bodies in `libdrivers_8258.a:gpio.o`.
        assert_eq!(AREG_PB_IE, 0xBD);
        assert_eq!(AREG_PB_DS, 0xBF);
        assert_eq!(AREG_PC_IE, 0xC0);
        assert_eq!(AREG_PC_DS, 0xC2);
    }

    #[test]
    fn ie_location_routes_ports_b_and_c_through_the_analog_bus() {
        for bit in 0..8 {
            assert_eq!(
                Pin::new(Port::B, bit).ie_location(),
                FieldLocation::Analog(AREG_PB_IE)
            );
            assert_eq!(
                Pin::new(Port::C, bit).ie_location(),
                FieldLocation::Analog(AREG_PC_IE)
            );
        }
    }

    #[test]
    fn ds_location_routes_ports_b_and_c_through_the_analog_bus() {
        for bit in 0..8 {
            assert_eq!(
                Pin::new(Port::B, bit).ds_location(),
                FieldLocation::Analog(AREG_PB_DS)
            );
            assert_eq!(
                Pin::new(Port::C, bit).ds_location(),
                FieldLocation::Analog(AREG_PC_DS)
            );
        }
    }

    #[test]
    fn ie_and_ds_location_use_the_digital_register_for_ports_a_d_e() {
        for port in [Port::A, Port::D, Port::E] {
            let pin = Pin::new(port, 2);
            assert_eq!(
                pin.ie_location(),
                FieldLocation::Digital(pin.reg(OFFSET_IE))
            );
            assert_eq!(
                pin.ds_location(),
                FieldLocation::Digital(pin.reg(OFFSET_DS))
            );
        }
    }

    #[test]
    fn bit_mask_matches_pin_index() {
        assert_eq!(Pin::new(Port::A, 0).mask(), 0x01);
        assert_eq!(Pin::new(Port::A, 7).mask(), 0x80);
        assert_eq!(Pin::new(Port::D, 5).mask(), 0x20);
    }

    #[test]
    fn try_new_accepts_valid_bit_indices() {
        assert!(Pin::try_new(Port::A, 0).is_some());
        assert!(Pin::try_new(Port::A, 7).is_some());
    }

    #[test]
    fn try_new_rejects_out_of_range_bit_indices() {
        assert_eq!(Pin::try_new(Port::A, 8), None);
        assert_eq!(Pin::try_new(Port::A, 255), None);
    }

    #[test]
    #[should_panic(expected = "GPIO bit index must be 0..8")]
    fn new_panics_on_out_of_range_bit_index() {
        let _ = Pin::new(Port::A, 8);
    }

    #[test]
    fn pull_location_low_nibble() {
        // PA0..PA3 -> areg 0x0E, shift 0/2/4/6.
        assert_eq!(Pin::new(Port::A, 0).pull_location(), Some((0x0E, 0)));
        assert_eq!(Pin::new(Port::A, 1).pull_location(), Some((0x0E, 2)));
        assert_eq!(Pin::new(Port::A, 2).pull_location(), Some((0x0E, 4)));
        assert_eq!(Pin::new(Port::A, 3).pull_location(), Some((0x0E, 6)));
    }

    #[test]
    fn pull_location_high_nibble() {
        // PA4..PA7 -> areg 0x0F, shift 0/2/4/6.
        assert_eq!(Pin::new(Port::A, 4).pull_location(), Some((0x0F, 0)));
        assert_eq!(Pin::new(Port::A, 7).pull_location(), Some((0x0F, 6)));
    }

    #[test]
    fn pull_location_covers_all_documented_ports() {
        assert_eq!(Pin::new(Port::B, 0).pull_location(), Some((0x10, 0)));
        assert_eq!(Pin::new(Port::B, 4).pull_location(), Some((0x11, 0)));
        assert_eq!(Pin::new(Port::C, 0).pull_location(), Some((0x12, 0)));
        assert_eq!(Pin::new(Port::C, 4).pull_location(), Some((0x13, 0)));
        assert_eq!(Pin::new(Port::D, 0).pull_location(), Some((0x14, 0)));
        assert_eq!(Pin::new(Port::D, 4).pull_location(), Some((0x15, 0)));
    }

    #[test]
    fn pull_location_none_on_port_e() {
        // Port E has no `areg_..._pull` entry in register.h.
        assert_eq!(Pin::new(Port::E, 0).pull_location(), None);
    }

    /// Raw evidence for the module-level confidence caveat: this is the
    /// `llvm-objdump -d` output for `gpio_setup_up_down_resistor` extracted
    /// from `libdrivers_8258.a:gpio.o` (Telink SDK v3.7.x), showing the
    /// tail of the function performing exactly
    /// `analog_write(reg, (analog_read(reg) & !(0x3 << shift)) | ((pull & 3) << shift))`
    /// with `shift` taking the values 0, 2, 4, 6 depending on which pin in
    /// the nibble was requested (matching this module's `pull_location`).
    /// Kept here (not executed) purely as an audit trail.
    #[test]
    fn pull_register_disassembly_notes() {
        const _EVIDENCE: &str = r#"
        00000000 <gpio_setup_up_down_resistor>:
          ...
          30: tjl analog_read
          34: tmov r1, #0x3       ; r1 = up_down & 0x3
          36: tand r5, r1
          3a: tshftl r1, r6       ; r1 <<= shift (r6 in {0,2,4,6})
          3c: tand r7, r0         ; r7 = analog_read(reg) & inverse_mask
          3e: tor r7, r1          ; r7 |= (pull << shift)
          46: tjl analog_write
        "#;
        assert!(!_EVIDENCE.is_empty());
    }
}
