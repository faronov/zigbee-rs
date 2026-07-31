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
//! Peripheral function-mux support is intentionally limited to the I2C, SPI,
//! UART, and non-complementary PWM routes used by this HAL. The selector
//! table and two-bit packing were checked against Telink's `gpio_set_func()`
//! object and the independent Apache-2.0 `modern-tc32/tlsr82xx`
//! implementation. Invalid pin/function pairs are rejected instead of
//! silently selecting slot zero.
//!
//! All twelve documented UART TX/RX routes are supported: TX on
//! PA2/PB1/PC2/PD0/PD3/PD7 and RX on PA0/PB0/PB7/PC3/PC5/PD6 (the same set
//! enumerated by `platform/chip_8258/uart.h`'s `UART_TxPinDef`/
//! `UART_RxPinDef`). `gpio_set_func()` itself is compiled, not inline, so
//! rather than trust a single decompilation these selector values were
//! derived by writing a small tc32 instruction interpreter and
//! symbolically executing the actual compiled `gpio_set_func()` body from
//! the official `telink-semi/telink_zigbee_sdk` repository's
//! `platform/lib/libdrivers_8258.a:gpio.o` for `func == AS_UART (3)` on
//! every pin value above — i.e. this is the vendor's real shipped object
//! code, disassembled and executed, not a guess. Every one of the twelve
//! resulting selectors was then independently cross-checked against
//! `modern-tc32/tlsr82xx`'s `tlsr82xx-hal/src/gpio.rs` `pinmux_mode_for`
//! table, and both sources agree on all twelve pins with no discrepancies
//! (PB1 `0b01`/PB7 `0b10` also match the values this HAL already shipped).
//! This is held at the same full-confidence tier as the vendor-header
//! fields above, not the lower "independent-source-only" tier.
//!
//! UART flow control (RTS on PA4/PB3/PB6/PC0, CTS on PA3/PB2/PC4/PD1) is
//! deliberately **not** listed in [`PinFunction`]/`function_selector` at
//! all: disassembling `uart_set_rts`/`uart_set_cts`/`uart_set_rts_level`
//! from the same `uart.o` shows they never call `gpio_set_func` — they
//! only pull up and enable input/output on the plain GPIO pin and then
//! configure `reg_uart_ctrl1`/`reg_uart_ctrl2` bits (see `uart.rs`). RTS/CTS
//! pins are therefore configured with this module's ordinary
//! `set_pull`/`set_output_enable`/`set_input_enable` primitives, consuming
//! a plain [`Pin`] token — no special mux path exists for them to omit or
//! get wrong.
//!
//! GPIO IRQ support (module-level: [`GpioIrqSource`],
//! [`set_source_interrupt_enable`], [`set_core_interrupt_enable`],
//! [`set_global_interrupt_enable`], [`interrupt_pending`],
//! [`clear_interrupt_pending`]) is transcribed directly from the *inline*
//! `static inline` bodies of `gpio_set_interrupt`/`gpio_en_interrupt`/
//! `gpio_set_interrupt_risc0`/`gpio_en_interrupt_risc0`/
//! `gpio_set_interrupt_risc1`/`gpio_en_interrupt_risc1` in
//! `platform/chip_8258/gpio.h`, plus `reg_gpio_wakeup_irq`'s
//! `FLD_GPIO_CORE_INTERRUPT_EN` and the `reg_irq_mask`/`reg_irq_src`
//! `FLD_IRQ_GPIO_EN`/`FLD_IRQ_GPIO_RISC0_EN`/`FLD_IRQ_GPIO_RISC1_EN` bits in
//! `platform/chip_8258/register.h`. These are open, shipped-as-source
//! functions (not compiled objects), so this path is held at full
//! confidence, same as the rest of this file's per-pin digital registers.

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

/// Uniquely owned GPIO pin.
#[derive(Debug, PartialEq, Eq)]
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
    /// This peripheral function is not routed to the selected pin.
    UnsupportedFunction,
}

impl From<super::mmio::AnalogError> for GpioError {
    fn from(error: super::mmio::AnalogError) -> Self {
        GpioError::Analog(error)
    }
}

const REG_GPIO_BASE: u32 = super::mmio::REG_BASE + 0x580;
const REG_MUX_BASE: u32 = super::mmio::REG_BASE + 0x5A8;

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

/// Peripheral functions supported by the pure-Rust pin-mux layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinFunction {
    Gpio,
    I2c,
    Spi,
    Uart,
    Pwm0,
    Pwm1,
    Pwm2,
    Pwm3,
    Pwm4,
    Pwm5,
}

impl Pin {
    pub(crate) const fn new(port: Port, bit: u8) -> Self {
        assert!(bit < 8, "GPIO bit index must be 0..8");
        Self { port, bit }
    }

    /// Construct a raw pin without participating in [`crate::Peripherals`]
    /// ownership.
    ///
    /// # Safety
    ///
    /// The caller must ensure no other `Pin` or peripheral driver owns or
    /// accesses the same physical pad.
    pub const unsafe fn steal(port: Port, bit: u8) -> Self {
        assert!(bit < 8, "GPIO bit index must be 0..8");
        Self { port, bit }
    }

    const fn mask(&self) -> u8 {
        1u8 << self.bit
    }

    /// Expose the pin's port and bit index (e.g. for peripherals like the
    /// ADC that need to look pins up in their own channel tables).
    pub const fn port_and_bit(&self) -> (Port, u8) {
        (self.port, self.bit)
    }

    const fn reg(&self, offset: u32) -> u32 {
        REG_GPIO_BASE + ((self.port.index() as u32) << 3) + offset
    }

    const fn mux_reg(&self) -> u32 {
        REG_MUX_BASE + (self.port.index() as u32 * 2) + (self.bit as u32 / 4)
    }

    const fn mux_shift(&self) -> u8 {
        (self.bit % 4) * 2
    }

    /// Register location for this pin's input-enable bit. Ports B and C
    /// route this through the analog bus instead of the digital register
    /// the generic formula would otherwise compute — see module docs.
    pub const fn ie_location(&self) -> FieldLocation {
        match self.port {
            Port::B => FieldLocation::Analog(AREG_PB_IE),
            Port::C => FieldLocation::Analog(AREG_PC_IE),
            _ => FieldLocation::Digital(self.reg(OFFSET_IE)),
        }
    }

    /// Register location for this pin's drive-strength bit. See
    /// [`Self::ie_location`] for why Ports B/C differ.
    pub const fn ds_location(&self) -> FieldLocation {
        match self.port {
            Port::B => FieldLocation::Analog(AREG_PB_DS),
            Port::C => FieldLocation::Analog(AREG_PC_DS),
            _ => FieldLocation::Digital(self.reg(OFFSET_DS)),
        }
    }

    /// Analog register address and 2-bit shift for this pin's pull
    /// resistor, or `None` for Port E (unsupported, see module docs).
    const fn pull_location(&self) -> Option<(u8, u8)> {
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
pub fn set_output_enable(pin: &Pin, enable: bool) {
    let addr = pin.reg(OFFSET_OEN);
    let mask = pin.mask();
    super::mmio::with_irqs_disabled(|| unsafe {
        let value = r8(addr);
        w8(addr, if enable { value & !mask } else { value | mask });
    });
}

/// Enable or disable a pin's input buffer/read-back path.
///
/// Ports B/C route this bit through the analog bus instead of the digital
/// per-port block every other port (and every other field) uses — see
/// module docs and [`Pin::ie_location`].
#[cfg(target_arch = "tc32")]
pub fn set_input_enable(pin: &Pin, enable: bool) -> Result<(), GpioError> {
    set_field_bit(pin.ie_location(), pin.mask(), enable)
}

/// Drive a pin high (`true`) or low (`false`). Has no effect unless the
/// pin's output is also enabled via [`set_output_enable`].
#[cfg(target_arch = "tc32")]
pub fn write(pin: &Pin, high: bool) {
    let addr = pin.reg(OFFSET_OUT);
    let mask = pin.mask();
    super::mmio::with_irqs_disabled(|| unsafe {
        let value = r8(addr);
        w8(addr, if high { value | mask } else { value & !mask });
    });
}

/// Flip a pin's output latch (independent of the input read-back value).
#[cfg(target_arch = "tc32")]
pub fn toggle(pin: &Pin) {
    let addr = pin.reg(OFFSET_OUT);
    let mask = pin.mask();
    super::mmio::with_irqs_disabled(|| unsafe {
        w8(addr, r8(addr) ^ mask);
    });
}

/// Read a pin's current input level.
#[cfg(target_arch = "tc32")]
pub fn read(pin: &Pin) -> bool {
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
pub fn set_drive_strength(pin: &Pin, strong: bool) -> Result<(), GpioError> {
    set_field_bit(pin.ds_location(), pin.mask(), strong)
}

/// Shared read-modify-write for a single-bit field that may live in either
/// the digital or analog register space — see [`FieldLocation`].
#[cfg(target_arch = "tc32")]
fn set_field_bit(location: FieldLocation, mask: u8, set: bool) -> Result<(), GpioError> {
    super::mmio::with_irqs_disabled(|| {
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
    })
}

/// Route a pin back to plain GPIO.
#[cfg(target_arch = "tc32")]
pub fn set_function_gpio(pin: &Pin) {
    let addr = pin.reg(OFFSET_FUNC);
    let mask = pin.mask();
    super::mmio::with_irqs_disabled(|| unsafe {
        w8(addr, r8(addr) | mask);
    });
}

/// Route a pin to a validated I2C, SPI, or PWM function.
///
/// I2C and SPI also require their group-level input/output selectors at
/// `0x5b6/0x5b7`; those are configured by the corresponding driver after
/// this per-pin selector is installed.
#[cfg(target_arch = "tc32")]
pub fn set_function(pin: &Pin, function: PinFunction) -> Result<(), GpioError> {
    if matches!(function, PinFunction::Gpio) {
        set_function_gpio(pin);
        return Ok(());
    }
    let selector = function_selector(pin, function).ok_or(GpioError::UnsupportedFunction)?;
    let shift = pin.mux_shift();
    let mask = 0x03u8 << shift;
    super::mmio::with_irqs_disabled(|| unsafe {
        let mux = r8(pin.mux_reg());
        w8(pin.mux_reg(), (mux & !mask) | ((selector & 0x03) << shift));
        let func = r8(pin.reg(OFFSET_FUNC));
        w8(pin.reg(OFFSET_FUNC), func & !pin.mask());
    });
    Ok(())
}

/// Return whether a pin has a proven route for a peripheral function.
pub const fn supports_function(pin: &Pin, function: PinFunction) -> bool {
    matches!(function, PinFunction::Gpio) || function_selector(pin, function).is_some()
}

const fn function_selector(pin: &Pin, function: PinFunction) -> Option<u8> {
    let (port, bit) = pin.port_and_bit();
    match (port, bit, function) {
        // I2C groups: PA3/PA4, PB6/PD7, PC0/PC1, PC2/PC3.
        (Port::A, 3, PinFunction::I2c)
        | (Port::A, 4, PinFunction::I2c)
        | (Port::C, 0, PinFunction::I2c)
        | (Port::C, 1, PinFunction::I2c)
        | (Port::D, 7, PinFunction::I2c) => Some(0),
        (Port::B, 6, PinFunction::I2c) => Some(1),
        (Port::C, 2, PinFunction::I2c) | (Port::C, 3, PinFunction::I2c) => Some(2),

        // SPI groups: PA2/PA3/PA4 and PB6/PB7/PD2/PD7. PD6 is the
        // vendor group's software-controlled CS pin and is accepted as SPI
        // by gpio_set_func, though SpiBus itself deliberately owns no CS.
        (Port::A, 2, PinFunction::Spi)
        | (Port::A, 3, PinFunction::Spi)
        | (Port::A, 4, PinFunction::Spi)
        | (Port::D, 2, PinFunction::Spi)
        | (Port::D, 6, PinFunction::Spi)
        | (Port::D, 7, PinFunction::Spi) => Some(0),
        (Port::B, 6, PinFunction::Spi) | (Port::B, 7, PinFunction::Spi) => Some(1),

        // UART TX routes: PA2, PB1, PC2, PD0, PD3, PD7.
        // UART RX routes: PA0, PB0, PB7, PC3, PC5, PD6.
        // Derived by symbolically executing the compiled `gpio_set_func()`
        // object for `func == AS_UART` on every pin and cross-checked
        // against `modern-tc32/tlsr82xx` — see the module docs for the
        // full evidence trail. RTS/CTS pins are intentionally absent: they
        // never go through `gpio_set_func` (see module docs).
        (Port::A, 0, PinFunction::Uart) => Some(0b10), // RX
        (Port::A, 2, PinFunction::Uart) => Some(0b01), // TX
        (Port::B, 0, PinFunction::Uart) => Some(0b01), // RX
        (Port::B, 1, PinFunction::Uart) => Some(0b01), // TX
        (Port::B, 7, PinFunction::Uart) => Some(0b10), // RX
        (Port::C, 2, PinFunction::Uart) => Some(0b01), // TX
        (Port::C, 3, PinFunction::Uart) => Some(0b01), // RX
        (Port::C, 5, PinFunction::Uart) => Some(0b01), // RX
        (Port::D, 0, PinFunction::Uart) => Some(0b10), // TX
        (Port::D, 3, PinFunction::Uart) => Some(0b10), // TX
        (Port::D, 6, PinFunction::Uart) => Some(0b01), // RX
        (Port::D, 7, PinFunction::Uart) => Some(0b10), // TX

        // Positive PWM outputs from the TLSR8258 gpio_set_func table.
        (Port::A, 2, PinFunction::Pwm0) | (Port::C, 1, PinFunction::Pwm0) => Some(2),
        (Port::C, 2, PinFunction::Pwm0) | (Port::D, 5, PinFunction::Pwm0) => Some(0),
        (Port::A, 3, PinFunction::Pwm1) => Some(2),
        (Port::C, 3, PinFunction::Pwm1) => Some(0),
        (Port::A, 4, PinFunction::Pwm2) => Some(2),
        (Port::C, 4, PinFunction::Pwm2) => Some(0),
        (Port::B, 0, PinFunction::Pwm3) => Some(0),
        (Port::D, 2, PinFunction::Pwm3) => Some(2),
        (Port::B, 1, PinFunction::Pwm4) => Some(0),
        (Port::B, 4, PinFunction::Pwm4) => Some(1),
        (Port::B, 2, PinFunction::Pwm5) => Some(0),
        (Port::B, 5, PinFunction::Pwm5) => Some(1),
        _ => None,
    }
}

/// Set the edge polarity (`true` = falling, `false` = rising) that this
/// pin's interrupt/wakeup logic reacts to. Does not itself enable the
/// interrupt — pair with [`set_wakeup_interrupt_enable`] (or
/// [`set_source_interrupt_enable`]) and [`set_global_interrupt_enable`].
///
/// Takes `&Pin` (not `Pin`) so callers can keep configuring/reading the same
/// pin afterwards instead of losing the ownership token to this one call.
#[cfg(target_arch = "tc32")]
pub fn set_interrupt_polarity(pin: &Pin, falling: bool) {
    let addr = pin.reg(OFFSET_POL);
    let mask = pin.mask();
    super::mmio::with_irqs_disabled(|| unsafe {
        let value = r8(addr);
        w8(addr, if falling { value | mask } else { value & !mask });
    });
}

/// Enable or disable this pin as a source of the *primary* GPIO IRQ
/// (`reg_gpio_irq_wakeup_en`, gated by `FLD_IRQ_GPIO_EN` in the global IRQ
/// mask — left to the caller via [`set_global_interrupt_enable`], matching
/// this crate's "IRQs stay off unless the application opts in" convention).
/// This helper only changes the per-pin source enable; callers using the
/// primary source must also own the shared
/// [`set_core_interrupt_enable`] policy.
///
/// Equivalent to `set_source_interrupt_enable(pin, GpioIrqSource::Primary,
/// enable)`. Takes `&Pin` (not `Pin`) for the same reason as
/// [`set_interrupt_polarity`].
#[cfg(target_arch = "tc32")]
pub fn set_wakeup_interrupt_enable(pin: &Pin, enable: bool) {
    set_source_interrupt_enable(pin, GpioIrqSource::Primary, enable);
}

/// The three independent GPIO-edge interrupt sources the TLSR8258 core can
/// see, each with its own per-pin enable register, its own bit in
/// `reg_irq_mask`/`reg_irq_src`, and (for [`GpioIrqSource::Primary`] only)
/// a shared core-interrupt gate — see [`set_core_interrupt_enable`].
///
/// Transcribed from the `static inline gpio_set_interrupt`/
/// `gpio_set_interrupt_risc0`/`gpio_set_interrupt_risc1` and
/// `gpio_en_interrupt`/`gpio_en_interrupt_risc0`/`gpio_en_interrupt_risc1`
/// bodies in `platform/chip_8258/gpio.h`. Because these are three distinct
/// hardware comparators (not a software fan-out of one), a caller can
/// assign different pins to different sources and distinguish which pin's
/// edge fired purely from which `reg_irq_src` bit is set — no software
/// pin-level polling required. See [`crate::capture`] for a reusable
/// consumer of exactly this property.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioIrqSource {
    /// `reg_gpio_irq_wakeup_en` / `FLD_IRQ_GPIO_EN` (`reg_irq_mask`/
    /// `reg_irq_src` bit 18). The only source gated by
    /// `FLD_GPIO_CORE_INTERRUPT_EN` — see [`set_core_interrupt_enable`].
    Primary,
    /// `reg_gpio_irq_risc0_en` / `FLD_IRQ_GPIO_RISC0_EN` (bit 21).
    Risc0,
    /// `reg_gpio_irq_risc1_en` / `FLD_IRQ_GPIO_RISC1_EN` (bit 22).
    Risc1,
}

impl GpioIrqSource {
    /// Base address of this source's per-port, one-byte-per-port,
    /// one-bit-per-pin enable register (`reg_gpio_irq_wakeup_en(i)` =
    /// `REG_ADDR8(0x587+((i>>8)<<3))`, `reg_gpio_irq_risc0_en(i)` =
    /// `REG_ADDR8(0x5b8+(i>>8))`, `reg_gpio_irq_risc1_en(i)` =
    /// `REG_ADDR8(0x5c0+(i>>8))`). Note `Primary` uses the same 8-byte
    /// per-port stride as every other per-pin digital field in this module
    /// (it is `reg(OFFSET_IRQ_WAKEUP_EN)`), while `Risc0`/`Risc1` use a
    /// contiguous one-byte-per-port array instead.
    const fn enable_register(self, port: Port) -> u32 {
        match self {
            GpioIrqSource::Primary => {
                REG_GPIO_BASE + ((port.index() as u32) << 3) + OFFSET_IRQ_WAKEUP_EN
            }
            GpioIrqSource::Risc0 => REG_GPIO_IRQ_RISC0_BASE + port.index() as u32,
            GpioIrqSource::Risc1 => REG_GPIO_IRQ_RISC1_BASE + port.index() as u32,
        }
    }

    /// This source's bit in `reg_irq_mask`/`reg_irq_src`
    /// (`FLD_IRQ_GPIO_EN`/`FLD_IRQ_GPIO_RISC0_EN`/`FLD_IRQ_GPIO_RISC1_EN`,
    /// `platform/chip_8258/register.h`). Delegates to [`crate::irq`]'s
    /// canonical bit table instead of repeating these three literals here
    /// — see that module's docs for the full `reg_irq_mask` layout.
    pub const fn global_mask(self) -> u32 {
        self.as_irq_source().mask()
    }

    /// This source's counterpart in [`crate::irq::IrqSource`], the
    /// generic CPU-IRQ-controller facade every `reg_irq_mask`/
    /// `reg_irq_src` access in this crate now goes through.
    const fn as_irq_source(self) -> crate::irq::IrqSource {
        match self {
            GpioIrqSource::Primary => crate::irq::IrqSource::GpioPrimary,
            GpioIrqSource::Risc0 => crate::irq::IrqSource::GpioRisc0,
            GpioIrqSource::Risc1 => crate::irq::IrqSource::GpioRisc1,
        }
    }
}

/// `reg_gpio_irq_risc0_en` base (`platform/chip_8258/register.h`): one byte
/// per port, contiguous (`0x5b8 + port_index`), unlike the 8-byte-stride
/// digital per-port block every other field in this module uses.
const REG_GPIO_IRQ_RISC0_BASE: u32 = super::mmio::REG_BASE + 0x5B8;
/// `reg_gpio_irq_risc1_en` base, same layout as [`REG_GPIO_IRQ_RISC0_BASE`].
const REG_GPIO_IRQ_RISC1_BASE: u32 = super::mmio::REG_BASE + 0x5C0;
/// `reg_gpio_wakeup_irq` (`platform/chip_8258/register.h`).
const REG_GPIO_WAKEUP_IRQ: u32 = super::mmio::REG_BASE + 0x5B5;
/// `FLD_GPIO_CORE_INTERRUPT_EN` (`reg_gpio_wakeup_irq` bit 3). Must be set
/// for [`GpioIrqSource::Primary`] to ever reach the core, per
/// `gpio_set_interrupt()`'s body — `Risc0`/`Risc1` do not touch this bit.
const FLD_GPIO_CORE_INTERRUPT_EN: u8 = 1 << 3;

/// Enable or disable this pin as a source of the given [`GpioIrqSource`].
/// See [`GpioIrqSource`] and [`set_wakeup_interrupt_enable`] (the
/// `Primary`-only convenience wrapper this generalizes).
#[cfg(target_arch = "tc32")]
pub fn set_source_interrupt_enable(pin: &Pin, source: GpioIrqSource, enable: bool) {
    let (port, _bit) = pin.port_and_bit();
    let addr = source.enable_register(port);
    let mask = pin.mask();
    super::mmio::with_irqs_disabled(|| unsafe {
        let value = r8(addr);
        w8(addr, if enable { value | mask } else { value & !mask });
    });
}

/// Set (`enable = true`) or clear the shared `FLD_GPIO_CORE_INTERRUPT_EN`
/// gate that [`GpioIrqSource::Primary`] needs in addition to its
/// `reg_irq_mask` bit — see [`GpioIrqSource::Primary`]'s docs and the
/// vendor `gpio_set_interrupt()` body this is transcribed from.
#[cfg(target_arch = "tc32")]
pub fn set_core_interrupt_enable(enable: bool) {
    super::mmio::with_irqs_disabled(|| unsafe {
        let value = r8(REG_GPIO_WAKEUP_IRQ);
        w8(
            REG_GPIO_WAKEUP_IRQ,
            if enable {
                value | FLD_GPIO_CORE_INTERRUPT_EN
            } else {
                value & !FLD_GPIO_CORE_INTERRUPT_EN
            },
        );
    });
}

/// Enable or disable one or more [`GpioIrqSource`] bits in `reg_irq_mask`
/// (`0x640`). Delegates to [`crate::irq::set_enabled`], which performs the
/// masked read-modify-write under `REG_IRQ_EN=0` (this crate's established
/// convention — see `radio::hw::mask_cpu_rx_irq`/`enable_cpu_rx_irq`) so a
/// half-updated 32-bit mask is never visible to an interrupted vector.
#[cfg(target_arch = "tc32")]
pub fn set_global_interrupt_enable(source: GpioIrqSource, enable: bool) {
    crate::irq::set_enabled(source.as_irq_source(), enable);
}

/// `true` if `source`'s bit is currently latched in `reg_irq_src` (`0x648`).
#[cfg(target_arch = "tc32")]
pub fn interrupt_pending(source: GpioIrqSource) -> bool {
    crate::irq::pending(source.as_irq_source())
}

/// Acknowledge `source`'s pending bit in `reg_irq_src`. `reg_irq_src` is
/// write-to-clear per bit (writing a 0 bit leaves the corresponding source
/// untouched — see `radio::hw::clear_cpu_rx_irq_sources`'s identical single
/// `w32` for the RF/DMA sources), so this cannot disturb unrelated pending
/// bits. Delegates to [`crate::irq::clear_pending`].
#[cfg(target_arch = "tc32")]
pub fn clear_interrupt_pending(source: GpioIrqSource) {
    crate::irq::clear_pending(source.as_irq_source());
}

/// Configure a pin's internal pull resistor.
///
/// See the module-level docs for the confidence caveat on this specific
/// path (addresses are vendor-header-verified; the 2-bit packing was
/// confirmed by disassembly rather than open source).
#[cfg(target_arch = "tc32")]
pub fn set_pull(pin: &Pin, pull: Pull) -> Result<(), GpioError> {
    let (addr, shift) = pin
        .pull_location()
        .ok_or(GpioError::PullNotSupportedOnPort)?;
    let mask = 0x03u8 << shift;
    super::mmio::with_irqs_disabled(|| {
        let value = super::mmio::analog_read(addr)?;
        let updated = (value & !mask) | ((pull as u8) << shift);
        super::mmio::analog_write(addr, updated)?;
        Ok(())
    })
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

    #[test]
    fn peripheral_routes_reject_wrong_pins() {
        assert!(supports_function(&Pin::new(Port::C, 0), PinFunction::I2c));
        assert!(supports_function(&Pin::new(Port::B, 7), PinFunction::Spi));
        assert!(!supports_function(&Pin::new(Port::B, 5), PinFunction::I2c));
        assert!(!supports_function(&Pin::new(Port::C, 5), PinFunction::Spi));
    }

    #[test]
    fn tb04_rgb_pwm_routes_have_expected_selectors() {
        assert_eq!(
            function_selector(&Pin::new(Port::C, 1), PinFunction::Pwm0),
            Some(2)
        );
        assert_eq!(
            function_selector(&Pin::new(Port::B, 5), PinFunction::Pwm5),
            Some(1)
        );
        assert_eq!(
            function_selector(&Pin::new(Port::C, 4), PinFunction::Pwm2),
            Some(0)
        );
        assert_eq!(
            function_selector(&Pin::new(Port::C, 4), PinFunction::Pwm3),
            None
        );
    }

    #[test]
    fn uart_pb1_tx_and_pb7_rx_routes_have_cross_checked_selectors() {
        // PB1/PB7 are the pre-existing, most heavily validated routes;
        // kept as their own test so a regression here is unambiguous.
        assert_eq!(
            function_selector(&Pin::new(Port::B, 1), PinFunction::Uart),
            Some(0b01)
        );
        assert_eq!(
            function_selector(&Pin::new(Port::B, 7), PinFunction::Uart),
            Some(0b10)
        );
        assert!(supports_function(&Pin::new(Port::B, 1), PinFunction::Uart));
        assert!(supports_function(&Pin::new(Port::B, 7), PinFunction::Uart));
    }

    #[test]
    fn uart_all_documented_tx_routes_have_disassembly_verified_selectors() {
        // Every TX pin in uart.h's UART_TxPinDef, selector derived from the
        // compiled gpio_set_func() object (see module docs).
        let tx_routes = [
            (Port::A, 2, 0b01u8),
            (Port::B, 1, 0b01),
            (Port::C, 2, 0b01),
            (Port::D, 0, 0b10),
            (Port::D, 3, 0b10),
            (Port::D, 7, 0b10),
        ];
        for (port, bit, expected) in tx_routes {
            assert_eq!(
                function_selector(&Pin::new(port, bit), PinFunction::Uart),
                Some(expected),
                "TX route {port:?}{bit}"
            );
            assert!(supports_function(&Pin::new(port, bit), PinFunction::Uart));
        }
    }

    #[test]
    fn uart_all_documented_rx_routes_have_disassembly_verified_selectors() {
        // Every RX pin in uart.h's UART_RxPinDef, selector derived from the
        // compiled gpio_set_func() object (see module docs).
        let rx_routes = [
            (Port::A, 0, 0b10u8),
            (Port::B, 0, 0b01),
            (Port::B, 7, 0b10),
            (Port::C, 3, 0b01),
            (Port::C, 5, 0b01),
            (Port::D, 6, 0b01),
        ];
        for (port, bit, expected) in rx_routes {
            assert_eq!(
                function_selector(&Pin::new(port, bit), PinFunction::Uart),
                Some(expected),
                "RX route {port:?}{bit}"
            );
            assert!(supports_function(&Pin::new(port, bit), PinFunction::Uart));
        }
    }

    #[test]
    fn uart_route_rejects_pins_outside_uart_h() {
        // PC1 sits right next to the PC2 TX route in the mux decision
        // tree, but tracing gpio_set_func(0x0202 /* PC1 */, AS_UART)
        // resolves to the "no match" default (selector-writing code path
        // never taken), matching PC1's absence from uart.h. PA1 is not
        // documented in uart.h's TX/RX enums either, so this HAL rejects
        // it too — note that gpio_set_func's silicon-level decision tree
        // does resolve *some* selector for PA1 under AS_UART; this HAL
        // deliberately stays within uart.h's documented pin list rather
        // than routing undocumented pins the vendor SDK itself never
        // exposes as UART pins, per the "reject anything not proven by an
        // in-scope, documented route" policy.
        assert_eq!(
            function_selector(&Pin::new(Port::C, 1), PinFunction::Uart),
            None
        );
        assert!(!supports_function(&Pin::new(Port::C, 1), PinFunction::Uart));
        assert_eq!(
            function_selector(&Pin::new(Port::A, 1), PinFunction::Uart),
            None
        );
        assert!(!supports_function(&Pin::new(Port::A, 1), PinFunction::Uart));
    }

    #[test]
    fn gpio_irq_source_global_mask_matches_register_h_bits() {
        // FLD_IRQ_GPIO_EN/FLD_IRQ_GPIO_RISC0_EN/FLD_IRQ_GPIO_RISC1_EN,
        // platform/chip_8258/register.h.
        assert_eq!(GpioIrqSource::Primary.global_mask(), 1 << 18);
        assert_eq!(GpioIrqSource::Risc0.global_mask(), 1 << 21);
        assert_eq!(GpioIrqSource::Risc1.global_mask(), 1 << 22);
    }

    #[test]
    fn gpio_irq_source_enable_register_matches_register_h_formula() {
        // reg_gpio_irq_wakeup_en(i) = REG_ADDR8(0x587+((i>>8)<<3)): same
        // 8-byte-stride per-port block as every other digital field.
        assert_eq!(GpioIrqSource::Primary.enable_register(Port::A), 0x800587);
        assert_eq!(GpioIrqSource::Primary.enable_register(Port::B), 0x80058F);
        // reg_gpio_irq_risc0_en(i) = REG_ADDR8(0x5b8 + (i>>8)): contiguous
        // one byte per port, not the 8-byte stride.
        assert_eq!(GpioIrqSource::Risc0.enable_register(Port::A), 0x8005B8);
        assert_eq!(GpioIrqSource::Risc0.enable_register(Port::B), 0x8005B9);
        assert_eq!(GpioIrqSource::Risc0.enable_register(Port::E), 0x8005BC);
        // reg_gpio_irq_risc1_en(i) = REG_ADDR8(0x5c0 + (i>>8)).
        assert_eq!(GpioIrqSource::Risc1.enable_register(Port::A), 0x8005C0);
        assert_eq!(GpioIrqSource::Risc1.enable_register(Port::B), 0x8005C1);
    }

    #[test]
    fn gpio_core_interrupt_enable_bit_matches_register_h() {
        // reg_gpio_wakeup_irq (0x5b5), FLD_GPIO_CORE_INTERRUPT_EN = BIT(3).
        assert_eq!(REG_GPIO_WAKEUP_IRQ, 0x8005B5);
        assert_eq!(FLD_GPIO_CORE_INTERRUPT_EN, 0x08);
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
