//! General-purpose, non-DMA TLSR8258 UART: every documented TX/RX route,
//! typed optional RTS/CTS hardware flow control, bounded polled interrupt
//! support, a validated baud-divider search, and bounded, nonblocking byte
//! I/O.
//!
//! # The smart-plug profile
//!
//! This driver started as a BL0942/TS011F-style smart-plug-metering-IC
//! route — PB1 TX, PB7 RX, 4800 baud, 8 data bits, no parity, one stop bit
//! — and that specific pin/baud/framing combination remains fully
//! supported and is this module's most heavily cross-checked
//! configuration (see [`tests::baud_divider_is_exact_for_the_bl0942_route`]
//! and the PB1/PB7-specific pinmux tests in `gpio.rs`). It is **not**
//! silicon-validated on real hardware by this crate — treat it as a
//! starting point requiring bring-up on an actual TLSR8258 + BL0942 board,
//! not a proven-on-hardware driver.
//!
//! # Register evidence
//!
//! `platform/chip_8258/register.h`'s `uart registers: 0x90` block
//! (`reg_uart_data_buf0..3`, `reg_uart_clk_div`, `reg_uart_ctrl0/1/2`,
//! `reg_uart_rx_timeout0/1`, `reg_uart_buf_cnt`, `reg_uart_status0/1`, and
//! every `FLD_UART_*` bit used below) are transcribed directly from that
//! official, Apache-2.0, shipped-as-C-source header — not a compiled
//! object — so the register map and bit positions in this module are held
//! at full confidence, the same tier as `gpio.rs`'s per-pin digital
//! registers. `reg_clk_en0`/`reg_rst0`'s `FLD_CLK0_UART_EN`/`FLD_RST0_UART`
//! (bit 2 of each) are handled by the crate-wide, generic
//! `crate::reset::Peripheral::Uart` facade instead of being duplicated
//! locally — see `reset.rs` for that evidence and
//! `enable_peripheral`/`tests::clock_and_reset_bits_match_register_h` below
//! for the local cross-check.
//!
//! `platform/chip_8258/uart.h` (also open C source) additionally documents,
//! by name in its own doc comments:
//! - the worked `sys_clk`/baud-rate/`g_uart_div`/`g_bwpc` table this
//!   module's [`compute_baud_divider`] reproduces exactly (`24Mhz`/`9600`/
//!   `249`/`9`, `24Mhz`/`19200`/`124`/`9`, `24Mhz`/`115200`/`12`/`15` — see
//!   `tests::baud_divider_matches_vendor_worked_examples`);
//! - `UART_TX_PB1`/`UART_RX_PB7` in its `UART_TxPinDef`/`UART_RxPinDef`
//!   enums, confirming PB1/PB7 are documented TX/RX routes (the pin *mux
//!   selector value* itself is a `gpio.rs` concern — see that module's docs
//!   for the separate cross-check behind `PinFunction::Uart`'s PB1/PB7
//!   entries); and
//! - `uart_ndma_send_byte`'s doc comment, which states TX must "cycle the
//!   four registers 0x90 0x91 0x92 0x93" in non-DMA mode — exactly what
//!   [`Uart::try_write`]'s `tx_index` does.
//!
//! # TX/RX pin routes
//!
//! All six documented TX pins (PA2/PB1/PC2/PD0/PD3/PD7,
//! `UART_TxPinDef`) and all six documented RX pins (PA0/PB0/PB7/PC3/PC5/
//! PD6, `UART_RxPinDef`) are accepted by [`validate_pins`]/[`Uart::new`].
//! The pin *mux selector* for each of the twelve is `gpio.rs`'s concern —
//! see that module's docs for the disassembly-derived evidence — this
//! module only fixes the *set* of pins it will accept for each role.
//!
//! `uart_gpio_set()`'s disassembly (see `gpio.rs` module docs for the
//! toolchain/method) shows it pulls up **both** the TX and RX pin
//! (`PM_PIN_PULLUP_10K`) and enables input on both, not just RX as an
//! earlier version of this module did — [`Uart::configure_pins`] now
//! matches that exactly. It does not touch either pin's output-enable bit;
//! this module still sets it defensively (TX enabled, RX disabled) as
//! belt-and-suspenders, since `gpio_set_output_en`'s own header doc
//! ("`GPIO_OEN`... `1`: enable") describes a plain-GPIO-mode bit that a
//! non-GPIO mux selection is expected to override anyway — doing so
//! matches the pre-existing, unaudited PB1/PB7 behavior this task requires
//! to stay unchanged, and is not known to conflict with the vendor's own
//! (pin-agnostic) behavior.
//!
//! # RTS/CTS hardware flow control
//!
//! RTS (PA4/PB3/PB6/PC0, `UART_RtsPinDef`) and CTS (PA3/PB2/PC4/PD1,
//! `UART_CtsPinDef`) are both optional (`Config::rts`/`Config::cts`,
//! [`RtsConfig`]/[`CtsConfig`]). Disassembling `uart_set_rts`/
//! `uart_set_cts`/`uart_set_rts_level` from the same official
//! `libdrivers_8258.a:uart.o` shows neither ever calls `gpio_set_func`:
//! both only pull the pin up (10K) and enable input (RTS) — CTS does the
//! same — before touching `reg_uart_ctrl1`/`reg_uart_ctrl2`. There is
//! therefore no mux selector to derive or omit for these eight pins; they
//! are configured with this module's own [`crate::gpio::set_pull`]/
//! [`crate::gpio::set_input_enable`] calls directly, consuming a plain
//! [`Pin`] token.
//!
//! The exact bit-level procedure below is transcribed from that same
//! disassembly (compiled, not inline, functions — held at the
//! disassembly-verified tier, like [`Uart::clear_rx_error`]):
//! - `uart_set_rts(Enable, Mode, Thresh, Invert, pin)`: clears
//!   `FLD_UART_CTRL2_RTS_EN` first; if `Enable`, pulls up/input-enables
//!   `pin`; sets/clears `FLD_UART_CTRL2_RTS_MANUAL_EN` from `Mode`;
//!   sets/clears `FLD_UART_CTRL2_RTS_PARITY` from `Invert`; masks in
//!   `Thresh & 0xF` as `FLD_UART_CTRL2_RTS_TRIG_LVL`; and, only if
//!   `Enable`, finally sets `FLD_UART_CTRL2_RTS_EN`. **Discrepancy from
//!   `uart.h`'s own doc comment:** the header says `Invert` applies "only
//!   for auto mode", but the compiled code applies it identically in both
//!   the `Auto` and `Manual` branches (they jump into the same
//!   invert-handling block) — [`RtsConfig::invert`] therefore documents
//!   the disassembly-observed behavior (applies in both modes), not the
//!   header's narrower claim.
//! - `uart_set_rts_level(Polarity)` sets/clears
//!   `FLD_UART_CTRL2_RTS_MANUAL_VAL` directly — see
//!   [`Uart::set_rts_manual_level`].
//! - `uart_set_cts(Enable, Select, pin)`: if `Enable`, pulls up/
//!   input-enables `pin`, then sets `FLD_UART_CTRL1_CTS_EN`; if not
//!   `Enable`, clears it instead. Either way, `FLD_UART_CTRL1_CTS_SELECT`
//!   is set or cleared from `Select` afterwards (configured even when
//!   `Enable` is false, matching `uart_set_rts`'s same pattern of
//!   configuring fields unconditionally while only the final `*_EN` bit
//!   itself is gated).
//!
//! # Polled, non-DMA UART interrupt support
//!
//! `reg_uart_ctrl0`'s `FLD_UART_RX_IRQ_EN`/`FLD_UART_TX_IRQ_EN` (bits 6/7)
//! and `reg_uart_ctrl3`'s trigger-level nibbles are open `register.h`
//! macros (full confidence). Disassembling `uart_irq_enable(rx_en,
//! tx_en)` additionally shows it toggles the *global* CPU IRQ enable bit
//! for the whole peripheral (`reg_irq_mask`'s `FLD_IRQ_UART_EN`, already
//! modeled crate-wide as [`crate::irq::IrqSource::Uart`]) to `rx_en |
//! tx_en` as a side effect, and `uart_mask_error_irq_enable()` does the
//! same (`reg_uart_rx_timeout1`'s `FLD_UART_MASK_ERR_IRQ` plus the same
//! global bit). [`Uart::set_rx_irq_enabled`]/[`Uart::set_tx_irq_enabled`]/
//! [`Uart::set_error_irq_masked`] reproduce that same fold-in via
//! [`Uart::sync_global_irq_enable`], each keeping [`crate::irq::IrqSource::Uart`]
//! enabled for as long as *any* of the three local reasons still wants it,
//! rather than one caller's `disable` silently turning off another's IRQ
//! source. This module does not install an ISR or own a global queue —
//! `crate::irq::pending`/`clear_pending` (or a future ISR elsewhere in
//! this crate) is the caller's responsibility, matching this crate's
//! "consume, don't own" IRQ-facade convention for a polled peripheral like
//! this one.
//!
//! `uart_ndmairq_get()`'s doc comment ("get the status of uart irq...
//! 0x9d[3]") and `reg_uart_status0`'s bare `FLD_UART_IRQ_FLAG` macro
//! (bit 3) show the non-DMA trigger-level IRQ condition is a plain level
//! flag with no vendor-documented or disassembly-observed clear
//! procedure (unlike bit 6/7's documented `uart_clear_parity_error`
//! read-modify-write) — [`Uart::poll_events`] therefore only reads it,
//! and this module does not invent a write-clear for it. `reg_uart_status1`
//! (`FLD_UART_TX_DONE`/`FLD_UART_TX_BUF_IRQ`/`FLD_UART_RX_DONE`/
//! `FLD_UART_RX_BUF_IRQ`) is read the same way, for the same reason —
//! `modern-tc32/tlsr82xx` also only ever reads `FLD_UART_TX_DONE`, never
//! writes it (see [`Uart::flush`], pre-existing).
//!
//! # DMA is explicitly unsupported
//!
//! `platform/chip_8258/dma.h` names two dedicated UART DMA channels,
//! `DMA0_UART_RX` and `DMA1_UART_TX`, and `reg_uart_ctrl0`'s
//! `FLD_UART_RX_DMA_EN`/`FLD_UART_TX_DMA_EN` (bits 4/5, left permanently
//! clear by this module) are the peripheral-side enables for them. This
//! module does not implement either channel:
//! - **Channel ownership**: DMA0/DMA1 are two of a small, shared pool of
//!   TLSR8258 DMA channels also used (or reservable) by other peripherals
//!   in this crate's scope (e.g. the radio's own RX/TX DMA descriptors in
//!   `radio/mod.rs`); wiring up UART DMA would need a crate-wide DMA
//!   channel *ownership* type (so two drivers cannot silently claim the
//!   same channel), which does not exist yet and is out of this module's
//!   scope to invent unilaterally.
//! - **Linker/SRAM requirements**: DMA descriptors and buffers must live
//!   in a fixed, hardware-addressable SRAM window your linker script must
//!   reserve and this crate must be able to prove buffers actually fall
//!   inside at runtime (mirroring [`crate::mmio::sram_contains`], already
//!   used by [`crate::adc`]'s own DMA sample buffer) — no such reserved
//!   `.uart_dma`-style region exists in this crate's linker scripts today.
//! - **No unsafe global buffers**: per this crate's convention (see
//!   [`crate::radio`]'s and [`crate::adc`]'s DMA buffer designs), a DMA
//!   ring buffer would need to be a checked, owned Rust value (or
//!   plugged into an existing checked one), never a bare `static mut`
//!   slice reachable from an ISR without synchronization. None of that
//!   scaffolding is added here.
//!
//! Revisit only alongside a crate-wide DMA-channel-ownership type; see
//! "Why non-DMA" below for why this module's actual target baud rates do
//! not need it in the meantime.
//!
//! # Why this stays polled even though DMA is available
//!
//! `platform/chip_8258/dma.h` names two dedicated UART DMA channels
//! (`DMA0_UART_RX`, `DMA1_UART_TX`), so hardware DMA *is* available for
//! this peripheral in general — see "DMA is explicitly unsupported" above
//! for why this module still does not use them structurally. Separately,
//! even the highest documented UART baud rate this module supports
//! (115200) is many orders of magnitude slower than this core's
//! instruction rate, and [`Uart::try_read`]/[`Uart::try_write`] are
//! already nonblocking, single-poll, bounded operations designed to be
//! called from a cooperative main loop (this crate has no RTOS/threads to
//! block) — so polling costs no meaningful throughput or latency at any
//! route this module documents. Revisit this decision only if a future
//! route needs sustained UART throughput approaching what would make
//! single-byte polling a measurable CPU burden (roughly two to three
//! orders of magnitude faster than 115200 baud).
//!
//! # Explicit confidence gaps
//!
//! - `uart.h`'s own `g_bwpc` doc comment is truncated mid-sentence in the
//!   shipped header ("bitwidth, should be set to larger than" — no number
//!   follows in the file as published). [`MIN_BWPC`]/[`MAX_BWPC`] instead
//!   follow the independently reasoned `3..=15` search range from the
//!   openly available `modern-tc32/tlsr82xx` project's own
//!   `compute_baud_params`, which is itself consistent with the vendor
//!   table's own `bwpc` values (9, 9, 15) and the register's 4-bit field
//!   width. Cross-checked, not vendor-specified.
//! - RX data-buffer cycling (`rx_index` wrapping `0..=3` across
//!   `reg_uart_data_buf0..3`, mirroring the vendor-documented TX side) is
//!   symmetric by hardware construction (both directions share the same
//!   four physical byte registers) and is cross-checked against
//!   `modern-tc32/tlsr82xx`'s `Uart::read_byte`, which does the same;
//!   `uart.h` itself only narrates the TX index by name (`uart_TxIndex`).
//! - `reg_uart_status0`'s `FLD_UART_RX_ERR_FLAG` (bit 7) is the hardware's
//!   only combined framing/parity/overrun indicator — `register.h` does
//!   not split these into separate bits, so [`UartError::RxError`] does not
//!   either. `uart.h`'s own `uart_clear_parity_error()`/
//!   `uart_is_parity_error()` are compiled (not inline) functions, so the
//!   exact clear procedure was therefore checked against the shipped
//!   `libdrivers_8258.a:uart.o`: `uart_clear_parity_error()` reads
//!   `reg_uart_status0`, ORs `FLD_UART_CLEAR_RX_FLAG` (bit 6), and writes
//!   the combined byte back. `uart.h` additionally requires non-DMA reads
//!   to restart from data register `0x90` after the clear, so
//!   [`Uart::clear_rx_error`] reproduces the register write and resets its
//!   RX ring index to zero. [`Uart::reset`] remains the fail-closed fallback
//!   if hardware reports the error again; see both functions' own docs.
//! - The non-DMA TX backpressure threshold ([`TX_FIFO_BACKPRESSURE_THRESHOLD`])
//!   and its exact boundary ([`tx_backpressure`]) are cross-checked
//!   against `modern-tc32/tlsr82xx`'s `UART_TX_FIFO_MAX_COUNT_NDMA` and its
//!   `write_byte`/`try_write_byte`, not an inline vendor constant/function.
//!
//! This module never busy-waits unboundedly: [`Uart::try_read`] and
//! [`Uart::try_write`] are single-poll and nonblocking, and [`Uart::flush`]
//! takes an explicit, caller-supplied iteration bound.

use crate::gpio::{Pin, Port};
#[cfg(target_arch = "tc32")]
use crate::gpio::{PinFunction, Pull};
#[cfg(target_arch = "tc32")]
use crate::irq::IrqSource;
#[cfg(target_arch = "tc32")]
use crate::mmio::{r8, w8, w16};

const REG_UART_BASE: u32 = crate::mmio::REG_BASE + 0x90;
const REG_UART_DATA_BUF0: u32 = REG_UART_BASE;
const REG_UART_CLK_DIV: u32 = REG_UART_BASE + 0x04;
const REG_UART_CTRL0: u32 = REG_UART_BASE + 0x06;
const REG_UART_CTRL1: u32 = REG_UART_BASE + 0x07;
/// Low byte of the 16-bit `reg_uart_ctrl2` pair: RTS trigger level/invert/
/// manual-value/manual-enable/enable. Addressed as an independent 8-bit
/// register so this module's RTS writes never need to read back (and risk
/// racing) the unrelated IRQ-trigger-level byte at [`REG_UART_CTRL3`].
const REG_UART_CTRL2: u32 = REG_UART_BASE + 0x08;
/// High byte of the 16-bit `reg_uart_ctrl2` pair, `register.h`'s
/// `reg_uart_ctrl3` alias: non-DMA RX/TX IRQ trigger levels (low/high
/// nibble respectively), see `uart_ndma_irq_triglevel()`.
const REG_UART_CTRL3: u32 = REG_UART_BASE + 0x09;
const REG_UART_RX_TIMEOUT0: u32 = REG_UART_BASE + 0x0A;
const REG_UART_RX_TIMEOUT1: u32 = REG_UART_BASE + 0x0B;
const REG_UART_BUF_CNT: u32 = REG_UART_BASE + 0x0C;
const REG_UART_STATUS0: u32 = REG_UART_BASE + 0x0D;
const REG_UART_STATUS1: u32 = REG_UART_BASE + 0x0E;

const FLD_UART_BPWC: u8 = 0x0F;
/// `reg_uart_ctrl0` bits 4..7 (`RX_DMA_EN`/`TX_DMA_EN`/`RX_IRQ_EN`/
/// `TX_IRQ_EN`) — everything this module's own [`configure_registers`]
/// must *not* clobber when reprogramming `FLD_UART_BPWC` in bits 0..3.
/// `uart_init()`'s disassembly does a scoped clear-then-OR of only bits
/// 0..3 for exactly this reason (it never touches bits 4..7), which this
/// mask reproduces.
const FLD_UART_CTRL0_UPPER_MASK: u8 = 0xF0;
const FLD_UART_RX_IRQ_EN: u8 = 1 << 6;
const FLD_UART_TX_IRQ_EN: u8 = 1 << 7;
const FLD_UART_CLK_DIV_MASK: u16 = 0x7FFF;
const FLD_UART_CLK_DIV_EN: u16 = 1 << 15;

const FLD_UART_CTRL1_CTS_SELECT: u8 = 1 << 0;
const FLD_UART_CTRL1_CTS_EN: u8 = 1 << 1;
const FLD_UART_CTRL1_PARITY_EN: u8 = 1 << 2;
const FLD_UART_CTRL1_PARITY_POLARITY: u8 = 1 << 3;
/// Bits this module's own [`configure_registers`] owns in `reg_uart_ctrl1`
/// (parity, bits 2..3, and stop bits, bits 4..5) — the complement of this
/// mask (`CTS_SELECT`/`CTS_EN`, bits 0..1, and `TTL`/`LOOPBACK`, bits 6..7)
/// must survive a baud/parity reprogram. `uart_init()`'s disassembly does
/// exactly this: it clears/sets only the parity and stop-bit fields,
/// leaving every other `reg_uart_ctrl1` bit alone.
const FLD_UART_CTRL1_OWNED_MASK: u8 = 0x3C;

const FLD_UART_CTRL2_RTS_TRIG_LVL_MASK: u8 = 0x0F;
const FLD_UART_CTRL2_RTS_PARITY: u8 = 1 << 4;
const FLD_UART_CTRL2_RTS_MANUAL_VAL: u8 = 1 << 5;
const FLD_UART_CTRL2_RTS_MANUAL_EN: u8 = 1 << 6;
const FLD_UART_CTRL2_RTS_EN: u8 = 1 << 7;

/// `reg_uart_ctrl3` bits 0..3, `uart_ndma_irq_triglevel`'s RX argument.
const FLD_UART_CTRL3_RX_IRQ_TRIG_LVL_MASK: u8 = 0x0F;
/// `reg_uart_ctrl3` bits 4..7, `uart_ndma_irq_triglevel`'s TX argument.
const FLD_UART_CTRL3_TX_IRQ_TRIG_LVL_SHIFT: u8 = 4;

const FLD_UART_TIMEOUT_MUL_MASK: u8 = 0b11;
const UART_TIMEOUT_MUL_3X_BWPC: u8 = 0b01;
/// `reg_uart_rx_timeout1` bit 7, `FLD_UART_MASK_ERR_IRQ` in
/// `platform/chip_8258/register.h`. Disassembling
/// `uart_mask_error_irq_enable()` shows it ORs this bit into
/// `reg_uart_rx_timeout1` *and* enables [`IrqSource::Uart`] at the CPU
/// level — see [`Uart::set_error_irq_masked`].
const FLD_UART_MASK_ERR_IRQ: u8 = 1 << 7;

const FLD_UART_RX_BUF_CNT: u8 = 0x0F;
const FLD_UART_TX_BUF_CNT: u8 = 0xF0;
/// Non-DMA TX backpressure threshold — see the module docs' confidence gap
/// note (cross-checked against `modern-tc32/tlsr82xx`, not vendor-header).
/// `modern-tc32/tlsr82xx`'s own `write_byte`/`try_write_byte` spin `while
/// tx_fifo_count() > UART_TX_FIFO_MAX_COUNT_NDMA`, i.e. a count *equal* to
/// this threshold (7 of the FIFO's 8 physical slots in use) is still safe
/// to write into; only a strictly *greater* count blocks. [`tx_backpressure`]
/// is the single, host-tested place that decision is made.
const TX_FIFO_BACKPRESSURE_THRESHOLD: u8 = 7;

/// `reg_uart_status0` bit 3, `FLD_UART_IRQ_FLAG` in `register.h` — the
/// non-DMA RX/TX trigger-level condition `uart_ndma_irq_triglevel`
/// configures and `uart_ndmairq_get()` reads. See the module docs'
/// "Polled, non-DMA UART interrupt support" section for why this is
/// modeled as read-only (no vendor-documented or disassembly-observed
/// clear procedure exists for it).
const FLD_UART_IRQ_FLAG: u8 = 1 << 3;
const FLD_UART_RX_ERR_FLAG: u8 = 1 << 7;
/// `reg_uart_status0` bit 6, `platform/chip_8258/register.h`. The header
/// names this bit *both* `FLD_UART_CLEAR_RX_FLAG` and, as part of the
/// adjacent 3-bit read-only `FLD_UART_WBCNT` field (bits 4..6), the top bit
/// of the hardware's own write-buffer-count. The shipped
/// `libdrivers_8258.a:uart.o` resolves the write semantics:
/// `uart_clear_parity_error()` reads this register, ORs bit 6, and writes
/// the combined value back. [`Uart::clear_rx_error`] reproduces that exact
/// read-modify-write.
const FLD_UART_CLEAR_RX_FLAG: u8 = 1 << 6;
const FLD_UART_TX_DONE: u8 = 1 << 0;
/// `reg_uart_status1` bits 1..3 (`register.h`). Read-only, same rationale
/// as [`FLD_UART_IRQ_FLAG`] — see [`Uart::poll_events`].
const FLD_UART_TX_BUF_IRQ: u8 = 1 << 1;
const FLD_UART_RX_DONE: u8 = 1 << 2;
const FLD_UART_RX_BUF_IRQ: u8 = 1 << 3;

const DATA_BUF_RING_LEN: u8 = 4;
const DATA_BUF_RING_MASK: u8 = DATA_BUF_RING_LEN - 1;

/// Lowest `bwpc` this module's search considers. See the module docs'
/// confidence gap note on the vendor's own truncated guidance here.
const MIN_BWPC: u8 = 3;
/// Highest representable `bwpc` (`FLD_UART_BPWC` is a 4-bit field).
const MAX_BWPC: u8 = 15;

/// UART parity selection (`reg_uart_ctrl1`'s `FLD_UART_CTRL1_PARITY_EN`/
/// `FLD_UART_CTRL1_PARITY_POLARITY`, `platform/chip_8258/register.h`; the
/// comment there states "1:odd parity 0:even parity" for the polarity bit).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    None,
    Even,
    Odd,
}

/// Stop-bit selection, matching `uart.h`'s `UART_StopBitTypeDef` both in
/// name and in the raw `reg_uart_ctrl1` bit-4/5 field value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StopBits {
    One = 0x00,
    OneAndHalf = 0x10,
    Two = 0x20,
}

/// UART RTS mode, matching `uart.h`'s `UART_RTSModeTypeDef` both in name
/// and discriminant value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RtsMode {
    Auto = 0,
    Manual = 1,
}

/// Optional RTS (request-to-send) hardware flow control, applied to one of
/// the four documented RTS pins (PA4/PB3/PB6/PC0, `uart.h`'s
/// `UART_RtsPinDef`). See the module docs' "RTS/CTS hardware flow
/// control" section for the disassembly evidence behind every field here.
#[derive(Debug, PartialEq, Eq)]
pub struct RtsConfig {
    /// Consumed (moved in) to prove exclusive ownership, matching
    /// `Config::tx`/`Config::rx`.
    pub pin: Pin,
    pub mode: RtsMode,
    /// `FLD_UART_CTRL2_RTS_TRIG_LVL` — only the low 4 bits are wired to
    /// hardware; higher bits are masked off silently, matching
    /// `uart_set_rts`'s own `Thresh & 0xF`.
    pub threshold: u8,
    /// `FLD_UART_CTRL2_RTS_PARITY`. **Applies in both [`RtsMode::Auto`]
    /// and [`RtsMode::Manual`]** — see the module docs for why this is
    /// broader than `uart.h`'s own doc comment ("only for auto mode").
    pub invert: bool,
}

/// Optional CTS (clear-to-send) hardware flow control, applied to one of
/// the four documented CTS pins (PA3/PB2/PC4/PD1, `uart.h`'s
/// `UART_CtsPinDef`). See the module docs' "RTS/CTS hardware flow
/// control" section for the disassembly evidence behind every field here.
#[derive(Debug, PartialEq, Eq)]
pub struct CtsConfig {
    /// Consumed (moved in) to prove exclusive ownership, matching
    /// `Config::tx`/`Config::rx`.
    pub pin: Pin,
    /// `FLD_UART_CTRL1_CTS_SELECT` — the CTS input level that stops TX
    /// (`uart.h`: "when CTS's input equals to select, tx will be
    /// stopped").
    pub stop_level: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartError {
    /// No `(div, bwpc)` pair reproduces `baud_rate` within this module's
    /// search range (`baud_rate` or `system_clock_hz` is zero, or the
    /// requested baud rate cannot fit the 15-bit divider at this clock).
    InvalidBaudRate,
    /// `tx`/`rx` are not both drawn from `uart.h`'s documented
    /// `UART_TxPinDef`/`UART_RxPinDef` pin sets, or the pin mux/pull
    /// configuration failed.
    InvalidPins,
    /// `Config::rts`'s pin is not one of `uart.h`'s documented
    /// `UART_RtsPinDef` pins, or its pull/input configuration failed.
    InvalidRtsPin,
    /// `Config::cts`'s pin is not one of `uart.h`'s documented
    /// `UART_CtsPinDef` pins, or its pull/input configuration failed.
    InvalidCtsPin,
    /// `reg_uart_status0`'s combined framing/parity/overrun flag was set.
    /// Already acknowledged (see [`Uart::clear_rx_error`]'s confidence
    /// caveat) by the time this is returned.
    RxError,
    /// [`Uart::flush`]'s caller-supplied iteration bound elapsed before
    /// `FLD_UART_TX_DONE` was observed.
    FlushTimeout,
}

/// Configuration for [`Uart::new`]. `tx`/`rx` (and `rts`/`cts`'s pins, if
/// present) are consumed (moved in) to prove exclusive ownership of those
/// pads for the `Uart`'s lifetime, matching this crate's
/// `SpiMaster`/`I2cMaster` convention.
#[derive(Debug, PartialEq, Eq)]
pub struct Config {
    /// Any of `uart.h`'s six documented TX pins: PA2/PB1/PC2/PD0/PD3/PD7.
    pub tx: Pin,
    /// Any of `uart.h`'s six documented RX pins: PA0/PB0/PB7/PC3/PC5/PD6.
    pub rx: Pin,
    pub baud_rate: u32,
    pub system_clock_hz: u32,
    pub parity: Parity,
    pub stop_bits: StopBits,
    /// `None` leaves RTS permanently disabled (`FLD_UART_CTRL2_RTS_EN`
    /// clear).
    pub rts: Option<RtsConfig>,
    /// `None` leaves CTS permanently disabled (`FLD_UART_CTRL1_CTS_EN`
    /// clear).
    pub cts: Option<CtsConfig>,
}

/// Search `bwpc` in `[MIN_BWPC, MAX_BWPC]` and, for each, both the divider
/// that floors `system_clock_hz / (baud_rate * (bwpc + 1))` and the next
/// divider up, for the `(div, bwpc)` pair whose
/// `system_clock_hz / ((div + 1) * (bwpc + 1))` is closest to `baud_rate`.
///
/// `actual_baud(div)` is monotonically *decreasing* in `div` for a fixed
/// `bwpc`, so the floored divider always yields an `actual_baud_rate >=
/// baud_rate` (it never *undershoots* the requested rate — it may
/// overshoot it) while the next divider up always yields an
/// `actual_baud_rate <= baud_rate` (never overshoots, may undershoot); the
/// true closest achievable rate for that `bwpc` is whichever of those two
/// brackets `baud_rate` more tightly, and it is not always the floored one.
/// Searching only the floored divider (as `modern-tc32/tlsr82xx`'s own
/// `compute_baud_params` does, and as this function's own first
/// implementation did) happens to still reproduce `uart.h`'s worked table
/// and this module's 4800-baud target exactly, but *not* because every one
/// of those rates has zero error: 9600, 19200, and the 4800 BL0942 route
/// all divide 24 MHz exactly (zero error at the floored divider, so no
/// ceiling candidate could possibly do better), but 115200 does **not** —
/// `uart.h`'s own worked table entry for 115200 (`div=12`, `bwpc=15`) is
/// itself only an approximation, achieving 115384 baud (184 baud, ~0.16%,
/// high), and this function reproduces that exact approximate vendor value
/// (see `tests::baud_divider_matches_vendor_worked_examples`) because the
/// floored divider still happens to be the *closest* of the two candidates
/// at `bwpc=15` for this particular rate (the next divider up lands at
/// ~107143 baud, far worse) — not because it is exact. Floor-only search
/// is not reliably correct in general: for other rates the ceiling
/// candidate from the very same `bwpc` is the closer one, and floor-only
/// search silently picks the worse candidate — see
/// `tests::baud_divider_ceiling_candidate_beats_floor_only_search_for_7000_baud`
/// for a concrete worked counter-example, and
/// `tests::baud_divider_matches_true_global_optimum_by_brute_force`, which
/// independently brute-forces every `(div, bwpc)` pair to prove this
/// function always returns the true minimum-error candidate, not just the
/// floor-only one.
///
/// Ties (equal error) prefer the larger `bwpc` (finer per-bit sampling,
/// matching `modern-tc32/tlsr82xx`'s tie-break) and, only if `bwpc` also
/// ties (floor and ceiling landing on the same error for one `bwpc` — this
/// does happen, e.g. at 300 baud/24 MHz both the floor and ceiling
/// dividers for `bwpc=15` truncate to the identical achieved rate), the
/// smaller `div` between *those two candidates specifically*. This
/// function does not exhaustively search every `div` that might tie with
/// them at the same `bwpc` (see
/// `tests::baud_divider_matches_true_global_optimum_by_brute_force`'s own
/// note) — doing so cannot change the achieved baud rate or its error,
/// only the specific divider value chosen, and this module only commits
/// to minimizing error, not to finding the globally smallest tied `div`.
/// `div` is bounded to the 15-bit `FLD_UART_CLK_DIV` field.
///
/// Returns `(div, bwpc, actual_baud_rate)`, or `None` if `baud_rate`/
/// `system_clock_hz` is zero or no candidate fits the divider width.
pub fn compute_baud_divider(system_clock_hz: u32, baud_rate: u32) -> Option<(u16, u8, u32)> {
    if system_clock_hz == 0 || baud_rate == 0 {
        return None;
    }

    // All arithmetic below is done in `u64`, not `u32`, even though both
    // inputs are `u32`: `denominator` (`baud_rate * (bwpc + 1)`, bwpc up to
    // 15) and `div_plus_1 * (bwpc + 1)` (div_plus_1 up to `system_clock_hz`)
    // can each exceed `u32::MAX` for large-but-still-valid `u32` inputs
    // (e.g. `system_clock_hz` near `u32::MAX` with a very small
    // `baud_rate`). A `u32` `saturating_mul`/plain `*` here would either
    // silently clamp to a wrong value or (for the unchecked `*` on
    // `div_plus_1 * (bwpc + 1)`) wrap, in both cases potentially making
    // this search select a bogus candidate instead of either computing the
    // right answer or (as a search helper with no hardware side effects)
    // simply taking longer/wider integer math to always get it right.
    // `u64` has enough headroom for any `u32` product (`u32::MAX * 16 <
    // 2^36 << u64::MAX`), so no such overflow can occur here regardless of
    // input. The final `div`/`actual` values are always within `u16`/`u32`
    // range by construction (`div` is checked against the 15-bit
    // `FLD_UART_CLK_DIV_MASK` before use; `actual <= system_clock_hz <=
    // u32::MAX`), so the narrowing casts at the end are lossless. The
    // 24 MHz/4800 route and every worked-table value are unchanged by this
    // (see `tests::baud_divider_is_exact_for_the_bl0942_route`/
    // `tests::baud_divider_matches_vendor_worked_examples`).
    let system_clock_hz = u64::from(system_clock_hz);
    let baud_rate_u64 = u64::from(baud_rate);

    let mut best: Option<(u64, u16, u8, u32)> = None; // (error, div, bwpc, actual)
    for bwpc in MIN_BWPC..=MAX_BWPC {
        let denominator = baud_rate_u64 * (u64::from(bwpc) + 1);
        if denominator == 0 {
            continue;
        }
        let floor_div_plus_1 = system_clock_hz / denominator;
        if floor_div_plus_1 == 0 {
            // Even div=0 (the fastest this bwpc can go) does not reach
            // baud_rate; no candidate at this bwpc can, either.
            continue;
        }
        // The floored divider (`actual >= baud_rate`) and the next divider
        // up (`actual <= baud_rate`) are the only two candidates that can
        // possibly be closest to `baud_rate` for this `bwpc` — anything
        // else is farther from `baud_rate` in the same direction. See this
        // function's own docs.
        for div_plus_1 in [floor_div_plus_1, floor_div_plus_1 + 1] {
            let div = div_plus_1 - 1;
            if div > u64::from(FLD_UART_CLK_DIV_MASK) {
                continue;
            }
            let actual = system_clock_hz / (div_plus_1 * (u64::from(bwpc) + 1));
            let error = actual.abs_diff(baud_rate_u64);
            let better = match best {
                None => true,
                Some((best_error, best_div, best_bwpc, _)) => {
                    error < best_error
                        || (error == best_error
                            && (bwpc > best_bwpc || (bwpc == best_bwpc && (div as u16) < best_div)))
                }
            };
            if better {
                best = Some((error, div as u16, bwpc, actual as u32));
            }
        }
    }
    best.map(|(_, div, bwpc, actual)| (div, bwpc, actual))
}

/// `uart.h`'s documented `UART_TxPinDef` set, disassembly-verified in
/// `gpio.rs`'s `function_selector()`.
const UART_TX_PINS: [(Port, u8); 6] = [
    (Port::A, 2),
    (Port::B, 1),
    (Port::C, 2),
    (Port::D, 0),
    (Port::D, 3),
    (Port::D, 7),
];
/// `uart.h`'s documented `UART_RxPinDef` set, disassembly-verified in
/// `gpio.rs`'s `function_selector()`.
const UART_RX_PINS: [(Port, u8); 6] = [
    (Port::A, 0),
    (Port::B, 0),
    (Port::B, 7),
    (Port::C, 3),
    (Port::C, 5),
    (Port::D, 6),
];
/// `uart.h`'s documented `UART_RtsPinDef` set. These pins need no
/// `gpio_set_func` mux entry at all (see the module docs' "RTS/CTS
/// hardware flow control" section) — only pull/input configuration.
const UART_RTS_PINS: [(Port, u8); 4] = [(Port::A, 4), (Port::B, 3), (Port::B, 6), (Port::C, 0)];
/// `uart.h`'s documented `UART_CtsPinDef` set. Same no-mux-entry note as
/// [`UART_RTS_PINS`] applies.
const UART_CTS_PINS: [(Port, u8); 4] = [(Port::A, 3), (Port::B, 2), (Port::C, 4), (Port::D, 1)];

fn validate_pins(tx: &Pin, rx: &Pin) -> Result<(), UartError> {
    if !UART_TX_PINS.contains(&tx.port_and_bit()) || !UART_RX_PINS.contains(&rx.port_and_bit()) {
        return Err(UartError::InvalidPins);
    }
    Ok(())
}

fn validate_rts_pin(pin: &Pin) -> Result<(), UartError> {
    if !UART_RTS_PINS.contains(&pin.port_and_bit()) {
        return Err(UartError::InvalidRtsPin);
    }
    Ok(())
}

fn validate_cts_pin(pin: &Pin) -> Result<(), UartError> {
    if !UART_CTS_PINS.contains(&pin.port_and_bit()) {
        return Err(UartError::InvalidCtsPin);
    }
    Ok(())
}

/// `reg_uart_ctrl0`'s next value when reprogramming `FLD_UART_BPWC` —
/// scoped RMW matching `uart_init()`'s disassembly (see
/// [`FLD_UART_CTRL0_UPPER_MASK`]'s doc for the evidence). Pure/host-testable.
const fn ctrl0_with_bwpc(current: u8, bwpc: u8) -> u8 {
    (current & FLD_UART_CTRL0_UPPER_MASK) | (bwpc & FLD_UART_BPWC)
}

/// `reg_uart_ctrl1`'s parity+stop-bit field value in isolation (bits 2..5),
/// matching `uart.h`'s own bit layout. Pure/host-testable.
const fn parity_stop_bits(parity: Parity, stop_bits: StopBits) -> u8 {
    let base = match parity {
        Parity::None => 0u8,
        Parity::Even => FLD_UART_CTRL1_PARITY_EN,
        Parity::Odd => FLD_UART_CTRL1_PARITY_EN | FLD_UART_CTRL1_PARITY_POLARITY,
    };
    base | stop_bits as u8
}

/// `reg_uart_ctrl1`'s next value when reprogramming parity/stop bits —
/// scoped RMW matching `uart_init()`'s disassembly (see
/// [`FLD_UART_CTRL1_OWNED_MASK`]'s doc for the evidence): `CTS_SELECT`/
/// `CTS_EN`/`TTL`/`LOOPBACK` (outside the mask) survive untouched.
/// Pure/host-testable.
const fn ctrl1_with_parity_stop(current: u8, parity_stop: u8) -> u8 {
    (current & !FLD_UART_CTRL1_OWNED_MASK) | (parity_stop & FLD_UART_CTRL1_OWNED_MASK)
}

/// `reg_uart_ctrl1`'s next value when reprogramming CTS — scoped RMW that
/// only ever touches `CTS_SELECT`/`CTS_EN` (bits 0..1), leaving parity,
/// stop bits, `TTL`, and `LOOPBACK` alone, matching `uart_set_cts`'s
/// disassembly (see the module docs' "RTS/CTS hardware flow control"
/// section). Pure/host-testable.
const fn ctrl1_with_cts(current: u8, enabled: bool, stop_level: bool) -> u8 {
    let mut v = current & !(FLD_UART_CTRL1_CTS_SELECT | FLD_UART_CTRL1_CTS_EN);
    if enabled {
        v |= FLD_UART_CTRL1_CTS_EN;
    }
    if stop_level {
        v |= FLD_UART_CTRL1_CTS_SELECT;
    }
    v
}

/// `reg_uart_ctrl2`'s RTS field bits (threshold, invert, manual value,
/// manual enable) in isolation — everything *except* `RTS_EN`, which the
/// caller ORs in separately only when RTS is actually enabled, matching
/// `uart_set_rts`'s disassembled order (threshold/invert are written
/// unconditionally; `RTS_EN` is set last, only if `Enable`).
/// `invert` is intentionally accepted for *both* [`RtsMode::Auto`] and
/// [`RtsMode::Manual`] — see the module docs' discrepancy note against
/// `uart.h`'s own doc comment. Pure/host-testable.
const fn rts_ctrl2_fields(mode: RtsMode, threshold: u8, invert: bool, manual_level: bool) -> u8 {
    let mut v = threshold & FLD_UART_CTRL2_RTS_TRIG_LVL_MASK;
    if invert {
        v |= FLD_UART_CTRL2_RTS_PARITY;
    }
    if manual_level {
        v |= FLD_UART_CTRL2_RTS_MANUAL_VAL;
    }
    if let RtsMode::Manual = mode {
        v |= FLD_UART_CTRL2_RTS_MANUAL_EN;
    }
    v
}

/// `reg_uart_ctrl3`'s IRQ-trigger-level byte (`uart_ndma_irq_triglevel`'s
/// two nibble arguments packed together). Pure/host-testable.
const fn irq_trigger_levels_byte(rx_level: u8, tx_level: u8) -> u8 {
    (rx_level & FLD_UART_CTRL3_RX_IRQ_TRIG_LVL_MASK)
        | ((tx_level & FLD_UART_CTRL3_RX_IRQ_TRIG_LVL_MASK) << FLD_UART_CTRL3_TX_IRQ_TRIG_LVL_SHIFT)
}

/// `true` if `tx_fifo_count` is *strictly greater than*
/// [`TX_FIFO_BACKPRESSURE_THRESHOLD`] — i.e. the non-DMA TX FIFO is too
/// full to accept another byte right now and [`Uart::try_write`] must
/// return `Ok(false)` instead of writing. A count *equal to* the threshold
/// still has room (see [`TX_FIFO_BACKPRESSURE_THRESHOLD`]'s doc for the
/// `modern-tc32/tlsr82xx` evidence for this exact boundary).
const fn tx_backpressure(tx_fifo_count: u8) -> bool {
    tx_fifo_count > TX_FIFO_BACKPRESSURE_THRESHOLD
}

const fn cleared_rx_status(status: u8) -> u8 {
    status | FLD_UART_CLEAR_RX_FLAG
}

/// Read-only decode of [`Uart::poll_events`]'s two status registers.
/// Every field here is a plain bit test — see [`decode_events`]'s doc for
/// why none of these have a `clear`/acknowledge method.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct UartEvents {
    /// `reg_uart_status0` bit 3, `FLD_UART_IRQ_FLAG` — the non-DMA RX/TX
    /// trigger-level condition `uart_ndma_irq_triglevel` configures.
    pub irq_flag: bool,
    /// `reg_uart_status0` bit 7, `FLD_UART_RX_ERR_FLAG` — same condition
    /// [`Uart::rx_error_pending`] reports; included here too since
    /// [`Uart::poll_events`] is meant to be the one-stop status read.
    pub rx_error: bool,
    /// `reg_uart_status1` bit 0, `FLD_UART_TX_DONE`.
    pub tx_done: bool,
    /// `reg_uart_status1` bit 1, `FLD_UART_TX_BUF_IRQ`.
    pub tx_buf_irq: bool,
    /// `reg_uart_status1` bit 2, `FLD_UART_RX_DONE`.
    pub rx_done: bool,
    /// `reg_uart_status1` bit 3, `FLD_UART_RX_BUF_IRQ`.
    pub rx_buf_irq: bool,
}

/// Pure decode of `reg_uart_status0`/`reg_uart_status1` into
/// [`UartEvents`]. None of these bits have a vendor-documented or
/// disassembly-observed clear/write procedure (only `reg_uart_status0`
/// bit 6, handled separately by [`cleared_rx_status`]/
/// [`Uart::clear_rx_error`], does) — `modern-tc32/tlsr82xx` likewise only
/// ever reads `FLD_UART_TX_DONE`, never writes it. Modeled read-only for
/// that reason. Pure/host-testable.
const fn decode_events(status0: u8, status1: u8) -> UartEvents {
    UartEvents {
        irq_flag: status0 & FLD_UART_IRQ_FLAG != 0,
        rx_error: status0 & FLD_UART_RX_ERR_FLAG != 0,
        tx_done: status1 & FLD_UART_TX_DONE != 0,
        tx_buf_irq: status1 & FLD_UART_TX_BUF_IRQ != 0,
        rx_done: status1 & FLD_UART_RX_DONE != 0,
        rx_buf_irq: status1 & FLD_UART_RX_BUF_IRQ != 0,
    }
}

/// Blocking, non-DMA TLSR8258 UART.
pub struct Uart {
    config: Config,
    tx_index: u8,
    rx_index: u8,
    rx_irq_enabled: bool,
    tx_irq_enabled: bool,
    rx_irq_trigger_level: u8,
    tx_irq_trigger_level: u8,
    error_irq_masked: bool,
    /// Cached so [`Uart::reset`] can reproduce the last
    /// [`Uart::set_rts_manual_level`] call even though the hardware reset
    /// pulse clears `reg_uart_ctrl2`.
    manual_rts_level: bool,
}

impl Uart {
    /// Bring up the UART peripheral (clock/reset), validate and mux the
    /// PB1/PB7 pins, and program the baud/parity/stop-bit registers.
    ///
    /// `_peripheral` proves exclusive ownership of the independent UART
    /// controller (does not share registers with [`crate::i2c`]/
    /// [`crate::spi`]'s `SerialController`).
    #[cfg(target_arch = "tc32")]
    pub fn new(_peripheral: crate::peripherals::Uart, config: Config) -> Result<Self, UartError> {
        let (div, bwpc, _actual_baud_rate) =
            compute_baud_divider(config.system_clock_hz, config.baud_rate)
                .ok_or(UartError::InvalidBaudRate)?;
        validate_pins(&config.tx, &config.rx)?;
        if let Some(rts) = &config.rts {
            validate_rts_pin(&rts.pin)?;
        }
        if let Some(cts) = &config.cts {
            validate_cts_pin(&cts.pin)?;
        }

        let mut uart = Self {
            config,
            tx_index: 0,
            rx_index: 0,
            rx_irq_enabled: false,
            tx_irq_enabled: false,
            rx_irq_trigger_level: 0,
            tx_irq_trigger_level: 0,
            error_irq_masked: false,
            manual_rts_level: false,
        };
        uart.configure_pins()?;
        uart.enable_peripheral();
        uart.configure_registers(div, bwpc);
        uart.configure_flow_control()?;
        crate::irq::clear_pending(IrqSource::Uart);
        uart.sync_global_irq_enable();
        Ok(uart)
    }

    /// The actual baud rate [`Uart::new`] programmed, for diagnostics.
    pub fn actual_baud_rate(&self) -> Option<u32> {
        compute_baud_divider(self.config.system_clock_hz, self.config.baud_rate)
            .map(|(_, _, actual)| actual)
    }

    #[cfg(target_arch = "tc32")]
    fn configure_pins(&self) -> Result<(), UartError> {
        crate::gpio::set_function(&self.config.tx, PinFunction::Uart)
            .map_err(|_| UartError::InvalidPins)?;
        crate::gpio::set_function(&self.config.rx, PinFunction::Uart)
            .map_err(|_| UartError::InvalidPins)?;

        // `uart_gpio_set(tx, rx)`'s disassembly pulls up (10K) *and*
        // input-enables both pins, not just RX — this module previously
        // only pulled up RX, which this fixes. TX also gets
        // `set_output_enable` (harmless belt-and-suspenders: the vendor
        // function never calls it, but a pin's `oen` register is
        // superseded once its mux `func` != `AS_GPIO`, so keeping it is a
        // no-op rather than a bug) to preserve this module's previously
        // observed PB1 behavior unchanged.
        crate::gpio::set_output_enable(&self.config.tx, true);
        crate::gpio::set_input_enable(&self.config.tx, true).map_err(|_| UartError::InvalidPins)?;
        crate::gpio::set_pull(&self.config.tx, Pull::PullUp10K)
            .map_err(|_| UartError::InvalidPins)?;
        crate::gpio::set_output_enable(&self.config.rx, false);
        crate::gpio::set_input_enable(&self.config.rx, true).map_err(|_| UartError::InvalidPins)?;
        crate::gpio::set_pull(&self.config.rx, Pull::PullUp10K)
            .map_err(|_| UartError::InvalidPins)?;
        Ok(())
    }

    #[cfg(target_arch = "tc32")]
    fn enable_peripheral(&self) {
        // Clock-gate and reset the UART block via the generic
        // reg_clk_en0/reg_rst0 facade (see `crate::reset::Peripheral::Uart`)
        // instead of hand-rolling the same read-modify-write this module
        // used to perform locally.
        crate::reset::enable_clock(crate::reset::Peripheral::Uart)
            .expect("UART has a documented reg_clk_en0 bit");
        crate::reset::pulse_reset(crate::reset::Peripheral::Uart);
    }

    #[cfg(target_arch = "tc32")]
    fn configure_registers(&mut self, div: u16, bwpc: u8) {
        unsafe {
            // ctrl0: scoped RMW — only bits 0..3 (`FLD_UART_BPWC`) change;
            // bits 4..7 (RX/TX DMA + IRQ enables) are preserved exactly as
            // `uart_init()`'s disassembly does, so a later `reset()` never
            // silently clobbers whatever `set_rx_irq_enabled`/
            // `set_tx_irq_enabled` last programmed there.
            let ctrl0 = r8(REG_UART_CTRL0);
            w8(REG_UART_CTRL0, ctrl0_with_bwpc(ctrl0, bwpc));

            // 15-bit clock divider plus its own enable bit.
            w16(
                REG_UART_CLK_DIV,
                (div & FLD_UART_CLK_DIV_MASK) | FLD_UART_CLK_DIV_EN,
            );

            // RX timeout: see the module docs' confidence-gap note.
            w8(REG_UART_RX_TIMEOUT0, (bwpc.wrapping_add(1)).wrapping_mul(3));
            let timeout1 = r8(REG_UART_RX_TIMEOUT1) & !FLD_UART_TIMEOUT_MUL_MASK;
            w8(REG_UART_RX_TIMEOUT1, timeout1 | UART_TIMEOUT_MUL_3X_BWPC);

            // ctrl1: scoped RMW — only the parity (bits 2..3) and stop-bit
            // (bits 4..5) fields change; `CTS_SELECT`/`CTS_EN` (bits 0..1)
            // and `TTL`/`LOOPBACK` (bits 6..7) are preserved exactly as
            // `uart_init()`'s disassembly does, so [`Uart::configure_flow_control`]'s
            // CTS bits (applied right after this call, from both `new()`
            // and `reset()`) are never wiped by a later baud/parity
            // reprogram.
            let ctrl1 = r8(REG_UART_CTRL1);
            let parity_stop = parity_stop_bits(self.config.parity, self.config.stop_bits);
            w8(REG_UART_CTRL1, ctrl1_with_parity_stop(ctrl1, parity_stop));

            // ctrl2/ctrl3 (RTS control + IRQ trigger levels) are left
            // untouched here — `crate::reset::pulse_reset` already reset
            // them to their hardware default, and
            // [`Uart::configure_flow_control`] plus the cached
            // IRQ-trigger-level reapply (see `new()`/`reset()`) fully own
            // reprogramming them from this `Uart`'s own state, not a
            // blind zero write.
        }
        self.tx_index = 0;
        self.rx_index = 0;
    }

    /// Apply [`Config::rts`]/[`Config::cts`] (or leave both disabled if
    /// `None`) to hardware. Called from both [`Uart::new`] and
    /// [`Uart::reset`] so RTS/CTS state always survives a hardware reset
    /// pulse, which otherwise clears `reg_uart_ctrl2`/`reg_uart_ctrl1`'s
    /// CTS bits back to their power-on default.
    #[cfg(target_arch = "tc32")]
    fn configure_flow_control(&mut self) -> Result<(), UartError> {
        self.apply_rts()?;
        self.apply_cts()?;
        Ok(())
    }

    /// `reg_uart_ctrl2` is entirely owned by RTS fields (its high-byte
    /// alias, `reg_uart_ctrl3`, is the separate IRQ-trigger-level byte —
    /// see [`REG_UART_CTRL3`]), so this writes the whole byte fresh each
    /// time rather than an RMW; there is nothing else in it to preserve.
    #[cfg(target_arch = "tc32")]
    fn apply_rts(&mut self) -> Result<(), UartError> {
        let bits = match self.config.rts.as_ref() {
            Some(rts) => {
                // `uart_set_rts`'s disassembly pulls up (10K) and
                // input-enables the RTS pin with no `gpio_set_func` call
                // at all — see the module docs' "RTS/CTS hardware flow
                // control" section.
                crate::gpio::set_input_enable(&rts.pin, true)
                    .map_err(|_| UartError::InvalidRtsPin)?;
                crate::gpio::set_pull(&rts.pin, Pull::PullUp10K)
                    .map_err(|_| UartError::InvalidRtsPin)?;
                rts_ctrl2_fields(rts.mode, rts.threshold, rts.invert, self.manual_rts_level)
                    | FLD_UART_CTRL2_RTS_EN
            }
            None => rts_ctrl2_fields(RtsMode::Auto, 0, false, self.manual_rts_level),
        };
        unsafe { w8(REG_UART_CTRL2, bits) };
        Ok(())
    }

    /// `reg_uart_ctrl1`'s CTS bits only — see [`ctrl1_with_cts`] for the
    /// scoped-RMW rationale (parity/stop bits and TTL/loopback must
    /// survive).
    #[cfg(target_arch = "tc32")]
    fn apply_cts(&mut self) -> Result<(), UartError> {
        let current = unsafe { r8(REG_UART_CTRL1) };
        let next = match self.config.cts.as_ref() {
            Some(cts) => {
                // `uart_set_cts`'s disassembly pulls up (10K) and
                // input-enables the CTS pin with no `gpio_set_func` call
                // either.
                crate::gpio::set_input_enable(&cts.pin, true)
                    .map_err(|_| UartError::InvalidCtsPin)?;
                crate::gpio::set_pull(&cts.pin, Pull::PullUp10K)
                    .map_err(|_| UartError::InvalidCtsPin)?;
                ctrl1_with_cts(current, true, cts.stop_level)
            }
            None => ctrl1_with_cts(current, false, false),
        };
        unsafe { w8(REG_UART_CTRL1, next) };
        Ok(())
    }

    /// Set `FLD_UART_CTRL2_RTS_MANUAL_VAL` directly, matching
    /// `uart_set_rts_level()`'s disassembly (a single unconditional
    /// set/clear of that one bit, independent of RTS mode/enable state).
    /// Only meaningful when [`Config::rts`]'s [`RtsMode`] is
    /// [`RtsMode::Manual`], but harmless to call otherwise. The level is
    /// cached so [`Uart::reset`] reproduces it afterward.
    #[cfg(target_arch = "tc32")]
    pub fn set_rts_manual_level(&mut self, high: bool) {
        self.manual_rts_level = high;
        unsafe {
            let current = r8(REG_UART_CTRL2);
            let next = if high {
                current | FLD_UART_CTRL2_RTS_MANUAL_VAL
            } else {
                current & !FLD_UART_CTRL2_RTS_MANUAL_VAL
            };
            w8(REG_UART_CTRL2, next);
        }
    }

    /// Enable/disable the non-DMA RX IRQ-trigger-level condition
    /// (`reg_uart_ctrl0` bit 6, `FLD_UART_RX_IRQ_EN`), and fold the result
    /// into [`IrqSource::Uart`]'s single shared CPU-level enable bit via
    /// [`Uart::sync_global_irq_enable`] — matching `uart_irq_enable()`'s
    /// disassembly, which does exactly this (see the module docs' "Polled,
    /// non-DMA UART interrupt support" section).
    #[cfg(target_arch = "tc32")]
    pub fn set_rx_irq_enabled(&mut self, enabled: bool) {
        self.rx_irq_enabled = enabled;
        unsafe {
            let ctrl0 = r8(REG_UART_CTRL0);
            let next = if enabled {
                ctrl0 | FLD_UART_RX_IRQ_EN
            } else {
                ctrl0 & !FLD_UART_RX_IRQ_EN
            };
            w8(REG_UART_CTRL0, next);
        }
        self.sync_global_irq_enable();
    }

    /// Enable/disable the non-DMA TX IRQ-trigger-level condition
    /// (`reg_uart_ctrl0` bit 7, `FLD_UART_TX_IRQ_EN`). See
    /// [`Uart::set_rx_irq_enabled`]'s doc for the shared rationale.
    #[cfg(target_arch = "tc32")]
    pub fn set_tx_irq_enabled(&mut self, enabled: bool) {
        self.tx_irq_enabled = enabled;
        unsafe {
            let ctrl0 = r8(REG_UART_CTRL0);
            let next = if enabled {
                ctrl0 | FLD_UART_TX_IRQ_EN
            } else {
                ctrl0 & !FLD_UART_TX_IRQ_EN
            };
            w8(REG_UART_CTRL0, next);
        }
        self.sync_global_irq_enable();
    }

    /// Program the non-DMA RX/TX trigger levels (`reg_uart_ctrl3`,
    /// `uart_ndma_irq_triglevel`'s two nibble arguments — see
    /// [`irq_trigger_levels_byte`]). Only the low 4 bits of each level are
    /// wired to hardware; higher bits are masked off silently, matching
    /// the vendor function's own `& 0xF` on both arguments.
    #[cfg(target_arch = "tc32")]
    pub fn set_irq_trigger_levels(&mut self, rx_level: u8, tx_level: u8) {
        self.rx_irq_trigger_level = rx_level & FLD_UART_CTRL3_RX_IRQ_TRIG_LVL_MASK;
        self.tx_irq_trigger_level = tx_level & FLD_UART_CTRL3_RX_IRQ_TRIG_LVL_MASK;
        unsafe {
            w8(
                REG_UART_CTRL3,
                irq_trigger_levels_byte(self.rx_irq_trigger_level, self.tx_irq_trigger_level),
            );
        }
    }

    /// Enable/disable `reg_uart_rx_timeout1`'s `FLD_UART_MASK_ERR_IRQ` bit.
    /// `uart_mask_error_irq_enable()`'s disassembly *always* enables
    /// [`IrqSource::Uart`] at the CPU level when called (unlike
    /// `uart_irq_enable`, which only enables it if either RX or TX IRQ is
    /// requested) — this mirrors that by including `error_irq_masked` in
    /// [`Uart::sync_global_irq_enable`]'s OR, which has the same net
    /// effect (enabling stays sticky as long as this flag is `true`) while
    /// still allowing `false` to actually release the shared bit if RX/TX
    /// IRQs are also both off.
    #[cfg(target_arch = "tc32")]
    pub fn set_error_irq_masked(&mut self, masked: bool) {
        self.error_irq_masked = masked;
        unsafe {
            let timeout1 = r8(REG_UART_RX_TIMEOUT1);
            let next = if masked {
                timeout1 | FLD_UART_MASK_ERR_IRQ
            } else {
                timeout1 & !FLD_UART_MASK_ERR_IRQ
            };
            w8(REG_UART_RX_TIMEOUT1, next);
        }
        self.sync_global_irq_enable();
    }

    /// Fold all three IRQ-want flags into a single
    /// [`crate::irq::set_enabled`] call so that, e.g., disabling the TX
    /// IRQ never silently disables the shared [`IrqSource::Uart`] bit out
    /// from under a still-wanted RX IRQ or masked-error IRQ.
    #[cfg(target_arch = "tc32")]
    fn sync_global_irq_enable(&self) {
        crate::irq::set_enabled(
            IrqSource::Uart,
            self.rx_irq_enabled || self.tx_irq_enabled || self.error_irq_masked,
        );
    }

    /// Read-only, single-poll decode of `reg_uart_status0` bit 3 and
    /// `reg_uart_status1`'s four bits. No vendor-documented or
    /// disassembly-observed clear/write procedure exists for any of these
    /// (unlike [`Uart::clear_rx_error`]'s bit 6) — see the module docs'
    /// "Polled, non-DMA UART interrupt support" section.
    #[cfg(target_arch = "tc32")]
    pub fn poll_events(&self) -> UartEvents {
        let status0 = unsafe { r8(REG_UART_STATUS0) };
        let status1 = unsafe { r8(REG_UART_STATUS1) };
        decode_events(status0, status1)
    }

    #[cfg(target_arch = "tc32")]
    fn read_ready(&self) -> bool {
        unsafe { r8(REG_UART_BUF_CNT) & FLD_UART_RX_BUF_CNT != 0 }
    }

    /// `reg_uart_buf_cnt`'s TX count (bits 4..7).
    #[cfg(target_arch = "tc32")]
    fn tx_fifo_count(&self) -> u8 {
        unsafe { (r8(REG_UART_BUF_CNT) & FLD_UART_TX_BUF_CNT) >> 4 }
    }

    /// `true` if `reg_uart_status0`'s combined RX error flag is set.
    #[cfg(target_arch = "tc32")]
    pub fn rx_error_pending(&self) -> bool {
        unsafe { r8(REG_UART_STATUS0) & FLD_UART_RX_ERR_FLAG != 0 }
    }

    /// Acknowledge the RX error flag using the vendor driver's exact
    /// read-modify-write sequence.
    ///
    /// The shipped `uart_clear_parity_error()` reads
    /// `reg_uart_status0`, ORs [`FLD_UART_CLEAR_RX_FLAG`] (bit 6), and
    /// writes the resulting byte back. It does not write the
    /// [`FLD_UART_RX_ERR_FLAG`] bit (bit 7) explicitly. The vendor header
    /// also says the next non-DMA receive must restart the four-register
    /// cycle at `0x90`, so this resets `rx_index` to zero. Callers should
    /// treat re-observing
    /// [`Uart::rx_error_pending`] as `true` after this call as a signal to
    /// fully reinitialize the peripheral via [`Uart::reset`] rather than
    /// retrying the clear indefinitely.
    #[cfg(target_arch = "tc32")]
    pub fn clear_rx_error(&mut self) {
        unsafe {
            let status = r8(REG_UART_STATUS0);
            w8(REG_UART_STATUS0, cleared_rx_status(status));
        }
        self.rx_index = 0;
    }

    /// Fail-closed recovery: fully re-run the clock/reset toggle,
    /// baud/parity/stop-bit register programming, and RTS/CTS/IRQ-enable
    /// reprogramming this `Uart` was constructed (or last reconfigured)
    /// with, discarding the non-DMA TX/RX byte-index cursors.
    ///
    /// Use this if [`Uart::rx_error_pending`] is still `true` after
    /// [`Uart::clear_rx_error`] — i.e. the vendor bit-6 clear did not
    /// actually acknowledge the condition — instead of
    /// looping on `clear_rx_error` indefinitely, which this crate's
    /// no-infinite-wait convention rules out.
    ///
    /// The hardware reset pulse ([`Uart::enable_peripheral`]) clears
    /// `reg_uart_ctrl2` (RTS + IRQ trigger levels) and `reg_uart_ctrl1`'s
    /// CTS bits back to their power-on default, so every piece of cached
    /// state ([`Config::rts`]/[`Config::cts`], the RX/TX IRQ enables, the
    /// IRQ trigger levels, the error-IRQ mask, and the last
    /// [`Uart::set_rts_manual_level`] value) is reapplied here — this is
    /// the one place all of that cached state exists for.
    #[cfg(target_arch = "tc32")]
    pub fn reset(&mut self) -> Result<(), UartError> {
        // `Uart::new` already proved this config yields a valid divider,
        // so this recomputation cannot fail.
        let (div, bwpc, _) =
            compute_baud_divider(self.config.system_clock_hz, self.config.baud_rate)
                .expect("Uart::new already validated this config's baud divider");
        self.enable_peripheral();
        self.configure_registers(div, bwpc);
        self.configure_flow_control()?;

        unsafe {
            let mut ctrl0 = r8(REG_UART_CTRL0);
            ctrl0 = if self.rx_irq_enabled {
                ctrl0 | FLD_UART_RX_IRQ_EN
            } else {
                ctrl0 & !FLD_UART_RX_IRQ_EN
            };
            ctrl0 = if self.tx_irq_enabled {
                ctrl0 | FLD_UART_TX_IRQ_EN
            } else {
                ctrl0 & !FLD_UART_TX_IRQ_EN
            };
            w8(REG_UART_CTRL0, ctrl0);

            w8(
                REG_UART_CTRL3,
                irq_trigger_levels_byte(self.rx_irq_trigger_level, self.tx_irq_trigger_level),
            );

            let timeout1 = r8(REG_UART_RX_TIMEOUT1);
            w8(
                REG_UART_RX_TIMEOUT1,
                if self.error_irq_masked {
                    timeout1 | FLD_UART_MASK_ERR_IRQ
                } else {
                    timeout1 & !FLD_UART_MASK_ERR_IRQ
                },
            );
        }
        crate::irq::clear_pending(IrqSource::Uart);
        self.sync_global_irq_enable();
        Ok(())
    }

    /// Nonblocking, single-poll RX: `Ok(None)` if no byte is ready yet,
    /// `Ok(Some(byte))` if one was read, `Err(UartError::RxError)` if the
    /// hardware's combined framing/parity/overrun flag was set (already
    /// acknowledged before returning).
    #[cfg(target_arch = "tc32")]
    pub fn try_read(&mut self) -> Result<Option<u8>, UartError> {
        if self.rx_error_pending() {
            self.clear_rx_error();
            return Err(UartError::RxError);
        }
        if !self.read_ready() {
            return Ok(None);
        }
        let byte = unsafe { r8(REG_UART_DATA_BUF0 + u32::from(self.rx_index)) };
        self.rx_index = (self.rx_index + 1) & DATA_BUF_RING_MASK;
        Ok(Some(byte))
    }

    /// Nonblocking, single-poll TX: `Ok(true)` if `byte` was queued,
    /// `Ok(false)` if the non-DMA TX FIFO's count is currently *above*
    /// [`TX_FIFO_BACKPRESSURE_THRESHOLD`] (caller should retry later —
    /// never blocks). See [`tx_backpressure`] for the exact, host-tested
    /// boundary condition.
    #[cfg(target_arch = "tc32")]
    pub fn try_write(&mut self, byte: u8) -> Result<bool, UartError> {
        if tx_backpressure(self.tx_fifo_count()) {
            return Ok(false);
        }
        unsafe { w8(REG_UART_DATA_BUF0 + u32::from(self.tx_index), byte) };
        self.tx_index = (self.tx_index + 1) & DATA_BUF_RING_MASK;
        Ok(true)
    }

    /// Bounded wait for `FLD_UART_TX_DONE`. `max_iterations` must be
    /// caller-chosen and finite — this never waits unconditionally.
    #[cfg(target_arch = "tc32")]
    pub fn flush(&self, max_iterations: u32) -> Result<(), UartError> {
        for _ in 0..max_iterations {
            if unsafe { r8(REG_UART_STATUS1) } & FLD_UART_TX_DONE != 0 {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(UartError::FlushTimeout)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_addresses_match_register_h() {
        assert_eq!(REG_UART_DATA_BUF0, 0x800090);
        assert_eq!(REG_UART_CLK_DIV, 0x800094);
        assert_eq!(REG_UART_CTRL0, 0x800096);
        assert_eq!(REG_UART_CTRL1, 0x800097);
        assert_eq!(REG_UART_CTRL2, 0x800098);
        assert_eq!(REG_UART_RX_TIMEOUT0, 0x80009A);
        assert_eq!(REG_UART_RX_TIMEOUT1, 0x80009B);
        assert_eq!(REG_UART_BUF_CNT, 0x80009C);
        assert_eq!(REG_UART_STATUS0, 0x80009D);
        assert_eq!(REG_UART_STATUS1, 0x80009E);
    }

    #[test]
    fn clock_and_reset_bits_match_register_h() {
        // The local `FLD_CLK0_UART_EN`/`FLD_RST0_UART` consts were removed
        // when `enable_peripheral` migrated to `crate::reset`'s facade;
        // confirm that facade still reports the same bit position (0x04,
        // bit 2 of `reg_clk_en0`/`reg_rst0`) this module's own doc comment
        // (and the vendor header) documents for UART.
        assert_eq!(crate::reset::Peripheral::Uart.clock_bit(), Some(0x04));
        assert_eq!(crate::reset::Peripheral::Uart.reset_bit(), 0x04);
    }

    #[test]
    fn stop_bit_values_match_vendor_enum() {
        assert_eq!(StopBits::One as u8, 0x00);
        assert_eq!(StopBits::OneAndHalf as u8, 0x10);
        assert_eq!(StopBits::Two as u8, 0x20);
    }

    #[test]
    fn baud_divider_matches_vendor_worked_examples() {
        // uart.h's own worked table, 24 MHz column.
        assert_eq!(
            compute_baud_divider(24_000_000, 9_600),
            Some((249, 9, 9_600))
        );
        assert_eq!(
            compute_baud_divider(24_000_000, 19_200),
            Some((124, 9, 19_200))
        );
        // 115200 is the table's own approximate entry (the vendor's chosen
        // div/bwpc do not divide 24 MHz exactly either).
        assert_eq!(
            compute_baud_divider(24_000_000, 115_200),
            Some((12, 15, 115_384))
        );
    }

    #[test]
    fn baud_divider_is_exact_for_the_bl0942_route() {
        // PB1/PB7, 4800 baud, 24 MHz system clock (this module's target
        // configuration) — exactly half of the vendor's own 9600 example.
        assert_eq!(
            compute_baud_divider(24_000_000, 4_800),
            Some((499, 9, 4_800))
        );
    }

    #[test]
    fn baud_divider_rejects_zero_inputs() {
        assert_eq!(compute_baud_divider(0, 4_800), None);
        assert_eq!(compute_baud_divider(24_000_000, 0), None);
    }

    #[test]
    fn baud_divider_rejects_unreachable_low_baud_rates() {
        // Requires div > 0x7fff at every bwpc in range: unreachable at this
        // clock, so this must fail closed rather than silently truncate.
        assert_eq!(compute_baud_divider(24_000_000, 1), None);
    }

    /// Independently brute-forces *every* `(div, bwpc)` pair in range —
    /// not just the floored divider `compute_baud_divider` used to only
    /// consider — and asserts `compute_baud_divider` always returns a
    /// candidate whose error matches this ground truth exactly. This is
    /// the test that would have failed against this function's original,
    /// floor-only implementation for a rate like 7000 (see the next test
    /// for a concrete, worked demonstration of that).
    fn true_minimum_error(system_clock_hz: u32, baud_rate: u32) -> Option<(u32, u16, u8)> {
        let mut best: Option<(u32, u16, u8)> = None;
        for bwpc in MIN_BWPC..=MAX_BWPC {
            for div in 0u32..=u32::from(FLD_UART_CLK_DIV_MASK) {
                let actual = system_clock_hz / ((div + 1) * (u32::from(bwpc) + 1));
                let error = actual.abs_diff(baud_rate);
                let better = match best {
                    None => true,
                    Some((best_error, best_div, best_bwpc)) => {
                        error < best_error
                            || (error == best_error
                                && (bwpc > best_bwpc
                                    || (bwpc == best_bwpc && (div as u16) < best_div)))
                    }
                };
                if better {
                    best = Some((error, div as u16, bwpc));
                }
            }
        }
        best
    }

    #[test]
    fn baud_divider_matches_true_global_optimum_by_brute_force() {
        for &baud_rate in &[
            300, 1200, 2400, 4800, 7000, 9600, 12345, 19200, 38400, 57600, 115200,
        ] {
            let expected = true_minimum_error(24_000_000, baud_rate)
                .expect("brute force must find some candidate for these rates at 24 MHz");
            let (_div, bwpc, actual) = compute_baud_divider(24_000_000, baud_rate)
                .unwrap_or_else(|| panic!("compute_baud_divider found nothing for {baud_rate}"));
            let error = actual.abs_diff(baud_rate);
            assert_eq!(
                error, expected.0,
                "baud_rate={baud_rate}: compute_baud_divider error {error} != brute-force optimum {}",
                expected.0
            );
            // `bwpc` should match the brute-force optimum's under the same
            // documented tie-break. `div` is deliberately *not* compared
            // here: integer truncation of `system_clock_hz / (denominator)`
            // can make a whole contiguous range of `div` values collapse to
            // the identical truncated `actual` (see
            // `baud_rate=300`, where both `div=4983` and `div=4999` truncate
            // to exactly 300 with `bwpc=15`). This function's two-candidate
            // search always lands on the *floor* end of any such plateau
            // (the largest `div` in the tied range) rather than exhaustively
            // finding the smallest one, which is a cosmetic difference with
            // no effect on the achieved baud rate or its error — the
            // property this module actually needs (closest achievable rate)
            // is fully captured by the `error`/`bwpc` comparisons above.
            assert_eq!(bwpc, expected.2, "baud_rate={baud_rate}");
        }
    }

    #[test]
    fn baud_divider_ceiling_candidate_beats_floor_only_search_for_7000_baud() {
        // A concrete, worked demonstration of why this function must check
        // both the floored divider and the next one up: at bwpc=9, the
        // floored divider alone gives a worse match than the divider one
        // above it.
        let denominator = 7_000u32 * 10; // bwpc = 9
        let floor_div_plus_1 = 24_000_000u32 / denominator;
        let floor_actual = 24_000_000u32 / (floor_div_plus_1 * 10);
        let ceil_actual = 24_000_000u32 / ((floor_div_plus_1 + 1) * 10);
        let floor_error = floor_actual.abs_diff(7_000);
        let ceil_error = ceil_actual.abs_diff(7_000);
        assert!(
            ceil_error < floor_error,
            "expected the ceiling candidate to strictly beat the floor candidate at bwpc=9 \
             (floor actual={floor_actual} error={floor_error}, ceil actual={ceil_actual} error={ceil_error})"
        );

        // And compute_baud_divider's actual, global-search result must be
        // at least as good as (and in this case exactly matches) that
        // better, ceiling-derived candidate for bwpc=9 specifically.
        let (_, _, actual) = compute_baud_divider(24_000_000, 7_000).unwrap();
        assert!(actual.abs_diff(7_000) <= ceil_error);
    }

    #[test]
    fn tx_backpressure_boundary_matches_modern_tc32_evidence() {
        // Up to and including the threshold: still room to write.
        assert!(!tx_backpressure(0));
        assert!(!tx_backpressure(TX_FIFO_BACKPRESSURE_THRESHOLD));
        // Strictly above: back off.
        assert!(tx_backpressure(TX_FIFO_BACKPRESSURE_THRESHOLD + 1));
        assert!(tx_backpressure(15)); // max representable nibble value
    }

    #[test]
    fn clear_rx_flag_bit_is_distinct_from_the_rx_error_flag_it_acknowledges() {
        assert_eq!(FLD_UART_CLEAR_RX_FLAG, 0x40);
        assert_eq!(FLD_UART_RX_ERR_FLAG, 0x80);
        assert_ne!(FLD_UART_CLEAR_RX_FLAG, FLD_UART_RX_ERR_FLAG);
        assert_eq!(cleared_rx_status(0x00), 0x40);
        assert_eq!(cleared_rx_status(0x80), 0xC0);
        assert_eq!(cleared_rx_status(0x35), 0x75);
    }

    #[test]
    fn pin_validation_accepts_all_documented_tx_and_rx_routes() {
        // Every documented TX/RX pair should validate against any
        // documented RX/TX partner respectively (this HAL does not
        // restrict which TX pin may pair with which RX pin — only that
        // each is independently one of `uart.h`'s six documented pins).
        for &(tx_port, tx_bit) in &UART_TX_PINS {
            for &(rx_port, rx_bit) in &UART_RX_PINS {
                assert!(
                    validate_pins(&Pin::new(tx_port, tx_bit), &Pin::new(rx_port, rx_bit)).is_ok(),
                    "TX {tx_port:?}{tx_bit} / RX {rx_port:?}{rx_bit} should validate"
                );
            }
        }
    }

    #[test]
    fn pin_validation_rejects_undocumented_pins() {
        // PB1/PB7's own regression case (kept unchanged behavior for the
        // smart-plug profile).
        assert!(validate_pins(&Pin::new(Port::B, 1), &Pin::new(Port::B, 7)).is_ok());
        // A pin from the RX set used as TX, and vice versa, must be
        // rejected: these sets are disjoint per `uart.h`.
        assert_eq!(
            validate_pins(&Pin::new(Port::B, 0), &Pin::new(Port::B, 7)),
            Err(UartError::InvalidPins)
        );
        assert_eq!(
            validate_pins(&Pin::new(Port::B, 1), &Pin::new(Port::B, 1)),
            Err(UartError::InvalidPins)
        );
        // PC1/PA1: real, adjacent mux-tree pins but not documented UART
        // routes (see `gpio.rs`'s own `uart_route_rejects_pins_outside_uart_h`
        // for the disassembly-level detail behind this).
        assert_eq!(
            validate_pins(&Pin::new(Port::C, 1), &Pin::new(Port::B, 7)),
            Err(UartError::InvalidPins)
        );
        assert_eq!(
            validate_pins(&Pin::new(Port::B, 1), &Pin::new(Port::A, 1)),
            Err(UartError::InvalidPins)
        );
    }

    #[test]
    fn rts_pin_validation_accepts_only_documented_routes() {
        for &(port, bit) in &UART_RTS_PINS {
            assert!(validate_rts_pin(&Pin::new(port, bit)).is_ok());
        }
        assert_eq!(
            validate_rts_pin(&Pin::new(Port::A, 0)),
            Err(UartError::InvalidRtsPin)
        );
        // A CTS pin used as RTS must be rejected too — the two sets are
        // disjoint in `uart.h`.
        assert_eq!(
            validate_rts_pin(&Pin::new(Port::A, 3)),
            Err(UartError::InvalidRtsPin)
        );
    }

    #[test]
    fn cts_pin_validation_accepts_only_documented_routes() {
        for &(port, bit) in &UART_CTS_PINS {
            assert!(validate_cts_pin(&Pin::new(port, bit)).is_ok());
        }
        assert_eq!(
            validate_cts_pin(&Pin::new(Port::A, 0)),
            Err(UartError::InvalidCtsPin)
        );
        assert_eq!(
            validate_cts_pin(&Pin::new(Port::A, 4)),
            Err(UartError::InvalidCtsPin)
        );
    }

    #[test]
    fn ctrl0_with_bwpc_preserves_dma_and_irq_enable_bits() {
        // Bits 4..7 (RX/TX DMA + IRQ enables) must survive a BWPC
        // reprogram; only bits 0..3 change. This is the regression test
        // for the plain-overwrite bug `uart_init()`'s disassembly caught
        // (see `FLD_UART_CTRL0_UPPER_MASK`'s doc).
        let previously_irq_enabled = FLD_UART_RX_IRQ_EN | FLD_UART_TX_IRQ_EN;
        assert_eq!(
            ctrl0_with_bwpc(previously_irq_enabled, 9),
            previously_irq_enabled | 9
        );
        // A stale high nibble bit that isn't a real field must still be
        // masked out of the incoming `bwpc` byte (only its low nibble is
        // `FLD_UART_BPWC`).
        assert_eq!(ctrl0_with_bwpc(0, 0xFF), 0x0F);
    }

    #[test]
    fn ctrl1_with_parity_stop_preserves_cts_and_ttl_loopback_bits() {
        // CTS_SELECT|CTS_EN|TTL|LOOPBACK (bits 0,1,6,7) already
        // configured must survive a parity/stop-bit reprogram — the
        // regression test for the second plain-overwrite bug
        // `uart_init()`'s disassembly caught (see
        // `FLD_UART_CTRL1_OWNED_MASK`'s doc).
        let previously_configured = 0b1100_0011u8; // bits 0,1,6,7 set
        let parity_stop = parity_stop_bits(Parity::Odd, StopBits::Two);
        let next = ctrl1_with_parity_stop(previously_configured, parity_stop);
        assert_eq!(next & !FLD_UART_CTRL1_OWNED_MASK, previously_configured);
        assert_eq!(next & FLD_UART_CTRL1_OWNED_MASK, parity_stop);
    }

    #[test]
    fn parity_stop_bits_match_vendor_field_layout() {
        assert_eq!(parity_stop_bits(Parity::None, StopBits::One), 0x00);
        assert_eq!(
            parity_stop_bits(Parity::Even, StopBits::One),
            FLD_UART_CTRL1_PARITY_EN
        );
        assert_eq!(
            parity_stop_bits(Parity::Odd, StopBits::One),
            FLD_UART_CTRL1_PARITY_EN | FLD_UART_CTRL1_PARITY_POLARITY
        );
        assert_eq!(
            parity_stop_bits(Parity::None, StopBits::OneAndHalf),
            StopBits::OneAndHalf as u8
        );
        assert_eq!(
            parity_stop_bits(Parity::None, StopBits::Two),
            StopBits::Two as u8
        );
    }

    #[test]
    fn ctrl1_with_cts_only_touches_cts_select_and_cts_en() {
        let previously_configured = 0b0011_1100u8; // parity+stop bits set
        assert_eq!(
            ctrl1_with_cts(previously_configured, true, true),
            previously_configured | FLD_UART_CTRL1_CTS_EN | FLD_UART_CTRL1_CTS_SELECT
        );
        assert_eq!(
            ctrl1_with_cts(previously_configured, true, false),
            previously_configured | FLD_UART_CTRL1_CTS_EN
        );
        assert_eq!(
            ctrl1_with_cts(
                previously_configured | FLD_UART_CTRL1_CTS_EN | FLD_UART_CTRL1_CTS_SELECT,
                false,
                false
            ),
            previously_configured
        );
    }

    #[test]
    fn rts_ctrl2_fields_encode_threshold_invert_manual_val_and_manual_en() {
        assert_eq!(rts_ctrl2_fields(RtsMode::Auto, 0, false, false), 0);
        assert_eq!(
            rts_ctrl2_fields(RtsMode::Auto, 0xFF, false, false),
            FLD_UART_CTRL2_RTS_TRIG_LVL_MASK // high nibble masked off
        );
        assert_eq!(
            rts_ctrl2_fields(RtsMode::Auto, 0, true, false),
            FLD_UART_CTRL2_RTS_PARITY
        );
        assert_eq!(
            rts_ctrl2_fields(RtsMode::Auto, 0, false, true),
            FLD_UART_CTRL2_RTS_MANUAL_VAL
        );
        assert_eq!(
            rts_ctrl2_fields(RtsMode::Manual, 0, false, false),
            FLD_UART_CTRL2_RTS_MANUAL_EN
        );
        // Invert applies in Manual mode too — the disassembly-proven
        // discrepancy from `uart.h`'s own doc comment.
        assert_eq!(
            rts_ctrl2_fields(RtsMode::Manual, 5, true, true),
            5 | FLD_UART_CTRL2_RTS_PARITY
                | FLD_UART_CTRL2_RTS_MANUAL_VAL
                | FLD_UART_CTRL2_RTS_MANUAL_EN
        );
        // FLD_UART_CTRL2_RTS_EN is never set by this function — the
        // caller ORs it in only when `Enable` is true.
        assert_eq!(
            rts_ctrl2_fields(RtsMode::Manual, 0xF, true, true) & FLD_UART_CTRL2_RTS_EN,
            0
        );
    }

    #[test]
    fn rts_mode_discriminants_match_vendor_enum() {
        assert_eq!(RtsMode::Auto as u8, 0);
        assert_eq!(RtsMode::Manual as u8, 1);
    }

    #[test]
    fn irq_trigger_levels_byte_packs_rx_low_and_tx_high_nibble() {
        assert_eq!(irq_trigger_levels_byte(0, 0), 0);
        assert_eq!(irq_trigger_levels_byte(0xF, 0), 0x0F);
        assert_eq!(irq_trigger_levels_byte(0, 0xF), 0xF0);
        assert_eq!(irq_trigger_levels_byte(3, 5), 0x53);
        // Out-of-range nibble bits are masked off silently, matching the
        // vendor function's own `& 0xF` on both arguments.
        assert_eq!(irq_trigger_levels_byte(0xFF, 0xFF), 0xFF);
    }

    #[test]
    fn decode_events_maps_every_status_bit_independently() {
        assert_eq!(decode_events(0, 0), UartEvents::default());
        assert_eq!(
            decode_events(FLD_UART_IRQ_FLAG, 0),
            UartEvents {
                irq_flag: true,
                ..Default::default()
            }
        );
        assert_eq!(
            decode_events(FLD_UART_RX_ERR_FLAG, 0),
            UartEvents {
                rx_error: true,
                ..Default::default()
            }
        );
        assert_eq!(
            decode_events(0, FLD_UART_TX_DONE),
            UartEvents {
                tx_done: true,
                ..Default::default()
            }
        );
        assert_eq!(
            decode_events(0, FLD_UART_TX_BUF_IRQ),
            UartEvents {
                tx_buf_irq: true,
                ..Default::default()
            }
        );
        assert_eq!(
            decode_events(0, FLD_UART_RX_DONE),
            UartEvents {
                rx_done: true,
                ..Default::default()
            }
        );
        assert_eq!(
            decode_events(0, FLD_UART_RX_BUF_IRQ),
            UartEvents {
                rx_buf_irq: true,
                ..Default::default()
            }
        );
        // All six bits set at once, none stomp on each other.
        assert_eq!(
            decode_events(
                FLD_UART_IRQ_FLAG | FLD_UART_RX_ERR_FLAG,
                FLD_UART_TX_DONE | FLD_UART_TX_BUF_IRQ | FLD_UART_RX_DONE | FLD_UART_RX_BUF_IRQ
            ),
            UartEvents {
                irq_flag: true,
                rx_error: true,
                tx_done: true,
                tx_buf_irq: true,
                rx_done: true,
                rx_buf_irq: true,
            }
        );
    }

    #[test]
    fn mask_err_irq_bit_matches_register_h() {
        assert_eq!(FLD_UART_MASK_ERR_IRQ, 0x80);
    }

    #[test]
    fn rx_tx_irq_enable_bits_are_ctrl0_upper_nibble_bits_6_and_7() {
        assert_eq!(FLD_UART_RX_IRQ_EN, 1 << 6);
        assert_eq!(FLD_UART_TX_IRQ_EN, 1 << 7);
        // Both fit within the mask `ctrl0_with_bwpc` preserves.
        assert_eq!(
            FLD_UART_RX_IRQ_EN & FLD_UART_CTRL0_UPPER_MASK,
            FLD_UART_RX_IRQ_EN
        );
        assert_eq!(
            FLD_UART_TX_IRQ_EN & FLD_UART_CTRL0_UPPER_MASK,
            FLD_UART_TX_IRQ_EN
        );
    }

    #[test]
    fn documented_rts_and_cts_pin_sets_are_disjoint_from_tx_rx_and_each_other() {
        for rts in &UART_RTS_PINS {
            assert!(!UART_TX_PINS.contains(rts));
            assert!(!UART_RX_PINS.contains(rts));
            assert!(!UART_CTS_PINS.contains(rts));
        }
        for cts in &UART_CTS_PINS {
            assert!(!UART_TX_PINS.contains(cts));
            assert!(!UART_RX_PINS.contains(cts));
        }
    }

    #[test]
    fn data_buf_ring_wraps_at_four() {
        let mut index: u8 = 3;
        index = (index + 1) & DATA_BUF_RING_MASK;
        assert_eq!(index, 0);
    }
}
