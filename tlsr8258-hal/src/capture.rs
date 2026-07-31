//! Generic rising-edge GPIO capture, timestamped against the free-running
//! Timer0 tick counter (see [`crate::timer::now_ticks`]).
//!
//! This module adds **no new registers of its own**: it is a reusable
//! consumer of two pieces already implemented and evidenced elsewhere in
//! this crate:
//!
//! * [`crate::gpio::GpioIrqSource`] (`Primary`/`Risc0`/`Risc1`) — three
//!   independent hardware edge-interrupt comparators, each with its own
//!   per-pin enable register and its own bit in `reg_irq_mask`/
//!   `reg_irq_src`. See that type's doc comment in `gpio.rs` for the exact
//!   `platform/chip_8258/gpio.h`/`register.h` citations.
//! * [`crate::timer::now_ticks`] — the free-running Timer0 tick counter
//!   already used by `radio`'s bounded waits.
//!
//! Naming here is deliberately generic (`channel`, not `cf`/`cf1`): this is
//! a reusable HAL facility, not an application driver. A caller wiring up
//! e.g. a BL0937 energy-metering front-end assigns its `CF` pin to
//! [`crate::gpio::GpioIrqSource::Primary`] and its `CF1` pin to
//! [`crate::gpio::GpioIrqSource::Risc0`] (or any other two distinct
//! sources) via [`configure_channel`] — because those are independent
//! hardware comparators, the resulting two channels are told apart by
//! hardware alone, with no software level-polling of "which candidate pin
//! is high right now" needed.
//!
//! # GPIO-source exclusivity
//!
//! A hardware [`crate::gpio::GpioIrqSource`] identifies a comparator, not
//! an individual pin. This module guarantees that it assigns each source to
//! at most one capture channel, but the lower-level public GPIO API can also
//! enable pins on the same source. Callers must therefore dedicate every
//! source passed to [`configure_channel`] to capture for as long as the
//! channel is active. Enabling another pin on that source later would make
//! the single pending bit ambiguous; this module cannot recover which pin
//! caused it and would attribute it to the configured channel. A future
//! source-ownership token can make this invariant compile-time enforced;
//! until then it is an explicit integration requirement, not an implicit
//! claim that the pending bit contains a pin identity.
//!
//! # ISR ordering (required)
//!
//! This crate's radio driver requires its own IRQ vector handler,
//! [`crate::radio::handle_irq`], to run first in every interrupt because it
//! must re-arm RX/clear RF+DMA IRQ sources before any bounded wait
//! elsewhere in the application can observe a consistent state (see that
//! function's own docs and `examples/telink-tlsr8258-router/src/main.rs`'s
//! `irq_handler()`). [`handle_irq`] in this module does not touch the RF/
//! DMA path at all, but an application combining both facilities in one
//! IRQ vector **must** still call them in this order:
//!
//! ```ignore
//! #[no_mangle]
//! extern "C" fn irq_handler() {
//!     tlsr8258_hal::radio::handle_irq();
//!     tlsr8258_hal::capture::handle_irq();
//! }
//! ```
//!
//! Reversing the order does not corrupt capture state, but it delays the
//! radio's mandatory housekeeping by however long [`handle_irq`] (a few
//! register reads/writes per configured channel, bounded and
//! allocation-free) takes, which is exactly the kind of RF turnaround
//! regression this crate's existing docs (see `radio/mod.rs`) call out as
//! unacceptable — so radio must run first.
//!
//! Note also that `radio::handle_irq()` re-enables the global `REG_IRQ_EN`
//! bit near the *end* of its own body (the TLSR8258 clears it automatically
//! on IRQ entry; the vector is responsible for restoring it before return —
//! see that function's own comment). That means by the time
//! [`handle_irq`] runs, global IRQs may already be back on, so it cannot
//! assume IRQ-context alone serializes it against [`configure_channel`]/
//! [`take_event`]/[`overflow_count`] running from the main loop. See
//! "Synchronization" below for how this module actually guarantees that.
//!
//! # Synchronization
//!
//! [`ChannelTable`] and [`EventQueue`] are plain, non-atomic, `&mut self`
//! structures with **no built-in concurrency safety** — pushing from an
//! interrupt while popping from main-line code would be a real data race
//! if nothing else serialized the two call sites (a
//! [`core::sync::atomic::compiler_fence`] only orders a single side's own
//! instructions; it cannot make a genuinely concurrent read/write pair on
//! plain fields sound). The `tc32`-only free functions in this module
//! ([`configure_channel`], [`handle_irq`], [`take_event`],
//! [`overflow_count`]) close that gap by running every access to the single
//! shared `static` instances inside [`crate::mmio::with_irqs_disabled`],
//! the same save/disable/restore critical section `analog_read`/
//! `analog_write` and `radio::hw::mask_cpu_rx_irq` already use elsewhere in
//! this crate. With global IRQs masked for the duration, [`handle_irq`] and
//! any main-loop caller can never execute over the same data at once,
//! regardless of the `REG_IRQ_EN` timing note above. Each critical section
//! is a handful of register/field reads and writes — bounded, and short
//! enough not to meaningfully delay a pending RF interrupt.
//!
//! # Timer wrap
//!
//! [`CaptureEvent::timestamp_ticks`] is a raw Timer0 tick snapshot from a
//! free-running 32-bit counter that wraps roughly every 178 s at 24 MHz
//! (`TICKS_PER_MS = 24_000`, see `timer.rs`). Never subtract two timestamps
//! directly; use [`elapsed_ticks`], which matches the
//! `now_ticks().wrapping_sub(start)` idiom `timer::wait_until` already
//! uses elsewhere in this crate.
//!
//! # Known hardware limitation: same-channel edge coalescing
//!
//! Each [`crate::gpio::GpioIrqSource`] is a single sticky pending bit in
//! `reg_irq_src`, not a counter (see that type's own docs). If the *same*
//! channel's pin produces a second qualifying edge before [`handle_irq`]
//! next runs and clears the first one, hardware has no way to signal that
//! a second edge happened — the pending bit is simply still set, and the
//! second edge is silently absorbed. This is an inherent hardware
//! constraint of the interrupt controller, not a bug in this module or a
//! failure this module can detect; it is unrelated to (and cannot be
//! conflated with) [`EventQueue::overflow_count`], which instead counts a
//! *different*, software-side failure mode: the application not draining
//! [`take_event`] fast enough to keep up with a queue that *did* receive
//! every hardware edge. Applications with a tight same-channel pulse rate
//! (not the BL0937/BL0942 energy-metering pulse trains this module targets,
//! which are on the order of tens of Hz) must budget [`handle_irq`]'s
//! latency accordingly; this module cannot compensate for it.

use crate::gpio::{GpioIrqSource, Port};

/// Number of independent capture channels this module supports — one per
/// [`crate::gpio::GpioIrqSource`] variant (`Primary`, `Risc0`, `Risc1`).
///
/// The `Primary` source shares its core gate with GPIO wake/interrupt users
/// outside this module. Once capture enables that shared gate it deliberately
/// leaves it enabled; this module cannot know whether another owner still
/// needs it, so it does not expose automatic teardown or reconfiguration.
pub const MAX_CHANNELS: usize = 3;

/// Depth of the software event queue drained by [`take_event`]. Sized well
/// above the handful of channels this module supports so a main loop that
/// polls at, say, 10 Hz has ample headroom against BL0937/BL0942-class
/// pulse rates (tens of Hz) before [`EventQueue::overflow_count`] would
/// ever increment.
pub const QUEUE_CAPACITY: usize = 16;

/// A single captured rising edge: which configured channel it came from,
/// and the [`crate::timer::now_ticks`] snapshot taken while servicing the
/// interrupt. See the module docs for required wraparound-safe handling of
/// this timestamp via [`elapsed_ticks`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaptureEvent {
    pub channel: u8,
    pub timestamp_ticks: u32,
}

/// Errors from [`configure_channel`] / [`ChannelTable::configure`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaptureError {
    /// `channel >= MAX_CHANNELS`.
    InvalidChannel,
    /// This channel is already configured for a *different* `(pin,
    /// source)` than requested. This module fails closed instead of
    /// silently re-pointing an already-armed channel: replacing a
    /// channel's source/pin in place would leave the *previous* hardware
    /// source still armed (per-pin enable bit set, global mask bit set)
    /// with nothing left in software tracking it, which is exactly the
    /// kind of unbounded/unaccounted interrupt source this crate's
    /// bounded-everything convention rules out. Re-configuring a channel
    /// with the *same* `(pin, source)` it already has is allowed and is a
    /// no-op (idempotent). To genuinely reassign a channel, tear down its
    /// hardware source yourself first (`gpio::set_global_interrupt_enable`/
    /// `set_source_interrupt_enable(.., false)`) — this module does not
    /// expose an automated teardown (see the module docs).
    ChannelAlreadyConfigured,
    /// This [`GpioIrqSource`] is already assigned to a different channel —
    /// two channels sharing one hardware comparator could never be told
    /// apart, defeating the point of this module.
    SourceAlreadyConfigured,
    /// This physical `(Port, bit)` is already assigned to a different
    /// channel.
    PinAlreadyConfigured,
    /// A [`crate::gpio::GpioError`] surfaced while configuring the pin
    /// (e.g. input-enable failing on a port with no analog input-enable
    /// register).
    Gpio(crate::gpio::GpioError),
}

impl From<crate::gpio::GpioError> for CaptureError {
    fn from(error: crate::gpio::GpioError) -> Self {
        CaptureError::Gpio(error)
    }
}

/// `later.wrapping_sub(earlier)`, the only correct way to compute a period
/// between two [`CaptureEvent::timestamp_ticks`] values across a Timer0
/// wrap — see the module docs. Matches the idiom `timer::wait_until`
/// already uses for the same reason.
pub const fn elapsed_ticks(earlier: u32, later: u32) -> u32 {
    later.wrapping_sub(earlier)
}

/// Which physical pin (and, transitively, which [`GpioIrqSource`]) a
/// configured channel is bound to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ChannelEntry {
    port: Port,
    bit: u8,
    source: GpioIrqSource,
}

/// Channel-to-hardware-source bookkeeping. Plain, `unsafe`-free, and fully
/// host-testable in isolation from real registers — the `tc32`-only
/// [`configure_channel`]/[`handle_irq`] free functions below are thin
/// wrappers around a single static instance of this type plus the actual
/// `gpio::` register calls.
#[derive(Debug, Clone, Copy)]
pub struct ChannelTable {
    entries: [Option<ChannelEntry>; MAX_CHANNELS],
}

impl ChannelTable {
    pub const fn new() -> Self {
        Self {
            entries: [None; MAX_CHANNELS],
        }
    }

    /// Record that `channel` is bound to `source` on `(port, bit)`.
    ///
    /// Fails closed: an out-of-range channel, a source/pin already claimed
    /// by a *different* channel, and — new — a channel already configured
    /// for a *different* `(port, bit, source)` than requested are all
    /// rejected (the last case as [`CaptureError::ChannelAlreadyConfigured`]
    /// specifically). Calling this again for a channel with the exact same
    /// `(port, bit, source)` it already has is allowed and simply confirms
    /// the existing binding (idempotent) — see [`CaptureError`]'s docs for
    /// why replacing a channel's binding outright is not supported.
    pub fn configure(
        &mut self,
        channel: usize,
        port: Port,
        bit: u8,
        source: GpioIrqSource,
    ) -> Result<(), CaptureError> {
        if channel >= MAX_CHANNELS {
            return Err(CaptureError::InvalidChannel);
        }
        let requested = ChannelEntry { port, bit, source };
        if let Some(existing) = self.entries[channel] {
            return if existing == requested {
                Ok(())
            } else {
                Err(CaptureError::ChannelAlreadyConfigured)
            };
        }
        for (index, entry) in self.entries.iter().enumerate() {
            let Some(entry) = entry else { continue };
            debug_assert_ne!(index, channel, "channel slot was just checked empty above");
            if entry.source == source {
                return Err(CaptureError::SourceAlreadyConfigured);
            }
            if entry.port == port && entry.bit == bit {
                return Err(CaptureError::PinAlreadyConfigured);
            }
        }
        self.entries[channel] = Some(requested);
        Ok(())
    }

    /// The [`GpioIrqSource`] bound to `channel`, if any.
    pub fn source(&self, channel: usize) -> Option<GpioIrqSource> {
        self.entries
            .get(channel)
            .copied()
            .flatten()
            .map(|e| e.source)
    }

    /// Number of currently configured channels.
    pub fn configured_count(&self) -> usize {
        self.entries.iter().filter(|e| e.is_some()).count()
    }

    /// Forget every configured channel (does not touch hardware — pair
    /// with re-running [`crate::gpio::set_global_interrupt_enable`] if a
    /// full teardown is required).
    pub fn clear(&mut self) {
        self.entries = [None; MAX_CHANNELS];
    }
}

impl Default for ChannelTable {
    fn default() -> Self {
        Self::new()
    }
}

/// Fixed-capacity, allocation-free ring buffer of [`CaptureEvent`]s.
///
/// This is an ordinary `&mut self` structure with **no concurrency safety
/// of its own** — see the module docs' "Synchronization" section. It is
/// host-testable and driven directly by the `#[cfg(test)]` module below
/// (unlike `radio::hw`'s equivalent `IRQ_RX_QUEUE`, which lives entirely
/// inside a `#[cfg(target_arch = "tc32")]` module and so cannot be
/// exercised on the host); the exact same `push`/`pop` code also runs on
/// hardware, through a single `static mut` instance the `tc32`-only `hw`
/// module below only ever touches from inside
/// [`crate::mmio::with_irqs_disabled`].
#[derive(Debug)]
pub struct EventQueue<const N: usize> {
    ring: [CaptureEvent; N],
    head: u8,
    len: u8,
    overflow_count: u32,
}

impl<const N: usize> EventQueue<N> {
    pub const fn new() -> Self {
        assert!(
            N > 0 && N <= u8::MAX as usize,
            "EventQueue capacity must be 1..=255 to fit the u8 head/len fields"
        );
        Self {
            ring: [CaptureEvent {
                channel: 0,
                timestamp_ticks: 0,
            }; N],
            head: 0,
            len: 0,
            overflow_count: 0,
        }
    }

    /// Push `event`, or count it in [`overflow_count`](Self::overflow_count)
    /// and drop it if the queue is full. Never blocks.
    pub fn push(&mut self, event: CaptureEvent) {
        let len = self.len as usize;
        if len == N {
            self.overflow_count = self.overflow_count.wrapping_add(1);
            return;
        }
        let index = (self.head as usize + len) % N;
        self.ring[index] = event;
        self.len = (len + 1) as u8;
    }

    /// Pop the oldest event, if any. Never blocks.
    pub fn pop(&mut self) -> Option<CaptureEvent> {
        if self.len == 0 {
            return None;
        }
        let head = self.head as usize;
        let event = self.ring[head];
        self.head = ((head + 1) % N) as u8;
        self.len -= 1;
        Some(event)
    }

    /// Count of events dropped because [`push`](Self::push) was called
    /// while the queue was already full — the *software* drain-rate
    /// failure mode, distinct from the hardware same-channel edge
    /// coalescing described in the module docs.
    pub const fn overflow_count(&self) -> u32 {
        self.overflow_count
    }

    pub const fn len(&self) -> usize {
        self.len as usize
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Drop all queued events and reset [`overflow_count`](Self::overflow_count) to 0.
    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
        self.overflow_count = 0;
    }
}

impl<const N: usize> Default for EventQueue<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(target_arch = "tc32")]
mod hw {
    use super::{
        CaptureError, CaptureEvent, ChannelTable, EventQueue, MAX_CHANNELS, QUEUE_CAPACITY,
    };
    use crate::gpio::{self, GpioIrqSource, Pin};
    use crate::mmio::with_irqs_disabled;
    use crate::timer;

    static mut CHANNELS: ChannelTable = ChannelTable::new();
    static mut QUEUE: EventQueue<QUEUE_CAPACITY> = EventQueue::new();

    /// Bind `channel` to `pin`'s rising edge via `source`, and enable the
    /// corresponding hardware path (per-pin source enable, the `Primary`
    /// core-interrupt gate when applicable, and the global
    /// `reg_irq_mask` bit).
    ///
    /// `pin` is borrowed, not consumed: this module stores only its
    /// `(Port, bit)` for channel bookkeeping (see [`ChannelTable`]), not
    /// the ownership token itself, so the caller keeps the `Pin` for any
    /// further per-pin configuration (e.g. `gpio::set_pull`) it needs.
    ///
    /// Configures the pin as a plain GPIO input as a side effect (function
    /// mux back to GPIO, output driver disabled, input buffer enabled)
    /// since a capture channel that is not readable as a GPIO input cannot
    /// latch edges, and a pin left driving as an output — e.g. a reused
    /// pin that was previously configured as an output elsewhere — would
    /// never see the external signal's edges at all (its own output latch
    /// would win the bus). Order matters here: the output driver is
    /// disabled *before* the input buffer is enabled, so there is no
    /// window where both are simultaneously active. Pull resistor
    /// selection is left to the caller since it is board-specific (e.g. a
    /// BL0937 `CF`/`CF1` line's idle level depends on the metering IC and
    /// any external pull already on the board).
    ///
    /// Validates against a *copy* of the current channel table (see
    /// [`ChannelTable::configure`]'s fail-closed rules), then performs the
    /// actual register writes and commits that copy back over [`CHANNELS`],
    /// all inside a single [`with_irqs_disabled`] critical section, so
    /// [`handle_irq`] — which reads [`CHANNELS`] under the same kind of
    /// critical section — never observes a table that names a channel
    /// before its hardware source is actually armed, or vice versa. If any
    /// step fails, neither the table nor `reg_irq_mask`'s bit for `source`
    /// is left changed: the fallible steps run before
    /// `set_global_interrupt_enable`, which is the last hardware write and
    /// the one that actually lets an interrupt reach the core, so a
    /// failure never leaves a live, untracked IRQ source.
    pub fn configure_channel(
        channel: usize,
        pin: &Pin,
        source: GpioIrqSource,
    ) -> Result<(), CaptureError> {
        let (port, bit) = pin.port_and_bit();

        with_irqs_disabled(|| -> Result<(), CaptureError> {
            // Validate and commit against one uninterrupted snapshot. A
            // second configuration attempt (including one from an IRQ
            // context) cannot interleave between validation and commit and
            // overwrite this channel table with a stale copy.
            let mut candidate = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(CHANNELS)) };
            candidate.configure(channel, port, bit, source)?;

            gpio::set_function_gpio(pin);
            // Disable the output driver *before* enabling the input
            // buffer: a reused/previously-output pin left driving would
            // otherwise never show the external signal's edges (see this
            // function's doc). Infallible — mirrors `gpio_set_output_en()`
            // and never touches the analog bus, unlike input-enable below.
            gpio::set_output_enable(pin, false);
            gpio::set_input_enable(pin, true)?;
            // `falling = false` selects the rising edge this module
            // captures.
            gpio::set_interrupt_polarity(pin, false);
            gpio::set_source_interrupt_enable(pin, source, true);
            if matches!(source, GpioIrqSource::Primary) {
                gpio::set_core_interrupt_enable(true);
            }
            // Clear any pending bit latched while function/polarity/enable
            // were being configured before arming the global mask,
            // matching this crate's existing "configure, clear stale
            // pending, then unmask" ordering (see `radio::hw`'s IRQ setup
            // for the same pattern). This is the last fallible-adjacent
            // step before the source can actually reach the core, so the
            // table commit happens immediately after it, still inside this
            // same critical section.
            gpio::clear_interrupt_pending(source);
            gpio::set_global_interrupt_enable(source, true);
            unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(CHANNELS), candidate) };
            Ok(())
        })
    }

    /// ISR-callable: for every configured channel whose source is pending,
    /// timestamp it and push a [`CaptureEvent`], then acknowledge the
    /// source's pending bit. Bounded (at most [`MAX_CHANNELS`] iterations,
    /// no loops, no allocation) and safe to call unconditionally from an
    /// application's IRQ vector after `radio::handle_irq()` — see the
    /// module docs' "ISR ordering" and "Synchronization" sections. The
    /// whole scan runs inside one [`with_irqs_disabled`] critical section
    /// so it can never interleave with [`configure_channel`]/
    /// [`take_event`]/[`overflow_count`] running from the main loop.
    pub fn handle_irq() {
        with_irqs_disabled(|| {
            for channel in 0..MAX_CHANNELS {
                let source = unsafe { (*core::ptr::addr_of!(CHANNELS)).source(channel) };
                let Some(source) = source else { continue };
                if gpio::interrupt_pending(source) {
                    let event = CaptureEvent {
                        channel: channel as u8,
                        timestamp_ticks: timer::now_ticks(),
                    };
                    unsafe { (*core::ptr::addr_of_mut!(QUEUE)).push(event) };
                    gpio::clear_interrupt_pending(source);
                }
            }
        });
    }

    /// Drain the oldest queued [`CaptureEvent`], if any. Intended for a
    /// main-loop poll, not the ISR. Runs inside a
    /// [`with_irqs_disabled`] critical section — see the module docs'
    /// "Synchronization" section.
    pub fn take_event() -> Option<CaptureEvent> {
        with_irqs_disabled(|| unsafe { (*core::ptr::addr_of_mut!(QUEUE)).pop() })
    }

    /// Number of [`CaptureEvent`]s dropped because [`take_event`] was not
    /// called often enough to keep the queue from filling — see the
    /// module docs' distinction between this and the hardware same-channel
    /// coalescing limitation. Runs inside a [`with_irqs_disabled`]
    /// critical section — see the module docs' "Synchronization" section.
    pub fn overflow_count() -> u32 {
        with_irqs_disabled(|| unsafe { (*core::ptr::addr_of!(QUEUE)).overflow_count() })
    }
}

#[cfg(target_arch = "tc32")]
pub use hw::{configure_channel, handle_irq, overflow_count, take_event};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gpio::Port;

    #[test]
    fn elapsed_ticks_handles_normal_and_wrapped_order() {
        assert_eq!(elapsed_ticks(100, 150), 50);
        // Timer0 wrapped between the two samples.
        assert_eq!(elapsed_ticks(u32::MAX - 5, 10), 16);
        assert_eq!(elapsed_ticks(0, 0), 0);
    }

    #[test]
    fn channel_table_rejects_out_of_range_channel() {
        let mut table = ChannelTable::new();
        assert_eq!(
            table.configure(MAX_CHANNELS, Port::B, 5, GpioIrqSource::Primary),
            Err(CaptureError::InvalidChannel)
        );
    }

    #[test]
    fn channel_table_rejects_duplicate_source_on_a_different_channel() {
        let mut table = ChannelTable::new();
        table
            .configure(0, Port::B, 5, GpioIrqSource::Primary)
            .unwrap();
        assert_eq!(
            table.configure(1, Port::B, 6, GpioIrqSource::Primary),
            Err(CaptureError::SourceAlreadyConfigured)
        );
    }

    #[test]
    fn channel_table_rejects_duplicate_pin_on_a_different_channel() {
        let mut table = ChannelTable::new();
        table
            .configure(0, Port::B, 5, GpioIrqSource::Primary)
            .unwrap();
        assert_eq!(
            table.configure(1, Port::B, 5, GpioIrqSource::Risc0),
            Err(CaptureError::PinAlreadyConfigured)
        );
    }

    #[test]
    fn channel_table_rejects_reassigning_an_already_configured_channel() {
        let mut table = ChannelTable::new();
        table
            .configure(0, Port::B, 5, GpioIrqSource::Primary)
            .unwrap();
        // Same channel, *different* pin/source: fails closed instead of
        // silently re-pointing it (see `CaptureError::ChannelAlreadyConfigured`'s
        // docs — the old hardware source would otherwise be left armed).
        assert_eq!(
            table.configure(0, Port::B, 6, GpioIrqSource::Risc0),
            Err(CaptureError::ChannelAlreadyConfigured)
        );
        // The original binding must be unchanged after the rejected call.
        assert_eq!(table.source(0), Some(GpioIrqSource::Primary));
    }

    #[test]
    fn channel_table_allows_idempotent_reconfiguration_of_the_same_channel() {
        let mut table = ChannelTable::new();
        table
            .configure(0, Port::B, 5, GpioIrqSource::Primary)
            .unwrap();
        // Exact same (port, bit, source) again: a no-op, not a conflict.
        table
            .configure(0, Port::B, 5, GpioIrqSource::Primary)
            .unwrap();
        assert_eq!(table.source(0), Some(GpioIrqSource::Primary));
        assert_eq!(table.configured_count(), 1);
    }

    #[test]
    fn channel_table_distinguishes_the_bl0937_style_two_channel_case() {
        // CF -> Primary, CF1 -> Risc0: two channels, hardware-disambiguated.
        let mut table = ChannelTable::new();
        table
            .configure(0, Port::B, 5, GpioIrqSource::Primary)
            .unwrap();
        table
            .configure(1, Port::B, 6, GpioIrqSource::Risc0)
            .unwrap();
        assert_eq!(table.source(0), Some(GpioIrqSource::Primary));
        assert_eq!(table.source(1), Some(GpioIrqSource::Risc0));
        assert_eq!(table.source(2), None);
        assert_eq!(table.configured_count(), 2);
    }

    #[test]
    fn event_queue_pops_in_fifo_order() {
        let mut queue: EventQueue<4> = EventQueue::new();
        queue.push(CaptureEvent {
            channel: 0,
            timestamp_ticks: 10,
        });
        queue.push(CaptureEvent {
            channel: 1,
            timestamp_ticks: 20,
        });
        assert_eq!(
            queue.pop(),
            Some(CaptureEvent {
                channel: 0,
                timestamp_ticks: 10
            })
        );
        assert_eq!(
            queue.pop(),
            Some(CaptureEvent {
                channel: 1,
                timestamp_ticks: 20
            })
        );
        assert_eq!(queue.pop(), None);
    }

    #[test]
    fn event_queue_wraps_the_ring_index() {
        let mut queue: EventQueue<2> = EventQueue::new();
        queue.push(CaptureEvent {
            channel: 0,
            timestamp_ticks: 1,
        });
        queue.push(CaptureEvent {
            channel: 0,
            timestamp_ticks: 2,
        });
        assert_eq!(queue.pop().unwrap().timestamp_ticks, 1);
        // head has advanced; this push must land back at index 0.
        queue.push(CaptureEvent {
            channel: 0,
            timestamp_ticks: 3,
        });
        assert_eq!(queue.pop().unwrap().timestamp_ticks, 2);
        assert_eq!(queue.pop().unwrap().timestamp_ticks, 3);
        assert_eq!(queue.pop(), None);
    }

    #[test]
    fn event_queue_counts_overflow_instead_of_blocking_or_panicking() {
        let mut queue: EventQueue<2> = EventQueue::new();
        queue.push(CaptureEvent {
            channel: 0,
            timestamp_ticks: 1,
        });
        queue.push(CaptureEvent {
            channel: 0,
            timestamp_ticks: 2,
        });
        assert_eq!(queue.overflow_count(), 0);
        // Queue is full; this must be dropped and counted, not block.
        queue.push(CaptureEvent {
            channel: 0,
            timestamp_ticks: 3,
        });
        assert_eq!(queue.overflow_count(), 1);
        assert_eq!(queue.len(), 2);
        // The two events that *did* fit are still delivered in order.
        assert_eq!(queue.pop().unwrap().timestamp_ticks, 1);
        assert_eq!(queue.pop().unwrap().timestamp_ticks, 2);
    }

    #[test]
    fn event_queue_clear_resets_state_and_overflow_count() {
        let mut queue: EventQueue<2> = EventQueue::new();
        queue.push(CaptureEvent {
            channel: 0,
            timestamp_ticks: 1,
        });
        queue.push(CaptureEvent {
            channel: 0,
            timestamp_ticks: 2,
        });
        queue.push(CaptureEvent {
            channel: 0,
            timestamp_ticks: 3,
        });
        assert_eq!(queue.overflow_count(), 1);
        queue.clear();
        assert_eq!(queue.overflow_count(), 0);
        assert!(queue.is_empty());
        assert_eq!(queue.pop(), None);
    }

    #[test]
    fn event_queue_is_empty_reports_correctly() {
        let mut queue: EventQueue<4> = EventQueue::new();
        assert!(queue.is_empty());
        queue.push(CaptureEvent {
            channel: 0,
            timestamp_ticks: 1,
        });
        assert!(!queue.is_empty());
    }

    #[test]
    fn max_channels_matches_the_number_of_gpio_irq_sources() {
        // One capture channel per independent hardware comparator.
        assert_eq!(MAX_CHANNELS, 3);
    }
}
