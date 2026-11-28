//! BL702 parent (router) radio: hardware Imm-Ack with a hardware Frame
//! Pending table and an interrupt-driven receive ring.
//!
//! The end-device [`Bl702RadioPhy`] keeps the hardware-tested polling path
//! (software Imm-Ack from the core, receiver on only inside `receive`).
//! That is too late for a parent: a router must acknowledge child Data
//! Requests inside the Imm-Ack turnaround and set Frame Pending from a table
//! the radio itself reads (see `docs/book/src/advanced/soft-mac-porting.md`,
//! "Frame Pending: choose one owner"). [`Bl702ParentRadioPhy`] therefore:
//!
//! - programs the M154 address filter (`PAN << 16 | short`, extended
//!   address) and turns on the M154 hardware auto-ACK whenever the core
//!   installs a filter; scans (`filter == None`) return to promiscuous
//!   receive with auto-ACK off;
//! - owns the M154 Frame Pending table (FPT). The core runs with a
//!   zero-depth software pending table ([`Bl702RouterRole`]), so the
//!   hardware is the only Frame Pending owner;
//! - keeps the receiver armed between MAC operations (`macRxOnWhenIdle`)
//!   and copies every completed frame from the M154 RX buffer into a
//!   bounded ring in [`on_interrupt`], because a hardware-acknowledged frame
//!   must not be lost when the next frame overwrites the single RX buffer.
//!
//! # Frame Pending table policy
//!
//! The M154 sends FP=1 for a source **missing** from the table (vendor
//! `lmac154.h`: "if the address is not found in the table ... the frame
//! pending bit of hw auto tx ack is 1"). Spurious FP=1 only costs a child a
//! short receive window; a missing FP=1 strands queued data. The table is
//! therefore used as follows ([`FramePendingTable`], host-tested):
//!
//! - short sources get an explicit entry with the requested bit;
//! - a full table stores nothing, so the source keeps the miss default
//!   FP=1 (correct for `pending == true`, a spurious FP=1 for
//!   `pending == false`); [`Bl702ParentRadioPhy::stats`] counts it;
//! - extended sources (association and rejoin polls) are never stored and
//!   always see the miss default FP=1, which is exactly what an
//!   association transaction needs.
//!
//! Every queued Data Request carries its Imm-Ack's Frame Pending decision
//! ([`PhyRxFrame::acked_without_frame_pending`]; FP = 0 means the child
//! turned its receiver off, so a frame queued after that ACK waits for its
//! next poll). The M154 reports no per-frame result, so the interrupt
//! derives it from the table mirror when it copies the poll
//! (`mark_poll_ack`), and [`RadioPhy::set_ack_frame_pending`] first queues
//! the frames already completed under the old table, then changes mirror
//! and registers, all in one critical section. Residual race (unproven
//! timing, no vendor documentation of when the M154 reads its table): a
//! poll whose reception straddles a table update is attributed to the new
//! table. If the M154 had looked the source up before an update that arms
//! it, the poll is reported FP = 1 although its ACK said FP = 0 and is
//! served at once (the behaviour before this record); the reverse only
//! delays the frame to the next poll.
//!
//! # Interrupt wiring
//!
//! The composition root routes `bl702_hal::interrupt::M154_IRQ` (CLIC
//! source `16 + 48`) to [`on_interrupt`] and enables it after constructing
//! this PHY. The M154 interrupt-enable mask is the one the end-device path
//! already programs in `initialize_m154`.
//! [`RadioPhy::receive`] also services pending status itself, so a missing
//! interrupt degrades to receive-window polling rather than to silence.
//!
//! # Validation
//!
//! Compiles for `riscv32imc` and the table/ring logic is host-tested.
//! XT-ZB1 diagnostic capture: the M154 CLIC source fires and a received
//! beacon raises only status `0x0010_0000` (an `IRQ_RX_AUX` bit), which
//! the ISR must treat as frame completion.
//! **Not hardware-tested:** the auto-ACK enable bit, the address and FPT
//! register encodings and the receiver re-arm after a hardware ACK come
//! from vendor headers and the previous register reverse-engineering.

use core::cell::RefCell;

use embassy_futures::select::select;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use zigbee_types::MacAddress;

use super::radio_phy::{
    Bl702RadioPhy, TX_DONE_TIMEOUT, TxResult, clear_tx_events, decode_tx_result, disable_rx,
    enable_rx, read_rx_frame, reset_tx, run_cca_once, trigger_tx, validate_frame, write_tx_frame,
    yield_now,
};
use super::registers::*;
use crate::ack_pending::data_request_source;
use crate::pib::MacPib;
use crate::{
    MacError, MacRole, Parent, PhyAddressFilter, PhyCapabilities, PhyError, PhyRxFrame,
    PlatformServices, RadioPhy, SoftMacCore,
};

/// Parent role of the BL702 router: eight queued MAC command events, eight
/// concurrent Association Response transactions and **no** software Frame
/// Pending table, because the M154 FPT is the single Frame Pending owner.
pub type Bl702RouterRole = Parent<8, 8, 0>;

/// Retained-RX depth of the always-on router (child and neighbour traffic
/// arrives between runtime receive calls).
pub const BL702_ROUTER_PENDING_RX: usize = 8;

/// Shared software MAC, parent role, over the BL702 parent radio.
pub type Bl702RouterMac =
    SoftMacCore<Bl702ParentRadioPhy, Bl702RouterRole, BL702_ROUTER_PENDING_RX>;

/// Frames the ISR can hold until the MAC's next `receive`.
const RX_RING: usize = 4;

/// Short-address entries the driver uses (the hardware has 128 slots). A
/// router's NWK child table holds at most 20 children (`nwkMaxChildren`).
pub const FPT_ENTRIES: usize = 32;

const _: () = assert!(FPT_ENTRIES <= 32, "FPT bitmaps are one 32-bit word");

/// Status bits after which the M154 RX buffer holds a complete frame, as in
/// the vendor `LMAC154_ISR`: the frame-received events `IRQ_RX_AUX`
/// (XT-ZB1 capture: a beacon during an active scan raises `0x0010_0000`
/// and nothing else), the promiscuous RX done and the received-ACK event.
/// `IRQ_RX_CRC` is not a completion: with `M154_RX_STATUS[0]` set it marks
/// a CRC failure, otherwise the frame arrives through these bits.
const IRQ_RX_COMPLETE: u32 = IRQ_RX_DONE | IRQ_RX_AUX | IRQ_RX_ACK;

/// Outcome of a [`FramePendingTable::update`] for the register writer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FptUpdate {
    /// The entry already held this bit.
    Unchanged,
    /// Existing `slot` changes its pending bit.
    Updated { slot: usize },
    /// New entry in `slot`: write the address, the bit, then mark it valid.
    Inserted { slot: usize },
    /// No free slot: nothing stored, the source keeps the hardware's miss
    /// default (Frame Pending set).
    Full,
}

/// Software mirror of the short-address part of the M154 Frame Pending
/// table, so lookups never read the radio registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FramePendingTable<const N: usize> {
    keys: [u16; N],
    valid: u32,
    pending: u32,
}

impl<const N: usize> Default for FramePendingTable<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> FramePendingTable<N> {
    pub const fn new() -> Self {
        Self {
            keys: [0; N],
            valid: 0,
            pending: 0,
        }
    }

    fn find(&self, key: u16) -> Option<usize> {
        (0..N).find(|&slot| self.valid & (1 << slot) != 0 && self.keys[slot] == key)
    }

    /// Store `pending` for `key`.
    pub fn update(&mut self, key: u16, pending: bool) -> FptUpdate {
        if let Some(slot) = self.find(key) {
            if (self.pending & (1 << slot) != 0) == pending {
                return FptUpdate::Unchanged;
            }
            self.set_pending_bit(slot, pending);
            return FptUpdate::Updated { slot };
        }
        let Some(slot) = (0..N).find(|&slot| self.valid & (1 << slot) == 0) else {
            return FptUpdate::Full;
        };
        self.keys[slot] = key;
        self.set_pending_bit(slot, pending);
        self.valid |= 1 << slot;
        FptUpdate::Inserted { slot }
    }

    fn set_pending_bit(&mut self, slot: usize, pending: bool) {
        if pending {
            self.pending |= 1 << slot;
        } else {
            self.pending &= !(1 << slot);
        }
    }

    /// Frame Pending bit the hardware Imm-Ack carries for a Data Request
    /// from short address `key` (a miss sends FP=1).
    pub fn ack_frame_pending(&self, key: u16) -> bool {
        self.find(key)
            .is_none_or(|slot| self.pending & (1 << slot) != 0)
    }

    pub fn clear(&mut self) {
        *self = Self::new();
    }

    pub fn len(&self) -> usize {
        self.valid.count_ones() as usize
    }

    pub fn is_empty(&self) -> bool {
        self.valid == 0
    }
}

/// Fixed-capacity FIFO between the ISR (producer) and `receive` (consumer).
struct RxRing<const N: usize> {
    frames: [PhyRxFrame; N],
    head: usize,
    len: usize,
}

impl<const N: usize> RxRing<N> {
    const fn new() -> Self {
        Self {
            frames: [PhyRxFrame::EMPTY; N],
            head: 0,
            len: 0,
        }
    }

    /// Slot for the next frame, or `None` when every slot is occupied.
    fn tail_slot(&mut self) -> Option<&mut PhyRxFrame> {
        (self.len < N).then(|| &mut self.frames[(self.head + self.len) % N])
    }

    fn commit_tail(&mut self) {
        self.len += 1;
    }

    fn pop_into(&mut self, frame: &mut PhyRxFrame) -> bool {
        if self.len == 0 {
            return false;
        }
        *frame = self.frames[self.head];
        self.head = (self.head + 1) % N;
        self.len -= 1;
        true
    }

    /// Copy one completed frame into the ring with `fill` and record its
    /// Imm-Ack's Frame Pending decision ([`mark_poll_ack`]). `None` when
    /// every slot is occupied (the frame is lost with its decision), else
    /// whether `fill` produced a frame.
    fn queue_received<const T: usize>(
        &mut self,
        auto_ack: bool,
        fpt: &FramePendingTable<T>,
        fill: impl FnOnce(&mut PhyRxFrame) -> Result<(), PhyError>,
    ) -> Option<bool> {
        let slot = self.tail_slot()?;
        if fill(slot).is_err() {
            return Some(false);
        }
        mark_poll_ack(slot, auto_ack, fpt);
        self.commit_tail();
        Some(true)
    }
}

/// Record the M154 auto-ACK's Frame Pending decision with a received Data
/// Request ([`PhyRxFrame::acked_without_frame_pending`]): the child turned
/// its receiver off after FP = 0, so a frame queued after that ACK waits for
/// its next poll.
///
/// The M154 reports no per-frame Frame Pending result, so the decision is
/// derived from the table it acknowledged with: the mirror of its FPT,
/// updated only together with the registers and only after the frames
/// already completed under the old table were queued
/// ([`RadioPhy::set_ack_frame_pending`]). Short sources follow their entry
/// (a miss is FP = 1); extended sources always see the miss default FP = 1.
fn mark_poll_ack<const T: usize>(
    frame: &mut PhyRxFrame,
    auto_ack: bool,
    fpt: &FramePendingTable<T>,
) {
    let acked_without = auto_ack
        && matches!(
            data_request_source(frame.as_slice()),
            Some(MacAddress::Short(_, short)) if !fpt.ack_frame_pending(short.0)
        );
    frame.set_acked_without_frame_pending(acked_without);
}

/// Receive-path counters of the parent radio.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Bl702ParentStats {
    /// Completed frames dropped because the ISR ring was full.
    pub rx_overruns: u32,
    /// Frames that failed the hardware CRC check.
    pub crc_errors: u32,
    /// `set_ack_frame_pending` calls that found the table full (the source
    /// keeps the FP=1 miss default).
    pub fpt_full: u32,
}

/// State shared by [`on_interrupt`] and the radio methods, always accessed
/// inside the single-hart critical section.
struct Shared {
    ring: RxRing<RX_RING>,
    /// Mirror of the short-address M154 Frame Pending table. Here rather
    /// than in the PHY so the interrupt reads the table the hardware
    /// acknowledged with when it queues a poll.
    fpt: FramePendingTable<FPT_ENTRIES>,
    /// M154 hardware auto-ACK on (an address filter is installed).
    auto_ack: bool,
    /// TX status bits the ISR cleared from `M154_IRQ_STATUS`.
    tx_status: u32,
    /// A transmission owns the radio; the ISR must not re-arm RX.
    tx_active: bool,
    rx_on_when_idle: bool,
    rx_overruns: u32,
    crc_errors: u32,
}

impl Shared {
    const fn new() -> Self {
        Self {
            ring: RxRing::new(),
            fpt: FramePendingTable::new(),
            auto_ack: false,
            tx_status: 0,
            tx_active: false,
            rx_on_when_idle: false,
            rx_overruns: 0,
            crc_errors: 0,
        }
    }

    /// Consume pending M154 status: latch TX results, copy a completed
    /// frame into the ring and re-arm the receiver. Returns whether a frame
    /// was queued.
    fn service(&mut self) -> bool {
        let status = read32(M154_IRQ_STATUS);
        if status == 0 {
            return false;
        }
        // Every read bit is cleared so the level interrupt cannot storm;
        // TX results are kept for the transmit poller.
        write32(M154_IRQ_CLEAR, status);
        self.tx_status |= status & IRQ_TX_MASK;

        let mut queued = false;
        if status & IRQ_RX_CRC != 0 && read32(M154_RX_STATUS) & 1 != 0 {
            self.crc_errors = self.crc_errors.wrapping_add(1);
        } else if status & IRQ_RX_COMPLETE != 0 {
            match self
                .ring
                .queue_received(self.auto_ack, &self.fpt, read_rx_frame)
            {
                Some(stored) => queued = stored,
                None => self.rx_overruns = self.rx_overruns.wrapping_add(1),
            }
        }
        if self.rx_on_when_idle && !self.tx_active {
            // No-op while the hardware still sends the Imm-Ack; its
            // auto-ACK-done event (`0x2000_0000`, vendor
            // `lmac154_hwAutoTxAckDoneEvent`) re-enters here and re-arms.
            let _ = enable_rx();
        }
        queued
    }
}

static SHARED: Mutex<CriticalSectionRawMutex, RefCell<Shared>> =
    Mutex::new(RefCell::new(Shared::new()));
static RX_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

fn with_shared<R>(operation: impl FnOnce(&mut Shared) -> R) -> R {
    SHARED.lock(|shared| operation(&mut shared.borrow_mut()))
}

/// Arm the receiver from task context. Inside the critical section, so the
/// read-modify-write of `M154_RX_CONTROL` cannot interleave with the ISR's
/// re-arm. Does nothing while the radio is not idle.
fn rx_on() {
    with_shared(|_| enable_rx());
}

/// Stop the receiver from task context (see [`rx_on`]).
fn rx_off() {
    with_shared(|_| disable_rx());
}

/// M154 interrupt service routine; call it from the CLIC dispatcher for
/// `bl702_hal::interrupt::M154_IRQ`. Bounded: one status read, at most one
/// 127-byte copy and one receiver re-arm, no logging.
pub fn on_interrupt() {
    if with_shared(Shared::service) {
        RX_READY.signal(());
    }
}

/// BL702 radio with hardware auto-ACK, hardware Frame Pending table and
/// interrupt-driven receive, for parent (router) roles.
///
/// Channel, TX power, energy detect, the clock and the SEC_ENG AES engine
/// are delegated to the wrapped end-device [`Bl702RadioPhy`].
pub struct Bl702ParentRadioPhy {
    inner: Bl702RadioPhy,
    fpt_full: u32,
}

impl Bl702RadioPhy {
    /// Switch an initialized radio to the parent receive path.
    ///
    /// The receiver stays off and the hardware FPT is empty until the MAC
    /// configures them. The composition root must route
    /// `bl702_hal::interrupt::M154_IRQ` to [`on_interrupt`] and enable that
    /// source.
    pub fn into_parent_phy(self) -> Bl702ParentRadioPhy {
        disable_rx();
        clear_hardware_fpt();
        write32(M154_IRQ_CLEAR, read32(M154_IRQ_STATUS));
        with_shared(|shared| *shared = Shared::new());
        Bl702ParentRadioPhy {
            inner: self,
            fpt_full: 0,
        }
    }
}

impl Bl702ParentRadioPhy {
    /// Build a parent-role MAC, normally [`Bl702RouterMac`].
    #[inline(never)]
    pub fn into_parent_mac<R: MacRole, const RX: usize>(
        self,
        pib: MacPib,
    ) -> Result<SoftMacCore<Self, R, RX>, MacError> {
        SoftMacCore::new_custom(self, pib)
    }

    /// Receive-path and Frame Pending table counters.
    pub fn stats(&self) -> Bl702ParentStats {
        let (rx_overruns, crc_errors) =
            with_shared(|shared| (shared.rx_overruns, shared.crc_errors));
        Bl702ParentStats {
            rx_overruns,
            crc_errors,
            fpt_full: self.fpt_full,
        }
    }

    async fn transmit(&mut self, frame: &[u8], run_cca: bool) -> Result<(), PhyError> {
        validate_frame(frame)?;
        let _owner = TxOwner::claim();

        if run_cca {
            let (busy, _) = run_cca_once()?;
            if busy {
                return Err(PhyError::ChannelBusy);
            }
        }

        clear_tx_events();
        with_shared(|shared| shared.tx_status = 0);
        update32(M154_CSMA_CONTROL, 1, 0);
        write_tx_frame(frame);
        trigger_tx()?;

        let deadline = Instant::now() + TX_DONE_TIMEOUT;
        loop {
            let status = with_shared(|shared| {
                let hardware = read32(M154_IRQ_STATUS) & IRQ_TX_MASK;
                if hardware != 0 {
                    write32(M154_IRQ_CLEAR, hardware);
                }
                hardware | core::mem::take(&mut shared.tx_status)
            });
            if let Some(result) = decode_tx_result(status) {
                return match result {
                    TxResult::Finished => Ok(()),
                    TxResult::ChannelBusy => Err(PhyError::ChannelBusy),
                    TxResult::Aborted | TxResult::HardwareError => Err(PhyError::Hardware),
                };
            }
            if Instant::now() >= deadline {
                reset_tx();
                return Err(PhyError::Hardware);
            }
            yield_now().await;
        }
    }
}

/// Holds the radio for one transmission; on drop (completion or a
/// cancelled future) hands the receiver back to `macRxOnWhenIdle`.
struct TxOwner;

impl TxOwner {
    fn claim() -> Self {
        with_shared(|shared| shared.tx_active = true);
        rx_off();
        Self
    }
}

impl Drop for TxOwner {
    fn drop(&mut self) {
        let rx_on_when_idle = with_shared(|shared| {
            shared.tx_active = false;
            shared.rx_on_when_idle
        });
        if rx_on_when_idle {
            rx_on();
        }
    }
}

/// Turns the receiver off after `receive` unless `macRxOnWhenIdle` keeps
/// it on.
struct ReceiveWindow;

impl Drop for ReceiveWindow {
    fn drop(&mut self) {
        if !with_shared(|shared| shared.rx_on_when_idle) {
            rx_off();
        }
    }
}

impl RadioPhy for Bl702ParentRadioPhy {
    /// Software CSMA-CA and ACK wait; the receive side is hardware-ACKed.
    type Tx = crate::SoftwareTx;

    fn capabilities(&self) -> PhyCapabilities {
        PhyCapabilities {
            hardware_auto_ack: true,
            hardware_address_filter: true,
            ..self.inner.capabilities()
        }
    }

    async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError> {
        self.transmit(frame, true).await
    }

    /// Never called: the M154 sends every Imm-Ack.
    async fn send_ack(&mut self, _sequence: u8, _frame_pending: bool) -> Result<(), PhyError> {
        Err(PhyError::Unsupported)
    }

    async fn receive(&mut self, timeout_us: u32, frame: &mut PhyRxFrame) -> Result<bool, PhyError> {
        let _window = ReceiveWindow;
        let deadline = Instant::now() + Duration::from_micros(u64::from(timeout_us));
        loop {
            let popped = with_shared(|shared| {
                shared.service();
                shared.ring.pop_into(frame)
            });
            if popped {
                return Ok(true);
            }
            rx_on();
            if Instant::now() >= deadline {
                return Ok(false);
            }
            select(RX_READY.wait(), Timer::at(deadline)).await;
        }
    }

    fn set_channel(&mut self, channel: u8) -> Result<(), PhyError> {
        rx_off();
        let result = self.inner.set_channel(channel);
        if with_shared(|shared| shared.rx_on_when_idle) {
            rx_on();
        }
        result
    }

    fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
        self.inner.set_tx_power(dbm)
    }

    async fn energy_detect(&mut self, duration_us: u32) -> Result<u8, PhyError> {
        self.inner.energy_detect(duration_us).await
    }

    fn set_address_filter(&mut self, filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
        with_shared(|shared| {
            shared.auto_ack = filter.is_some();
            match filter {
                Some(filter) => {
                    let (pan_short, long_lo, long_hi) = filter_words(&filter);
                    write32(M154_PAN_SHORT, pan_short);
                    write32(M154_LONG_LO, long_lo);
                    write32(M154_LONG_HI, long_hi);
                    clear32(M154_FILTER_CONTROL, FILTER_PROMISCUOUS);
                    set32(M154_TX_CONTROL, TX_CONTROL_AUTO_ACK);
                }
                None => {
                    // Scans hear every beacon and acknowledge nothing.
                    clear32(M154_TX_CONTROL, TX_CONTROL_AUTO_ACK);
                    set32(M154_FILTER_CONTROL, FILTER_PROMISCUOUS);
                }
            }
        });
        Ok(())
    }

    fn set_ack_frame_pending(
        &mut self,
        source: &MacAddress,
        pending: bool,
    ) -> Result<(), PhyError> {
        // Extended sources always see the miss default FP=1.
        let MacAddress::Short(_, short) = *source else {
            return Ok(());
        };
        let (update, queued) = with_shared(|shared| {
            // Frames the M154 already acknowledged under the current table
            // are queued, with that table's decision, before it changes.
            let queued = shared.service();
            let update = shared.fpt.update(short.0, pending);
            match update {
                FptUpdate::Unchanged | FptUpdate::Full => {}
                FptUpdate::Updated { slot } => write_fpt_bit(M154_FPT_PENDING, slot, pending),
                FptUpdate::Inserted { slot } => {
                    write_fpt_slot(slot, short.0);
                    write_fpt_bit(M154_FPT_LONG, slot, false);
                    write_fpt_bit(M154_FPT_PENDING, slot, pending);
                    write_fpt_bit(M154_FPT_VALID, slot, true);
                }
            }
            (update, queued)
        });
        if queued {
            RX_READY.signal(());
        }
        if update == FptUpdate::Full {
            self.fpt_full = self.fpt_full.wrapping_add(1);
        }
        Ok(())
    }

    fn clear_ack_frame_pending(&mut self) {
        with_shared(|shared| {
            shared.fpt.clear();
            clear_hardware_fpt();
        });
    }

    fn set_rx_on_when_idle(&mut self, enabled: bool) {
        let transmitting = with_shared(|shared| {
            shared.rx_on_when_idle = enabled;
            shared.tx_active
        });
        if transmitting {
            return;
        }
        if enabled {
            rx_on();
        } else {
            rx_off();
        }
    }

    /// A parent never sleeps; refuse instead of silently dropping the
    /// always-on receiver.
    fn sleep(&mut self) -> Result<(), PhyError> {
        Err(PhyError::Unsupported)
    }
}

impl PlatformServices for Bl702ParentRadioPhy {
    fn monotonic_micros(&self) -> u32 {
        self.inner.monotonic_micros()
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        self.inner.delay_micros(duration_us).await;
    }

    fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
        self.inner.fill_random(output)
    }
}

#[cfg(not(feature = "hardware-aes-bl702"))]
impl zigbee_crypto::ForwardAesProvider for Bl702ParentRadioPhy {}

/// Delegates to the SEC_ENG engine owned by the wrapped radio.
#[cfg(feature = "hardware-aes-bl702")]
impl zigbee_crypto::ForwardAesProvider for Bl702ParentRadioPhy {
    fn forward_cipher(
        &mut self,
        key: &zigbee_crypto::AesKey,
    ) -> impl zigbee_crypto::Aes128Forward + '_ {
        self.inner.forward_cipher(key)
    }
}

/// `M154_PAN_SHORT`, `M154_LONG_LO` and `M154_LONG_HI` for `filter`.
const fn filter_words(filter: &PhyAddressFilter) -> (u32, u32, u32) {
    let address = filter.extended_address;
    (
        ((filter.pan_id.0 as u32) << 16) | filter.short_address.0 as u32,
        u32::from_le_bytes([address[0], address[1], address[2], address[3]]),
        u32::from_le_bytes([address[4], address[5], address[6], address[7]]),
    )
}

/// Register word and bit of FPT `slot` in a 128-bit bitmap at `base`.
const fn fpt_bit(base: u32, slot: usize) -> (u32, u32) {
    (base + (slot as u32 / 32) * 4, 1 << (slot % 32))
}

/// Register word and half-word shift of FPT address `slot`.
const fn fpt_slot(slot: usize) -> (u32, u32) {
    (
        M154_FPT_SLOTS + (slot as u32 / 2) * 4,
        (slot as u32 % 2) * 16,
    )
}

fn write_fpt_bit(base: u32, slot: usize, set: bool) {
    let (address, bit) = fpt_bit(base, slot);
    if set {
        set32(address, bit);
    } else {
        clear32(address, bit);
    }
}

fn write_fpt_slot(slot: usize, key: u16) {
    let (address, shift) = fpt_slot(slot);
    update32(address, 0xffff << shift, u32::from(key) << shift);
}

fn clear_hardware_fpt() {
    for word in 0..4 {
        write32(M154_FPT_VALID + word * 4, 0);
        write32(M154_FPT_LONG + word * 4, 0);
        write32(M154_FPT_PENDING + word * 4, 0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_types::{PanId, ShortAddress};

    #[test]
    fn missing_sources_keep_the_hardware_frame_pending_default() {
        let table = FramePendingTable::<4>::new();
        assert!(table.ack_frame_pending(0x1234));
        assert!(table.is_empty());
    }

    #[test]
    fn explicit_entries_carry_the_requested_bit() {
        let mut table = FramePendingTable::<4>::new();
        assert_eq!(table.update(0x1234, true), FptUpdate::Inserted { slot: 0 });
        assert!(table.ack_frame_pending(0x1234));
        assert_eq!(table.update(0x1234, false), FptUpdate::Updated { slot: 0 });
        assert!(!table.ack_frame_pending(0x1234));
        assert_eq!(table.update(0x1234, false), FptUpdate::Unchanged);
        assert_eq!(table.update(0x5678, false), FptUpdate::Inserted { slot: 1 });
        assert!(!table.ack_frame_pending(0x5678));
        assert_eq!(table.len(), 2);
    }

    #[test]
    fn a_full_table_never_hides_pending_data() {
        let mut table = FramePendingTable::<2>::new();
        table.update(1, false);
        table.update(2, false);
        // Unseen source with queued data: nothing stored, miss → FP=1.
        assert_eq!(table.update(3, true), FptUpdate::Full);
        assert!(table.ack_frame_pending(3));
        // Unseen source without data: spurious FP=1, never a lost frame.
        assert_eq!(table.update(4, false), FptUpdate::Full);
        assert!(table.ack_frame_pending(4));
        // Stored sources still update in place.
        assert_eq!(table.update(2, true), FptUpdate::Updated { slot: 1 });
        assert!(table.ack_frame_pending(2));
    }

    #[test]
    fn clear_forgets_every_source() {
        let mut table = FramePendingTable::<2>::new();
        table.update(7, false);
        table.clear();
        assert!(table.is_empty());
        assert!(table.ack_frame_pending(7));
        assert_eq!(table.update(9, false), FptUpdate::Inserted { slot: 0 });
    }

    #[test]
    fn rx_ring_is_fifo_and_bounded() {
        let mut ring = RxRing::<2>::new();
        let mut out = PhyRxFrame::EMPTY;
        assert!(!ring.pop_into(&mut out));
        for byte in 1..=2u8 {
            ring.tail_slot().unwrap().set(&[byte; 3], byte).unwrap();
            ring.commit_tail();
        }
        assert!(ring.tail_slot().is_none());
        assert!(ring.pop_into(&mut out));
        assert_eq!((out.as_slice(), out.lqi), (&[1u8; 3][..], 1));
        ring.tail_slot().unwrap().set(&[3; 4], 3).unwrap();
        ring.commit_tail();
        assert!(ring.pop_into(&mut out));
        assert_eq!(out.as_slice(), &[2u8; 3]);
        assert!(ring.pop_into(&mut out));
        assert_eq!(out.as_slice(), &[3u8; 4]);
        assert!(!ring.pop_into(&mut out));
        assert!(ring.tail_slot().is_some());
    }

    /// Data Request from short `child` to the parent 0x0000 (PAN 0x1A62).
    fn poll(sequence: u8, child: u16) -> [u8; 10] {
        let [c0, c1] = child.to_le_bytes();
        [0x63, 0x88, sequence, 0x62, 0x1A, 0x00, 0x00, c0, c1, 0x04]
    }

    fn fill(frame: &[u8]) -> impl FnOnce(&mut PhyRxFrame) -> Result<(), PhyError> + '_ {
        move |slot| slot.set(frame, 200)
    }

    /// The secured-rejoin sequence of #23 on the BL702 parent: the child is
    /// in the table without data, polls (FP = 0), the response is armed,
    /// it polls again (FP = 1). Each queued poll keeps its own decision.
    #[test]
    fn each_queued_poll_carries_its_ack_decision() {
        let mut ring = RxRing::<4>::new();
        let mut fpt = FramePendingTable::<4>::new();
        fpt.update(0x12D2, false);
        assert_eq!(
            ring.queue_received(true, &fpt, fill(&poll(0x10, 0x12D2))),
            Some(true)
        );
        fpt.update(0x12D2, true);
        assert_eq!(
            ring.queue_received(true, &fpt, fill(&poll(0x11, 0x12D2))),
            Some(true)
        );
        // A child the table does not hold gets the hardware's FP = 1.
        assert_eq!(
            ring.queue_received(true, &fpt, fill(&poll(0x12, 0x3344))),
            Some(true)
        );

        let mut out = PhyRxFrame::EMPTY;
        let mut taken = [(0u8, false); 3];
        for entry in &mut taken {
            assert!(ring.pop_into(&mut out));
            *entry = (out.as_slice()[2], out.acked_without_frame_pending());
        }
        assert_eq!(taken, [(0x10, true), (0x11, false), (0x12, false)]);
    }

    #[test]
    fn only_auto_acknowledged_short_polls_are_marked() {
        let mut ring = RxRing::<4>::new();
        let mut fpt = FramePendingTable::<4>::new();
        fpt.update(0x12D2, false);
        let mut out = PhyRxFrame::EMPTY;
        // Promiscuous receive (scan): the M154 acknowledged nothing.
        ring.queue_received(false, &fpt, fill(&poll(1, 0x12D2)));
        assert!(ring.pop_into(&mut out) && !out.acked_without_frame_pending());
        // A data frame from that child is not a poll.
        let mut data = poll(2, 0x12D2);
        data[0] = 0x61;
        ring.queue_received(true, &fpt, fill(&data));
        assert!(ring.pop_into(&mut out) && !out.acked_without_frame_pending());
        // Extended-source poll: never stored, the miss default FP = 1.
        let mut extended = [0u8; 16];
        extended[..7].copy_from_slice(&[0x63, 0xC8, 3, 0x62, 0x1A, 0x00, 0x00]);
        extended[7..15].copy_from_slice(&[7; 8]);
        extended[15] = 0x04;
        ring.queue_received(true, &fpt, fill(&extended));
        assert!(ring.pop_into(&mut out) && !out.acked_without_frame_pending());
    }

    /// A frame the ring cannot hold is lost with its decision; a slot that
    /// held a marked poll does not pass the mark on.
    #[test]
    fn dropped_and_reused_slots_never_misattribute_a_decision() {
        let mut ring = RxRing::<2>::new();
        let mut fpt = FramePendingTable::<4>::new();
        fpt.update(0x0001, false);
        fpt.update(0x0002, true);
        assert_eq!(
            ring.queue_received(true, &fpt, fill(&poll(1, 0x0001))),
            Some(true)
        );
        assert_eq!(
            ring.queue_received(true, &fpt, fill(&poll(2, 0x0002))),
            Some(true)
        );
        // Full: the third poll (FP = 0) is dropped, nothing is recorded.
        assert_eq!(
            ring.queue_received(true, &fpt, fill(&poll(3, 0x0001))),
            None
        );
        let mut out = PhyRxFrame::EMPTY;
        assert!(ring.pop_into(&mut out));
        assert_eq!(
            (out.as_slice()[2], out.acked_without_frame_pending()),
            (1, true)
        );
        // A failed copy keeps the slot free and unmarked.
        assert_eq!(
            ring.queue_received(true, &fpt, |_| Err(PhyError::Hardware)),
            Some(false)
        );
        // The first slot (marked poll 1) is reused by the armed child.
        assert_eq!(
            ring.queue_received(true, &fpt, fill(&poll(4, 0x0002))),
            Some(true)
        );
        assert!(ring.pop_into(&mut out));
        assert_eq!(
            (out.as_slice()[2], out.acked_without_frame_pending()),
            (2, false)
        );
        assert!(ring.pop_into(&mut out));
        assert_eq!(
            (out.as_slice()[2], out.acked_without_frame_pending()),
            (4, false)
        );
        assert!(!ring.pop_into(&mut out));
    }

    #[test]
    fn filter_words_follow_the_m154_layout() {
        let filter = PhyAddressFilter {
            pan_id: PanId(0x1A62),
            short_address: ShortAddress(0x0001),
            extended_address: [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88],
        };
        assert_eq!(
            filter_words(&filter),
            (0x1A62_0001, 0x4433_2211, 0x8877_6655)
        );
    }

    #[test]
    fn fpt_register_addressing() {
        assert_eq!(fpt_bit(M154_FPT_VALID, 0), (0x4C00_0600, 1));
        assert_eq!(fpt_bit(M154_FPT_PENDING, 31), (0x4C00_0620, 1 << 31));
        assert_eq!(fpt_bit(M154_FPT_LONG, 32), (0x4C00_0614, 1));
        assert_eq!(fpt_slot(0), (0x4C00_0630, 0));
        assert_eq!(fpt_slot(1), (0x4C00_0630, 16));
        assert_eq!(fpt_slot(127), (0x4C00_072C, 16));
    }

    #[test]
    fn rx_completion_bits_follow_the_vendor_isr() {
        // XT-ZB1: a received beacon raises only 0x0010_0000.
        assert_ne!(IRQ_RX_COMPLETE & 0x0010_0000, 0);
        assert_eq!(IRQ_RX_COMPLETE & IRQ_RX_AUX, IRQ_RX_AUX);
        // The CRC status is a verdict, never a completion on its own.
        assert_eq!(IRQ_RX_COMPLETE & IRQ_RX_CRC, 0);
        assert_eq!(IRQ_RX_COMPLETE | IRQ_RX_CRC, IRQ_RX_MASK);
    }
}
