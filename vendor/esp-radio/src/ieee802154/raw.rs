use alloc::collections::VecDeque as Queue;

use esp_hal::{
    clock::{ModemClockController, PhyClockGuard, init_radio_clocks},
    handler,
    interrupt::Priority,
    peripherals::IEEE802154,
};
use esp_phy::{PhyController, PhyInitGuard};
use esp_sync::NonReentrantMutex;
use esp_wifi_sys::include::{
    ieee802154_coex_event_t, ieee802154_coex_event_t_IEEE802154_IDLE,
    ieee802154_coex_event_t_IEEE802154_LOW, ieee802154_coex_event_t_IEEE802154_MIDDLE,
};

use super::{
    frame::{
        FRAME_SIZE, FRAME_VERSION_1, FRAME_VERSION_2, frame_get_version, frame_is_ack_required,
    },
    hal::*,
    pib::*,
};

const PHY_ENABLE_VERSION_PRINT: u8 = 1;

static mut RX_BUFFER: [u8; FRAME_SIZE] = [0u8; FRAME_SIZE];

struct IeeeState {
    state: Ieee802154State,
    rx_queue: Queue<RawReceived>,
    rx_queue_size: usize,
}

static STATE: NonReentrantMutex<IeeeState> = NonReentrantMutex::new(IeeeState {
    state: Ieee802154State::Idle,
    rx_queue: Queue::new(),
    rx_queue_size: 10,
});

/// Decides the Frame Pending bit of the hardware auto-ACK for one received
/// frame (local patch, see `LOCAL-PATCHES.md`). Runs in the RX_DONE
/// interrupt between the end of the frame and the ACK the hardware sends
/// `aTurnaroundTime` later, so it must be short and must not block.
#[cfg(feature = "ieee802154-ack-pending")]
pub type AckPendingFn = fn(&[u8]) -> bool;

#[cfg(feature = "ieee802154-ack-pending")]
static ACK_PENDING_FN: NonReentrantMutex<Option<AckPendingFn>> = NonReentrantMutex::new(None);

#[cfg(feature = "ieee802154-ack-pending")]
pub(crate) fn set_ack_pending_fn(hook: Option<AckPendingFn>) {
    // No register access here (the driver may already be shut down): the
    // RX_DONE handler rewrites the bit before every auto-ACK, `false` once
    // the hook is gone.
    ACK_PENDING_FN.with(|slot| *slot = hook);
}

/// ESP-IDF `ieee802154_ack_config_pending_bit()` equivalent for
/// `PendingMode::Enable`: before the hardware ACK leaves, program its Frame
/// Pending bit from the hook. Every auto-acknowledged frame rewrites the
/// bit, so a stale `true` never leaks into a later ACK.
#[cfg(feature = "ieee802154-ack-pending")]
fn config_ack_pending_bit() {
    // SAFETY: RX_BUFFER is only written by the RX DMA, which is between
    // frames while RX_DONE is handled; this is the same read the queue push
    // below performs.
    let frame = unsafe {
        let buffer = &*core::ptr::addr_of!(RX_BUFFER);
        let len = usize::from(buffer[0]).min(FRAME_SIZE - 1);
        &buffer[1..][..len]
    };
    if !will_auto_send_ack(frame) {
        return;
    }
    let pending = ACK_PENDING_FN.with(|slot| slot.is_some_and(|hook| hook(frame)));
    set_ack_frame_pending(pending);
}

unsafe extern "C" {
    fn bt_bb_v2_init_cmplx(print_version: u8); // from libbtbb.a

    fn bt_bb_set_zb_tx_on_delay(time: u16); // from libbtbb.a

    fn esp_coex_ieee802154_ack_pti_set(event: ieee802154_coex_event_t); // from ???

    fn esp_coex_ieee802154_txrx_pti_set(event: ieee802154_coex_event_t); // from ???
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Ieee802154State {
    Idle,
    Receive,
    Transmit,
    TxAck,
    Sleep,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy, PartialEq)]
enum Ieee802154TxRxScene {
    Idle,
    Tx,
    Rx,
    TxAt,
    RxAt,
}

/// A raw payload received on some channel
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawReceived {
    /// Payload
    pub data: [u8; FRAME_SIZE],
    /// Receiver channel
    pub channel: u8,
    /// `esp_hal::time::Instant` microseconds when RX_DONE was handled
    /// (`ieee802154-rx-timestamp`): the end of the frame plus the interrupt
    /// latency.
    #[cfg(feature = "ieee802154-rx-timestamp")]
    pub timestamp_us: u64,
}

#[cfg(feature = "ieee802154-rx-timestamp")]
#[inline(always)]
fn rx_timestamp_us() -> u64 {
    esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_micros()
}

/// Decides, in the RX_DONE interrupt, whether a received frame (FCS bytes
/// replaced by RSSI/LQI) triggers the armed transmission (local patch
/// `ieee802154-triggered-tx`). Must be short and must not block.
#[cfg(feature = "ieee802154-triggered-tx")]
pub type TriggerFn = fn(&[u8]) -> bool;

/// What became of an armed transmission (`ieee802154-triggered-tx`).
#[cfg(feature = "ieee802154-triggered-tx")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggeredOutcome {
    /// Started `fired_after_us` after the triggering frame's RX_DONE.
    Sent { fired_after_us: u32 },
    /// The radio was transmitting or sending an ACK until the instant had
    /// passed by [`TRIGGERED_LATE_US`]; nothing was sent.
    Busy { fired_after_us: u32 },
    /// The trigger was handled `handled_after_us` after its RX_DONE, past
    /// the instant (interrupt latency); nothing was sent.
    Late { handled_after_us: u32 },
}

/// How late an armed transmission may still start (`ieee802154-triggered-tx`).
#[cfg(feature = "ieee802154-triggered-tx")]
pub const TRIGGERED_LATE_US: u32 = 300;

#[cfg(feature = "ieee802154-triggered-tx")]
#[derive(Clone, Copy, PartialEq, Eq)]
enum TriggeredPhase {
    Idle,
    /// Waiting for the trigger frame.
    Armed,
    /// Timer0 runs towards the instant (`esp_hal` microseconds).
    Waiting { rx_done_us: u64, target_us: u64 },
}

#[cfg(feature = "ieee802154-triggered-tx")]
struct Triggered {
    phase: TriggeredPhase,
    trigger: Option<TriggerFn>,
    /// From the trigger's RX_DONE time stamp to the TX command, µs.
    delay_us: u32,
    outcome: Option<TriggeredOutcome>,
    /// The triggered frame is on air: its TX_DONE is not reported to the
    /// thread, which may be waiting for a TX_DONE of its own.
    on_air: bool,
}

#[cfg(feature = "ieee802154-triggered-tx")]
static TRIGGERED: NonReentrantMutex<Triggered> = NonReentrantMutex::new(Triggered {
    phase: TriggeredPhase::Idle,
    trigger: None,
    delay_us: 0,
    outcome: None,
    on_air: false,
});

/// Length-prefixed frame of the armed transmission; the TX DMA reads it.
#[cfg(feature = "ieee802154-triggered-tx")]
static mut TRIGGERED_BUFFER: [u8; FRAME_SIZE] = [0u8; FRAME_SIZE];

/// Arm a transmission of `frame` (length-prefixed PSDU, FCS included in the
/// length) that the RX_DONE interrupt starts `delay_us` after the end of
/// the first received frame `trigger` accepts. Replaces an armed frame;
/// refused with `Busy` while a triggered transmission is already counting
/// down.
#[cfg(feature = "ieee802154-triggered-tx")]
pub(crate) fn arm_triggered(frame: &[u8], trigger: TriggerFn, delay_us: u32) -> Result<(), super::Error> {
    if frame.len() > FRAME_SIZE {
        return Err(super::Error::Incomplete);
    }
    TRIGGERED.with(|t| {
        if matches!(t.phase, TriggeredPhase::Waiting { .. }) {
            return Err(super::Error::Busy);
        }
        // SAFETY: no triggered TX is counting down or on air (Idle/Armed),
        // so the DMA does not read the buffer; interrupts are masked while
        // `TRIGGERED` is held.
        unsafe {
            let buffer = &mut *core::ptr::addr_of_mut!(TRIGGERED_BUFFER);
            buffer[..frame.len()].copy_from_slice(frame);
        }
        t.trigger = Some(trigger);
        t.delay_us = delay_us;
        t.phase = TriggeredPhase::Armed;
        Ok(())
    })
}

/// Disarm; a countdown in progress is cancelled.
#[cfg(feature = "ieee802154-triggered-tx")]
pub(crate) fn disarm_triggered() {
    TRIGGERED.with(|t| {
        if matches!(t.phase, TriggeredPhase::Waiting { .. }) {
            set_cmd(Command::Timer0Stop);
            disable_events(Event::Timer0Overflow as u16);
            clear_events(Event::Timer0Overflow as u16);
        }
        t.phase = TriggeredPhase::Idle;
    });
}

/// Take the outcome of the last triggered transmission.
#[cfg(feature = "ieee802154-triggered-tx")]
pub(crate) fn take_triggered_outcome() -> Option<TriggeredOutcome> {
    TRIGGERED.with(|t| t.outcome.take())
}

/// A triggered transmission is counting down: thread-level transmissions
/// wait, so the radio is free at the instant.
#[cfg(feature = "ieee802154-triggered-tx")]
#[inline(always)]
fn triggered_waiting() -> bool {
    TRIGGERED.with(|t| matches!(t.phase, TriggeredPhase::Waiting { .. }))
}

#[cfg(not(feature = "ieee802154-triggered-tx"))]
#[inline(always)]
fn triggered_waiting() -> bool {
    false
}

/// RX_DONE: start the countdown when the armed trigger accepts `frame`.
#[cfg(feature = "ieee802154-triggered-tx")]
fn triggered_on_rx(frame: &[u8], rx_done_us: u64) {
    TRIGGERED.with(|t| {
        if t.phase != TriggeredPhase::Armed || !t.trigger.is_some_and(|hook| hook(frame)) {
            return;
        }
        let target_us = rx_done_us + u64::from(t.delay_us);
        let now = rx_timestamp_us();
        if now >= target_us {
            t.outcome = Some(TriggeredOutcome::Late {
                handled_after_us: (now - rx_done_us) as u32,
            });
            t.phase = TriggeredPhase::Idle;
            return;
        }
        IEEE802154::regs()
            .time(0)
            .threshold()
            .write(|w| unsafe { w.timer0_threshold().bits((target_us - now) as u32) });
        clear_events(Event::Timer0Overflow as u16);
        enable_events(Event::Timer0Overflow as u16);
        set_cmd(Command::Timer0Start);
        t.phase = TriggeredPhase::Waiting {
            rx_done_us,
            target_us,
        };
    });
}

/// Timer0 overflow: start the armed frame now, abandoning a reception (GP
/// TempMaster: no CSMA/CA); an own TX or ACK is waited for in short steps
/// until [`TRIGGERED_LATE_US`].
#[cfg(feature = "ieee802154-triggered-tx")]
fn triggered_fire() {
    set_cmd(Command::Timer0Stop);
    disable_events(Event::Timer0Overflow as u16);
    let mut notify = None;
    TRIGGERED.with(|t| {
        let TriggeredPhase::Waiting {
            rx_done_us,
            target_us,
        } = t.phase
        else {
            return;
        };
        let now = rx_timestamp_us();
        let fired_after_us = now.saturating_sub(rx_done_us) as u32;
        STATE.with(|state| match state.state {
            Ieee802154State::Transmit | Ieee802154State::TxAck | Ieee802154State::Sleep => {
                if now.saturating_sub(target_us) < u64::from(TRIGGERED_LATE_US) {
                    // Try again shortly.
                    IEEE802154::regs()
                        .time(0)
                        .threshold()
                        .write(|w| unsafe { w.timer0_threshold().bits(25) });
                    enable_events(Event::Timer0Overflow as u16);
                    set_cmd(Command::Timer0Start);
                    return;
                }
                t.outcome = Some(TriggeredOutcome::Busy { fired_after_us });
                t.phase = TriggeredPhase::Idle;
            }
            _ => {
                use super::suspend::Boundary;
                let mut boundary = SleepBoundary(state);
                boundary.stop();
                // A reception that completed during STOP is preserved.
                if boundary.finish_stop() {
                    notify = Some(Ieee802154State::Receive);
                }
                tx_init(core::ptr::addr_of!(TRIGGERED_BUFFER).cast());
                #[cfg(not(feature = "ieee802154-extcoex"))] ieee802154_set_txrx_pti(Ieee802154TxRxScene::Tx); #[cfg(feature = "ieee802154-extcoex")] ieee802154_set_txrx_pti(Ieee802154TxRxScene::TxAt);
                set_cmd(Command::TxStart);
                state.state = Ieee802154State::Transmit;
                t.outcome = Some(TriggeredOutcome::Sent { fired_after_us });
                t.on_air = true;
                t.phase = TriggeredPhase::Idle;
            }
        });
    });
    if let Some(state) = notify {
        notify_state(state);
    }
}

/// TX_DONE of the triggered frame (consumed: the thread is not notified).
#[cfg(feature = "ieee802154-triggered-tx")]
fn triggered_tx_done() -> bool {
    TRIGGERED.with(|t| core::mem::replace(&mut t.on_air, false))
}

pub(crate) fn esp_ieee802154_enable<'d>(
    radio: &mut IEEE802154<'d>,
) -> (PhyClockGuard<'d>, PhyInitGuard<'d>) {
    init_radio_clocks();
    let phy_clock_guard = radio.enable_phy_clock();
    radio.enable_modem_clock(true);

    let phy_init_guard = radio.enable_phy();

    esp_btbb_enable();
    ieee802154_mac_init();
    STATE.with(|state| {
        state.state = Ieee802154State::Idle;
        state.rx_queue.clear();
    });

    info!("date={:x}", mac_date());
    (phy_clock_guard, phy_init_guard)
}

fn esp_btbb_enable() {
    unsafe { bt_bb_v2_init_cmplx(PHY_ENABLE_VERSION_PRINT) };
}

fn ieee802154_mac_init() {
    #[cfg(feature = "esp32c6")]
    unsafe {
        unsafe extern "C" {
            static mut coex_pti_tab_ptr: u32;
            static coex_pti_tab: u8;
        }

        // Manually set `coex_pti_tab_ptr` pointing to `coex_pti_tab`
        core::ptr::addr_of_mut!(coex_pti_tab_ptr).write_volatile(&coex_pti_tab as *const _ as u32);
    }

    ieee802154_pib_init();

    enable_events(Event::mask());
    disable_events(Event::Timer0Overflow | Event::Timer1Overflow);

    enable_tx_abort_events(
        TxAbortReason::RxAckTimeout
            | TxAbortReason::TxCoexBreak
            | TxAbortReason::TxSecurityError
            | TxAbortReason::CcaFailed
            | TxAbortReason::CcaBusy
            | TxAbortReason::TxStop,
    );
    enable_rx_abort_events(
        RxAbortReason::TxAckTimeout
            | RxAbortReason::TxAckCoexBreak
            | RxAbortReason::RxStop
            // Local patch `ieee802154-tx-ack-abort`: every way an automatic
            // ACK can end without AckTxDone must reach the handler.
            | RxAbortReason::TxAckStop,
    );

    set_ed_sample_mode(EdSampleMode::Avg);

    #[cfg(not(feature = "ieee802154-extcoex"))] unsafe { esp_coex_ieee802154_ack_pti_set(ieee802154_coex_event_t_IEEE802154_MIDDLE) }; #[cfg(feature = "ieee802154-extcoex")] extcoex::apply_ack_pti();
    ieee802154_set_txrx_pti(Ieee802154TxRxScene::Idle);

    unsafe {
        bt_bb_set_zb_tx_on_delay(50); // set tx on delay for libbtbb.a
    }
    set_rx_on_delay(50);

    // memset(s_rx_frame, 0, sizeof(s_rx_frame));
    // s_ieee802154_state = IEEE802154_STATE_IDLE;

    unsafe {
        esp_hal::interrupt::bind_interrupt(
            esp_hal::peripherals::Interrupt::ZB_MAC,
            zb_mac_handler.handler(),
        );
    }
    esp_hal::interrupt::enable(
        esp_hal::peripherals::Interrupt::ZB_MAC,
        zb_mac_handler.priority(),
    )
    .unwrap();
}

#[cfg(not(feature = "ieee802154-extcoex"))] fn ieee802154_set_txrx_pti(txrx_scene: Ieee802154TxRxScene) {
    match txrx_scene {
        Ieee802154TxRxScene::Idle => {
            unsafe { esp_coex_ieee802154_txrx_pti_set(ieee802154_coex_event_t_IEEE802154_IDLE) };
        }
        Ieee802154TxRxScene::Tx | Ieee802154TxRxScene::Rx => {
            unsafe { esp_coex_ieee802154_txrx_pti_set(ieee802154_coex_event_t_IEEE802154_LOW) };
        }
        Ieee802154TxRxScene::TxAt | Ieee802154TxRxScene::RxAt => {
            unsafe { esp_coex_ieee802154_txrx_pti_set(ieee802154_coex_event_t_IEEE802154_MIDDLE) };
        }
    }
}

pub fn tx_init(frame: *const u8) {
    let tx_frame = frame;
    stop_current_operation();
    ieee802154_pib_update();
    ieee802154_sec_update();

    set_tx_addr(tx_frame);

    if true
    // ieee802154_frame_is_ack_required(frame)
    {
        // set rx pointer for ack frame
        set_next_rx_buffer(); #[cfg(feature = "ieee802154-extcoex")] extcoex::tx_stage();
    }
}

pub(crate) fn set_queue_size(rx_queue_size: usize) {
    STATE.with(|state| {
        state.rx_queue_size = rx_queue_size;
    });
}

pub fn ieee802154_transmit(frame: *const u8, cca: bool) -> Result<(), super::Error> {
    transmit_discarding(frame, cca, false)
}

/// [`ieee802154_transmit`] that first drops every queued received frame when
/// `discard_queued` is set (local patch, see LOCAL-PATCHES.md).
#[inline(always)]
pub(crate) fn transmit_discarding(
    frame: *const u8,
    cca: bool,
    discard_queued: bool,
) -> Result<(), super::Error> {
    transmit_inner(frame, cca, discard_queued, false)
}

/// Transmit at once, abandoning a frame being received (local patch,
/// `ieee802154-rx-timestamp`): a Green Power TempMaster starts its frame at
/// an instant, without CSMA/CA (GP 1.1 A.1.5.2.2), as ESP-IDF's
/// `ieee802154_transmit` does (`stop_current_operation`). An application
/// TX, a hardware ACK and unprocessed interrupt work still refuse with
/// `Busy`; a reception that completes during the STOP is preserved (and
/// refuses with `Busy`) as in [`transmit_discarding`].
#[cfg(feature = "ieee802154-rx-timestamp")]
#[inline(always)]
pub(crate) fn transmit_preempting_rx(frame: *const u8) -> Result<(), super::Error> {
    transmit_inner(frame, false, false, true)
}

#[inline(always)]
fn transmit_inner(
    frame: *const u8,
    cca: bool,
    discard_queued: bool,
    preempt_rx: bool,
) -> Result<(), super::Error> {
    use super::suspend::Boundary;
    STATE.with(|state| {
        #[cfg(feature = "ieee802154-tx-ack-watchdog")] tx_ack_watchdog::recover(state); match state.state {
            Ieee802154State::Sleep => return Err(super::Error::Sleeping),
            Ieee802154State::Transmit | Ieee802154State::TxAck => return Err(super::Error::Busy),
            _ if events() != 0
                || (!preempt_rx && receive_in_progress())
                || rx_queue_blocks_tx(state)
                || triggered_waiting() =>
            {
                return Err(super::Error::Busy);
            }
            _ => {}
        }
        if discard_queued {
            state.rx_queue.clear();
        }
        let mut boundary = SleepBoundary(state);
        boundary.stop();
        if boundary.finish_stop() {
            boundary.resume_receive();
            return Err(super::Error::Busy);
        }
        tx_init(frame);

        #[cfg(not(feature = "ieee802154-extcoex"))] ieee802154_set_txrx_pti(Ieee802154TxRxScene::Tx); #[cfg(feature = "ieee802154-extcoex")] ieee802154_set_txrx_pti(if preempt_rx { Ieee802154TxRxScene::TxAt } else { Ieee802154TxRxScene::Tx });

        if cca {
            // disable_events(IEEE802154_EVENT_ED_DONE);
            // set_cmd(IEEE802154_CMD_CCA_TX_START);
            // ieee802154_state = IEEE802154_STATE_TX_CCA;
        } else {
            set_cmd(Command::TxStart);
            // if (ieee802154_frame_get_type(frame) == IEEE802154_FRAME_TYPE_ACK
            //     && ieee802154_frame_get_version(frame) == IEEE802154_FRAME_VERSION_2)
            // {
            //     ieee802154_state = IEEE802154_STATE_TX_ENH_ACK;
            // } else {
            state.state = Ieee802154State::Transmit;
            // }
        }
        Ok(())
    })
}

pub fn ieee802154_receive() -> i32 {
    STATE.with(|state| {
        if matches!(
            state.state,
            Ieee802154State::Receive
                | Ieee802154State::TxAck
                | Ieee802154State::Transmit
                | Ieee802154State::Sleep
        ) {
            return;
        }
        rx_init();
        enable_rx();

        state.state = Ieee802154State::Receive;
    });

    0 // ESP-OK
}

/// Write the software PIB to the radio registers now (local patch, see
/// LOCAL-PATCHES.md, "Apply a changed configuration at once").
///
/// `ieee802154_receive()` returns early while the driver is already in
/// `Receive`, so after `set_config()` the address filter, PAN ID and
/// channel registers would only be written by the next `tx_init()`. ESP-IDF
/// v5.5.1 instead marks the PIB pending and re-runs `rx_init()` from
/// `ieee802154_receive()`. This is that re-initialisation with the TX/ACK
/// veto of the transmit path: an application TX, an automatic ACK, an
/// inbound frame or unprocessed interrupt work refuses with `Busy` and
/// leaves the hardware untouched. In `Receive` the radio is stopped first;
/// a frame that completed during the stop is preserved (`finish_stop`)
/// and the receiver is restarted with the new registers. `Sleep` is left
/// to `resume()`, which writes the PIB itself.
pub(crate) fn apply_pib() -> Result<(), super::Error> {
    use super::suspend::Boundary as _;

    STATE.with(|state| {
        match state.state {
            Ieee802154State::Sleep => return Ok(()),
            Ieee802154State::Transmit | Ieee802154State::TxAck => return Err(super::Error::Busy),
            _ if events() != 0 || receive_in_progress() => return Err(super::Error::Busy),
            _ => {}
        }
        if state.state == Ieee802154State::Receive {
            let mut boundary = SleepBoundary(state);
            boundary.stop();
            boundary.finish_stop();
            ieee802154_pib_update();
            boundary.resume_receive();
        } else {
            ieee802154_pib_update();
        }
        Ok(())
    })
}

fn receive_in_progress() -> bool {
    // ESP-IDF 5.5.1 ieee802154_ll_is_current_rx_frame.
    IEEE802154::regs().rx_status().read().rx_state().bits() > 1
}

/// Whether a full receive queue refuses transmission. Upstream esp-radio
/// refuses; ESP-IDF does not (a full RX path only drops further frames).
/// A router that is relaying a broadcast storm must still transmit, so the
/// local `ieee802154-tx-when-rx-full` patch removes the check.
#[inline(always)]
fn rx_queue_blocks_tx(state: &IeeeState) -> bool {
    !cfg!(feature = "ieee802154-tx-when-rx-full") && state.rx_queue.len() >= state.rx_queue_size
}

pub(crate) fn can_transmit() -> Result<(), super::Error> {
    STATE.with(|state| { #[cfg(feature = "ieee802154-tx-ack-watchdog")] tx_ack_watchdog::recover(state); match state.state {
        Ieee802154State::Sleep => Err(super::Error::Sleeping),
        Ieee802154State::Transmit | Ieee802154State::TxAck => Err(super::Error::Busy),
        _ if events() != 0 || receive_in_progress() => Err(super::Error::Busy),
        #[cfg(feature = "ieee802154-triggered-tx")]
        _ if triggered_waiting() => Err(super::Error::Busy),
        _ => Ok(()),
    }})
}

struct SleepBoundary<'a>(&'a mut IeeeState);

impl super::suspend::Boundary for SleepBoundary<'_> {
    fn busy(&self) -> bool {
        matches!(
            self.0.state,
            Ieee802154State::Transmit | Ieee802154State::TxAck
        ) || receive_in_progress()
    }

    fn queued(&self) -> bool {
        !self.0.rx_queue.is_empty()
    }

    fn events_pending(&self) -> bool {
        events() != 0
    }

    fn stop(&mut self) {
        // ESP-IDF stop_rx: STOP first, then inspect completed RX.
        set_cmd(Command::Stop);
    }

    fn finish_stop(&mut self) -> bool {
        let stopped_events = events();
        let received = stopped_events & Event::RxDone != 0;
        if received {
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
            self.0.rx_queue.push_back(RawReceived {
                data: unsafe { RX_BUFFER },
                channel: freq_to_channel(freq()),
                #[cfg(feature = "ieee802154-rx-timestamp")]
                timestamp_us: rx_timestamp_us(),
            });
            warn!("Radio quiesce veto: RX completed during STOP; automatic ACK may need retry");
        }
        clear_events(stopped_events);
        abort_rx();
        received
    }

    fn resume_receive(&mut self) {
        self.0.state = Ieee802154State::Receive;
        enable_rx();
    }

    fn suspend(&mut self) {
        disable_events(Event::mask());
        ieee802154_set_txrx_pti(Ieee802154TxRxScene::Idle);
        self.0.state = Ieee802154State::Sleep;
    }
}

pub(crate) fn try_suspend(discard_queued: bool) -> Result<(), super::SuspendError> {
    #[cfg(feature = "ieee802154-triggered-tx")]
    if triggered_waiting() {
        return Err(super::SuspendError::Busy);
    }
    STATE.with(|state| {
        if discard_queued {
            state.rx_queue.clear();
        }
        super::suspend::try_suspend(&mut SleepBoundary(state))
    })
}

pub(crate) fn resume() {
    STATE.with(|state| {
        if state.state == Ieee802154State::Sleep {
            ieee802154_pib_update();
            clear_events(events());
            enable_events(Event::mask());
            disable_events(Event::Timer0Overflow | Event::Timer1Overflow);
            state.state = Ieee802154State::Receive;
            enable_rx();
        }
    });
}

pub(crate) fn cancel_transmit() {
    STATE.with(|state| {
        if state.state == Ieee802154State::Transmit {
            set_cmd(Command::Stop);
            clear_events(Event::TxDone | Event::TxAbort | Event::TxSfdDone);
            abort_tx();
            state.state = Ieee802154State::Receive;
            enable_rx();
        }
    });
    // A cancelled frame produces no TX_DONE.
    #[cfg(feature = "ieee802154-triggered-tx")]
    TRIGGERED.with(|t| t.on_air = false);
}

pub(crate) fn shutdown() {
    STATE.with(|state| {
        set_cmd(Command::Stop);
        disable_events(Event::mask());
        clear_events(events());
        state.state = Ieee802154State::Sleep;
        state.rx_queue.clear();
    });
}

pub fn ieee802154_poll() -> Option<RawReceived> {
    STATE.with(|state| state.rx_queue.pop_front())
}

fn rx_init() {
    stop_current_operation();
    ieee802154_pib_update();
}

fn enable_rx() {
    #[cfg(feature = "ieee802154-extcoex")] extcoex::rx_stage(); set_next_rx_buffer();
    ieee802154_set_txrx_pti(Ieee802154TxRxScene::Rx);

    set_cmd(Command::RxStart);

    // ieee802154_state = IEEE802154_STATE_RX;
}

fn stop_current_operation() {
    let events = events();
    set_cmd(Command::Stop);
    clear_events(events);
}

fn set_next_rx_buffer() {
    set_rx_addr(core::ptr::addr_of_mut!(RX_BUFFER).cast());
}

pub fn set_promiscuous(enable: bool) {
    ieee802154_pib_set_promiscuous(enable);
}

pub fn set_auto_ack_tx(enable: bool) {
    ieee802154_pib_set_auto_ack_tx(enable);
}

pub fn set_auto_ack_rx(enable: bool) {
    ieee802154_pib_set_auto_ack_rx(enable);
}

pub fn set_enhance_ack_tx(enable: bool) {
    ieee802154_pib_set_enhance_ack_tx(enable);
}

pub fn set_coordinator(enable: bool) {
    ieee802154_pib_set_coordinator(enable);
}

pub fn set_rx_when_idle(enable: bool) {
    ieee802154_pib_set_rx_when_idle(enable);
}

pub fn set_tx_power(power: i8) {
    ieee802154_pib_set_tx_power(power);
}

pub fn set_channel(channel: u8) {
    ieee802154_pib_set_channel(channel);
}

#[allow(unused)]
pub fn set_pending_mode(mode: PendingMode) {
    ieee802154_pib_set_pending_mode(mode);
}

#[allow(unused)]
pub fn set_multipan_enable(mask: u8) {
    set_multipan_enable_mask(mask);
}

pub fn set_short_address(index: u8, address: u16) {
    ieee802154_pib_set_short_address(index, address);
}

pub fn set_extended_address(index: u8, address: [u8; IEEE802154_FRAME_EXT_ADDR_SIZE]) {
    ieee802154_pib_set_extended_address(index, address);
}

pub fn set_cca_theshold(cca_threshold: i8) {
    ieee802154_pib_set_cca_theshold(cca_threshold);
}

pub fn set_cca_mode(mode: CcaMode) {
    ieee802154_pib_set_cca_mode(mode);
}

pub fn set_panid(index: u8, id: u16) {
    ieee802154_pib_set_panid(index, id);
}

#[inline(always)]
fn ieee802154_sec_update() {
    let is_security = false;
    set_transmit_security(is_security);
    // ieee802154_sec_clr_transmit_security();
}

fn next_operation_inner(state: &mut IeeeState) -> Ieee802154State {
    let prev_state = state.state;
    state.state = if ieee802154_pib_get_rx_when_idle() {
        enable_rx();
        Ieee802154State::Receive
    } else {
        Ieee802154State::Idle
    };

    prev_state
}

fn notify_state(state: Ieee802154State) {
    match state {
        Ieee802154State::Receive => super::rx_available(),
        Ieee802154State::Transmit => super::tx_done(),
        Ieee802154State::TxAck => super::rx_available(),
        _ => (),
    }
}

fn next_operation() {
    let previous_operation = STATE.with(next_operation_inner);

    notify_state(previous_operation)
}

// FIXME: we shouldn't need this - we need to re-align the original driver with our port
pub(crate) fn ensure_receive_enabled() {
    // shouldn't be necessary but avoids a problem with rx stopping
    // unexpectedly when used together with BLE
    //
    // Local patch `ieee802154-tx-ack-abort`: never restart while an event is
    // pending or a frame is on the air. A pending RX_DONE means the hardware
    // is in its automatic-ACK turnaround; RxStart then aborts the ACK, and
    // an inbound frame would be cut off.
    STATE.with(|state| {
        #[cfg(feature = "ieee802154-tx-ack-watchdog")] tx_ack_watchdog::recover(state); if state.state == Ieee802154State::Receive && events() == 0 && !receive_in_progress() {
            set_cmd(Command::RxStart);
        }
    });
}

/// Local patch `ieee802154-tx-ack-abort`: an automatic ACK that ends in a
/// receive abort (timeout, coexistence break or stop) produces no AckTxDone.
/// Leave `TxAck` as ESP-IDF 5.5.1 `isr_handle_rx_abort` does; otherwise the
/// state stays `TxAck`, every transmission is refused as busy and the
/// receiver is never restarted.
fn tx_ack_aborted() {
    let reason = rx_abort_reason();
    if reason == RxAbortReason::TxAckTimeout as u8
        || reason == RxAbortReason::TxAckStop as u8
        || reason == RxAbortReason::TxAckCoexBreak as u8
    {
        let previous = STATE.with(|state| {
            if state.state == Ieee802154State::TxAck {
                next_operation_inner(state)
            } else {
                Ieee802154State::Idle
            }
        });
        notify_state(previous);
    }
}

#[handler(priority = Priority::Priority1)]
fn zb_mac_handler() {
    trace!("ZB_MAC interrupt");

    let events = events();
    clear_events(events);

    trace!("events = {:032b}", events);

    if events & Event::RxSfdDone != 0 {
        // IEEE802154_STATE_TX && IEEE802154_STATE_TX_CCA && IEEE802154_STATE_TX_ENH_ACK
        // for isr processing delay
        trace!("rx sfd done");
    }

    if events & Event::TxSfdDone != 0 {
        // IEEE802154_STATE_RX for isr processing delay, only 821
        // IEEE802154_STATE_TX_ACK for workaround jira ZB-81.
        trace!("tx sfd done");
    }

    if events & Event::TxDone != 0 {
        trace!("tx done");
        #[cfg(feature = "ieee802154-triggered-tx")]
        if triggered_tx_done() {
            STATE.with(next_operation_inner);
        } else {
            next_operation();
        }
        #[cfg(not(feature = "ieee802154-triggered-tx"))]
        next_operation();
    }

    // Before RxDone: an aborted ACK belongs to the previous frame.
    if events & Event::RxAbort != 0 {
        trace!("RxAbort");
        tx_ack_aborted();
        abort_rx();
    }

    #[cfg(feature = "ieee802154-triggered-tx")]
    if events & Event::Timer0Overflow != 0 {
        triggered_fire();
    }

    if events & Event::RxDone != 0 {
        trace!("rx done");
        #[cfg(feature = "ieee802154-rx-timestamp")]
        let timestamp_us = rx_timestamp_us();
        // First, before the queue push: the ACK leaves 192 µs after RX_DONE.
        #[cfg(feature = "ieee802154-ack-pending")]
        config_ack_pending_bit();
        unsafe {
            trace!(
                "Received raw {:?}",
                crate::fmt::Bytes(&*core::ptr::addr_of!(RX_BUFFER))
            );
            let mut state_for_notify = Ieee802154State::Idle;
            STATE.with(|state| {
                if state.rx_queue.len() <= state.rx_queue_size {
                    let item = RawReceived {
                        data: RX_BUFFER,
                        channel: freq_to_channel(freq()),
                        #[cfg(feature = "ieee802154-rx-timestamp")]
                        timestamp_us,
                    };
                    state.rx_queue.push_back(item);
                } else {
                    warn!("Receive queue full");
                }

                let frm = if RX_BUFFER[0] >= FRAME_SIZE as u8 {
                    warn!("RX_BUFFER[0] {} is larger than frame size", RX_BUFFER[0]);
                    &RX_BUFFER[1..][..FRAME_SIZE - 1]
                } else {
                    &RX_BUFFER[1..][..RX_BUFFER[0] as usize]
                };
                if will_auto_send_ack(frm) {
                    state.state = Ieee802154State::TxAck; #[cfg(feature = "ieee802154-extcoex")] extcoex::tx_stage(); #[cfg(feature = "ieee802154-tx-ack-watchdog")] tx_ack_watchdog::armed();
                } else if should_send_enhanced_ack(frm) {
                    // TODO
                } else {
                    state_for_notify = next_operation_inner(state)
                    // esp_ieee802154_coex_pti_set(IEEE802154_IDLE_RX);
                }
            });

            // After the frame is queued and the receiver restarted; a
            // triggering frame is broadcast, never auto-acknowledged.
            #[cfg(feature = "ieee802154-triggered-tx")]
            {
                let buffer = &*core::ptr::addr_of!(RX_BUFFER);
                let len = usize::from(buffer[0]).min(FRAME_SIZE - 1);
                triggered_on_rx(&buffer[1..][..len], timestamp_us);
            }

            notify_state(state_for_notify)
        }
    }

    if events & Event::AckRxDone != 0 {
        trace!("EventAckRxDone"); // ED_DONE is handled after this block (`ieee802154-energy-detect`).
    }

    if events & Event::AckTxDone != 0 {
        trace!("EventAckTxDone"); #[cfg(feature = "ieee802154-extcoex")] extcoex::rx_stage();
        next_operation();
    }

    if events & Event::TxAbort != 0 {
        trace!("TxAbort");
        abort_tx();
    } #[cfg(feature = "ieee802154-energy-detect")] energy_detect::on_interrupt(events);
}

fn freq_to_channel(freq: u8) -> u8 {
    (freq - 3) / 5 + 11
}

fn will_auto_send_ack(frame: &[u8]) -> bool {
    frame_is_ack_required(frame) && frame_get_version(frame) <= FRAME_VERSION_1 && tx_auto_ack()
}

fn should_send_enhanced_ack(frame: &[u8]) -> bool {
    frame_is_ack_required(frame) && frame_get_version(frame) <= FRAME_VERSION_2 && tx_enhance_ack()
}

/// Energy detection (local patch `ieee802154-energy-detect`, declared last).
///
/// Ordering reference: ESP-IDF **v5.5.1** `esp_ieee802154_dev.c`
/// (`ieee802154_energy_detect`: stop the current operation, update the PIB,
/// `start_ed`; `isr_handle_ed_done` reads `ieee802154_ll_get_ed_rss`, then
/// `next_operation`). The caller owns the driver exclusively while a
/// measurement runs, so no transmission can start in the meantime.
#[cfg(feature = "ieee802154-energy-detect")]
pub(crate) mod energy_detect {
    use core::sync::atomic::{AtomicI8, AtomicU8, Ordering};

    use super::*;

    const IDLE: u8 = 0;
    const RUNNING: u8 = 1;
    const DONE: u8 = 2;

    static ED_STATE: AtomicU8 = AtomicU8::new(IDLE);
    static ED_RSS: AtomicI8 = AtomicI8::new(i8::MIN);

    /// Start one measurement of `duration_symbols` × 16 µs on the current
    /// channel. Refused while a transmission, ACK or reception is active.
    pub(crate) fn start(duration_symbols: u32) -> Result<(), crate::ieee802154::Error> {
        STATE.with(|state| match state.state {
            Ieee802154State::Sleep => Err(crate::ieee802154::Error::Sleeping),
            Ieee802154State::Transmit | Ieee802154State::TxAck => {
                Err(crate::ieee802154::Error::Busy)
            }
            _ if events() != 0 || receive_in_progress() => Err(crate::ieee802154::Error::Busy),
            _ => {
                stop_current_operation();
                ieee802154_pib_update();
                ED_STATE.store(RUNNING, Ordering::Release);
                set_ed_duration(duration_symbols);
                enable_events(Event::EdDone as u16);
                set_cmd(Command::EdStart);
                Ok(())
            }
        })
    }

    /// ZB_MAC interrupt part: latch the result and restart the receiver.
    pub(crate) fn on_interrupt(events: u16) {
        if events & Event::EdDone != 0 && ED_STATE.load(Ordering::Acquire) == RUNNING {
            ED_RSS.store(ed_rss(), Ordering::Relaxed);
            ED_STATE.store(DONE, Ordering::Release);
            STATE.with(next_operation_inner);
        }
    }

    /// The finished measurement in dBm, once.
    pub(crate) fn take() -> Option<i8> {
        ED_STATE
            .compare_exchange(DONE, IDLE, Ordering::AcqRel, Ordering::Acquire)
            .ok()
            .map(|_| ED_RSS.load(Ordering::Relaxed))
    }

    /// Abandon a measurement that did not complete and resume reception.
    pub(crate) fn cancel() {
        if ED_STATE.swap(IDLE, Ordering::AcqRel) == RUNNING {
            STATE.with(|state| {
                stop_current_operation();
                next_operation_inner(state);
            });
        }
    }
}

/// Scene priority with configurable PTIs (local patch `ieee802154-extcoex`;
/// replaces the fixed-PTI function above, which is compiled out).
#[cfg(feature = "ieee802154-extcoex")]
fn ieee802154_set_txrx_pti(txrx_scene: Ieee802154TxRxScene) {
    let pti = extcoex::coex_pti();
    let priority = match txrx_scene {
        Ieee802154TxRxScene::Idle => pti.idle,
        Ieee802154TxRxScene::Tx => pti.tx,
        Ieee802154TxRxScene::Rx => pti.rx,
        Ieee802154TxRxScene::TxAt | Ieee802154TxRxScene::RxAt => pti.timed,
    };
    unsafe { esp_coex_ieee802154_txrx_pti_set(priority.event()) };
}

/// External (PTA) coexistence support of the 802.15.4 driver (local patch
/// `ieee802154-extcoex`, declared last so earlier line numbers are kept).
///
/// ESP-IDF **v5.5.1** reference: `components/ieee802154/driver/
/// esp_ieee802154_dev.c` (`extcoex_tx_stage_start`/`extcoex_rx_stage_start`
/// call sites, `CONFIG_ESP_COEX_EXTERNAL_COEXIST_ENABLE`) and
/// `esp_ieee802154_util.c` (`s_coex_config`, `ieee802154_set_txrx_pti`).
/// The functions are in the ESP32-H2/C6 `libcoexist.a` that `esp-wifi-sys`
/// already links (the H2 archive is byte-identical to ESP-IDF v5.5.1's).
/// Enabling the external arbiter itself (GPIO matrix, `esp_coex_external_*`)
/// is the application's job; this module only gives the radio the same
/// per-operation hooks and priorities as the ESP-IDF driver.
#[cfg(feature = "ieee802154-extcoex")]
pub(crate) mod extcoex {
    use core::sync::atomic::{AtomicU32, Ordering};

    use esp_wifi_sys::include::{
        ieee802154_coex_event_t, ieee802154_coex_event_t_IEEE802154_HIGH,
        ieee802154_coex_event_t_IEEE802154_IDLE, ieee802154_coex_event_t_IEEE802154_LOW,
        ieee802154_coex_event_t_IEEE802154_MIDDLE,
    };

    unsafe extern "C" {
        // libcoexist.a (coexist_api.o): PTI offset 0 / 0xF of the external
        // arbiter for the coming TX / RX stage.
        fn esp_coex_ieee802154_extcoex_tx_stage();
        fn esp_coex_ieee802154_extcoex_rx_stage();
    }

    /// Coexistence priority of a radio operation
    /// (`ieee802154_coex_event_t`). The blob maps it through `coex_pti_tab`
    /// to the PTI the 802.15.4 MAC presents to the arbiter.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum CoexPriority {
        High,
        Middle,
        Low,
        Idle,
    }

    impl CoexPriority {
        pub(crate) const fn event(self) -> ieee802154_coex_event_t {
            match self {
                Self::High => ieee802154_coex_event_t_IEEE802154_HIGH,
                Self::Middle => ieee802154_coex_event_t_IEEE802154_MIDDLE,
                Self::Low => ieee802154_coex_event_t_IEEE802154_LOW,
                Self::Idle => ieee802154_coex_event_t_IEEE802154_IDLE,
            }
        }

        const fn bits(self) -> u32 {
            match self {
                Self::High => 0,
                Self::Middle => 1,
                Self::Low => 2,
                Self::Idle => 3,
            }
        }

        const fn from_bits(bits: u32) -> Self {
            match bits & 3 {
                0 => Self::High,
                1 => Self::Middle,
                2 => Self::Low,
                _ => Self::Idle,
            }
        }
    }

    /// Priorities per radio scene. `timed` covers transmissions started at
    /// an instant (`transmit_raw_preempting_rx` and the triggered reply,
    /// ESP-IDF's `TX_AT` scene); `ack` is the hardware automatic ACK.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct CoexPti {
        pub idle: CoexPriority,
        pub rx: CoexPriority,
        pub tx: CoexPriority,
        pub timed: CoexPriority,
        pub ack: CoexPriority,
    }

    impl CoexPti {
        /// ESP-IDF v5.5.1 defaults (`s_coex_config` and the MAC-init ACK
        /// PTI); also what the driver uses without this feature, except
        /// that timed transmissions there use the `tx` priority.
        pub const DEFAULT: Self = Self {
            idle: CoexPriority::Idle,
            rx: CoexPriority::Low,
            tx: CoexPriority::Low,
            timed: CoexPriority::Middle,
            ack: CoexPriority::Middle,
        };

        const fn pack(self) -> u32 {
            self.idle.bits()
                | self.rx.bits() << 2
                | self.tx.bits() << 4
                | self.timed.bits() << 6
                | self.ack.bits() << 8
        }

        const fn unpack(bits: u32) -> Self {
            Self {
                idle: CoexPriority::from_bits(bits),
                rx: CoexPriority::from_bits(bits >> 2),
                tx: CoexPriority::from_bits(bits >> 4),
                timed: CoexPriority::from_bits(bits >> 6),
                ack: CoexPriority::from_bits(bits >> 8),
            }
        }
    }

    static PTI: AtomicU32 = AtomicU32::new(CoexPti::DEFAULT.pack());

    /// The configured priorities.
    pub fn coex_pti() -> CoexPti {
        CoexPti::unpack(PTI.load(Ordering::Relaxed))
    }

    /// Set the priorities. The ACK PTI is written at once; the scene PTIs
    /// take effect at the next scene change (every TX start and receiver
    /// restart). Must not be called from the ZB_MAC interrupt.
    pub fn set_coex_pti(pti: CoexPti) {
        PTI.store(pti.pack(), Ordering::Relaxed);
        // The ACK and TX/RX PTIs share one register; the interrupt rewrites
        // the TX/RX field, so the read-modify-write runs with it masked.
        super::STATE.with(|_| apply_ack_pti());
    }

    pub(crate) fn apply_ack_pti() {
        unsafe { super::esp_coex_ieee802154_ack_pti_set(coex_pti().ack.event()) };
    }

    #[inline(always)]
    pub(crate) fn tx_stage() {
        unsafe { esp_coex_ieee802154_extcoex_tx_stage() };
    }

    #[inline(always)]
    pub(crate) fn rx_stage() {
        unsafe { esp_coex_ieee802154_extcoex_rx_stage() };
    }
}

/// Local patch `ieee802154-debug-state` (declared last): the driver state a
/// stalled radio is diagnosed from: `(state, pending events, hardware
/// rx_state, queued frames)`; state 0 idle, 1 receive, 2 transmit, 3 TX
/// ACK, 4 sleep.
pub(crate) fn debug_state() -> (u8, u16, u8, u8) {
    let pending = events();
    let rx_state = IEEE802154::regs().rx_status().read().rx_state().bits() as u8;
    STATE.with(|state| {
        let code = match state.state {
            Ieee802154State::Idle => 0,
            Ieee802154State::Receive => 1,
            Ieee802154State::Transmit => 2,
            Ieee802154State::TxAck => 3,
            Ieee802154State::Sleep => 4,
        };
        (code, pending, rx_state, state.rx_queue.len() as u8)
    })
}

/// Local patch `ieee802154-tx-ack-watchdog` (declared last): RX_DONE of a
/// frame with the ACK-request bit enters `TxAck`, which only `AckTxDone` or
/// an RX abort leaves. When neither arrives (observed on an ESP32-H2
/// coordinator with a sleepy child: state `TxAck`, no event pending, the
/// receiver idle) every transmission is refused as busy and the receiver is
/// never restarted. A `TxAck` older than any automatic ACK (192 µs
/// turnaround + 352 µs frame), with nothing pending and nothing on the air,
/// is left like an aborted ACK.
#[cfg(feature = "ieee802154-tx-ack-watchdog")]
mod tx_ack_watchdog {
    use core::sync::atomic::{AtomicU32, Ordering};

    use super::{IeeeState, Ieee802154State};

    static SINCE_US: AtomicU32 = AtomicU32::new(0);
    static RECOVERED: AtomicU32 = AtomicU32::new(0);
    const LIMIT_US: u32 = 2_000;

    fn now_us() -> u32 {
        esp_hal::time::Instant::now()
            .duration_since_epoch()
            .as_micros() as u32
    }

    #[inline(always)]
    pub(super) fn armed() {
        SINCE_US.store(now_us(), Ordering::Relaxed);
    }

    pub(super) fn recover(state: &mut IeeeState) {
        if state.state == Ieee802154State::TxAck
            && super::events() == 0
            && !super::receive_in_progress()
            && now_us().wrapping_sub(SINCE_US.load(Ordering::Relaxed)) > LIMIT_US
        {
            RECOVERED.fetch_add(1, Ordering::Relaxed);
            super::next_operation_inner(state);
        }
    }

    /// Stalled `TxAck` states left since boot.
    pub(crate) fn recovered() -> u32 {
        RECOVERED.load(Ordering::Relaxed)
    }
}

/// `TxAck` stalls the watchdog left (local patch
/// `ieee802154-tx-ack-watchdog`).
#[cfg(feature = "ieee802154-tx-ack-watchdog")]
pub(crate) fn tx_ack_recoveries() -> u32 {
    tx_ack_watchdog::recovered()
}
