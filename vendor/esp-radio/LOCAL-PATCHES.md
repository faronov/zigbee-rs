# Local IEEE 802.15.4 retained-sleep patch

Baseline: published `esp-radio 0.17.0`, upstream `esp-rs/esp-hal`
commit `5ba50dd2d21136370050dd2f7a767761e01cfeef`, directory `esp-radio`.
The upstream MIT/Apache-2.0 licenses remain applicable.

Local changes are limited to IEEE 802.15.4:

- Retain the peripheral token and release/reacquire the PHY initialization
  guard around explicit `try_suspend`/`resume`.
- Veto TX, auto-ACK, queued receive and pending interrupt work. STOP and
  recheck RX_DONE before disabling interrupts/PHY. Preserve a receive racing
  STOP and veto sleep; a raced hardware ACK may require the sender's retry.
- Retain MAC registers/PIB across sleep. This API requires retained CPU, RAM,
  top and modem register domains; it is not a deep-sleep/reconstruction API.
- Prevent TX-buffer reuse while TX/ACK is active, provide explicit TX
  cancellation, and stop DMA before dropping the driver.
- Do not report an automatic ACK transmission as completion of an application
  transmission.

Ordering reference: ESP-IDF **v5.5.1**,
`components/ieee802154/driver/esp_ieee802154_dev.c` (`stop_rx`,
`ieee802154_sleep`) and
`components/hal/include/hal/ieee802154_common_ll.h`
(`ieee802154_ll_set_cmd`). STOP is a register command in that reference, not
an undocumented software wait. No ESP-IDF Zigbee stack is linked.

The boundary algorithm in `src/ieee802154/suspend.rs` is source-linked into
product host tests. Radio timing, PHY wake and RF interoperability still need
hardware qualification; host tests are not a substitute for it.

## Frame Pending in the hardware auto-ACK (`ieee802154-ack-pending`)

Opt-in cargo feature, used only by parent (router) images; without it the
crate compiles to the same code as before.

- `Ieee802154::set_ack_pending_fn(fn(&[u8]) -> bool)` installs a hook that
  the RX_DONE interrupt calls first, before the receive-queue push, for every
  frame the hardware will auto-acknowledge (`will_auto_send_ack`). The hook's
  result is written to `IEEE802154.ACK_FRAME_PENDING_EN` every time, so no
  stale `true` survives into a later ACK. Dropping the driver or
  `clear_ack_pending_fn` removes the hook; later auto-ACKs carry `false`.
- With `autopend_enhance` clear (the default `PendingMode::Disable`), the
  hardware copies that bit only into the ACK of a MAC Data Request.

Ordering reference: ESP-IDF **v5.5.1**,
`components/ieee802154/driver/esp_ieee802154_dev.c` (`isr_handle_rx_done`
calls `ieee802154_ack_config_pending_bit` before the ACK) and
`esp_ieee802154_ack.c`. The hook must finish within the 192 µs turnaround;
that is a hardware-qualification item.

## Transmit while the receive queue is full (`ieee802154-tx-when-rx-full`)

Opt-in cargo feature. `zigbee-mac` enables it for every ESP32-C6/H2 image;
without it the crate compiles to the same code as before.

- Upstream `ieee802154_transmit` returns `Error::Busy` while
  `rx_queue.len() >= rx_queue_size`. Nothing else drains the queue, so the
  refusal can be permanent:
  - A router relaying a broadcast burst (RREQ, link status) waits in its
    jitter/backoff delays without draining the queue; every CSMA attempt saw
    `Busy`, and forwarding failed with a channel-access error (ESP32-C6,
    hwlab session `2026-11-01T101400Z-esp32c6-router-v1-run`).
  - An always-on end device whose chosen parent never acknowledged its
    Transport-Key polls filled the queue while polling; every later Beacon
    Request and Association Request was refused, and the device never
    joined again (live state: queue length 21, size 20, state `Receive`;
    session `2026-11-01T105654Z-esp32c6-ed-v2-join`).
- With the feature, `rx_queue_blocks_tx` is `false` and only a real TX/ACK,
  a pending event or an in-progress receive defers the transmission. The
  next receive drains the queue.
- Receive-queue overflow behaviour is unchanged: the RX_DONE interrupt still
  drops a frame when the queue is full ("Receive queue full").

Reference: ESP-IDF **v5.5.1**, `esp_ieee802154_dev.c`
(`ieee802154_transmit`) has no receive-buffer-full refusal; frames are
handed to the upper layer through `esp_ieee802154_receive_done`.

## Drop received frames when an idle receiver would have been off

Additive API, no cargo feature: `Ieee802154::transmit_raw_discarding(frame,
discard_received)` and `Ieee802154::try_suspend_discarding_received()`.
`transmit_raw` and `try_suspend` call them with `false` and keep the
upstream behaviour.

- esp-radio keeps the receiver on between operations (`rx_when_idle`), so a
  sleepy end device hears every broadcast between two polls while its stack
  is not receiving. On a busy network (about 5 broadcasts/s in the lab) the
  20-frame queue is full before the next poll. The software ACK window
  after the poll then met the stale backlog instead of the ACK (or the full
  queue dropped the ACK). Every poll was reported unacknowledged, and the
  device left and rejoined in a loop. The queued frames also vetoed
  `try_suspend` indefinitely (ESP32-C6, hwlab session
  `2026-11-01T111317Z-esp32c6-sed-ls-v3-join`).
- With `true`, the queue is cleared inside the `STATE` critical section
  that the transmission or suspend already takes, after the busy checks. A
  refused transmission keeps the queue. Frames received after that point
  are kept.
- `zigbee-mac` passes `true` only while `macRxOnWhenIdle` is false (sleepy
  end device). Routers and always-on end devices keep every frame.

Reference: ESP-IDF **v5.5.1** `esp_ieee802154_dev.c` hands each frame to
the upper layer from the RX_DONE interrupt and has no driver queue; the
Zigbee stack there turns the receiver off while idle on a sleepy device.

## Apply a changed configuration at once; ask whether TX would be accepted

Additive API, no cargo feature: `Ieee802154::apply_config()` and
`Ieee802154::can_transmit()`. Nothing upstream calls them, so the driver
behaves as before unless the caller uses them.

- `set_config` only updates the software PIB. Upstream
  `ieee802154_receive()` returns early when the driver is already in
  `Receive`, so the address filter, PAN ID and channel registers were
  written only by the next `tx_init()`.
  - After MAC association the short-address filter therefore stayed at
    0xFFFF until the joiner's first transmission. A Transport-Key sent
    directly to the new short address in that gap was neither
    acknowledged nor received, so the key was lost and the join failed
    (ESP32-H2, hwlab sessions `2026-11-01T171249Z-esp32h2-ed-v3-join` and
    `2026-11-01T173811Z-esp32h2-router-v1-final-join`).
  - A sleepy device was not affected, because its first poll (a TX)
    applied the PIB before the parent released the key.
- `apply_config` writes the PIB to the registers now. It refuses with
  `Busy`, leaving the hardware unchanged, while a TX, an automatic ACK, an
  inbound frame or unprocessed interrupt work is in progress (the same
  veto as `transmit_raw`). In `Receive` it stops the radio, keeps a frame
  that completed during the stop (`finish_stop`), writes the PIB and
  restarts the receiver. In `Sleep` it does nothing; `resume()` writes the
  PIB.
- `can_transmit` reports the `Busy` conditions of `transmit_raw` without
  starting anything. `zigbee-mac` uses it to wait for the end of a receive
  or an ACK instead of reporting the refusal as a channel-access failure
  (see the next paragraph).

`zigbee-mac` (`src/esp/radio_phy.rs`) handles a `Busy` refusal from
`transmit_raw` in thread context: it spins until `can_transmit` is OK
(bounded by 6 ms: a maximum PPDU, its ACK and a guard), waits a further
aTurnaroundTime plus SHR so another node's immediate ACK is not stepped
on, then a random 0..2^macMinBE-1 unit backoff periods (as after a busy
CCA, without incrementing NB), and resubmits. The refusal does not consume a CSMA backoff or a MAC
retry. Only a refusal that outlives the bound is reported as
`ChannelAccessFailure`. The interrupt handler, and with it the automatic
ACK and its Frame Pending bit, is never delayed.

Reference: ESP-IDF **v5.5.1** `esp_ieee802154_dev.c`:
`ieee802154_receive()` checks `ieee802154_pib_is_pending()` and re-runs
`rx_init()`, which calls `ieee802154_pib_update()`.

## Leave `TxAck` when the automatic ACK is aborted (`ieee802154-tx-ack-abort`)

Unconditional fix, no cargo feature: the old behaviour can wedge the
radio permanently.

- **Observed:** a master router on ESP32-H2 (0xB67A, hwlab session
  `2026-11-02T150137Z-esp32h2-router-master-txbusy-baseline`) became deaf
  and mute while running.
  - Every transmission was refused as `Busy` (tx +20 / tx-err +20 per
    summary), and every link status failed.
  - Live reads through the debug probe (hart running) showed `STATE.state
    == TxAck` with an empty queue, `event_status == 0`, the receiver idle
    (`rx_state 0`), and every receive error counter frozen across five
    minutes.
- **Cause:** upstream enables the `TxAckTimeout`, `TxAckCoexBreak` and
  `RxStop` receive-abort interrupts, but its handler only clears the abort
  status. When an automatic ACK ends in an abort there is no AckTxDone, so
  the state stays `TxAck`:
  - `transmit_raw` refuses forever;
  - `ensure_receive_enabled` only restarts in `Receive`;
  - nothing restarts the receiver.
- **Trigger:** `ensure_receive_enabled`, which `received()` and
  `raw_received()` call on every poll, issued `RxStart` even while an
  RX_DONE was pending (the hardware ACK turnaround) or a frame was on the
  air. That aborted the ACK, and also cut off inbound frames.
  - During a join, the same race can abort the joiner's ACK of the
    parent's Transport-Key and cut off the parent's retransmissions.
- **Patch:**
  1. The receive-abort handler reads `rx_abort_status` before clearing it.
     For `TxAckTimeout`, `TxAckStop` or `TxAckCoexBreak` in state `TxAck`
     it runs `next_operation` (receiver back on, `rx_available` notified),
     as ESP-IDF v5.5.1 `isr_handle_rx_abort` does. It runs before the
     RX_DONE handling of the same interrupt, because an aborted ACK belongs
     to the previous frame.
  2. `TxAckStop` is added to the enabled receive-abort interrupts, so a
     stopped ACK is reported too. The driver never stops the radio itself
     in `TxAck`: `transmit_raw`, `apply_config` and suspend all refuse there.
  3. `ensure_receive_enabled` issues `RxStart` only in `Receive` with no
     pending event and no frame on the air. The ESP-IDF thread never
     restarts an active receive.
