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
