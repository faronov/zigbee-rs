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
