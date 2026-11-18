# esp-radio 0.18.0 for the ESP32-C5, with the local IEEE 802.15.4 driver

Used only by ESP32-C5 images (`examples/esp32c5-sensor`). The ESP32-C6/H2
images keep `vendor/esp-radio` (0.17.0) and are not affected by this
directory.

## Provenance

Base: the published `esp-radio-0.18.0.crate`, SHA-256
`23fbff98b06a96b6ce3791ecec5c668524052a068e23aacd23afe17ddba844ce`
(crates.io checksum). The published VCS metadata identifies upstream
commit `347003de8a48320bb7724f53045be3afa9204411`, directory `esp-radio`.
`LICENSE-MIT` and `LICENSE-APACHE` are byte-identical to the repository
root licences at that commit (the archive omits them). The archive's
`Cargo.lock` is removed. MIT/Apache-2.0 apply unchanged.

esp-radio 0.18.0 (with esp-hal 1.1, esp-phy 0.2 and
`esp-wifi-sys-esp32c5` 0.2.0) is the first release with ESP32-C5 IEEE
802.15.4 support (upstream PR #5070). Its C5 support outside the 802.15.4
driver is used as published: `radio_clocks` (modem clock gates,
`clocks_ll/esp32c5.rs`), the PHY initialisation and calibration in
`esp-phy`, the `esp-wifi-sys-esp32c5` libraries (`libphy.a`,
`libbtbb.a`, `libcoexist.a`) and the C5 linker fragments in `ld/`.

## What is local

`src/ieee802154/` is **not** upstream 0.18's driver. Upstream 0.18
rewrote the driver (TX deferral, hardware ACK reception with a 200 ms
Timer0 ACK timeout, `tx_failed` callbacks, ACK frames kept out of the
receive queue). Those are behaviour changes for the MAC above it, and the
upstream Timer0 use collides with the local triggered transmission. To give
the C5 exactly the behaviour already proven on the C6/H2, the directory
holds the C6/H2 driver from `vendor/esp-radio` (upstream 0.17.0 plus every
patch documented in `vendor/esp-radio/LOCAL-PATCHES.md`), adapted only to
0.18's crate internals:

| File | Origin | Change for 0.18 |
| --- | --- | --- |
| `mod.rs` | `vendor/esp-radio` | `PhyClockGuard`/`PhyInitGuard` and `enable_phy()` from `esp_phy` (moved out of `esp_hal::clock` / `PhyController`); `#![allow(missing_docs)]` as upstream 0.18's own `mod.rs` |
| `raw.rs` | `vendor/esp-radio` | imports from `crate::radio_clocks` and `crate::sys` as upstream 0.18 `raw.rs`; `esp_phy::enable_phy_clock()`, `enable_ieee802154(true)` and `esp_phy::enable_phy()` instead of the 0.17 peripheral clock traits; ZB_MAC bound with `IEEE802154::bind_mac_interrupt` (0.18); the C6 `coex_pti_tab_ptr` fix-up also on the C5 (`cfg(any(esp32c6, esp32c5))`, as upstream 0.18) |
| `hal.rs`, `frame.rs`, `fcf.rs`, `suspend.rs` | `vendor/esp-radio` | none |
| `pib.rs` | upstream 0.18.0 | none (has the C5 TX-power table: ESP-IDF v5.5.1 `hal/esp32c5/include/hal/ieee802154_ll.h`, −15…20 dBm, index min 3, same as the C6) |

Cargo features added to `Cargo.toml` and `Cargo.toml.orig`, with the same
meaning as in `vendor/esp-radio`: `ieee802154-ack-pending`,
`ieee802154-tx-when-rx-full`, `ieee802154-rx-timestamp`,
`ieee802154-triggered-tx`.

So the C5 gets the same local behaviour as the C6/H2, by construction:
retained-sleep suspend/resume, Frame Pending in the hardware auto-ACK,
transmit while the receive queue is full, discarding receives for a sleepy
device, `apply_config`/`can_transmit`, the TxAck-abort fix, the FCF-offset
fix (#5650), the RX time stamp, `transmit_raw_preempting_rx` and the
interrupt-started triggered transmission (Timer0, `TIMER0_THRESHOLD` in µs;
the C5 IEEE802154 register block and Timer0 match the C6 in the esp32c5
PAC used by esp-hal 1.1).

The `set_config` extended-address byte order of 0.17 is kept (upstream 0.18
changed it to little-endian); `zigbee-mac` relies on the 0.17 order on every
chip.

## Validation level

- Compiles for `riscv32imac-unknown-none-elf` with esp-hal 1.1.2 in every
  ESP32-C5 role image (sensor, ED, router, router with bidirectional GP).
- Host tests of `suspend.rs` and `fcf.rs` are the C6/H2 ones (same files).
- **Hardware:** see `docs/book/src/platforms/esp32.md` (ESP32-C5 section)
  for what has and has not been exercised on a C5. Radio timing (ACK
  turnaround, Frame Pending hook, triggered TX) is not qualified on the C5.

Re-basing on upstream: when the C6/H2 move to the esp-hal 1.1 train, both
directories can collapse into this one; until then a change to the driver
in `vendor/esp-radio/src/ieee802154/` must be copied here as well
(`diff -r vendor/esp-radio/src/ieee802154 vendor/esp-radio-0.18/src/ieee802154`
shows only the adaptations listed above and the upstream `pib.rs`).
`tools/check-esp-radio-c5-sync.sh` (CI) enforces it: the shared files must be
identical and `mod.rs`/`raw.rs` may differ only by the recorded
`ieee802154-port.diff`; regenerate the record with `--update` after porting
a change.

## Energy detection (`ieee802154-energy-detect`)

Same patch as `vendor/esp-radio/LOCAL-PATCHES.md` (shared `hal.rs`; the
`mod.rs`/`raw.rs` additions are identical and outside the recorded port
diff). Off by default; no C5 image enables it yet, and it has not run on a
C5.
