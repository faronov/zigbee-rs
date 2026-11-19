# ncp-coordinator-app

Platform-independent ZRHP v1 network co-processor
([spec](../../docs/book/src/advanced/host-protocol.md)): the zigbee-rs
coordinator and Trust Center (`router_app::TrustCenterCoordinatorApp`) behind
the shared `zigbee-zrhp` link and message codecs. The product supplies the
transport (`HostTransport`), the identity (`NcpIdentity`) and the stack
composition; this crate has no board, product or vendor dependency.

| Layer | Crate |
|---|---|
| Composition root (ESP32-H2/C6) | `examples/esp32-ncp` |
| Product: layout, NV stores, transport policy | `products/esp32-zigbee-ncp` |
| Board (Thread BR, H2 side) | `boards/esp-thread-br-h2` |
| Transports (USJ, UART + UHCI DMA) | `esp-serial-stream` |
| NCP dispatch, ports | this crate |
| Lab host harness | `tools/zrhp-host` |

## Implemented (host-tested end to end over an in-memory link)

* LINK: session, cookies/epoch, credits, `EVT_NCP_READY` first (with
  `table_epoch`), retry cache (6.5: 64 cached results ≤ 16 B, keyed by TID
  and message ID, cleared on a host restart).
* SYSTEM: `SYS_NEGOTIATE` (capabilities from the product platform:
  `NCP_FW_UPDATE`, `IEEE_REWRITABLE`, `PTA_COEX`; `BACKUP_RESTORE`),
  `SYS_GET_INFO` (factory/active IEEE), `SYS_RESET` (modes 0/1/2),
  `SYS_GET_STATUS` (PTA, restore, permit-join flags), `SYS_SET_CONFIG`/
  `SYS_GET_CONFIG` (log level, `coex_enable`, `coex_tx_priority_mask`),
  `SYS_ECHO`, `SYS_SET_IEEE` (persisted override, applied to the MAC and NIB
  at once when no network runs; `FF…` reverts), `SYS_SET_UART` (UART
  images), `EVT_FATAL` on a flash-layout mismatch.
* DIAG: `DIAG_GET_COUNTERS` (link group 0x01, NCP group 0x07),
  `DIAG_RESET_COUNTERS`.
* NWK: `NWK_INIT` (resume; host frame-counter floors raise the durable
  reservations first), `NWK_FORM` (NCP-chosen identity through BDB, or a
  host-supplied PAN ID/EPID/key/key_seq/update ID/manager/TX power on the
  quietest channel of the mask; joining stays closed), `NWK_LEAVE_RESET`
  (flags bit1 keeps host endpoints; a new network can be formed or restored
  in the same session), `NWK_GET_INFO`, `NWK_PERMIT_JOIN`,
  `NWK_ADD_ENDPOINT`, `EVT_NETWORK_STATE`, `EVT_DEVICE_JOINED`,
  `EVT_DEVICE_LEFT`.
* DATA: `APS_DATA_REQUEST` → `EVT_APS_DATA_CONFIRM` (after the APS ACK
  outcome for `ack_request` unicasts), `EVT_APS_DATA_INDICATION` with
  last-hop RSSI, APS security and key source.
* SEC: `SEC_GET_NETWORK_KEY` (until `SEC_LOCK_KEY_EXPORT`),
  `SEC_GET_FRAME_COUNTERS`; one global outgoing APS counter.
* TABLES (standalone mode, 8.1): `TBL_READ`/`TBL_STATS` of the DEVICE table
  (Trust Center records and children), `TBL_PUSH` (unique TCLK provisioning),
  `TBL_INVALIDATE` (device deletion), `TBL_SYNC`; `EVT_TBL_EPOCH_CHANGED`
  after a wipe (reason 1) and a restore (reason 2).
* BACKUP: export (`NODE`, `NETWORK`, keys, `DEVICE` records, `RS_PARAMS`,
  `END` with CRC-32C, generated per read) and restore (records validated per
  9.4 rule 2, staged in RAM with up to 64 link keys, `END` checked at
  commit, counter rules 4/6 including the 2²⁰ unknown-TCLK margin, IEEE
  override with flags bit0).
* FW (product platform): `FW_BEGIN`/`WRITE`/`FINISH`/`ACTIVATE`/`CONFIRM`/
  `ABORT`/`GET_SLOTS`: inactive A/B slot, SHA-256 and signature policy,
  pending-verify trial with host confirmation, rollback by the IDF
  bootloader or the 10-minute trial window (`products/esp32-zigbee-ncp`).
* GREEN POWER: the NCP combo sink/proxy commands and events (7.11).
* PTA (product `pta` feature, Thread BR board): `coex_enable` and the
  priority mask drive the H2 external-coexistence follower; off by default.
* LOG: `LOG_CONFIG`, `EVT_LOG_RECORD` from a bounded ring.

## Not implemented yet

Host paging (`TBL_LOOKUP` misses, eviction `TBL_WRITEBACK`, `HOST_TABLES`
mode), per-entry `rev` and dirty tracking (always 0), SOURCE_ROUTE tables,
child-table import on restore (children are `CHILD_DEMOTED` and rejoin),
`BKP_RESTORE_BEGIN` while the network runs (the host restores after a reset,
before `NWK_INIT`), restore commit marker (a power cut during
`BKP_RESTORE_COMMIT` leaves the previous network wiped or the new one
partly written; the host repeats the restore), `table_epoch` as a persisted
counter (it is a CRC of the stored network identity, so restoring the same
network keeps the epoch), `src_ep` validation, `EVT_DEVICE_AUTHORIZED`,
`EVT_ADDRESS_CHANGED`, `NWK_ENERGY_SCAN`, `NWK_CHANGE_CHANNEL`, network key
rotation commands.

## Hardware status

* ESP32-C6 DevKit, USB-Serial-JTAG (hwlab, 2026-11-18): link,
  negotiate/info/echo/status/LOG/DIAG, `NWK_FORM` (channel 16, joining
  closed), `NWK_INIT` resume after reset with forward-only counters.
* ESP32-H2 on the ESP Thread Border Router board, UART 460 800 through the
  S3 bridge (2026-11-18, IDF v5.5.1 rollback bootloader, 4 MB A/B table):
  smoke (echo RTT ≈ 5.6 ms), `NWK_FORM` (PAN 0x1C06, channel 17), reset +
  `NWK_INIT` resume with forward-only counters (gates H2/H8 subset).
* Not run on hardware: TABLES, BACKUP, FW_* (update, trial, rollback), IEEE
  override, PTA, GP sink, the heap in `dram2_seg`, any device join and the
  remaining gates.

## Sizes (release, nightly-2026-08-01, 2026-11-19)

| Image | App image | .text | .rodata | .data | .bss | Stack gap |
|---|---|---|---|---|---|---|
| H2 USJ | 695,456 | 626,634 | 57,644 | 7,508 | 169,356 | 41,420 |
| H2 Thread BR UART | 702,000 | 633,244 | 57,680 | 7,540 | 172,068 | 38,572 |
| H2 Thread BR UART + PTA | 709,264 | 638,632 | 59,288 | 7,640 | 172,076 | 38,284 |
| C6 USJ | 711,424 | 637,218 | 58,324 | 7,816 | 169,420 | 230,780 |

The 4 MB layout's app slot is 1,966,080 bytes (36 %); the 2 MB layout's is
917,504 (77 % for the UART image). `.bss` includes the `Ncp` (link windows,
queues, retry cache, restore staging) and the device with the
`ncp-capacity` tables; the 32 KiB esp-alloc heap is in `dram2_seg`. The stack
gap is the static worst case (`tools/esp32-ncp-stack-gate.py`) against the
RAM above `.bss`.

## Tests

```text
cargo test --manifest-path apps/ncp-coordinator/Cargo.toml
cargo run -p zrhp-host -- --hwlab uart/<port> smoke [--form]
```

The crate is excluded from the root workspace: its runtime features
(`global-aps-counter`, `host-passthrough`) must not unify into the workspace
test graph.
