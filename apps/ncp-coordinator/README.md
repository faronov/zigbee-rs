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

* LINK: session, cookies/epoch, credits, `EVT_NCP_READY` first.
* SYSTEM: `SYS_NEGOTIATE`, `SYS_GET_INFO`, `SYS_RESET` (modes 0/1),
  `SYS_GET_STATUS`, `SYS_SET_CONFIG`/`SYS_GET_CONFIG` (known ids), `SYS_ECHO`,
  `SYS_SET_UART` (UART images only), `EVT_FATAL` on a flash-layout mismatch
  (NWK_INIT/NWK_FORM then answer `INVALID_STATE`).
* DIAG: `DIAG_GET_COUNTERS` (link group 0x01, NCP group 0x07),
  `DIAG_RESET_COUNTERS`.
* NWK: `NWK_INIT` (resume; host frame-counter floors raise the durable
  reservations first), `NWK_FORM` (NCP-chosen identity only; joining stays
  closed), `NWK_LEAVE_RESET`, `NWK_GET_INFO`, `NWK_PERMIT_JOIN`,
  `EVT_NETWORK_STATE`, `EVT_DEVICE_JOINED` (from Device_annce).
* DATA: `APS_DATA_REQUEST` → `EVT_APS_DATA_CONFIRM` (MAC/NWK delivery only),
  `EVT_APS_DATA_INDICATION` for host endpoints (runtime `host-passthrough`).
* SEC: `SEC_GET_NETWORK_KEY` (until `SEC_LOCK_KEY_EXPORT`),
  `SEC_GET_FRAME_COUNTERS`; one global outgoing APS counter with durable
  reservation (runtime `global-aps-counter`).
* LOG: `LOG_CONFIG`, `EVT_LOG_RECORD` from a bounded ring.

## Not implemented yet

TABLES (host paging), BULK backup/restore (OCB records), FW_* update, GP
sink events/commands, host-supplied formation parameters (PAN/EPID/key/
channel), `NWK_ADD_ENDPOINT` registry, retry cache, APS-ACK outcome in the
confirm, `EVT_DEVICE_LEFT`, RSSI in indications, PTA, IEEE override, the
256-entry link-key / 512-entry address caches, 64 children and streamed TC
snapshots.

## Hardware status

ESP32-C6 DevKit, USB-Serial-JTAG, `esp32c6,transport-usj` image through
hwlab (2026-11-18): link established, negotiate/info/echo/status/LOG/DIAG,
`NWK_FORM` formed a PAN on channel 16 after a 16-channel ED scan with joining
closed, `NWK_INIT` resumed it after a reset with NWK/APS counters advanced by
one reservation stride. Not run: any device join, ESP32-H2, the UART/Thread
BR transport and every gate H1–H12.

## Sizes (release, nightly-2026-08-01)

| Image | App image | .text | .rodata | .data | .bss |
|---|---|---|---|---|---|
| H2 USJ | 543,856 | 483,214 | 50,832 | 6,140 | 128,120 |
| H2 Thread BR UART | 550,208 | 489,616 | 50,884 | 6,172 | 130,736 |
| C6 USJ | 559,872 | 493,852 | 51,516 | 6,440 | 128,184 |

The 4 MB layout's app slot is 1,966,080 bytes (28 %); the 2 MB layout's is
917,504 (60 % for the UART image). `.bss` includes a 32 KiB heap (esp-radio),
the 33.5 KB `Ncp` (link windows + queues) and the 26 KB device.

## Tests

```text
cargo test --manifest-path apps/ncp-coordinator/Cargo.toml
cargo run -p zrhp-host -- --hwlab uart/<port> smoke [--form]
```

The crate is excluded from the root workspace: its runtime features
(`global-aps-counter`, `host-passthrough`) must not unify into the workspace
test graph.
