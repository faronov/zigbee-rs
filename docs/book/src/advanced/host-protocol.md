# Host ↔ Coordinator Protocol (ZRHP v1, design draft)

> **Status: design specification. The user's decisions of 2026-11-18 are
> applied (section 16).** The shared crate `zigbee-zrhp` (step 1 of section
> 15) implements the framing, the link layer and every message and record
> codec, and passes all vectors; it is host-tested only. A first NCP
> (`apps/ncp-coordinator`, `products/esp32-zigbee-ncp`, `examples/esp32-ncp`)
> implements the SYSTEM, DIAG, NWK, DATA, SEC and LOG subset listed in
> [`apps/ncp-coordinator/README.md`](https://github.com/faronov/zigbee-rs/blob/zrhp-ncp/apps/ncp-coordinator/README.md);
> on an ESP32-C6 DevKit (lab variant, USB-Serial-JTAG) it completed the
> session, formed a network (soft MAC `pan-coordinator` + `energy-detect`)
> and resumed it after a reset. No device has joined it yet and none of the
> hardware gates H1–H12 has passed. The zigpy library, Go adapter,
> zigbee-herdsman adapter and embedded hub described here do not exist yet.
> Byte layouts are normative for v1. The golden vectors in
> [`host-protocol-vectors/`](host-protocol-vectors/generate.py) are generated
> from these layouts by a small Python script. The script is a vector
> generator, not an implementation.

ZRHP (zigbee-rs Host Protocol) connects a host to a zigbee-rs Network
Co-Processor (NCP). The NCP runs a complete Zigbee PRO coordinator, Trust
Center and Green Power combo (proxy + sink). The host keeps the large tables.

Supported hosts:

- **Desktop-class hosts:** Home Assistant through zigpy, the Go Zigbee hub,
  and Zigbee2MQTT through zigbee-herdsman.
- **Embedded host:** the Rust hub `faronov/zigbee-rs-hub`, running on the
  ESP32-S3 of the Espressif Thread Border Router / Zigbee Gateway board.

**Transport scope.** ZRHP is defined only over USB-Serial-JTAG (CDC-ACM) and
UART, including the S3↔H2 UART of that board. There is no network transport
in ZRHP. A hub that offers a network API, such as the embedded hub's
WebSocket JSON API, does so at the hub level, outside this specification.

Contents:

1. [Goals and non-goals](#1-goals-and-non-goals)
2. [Prior art and what we take](#2-prior-art-and-what-we-take)
3. [Architecture, state ownership and host profiles](#3-architecture-state-ownership-and-host-profiles)
4. [Transports](#4-transports)
5. [Link layer](#5-link-layer)
6. [Message layer](#6-message-layer)
7. [Command set v1](#7-command-set-v1)
8. [Hybrid table protocol](#8-hybrid-table-protocol)
9. [Backup, restore and Open Coordinator Backup](#9-backup-restore-and-open-coordinator-backup)
10. [Performance model](#10-performance-model)
11. [First target: ESP32-H2 / ESP32-C6](#11-first-target-esp32-h2--esp32-c6)
12. [Host integration](#12-host-integration)
13. [Security of the host link](#13-security-of-the-host-link)
14. [Test plan](#14-test-plan)
15. [Implementation work and validation gates](#15-implementation-work-and-validation-gates)
16. [Decisions (2026-11-18) and new open questions](#16-decisions-2026-11-18-and-new-open-questions)
17. [References](#17-references)

All multi-byte integers are **little-endian**. IEEE addresses travel in
on-air order: least significant byte first, exactly as they appear in a MAC
header. Keys travel as the 16 bytes fed to AES, in the order zigpy's
`KeyData.serialize()` produces. `u8`/`u16`/`u32`/`u64` are unsigned, `i8` is
two's complement and `[N]` is a fixed byte array.

---

## 1. Goals and non-goals

### Goals

| # | Goal | How it is met |
|---|---|---|
| G1 | Detect corruption on noisy lines and recover quickly | COBS framing resynchronizes at every `0x00`. A CRC-32C covers each frame (HD=6 at the chosen maximum length). Immediate NAK on a CRC error or sequence gap. |
| G2 | Fast in large networks | Selective-repeat ARQ with a window of 8–32 frames. 16-bit transaction IDs with many requests in flight. Several messages per frame. Indications never wait for a host request. |
| G3 | No head-of-line blocking | Endpoints are multiplexed. Each endpoint has its own ordering sequence and its own credits, so a lost bulk frame never delays Zigbee data. |
| G4 | Never drop silently | Credit flow control. Every overload ends in an explicit `BUSY`/`REJECT` status or a counted, reported drop. |
| G5 | Either side may reboot at any time | Boot cookies and a session epoch. Idempotent retransmission. The network is resumed without frame-counter reuse. |
| G6 | 1000+ devices on a chip with tens of KiB of RAM | Hybrid split: the stack runs on the chip and the host owns the large tables, with a paging protocol between them. |
| G7 | HA backups and migrations work | Full Open ZigBee Coordinator Backup (OCB) v1 export and import, plus migration in both directions with EZSP, ZNP, ZBOSS and deCONZ. |
| G8 | One reference implementation | The shared crate `zigbee-zrhp` (`no_std`, optional `alloc`/`std`) is used by the NCP firmware, by the embedded hub on esp-idf-svc, and by desktop tooling. Rust, Go, Python and TypeScript implementations must pass the same JSON vectors. |
| G9 | Embedded hosts are first-class | No layout, buffer or behaviour assumes a desktop-class host: bounded buffers, bounded TID tables, no JSON on any protocol path, and stated minimum host resources (3.3). |

### Non-goals (v1)

- No Zigbee stack on the host. This is not an RCP split; MAC, NWK, APS, ZDO,
  BDB, security and the Trust Center stay on the chip.
- No compatibility with EZSP, ZNP, ZBOSS NCP or deCONZ on the wire.
  Compatibility is provided at the *backup* level (OCB) and by new host
  drivers.
- No network transport (TCP, TLS, WebSocket, ser2net framing). ZRHP is
  defined over USB-Serial-JTAG and UART only. Tunnelling the byte stream
  across a network is outside this specification and unsupported.
- No link encryption in v1. Section 13 reserves the hooks for it.
- No ZDO loopback. Requests to the NCP's own address are refused; hosts use
  `NWK_*`/`DIAG_*` commands instead (7.4).
- No Touchlink or inter-PAN in v1 (capability bit 19 stays reserved).
- No multi-network or multi-PAN NCP.
- No sleepy NCP. The NCP is mains-powered and its receiver is always on.
- No router or end-device NCP. The NCP is always a centralized-security
  coordinator and Trust Center.

### Terminology

| Term | Meaning |
|---|---|
| NCP | The chip running zigbee-rs as coordinator, Trust Center and GP combo. |
| Host | zigpy (Python), the Go hub, zigbee-herdsman (TypeScript) or the embedded Rust hub, connected over one byte stream. |
| Desktop host / embedded host | The two host profiles of 3.3. |
| Frame | One COBS-delimited link-layer unit. |
| Message | One request, response or event inside a DATA frame. |
| Endpoint (link) | A multiplexed lane (`SYSTEM`, `ZIGBEE`, …). Not a Zigbee endpoint. |
| Cookie | A random 32-bit value chosen at each boot (NCP) or port open (host). |
| Epoch | An 8-bit session tag derived from both cookies. |
| Reservation | A durable outgoing frame-counter upper bound (`FRAME_COUNTER_RESERVATION_SIZE` = `0x400`). |

---

## 2. Prior art and what we take

| Protocol | Framing | Integrity | Reliability / window | Concurrency | Multiplexing | Notes / source |
|---|---|---|---|---|---|---|
| ASH + EZSP (Silicon Labs) | `0x7E` flag, byte stuffing of 6 reserved bytes, LFSR data randomization | CRC-CCITT 16 | 3-bit frame/ack numbers, `TX_K` ≤ 7; bellows uses window 1 | EZSP: one command at a time (bellows `MAX_COMMAND_CONCURRENCY = 1`) | None (callbacks share the stream) | DATA 3–128 B. T_RX_ACK 0.4–3.2 s adaptive. RST/RSTACK reset. [UG101], [UG100], [bellows] |
| ZBOSS NCP LL | `DE AD` signature, length | Header CRC-8, body CRC-16 | 2-bit seq, stop-and-wait, 1 s ACK timeout | 8-bit TSN, several high-level requests in flight | None | LL body ≤ 247 B. The fragment flags are unused by both hosts. [zigpy-zboss] |
| TI MT / ZNP | SOF `0xFE` + length | XOR FCS (8 bit) | None | One SREQ at a time | None | Payload ≤ 250 B. [SWRA198], [zigpy-znp] |
| deCONZ serial | SLIP (RFC 1055) | 16-bit two's-complement sum | None; device "free slots" flag | 8-bit seq | None | ASDU ≤ 127 B. [deCONZ] |
| Spinel / HDLC-lite (OpenThread) | `0x7E` HDLC-lite | CRC-16 (FCS) | None at framing level | 4-bit TID | 2-bit IID | Property model; MTU ≥ 1300 B. [Spinel] |
| Silicon Labs CPC | HDLC-like header with HCS | Header CRC-16 + payload FCS | I/S/U frames, window 1–7 (API states 1 is currently supported) | Per endpoint | Endpoints (user range 90–99) | Optional AES-GCM security. [CPC] |
| **ZRHP v1** | **COBS, `0x00` delimiter** | **CRC-32C over header + payload** | **Selective repeat, window 8–32, SACK bitmap, immediate NAK** | **16-bit TIDs, many in flight, batching** | **6 endpoints, per-endpoint ordering and credits** | This chapter |

What we take, and why:

- **From ASH:** explicit RST/RSTACK-style session reset, NAK on error, and an
  adaptive retransmission timer. We do not take byte stuffing: HDLC escaping
  can double the size of worst-case data, whereas COBS has a bounded overhead
  of 1 byte per 254. We do not take LFSR randomization, which only exists to
  avoid long runs for flag stuffing. A window of 1–7 is far too small.
- **From EZSP / ZBOSS:** a two-phase data send (accepted, then confirmed),
  callbacks as first-class events, and structured network/security commands.
  We do not take serialized command execution or "any timeout kills the
  session" (both are pain points in the Go hub's EZSP adapter).
- **From MT/ZNP:** the clear SREQ/SRSP/AREQ distinction, mapped to our
  request, response and event kinds. We do not take XOR checks or the lack of
  ARQ.
- **From deCONZ:** device-driven "free slots" flow control, generalized here
  into per-endpoint credits.
- **From Spinel:** small TIDs and a clean split between framing and
  semantics. We use 16-bit TIDs so that hundreds of requests can be in
  flight.
- **From CPC:** endpoints multiplexed over one link, each with its own
  service, plus a reserved security option. Unlike CPC we add priorities,
  credits and a selective-repeat window larger than 1.

Using CRC-32C (iSCSI, SCTP, ext4) instead of a 16-bit CRC raises the
undetected-error floor from about 2⁻¹⁶ to 2⁻³² per corrupted frame and gives
Hamming distance 6 up to 655 data bytes. Section 5.14 has the analysis.

---

## 3. Architecture, state ownership and host profiles

```text
 Host (zigpy / Go hub / herdsman / zigbee-rs-hub)   NCP (zigbee-rs)
+--------------------------------+            +--------------------------------------+
| ControllerApplication / Adapter|            | ncp-coordinator app (new)            |
|   device DB (authoritative)    |            |   command dispatch, host-endpoint    |
|   link-key store (all devices) |  ZRHP msgs |   table, indication/confirm queues   |
|   FC checkpoint log            |<==========>| TrustCenterCoordinatorApp + GP combo |
|   group/binding mirrors        |            |   zigbee-runtime (ZigbeeNode)        |
|   OCB export/import            |            |   BDB -> ZDO/APS -> NWK -> MAC       |
+--------------------------------+            |   host-backed caches (new)           |
| zigbee-zrhp (Rust) or a Go/Py/ |  bytes     | zigbee-zrhp (shared Rust, no_std)    |
| TS port; serial: USJ / UART    |<==========>| transport: USB-Serial-JTAG / UART    |
+--------------------------------+            +--------------------------------------+
```

Planned placement, following the repository layering rules:

| Piece | Location (new) | Depends on |
|---|---|---|
| `zigbee-zrhp`: COBS, CRC-32C, frames, sessions, ARQ, credits, scheduler (`link`); message, command and record codecs (`msg`, `records`); optional host helpers (resume, retry cache, TID map) | `zigbee-zrhp/` in the zigbee-rs workspace. `no_std` by default; features `alloc` (growable host queues) and `std` (blocking/async serial adapters, desktop tools). The embedded hub uses it from esp-idf-svc `std` on the ESP32-S3. | `heapless`, `zigbee-types` only. No runtime, board, product or vendor SDK. |
| NCP application (command dispatch, caches, GP sink events) | `apps/ncp-coordinator/` | `zigbee-runtime`, `zigbee-zrhp` |
| ESP32-H2 NCP product (layout, NV partitions, identity, signed update, transport choice) | `products/esp32-zigbee-ncp/` | app, chip HAL, board |
| Boards | ESP32-H2 devkit (USJ, or UART through an external bridge with RTS/CTS); Espressif Thread BR / Zigbee GW board, H2 side (UART0 to the S3, PTA, BOOT/EN driven by the host) | chip HAL |
| Byte-stream transports (USJ, UART + optional RTS/CTS, UHCI DMA RX) | chip HAL / platform glue exposing `embedded-io(-async)` `Read + Write` | `esp-hal` |
| Desktop host tools (vector runner, noise bench, CLI, reference host) | `tools/zrhp/` (`std`) | `zigbee-zrhp` (`std`) |
| Embedded hub | separate public repository `faronov/zigbee-rs-hub` (ESP32-S3, esp-idf-svc) | `zigbee-zrhp` (git dependency, `std`) |
| zigpy library / Go adapter / zigbee-herdsman adapter | `zigpy-zrhp` (Python, under `faronov`, with the GP hook in the `faronov/zigpy` fork), `~/go_zboss-ncp/adapter/zrhp`, `src/adapter/zrhp` in a maintained zigbee-herdsman fork | Ports of `zigbee-zrhp`, checked against the shared vectors |

The protocol crate must not depend on boards, products, vendor SDKs or
logging transports. The transport adapters are the only platform-specific
parts of the link.

### 3.1 State ownership (summary)

Section 8 defines the paging protocol and section 9 maps every backup field.

| State | Authoritative owner | Copy | Durability |
|---|---|---|---|
| NIB: PAN ID, EPID, channel, mask, update ID, manager, short address | NCP | Host (backup) | NCP NV journal |
| Coordinator IEEE (factory or override) | NCP | Host | NCP identity NV item |
| Active and alternate network key, key sequence | NCP | Host (backup, export-gated) | `SecurityStateJournal` |
| Outgoing NWK frame counter | NCP | Host (checkpoint log) | NCP reservation journal; the host log is a floor |
| **Global outgoing APS frame counter** (one counter for all link keys, 8.4) | NCP | Host (checkpoint log) | NCP reservation journal |
| Global TCLK, EZSP hashed-TCLK seed, TC policy | NCP | Host (backup) | NCP NV |
| Incoming NWK replay floors | NCP (RAM-only, as today) | — | — |
| Neighbor, routing, route-discovery and broadcast-transaction tables | NCP | Host reads for diagnostics | RAM |
| Child table (coordinator's own children) | NCP | Host (backup) | `ChildTableJournal` |
| Indirect queue, APS duplicate/fragment state | NCP | — | RAM |
| **Device database** (IEEE ↔ NWK, type, capability) | **Host** | NCP address-map cache (clean, rebuildable) | Host store |
| **Per-device link keys + incoming APS floor** | **Host** | NCP TC-device cache | Host store; NCP journal while cached |
| **Source routes for all devices** | **Host** | NCP source-route cache | Host store |
| Bindings and groups whose source is the coordinator | Host | NCP binding/group tables | Both |
| GP Proxy Table | NCP | Host (backup) | GP journal |
| **GP Sink Table** (the NCP is the sink, 7.11) | NCP | Host (event mirror, backup) | GP journal (`ZBGS`) |

### 3.2 Repository and naming

- **Protocol name:** ZRHP.
- **Shared Rust crate:** `zigbee-zrhp`.
- **Python package:** `zigpy-zrhp` (repository under `faronov`); the zigpy
  radio type is `zrhp`. The GP sink-on-radio hook lives in the
  `faronov/zigpy` fork (decision N2).
- **Go adapter:** `adapter/zrhp`.
- **zigbee-herdsman adapter:** `zrhp`.
- **Embedded hub:** `faronov/zigbee-rs-hub`.

The hub's own API and UI (a WebSocket JSON API) is a hub-level interface. It
is not part of ZRHP and is not specified here.

### 3.3 Host profiles

| Aspect | Desktop host (D) | Embedded host (E) |
|---|---|---|
| Examples | zigpy (HA), Go hub, zigbee-herdsman (Z2M) on Linux/macOS/Windows | `zigbee-rs-hub` on ESP32-S3-WROOM-1 (8 MB flash, or 4 MB on early boards; 2 MB PSRAM) |
| Transport | USB-Serial-JTAG CDC (H2/C6 native USB), or UART through a USB-UART bridge with RTS/CTS (default 921 600) | Board UART to the H2 (Thread BR board: 460 800 default, RTS/CTS optional on the reserved lines, 11.5) |
| NCP reset / update | USJ DTR/RTS reset (11.3); firmware update via `FW_*` over BULK | `FW_*` over BULK (normal path); H2 ROM bootloader through host-driven EN/BOOT GPIOs (recovery and provisioning, 11.5) |
| Host store | SQLite (zigpy sidecar, Go hub DB) or a sidecar file (herdsman) | Raw-flash log-structured journal partition. No filesystem assumed. |
| Store sizing for 1000+ devices | Unbounded | ≈ 64 B/device in PSRAM: 2 000 devices ≈ 128 KiB (3.4) |
| RX window advertised | 32 | 8 (16 when PSRAM-backed) |
| Paging (`HOST_TABLES`) | Optional; auto-enabled above NCP capacity (8.1) | Same |
| OCB JSON export/import | Required (zigpy, herdsman) or optional (Go hub) | Optional. The hub may stream binary records to a desktop client that converts them. |
| LOG endpoint | Subscribed (credit > 0) | Optional (grant 0 credit to disable, 5.10) |
| Seamless NCP-reboot resume | zigpy: required (decision 5); others: recommended | Required |
| GP sink mode | NCP sink (zigpy); host sink (Go, herdsman v1, 7.11) | NCP sink |

**Rules for every host.** Nothing in ZRHP may require more than these:

- **Receive.** One COBS accumulation buffer of 656 B, plus `rx_window` ×
  652 B decoded frames. The minimum `rx_window` is **4**, giving ≈ 3.3 KB.
  Hosts may decode in place.
- **MTU.** `rx_mtu` is fixed at **640** in v1. Every message fits one frame,
  so no host needs reassembly.
- **Transmit.** Up to `min(peer rx_window, local TX buffers)` stored frames
  for byte-identical retransmission. With a window of 4 that is ≈ 2.6 KB.
- **Pending host requests.** A host-chosen bound. The NCP never requires
  more than one host request to be in flight.
- **NCP-originated requests** (`TBL_LOOKUP`, `TBL_WRITEBACK`) are bounded by
  `max_ncp_requests` (capacity 17, default 8). A host must be able to hold
  that many.
- **Batching.** Optional for senders. Receivers parse batches in place,
  without an extra buffer.
- **No JSON or text parsing on any protocol path.** OCB JSON is a host-side
  export format built from binary records (9.3). The only strings are short
  bounded UTF-8 names (≤ 32 B), versions (≤ 32 B) and log text (≤ 200 B).
- **Clock.** A monotonic millisecond clock (`esp_timer` on the S3).
- **Responses.** `TBL_LOOKUP` must be answered within 250 ms, from RAM or
  PSRAM. A store that needs longer must preload.

### 3.4 Embedded host budget (ESP32-S3 on the Thread BR board)

| Item | Size | Placement |
|---|---:|---|
| ZRHP link: COBS buffer + RX window 8 × 652 + TX window 8 × 652 | 11.1 KB | Internal SRAM (DMA- and ISR-accessible) |
| Pending request map (64 × 16 B) + NCP request slots (8 × 32 B) | 1.3 KB | SRAM |
| Link-key store record (IEEE 8, key 16, rx floor 4, rev 4, attrs/origin/seq 4, NWK 2, flags 2) | 40 B/device | PSRAM |
| Minimal device registry for ZRHP purposes (IEEE, NWK, capability, rx-on, last seen) | 24 B/device | PSRAM |
| **2 000 devices** | **≈ 128 KiB** | PSRAM (2 MB) |
| GP sink mirror (16 × 40 B), FC checkpoint ring in RAM (16 × 16 B) | < 1 KB | SRAM |

Application data such as names, clusters and attribute caches is hub-level,
outside ZRHP.

**Flash store.** A 128 KiB journal (32 × 4 KiB) with the same commit
discipline as the NCP journals: CRC + commit marker, two-generation snapshot
plus append records. It lives in the hub's data partition (below): a raw
region, or a preallocated file pair in a power-loss-safe filesystem
(LittleFS). ESP-IDF NVS is acceptable if it meets the durability rule of 8.8.

**S3 flash layout** (decision N9). Retail Thread BR boards carry 8 MB; early
samples carry 4 MB (the hwlab board: Zbit 5e4016). The hub detects the size
at boot and uses the matching partition table; a table that does not match
the detected size is a fatal configuration error. **The S3 stores no NCP
images** (decision N6, amended 2026-11-18): NCP rollback is on the H2 (8.9).

| Partition | 4 MB | 8 MB |
|---|---|---|
| Bootloader | 0x000000, 32 KiB | same |
| Partition table | 0x008000, 4 KiB | same |
| `nvs` (Wi-Fi, PHY calibration, small settings) | 0x009000, 24 KiB | same |
| `otadata` | 0x00F000, 8 KiB | same |
| `phy_init` | 0x011000, 4 KiB | same |
| Reserved (optional core dump) | 0x012000, 56 KiB | same |
| Hub app `ota_0` | 0x020000, **1.5 MiB** | 0x020000, 3 MiB |
| Hub app `ota_1` | 0x1A0000, **1.5 MiB** | 0x320000, 3 MiB |
| `hubdata` (LittleFS or log-structured store) | 0x320000, **896 KiB** | 0x620000, 1 920 KiB |

- **Hub app ≤ 1.5 MiB** on 4 MB boards. The hub CI enforces the limit.
  The stock `esp_ot_br` (Wi-Fi, OpenThread, web server) is 1.6 MB; the hub
  has no OpenThread.
- **`hubdata` (4 MB)**: host store journal 128 KiB, configuration 32 KiB,
  device-converter database 2 × 256 KiB (active + staged, swapped
  atomically), the rest (224 KiB, 25 %) free for copy-on-write and wear
  levelling.
- **Converter database.** Device definitions are **declarative data**
  (compact CBOR records: model and manufacturer ids, exposes, cluster and
  attribute mappings, reporting, references to generic handlers built into
  the app). They are updated without an app OTA. Converter logic stays in
  the app as a fixed set of generic handlers; the database contains no code.
  This is hub-level and outside ZRHP.

**Write-rate estimate** (one 32-byte append per event):

- **FC checkpoints.** The host must persist one record per ≤ 8 checkpoints
  (8.4).
  - At an air-limited 200 frames/s that is one record per 41 s, or ≈ 67 KB
    per day.
  - A 128 KiB ring is therefore rewritten every ≈ 1.9 days. Each sector sees
    about 190 erases per year, so 100 k-cycle NOR flash outlasts the product
    by orders of magnitude.
  - A typical home at 20 frames/s writes 10 × less.
- **Writebacks.** NEW keys (joins), attribute changes and evictions are all
  rare.

---

## 4. Transports

ZRHP v1 needs one full-duplex, ordered, unreliable byte stream. "Unreliable"
means bytes may be corrupted, dropped or inserted by noise, but the stream
never reorders bytes. The link layer relies on this FIFO property for
loss inference (section 5.6). Exactly two transports are defined:
USB-Serial-JTAG CDC-ACM and UART.

### 4.1 UART

- **Format.** 8N1. Defaults:
  - **921 600 baud** with RTS/CTS for desktop hosts behind a USB-UART bridge;
  - **460 800 baud** on the Espressif Thread BR board (11.5).

  Supported rates are 460 800, 921 600, 2 000 000 and 3 000 000 (C6 only),
  subject to 4.4. The NCP does not autobaud.
- **Changing the rate.** A host may change the rate at runtime with
  `SYS_SET_UART` (7.1). The new rate is volatile; the product default is
  restored on reboot.
- **With RTS/CTS.** Credits make sure RTS/CTS only ever pauses a frame and
  never loses one.
- **Without RTS/CTS.** Credits are the flow control for whole frames: each
  direction has at most `window × 656` bytes outstanding, and the receiver
  has buffer space for all of them. Credits cannot pause the hardware FIFO
  in the middle of a frame. Both sides must therefore receive through DMA
  (ESP UHCI/GDMA) or an IRAM-resident RX interrupt into a ring of at least
  `rx_window × 656` bytes. The receiver must keep running while the same
  chip erases or writes flash.
- **No XON/XOFF.** Software XON/XOFF is not used: `0x11`/`0x13` occur in
  COBS data.
- **Line errors.** A UART framing, parity or overrun error is handled
  exactly like a CRC failure. The receiver aborts the current frame,
  discards bytes until the next `0x00`, increments `nbad` and sends a NAK
  (5.7). It also increments `rx_overflow` (5.12) when the error is a FIFO
  overrun.
- **C6 UART.** Up to 5 Mbaud from an 80 MHz source, with 128-byte FIFOs,
  RTS/CTS, an RX-timeout interrupt and UHCI GDMA [ESP-TRM].
- **H2 UART.** The clock sources are 48 MHz PLL, 32 MHz XTAL and RC. With
  the ESP UART divider this caps the H2 at about 3 Mbaud (48 MHz / 16).
  The **measured** maximum is still open (gate H2).

### 4.2 USB CDC-ACM (USB-Serial-JTAG)

The host sees a normal CDC-ACM serial port. On the ESP32-H2/C6
USB-Serial-JTAG peripheral, line coding is ignored, there is no RTS/CTS, and
flow control is USB NAK only. Section 11 lists the constraints this imposes.

### 4.3 Later transports (extensions, not v1)

| Transport | Framing difference | Notes |
|---|---|---|
| USB vendor bulk | No COBS. Each frame is one USB bulk transfer terminated by a short packet or ZLP. CRC-32C is kept because USB CRC does not cover host-stack bugs. | Higher throughput on chips with a USB-OTG controller. Keeps Espressif's VID/PID (decision N8). |
| SPI + INT (embedded host is the SPI master) | No COBS. Each transaction carries a 4-byte prefix `{u16 len, u16 ~len}` followed by the frame. The NCP raises INT while it has TX data or new credits. | ARQ, sessions and messages are unchanged. |

No network transport is planned (decision 2026-11-18).

### 4.4 Maximum safe UART rate

| Configuration | Maximum rate | Condition |
|---|---|---|
| RTS/CTS on both sides | 2 000 000 on the H2 (3 M if gate H2 passes); 3 000 000 on the C6 | Bridge supports the rate (for example CP2102N up to 3 M) |
| No RTS/CTS, both sides DMA or IRAM-ISR RX into a ring ≥ `rx_window × 656` | 921 600 | Gate H8: `rx_overflow = 0` under host and NCP flash-write load |
| No RTS/CTS, any side without DMA/IRAM RX | 460 800, ARQ recovers losses at a goodput cost | Espressif's own RCP link default on the BR board |

The worst-case FIFO-free time at 921 600 with a 128-byte FIFO and an RX
threshold of 64 bytes is 64 × 10.85 µs ≈ 0.69 ms. An IRAM ISR meets this.
A flash-cache-dependent ISR does not, because ESP32 flash erase stalls the
cache for milliseconds.

---

## 5. Link layer

### 5.1 COBS framing

- Each frame is COBS-encoded (Cheshire & Baker [COBS]) and followed by one
  `0x00` delimiter. The encoded bytes never contain `0x00`.
- A sender MAY emit an extra `0x00` before a frame. It SHOULD do so before
  the first frame after the line has been idle for 100 ms or more, and always
  after opening the port or booting. A receiver ignores empty frames (two
  consecutive delimiters).
- The receiver accumulates bytes up to `0x00`, then decodes. Any of these
  conditions discards the frame, increments `rx_bad_frame` and counts as one
  `nbad` (5.7):
  - more than 656 encoded bytes, which also covers a missing delimiter;
  - a COBS code byte pointing past the end of the frame;
  - a decoded length different from `8 + len + 4`;
  - a CRC mismatch.
- **Canonical encoding.** Encoders emit the *minimal* COBS form: when the
  data ends exactly after a full 254-byte block, that block's `0xFF` code
  closes the frame and no empty `0x01` block follows (254 non-zero bytes
  encode to 255 bytes, not 256). Decoders must also accept the non-minimal
  form with that trailing `0x01` code (`cobs.json`, `decode_only`).
- Worst-case overhead: for a decoded frame of `n` bytes the encoding is at
  most `n + ⌈n/254⌉` bytes, plus 1 delimiter. With the maximum `n = 652`
  (8 header + 640 payload + 4 CRC) the maximum wire size is **656 bytes**
  (0.46 % + 1 byte). The minimum is 1 code byte + 12 + delimiter = 14 bytes
  for an empty payload.

COBS was chosen over HDLC because its overhead is bounded, and because a
`0x00` (a common result of line noise, breaks and floating lines) always
resynchronizes the receiver within one byte.

### 5.2 Frame layout (decoded)

| Offset | Size | Field | Description |
|---:|---:|---|---|
| 0 | 1 | `ctrl` | bits 7–6 `version` (= 1); bit 5 `POLL`; bit 4 `RETX`; bits 3–0 `type` |
| 1 | 1 | `epoch` | Session tag (5.5); `0` only in RESET frames |
| 2 | 1 | `endpoint` | Link endpoint (5.10); `0` in non-DATA frames |
| 3 | 1 | `seq` | Link sequence number of a DATA frame (mod 256); `0` otherwise |
| 4 | 1 | `ack` | Cumulative ack: next DATA `seq` the sender of this frame expects (all earlier seqs received). Valid in DATA/ACK/NAK/REJECT. |
| 5 | 1 | `ep_seq` | Per-endpoint ordering sequence of a DATA frame (mod 256); `0` otherwise |
| 6 | 2 | `len` | Payload length, 0..640 |
| 8 | `len` | `payload` | Type-specific |
| 8+len | 4 | `crc` | CRC-32C over bytes `[0, 8+len)`, little-endian |

The CRC is CRC-32C (Castagnoli): reflected polynomial `0x82F63B78`, init
`0xFFFFFFFF`, xorout `0xFFFFFFFF`. Check value `crc32c("123456789") =
0xE3069283` (`crc32c.json`).

Both sides validate in this order: COBS decode, length equality, CRC,
`version == 1`, known `type`, `epoch` (5.5), then the type-specific rules.
After a valid CRC, an unknown version or type is answered with REJECT
`UNSUPPORTED` (5.3), because it signals a peer bug or a version mismatch
rather than noise.

### 5.3 Frame types

| `type` | Name | Payload | Reliable | Notes |
|---:|---|---|---|---|
| 0 | `DATA` | Batched messages (section 6) | yes | Consumes one window slot and one endpoint credit |
| 1 | `ACK` | `AckBody` | no | Periodic, delayed, POLL reply, credit update |
| 2 | `NAK` | `AckBody` | no | Same body as ACK. Sent immediately on a gap or CRC error. |
| 3 | `RESET_REQ` | `ResetBody` | no | `epoch = 0` |
| 4 | `RESET_ACK` | `ResetBody` | no | `epoch = 0` |
| 5 | `REJECT` | `RejectBody` | no | Explicit refusal of one DATA frame |
| 6–15 | reserved | — | — | Answered with REJECT `UNSUPPORTED` |

`AckBody`:

| Offset | Size | Field | Description |
|---:|---:|---|---|
| 0 | 4 | `sack` | Bit *i* set ⇒ DATA `seq = ack + 1 + i` was received (mod 256) |
| 4 | 1 | `nbad` | Frames discarded for COBS/length/CRC/UART errors since the last valid frame (saturating) |
| 5 | 1 | `n_credits` | Number of credit entries |
| 6 | 2·n | `{u8 endpoint, u8 credit_edge}` | Absolute credit edge per endpoint (5.10) |

`ResetBody`:

| Offset | Size | Field | Description |
|---:|---:|---|---|
| 0 | 4 | `cookie` | The sender's boot (NCP) or open (host) cookie. Random and non-zero. |
| 4 | 4 | `peer_cookie` | The peer cookie being acknowledged (RESET_ACK) or last known (RESET_REQ); 0 = none |
| 8 | 1 | `role` | 0 = host, 1 = NCP |
| 9 | 1 | `link_version` | Highest link version supported (1) |
| 10 | 1 | `rx_window` | Out-of-order frames the sender can buffer, 4..32 (embedded hosts may use 4–8, 3.3) |
| 11 | 1 | `reason` | 0 power-on, 1 peer timeout, 2 protocol error, 3 retry exhausted, 4 host open, 5 peer requested, 6 watchdog, 7 firmware update |
| 12 | 2 | `rx_mtu` | Largest payload accepted; must be 640 in v1 (reserved for later versions) |
| 14 | 2 | `ack_delay_us` | Delayed-ack timer of the sender's receiver |
| 16 | 2 | `keepalive_ms` | Proposed idle keepalive interval |
| 18 | 2 | `link_timeout_ms` | Proposed silence that declares the link down |
| 20 | 1 | `n_credits` | Number of initial credit entries |
| 21 | 2·n | `{u8 endpoint, u8 initial_credit}` | Frames the peer may send on that endpoint from `ep_seq = 0` |

`RejectBody`: `{u8 seq, u8 endpoint, u8 ep_seq, u8 reason}`. Reasons: 1
`NO_CREDIT`, 2 `TOO_LONG`, 3 `UNKNOWN_ENDPOINT`, 4 `UNSUPPORTED`, 5
`WINDOW_VIOLATION`, 6 `STALE_EPOCH`, 7 `NOT_IN_SESSION`.

A REJECT of a DATA frame **consumes** that `seq` and that `ep_seq`. The
receiver marks the seq as received (it appears in `ack`/`sack`), discards the
payload, and advances that endpoint's expected `ep_seq` and its credit
edge past it (a rejected frame holds no buffer, so it costs no credit). The sender
must report every message in the rejected frame to its upper layer with the
local status `LINK_REJECTED` and must not retransmit the frame. The peer is
told explicitly, the counter `tx_rejected` increases, and the link keeps
making progress. Rejection is never silent.

Only the rejections a receiver can record consume the `seq`: `NO_CREDIT`,
`TOO_LONG` and `UNKNOWN_ENDPOINT` (the latter consumes no `ep_seq`, because
the endpoint is unknown). The other reasons do not concern a frame the
receiver recorded:

- `WINDOW_VIOLATION` (R5): the `seq` lies outside the receiver's bitmap, so
  the receiver cannot mark it and its `ack` could never pass it. The sender
  reports the frame's messages with `LINK_REJECTED`, reports `LINK_FAILED`
  and resets the session with reason 2 (protocol error). Only a peer bug
  causes this.
- `STALE_EPOCH` and `UNSUPPORTED` refer to frames of another session or of
  an unknown version/type. They are counted and change no ARQ state.
- `NOT_IN_SESSION` is reserved: a side in `RESETTING` discards non-RESET
  frames silently and counts them (5.5).

### 5.4 Example (from `link_frames.json`)

Host `RESET_REQ` (cookie `0x1A2B3C4D`, window 32, MTU 640, ack delay
1000 µs, keepalive 500 ms, timeout 3000 ms, five credit entries):

```text
decoded: 43 00 00 00 00 00 1f 00 | 4d3c2b1a 00000000 00 01 20 04 8002 e803 f401 b80b 05
         01 04 02 08 03 04 04 04 05 02 | 5c29e1a3
wire:    02 43 01 01 01 01 02 1f 05 4d 3c 2b 1a 01 01 01 01 1b 01 20 04 80 02 e8 03 f4 01
         b8 0b 05 01 04 02 08 03 04 04 04 05 02 5c 29 e1 a3 00
```

`NAK` for a missing `seq` 2 with `seq` 3 and 4 received (epoch 33):

```text
decoded: 42 21 00 00 02 00 06 00 | 03000000 00 00 | 793e4fb1
wire:    03 42 21 01 02 02 02 06 02 03 01 01 01 01 05 79 3e 4f b1 00
```

### 5.5 Session handshake, cookies and epoch

Each side has a `cookie`:

- The NCP draws a new random non-zero 32-bit cookie from the hardware RNG at
  every boot.
- The host draws a new one every time it opens the port, or every time its
  link object is (re)created.

A session exists when both sides agree on the pair `(host_cookie,
ncp_cookie)`. The session epoch is

```text
epoch = 1 + crc32c(le32(host_cookie) || le32(ncp_cookie)) mod 255     (1..255)
```

For the vector cookies (`0x1A2B3C4D`, `0xC0FFEE01`) the epoch is 33.

States are `UNSYNCED`, `RESETTING` and `ESTABLISHED`.

1. **Host start.** The host flushes its OS input buffer, sends `0x00` and
   `RESET_REQ{cookie=H, peer_cookie=last known NCP cookie or 0}`, then
   enters `RESETTING`.
2. **NCP boot.** The NCP sends `RESET_REQ{cookie=N, peer_cookie=0,
   reason=power-on/watchdog/...}` and repeats it every `reset_retry` (200 ms,
   doubling to 2 s) while `UNSYNCED`. On a USB link without a reader these
   writes are non-blocking and are counted as `tx_discarded_no_host` when
   the FIFO is full (section 11).
3. A side that receives `RESET_REQ{cookie=C}`:
   - If `C` differs from the last known peer cookie, the peer has
     **rebooted or reopened**. This is reported to the upper layer as
     `PEER_RESTARTED`.
   - If `C` equals the last known peer cookie, the peer **lost
     synchronization** but did not restart. This is reported as
     `PEER_RESYNC`.
   - In both cases the side resets all ARQ and endpoint state. Every unacked
     message fails locally with `LINK_RESET` (subject to 6.5). The side
     replies `RESET_ACK{cookie=own, peer_cookie=C}` and enters `RESETTING`.
     Frames that were already accepted for delivery before the reset are
     still delivered to the upper layer.
   - A repeated `RESET_REQ{cookie=C}` that arrives while the side is still
     `RESETTING` and has already answered `C` means the answer was lost: the
     side sends `RESET_ACK` again, without another reset or report.
4. A side that receives `RESET_ACK{peer_cookie == own cookie}` enters
   `ESTABLISHED`. If it has not yet answered the peer's request, it sends one
   `RESET_ACK` (with the cookie from the received `RESET_ACK`) so that both
   sides establish. A `RESET_ACK` whose `peer_cookie` does not match is
   ignored and counted (`rx_stale_reset`).
5. Simultaneous `RESET_REQ`s are allowed. Each side answers the other's, and
   both establish when the matching `RESET_ACK` arrives.
6. **Retries and lost `RESET_ACK`s.** While `RESETTING`, a side repeats its
   last handshake frame every `reset_retry` (200 ms doubling to 2 s): the
   `RESET_ACK` if it has answered a request in this handshake, otherwise its
   `RESET_REQ`. A side that is already `ESTABLISHED` and receives a
   duplicate `RESET_ACK` of the current session (same cookies) answers it
   with one `RESET_ACK`, but only while it has not yet received any
   in-session frame from the peer and at most once per `reset_retry`. This
   repairs a lost final `RESET_ACK` without a ping-pong between two
   established sides.

**Negotiated parameters (per direction):**

| Parameter | Value |
|---|---|
| Send window A→B | `B.rx_window` |
| MTU A→B | `B.rx_mtu` |
| Keepalive | `min(A.keepalive_ms, B.keepalive_ms)` |
| Link timeout | `max(A.link_timeout_ms, B.link_timeout_ms, 3 × keepalive)` |
| Initial credit edges | Taken from the peer's credit list |

**Epoch check.** After `ESTABLISHED`, a frame whose epoch differs from the
current epoch is discarded and counted (`rx_epoch_mismatch`). DATA with a
wrong epoch is answered with REJECT `STALE_EPOCH`, at most once per 100 ms.
This removes frames left over from a previous session in OS or USB buffers.
In `RESETTING`, everything except `RESET_*` is discarded and counted
(`rx_not_in_session`).

**After the handshake.** After every `ESTABLISHED` transition the NCP sends
`EVT_NCP_READY` (7.1) as its first message. The host must then send
`SYS_NEGOTIATE` before any other request.

### 5.6 ARQ sender

Sender state:

- `base`: the oldest unacked seq.
- `next_seq`.
- A buffer of up to `W` unacked frames. Each buffered frame carries its
  bytes, `endpoint`, `ep_seq`, `sacked`, `last_tx_time`, `retx_count` and
  `last_tx_order`. `tx_order` is a monotonically increasing counter
  incremented at every (re)transmission.
- `nbad_handled`.
- RTO estimator state.

Rules:

- **S0 (admission).** A new DATA frame may be sent only if **all** of these
  hold:
  - `(next_seq − base) mod 256 < W`;
  - the endpoint has credit, i.e. `(credit_edge[ep] − next_ep_seq[ep]) mod 256`
    is in `1..=128`;
  - the endpoint is within its slot quota: `BULK` + `LOG` together use at
    most `max(1, W/2)` unacked frames.

  Retransmissions do not need credit.
- **S1 (ack/sack processing).** On any frame carrying `ack`, mark every frame
  before `ack` as acked and advance `base`. On ACK/NAK also mark the frames
  set in `sack`. Let `D` be the largest `last_tx_order` among frames newly
  acked or sacked by this frame. Because the stream is FIFO, every unacked,
  unsacked frame with `last_tx_order < D` is lost: retransmit each of them
  now in `tx_order` order, with `RETX` set. Identical or older
  acknowledgements therefore never cause duplicate retransmissions.
- **S2 (CRC-error NAK).** Let `F` be the list of unacked, unsacked frames
  with `last_tx_order > D_max`, in `tx_order` order. `D_max` is the
  largest `last_tx_order` ever acknowledged. When a NAK carries
  `nbad = k > nbad_handled`, retransmit `F[nbad_handled..k]` and set
  `nbad_handled = k`. Whenever `D_max` increases, reset `nbad_handled = 0`.
- **S3 (RTO, tail loss).** If the oldest unacked, unsacked frame satisfies
  `now − last_tx_time ≥ RTO`, retransmit it with `RETX | POLL`, double `RTO`
  (capped at `RTO_max`), and do not take RTT samples from retransmitted
  frames (Karn). After an RTT sample, recompute
  `RTO = clamp(SRTT + 4·RTTVAR, RTO_min, RTO_max)` as in RFC 6298.
- **S4 (failure).** When a frame that was already retransmitted
  `N_retx_max` (10) times needs another retransmission, the link is
  declared failed. The sender reports `LINK_FAILED` and starts a reset with
  reason 3.
- **S5 (blocked sender).** A sender that is blocked on credit or window for
  more than `keepalive_ms` sends `ACK | POLL` to elicit a fresh credit and
  ack state.

Retransmitted frames keep their `seq`, `ep_seq` and payload byte-for-byte.
Only `ctrl.RETX`, `ctrl.POLL`, `ack` and the CRC may change, so
retransmission is idempotent.

### 5.7 ARQ receiver

Receiver state:

- `ack`: the next expected seq.
- A received-bitmap for `ack+1 .. ack+32`.
- A reorder buffer of `rx_window` frames.
- Per-endpoint `expected_ep_seq` and `credit_edge`.
- `nbad`.
- The delayed-ack timer.

On a valid DATA frame with seq `s` (`d = (s − ack) mod 256`):

- **R1 (in order, `d = 0`, no later frames buffered).** Accept the frame.
  Advance `ack`. Start the delayed-ack timer if it is not running. If
  `frames_since_last_ack ≥ max(1, rx_window/2)`, send ACK now.
- **R2 (ahead, `0 < d < rx_window`, not yet received).** Buffer the frame
  and set its sack bit. Send NAK immediately.
- **R3 (hole filled, `d = 0` while later frames are buffered).** Advance
  `ack` over every contiguous received frame and send ACK immediately.
- **R4 (duplicate).** Either `d ≥ 256 − 32` (behind the window) or the sack
  bit is already set. Discard the frame, increment `rx_duplicate`, and send
  ACK immediately. Duplicate delivery to an endpoint is impossible.
- **R5 (outside the window).** `rx_window ≤ d < 256 − 32`: discard the
  frame, send REJECT `WINDOW_VIOLATION` and count it.
- **R6 (error).** On a COBS/length/CRC/UART error, increment `nbad` and send
  NAK immediately. CRC-triggered NAKs are rate-limited to one per
  `ack_delay`; when the hold-off ends, one more NAK is sent if `nbad`
  differs from the value carried by the last ACK or NAK sent. `nbad` resets to 0 on the next valid frame.
- **R7 (POLL).** Any frame with `POLL` gets an ACK immediately after it is
  processed.
- **R8 (credits).** When a consumer frees a buffer, advance that endpoint's
  `credit_edge` by 1. If the peer is blocked on that endpoint (the last
  received `ep_seq == credit_edge_old − 1`), send ACK immediately. Otherwise
  the new edge rides on the next ACK.
- **R9 (timer).** When the delayed-ack timer expires, send ACK. Sending any
  ACK or NAK cancels the timer.

**Endpoint delivery.**

- A frame of endpoint `e` is delivered as soon as its `ep_seq ==
  expected_ep_seq[e]`, even if earlier link seqs of other endpoints are
  missing. Buffered frames of `e` with consecutive `ep_seq`s follow
  immediately.
- This is why ZRHP has no head-of-line blocking between endpoints. A lost
  `BULK` frame occupies one window slot but never delays `ZIGBEE` delivery.
- Delivery atomically consumes one endpoint credit.
- A frame that would exceed the advertised credit is answered with REJECT
  `NO_CREDIT`. This can only be caused by a peer bug.

**Credit re-advertisement.** Every ACK sent by keepalive (5.11) carries the
credit edges of all endpoints. Other ACKs carry only the edges that changed
since the last ACK. A lost credit update is therefore repaired within one
keepalive.

### 5.8 Timers and defaults

`T_byte = 10 / baud` for UART. `T_frame_max = 656 · T_byte`. `T_host` is the
host's scheduling slack. Its default is 10 ms, and it should be raised to
30 ms for Python on busy hosts.

| Timer / constant | Formula | 460 800 (BR board) | 921 600 | 2 M | 3 M | USB-Serial-JTAG | Range |
|---|---|---:|---:|---:|---:|---:|---|
| `T_frame_max` | 656·T_byte | 14.2 ms | 7.1 ms | 3.3 ms | 2.2 ms | ≤ 11 ms (11 × 64 B packets at 1 packet per 1 ms frame, worst case) | — |
| `ack_delay` | ≤ T_frame_max/2, ≤ 2 ms | 2 ms | 1 ms | 1 ms | 0.5 ms | 1 ms | 0.2–5 ms |
| `RTO_min` | max(10 ms, 2·T_frame_max + ack_delay + T_host) | 41 ms | 25 ms | 18 ms | 16 ms | 33 ms | — |
| `RTO_initial` | — | 100 ms | 100 ms | 100 ms | 100 ms | 100 ms | 20–1000 ms |
| `RTO_max` | — | 1000 ms | 1000 ms | 1000 ms | 1000 ms | 1000 ms | — |
| `N_retx_max` | — | 10 | 10 | 10 | 10 | 10 | 4–16 |
| `keepalive_ms` | idle time before `ACK\|POLL` | 500 | 500 | 500 | 500 | 500 | 100–5000 |
| `link_timeout_ms` | silence ⇒ link down | 3000 | 3000 | 3000 | 3000 | 3000 | ≥ 3·keepalive |
| `reset_retry` | doubling | 200 ms → 2 s | ← | ← | ← | ← | — |
| Window `W` (each direction) | min(peer `rx_window`, local TX buffers) | 16 (NCP rx), 8 (S3 host rx) | 16 (NCP rx), 32 (desktop host rx) | ← | ← | ← | 4–32 (8–32 recommended) |

Every timer runs on a real monotonic clock: the embassy time driver on the
NCP, a monotonic OS clock or `esp_timer` on hosts. A platform without one
cannot run ZRHP.

### 5.9 Normative traces

`arq_scenarios.json` contains receiver and sender traces with explicit
microsecond timestamps: in-order delivery with a delayed ACK, a gap and its
fill, sender reaction to the gap, a CRC error with `nbad`, sender reaction to
`nbad`, tail loss recovered by RTO, a duplicate after a lost ACK, and credit
unblocking. Every implementation must drive its state machine with a virtual
clock and emit exactly the listed frames (type, `ack`, `sack`, `nbad`,
retransmitted seq, flags) at exactly the listed times. In the sender traces,
`final.unacked` lists the frames acknowledged neither cumulatively nor by
SACK. Section 14 lists the remaining conformance cases.

### 5.10 Endpoints, priorities and credits

| ID | Name | Carries | Scheduler priority | Initial credit granted by NCP / by host |
|---:|---|---|---|---|
| 0 | `LINK` | Never used for DATA | — | — |
| 1 | `SYSTEM` | Session, negotiation, config, reset, diagnostics, identity | 1 (highest) | 4 / 4 |
| 2 | `ZIGBEE` | Network, data, ZDO, security, GP commands and events | 3 | 8 / 8 |
| 3 | `TABLES` | Hybrid table paging (section 8) | 2 | 4 / 4 |
| 4 | `BULK` | Backup export/restore, NCP firmware update | 4 | 2 / 4 |
| 5 | `LOG` | Log records, packet capture (reserved) | 5 (lowest) | 0 / 2 |
| 6–31 | reserved | — | — | — |
| 32–63 | experimental / vendor | — | — | — |

`TABLES` ranks above `ZIGBEE` because a table miss can block an inbound
secured frame or a join (section 8).

Credit numbers in the table are defaults. A host advertises what its buffers
allow. An embedded host can grant `LOG` 0 credits to disable log delivery;
the NCP then counts the records it drops. A host can grant `BULK` 1 credit
when it has no firmware-update or backup traffic. The allocation of endpoint
IDs and message-ID groups is final for v1 (decision 1).

**Scheduler.**

- Retransmissions are sent first.
- Otherwise the scheduler picks the highest-priority endpoint that has queued
  messages, credit and a slot.
- Anti-starvation: after 8 consecutive frames from higher priorities while a
  lower-priority endpoint is ready, one frame of the longest-waiting ready
  endpoint is sent.

**Batching is opportunistic.**

- When the transmitter becomes free, it packs every queued message of the
  chosen endpoint that fits into the peer MTU into one frame.
- It never waits to collect more (no Nagle delay).
- Under light load every message goes out alone with minimum latency. Under
  load, frames fill up automatically.

**What a credit means.** A credit is one receive buffer of up to `rx_mtu`
bytes on the peer for that endpoint. Credits limit *frames*, not messages.
Per-command capacity, such as the APS request queue, is signalled with the
message-level status `BUSY` and advertised in advance as a capacity
(`max_aps_inflight`, 6.6). This lets hosts size their semaphores so that
`BUSY` is rare.

### 5.11 Keepalive, link failure and recovery

- If nothing has been sent for `keepalive_ms`, send `ACK | POLL` (with all
  credit edges). The peer answers immediately (R7).
- If no valid frame has been received for `link_timeout_ms`, the link is
  **down**:
  1. Report `LINK_DOWN` upward.
  2. Fail all pending link deliveries with `LINK_RESET`.
  3. Start the reset procedure with reason 1, keeping your own cookie.
- On the NCP, link-down does **not** stop the Zigbee network. The NCP keeps
  routing, the Trust Center keeps answering joins under its configured
  policy, and inbound indications go into a bounded host-absent queue
  (default 32 messages). Overflow is counted and reported once with
  `EVT_INDICATIONS_DROPPED` after the next session.
- On USB-Serial-JTAG with no reader, the NCP must never block. Section 11
  covers this.
- **Which cookie identifies the NCP.** For resume decisions, hosts identify
  the NCP instance by `EVT_NCP_READY.ncp_cookie`, `table_epoch` and the NIB,
  not by link timing. Section 12.1 describes seamless resume.

### 5.12 Link counters

Both sides keep `u32` saturating counters. The host can read the NCP's set
with `DIAG_GET_COUNTERS` (group `0x01`).

| ID | Counter | ID | Counter |
|---:|---|---:|---|
| 0x0101 | `tx_frames` | 0x0111 | `rx_frames` |
| 0x0102 | `tx_bytes` | 0x0112 | `rx_bytes` |
| 0x0103 | `tx_retransmissions` | 0x0113 | `rx_bad_frame` (COBS/len/CRC/UART) |
| 0x0104 | `tx_rto_expiries` | 0x0114 | `rx_duplicate` |
| 0x0105 | `tx_rejected` | 0x0115 | `rx_out_of_order` |
| 0x0106 | `tx_blocked_credit_ms` | 0x0116 | `rx_epoch_mismatch` |
| 0x0107 | `tx_blocked_window_ms` | 0x0117 | `rx_not_in_session` |
| 0x0108 | `tx_discarded_no_host` | 0x0118 | `rx_rejected_sent` |
| 0x0109 | `naks_sent` | 0x0119 | `rx_overflow` (transport FIFO/ring) |
| 0x010A | `naks_received` | 0x011A | `rx_stale_reset` |
| 0x010B | `sessions_established` | 0x011B | `peer_restarted` |
| 0x010C | `link_down_events` | 0x011C | `peer_resync` |

### 5.13 Behaviour under faults

| Fault | Detection | Recovery | Visible effect |
|---|---|---|---|
| Bit errors in a frame | CRC-32C (or COBS/length) | R6 NAK with `nbad`, then S2 retransmit | ~1 frame time + RTT latency |
| Noise inserts `0x00` (frame split) | Both halves fail length/CRC | Same as above | Same |
| Delimiter destroyed (frames merged) | Length/CRC mismatch, or >656 bytes | Both frames NAKed via `nbad`, or a later gap triggers S1 | Two retransmissions |
| Lost frame mid-stream | Gap at the next frame (R2) | S1 FIFO inference | ~1 RTT |
| Lost last frame | No following frame | S3 RTO with POLL | `RTO_min`..`RTO` |
| Lost ACK/NAK | Next ACK/NAK carries the full state; RTO+POLL | S1/S3 | Possible duplicate, suppressed by R4 |
| Duplicated frame | sack bitmap / behind window | R4, ACK | None |
| Reordering | Impossible on a byte stream; would look like loss + duplicate | S1 + R4 | Retransmission |
| Stall (host not reading, RTS deasserted) | Window/credit full; keepalive silence | Sender blocks without dropping; link down after `link_timeout_ms`; reset | Explicit `LINK_DOWN`; NCP indications queued, then counted |
| Peer reboot | New cookie in RESET_REQ | Handshake; pending messages fail `LINK_RESET` | `PEER_RESTARTED` |
| Stale bytes from the old session | Epoch mismatch | Discard and count | None |
| Babbling peer (garbage storm) | `rx_bad_frame` rate | NAK rate limit; link down if no valid frame | `LINK_DOWN` |

### 5.14 Undetected-error analysis

CRC-32C has Hamming distance 6 for data words up to 5 243 bits and HD 4 up
to 2³¹ bits [Koopman]. The polynomial contains the factor `(x+1)`, so every
odd-weight error is detected. The maximum CRC-covered length is 648 bytes =
5 184 bits ≤ 5 243, so **every error pattern of up to 5 bit flips, and every
burst of up to 32 bits, in any ZRHP frame is detected.** The maximum payload
of 640 bytes was chosen for this reason.

For random independent bit errors at BER `p` on a 5 248-bit wire frame, an
undetected error needs at least 6 flips, so `P_ud ≲ P(≥6 errors) · 2⁻³²`:

| BER | P(frame corrupted) | P(≥ 6 bit errors) | P_ud per frame (upper bound) |
|---|---:|---:|---:|
| 1e-7 | 5.2e-4 | 2.9e-23 | 6.8e-33 |
| 1e-6 | 5.2e-3 | 2.9e-17 | 6.8e-27 |
| 1e-5 | 5.1e-2 | 2.8e-11 | 6.5e-21 |
| 1e-4 | 0.41 | 1.9e-5 | 4.3e-15 |

For **burst garbage** (a frame replaced by arbitrary bytes), the CRC alone
gives 2⁻³² ≈ 2.3e-10 per corrupted frame:

- At 1 corrupted frame/s that is about one false accept per 136 years.
- If every frame at 3 Mbaud full load (457 frames/s) were garbage, it would
  be about one every 0.3 years.

ZRHP adds independent structural checks before a frame is accepted: the
decoded length must equal `len + 12` (≈ 2⁻¹⁶ for random data), the epoch
must match (2⁻⁸), and version and type must be valid (≈ 2⁻³·⁴). Together
they bring the garbage false-accept probability to about 2⁻⁵⁹ ≈ 1.3e-18 per
frame. That is roughly one event per 5·10⁷ years of continuous full-rate
garbage at 3 Mbaud. This is an estimate that assumes the checks are
independent for random garbage. It does not cover structured bit errors,
which the HD=6 bound above handles.

A false accept is further limited by the message layer: lengths must parse
exactly, IDs must be known and TIDs must match. A link that receives no valid
frame for `link_timeout_ms` is reset.

---

## 6. Message layer

### 6.1 Message header

A DATA payload is a concatenation of one or more messages. Messages never
span frames.

| Offset | Size | Field | Description |
|---:|---:|---|---|
| 0 | 1 | `flags` | bits 7–6 `kind`: 0 request, 1 response, 2 event, 3 reserved. Bit 5 `RETRY`. Bits 4–0 reserved (0). |
| 1 | 2 | `msg_id` | Command or event ID (section 7) |
| 3 | 2 | `tid` | Request/response: transaction ID. Event: per-endpoint event sequence. |
| 5 | 2 | `len` | Payload length |
| 7 | `len` | `payload` | Command-specific |

The largest message is `640 − 7 = 633` payload bytes. Larger objects
(backups, firmware, table dumps) are chunked by their commands.

**Parsing is atomic per frame.** The receiver first validates that the
message lengths tile the payload exactly. If they do not, the whole frame's
messages are dropped, `msg_bad_batch` is counted and `EVT_PROTOCOL_ERROR` is
emitted. Because the frame passed CRC, this indicates a peer bug, not noise.
Only then are the messages dispatched in order.

### 6.2 Kinds and TID rules

- **Request.** Can originate from either side. The host sends most commands.
  The NCP sends `TBL_LOOKUP` and `TBL_WRITEBACK` (section 8). Each originator
  allocates TIDs from its own 16-bit counter, skipping 0. The two directions
  have independent TID namespaces.
- **Response.** Same `msg_id` and `tid` as the request, on the same
  endpoint, from the other side. The first payload byte is always `status`
  (6.4). Exactly one response is sent for every request. Silence is never
  valid: unknown IDs get `UNSUPPORTED_COMMAND`, malformed payloads get
  `INVALID_PARAMETER`, and requests on the wrong endpoint get
  `WRONG_ENDPOINT`.
- **Event.** Unsolicited. `tid` is a per-endpoint, per-session event
  sequence starting at 1, which helps with ordering diagnostics. Events are
  never acknowledged at the message level, because the link is reliable.
- **TID reuse.** A TID may be reused only after its response arrived, or
  after its host timeout plus `2 × link_timeout_ms`. A wrapping counter with
  at most a few hundred requests in flight satisfies this naturally.
- **Two-phase commands.** Some commands (`APS_DATA_REQUEST`, energy scan,
  formation) respond quickly with `SUCCESS` ("accepted") or a refusal. The
  final outcome is a later event that carries the original request TID in
  its payload.

### 6.3 Batching

A sender batches several messages of the **same endpoint** into one frame
(5.10). Responses, events and requests may be mixed in one frame as long as
they all originate from the sending side. Batching never reorders messages
within an endpoint.

### 6.4 Status codes (`u8`)

| Code | Name | Meaning |
|---:|---|---|
| 0x00 | `SUCCESS` | Done, or accepted for a two-phase command |
| 0x01 | `FAILURE` | Generic failure; details in a command-specific field or counter |
| 0x02 | `BUSY` | A queue or resource is full; nothing was done; retry later |
| 0x03 | `INVALID_PARAMETER` | Malformed or out-of-range payload |
| 0x04 | `UNSUPPORTED_COMMAND` | Unknown `msg_id`, or capability not present |
| 0x05 | `INVALID_STATE` | Not allowed now (not negotiated, network down, restore in progress) |
| 0x06 | `NOT_FOUND` | Unknown entry |
| 0x07 | `NO_SPACE` | Table full |
| 0x08 | `TIMEOUT` | An internal deadline expired (for example a host table lookup) |
| 0x09 | `LINK_RESET` | **Host-local only:** the session reset before the response arrived |
| 0x0A | `ACCESS_DENIED` | Policy forbids it (key export locked, unsigned firmware) |
| 0x0B | `NOT_FORMED` | No network in NV |
| 0x0C | `ALREADY_EXISTS` | Duplicate entry |
| 0x0D | `STORAGE_ERROR` | NV write or commit failed; state unchanged |
| 0x0E | `SECURITY_ERROR` | Key, counter or authentication rule violated |
| 0x0F | `VERSION_MISMATCH` | No common protocol major version |
| 0x10 | `IN_PROGRESS` | Another exclusive operation of the same kind is running |
| 0x11 | `TOO_LARGE` | Payload exceeds the NCP limit (for example the ASDU) |
| 0x12 | `WRONG_ENDPOINT` | Command sent on the wrong link endpoint |
| 0x13 | `CONFLICT` | Host push conflicts with a newer dirty NCP entry (8.3) |
| 0x14 | `ABORTED` | Cancelled by reset, leave or restore |
| 0x15 | `LINK_REJECTED` | **Host-local only:** the carrying frame was REJECTed (5.3) |

Zigbee MAC/NWK/APS/ZDO outcomes are carried separately as `zb_status`
(`u8`), using the raw Zigbee status values (for example `0x00` SUCCESS,
`0xA7` APS NO_ACK, `0xE9` MAC NO_ACK, `0xD0` NWK ROUTE_ERROR).

### 6.5 Retry and idempotency

A request that failed locally with `LINK_RESET` has an unknown outcome.
After a **`PEER_RESYNC`** (same cookies) the host MAY resend it with the
same `tid` and the `RETRY` flag. The NCP keeps a result cache of the last
64 completed host requests for the current cookie pair, holding `(tid,
msg_id, status, ≤ 16 bytes of result)`. A `RETRY` request whose TID is in the
cache returns the cached response without executing again. A `RETRY` request
whose TID is unknown is executed normally.

After **`PEER_RESTARTED`** the cache is gone, and the host must treat
unconfirmed non-idempotent operations (for example `APS_DATA_REQUEST`) as
"delivery unknown". zigpy surfaces this as a `DeliveryError`.

### 6.6 Versioning and capabilities

- `SYS_NEGOTIATE` selects one **major** version. Each side states the
  `[major_min, major_max]` range it supports. A different major may change
  any layout.
- **Minor** versions are additive. New commands and events get new IDs, and
  new fields are appended to the end of payloads. Receivers ignore trailing
  bytes they do not understand and treat missing trailing fields as their
  documented default.
- Fields are never reordered or reinterpreted within a major version.
- The layouts of `RESET_*`, `SYS_NEGOTIATE` and `EVT_NCP_READY` are frozen
  across all majors, so that any host can discover any NCP.

Capability bits (`u64`, NCP → host):

| Bit | Capability | Bit | Capability |
|---:|---|---:|---|
| 0 | `HOST_SOURCE_ROUTES` (APS request may carry relays) | 10 | `NETWORK_KEY_ROTATION` |
| 1 | `CONCENTRATOR` (MTORR origination) | 11 | reserved (was `ZDO_LOOPBACK`; dropped from v1, never set) |
| 2 | `APS_FRAGMENTATION` | 12 | `PACKET_CAPTURE` (reserved) |
| 3 | `INSTALL_CODES` | 13 | `LINK_AUTH` (reserved, section 13) |
| 4 | `KEY_EXPORT` (network/link keys readable) | 14 | `EZSP_HASHED_TCLK` |
| 5 | `IEEE_REWRITABLE` (repeatable IEEE override) | 15 | `BACKUP_RESTORE` |
| 6 | `HOST_BACKED_TABLES` | 16 | `MANUFACTURER_CODE` |
| 7 | `NCP_FW_UPDATE` | 17 | `GROUP_MEMBERSHIP` |
| 8 | `GREEN_POWER_PROXY` | 18 | `TX_POWER` |
| 9 | `ENERGY_SCAN` | 19 | `INTERPAN` (reserved) |
| 20 | `GREEN_POWER_SINK` (combo sink, 7.11) | 21 | `PTA_COEX` (external 802.15.4/Wi-Fi coexistence wired and supported, 11.5) |
| 22 | `UART_FLOW_CONTROL` (RTS/CTS active on this link) | 23–63 | reserved |

Capacities (`{u8 id, u32 value}` list):

| ID | Capacity | ID | Capacity |
|---:|---|---:|---|
| 1 | `max_aps_inflight` | 8 | `max_host_endpoints` |
| 2 | `max_children` | 9 | `group_table` |
| 3 | `link_key_cache` | 10 | `binding_table` |
| 4 | `address_map_cache` | 11 | `neighbor_table` |
| 5 | `source_route_cache` | 12 | `route_table` |
| 6 | `max_asdu` (unfragmented) | 13 | `max_restore_records_per_put` |
| 7 | `fc_stride` (reservation size) | 14 | `max_source_route_relays` |
| 15 | `gp_sink_table` | 16 | `gp_proxy_table` |
| 17 | `max_ncp_requests` (outstanding NCP → host requests, default 8) | 18 | `max_tc_devices` (TC-device cache incl. pinned) |

### 6.7 Endpoint assignment

Each `msg_id` group has one home endpoint. The NCP rejects requests on any
other endpoint with `WRONG_ENDPOINT`. **This allocation is final for v1**
(decision 1).

| Group | IDs | Endpoint |
|---|---|---|
| SYSTEM | `0x00xx` | 1 `SYSTEM` |
| NETWORK | `0x01xx` | 2 `ZIGBEE` |
| DATA | `0x02xx` | 2 `ZIGBEE` |
| ZDO | `0x03xx` | 2 `ZIGBEE` |
| SECURITY | `0x04xx` | 2 `ZIGBEE` |
| TABLES | `0x05xx` | 3 `TABLES` |
| BACKUP | `0x06xx` | 4 `BULK` |
| DIAG | `0x07xx` | 1 `SYSTEM` |
| FIRMWARE | `0x08xx` | 4 `BULK` |
| LOG | `0x09xx` | 5 `LOG` |
| GREEN POWER | `0x0Axx` | 2 `ZIGBEE` |
| Experimental | `0xF0xx`–`0xFFxx` | any ≥ 32 |

Within a group, `0x00`–`0x7F` in the low byte are requests and `0x80`–`0xFF`
are events.

---

## 7. Command set v1

Runtime mapping legend:

- **E**: an existing zigbee-rs API implements the behaviour (mostly
  host-tested).
- **P**: a primitive exists, but the runtime or app needs new exposure or an
  extension.
- **N**: new work.

Request payloads are listed first. Responses always start with `status`; any
fields after it are shown as "→ …". Events list their full payload.

### 7.1 SYSTEM (`0x00xx`, endpoint 1)

| ID | Name | Request payload | Response after `status` | Map |
|---|---|---|---|---|
| `0x0001` | `SYS_NEGOTIATE` | `u8 major_min, u8 major_max, u64 host_caps, u8 name_len, name[]` | `u8 major, u8 minor, u64 ncp_caps, u8 n, n×{u8 id, u32 value}, u8 fw_len, fw[], [8] build_id, u8 chip` | N |
| `0x0002` | `SYS_GET_INFO` | — | `[8] ieee_factory, [8] ieee_active, u8 ieee_source (0 factory, 1 override), u16 chip_rev, u32 flash_size, u8 transport (1 USJ, 2 UART), u32 uart_baud (0 for USJ), u8 uart_flow (0 none, 1 RTS/CTS), u8 board_len, board[], u8 product_len, product[]` | N |
| `0x0003` | `SYS_RESET` | `u8 mode (0 reboot, 1 reboot to bootloader, 2 factory reset), u32 confirm` (`0x46524553` "FRES" for mode 2) | — (the response is sent before reset) | P (factory reset of stores exists) |
| `0x0004` | `SYS_GET_STATUS` | — | `u8 network_state, u32 uptime_s, u16 free_aps_slots, u16 host_absent_queue, u16 dirty_writebacks, u8 restore_in_progress, u8 flags (bit0 key export locked, bit1 paging active, bit2 PTA coex enabled, bit3 GP sink commissioning open, bit4 permit join open)` | N |
| `0x0005` | `SYS_SET_CONFIG` | `u8 n, n×{u16 id, u8 len, value[]}` | `u8 n, n×{u16 id, u8 status}` | N |
| `0x0006` | `SYS_GET_CONFIG` | `u8 n, n×u16 id` | `u8 n, n×{u16 id, u8 status, u8 len, value[]}` | N |
| `0x0007` | `SYS_ECHO` | `data[]` (≤ 64) | `data[]` | N |
| `0x0008` | `SYS_SET_IEEE` | `[8] ieee` (all `FF` = revert to factory) | — | E (`products/esp32-zigbee-ncp` identity item applied at boot; the NCP also sets the MAC PIB and NIB at once while no network runs) |
| `0x0009` | `SYS_SET_UART` | `u32 baud, u8 flow (0 none, 1 RTS/CTS), u16 apply_delay_ms` | — (sent at the old rate; see the rule below) | N |
| `0x0080` | `EVT_NCP_READY` | `u32 ncp_cookie, u8 reset_reason, u8 network_state (0 none, 1 formed-down, 2 up), u32 table_epoch, u32 nwk_fc_limit, u32 aps_fc_limit, u16 dirty_writebacks, u32 dropped_indications` | event | N |
| `0x0081` | `EVT_INDICATIONS_DROPPED` | `u8 endpoint, u8 reason (1 host absent, 2 no credit, 3 queue full), u32 count, u32 first_ms, u32 last_ms` | event | N |
| `0x0082` | `EVT_FATAL` | `u8 kind, u32 code, u8 len, text[]` (sent just before a deliberate reset) | event | N |
| `0x0083` | `EVT_PROTOCOL_ERROR` | `u8 endpoint, u16 msg_id, u8 reason` | event | N |

Config IDs (v1):

| ID | Setting |
|---|---|
| `0x0001` | `host_absent_queue_len` |
| `0x0002` | `tbl_lookup_timeout_ms` (default 250) |
| `0x0003` | `tbl_miss_queue_len` (default 8) |
| `0x0004` | `zdo_forward_handled` (default 1) |
| `0x0005` | `aps_ack_timeout_ms` |
| `0x0006` | `indirect_timeout_ms` |
| `0x0007` | reserved (key-export locking is the command `SEC_LOCK_KEY_EXPORT`, 7.5) |
| `0x0008` | `log_level` |
| `0x0009` | `coex_enable` (0/1; volatile, default 0; only when `PTA_COEX`; the embedded hub drives it from its Wi-Fi state, 11.5) |
| `0x000A` | `coex_tx_priority_mask` (bit0 MAC ACK frames, bit1 GP timed TX, bit2 TC/key transport, bit3 all) |

`reset_reason` values: 0 unknown, 1 power-on, 2 software, 3 watchdog, 4
brown-out, 5 firmware update, 6 fatal, 7 host request, 8 host EN/BOOT line
(embedded host).

**`SYS_SET_UART` rule.**

1. The NCP answers `SUCCESS` at the old rate.
2. Both sides switch after `apply_delay_ms`.
3. The host then sends `RESET_REQ` at the new rate. The link session is
   re-established; the NCP cookie is unchanged, so this is a `PEER_RESYNC`.
4. If the NCP receives no valid frame within 3 s at the new rate, it reverts
   to the previous rate and flow-control setting. A host that gets no
   `RESET_ACK` within 2 s also reverts.

The setting is volatile: the product default is restored on reboot.
Enabling `flow = 1` is refused with `UNSUPPORTED_COMMAND` unless the board
has RTS/CTS wired (`UART_FLOW_CONTROL` capability semantics, 11.5).

`chip` values: 1 ESP32-H2, 2 ESP32-C6, 3 nRF52840, 4 EFR32MG21, 5 EFR32MG22,
6 TLSR8258, 7 BL702, 8 STM32WB55, 9 CC2340R5.

### 7.2 NETWORK (`0x01xx`, endpoint 2)

| ID | Name | Request payload | Response after `status` | Map |
|---|---|---|---|---|
| `0x0101` | `NWK_INIT` | `u32 nwk_fc_floor, u32 aps_fc_floor` (host checkpoint floors, 0 = none) | `u8 network_state`; `NOT_FORMED` if NV has no network. Completes with `EVT_NETWORK_STATE`. | E (`CoordinatorApp`/`TrustCenterCoordinatorApp::initialize` persisted path) + N (floors) |
| `0x0102` | `NWK_FORM` | `u32 channel_mask, u16 pan_id (FFFF = random), [8] ext_pan_id (0 = random), [16] network_key (0 = random), u8 key_seq, u8 nwk_update_id, i8 tx_power, u16 nwk_manager` | accepted; `EVT_NETWORK_STATE` follows | E (BDB formation; host-supplied parameters install the network directly on the quietest channel of the mask) |
| `0x0103` | `NWK_LEAVE_RESET` | `u8 flags (bit0 keep IEEE override, bit1 keep host-endpoint registrations)` | — | E (`urgent_factory_reset_and_recommission`; a new network can follow in the same session) |
| `0x0104` | `NWK_GET_INFO` | — | `u8 state, u16 pan_id, [8] ext_pan_id, u8 channel, u32 channel_mask, u16 short, [8] ieee, u8 nwk_update_id, u16 nwk_manager, u8 key_seq, u8 security_level, i8 tx_power, u8 stack_profile` | P |
| `0x0105` | `NWK_PERMIT_JOIN` | `u8 duration_s, u16 target (0xFFFC all routers + NCP, 0x0000 NCP only, else a router), u8 flags (bit0 TC significance)` | — | E (`nlme_permit_joining`, `mgmt_permit_joining_req`) |
| `0x0106` | `NWK_ENERGY_SCAN` | `u32 channel_mask, u8 duration_exp, u8 count` | accepted; `EVT_ENERGY_SCAN_RESULT` follows | P (`nlme_ed_scan` exists; running it on an active coordinator must pause TX/RX and is N) |
| `0x0107` | `NWK_CHANGE_CHANNEL` | `u8 channel, u8 nwk_update_id (0 = current+1)` | accepted; `EVT_CHANNEL_CHANGED` follows | P (`nlme_set_channel` and Mgmt_NWK_Update codec exist; the broadcast plus delayed local switch plus persistence is N) |
| `0x0108` | `NWK_SET_TX_POWER` | `i8 dbm` | `i8 applied_dbm` | E (MAC PIB `PhyTransmitPower`) |
| `0x0109` | `NWK_CONCENTRATOR` | `u8 mode (0 off, 1 low-RAM, 2 high-RAM), u16 min_s, u16 max_s, u8 radius` | — | P (NWK concentrator state exists) |
| `0x010A` | `NWK_ACTIVE_SCAN` | `u32 channel_mask, u8 duration_exp` | `u8 n, n×{u8 channel, u16 pan, [8] epid, u16 src, u8 lqi, i8 rssi, u8 permit, u8 stack_profile, u8 update_id}` | P (`nlme_network_discovery`) |
| `0x010B` | `NWK_ADD_ENDPOINT` | `u8 ep, u16 profile, u16 device_id, u8 version, u8 n_in, n_in×u16, u8 n_out, n_out×u16` | — (`NO_SPACE` beyond `max_host_endpoints`; `INVALID_PARAMETER` for `ep` outside 1..=240 or more than 16 clusters in a list) | E (`ZigbeeDevice::register_host_endpoint`: the ZDO registry the server answers from; `max_host_endpoints` = 8; registering an endpoint again replaces it, including the product profile's endpoint with that number; registrations are RAM only and cleared by `NWK_LEAVE_RESET` without flags bit1) |
| `0x010C` | `NWK_SET_MANUFACTURER_CODE` | `u16 code` | — | P (node descriptor field) |
| `0x010D` | `NWK_GROUP_MEMBERSHIP` | `u8 op (1 add, 2 remove, 3 clear), u16 group, u8 ep` | — | E (APS group table, 16 groups × 8 endpoints) |
| `0x010E` | `NWK_REMOVE_DEVICE` | `u16 nwk, [8] ieee, u8 flags (bit0 rejoin, bit1 remove children, bit2 TC remove via parent)` | — | E (TC Remove-Device/leave journal) + P |
| `0x0180` | `EVT_NETWORK_STATE` | `u8 state (0 down, 1 forming, 2 up, 3 leaving), u8 reason, u8 zb_status, u16 req_tid` | event | N |
| `0x0181` | `EVT_DEVICE_JOINED` | `u16 nwk, [8] ieee, u16 parent_nwk, u8 join_type (0 association, 1 secured rejoin, 2 unsecured/TC rejoin, 3 direct), u8 capability, u8 decision (0 admitted, 1 denied)` | event | E (`TrustCenterHostEvent::Joined`: one event per Trust Center Update-Device join decision, through any parent including the NCP itself; `capability` is reconstructed from the neighbour entry for a child of the NCP and 0 for a device that joined through another parent; `parent_nwk` `0xFFFF` when unknown. `Device_annce` is forwarded as an indication, not as this event) |
| `0x0182` | `EVT_DEVICE_AUTHORIZED` | `u16 nwk, [8] ieee, u8 result (0 key delivered, 1 TCLK verified, 2 failed), u8 link_key_type` | event | E (`TrustCenterHostEvent::Authorized`: result 0 when the device first uses the network key (its Request-Key) while no TCLK exchange already closed it, 1/2 at the Verify-Key decision; `link_key_type` 0 global default TCLK, 1 unique TCLK) |
| `0x0183` | `EVT_DEVICE_LEFT` | `u16 nwk, [8] ieee, u8 flags (bit0 rejoin, bit1 removed by TC, bit2 reported by parent), u8 reason (0 unknown, 1 NWK Leave heard by the NCP, 2 parent's Update-Device "device left", 3 Trust Center removal, 4 child of the NCP aged out)` | event | E (NWK Leave indications of neighbours with the rejoin bit and aged-out children: `NwkLayer::take_host_departure`; parent reports and removals: `TrustCenterHostEvent`; `ieee` all zero when unknown. A child of the NCP is reported once, from its Leave) |
| `0x0184` | `EVT_ADDRESS_CHANGED` | `[8] ieee, u16 old_nwk, u16 new_nwk, u8 cause (0 announce, 1 conflict)` | event | P |
| `0x0185` | `EVT_ROUTE_RECORD` | `u16 src_nwk, [8] src_ieee (0 if unknown), u8 n, n×u16 relays` (Route Record order: entry 0 is nearest the originator) | event | P (route-record receive exists) |
| `0x0186` | `EVT_NWK_STATUS` | `u16 dst, u8 nwk_status, u16 reporter` | event | P |
| `0x0187` | `EVT_PAN_ID_CONFLICT` | `u16 conflicting_pan, u8 count` | event | P (`conflict.rs`) |
| `0x0188` | `EVT_PERMIT_JOIN_STATE` | `u8 remaining_s` | event | N |
| `0x0189` | `EVT_CHANNEL_CHANGED` | `u8 channel, u8 nwk_update_id, u16 req_tid` | event | N |
| `0x018A` | `EVT_ENERGY_SCAN_RESULT` | `u16 req_tid, u8 status, u8 n, n×{u8 channel, i8 max_rssi_dbm, u8 energy}` | event | N |

**RF front-end modules (external PA/LNA).** Every power and level on this
interface is **antenna-referred**, with or without a FEM, so no capability
bit is needed and a host never applies FEM gains itself. `NWK_SET_TX_POWER`
`dbm` (and `NWK_FORM`/restore `tx_power`) is the requested output at the
antenna port. An NCP with a FEM caps it at the product's regulatory
maximum and programs the radio with `dbm − PA gain`, rounded down to a
supported radio step. It must return the effective antenna output in
`applied_dbm`, which on nRF52 is `TxPowerSplit::antenna_dbm` from
`NrfFemPhy`. `applied_dbm` can therefore be lower than the request, for
example 18 for a request of 20 with a 22 dB PA. RSSI, LQI and energy-scan
values exclude the LNA gain. This is `zigbee-mac` `fem`, with the nRF52
backend `nrf-fem`, which is hardware-unverified; see the nRF platform
guide. No current NCP image has a FEM.

### 7.3 DATA (`0x02xx`, endpoint 2)

`APS_DATA_REQUEST` (`0x0201`) request payload:

| Field | Size | Notes |
|---|---:|---|
| `dst_mode` | 1 | `0x01` group, `0x02` NWK, `0x03` IEEE, `0x0F` broadcast (zigpy `AddrMode` values) |
| `dst` | 2 or 8 | group / NWK / broadcast (`0xFFFF`, `0xFFFD`, `0xFFFC`, `0xFFFB`); IEEE for mode 3 |
| `dst_ep` | 1 | ignored for groups |
| `src_ep` | 1 | must be a registered host endpoint, 0 (ZDO), or 242 (GP) |
| `profile` | 2 | |
| `cluster` | 2 | |
| `tx_options` | 1 | bit0 APS ACK; bit1 APS link-key security; bit2 force route discovery; bit3 fragmentation permitted; bit4 use alias; bit5 extended timeout (sleepy destination); bit6 source route present; bit7 reserved |
| `radius` | 1 | 0 = default (2 × max depth) |
| `non_member_radius` | 1 | groups only (zigpy default 0; EZSP uses 3) |
| `priority` | 1 | 0 low, 1 normal, 2 high, 3 critical. Orders the NCP APS queue only. |
| `relay_count`, `relays[]` | 1 + 2n | only if bit6. **Path order from the coordinator** (entry 0 = first hop). n ≤ `max_source_route_relays` (8). n = 0 means "direct, no source-route subframe". Since master fd3bc03 the NWK layer also *accepts* inbound source-routed frames with an empty relay list, which some coordinators send. |
| `alias_src`, `alias_seq` | 2 + 1 | only if bit4 (GP proxy, broadcast/groupcast) |
| `asdu_len`, `asdu` | 2 + n | n ≤ `max_asdu`, or larger with fragmentation |

Response `status`:

- `SUCCESS`: queued. An `EVT_APS_DATA_CONFIRM` will follow.
- `BUSY`: the APS queue is full and nothing was queued.
- `TOO_LARGE`, `INVALID_PARAMETER` or `INVALID_STATE` (network down).

Map: **E** (`apsde_data_request`, fragmentation feature, alias) + **P**
(host-supplied relays must be installed in `SourceRouteTable` for that send;
an APS queue with a bounded in-flight count and confirm events is N).

`EVT_APS_DATA_CONFIRM` (`0x0281`): `u16 req_tid, u8 status, u8 zb_status,
u8 aps_counter, u16 dst_nwk, u8 flags (bit0 APS ACK received, bit1 via source
route, bit2 route discovery done), u16 elapsed_ms`. `status` is `SUCCESS` or
`FAILURE`. `zb_status` carries the Zigbee reason.

For a unicast with `tx_options` bit0 the confirm waits for the APS ACK: it
follows once the ACK arrives (`SUCCESS`, flags bit0) or once the APS retries
are exhausted, the ACK tracking was evicted by newer acknowledged sends, or
the network was reset (`FAILURE`, `zb_status` `0xA7` NO_ACK). The NCP keeps
running the stack meanwhile (`ApsLayer::last_ack_handle` + `ack_status`).
`elapsed_ms` then counts from the transmission. Group and broadcast frames,
and frames without bit0, are confirmed after the MAC/NWK transmission.

`EVT_APS_DATA_INDICATION` (`0x0280`):

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | `dst_mode` (`0x01` group, `0x02` unicast to NCP, `0x0F` broadcast) |
| 1 | 2 | `dst` (group ID, `0x0000`, or broadcast address) |
| 3 | 1 | `dst_ep` |
| 4 | 2 | `src_nwk` |
| 6 | 8 | `src_ieee` (valid if flags bit2) |
| 14 | 1 | `src_ep` |
| 15 | 2 | `profile` |
| 17 | 2 | `cluster` |
| 19 | 1 | `flags`: bit0 APS secured; bit1 NWK secured; bit2 src IEEE valid; bit3 broadcast; bit4 groupcast; bit5 reassembled; bit6 handled by NCP (ZDO request already answered locally); bit7 local loopback (GP proxy, ZDO loopback) |
| 20 | 1 | `lqi` |
| 21 | 1 | `rssi` (`i8`, dBm, last hop) |
| 22 | 2 | `last_hop_nwk` |
| 24 | 1 | `aps_counter` |
| 25 | 1 | `key_source` (APS key if bit0: 0 n/a, 1 global TCLK, 2 unique TCLK, 3 application link key) |
| 26 | 4 | `rx_time_ms` (NCP monotonic) |
| 30 | 2 | `asdu_len` |
| 32 | n | `asdu` |

Map: **E** (runtime `host-passthrough`). `rssi` is the received signal
strength of the last hop as measured by the radio (`zigbee-mac` `rx-rssi`:
the ESP32-H2/C6/C5 PHY; `-128` on backends that do not measure it). Flags
bit0 and `key_source` come from the APS layer's verification of the frame
(`ApsLayer::last_rx_security`), independently of bit1.

### 7.4 ZDO (`0x03xx`, endpoint 2)

ZDO has no dedicated request commands. Hosts send ZDP frames as ordinary
`APS_DATA_REQUEST`s (profile 0, endpoints 0/0), which is what zigpy and the
Go hub's generic interviewer already do.

- **Inbound ZDO.** The NCP's ZDO server (`ZdoHandler`) answers every
  mandatory server request addressed to it (Node/Power/Simple/Active EP
  descriptors, NWK/IEEE_addr, Match_Desc over the registered host endpoints,
  Mgmt_Lqi/Rtg/Bind, Mgmt_Leave/Permit_Joining/NWK_Update). It then forwards
  the frame to the host with flag `handled by NCP`; the host must not reply
  again. Every ZDO response and every `Device_annce` is forwarded unchanged.
- **TSN rules.** The host chooses TSNs for its own requests. NCP-originated
  ZDO requests (channel change, internal discovery) use the NCP's own
  counter. A response is consumed internally only if it matches a pending
  NCP request on `(cluster, src, tsn)`, and it is still forwarded with
  `handled by NCP`. A TSN collision therefore costs one ignored response,
  never a lost one.
- **No loopback** (decision 10). An `APS_DATA_REQUEST` with profile 0
  addressed to the NCP's own NWK address or IEEE fails with
  `INVALID_PARAMETER`. Hosts get the same information from dedicated
  commands, and their libraries synthesize ZDO responses locally where the
  host framework expects them:

  | Host-side need | ZRHP replacement |
  |---|---|
  | Energy scan | `NWK_ENERGY_SCAN` |
  | Channel change | `NWK_CHANGE_CHANNEL` |
  | Coordinator node descriptor | `NWK_GET_INFO` + constant descriptor |
  | Mgmt_Lqi / Mgmt_Rtg of the coordinator (zigpy topology scan, Z2M network map) | `DIAG_GET_NEIGHBORS` / `DIAG_GET_ROUTES` |
  | Mgmt_Bind of the coordinator | Host binding mirror |
  | Permit join on the coordinator | `NWK_PERMIT_JOIN{target 0x0000}` |

| ID | Name | Request payload | Response after `status` | Map |
|---|---|---|---|---|
| `0x0301` | `ZDO_CONFIG` | `u8 flags (bit0 forward handled requests, bit1 forward Match_Desc)` | — | N |

### 7.5 SECURITY (`0x04xx`, endpoint 2)

| ID | Name | Request payload | Response after `status` | Map |
|---|---|---|---|---|
| `0x0401` | `SEC_GET_NETWORK_KEY` | `u8 which (0 active, 1 alternate)` | `[16] key, u8 seq, u32 nwk_fc_limit` (`ACCESS_DENIED` if export is locked) | P (`SecurityStateJournal` holds it; the read API is N) |
| `0x0402` | `SEC_SET_TC_POLICY` | `u8 allow_join (0 deny, 1 global TCLK, 2 install code only), u8 require_tclk_update, u8 allow_tc_rejoin, u8 allow_app_link_keys, u16 join_timeout_s` | — | P (BDB TC fields, `allow/deny_application_key_request`) |
| `0x0403` | `SEC_ADD_INSTALL_CODE` | `[8] ieee, u8 len (8/10/14/18), code[]` (includes CRC, little-endian) | `[16] derived_key`; the host stores it (section 8) | E (`provision_install_code`, `derive_install_code_key`) |
| `0x0404` | `SEC_ADD_LINK_KEY` | `[8] ieee, [16] key, u8 key_type (1 TCLK unique, 3 application), u8 attrs, u32 rx_floor, u8 flags (bit0 temporary until join)` | — | E (`provision_link_key`) + N (floors/attributes import) |
| `0x0405` | `SEC_REMOVE_LINK_KEY` | `[8] ieee` | — | P |
| `0x0406` | `SEC_ROTATE_NETWORK_KEY` | `[16] key (0 = random), u8 mode (0 broadcast, 1 unicast to routers), u16 switch_delay_s` | accepted; `EVT_NETWORK_KEY_SWITCHED` follows | E (`prepare/commit/activate/complete_network_key_rotation`) |
| `0x0407` | `SEC_GET_LINK_KEY` | `[8] ieee` | `[16] key, u8 type, u8 attrs, u32 tx (global limit), u32 rx_floor` | P |
| `0x0408` | `SEC_GET_FRAME_COUNTERS` | — | `u32 nwk_current, u32 nwk_limit, u32 aps_current, u32 aps_limit, u32 stride` | P (reservation exists; global APS counter is N, see 8.4) |
| `0x0409` | `SEC_DERIVE_EZSP_KEY` | `[8] ieee` | `[16] key` (capability bit 14; export-gated like other keys) | N |
| `0x040A` | `SEC_LOCK_KEY_EXPORT` | `u32 confirm` = `0x4B434F4C` ("LOCK") | — (idempotent; lasts until the next NCP reboot) | N |
| `0x0480` | `EVT_LINK_KEY_UPDATED` | `[8] ieee, u16 nwk, u8 key_type, u8 attrs (0 provisional, 1 unverified, 2 verified), u8 origin` | event (the host then reads the key with `TBL_WRITEBACK`) | P |
| `0x0481` | `EVT_KEY_FAILURE` | `[8] ieee, u16 nwk, u8 reason (1 verify failed, 2 request denied, 3 no key, 4 MIC failure burst)` | event | P |
| `0x0482` | `EVT_NETWORK_KEY_SWITCHED` | `u8 new_seq, u16 req_tid` | event | P |
| `0x0483` | `EVT_FC_CHECKPOINT` | `u8 domain (1 NWK, 2 APS), u8 key_seq, u32 durable_limit, u32 generation` | event, emitted after every durable reservation commit | N |

**Key export** (decision 3).

- **Default.** `KEY_EXPORT` is **enabled by default** in the ESP32 NCP
  product.
- **Locking.** `SEC_LOCK_KEY_EXPORT` locks it until the next reboot. After
  that, every key-revealing response is refused with `ACCESS_DENIED`:
  - `SEC_GET_NETWORK_KEY`, `SEC_GET_LINK_KEY` and `SEC_DERIVE_EZSP_KEY`;
  - key-bearing records in `BKP_EXPORT_*`, `TBL_*` and `GP_*` (keys are
    omitted; flags mark them absent).

  `SYS_GET_STATUS.flags` bit0 reports the lock.
- **Reboot.** The lock does not survive a reboot. A host that wants it
  permanently re-applies it during resume (12.1).
- **Paging is unaffected.** NCP-originated `TBL_WRITEBACK` of NEW keys still
  carries the key, because the host store must stay complete.
- **Periodic HA backups.** zigpy takes periodic backups that need keys, so
  the zigpy library does not lock by default.

**EZSP hashed TCLK in v1** (decision 8).

- **Input.** When a restore carries `EZSP_HASHED_TCLK` (from
  `stack_specific.ezsp.hashed_tclk`), the NCP keeps the seed and sets
  `TC_LINK_KEY.flags` bit0.
- **Derivation.** For every device that has no explicit link key, the NCP
  derives the unique TCLK on demand: Silicon Labs' "AES HMAC" of the device
  EUI64 keyed with the seed [Silabs-HLK]. We implement it as the Zigbee
  keyed hash, HMAC with the AES-MMO hash and a 16-byte block. The derived
  key is cached like any other key, with origin 4.
- **Unverified.** The EUI64 byte order fed to the hash is not confirmed.
  `ezsp_hashed_tclk.json` lists both candidates. The capability bit stays
  clear until gate H5 (14.6) has matched a key read from a real ZBT-1.
- **Not used for new networks** (decision N3). A network formed by
  zigbee-rs uses only random per-device TCLKs, never derived ones. Hashed
  mode exists only to keep EZSP-migrated devices working.

### 7.6 TABLES (`0x05xx`, endpoint 3)

Section 8 defines these commands.

### 7.7 BACKUP (`0x06xx`, endpoint 4)

Section 9 defines these commands.

### 7.8 DIAG (`0x07xx`, endpoint 1)

| ID | Name | Request payload | Response after `status` | Map |
|---|---|---|---|---|
| `0x0701` | `DIAG_GET_COUNTERS` | `u8 group_mask` (bit0 link, bit1 MAC, bit2 NWK, bit3 APS, bit4 security, bit5 tables, bit6 queues, bit7 Green Power) | `u8 n, n×{u16 id, u32 value}` | P (several counters exist, e.g. `replay_table_full`, `frame_pending_acks`; a registry is N) |
| `0x0702` | `DIAG_RESET_COUNTERS` | `u8 group_mask` | — | N |
| `0x0703` | `DIAG_GET_NEIGHBORS` | `u8 start` | `u8 total, u8 n, n×{[8] ieee, u16 nwk, u8 type, u8 rx_on, u8 relationship, u8 lqi, u8 outgoing_cost, u8 age}` | P |
| `0x0704` | `DIAG_GET_ROUTES` | `u8 start` | `u8 total, u8 n, n×{u16 dst, u16 next_hop, u8 status, u8 flags}` | P |
| `0x0705` | `DIAG_GET_CHILDREN` | `u8 start` | `u8 total, u8 n, n×{[8] ieee, u16 nwk, u8 capability, u8 timeout_exp, u32 last_poll_ms}` | P |
| `0x0706` | `DIAG_GET_SOURCE_ROUTES` | `u8 start` | `u8 total, u8 n, n×{u16 dst, u8 k, k×u16}` (cache only) | P |

Counter ID groups:

- `0x01xx`: link (5.12).
- `0x02xx`: MAC (TX, RX, CCA failures, no-ACK, retries, indirect expired,
  frame-pending ACKs).
- `0x03xx`: NWK (decrypt failures, replay rejects, `replay_table_full`,
  route discoveries, route errors, link-status sent, address conflicts,
  broadcast-table full).
- `0x04xx`: APS (TX, RX, ACK failures, duplicates, fragmentation failures).
- `0x05xx`: security / TC (joins admitted or denied, key transports, verify
  failures, reservation commits).
- `0x06xx`: tables (lookups, misses, lookup timeouts, miss-queue drops,
  writebacks, evictions, conflicts).
- `0x07xx`: queues (APS queue high-water, `BUSY` responses, host-absent
  drops, indirect queue full).
- `0x08xx` (group bit 7): Green Power (7.11).

### 7.9 FIRMWARE (`0x08xx`, endpoint 4)

These commands update the NCP's own firmware.

| ID | Name | Request payload | Response after `status` | Map |
|---|---|---|---|---|
| `0x0801` | `FW_BEGIN` | `u32 image_size, u8 format (1 ESP app image + MCUboot TLVs, 2 MCUboot), [32] sha256, u32 version, u8 flags (bit0 allow downgrade, dev builds only)` | `u16 max_chunk, u8 slot` | E (`products/esp32-zigbee-ncp` A/B slots, signed policy, IDF rollback bootloader; host-tested, not yet on hardware) |
| `0x0802` | `FW_WRITE` | `u32 offset, data[]` | `u32 next_offset` | E |
| `0x0803` | `FW_FINISH` | — | `u8 verify_result` | E |
| `0x0804` | `FW_ACTIVATE` | `u16 delay_ms` | — (then reboot) | E |
| `0x0805` | `FW_CONFIRM` | — | — | E |
| `0x0806` | `FW_ABORT` | — | — | E |
| `0x0807` | `FW_GET_SLOTS` | — | `u8 n, n×{u8 slot, u8 state, u32 version, [32] sha256}` | E |

Rules:

- **Writes.** Writes are sequential. A `FW_WRITE` at an offset below
  `next_offset` whose bytes are identical to what is stored succeeds without
  writing, which makes retries idempotent. A write with different bytes
  fails with `CONFLICT`.
- **Verification.** `FW_FINISH` verifies the length, the SHA-256 and the
  product's signature policy. Unsigned images are `ACCESS_DENIED` unless the
  NCP is a development build.
- **Activation and confirmation.** `FW_ACTIVATE` marks a **trial** boot.
  After the reboot, the new image must establish a session, resume the
  network (`NWK_INIT` succeeds) and receive `FW_CONFIRM` from the host
  within the product's trial window (default 10 minutes). Otherwise the
  bootloader rolls back.
- **Slot size.** `FW_BEGIN` with an `image_size` larger than the inactive
  application slot of the NCP's layout (8.9) fails with `TOO_LARGE` before
  anything is erased.
- **Protected regions.** Staging never touches the NV journals, the identity
  item or the bootloader.
- **Recovery path.** `FW_*` is the normal update path for every host. An
  embedded host that controls the H2's EN and BOOT lines (Thread BR board)
  also has a recovery path: the H2 ROM serial bootloader, for an NCP that
  cannot establish a ZRHP session or for first provisioning. It is a
  manual/development path with an operator-supplied image (11.5); firmware
  rollback itself is the NCP's A/B slots (8.9).

### 7.10 LOG (`0x09xx`, endpoint 5)

| ID | Name | Payload | Map |
|---|---|---|---|
| `0x0901` | `LOG_CONFIG` (request) | `u8 level (0 off … 5 trace), u32 module_mask` → `status` | N |
| `0x0980` | `EVT_LOG_RECORD` | `u32 time_ms, u8 level (1 error … 5 trace), u8 module, u16 seq, text[]` (UTF-8, ≤ 200 B, no terminator; decision 12) | N |
| `0x0981` | `EVT_LOG_DROPPED` | `u32 count, u16 first_seq` | N |

The NCP keeps logs in a bounded ring. When the LOG endpoint has no credit,
records are dropped and counted, then reported. Logs never take credit,
window slots or CPU priority away from other endpoints. A record that is not
valid UTF-8 is a firmware bug; hosts replace the bad sequences and count
them. When USB-Serial-JTAG carries ZRHP, the NCP firmware must not print to
it through `esp-println` (section 11).

### 7.11 GREEN POWER (`0x0Axx`, endpoint 2)

**What the NCP runs** (decision 11). The NCP runs the zigbee-rs GP
**combo**: the proxy (Proxy Basic, default in every zigbee-rs router since
2026-11-13), the sink (`green-power-sink`), and, on the ESP32-H2, the
TempMaster (`green-power-bidirectional`).

State on master 184b82d (see [Green Power](green-power.md)):

| Piece | Validation |
|---|---|
| Sink (`Sink<8, 4>`; Sink Table persisted in the GP journal as `ZBGS` records paired with the Proxy Table; adaptive per-GPD counter persistence) | Host-tested and lab-tested on the ESP32-H2-Zero combo light |
| Bidirectional commissioning with the combo as its own TempMaster (reply armed in the radio and started from its interrupts, `MacDriver::arm_gp_reply`, ESP32 only) | Proven 3 of 3 on the H2 combo light on 2026-11-17 |

The NCP integration (2026-11-19) is host-tested only: the commands and
events below over the in-memory host link with the real Trust Center
coordinator on a mock MAC (`apps/ncp-coordinator/tests/gp.rs`), and the GP
journal in the NCP's `zbnv` geometry. None of it is validated **as a
coordinator NCP** on hardware yet; gate H10 covers that. Since fd3bc03 the NWK layer also accepts the empty source-route relay
lists that some coordinators send (section 7.3).

**Sink modes.** There is exactly **one sink authority** per network on the
coordinator, chosen with `GP_SINK_CONFIG.mode`:

| Mode | Sink | What the host sees | Default for |
|---|---|---|---|
| 1 `NCP_SINK` | The NCP sink commissions, pairs, verifies security and counters, de-duplicates and keeps the Sink Table | `EVT_GP_*` events (below); no raw GP cluster frames | zigpy library, embedded hub |
| 0 `HOST_SINK` | The host (its own GP sink) | GP Notification and Commissioning Notification frames from proxies, and the NCP's local proxy output, as `EVT_APS_DATA_INDICATION` on EP 242 (flag bit7 for local) | Go hub and zigbee-herdsman in v1 |

Behaviour in `HOST_SINK` mode:

- A host-originated GP Response that appoints the coordinator as TempMaster
  (`temp_master` = own NWK address) is also handed to the NCP's local proxy
  gpTxQueue. A broadcast is not looped back to its sender, so without this
  the NCP's proxy would never see it.
- The NCP sink is disabled. The proxy behaves as on a router.

**Security level** (decision N1).

- The NCP product's sink default is `gpsSecurityLevel` **0x00**: GPDs with
  security level 0, 2 or 3 are commissioned, as in zigpy.
- This differs from the zigbee-rs **router** combo default of 0x01 (user
  decision 2026-11-15), which is unchanged.
- Unsecured GPDs are protected only by the 2 s duplicate filter and the MAC
  sequence check, so anyone in radio range can replay or spoof them (13).
  A host can raise the level with `GP_SINK_CONFIG`.

**Commands.**

| ID | Name | Request payload | Response after `status` | Map |
|---|---|---|---|---|
| `0x0A01` | `GP_SINK_CONFIG` | `u8 mode (0 host sink, 1 NCP sink), u8 communication_mode (0 full unicast, 1 derived groupcast, 2 pre-commissioned groupcast, 3 lightweight unicast), u16 group (for mode 2), u8 exit_mode (gpsCommissioningExitMode bits), u16 window_s, u8 security_level (gpsSecurityLevel), u8 groupcast_radius` | — | P (`SinkConfig`; runtime switch between modes is N) |
| `0x0A02` | `GP_SET_SHARED_KEY` | `u8 key_type, [16] key` | — | P |
| `0x0A03` | `GP_SINK_COMMISSIONING` | `u8 action (1 enter, 0 exit), u8 flags (bit0 involve proxies, bit1 exit on first pairing), u16 window_s` | — | E (`gp_sink_enter_commissioning`, `gp_sink_exit_commissioning`) |
| `0x0A04` | `GP_SINK_TABLE_READ` | `u8 start` | `u8 total, u8 n, n×R_GP_SINK record` (keys omitted when export is locked) | P |
| `0x0A05` | `GP_SINK_TABLE_PUT` | `u8 flags (bit0 send GP Pairing AddSink), u8 n, n×R_GP_SINK record` | `u8 n, n×u8 status` (`NO_SPACE`, `ALREADY_EXISTS`, `INVALID_PARAMETER`) | N (import API) |
| `0x0A06` | `GP_SINK_REMOVE` | `gpd_id, u8 flags (bit0 send GP Pairing RemoveGPD)` | — (`NOT_FOUND`) | E (`gp_sink_remove`) |
| `0x0A07` | `GP_PROXY_TABLE_READ` | `u8 start` | `u8 total, u8 n, n×R_GP_PROXY record` | P |
| `0x0A80` | `EVT_GP_FRAME` | `u8 origin (1 direct GPDF, 2 GP Notification, 3 GP Commissioning Notification), u8 result (0 translated, 1 passed through, 2 commissioned, 3 Commissioning Reply sent, 4 decommissioned, 5 commissioning step accepted), gpd_id, u8 device_id, u8 command_id, u32 frame_counter, u8 security_level, u8 key_type, u8 flags (bit0 rxAfterTx, bit1 payload decrypted, bit2 key in payload was TC-LK wrapped), u16 proxy_nwk (FFFF = direct), u8 gpp_gpd_link, i8 rssi, u8 lqi, u8 len, payload[]` | event | P (`SinkOutcome`, `SinkEvent::GpdCommand`; the surface is N) |
| `0x0A81` | `EVT_GP_COMMISSIONING` | `u8 state (1 entered, 0 left), u16 window_s (FFFF none), u8 involve_proxies, u8 exit_reason (1 explicit, 2 first pairing, 3 timeout)` | event | E (`SinkEvent::CommissioningEntered/Left`) |
| `0x0A82` | `EVT_GP_COMMISSIONED` | `gpd_id, u8 device_id, u8 flags (bit0 bidirectional, bit1 key present), R_GP_SINK record` | event | E (`SinkEvent::Commissioned`) + N |
| `0x0A83` | `EVT_GP_COMMISSIONING_FAILED` | `gpd_id, u8 reason` (`SinkDrop`: 1 Invalid, 2 NotCommissioning, 3 NoEntry, 4 SecurityMismatch, 5 AuthFailed, 6 Stale, 7 Duplicate, 8 SecurityPolicy, 9 NoKey, 10 NoTempMaster, 11 TableFull, 12 Unsupported, 13 QueueFull) | event | E |
| `0x0A84` | `EVT_GP_DECOMMISSIONED` | `gpd_id` | event | E |

- `gpd_id` is `u8 app_id` (0 SrcID, 2 IEEE), followed by `u32 src_id`, or by
  `[8] ieee, u8 endpoint`.
- `R_GP_SINK` (record type `0x0B`, 9.3) is defined by ZRHP, not as the
  runtime's journal encoding, so that hosts can mirror and back it up.
- `EVT_GP_FRAME` is emitted **only for frames the sink accepted**:
  authenticated, fresh and not a duplicate. Duplicates from several proxies,
  replays and unknown GPDs are counted (DIAG group `0x08xx`) and never
  forwarded.
- `payload` is the decrypted GPD command payload. For commissioning
  frames it is the Commissioning command payload exactly as received,
  including the TC-LK-wrapped key and its MIC, so that a host-side sink
  shadow can process it byte-identically.

**Coexistence with zigpy's own GP sink (normative).** zigpy (current `dev`)
has a complete sink in `zigpy.zgp.GreenPowerManager`:

- `permit_join` broadcasts GP Proxy Commissioning Mode;
- `_process_commissioning` sends a GP Response, a `Device_annce` for the
  alias, and a GP Pairing to pre-commissioned groupcast 0x0B84;
- `gp_packet_received` feeds `GreenPowerDevice` objects, which carry the
  quirks and entities;
- it subscribes the coordinator to group 0x0B84.

Both sinks must never pair, so in `NCP_SINK` mode the zigpy library (12.1)
makes zigpy a **shadow sink**:

1. **Same pairing semantics.** The library configures the NCP sink with
   `communication_mode = 2`, `group = 0x0B84` and **`security_level = 0x00`**
   (decision N1). This matches zigpy, so unsecured GPDs such as FlowerX GP
   and BGT1 GP are accepted. Pairings the NCP sends are
   then identical in semantics to zigpy's, so a later move back to a zigpy
   sink (for example EZSP) keeps every proxy's pairing valid.
2. **Permit join.** `permit(time_s)` → `NWK_PERMIT_JOIN` plus
   `GP_SINK_COMMISSIONING{enter, involve proxies, time_s}`. zigpy's
   `GreenPowerManager.permit_join` still runs and enters its commissioning
   window, but its transmissions are swallowed (step 4).
3. **Feeding zigpy.** Every `EVT_GP_FRAME` becomes a
   `zigpy.types.ZigbeeGpPacket` passed to `app.gp_packet_received()`:
   - commissioning frames with `result` 2 or 5, so zigpy creates the
     `GreenPowerDevice` with the same key and counter;
   - operational frames with `result` 1, so quirk entities update.

   zigpy's own counter and duplicate checks then see a strictly increasing
   counter, because the NCP already removed duplicates.
4. **Swallowed transmissions.** The library **does not transmit**, and
   reports success to zigpy for, every zigpy-originated packet that belongs
   to sink behaviour: GP cluster `0x0021` server → client commands from
   EP 242 (Pairing 0x01, Proxy Commissioning Mode 0x02, Response 0x06,
   Translation Table Update/Request) and zigpy's alias `Device_annce` for a
   GPD. The NCP sink already sent its own.
5. **Defence in depth.** As a second guard, the NCP rejects such frames from
   the host with `CONFLICT` in `NCP_SINK` mode and counts them.
6. **Unpair.** `zigpy.green_power.unpair()` / `force_remove` of a GP device
   → `GP_SINK_REMOVE{send RemoveGPD}`.
7. **Migration and resume.** GP devices that zigpy already knows (from an
   EZSP era) are pushed once with `GP_SINK_TABLE_PUT` (flags 0, no Pairing
   on air). The proxies are already paired to 0x0B84. The library compares
   `GP_SINK_TABLE_READ` with zigpy's GP devices at every start and resume.
   Imported level-0 GPDs work, and so does re-commissioning them, because
   the NCP sink accepts security level 0 (decision N1).

Whether zigpy's internal sink path accepts NCP-decrypted packets
(`security_status` already processed) and swallowed Pairings without
retries must be proven by a zigpy-level test (14.3).

**Roadmap** (decision N2, amended).

- **zigpy fork.** A "sink on radio" hook is added in **our zigpy fork**
  (`faronov/zigpy`): a radio library declares that it owns the GP sink, and
  `GreenPowerManager` delegates permit, pair and unpair to it instead of
  transmitting. With the hook, steps 2 and 4 disappear; without it (stock
  zigpy), the library keeps them.
- **zigpy-zrhp.** Also maintained under `faronov`. Using the hook in Home
  Assistant means installing the forked zigpy. The library must keep working
  with stock zigpy through steps 2 and 4.
- **Upstreaming.** No upstream zigpy PR is planned. Proposing the hook or
  the library upstream is a later step that needs the user's approval.
- **Go hub and zigbee-herdsman.** They move to `NCP_SINK` later. v1 keeps
  them in `HOST_SINK`.

**Translation.** The NCP has no local application endpoint, so
`SinkConfig.endpoint = None`: no built-in GPD → ZCL translation runs on the
coordinator, and every accepted GPD command reaches the host as
`EVT_GP_FRAME`.

**Capacity.** `gp_sink_table` = 16 and `gp_proxy_table` = 8 on the H2 NCP
(8.9). `GP_SINK_TABLE_ENTRIES` follows the runtime's `ncp-capacity` product
parameter (8 otherwise, so router images are unchanged); both values are
reported in `SYS_NEGOTIATE`.

**Backup.** Sink entries go to `stack_specific.zigbee_rs.gp_sink_table` as
hex `R_GP_SINK` values, and proxy entries to `gp_proxy_table` (9.2). OCB
itself has no GP fields.

**Implementation notes** (`apps/ncp-coordinator/src/gp.rs` over
`zigbee_runtime::green_power::ncp`, feature `green-power-ncp`):

- The NCP boots in `NCP_SINK` with the defaults above (window 180 s, exit
  on first pairing, derived groupcast). `GP_SINK_CONFIG` is kept in RAM:
  a host sends it at every start. `window_s` 0 selects 180 s; a
  `security_level` with bit 3 (involve the Trust Center) is
  `UNSUPPORTED_COMMAND`. `GP_SINK_COMMISSIONING` (enter) also sets
  `gpsCommissioningWindow` (non-zero `window_s`) and the "on first pairing"
  exit bit from its flags; it needs a running network and `NCP_SINK`
  (`INVALID_STATE`).
- `GP_SET_SHARED_KEY` is `UNSUPPORTED_COMMAND`: the sink provisions no
  shared GPD key (`zigbee_gp::sink`).
- `EVT_GP_FRAME.result` follows the sink outcome: 0–4 as listed. Result 5
  is not emitted by this build. A bidirectional commissioning reports the
  Commissioning frame with result 3 and the completing frame (Success)
  with result 2; `EVT_GP_COMMISSIONED` carries the full `R_GP_SINK`
  (key, counter). `rssi` is -128 (the MAC reports none); `lqi` is the
  GPDF's, or the APS frame's for a proxied one. An encrypted frame the sink
  could not decrypt for the report keeps its raw payload, flag bit 1 clear,
  `command_id` 0xFF.
- `R_GP_SINK` carries one group: a pre-commissioned groupcast entry exports
  its first group. `GP_SINK_TABLE_PUT` and `GP_SINK_REMOVE` need a running
  network (`INVALID_STATE`; the table is restored from NV when the network
  resumes). An imported entry's counter is taken as the last accepted one
  and is persisted exactly. Table reads page at
  400 record bytes (`n` may be smaller than the remaining entries; continue
  at `start + n`). `R_GP_PROXY` holds one entry per record (`format_version`
  1); with export locked its key bytes are zero.
- `HOST_SINK` hands the host's GP Pairing and Proxy Commissioning Mode, not
  only GP Response, to the local proxy (broadcast, group or unicast to the
  NCP): its proxy must be paired and in commissioning mode like any other.
  Its GP Notifications and Commissioning Notifications are copied once per
  GPDF (several destinations), and one addressed to the NCP itself is not
  transmitted. Group-addressed GP frames reach the host only for groups the
  GP endpoint is a member of (the NCP sink's or the proxy's; there is no
  host group-membership command yet).
- Counters (`DIAG_GET_COUNTERS` group bit 7, IDs `0x08xx`): `0x0801`–
  `0x080D` sink drops by `SinkDrop` reason (duplicates `0x0807`, stale
  `0x0806`, unknown GPD `0x0803`), `0x0810` reports lost to a full queue,
  `0x0811` local proxy copies, `0x0812` frames handed to a host sink,
  `0x0813` `CONFLICT` refusals.
- Persistence: the runtime's GP journal (`ZBGS` sink record paired with the
  Proxy Table record, adaptive counter records) in the product's `zbnv` GP
  window, 2 × 8 KiB logical sectors; restored by the coordinator's
  `NWK_INIT` resume like the router combo's.

---

## 8. Hybrid table protocol

### 8.1 Principle, standalone mode and automatic paging

The NCP runs the complete stack with **bounded caches**. The host stores
the **complete tables** and can serve entries to the NCP on demand.

The protocol is designed so that host data loss can cost availability but
never safety. If the host loses data, some devices may have to rejoin, but
no frame counter is ever reused and no key is ever silently replaced. That
guarantee rests on two decisions:

1. Every outgoing frame counter lives only on the NCP, as one NWK counter
   and **one global APS counter**, and is reserved durably there. The host
   only ever supplies *floors* (8.4).
2. A cached entry that holds state the host has not yet durably stored
   (*dirty*) cannot be evicted (I1).

**Modes** (decision 4). The NCP decides; the host does not have to
configure anything.

| Mode | When | Behaviour |
|---|---|---|
| **Standalone** (default) | All TC-device entries fit the NCP cache (`max_tc_devices`, 256 on the H2) | No lookups and no evictions. The NCP still writes back `NEW`/`UPDATE` entries if the host set `HOST_TABLES`, so the host store stays complete and paging can start at any time without a bulk transfer. |
| **Paging** (automatic) | A new keyed device must be admitted while the cache is full of clean, unpinned entries, and the host set `HOST_TABLES` | The LRU clean entry is evicted (it is already on the host). Later misses use `TBL_LOOKUP` (8.6). `SYS_GET_STATUS.flags` bit1 and counters report paging. |
| **Full without host tables** | Cache full and the host did not set `HOST_TABLES` (small embedded hosts without a store, or a misconfigured host) | The joiner is refused with `EVT_DEVICE_JOINED{decision=denied}` and a counter. The network keeps working for existing devices. |

The **address-map cache** is never a "mode". Its entries are clean, derived
data (IEEE ↔ NWK learned from frames, announces and the host). They are
evicted freely and re-learned, and they are only looked up on the security
paths of 8.6.

### 8.2 Detailed placement

| Data | NCP | Host | Paging |
|---|---|---|---|
| Device identity (IEEE, NWK, capability) | Address-map cache (512) + child table + neighbor table | Device registry (all devices) | `TBL_LOOKUP` only on the security paths (8.6); `TBL_PUSH` preload |
| Per-device unique TCLK / application key + incoming APS floor + attributes | TC-device cache (256), journaled | Link-key store (all) | Standalone, or pull on demand in paging mode; write back on NEW/UPDATE/EVICT |
| EZSP-hashed devices without an explicit key | Derived on demand from the seed (7.5), then cached like other keys | Nothing beyond the floor | Floors written back on eviction only |
| Children of the coordinator | Child table + `ChildTableJournal` (authoritative, 64) | Mirror (backup) | Not paged; capacity `max_children` |
| Source routes | Cache of the latest route records (128) | Relays per device (zigpy `device.relays`) | Host sends relays in `APS_DATA_REQUEST`, or `TBL_PUSH` |
| Neighbor / routing / discovery tables | Authoritative, RAM | — (read via DIAG) | None; rebuilt by Link Status and MTORR |
| Bindings / groups with the coordinator as source | APS tables (32 bindings, 16 groups × 8 endpoints) | Mirror | `NWK_GROUP_MEMBERSHIP`; resynced at start |
| Network key, NIB, TCLK, hashed-TCLK seed, TC policy | Authoritative | Backup copy | None |
| Outgoing NWK counter and global APS counter | Authoritative, reserved | Checkpoint log (floors) | `EVT_FC_CHECKPOINT`, floors in `NWK_INIT` |
| GP Proxy Table / GP Sink Table | Authoritative (GP journal) | Mirror via `EVT_GP_*` and backup | `GP_*_TABLE_READ`, `GP_SINK_TABLE_PUT` (7.11) |

### 8.3 Cache model and invariants

Each cached TC-device entry has:

- the record (9.3);
- `rev`, a `u32` from an NCP-wide, NV-persisted counter, incremented on
  every NCP-side modification;
- `dirty`: changed since the host last acknowledged it;
- `pinned`: children, keys of in-progress TC transactions, or
  host-requested.

Eviction is LRU among clean, unpinned entries.

With the global APS counter, an entry carries **no outgoing-counter state**.
Only these changes make it dirty:

- a new key;
- an attribute change (provisional → verified);
- a key-sequence change;
- the incoming APS floor, but only for the writeback that accompanies an
  eviction.

| Invariant | Statement |
|---|---|
| **I1** key durability | Every link key a device may be using is durable in an NCP journal, or acknowledged durable by the host, at every instant. A NEW key is journaled on the NCP before Transport-Key/Verify-Key, which the existing TC journal already does. A dirty entry is evicted only after a `TBL_WRITEBACK(EVICT)` has been acknowledged. |
| **I2** counter monotonicity | The NWK counter and the global APS counter are allocated only below an NCP-durable reservation limit. Every input (restore, floors) can only raise them. |
| **I3** replay floors | An inbound APS floor is durable on the NCP before the frame is accepted (existing two-phase commit). The eviction writeback carries the floor, and the host stores `max(stored, received)`. |
| **I4** stale updates | The host ignores a writeback whose `rev` ≤ its stored `rev` for that key. The NCP refuses a host push to a dirty entry with `CONFLICT` unless `force` is set. |
| **I5** host deletion | When the host deletes a device, it sends `TBL_INVALIDATE` (and `NWK_REMOVE_DEVICE` if the device must leave). The NCP drops the cached entry even if it is dirty, because deletion is the host's decision. |

### 8.4 Frame counters and keys

**NWK outgoing counter** (existing).

- There is one counter for the active network key. Before use, the NCP
  commits a reservation of `fc_stride = FRAME_COUNTER_RESERVATION_SIZE =
  0x400` values to `SecurityStateJournal`.
- After every commit it emits `EVT_FC_CHECKPOINT{domain=1, key_seq,
  durable_limit, generation}`. At 200 frames/s that is about one event every
  5 s.
- After a reboot the NCP resumes at the durable limit.

**Global APS outgoing counter** (decision 2; runtime change **N**).

| Aspect | Specification |
|---|---|
| Semantics | One `u32` counter is used for **every** APS-secured frame the NCP sends, under any link key: global TCLK, unique TCLKs, EZSP-derived keys, application keys, key-transport and key-load keys. Each partner sees a strictly increasing counter with gaps, which is all the receiver's replay check requires. No `(key, counter)` pair can ever repeat. This matches EmberZNet, which keeps one APS frame counter next to the NWK counter (bellows `write_aps_frame_counter`). |
| Scope | Selected by the coordinator/TC app (`TrustCenterCoordinatorApp` in global mode, the NCP product). End-device and router images keep their current per-key counters; their single TCLK makes the two schemes equivalent. |
| Allocation | `next()` returns `c` and increments it. If `c ≥ limit`, the runtime first commits `limit' = c + fc_stride` to `SecurityStateJournal` as a new `ApsOutgoingReservation` record (beside the NWK reservation, same CRC/commit discipline), then emits `EVT_FC_CHECKPOINT{domain=2}`. A failed commit fails the send with `SECURITY_ERROR`, and nothing is transmitted. At `0xFFFF_FFFF` APS-secured sends are refused and `EVT_FATAL` is raised; this is unreachable in practice. |
| Reboot | Resume at the durable limit, as for NWK. |
| Data structures | `ApsLinkKeyEntry.outgoing_frame_counter` / `outgoing_frame_counter_limit` are not used in global mode. The TC-device journal record drops its per-key reservation fields (new record version), which also shrinks each entry. `ApsSecurity` obtains outgoing counters from an `ApsOutgoingCounter` provider owned by the runtime. |
| Migration from per-key counters | On the first boot of firmware in global mode over an existing per-key journal: `start = max(every per-key outgoing_frame_counter_limit in the TC-device journal and the APS key table, every TC APS reservation)`. The runtime commits the global reservation, then writes the new journal format version. Until that commit completes, no APS-secured frame is sent. |
| Downgrade | Firmware that predates the format version must not run on migrated NV. Signed updates refuse it: the image carries a minimum NV format, and `FW_BEGIN` checks it. If old firmware is forced on through the ROM recovery path, the journal version check fails closed: the old firmware treats the NV as unknown, starts factory-new, and the host restores with floors (8.7). It never reuses a counter. |
| Host-visible | `SEC_GET_FRAME_COUNTERS` (`aps_current`, `aps_limit`), `EVT_FC_CHECKPOINT{domain=2}`, `NWK_INIT.aps_fc_floor`, `EVT_NCP_READY.aps_fc_limit`. |
| Backup mapping | Export: OCB `tc_link_key.frame_counter` and **every** `devices[].link_key.tx_counter` = the durable APS limit. Import: `aps start = max(all tx counters, TCLK counter, RS_PARAMS limit) + aps_margin` (9.4). |

**Floors from the host.**

- The host appends every `EVT_FC_CHECKPOINT` to a durable checkpoint log.
  It must persist at least once per 8 checkpoints per domain, so the log
  lags the NCP by at most `8 × fc_stride`.
- `NWK_INIT{nwk_fc_floor, aps_fc_floor}` passes `last_logged_limit +
  9 × fc_stride` (≤ 9 216 above the logged value).
- The NCP sets `counter = max(own durable limit, floor)` and commits a new
  reservation before the first transmission.
- This protects against an NCP whose NV was rolled back, for example by
  reflashing an old NV image, a ROM-path recovery, or swapping a chip with a
  stale backup.

**Proof sketch (no reuse).** Let L be the NCP's last durable limit in either
domain.

- The NCP transmits only values below L.
- After a reboot it continues at ≥ L.
- Restore sets the start to ≥ the backup value + margin (9.4).
- Floors only raise the value.
- With one global APS counter, the argument covers every APS key at once.

The only remaining hazard is two NCPs active at once with the same keys, for
example an old adapter still powered during a migration. Hosts must refuse
to restore while another adapter for the same network is connected. Section
9.6 requires unplugging the old adapter.

**Keys.**

- The network key, global TCLK and hashed-TCLK seed stay on the NCP and are
  copied into backups.
- Unique link keys are created on the NCP (TC generated, or derived from an
  install code) and are written back with `NEW` while pinned (I1).
- Keys pushed by the host (`SEC_ADD_LINK_KEY`, `TBL_PUSH`, restore) are
  stored as provided.

### 8.5 TABLES commands (`0x05xx`, endpoint 3)

Table IDs equal record types (9.3): `0x05` DEVICE, `0x08` SOURCE_ROUTE,
`0x09` GP_PROXY, `0x0B` GP_SINK (read-only here; written with `GP_*`).

| ID | Name | Direction | Request payload | Response after `status` |
|---|---|---|---|---|
| `0x0501` | `TBL_LOOKUP` | NCP → host | `u8 table, u8 key_type (1 IEEE, 2 NWK), key[8\|2]` | `u32 rev, record` (`NOT_FOUND` if unknown) |
| `0x0502` | `TBL_WRITEBACK` | NCP → host | `u8 table, u8 reason (1 NEW, 2 UPDATE, 3 EVICT, 4 DELETE), u32 rev, record` | — (`SUCCESS` only after a durable host commit; otherwise `STORAGE_ERROR`, and the NCP retries with backoff while keeping the entry dirty) |
| `0x0503` | `TBL_PUSH` | host → NCP | `u8 table, u8 flags (bit0 pin, bit1 force, bit2 only if free space), u8 n, n×record` | `u8 n, n×{u8 status, u32 rev}` |
| `0x0504` | `TBL_INVALIDATE` | host → NCP | `u8 table, u8 key_type, key[]` | — (`NOT_FOUND` is not an error) |
| `0x0505` | `TBL_SYNC` | host → NCP | `u32 host_db_epoch, u32 last_ncp_table_epoch, u32 host_max_rev, u8 flags (bit0 host store rolled back)` | `u32 ncp_table_epoch, u32 host_acked_rev, u16 dirty, u16 cached_devices, u16 cached_routes, u8 paging_active` |
| `0x0506` | `TBL_READ` | host → NCP | `u8 table, u16 cursor, u8 max` | `u16 next_cursor (FFFF = end), u8 n, n×{u32 rev, u8 state (bit0 dirty, bit1 pinned), record}` |
| `0x0507` | `TBL_STATS` | host → NCP | `u8 table` | `u16 capacity, u16 used, u16 dirty, u16 pinned, u32 hits, u32 misses, u32 evictions, u32 lookup_timeouts` |
| `0x0580` | `EVT_TBL_EPOCH_CHANGED` | NCP → host | `u32 new_epoch, u8 reason (1 factory reset, 2 restore, 3 NV re-initialized)` | event |
| `0x0581` | `EVT_TBL_PAGING` | NCP → host | `u8 active, u16 cached, u16 capacity` | event (paging started or stopped) |

**`TBL_SYNC` handling.**

- After `TBL_SYNC` the NCP re-sends every dirty writeback.
- With bit0, or when `host_max_rev < host_acked_rev`, it re-sends **every**
  cached entry as `UPDATE`.
- `host_acked_rev` is the highest `rev` the host ever acknowledged. The NCP
  persists it in the TC journal append that clears the dirty bit. A host
  whose store was restored from an older snapshot (for example an HA backup
  restore) is detected automatically, because its `host_max_rev` is smaller.

`table_epoch` is NCP-persisted. It changes whenever the NCP's tables are
wiped or replaced. The host stores `(ncp_ieee, table_epoch)`, and a mismatch
in `EVT_NCP_READY` means "the NCP no longer holds what you think".

Example (`messages.json`): the NCP looks up NWK `0x4F21`, and the host
returns rev 41 with a link key whose `tx_counter` (informational: the
global APS limit) is 14 024 and whose rx floor is 3:

```text
req: 00 0105 0100 0400 | 05 02 214f
rsp: 40 0105 0100 2e00 | 00 29000000 | 05 2600 0c0b0afeff142e84 214f 0a
     a0a1a2a3a4a5a6a7a8a9aaabacadaeaf 00 02 03 c8360000 03000000
```

### 8.6 Miss handling (paging mode only)

| Situation | NCP action | Bound |
|---|---|---|
| Inbound APS-secured frame, key not cached | Resolve the source IEEE (APS auxiliary header extended nonce, NWK header extended source, or address-map cache). If unknown, or the key is not cached, send `TBL_LOOKUP`. Park the frame in the miss queue. When the key arrives, run the normal replay and MIC checks with the record's floor. EZSP-hashed devices need no lookup for the key itself, only for the floor of an evicted entry. | Miss queue `tbl_miss_queue_len` (8) × 128 B. Timeout `tbl_lookup_timeout_ms` (250). On overflow or timeout the frame is dropped and counted (no APS ACK, so the sender retries). |
| Join or rejoin of an IEEE not cached (Update-Device or association) | Lookup by IEEE for a preconfigured key or install code. `NOT_FOUND` ⇒ apply the TC policy (global TCLK allowed or not). | Joins tolerate seconds; lookup bounded by the timeout. Host absent ⇒ policy-only decision, counted. |
| Request-Key / TCLK update | Generate the key, journal it (pinned), transport it, write back `NEW`. | At most `max_ncp_requests` (8) outstanding NCP → host requests |
| Outbound APS-secured unicast, key not cached | Lookup, then send. Without a key: confirm `FAILURE` with `zb_status` = APS security fail, counted. | Same timeout |
| Outbound IEEE-mode unicast, NWK unknown | Lookup by IEEE. | Same timeout |
| Outbound unicast without relays | Use the cached source route, else NWK route discovery. **No lookup in v1.** | — |
| Indication source IEEE | Caches only, **no lookup**. Flag bit2 tells the host whether it was filled. | — |
| Cache full of dirty or pinned entries | Admission of a new joiner is refused (TC denies) with counter + `EVT_DEVICE_JOINED{decision=denied}`. Lookups still work through one temporary slot. | — |

In standalone mode none of these lookups happen.

### 8.7 Consistency after reboots

| Event | NCP | Host |
|---|---|---|
| NCP reboot, NV intact | Restores the network, children, cached keys, floors and both counters from its journals. Address-map and source-route caches start empty. Keeps `table_epoch`. Re-sends dirty writebacks after the session. | Sees a new `ncp_cookie`. Runs **seamless resume** (12.1): `NWK_INIT` with floors, `TBL_SYNC`, replay of endpoints, groups, TC policy, GP sink configuration, permit-join remainder and key-export lock. It falls back to a full re-initialization only if the state cannot be reconciled. |
| Host restart, NCP running | Network keeps running. Indications are queued, then counted. Dirty writebacks wait. | Handshake, `SYS_NEGOTIATE`, `EVT_NCP_READY(network up)`. `NWK_INIT` is idempotent (applies floors only). `TBL_SYNC`. |
| Host store restored from an older snapshot | Detected by `TBL_SYNC` (`host_max_rev < host_acked_rev`) ⇒ writes back **all** cached entries. | Merges by `max(rev)` and `max(floor)`. Keys created **and evicted** after the snapshot are lost: those devices raise `EVT_KEY_FAILURE` and must rejoin (I1 still prevents silent key replacement). In standalone mode nothing is lost, because every key is still on the NCP. |
| NCP NV wiped, chip replaced, or `table_epoch` mismatch | `EVT_NCP_READY{network_state=0}` or a new epoch. | Restore from the latest backup (9.4) with checkpoint-log floors. |
| Power cut of both | Each side recovers as above. | — |
| Restore interrupted | Staging is discarded at boot (atomic commit); the old state remains. | Retry the restore. |

### 8.8 Host persistence duties

A host that sets `HOST_TABLES` (host capability bit0) must:

- commit a writeback durably before answering `SUCCESS`:
  - SQLite: a transaction with `synchronous=FULL`;
  - embedded raw-flash journal: a record written and read back with a valid
    CRC and commit marker;
- keep the FC checkpoint log as in 8.4;
- keep `(ncp_ieee, table_epoch, host_db_epoch, host_max_rev)`;
- answer `TBL_LOOKUP` within the timeout, from RAM or PSRAM;
- never block the TABLES endpoint behind ZIGBEE traffic.

Per-host store:

- **zigpy library:** a sidecar SQLite database next to zigpy's
  `zigbee.db` (12.1).
- **Go hub:** its own SQLite database.
- **zigbee-herdsman:** a sidecar file (12.3).
- **Embedded hub:** a raw-flash journal (3.4).

A host that does not set `HOST_TABLES` gets standalone behaviour only, and
new keyed devices beyond the NCP cache are refused (8.1).

Host capability bits (`host_caps`):

| Bit | Meaning |
|---:|---|
| 0 | `HOST_TABLES`: the host serves 8.5 |
| 1 | `FC_CHECKPOINTS`: the host keeps the checkpoint log |
| 2 | `RETRY_CACHE`: the host uses 6.5 |
| 3 | `GP_SHADOW`: the host mirrors NCP-sink GPDs into its own GP layer (7.11) |

### 8.9 ESP32-H2 NCP budget (decision 9)

**RAM.** The H2 application RAM region in the vendored esp-hal linker
script is `RAM: ORIGIN 0x40800000, LENGTH 0x3EFD0` = **258 000 B**
(`vendor/esp-hal/ld/esp32h2/memory.x`). `.data`, `.bss` and the stacks
share it.

| Item | Bytes | Basis |
|---|---:|---|
| Today's H2 combo-light router (`.bss` 70 684 + `.data` ≈ 3 300) | ≈ 74 000 | [Green Power §3.7](green-power.md) size table; includes the 32 KiB `esp-alloc` heap that the H2 example declares |
| ZRHP link: RX 16 × 652 + COBS 656 + TX 16 × 652 | 21 520 | 5.8 |
| TC-device / link-key cache: 256 × 48 B (IEEE 8, NWK 2, key 16, rx floor 4, rev 4, attrs/origin/seq/flags 4, LRU and index 8, padding 2) | 12 288 | decision 9 |
| Address-map cache: 512 × 12 B | 6 144 | decision 9 |
| Source-route cache: 128 × 20 B | 2 560 | 8 relays |
| Children 64 (see below): neighbor table 32 → 96 (+64 × 32 B), child records 64 × 24 B, source-match table 16 → 64 (+48 × 10 B), indirect queue 8 → 16 frames (+8 × 128 B) | 5 088 | — |
| APS request queue: `max_aps_inflight` 16 × 160 B | 2 560 | — |
| Host-absent queue 32 × 80 B + miss queue 8 × 128 B + retry cache 64 × 24 B | 5 120 | — |
| GP Sink Table 8 → 16 entries (+8 × ~80 B) | 640 | Measured 2026-11-19 for the whole NCP GP combo (H2 Thread-BR image): device +4 928 B (Proxy 2 264 + Sink 2 480 + reply buffers), NCP report queue 916 B, `.data` +776 B; the 32 KiB `esp-alloc` heap moved to the reclaimed 2nd-stage bootloader RAM (`dram2_seg`), so the main stack grew by 25 860 B |
| **Total static** | **≈ 130 000** | about 50 % of the region |
| Remaining for stacks and margin | ≈ 128 000 | Must be confirmed by the product's post-link check and stack painting |

**`max_children = 64`.**

- RAM is not the limit: about 80 B per child, including its neighbor slot,
  source-match entry and its share of the indirect queue.
- The binding limit is the child journal record: `8 + 2 + C × 14` bytes
  (`CHILD_ENTRY_LEN` = 14) must fit a slot minus its 16-byte CRC and
  commit trailer.
  - Router images use 512 B slots, so C ≤ 34 (`MAX_PERSISTED_CHILDREN` = 32).
  - With **1 024-byte slots**, C ≤ 71. 64 children need 906 B. The
    `ncp-capacity` feature selects them (`MAX_PERSISTED_CHILDREN` = 64).
- 64 is the first power of two that is ≥ 32 and fits. A restore whose child
  list exceeds 64 demotes the extras (note `CHILD_DEMOTED`), and they
  rejoin.
- Concurrent delivery to sleepy children is bounded separately, by the
  16-frame indirect queue and the 64-entry source-match table.

**Flash** (NV journals, both module variants). The ESP32-H2-MINI-1 on the
BR board exists with 2 MB (`-H2S`) and 4 MB (`-H4S`) flash, so the product
defines both layouts.

| Journal | Geometry | Size | Calculation |
|---|---|---:|---|
| `SecurityStateJournal` (NWK and global APS reservations, keys, APS replay floors) | 2 × 16 KiB logical sectors | 32 KiB | Coordinator guide: 16 KiB sectors for the full 64-entry replay table |
| `TrustCenterStreamJournal` (256 cached entries) | 2 × 32 KiB | 64 KiB | Implemented snapshot (record version 6): 43 B header + 256 × 75 B (per-key counter fields still encoded) + 1 + 8 × 16 B allowlist + 3 B rotation + 1 + 32 removal intents × 16 B + 5 B trailer = 19 893 B, **streamed** across 5 × 4 KiB chunk slots (4 060 B payload each) with a single commit marker after all chunks. The planned 72 B entries (rev/host-ack) and delta appends are not implemented: every store writes a full snapshot (a sector holds 8 one-chunk or one 5-chunk snapshot). |
| `ChildTableJournal` (64 children) | 2 × 4 KiB, 1 024 B slots | 8 KiB | 906 B record + trailer |
| GP journal (Proxy 8 + Sink 16) | 2 × 8 KiB, 1 024 B slots | 16 KiB | Sink snapshot 1 + 16 × (1 + 45) = 737 B fits a slot. The proxy+sink pair uses 2 slots. 8 KiB sectors give room for adaptive counter records (32 B). |
| APS tables (bindings, groups, application keys) | 2 × 4 KiB | 8 KiB | as on routers |
| Identity and settings (IEEE override, UART default) | 2 × 4 KiB | 8 KiB | — |
| **Total** | | **136 KiB** | — |

Partition tables (proposal; the product's post-link layout check must
enforce them). Both have **two application slots**: they carry the NCP's
firmware rollback (decision N6, amended 2026-11-18). `FW_*` writes the
inactive slot, `FW_ACTIVATE` starts a trial boot, the host confirms with
`FW_CONFIRM`, and the H2 bootloader returns to the previous slot if the new
image is not confirmed (7.9).

| Flash | Bootloader | Partition table | otadata | ota_0 | ota_1 | NV (`zbnv`) |
|---|---|---|---|---|---|---|
| 4 MB | 0x0 (32 KiB) | 0x8000 (4 KiB) | 0x9000 (8 KiB) | 0x10000, 0x1E0000 (1.875 MiB) | 0x1F0000, 0x1E0000 | 0x3D0000–0x400000 (192 KiB: 136 used + 56 spare) |
| 2 MB | 0x0 (32 KiB) | 0x8000 (4 KiB) | 0x9000 (8 KiB) | 0x10000, 0xE0000 (896 KiB) | 0xF0000, 0xE0000 | 0x1D0000–0x200000 (192 KiB) |

- The NCP image is estimated at 650–700 KB: today's combo light is 562 960
  B, plus the ZRHP link, the app and the caches. Both slots of the 2 MB
  layout hold such an image (917 504 B each, ≥ 200 KiB headroom).
- The hwlab Thread BR board's H2 has 4 MB (GigaDevice c84016, measured
  2026-11-18).
- This is an NCP-product layout. It does not move the existing devkit
  product's `zbnv` at 0x3F0000.
- `SYS_GET_INFO.flash_size` lets hosts and the ROM recovery path pick the
  right images.

**Implementation status (`ncp-capacity`).** One opt-in cargo feature per
crate (`zigbee-nwk`, `zigbee-aps`, `zigbee-bdb`, `zigbee-mac`,
`zigbee-runtime`, `router-app`) selects the NCP table sizes; only
`apps/ncp-coordinator`, `products/esp32-zigbee-ncp` and
`examples/esp32-ncp` enable it, and images without it are byte-identical.

| Table | Router / TC build | `ncp-capacity` |
|---|---:|---:|
| APS link-key table (`MAX_KEY_TABLE_ENTRIES`) | 32 | 256 |
| Trust Center devices (`MAX_TRUST_CENTER_DEVICES`) | 32 | 256 |
| TC removal intents (`MAX_PENDING_UNKNOWN_REMOVALS`) | 32 | 32 |
| Persisted APS application keys | 32 | 32 |
| Address map | 32 | 512 |
| Neighbor table | 32 | 96 |
| NIB `max_children` default | 20 | 64 |
| Persisted children / child-journal slot | 32 / 512 B | 64 / 1 024 B |
| Source-route cache | 32 | 128 |
| Indirect queue | 8 | 16 |
| NWK replay-floor table | 64 | 128 |
| ESP Frame Pending source table | 16 | 64 |
| Security-journal replay domains | 112 | 400 (16 KiB sectors required) |

The streamed Trust Center journal (`trust_center_stream`) writes every chunk
prefix first and the commit marker of the last chunk last; a snapshot is
current only when all chunks are contiguous, agree on generation/count/
length, pass their chunk CRCs and the whole-snapshot CRC, are committed and
decode. Torn stores, bit errors, missing chunks and generation wraparound
fall back to (or keep) the previous complete snapshot (host fault-injection
tests). A single-slot record of the previous NCP layout is read and
migrated on the next store.

Measured on the ESP32-H2 USJ image: `.bss` 128 120 → 192 216 B, `.data`
6 308 B, so 56 160 B of the 258 000 B region remain for the stack (56 096 B
above esp-hal's stack-guard word).

The first `ncp-capacity` image did not fit: the Trust Center runtime cloned
its full persisted state (~23 KiB at 256 devices) for every mutation and the
composition built the coordinator application (and the NCP) on the stack
before moving them into statics (`main`'s frame 77 824 B, Trust Center poll
frames up to 116 KiB, a 650 KiB static worst case). Now:

- `TrustCenterRuntime` mutates its live state **in place** and commits it;
  a failed store re-synchronises the live state from the journal in place
  (`TrustCenterDeviceStore::load_into`), or closes the runtime (not
  initialized) until it is re-initialized when that reload fails. RAM never
  runs ahead of flash. The journals decode straight into caller storage and
  scan by framing/CRC without materializing candidate states.
- The app (`TrustCenterCoordinatorApp::new_with_aps_tables_into`), its Trust
  Center runtime and table, and the NCP (`Ncp::new_into`) are constructed in
  their static storage; `TcPort` borrows the app. The composition root runs
  each construction step in its own frame.

`tools/esp32-ncp-stack-gate.py` (CI job `build-esp32-ncp`) bounds the static
worst case: the boot composition (device assembly, 34 112 B) plus the log
sink and every interrupt handler nested once leaves 20.6 KiB on the H2 USJ
image, 17.8 KiB on the Thread BR image (both ≥ the 16 KiB gate) and
205 KiB on the C6. Trust Center chains are ≤ 16 KiB (step) and ≤ 9.5 KiB
(runtime/journal). This is a direct-call bound, not hardware evidence: the
images have not been flashed or stack-painted yet.

## 9. Backup, restore and Open Coordinator Backup

### 9.1 Compatibility target

ZRHP hosts must import and export the **Open ZigBee Coordinator Backup**
format [OCB]: JSON with `metadata.format = "zigpy/open-coordinator-backup"`
and `metadata.version = 1`. zigpy/ZHA and zigbee-herdsman/Z2M use it.

Encoding rules from the specification, followed exactly:

- Byte sequences are hex strings, case-insensitive.
- IEEE, PAN ID, EPID and NWK addresses are **big-endian** hex (reversed
  from on-air order).
- Keys are hex in key-byte order (not reversed). This matches zigpy's
  `key.serialize().hex()`.
- `channel_mask` is a list of channel numbers.
- `devices[].is_child` is optional and defaults to **true**.
- `nwk_address` may be `null`.

Real files deviate from the specification in ways an importer must tolerate:

| Source | Quirk | Handling |
|---|---|---|
| Z2M (herdsman `toUnifiedBackup`) | `pan_id` and `nwk_address` written with `toString(16)`, i.e. without leading zeros (`"a62"`, `"abc"`) | Left-pad to 4 hex digits (zigpy does `zfill(4)` for NWK only; ZRHP hosts pad both) |
| Z2M | `metadata.internal.date` instead of `creation_time`; `znpVersion` / `ezspVersion` in `internal` | Accept either date key; pass the version keys through |
| Z2M | **No** `internal.network`: no TCLK, TCLK counter or TC address | Default `ZigBeeAlliance09`; the APS counter is unknown, so the host uses `aps_margin = 2²⁰` (9.4) |
| Z2M | `stack_specific` carries only `zstack.tclk_seed` or `ezsp.hashed_tclk`; other stacks' sections are dropped on re-export | A ZRHP-aware herdsman must preserve `stack_specific.zigbee_rs` (12.3) |
| Z2M ember adapter | Needs `metadata.internal.ezspVersion` for EZSP-origin backups (bellows comment) | Pass through unknown `internal` keys unchanged on re-export |
| zigpy | TCLK only in `metadata.internal.network.tc_link_key {key, frame_counter}` and `tc_address` | Read there; default `ZigBeeAlliance09`, counter 0, TC address = coordinator IEEE |
| zigpy | `internal.link_key_seqs`, `internal.tx_power`, `internal.route_table` | Map as below |
| zigpy | Imports `internal.network.nwk_manager` **without** byte reversal but exports it reversed | Always emit `"0000"` for a coordinator-managed network; when importing a non-zero value, warn and treat it as big-endian |
| bellows | `stack_specific.ezsp.hashed_tclk`; `tc_link_key` reported as `ZigBeeAlliance09` when hashed mode is active | Keep `hashed_tclk` (9.6) |
| zigpy-znp | `stack_specific.zstack.tclk_seed`; per-device keys already expanded into `devices[].link_key` on load | Keep the explicit keys; pass the seed through for re-export |
| zigpy-deconz | Old firmware cannot read the NWK counter, so 2³⁰ is reported | Accept it; the restore starts above it |
| zigpy-zboss | `stack_specific.zboss.*` TC policy settings | Pass through; apply the equivalent `SEC_SET_TC_POLICY` only when unambiguous |

### 9.2 Field mapping (hybrid split)

| OCB field | Record.field (9.3) | Authoritative | Export source | Import action | Runtime |
|---|---|---|---|---|---|
| `coordinator_ieee` | `NODE.ieee` | NCP identity item | Active IEEE (factory or override) | Override if it differs and `allow_ieee_change` is set (`IEEE_REWRITABLE`); otherwise `SECURITY_ERROR` | P (PIB settable) + N (persisted override) |
| `metadata.internal.node.nwk` / `.type` | `NODE.nwk` / `.logical_type` | NCP | `0x0000` / coordinator | Must be `0000` / coordinator, else `INVALID_PARAMETER` | E |
| `pan_id`, `extended_pan_id`, `channel`, `channel_mask`, `nwk_update_id` | `NETWORK.*` | NCP NIB | NIB | Written in the restore transaction | P |
| `security_level` | `NETWORK.security_level` | NCP | 5 | Only 5 accepted | E |
| `internal.network.nwk_manager` | `NETWORK.nwk_manager` | NCP | NIB | Written | P |
| `internal.tx_power` | `NETWORK.tx_power` | NCP | PIB | Applied, clamped to the product limit | E |
| `network_key.key`, `.sequence_number` | `NETWORK_KEY.key`, `.seq` | NCP `SecurityStateJournal` | Active key | Written | P |
| `network_key.frame_counter` | `NETWORK_KEY.tx_counter` | NCP reservation | **Durable limit** (≥ every value used) | start = max(value, floors) + `nwk_margin`; reserve | P + N |
| `internal.network.tc_link_key.key`, `tc_address` | `TC_LINK_KEY.key`, `.tc_ieee` | NCP | Global TCLK | Written | P |
| `internal.network.tc_link_key.frame_counter` | `TC_LINK_KEY.tx_counter` | NCP global APS counter | **Durable APS limit** | Contributes to the APS start (below) | N |
| `devices[].ieee_address`, `nwk_address` | `DEVICE.ieee`, `.nwk` | Host device DB | Host DB merged with NCP caches | Host DB + NCP address-map cache | N |
| `devices[].is_child` | `DEVICE.flags.is_child` | NCP child table | Child table | NCP child table (`CHILD_DEMOTED` if full) | E (`ChildTableJournal`) + N |
| `devices[].link_key.key` | `DEVICE.link.key` | Host link-key store | Host store merged with NCP cache (newer `rev` wins) | Host store; NCP cache as space permits (`HOST_BACKED` note) | N |
| `devices[].link_key.tx_counter` | `DEVICE.link.tx` | NCP global APS counter | Durable APS limit (same for all keys) | APS start = max(all tx, TCLK tx, floors) + `aps_margin` | N |
| `devices[].link_key.rx_counter` | `DEVICE.link.rx_floor` | Host store / NCP cache | Floor | Stored as the incoming floor (no margin) | P |
| `internal.link_key_seqs[ieee]` | `DEVICE.link.key_seq` | Host | Stored | Stored | N |
| `internal.route_table` | — | NCP (RAM) | Optional from `DIAG_GET_ROUTES` | **Ignored**: routes are rediscovered (MTORR at start) | — |
| `stack_specific.ezsp.hashed_tclk` | `EZSP_HASHED_TCLK` | NCP | Stored value | Stored; devices without an explicit key get derived keys (7.5, decision 8) | N |
| `stack_specific.zstack.*`, `.zboss.*`, other unknown keys | — | Host (pass-through) | Re-emitted unchanged | Not sent to the NCP | — |
| `stack_specific.zigbee_rs.*` | `RS_PARAMS`, `GP_PROXY`, `GP_SINK` | NCP | See below | Limits become floors; GP Proxy and Sink entries restored (no GP Pairing sent) | N |
| `metadata.*` (except the above) | — | Host | Host | Host | — |

The `stack_specific.zigbee_rs` object, written by ZRHP hosts:

```json
"zigbee_rs": {
  "version": 1,
  "frame_counter_stride": 1024,
  "table_epoch": 7,
  "nwk_frame_counter_limit": 2011024,
  "aps_frame_counter_limit": 14024,
  "host_backed_tables": true,
  "gp_proxy_table": ["<hex R_GP_PROXY value>", "..."],
  "gp_sink_table": ["<hex R_GP_SINK value>", "..."]
}
```

Hosts may add `metadata.internal.zigbee_rs = {fw, chip, build_id}` for
diagnostics. Importers must not depend on it.

### 9.3 Records (shared by BACKUP and TABLES)

Each record is a TLV: `u8 type, u16 len, value[len]`.

| Type | Name | Value layout | Size |
|---:|---|---|---:|
| `0x01` | `NODE` | `[8] ieee, u16 nwk, u8 logical_type (0 coordinator), u8 flags (bit0 IEEE is/should be an override)` | 12 |
| `0x02` | `NETWORK` | `u16 pan_id, [8] ext_pan_id, u8 channel, u32 channel_mask (bit n = channel n), u8 nwk_update_id, u16 nwk_manager, u8 security_level, i8 tx_power (0x7F unknown)` | 20 |
| `0x03` | `NETWORK_KEY` | `[16] key, u8 seq, u32 tx_counter, u8 flags (bit0 active)` | 22 |
| `0x04` | `TC_LINK_KEY` | `[16] key, [8] tc_ieee, u32 tx_counter, u8 flags (bit0 EZSP hashed mode active)` | 29 |
| `0x05` | `DEVICE` | `[8] ieee, u16 nwk (FFFF unknown), u8 flags`, then optional child info, then optional link block (below) | 11..38 |
| `0x06` | `EZSP_HASHED_TCLK` | `[16] seed` | 16 |
| `0x07` | reserved | (Z-Stack seed: host pass-through only) | — |
| `0x08` | `SOURCE_ROUTE` | `u16 dst, u8 n, n×u16 relays` (path order from the coordinator) | 3+2n |
| `0x09` | `GP_PROXY` | `u8 format_version, entry[]` (zigbee-rs `gp_store` entry encoding) | var |
| `0x0A` | `RS_PARAMS` | `u32 fc_stride, u32 table_epoch, u32 nwk_fc_limit, u32 aps_fc_limit, u8 flags (bit0 host-backed tables)` | 17 |
| `0x0B` | `GP_SINK` | `gpd_id` (`u8 app_id`; then `u32 src_id`, or `[8] ieee, u8 endpoint`), `u8 device_id, u8 communication_mode, u16 group (FFFF none), u16 assigned_alias (FFFF derived), u8 flags (bit0 MAC seq capable, bit1 rx-on capable, bit2 fixed location, bit3 key present), u8 security_level, u8 key_type, [16] key (if bit3), u32 frame_counter, u8 groupcast_radius` | 19..40 |
| `0xFF` | `END` | `u32 record_count, u32 crc32c` over the concatenated TLV bytes of all preceding records in the stream | 8 |

`DEVICE.flags`:

| Bit | Meaning |
|---|---|
| 0 | `is_child` |
| 1 | Link block present |
| 2 | Child info present |
| 3 | `nwk` valid |

Child info: `u8 capability, u8 timeout_exp`.

- OCB has no capability field. The host fills it from its device DB (zigpy
  node descriptor). Otherwise it uses `0x80` (allocate address, rx-off) and
  timeout index 8 (256 min). The NCP corrects both on the child's next
  poll or rejoin.

Link block: `[16] key, u8 key_seq, u8 attrs (0 provisional, 1 unverified,
2 verified), u8 origin (0 global, 1 install code, 2 TC generated, 3
imported/host, 4 EZSP-hashed derived), u32 tx_counter, u32 rx_floor`.

`tx_counter` is **informational**, because outgoing APS counters are global
(8.4):

- On export and in `TBL_*` the NCP writes the global durable APS limit.
- On import it only contributes to the global start, through the `max` in
  9.4.

The field keeps OCB's per-device `tx_counter` round-trippable.

Unknown record types are skipped on import, reported as an `IGNORED` note,
and preserved by hosts. Within a major version, records may grow by
appending fields.

### 9.4 Restore (import) commands

| ID | Name | Request payload | Response after `status` |
|---|---|---|---|
| `0x0611` | `BKP_RESTORE_BEGIN` | `u8 flags (bit0 allow IEEE change, bit1 start network after commit), u32 nwk_margin, u32 aps_margin, u32 expected_records` | `u16 txn, u16 max_records_per_put` (`IN_PROGRESS` if a restore is active) |
| `0x0612` | `BKP_RESTORE_PUT` | `u16 txn, records…` (the last PUT ends with `END`) | `u16 accepted, u8 n_notes, n×{u16 index, u8 note}` |
| `0x0613` | `BKP_RESTORE_COMMIT` | `u16 txn` | `u32 nwk_fc_start, u32 aps_fc_start, u16 link_keys_on_ncp, u16 link_keys_host_backed, u16 children` |
| `0x0614` | `BKP_RESTORE_ABORT` | `u16 txn` | — |

Notes: 1 `HOST_BACKED` (key not cached on the NCP; the host must hold it),
2 `CHILD_DEMOTED` (child table full; kept as a non-child), 3 `IGNORED`,
4 `FLOOR_RAISED`.

Rules:

1. `BEGIN` stops the Zigbee network (radio quiet, no beacons) and opens a
   staging area. The previous network stays in NV until `COMMIT`. `ABORT`,
   a session reset or a reboot discards the staging area and restores the
   previous state. Restore requires capability bit 15.
2. Records are validated as they arrive. Violations fail the PUT with
   `INVALID_PARAMETER` and abort the transaction: security level ≠ 5, node
   NWK ≠ `0x0000`, a channel outside 11–26, an all-zero network key, a PAN
   ID of `0x0000` or `0xFFFF`, or an EPID of all-zero or all-ones.
3. **Before `COMMIT`**, the host must have durably stored every `DEVICE`
   record in its own tables. The host is authoritative for host-backed keys.
4. `COMMIT` checks the `END` record (count and CRC-32C) and then applies the
   counter rules:
   - `nwk_fc_start = max(NETWORK_KEY.tx_counter, RS_PARAMS.nwk_fc_limit) +
     nwk_margin`;
   - `aps_fc_start = max(TC_LINK_KEY.tx_counter, max(DEVICE.link.tx),
     RS_PARAMS.aps_fc_limit) + aps_margin`;
   - both reservations are committed before the response.
5. `COMMIT` is **atomic and power-cut safe**: it writes the staging area,
   then a single commit marker. It increments `table_epoch`, emits
   `EVT_TBL_EPOCH_CHANGED`, writes the IEEE override if allowed, and either
   starts the network (bit1) or leaves it for `NWK_INIT`.
6. Margins:
   - The zigpy library passes `nwk_margin = 0`, because zigpy's
     `restore_backup()` already adds 10 000 to the network-key counter, and
     `aps_margin = 10 000`, because zigpy adds nothing to APS counters.
   - **Go hub (decision 13):** the adapter must pass 10 000 / 10 000, and
     the hub's `FrameCounterRestoreMargin` must change from 2 500 to
     10 000. This is a required change in `~/go_zboss-ncp`, to be made in
     that repository.
   - **Unknown TCLK counter.** If the backup has no TCLK frame counter (Z2M
     backups, 9.1), every host passes `aps_margin = 2²⁰` (1 048 576). Its
     APS counter is unknown, and a megaframe margin costs nothing in a
     32-bit space (`ocb_z2m_import.json`).
   - A backup produced by a zigbee-rs NCP already carries durable limits,
     so its margins only need to cover "the old NCP kept running after the
     backup". Hosts must still not restore while it runs (8.4).
7. **IEEE.** If `NODE.ieee` differs from the active IEEE and bit0 is set, the
   NCP stores an override. On ESP32 the factory IEEE comes from the eFuse
   MAC, and the override is a persisted identity item applied to
   `MacExtendedAddress` at boot. It can be repeated or reverted
   (`SYS_SET_IEEE FF…`), so `IEEE_REWRITABLE` is advertised. A product
   without a writable identity item reports `SECURITY_ERROR` unless bit0 is
   clear and the IEEEs match. Hosts then follow zigpy's
   `can_write_network_settings` rules (`DestructiveWriteNetworkSettings`).

`EVT_NCP_READY`, `NWK_GET_INFO` and the export always report the IEEE the
radio actually uses.

### 9.5 Export commands

| ID | Name | Request payload | Response after `status` |
|---|---|---|---|
| `0x0601` | `BKP_EXPORT_BEGIN` | `u8 flags (bit0 keys, bit1 devices, bit2 GP tables)` | `u16 txn, u32 record_count` |
| `0x0602` | `BKP_EXPORT_READ` | `u16 txn, u16 max_bytes` | `u8 more, records…` (the stream ends with `END`) |
| `0x0603` | `BKP_EXPORT_END` | `u16 txn` | — |

- The snapshot is consistent. It is taken under the stack lock in one cycle
  and then streamed from a RAM or flash copy.
- Counters are exported as **durable limits**. The NWK value equals the last
  `EVT_FC_CHECKPOINT`, so a backup never under-reports a counter.
- `DEVICE` records cover children and cached entries. The host merges them
  with its own store: for each device, the NCP record wins if its `rev` is
  newer; otherwise the host's record is used. Every link-key `tx_counter` is
  then set to the exported APS limit.
- Without bit0 (or when export is locked), keys are omitted and the host
  produces an **incomplete** backup. zigpy's `is_complete()` then rejects
  it. The UI must say so instead of storing an unusable backup.

### 9.6 Migration

#### EZSP (Home Assistant Connect ZBT-1) → zigbee-rs ESP32-H2

1. Flash the zigbee-rs NCP firmware on the ESP32-H2. Check
   `SYS_NEGOTIATE`: `BACKUP_RESTORE`, `KEY_EXPORT` and `IEEE_REWRITABLE`
   must be present. `EZSP_HASHED_TCLK` is implemented in v1, but it is
   advertised only after gate H5 has verified it.
2. In ZHA, start *Migrate radio → keep network*. bellows creates a fresh
   backup with `load_network_info(load_devices=True)`. This includes the
   NWK counter, the TCLK, the key table, the child table and the address
   table, and `stack_specific.ezsp.hashed_tclk` when hashed mode is active.
   bellows enables hashed mode on EZSP > v4.
3. **Unplug the ZBT-1.** Two coordinators on the same PAN with the same key
   break frame-counter safety and cause address conflicts.
4. Select the zigbee-rs radio. The zigpy library runs the link handshake,
   `SYS_NEGOTIATE`, then `write_network_info` → `BKP_RESTORE_BEGIN{allow
   IEEE, nwk_margin 0, aps_margin 10000}` → PUT(s) → `COMMIT` → `NWK_INIT`.
5. zigpy calls `load_network_info` and checks the backup compatibility
   (`NetworkBackup.is_compatible_with`). This requires the same coordinator
   IEEE, which is why the IEEE override is needed.
6. Afterwards:
   - All devices keep talking, because the network key and PAN are
     unchanged and counters only moved forward.
   - Restored children are served from the NCP child table. Children beyond
     `max_children` fall back to rejoin; the secured rejoin uses the same
     key.
   - Routes are rebuilt by MTORR and route records within minutes.
7. **Hashed TCLK** (decision 8).
   - **Why it matters.** With hashed mode, devices that completed a TCLK
     update hold a key derived from `hashed_tclk` and their EUI64.
     EmberZNet does not list that key in the key table [Silabs-HLK].
   - **What v1 does.** The zigbee-rs NCP derives these keys on demand
     (7.5), so the devices keep APS-level security with the TC: network-key
     rotation, TCLK verification and APS-encrypted commands keep working.
   - **Other radios.** From their source, zigpy-znp, zigpy-deconz and
     zigpy-zboss ignore `stack_specific.ezsp`, so migrations to those radios
     lose these keys.
   - **Verification.** Gate H5 (14.6) proves the derivation against a key
     read from the user's real ZBT-1 before the capability bit is set.
     Until then the NCP still stores the seed, so a later firmware update
     enables derivation without another migration.
8. **Green Power.** zigpy's GP devices from the EZSP era are imported into
   the NCP Sink Table with `GP_SINK_TABLE_PUT` (7.11, step 7). No GP
   Pairing is sent, and the proxies keep their group-0x0B84 pairings.

#### zigbee-rs → EZSP (ZBT-1)

1. Take a backup through the zigpy library: `BKP_EXPORT` plus host merge.
   - `network_key.frame_counter` and every `tx_counter` are durable limits.
   - `stack_specific.ezsp.hashed_tclk` is kept if it came from EZSP, so
     derived keys stay valid.
2. Unplug the ESP32-H2 and plug in the ZBT-1. bellows' `write_network_info`:
   - writes the NWK counter (zigpy adds 10 000) and the APS counter;
   - imports `key_table` and the child table;
   - forms the network.
3. **IEEE.** If the network originally came from this ZBT-1, its EUI64 already
   matches. Otherwise bellows can only change the EUI64 if the firmware
   supports a custom EUI64, or by a one-time token burn behind an explicit
   opt-in.
4. **Capacity caveats.** The EZSP key-table and child-table sizes limit what
   can be imported. Devices beyond those limits keep working on NWK
   security, but lose their unique TCLK until they rejoin.

#### Other sources

| Source | Notes |
|---|---|
| ZNP (zigpy-znp) | Keys are already explicit in `devices[].link_key`; `tclk_seed` is passed through. For the reverse migration, zigpy-znp recomputes an optimal seed from the key table. |
| deCONZ | Older firmware reports a NWK counter of 2³⁰; restore starts above it, leaving about 3·2³⁰ counters. Link keys are usually absent. |
| ZBOSS (zigpy-zboss) | Key table and NIB counter are supported. TC policies in `stack_specific.zboss` are host pass-through. |
| Z2M / zigbee-herdsman | Same OCB file (`coordinator_backup.json`), with the Z2M quirks of 9.1. A zigbee-herdsman ZRHP adapter is in scope (12.3). Import from a Z2M backup uses the unknown-TCLK-counter margin. Export through herdsman loses `stack_specific.zigbee_rs` unless herdsman's backup utilities are extended (12.3). |
| ZRHP embedded hub | Stores the same binary records. It can export OCB JSON through a streaming writer, or hand the records to a desktop tool that converts them with the same rules (`ocb_roundtrip.json`). |

### 9.7 Round-trip vector

`ocb_roundtrip.json` restores a synthetic EZSP-origin backup into a model
NCP with `nwk_margin = aps_margin = 10 000`. The network has 4 devices: a
router with a key, two children (one without `is_child` and with Z2M-style
`"abc"`), and a key-only device. The vector then exports it again.

| Value | Input | After restore (start / durable limit) | Output OCB |
|---|---:|---:|---:|
| NWK frame counter | 2 000 000 | 2 010 000 / 2 011 024 | 2 011 024 |
| APS counter (TCLK 3 000, keys 0 and 12) | max = 3 000 | 13 000 / 14 024 | 14 024 for the TCLK and every key |

Restore records (TLV hex):

```text
NODE             01 0c00 3c2b1afeffb1b3f4 0000 00 01
NETWORK          02 1400 2a4c 1807f6e5d4c3b2a1 14 00801002 02 0000 05 08
NETWORK_KEY      03 1600 8b5d1f3c6a7e9d2b4f0a1c3e5d7f9b2a 01 80841e00 01
TC_LINK_KEY      04 1d00 5a6967426565416c6c69616e63653039 3c2b1afeffb1b3f4 b80b0000 01
EZSP_HASHED_TCLK 06 1000 0f1e2d3c4b5a69788796a5b4c3d2e1f0
DEVICE           05 0d00 3b2a1f0e006f0d00 bc0a 0d 80 08
DEVICE           05 0d00 e7d6c504008d1500 3e8f 0d 80 08
DEVICE           05 2600 c3b2a10201881700 2b1a 0a 11223344556677881122334455667788 00 02 03 00000000 39000000
DEVICE           05 2600 0c0b0afeff142e84 ffff 02 a0a1a2a3a4a5a6a7a8a9aaabacadaeaf 00 02 03 0c000000 03000000
END              ff 0800 09000000 e4c2ac7a
```

The generator asserts four properties:

1. Converting the output OCB back to records yields byte-identical `DEVICE`
   records.
2. Restoring the output with zero margins starts exactly at the exported
   limits.
3. The `END` CRC matches.
4. The install-code key derivation matches the zigbee-crypto vector.

`ocb_z2m_import.json` covers a Z2M-style backup:

- `pan_id` `"a62"` and NWK `"abc"`, which the host pads;
- `internal.date` and `ezspVersion`;
- no TCLK counter, so the host uses `aps_margin = 2²⁰`;
- NWK counter 77 000 + 10 000 → start 87 000;
- APS start = max(0, 5) + 1 048 576 = 1 048 581.

---

## 10. Performance model

### 10.1 Message and frame sizes

| Item | Bytes |
|---|---:|
| Typical inbound ZCL attribute report (ASDU 10 B) as `EVT_APS_DATA_INDICATION` incl. 7 B message header | 49 |
| Same, alone in one frame, on the wire (8 hdr + 4 CRC + COBS 1 + delimiter 1) | 63 |
| 13 such messages batched in one frame (637 B payload) on the wire | 653 → 50.2 per message |
| Typical outbound `APS_DATA_REQUEST` (NWK mode, 8 B ZCL read) incl. header | 32 (46 on the wire alone) |
| `ACK`/`NAK` without credits on the wire | 20 |

### 10.2 Link capacity (one direction, messages/s)

UART (10 bits/byte):

| Link | Bytes/s | Indications/s unbatched | Indications/s batched |
|---|---:|---:|---:|
| 460 800 baud (Thread BR board default) | 46 080 | 731 | 917 |
| 921 600 baud | 92 160 | 1 463 | 1 835 |
| 2 000 000 baud | 200 000 | 3 175 | 3 982 |
| 3 000 000 baud | 300 000 | 4 762 | 5 972 |

USB-Serial-JTAG (no measurement on H2 or C6 yet; three planning tiers):

| Tier | Bytes/s | Indications/s unbatched | Indications/s batched |
|---|---:|---:|---:|
| Worst case (one 64 B packet per 1 ms frame) | 64 000 | ≈ 1 016 | 1 274 |
| Planning | 300 000 | 4 762 | 5 972 |
| Optimistic (≈ 1 MB/s seen on C3/S3 USJ with ≥ 10 B writes [asukiaaa]) | 1 000 000 | 15 873 | 19 908 |

The full-duplex reverse direction carries the ACKs. With the delayed ACK,
there is at most one 20-byte ACK per `ack_delay`, which is ≤ 20 KB/s. That
is 7 % of the reverse capacity at 3 Mbaud and up to 22 % at 921 600 baud with
unbatched small frames. ACKs compete only with host → NCP traffic, which is
light for a coordinator, and they never reduce the forward capacity. A host
that sends heavily can raise `ack_delay` to 2–5 ms.

### 10.3 The radio is the bottleneck

A typical NWK+APS-secured unicast is a 55-byte MPDU, or 61 bytes with the PHY
header. One transmission takes about 4.4 ms of airtime:

| Component | Time |
|---|---:|
| Frame on air | 1.95 ms |
| Turnaround | 0.19 ms |
| MAC ACK | 0.35 ms |
| LIFS | 0.64 ms |
| Average initial CSMA backoff + CCA | 1.25 ms |

That gives at most about **228 frames/s per channel**, from a single
transmitter, with no retries and no relaying. Multi-hop relaying and
broadcasts (three transmissions each) cut this substantially.

A 1000-device network therefore produces the following load:

| Load | Reports/s | Air utilisation near the coordinator | Link utilisation at 460 800 (BR board) | at 921 600 (63 B frames) | at USJ worst case |
|---|---:|---:|---:|---:|---:|
| 1 report / 5 min / device | 3.3 | 1.5 % | 0.5 % | 0.2 % | 0.3 % |
| 1 report / 60 s / device | 16.7 | 7.3 % | 2.3 % | 1.1 % | 1.6 % |
| 1 report / 10 s / device | 100 | 44 % (saturating in practice) | 13.7 % | 6.8 % | 9.8 % |
| Air-limit burst | ~228 | 100 % | 31 % | 15.6 % | 22 % |

Even the slowest configuration, the BR board's 460 800 baud without flow
control, has more than 3 times the serial capacity the radio can produce. ZRHP's speed requirement is therefore about:

- **latency under concurrency**: many requests in flight, no lock-step;
- **burst absorption**: after a power outage, every device rejoins,
  announces and reports at once;
- **bulk transfers** that do not disturb the data plane:
  - backups: 1000 devices with keys ≈ 41 KB, 0.45 s at 921 600;
  - NCP firmware: 1 MiB ≈ 23 s at 460 800, 11.6 s at 921 600, 3.6 s at
    3 Mbaud, up to 17 s at the USJ worst case;
- **table paging** on the critical path of secured traffic (8.6).

### 10.4 Latency budget (host request → NCP → first transmission)

| Step | UART 3 M | UART 921 600 | USJ |
|---|---:|---:|---:|
| Host write syscall + driver | 0.05–0.2 ms | same | same |
| 46 B request on the wire | 0.15 ms | 0.5 ms | ≤ 1 ms (next USB frame) |
| NCP RX ISR + COBS/CRC + dispatch | < 0.1 ms | < 0.1 ms | < 0.1 ms |
| `SUCCESS` (accepted) back at the host | ≈ 0.5 ms | ≈ 1.2 ms | 2–3 ms |
| APS/NWK encode + AES-CCM* (estimate) | 0.2–0.5 ms | same | same |
| CSMA + air + MAC ACK (1 hop) | 2.5–6 ms | same | same |
| `EVT_APS_DATA_CONFIRM` (MAC/NWK delivery, no APS ACK) | ≈ 4–8 ms | ≈ 5–9 ms | ≈ 6–11 ms |
| With an APS ACK across a 2–4 hop mesh | 20–200 ms | same | same |

### 10.5 How the window and batching hide host latency

- Throughput is decoupled from round-trip time as long as `W × MTU ≥ rate ×
  RTT`.
- A Python host with 30 ms scheduling hiccups, receiving 5 000
  indications/s at 3 Mbaud, needs about 150 messages in flight. That is
  12 batched frames, which fits within `W = 16`.
- Batching is automatic. When the transmitter or the host is slow, the
  queues fill and the next frame carries more messages, so overhead per
  message falls as load rises.

Compared with stop-and-wait links (ASH as used by bellows, ZBOSS LL) plus
one command at a time (EZSP, MT SREQ), ZRHP removes the per-command RTT. At
921 600 baud a stop-and-wait exchange costs ≥ 2 frame times + ACK delay +
host wake-up, typically 3–15 ms. That limits such hosts to roughly 70–300
commands/s regardless of baud rate.

---

## 11. First target: ESP32-H2 / ESP32-C6

### 11.1 USB-Serial-JTAG (USJ) facts

Sources: [ESP-TRM], [IDF-USJ], [esptool], [esp-hal].

- **Hardware.** Full-speed, hard-wired CDC-ACM plus JTAG. The bulk IN and
  bulk OUT endpoints use 64-byte packets, each with **one 64-byte buffer**.
  TX is flushed on the 64th byte or by `WR_DONE`, and the buffer cannot be
  refilled until the host has read it.
- **Flow control.** No RTS/CTS. Line coding is ignored, and flow control is
  USB NAK only.
- **DTR/RTS reset logic** (TRM table "USB-Serial-JTAG reset"):

  | RTS | DTR | Action |
  |---|---|---|
  | 0 | 0 | Clear download flag |
  | 0 | 1 | Set download flag |
  | 1 | 0 | **Reset the chip** |
  | 1 | 1 | None |

  `USB_SERIAL_JTAG_USB_UART_CHIP_RST_DIS` disables this logic.
- **Reset and re-enumeration.** A chip reset makes the device disappear from
  the bus and re-enumerate. esptool waits for this.
- **Sleep.** Deep sleep powers USJ down; light sleep makes it unresponsive.
  The NCP never sleeps.
- **esp-hal** (repository pin: `esp-hal = "=1.0.0"`, vendored at
  `vendor/esp-hal/src/usb_serial_jtag.rs`):
  - Blocking `write()` writes bytes and then **spins forever** waiting for
    the host. It also writes before checking `serial_in_ep_data_free`,
    which loses data when the FIFO is already full, for example with ROM
    output left at boot (upstream issue #6587).
  - `write_byte_nb` checks first.
  - Async `write_async` awaits `SERIAL_IN_EMPTY`.
  - There is no host-connected (SOF) detection (upstream issue #6787), and
    no timeout.

### 11.2 Constraints these impose on the NCP

1. **Never block the Zigbee stack on USB.**
   - The NCP's USJ transport is a non-blocking pump that feeds a TX ring
     with `write_byte_nb` and flushes with `flush_tx_nb` (or an
     interrupt-driven equivalent).
   - The pump runs from the main loop or ISR and never from inside stack
     code.
   - A full FIFO simply leaves bytes in the ring. ARQ and credits keep the
     ring bounded (TX window × 656 B).
   - In `UNSYNCED` state with a full ring, `RESET_REQ` announcements are
     discarded and counted (`tx_discarded_no_host`).
2. **Host-absence detection.** The link layer infers host absence: no RX and
   no TX-FIFO progress for `link_timeout_ms` means link down (5.11).
   `DTR_CHG`/`RTS_CHG` interrupts are an additional hint that a host opened
   or closed the port.
3. **Frame end flush.**
   - After the last byte of a burst the NCP issues `WR_DONE` so the host
     receives a short packet.
   - If the total since the last short packet is an exact multiple of 64,
     the NCP appends one extra `0x00`. That is an empty COBS frame, which
     is a no-op. It avoids depending on ZLP behaviour.
4. **Stale bytes at boot.**
   - The NCP drains and discards the TX FIFO state left by the ROM and
     bootloader before the first frame.
   - Production NCP images must disable ROM and bootloader console output
     on USJ, or route it to UART0. This is a product setting.
   - Leftover text is harmless anyway: it contains no valid COBS frame with
     a valid CRC and epoch, and the leading `0x00` resynchronizes.
5. **Logs.**
   - Firmware logs go only to the `LOG` endpoint.
   - `esp-println` on USJ, which the sensor examples use today, must be
     compiled out of NCP images that use USJ as the ZRHP transport.
   - A product build check must fail if both the ZRHP USJ transport and
     `esp-println`'s jtag-serial backend are linked.
   - A UART-transport image (for example on the Thread BR board) may keep a
     USJ debug console, because USJ then carries no ZRHP.
6. **Credits replace RTS/CTS.**
   - Host → NCP: the NCP drains the 64 B OUT buffer from the
     `SERIAL_OUT_RECV_PKT` ISR into an RX ring of at least `rx_window × 656`
     bytes (10.3 KiB for window 16). Because the host respects window and
     credits, the ring cannot overflow. If it ever would, the excess is
     counted as `rx_overflow` and recovered by NAK.
   - USB NAK back-pressure stalls the host's write rather than losing
     bytes.

### 11.3 Host rules for USJ ports

- **Port identity.** Open the port by a stable identity, not by node name.
  Match the Espressif VID/PID, which ZRHP keeps (decision N8). It is
  believed to be `303a:1001`. Also match the USB serial number. Both values
  are **unverified for H2/C6**; gate H1 confirms them. After an
  NCP reset, expect the port to disappear and come back, possibly as a new
  `/dev/ttyACM*` or `/dev/cu.usbmodem*`. Reopen it and redo the handshake.
- **DTR/RTS handling.**
  - Never leave `RTS=1, DTR=0`, because that resets the chip.
  - Never set `RTS=0, DTR=1` except to enter download mode on purpose.
  - Change both lines in one ioctl (`TIOCMSET`) where the OS allows it.
  - The normal states are `(1,1)` and `(0,0)`.
  - Opening a port on Linux or macOS raises both lines together, and
    closing (HUPCL) drops both together. Neither action resets the chip.
    pyserial's separate `dtr`/`rts` setters can pass through a transient
    state, so the zigpy library must set both lines with one ioctl.
  - The Windows behaviour differs (esptool notes) and must be tested.
- **Hard reset** (decision 6).
  - The USJ DTR/RTS reset logic stays **enabled** in production NCP images:
    `USB_SERIAL_JTAG_USB_UART_CHIP_RST_DIS` is not set.
  - A deliberate hard reset of the NCP is `(0,0) → (1,0) for ≥ 10 ms →
    (0,0)`.
  - Host libraries use it as a recovery step when the link stays down past
    the resume budget (12.1).
  - Because any host-side tool that toggles the lines can reset the NCP,
    hosts must follow the DTR/RTS rules above. The NCP reports the resulting
    reset as `reset_reason` 2/7 in `EVT_NCP_READY`.
- **Flow control settings.** Disable `rtscts` and `dsrdtr`. Flow control is
  ZRHP credits and USB NAK.

### 11.4 UART variant (v1, decision 7)

The UART transport ships in v1 next to USJ:

- **ESP32-C6:** UART with RTS/CTS up to 3 Mbaud (5 Mbaud hardware limit),
  behind an external USB-UART bridge or directly to an embedded host.
- **ESP32-H2:** about 3 Mbaud ceiling from the 48 MHz clock; the measured
  maximum is gate H2.
- **Defaults:** **921 600 baud** with RTS/CTS behind a USB-UART bridge (for
  example CP2102N or CH343). The ESP32-H2 devkit's on-board bridge serves
  programming; its DTR/RTS drive EN/BOOT through the auto-program circuit,
  not UART flow control. A flow-controlled UART link therefore uses
  dedicated pins described by the board crate.
- **RX path:** UHCI GDMA, or an IRAM interrupt, into a ring of at least
  `rx_window × 656` bytes. This is required without RTS/CTS (4.1) and
  recommended with it.
- **Product:** selects USJ or UART at build time (`transport-usj`,
  `transport-uart`). Only one ZRHP transport is active per image.
- **Board crate:** describes the UART pins and whether RTS/CTS are wired.
  This drives `SYS_GET_INFO.uart_flow` and capability 22.

### 11.5 Espressif Thread Border Router / Zigbee Gateway board

Sources: [ESP-TBR-HW] and the board schematic v1.3 [ESP-TBR-SCH].

**The board.**

| Part | Details |
|---|---|
| Host | ESP32-S3-WROOM-1 (8 MB flash, 2 MB PSRAM). Early samples have **4 MB** flash (Espressif's ordering table; the hwlab board, measured); see 3.4. |
| NCP | ESP32-H2-MINI-1 (2 MB `-H2S` or 4 MB `-H4S` flash). The variant on the user's board is **unknown**; see "Flash size" below. |
| USB | Two USB-C ports, each wired to its SoC's USB-Serial-JTAG |
| Ethernet | Optional W5500 daughter board on S3 SPI2 (hub-level, irrelevant to ZRHP) |

**Host ↔ NCP wiring** (S3 side ↔ H2 side):

| Signal | ESP32-S3 | ESP32-H2 | Use in ZRHP |
|---|---|---|---|
| UART TX → RX | GPIO18 | GPIO23 (U0RXD) | ZRHP byte stream, host → NCP |
| UART RX ← TX | GPIO17 | GPIO24 (U0TXD) | ZRHP byte stream, NCP → host. Also carries the H2 ROM boot log at 115 200 after every reset; the framing resynchronizes. |
| RCP RESET | GPIO7 | EN | Host-controlled NCP reset |
| RCP BOOT | GPIO8 | GPIO9 (strapping) | Host-controlled ROM download mode |
| PTA request | GPIO12 | GPIO0 | Coexistence (optional) |
| PTA priority | GPIO10 | GPIO2 | Coexistence (optional) |
| PTA grant | GPIO13 | GPIO1 | Coexistence (optional) |
| PTA tx_line | GPIO11 | GPIO3 | Coexistence; the S3 needs the 4th wire |
| Reserved (R18, 0 Ω) | GPIO6 | GPIO12 | Unused by ZRHP v1 |
| Reserved (R19, 0 Ω) | GPIO5 | GPIO4 | **Optional H2 RTS → S3 CTS** |
| Reserved (R20, 0 Ω) | GPIO4 | GPIO5 | **Optional S3 RTS → H2 CTS** |

**Flow control on this board.**

- **No dedicated RTS/CTS lines.** The documentation lists none, and the
  board's default RCP link runs TX/RX only at 460 800.
- **ZRHP default:** 460 800 baud, no hardware flow control. Credits bound
  each direction to `window × 656` bytes.
- **Required on both sides:** a DMA or IRAM RX path (4.1).
  - NCP: zigbee-rs UART driver with UHCI RX (new HAL work).
  - S3 hub: ESP-IDF UART driver with `CONFIG_UART_ISR_IN_IRAM` or UHCI,
    with the ring buffer in internal RAM.
- **921 600 without flow control** is allowed after gate H8 shows no
  overflow while both chips write flash.
- **Default** (decision N4): 460 800 without flow control. The DMA/IRAM RX
  path above is **mandatory** on both sides.
- **Hardware flow control: optional board modification.** This is
  documented, not a default. The schematic's three populated "Reserved"
  0 Ω links (R18–R20) can carry RTS/CTS, routed through both chips' GPIO
  matrices:
  - H2 `U0RTS` → GPIO4 → S3 GPIO5 as `U1CTS`;
  - S3 `U1RTS` → GPIO4 → H2 GPIO5 as `U0CTS`.

  This is enabled only when **both** images are built for it (board
  feature `rtscts-reserved-lines`), and up to 2 Mbaud then applies (4.4).
  Points to verify on the physical board (gate H8): R18–R20 are fitted on
  the delivered revision; H2 GPIO4/5 are the pad-JTAG MTMS/MTDI pins, and
  using them as GPIO disables pad JTAG while USB-JTAG still works.
- **Mismatch handling.** A misconfigured pair (only one side using CTS) can
  stall. The hub checks `SYS_GET_INFO.uart_flow` at session start and
  disables its CTS if the NCP reports none.

**Flash size** (decision N4). Both partition tables of 8.9 (2 MB and 4 MB)
are kept, and the size is detected at runtime:

- **Provisioning.** The hub reads the H2's JEDEC flash ID through the ROM
  loader before the first write, as esptool's `flash_id` does. It then
  writes the bootloader, partition table and image built for that size.
- **NCP.** At boot the NCP reads its flash ID and reports the size in
  `SYS_GET_INFO.flash_size`. If the detected size is smaller than the end of
  its partition table (a 4 MB image on a 2 MB part), the NCP refuses to
  start the stack. It still answers `SYS_*`, raises `EVT_FATAL` (kind
  "layout mismatch") and waits for a corrected image.
- **Gate H8** checks which variant the user's board carries once it is on
  hwlab.

**NCP firmware update by the S3 host.**

1. **Normal path: ZRHP `FW_*` over BULK.**
   - The signed image is staged in the passive slot, while the network keeps
     running until `FW_ACTIVATE`.
   - Trial boot, then `FW_CONFIRM` from the hub after a successful
     `NWK_INIT`, with bootloader rollback otherwise.
   - **Rollback lives on the H2** (decision N6, amended 2026-11-18): A/B
     application slots (8.9). The previous image stays in its slot until a
     new image is confirmed; an unconfirmed image (trial window expired,
     crash or no session) makes the H2 bootloader boot the previous slot,
     and the hub sees `EVT_NCP_READY` with the old version and reports the
     failed update.
   - **The S3 stores no NCP images.** The hub streams an image from its
     source (hub API upload or download) straight into `FW_WRITE` chunks.
     Interrupted transfers resume at `next_offset` (idempotent writes) or
     end with `FW_ABORT`. Images larger than the H2 slot are refused
     (`TOO_LARGE`, 7.9).
2. **Recovery path: H2 ROM serial bootloader — manual / development
   only.** Used when no ZRHP session is possible (blank or corrupt NCP,
   major-version mismatch, layout mismatch) and an operator supplies the
   image: through a host or user action, for example the hub API or a
   desktop over the phase-0 USB bridge. The hub never runs it on its own and
   keeps no image for it.
   1. Drive BOOT (S3 GPIO8) low.
   2. Pulse RESET (S3 GPIO7) low for at least 10 ms and release it.
   3. Wait 50 ms, then release BOOT.
   4. Speak the ESP ROM serial protocol on the same UART: SLIP frames,
      `SYNC` at 115 200, `CHANGE_BAUDRATE` to 460 800, `FLASH_BEGIN/DATA/END`
      with MD5 verification. This is what `esp-serial-flasher` does.
   5. Release BOOT high and pulse RESET for a normal boot.

   Rules for the ROM path:
   - **Write only:** the bootloader, the partition table (first provisioning
     only), an application slot and `otadata`.
   - **Never write** the `zbnv` range (9.4 layout), so network state, keys
     and counters survive.
   - **Secure boot.** With secure boot enabled, the ROM path writes only
     images signed for that key.
   - **Downgrades.** An image below the NV format version fails closed (8.4).
   - After a ROM-path boot the hub runs the normal resume. `EVT_NCP_READY`
     carries `reset_reason = 8`.
   - The NCP's own A/B rollback does not cover the ROM path: it writes a
     slot and `otadata` directly.
3. **ROM boot log.** The ROM prints a boot log on U0TXD at 115 200 after
   every reset. The hub ignores it; it is not valid COBS/CRC framing.
   Production images may suppress it with the `UART_PRINT_CONTROL` eFuse
   (product decision).

**PTA coexistence (Wi-Fi ↔ 802.15.4).**

- **Hardware.** The S3 Wi-Fi and the H2 radio share 2.4 GHz. The board wires
  4-wire PTA. The S3 is the coexistence leader: its Wi-Fi arbitrates,
  configured on the S3 with ESP-IDF's external-coexistence GPIO API. The H2
  is the follower. Espressif disables it by default
  (`EXTERNAL_COEX_ENABLE`).
- **NCP follower behaviour (new MAC work, N).** The zigbee-rs ESP MAC has no
  external-coexistence support today. In v1 it implements:
  - assert *request*, and *priority* when the frame class is in
    `coex_tx_priority_mask`, before every CSMA-CA attempt;
  - transmit only on *grant*, otherwise back off as a busy channel and count
    `coex_denied`;
  - drive *tx_line* for the duration of each TX.
- **Frames that cannot wait for grant.**
  - MAC ACKs are sent by the radio's automatic ACK path; priority is
    asserted around expected ACK windows.
  - The GP TempMaster reply, a timed TX at +20 ms armed in the radio,
    requests early with high priority. If grant is denied at the
    transmission instant, the reply is reported `Missed` and counted. The
    GPD retries commissioning.
- **Enabling it** (decision N7).
  - PTA is **disabled by default**. `coex_enable` (config `0x0009`) is
    volatile and reads 0 after every NCP boot.
  - The hub enables it **automatically while its Wi-Fi is active**, and
    disables it when Wi-Fi stops, for example on Ethernet-only operation
    with the W5500. It re-applies the current state in every resume
    replay (12.4).
  - Capability 21 is set only when the board crate declares the PTA
    wiring.
- **Validation.** Gate H9 (14.6) measures PER and latency with Wi-Fi load,
  with coexistence on and off.

---

## 12. Host integration

### 12.1 zigpy radio library (`zigpy-zrhp`, radio type `zrhp`)

The library and the zigpy GP hook are maintained in `faronov` forks
(decision N2). The library must also run on stock zigpy, falling back to the
shadow-sink steps of 7.11.

| zigpy `ControllerApplication` | ZRHP |
|---|---|
| `connect()` | Open the port (stable identity, 11.3), link handshake, `SYS_NEGOTIATE`, wait for `EVT_NCP_READY`. Must be side-effect free on failure (used by `probe()`). |
| `disconnect()` | Close the port. No NCP reset. |
| `start_network()` | `NWK_INIT{floors from the checkpoint log}`, `TBL_SYNC`, register endpoints (`NWK_ADD_ENDPOINT` for each `add_endpoint`), `NWK_GROUP_MEMBERSHIP` resync, `SEC_SET_TC_POLICY`, `GP_SINK_CONFIG{NCP sink, pre-commissioned groupcast 0x0B84}` and the GP shadow reconciliation (7.11 step 7), create the coordinator device. Set `_concurrent_requests_semaphore.max_concurrency = max_aps_inflight`. |
| `load_network_info(load_devices)` | `NWK_GET_INFO` + `SEC_GET_NETWORK_KEY`. With `load_devices`: `BKP_EXPORT_*` merged with the host store, filling `network_info.children`, `nwk_addresses`, `key_table` (with `tx_counter = aps limit`, `rx_counter = floor`, `seq`), `tc_link_key`, `stack_specific.zigbee_rs` and `ezsp.hashed_tclk`, `metadata.zigbee_rs`. `NOT_FORMED` ⇒ `NetworkNotFormed`. |
| `write_network_info(network_info, node_info)` | `BKP_RESTORE_*` (9.4) with `nwk_margin = 0`, `aps_margin = 10000`, `allow_ieee_change = IEEE_REWRITABLE`. `form_quickly` / new networks produce the same records with zero counters. |
| `reset_network_info()` | `NWK_LEAVE_RESET`. |
| `send_packet(packet)` | `APS_DATA_REQUEST`; `BUSY` ⇒ retry with backoff inside the concurrency limit. Wait for `EVT_APS_DATA_CONFIRM` (timeout 3 s mains / 8 s with `extended_timeout`, same as bellows). Non-success ⇒ `DeliveryError`. `packet.source_route` ⇒ relays (zigpy already reverses `device.relays`). ZDO requests to the coordinator itself are answered locally from `DIAG_*`/`NWK_GET_INFO` (7.4). GP sink transmissions are swallowed in NCP-sink mode (7.11 step 4). |
| `permit_ncp(time_s)` | `NWK_PERMIT_JOIN{target 0x0000}` |
| `permit(time_s, node)` | `NWK_PERMIT_JOIN{target 0xFFFC or the router}` + `GP_SINK_COMMISSIONING{enter, involve proxies, time_s}` |
| `permit_with_link_key(node, key, time_s)` | `SEC_ADD_LINK_KEY{temporary}` + permit; or `SEC_ADD_INSTALL_CODE` when the host has a code |
| `force_remove(dev)` | `NWK_REMOVE_DEVICE` + `TBL_INVALIDATE`; for a GP device `GP_SINK_REMOVE{send RemoveGPD}` |
| `add_endpoint(desc)` | `NWK_ADD_ENDPOINT` |
| `energy_scan(channels, duration_exp, count)` | `NWK_ENERGY_SCAN` → `EVT_ENERGY_SCAN_RESULT` |
| `_move_network_to_channel(ch, update_id)` | `NWK_CHANGE_CHANNEL` |
| `_set_tx_power(dbm)` | `NWK_SET_TX_POWER` |
| `_watchdog_feed()` | `SYS_ECHO`, plus `DIAG_GET_COUNTERS` every 30 s into `state.counters` |
| `_network_scan` | `NWK_ACTIVE_SCAN` |
| `_subscribe_to_multicast_group` | `NWK_GROUP_MEMBERSHIP` |

Callbacks into zigpy:

| ZRHP event | zigpy call |
|---|---|
| `EVT_APS_DATA_INDICATION` | `packet_received(ZigbeePacket{src, src_ep, dst, dst_ep, profile_id, cluster_id, data, lqi, rssi, tsn from the ZCL/ZDP payload})` |
| `EVT_DEVICE_JOINED` (admitted) | `handle_join(nwk, ieee, parent_nwk)` |
| `EVT_DEVICE_LEFT` | `handle_leave(nwk, ieee)` |
| `EVT_ROUTE_RECORD` | `handle_relays(nwk, relays)` (same order as zigpy's `relays`) |
| `EVT_GP_FRAME` | `gp_packet_received(ZigbeeGpPacket{...})` (7.11 step 3) |
| NCP restart or link loss | **Seamless resume** (below). `connection_lost(exc)` only if reconciliation fails. |

**Seamless resume** (decision 5). The library hides an NCP reboot or a
transient link loss from zigpy whenever the NCP state can be reconciled:

| Step | Action | Failure ⇒ |
|---|---|---|
| 1. Detect | One of: `PEER_RESTARTED` (new NCP link cookie); `LINK_DOWN`; the serial port disappearing (USJ re-enumeration). | — |
| 2. Hold | New `send_packet` calls wait, bounded by zigpy's concurrency limit, within the **resume budget** (default 20 s; 60 s while an NCP firmware update is activating). In-flight requests: after `PEER_RESYNC` they are re-sent with `RETRY` (6.5); after a restart they fail with `DeliveryError("NCP restarted, delivery unknown")`. | — |
| 3. Reconnect | Reopen the port by stable identity (11.3). Handshake, then `SYS_NEGOTIATE`. A newer firmware is fine; a different major version is not. | `connection_lost` |
| 4. Reconcile | `EVT_NCP_READY` and `NWK_GET_INFO` must match zigpy's `state.node_info` / `state.network_info`: IEEE, PAN ID, EPID, channel, security level, `nwk_update_id` ≥ known, key sequence ≥ known. `network_state` must be formed (1/2) and `table_epoch` must equal the sidecar's. A higher key sequence or update ID (rotation or channel change that completed during the outage) is accepted and copied into zigpy's state. | `connection_lost` (zigpy re-initializes and may restore from backup) |
| 5. Replay | `NWK_INIT` with checkpoint floors (if not up). Then: `SEC_SET_TC_POLICY`, `NWK_ADD_ENDPOINT` ×n, `NWK_GROUP_MEMBERSHIP`, `NWK_SET_TX_POWER`, `NWK_CONCENTRATOR`, `NWK_SET_MANUFACTURER_CODE`, `GP_SINK_CONFIG`, the open permit-join and GP-commissioning remainders, `SEC_LOCK_KEY_EXPORT` if the user configured the lock, `TBL_SYNC`, and the GP shadow check. | `connection_lost` |
| 6. Release | Resume queued calls. Log the event and increment `state.counters["zrhp"]["ncp_resumed"]`. If the port did not come back within the budget, try the USJ hard reset (11.3) once before giving up. | `connection_lost` |

**Sidecar store** (decision 4).

- **Location:** SQLite file `zigbee_zrhp.db` in the same directory as
  zigpy's `database_path` (`zigbee.db`). It is opened with WAL and
  `synchronous=FULL`. HA backups of the configuration directory include it.
- **Tables:**

  | Table | Contents |
  |---|---|
  | `link_keys` | `ieee` PK, key, type, attrs, origin, key_seq, rx_floor, rev, updated |
  | `fc_checkpoints` | domain, key_seq, limit, generation, time |
  | `meta` | ncp_ieee, table_epoch, host_db_epoch, host_max_rev, last_ncp_cookie |
  | `gp_shadow` | R_GP_SINK records last seen |

- **Rollback detection.** Restoring an older HA backup rolls the sidecar
  back. `TBL_SYNC` detects this through `host_max_rev` (8.5).
- **Paging.** The library always sets `HOST_TABLES`. Standalone versus
  paging is decided by the NCP (8.1), so small networks never page.
- **Key export.** The library does not lock it by default, because HA's
  periodic backups need keys (7.5). A configuration option enables
  `SEC_LOCK_KEY_EXPORT` after each successful backup.

### 12.2 Go hub adapter (`adapter/zrhp`, adapter #8)

The ZRHP adapter has the same layers as the existing adapters:

```text
transport (serial) → link (port of zigbee-zrhp) → message layer → adapter
```

Differences from the existing adapters:

- One reader goroutine and one writer goroutine.
- A pending map keyed by `(msg_id, tid)` instead of a single slot.
- Indications are dispatched from a separate goroutine, not on the reader.
- No session retirement on a single timeout.

Required methods:

| `zigbee.Adapter` | ZRHP |
|---|---|
| `Start()` / `Stop()` | Handshake + `SYS_NEGOTIATE` / close |
| `FormNetwork(cfg)` | `NWK_FORM{cfg.ChannelMask, PanID, ExtPanID, key from NetworkKeyProvider if set}` |
| `StartNetwork()` | `NWK_INIT`; `NOT_FORMED` ⇒ wrapped `ErrNetworkNotFormed`; transport errors are returned as-is (`manager.go` contract) |
| `PermitJoin(d)` | `NWK_PERMIT_JOIN{target 0xFFFC, TC significance}` |
| `SendData(req)` | `APS_DATA_REQUEST`. `AddrMode` 0/2 → `0x02`, 1 → `0x01`, 3 → `0x03`. Returns on `SUCCESS` (accepted), matching current hub semantics. The confirm is exposed through `UnconfirmedCommandSender`. |
| `GetVersion()` | `fw` from `SYS_NEGOTIATE` |
| `GetNetworkInfo()` | `NWK_GET_INFO` |
| `ReadAttribute(...)` | Generic: ZCL over `SendData` + indication waiter |
| `Bind` / `Unbind` | ZDP `Bind_req` / `Unbind_req` over `APS_DATA_REQUEST` (profile 0), wait for the response indication |
| `LeaveRequest(...)` | `NWK_REMOVE_DEVICE` |
| `SetManufacturerCode` | `NWK_SET_MANUFACTURER_CODE` |
| `SetIndicationHandler` | — |

Optional interfaces:

| Interface | ZRHP |
|---|---|
| `GroupableAdapter`, `CoordinatorGroupTableProvider/Clearer` | `NWK_GROUP_MEMBERSHIP` |
| `NetworkKeyProvider` | `SEC_GET_NETWORK_KEY`, `NWK_FORM` key |
| `BackupExporter`, `SnapshotBackupExporter`, `BackupImporter`, `BackupRestorer`, `LinkKeyReader`, `LinkKeyImporter` | `BKP_*`, `SEC_*_LINK_KEY` |
| `NeighborTableProvider`, `RouteTableProvider` | `DIAG_GET_NEIGHBORS`, `DIAG_GET_ROUTES` |
| `TxPowerProvider` | `NWK_SET_TX_POWER` |
| `LinkHealthReporter`, `TransportCounterReporter` | Link counters (5.12) |
| `GreenPowerAdapter` | v1 uses `HOST_SINK` mode (7.11): `GPCommissioningMode` sends the hub's own GP Proxy Commissioning Mode broadcast, which the NCP's local proxy also receives. `GPSetSharedKey` → `GP_SET_SHARED_KEY`. `NCP_SINK` mode is on the roadmap after v1 (decision N2). |
| `RuntimeCapabilityReporter` | `RuntimeCapabilityStatus{Negotiated, ProtocolMajor/Minor, BuildID, CapabilityBits = ncp_caps, MaxPayload = max_asdu}`, which matches `SYS_NEGOTIATE` one-to-one |
| `InterviewableAdapter` | Not needed: ZDP passes through raw, so the generic interviewer works |

Indication mapping:

| ZRHP | Go `IndicationType` |
|---|---|
| `EVT_DEVICE_JOINED{association}` | `DeviceJoined` |
| `EVT_DEVICE_JOINED{rejoin}` | `DeviceRejoined`, **with IEEE** (fixes the ZBOSS IEEE lookup) |
| `EVT_DEVICE_AUTHORIZED` | `DeviceAuthorized` |
| `EVT_DEVICE_LEFT` | `DeviceLeft` (sets `Rejoin`) |
| `EVT_ADDRESS_CHANGED` | `AddressUpdate` |
| `EVT_LINK_KEY_UPDATED` | `TCKeyUpdate` |
| `EVT_KEY_FAILURE` | `TCKeyFailed` |
| `EVT_PAN_ID_CONFLICT` | `PanIDConflict` |
| `EVT_APS_DATA_INDICATION` | `DataReceived`. ZDO responses arrive with `ClusterID \| 0x8000` and TSN at `Data[0]`, as the generic interviewer expects. |
| New NCP cookie | `NCPReset` |

The hub's `Indication` struct cannot carry RSSI, group, destination mode,
security flags or the source IEEE on data frames. Exposing these needs
hub-side work.

Auto-detect: add `zrhp` to the probe order. The probe sends `00` +
`RESET_REQ` and expects a valid `RESET_ACK` within 500 ms. Other NCPs ignore
the bytes, because they contain no valid frame of their protocol.

**Required hub-side changes** (to be made in `~/go_zboss-ncp`, not here):

1. **Restore margin (decision 13).** `FrameCounterRestoreMargin` changes
   from 2 500 to **10 000**, and the adapter passes `nwk_margin = aps_margin
   = 10 000` (2²⁰ for APS when the backup has no TCLK counter).
2. **Resume.** The adapter implements the same seamless resume as zigpy
   (12.1). It raises `IndicationNCPReset` (→ `restartNetwork`) only when
   reconciliation fails.
3. **Richer indications.** Carrying RSSI, group, security flags and source
   IEEE requires extending `zigbee.Indication`.

### 12.3 zigbee-herdsman / Zigbee2MQTT adapter (decision 14)

**Placement.** A herdsman adapter `src/adapter/zrhp/` (TypeScript) in a
**maintained fork** of zigbee-herdsman, with Zigbee2MQTT pointed at the fork
(decision N5). No upstream PR is planned for v1. The adapter contains:

- adapter type `zrhp` in `TsType.Adapter`;
- USB discovery by the USJ VID/PID and product string once they are
  verified (11.3);
- a TypeScript port of `zigbee-zrhp`'s link and message layers, checked
  against the shared vectors;
- serial options: `path`, `baudRate` (UART: default 921 600), `rtscts`.

There is no network path (decision of 2026-11-18).

| herdsman `Adapter` | ZRHP |
|---|---|
| `start(): StartResult` | Handshake, `SYS_NEGOTIATE`, `NWK_INIT`, `NWK_GET_INFO`. See the start-result rules below. |
| `stop()` | Close the port (no NCP reset) |
| `getCoordinatorIEEE()` | `NWK_GET_INFO.ieee` (big-endian `0x…` string) |
| `getCoordinatorVersion()` | `{type: "zigbee-rs ZRHP", meta: {fw, build, chip, major, minor}}` from `SYS_NEGOTIATE` |
| `reset("soft")` / `reset("hard")` | `SYS_RESET{mode 0}` / USJ DTR/RTS hard reset (11.3); on UART-only links, `SYS_RESET` |
| `supportsBackup()` | `BACKUP_RESTORE && KEY_EXPORT && !locked` |
| `backup(ieeeAddressesInDatabase)` | `BKP_EXPORT_*` merged with the sidecar store, filtered to the IEEE list herdsman passes, converted to herdsman's internal `Models.Backup` |
| `getNetworkParameters()` | `NWK_GET_INFO` → `{panID, extendedPanID, channel, nwkUpdateID}` |
| `addInstallCode(ieee, key, hashed)` | `hashed=false` → `SEC_ADD_INSTALL_CODE`; `hashed=true` → `SEC_ADD_LINK_KEY` |
| `waitFor(...)` | Host-side matcher on `zclPayload` events |
| `sendZdo(ieee, nwk, clusterId, payload, disableResponse)` | `APS_DATA_REQUEST` (profile 0, EP 0, payload with the TSN; `hasZdoMessageOverhead = true`), then wait for the `0x8000 \| cluster` indication with the same TSN. Requests to the coordinator itself are answered locally (7.4). |
| `permitJoin(seconds, networkAddress?)` | `NWK_PERMIT_JOIN{target networkAddress ?? 0xFFFC}` |
| `sendZclFrameToEndpoint(...)` | `APS_DATA_REQUEST` (NWK mode, APS ACK unless `disableResponse`), then `EVT_APS_DATA_CONFIRM`, then optionally `waitFor` the response. `disableRecovery = false` → one retry with `FORCE_ROUTE_DISCOVERY`. |
| `sendZclFrameToGroup(groupID, …)` | `APS_DATA_REQUEST{dst_mode 0x01}` |
| `sendZclFrameToAll(endpoint, …, destination)` | `APS_DATA_REQUEST{dst_mode 0x0F, dst 0xFFFF/0xFFFD/0xFFFC}` |
| `setChannelInterPAN`, `sendZclFrameInterPAN*`, `restoreChannelInterPAN` | Throw "not supported": no inter-PAN or Touchlink in v1 |

Events:

| ZRHP | herdsman event |
|---|---|
| `EVT_DEVICE_JOINED{admitted}` | `deviceJoined{networkAddress, ieeeAddr}` |
| `EVT_DEVICE_LEFT` | `deviceLeave{networkAddress, ieeeAddr}` |
| `EVT_APS_DATA_INDICATION` (profile ≠ 0) | `zclPayload{clusterID, address: src_nwk, header, data, endpoint: src_ep, linkquality: lqi, groupID, wasBroadcast, destinationEndpoint}` |
| `EVT_APS_DATA_INDICATION` (ZDO responses) | `zdoResponse(clusterId, parsed response)` |
| Link failure beyond the resume budget | `disconnected` |

The adapter performs seamless resume internally, as in 12.1.

**`start()` result rules.** herdsman passes `networkOptions` (`panID`,
`extendedPanID`, `channelList`, `networkKey`) and `backupPath`
(`coordinator_backup.json`).

| Situation | Action | Result |
|---|---|---|
| NCP formed and its parameters match `networkOptions` | — | `"resumed"` |
| NCP not formed, or parameters mismatch, and the backup file matches `networkOptions` | `BKP_RESTORE_*` from the backup (margins 10 000, or 2²⁰ for an unknown TCLK counter), then `NWK_INIT` | `"restored"` |
| Otherwise | `NWK_FORM` with `networkOptions` | `"reset"` |
| Mismatch while `forceStartWithInconsistentAdapterConfiguration` is unset | herdsman's usual refusal | — |

**Concurrency.** `AdapterOptions.concurrent` is capped at
`max_aps_inflight`.

**Host store.** herdsman has no SQLite. The adapter keeps a sidecar
append-only file `zrhp_store.jsonl` next to `database.db`. Each line is a
writeback or checkpoint record with fsync before ack. It is compacted on
start.

**Z2M OCB quirks.** herdsman's `toUnifiedBackup` writes:

- `pan_id` and `nwk_address` without zero padding;
- `metadata.internal.date`, `znpVersion` / `ezspVersion`;
- no TCLK, TC counter or TC address;
- `stack_specific` only for `zstack` / `ezsp`.

The adapter's import applies 9.1 and 9.4 (padding, `aps_margin = 2²⁰`).

**Required fork change.** To keep `stack_specific.zigbee_rs` (FC limits,
GP tables) in `coordinator_backup.json`, herdsman's
`toUnifiedBackup`/`fromUnifiedBackup` and `Models.Backup` need a pass-through
for unknown `stack_specific` sections. Without it, a Z2M backup restores the
network with margins, but GP sink entries are lost and the GPDs must
re-commission.

**GP.** herdsman's controller implements its own GP sink (proxy
commissioning, pairing). v1 runs the NCP in `HOST_SINK` mode for herdsman.
`NCP_SINK` mode with a herdsman shadow is on the roadmap after v1 (decision
N2).

**Tests.**

- herdsman-style unit tests: the adapter against the Rust reference NCP
  simulator over a PTY;
- shared vectors;
- `ocb_z2m_import.json`;
- a Z2M round trip: Z2M backup → zigbee-rs → Z2M `coordinator_backup.json`
  → zigpy import (14.3);
- hardware gate H11 (14.6).

### 12.4 Embedded hub (`faronov/zigbee-rs-hub`, ESP32-S3)

**Placement and stack.**

- The hub is a **standalone host**. It runs on the S3 of the Thread BR board
  and speaks ZRHP over the board UART (11.5).
- It uses `zigbee-zrhp` with `std` on esp-idf-svc (FreeRTOS threads,
  `esp_timer`, UART driver with IRAM ISR or UHCI).
- It holds the ZCL/device logic and the host store. Store: raw-flash
  journal partition (3.4).
- Its user-facing interface is the hub's own WebSocket JSON API, a
  hub-level interface outside ZRHP.

**Partitions** (decision N9; full tables in 3.4). The hub detects the S3
flash size at boot:

- **4 MB:** hub app ×2 at **1.5 MiB**; one 896 KiB `hubdata` partition with
  the host store journal (128 KiB), configuration (32 KiB) and the
  declarative converter database (2 × 256 KiB, active + staged).
- **8 MB:** hub app ×2 at 3 MiB; `hubdata` 1 920 KiB.
- **No NCP image partition.** NCP rollback is on the H2's A/B slots
  (8.9, decision N6 amended 2026-11-18).

**H2 layouts** (8.9): two application slots + `zbnv`.

| H2 flash | ota_0 | ota_1 | `zbnv` |
|---|---|---|---|
| 4 MB | 0x10000, 1.875 MiB | 0x1F0000, 1.875 MiB | 0x3D0000, 192 KiB |
| 2 MB | 0x10000, 896 KiB | 0xF0000, 896 KiB | 0x1D0000, 192 KiB |

**Behaviour.**

| Area | Behaviour |
|---|---|
| Profile | Embedded (3.3): RX window 8, LOG credit 0 by default |
| Paging | `HOST_TABLES` set; standalone until the NCP cache fills (8.1) |
| GP | `NCP_SINK` with the hub as GP shadow (`host_caps` bit3), security level 0x00 |
| PTA | `coex_enable` follows the hub's Wi-Fi state (decision N7), re-applied in resume |
| NCP images | None on the S3. Updates are streamed into `FW_*`; rollback is the H2's A/B slots and bootloader (decision N6, amended 2026-11-18). H2 flash size from `SYS_GET_INFO.flash_size` (decision N4). |
| Resume | Seamless (12.1 steps). The hub owns EN/BOOT, so after the resume budget it can reset the NCP itself. A ROM-path reflash happens only on an explicit operator request with a supplied image (11.5). |
| Backup | Binary records (9.3) in the hub store. OCB JSON is optional: streamed on request through the hub API, or converted by a desktop tool. |

---

## 13. Security of the host link

**Threat model (v1).**

- The serial link and the host are **trusted**, as with EZSP, ZNP, ZBOSS
  and deCONZ. ZRHP has no network transport, so its attack surface is
  physical (USB/UART access).
- **Embedded hub.** On the Thread BR board, the S3 hub's network exposure
  (Wi-Fi/Ethernet, its WebSocket JSON API) is a hub-level concern. A
  compromised hub has the full power of a ZRHP host.
- An attacker with access to the USB/UART link or the host already
  controls the network. They can read keys through backup, send any frame
  and reflash the NCP.
- ZRHP v1 defends against:
  - accidental corruption (CRC, 5.14);
  - stale or replayed frames from earlier sessions (epoch, cookies);
  - confused-deputy mistakes: an accidental factory reset needs a confirm
    word, firmware must be signed, and a restore is atomic.

Controls:

- **Key export** (decision 3).
  - Network and link keys are readable only with capability bit 4. It is
    **on by default** in the ESP32 NCP product, so HA backups are complete.
  - `SEC_LOCK_KEY_EXPORT` locks it until the next reboot (7.5). A host can
    drop the privilege after its backup and re-apply the lock on every
    resume.
  - A product may still build the capability out; backups then become
    incomplete (9.5).
- **NCP firmware updates** must pass the product's signature policy (MCUboot
  TLV verification, reused from the ESP32 product's signed OTA). Downgrades
  are refused unless the build is a development build.
- **Factory reset** (`SYS_RESET` mode 2) needs the confirm word. Restore
  needs an explicit `BEGIN`/`COMMIT` and keeps the old state until the
  commit.
- **Reset lines.** The USJ DTR/RTS reset (kept enabled, decision 6) and
  the BR board's EN/BOOT lines are physical-access-equivalent capabilities.
  The ROM path writes only signed images when secure boot is on, and never
  touches `zbnv` (11.5).
- **Unsecured GPDs** (decision N1). Level-0 GPDs are accepted for zigpy
  compatibility. Their commands are not authenticated: the duplicate filter
  stops replays only within 2 s, and spoofing within radio range is
  possible. Deployments that need authenticated switches set
  `security_level` 2 or 3 with `GP_SINK_CONFIG`.
- **Never logged:** keys, install codes and frame counters above debug
  level. `EVT_LOG_RECORD` must not contain key material at any level.

**Future authenticated link** (capability bit 13, reserved):

- The RESET handshake would carry an ephemeral X25519 exchange,
  authenticated by a pairing code printed on the device or by a host-pinned
  NCP public key.
- That would derive per-direction AES-128-CCM keys.
- `ctrl` would gain a `SEC` flag (taken from reserved type space), followed
  by an 8-byte MIC and a 32-bit nonce counter.
- The CRC would remain as the outer integrity check, so noise handling is
  unchanged.
- This design follows the optional CPC security model. It is not part of v1.

---

## 14. Test plan

### 14.1 Shared vectors (all implementations)

`host-protocol-vectors/`:

| File | Contents | Must pass |
|---|---|---|
| `crc32c.json` | Check value + edge inputs | Rust, Go, Python, TypeScript |
| `cobs.json` | Empty, zeros, 254/255 runs, max frame | Encode **and** decode |
| `link_frames.json` | RESET_REQ/ACK, DATA (single, batched, RETX\|POLL), ACK, NAK (gap, CRC), credits, keepalive, REJECT, max payload; epoch derivation | Build byte-exact and parse back to fields |
| `messages.json` | NEGOTIATE, NCP_READY, APS request (plain, source-routed), accept/BUSY, confirm, indication, permit join, joined, route record, install code (+ derived key), FC checkpoint, TBL lookup/writeback, `SEC_LOCK_KEY_EXPORT`, `SYS_SET_UART`, UTF-8 `EVT_LOG_RECORD`, `GP_SINK_CONFIG` (zigpy profile), `GP_SINK_COMMISSIONING`, `EVT_GP_COMMISSIONED` with `R_GP_SINK`, `EVT_GP_FRAME` | Build and parse |
| `arq_scenarios.json` | Timed receiver and sender traces (5.9) | State machine with a virtual clock emits exactly the expected frames |
| `ocb_roundtrip.json` | OCB → records → NCP model (global APS counter) → export → OCB, restore message bytes | Host converters (Go, Python, TypeScript, Rust hub) byte- and JSON-equal |
| `ocb_z2m_import.json` | Z2M-style backup (unpadded hex, `date`, no TCLK counter) → records with `aps_margin = 2²⁰` | Same |
| `ezsp_hashed_tclk.json` | **Candidate** EZSP hashed-TCLK keys for both EUI64 byte orders | NCP derivation; one candidate is promoted to normative by gate H5 |

`generate.py --check` fails if a committed JSON file differs from the
generator output; CI runs it. Implementations load the JSON files directly;
they do not copy values. The Rust reference (`zigbee-zrhp/tests/`) drives
every file byte-exact.

### 14.2 Link conformance and property tests (Rust reference first)

1. Every scenario in `arq_scenarios.json`.
2. Additional normative cases:
   - window full;
   - `seq` wrap 255 → 0;
   - `ep_seq` wrap with credits;
   - duplicate behind the window;
   - REJECT `NO_CREDIT` and `WINDOW_VIOLATION`;
   - retry exhausted → reset;
   - simultaneous `RESET_REQ`;
   - stale-epoch frames after a reset;
   - peer restarted vs resync;
   - BULK loss not delaying ZIGBEE delivery (HOL test);
   - anti-starvation;
   - keepalive credit repair;
   - a minimum host (`rx_window` 4, LOG credit 0, BULK credit 1) receiving
     a storm of 1 000 indications with no loss beyond explicit counters;
   - `SYS_SET_UART` switch and automatic revert after 3 s without frames.
3. Property tests with seeded randomness, run in both directions:
   - for any interleaving of bit flips, byte drops, inserted `0x00`,
     duplicated chunks and stalls up to `link_timeout_ms`, every accepted
     message is delivered exactly once, in per-endpoint order;
   - no message is delivered from a frame whose CRC failed;
   - every non-delivered message is reported with an explicit status;
   - memory stays within the configured bounds.
4. Fuzzing (`cargo fuzz`) of the COBS decoder, frame parser, message parser
   and record parser. No panics, and bounded work per input byte.

### 14.3 Message-layer and host conformance (Rust, Go, Python)

- Round-trip every command and event layout. Unknown trailing bytes are
  ignored and missing trailing fields get their defaults (6.6).
- Unknown `msg_id` ⇒ `UNSUPPORTED_COMMAND`; wrong endpoint ⇒
  `WRONG_ENDPOINT`; malformed batch ⇒ `EVT_PROTOCOL_ERROR`.
- Retry cache semantics (6.5).
- OCB import vectors from real-world samples:
  - a zigpy-bellows export;
  - a zigpy-znp export with `tclk_seed`;
  - a zigpy-deconz export with the 2³⁰ counter;
  - a Z2M export with short NWK strings and `date`;
  - the OCB repository sample `z2m-sample-1.json` [OCB].

  Each must convert to the expected records, and back to JSON accepted by
  `zigpy.backups.NetworkBackup.from_dict` with `is_compatible_with(original)`
  true.
- A zigpy-level test runs `zigpy.application` tests against a simulated
  NCP: the Rust reference NCP model (`std`) behind a PTY. It covers:
  - **Seamless resume** (12.1): an NCP restart with an unchanged NIB
    resumes without `connection_lost`; a changed EPID or `table_epoch`
    raises it; a raised key sequence is adopted.
  - **GP shadow** (7.11): an NCP-commissioned GPD becomes a zigpy
    `GreenPowerDevice` with one pairing on air. zigpy's Pairing, Proxy
    Commissioning Mode, Response and alias `Device_annce` are swallowed.
    `EVT_GP_FRAME` packets pass zigpy's counter checks. Import of
    existing zigpy GP devices sends nothing on air. A level-0 GPD
    (FlowerX-GP-like Multi-Cluster Reporting) commissions and reports
    with `security_level = 0x00`. Run against stock zigpy and against the
    `faronov/zigpy` fork with the sink-on-radio hook.
  - **Sidecar rollback:** restoring an older `zigbee_zrhp.db` triggers the
    full writeback through `host_max_rev`.
  - **Key-export lock:** with the lock set, a backup is reported
    incomplete.
  - **ZDO to the coordinator itself:** answered locally, never sent.
- **zigbee-herdsman** (12.3): adapter unit tests against the same simulator;
  the start-result rules (resumed / restored / reset); a Z2M
  `coordinator_backup.json` round trip including `stack_specific.zigbee_rs`
  pass-through, which is part of the fork.
- **Embedded host:** the hub's ZRHP layer runs the vectors and the ARQ traces
  on the host (`std`). The same code on the S3 is exercised by gate H12.

### 14.4 Noise-injection bench (todo `coord-proto-bench`)

A host-side harness sits between two endpoints, over a PTY pair or a real
UART loopback through a USB-UART bridge, and injects:

- a BER from 1e-7 to 1e-3;
- burst garbage;
- byte drops;
- `0x00` insertion;
- chunk duplication;
- RTS stalls;
- USB disconnect/reconnect;
- host process kill and restart;
- **UART without flow control** with a simulated receiver stall (flash
  erase of 15–400 ms on either side), to prove the DMA/IRAM RX requirement
  of 4.1.

It drives 1000 simulated devices of traffic (reports at 1/10 s and 1/60 s,
plus join/announce storms) and reports:

- goodput;
- p50/p99/p999 message latency per endpoint;
- recovery time after a fault;
- counter consistency, i.e. that every drop was reported.

The same bench runs ASH and ZBOSS LL implementations to compare against the
prior art (a requirement of the original protocol decision). Exit
criteria for v1:

- zero undelivered-without-status messages;
- zero duplicate deliveries;
- p99 ZIGBEE latency < 20 ms at BER 1e-5 on 921 600 baud.

### 14.5 Simulation (mcuemu)

[mcuemu] has no ESP32-H2/C6 chip model yet; only register headers and the
RISC-V groundwork exist. Its nRF52840 model has UARTE, RADIO, RNG and NVMC,
and its EFR32MG21 model runs zigbee-rs images. The protocol and NCP app are
platform-independent, so the simulation plan is:

1. Build the NCP app for the nRF52840 mcuemu target with a UARTE transport.
2. Attach its UART to the host reference through mcuemu `tcp:` sinks. This
   is simulator plumbing, not a ZRHP transport.
3. Put zigbee-rs and vendor end devices (TLSR8258, EFR32MG21) on the shared
   radio medium.
4. Script these scenarios:
   - formation, joins and interview through zigpy;
   - an NCP reset in the middle of traffic;
   - a restore followed by device continuity;
   - a power cut during `BKP_RESTORE_COMMIT`, which needs a flash
     fault/power-cut model for the nRF NVMC (it exists for CC2530 only
     today).
5. UART noise injection needs a corruption option on mcuemu `links`.

This is simulation-level evidence, not hardware evidence.

### 14.6 Hardware gates (none run yet)

| Gate | Evidence required |
|---|---|
| H1 | USJ throughput and latency on H2 and C6 with the Linux and macOS CDC drivers. Host-absent behaviour (no blocking, counters). Re-enumeration identity (VID/PID/serial). DTR/RTS transitions with pyserial and Go serial. |
| H2 | H2 UART maximum stable baud with RTS/CTS through a USB-UART bridge. |
| H3 | Formation, then 30+ real devices joining, interviewing and reporting via zigpy (HA) and the Go hub, with sniffer captures. |
| H4 | NCP reboot under load: no frame-counter reuse (sniffer: NWK counter strictly increasing across the reboot). Host reboot: network keeps running and queued indications are reported. |
| H5 | **ZBT-1 hashed-TCLK verification** (decision 8): read a device's derived TCLK from the user's ZBT-1 (EZSP key export of a device that completed a TCLK update) and compare it with both candidates in `ezsp_hashed_tclk.json` computed from that network's `hashed_tclk`. The match becomes the normative vector and enables capability bit 14. Then migrate ZBT-1 → H2 → ZBT-1 with real devices, checking APS-secured exchanges (Verify-Key, network-key rotation) with a hashed-key device after each move. |
| H6 | NCP firmware update with trial boot, confirm, rollback and retained network state. |
| H7 | 24 h soak with noise injection on the UART variant (921 600, RTS/CTS). |
| H8 | **Thread BR board UART:** 460 800 (default) and 921 600 without flow control, with both chips writing flash (`rx_overflow = 0`). Optional board mod: RTS/CTS on R19/R20 (fitted? H2 GPIO4/5 usable?) up to 2 Mbaud. Identify the H2 module's flash size (2 MB or 4 MB) via JEDEC ID, and check the NCP's layout-mismatch refusal. ROM-path recovery via S3 GPIO7/8 with `zbnv` preserved. |
| H9 | **PTA coexistence:** Zigbee PER and latency with S3 Wi-Fi under load, coexistence off and on. The hub's automatic enable/disable follows Wi-Fi up/down, including Ethernet-only. GP TempMaster reply behaviour with grant denied. |
| H10 | **GP combo on the NCP:** direct and proxy-tunnelled commissioning into the NCP sink with zigpy as shadow, including an unsecured (level-0) FlowerX GP / BGT1 GP; a bidirectional GPD with the NCP as its own TempMaster; Sink Table across an NCP reboot (seamless resume) and across backup/restore. |
| H11 | **Z2M end to end:** zigbee-herdsman adapter with 10+ real devices; Z2M backup → restore on a fresh NCP. |
| H12 | **Embedded hub:** S3 hub with a synthetic 2 000-device store (PSRAM budget of 3.4), paging forced by a reduced NCP cache, power cuts during writebacks, store journal recovery. Candidate NCP image that fails its trial (no `FW_CONFIRM`) → the H2 bootloader returns to the previous slot, network state retained; an image larger than the H2 slot refused with `TOO_LARGE` before any erase. Hub app within 1.5 MiB on a 4 MB S3. |

---

## 15. Implementation work and validation gates

| Step | Work | Depends on | Validation level reachable without hardware |
|---|---|---|---|
| 1 | `zigbee-zrhp`: COBS, CRC-32C, frame codec, sessions, ARQ, credits, scheduler, message/record codecs (`no_std`; `alloc`/`std` features); wire `generate.py --check` into CI | this spec | Host-tested, vectors, fuzz, property tests |
| 2 | Runtime: **global APS outgoing counter** with reservation, journal format version and migration (8.4) | existing runtime | Host-tested, including migration and power-cut tests |
| 3 | Runtime: coordinator event surface, raw APS passthrough for host endpoints, dynamic host endpoints, RSSI/last hop, APS queue with confirms, per-send source routes, ZDO forwarding/TSN rules, no-loopback guard | existing runtime | Host-tested with `MockMac` and `examples/mock-coordinator` |
| 4 | Runtime/persistence: FC checkpoint hook and floors; IEEE override; atomic restore transaction; snapshot export; TC-device cache with writeback/eviction, `host_acked_rev`, streamed TC-journal snapshots (256 entries); child journal with 1 024-byte slots (64 children); key-export lock | 2 | Host-tested with journal fault injection |
| 5 | Runtime: EZSP hashed-TCLK derivation (HMAC-MMO), cached with origin 4 | 4 | Host-tested against the candidate vectors; normative after H5 |
| 6 | GP: NCP-sink mode surface (`EVT_GP_*`, `GP_*` commands, `R_GP_SINK` codec), sink table size as a product parameter (16), host-sink mode local-proxy hand-off | GP sink on master | Host-tested with zigpy golden vectors |
| 7 | MAC/HAL: H2/C6 UART transport with UHCI RX DMA and optional RTS/CTS; USJ non-blocking pump; PTA follower in the ESP MAC | — | Host/unit tests, builds; hardware gates H1, H2, H8, H9 |
| 8 | `apps/ncp-coordinator` over `TrustCenterCoordinatorApp` + GP combo | 1–6 | Host-tested against the Rust reference host over an in-memory pipe |
| 9 | `products/esp32-zigbee-ncp`: 2 MB and 4 MB layouts (8.9), NV journals, identity, transport features (`transport-usj`, `transport-uart`), boards (H2 devkit, Thread BR H2 side), signed update, size report | 7, 8 | Builds, post-link layout checks, size budget |
| 10 | Hosts: `zigpy-zrhp` (Python, under `faronov`) plus the sink-on-radio hook in the `faronov/zigpy` fork (no upstream PR; upstreaming later, with user approval), Go `adapter/zrhp` (+ margin 10 000), zigbee-herdsman fork with `zrhp` (TypeScript, plus the backup pass-through), embedded `faronov/zigbee-rs-hub` (Rust, S3) | 1 vectors | Host-tested vs vectors and the simulated NCP |
| 11 | Noise bench (14.4), mcuemu scenario (14.5) | 1, 8 | Simulation |
| 12 | Hardware gates H1–H12 | 9, 10 | Lab, then production-path evidence |

Nothing beyond step 1 should claim more than "host-tested" until gates H1–H4
pass. The coordinator remains documented as host-tested in
[Coordinator & Router](coordinator-router.md) until then.

---

## 16. Decisions (2026-11-18) and new open questions

The user answered all 14 open questions of the 2026-11-15 draft and added
two requirements. Each decision is applied in the sections listed.

| # | Decision | Applied in |
|---:|---|---|
| 1 | Name **ZRHP**; the endpoint and message-ID allocation of 6.7 is **final**. | 3.2, 5.10, 6.7 |
| 2 | **One reserved global outgoing APS counter**, as in EmberZNet. Includes the runtime change, migration from per-key counters and backup mapping. Host table paging carries no outgoing-counter state. | 3.1, 8.3, 8.4, 9.2, 9.3, 15 |
| 3 | **`KEY_EXPORT` on by default** in the ESP32 NCP product, with `SEC_LOCK_KEY_EXPORT` locking it until the next reboot. | 7.1, 7.5, 12.1, 13 |
| 4 | zigpy host store is a **sidecar SQLite** next to `zigbee.db`. **Standalone by default**; paging starts automatically above the NCP cache capacity. | 8.1, 8.8, 12.1 |
| 5 | NCP reboot in zigpy: **seamless resume** inside the library, with `connection_lost` only when the state cannot be reconciled. | 5.11, 8.7, 12.1 |
| 6 | USJ DTR/RTS reset **stays enabled**. | 11.3, 13 |
| 7 | Transports: **USB-Serial-JTAG and UART with RTS/CTS** from the start, UART default **921 600**. | 4.1, 4.4, 11.4 |
| 8 | **EZSP hashed TCLK in v1**, with a real ZBT-1 verification step. | 7.5, 9.6, 14.1, gate H5 |
| 9 | H2 capacities: link-key cache **256**, address-map cache **512**. `max_children` = 64 and journal sizes calculated from the H2 RAM/flash budget. | 8.9 |
| 10 | **ZDO loopback dropped** from v1 (capability 11 reserved). | 1, 6.6, 7.4 |
| 11 | **GP combo sink on the NCP** in v1, with sink events, Sink Table exposure and zigpy shadow-sink rules (no double pairing). | 3.1, 7.11, 9.2, 9.3, 12 |
| 12 | LOG endpoint carries **UTF-8 text**. | 7.10 |
| 13 | Go hub `FrameCounterRestoreMargin` **2 500 → 10 000**. This is a required change in the Go repository; this document does not edit it. | 9.4, 12.2 |
| 14 | **zigbee-herdsman / Z2M adapter in scope.** | 9.1, 9.6, 12.3, 14.3, gate H11 |
| R1 | **Embedded host** from the start: Rust hub `faronov/zigbee-rs-hub` on the ESP32-S3 of the Espressif Thread BR / Zigbee GW board, using the shared crate `zigbee-zrhp`. Covers host profiles, minimum host buffers, BR-board UART, ROM-path NCP update and PTA. | 1 (G9), 3.2–3.4, 4.1, 4.4, 11.5, 12.4, gates H8, H9, H12 |
| R2 | **No network transport in ZRHP** (USJ and UART only). The S3 hub's WebSocket JSON API is hub-level and out of scope. | Header, 1, 3.2, 4, 13 |

**Answers to the follow-up questions** (same day):

| # | Decision | Applied in |
|---:|---|---|
| N1 | NCP GP sink `gpsSecurityLevel` = **0x00** (zigpy-compatible): unsecured GPDs such as FlowerX GP / BGT1 GP are accepted. | 7.11, 12.4, 13, 14.3, H10, `messages.json` (`gp_sink_config_zigpy`) |
| N2 | The **"sink on radio" hook** and all zigpy / `zigpy-zrhp` work live in **our fork** (`faronov/zigpy`, `faronov/zigpy-zrhp`). No upstream zigpy PR for now; upstreaming is a later step that needs the user's approval. The Go hub and zigbee-herdsman move to `NCP_SINK` later; v1 keeps `HOST_SINK` there (roadmap). | 3, 7.11, 12.2, 12.3, 14.3, 15 |
| N3 | **No derived TCLKs** for new networks: random per-device TCLKs only. Hashed mode exists only for EZSP migration. | 7.5 |
| N4 | BR board UART: **460 800 without flow control** by default, DMA/IRAM RX mandatory; RTS/CTS via R19/R20 documented as an optional board mod. H2 module flash size unknown: both 2 MB/4 MB partition tables kept, size detected at runtime (checked when the board is on hwlab). | 11.5, 8.9, H8 |
| N5 | zigbee-herdsman: **maintained fork**, no upstream PR plan for v1. | 3, 12.3, 15 |
| N6 | ~~The S3 stores known-good + candidate NCP images, with rollback through the H2 ROM bootloader.~~ **Amended 2026-11-18:** rollback lives on the **H2**: A/B application slots on the H2 flash, `FW_*` writes the inactive slot, trial boot, host `FW_CONFIRM`, H2 bootloader rollback of an unconfirmed image. The S3 stores **no** NCP images. The H2 ROM-bootloader path is a manual/development recovery path with an operator-supplied image. | 7.9, 8.9, 11.5, 12.4, H12 |
| N7 | PTA **disabled by default**; the hub enables it automatically while Wi-Fi is active (off with Ethernet-only). | 7.1, 11.5, 12.4, H9 |
| N8 | USB identity: **keep Espressif's VID/PID**. | 4.3, 11.3, H1 |
| N9 | (2026-11-18) S3 flash: **detect 4 MB / 8 MB at boot, one partition table per size.** 4 MB: hub app ×2 at 1.5 MiB + one 896 KiB data partition (host store, configuration, declarative converter database); no NCP image partition. | 3.4, 12.4, H12 |

**Open questions.** None block the start of implementation. The remaining
unknowns are hardware facts tracked as gates:

- the USJ VID/PID and serial-number behaviour on H2/C6 (H1);
- the H2 UART maximum rate (H2);
- the EZSP hashed-TCLK byte order (H5);
- the BR board's flash variant and R19/R20 fitting (H8);
- PTA behaviour under Wi-Fi load (H9).

## 17. References

- [OCB] Open ZigBee Coordinator Backup format,
  <https://github.com/zigpy/open-coordinator-backup> (README, `samples/`).
- [zigpy] `zigpy/application.py`, `zigpy/backups.py`, `zigpy/state.py`,
  `zigpy/types/named.py`, <https://github.com/zigpy/zigpy> (branch `dev`).
- [bellows] `bellows/ash.py`, `bellows/ezsp/protocol.py`,
  `bellows/zigbee/application.py` (`load_network_info`,
  `write_network_info`), `bellows/zigbee/util.py` (`zha_security`, hashed
  TCLK), <https://github.com/zigpy/bellows>.
- [zigpy-znp] `zigpy_znp/api.py`, `zigpy_znp/frames.py`,
  `zigpy_znp/znp/security.py` (TCLK seed, `read_hashed_link_keys`),
  <https://github.com/zigpy/zigpy-znp>.
- [zigpy-deconz] `zigpy_deconz/zigbee/application.py`,
  <https://github.com/zigpy/zigpy-deconz>.
- [zigpy-zboss] `zigpy_zboss/frames.py`, `uart.py`, `zigbee/application.py`,
  <https://github.com/kardia-as/zigpy-zboss>.
- [UG101] Silicon Labs, *UART-EZSP Gateway Protocol Reference (ASH)*,
  <https://www.silabs.com/documents/public/user-guides/ug101-uart-gateway-protocol-reference.pdf>.
- [UG100] Silicon Labs, *EZSP Reference Guide*, protocol format,
  <https://docs.silabs.com/zigbee/8.2.0/sisdk-ezsp-reference-guide/03-protocol-format>.
- [SWRA198] Texas Instruments, *Z-Stack Monitor and Test API*.
- [deCONZ] dresden elektronik, *deCONZ Serial Protocol* v1.22,
  <https://deconz.phoscon.de/raspbian/deCONZ-Serial-Protocol_en_1.22.pdf>.
- [Spinel] OpenThread Spinel specification, `core/intro-frames.md`,
  `core/appendix-framing.md`, <https://github.com/openthread/spinel-spec>.
- [CPC] Silicon Labs Co-Processor Communication,
  <https://docs.silabs.com/gecko-platform/5.2.0/platform-cpc-overview/>,
  <https://github.com/SiliconLabs/cpc-daemon>.
- [COBS] S. Cheshire, M. Baker, "Consistent Overhead Byte Stuffing",
  IEEE/ACM Transactions on Networking 7(2), 1999.
- [Koopman] P. Koopman, CRC polynomial zoo, CRC-32C `0x8f6e37a0`,
  <https://users.ece.cmu.edu/~koopman/crc/c32/0x8f6e37a0_len.txt>.
- [RFC6298] *Computing TCP's Retransmission Timer*.
- [ESP-TRM] Espressif, ESP32-H2 and ESP32-C6 Technical Reference Manuals
  (USB Serial/JTAG controller, UART).
- [IDF-USJ] ESP-IDF, *USB Serial/JTAG Controller Console*,
  <https://docs.espressif.com/projects/esp-idf/en/latest/esp32h2/api-guides/usb-serial-jtag-console.html>.
- [esptool] `esptool/reset.py` (`USBJTAGSerialReset`, `HardReset`),
  <https://github.com/espressif/esptool>.
- [esp-hal] `esp-hal` USB-Serial-JTAG driver and issues #6587, #6787, #5799,
  <https://github.com/esp-rs/esp-hal>.
- [asukiaaa] Throughput measurement of the C3/S3 built-in USB,
  <https://asukiaaa.blogspot.com/2023/02/esp32-c3-s3-builtin-usb-can-send-700kbps.html>
  (single source, different chips).
- [ESP-TBR-HW] Espressif, ESP Thread Border Router SDK, *Hardware
  Platforms* (board features, host-to-RCP GPIO allocation, PTA, USB ports),
  <https://docs.espressif.com/projects/esp-thread-br/en/latest/hardware_platforms.html>.
- [ESP-TBR-SCH] Espressif, *ESP Thread Border Router/Zigbee Gateway Board
  schematic* v1.3 ("Interface" sheet: R14–R20, PTA R21–R24),
  <https://dl.espressif.com/dl/schematics/esp_thread_br_zigbee_gw_schematics_v1.3.pdf>.
- [ESP-TBR-RCP] Espressif, *RCP Update Mechanism*,
  <https://docs.espressif.com/projects/esp-thread-br/en/latest/dev-guide/rcp_update.html>.
- [Silabs-HLK] Silicon Labs community, *Hashed Link Keys* (AES HMAC of the
  EUI64 with the TC master key), <https://community.silabs.com/s/article/hashed-link-keys>.
- [herdsman] zigbee-herdsman `src/adapter/adapter.ts`, `src/adapter/events.ts`,
  `src/adapter/tstype.ts`, `src/utils/backup.ts`,
  <https://github.com/Koenkk/zigbee-herdsman>.
- [zigpy-zgp] zigpy `zigpy/zgp/manager.py` and `application.py`
  (`gp_packet_received`), <https://github.com/zigpy/zigpy> (branch `dev`).
- [ESP32-H2-MINI-1] Espressif module datasheet (2 MB / 4 MB in-package flash
  variants).
- [mcuemu] Local emulator `~/mcuemu`, `docs/esp32-riscv.md`,
  `docs/system-config.md`.
- zigbee-rs: `zigbee-bdb/src/security_persistence.rs`
  (`FRAME_COUNTER_RESERVATION_SIZE`), `zigbee-bdb/src/trust_center.rs`,
  `zigbee-runtime/src/trust_center_runtime.rs`,
  `zigbee-aps/src/apsde.rs`, `zigbee-nwk/src/routing.rs`,
  `zigbee-mac/src/esp/radio_phy.rs`, `vendor/esp-hal/src/usb_serial_jtag.rs`,
  `vendor/esp-hal/ld/esp32h2/memory.x`, `zigbee-runtime/src/child_store.rs`,
  `zigbee-runtime/src/trust_center_store.rs`, `zigbee-runtime/src/gp_store.rs`,
  `zigbee-gp/src/sink.rs`, [Green Power](green-power.md),
  [Coordinator & Router](coordinator-router.md), [Security](security.md).

