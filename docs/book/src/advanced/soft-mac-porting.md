# Porting a Radio Backend to `SoftMacCore`

Every MLME/MCPS procedure lives once, in
[`zigbee_mac::SoftMacCore`](https://github.com/faronov/zigbee-rs/blob/master/zigbee-mac/src/soft_mac.rs):

- active, passive and ED scan;
- association, including direct and indirect responses;
- disassociation;
- polling;
- unslotted CSMA-CA, ACK wait and retries;
- the retained-RX queue;
- for routers, association responses, beacon and orphan responses, and
  indirect Frame Pending.

Frames are built and parsed only by `zigbee_mac::frames`. A backend provides
two things:

- a thin `RadioPhy`, which drives the radio hardware;
- `PlatformServices`, which supplies a monotonic clock, delays and entropy.

```text
zigbee-runtime (MacDriver / ParentMacDriver)
        │
SoftMacCore<P, R = EndDevice, const RX = 4>
        │   scan · association · poll · CSMA-CA · ACK wait · retries · queues
        │
P: RadioPhy + PlatformServices      ← the only per-chip code
```

Production status:

| Backend | Uses `SoftMacCore` | Validation |
|---|---|---|
| BL702 | yes, via `Bl702SoftMac` | hardware-tested |
| nRF52840/52833 | yes, via `NrfSoftMac` | build- and host-tested; no hardware run since the migration |
| CC2340, EFR32MG1, EFR32MG21, ESP32-C6/H2, PHY62x2, TLSR8258 | no, each has its own `MacDriver` | — |

The last group is Phase B of the MAC unification.

## What the PHY must implement

`RadioPhy` (`zigbee-mac/src/phy.rs`) has these required items:

| item | contract |
|---|---|
| `type Tx` | The transmit offload level: `SoftwareTx`, `AckWaitTx` or `FullTx` (see below). |
| `capabilities()` | Reports `hardware_auto_ack`, `hardware_address_filter` and the TX power range. |
| `try_transmit(frame)` | One CCA, then transmit. The PHY appends the FCS. A busy CCA returns `PhyError::ChannelBusy`. Never retry inside the PHY. |
| `send_ack(seq, fp)` | Immediate ACK, used only as a fallback when `hardware_auto_ack == false`. |
| `receive(timeout_us, frame)` | Write one FCS-valid frame, without its FCS, into `frame` and return `Ok(true)`, or return `Ok(false)` on timeout. Dropping the future must leave the radio usable. |
| `set_channel`, `set_tx_power`, `energy_detect` | Synchronous configuration. ED uses the 0–255 IEEE scale. |
| `set_address_filter(Option<…>)` | `None` means promiscuous mode, used during a scan. |

`PlatformServices` adds:

- `monotonic_micros()`;
- `delay_micros()`, which must be a real timer and never a busy loop that
  never yields;
- `fill_random()`;
- through `zigbee_crypto::ForwardAesProvider`, the AES backend.

`receive` gets the retained-RX queue's free slot as `frame: &mut PhyRxFrame`.
Fill it in place: `frame.set(bytes, lqi)` copies a frame (and leaves `frame`
unchanged when it is too long); a driver that reads its RX FIFO directly can
write `frame.buffer_mut()` and then call `frame.set_len(len, lqi)`. Don't
build a `PhyRxFrame` on the stack and return it: before this API, every
received frame was moved through the PHY future, `listen` and `accept`, and
these 127-byte copies made `listen` the largest MAC stack frame on every
target (736 B on nRF, 976 B on BL702, 1,480 B on CC2340). `PhyRxFrame` is
`Copy`, so a PHY that parks frames can still hand them over with
`*frame = parked`.

Construct the MAC with one of these:

```rust,ignore
let mac: SoftMacCore<MyPhy> = SoftMacCore::new(phy, MacPib::new(eui64, dsn, bsn))?;
// Explicit role and retained-RX depth:
let mac = SoftMacCore::<MyPhy, EndDevice, 8>::new_custom(phy, pib)?;
let router = SoftMacCore::<MyPhy, Parent<8, 8, 8>, 8>::new_custom(phy, pib)?;
```

`RX`, the retained-RX depth, must be 1..=255 (a compile-time assertion).
Each slot costs 127 B in the MAC struct.

Keep the constructor `#[inline(never)]` if the composition root stores the
MAC in an Embassy task future (see `NrfRadioPhy::into_soft_mac`). Otherwise
the main task's stack frame grows by the size of the core.

## Optional capabilities

Every option is the `Tx` marker type, an associated constant or a provided
method with a default. None of them uses runtime flags or `dyn`. Only the
selected transmit path is compiled, and the futures have exactly the size
that path needs.

| `type Tx =` | the PHY implements | the core keeps |
|---|---|---|
| `zigbee_mac::SoftwareTx` | `try_transmit` (one CCA + TX) and `send_ack` | CSMA-CA, ACK wait, retries |
| `zigbee_mac::AckWaitTx` | `try_transmit_acknowledged(frame)`: one CCA + TX + Imm-Ack wait in the ISR or radio core. Returns `Ok(Some(ack))`, `Ok(None)` when no ACK arrived, or `ChannelBusy`. Used by EFR32 and PHY62x2. | CSMA-CA backoffs, retries |
| `zigbee_mac::FullTx` | `transmit_offloaded(frame, ack_requested, TxParams)`: the complete CSMA-CA, ACK wait and retry sequence. `TxParams` carries `macMaxCSMABackoffs`, `macMinBE`, `macMaxBE` and `macMaxFrameRetries`. Used by CC2340 PBE and ESP. | nothing |

The unused methods keep their `Unsupported` defaults.

| item | default | use it when |
|---|---|---|
| `const POLL_RESPONSE_WAIT_US` | 30 ms | How long to wait for indirect data after an ACK with FP=1. nRF uses 1.5 s, the value proven on hardware with the legacy MAC. |
| `const POLL_EXTENDED_FALLBACK` | `false` | Retry an empty short-source poll once with the EUI-64. EmberZNet parents queue a Rejoin Response or Transport-Key by EUI-64. |
| `set_ack_frame_pending(src, on)` / `clear_ack_frame_pending()` | `Unsupported` / no-op | A parent with `hardware_auto_ack` must arm FP in the hardware or ISR source-match table. |
| `sleep()` / `wake()` | `Ok(())` | Radio power-down between polls. `SoftMacCore::wake` re-applies channel, power and filter. |

The core selects the transmit path through the sealed `TxOffload` trait
implemented by the three marker types. Do not write
`if P::CONST { a.await } else { b.await }` in the core: coroutine layout is
computed before monomorphisation, so both await states, and the code for
both paths, would stay in every future of every image.

*API change:* `type Tx` replaces the former `const TX_OFFLOAD: TxOffload`
enum (`None` / `AckWait` / `Full` become `SoftwareTx` / `AckWaitTx` /
`FullTx`). It is a required item because stable Rust has no associated-type
defaults; existing PHYs add one line, e.g. `type Tx = zigbee_mac::SoftwareTx;`.

### ACK generation

The 192 µs turnaround cannot be met from task context on several chips; PHY62x2
measured 260–450 µs. A chip that can acknowledge in hardware or in an ISR
must do it there and report `hardware_auto_ack = true`. The core then never
calls `send_ack`, and a parent arms Frame Pending through
`set_ack_frame_pending`.

The task-context `send_ack` exists only for radios without any such path.
The nRF RADIO currently uses it; its timing is a hardware gate.

A busy `send_ack` is not fatal. The originator simply retransmits.

### Received-frame rules

Every receive window (ACK wait, poll response, association response,
post-association capture, data indication, command events, beacon
collection) is the one `SoftMacCore::listen` loop. Each received frame is
classified synchronously by `accept` the moment it is received:

- During a scan only beacons are used; nothing is retained or acknowledged.
- An Imm-Ack ends the window only when it matches the awaited sequence.
- A data frame or MAC command addressed exactly to us with the ACK-request
  bit gets one software ACK, in every window, unless the PHY reports
  `hardware_auto_ack`. Frames taken later from the retained-RX queue are
  never acknowledged again.
- Retained: data frames to an accepted destination (pending RX), MAC
  commands on a parent (command queue), and Association Responses addressed
  to us. Malformed frames, frames for other destinations, beacons outside a
  scan and MAC commands on an end device are dropped.
- A poll response from our parent is put at the front of pending RX.
- Frames are received straight into pending RX's free slot, so a full
  pending RX evicts its oldest frame when the next frame is *received*,
  whether or not that frame is then retained. Taking a retained Association
  Response leaves the other frames in order.
- One window handles at most 256 frames, so a PHY whose clock stalls cannot
  keep the MAC spinning.

No future or stack frame holds a received frame by value; only the software
ACK (sequence and Frame Pending bit) survives the `send_ack` await. Keep it
that way when changing the core: a `PhyRxFrame` (127 B) moved into a nested
future is duplicated in every enclosing future, i.e. on every NWK/APS/ZDO
caller's stack. For the same reason, MAC procedures that take a request or
frame by value, the `TxOffload` hooks and `csma` are written as
`fn … -> impl Future { async move { … } }` rather than `async fn` (an
`async fn` stores its arguments twice). `SoftwareTx` is the exception: its
`async fn` form inlines better into the nRF main task.

## Router / parent role

`SoftMacCore<P, Parent<EVENTS, ASSOC, PENDING>, RX>` implements
`ParentMacDriver`:

| primitive | behaviour |
|---|---|
| `mlme_start` | Uses the shared `validate_router_start`; only non-beacon router starts are accepted. Sets channel, PAN ID and `macRxOnWhenIdle`. |
| `mlme_associate_response` | Indirect transaction keyed by the child's EUI-64, kept for `macTransactionPersistenceTime`. It is delivered, acknowledged, when the child sends an extended-source Data Request. A failed attempt stays queued until it expires. |
| `mac_command_event[_timeout]` | Beacon Request, Association Request, Data Request and Orphan Notification, parsed strictly by `frames::parse_parent_command`. Also reports `AssociationResponseDelivery` (`Ok`, a failure, or `TransactionExpired`). Data frames heard meanwhile are retained for `mcps_data_indication`. |
| `set_indirect_data_pending` | With `hardware_auto_ack`, sets the hardware table; otherwise sets a software table of `PENDING` entries that feeds the `send_ack` FP bit. |
| `mlme_beacon_response` / `mlme_orphan_response` / `mcps_indirect_data` | Built with the shared builders. |

The parent code lives in `soft_mac/parent.rs` and is reachable only through
`Parent`. `EndDevice` is a zero-sized type, and every shared receive path
tests the constant `R::PARENT`. A Sleepy End Device image therefore contains
no parent parser, queues or frame-pending code. CI greps for router and parent
symbols in sensor images.

## Measuring the footprint

`tools/mac-footprint.py` builds every CI production image with CI's
toolchains and flags, then measures the ELF:

```bash
# all images, or a regex subset
python3 tools/mac-footprint.py build --keep-going --out target/mac-footprint/new efr32mg1
python3 tools/mac-footprint.py report --out target/mac-footprint/new --json new.json
python3 tools/mac-footprint.py compare tools/mac-footprint-baseline.json new.json
```

Before changing a backend, build its images at the parent commit. The
committed baseline (see [Firmware Size](./firmware-size.md#mac-unification-footprint-baseline))
also works, but drifts as unrelated work lands.

`compare` exits non-zero if any of these grew:

- `flash`, `data`, `bss` or `pool`;
- `stack`, the static worst case;
- `mac_chain`, the deepest call chain executing MAC code.

`text`, `mac_code`, `mac_stack` and `mac_frame` are diagnostic only.
`mac_stack` and `mac_frame` move whenever the inliner shifts a wrapper
between the runtime and the MAC.

To find growth:

- `report -v` lists the deepest chains;
- `cargo rustc --release -- -Zprint-type-sizes` shows future sizes;
- `nm --size-sort` diffs show which symbols grew.

## Acceptance criteria for a migrated backend

1. The backend contains no MLME/MCPS logic and no copies of `frames.rs`
   helpers. It contains only `RadioPhy`, `PlatformServices`, hardware
   bring-up and any chip-specific ISR ACK path.
2. Every image of the platform meets `compare` against the parent commit.
   No flash, `.data`, `.bss`, task-pool, `stack` or `mac_chain` growth is
   allowed. Report exact deltas.
3. The tight budgets stay intact: EFR32MG1 has a 376 B margin above its
   16 KiB stack gate, and the TLSR8258 retained SVC stack has 256 B above its
   8 KiB gate. Run the platform's layout checker.
4. Add host tests with a mock PHY for any new capability the backend
   relies on (see the `soft_mac.rs` tests for the `TestPhy` pattern).
5. Pass `cargo fmt --check`, workspace clippy and tests, and build the
   example with `--locked`.
6. Hardware: commissioning, interview, reporting, rejoin and polling on the
   real board, with ACK timing confirmed by an independent sniffer for any
   new ACK path. Until that has run, report the backend as build-validated
   only.

## Known gaps in the shared core

- **Orphan scan.** `MlmeScanRequest { scan_type: Orphan }` returns
  `Unsupported` on end devices. The builders and parsers exist in
  `frames.rs`.
- **ED scan on nRF.** `NrfRadioPhy::energy_detect` returns `Unsupported`,
  as the legacy `NrfMac` did, because embassy-nrf exposes no ED primitive.
- **Hard-bounded poll.** `mlme_poll_timeout` (`SoftMacCore::poll_within`)
  discards a result that completed at or after the deadline, but does not
  cancel the radio operation. EFR32 and Telink cancel it with a timer.
- **Per-backend diagnostics counters.** Telink and EFR32 keep their own;
  the core exposes only `dropped_command_events()`.
- **Hardware security offload** is not modelled. Every backend uses software
  CCM* over the AES provider.
