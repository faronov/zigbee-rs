# Zigbee Green Power (design draft)

> **Status: Proxy Basic and the unidirectional GPD app (MG22 sensor and
> switch) are implemented on the `green-power` branch. The MG22 GPD
> sensor and switch are lab-tested on hardware directly against ZHA; the
> Proxy Basic is lab-tested on the ESP32-H2-Zero router against ZHA
> (2026-11-12, see §6.1).** Bidirectional operation, the full
> proxy and Combo/Sink are still design only. Spec
> references are to GP 1.0a (docs-09-5499-26) and GP 1.1 where noted.

## 0. Implementation status

| Piece | State | Validation |
|---|---|---|
| `zigbee-gp` crate: GPDF codec, GP CCM* security, duplicate filter, Proxy Table, Pairing / Proxy Commissioning Mode / Response / Proxy Table Request, Notification and Commissioning Notification codecs | implemented | host tests, zigpy byte patterns, corrected GP 1.0a security vectors |
| ZCL GP cluster IDs (`zigbee-zcl/green-power`) | fixed | host tests |
| MAC sequence number on `MacFrame` (`zigbee-mac/green-power`) | implemented | host tests |
| NWK/APS alias TX (`zigbee-nwk`/`zigbee-aps` `green-power`) | implemented | host tests |
| Runtime proxy (`zigbee-runtime/green-power`): protocol-3 demux, GPEP 242 dispatch, Device_annce hooks, `gp_poll` / `gp_next_wait_us`, clear on leave | implemented | end-to-end host test: sink Pairing → tunnelled L3 Notification decrypted by a sink → replay dropped → reboot restore |
| `GpProxyJournal` (two sectors, 1024-byte slots, EPID-bound, CRC + commit marker) | implemented | host tests: rollover, interrupted write, CRC fallback, erased, generation exhaustion, bad geometry |
| `router-app/green-power`: product store (missing store is an error), restore after APS tables, persist when dirty, clear on reset / stale fresh join, receive slice bounded by tunnelling deadlines | implemented | lifecycle test |
| ESP32-H2 router example `green-power` + 8 KiB `zbnv` journal at 0x3F8000 | lab-tested on hardware (2026-11-12) | Waveshare ESP32-H2-Zero, app-only flash over the shipped router (joined state kept, no rejoin): Proxy Commissioning Mode, Commissioning Notification tunnelled to ZHA, Pairing → proxy table entry persisted, groupcast Notification decoded by ZHA, table restored after reset without re-pairing (§6.1) |
| `efr32s2-hal::trng` (MG22 CRYPTOACC BA431 TRNG: GSDK init, conditioning KAT, health-failure reinit) | implemented | host model tests; lab bin `s2-trng` built, not run |
| `apps/gpd` (`gpd-app`): provisioning, `GpdJournal`, beacon scan for a permit-join network, unidirectional commissioning, L3 Multi-Cluster Reports and switch commands, CCA backoff | implemented | host tests: sink-side decode of commissioning (key unwrap), reports and Toggle; counter never reused across reboot; failed reservation transmits nothing; journal rollover / interrupted write / CRC fallback |
| `products/efr32mg22-gpd` + `examples/efr32mg22-gpd` (bins `efr32mg22-gpd-sensor`, `efr32mg22-gpd-switch`) | sensor lab-tested on hardware (2026-11-11) | BRD4183A, direct to the ZHA sink (no proxy): TRNG key, L3 commissioning on channel 15, reports every 60.09 s decrypted by ZHA, counter resumed at 16 after reset (was 9) and accepted, EM2 floor ~8 µA, ~11 µA mean between reports, ~0.21 mC per report incl. LED/VCOM. Switch lab-tested the same day with a separate SrcID (0x7A6B9879) and journal: commissioned, Off/On/Toggle events in HA every 30 s (`lab-switch-cycle` build), counter resumed at 16 after reset and accepted. Re-flashing the sensor resumed its own SrcID (counter 80) and ZHA accepted it. Battery reporting (IADC0 AVDD, 331f8ea) lab-tested on hardware 2026-11-13: 3355 mV → 34 × 100 mV, 200 half-% (WSTK 3.3 V); HA shows 3.4 V / 100 % (§6.4) |
| ZHA quirk `tools/zha-quirks/zigbee_rs_gp.py` | lab-tested | loaded by a ZHA config-entry reload (no HA restart) on HA 2026.11; temperature/humidity entities; battery voltage/percentage entities (331f8ea) deployed 2026-11-13 and showing 3.4 V / 100 %; switch event entity with `off`/`on`/`toggle` |
| Bidirectional GPD, full proxy, Combo/Sink | not started | — |

Implementation notes that differ from the design below:

- Commit ids: on 2026-11-13 `green-power` was rebased onto master 5c2762e
  (which merged `aps-group-fffd`); the cherry-pick 94d6127 dropped out.
  Ids in §6 name the commits the hardware images were built from.
  99ebba6 has the same tree as the rebased f11c437. Earlier ids (ce7e806,
  9f118e3, …) predate the APS fix and have no tree-identical rebased commit.

- The MAC sequence number travels on `MacFrame` (`mac_sequence()`), not
  on `McpsDataIndication`.
- GPEP (242) group memberships are owned by the Proxy Table: the APS
  group snapshot drops them and the GP restore re-adds them after the APS
  restore.
- The proxy persists the per-GPD security frame counter with a stride
  (`ProxyConfig::counter_persist_stride`, default 16). After a power loss
  up to stride − 1 already-tunnelled frames could be accepted once more by
  this proxy; zigpy's sink checks the counter again. A stride of 1 removes
  the window at the cost of one flash slot per frame.
- The corrected ZCL GP cluster is behind `zigbee-zcl/green-power`
  (enabled by `zigbee-runtime/green-power`). Without the feature,
  `clusters/green_power_legacy.rs` (master's skeleton, verbatim) is
  compiled. Changing the ungated cluster altered fat-LTO code folding
  and so changed ESP router codegen, even though no image uses the
  cluster.
- Feature-off identity against master 8ce234f, all 13 shipped images
  (release, `--locked`):
  - BL702 router and sensor, EFR32MG1, TLSR8258 router and sensor, and
    the ESP32-H2 thermometer are byte-identical as shipped.
  - The other 7 (MG22 sensor, ESP32-C6/H2 routers, ESP32-H2 sensor,
    nRF52833 ED, nRF52840 router, ZBT-1 router) differ only in the
    line/column fields of `core::panic::Location` records, because shared
    source files gained lines. A checker confirms that every differing
    word is such a field.
  - Built with `-Zlocation-detail=none`, all 11 non-TLSR images are
    byte-identical.
  - Re-checked after the logging and write-skip fixes, against master
    7e7342c, with both trees built from the same worktree path (the
    absolute path changes string lengths in path-dependency panic
    locations):
    - BL702 router and sensor, EFR32MG1 sensor, TLSR8258 router and
      sensor: byte-identical.
    - MG22 sensor, ESP32-C6 router, ESP32-H2 router (devkit and
      Waveshare H2-Zero), ESP32-H2 sensor (also the light-sleep build),
      nRF52833 ED, nRF52840 router and ZBT-1 router: panic `Location`
      line/column fields only. Section layout is unchanged.

**Measured cost (ESP32-H2 router, release, `green-power` on vs off):**
`.text` +25,284 B, `.rodata` +1,572 B, `.data` +320 B, `.bss` +2,280 B
(about 27.2 KB flash, 2.6 KB RAM), plus about 1.8 KB of stack during a
Proxy Table save or restore. Re-measured at the commit that gated the ZCL
cluster: unchanged.

Zigbee Green Power (GP) lets batteryless or very-low-power devices (GPDs)
send short frames without joining the network. GP Data Frames (GPDFs) use
NWK protocol version 3. Routers that run a **GP proxy** tunnel them to a
**GP sink** (in our case the Home Assistant ZHA coordinator) inside ordinary
NWK/APS frames: GP Notification and GP Commissioning Notification on
endpoint 242 (GPEP), profile 0xA1E0, cluster 0x0021.

Zigbee 3.0 requires every router to implement **Proxy Basic**. Today
zigbee-rs implements no part of GP. `NwkHeader::parse` drops every frame
whose protocol version is not 2, and `zigbee-zcl/src/clusters/green_power.rs`
is only a skeleton. Its sink attribute IDs are off by one, several command
IDs are missing, and `handle_command` does nothing.

## 1. Target sink: what ZHA actually does (HA 2026.11)

The design targets what the sink actually does, not everything the spec
allows. Versions checked: HA 2026.11.0 (the user's instance), zigpy 2.4.1,
zha 2.4.3, zha-quirks 2.4.1, bellows 1.2.1. The code is in `zigpy/zgp/`
(manager, tunneling, crypto) and the zha `GreenPowerDevice`.

Only EZSP adapters work. bellows forwards raw GPDFs to zigpy, and the user's
coordinator is a ZBT-1 (EZSP), so this applies to him.

**Commissioning**

- Opening permit-join broadcasts *GP Proxy Commissioning Mode* (0x02) to
  0xFFFD, endpoint 242, with these options:
  - enter;
  - commissioning window present (or exit only on explicit exit);
  - unicast-notifications = 0.
- The sink accepts commissioning in two ways:
  - a GPD Commissioning (0xE0) or Channel Request (0xE3) GPDF that the
    coordinator receives directly;
  - a *GP Commissioning Notification* (0x04) from a proxy.
- Only ApplicationID 0 (32-bit SrcID) is accepted. IEEE-addressed GPDs are
  dropped.
- Security:
  - levels 0, 2 and 3 are accepted;
  - level 1 is rejected;
  - a protected GPD must send its own key and outgoing frame counter. The key
    may be encrypted with the well-known TC-LK, and zigpy decrypts it;
  - zigpy never provisions keys.
- After acceptance zigpy subscribes the coordinator to group 0x0B84. For a
  new GPD it sends a Device_annce for the alias, then broadcasts a
  *GP Pairing* (0x01) with these settings:
  - add-sink;
  - CommunicationMode = **pre-commissioned groupcast** to group **0x0B84**;
  - the GPD key in clear inside the NWK-encrypted frame;
  - the frame counter.
- Unicast, lightweight-unicast and derived-groupcast pairings are never sent.
- Bidirectional commissioning (GPDF with `rxAfterTx=1`):
  - zigpy answers with *GP Response* (0x06) carrying Commissioning Reply
    (0xF0). TempMaster is the `gppShortAddress` from the Commissioning
    Notification, and the Response includes the TX channel;
  - it then waits for Success (0xE2);
  - a Channel Request is answered with Channel Configuration (0xF3) in the
    same way;
  - the coordinator never transmits a GPDF itself, so **bidirectional
    commissioning only works through a proxy** that has a gpTxQueue.

**Operation**

- GP Notification payloads are parsed strictly: **trailing bytes are an
  error**.
  - `gppShortAddress` and the GPP-GPD link byte are present only if
    `proxy_info_present` or `rx_after_tx` is set.
  - For security level 0 the frame-counter field must carry the MAC sequence
    number, because zigpy de-duplicates unsecured GPDFs by it within 2 s.
- **A GPD without a quirk exposes no entities.** Every product needs a
  `GreenPowerQuirkBuilder` quirk:
  - it matches on `device_id`, `manufacturer_id`, `model_id` (or a SrcID
    range);
  - it adds entities that subscribe to `GreenPowerCommandReceived`.
  - The upstream quirks only cover Hue Tap and EnOcean PTM215Z/216Z, which
    appear as event entities.
- **The user's existing GP sensors use this pattern** (DIYRuZ FlowerX GP and
  EfektaLab BGT1 GP; their quirks are in `/config/custom_zha_quirks`):
  - they commission with DeviceID 0x33 or 0x30, manufacturer 0x6D6B and model
    0xF100|bits or 0xF200|bits;
  - they then send **unsecured Multi-Cluster Reporting (0xA2)** with ZCL
    MeasuredValue encodings.
  - **Our GPD app follows the same pattern**, so a quirk can reuse that
    shape.
- An open zigpy PR (#1959, "GPDF sending") may add sink-side transmission
  later. We do not depend on it.

What this means for the design:

| Item | Required for ZHA? |
|---|---|
| Pre-commissioned groupcast with NWK alias | **Yes**: the only mode the sink pairs |
| Exact Notification and Commissioning Notification encodings | **Yes**: strict parser |
| MAC sequence number as the level-0 counter | **Yes**: zigpy de-duplication |
| gpTxQueue and GP Response | Only for bidirectional GPDs |
| Derived groupcast, lightweight and full unicast | No (spec or certification only) |

## 2. Layers and feature flags

Every flag is **off by default**, so shipped images stay byte-identical.
That has to be proven with an ELF/bin diff before merging.

| Layer | Feature (`zigbee-gp` / `zigbee-runtime` / `router-app`) | Contents |
|---|---|---|
| (a) Proxy Basic | `proxy-basic` / `green-power-proxy` / `green-power` | dGP stub (version-3 demux, GPDF codec, security processing for proxy-table entries), proxy table plus journal, duplicate filter, commissioning mode, Commissioning Notification, Notification in derived/pre-commissioned groupcast and lightweight unicast, Pairing, Proxy Commissioning Mode, Proxy Table Request/Response, gppFunctionality attributes, LQI-based tunnelling delay. Unidirectional only. |
| (a+) Basic with bidirectional commissioning | `proxy-bidir-commissioning` | gpTxQueue (≥ 5 entries, one per GPD), GP Response handling in the TempMaster role, timed transmission on the operational channel, Commissioning Reply. *(verify 1.1: possibly mandatory for Basic)* |
| (b) Full proxy | `proxy-full` (implies the above) | Bidirectional operation (any GPDF from the queue, including in operational mode), Channel Request/Configuration with a temporary channel switch, full unicast, every key type (incl. NWK-key-derived and group key) and security levels 1/2/3, IEEE ApplicationID 2 with endpoint, all Notification options |
| (c) Combo / Sink | `sink` (implies `proxy-full`) | GP server on GPEP: sink table plus journal, sink commissioning mode (local button or Sink Commissioning Mode command), direct GPDF reception, Notification handling, Pairing generation, translation table (GPD command → local ZCL command on an application endpoint), Sink Table Request/Response, Translation Table commands |
| (d) GPD app | `gpd` (frame builder and security only) + `apps/gpd` | No NWK/APS/ZDO/BDB. Commissioning (uni- and bidirectional), Channel Request, data and attribute reports, switch commands, secure frame-counter journal, sleep between reports |

`security-level-1` is not a separate flag. Level 1 costs a few dozen bytes,
and zigpy rejects it anyway.

## 3. Architecture

### 3.1 Placement

```text
apps/router (green-power) ─┐            apps/gpd ──────────────┐
zigbee-runtime (green-power-proxy|sink) │  products/*-gpd       │
   ├─ RX demux: NWK FC proto==3 ────────┼─► zigbee-gp::dgp      │
   ├─ GP cluster client/server (EP 242) │   zigbee-gp::proxy    │
   ├─ GpTableJournal (product partition)│   zigbee-gp::sink     │
   └─ alias TX via APS/NWK              │   zigbee-gp::frame ◄──┘ (gpd feature)
zigbee-nwk  (+alias request fields, cfg)│   zigbee-gp::security (zigbee-crypto CCM*)
zigbee-mac  (+mac_seq in indication, cfg; optional timed-TX PHY capability)
```

- **New crate `zigbee-gp`** (`no_std`, heapless, no allocation). It depends
  only on `zigbee-types`, `zigbee-crypto`, and `zigbee-mac` (for its types).
  Modules:
  - `frame`: GPDF parse/build, ApplicationID 0 and 2, extended NWK FC;
  - `security`: GP CCM* nonce, levels 1–3, key en/decryption with the
    TC-LK;
  - `dgp`: duplicate filter (gpDuplicateTimeout 2 s, by MAC seq or security
    counter) and gpTxQueue;
  - `proxy`: proxy table and commissioning state machine;
  - `sink`;
  - `cluster`: exact codecs for the GP commands, golden-tested against
    zigpy's encodings.

  Protocol crates get no new dependency on it.
- **RX demux:** the runtime checks bits 2–5 of the NWK frame control before
  `process_incoming_with_replay_commit` calls NWK parsing (protocol 3 →
  `zigbee_gp`). `zigbee-nwk` keeps rejecting version 3; it does not need to
  know about GP.
- **MAC:**
  - `McpsDataIndication` gains `#[cfg(feature = "green-power")] mac_seq: u8`.
    It is needed for unsecured de-duplication and for the level-0 counter.
  - GPDFs are sent with no source address, a 0xFFFF destination and no ACK;
    `SoftMacCore::accepts_destination` already accepts them.
  - Platforms with hardware address filters (ESP, BL702 parent PHY, EFR32
    RAIL) must be verified to pass frames that have no source address.
- **NWK/APS alias TX** (feature-gated): `ApsdeDataRequest` already has
  `alias_src_addr`/`alias_seq`, but nothing reads them.
  - Add `use_alias`, `alias_src`, `alias_seq` to the NLDE data request.
  - The broadcast transaction table (BTT) entry is keyed by alias
    source/sequence, so several proxies' copies collapse.
  - Address-conflict detection must not treat frames sourced from an alias
    as a conflict *(test item)*.
- **Timed TX (bidirectional only):** GP needs the queued GPDF to go out
  without CSMA, gpTxOffset = 5 ms after the *start* of the GPD's frame. The
  GPD listens for at least gpdMinRxWindow = 0.576 ms. The current `RadioPhy`
  has no RX timestamp and no scheduled TX, so add an optional capability:
  - `PhyCapabilities::timed_tx`;
  - an `rx_sfd_timestamp` in `PhyRxFrame`;
  - `transmit_at(frame, t)`.

  A backend without it must return `Unsupported`. The proxy then clears the
  bidirectional bits in gppFunctionality and Commissioning Notification and
  never claims support it lacks. Expected mechanisms (all unverified):

  | Platform | Expected mechanism |
  |---|---|
  | nRF52840 | RADIO + TIMER/PPI |
  | EFR32 | RAIL scheduled TX |
  | ESP32-C6/H2 | `ieee802154` transmit_at |
  | TLSR8258 | system-tick stamp plus busy-wait |
  | BL702, PHY62x2 | unknown |

### 3.2 Proxy data flow

1. Process a received GPDF in this order:
   1. version 3;
   2. frame type data or maintenance;
   3. ApplicationID;
   4. duplicate filter;
   5. proxy table lookup.
2. Choose the action by mode and table state:

   | State | Action |
   |---|---|
   | Commissioning mode, any GPD | Commissioning Notification after the tunnelling delay. Unicast to the commissioning sink if that was requested, otherwise broadcast 0xFFFD. If security fails, set `security_failed` and append the MIC |
   | Operational, entry found | Check security level and key type against the entry, check the frame counter is above the stored one, decrypt (level 3). Then send a GP Notification for each sink group (alias source = derived or entry alias; alias NWK seq and APS counter = MAC seq − 9, GP 1.1.2 A.3.6.3.3.2), and lightweight/full unicast to each listed sink |
   | Operational, no entry | Drop |

3. Handle GPEP commands:
   - *Pairing:* add or remove a sink, or remove the GPD; store key, counter,
     alias, radius;
   - *Proxy Commissioning Mode:* window, exit-on-first-pairing, unicast
     flag;
   - *Response:* if TempMaster is this node, queue the frame in gpTxQueue on
     the given channel;
   - *Proxy Table Request:* answer from the table.
4. On the first transmission the proxy may defer, using the LQI-based
   tunnelling delay (Dmin 15 ms … Dmax 100 ms). Its own pending Notification
   is cancelled if an identical one from another proxy is heard first
   (A.3.5.2.1). `Proxy::forward_stats()` counts forwards sent and cancelled,
   and the runtime logs each cancellation at debug level.
5. Commissioning mode transitions are reported as `CommissioningEvent`
   (entered with its window, replaced by a new enter from the same sink, or
   left on explicit exit / first pairing). A window that elapses is
   reported once by `Proxy::take_commissioning_timeout()`. The runtime logs
   all of them at info level.

### 3.3 Persistence

**`GpTableJournal`**

- Two 4 KiB sectors in a **product-owned** partition, following the
  `ApsTableJournal` pattern.
- Records are bound to the extended PAN ID. CRC, a generation counter and
  rollover are handled explicitly.
- A proxy entry is about 72 B: GPD ID, endpoint, options, security
  options, key, counter, alias, radius, search counter, up to 2 sink groups,
  up to 2 lightweight sinks.
- Sink-table and translation records share the journal with distinct record
  kinds.
- Leaving the network or a factory reset clears the journal.

**Proxy-side GPD frame counter**

- Authoritative in RAM.
- Checkpointed lazily: when it has advanced by `counter_persist_stride`
  (default 16) since the last written value, or with any other table
  change. GPDFs and GP Pairings share the stride.
- A GP Pairing that changes nothing but the counter, and does not complete
  the stride, is not written. A counter decrease (a re-commissioned GPD) is
  always written, so the stored value never exceeds the live one.
- After a reset, frames in the window up to the last checkpoint could be
  replayed through the proxy. The sink checks counters again (zigpy keeps
  `frame_counter`). This is the same documented trade-off as the NWK replay
  floors.

**GPD outgoing frame counter (security levels 2/3)**

- It must never be reused, so it is reserved ahead: each record reserves
  `counter + 1024`.
- On boot the device resumes at the last reservation.
- The two-sector append journal sees one flash write per 1024 frames.
- An interrupted write, erased flash, a bad CRC and generation wrap all
  have host tests.
- The same journal holds the GPD key (random from the TRNG on first
  commissioning) and the operational channel.

No board crate gets GP storage. Products own the partitions.

### 3.4 ZCL

- Fix the attribute IDs in `zigbee-zcl/src/clusters/green_power.rs`:

  | Range | Attributes |
  |---|---|
  | Sink, 0x0000–0x0007 | gpsMaxSinkTableEntries, SinkTable, CommunicationMode, CommissioningExitMode, CommissioningWindow, SecurityLevel, Functionality, ActiveFunctionality |
  | Proxy, 0x0010–0x0017 | |
  | Shared, 0x0020–0x0022 | |

- Add the missing commands:

  | Direction | Commands |
  |---|---|
  | Client → server | 0x05 Sink Commissioning Mode |
  | Server → client | 0x00 Notification Response, 0x08 Translation Table Response |

- **Proxy-only builds** expose a GP **client** on endpoint 242.
  gppFunctionality is computed from the enabled features. For Basic it is
  believed to be 0x09AC2F, the EmberZNet/ZBOSS value *(verify 1.1)*.
  gppActiveFunctionality is 0x0000FF.
- **Combo builds** add the GP **server**.
- Endpoint 242 is added to the simple descriptor only when the feature is on.
  `compact-single-endpoint` (`MAX_ENDPOINTS` = 2) needs checking for room.

### 3.5 Router integration

- `apps/router` gains a `green-power` feature that forwards to the runtime.
- The product provides:
  - the GP journal partition;
  - table sizes (const generics; Basic default 8 entries, Telink 5);
  - commissioning exit policy;
  - the TX power used for GPDF replies.
- Example `main.rs` files change by one constructor argument at most.
- The runtime ticks GP timers (commissioning window, tunnelling delay,
  gpTxQueue lifetime, checkpoint) from the existing monotonic clock.

### 3.6 GPD app (`apps/gpd` plus `products/<board>-gpd`)

**As implemented (unidirectional, MG22):**

- `gpd-app` is platform-independent: `GpdDevice<P: RadioPhy +
  PlatformServices, S: GpdStore>`. The product supplies the profile, the
  journal and an entropy closure; the MG22 product feeds it from the
  CRYPTOACC TRNG, which is opened only to provision a key and then
  released. `Efr32s2RadioPhy::fill_random` stays `Unsupported`, so the
  shipped MG22 sensor image is unchanged.
- Identity: SrcID derived from the EUI-64 and the device kind; a random
  individual key (security level 3, key type 0b100), persisted before the
  first use. The key survives recommissioning; only the channel is
  forgotten, and the frame counter never moves backwards.
- `GpdJournal`: two erase sectors of 64-byte records (magic, generation,
  SrcID, key, counter reservation, channel, CRC-32, commit marker written
  last). The counter is reserved in blocks of 16 (`counter_block`), so
  flash is written once per 16 frames; after a reset the device continues
  from the reserved limit.
- Commissioning: an active scan (Beacon Request per channel) picks the
  strongest Zigbee PRO network that permits joining; the device then sends
  three Commissioning GPDFs (0xE0, no `rxAfterTx`) with the key wrapped by
  `ZigBeeAlliance09`. Each commissioning frame consumes a frame counter,
  because zigpy stores the commissioning counter as the replay floor.
- Operation: Multi-Cluster Reporting (0xA2) with temperature (0x0402),
  humidity (0x0405) and optional battery voltage and percentage
  (0x0001/0x0020 in 100 mV, 0x0001/0x0021 in 0.5 %); switch commands
  Off/On/Toggle (0x20–0x22). Each GPDF is sent twice with the same counter
  (sinks and proxies drop the duplicate), after CCA with random backoff.
- MG22 composition (`examples/efr32mg22-gpd`): commission once at boot if
  needed; BTN0 short press = report now / Toggle; BTN0 held 3 s = forget
  the channel and recommission; the sensor reports every 60 s; EM2 with
  the radio idled between events. Temperature is the die sensor and the
  humidity is **synthetic** (BRD4183A has no humidity sensor).
- Battery (331f8ea): before each report the sensor measures AVDD with one
  calibrated IADC0 conversion (`efr32s2-hal::iadc`, AVDD/4 against the
  1.21 V band-gap, `SCALE0` from the DEVINFO gain/offset words as GSDK
  `IADC_init` computes them; disabled again after the read). The board
  exposes it as `SupplyToken`; `gpd-app::BatteryCurve` converts millivolts
  into both ZCL attributes and the product selects a CR2032 curve
  (1.8 V = 0 %, 3.0 V = 100 %). On BRD4183A AVDD is VMCU: the WSTK's
  regulated 3.3 V with its power switch at AEM (so the percentage clamps to
  100 %), the coin cell only with the switch at BAT. A failed measurement
  omits both attributes and logs a warning.
- Flash map: application 0x0–0x70000 (no bootloader), one GPD journal per
  device kind (switch 0x70000–0x74000, sensor 0x74000–0x78000), so
  swapping the sensor and switch images on one board keeps both
  identities; 0x78000–0x80000 never written (0x7C000 keeps the Zigbee
  sensor product's security journal).
- Measured (release): 41,228 B file-backed flash, `.bss` 2,312 B;
  29.9 KiB linked stack. Sensor and switch are the same size. Battery
  reporting (331f8ea) adds 884 B (`.text` +812, `.rodata` +72, same build
  path); `.data`/`.bss` unchanged; sensor `.bin` 42,264 B.
- `lab-switch-cycle` (lab only, never shipped): the switch sends
  Off/On/Toggle every 30 s without a button press, because hwlab cannot
  press BTN0.
- BTN0 long press (sensor) is hardware-proven (2026-11-12): held ~5 s by
  the user → `[GPD] long press: recommission` → scan → commissioned on
  channel 15 in 2.5 s with the same SrcID, key and journal, LED0 lit while
  scanning. The three commissioning GPDFs used counters 1176–1178 and the
  next report used 1179 (no reuse). BTN0 is the WSTK button labelled
  **PB0**; PB1 does nothing in this firmware.
- NWK alias: zigpy gives a GPD the NWK `derive_alias(SrcID)` (the low 16
  bits). The first switch identity used a salt that only changed the high
  16 bits, so sensor and switch on one chip shared alias 0x9879. zigpy
  tolerates alias collisions (GP spec A.3.6.3.3) and both worked; the salt
  now differs in both halves for new identities (a host test checks the
  aliases differ). Persisted SrcIDs are never re-derived.
- Open: one sensor report (fc 5, first hardware run) never reached ZHA
  while all others did; the ZHA reload window does not explain it. To be
  checked with an over-the-air capture.
- ZHA: identity 0x6D6B / 0xF300 (DeviceID 0x33) and 0x6D6B / 0xF400
  (DeviceID 0x02). Without a quirk ZHA creates no entities for a GPD;
  `tools/zha-quirks/zigbee_rs_gp.py` adds temperature, humidity and battery
  sensors, and an event entity (`on`/`off`/`toggle`) for the switch.

**Original design:**

- Built directly on `RadioPhy`: one CCA and TX per channel repetition, then a
  bounded RX window for bidirectional operation. It does **not** use
  `SoftMacCore`, NWK or the runtime.
- Startup:
  - if commissioned, load key, counter and channel and report;
  - otherwise commission:
    - **unidirectional:** send Commissioning (0xE0) on the configured
      channel, or across channels 11–26;
    - **bidirectional:** Channel Request hops until Channel Configuration
      arrives, then Commissioning with `rxAfterTx`, then Commissioning Reply
      (decrypt the key if the sink supplies one), then Success (0xE2).
- The commissioning payload follows the user's existing devices:
  - DeviceID 0x30, 0x33 or 0x02 (on/off switch);
  - options: MAC-seq capable, RxOnCapability per build;
  - extended options: security level 3, key type 0b100 individual, key
    present, key encrypted with the TC-LK, counter present;
  - application information: manufacturer ID 0x6D6B (or another; ask the
    user) and a new model ID;
  - the trailing format exactly as zigpy's `GPCommissioningPayload`
    expects.
- Data:
  - sensors send Multi-Cluster Reporting 0xA2 (or 0xA0) with ZCL encodings;
  - switches send 0x20/0x21/0x22 (Off/On/Toggle), plus optional
    scene/level commands.
- A ZHA `GreenPowerQuirkBuilder` quirk is kept in the repo under
  `tools/zha-quirks/`. The user installs it in `custom_zha_quirks`.
- First platform: **EFR32MG22 BRD4183A on the WSTK**.
  - It is hardware-proven, the PHY is already in-tree, and it is free in
    hwlab (it is not the ZBT-1).
  - The AEM gives real current measurements for the sleep and wake/TX burst
    profile.
  - Later candidates: PHY6222 / PB-03F, TLSR8258 sensor, ESP32-H2.

## 4. Size estimates (not measured)

The figures assume Thumb-2 `opt-level=s`. Scale them by roughly 1.2–1.4 for
TC32 and RISC-V. RAM covers static tables, not journal sector buffers; the
journals reuse the existing stack-scoped flash path.

| Layer | Flash | Static RAM | Extra stack |
|---|---|---|---|
| (a) Proxy Basic, unidirectional (5–8 entries) | 8–10 KB | 0.6–0.9 KB | 0.3–0.5 KB |
| (a+) bidirectional commissioning (gpTxQueue 5) | +2–3 KB, plus PHY timed TX 0.3–1 KB | +0.4 KB | +0.1 KB |
| (b) Full proxy | +4–6 KB | +0.3 KB | +0.2 KB |
| (c) Combo/Sink (8 sinks, 16 translations) | +8–12 KB | +1.0–1.6 KB | +0.5 KB |
| (d) GPD app, complete image (radio, CCM*, counter journal, one sensor) | 15–30 KB total (**measured 40.0 KB** on MG22 with Embassy, VCOM `log` formatting and the TRNG) | 1.5–3 KB (**measured 2.6 KB**) | 1–2 KB |

Which layers each platform can take, based on the current numbers in
[Firmware Size](./firmware-size.md):

| Platform / image | Current | Proxy Basic | +Bidir/Full | Combo | GPD |
|---|---|---|---|---|---|
| TLSR8258 TB-04 router | 288,508 B; ~170 KB flash headroom; **.bss had 3,176 B free**, stack 20 KiB (~14.6 KB worst case) | Yes, RAM tight | Probably; RAM must be measured | No (RAM) | n/a |
| nRF52840 router | 236,780 B / 1 MB, 19 KB bss | Yes | Yes | Yes | Yes |
| ESP32-C6/H2 router | ~365–392 KB | Yes | Yes | Yes | H2 yes |
| BL702 XT-ZB1 router | gates in firmware-size doc | Yes | Yes (timed TX unknown) | Yes | n/a |
| ZBT-1 (EFR32MG21) router | ~230 KB / 1 MB | Yes | Yes | Yes | n/a |
| EFR32MG22 | router does not fit 32 KiB RAM | No | No | No | **Yes, first GPD** |
| PHY6222 / PB-03F | sensor-only; PB-03F 7,232 B margin | n/a | n/a | n/a | Yes (small image) |
| EFR32MG1 TRÅDFRI | sensor, above former budget | n/a | n/a | n/a | Possible |

**First proxy router for hardware testing:** the **TLSR8258 TB-04 router**.
It is already joined to the user's ZHA (nwk 0x045B,
`a4:c1:38:36:39:33:4e:a7`), lab-tested, and the hardware is free. Its RAM is
the tightest of the routers, so it is the most demanding place to measure
Proxy Basic. Fallback: an ESP32-C6/H2 router, which has plenty of room but
is not visible in hwlab right now.

## 5. Risks

1. **The 5 ms timed TX** on soft-MAC PHYs. Without it, bidirectional must be
   reported as unsupported. If GP 1.1 makes bidirectional commissioning
   mandatory for Basic, a PHY without timed TX is not compliant.
2. **Channel switching** for Channel Configuration (full proxy). The router
   leaves the operational channel for up to about 5 s and loses routing
   traffic while away. This stays opt-in in `proxy-full`.
3. **The alias NWK extension** touches the BTT and conflict detection, which
   is shared router code. It needs regression tests on the existing router
   images with the feature off and on.
4. **TLSR8258 RAM and stack.** Proxy Basic adds at least 0.6 KB static RAM
   and the stack has only about 5 KB analysed margin. Mitigations:
   - re-measure with the existing stack analysis;
   - shrink table sizes.
5. **Strict zigpy parsing.** An encoding mistake is silently ignored by ZHA.
   Mitigation: golden vectors generated from zigpy's serializers and run as
   host tests.
6. **No real 802.15.4 sniffer** (hwlab only has `fake/radio0`). Timing
   claims for gpTxOffset and RX windows cannot be verified independently
   until a sniffer exists. The GPD↔proxy path can only be inferred from
   counters.
7. **The GP 1.1 Basic spec text is missing.** The mandatory feature set and
   gppFunctionality are partly from memory or other stacks.
8. **ZHA entities need quirks.** "It works" also requires shipping and
   installing a quirk.

## 6. Hardware test plan

All steps use the user's HA 2026.11 ZHA and ZBT-1 coordinator. The proxy is
the TLSR8258 router. The GPD is the MG22 on the WSTK. ZBT-1 lab hardware
used by other work is not touched.

1. **Host:**
   - golden vectors: GPDF levels 0/2/3, commissioning payloads,
     Notification, Commissioning Notification, Pairing, Response, alias
     derivation;
   - journal fault injection;
   - duplicate filter;
   - feature-off byte-identity of every shipped image.
2. **GPD direct to the sink (no proxy):**
   - commission the MG22 GPD with security 0, then 3;
   - the quirk shows temperature, humidity and battery;
   - AEM measures sleep current and TX-burst charge.
3. **Through the proxy:**
   - place the GPD out of the coordinator's range, or lower the GPD's TX
     power to -20 dBm next to the router;
   - open permit join;
   - the proxy's diagnostic counters (debug build) show: Proxy Commissioning
     Mode received → Commissioning Notification sent → Pairing received →
     proxy table entry → Notifications sent;
   - ZHA logs show the notification came from the proxy's `gppShortAddress`
     with a GPP link byte, not the coordinator's direct LQI 255.
4. **Security:**
   - level 3, key encrypted with the TC-LK;
   - replay a captured GPDF (counter not higher): the proxy drops it, its
     counter goes up, and ZHA shows no update;
   - a GPDF with a wrong MIC is dropped.
5. **Persistence:**
   - reboot the proxy: the table is restored and reports keep flowing
     without re-commissioning;
   - power-cycle the GPD many times: the counter never goes backwards
     (check the journal dump);
   - the proxy keeps its NWK/APS state.
6. **Bidirectional** (after a-plus): GPD with `rxAfterTx`; Commissioning
   Reply received through the proxy's gpTxQueue; Success. A timing check
   needs a sniffer.
7. **Combo** (layer c, on nRF52840, ESP or BL702): pair a GP switch directly
   to the router's on/off endpoint with ZHA offline. ZHA must still pair the
   same GPD through the shared proxy table.
8. **Soak:** 24 h with periodic reports. Then re-check the feature-off images
   for size and behaviour.

### 6.1 Results: ESP32-H2-Zero proxy (2026-11-12)

The proxy ran on the Waveshare ESP32-H2-Zero router (NWK 0x8C23, PAN
0xDFE9, channel 15). The coordinator is EZSP (bellows), HA 2026.11.0,
zigpy 2.4.1. The GPD was the MG22 sensor (SrcID 0x293C9879, level 3).
All times are EET.

- **Image:** `cargo build --release --locked --no-default-features
  --features router,waveshare-h2-zero,green-power`.
  - `app.bin` 512,096 B, sha256 `3537c4c2…cbb6a`; ELF 995,920 B.
  - The earlier build of the same commit had `58612ccf…0d2a`. The only
    difference is the build time embedded by `esp_app_desc!`: with that
    timestamp the bytes reproduce exactly.
- **Flash:** app only, 0x10000–0x8E000 (126 sectors), verified.
  Bootloader, partition table, otadata, ota_1 and `zbnv` were unchanged.
  The GP window 0x3F8000–0x3FA000 read back all 0xFF (empty) before
  pairing.
- **Boot:** resumed NWK 0x8C23 with no association or rejoin;
  `0 GP proxy table entries restored`.
- **Proxy Commissioning Mode:** each `zha.permit` produced
  `[GP] proxy commissioning mode entered` in the same second.
  - When the window expires, the proxy left the mode silently, with no
    log line (fixed since, see below).
  - The router LED follows the NWK permit-join timer, which ends at the
    same time.
- **Commissioning Notification:** zigpy received a CN for 0x293C9879 with
  `gpp_short_addr=0x8C23` at 16:53:20.018. The router logged the matching
  CN TX, broadcast with alias source 0x9879 and the derived alias sequence.
- **Pairing:**
  - 16:59:51.9: the GPD logged the BTN0 long press. It was commissioned
    at 16:59:54.4.
  - zigpy then sent GP Pairing (`add_sink`) three times, once per
    commissioning GPDF, using commissioned groupcast to group 0x0B84.
  - Each Pairing produced `GP proxy table saved`. The window holds three
    committed 1 KiB records, generations 0–2, each with the `ZBGP` magic,
    the network EPID, the SrcID, group 0x0B84 and counters 1176–1178.
  - Every Pairing wrote a new record, even when only the counter changed
    (fixed since, see below).
- **Operational forwarding:** the router matched every report (fc
  1179–1188) to the entry and logged `[GP] GPDF … Notification`.
  - Its groupcast GP Notification reached zigpy at 17:05:21.314 (fc 1184)
    and 17:06:21.436 (fc 1185). The 17:06:21.436 frame is attributed by an
    exact match with the router's TX at 17:06:21.252: APS counter 0, ZCL
    tsn 1, alias 0x9879 and seq 0x94. The 17:05:21.314 frame (APS counter
    3, ZCL tsn 4) is consistent with the router's pre-reset counters
    (its last CN used APS counter 2 and ZCL tsn 3).
  - zigpy decoded both as MultiClusterReporting and updated the HA
    entities.
  - In the other cycles, IKEA proxies (0x1AA4, 0x7B65) tunnelled the same
    GPDF within the 15 ms D_min. Our pending forward was then cancelled by
    the duplicate rule (A.3.5.2.1), so no APS TX was logged. This mesh
    therefore cannot attribute every report to a single proxy.
  - The coordinator also hears the GPD directly, so an HA update alone does
    not prove forwarding.
- **Persistence:** after a debug-module system reset at 17:05:33 the
  router resumed NWK 0x8C23 and logged `1 GP proxy table entries
  restored`. It forwarded the next report at 17:06:21 (above) with no
  Commissioning or Pairing traffic.
- **Not covered:**
  - a GPD out of the coordinator's range;
  - an over-the-air capture of tunnelling timing;
  - replay and bad-MIC tests on air;
  - the soak test;
  - bidirectional and Combo operation.
- **Fixed after this run (host-tested, hardware re-checked in 6.2):**
  - Commissioning mode now logs `entered by 0x…, window … s`,
    `window replaced`, and `left: Explicit | FirstPairing | Timeout`. A
    window that elapsed before the next GP command is logged as a timeout
    before that command's own transition.
  - Forwards sent and cancelled by another proxy's copy (A.3.5.2.1) are
    counted. Each cancellation is logged at debug level. The ESP32-H2
    router prints `GP proxy entries … | forwards sent … cancelled …` with
    its 60 s neighbour summary.
  - A Pairing that changes nothing is not written. A counter-only change
    follows the 16-value persistence stride (see 3.3). zigpy's three
    Pairings in this run would now cost one record instead of three.
  - Cost on the Waveshare H2-Zero GP router: `.text` +2,632 B, `.rodata`
    +376 B, `.data` +32 B, `.bss` +8 B (app image 512,096 B → 515,136 B).
    Feature-off images keep their size and layout (see the identity
    re-check above).
- **Open issues:**
  - Pre-existing, not GP: `[NWK] Frame counter replay from <nwk>` warnings
    print the NWK source rather than the auxiliary-header source. The
    router's direct link to the coordinator is weak (LQI ~22–33).

### 6.2 Results: ce7e806 re-check with an air capture (2026-11-12)

Same network, router and GPD as 6.1. Times are EET. hwlab session
`2026-11-12T194925Z-gp-h2-ce7e806-flash-sniff` recorded three streams for
55 min (21:49:25–22:44:25): the router UART, the GPD VCOM, and an nRF52840
running the nRF 802.15.4 sniffer on channel 15.

- **Image and flash:** `app.bin` 515,136 B, sha256 `f8b4c7b6…cb95`.
  - Written to ota_0 only (0x10000–0x8E000, 126 sectors) against a fresh
    full backup (`d8dd3f5b…a37c`).
  - An independent readback of 515,136 B from 0x10000 matched the sha.
  - Bootloader, partition table, otadata, ota_1 and `zbnv` were not
    written.
- **Boot (21:50:24):** resumed NWK 0x8C23 with parent 0x4A76 and no rejoin;
  `1 GP proxy table entries restored`.
- **Attribution on air:** every proxy's GP Notification carries alias
  source 0x9879, so the MAC source of the radius-30 copy identifies the
  originating proxy.
  - **Coverage:** 57 GPDFs between boot and the end of the capture: 54
    data frames (fc 1469–1478 and 1482–1525) and 3 commissioning frames.
  - **Final router counters:** `forwards sent 12 cancelled 45`, and
    12 + 45 = 57.
  - **Our sends:** the 12 sends are 11 data Notifications (fc 1469, 1470,
    1493, 1496, 1498, 1501, 1503, 1515, 1521, 1523, 1524) plus one
    Commissioning Notification. Each one was seen on air from MAC 0x8C23,
    28–43 ms after the GPDF. For fc 1493 the sniffer missed our radius-30
    original. The router's TX log at 22:11:54.540 and our own relays of that
    sequence (below) attribute it.
  - **Our cancellations:** for each of the 45 cancelled frames another
    proxy originated a copy first, usually IKEA 0x1AA4 at +8–20 ms. While
    commissioning mode was open, routers 0xD586, 0x43FD, 0x1F0F, 0xF94D and
    0x04F8 also originated copies.
  - **Race:** in 5 of our 11 data sends another proxy's copy was on air
    first, 17–37 ms after the GPDF, while ours went out at 29–38 ms. Our
    transmission was already committed, so the duplicate rule could not
    cancel it. This race is expected.
  - **Forwarding share:** about 20% of data GPDFs (11 of 54).
- **zigpy distinguishes our copy from IKEA's** (debug log). It decodes our
  groupcast Notification as `MultiClusterReporting`, multicast to group
  0x0B84. It decodes IKEA's broadcast copy, and the coordinator's own direct
  reception through `gpepIncomingMessageHandler` (`UNPROCESSED`), as
  undefined command ids, because the command is still encrypted.
- **Proxy Commissioning Mode:** `zha.permit` was issued at 21:59:25,
  22:03:25, 22:07:25 and 22:11:26, each for 254 s.
  - `entered by 0x0000, window Some(254) s` at 21:59:25.397.
  - `window replaced, window Some(254) s` at 22:03:25.716, 22:07:25.993 and
    22:11:26.334.
  - `left: Timeout` at 22:15:40.336. This is 254 s after the last
    replacement, and 0.23 s after `Permit joining: duration=0`.
- **Re-commissioning (PB0 long press):**
  - The GPD logged `long press: recommission` at 22:00:41.375 and
    `commissioned on channel 15` at 22:00:43.877.
  - It sent three Commissioning GPDFs (MAC seq 132–134, GPD counters
    1479–1481).
  - Our router logged all three. Its Commissioning Notifications for the
    first two were cancelled by other proxies' copies. It sent the third at
    22:00:43.894: NWK dst 0xFFFD, alias seq 122 (MAC 134 − 12), radius 30.
  - zigpy logged `Sent GP Pairing … (add_sink=True)` three times
    (22:00:43.915, 22:00:44.047, 22:00:44.096). The router received all three
    Pairings from 0x0000.
- **Pairing write skip:** none of the three Pairings wrote a record.
  - Before the press, the GP window held generations 16–20; the newest was
    gen 20 with counter 1469.
  - The Pairing counters (≤ 1481) were within the 16-value stride. The next
    record was the stride checkpoint: gen 21, counter 1485, at 22:03:53.644.
  - Later checkpoints followed at counter 1501 (22:19:55) and counter 1517
    (22:35:57).
  - The final window holds gens 16–23, at counters 1402, 1418, …, 1517.
  - In the 6.1 run the same three Pairings had written three records.
- **HA:** the GP sensor entities kept updating (temperature at 22:42:57,
  humidity at 22:45:58).
- **Open issues found by the capture:**
  - **Echoes of our own alias broadcasts are relayed again.**
    - The NWK layer drops frames whose source is its own address, but an
      alias source is not its own address. It also does not record its own
      originated broadcast in the BTT.
    - So when neighbours relay our alias Notification back (radius 29), the
      router delivers it locally again and relays it again with radius 28,
      up to 4 times.
    - Neighbour relays also do not count as passive acknowledgements, so
      the original is retransmitted up to 3 extra times.
    - The cost is about 8 frames per forward where 1–2 should be enough.
      Seen at every forward, for example 22:41:57–22:41:59.
    - The alias TX path is GP-only (added in 9f118e3). The fix is to record
      `(alias, seq)` in the BTT when originating a broadcast with an alias.
      Fixed in 99ebba6; see 6.3.
  - **NWK destination of groupcast Notifications.** Ours go to 0xFFFF. The
    spec (A.3.5.2) requires 0xFFFD, and IKEA sends 0xFFFD. The APS group →
    NWK 0xFFFF mapping (`zigbee-aps/src/apsde.rs`) predates GP. Commissioning
    Notifications already use 0xFFFD. Fixed on master's side in branch
    `aps-group-fffd` (6a1b276); see 6.3.
  - **Alias sequence offset (corrected 2026-11-13):** the IKEA frames
    with alias seq MAC − 12 are **Commissioning Notifications** (cmd 0x04,
    broadcast without a group), for which −12 is the correct offset; they
    are not operational Notifications. In the 6.3 re-check IKEA 0x1AA4 and
    0x7B65 sent only Commissioning Notifications for our GPD, so every
    operational Notification on the air was ours. Our commissioned-groupcast
    offset is −13; EmberZNet (GSDK 4.5, certified) uses −9 for that case
    (`EMBER_GP_NOTIFICATION_COMMISSIONED_GROUPCAST_SEQUENCE_NUMBER_OFFSET`).
    Resolved in §6.4: GP 1.1.2 A.3.6.3.3.2 says −9 (GP 1.0a said −13).
  - **Forwarding in commissioning mode:** while in commissioning mode we
    forward data GPDFs as ordinary Notifications. The other proxies switch
    to Commissioning Notifications (−12). The spec leaves this to the
    application profile (A.3.5.2.1 (iii)).
  - **APS counter:** ours is the router's own APS counter. Other proxies
    reuse the alias NWK sequence. Fixed in §6.4.

### 6.3 Fixes after 6.2 (hardware re-checked 2026-11-13)

- **APS group → NWK 0xFFFD** (branch `aps-group-fffd`, 6a1b276, based on
  master 7e7342c; cherry-picked here as 94d6127):
  - R22 §2.2.4.1.1: a group-addressed APS frame is a NWK broadcast to the
    rx-on-when-idle devices, `ShortAddress::BROADCAST_RX_ON_WHEN_IDLE`.
  - Host test `group_frames_are_nwk_broadcast_to_rx_on_when_idle_devices`
    fails on master (0xFFFF) and passes with the fix.
  - Every image that sends groupcasts changes. Size delta over the standard
    image set (same worktree path): 0 B everywhere except −4 B `.text` on
    `efr32mg22-sensor` and `nrf52833-ed`; `.data`/`.bss` unchanged.
- **Alias broadcasts recorded in the BTT** (99ebba6, `green-power` only):
  - When a router sends a broadcast under a GP alias, it records
    `(alias, seq)` in the BTT. A neighbour's relay of it then takes the BTT
    duplicate path: it is not delivered or relayed again, and it counts as
    that neighbour's passive acknowledgement (R22 §3.6.5).
  - Host test `a_neighbours_relay_of_an_alias_broadcast_is_its_passive_ack`
    fails without the fix (the echo is delivered back) and passes with it.
    CI now runs the `zigbee-nwk` `router,green-power` tests and clippy.
  - Feature-off images built from the same path as master + 6a1b276 are
    identical (BL702 ×2, EFR32MG1, TLSR8258 ×2) or differ only in panic
    `Location` line/column words (the other 9).
- **GP H2-Zero image** (`router,waveshare-h2-zero,green-power`): app.bin
  515,376 B (sha `4a2a89ad…c996`), ending at 0x8DD30 in ota_0. Against the
  ce7e806 image: `.text` +172, `.rodata` +72, `.data`/`.bss` +0.
- **Hardware re-check (2026-11-13, 00:19–00:44 EET):**
  - App-only flash of `4a2a89ad…c996` to ota_0 at 0x10000 (126 sectors up
    to 0x8E000), read back and sha-verified. Boot at 00:20:01 resumed NWK
    0x8C23 (PAN 0xDFE9, channel 15, parent 0x4A76) without a rejoin and
    restored 1 GP proxy table entry.
  - 25-minute capture: router log, GPD VCOM and the nRF52840 sniffer on
    channel 15 (`2026-11-12T221855Z-gp-h2-99ebba6-flash-sniff`).
  - **No echo relays:** 0x8C23 sent no radius-28/29 copy of any
    operational Notification (cmd 0x00); before the fix every forward had up
    to 4 radius-28 echoes. Its only radius-28/29 frames are ordinary relays
    of other proxies' Commissioning Notifications (cmd 0x04).
  - **Groupcast addressing:** all 77 frames from 0x8C23 carrying our alias
    went to NWK 0xFFFD; all 29 operational Notifications carried APS group
    0x0B84, radius 30.
  - **Copies per forward:** 12 forwards on the air, 1–4 radius-30 copies
    each (29 in total, mean 2.4) about 510 ms apart. These are passive-ack
    retries within `nwkMaxBroadcastRetries` = 3 (R22 default): the router
    stops early once every router neighbour has relayed, otherwise it
    retries. Neighbour relays (0x1AA4, 0x43FD, 0x4A76, 0x670B, 0xD586, …)
    now count as passive acks. Retries continue when a router neighbour,
    for example the weakly linked coordinator, is not heard relaying.
  - **Counters vs air:** 24 GPDFs from 00:20:08 to 00:43:10. The router's
    00:43:01 summary read `forwards sent 11 cancelled 12`, and the 00:43:10
    GPDF was forwarded after it. Every `sent` increment matches a forward
    seen on the air (00:20:08, 00:25:09, 00:26:09, 00:32:09, 00:33:09,
    00:35:10, 00:36:10, 00:38:10, 00:40:10, 00:41:10, 00:42:10, 00:43:10).
    No other proxy originated an operational Notification. The "cancelled"
    reports therefore rest on other proxies' Commissioning Notifications.
  - **HA:** the humidity entity changed every 60 s with no gap, 26 updates
    from 00:19:08 to 00:44:11 EET.
- **HA battery entity "unknown":** this is not a regression. The MG22 lab GPD
  sent no battery attribute (`battery_half_percent: None`; it logged
  "no battery report" at boot). The quirk always adds a Battery entity, so the
  entity has been `unknown` since it was created on 2026-11-11. At each
  Pairing ZHA re-initialises the entities: unavailable → unknown at 22:00:43.
  Addressed by 331f8ea (measured AVDD reported as battery voltage and
  percentage, plus a Battery voltage entity in the quirk); hardware-checked
  in §6.4.

### 6.4 GPD battery on hardware and the alias sequence rule (2026-11-13)

- **MG22 GPD battery (331f8ea image `f7376833…c456`, 42,264 B):**
  - Fresh full backup (`…-de7e3feddaff.bin`), app-only program at 0x0
    (6 sectors, 0x0–0xC000), read back sha-verified. 0x70000, 0x78000 and
    0x7C000–0x80000 read back identical to the backup; 0x74000 differs only
    by the GPD's own counter reservations after the reboots.
  - VCOM 08:42:56 EET: `Resumed SrcID 0x293C9879 counter 2160 channel 15`
    (the old image's last report was fc 2131; each boot reserves 16), then
    `report fc 2160 … Some(3355) mV Some(200) half-%`. A second boot at
    08:43:01 (probe readback) resumed at 2176 and read 3354 mV. No
    re-commissioning.
  - Quirk deployed to `/config/custom_zha_quirks`, ZHA config entry
    reloaded at 08:45:26 (no HA restart). 08:46:02: Battery 100 % and
    Battery voltage 3.4 V; temperature and humidity continue every 60 s.
  - The WSTK supplies 3.3 V (switch at AEM), so 100 % is expected. The
    CR2032 curve (1800–3000 mV) has not been seen on a coin cell.
- **Alias sequence rule:** GP Basic 1.1.2 (CSA 14-0563-19) A.3.6.3.3.2:
  derived groupcast Notification = MAC seq; Pairing Search −10; Tunneling
  Stop −11; Commissioning Notification −12; **commissioned groupcast
  Notification −9**; broadcast Notification −14; Device_annce 0x00. The
  same value is used as the alias NWK sequence number **and** the APS
  counter. GP 1.0a (095499r26) had −13 for the commissioned groupcast;
  our −13 came from 1.0a.
  - EmberZNet (GSDK 4.5 `green-power-common.h`): commissioned groupcast 9,
    Commissioning Notification 12, derived groupcast exact; it sets
    `apsFrame->sequence` to the same value.
  - IKEA (air, `2026-11-12T221855Z`): 66 radius-30 Commissioning
    Notifications from 0x1AA4/0x7B65 all have APS counter = alias NWK
    seq = MAC − 12. Ours (29 operational Notifications) had our own APS
    counter.
  - Fix: `alias_seq::COMMISSIONED_GROUPCAST` = 9, and an alias frame's APS
    counter is the alias seq without consuming the router's own counter
    (`zigbee-aps`, `green-power` only). Host tests
    `alias_sequence_offsets_match_gp_1_1_2`,
    `gp_alias_replaces_nwk_source_and_sequence_for_one_frame` and
    `gp_proxy_tunnels_paired_gpdf_and_restores_its_table` fail without it.
    Feature-off images: 5 of 14 byte-identical, 9 differ only in panic
    locations. GP H2-Zero app image 515,392 B (sha `63b74365…1a5a`),
    `.text` +20 B, `.rodata`/`.data`/`.bss` unchanged.
  - **On hardware (2026-11-13, H2-Zero, 1a58ca8):** app-only flash of
    `63b74365…` to ota_0 (0x10000–0x8E000; fresh backup `c3fc8f77…`),
    readback sha matched. Boot at 09:04:49 EET resumed 0x8C23 (PAN 0xDFE9,
    ch 15) without rejoin and restored 1 GP proxy entry. 14-minute capture
    (`2026-11-13T070350Z-gp-h2-1a58ca8-flash-sniff`, router log + GPD VCOM
    + nRF52840 sniffer):
    - Every radius-30 frame from 0x8C23 for SrcID 0x293C9879 had
      NWK seq = APS counter = GPDF MAC seq − 9 (151→142 … 162→153), dst
      0xFFFD.
    - 12 GPDFs (fc 2199–2210) forwarded; the router's 09:17:49 summary read
      `forwards sent 12 cancelled 0`. fc 2197 arrived during programming and
      fc 2198 during the readback; neither was forwarded. No other proxy
      originated a Notification for this GPD.
    - No radius-28 echo from 0x8C23. 16 radius-30 copies for 12 forwards:
      9 forwards had 1 copy, 3 had 2–3 copies about 515 ms apart. In those
      3 cycles every neighbour except 0x43FD (the next hop) relayed within
      110 ms, and 0x43FD relayed only after the retry. These are
      spec-conforming passive-ack retries; before the echo fix the mean was
      2.4 copies.
    - HA temperature changed every 60 s from 09:04:04 to 09:18:06 with no
      gap.

## 7. Open questions for the user

1. GPD identity: reuse manufacturer 0x6D6B (as on FlowerX GP and BGT1 GP)
   with a new model ID such as 0xF300? Which sensor first (internal
   temperature, SHT/BME on the WSTK expansion header, or a switch)?
2. GPD security: level 3 with a per-device random key encrypted with the
   TC-LK (proposed), or level 0 like the existing sensors?
3. Is the MG22 BRD4183A on the WSTK free for GPD work? Is the TLSR8258 TB-04
   router acceptable as the first proxy, or should a roomier ESP32-C6/H2 come
   first?
4. Can your existing GP sensors (FlowerX GP / BGT1 GP) be moved out of the
   coordinator's range to test our proxy with third-party GPDs? Which stack
   do they run on, and do any use `rxAfterTx`?
5. Do you have, or could you add, a real 802.15.4 sniffer (nRF52840 dongle,
   CC2531) in hwlab for the timing claims?
6. Are the gated NWK/APS alias changes in shared code acceptable? With the
   feature off they compile out.
7. When `green-power` is on, approve the flash and RAM growth on router
   images (table above). Should it later become default-on for routers, as
   Zigbee 3.0 compliance requires?
8. Is Combo/Sink (layer c) wanted on a specific router product, such as a
   light or relay endpoint for direct switch pairing?

## 8. Proposed implementation order

1. `zigbee-gp` frame, security and cluster codecs, with zigpy golden
   vectors (host only).
2. GPD app on MG22, unidirectional: commissioned directly into ZHA, with a
   quirk.
3. MAC `mac_seq`, runtime version-3 demux, NWK/APS alias, Proxy Basic
   unidirectional with `GpTableJournal`, on the TLSR8258 router.
4. Timed-TX PHY capability: gpTxQueue and bidirectional commissioning,
   first on the platform where it can be proven. GPD bidirectional mode.
5. Full proxy (channel switch, unicast modes, key types, IEEE ApplicationID).
6. Combo/Sink with translation table.

Each step ends with host tests, the feature-off byte-identity check, and
size and RAM deltas recorded here before the next step starts.
