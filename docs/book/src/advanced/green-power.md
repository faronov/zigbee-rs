# Zigbee Green Power (design draft)

> **Status: Proxy Basic and the unidirectional GPD app (MG22 sensor and
> switch) are implemented and host-tested on the `green-power` branch;
> nothing is hardware-tested yet.** Bidirectional operation, the full
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
| ESP32-H2 router example `green-power` + 8 KiB `zbnv` journal at 0x3F8000 | builds | not flashed |
| `efr32s2-hal::trng` (MG22 CRYPTOACC BA431 TRNG: GSDK init, conditioning KAT, health-failure reinit) | implemented | host model tests; lab bin `s2-trng` built, not run |
| `apps/gpd` (`gpd-app`): provisioning, `GpdJournal`, beacon scan for a permit-join network, unidirectional commissioning, L3 Multi-Cluster Reports and switch commands, CCA backoff | implemented | host tests: sink-side decode of commissioning (key unwrap), reports and Toggle; counter never reused across reboot; failed reservation transmits nothing; journal rollover / interrupted write / CRC fallback |
| `products/efr32mg22-gpd` + `examples/efr32mg22-gpd` (bins `efr32mg22-gpd-sensor`, `efr32mg22-gpd-switch`) | builds | layout verified; not flashed |
| ZHA quirk `tools/zha-quirks/zigbee_rs_gp.py` | written | byte-compiled only; not loaded in ZHA |
| Bidirectional GPD, full proxy, Combo/Sink | not started | — |

Implementation notes that differ from the design below:

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
   | Operational, entry found | Check security level and key type against the entry, check the frame counter is above the stored one, decrypt (level 3). Then send a GP Notification for each sink group (alias source = derived or entry alias, alias seq = MAC seq), and lightweight/full unicast to each listed sink |
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
   is cancelled if an identical one from another proxy is heard first *(verify
   1.1 rule)*.

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
- Checkpointed lazily: after a counter delta of 256, after 60 s, or on table
  changes.
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
  humidity (0x0405) and optional battery (0x0001/0x0021); switch commands
  Off/On/Toggle (0x20–0x22). Each GPDF is sent twice with the same counter
  (sinks and proxies drop the duplicate), after CCA with random backoff.
- MG22 composition (`examples/efr32mg22-gpd`): commission once at boot if
  needed; BTN0 short press = report now / Toggle; BTN0 held 3 s = forget
  the channel and recommission; the sensor reports every 60 s; EM2 with
  the radio idled between events. Temperature is the die sensor and the
  humidity is **synthetic** (BRD4183A has no humidity sensor).
- Flash map: application 0x0–0x74000 (no bootloader), GPD journal
  0x74000–0x78000, 0x78000–0x80000 never written (0x7C000 keeps the Zigbee
  sensor product's security journal).
- Measured (release): 40,048 B file-backed flash (`.text` 32,728,
  `.rodata` 6,668, `.data` 332), `.bss` 2,312 B; 29.9 KiB linked stack.
  Sensor and switch are the same size.
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
