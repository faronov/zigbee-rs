# Zigbee Green Power (design draft)

> **Status: Proxy Basic and the unidirectional GPD app (MG22 sensor and
> switch) are implemented on the `green-power` branch. The MG22 GPD
> sensor and switch are lab-tested on hardware directly against ZHA; the
> Proxy Basic is lab-tested on the ESP32-H2-Zero router against ZHA
> (2026-11-12, see §6.1). Since 2026-11-13 the Proxy Basic is on by
> default in every router build (§2, §4.1); only the ESP32-H2-Zero router
> is hardware-tested.** Bidirectional commissioning (M4: ESP32-H2
> TempMaster with timed TX, MG22 GPD with `rxAfterTx`) is implemented
> opt-in. The H2's reply timing and the MG22's bidirectional commissioning
> into ZHA are lab-tested on hardware, but through a third-party
> TempMaster (IKEA), not through our proxy. M4 hardware gate 2 was
> accepted as partial on 2026-11-14 (gate 1 + gate 2b, §6.7, §6.9). The full proxy
> is in progress (M5, opt-in). Combo/Sink (layer (c)) is implemented opt-in
> behind `green-power-sink`; it is in no default image. On the ESP32-H2-Zero
> combo light (`combo-light`) it is lab-tested on hardware: a GP switch and
> a bidirectional MG22 sensor commission directly into the sink, the latter
> with the combo as its own TempMaster (3 of 3 rounds, 2026-11-17; §3.7,
> §4.5, §6.10). Spec
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
| Default-on in every router (`router` = `router-core` + `green-power`) with a product-owned Proxy Table partition: ESP32-C6/H2 (`zbnv` 0x3F8000), nRF52840 (0xF8000), nRF52833 (0x78000), EFR32MG21 (0x70000), ZBT-1 (0xF0000), STM32WB5MM-DK (0x080CF000), BL702 (0xF7000), TLSR8258 (0x6E000), mock-light (RAM store) | implemented | every image builds and passes its layout and stack gates; product host tests with and without `green-power`; hardware: ESP32-H2-Zero forwarding (§6); ZBT-1 `zrs-boot` boots with the proxy running, empty table, no forwarding observed (§4.1). The other targets compile only |
| Timed-TX capability (`zigbee-mac/timed-tx`): `RadioPhy::TIMED_TX` / `transmit_at`, `PhyRxFrame::rx_end_us`, `PhyError::Missed`, `MacDriver::timed_tx_supported` / `transmit_raw_at`; ESP backend with an RX_DONE timestamp (vendored esp-radio `ieee802154-rx-timestamp`) | implemented, opt-in | host tests (SoftMac timed payload, mock timed TX); ESP32-H2 lab-tested on air 2026-11-14: reply preamble 19,998–20,007 µs after the end of the GPDF in 4 of 5, 20,222 µs in 1, after calibrating `TIMED_TX_START_LEAD_US` to 425 µs (§6.8). ESP32-C6 not measured |
| TempMaster (`zigbee-runtime/green-power-bidirectional`, router `green-power-bidirectional`): proxy advertises bidirectional only when the MAC has timed TX, sends a queued GP Response GPDF `tx_offset_us` (20 ms) after the end of the triggering GPDF, no CSMA, no ACK | implemented, opt-in (not in any default image) | end-to-end host test: GP Response from the sink → rxAfterTx GPDF → Commissioning Reply scheduled at RX end + 20 ms. Lab-tested on the H2-Zero 2026-11-14 with an injected GPD (radio-lab): forwarded first (+41 ms), appointed TempMaster by zigpy, 10 of 11 Commissioning Replies sent on time, 1 reported `Missed` (§6.8). With a real bidirectional GPD (MG22 sensor) the combo light as its own TempMaster commissioned it 3 of 3 times on 2026-11-17, the reply armed in the radio and started by its interrupts (`MacDriver::arm_gp_reply`, ESP32 only; other MACs keep the task-scheduled reply) (§6.10). With zigpy appointing our proxy for a GPD commissioning into ZHA: not tested |
| GPD bidirectional mode (`gpd-app/bidirectional`, MG22 example feature `bidirectional`, product `BIDIRECTIONAL` policy) | implemented, opt-in | host tests: reply in the window → Success and channel persisted; replies at both edges of the spec window (20 ms and 25 ms + GPDF airtime after the GPDF start) and at the measured IKEA offset received; no reply → stays uncommissioned without counter reuse; wrong security level rejected; delivered key replaces the GPD key; invalid policy rejected. Lab-tested on hardware 2026-11-14 (182b358, `lab-tx-power` build): commissioned into ZHA through IKEA 0x43FD as TempMaster, Success accepted, reports in HA, Resumed after reset with channel and counter (§6.9). **Not yet through our proxy** |
| Full proxy (`zigbee-gp/proxy-full`, `zigbee-runtime/green-power-full-proxy`) | in progress, opt-in (§2 row (b)) | host tests |
| Combo / Sink (`zigbee-gp/sink`, `zigbee-runtime/green-power-sink`, router-app `green-power-sink`): Sink Table, sink commissioning mode, GPD commissioning (direct and Commissioning Notification, bidirectional through a TempMaster), Notification processing, Notification Response, Sink Table Request, sink attributes, built-in GPD → ZCL translation to one local endpoint, Sink Table in the GP journal | implemented, opt-in (not in any default image) | host tests (`zigbee-gp` sink unit tests, zigpy golden vectors, `gp_store` sink journal tests, runtime end-to-end `gp_combo_commissions_a_switch_and_toggles_the_local_light`); ESP32-H2 and nRF52840 router builds (§4.5); first product: Waveshare ESP32-H2-Zero combo light (`combo-light`, On/Off Light endpoint on the WS2812, BOOT opens sink commissioning, TempMaster included; §3.7). **Lab-tested on the H2-Zero (2026-11-15/16, §6.10)**: BOOT window, direct commissioning of a unidirectional GP switch, Toggles to the light, Sink Table across a power cycle. Adaptive counters across a power cycle and the 180 s BOOT timeout: lab-tested 2026-11-17. Bidirectional GPD directly into the sink with the combo as its own TempMaster: **proven on hardware on the ESP32-H2 2026-11-17**: 3 of 3 rounds with the reply started from the radio interrupts (7b2d6ca), after the fixes for zigpy's Response cancelling the combo's reply, the reply ordering and the busy radio (§6.10) |
| GP Pairing Configuration and Translation Table on the sink (`zigbee-gp/translation-table`, `zigbee-runtime/green-power-translation-table`, router-app `green-power-translation-table`; in `combo-light`): Pairing Configuration actions no action / extend / replace / remove pairing / remove GPD with the "send GP Pairing" flag, all communication modes with pre-commissioned group lists, security fields, paired endpoints, application information (validated, not stored); Translation Table Update (add/replace/remove) and Request/Response, per-GPD translations to any local endpoint with a fixed, derived (`0xFE`) or copied (`0xFF`) payload, pass-to-application (`0xFC`); built-in translation kept when no explicit entry exists; gpsFunctionality bits 11 and 18; translations and paired endpoints in the GP journal's sink record (§3.8) | implemented, opt-in (2026-11-19) | host tests: `zigbee_gp::config` codec vectors (gecko_sdk field layout), `zigbee_gp::translation_table`, sink behaviour (`sink::configuration::tests`), journal migration / interrupted write / unknown section (`gp_store::translations`), runtime end to end `gp_translation_tests::pairing_configuration_and_translation_update_retarget_a_gpd_toggle` (unicast Pairing Configuration and Translation Table Update from a coordinator, Default Responses, Translation Table Response, GPD Toggle driving endpoint 2 with On, reboot), `locally_issued_configuration_retargets_a_paired_gpd_without_a_tool`. ESP32-H2-Zero `combo-light` builds. **Lab-tested on the H2-Zero in the ZHA network (2026-11-19, §6.11)** with a locally issued Pairing Configuration (Extend) and Translation Table Update (`lab-gp-translation` image): journal migration from the previous firmware, the MG22 switch's Toggle driving On, the translation across a reset, its removal, and the previous firmware resuming on the journal afterwards (a journal still holding a translation section under older firmware is host-tested only). **Over the air from the ZRHP NCP, lab-tested on an ESP32-C6 combo (2026-11-20, §6.12)**: Pairing Configuration Extend and Remove GPD, Translation Table Update add/remove, Sink and Translation Table Requests, the translation across a reset, an MG22 switch's Toggle driving On |
| GPD Channel Request on the sink (`zigbee-gp/channel-request`, `zigbee-runtime/green-power-channel-request`, router-app `green-power-channel-request`; in `combo-light` and the ZRHP NCP): in commissioning mode a Channel Request heard directly, or tunnelled in a GP Commissioning Notification / GP Notification from SrcID 0 with rxAfterTx, is answered with a GP Response carrying a Channel Configuration (operational channel, Basic) for the TempMaster; repeats within 1 s share one GP Response; gpsFunctionality bit 12 (GPD maintenance); `EVT_GP_FRAME` result 6 on the NCP (§3.9) | implemented, opt-in (2026-11-21) | host tests: `sink::maintenance` (direct, Commissioning Notification and Notification paths, refusals, repeat suppression, gpsFunctionality), runtime end to end `gp_channel_request_tests` (combo as its own TempMaster delivers the Channel Configuration by timed TX in the GPD's window). **Not run on hardware** |
| gpsSecurityLevel InvolveTC (`zigbee-gp/involve-tc`, `zigbee-runtime/green-power-involve-tc`, router-app `green-power-involve-tc`; in `combo-light` and the ZRHP NCP): the bit is writable; `gp_sink_derive_involve_tc` sets it after an install-code join, clears it in a distributed network and keeps it after a default-key join (gecko_sdk `writeInvolveTCBit`); while set a sink refuses GP Sink Commissioning Mode (UNSUP_COMMAND), local commissioning and Commissioning GPDFs (`SinkDrop::InvolveTc`, `EVT_GP_COMMISSIONING_FAILED` reason 14); a sink that is the Trust Center (the NCP) is not stopped (§3.10) | implemented, opt-in (2026-11-21) | host tests: `sink::involve_tc` (write, refusal paths, Trust Center sink, derivation), runtime `gp_involve_tc` (install-code join, default-key join, Trust Center), NCP `GP_SINK_CONFIG` accepts bit 3. **Not run on hardware** |
| GPD key provisioning on the sink (`zigbee-gp/key-provisioning`, `zigbee-runtime/green-power-key-provisioning`, router-app `green-power-key-provisioning`; in `combo-light` and the ZRHP NCP): gpSharedSecurityKeyType / gpSharedSecurityKey (write-only key) / gpLinkKey (write-only), the shared key handed to a GPD that asks for one in a bidirectional commissioning (network key, NWK-key derived group key, GPD group key, derived individual key, as gecko_sdk `sendCommissioningReply`), protected with gpLinkKey when the GPD uses key encryption or gpsSecurityLevel requires it, the network key always protected; out-of-band keys for GPDs that commission without sending theirs, and a re-commissioning without a key keeps the entry's key; `GP_SET_SHARED_KEY` on the NCP (§3.11) | implemented, opt-in (2026-11-21) | host tests: `sink::keys` (GP spec vectors A.1.5.7.1, A.1.5.7.2, A.1.5.12.1 and A.1.5.8.3 from gecko_sdk, attributes, provided key with TC-LK protection and Success under the new key, own key kept without a request, out-of-band keys), runtime end to end `gp_key_provisioning_tests` (the combo hands the network key, protected, by timed TX), NCP `GP_SET_SHARED_KEY`. **Not run on hardware** |
| GPD Application Description on the sink (`zigbee-gp/app-description`, `zigbee-runtime/green-power-app-description`; in the ZRHP NCP, not in `combo-light`): a GPD whose Commissioning announces "application description follows" waits (20 s) until all its report descriptors arrive in Application Description commands (direct or in GP Commissioning Notifications), then pairs, with a Commissioning Reply when the last report asked for one; new descriptors become `SinkEvent::ApplicationDescription` and, on the NCP, `EVT_GP_APP_DESCRIPTION`; with `translation-table` also GP Pairing Configuration action 0b101 and Extend/Replace carrying descriptors (§3.12) | implemented, opt-in (2026-11-21) | host tests: `zigbee_gp::app_description` codec, `sink::descriptions` (collection, timeout, repetitions, refusals, bidirectional reply, Pairing Configuration), NCP end to end `application_description_reaches_the_host_before_the_pairing`, zigbee-zrhp vector `evt_gp_app_description`. **Not run on hardware** (no multi-sensor GPD in the lab) |
| GP Pairing Search (sink side `zigbee-gp/pairing-search`, `zigbee-runtime/green-power-sink-pairing-search`, router-app `green-power-sink-pairing-search`, in `combo-light` and the ZRHP NCP; proxy side `zigbee-runtime/green-power-pairing-search`, router-app `green-power-pairing-search`, in no image): the sink answers a search with GP Pairing for a Sink Table entry in a requested communication mode, unicast to the searching proxy, frame counter and key only when requested; the proxy broadcasts a search for a GPD whose data GPDF it dropped for want of a Proxy Table entry, at most one per minute and the same GPD again after 15 minutes (§3.13) | implemented, opt-in (2026-11-21) | host tests: `sink::pairing_search` (gecko_sdk field layout, request bits, no answer without a match), runtime end to end `pairing_search::tests` (router proxy without an entry searches, the coordinator's sink answers, the proxy stores the pairing and tunnels the next GPDF; rate limits). **Not run on hardware** |
| Remote GP provisioning on the ZRHP NCP (`GP_PAIRING_CONFIGURATION`, `GP_SEND_PAIRING`, `GP_TRANSLATION_UPDATE`/`REQUEST`, `GP_REMOTE_SINK_TABLE_REQUEST`, `GP_REMOTE_PROXY_TABLE_REQUEST`, `EVT_GP_REMOTE_*`, [ZRHP 7.11](host-protocol.md#711-green-power-0x0axx-endpoint-2)) | implemented (2026-11-19) | host tests: zigbee-zrhp vectors (`generate.py --check`), NCP request mapping, NCP end to end against a second zigbee-rs router stack (`remote_provisioning_reaches_another_proxy_and_sink`). H2 NCP image builds and passes the stack gate. **Lab-tested on hardware 2026-11-20 (§6.12)**: master 2ab8ac0 NCP on the Thread BR board's H2 (921 600, RTS/CTS) provisioned an ESP32-C6 combo over the air with `GP_PAIRING_CONFIGURATION`, `GP_TRANSLATION_UPDATE`, `GP_TRANSLATION_REQUEST` and `GP_REMOTE_SINK_TABLE_REQUEST`, each answered (`EVT_GP_REMOTE_*`) within 50–85 ms; host tool `tools/zrhp-host` `--gp-*`. `GP_SEND_PAIRING` and `GP_REMOTE_PROXY_TABLE_REQUEST` not run on hardware |
| Adaptive frame-counter persistence (`zigbee_gp::counter`, Proxy and Sink Table, exact flag in the GP journal's counter records) | implemented, in every GP image | host tests (§3.3); lab-tested on the H2-Zero combo 2026-11-17: after a power cycle the switch's first presses toggled the light, none dropped (§6.10) |
| GP combo on the ZRHP coordinator NCP (`zigbee-runtime/green-power-ncp` with `ncp-capacity`, `apps/ncp-coordinator` `gp`, `examples/esp32-ncp`): Proxy Basic + sink (Sink Table 16, Proxy Table 8) + TempMaster, sink authority switchable at runtime (`NCP_SINK` / `HOST_SINK`), NCP default `gpsSecurityLevel` 0x00 with no translation endpoint, GP journal in the NCP's `zbnv` window (2 × 8 KiB), all GREEN POWER commands and events of [ZRHP 7.11](host-protocol.md#711-green-power-0x0axx-endpoint-2) | implemented (2026-11-19) | host tests: `zigbee-gp` import/quiet-removal unit tests, runtime `green_power::ncp` unit tests, NCP end-to-end `apps/ncp-coordinator/tests/gp.rs` over the host link (commissioning of a level-3 GPD heard directly, reports, a proxy's GP Notification, table read/put/remove, `CONFLICT`, key-export lock, `HOST_SINK` raw frames and local proxy output), journal geometry tests in `products/esp32-zigbee-ncp`; H2 and C6 NCP images build and pass the stack gate. **Not run on hardware** (ZRHP gate H10) |

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
- The proxy and the sink persist each GPD's security frame counter
  adaptively (`zigbee_gp::counter`, §3.3): event commands (switch presses)
  are written exactly, every accepted frame, within a write budget, so a
  reboot loses none; reports stay batched with a stride
  (`counter_persist_stride`, default 64) and skip ahead by stride − 1 on
  restore. No frame accepted before a power loss is accepted again.
- A counter-only change is journalled as a 32-byte compact record instead
  of a new 1 KiB snapshot, about 24× less flash wear (§3.3).
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

The protocol crates keep every GP flag **off by default**. Zigbee 3.0
requires Proxy Basic on every router, so the router builds turn it on:

- Multi-role examples (ESP32-C6/H2, nRF52833/40, EFR32MG21, ZBT-1, BL702,
  TLSR8258): `router = ["router-core", "green-power"]`. The GP-off router
  is `--no-default-features --features router-core` (BL702 and TLSR8258:
  `--features router-core` / `scripts/tlsr8258.sh build router-core`).
- STM32WB5MM-DK: `green-power` is in the example's default features;
  `--no-default-features` builds the router without GP.
- ZBT-1 with the `zrs-boot` loader: `router,zrs-boot` includes GP. Loader
  layout version 2 shrank both slots by 8 KiB (cap 465,664 B) and reserves
  the GP window `0xF0000..0xF4000` below the router tables, so direct and
  loader-linked routers share the same Proxy Table journal. The GP-off
  loader build is `router-core,zrs-boot`.
- Sensors, EDs, SEDs and the GPD apps are unchanged.

The GP-off builds must stay byte-identical to master's routers (apart from
panic `Location` line/column fields); §4.1 records the proof.

| Layer | Feature (`zigbee-gp` / `zigbee-runtime` / `router-app`) | Contents |
|---|---|---|
| (a) Proxy Basic | `proxy-basic` / `green-power-proxy` / `green-power` | dGP stub (version-3 demux, GPDF codec, security processing for proxy-table entries), proxy table plus journal, duplicate filter, commissioning mode, Commissioning Notification, Notification in derived/pre-commissioned groupcast and lightweight unicast, Pairing, Proxy Commissioning Mode, Proxy Table Request/Response, gppFunctionality attributes, GP 1.1.1 tunnelling delay (A.3.6.3.1). Unidirectional only. |
| (a+) Basic with bidirectional commissioning | `proxy-bidir-commissioning` | gpTxQueue (≥ 5 entries, one per GPD), GP Response handling in the TempMaster role, timed transmission on the operational channel, Commissioning Reply. *(verify 1.1: possibly mandatory for Basic)* |
| (b) Full proxy | `proxy-full` (implies the above) | **In progress (M5):** protected IEEE ApplicationID 2 GPDF authentication/forwarding; full-unicast Pairing and Notification Response matching; bounded retries with readable/writable retry attributes; temporary channel switch for a Maintenance Channel Configuration (A.3.9.1 step 8: the radio moves to the GPD's transmit channel, serves only its Channel Request, defers network transmissions and returns after delivery or 5 s); strict GP Tunneling Stop payload codec. gppFunctionality adds full unicast and GPD IEEE address. Tunneling Stop is a GP 1.0 optional command that the Silicon Labs GP 1.1 client neither sends nor handles, so no behaviour is attached to it. Pairing Search is opt-in (§3.13); Proxy Table maintenance is deferred: neither zigpy nor the Silicon Labs client uses them. Remaining: Combo/Sink and counter-reservation optimization. |
| (c) Combo / Sink | `sink` (implies `proxy`) / `green-power-sink` (implies `green-power`) / `green-power-sink` | **Implemented opt-in, host-tested; lab-tested on the ESP32-H2-Zero combo light (§3.7, §6.10):** GP server on GPEP: Sink Table plus journal, sink commissioning mode (local API or GP Sink Commissioning Mode, window, exit on first pairing, GP Proxy Commissioning Mode to the proxies), direct GPDF reception, Notification and Commissioning Notification handling, bidirectional commissioning (GP Response with Commissioning Reply, Success), decommissioning, GP Pairing and alias Device_annce generation, GP Notification Response, Sink Table Request/Response, sink attributes 0x0000–0x0007, built-in translation (Off/On/Toggle → On/Off, Level Control Move/Step/Stop) to one local application endpoint. With `translation-table` / `green-power-translation-table` / `green-power-translation-table` (opt-in, §3.8): GP Pairing Configuration, Translation Table Update/Request and per-GPD translations to any local endpoint. With `channel-request` / `green-power-channel-request` (opt-in, §3.9): GPD Channel Request answered with a Channel Configuration. With `involve-tc` / `green-power-involve-tc` (opt-in, §3.10): gpsSecurityLevel InvolveTC derived from the join. With `key-provisioning` / `green-power-key-provisioning` (opt-in, §3.11): keys handed to GPDs and out-of-band GPD keys. With `app-description` / `green-power-app-description` (opt-in, §3.12): GPD Application Description collected before the pairing and handed to the application. With `pairing-search` / `green-power-sink-pairing-search` (opt-in, §3.13): GP Pairing Search answered with GP Pairing. **Deferred:** GPM commissioning |
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
- **Timed TX (bidirectional only):** the TempMaster sends the queued GPDF
  without CSMA/CA and without an ACK request, starting gpTxOffset after
  the *end* of the GPDF that carried `rxAfterTx`. GP 1.1 sets this to
  20 ms (GP 1.0 had 5 ms); ZBOSS (`ZB_GPD_RX_OFFSET_MS` 20) and EmberZNet
  (`GP_RX_OFFSET_USEC` 20500 from the RX time stamp) agree. The GPD
  listens for at least gpdMinRxWindow = 0.576 ms. Implemented in M4 as an
  optional capability behind `zigbee-mac/timed-tx`:
  - `RadioPhy::TIMED_TX` (associated const, default `false`) and
    `RadioPhy::transmit_at(frame, start_us)`;
  - `PhyRxFrame::rx_end_us` / `MacFrame::rx_end_us()`: end of the
    received frame on the `PlatformServices::monotonic_micros` clock;
  - `MacDriver::timed_tx_supported()` and `transmit_raw_at()`;
  - `PhyError::Missed` when the instant has passed or is too far ahead.

  A backend without it returns `Unsupported`. The runtime asks
  `timed_tx_supported()` before every GPDF; without it the proxy clears
  the bidirectional bits in gppFunctionality and Commissioning
  Notification and rejects a GP Response that appoints it TempMaster, so
  it never claims support it lacks. Mechanisms:

  | Platform | Expected mechanism |
  |---|---|
  | nRF52840 | RADIO + TIMER/PPI |
  | EFR32 | RAIL scheduled TX |
  | ESP32-C6/H2 | **implemented (M4), built for ESP32-H2 only:** RX_DONE timestamp in the vendored esp-radio, then a timer wait plus a short spin before `transmit_raw`; start lead `TIMED_TX_START_LEAD_US` (150 µs, to be calibrated on air) |
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
4. On the first transmission the proxy defers by the GP 1.1.1 tunnelling
   delay (A.3.6.3.1): Dmin_u = 5 ms (rxAfterTx = 0) or Dmin_b = 32 ms
   (rxAfterTx = 1), plus (3 − Link quality) × 32 ms. The TempMaster for the
   GPD and, in full unicast, a FirstToForward entry use the shortest delay;
   the v1.0a distance formula (Dmin 15 ms, up to ~100 ms for most links) is
   no longer used. Link quality is banded from RSSI (≥ −70 dBm Excellent,
   ≥ −85 High, ≥ −95 Moderate, else Poor), which matches what commercial
   proxies report on air (§6.8). A pending Notification is cancelled if an
   identical one from another proxy is heard first (A.3.5.2.1); for
   rxAfterTx frames only if that proxy is bidirectional-capable, its TX queue
   is not full when reported, and it has a better link (or an equal link and
   a lower address). Commissioning Notifications do not report the TX-queue
   state. `Proxy::forward_stats()` counts forwards sent and cancelled,
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
- Sink-table records share the journal as a distinct record kind (`ZBGS`,
  implemented, §3.7); translation records are not implemented.
- Leaving the network or a factory reset clears the journal.

**Proxy-side and sink-side GPD frame counter: adaptive persistence**

Implemented 2026-11-16 (`zigbee_gp::counter`, used by the Proxy Table and
the Sink Table), after the H2-Zero combo showed the batched-only cost on
hardware: after a power cycle the GP switch looked dead for ~55 presses
(§6.10). User decision: rare GPDs (switches) are written on every
accepted frame, frequent ones (sensors) stay batched.

- Per accepted protected frame: an *event* command (GPD command ID below
  0xA0: Off/On/Toggle, level, scenes, door lock, generic switch, ...) is
  written exactly while a per-table token bucket allows it
  (`exact_counter_burst` 16, refilled one per `exact_counter_refill_ms`
  60 s, so at most ~1,440 exact writes a day per table); reports
  (0xA0..), manufacturer commands and events beyond the budget are
  batched. Every mode change is written together with the counter:
  entering exact writes the current counter; leaving exact writes the
  current counter as batched, so the batched floor covers what follows.
- The stored mode (exact flag) belongs to the stored counter. Restore uses
  the stored counter itself when exact, `stored + N − 1` when batched.
  A new entry (GP Pairing or sink commissioning) starts exact; its first
  report switches it to batched with one write.
- Journal: the exact flag is bit 0 of the counter record's byte 6 (written
  zero and never read by earlier firmware, inside the record CRC). A full
  record carries no flags, so after a full record the journal immediately
  appends a counter record per exact entry ("re-assert") and never places a
  full record in a sector's last slot while one must follow (it rolls
  over). Journals without flags, and a torn re-assert, restore batched,
  which is safe; earlier firmware reading a new journal restores batched.
- Wear (2 × 4 KiB `zbnv` sectors; a counter record is 32 B; with the sink
  a full record is a 2 KiB slot pair, so 63 records fit per sector before
  a full rewrite erases the other sector; Proxy-only: 95):

  | load | records/day | erases/sector/day | 100k-cycle SPI NOR (ESP32-H2) | 10k-cycle internal flash |
  |---|---:|---:|---:|---:|
  | switch 1,000 presses/day, sink + proxy entry (2 records/press) | 2,000 | 15.9 | 17 years | 1.7 years |
  | switch 1,000 presses/day, proxy only | 1,000 | 5.3 | 52 years | 5.2 years |
  | sensor every 60 s (batched) | 22.5 | 0.18 | > 100 years | 150 years |
  | budget ceiling, both tables (2 × 1,440 + fallbacks) | ~2,900 | ~23 | 12 years | 1.2 years |

  A switch pressed 1,000 times a day is an extreme load; products on
  10k-cycle internal flash (nRF52840, EFR32) that expect it should raise
  `exact_counter_refill_ms` or give the GP journal more sectors.
- Host tests: `zigbee_gp::counter` (budget, reboot after 1/2/64/65 frames
  in both modes, mode changes with a reboot after every frame never accept
  a counter twice), proxy and sink reboot tests with real GPDFs in both
  modes and beyond the budget, journal tests (flag round trip, flag-only
  change as a counter record, legacy and torn flags restore batched, full
  record leaves room for the re-assert), runtime end-to-end (the sink's
  first Toggle after a journal reboot is accepted).

**Batched mode (reports, events beyond the budget)**

- Authoritative in RAM.
- Checkpointed lazily: when it has advanced by `counter_persist_stride`
  (`N`, default 64) since the last written value, or with any other table
  change. GPDFs and GP Pairings share the stride.
- A GP Pairing that changes nothing but the counter, and does not complete
  the stride, is not written. A counter decrease (a re-commissioned GPD) is
  always written, so the stored value never exceeds the live one.
- Skip-ahead on restore: every counter accepted since the last write is
  below `stored + N`, so `Proxy::restore` sets a protected entry's counter
  to `stored + N − 1` (saturating) and keeps `stored` as the persisted
  value. Replays of anything seen before the reboot are dropped as stale;
  the first frame above the floor is forwarded and written at once. Entries
  without security (MAC sequence window) are restored unchanged.
- Cost: after a router reboot up to `N − 1` genuinely new frames per
  batched GPD are rejected until the GPD's counter passes the floor: about
  an hour for a sensor reporting every 60 s. Switches no longer pay it
  (exact mode above).
- Remaining window: a frame accepted after its write was triggered but
  before the journal write in the same router loop pass completed. The sink
  checks counters again (zigpy keeps `frame_counter`).
- Host tests (`zigbee-gp` proxy tests): reboot skip-ahead rejects replays
  of every pre-reboot counter and accepts the first frame above the floor;
  a reboot after a stride write uses the new floor; stride 1 restores the
  exact counter; saturation at `u32::MAX`; unsecured entries keep their
  counter. The runtime and `router-app` lifecycle tests restore through
  the journal.

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

**Compact counter records** (`zigbee-runtime/src/gp_store.rs`,
`GP_COUNTER_RECORD_SIZE` = 32). A change that only moves security frame
counters no longer rewrites the 1 KiB snapshot. The journal appends one
32-byte record per changed entry after the newest full record, in the
same sector:

| Bytes | Field |
|---|---|
| 0..4 | magic `ZBGC` |
| 4 | version 1 |
| 5 | entry index in the full record |
| 6..8 | zero |
| 8..12 | generation of the full record it amends |
| 12..16 | GPD tag: CRC32 of the entry's encoded GPD ID bytes |
| 16..20 | frame counter (LE) |
| 20..24 | CRC32 of bytes 0..20 |
| 24..28 | commit marker `CMIT`, written after the body |
| 28..32 | left erased |

- On load the newest full record is read, then the 32-byte units after
  it are walked up to the first fully erased unit. A record is applied
  only if its CRC and commit marker are valid and its generation, index
  and GPD tag match an entry that carries a counter
  (`ProxyEntry::encoded_layout`). Torn, corrupt or foreign records are
  skipped; a later valid record wins.
- `commit` compares the new snapshot with the stored one with the counter
  bytes masked. Identical: nothing is written. Only counters differ: one
  record per changed entry, each read back. Anything structural (entry
  added or removed, flags, groups, key, a re-pair) or a full sector: a new
  full record with a new generation, which makes every older counter
  record stale, rolling over to the other sector as before.
- The first GPDFs after a pairing set `in_range` and `first_to_forward`
  (Table 39 flags) without a write of their own, so the next stride write
  after a pairing is one full record; after that only counter records.
- Geometry: the 24-byte body and the 4-byte marker must be multiples of
  the flash write size and the 32-byte unit of the read size. All current
  products qualify (write size ≤ 4); other geometries keep the full-record
  path only.
- Compatibility: the full-record format is unchanged, so the GP
  partitions already written by the earlier firmware (H2-Zero, ZBT-1
  0x4A76) load as-is and start taking counter records. Older firmware
  ignores `ZBGC` units and never writes over them (it only uses fully
  erased slots).
- Downgrade caveat: an older image restores only the full record's
  counter plus its skip-ahead (`N − 1`), so a counter recorded only in
  compact records more than `N − 1` past that value could be accepted
  again by the older image. Downgrading the router firmware therefore
  reopens a replay window for frames sent since the last full record.
- Host tests (`gp_store`): compact records fill a sector (96 per 4 KiB)
  then roll over; an unchanged snapshot is not written; a structural
  change drops older counter records (including a higher counter from
  before a re-pair); torn and corrupt records are skipped; records for
  another generation, index or GPD are ignored; a hand-built legacy
  journal loads byte-identical and accepts counter records; end to end
  with `Proxy`, real GPDFs and AES: reboot skip-ahead from a compact
  record rejects every replay and accepts the first frame above the
  floor.

**Flash endurance.** A full record is a 1024-byte slot; the rest of the
sector takes 32-byte counter records. A 4 KiB sector therefore takes 1
full + 96 counter updates per erase (8 KiB: 1 + 224), against 4 (8)
updates per erase before. Each GPD causes one counter update per `N`
frames. Years until 10,000 erase cycles per sector (internal flash:
nRF52, EFR32, STM32WB); the ESP32, TLSR8258 and BL702 SPI flash is rated
for 100,000 cycles, ten times longer:

| GPDs reporting every 60 s | `N` = 16 | `N` = 64 (default) | `N` = 64, full records only (before) |
|---|---:|---:|---:|
| 1, 4 KiB sectors (nRF52, STM32WB, ESP32, TLSR, BL702) | 59 y | 236 y | 9.7 y |
| 1, 8 KiB sectors (EFR32MG21, ZBT-1) | 137 y | 548 y | 19.5 y |
| 10, 4 KiB sectors | 5.9 y | 23.6 y | 0.97 y |
| 10, 8 KiB sectors | 13.7 y | 55 y | 1.9 y |

`N` = 64 stays the default: it was chosen to keep one 60 s GPD near ten
years with full records only, and with compact records it leaves margin
for ten GPDs, structural changes and re-pairs. A product with a
switch-only, proxy-only deployment can now pick a small `N` (fewer
frames lost after a reboot) at an acceptable wear cost.

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

Implemented: the product owns a `GpProxyJournal` over its GP partition and
the composition root attaches it with `set_green_power_store`; a router
built with `green-power` and no store fails `GreenPowerStoreMissing`
instead of running without persistence. The Device_annce alias defence
draws its delay from the NWK routing jitter PRNG, not
`MacDriver::fill_random`: on TLSR8258 `fill_random` links CTR_DRBG and
software AES, which the router image must not contain.

Design notes:

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

### 3.7 Combo / Sink (as implemented, opt-in)

No GP specification text is available. Wire formats and behaviour follow
the certified Silicon Labs GP 1.1 sink (gecko_sdk `app/zcl/green-power.xml`;
`protocol/zigbee/app/framework/plugin/green-power-server/`
`green-power-server.c`, `green-power-security.c`,
`green-power-server-gpd-cmd-maps.c`) and zigpy; the code names the gecko_sdk
routine each path mirrors.

**Layers.** `zigbee_gp::sink::Sink<N, Q>` is sans-IO like the proxy: it
takes direct GPDFs (`on_gpdf`), GP server commands (`on_server_command`),
attribute reads and writes, and the local commissioning API, and the caller
drains network frames (`poll`), translated commands (`take_command`), GPEP
group changes and application events. `zigbee_gp::translation` holds the
built-in GPD → ZCL map. `zigbee-runtime` (`green-power-sink`) owns a
`Sink<8, 4>` next to the proxy (`gp_sink*` API).

**Decisions.**

- A GPDF heard directly goes to the proxy *and* to the sink, each with its
  own security check, counter and duplicate filter. The proxy still
  tunnels it for other sinks (for example ZHA through the shared Proxy
  Table); nothing is tunnelled to this node.
- The sink's own GP Pairing is *not* looped back into the local Proxy
  Table (gecko_sdk does that because its sink only sees GPDFs through its
  proxy). Other sinks' Pairings still populate it, so multi-sink forwarding
  is unchanged and a single-sink combo sends no useless groupcast per press.
  The sink's GP Proxy Commissioning Mode is not looped back either.
- The sink needs neither the bidirectional nor the full proxy, nor a
  timed-TX MAC: `green-power-sink` implies only `green-power`, so it builds
  on every router MAC (nRF52840 included). Bidirectional commissioning
  through another proxy as TempMaster always works. For a GPD the combo
  hears directly, `green-power-bidirectional` is needed as well: then a GP
  Response from the sink that appoints this node TempMaster is handed to
  the local proxy (its gpTxQueue sends the Commissioning Reply), because
  the broadcast does not come back to the sender, and gpsFunctionality
  bit 6 is advertised only while the MAC can transmit at an instant.
  Without it such a GPD is refused (`NoTempMaster`).
- Translated commands are dispatched through the runtime's local ZCL
  dispatcher to the configured endpoint as locally originated, non-unicast
  commands (source: own NWK address, endpoint 242; Disable Default
  Response), so no ZCL response goes on air and the application sees the
  usual `CommandReceived` event. Untranslated commands (attribute reports,
  scenes, generic switch, …) are `SinkEvent::GpdCommand` events.
- Group membership of GPEP is shared: neither table leaves a group the
  other still needs.
- Defaults are the gecko_sdk attribute defaults: derived groupcast, exit on
  first pairing, 180 s window and **gpsSecurityLevel 0x01: only GPDs with
  security level 2 or 3 are commissioned. A level-0 (unsecured) GPD is
  refused (`SecurityPolicy`) unless the product opts in by setting
  `SinkConfig::security_level` to 0** (or a sink writes attribute 0x0005).
  This is a deliberate choice (user decision 2026-11-15): an unsecured
  switch can be replayed or spoofed by anyone in radio range. Commissioned
  groupcast requires a product-configured group. Written attributes are
  volatile.
- Protected GPDs must send their key (TC-LK wrapped or clear): this sink
  provisions none. Security level, key type and frame counter are checked
  for direct GPDFs and for Notifications (gecko_sdk
  `emberAfGreenPowerClusterGpNotificationCallback`); unprotected entries
  rely on the 2 s duplicate filter, as gecko_sdk and zigpy do.
- GP Notification Response is sent to unicast GP Notifications: for full
  unicast pairings with FirstToForward for the proxy whose copy arrived
  first, and with NoPairing for a GPD without a pairing. gecko_sdk's sink
  does not send it; the semantics are those our proxy (`proxy-full`)
  expects.
- A Commissioning Notification with "security processing failed" (for
  example the Success of a bidirectional commissioning, which the proxy
  cannot open yet) is verified and decrypted with the pending key after
  rebuilding the GPDF header (gecko_sdk `prepareHeader`). SrcID GPDs only.
- Protected Sink Table counters use the adaptive persistence of §3.3
  (exact for events, batched with stride 64 for reports); unprotected
  sequence numbers are not persisted.

**Persistence.** The GP snapshot carries the Sink Table after the Proxy
Table and `GpProxyJournal` stores both:

- a snapshot with sink entries is a pair of slots in one sector: a `ZBGS`
  sink record (full-record layout), then the `ZBGP` Proxy Table record of
  the same generation, flagged as paired in byte 7 (zero and unchecked in
  the original format); the `ZBGP` commit marker commits both;
- load pairs only a flagged newest `ZBGP` with the `ZBGS` of the same
  generation in the slot before it, so a torn pair or an orphan sink
  record left by an interrupted write is never applied;
- sink counters use the 32-byte counter records with bit 7 set in the
  entry index;
- a snapshot without sink entries is written exactly as before, existing
  proxy journals load unchanged, and firmware without the sink skips
  `ZBGS` slots and sink counter records (host-tested in the sink-less
  build). A 4 KiB sector holds two pairs instead of four full records.

**Validation.** Host only: `zigbee-gp` sink unit tests (entry codec and the
gecko_sdk layout, commissioning mode, direct and tunnelled commissioning,
bidirectional commissioning with a tunnelled Success, security policy,
translation, replay and duplicate handling across proxies, Notification
Response, decommissioning, Sink Table Request, attributes, restore
skip-ahead); zigpy golden vectors (the sink's GP Proxy Commissioning Mode
enter/exit, GP Pairing and GP Response with Commissioning Reply are
byte-identical to zigpy's, and zigpy-built Commissioning Notifications and
Notifications are consumed); `gp_store` sink journal tests; and the runtime
end-to-end test (sink commissioning mode → Commissioning GPDF heard
directly → Sink Table entry and GP Pairing on air → secured Toggle switches
a local On/Off cluster → replay rejected → a GP Notification from another
proxy toggles it back and its duplicate is dropped → two reboots through
the journal keep the entry and its counter floor).

**Hardware.** Lab-tested on the H2-Zero combo below (§6.10): BOOT opens
sink commissioning and times out after 180 s, a unidirectional GP switch
commissions directly, Toggles drive the On/Off Light, the Sink Table and
an exact switch counter survive a power cycle (first press after it
works).

**First product: Waveshare ESP32-H2-Zero combo light (`combo-light`,
implemented, host-tested, lab-tested on hardware, §6.10).** Build:
`examples/esp32h2-sensor`, bin `esp32h2-router`,
`--no-default-features --features router,waveshare-h2-zero,combo-light`.

- Features: example `combo-light = ["green-power-bidirectional",
  "router-app/green-power-sink", "esp32-zigbee-devkit-product/combo-light"]`;
  product `combo-light = ["status-led", "green-power-bidirectional",
  "router-app/green-power-sink", "zigbee-runtime/green-power-sink"]`. Off
  by default; `router` is unchanged. Since 2026-11-16 the combo includes
  the TempMaster (`green-power-bidirectional`): on hardware the MG22's
  bidirectional sensor build could not commission into the sink without
  it (the sink dropped its directly heard Commissioning with
  `NoTempMaster`, §6.10).
- Profile: the reusable `zigbee_runtime::profile::OnOffLight` (Basic,
  Identify, On/Off server; the runtime's `OnOffCluster`, no Level, no
  Groups/Scenes; OnOff reported on change with a 1 h heartbeat). The
  product's router profile is `DeviceProfile<OnOffLight>` on the existing
  endpoint 1 with device ID On/Off Light (0x0100) instead of Range
  Extender (0x0008); `COMBO_SINK_ENDPOINT` = 1 is set as the sink's
  translation target in the composition root. The profile's non-generic
  methods are `#[inline]`: a plain (non-inline) module in
  `zigbee-runtime` changed the code generation of the default router
  image by −18 B `.text` although nothing used it.
- LED: the product's `RgbStatusLed` remains the only owner of the RMT
  WS2812. `combo_color(status, light_on)`: a lit status indication always
  wins; otherwise the light shows dim warm white `(24, 14, 4)` while On and
  is dark while Off. After joining, the green joined hold (30 s) comes
  first. An open GP sink commissioning window blinks like permit join
  (blue, 1 s on / 1 s off).
- BOOT (GPIO9): pressed while joined it calls
  `gp_sink_enter_commissioning(true)` (proxies included, exit on the first
  pairing, and always after gpsCommissioningWindow 180 s: a locally opened
  window times out even though the Silicon Labs default exit mode is "on
  first pairing" only; GP Sink Commissioning Mode from the network keeps
  the exit mode exactly). On hardware before this change a BOOT window
  stayed open for hours (§6.10). Ignored while a window is open; the existing
  press-to-recommission after Leave is unchanged. Sink events
  (commissioning entered/left, commissioned, untranslated GPD commands
  such as attribute reports) are logged on the console; the 60 s summary
  adds the Sink Table size and the light state.
- OnTime/OffWaitTime advance in 100 ms ticks from the run loop. The OnOff
  state is volatile: the light starts Off after every reset.
- Persistence: none new; the Sink Table shares the `zbnv` GP journal at
  0x3F8000 with the Proxy Table.
- Measured size (`tools/mac-footprint.py`):

  | image | flash | `.bss` |
  |---|---:|---:|
  | `router,waveshare-h2-zero` (2026-11-15) | 521,330 | 68,116 |
  | `router,waveshare-h2-zero,combo-light` (2026-11-15, no TempMaster) | 553,370 | 70,220 |
  | `router,waveshare-h2-zero` (2026-11-16, adaptive counters) | 522,124 | 68,136 |
  | `router,router-app/green-power-sink` (2026-11-16) | 548,984 | 69,776 |
  | `router,waveshare-h2-zero,combo-light` (2026-11-16, + TempMaster, adaptive) | 562,864 | 70,684 |

  First combo light over the H2-Zero router: +32,040 B flash, +2,104 B
  `.bss` (the estimate was +28–31 KB), mostly the router app and node
  instantiated for the new profile type, the On/Off command handler
  (678 B), `Combo::service` (1,294 B) and the `SinkEvent` log formatter
  (486 B). Adding the TempMaster and adaptive counters: +9,494 B flash,
  +464 B `.bss` (the TempMaster alone was +6,278 B / +432 B on the
  router, §4.3). Combo light now: +41,534 B flash, +2,568 B `.bss` over
  the 2026-11-15 H2-Zero router; app image 562,960 B of the 2,031,616 B
  `ota_0`.
- Adaptive counters in the default GP routers: ESP32-H2(-Zero) router
  +794 B flash / +20 B `.bss`, bidirectional +960 / +20, sink-only
  +1,484 / +32, nRF52840 router +916 / +16, nRF52840 sink +1,124 / +32.
- Feature-off identity (same checkout path): 2026-11-15 against
  e8a6fa3/f5fa17c: ESP32-H2 router, ESP32-H2-Zero router, `router-core`,
  bidirectional and sink-only H2 routers differ only in 28–33 panic
  Location words; the nRF52840 router is identical. 2026-11-16 (adaptive
  counters change every GP image by design): ESP32-H2 `router-core`
  differs only in 28 panic Location words, nRF52840 `router-core` is
  identical.
- Not covered: GP endpoint 242 is not in the node's Active Endpoint list or
  simple descriptors (as for the proxy); ZHA does not need it for a proxy
  or for this sink, but a GP 1.1 combo should advertise it.

### 3.8 Pairing Configuration and Translation Table (opt-in)

A commissioning tool (the zigbee-rs hub through the ZRHP NCP, ZRHP 7.11
"Remote GP provisioning", or any GP 1.1 tool) pairs GPDs to a combo and
retargets their commands remotely. This is the "GP binding" zigpy lacks.
Formats and behaviour follow gecko_sdk 4.5 (`green-power.xml`, the
generated `zcl_decode_green_power_cluster_gp_pairing_configuration_command`,
`green-power-server.c` `emberAfGreenPowerClusterGpPairingConfigurationCallback`,
`green-power-translation-table.c`); no GP specification text is
available, as for §3.7.

**Code.** `zigbee_gp::config` holds the codecs (GP Pairing Configuration,
Translation Table Update/Request/Response, and decoders of the Sink and
Proxy Table Responses a tool receives); it is compiled with `sink`.
`zigbee_gp::translation_table` holds the bounded table
(`TranslationTable<T>`, 16 entries on a combo, 4 with
`compact-translation-table`, which `ncp-capacity` selects because the
16-entry Sink Table leaves little journal room). The sink's handlers live in
`zigbee_gp::sink::configuration`. `zigbee-runtime` needs no new hook: the
commands go through the existing GP server dispatch, and translated commands
through `gp_sink_deliver` to whatever local endpoint they name.

**GP Pairing Configuration** (client → server `0x09`):

| Action | Behaviour |
|---|---|
| 0b000 no action | With "send GP Pairing": re-send the GP Pairing of the entry (`NOT_FOUND` without one) |
| 0b001 extend | Create the entry, or update an entry of the same communication mode keeping its key and counter when the command has none; commissioned groups are merged (at most 2, else `INSUFFICIENT_SPACE`); translations are kept |
| 0b010 replace | A fresh entry; the GPD's translations are dropped |
| 0b011 remove pairing | When the communication mode and every listed group match (`NOT_FOUND` otherwise): remove the entry and its translations; with "send GP Pairing" one GP Pairing (AddSink 0, RemoveGPD 0) per group or for this node's unicast address |
| 0b100 remove GPD | Remove the entry and its translations; GP Pairing (RemoveGPD) only with "send GP Pairing" |
| 0b101 application description | `FAILURE`: no report descriptors are kept. Extend/replace announcing one (application information bit 5) is refused the same way |

- Communication mode, MAC sequence capability, RxOnCapability, fixed
  location, assigned alias, groupcast radius, security level, key type,
  key and frame counter come from the command. A level below
  gpsSecurityLevel is `FAILURE` (gecko_sdk silently ignores it); a reserved
  level or GPD ID is `INVALID_FIELD`; a protected level without a key type
  is `INVALID_FIELD`; involve-TC is `FAILURE`.
- "Send GP Pairing" broadcasts the GP Pairing (AddSink) and the alias
  `Device_annce`, as after a commissioning. Without it nothing is sent:
  the tool pairs the proxies itself (`GP_SEND_PAIRING`).
- `NumberOfPairedEndpoints`: 0x00 no endpoint (GPD commands are only
  reported to the application), 0xFE and 0xFF the sink's endpoint
  (`SinkConfig::endpoint`), a list of up to 4 endpoints (1–240): the
  built-in translation drives each listed endpoint. 0xFD is
  `INVALID_FIELD`.
- Application information (manufacturer and model ID, GPD commands,
  clusters, switch information) is parsed and validated, not stored.
- The sink's own looped-back command is ignored (as gecko_sdk). The local
  proxy is not put into commissioning mode: the combo hears its GPDs
  directly.
- A configured pairing raises `SinkEvent::Commissioned`, a removal
  `SinkEvent::Decommissioned`.

**Translation Table** (Update `0x07`, Request `0x08`, Response `0x08`):

- An entry is (GPD ID, GPD command, endpoint, profile, cluster, ZCL
  command, payload). Payload length `0x00`–`0x08` is a fixed payload,
  `0xFE` the payload the built-in translation derives from the GPD command
  (for example Step's mode, step size and default transition), `0xFF` the
  GPD payload copied (at most 8 bytes; a longer one is skipped and counted
  in `SinkStats::translations_dropped`). Endpoint `0xFC` passes the
  command to the application untranslated.
- The Index field is ignored (GP 1.1 sends 0xFF). Add appends (identical
  entries are not duplicated); Replace removes every entry of the same
  (GPD ID, GPD command, endpoint, profile, cluster) and adds the supplied
  ones; Remove removes the entries of the supplied quintuples (removing
  nothing is `SUCCESS`, as in gecko_sdk). An update is validated as a
  whole before anything changes (gecko_sdk applies it entry by entry):
  `INSUFFICIENT_SPACE` when it does not fit or a payload is longer than 8
  bytes, `INVALID_FIELD` for endpoint 0 or 0xF1–0xFE except 0xFC,
  `FAILURE` for an additional information block (generic switch contact
  masks, compact attribute reporting).
- Translations exist only for paired GPDs: an update for a GPD without a
  Sink Table entry is `NOT_FOUND` (gecko_sdk accepts it), and removing the
  entry removes its translations. So they are always persisted with, and
  never outlive, their pairing.
- A GPD command with explicit entries is translated by every one of them,
  in table order (up to 4 queued commands per GPDF); a command without
  explicit entries keeps the built-in translation (paired endpoints
  above). Explicit entries win over the built-in map per command, so
  retargeting Toggle keeps On and Off on the light.
- Translation Table Request is answered only when unicast (gecko_sdk):
  the entries from the start index that share the first entry's
  ApplicationID and fit one frame, `NOT_FOUND` past the end. There is no
  Translation Table attribute in the GP 1.1 server attribute set.
- gpsFunctionality adds bit 11 (CT-based commissioning) and bit 18
  (Translation Table), so a hub can detect the capability.
- Groupcast: a pre-commissioned group list joins the GPEP groups like a
  commissioning; a proxy's groupcast GP Notification to such a group is
  translated by every member sink (host test
  `groupcast_notifications_are_translated_by_member_sinks`).

**Persistence and migration.** The translations and the non-default
paired endpoints are a section behind the Sink Table payload *inside the
same `ZBGS` record*: `"TT"`, version 1, a 16-bit body length, the
translations (length-prefixed `TranslationEntry` encodings) and the
paired-endpoint records (Sink Table index, kind, endpoints). Byte 7 of the
sink record (zero and unchecked so far) flags it with `0x02`; the record
header length still counts only the Sink Table payload and the record CRC
covers the section, so one commit marker covers both:

- an interrupted write leaves the previous pair, translations included;
- a journal written before the feature (or without translations) loads
  with an empty Translation Table: the migration needs no step, and the
  next structural change writes the section;
- firmware without the feature reads a new journal's Sink Table and Proxy
  Table and ignores the section;
- a section of an unknown version or bad length is ignored with a warning,
  not trusted and not fatal;
- translation changes are structural (a new record pair); counter-only
  changes stay compact counter records.

With 8 Sink Table entries and 16 translations the worst-case record is
20 + 369 + 495 B of the 1,012 B before the CRC (a compile-time check); with
`ncp-capacity` (16 entries, 4 translations) 20 + 737 + 227 B.

**Local configuration.** `ZigbeeDevice::gp_sink_configure_locally`
lets the node's own application issue a GP Pairing Configuration or GP
Translation Table Update (a product provisioning step, or the
`lab-gp-translation` image of `examples/esp32h2-sensor`). The command takes
the receive path of a broadcast from the unassigned NWK address 0xFFFE (so
the sink's drop of its own looped-back Pairing Configuration does not
apply): it is validated, applied and journaled like a remote command, and
no ZCL response is produced; the caller inspects `gp_sink()` afterwards.

**Feature-off identity.** Without the feature no code changes: the
runtime hooks are `cfg` blocks placed where shifted lines move no panic
`Location` compiled into router images, and the sink-record buffer size is
a line-neutral constant. The ESP32-H2-Zero, ESP32-C6 and nRF52840 router
images are byte-identical to master cf12e65 (§4.6).

### 3.9 GPD Channel Request on the sink (opt-in)

A GPD that does not know the network's channel (a factory-new switch
without a scan, or one that lost its channel) toggles channels and sends
Channel Request GPDFs: Maintenance frames without a SrcID, one payload byte
naming the channel it listens on next (low nibble) and the one after (high
nibble), channel - 11. A sink in commissioning mode answers with a GP
Response for GPD ID 0 carrying a Channel Configuration (operational channel
- 11, Basic bit 0x10). The TempMaster transmits it on the GPD's next receive
channel when the GPD's next Channel Request opens the window; the GPD then
commissions on that channel.

**Code.** `zigbee_gp::sink` module `maintenance` (feature `channel-request`):

- **Direct reception (combo):** the runtime already passes every GPDF to
  the sink; a Maintenance frame without a GPD ID now reaches
  `on_maintenance_gpdf` instead of being dropped. The TempMaster is this
  node's own proxy, so the combo needs `green-power-bidirectional` (timed
  TX); without it the request is dropped with `NoTempMaster`.
- **Through proxies:** a Channel Request tunnelled in a GP Commissioning
  Notification (or GP Notification) must come from SrcID 0 with rxAfterTx
  set; the TempMaster is the forwarding proxy (its `ProxyInfo` short
  address), as gecko_sdk `handleChannelRequest` does. Any other GPD ID is
  `Invalid`, a missing receive window is `NoTempMaster`.
- **Policy:** only in sink commissioning mode (gecko_sdk answers tunnelled
  requests at any time; proxies forward them only in commissioning mode).
  A GPD repeats its request several times per channel: the same answer for
  the same TempMaster and channel is not queued again within 1 s
  (`REPEAT_SUPPRESSION_MS`).
- **Delivery off the operational channel:** the TempMaster must switch to
  the GPD's receive channel (`proxy-full`, A.3.9.1 step 8). The combo light
  has no full proxy: it delivers only when the GPD listens on the
  operational channel, and leaves other channels to a full-proxy
  TempMaster.
- gpsFunctionality adds bit 12 (GPD maintenance). The NCP reports an
  answered request as `EVT_GP_FRAME` result 6 with SrcID 0.

Proxy forwarding on routers was already in place: a Proxy Basic in
commissioning mode tunnels a Channel Request in a GP Commissioning
Notification, and a bidirectional proxy delivers a Channel Configuration
for SrcID 0 from its gpTxQueue on the GPD's next Channel Request (§3.2,
`proxy` host tests).

### 3.10 gpsSecurityLevel InvolveTC (opt-in)

GP 1.1 reserves bit 3 of gpsSecurityLevel for a commissioning that
involves the Trust Center; no GP 1.1 sink implements such a procedure (the
ZBOSS API documents it as always zero). gecko_sdk nevertheless maintains
the bit and acts on it, and a commissioning tool can write it, so a sink
that talks to Silicon Labs tools behaves the same way here:

- **Derivation** (`ZigbeeDevice::gp_sink_derive_involve_tc`, gecko_sdk
  `writeInvolveTCBit` at the first NETWORK_UP): an install-code
  (non-default preconfigured) Trust Center link key at the join sets it,
  a distributed network clears it, the default ZigBeeAlliance09 key leaves
  the configured value. The join kind comes from the persisted
  `bdbNodeJoinLinkKeyType`, so a resumed node derives the same value. The
  combo light derives it once per join; a later write by a tool sticks
  until the next join.
- **Effect:** GP Sink Commissioning Mode is answered with UNSUP_COMMAND,
  the local commissioning API fails and Commissioning GPDFs (direct or
  tunnelled) are dropped with `SinkDrop::InvolveTc`.
- **Trust Center sinks:** a sink that is itself the Trust Center is not
  stopped (`Sink::set_trust_center`; the ZRHP NCP sets it). The TC is
  involved by construction; its host decides which GPDs it keeps. This
  differs from gecko_sdk, which refuses commissioning on any node with the
  bit set. The NCP's `GP_SINK_CONFIG` therefore accepts bit 3.
- Without the feature the bit stays read-only zero in writes
  (INVALID_VALUE) and a configured bit refuses commissioning, as before.

### 3.11 GPD key provisioning (opt-in)

Until now the sink only accepted the key a GPD sent in its Commissioning
GPDF (in clear or TC-LK protected). With `key-provisioning` it also
provides keys and accepts keys known out of band (`zigbee_gp::sink`
module `keys`, following gecko_sdk `sendCommissioningReply`,
`emberAfGreenPowerServerDeriveSharedKeyFromSinkAttribute` and
`sli_zigbee_af_gp_key_tc_lk_derivation`):

- **Shared key attributes.** gpSharedSecurityKeyType (0x0020),
  gpSharedSecurityKey (0x0021) and gpLinkKey (0x0022, default
  ZigBeeAlliance09) are writable over ZCL; on the NCP the host sets the
  first two with `GP_SET_SHARED_KEY`. The two keys are write-only: a read
  returns UNSUPPORTED_ATTRIBUTE (gecko_sdk returns them to any node).
- **Key for a GPD that asks.** A protected GPD that sets "GP security key
  request" in a bidirectional Commissioning GPDF gets the shared key in the
  Commissioning Reply when gpSharedSecurityKeyType is set and differs from
  the GPD's own key type: type 1 the network's active key (the runtime
  keeps it current), type 3 HMAC-AES-MMO(network key, "ZGP"), type 2 / 4
  gpSharedSecurityKey, type 7 HMAC-AES-MMO(gpSharedSecurityKey, SrcID or
  GPD IEEE). The Sink Table entry takes that key and key type; the
  pairing completes on the GPD's Success protected with it.
- **Protection.** The key is TC-LK protected with gpLinkKey and frame
  counter = the GPD's outgoing counter + 1 (A.1.5.8.3) when the GPD set
  key encryption in its Commissioning GPDF, when gpsSecurityLevel requires
  protected keys (bit 2), and always for the network key: gecko_sdk would
  send the network key in clear to a GPD that did not ask for encryption.
  gpLinkKey is also used to unwrap a GPD's protected key (gecko_sdk always
  uses ZigBeeAlliance09).
- **Out-of-band keys.** `Sink::provision_oob_key` holds up to 4 keys of
  GPDs whose key is known out of band (label, QR code); a protected GPD
  that commissions without sending a key uses it once. A GPD that
  re-commissions without a key keeps its Sink Table entry's key. On the
  NCP the host imports such a GPD directly with `GP_SINK_TABLE_PUT`.
- Vectors: the GP spec vectors of gecko_sdk
  `green-power-security-test-vectors.c` for the NWK-key derived group key
  (A.1.5.7.1), the derived individual key by SrcID and by IEEE address
  (A.1.5.7.2, A.1.5.12.1) and the protected Commissioning Reply key
  (A.1.5.8.3).

### 3.12 GPD Application Description on the sink (opt-in)

A multi-sensor or Compact Attribute Reporting (CAR) GPD describes its
reports before it is paired: its Commissioning command sets "application
description follows" (Application Information bit 5) and Application
Description commands (`0xE4`) follow, each with the total number of reports
and one or more report descriptors (report ID, optional timeout, data point
descriptors: cluster, server/client, optional manufacturer, attribute
records with ID, type, offset in the report and optional fixed value). The
format is decoded by `zigbee_gp::app_description` (strict: every
descriptor is validated, `MIN_REPORT_LEN` 10 as gecko_sdk).

**Sink behaviour** (`zigbee_gp::sink` module `descriptions`, feature
`app-description`; gecko_sdk `gpCommissioningNotificationApplicationDescriptionGpdf`
and `saveReportDescriptor`):

- A unidirectional Commissioning with the flag does not pair at once: its
  entry waits in the pending list and the sink collects descriptors for
  `APP_DESCRIPTION_TIMEOUT_MS` (20 s, gecko_sdk's multi-sensor timeout).
  Repeated Commissioning frames keep what was collected. One GPD is
  collected at a time (gecko_sdk has one timer); another GPD's collection
  replaces it. rxAfterTx together with the flag stays refused, as in
  gecko_sdk.
- Application Description commands are accepted directly or in GP
  Commissioning Notifications (not in GP Notifications), authenticated
  with the pending entry's key when protected. Report IDs must be below the
  total (at most `MAX_REPORTS` 32), the total must not change. Each frame
  with reports not seen before becomes one
  `SinkEvent::ApplicationDescription` (GPD, total, the new descriptors as
  received); repetitions add nothing.
- When all reports are known the pairing completes (GP Pairing, events as
  any commissioning). If the frame that completed it had rxAfterTx, the
  sink sends a Commissioning Reply through that TempMaster and pairs on
  the GPD's Success (gecko_sdk `lastReportIdHasRxAfterTx`). Other GPD
  commands do not complete a collecting commissioning.
- After the timeout the pending entry is dropped and
  `SinkEvent::CommissioningFailed` reports `SinkDrop::DescriptionTimeout`
  (ZRHP reason 15). Leaving commissioning mode ends the collection.
- The sink does not interpret the descriptors (no CAR translation): the
  application or the ZRHP host does, from the events. CAR frames of the
  paired GPD reach the host as before (`EVT_GP_FRAME`).
- With `translation-table`: GP Pairing Configuration Extend / Replace with
  application information bit 5 is accepted when its descriptors are well
  formed (and reported as an event after the pairing); action 0b101 adds
  descriptors to the GPD being collected (completing its pairing) or
  reports them for a paired GPD, `NOT_FOUND` for an unknown GPD,
  `INVALID_FIELD` without descriptors. gecko_sdk's own 0b101 broadcast
  omits the application-information bit its decoder requires; such a frame
  is `INVALID_FIELD` here.

**Deviation:** gecko_sdk keeps the descriptors in RAM to build CAR
translations; this sink keeps only the received-report bitmap and hands
the descriptors out, so nothing grows the GP journal.

**NCP.** The ZRHP NCP enables it: `EVT_GP_APP_DESCRIPTION` (`0x0A89`)
carries each event, a frame that only advanced the collection is
`EVT_GP_FRAME` result 5 (decrypted with the pending entry's key), and the
completing frame result 2 with `EVT_GP_COMMISSIONED`.

**Images.** Hooks in `sink.rs` / `sink/configuration.rs` are line-neutral
`#[inline(always)]` calls into `sink/descriptions.rs`; the
`combo-light` image (without the feature) is byte-identical (§4.10).

### 3.13 GP Pairing Search (opt-in)

A proxy that hears a GPD it has no Proxy Table entry for (its table was
lost, or it joined after the GPD was commissioned) can ask the sinks with
GP Pairing Search (client → server `0x01`: options with the ApplicationID
and request bits for unicast, derived groupcast and pre-commissioned
groupcast sinks, the frame counter and the key; then the GPD ID). The codec
is `zigbee_gp::cluster::PairingSearch` (gecko_sdk `GpPairingSearch`
layout). Neither gecko_sdk (weak callback only, its client never sends it)
nor zigpy implements a behaviour; this one is ours:

- **Sink** (`zigbee_gp::sink` module `pairing_search`, feature
  `pairing-search`): a search for a GPD with a Sink Table entry in one of
  the requested communication modes is answered with GP Pairing (AddSink)
  built like the sink's commissioning GP Pairing
  (`sendGpPairingFromSinkEntry`), sent **unicast to the searching proxy**
  rather than broadcast (only that proxy lacks the entry); the frame
  counter and key are included only when requested. Unknown GPDs, other
  modes and the sink's own looped-back search get no answer.
- **Proxy** (`zigbee-runtime` `green_power/pairing_search.rs`, feature
  `green-power-pairing-search`): a data GPDF the proxy dropped with
  `NoEntry` (outside commissioning mode) queues a broadcast search (every
  sink kind, counter and key), sent after the proxy's own frames. GPDs of
  neighbouring networks are heard too, so searches are bounded: one per
  `SEARCH_GAP_MS` (60 s) from this node, the same GPD again after
  `SEARCH_REPEAT_MS` (15 min), the last `RECENT` (4) GPDs remembered. The
  GP Pairing answer is processed as any GP Pairing; the GPD's next GPDF is
  tunnelled. The state is per device (`GreenPowerState`), not persisted.
- The sink side is in `combo-light` and the ZRHP NCP. The proxy side is in
  no image: our proxies persist their tables, the search is a recovery
  path, and a router image should not broadcast for foreign GPDs unless a
  product chooses to.

**Images.** The proxy hooks (`pairing_search::on_gpdf` / `poll`, the
`GreenPowerState` field) are line-neutral and compile to the previous code
without the feature: the ESP32-H2 and ESP32-C6 router and nRF52840 router
images are byte-identical (§4.11).

## 4. Size estimates

§4.1 has the measured Proxy Basic cost; the rest of this section is the
original estimate.

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

### 4.1 Measured: Proxy Basic default-on (2026-11-13)

Release, `--locked`, the CI toolchains, measured with
`tools/mac-footprint.py` (`flash` = file-backed allocated bytes, `stack` =
static direct-call bound). Each row compares the default `router` with
`router-core` built from the same tree.

| Router image | flash GP / core | Δ flash | Δ `.bss` | Δ stack | Limit and fit |
|---|---:|---:|---:|---:|---|
| ESP32-H2 devkit | 516,568 / 485,490 | +31,078 | +2,296 | +32 | `ota_0` 2,031,616 B |
| ESP32-H2-Zero (Waveshare) | 515,514 / 483,066 | +32,448 | +2,296 | +32 | `ota_0` 2,031,616 B |
| ESP32-C6 | 532,642 / 501,688 | +30,954 | +2,296 | +32 | `ota_0` 2,031,616 B |
| nRF52840 | 389,352 / 361,884 | +27,468 | +2,304 | +40 | app 1,015,808 B (to 0xF8000) |
| nRF52833 | 389,924 / 362,492 | +27,432 | +2,304 | +40 | app 491,520 B (to 0x78000) |
| EFR32MG21 | 382,468 / 355,528 | +26,940 | +2,048 | +48 | app 442,368 B (0x4000–0x70000) |
| ZBT-1 (direct map) | 382,652 / 355,400 | +27,252 | +2,048 | +48 | app 966,656 B (0x4000–0xF0000) |
| STM32WB5MM-DK plain | 415,858 / 388,114 | +27,744 | +2,304 | +32 | app 847,872 B (to 0x080CF000) |
| STM32WB5MM-DK all peripherals (default) | 477,574 / 449,370 | +28,204 | +2,304 | +32 | as above |
| BL702 XT-ZB1 | 339,150 / 312,142 | +27,008 | +2,296 | +32 | app 482,304 B; stack gate gap 3,840 B (min 3,072) |
| TLSR8258 TB-04 | 287,809 / 264,260 | +23,549 | +2,304 | +16 | raw `.bin` 317,248 / 293,708 B of 450,560 (to 0x6E000); SVC gap 4,696 B (min 4,096); every chain budget +16 B and still inside its unchanged budget |

All gates pass: the TLSR8258 router stack gate (with the GP addendum
`tools/tlsr8258-router-gp-stack-gate.args`), the BL702 stack gate, the
EFR32MG21/ZBT-1 layout checkers and the linker assertions on every GP
partition. The TLSR8258 router links no CTR_DRBG or software AES. The
TLSR8258 sensor and ED share the router's `memory.x`, so their flash limit
dropped by 8 KiB to 0x6E000; they use about 194 KB.

GP stays off where the router does not link on master (EFR32MG22,
CC2340R5, PB-03F, PHY6222, EFR32MG1). The ZBT-1 `zrs-boot` loader build
has GP since loader layout version 2 (signed image 385,871 B in the
465,664 B cap, `.bss` 41,232 B, stack 56,048 B). On 2026-11-13 it was
installed on the lab ZBT-1 (router `0x4A76`): loader boot, journal and
network resumed, GP proxy started with an empty table, and a 0.2.0→0.2.1
upgrade through the shrunk slots confirmed (see the bootloader chapter,
"ZBT-1 layout v2 with Green Power"). No GPD was paired to it, so
forwarding on the ZBT-1 is not yet observed.

Feature-off identity against master 67044d2 (firmware-identical to
dd44367). Both trees were built from the same checkout path, because
absolute source paths are embedded in panic messages. 51 images were
compared, with each master `router` image paired with the branch's
`router-core` image:

- 14 images are byte-identical: BL702 ED and sensor, the CC2340R5 ED,
  EFR32MG1, PHY6222 and PHY6252, and every TLSR8258 image including the
  router.
- 29 differ only in panic `Location` line/column fields. These are the
  sensors, EDs and SEDs, and the STM32WB GP-off router.
- 8 routers (BL702, EFR32MG21, ZBT-1, ESP32-C6, ESP32-H2 and H2-Zero,
  nRF52833/40) are not byte-identical. The cause is the feature rename:
  `router-core` instead of `router` changes the example crate's metadata
  hash, which changes the link order. They contain the same symbols with
  the same sizes, and the text, rodata, data and bss totals are equal.
  Once alignment padding is counted, the file-backed size changes as
  follows:
  - EFR32MG21: −4 B;
  - H2-Zero: +4 B;
  - ZBT-1: +8 B;
  - the other five: unchanged.

  `.bss` is unchanged.

### 4.2 Measured: compact counter records (2026-11-14)

Master 42f37b5 against the branch, both built from the same checkout
path, 62 images. All 49 images without `green-power` (sensors, EDs,
SEDs, every `router-core`) are byte-identical. The GP routers grow:

| Router | `.text` | `.rodata` | `.bss` |
|---|---:|---:|---:|
| ESP32-C6, ESP32-H2 | +2,858 / +2,860 B | +272 B | +32 B (stack −32 B) |
| ESP32-H2-Zero | +2,674 B | +268 B | +32 B (stack −32 B) |
| nRF52833, nRF52840 | +2,328 / +2,332 B | +272 B | +32 B |
| EFR32MG21, ZBT-1 | +2,332 B | +272 B | 0 |
| STM32WB5MM-DK | +2,252 B | +272 B | +32 B |
| BL702 | +2,488 B | +8 B | +36 B |
| TLSR8258 | +2,464 B | 0 | +40 B |

The `.bss` growth is the cached base (per-entry counters and the append
offset).

### 4.3 Measured: bidirectional commissioning, opt-in (2026-11-14)

Not enabled in any default image. Release builds of the M4 branch:

| Image | `.text` | `.rodata` | `.data` | `.bss` | Flash | RAM |
|---|---:|---:|---:|---:|---:|---:|
| ESP32-H2-Zero router: `router,waveshare-h2-zero,green-power-bidirectional` vs `router,waveshare-h2-zero` | +6,278 B | +292 B | −24 B | +432 B | +6,546 B | +408 B |
| MG22 GPD (sensor and switch): `bidirectional` vs default | +3,900 B | +464 B | 0 | 0 | +4,364 B | 0 |
| MG22 GPD sensor, start-referenced receive window (182b358) vs 07db74a: `bidirectional` / `bidirectional,lab-tx-power` | | | | | +24 B (46,676 / 46,996 B file) | 0 |

The default MG22 sensor and switch (42,288 B) are unchanged by 182b358
except for 28 `core::panic::Location` line fields in `apps/gpd`.
`lab-tx-power` (07db74a, never in a default image) sets the radio to
−25 dBm (+320 B over `bidirectional`).

The router's stack use with the feature was not measured; the queued
GPDF (`GpDownlink`, about 60 B) is held on the receive path until it is
sent.

Feature-off identity (master e8c8060 against the M4 diff, both built in
the same checkout path): ESP32-H2 router and `router-core`, ESP32-H2-Zero
router, MG22 GPD sensor and switch and ZBT-1 `zrs-boot` router differ
only in `core::panic::Location` line fields of edited files; every other
section is byte-identical.

### 4.4 Measured: GP 1.1.1 tunnelling delay and Link-quality bands (2026-11-14)

This change is in the default-on proxy. Release builds, before against
after, in the same checkout path (flash = code + rodata + data):

| Image | Flash | RAM |
|---|---:|---:|
| ESP32-H2-Zero router `router,waveshare-h2-zero` | −78 B | 0 |
| ESP32-H2-Zero router `+green-power-bidirectional` | +12 B | 0 |
| ZBT-1 `router,zrs-boot` | −52 B | 0 |
| Nucleo-WB55 router (default) | −64 B | 0 |
| MG22 GPD sensor (default) | byte-identical | 0 |

The follow-up TX-queue suppression and timed-TX deadline checks were also
measured in the same checkout path against the preceding M4 build.
`llvm-size` reports:

| Image | `text` delta | `data` / `bss` delta |
|---|---:|---:|
| ESP32-C6 router | −4 B | 0 / 0 |
| ESP32-H2-Zero router | −4 B | 0 / 0 |
| ESP32-H2-Zero router `+green-power-bidirectional` | 0 | 0 / 0 |
| nRF52833 / nRF52840 router | −4 / −4 B | 0 / 0 |
| EFR32MG21 router | −4 B | 0 / 0 |
| ZBT-1 `router,zrs-boot` | −4 B | 0 / 0 |
| Nucleo-WB55 router (default) | −4 B | 0 / 0 |
| WB55 USB dongle router | −4 B | 0 / 0 |
| STM32WB5MM-DK router (default) | 0 | 0 / 0 |
| BL702 router | 0 | 0 / 0 |
| TLSR8258 router | −24 B | 0 / 0 |

The GP-off H2 `router-core` image has the same section sizes as the
previously saved build; its only byte differences are 11
`core::panic::Location` line numbers in `radio_phy.rs`.

### 4.5 Measured: Combo / Sink, opt-in (2026-11-15)

Release builds of `gp-sink`, `tools/mac-footprint.py report` (flash =
file-backed bytes; stack = static direct-call bound). ESP32-H2 router:
`examples/esp32h2-sensor`, bin `esp32h2-router`, nightly-2026-08-01.
nRF52840 router: `examples/nrf52840-router`, bin `nrf52840-router`.

| Image (`--no-default-features --features …`) | Flash | `.text` | `.data` | `.bss` | Stack bound |
|---|---:|---:|---:|---:|---:|
| H2 `router` | 521,338 | 441,810 | 3,728 | 68,116 | 29,648 |
| H2 `router,router-app/green-power-sink` | 547,500 | 466,544 | 3,936 | 69,744 | 29,648 |
| H2 `router,green-power-bidirectional,router-app/green-power-full-proxy` | 531,948 | 451,892 | 3,728 | 70,076 | 29,680 |
| H2 the same `+router-app/green-power-sink` | 558,802 | 477,170 | 3,936 | 71,704 | 29,664 |
| nRF52840 `router` | 393,164 | 331,232 | 316 | 40,800 | 20,216 |
| nRF52840 `router,router-app/green-power-sink` | 414,412 | 351,116 | 316 | 42,432 | 20,208 |

So the sink costs **+26,162 B flash / +1,628 B `.bss` on the H2 router,
+21,248 B / +1,632 B on the nRF52840 router**, and +26,854 B / +1,628 B
over the H2 full proxy. That is still above the §4 estimate (+8–12 KB,
+1.0–1.6 KB RAM). The nRF52840 router builds with the sink since the
sink no longer needs timed TX.

Trimming, measured on H2 full proxy + sink and nRF52840 + sink:

| Step | H2 flash | H2 `.bss` | nRF flash | nRF `.bss` |
|---|---:|---:|---:|---:|
| First implementation (full proxy + sink; nRF did not build) | 561,146 | 72,248 | — | — |
| Sink queues frame intents, encodes them in `poll` | −416 | −544 | −188 | −544 |
| Sink frames share the GP reply slot (one transmit path) | −98 | 0 | −140 | 0 |
| Journal: one entry-layout walker, header check, record writer and masked compare for both tables; no snapshot copy in validation | −1,232 | 0 | −1,172 | 0 |
| `finalize` moves the entry instead of cloning it | −598 | 0 | −464 | 0 |
| Result | 558,802 (−2,344) | 71,704 (−544) | 414,412 | 42,432 |

Tried and not kept: a sink-only commissioning parser (−28 B H2, −116 B
nRF, but a second copy of the codec) and a shared `dyn` attribute-read
loop for proxy and sink (−38 B H2, +48 B nRF). The rest is the sink's own
behaviour: GP server command handling (~2.7 KB), GPD processing and
commissioning (~2.6 KB), frame encoding (~1.9 KB), Sink Table codec and
journal (~3 KB), attribute read/write (~1.7 KB), local ZCL delivery
(~0.7 KB) and codecs the proxy never encodes (Pairing, Proxy
Commissioning Mode, Response, Notification Response).

Sink Table capacity (not changed, 8 entries stays the default): with 5
entries flash is unchanged (H2 full + sink +22 B, H2 sink +78 B, nRF −84 B)
and `.bss` drops by 212–224 B. The capacity only sizes static tables.

`.bss`: the `Sink<8, 4>` lives in the statically allocated device (Sink
Table of 8, 4 queued frame intents, 3 events with up to 104-byte GPD
payloads, 2 translated commands, 2 pending commissionings, its duplicate
filter); a GP snapshot on the save/restore path is about 380 B larger.

Feature-off identity against gp-m5 f169c61, both built in this worktree
(same absolute path), at gp-sink e8a6fa3: `nrf52840-router`,
`nrf52840-router-core`, `esp32h2-router`, `esp32h2-router-core` and the
opt-in H2 `green-power-bidirectional` and `green-power-full-proxy`
routers have equal section addresses and sizes and identical
`.text`/`.data`; every differing word (21–69 per image, all in
`.rodata`) is the line or column field of a `core::panic::Location` whose
file pointer names a `.rs` path. `tools/mac-footprint.py compare`
reports +0 for every metric of these six images.

### 4.6 Measured: Pairing Configuration and Translation Table (2026-11-19)

Release builds in one checkout path, nightly-2026-08-01, master cf12e65 →
`gp-translation`; `tools/mac-footprint.py report` (flash = file-backed
bytes, stack = static direct-call bound):

| Image (`--no-default-features --features …`) | Flash | `.text` | `.data` | `.bss` | Stack bound |
|---|---:|---:|---:|---:|---:|
| H2-Zero `router,waveshare-h2-zero` | identical | | | | |
| H2-Zero `router,waveshare-h2-zero,combo-light`, master | 569,500 | 485,864 | 4,144 | 71,132 | 29,744 |
| the same, `gp-translation` | 586,754 | 502,158 | 4,144 | 71,908 | 29,744 |
| H2 NCP `esp32h2,transport-uart,thread-br-h2`, master | 707,696 | 641,616 | 7,640 | 208,080 | 34,624 |
| the same, `gp-translation` (remote provisioning; no translation table) | 721,594 | 655,478 | 7,632 | 208,124 | 34,624 |

The combo light grows by **+17,254 B flash / +776 B `.bss`** (Pairing
Configuration and Translation Table handling, their codecs, the 16-entry
table, the journal section), the NCP by **+13,898 B / +44 B** (the
request encoders, the response decoders and the event writers). Static
stack bounds are unchanged; the NCP keeps its stack-gate budgets (serve
chain 30,720 B of 30,720 B, as before). The combo's GP snapshot on the
save/restore path grows by the 495-byte translation section, which is
encoded in place.

Feature-off identity: `router,waveshare-h2-zero` (ESP32-H2-Zero),
`esp32c6-sensor` `router` and `nrf52840-router` `router` built from the
same checkout path before and after are byte-identical in every allocated
section (addresses, sizes and contents), not only up to panic `Location`
words.

The local configuration API (`ZigbeeDevice::gp_sink_configure_locally`)
and the `lab-gp-translation` hook added afterwards leave the production
`combo-light` image byte-identical in every allocated section (the generic
method is not instantiated unless called). The lab image is 591,504 B as
an app image (`espflash save-image`), +4,656 B over the 586,848 B
production app image.

### 4.7 Measured: GPD Channel Request on the sink (2026-11-21)

Release builds in one checkout path, nightly-2026-08-01, master dd1ce41 →
`gp-channel-request` (file-backed flash = `.text` + `.rodata` + `.data`):

| Image | Flash | `.bss` |
|---|---:|---:|
| H2-Zero `router,waveshare-h2-zero,combo-light` | 584,062 → 584,808 (+746) | 71,880 → 71,888 (+8) |
| H2 NCP `esp32h2,transport-uart,thread-br-h2,rtscts-reserved-lines` | 730,634 → 732,016 (+1,382) | 176,232 → 176,248 (+16) |
| H2-Zero `router,waveshare-h2-zero`, C6 `router` | byte-identical | |

The NCP's stack gate still passes (serve chain 30,704 of 30,720 B).

### 4.8 Measured: InvolveTC (2026-11-21)

Release builds in one checkout path, master dd1ce41 → `gp-involve-tc`:

| Image | Flash | `.bss` |
|---|---:|---:|
| H2-Zero `router,waveshare-h2-zero,combo-light` | 583,676 → 584,492 (+816) | unchanged |
| H2 NCP `esp32h2,transport-uart,thread-br-h2,rtscts-reserved-lines` | 730,454 → 730,470 (+16) | unchanged |
| H2-Zero `router,waveshare-h2-zero` | byte-identical | |

Most of the combo's growth is the derivation's log line.

### 4.9 Measured: GPD key provisioning (2026-11-21)

Release builds in one checkout path, master dd1ce41 →
`gp-key-provisioning`:

| Image | Flash | `.bss` |
|---|---:|---:|
| H2-Zero `router,waveshare-h2-zero,combo-light` | 584,160 → 587,610 (+3,450) | 71,880 → 72,128 (+248) |
| H2 NCP `esp32h2,transport-uart,thread-br-h2,rtscts-reserved-lines` | 730,636 → 733,770 (+3,134) | 176,232 → 176,480 (+248) |
| H2-Zero `router,waveshare-h2-zero` | byte-identical | |

The growth is the HMAC-AES-MMO derivation, the key attributes, the reply
key protection and the out-of-band store (4 keys); the RAM is the key
store and the larger Commissioning Reply in the sink's queue. The NCP's
stack gate still passes (serve chain 30,704 of 30,720 B).

### 4.10 Measured: GPD Application Description (2026-11-21)

Release builds in one checkout path, nightly-2026-08-01, master dd1ce41 →
`gp-app-description` (file-backed flash = `.text` + `.rodata` + `.data`):

| Image | Flash | `.bss` | Note |
|---|---:|---:|---|
| ESP32-H2 ZRHP NCP (`thread-br-h2`) | 730,624 → 734,272 (+3,648) | 176,232 → 176,312 (+80) | feature on; stack gate passes (serve 30,704 / 30,720 B) |
| ESP32-H2-Zero `combo-light` | byte-identical | | feature off |
| ESP32-H2-Zero router | byte-identical | | |

The NCP growth is the descriptor codec (≈0.6 KB), the collection and reply
path in the sink (≈1.8 KB, partly inlined into `Sink::process`) and the
ZRHP event (≈0.6 KB).

### 4.11 Measured: GP Pairing Search (2026-11-21)

Release builds in one checkout path, nightly-2026-08-01, master dd1ce41 →
`gp-pairing-search` (file-backed flash = `.text` + `.rodata` + `.data`):

| Image | Flash | `.bss` | Note |
|---|---:|---:|---|
| ESP32-H2-Zero `combo-light` | 583,982 → 584,960 (+978) | ±0 | sink side |
| ESP32-H2 ZRHP NCP (`thread-br-h2`) | 730,526 → 731,398 (+872) | ±0 | sink side; stack gate passes |
| ESP32-H2-Zero router + `router-app/green-power-pairing-search` (lab measurement) | 520,474 → 521,668 (+1,194) | +96 | proxy side, not in a product |
| ESP32-H2-Zero router, ESP32-C6 router, nRF52840 router | byte-identical | | features off |

## 5. Risks

1. **The timed TX** (20 ms after the end of the GPDF, GP 1.1) on
   soft-MAC PHYs; implemented for the ESP radio only (§3.1). Without it, bidirectional must be
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
   until a sniffer exists. *(Resolved 2026-11-14: the nRF52840 radio lab
   captures and injects on air, §6.6.)* The GPD↔proxy path can only be inferred from
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
7. **Combo** (layer c, on nRF52840, ESP or BL702; host-tested only, §3.7,
   and blocked on a router product with an On/Off endpoint): pair a GP switch directly
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
  - replay and bad-MIC tests on air (done later, §6.6);
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
    follows the 16-value persistence stride (the default then; now 64, see 3.3). zigpy's three
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
  - The Pairing counters (≤ 1481) were within the 16-value stride (the default then). The next
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

### 6.5 H2-Zero re-identity and GP re-pairing (2026-11-13/14)

- Master 42f37b5 derives the ESP32-H2 EUI-64 like ESP-IDF. The H2-Zero
  router (app `203f6d2f…3d84`, app only at 0x10000, otadata erased, full
  backup first) booted with EUI-64 `10:51:db:ff:fe:63:b8:cd`. The
  identity guard discarded the old network state, and the router joined
  as NWK 0x07A3 at 00:06:24 EET with an empty Proxy Table.
- 00:07:34 EET: ZHA permit-join put the proxy into commissioning mode
  (254 s window, later "replaced" by each renewal).
- 00:13:49 EET: after a PB0 press the GPD (same SrcID, counter 3105 →
  3109) sent its Commissioning Request; zigpy sent three GP Pairings and
  reported the device commissioned.
- 00:15:00 EET: `GP proxy entries 1`. The GPD's reports then arrived as
  paired-mode Notifications (00:15:39, 00:16:39), and each one was
  cancelled because another proxy forwarded first (cancelled 8 → 10 → 12,
  sent stayed at 7). HA kept updating temperature, humidity, battery
  (100 %) and voltage (3.4 V).
- The proxy also relays Commissioning Notifications about every minute
  for SrcID 0x0CC35D9F, an already paired EfektaLab BGT1 that keeps
  sending commissioning frames (to investigate in M6).

### 6.6 On-air replay and bad-MIC injection (2026-11-14)

The nRF52840 dongle running zigbee-rs-radio-lab 0.1.0 transmitted two
frames on channel 15 (hwlab `PacketInjection.transmit`, no CCA or ACK,
0 dBm). The sniffer, H2-Zero console, GPD console and zigpy debug log
were recorded together in hwlab session
`2026-11-14T103808Z-m3-replay-badmic`. Proxies under test: the H2-Zero
(0x07A3) and the ZBT-1 (0x4A76). Both had 0x293C9879 in their Proxy
Table and both forwarded every genuine report as originators
(radius 30). The live GPD counter was 3852…3857.

| EET | Frame | Expected drop |
|---|---|---|
| 12:40:34.606 | Captured GPDF replayed byte for byte: counter 2197, MAC seq 149, valid MIC | `Stale` |
| 12:42:34.102 | Same frame with counter 3932 (live + 77), MAC seq 92 and the last MIC byte flipped | `AuthFailed` |

Results:

- **Proxies.** Within 3 s of either injected frame, nothing on air
  carried NWK source alias 0x9879: no Notification from the H2-Zero, the
  ZBT-1, the IKEA proxies or any relay. Every genuine report in the same
  run (12:39:03 to 12:44:04, one a minute) was followed by forwards from
  0x4A76 and 0x07A3 within about 30 ms. The H2-Zero logged
  `[GP] GPDF … Notification` only for the six genuine frames, and its
  `forwards sent` count rose by exactly one a minute (501 → 506). Drop
  reasons are logged at `debug` and are not in this `info` build.
- **Coordinator and zigpy.** Neither injected frame produced a
  `gpepIncomingMessageHandler` or a `Received a GP packet` line. The EZSP
  NCP filtered them before the host. HA sensor states changed only at
  the genuine report times (`:03` to `:04` past each minute).
- **No counter poisoning.** Counter 3932 was not taken from the bad-MIC
  frame. The next genuine reports (3856, 3857) were forwarded by both
  proxies and accepted by zigpy, and HA kept updating.
- The radio-lab TX audit log has a request and a result record for both
  frames. The FCS the radio appended matched the original capture
  (`6a7c`).

### 6.7 M4 bidirectional commissioning (2026-11-14, host only)

Design:

- **Proxy (TempMaster).** The sink's GP Response (Commissioning Reply,
  from zigpy after an rxAfterTx Commissioning Notification) is queued in
  the gpTxQueue as before. When the GPD's next GPDF with `rxAfterTx`
  arrives, `gp_receive_gpdf_bidirectional` builds the queued GPDF
  before the proxy advances the GPD's counter and returns it with the
  start instant `rx_end_us + tx_offset_us`; the runtime sends it with
  `transmit_raw_at` (next macDSN, no CSMA, no ACK) and logs
  `[GP] TempMaster GPDF sent` or the error (`Missed`).
- **GPD.** `commission_bidirectional[_on]` sends Commissioning with
  `rxAfterTx` and RxOnCapability, opens the receiver `rx_open_us` after
  the *start* of the frame (gpdRxOffset, A.1.6.3.1) and closes it
  `rx_window_us` after `rx_open_us` measured from the *end* of the frame
  (gpdMinRxWindow includes the GPDF duration, A.1.6.3.2); the policy must
  open by gpTxOffset (20 ms) and stay open past gpTxOffset +
  gpMaxTxOffsetVariation (25 ms). It repeats up to `attempts`
  times `attempt_gap_ms` apart. zigpy answers the *first* rxAfterTx
  Commissioning through the network, so the proxy can deliver the reply
  only on a later one. A reply must match the profile's security level
  and key type; a delivered key is unwrapped, persisted and replaces the
  GPD key. Success is then sent protected and the channel persisted.
  Without a reply the GPD stays uncommissioned; its counter keeps
  advancing through the journal as for any other frame.
- **MG22 policy** (`products/efr32mg22-gpd::BIDIRECTIONAL`): 6 attempts,
  500 ms apart, receiver from 19 ms after the start to 29 ms after the
  end of the GPDF (until 182b358: 19–23 ms after the end, which missed
  replies at the start of the spec window and the IKEA TempMaster, §6.9);
  checked at compile time.
- **zigpy 2.4.1:** the Commissioning Reply echoes the security level and
  key type and carries no key and no PAN ID; Success through a proxy
  arrives as a Commissioning Notification with `security_failed` and the
  MIC, which zigpy decrypts with the pending device's key; its counter
  must exceed the commissioning counter.

Validation reached: host tests (`zigbee-mac` timed TX, `zigbee-runtime`
`gp_tempmaster_answers_rx_after_tx_at_the_tx_offset`, `gpd-app`
`bidirectional`), clippy on the ESP32-H2 and MG22 images, feature-off
identity (§4.3).

Open hardware gates:

1. ~~On-air offset of the H2-Zero's Commissioning Reply after an injected
   rxAfterTx GPDF (radio-lab capture timestamps), and calibration of
   `TIMED_TX_START_LEAD_US`.~~ Done 2026-11-14 (§6.8).
2. Full bidirectional commissioning of the MG22 GPD into ZHA through the
   H2-Zero, with the coordinator and other proxies unable to answer.
   *Partial, accepted 2026-11-14 (§6.9): the H2's reply timing on air
   (gate 1) plus the MG22 commissioned into ZHA through IKEA 0x43FD as
   TempMaster (gate 2b). Commissioning through our H2 as TempMaster is
   not hardware-proven; no relocation retest is planned.*
3. ~~Reports after commissioning and a GPD reset (counter and key
   retained).~~ Done 2026-11-14 (§6.9).

### 6.8 M4 on air: the TempMaster race (2026-11-14)

Setup: H2-Zero (0x07A3) app-only flashed with the
`green-power-bidirectional` image of 30d8f74 (joined state and proxy table
kept); the nRF52840 radio-lab dongle injects an rxAfterTx Commissioning
GPDF (test SrcID 0x7E57B1D1, RxOnCapability) on channel 15 during
a ZHA permit-join and captures the air. Decoded with the network key.

Findings with the 30d8f74 proxy:

- **−40 dBm injection.** The H2 entered proxy commissioning mode and
  tunnelled a Commissioning Notification with the bidirectional bit and
  its own NWK address, but at +109 ms. IKEA VINDSTYRKA 0xD586 (not
  bidirectional) forwarded at +70 ms. zigpy appointed 0xD586 as
  TempMaster, so the H2 (correctly) dropped its queued reply.
- **0 dBm injection.** The coordinator heard the GPDF directly; zigpy sent
  one GP Response with TempMaster 0x0000 at +43 ms and another naming
  ZBT-1 0x4A76; the H2 forwarded at +106 ms.

Causes:

1. Our delay used the GP 1.0a distance formula (Dmin 15 ms) with an LQI
   from a linear RSSI curve, which gave ~95–100 ms for almost any link.
   GP 1.1.1 A.3.6.3.1 uses Dmin_b = 32 ms for rxAfterTx plus 32 ms per
   Link-quality step. Fixed (§3.2 step 4, §4.4).
2. Link quality was the LQI quartile (Excellent only at ≥ −40 dBm).
   Commercial proxies on this network report: IKEA −60/−50 dBm → 3,
   −82 → 2, −86 → 1; Tuya −96…−88 → 2. Now banded from RSSI.
3. A cancellation by an rxAfterTx duplicate now requires that the other
   forwarder be bidirectional-capable with a better link (A.3.5.2.1).
4. **zigpy 2.4.1 does not elect a TempMaster** (A.3.6.2.3: within Dmax,
   among bidirectional-capable candidates, by best link). It takes the
   `gpp_short_addr` of the first Commissioning Notification, or itself if
   the coordinator heard the GPDF directly (`zgp/manager.py`). A
   unidirectional proxy that forwards first therefore wins, and the
   coordinator always wins when the GPD is in its range. Bidirectional
   commissioning through our proxy needs the GPD out of the coordinator's
   range and our proxy to forward first. Fixed upstream in zigpy PR
   #1970 (open, not deployed in HA; §6.9).
5. VINDSTYRKA forwarded at ~70 ms even at −86 dBm / LQ 1, i.e. not by the
   1.1.1 formula; the H2 adds ~10 ms of processing to its computed delay.

**After the fix (86d2c24, same setup, −20 dBm injection; the H2 reports
RSSI −56 dBm, Link quality 3).** The H2 forwards first in every cycle,
41–42 ms after the GPDF (32 ms Dmin_b + ~10 ms processing); zigpy
appoints 0x07A3 as TempMaster; the H2 queues the GP Response and answers
the next rxAfterTx GPDF with a Commissioning Reply GPDF (15 B, NWK FC
0x8C, ext FC 0x80, command 0xF0, options 0x00 as zigpy echoes level 0).

Offset of the reply (radio-lab stamps the end of the SFD, so TX start
after the end of the trigger = ΔSFD − 19 × 32 µs − 160 µs; target
gpTxOffset 20,000 µs):

| `TIMED_TX_START_LEAD_US` | Samples | TX start after GPDF end |
|---|---:|---|
| 150 (initial estimate) | 5 | 20,273–20,280 µs (mean 20,275) |
| 425 (calibrated) | 5 | 19,998, 20,004, 19,999, 20,007, 20,222 µs |

One further cycle with the 150 µs build logged
`TempMaster GPDF not sent: TransactionExpired` (`PhyError::Missed`): the
GPDF arrived while the H2 was relaying a broadcast NWK_addr_req and was
handled more than 20.3 ms after its end, so the frame was not sent late.
The stall source is not yet identified. Each of the 11 triggered cycles
otherwise logged `TempMaster GPDF sent (15 B)`.

The follow-up deadline check runs before the first radio attempt and every
retry after `Busy`; host regressions cover a late first attempt, an expired
retry, the inclusive boundary, clock wrap and other radio errors. With that
change on the H2 (app SHA-256 `4db51a0c…6771d72`, app-only flash after a
verified full backup), the same nRF injection produced four replies at
19,984–20,015 µs after the GPDF end (hwlab session
`2026-11-14T212013Z-m4-deadline-regression`). This is still a software
deadline: an interrupt or driver delay between the final check and radio
start can make a frame late; the test does not prove a hard bound.

Not yet changed (minor deviations): a proxy without bidirectional
capability should send rxAfterTx Commissioning Notifications with the
derived alias, but ours uses its own address; and a unidirectional
groupcast proxy "SHALL NOT drop" its own forward, but ours cancels on a
duplicate when rxAfterTx = 0 (M1 behaviour).

### 6.9 M4 gates 2 and 3: MG22 bidirectional GPD into ZHA (2026-11-14)

Setup: MG22 BRD4183A flashed with the `bidirectional,lab-tx-power`
sensor (SrcID 0x293C9879, L3, key type 4). The MG22 cannot be moved away
from the coordinator, so the lab build transmits at −25 dBm (07db74a,
never in a default image). At that power the coordinator no longer hears
the MG22 directly; IKEA 0x43FD hears it at about −86 dBm (Link quality 1),
ZBT-1 0x4A76 earlier, and the H2-Zero only at about −90 dBm (Link
quality 0). The H2 runs the `green-power-bidirectional` image; nRF52840
radio-lab captures channel 15.

**Attempt 1 (PB0, receive window 19–23 ms after the GPDF end).** zigpy
appointed the first notifier, IKEA 0x43FD, as TempMaster (the H2
forwarded at ~111 ms, last). IKEA's Commissioning Reply started 27.93 ms
after the start of the GPDF, outside the GPD's window, so the GPD ended
with `NoCommissioningReply`. The failed long-press recommissioning also
left the GPD with no channel, so it stopped reporting until the next
successful commissioning.

**Window fix (182b358).** GP 1.1.1 measures gpdRxOffset and gpTxOffset
from the *start* of the GPDF (A.1.5.2.1.2, A.1.6.3.1), and a TempMaster
may start up to gpMaxTxOffsetVariation = 5 ms late. gpdMinRxWindow
therefore includes that 5 ms plus the GPDF duration (A.1.6.3.2). The GPD
measured from the end and closed after 4 ms, which missed spec-conformant
replies at 20–21.8 ms after the start and IKEA's late replies. The
receiver now opens 19 ms after the start and closes 29 ms after the end
(≈ 30.8 ms after the start for the 48 B Commissioning GPDF). IKEA's late
replies are outside the spec; the GPD accepts them only because the window
was widened. Our proxy's reply (20 ms after the end ≈ 21.8 ms after the
start) is inside the spec window and unchanged.

**Attempt 2 (jlink reset with permit join open; an uncommissioned GPD
commissions at boot).** zigpy again chose IKEA 0x43FD. IKEA sent the
Commissioning Reply (MPDU 15 B, options 0x98: level 3, key type 4, no
key) 25.25 ms after the start of the second Commissioning GPDF. The GPD
accepted it and sent Success (fc 4434) 27.3 ms after that GPDF and logged
`commissioned on channel 15` 3.3 s after boot. zigpy logged `Sent GP
Pairing` and `GP device commissioned`.

**Lost reports after commissioning: a zigpy bug.** The proxies forwarded
the second Commissioning GPDF 70–130 ms late (0x4A76, 0x07A3), after the
Success. zigpy's `_process_commissioning` created a new pending device
from it and sent another Response. While permit join stays open,
`gp_packet_received` sends GPDFs to the pending device, so reports fc
4435–4439 never reached the ZHA entities. From fc 4440, after permit join
closed, the entities updated (25.85 °C, 50.5 %). zigpy should ignore a
Commissioning GPDF from a GPD it just paired with the same key and an
outgoing counter below the registered one. This is to be fixed upstream
together with the TempMaster election. BGT1 0x0CC35D9F's repeated
commissioning (M6) may be related.

**Gate 3 (jlink reset of the commissioned GPD, no permit join).** The GPD
logged `Resumed SrcID 0x293C9879 counter 4448 channel Some(15)`; the last
pre-reset report was fc ≤ 4442, so the journal skipped ahead. It reported
at once and every 60 s. ZHA accepted fc 4448 and fc 4449 (humidity
entity updated at 19:53:40 and 19:54:40 UTC; device `last_seen` current).
The proxies, including the H2, kept forwarding.

**Our proxy as TempMaster: not proven, gate 2 accepted as partial.** zigpy appoints the first
notifier. A spec-conformant election (A.3.6.2.3: the bidirectional-capable
candidate with the best link) would also choose IKEA (−86 dBm) over the H2
(−90 dBm) at this placement. The H2's ZHA signature lacks endpoint 242,
so a manual GP Response through ZHA's cluster-command service is not
possible either. Gate 2 through our proxy needs the H2-Zero closer to
the MG22 (Link quality ≥ 2, forwarded within ~64 ms). On 2026-11-14
the user accepted gate 1 + gate 2b as the M4 gate-2 evidence, with no
relocation retest.

**zigpy fix upstream.** [zigpy PR #1970](https://github.com/zigpy/zigpy/pull/1970)
(branch `faronov:gp-tempmaster-election`) addresses both findings:
- it elects the TempMaster per A.3.6.2.3 within Dmax (100 ms): bidirectional
  proxies with room in gpTxQueue first, then the best link, then the lowest
  address;
- it ignores a Commissioning GPDF whose outgoing counter is below the last
  accepted frame counter for the same key.

The PR adds zigpy unit tests. It is open and not deployed in the user's HA.

**M5 note.** After an H2 reboot, the proxy drops up to 63 frames per GPD
(`counter_persist_stride` 64, skip-ahead on restore). Resolved for event
commands by adaptive persistence (§3.3, 2026-11-16); reports keep the
batched floor.

### 6.10 Combo light on the H2-Zero (2026-11-15/16)

Image b1f78b1 `router,waveshare-h2-zero,combo-light` (app sha
`e47e479f…358ecb`, no TempMaster, batched counters) written to `ota_0`
with hwlab (verified full backup sha `ea45a640…98578f8`, `zbnv` kept). The
router resumed as NWK 0x07A3 with its child and 2 Proxy Table entries
(an old proxy-only journal restored by the sink-capable firmware). ZHA
removed it (Leave: GP tables cleared), it rejoined as 0x7C87 and the ZHA
interview found endpoint 1 as On/Off Light (0x0100, servers 0x0000,
0x0003, 0x0006): `light.zigbee_rs_esp32_h2_router` and a power-on
behaviour select. `light.turn_on` from HA lit the warm white LED.

- **Sink window without timeout.** A BOOT press right after the rejoin
  opened sink commissioning; with the Silicon Labs default exit mode (on
  first pairing only) it never timed out, and the LED blinked blue (and,
  with the light on, alternated blue and warm white) for over 20 minutes.
  Fixed 2026-11-16: a locally opened window ends after 180 s.
- **MG22 sensor (bidirectional build) did not commission.** Its
  Commissioning GPDFs (0x293C9879, channel 15) were heard directly; the
  sink dropped them (`NoTempMaster`: the image had no TempMaster) and
  answered the copy tunnelled by the ZBT-1 router (0x4A76) with a
  Commissioning Reply through 0x4A76, which cannot transmit in the GPD's
  rxAfterTx window. ZHA commissioned the GPD in parallel (GP Response
  `0x06` seen, our proxy logged it as unsupported without the TempMaster).
  Fixed by adding `green-power-bidirectional` to `combo-light`; not yet
  re-tested on hardware.
- **MG22 switch (unidirectional build) commissioned directly.** The MG22
  was reflashed with the `efr32mg22-gpd-switch` build (hwlab, verified
  backup sha `d95a6ed6…329f05`; the sensor's identity journal at 0x74000
  kept). Switch SrcID 0x7A6B9879, alias 0x9879: `GP sink Commissioned {
  device_id: 2 }`, `CommissioningLeft(FirstPairing)`, GP Pairing and the
  alias Device_annce broadcast, journal saved, `GP sink entries 1`. ZHA
  (permit join open) paired the same switch; HA shows "zigbee-rs GP switch"
  button events.
- **Toggles.** 8 short presses: 8 × `Translated → cluster 0x0006 cmd 0x02
  to endpoint 1`, each followed by an OnOff attribute report to the
  coordinator; HA's light state followed (8 toggles from Off end Off,
  last change 17:31:01.78). Our proxy also tunnelled each press to ZHA.
  Proxies using the same alias 0x9879 as the sensor caused NWK
  `Frame counter replay from 0x9879` and, in the press burst,
  `Broadcast transaction table full` warnings; no press was lost.
- **Power cycle.** After USB power off/on the router restored 1 proxy and
  1 sink entry. Presses afterwards reached ZHA (button event 17:37:43)
  but not the light: with batched counters the sink and proxy floors were
  63 above the last write and the switch had sent only ~10 frames. This
  motivated adaptive counter persistence (§3.3, 2026-11-16). The boot
  log and the dropped frames themselves were not captured (the hwlab
  serial capture ended when the USB device disappeared).

**2026-11-16/17, image 47dc602** (`combo-light` with the TempMaster and
adaptive counters; app sha `61a7857a…9d6eaa`, hwlab app-only write after a
verified full backup sha `2b9a9737…352b1c`, `zbnv` kept). The router
resumed as 0x7C87 with 1 Proxy and 1 Sink Table entry (the journal from
b1f78b1, no exact flags: restored batched) and ZHA kept the On/Off Light.
Sessions in `~/.hwlab/sessions/2026-11-17T154805Z-h2zero-recommission-switch`,
`…155517Z-h2zero-adaptive-after-powercycle`,
`…160144Z-h2zero-boot-window-timeout`.

- **Re-commissioning the switch** (BOOT, ZHA permit join, BTN0 ≥ 3 s):
  `CommissioningLeft(FirstPairing)`; 6 Toggles translated to the light.
  Needed once because the entry predated the exact flag.
- **Adaptive counters, power cycle.** After USB power off/on the first
  presses toggled the light at once: 9 Toggles, none dropped (the APS
  counter restarted at 1, so the router had rebooted). Proven on hardware.
- **BOOT window without pairing:** `GP sink CommissioningLeft(Timeout)`
  about 180 s after the press. Proven on hardware.
- **MG22 back to the sensor, bidirectional build** (2026-11-17,
  `efr32mg22-gpd-sensor --features bidirectional`, image sha
  `2a47f74f…b1b585b`, verified backup sha `8bed4da6…900432`): the hwlab
  J-Link program step reported `Failed to init DAP` after writing (the GPD
  sleeps in EM2); a read-back showed the complete new image and both
  identity journals (0x70000, 0x74000) byte-identical to the backup, and
  `flash verify` passed. After a reset the GPD resumed SrcID 0x293C9879,
  counter 7072, channel none (not commissioned).
- **Bidirectional sensor directly into the sink: not exercised**
  (2026-11-17, session `…161149Z-h2zero-sink-bidi-sensor`, H2-Zero and
  MG22 logs). BOOT opened the sink window at 18:43:07.27 (`window_s:
  Some(180)`, Proxy Commissioning Mode broadcast). The BTN0 long press
  started the GPD's beacon scan at 18:43:22.22; it ended at 18:43:24.50
  with `commissioning failed: NoPermitJoinNetwork`. ZHA's
  Mgmt_Permit_Joining (60 s, to the H2-Zero only) reached the router at
  18:43:24.85, 0.35 s after the scan ended (zigpy's Proxy Commissioning
  Mode 0.08 s earlier). The GPD therefore sent no Commissioning GPDF:
  none of the proof lines (`CommissioningReplySent`, `TempMaster GPDF
  sent`, `Commissioned`) could appear and no sensor reports reached the
  sink (the GPD logged `Err(NotCommissioned)` every 60 s). The sink window
  ended with `CommissioningLeft(Timeout)` at 18:46:07.30, 180.0 s after
  BOOT (a second hardware confirmation of the timeout). Not a sink or
  TempMaster defect: the test ordering must be permit join confirmed at
  the router first (its log shows `Permit joining: duration=60`), then
  BOOT, then BTN0 within the remaining window. The GPD scans for a
  joinable network only once per long press (about 2.3 s).
- **Re-test in the right order: TempMaster reply cancelled (bug, fixed in
  9e9c430)** (2026-11-17, session `…171402Z-h2zero-sink-bidi-sensor-retest`,
  capture seconds). Permit join (90 s) at the H2-Zero 1533.4, BOOT 1539.9,
  MG22 long press 1545.7. Five Commissioning GPDFs (1548.0 … 1550.7) were
  heard directly: each time `CommissioningNotification` (our proxy) and
  `sink …: CommissioningReplySent`, our GP Response broadcast, then
  zigpy's GP Response (from 0x0000, ~140 ms later). No `TempMaster GPDF
  sent` / `not sent` line at all; the MG22 ended with `commissioning
  failed: NoCommissioningReply`. Cause: the sink's Response queued the
  Commissioning Reply in the local proxy's gpTxQueue, and zigpy's
  Response, appointing another proxy that could not transmit, made the
  proxy drop it (A.3.5.2.1) before the GPD's next attempt. Reproduced by
  the host test `gp_combo_is_its_own_temp_master_for_direct_bidirectional_
  commissioning` (zigpy appointing another proxy); fixed: with the sink, a
  gpTxQueue entry queued by the node's own sink is local and another
  sink's Response neither cancels nor replaces it (proxies without the
  sink unchanged; non-combo images identical apart from panic Location
  words). Image 9e9c430 (app sha `8858ab55…6d2833`, 563,160 B flash /
  70,692 B `.bss`, +296 / +8) flashed with hwlab (backup sha
  `4ae5a109…5031fa`, `zbnv` kept); the router resumed as 0x7C87 with 1
  proxy and 1 sink entry.
- **Re-test with 9e9c430: the reply survived zigpy but missed every
  instant** (session `…175942Z-h2zero-tempmaster-fix-retest`). Five
  attempts, each `CommissioningReplySent` and `TempMaster GPDF not sent:
  TransactionExpired`; MG22 `NoCommissioningReply`. With the sink the
  reply was built first but transmitted only after the proxy and the sink
  had processed the triggering GPDF. Fixed in 72ad6b3: in combo builds the
  reply is built and transmitted first, then the GPDF is processed (proxy
  counter checks keep their order; proxy-only builds unchanged); the
  host test checks with the mock MAC's AES-keying count that the reply
  leaves before the sink unwraps the GPD key. The combo now logs, after the
  TX, when the reply was built and when the MAC returned relative to the
  end of the GPDF.
- **Re-test with 72ad6b3: PASSED** (2026-11-17, session
  `…182043Z-h2zero-tempmaster-order-retest`, capture seconds). Three
  TempMaster replies to the MG22's attempts:
  - 1557.598 `TempMaster GPDF sent (15 B): built +1882 us, due +20000 us,
    done +20710 us` (not taken by the GPD);
  - 1558.130 `not sent: TransactionExpired: built +4326 us, due +20000 us,
    done +19912 us`;
  - 1558.663 `sent (15 B): built +1932 us, done +20707 us`; MG22
    `commissioned on channel 15 (LQI 153)` at 1558.703; H2-Zero
    `sink …: Commissioned`, `GP sink Commissioned { gpd_id:
    SrcId(691837049), device_id: 51 }`, `CommissioningLeft(FirstPairing)`,
    GP Pairing, alias Device_annce, journal saved.
  - First report 1571.4: `sink …: PassedThrough`, `GP sink GpdCommand {
    … command_id: 162 }` (0xA2 Multi-Cluster Reporting), MG22 `report …
    Ok(Some(7089))`.

  Bidirectional commissioning directly into the combo sink, with the combo
  as its own TempMaster, is proven on hardware. The reply is built
  1.9–4.3 ms after the end of the GPDF, well inside gpTxOffset.
- **The miss.** `done +19912 us` is not "before the due time": the PHY
  issues the command `TIMED_TX_START_LEAD_US` (425 µs) early, at +19,575 µs,
  and gives up `TIMED_TX_LATE_US` (300 µs) later, so a return at +19,912 µs
  is that deadline plus the return path. With the reply built at
  +4.3 ms the thread had 15 ms to spare, so a late wake-up is unlikely;
  the radio refused the transmission (`Busy`) for the whole 300 µs, which
  in the esp-radio patch set means a frame was being received (GP
  broadcasts of the commissioning are dense at that moment), an own
  hardware ACK, or pending interrupt work. The ESP PHY reported both "radio
  busy until the deadline" and "caller late" as `Missed`
  (`TransactionExpired`). Fixed in da33af4: `transmit_with_deadline`
  reports `ChannelBusy` (MAC `ChannelAccessFailure`) when the radio refused
  until the deadline, and the timed TX uses a new esp-radio local patch,
  `transmit_raw_preempting_rx`, that abandons a frame being received (as
  ESP-IDF `ieee802154_transmit` does); an own TX, a hardware ACK and
  pending interrupt work still refuse. GP 1.1 A.1.5.2.2: the TempMaster
  sends without CSMA/CA. Host test for the error split; the preemption
  needs a hardware run. Images without timed TX are unchanged (panic
  Location words only); combo light 563,368 B flash / 70,692 B `.bss`.
- **Three rounds with da33af4: 1 of 3 passed** (2026-11-17, session
  `…190251Z-h2zero-tempmaster-preempt-retest`, capture seconds). No
  `ChannelAccessFailure` any more (the preemption works), but the task now
  came too late:
  - round 1 (1758–1768): one reply `sent`, built +1925 us, done +20698 us,
    not taken by the GPD (its first window; not explained), then four
    `TransactionExpired` built +68,352 / +96,961 / +101,460 / +157,173 us;
    MG22 `NoCommissioningReply`;
  - round 2 (2547–2555): `sent`, built +1912 us, done +20722 us; MG22
    commissioned, sink `Commissioned`, `FirstPairing`. PASS;
  - round 3 (2764–2774): `TransactionExpired` built +20,340, +157,031,
    +131,115, +173,585, +92,930 us; MG22 `NoCommissioningReply`.

  The GPDF reached the reply code up to 173 ms after it ended: the single
  event loop was busy with other work (the log was silent during those
  gaps; commissioning brings GP Pairing / Proxy Commissioning Mode
  broadcasts and their relays, ZHA traffic, journal writes). A reply
  scheduled from the task cannot meet a 20 ms instant behind that.
- **Reply started from the radio interrupts (7b2d6ca, opt-in with
  `green-power-bidirectional`).** The reply no longer waits for the task:
  - `Proxy::radio_downlink` builds the gpTxQueue GPDF a radio can send by
    itself: the unprotected reply to an unprotected `rxAfterTx` GPDF from
    its GPD, byte for byte what `take_downlink` builds for that trigger
    (host test). Commissioning GPDFs are unprotected, so no MIC or
    freshness check is skipped; protected triggers keep the task path with
    its checks. Maintenance frames and ApplicationID 0b010 are not armed.
  - The runtime keeps it armed in the MAC (`MacDriver::arm_gp_reply`, with
    the next macDSN) after every `gp_poll` and re-arms when the queue
    changes; a reply the radio sent leaves the gpTxQueue; the task path
    does not send a trigger of an armed reply a second time.
  - ESP32-H2/C6: the PHY arms esp-radio (local patch
    `ieee802154-triggered-tx`, vendor `LOCAL-PATCHES.md`). The RX_DONE
    interrupt matches the trigger (`zigbee_mac::gp_reply::
    gpdf_triggers_reply`, host-tested) and programs IEEE802154 Timer0; its
    interrupt starts the frame `TIMED_TX_START_LEAD_US` before
    gpTxOffset, abandoning a reception, retrying while the radio sends
    until 300 µs late. While it counts down, thread transmissions wait.
    Other MACs return `Unsupported` and keep the task path.
  - Logs: `[GP] TempMaster GPDF sent by the radio (N B): on air +X us
    after the GPDF` (estimate: Timer0 interrupt time + the 425 µs lead) or
    `not sent by the radio: …`.
  - The combo lab image also logs loop iterations over 40 ms with the
    journals' flash erase/write count and longest duration, and the
    longest iteration in the 60 s summary. First boot: `slow loop step
    67845 us | flash erases 1 (max 21310 us) writes 4 (max 2936 us)`
    (resume persistence); in the minute before, the longest iteration was
    45 ms without any flash work.
  - Host test `gp_combo_radio_sends_the_armed_reply_however_late_the_task_is`
    (mock radio fires the armed reply; the task sees the trigger 150 ms
    later and must not send it again). Non-GP router images unchanged
    apart from panic Location words; combo light 568,418 B flash /
    70,924 B `.bss` (+5,050 / +232). Flash erases still mask interrupts
    (esp-storage), so a journal write during the 20 ms countdown can still
    delay a reply. Flashed with hwlab 2026-11-17 (backup sha
    `8b14e36d…0cedfd`, app sha `26fbc349…da2fbf`, `zbnv` kept); resumed as
    0x7C87 with its 4 GP journal entries.
- **Three rounds with 7b2d6ca: PASSED 3 of 3** (2026-11-17, session
  `…202955Z-h2zero-radio-reply-3round`, rounds at capture 1919–1926,
  2099–2106 and 2295–2303 s). Each round: `CommissioningReplySent` on the
  GPD's first Commissioning GPDF; on its second, `TempMaster GPDF sent by
  the radio (15 B): on air +20069 us after the GPDF`; MG22 `commissioned on
  channel 15` (LQI 165 / 143 / 159); sink `Commissioned { gpd_id:
  SrcId(691837049), device_id: 51 }`, `CommissioningLeft(FirstPairing)`. No
  `not sent` line in any round. The blue blink after commissioning was
  ZHA's 90 s permit-join window, as expected. **Bidirectional direct
  commissioning into the combo sink, with the combo as its own
  TempMaster, is proven on hardware on the ESP32-H2.** The on-air offset
  (+20,069 µs) is the PHY's estimate from the Timer0 interrupt time plus
  the calibrated 425 µs start lead, not a sniffer measurement; the GPD
  accepting the reply in every round confirms it lies in its window.
- **Follow-up: main-loop latency besides flash.** During commissioning the
  combo logged `slow loop step` iterations of 41–73 ms: 72.8 ms with one
  journal erase (21.5 ms) and 10 writes, but also 45 ms and 41 ms with no
  flash work at all. The radio-started reply no longer depends on the loop,
  but the other GP deadlines (tunnelling delays, Notification retries,
  Sink commissioning bookkeeping) and ZHA traffic still do. To do:
  attribute the flash-free stalls (candidates: CSMA-CA waits and busy-radio
  deferrals of GP Pairing / Proxy Commissioning Mode broadcasts and their
  relays, USB-Serial-JTAG log flushes, sink key unwrap / CCM* work), then
  bound or split them; keep journal erases out of the 20 ms countdown
  (esp-storage masks interrupts during an erase).

### 6.11 Pairing Configuration and Translation Table on the H2-Zero (2026-11-19)

Goal: run the sink side of the GP binding on hardware without moving the
combo out of the user's ZHA network (ZHA cannot send Pairing
Configuration). The `lab-gp-translation` image of `examples/esp32h2-sensor`
(`router,waveshare-h2-zero,lab-gp-translation`, `gp-translation` branch)
issues both commands locally ten seconds after boot through
`gp_sink_configure_locally` (§3.8): a GP Pairing Configuration (Extend)
rebuilt from the switch's current Sink Table entry, then a Translation
Table Update that retargets the switch's Toggle (0x22) to On/Off On (0x01)
on endpoint 1. Toggle → On is observable: repeated Toggles leave the light
on. `lab-gp-translation-remove` removes the translation. Neither is a
product image.

Setup: H2-Zero combo light 0x7C87 (PAN 0xDFE9, channel 15), full backup
`318a05a0…19a4` (its Sink Table already held the MG22 switch 0x7A6B9879
and the sensor 0x293C9879); MG22 (J-Link 440206286) backup `acfcdbd1…da74`,
then `efr32mg22-gpd-switch` with `lab-switch-cycle` (Off, On, Toggle every
30 s). App-only flashes (H2 `ota_0`, MG22 0x0–0x70000); `zbnv` and the GPD
journals were never written by the programmer. Evidence: H2 and MG22
console logs (hwlab sessions `…220730Z-gp-translation-h2zero-lab`,
`…221047Z-gp-translation-switch-cycle`, `…221422Z-gp-translation-reset-persist`,
`…221746Z-gp-translation-remove`, `…222727Z-gp-translation-restored`) and
the light's state history in HA (read only). The sniffer was in use by
another session on the same channel, so there is no air capture.

- **Migration:** the lab image resumed on the network written by the
  previous firmware (`Resume: addr=0x7C87 … ch=15`), loaded 2 sink and 3
  proxy entries and an empty Translation Table.
- **Configuration:** `sink cmd 0x09 from 0xFFFE: Commissioned` (Extend of
  the existing entry: DerivedGroupcast, level 3, counter 112), then the
  translation `{ SrcId(0x7A6B9879), cmd 0x22 → ep 1, 0x0104/0x0006 cmd 0x01,
  payload Fixed([]) }`; `GP proxy table saved`. The journal's new sink
  record (generation 0xA2) carries the translation section flag (byte 7 =
  0x02) next to the previous record (0xA1), read back from a later full
  backup.
- **Translation:** the switch's frames fc 128–133: Off → `cmd 0x00`, On →
  `cmd 0x01`, Toggle → `cmd 0x01 to endpoint 1` (fc 130 and 133). HA:
  on 22:11:54, off 22:12:54, on 22:13:24; no change at either Toggle.
- **Reset:** after a system reset the hook found the translation already
  restored (`translations 1, Toggle translation present true`) and sent
  nothing; Toggles fc 136 and 139 still became `cmd 0x01`. The first GPDF
  1.3 s after boot (fc 134) was accepted.
- **Removal:** `lab-gp-translation-remove` (TT Update remove) left
  `translations 0`; Toggles fc 148 and 151 went back to the built-in
  `cmd 0x02` (HA: off at 22:22:56 after On at 22:22:26). The new record
  (0xA3) has no translation section.
- **Restore:** the original `ota_0` bytes and the original MG22 app
  region were programmed back (read-back verified). The previous combo
  firmware resumed on the journal (2 sink entries, 3 proxy entries, as
  before the test); the MG22 sensor resumed SrcID 0x293C9879 at counter
  10192 (10144 before the test), so neither GPD journal was rolled back.

The hook that ran on hardware kept its once-per-boot state in a struct
field; the committed hook keeps it in a static so the production
`combo-light` image stays byte-identical (§4.6). The commands it issues
are unchanged.

Not covered here: Pairing Configuration and Translation Table commands
received over the air from a commissioning tool (§6.12), and older
firmware reading a journal that still holds a translation section
(host-tested in `gp_store::translations`).

### 6.12 Remote GP provisioning over the air: NCP → C6 combo (2026-11-20)

Goal: the GP binding end to end over the air: the hub's path (ZRHP host →
NCP → GP Pairing Configuration / Translation Table Update on endpoint 242)
configures a sink it does not run, and a real GPD then drives the sink's
light through the translation. Everything ran in the NCP's lab network
(PAN 0x1C06, channel 17); the user's ZHA network was not touched.

Setup:

- **NCP:** the Thread BR board's H2, master 2ab8ac0
  (`esp32h2,transport-uart,thread-br-h2,rtscts-reserved-lines`, 921 600
  with RTS/CTS), app-only write to `ota_0` through the S3 bridge after a
  full backup (two identical reads, `ef81f503…ccb`); it resumed its network
  (`Resuming formed PAN=0x1C06 ch=17`).
- **Sink:** the ESP32-C6 DevKit, the new lab `combo-light` router image of
  `examples/esp32c6-sensor` (`router` + the product's GP Combo light with
  the Translation Table; no BOOT commissioning window) built with
  `ZIGBEE_LAB_CHANNEL=17` (the router image now honours it too). Backup
  `44d31b8e…ab17`. The role change cleared its network state as designed
  (`cleared network state: role changed`, counter limits kept); it joined
  through an NCP-only permit join as router 0x00A4. Lab image 600,704 B
  as an app image (`.text` 508,232, `.bss` 71,944) against 539,120 B
  (`.text` 450,450, `.bss` 68,372) for the C6 `router` image, which stays
  byte-identical to master in every allocated section; so do the MG22
  sensor and switch images without `lab-gpd-channel`.
- **GPD:** the MG22 switch (`efr32mg22-gpd-switch`, `lab-switch-cycle` +
  the new `lab-gpd-channel` with `ZIGBEE_LAB_CHANNEL=17`: every frame on
  channel 17, journal untouched, so the switch's commissioned channel 15
  stays as it was). Backup `9b311b26…d560`. Its key and counter were never
  given to the C6 by commissioning: the C6 learned them only from the
  Pairing Configuration.
- **Host:** `tools/zrhp-host --serial … --baud 921600 smoke --form
  --gp-sink 0x00A4 --gp-src 0x7A6B9879 --gp-pair KEY:150 --gp-translate
  22:01:0006:01 --gp-tables --gp-watch 100` (key read from the H2-Zero's
  Sink Table backup into a local file, never logged).
- **Evidence:** C6 and MG22 console logs, the NCP's logs over ZRHP, and the
  nRF sniffer on channel 17 (hwlab session `…072911Z-gp-phaseb-air-ch17`).

Results:

- **Provisioning (07:29:27):** `GP_PAIRING_CONFIGURATION` Extend → C6
  `sink cmd 0x09 from 0x0000: Commissioned`, Default Response SUCCESS
  (`EVT_GP_REMOTE_STATUS` after 82 ms); `GP_TRANSLATION_UPDATE` add Toggle
  → ep 1 On/Off On → SUCCESS (83 ms); `GP_REMOTE_SINK_TABLE_REQUEST` by
  GPD ID → `EVT_GP_REMOTE_SINK_TABLE` with the entry (derived groupcast,
  level 3, key present, counter 150; 48 ms); `GP_TRANSLATION_REQUEST` →
  `EVT_GP_REMOTE_TRANSLATION_TABLE` with the translation (48 ms). On air:
  each command, its APS ACK and the Default Response or table response.
- **Translation:** the switch's frames fc 176–178: Off → `cmd 0x00`, On →
  `cmd 0x01`, Toggle (fc 178) → `cmd 0x01 to endpoint 1`; the C6 reported
  OnOff = 1 to the NCP. Each GPDF seen twice on air (the GPD's repeat).
- **Reset:** after a system reset the C6 resumed (`1 GP proxy table
  entries restored`: the sink record) and Toggles fc 181 and 184 still
  became `cmd 0x01`.
- **Remove translation:** `GP_TRANSLATION_UPDATE` remove → SUCCESS; the
  Translation Table Response is NOT_FOUND (0x8B, total 0); the sink entry
  remains (counter 184); Toggle fc 187 → built-in `cmd 0x02`.
- **Remove GPD:** `GP_PAIRING_CONFIGURATION` Remove GPD → SUCCESS,
  `Decommissioned`; Sink and Translation Table Requests → NOT_FOUND;
  On fc 189 and Toggle fc 190 were no longer delivered.
- **Restore:** C6 back to its sensor image (original `ota_0` bytes; it
  rejoined through an NCP-only permit join as 0x00A4); MG22 back to the
  sensor app (read-back identical to the original, resumed SrcID
  0x293C9879 at counter 10800, 10752 before). The BR H2 keeps the master
  NCP.

Wireshark 4.1 and master mark every GP Pairing Configuration with
`NumberOfPairedEndpoints` 0xFD–0xFF as malformed: the dissector sizes the
"Paired Endpoints" subtree as `n + 1` bytes before it checks for those
special values. All fields before that byte decode as sent (gecko_sdk
layout), and the C6 accepted both frames.

Not run on hardware: `GP_SEND_PAIRING`, `GP_REMOTE_PROXY_TABLE_REQUEST`,
groupcast-addressed (broadcast) Pairing Configuration and pre-commissioned
groupcast, and a GPD heard only through a remote proxy.

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
7. ~~Approve the router growth and make it default-on?~~ Approved
   2026-11-13: default-on in every router build (§2, §4.1).
8. Is Combo/Sink (layer c) wanted on a specific router product, such as a
   light or relay endpoint for direct switch pairing?
   *(Still open. The sink is implemented opt-in (§3.7) but no router product
   has an On/Off or Level endpoint to drive, and adding one is a product
   decision.)*

## 8. Proposed implementation order

1. `zigbee-gp` frame, security and cluster codecs, with zigpy golden
   vectors (host only).
2. GPD app on MG22, unidirectional: commissioned directly into ZHA, with a
   quirk.
3. MAC `mac_seq`, runtime version-3 demux, NWK/APS alias, Proxy Basic
   unidirectional with `GpTableJournal`, on the TLSR8258 router.
4. Timed-TX PHY capability: gpTxQueue and bidirectional commissioning,
   first on the platform where it can be proven. GPD bidirectional mode.
   *(M4: implemented opt-in on ESP32-H2 and MG22, host-tested; H2 reply
   timing and MG22 commissioning into ZHA (third-party TempMaster),
   reset and reports lab-tested; gate 2 accepted as partial (gate 1 +
   gate 2b), our proxy as TempMaster for a real GPD not proven; zigpy
   PR #1970 open, §6.9.)*
5. Full proxy (channel switch, unicast modes, key types, IEEE ApplicationID).
6. Combo/Sink with translation table. *(Implemented opt-in, host-tested,
   §3.7, §4.5; Translation Table commands deferred. Lab-tested on the
   ESP32-H2-Zero combo light: direct switch and bidirectional sensor
   commissioning, translated Toggles, persistence across power cycles,
   §6.10.)*

Each step ends with host tests, the feature-off byte-identity check, and
size and RAM deltas recorded here before the next step starts.
