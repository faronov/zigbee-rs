# NV Storage

Persistence separates physical flash, product layout, and Zigbee durability
semantics:

```text
chip HAL            flash controller and raw NOR operations
board               physical flash resource
product             bounded partitions, linker map, migration/reset policy
zigbee-runtime      journals and security/child semantics
application         when to restore, checkpoint, clear, or activate OTA
```

Partition addresses belong to the product, not a generic HAL or example loop.

## Security state

`SecurityStateJournal<F>` is a two-sector atomic journal for:

- network identity and keys;
- outgoing NWK/APS counter reservations;
- exact incoming NWK/APS replay floors, bound to the authenticating key;
- parent and commissioned state;
- staged or previous secondary Network-Key state and update state;
- End Device Timeout client state.

Each record has a version, generation, CRC, read-back verification, and a
commit marker written last. The scanner tolerates erased, torn, corrupt, and
unknown-version slots and selects the newest valid generation.

Outgoing counters are reserved ahead in durable storage. After a crash, the
runtime resumes above the reservation rather than reusing a secured frame
counter.

Incoming replay floors use compact append-only records. A verified frame is
committed after MIC verification but before relay, APS acknowledgement,
security-command dispatch, or ZCL side effects. Reboot restores NWK,
Trust-Center-link-key, application-link-key, and global commissioning-key
domains. If the append fails, the frame is dropped without acknowledgement or
application execution.

Persisted rejoin keeps `rejoin_pending` set across both secure and centralized
Trust Center attempts. When a Trust Center rejoin returns a current network
key, the runtime replaces the durable key/sequence, reserves the next outgoing
NWK counter range, and checkpoints the new parent plus Trust Center incoming
counter before `Device_annce` or normal traffic. A torn or failed checkpoint
therefore restarts from the previous committed key instead of announcing with
volatile security state. Distributed-security records never select this
fallback.

Do not replace the security journal with generic `LogStructuredNv`; their
durability contracts are different.

The default journal sector is 4 KiB. It has 32 slots. A router build can track
up to 112 active replay domains; a full rollover then consumes 28 replay slots
plus one state slot and leaves only 12 incoming-frame appends before the next
erase. The format fits, but that geometry is not an endurance claim.

Small end devices with few peers may keep 4 KiB sectors after calculating
their traffic and flash endurance. Parent routers and coordinators must use a
larger protected partition or a storage technology qualified for per-frame
appends. A practical starting point is 16 KiB per logical sector (32 KiB for
the two-sector journal), followed by product-specific lifetime testing.

A product fixes its logical sector size in the type while retaining the same
journal logic:

```rust,ignore
type SecurityStore =
    SecurityStateJournal<PartitionFlash, { PRODUCT_SECURITY_SECTOR_SIZE }>;

let store = SecurityStore::new_with_sector_size(
    partition_flash,
    0,
    PRODUCT_SECURITY_SECTOR_SIZE as u32,
);
```

`PartitionFlash` translates those relative offsets into the product-owned
protected address range. Boards never supply partition addresses or a
product-specific journal wrapper.

`SecurityStateJournal::FULL_TABLE_APPEND_CAPACITY` exposes the worst-case
number of incoming commits available after a full-domain rollover. Product
code and CI should assert an intentional lower bound for parent/coordinator
images.

Current security records use version 7. Version 6 distinguished a future
staged Network-Key from the previous key retained after activation. Version 7
also journals a pending PAN-ID transition/broadcast, deferred
`Device_annce`, and the provisional-parent gate used after a restored secured
rejoin. Older supported records decode through explicit migration paths.
Downgrading to firmware that cannot understand the current
record version is not supported because selecting an older counter reservation
can create replay risk. Production secure-boot/OTA policy must therefore keep
its rollback floor at or above `SECURITY_JOURNAL_FORMAT_VERSION` once a device
has run this format. The journal exposes that epoch, but software inside the
application cannot stop an older bootloader-approved image from ignoring it.

## Trust Center state

`TrustCenterDeviceJournal` is a separate two-sector Coordinator journal. It
stores the authoritative device/link-key table, reserved APS link-key counter
ranges, durable key/removal transactions, application-key allowlists, and the
Network-Key rotation phase.

TC keys must be restored before re-applying APS replay floors: the independent
TC snapshot may contain an older incoming counter than the security journal.
`TrustCenterCoordinatorApp` gates RX and transaction retries until both restores
succeed, including when initialization is retried after a storage error.

TC version 3 added a bounded list of parent/device
removal intents for rejected unknown secured rejoins. These records contain no
link key and never imply admission. An intent commits before transmission and
is removed only after local command submission/removal and durable replay
retirement. Remote submission is not proof that the child departed.
Versions 1 and 2 migrate with an empty list. The maximum encoded snapshot is
3,088 bytes (513 more than v2), within the unchanged 4 KiB journal slot;
partition addresses and journal erase geometry are unchanged. An older
firmware must not be selected after committing a newer journal epoch.

Current records use version 4. It adds known/unknown router capability in
previously reserved bits and a propagation-wait rotation phase, without
increasing the snapshot or changing partition geometry. Versions 1 through 3
remain readable; migrated device roles stay unknown until an authenticated
`Device_annce`, rather than being silently classified as end devices.
Coordinator rollback policy must not cross version 4 after writing it.

Confirm-Key and application-key progress describes local submission, never
APS-ACK confirmation. State is published only after the journal write succeeds;
a failed completion write retains its volatile submission marker to avoid
reissuing the same command within the running process.

Rotation key material exists only in `SecurityStateJournal`. Trust Center
journal version 2 introduced only the target sequence, broadcast/unicast method,
phase, and per-device unicast transmission bits. These bits do not prove
end-device key installation. The cross-journal order is
therefore security-key checkpoint first, Trust Center intent second; startup
recognizes a next-sequence staged key left by a torn intent commit.

After Switch-Key, the security journal commits the active key and NIB sequence
before the Trust Center transaction is cleared. A reboot at any boundary
resends the idempotent operation or completes the already durable activation.
Production Coordinator rollback policy must not cross either
`SECURITY_JOURNAL_FORMAT_VERSION` or
`TRUST_CENTER_JOURNAL_FORMAT_VERSION`.

## APS tables and application keys

`PersistentApsTables<A>` stores network-bound bindings, groups, application
link keys, and reserved outgoing application-key counter ranges. Restore
rejects a foreign extended PAN ID and advances every restored outgoing
reservation before the key can transmit.

Application-key Transport-Key reception spans this store and the security
replay journal. The required order is:

1. hold completion of the authenticated key-table mutation, including any
   compatibility acknowledgement for a legacy incoming AR=1 command;
2. commit the APS-table snapshot;
3. append the APS replay floor to `SecurityStateJournal`;
4. commit live replay state and release any such acknowledgement.

If power fails before step 2, neither the key nor its APS replay floor exists
after reboot and a repeated delivery can install it. If power fails between steps
2 and 3, the restored identical key is retained without resetting counters;
the repeated delivery commits the missing replay floor before any legacy AR=1
acknowledgement is released. Normal security commands use AR=0 and do not
participate in APS acknowledgement or retransmission.
`NoApsTables` therefore disables application-link-key installation rather than
accepting a mutation that cannot survive reboot.

Bind/Unbind reception in a router application selecting `PersistentApsTables`
also spans both stores. ZDO prepares an owned response while applying the
mutation, without transmitting it. The application commits the APS snapshot;
the runtime then commits the NWK and, when present, APS replay floors before
sending the APS ACK and finally the prepared ZDO response. A single pending
transaction blocks another store-backed receive/tick until completion.
Storage errors retain the transaction; ACK/response transport errors retain the unsent work for a
bounded retry on the next `step()`, without applying Bind/Unbind again.

Before the snapshot commits, a reboot retains the previous tables and does
not retire the incoming replay counter. After it commits, the binding mutation
survives reboot. The prepared response itself is RAM-owned, not a persistent
response cache. Compositions using `NoApsTables` retain their existing volatile
binding behavior.

## Generic application NV

`NvStorage` is the item API for non-security state. `LogStructuredNv<F>`
implements it over two NOR sectors. EFR32MG1 uses a separate application-NV
partition in addition to its security journal.

## Parent child table

Child persistence is intentionally separate from security state:

- `PersistentChildren<C>` is used by `ParentRouterApp`,
  `DistributedRouterApp`, and `CoordinatorApp`;
- `ChildTableJournal<F>` stores bounded child snapshots;
- `NoChildren` is used by `RelayRouterApp`.

The child journal stores the extended PAN ID and each admitted child's
identity/configuration/timeout enumeration. It does not store NWK/APS outgoing
counters. Restoring a foreign-network or corrupt snapshot is an explicit
error and the stale record is cleared before fresh parent operation.

The current child record is version 5:

- version 3 added per-child pending Remove-Device state;
- version 4 added a selected replacement address for crash-safe address
  conflict recovery;
- version 5 adds durable DeviceLeft notification state and the
  remove-children Leave-cascade/rejoin intent.

The parent durably stages an authenticated Trust Center Remove-Device before
sending the network-secured NWK Leave. An rx-on child is removed after direct
delivery; a sleepy child remains owned while the command sits in the indirect
queue and is removed only after its poll is acknowledged. Queue expiry or
reboot retries the same durable transaction, with bounded local eviction after
three attempts.

Address reassignment commits the replacement before sending the unsolicited
Rejoin Response and reuses it after reboot. A departed child is no longer
restored as live, but its DeviceLeft record remains until a local Trust Center
indication is durably consumed or a remote
`Update-Device(DeviceLeft)` is successfully submitted to the lower layer and
that local-send completion is committed. This does not prove TC receipt.
Replay tombstones precede submission; a failed completion write retains a
volatile submission marker so the same boot retries only the journal write.
Reboot may repeat an uncommitted submission. A parent-directed
remove-children Leave persists every child removal and delays its own
Leave/Rejoin until the child table is durably empty.

Ordinary snapshots are written only when the child table fingerprint changes;
lifecycle transaction checkpoints are explicit writes. Factory reset clears
child persistence before recommissioning, even if the next network has the
same extended PAN ID.

## Current product partitions

| product | security state | other protected storage |
|---|---|---|
| nRF52840 DK sensor/always-on End Device | `0xFE000..0x100000` | UF2 variants use board-specific product maps |
| nRF52833 sensor | `0x7E000..0x80000` | — |
| ESP32-C6/H2 | `0x3FE000..0x400000` | OTA slots and `otadata` are separate product partitions |
| BL702 XT-ZB1 | `0xFE000..0x100000` | — |
| CC2340R5 | `0x7E000..0x80000` | — |
| PHY6222 | `0x7E000..0x80000` | — |
| PHY6252 | `0x3E000..0x40000` | — |
| EFR32MG1 | `0x37000..0x39000` | application NV `0x39000..0x3A000`; bootloader/native regions preserved |
| EFR32MG21 | `0x7C000..0x80000` | bootloader `0x00000..0x04000` |
| TLSR8258 TB-04 | `0x74000..0x76000` | APS `0x70000..0x72000`, child `0x72000..0x74000`, factory EUI/config `0x76000..0x78000` |

Linker scripts and Rust partition wrappers independently assert the same
boundaries. Existing 8 KiB two-sector product partitions remain suitable only
for their currently bounded role/traffic assumptions; they are not a generic
production Coordinator geometry.

## Ownership examples

### ESP32

The board exposes raw chip flash. The product checks the 4 MiB partition table,
constructs a bounded security journal, performs legacy migration, and owns the
OTA writer. Neither example hard-codes the final 8 KiB address.

### TLSR8258

The product consumes one board flash token and splits it into distinct
APS/child/security capabilities while preserving factory storage. The sensor
drops the APS and child tokens; the router consumes both. Zbit page
program/erase requires a fresh, stable ADC/PC5 voltage check and fails closed.

### EFR32MG1

The product reserves security and generic application NV separately. Direct
USART0 access to external OTA storage and Gecko Bootloader-managed access are
alternative owners of the same physical path.

### EFR32MG21

The product bounds raw board flash to `0x7C000..0x80000` and instantiates
`SecurityStateJournal<PartitionFlash, { 8 * 1024 }>`. Its linker script keeps
the same region out of the application image.

## Identity and reset order

Before resuming persisted state, compare it with the factory/device EUI-64.
If identity changed, clear incompatible membership before constructing a
running node.

A durable factory reset must:

1. stop new secured work;
2. commit a factory-new security record while preserving counter floors;
3. clear child, APS-table, and Trust Center journals;
4. preserve factory identity/calibration and bootloader/OTA regions;
5. reset or start fresh commissioning.

The committed factory-new security record is the cross-journal tombstone. If
power fails before it commits, the old commissioned record and all auxiliary
stores remain usable together. If power fails after it commits, startup sees
an uncommissioned device and clears every stale auxiliary store before fresh
steering or formation, including the same-extended-PAN-ID case.

OTA activation follows the same safety rule: checkpoint security before the
reset-causing activation call.

## Hardware status

The exact 2026-09-06 images are build/layout-tested. Separately, earlier nRF,
ESP32, EFR32MG1, and deployed TLSR8258 security-persistence paths have hardware
evidence.
- BL702 journal integration builds, but destructive erase/program and
  reset/resume remain open.
- PHY62x2, CC2340, and EFR32MG21 flash paths remain hardware-unverified.
- Telink child-table persistence exists and passes host/target checks; complete
  corrected-image child acceptance remains a router HIL gate.

## Adding a backend

1. Implement `ReadNorFlash` and `NorFlash` in the chip HAL.
2. Expose the physical flash token from the board.
3. Bound it to product partitions.
4. reserve the same regions in the product linker layout;
5. construct the correct journal in the product;
6. pass the store to `ZigbeeNode`/the shared application.

Return controller failures. Never turn a failed erase/program/read-back into
success.
