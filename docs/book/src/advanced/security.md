# Security

Zigbee uses a layered security model to protect data in transit. zigbee-rs
implements two of the three layers — NWK-level encryption (shared network key)
and APS-level encryption (per-device link keys). MAC-level security is **not**
used for normal Zigbee 3.0 data frames.

---

## Security Model Overview

```text
┌────────────────────────────────────────────┐
│                APS Layer                    │  Optional end-to-end encryption
│  APS link keys (per device pair)            │  between two specific devices.
├────────────────────────────────────────────┤
│                NWK Layer                    │  Mandatory hop-by-hop encryption
│  Network key (shared by all devices)        │  for ALL routed frames.
├────────────────────────────────────────────┤
│                MAC Layer                    │  NOT used in Zigbee 3.0 for
│  (unused for normal Zigbee data frames)     │  data frames — only for beacons.
└────────────────────────────────────────────┘
```

**NWK security** is always on. Every frame routed through the mesh is encrypted
with the shared network key and authenticated with a 4-byte MIC (Message
Integrity Code).

**APS security** is optional and provides end-to-end confidentiality between
two specific devices. It's used for sensitive operations like network key
transport and can also be used for application-level data.

---

## NWK Security

The NWK security implementation lives in `zigbee_nwk::security`.

### Network Key

All devices on a Zigbee network share the same 128-bit AES network key. The
coordinator generates it during network formation; joining devices receive it
(encrypted) from the Trust Center.

```rust
pub type AesKey = [u8; 16];

pub struct NetworkKeyEntry {
    pub key: AesKey,
    pub seq_number: u8,   // 0–255, rotated on key update
    pub active: bool,
}
```

The stack stores up to `MAX_NETWORK_KEYS` (2) entries: the current active key
and either the next staged key or the previous key retained after activation.
Both slots survive reboot, including the distinction between "future" and
"previous", so a restart during rotation still accepts old- and new-sequence
traffic without treating the previous key as a pending Switch-Key target.

```rust
// Set a new network key (moves current key to "previous" slot)
nwk_security.set_network_key(new_key, seq_number);

// Retrieve the active key
let key = nwk_security.active_key().unwrap();

// Look up a key by its sequence number (for decrypting incoming frames)
let key = nwk_security.key_by_seq(frame_key_seq);
```

### AES-128-CCM\* Encryption

Zigbee uses Security Level 5: **ENC-MIC-32** — the payload is encrypted *and*
authenticated with a 4-byte MIC. The implementation uses the RustCrypto `aes`
and `ccm` crates (pure Rust, `#![no_std]`, no allocator):

```rust
type ZigbeeCcm = Ccm<Aes128, U4, U13>;  // M=4 byte MIC, L=2, nonce=13
```

The CCM\* nonce is built from the security auxiliary header:

```text
Nonce (13 bytes) = source_address (8) || frame_counter (4) || security_control (1)
```

> **Spec quirk:** The security level in the over-the-air security control byte
> is always `0` (per Zigbee spec §4.3.1.2). The actual level (`5` = ENC-MIC-32)
> is substituted when building the nonce for encryption/decryption.

### NWK Security Header

Every secured NWK frame carries an auxiliary security header:

```rust
pub struct NwkSecurityHeader {
    pub security_control: u8,       // always 0x2D for Zigbee PRO
    pub frame_counter: u32,         // replay protection
    pub source_address: IeeeAddress, // 64-bit IEEE address of sender
    pub key_seq_number: u8,         // which network key was used
}
```

The constant `NwkSecurityHeader::ZIGBEE_DEFAULT` (`0x2D`) encodes:
- Security Level = 5 (ENC-MIC-32)
- Key Identifier = 1 (Network Key)
- Extended Nonce = 1 (source address present)

### Replay Protection

Each device maintains a per-source frame counter table. Incoming frames are
accepted only if their counter is *strictly greater* than the last seen value
for that source:

```rust
// Step 1: check (before decryption, so we don't waste CPU)
if !nwk_security.check_frame_counter(&source_ieee, frame_counter) {
    // Replay attack — drop the frame
    return;
}

// Step 2: decrypt and verify MIC
let plaintext = nwk_security.decrypt(nwk_hdr, ciphertext, key, &sec_hdr)?;

// Step 3: durably append the key-bound replay floor.
store.commit_replay_counter(replay)?;

// Step 4: only now commit RAM state and relay/ACK/dispatch the frame.
nwk_security.commit_frame_counter(&source_ieee, frame_counter);
```

The two-phase check-then-commit pattern prevents an attacker from advancing the
counter table with forged frames that fail MIC verification. Durable commit
also prevents a power cut from reopening an already accepted nonce. The
secured Rejoin Response path uses the same ordering before changing the parent,
short address, or commissioned live state.

Replay floors are bound to both their sender/domain and the key fingerprint.
When a link key or the previous Network Key is finally retired, the
replacement state is committed first and the obsolete replay domains are then
removed with a crash-safe tombstone. Exact-domain tombstones preserve another
live APS domain that intentionally uses identical key material; device
tombstones do not erase the shared preconfigured/distributed global domains.

### Persisted rejoin policy

Persisted recovery first attempts a secure rejoin with the stored active
network key. If that fails, a Trust Center rejoin is allowed only when the
durable `NodeJoinLinkKeyType` identifies centralized security. The fallback
sends an unsecured NWK Rejoin Request, retains the APS Trust Center link key,
and waits for the current network key under APS key-transport protection.

The received key, key sequence, parent state, Trust Center incoming counter,
and a new outgoing NWK counter reservation are committed before
`Device_annce`, End Device Timeout negotiation, or normal traffic. A failed
commit leaves `rejoin_pending` durable and sends no announcement. Distributed
global and Touchlink link-key types never use this fallback, as required by
R22 §4.6.3.3.2.

Because that fallback accepted its NWK Rejoin Response before NWK
authentication was available, the selected parent is persisted as
**provisional**. Endpoint-0/ZDO traffic remains available for management and
proof, but application endpoint traffic fails with `SecurityFail`. Only a
direct NWK-secured frame from the selected parent clears the gate; a frame
relayed by another router does not. The cleared state is checkpointed, so a
reboot cannot reopen the provisional restriction or accidentally trust a
different neighbor.

---

## APS Security

The APS security implementation lives in `zigbee_aps::security`.

### Key Types

```rust
pub enum ApsKeyType {
    TrustCenterMasterKey    = 0x00,  // pre-installed master key
    TrustCenterLinkKey      = 0x01,  // TC ↔ device link key
    NetworkKey              = 0x02,  // the shared network key
    ApplicationLinkKey      = 0x03,  // app-level key between two devices
}
```

There is no `DistributedGlobalLinkKey = 0x04` APS key type. `0x04` is the
wire value for a Trust Center link key. A distributed global key is a
separately provisioned network credential used only to authenticate the
initial distributed Transport-Key exchange.

### The Default Trust Center Link Key

Every Zigbee 3.0 device ships with a well-known Trust Center link key
pre-installed:

```rust
/// "ZigBeeAlliance09" in ASCII
pub const DEFAULT_TC_LINK_KEY: [u8; 16] = [
    0x5A, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6C,  // ZigBeeAl
    0x6C, 0x69, 0x61, 0x6E, 0x63, 0x65, 0x30, 0x39,  // liance09
];
```

During joining, the Trust Center encrypts the network key with this link key
before sending it to the new device. Because the key is well-known, anyone
within radio range can capture the network key during the join window. For
production deployments, **install codes** provide per-device unique keys.

### Distributed security

A distributed PAN has no Trust Center:

- `apsTrustCenterAddress` is `FF:FF:FF:FF:FF:FF:FF:FF`;
- the joining device's parent transports the initial network key;
- the command descriptor names the all-ones Trust Center sentinel, while the
  APS auxiliary nonce still names the real parent IEEE address;
- Network-Key update/Switch-Key and TCLK exchange are not used.

Production firmware must provision its certified distributed global key
before formation or join. The runtime fails closed if no key is configured:

```rust,ignore
device.set_distributed_security_link_key(product::DISTRIBUTED_SECURITY_KEY);
```

`DISTRIBUTED_SECURITY_TEST_LINK_KEY` (`D0..DF`) is only a public BDB
certification key.

### APS Security Header

```rust
pub struct ApsSecurityHeader {
    pub security_control: u8,
    pub frame_counter: u32,
    pub source_address: Option<IeeeAddress>,  // if extended nonce bit set
    pub key_seq_number: Option<u8>,           // if Key ID = Network Key
}
```

Security level constants:

| Constant | Value | Meaning |
|----------|-------|---------|
| `SEC_LEVEL_NONE` | 0x00 | No security |
| `SEC_LEVEL_MIC_32` | 0x01 | Auth only, 4-byte MIC |
| `SEC_LEVEL_ENC_MIC_32` | 0x05 | Encrypt + 4-byte MIC (default) |
| `SEC_LEVEL_ENC_MIC_64` | 0x06 | Encrypt + 8-byte MIC |
| `SEC_LEVEL_ENC_MIC_128` | 0x07 | Encrypt + 16-byte MIC |

Key identifier constants:

| Constant | Value | When Used |
|----------|-------|-----------|
| `KEY_ID_DATA_KEY` | 0x00 | Link key (TC or application) |
| `KEY_ID_NETWORK_KEY` | 0x01 | Network key |
| `KEY_ID_KEY_TRANSPORT` | 0x02 | Key-transport key |
| `KEY_ID_KEY_LOAD` | 0x03 | Key-load key |

### Link Key Table

The `ApsSecurity` context manages a table of per-device link keys:

```rust
pub struct ApsSecurity {
    key_table: heapless::Vec<ApsLinkKeyEntry, 16>,  // MAX_KEY_TABLE_ENTRIES = 16
    default_tc_link_key: AesKey,
}

pub struct ApsLinkKeyEntry {
    pub partner_address: IeeeAddress,
    pub key: AesKey,
    pub key_type: ApsKeyType,
    pub outgoing_frame_counter: u32,
    pub incoming_frame_counter: u32,
}
```

Key management methods:

```rust
let mut aps_sec = ApsSecurity::new();

// The default TC link key is pre-loaded
assert_eq!(aps_sec.default_tc_link_key(), &DEFAULT_TC_LINK_KEY);

// Add an application link key for a specific partner
aps_sec.add_key(ApsLinkKeyEntry {
    partner_address: partner_ieee,
    key: my_app_key,
    key_type: ApsKeyType::ApplicationLinkKey,
    outgoing_frame_counter: 0,
    incoming_frame_counter: 0,
})?;

// Look up a key
let entry = aps_sec.find_key(&partner_ieee, ApsKeyType::ApplicationLinkKey);

// Remove a key
aps_sec.remove_key(&partner_ieee, ApsKeyType::ApplicationLinkKey);
```

### Durable application-key installation

Incoming application-link-key Transport-Key commands are an optional product
capability named `application-link-key-installation`. They are enabled only by
a composition that also owns durable APS-table storage.
`PersistentApsTables<A>` enables the runtime capability; `NoApsTables` leaves
it disabled and the command is rejected without mutating the live key table.

An accepted key uses this cross-store commit order:

1. install the key in RAM but hold completion and any compatibility
   acknowledgement for a legacy incoming AR=1 command;
2. commit the network-bound APS table and its application-key counter
   reservation;
3. when replacing an existing key, tombstone the retired key's replay domain;
4. append the authenticated APS replay floor to `SecurityStateJournal`;
5. commit the replay floor in RAM and release any such acknowledgement.

A storage error stops at the failed boundary and sends no acknowledgement.
Normal security commands use AR=0 and have no APS acknowledgement or APS
retransmission procedure; repeated delivery can still follow a sender restart.
After reboot, a retry before the APS-table commit installs the key normally.
A retry after the APS-table commit finds the identical durable key, preserves
its counters, completes any missing retirement tombstone, commits the incoming
replay floor, and only then releases a legacy AR=1 acknowledgement, if requested.

---

## Network Key Distribution

When a new device joins the network, the Trust Center distributes the network
key through this sequence:

1. **Device sends Association Request** (MAC layer, unencrypted).
2. **Parent router forwards the request** to the Trust Center.
3. **Trust Center encrypts the network key** with the joining device's TC link
   key (either the well-known default or an install-code-derived key).
4. **APS Transport-Key command** carries the encrypted network key to the
   device via its parent router.
5. **Device decrypts the network key** and stores it in NV.
6. **Device sends APS Update-Device** to confirm it's now secured.

After this exchange, the device can encrypt and decrypt NWK frames like all
other nodes on the network.

---

## Install Codes

Install codes provide a per-device unique link key, eliminating the security
weakness of the well-known default key. An install code is:

- A 6, 8, 12, or 16-byte random value printed on the device label
- Combined with a 2-byte CRC-16
- Hashed using Matyas–Meyer–Oseas (MMO) to derive a unique 128-bit link key
- Pre-provisioned on the Trust Center *before* the device joins

The authoritative Coordinator implementation can provision and derive an
install-code key before admitting the device:

```rust,ignore
let derived_key =
    app.provision_install_code(device_ieee, install_code_with_crc)?;
```

CRC validation and AES-MMO derivation are implemented. The
`trust_center_install_code_policy` and join policy attributes decide whether
unknown/default-key joins remain admissible. This path is host-tested with
`MockMac`; no production backend currently advertises Coordinator or
Trust-Center server capability.

---

## Trust Center security-command completion

R22 section 4.4.10 requires APS ACK-request **zero** for security commands;
sections 2.2.8.4.3–4 exclude them from APS retransmission. The TC's Confirm-Key,
application-link Transport-Key, and Remove-Device, and a parent's Update-Device
send intents therefore track local NLDE submission, not an APS ACK or proof of
remote processing. An indirect enqueue is only local submission. Normal APS
data acknowledgements and retries are unchanged.

A parent's `Update-Device(DeviceLeft)` intent is durable before submission.
Child replay retirement must finish before sending; successful submission
allows the child-journal intent to be retired without a peer response. If that
completion write fails, a volatile marker suppresses same-process resubmission;
after reboot the still-durable intent can be submitted again. New live child
membership supersedes an unsent departure. A local TC indication instead waits
for explicit durable consumption by the composition root.
Once the TC has a bound parent/address for a device, a DeviceLeft report for
an older parent or short address is ignored rather than revoking the new
membership. The wire command has no join-incarnation token, so this is not
an exactly-once guarantee for a later rejoin with the same parent and address.

A successful Verify-Key promotes the TC's replacement key before Confirm-Key
is sent (R22 section 4.4.7.2.3). Losing Confirm-Key does not roll that promotion
back. The journal commits the verified key and response intent together;
failure restores the prior policy state. A failed completion write retains
the volatile submission marker, avoiding another Confirm-Key request and
another incoming-counter reset within that boot. After reboot an uncommitted
send intent is conservatively resumed, without claiming exactly-once delivery.

Application-key distribution preserves one key generation for both recipients
and journals their local send progress separately. Local revocation after
Remove-Device submission still requires durable replay retirement; it does
not assert that the remote child has left. An authenticated DeviceLeft
indication is separate departure evidence.

---

## Key Rotation

`TrustCenterCoordinatorApp` performs centralized Network-Key rotation using
the BDB update method:

```rust,ignore
app.node_mut()
    .device_mut()
    .bdb_mut()
    .attributes_mut()
    .trust_center_network_key_update_method =
        NetworkKeyUpdateMethod::Broadcast; // or Unicast

app.request_network_key_rotation().await?;
```

The configured `trust_center_network_key_update_period` starts the same
transaction automatically when non-zero. Distributed-security networks do not
run this Trust Center procedure. The built-in interval is monotonic-uptime
based and restarts after reboot; products that require a wall-clock cadence
should invoke `request_network_key_rotation()` from their durable scheduler.

The crash-safe order is:

1. Generate sequence `(active + 1) mod 256` and stage the new key.
2. Commit the staged key to `SecurityStateJournal`.
   Retire replay records for a displaced previous key only after this write.
   For broadcast distribution, also commit the explicit local-child forwarding
   intent before the TC rotation record or any transmission.
3. Commit phase, update method, and per-router unicast progress to the Trust
   Center journal.
4. Send broadcast or router-only unicast APS Transport-Key under the old
   active NWK key. Security commands must not request an APS acknowledgement;
   recorded transmission progress is not proof that a child installed a key.
   Router membership comes from authenticated `Device_annce`; unicast mode
   returns `RouterListIncomplete` while an admitted peer's role is unknown.
5. Allow at least 30 seconds, or the configured local forwarding window if
   longer, on a populated network before broadcasting Switch-Key. This is
   implementation policy, not an R22 timer; a restart in this phase restarts
   the interval.
6. Broadcast Switch-Key while the old key is still active locally.
7. Mark the transaction `Activating`, switch the local key and NIB sequence
   together, then checkpoint the security journal.
8. Retain the previous key and its replay floors for receive compatibility
   until a subsequent update replaces the alternate slot. Never transmit
   with the previous key after activation.
9. Clear the Trust Center transaction only after those commits succeed.

Initial key transport is a separate durable intent. Indirect enqueue, MAC
delivery, or delivery of a tunnel to a remote parent does not close it.
A fresh authenticated matching `Device_annce` or security exchange closes it;
restored neighbor authorization alone does not. Otherwise it is retried after
restart and at bounded intervals during the join. Failed intent creation
neither admits the peer nor permits a later poll to send an uncommitted key.

Broadcast updates require parents to forward the all-zero-destination key
descriptor to sleepy children (R22 sections 4.4.2.1.3 and 4.4.2.3). This is
distinct from the router-only unicast update policy. A sleepy device that
misses key transport outside the parent's delivery window may need Trust
Center rejoin; R22 does not promise indefinite offline-key delivery.

The originating TC uses the same bounded forwarding path for its own children,
including after reboot during propagation. Products configure
`set_network_key_forwarding_max_poll_interval_us()` before initialization on
every boot. The window is twice that profile interval; the current eight-second
default interval is implementation policy, not an R22 numerical recommendation.

Every phase is idempotent across reboot. A staged security key whose Trust
Center intent was torn is recognized and resumed; a Switch-Key may be resent;
and an already checkpointed activation is completed without switching twice.
The previous NWK key remains in the second durable slot after transaction
completion, together with its replay floors. A later update replaces it.
An authenticated, fresh packet using a newer staged key can activate that
key even if a sleepy device missed Switch-Key; old-key reception never
switches the active sequence backward (R22 sections 4.3.1.2 and 4.7.3.10.6).
Secured indirect frames are rebuilt on each child poll with the current key
and a fresh reserved counter, not transmitted as stale queued ciphertext.
A subsequent forced rotation waits for the preceding broadcast delivery
interval; this conservative guard restarts after reboot. When a later update
replaces the previous-key slot, its replay records are retired only after
the replacement key snapshot is durable and before distribution begins.

### Host restart qualification

The `key_power_cuts` tests in `apps/router/tests/lifecycle.rs` record the
successful path's journal commits, then rerun it with a failure before each
commit in turn. Each run requires the selected failure to occur and propagate
as an error before rebuilding the coordinator from durable state.

- `sleepy_rotation_recovers_at_every_security_and_tc_commit` exercises staged
  preparation, local sleepy-child forwarding, propagation, Switch-Key,
  activation and completion. Recovery preserves the staged key once committed,
  restarts the propagation wait where needed, retains the previous receive key
  and skips the previous outgoing-counter reservation.
- `sleepy_initial_completion_recovers_at_every_replay_and_tc_commit` starts
  from a durable initial-delivery intent. MAC delivery and restored child
  state cannot complete it. After a replay/TC commit failure, a previously
  committed announcement remains rejected as a replay; fresh authenticated
  evidence completes the intent.

These are bounded software failure/restart sweeps with a real coordinator
application and `MockMac`. They model errors before atomic store commits, not
torn flash writes, radio timing, or a hardware power-cut campaign.

---

## Summary

| Layer | Key | Scope | MIC Size | Required? |
|-------|-----|-------|----------|-----------|
| NWK | Network key | All devices | 4 bytes | **Yes** (always on) |
| APS | Link key (TC or app) | Two specific devices | 4 bytes | Optional |
| MAC | — | — | — | Not used in Zigbee 3.0 |
