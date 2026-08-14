# Telink TLSR8258

The supported Telink platform is a pure-Rust TLSR8258 implementation.

## Hardware and SRAM layout

| Property | TLSR8258 |
|---|---|
| Core | Telink TC32 |
| HAL flash geometries | 512 KiB, 1 MiB, 2 MiB, 4 MiB |
| TB-04 fitted flash | 512 KiB |
| SRAM | 64 KiB at `0x840000..0x850000` |
| Rust target | `tc32-unknown-none-elf` |
| Toolchain | [modern-tc32](https://github.com/modern-tc32/rust) |

RAM-resident code occupies the bottom of SRAM. The hardware instruction cache
then requires 0x100 bytes of tags and 0x800 bytes of cache data. Writable
`.data` and `.bss` therefore start at:

```text
0x840900 + align256(ram_code_size)
```

Both production linker scripts and the post-link checker enforce this
reservation. The SVC stack occupies `0x84BC00..0x84FC00`; the IRQ stack ends
at `0x850000`.

## Repository structure

```text
examples/telink-tlsr8258-sensor/  polling end-device sensor
examples/telink-tlsr8258-router/  always-on parent router
tools/telink-tlsr8258-lab/        bring-up and regression firmware
tlsr8258-hal/                     clocks, timers, flash, radio, GPIO, ADC,
                                  I2C, SPI, UART, PWM, PM, IRQ/reset,
                                  capture, RNG, AES, ownership tokens
tlsr8258-rt/                      reset, IRQ context, RAM initialization
boards/tlsr8258-tb04/             fitted LEDs, flash token, typed resources
products/tlsr8258-tb04/           protected flash partition and linker policy
zigbee-mac/src/telink/            reusable TLSR8258 MacDriver
```

The board crate exposes only physical resources. The product crate owns the
bounded security partition, journal construction, and linker layout. The
application examples contain role-specific Zigbee logic. The old direct-MMIO
radio, local MAC, SRAM markers, and diagnostic modes are retained only in the
hardware lab.

## Toolchain

Install the current supported release under `.toolchains`:

```bash
TAG=tc32-stage2-tc32-45
case "$(uname -s)-$(uname -m)" in
  Darwin-arm64)  ASSET=tc32-rust-toolchain-macos-arm64.tar.gz ;;
  Darwin-x86_64) ASSET=tc32-rust-toolchain-macos-amd64.tar.gz ;;
  Linux-x86_64)  ASSET=tc32-rust-toolchain-linux-amd64.tar.gz ;;
  *) echo "Unsupported host"; exit 1 ;;
esac

DEST=".toolchains/${TAG}"
mkdir -p "$DEST"
curl -fL \
  "https://github.com/modern-tc32/rust/releases/download/${TAG}/${ASSET}" \
  -o /tmp/tc32-toolchain.tar.gz
tar -xzf /tmp/tc32-toolchain.tar.gz --strip-components=1 -C "$DEST"
"$DEST/bin/rustc" --version
```

An external extraction can be selected with `TC32_TOOLCHAIN=/path/to/toolchain`.

## Production examples

Build from the repository root:

```bash
./scripts/tlsr8258.sh build sensor
./scripts/tlsr8258.sh build router
```

The generated images are:

```text
examples/telink-tlsr8258-sensor/target/tc32-unknown-none-elf/release/telink-tlsr8258-sensor.bin
examples/telink-tlsr8258-router/target/tc32-unknown-none-elf/release/telink-tlsr8258-router.bin
```

### Firmware footprint

With the pinned `tc32-45` toolchain, the current production payloads are:

| Image | Raw payload | Complete-HAL baseline | Reduction | CI budget |
|---|---:|---:|---:|---:|
| End-device sensor | 272,600 B | 323,876 B | 51,276 B (15.8%) | 280 KiB |
| Parent router | 332,440 B | 349,792 B | 17,352 B (5.0%) | 336 KiB |

The router is 59,840 bytes larger because it retains route maintenance, child
admission and aging, indirect delivery, parent-side MAC commands,
Update-Device handling, and Parent Announce. The sensor compiles those paths
out and retains only its leaf behavior, including the R22 End Device Timeout
client. See [Firmware Size and Role Specialization](../advanced/firmware-size.md)
for the cross-platform measurements and CI gates.

`tools/tlsr8258-firmware.sh` builds with tc32-45, emits the binary, and checks
the cache reservation, RAM code, BSS/stack separation, production image size,
and absence of the legacy lab MAC.

### Sensor

The sensor is a polling Zigbee end device:

- Basic, Power Configuration, Identify, Temperature, and Humidity clusters;
- deterministic test variation for temperature and humidity;
- crash-safe two-sector security journal;
- secured rejoin and parent polling;
- `rx_on_when_idle = false`.

`PowerMode::Sleepy` currently selects the polling end-device behavior. It does
not put the TC32 CPU into retention sleep. A separate SED example will be
added only after the production runtime layout and full Zigbee state survive
repeated LOW32K retention wakeups.

### Router

The router joins as an FFD, enters continuous receive, relays NWK traffic, and
sends router maintenance frames. Hardware has proven join, interview,
TCLK exchange, Identify, routed-frame relay, and silent reset/resume. The
accepted reset capture resumed on the previous short address without a Beacon
Request, Association Request, or Device_annce.

The bounded parent path now handles beacon requests, finite or indefinite
permit joining, child association, and indirect delivery to polling sleepy
children. Child-table entries are RAM-only and children must re-associate
after router reboot; commissioned network and security state remain
persistent.

The security path waits for the over-the-air Association Response ACK before
notifying the Trust Center, forwards tunneled Transport-Key commands without
rewriting the embedded APS frame, and keeps a child provisional until it
proves possession of the network key. Secured and centralized unsecured
rejoins are supported; distributed unsecured rejoins are rejected.

Child-parent operation has been exercised once on hardware, including child
admission and Trust Center link-key exchange. That run used an older BL702
child image, and a later corrected capture exposed parent-delivery blockers
that are now fixed in source. The release gate therefore remains a clean
first-attempt join with the corrected child image, followed by a complete ZHA
interview under an independent channel-15 sniffer.

## Hardware lab

The 7,000-line bring-up firmware is deliberately not an example. Run its
diagnostics through the root wrapper:

```bash
./scripts/tlsr8258.sh build diag-beacon
./scripts/tlsr8258.sh build diag-assoc
./scripts/tlsr8258.sh build diag-smoke
./scripts/tlsr8258.sh build diag-pm
./scripts/tlsr8258.sh build lab-sensor
```

The lab preserves the hardware evidence for raw RF, MAC timing, startup,
flash, and retention PM without obscuring the production applications.

## Peripheral HAL status

The chip HAL is direct-register Rust. Vendor objects were used only as
disassembly evidence where open headers did not contain a function body; no
Telink library is linked.

| Surface | Implemented support | Validation boundary |
|---|---|---|
| System control | Fail-closed clock bring-up, typed clock/reset gates, canonical IRQ masks/W1C acknowledgement, Timer0/Timer1, Timer2 watchdog, immediate software reset | Clock/radio/timer behavior has hardware evidence; the consolidated facades are host-tested and TC32-built |
| GPIO and capture | Unique PA0-PE7 tokens, GPIO/mux/pulls/drive, three GPIO IRQ comparators, fixed-capacity rising-edge capture queue | Existing GPIO paths have hardware evidence; the generalized IRQ/capture APIs are compile-tested |
| I2C/SPI | Four I2C groups with repeated starts and bounded recovery; two MSB-first SPI groups | Host-tested and TC32-built, not silicon-tested |
| UART | All documented TX/RX routes, fixed 8 data bits, parity/stop bits, RTS/CTS, bounded flush, nonblocking byte I/O, trigger/error IRQ status | Disassembly-derived and cross-checked, including the PB1/PB7 4800-8N1 smart-plug profile; not silicon-tested |
| ADC | Exclusive MISC-channel owner, geometry-aware factory calibration, GPIO voltage sampling, serialized sharing with RNG/flash | Register path is evidenced; physical accuracy depends on fitted calibration and board wiring |
| Flash | Raw NOR read/program/erase, bounded partitions, geometry verification, factory EUI/UID fallback, Zbit voltage guard | Deployed TB-04 persistence is hardware-proven; new geometry/guard behavior is compile-tested |
| PWM | Six channels, normal/count/IR modes, CPU-fed IR FIFO, shadow cycle/duty, typed IRQ status | Basic register path is compile-tested; advanced modes are not silicon-tested |
| Power management | Suspend/deep-sleep/retention entry, timer/pad/comparator wake-source arming, RC32K calibration, typed wake status | Timer-only suspend is hardware-proven. Pad/comparator wake and comparator front-end behavior remain unvalidated |
| RNG | AES-128 CTR_DRBG with SHA-256-conditioned VBAT/GND ADC samples, stuck-source rejection, full ADC state restore | NIST DRBG vector passes; physical min-entropy is uncharacterized and still requires SP 800-90B work |
| AES | Token-owned, bounded AES-128 ECB encrypt/decrypt, used by the production NWK/APS CCM* and AES-MMO `zigbee-crypto` provider | Dual startup KAT, secured commissioning, TCLK exchange, ZHA interview, sustained traffic, and reset/resume are hardware-proven on TB-04 |

`tlsr8258_hal::peripherals::Peripherals::take()` returns one
`SerialController`, plus independent PWM, UART, ADC, AES, and non-`Clone` GPIO
tokens. I2C and SPI consume the same serial token because their control and
route registers overlap. Radio and RNG use separate IRQ-safe singleton
handles. Shared register read-modify-writes are serialized by the central
nested-safe IRQ critical section.

`boards/tlsr8258-tb04::resources::BoardResources` retains the serial, UART,
ADC, AES, lighting, PC5, and fitted-flash ownership. The production sensor
and router install the owned ADC/PC5 flash-voltage guard before constructing
the security journal. `products/tlsr8258-tb04` bounds that journal to
`0x74000..0x76000`; the linker script prevents firmware overlap.

### Flash geometry and identity

Telink factory locations move with the fitted flash:

| Geometry | Factory EUI-64 | Factory config | ADC calibration |
|---|---:|---:|---:|
| 512 KiB | `0x76000` | `0x77000` | `0x770C0` |
| 1 MiB | `0xFF000` | `0xFE000` | `0xFE0C0` |
| 2 MiB | `0x1FF000` | `0x1FE000` | `0x1FE0C0` |
| 4 MiB | `0x3FF000` | `0x3FE000` | `0x3FE0C0` |

Non-512-KiB products must use geometry-aware constructors. They verify the
JEDEC capacity before reading a factory sector, preventing ordinary
application bytes at another geometry's address from becoming a plausible
device identity. Existing TB-04 sensor/router EUI offsets remain unchanged
for deployed persistence and ZHA compatibility; new products should use the
unchanged factory/UID-derived EUI.

The factory slot uses Telink's vendor byte layout rather than the internal
on-air byte order used by the Rust MAC. The HAL validates the OUI patterns
used by the vendor stack and decodes raw bytes as `[6, 7, 0, 1, 2, 3, 4, 5]`.
For example, raw `db 77 69 38 c1 a4 55 ed` becomes internal
`55 ed db 77 69 38 c1 a4`, or canonical
`a4:c1:38:69:77:db:ed:55`. If the factory slot is absent, the stable flash
UID supplies five device-unique bytes and the internal address ends in
`38 c1 a4`, preserving the canonical Telink `a4:c1:38` prefix.

Zbit `ZB25WD40B`/`ZB25WD80B` parts require a real ADC check before every
physical page program or sector erase. The HAL drives PC5 high, samples it,
and fails closed below 2200 mV, at 500 mV or more fluctuation, or whenever
the reading is unavailable. A constant-voltage callback cannot be installed
through the public API.

### Explicitly unsupported

UART DMA, generic DMA-channel ownership, PWM DMA, complementary PWM outputs,
USB, audio/PGA, QDEC, EMI/test features, and SWire debug control are not
modeled. The hardware-proven AES accelerator is the standard production
Zigbee CCM* backend. `MacCapabilities.hardware_security` remains `false`
because the MAC
still performs Zigbee security in the Rust stack and exposes only a hardware
block-cipher provider, not autonomous MAC security offload.

Host tests and TC32 builds cover the complete API. TB-04 exposes no fixed
I2C/SPI convenience constructor because fitted bus wiring is undocumented;
applications choose from the validated generic route groups.

## Hardware AES backend

The production sensor and router always enable the low-level
`zigbee-mac/hardware-aes` feature. The TLSR8258 AES-128 accelerator serves
every Zigbee CCM* and AES-MMO operation, replacing the RustCrypto software
core in the standard image.

### Architecture

- `zigbee_crypto::ForwardAesProvider::forward_cipher(&mut self, key)` returns a
  keyed forward AES-128 permutation. The software default returns
  `SoftwareAes128`; `PlatformServices` requires this trait as a supertrait so
  every backend is a provider.
- Under `hardware-aes`, `TelinkMac` owns a `tlsr8258_hal::aes::AesEngine`
  (installed once by the composition root from the board's exclusive `Aes`
  token via `TelinkMac::install_aes_engine`) and overrides `forward_cipher`
  to return a `HardwareAes128` borrowing that engine. There is no global
  mutable alias and no repeated peripheral re-init. Installation runs two
  back-to-back AES-128 known-answer vectors on the real accelerator before the
  engine is accepted, including a re-key/reuse cycle required by AES-MMO.
- NWK/APS security route through `_with<P: ForwardAesProvider>` variants
  (`encrypt_with`, `decrypt_with`, `derive_*_with`, MMO) passing the owned
  MAC. Plain non-`_with` wrappers stay software and are host-test only.
- A hardware AES failure (bounded timeout / handshake error) surfaces as a
  dropped frame (`BadCcmOutput` / `SecurityFail`), never a silent software
  fall-back.

### Build

```bash
# Standard production image (hardware AES):
./scripts/tlsr8258.sh build sensor
```

Confirm the software AES core is absent and the hardware engine is present:

```bash
NM=.toolchains/tc32-stage2-tc32-45/llvm/bin/llvm-nm
ELF=examples/telink-tlsr8258-sensor/target/tc32-unknown-none-elf/release/telink-tlsr8258-sensor
"$NM" -C "$ELF" | grep -c 'aes::soft::fixslice'                 # want 0
"$NM" -C "$ELF" | grep -c 'HardwareAes128\|AesEngine'           # want > 0
```

### Footprint (tc32-45, `-Os` `lto=fat`, one codegen unit)

| Image | Former software build | Current hardware release | Δ flash | Δ RAM (`_ebss`) |
|---|---|---|---|---|
| Sensor | 272 600 B | 269 960 B | −2 640 B | +8 B |
| Router | 332 440 B | 327 760 B | −4 680 B | +8 B |

The hardware image removes the ~5 KiB RustCrypto core while adding the
on-silicon startup KATs; the +8 B RAM is the `AesEngine` handle stored in the
MAC. Both hardware images pass the TC32 layout/symbol gate (`_ebss` far below
the stack, image below the `0x74000` security-journal boundary, reusable
`TelinkMac` linked).

### Hardware validation evidence

The TB-04 parent-router image passed the following channel-15 acceptance run:

- both startup AES-128 known-answer vectors passed on every boot;
- association, Transport-Key installation, NWK CCM*, APS CCM*, and secured
  Device Announcement completed;
- Request-Key, unique-TCLK installation, AES-MMO Verify-Key, and Confirm-Key
  each succeeded once with no rejection in the pre-fix image; its legacy
  APS-secured Verify-Key used frame counter 173,066;
- Node, Active Endpoint, and Simple Descriptor requests completed, followed
  by Basic and Identify traffic and ZHA availability;
- the successful commissioning completed in 2.551 seconds and the capture
  retained 22,773 packets over 658.874 seconds of secured traffic;
- a separate reset capture resumed short address `0xBC92` without Beacon
  Request, Association Request, or Device Announcement, then emitted secured
  Link Status and relayed secured ZCL traffic for 7,231 captured packets.

The first commissioning attempt exposed a persistence-procedure problem, not
an AES error. A manually erased journal restarted the outgoing default-TCLK
counter below the Trust Center's retained replay floor. The captured
Request-Key ciphertext and MIC exactly matched an independent software CCM*
calculation. Repeating commissioning with a credential-free journal record
that retained the previous 173,056 counter bound completed immediately.

A later GSDK-style TCLK regression used the same TB-04 against ZHA. A fresh
join reached network-up in 689 ms and completed the unique-TCLK exchange in
4.247 seconds with one Node Descriptor, Request-Key, Verify-Key, and
Confirm-Key pass and no rejection. ZHA then removed the router with
`Remove Children` set; the device cleared its persisted credentials while
retaining both counter floors, left normally, and immediately recommissioned
from short address `0x7AF5` to `0x6131`. The second TCLK exchange completed in
2.693 seconds, again on the first pass, and its Verify-Key counter advanced
from 225,291 to 227,339. This run used ZHA state plus on-device RAM metrics,
not an independent packet capture, so it is regression evidence rather than
a substitute for the ZiGate capture gate below.

Those runs remain valid hardware-AES and runtime evidence. The corrected
APS-unsecured Verify-Key wire format now also has the fresh ZiGate
commissioning capture described below; reset/resume remains covered by the
earlier hardware-AES acceptance run.

### Safe recommissioning procedure

1. Back up the two-sector security journal before changing network state.
2. Remove the device from ZHA and let the running firmware process Leave or
   factory reset. The runtime clears credentials while preserving outgoing
   counter bounds.
3. Do **not** erase `0x74000..0x76000` merely to force a clean join. If raw
   erasure is unavoidable, seed a valid uncommissioned record with the
   previous counter bounds for that device identity; never copy old network
   keys or TCLK credentials into the clean record.
4. Flash with `./scripts/tlsr8258.sh flash router`, open permit
   joining, and confirm the secured join and TCLK exchange with an independent
   sniffer.
5. If a startup KAT fails, do not commission the device; diagnose the
   accelerator path or re-flash a previously known-good hardware-AES release.

### ZiGate TCLK interoperability gate

A router that appears online only while ZiGate permit-join is open is not
gating ordinary traffic on the permit bit. The observed failure path is a
post-join Trust Center link-key exchange failure: after the bounded TCLK
attempts are exhausted, `reset_after_tclk_failure()` performs
`nlme_reset(false)`, the runtime reports failed commissioning, and the
application starts full network steering again. That new association succeeds
only while permit-join remains open.

The regression boundary changed the local Node Descriptor server mask from
stack-compliance revision 0 to revision 22. ZiGate's build requires TCLK
exchange for R21+ nodes and gives a newly joined node 15 seconds to complete
authentication. The zigbee-rs exchange now starts after 300 ms and keeps
independent three-transmission budgets for Node Descriptor, Request-Key, and
Verify-Key. Their response windows are 1.5, 3, and 5 seconds respectively,
with 250 ms paced retransmissions and one strict 15-second overall deadline.
The first pass completes within 9.8 seconds; retries stay on the current
message type, and a lost Confirm-Key retransmits Verify-Key without discarding
the installed unique TCLK.

ZiGate `v3.1d` also had a Trust Center callback bug:
`APP_bSendHATransportKey` used
`bSetTclkFlashFeature || u8Status == 1`. Version `v3.1e` changed this to `&&`
and lists `Fix HATransportKey function (Device Authentification)`; current
`v3.23` retains the fix. Check the radio version with ZiGate command `0x0010`
and response `0x8010` before debugging the device.

Do not add a `ZigBeeAlliance09` fallback for Confirm-Key: that public key
cannot prove possession of the negotiated pairwise TCLK. Diagnose with an
independent capture plus `SteeringDiagnostics`:

- Request-Key sent late or a coordinator Leave near 15 seconds indicates the
  coordinator authentication timeout;
- Verify-Key must have APS frame control `0x41`: command, unicast,
  acknowledgement requested, APS security disabled;
- a non-zero legacy `last_verify_key_frame_counter` means the sender consumed a
  TCLK APS security counter and emitted the wrong wire format;
- a decrypted Confirm-Key with `confirm_key_rejections > 0` identifies a
  field/source validation mismatch rather than a MIC/key-selection failure.

### ZiGate v3.23 Verify-Key wire-format isolation

The decisive 2026-08-13 channel-15 capture contains a successful factory IKEA
join and the first clean TLSR8258 attempt in the same ZiGate network:

- IKEA sends `Verify-Key` with APS frame control `0x41`: command, unicast,
  acknowledgement requested, **APS security disabled**. ZiGate validates the
  hash and sends `Confirm-Key`.
- TLSR8258 sent the same semantic command with `0x61`: the APS security bit was
  set and a Data-Key auxiliary header/MIC followed. ZiGate acknowledged the
  frame but never entered its Verify-Key hash-validation callback and never
  sent `Confirm-Key`.
- Command ID `0x0F`, key type `0x04`, source IEEE address, hash, destination,
  and timing were otherwise correct.

This matches R22 Table 4-7 and §4.4.7.1.3: `Verify-Key` **shall not be APS
encrypted**. The implementation now:

1. sends `Verify-Key` as `0x41`, inside a secured NWK frame, with no APS
   auxiliary security header or MIC;
2. computes the hash from the installed unique TCLK but does not consume that
   key's outgoing APS security counter;
3. treats any APS acknowledgement as delivery feedback only — commissioning
   requires an authenticated successful `Confirm-Key`; and
4. uses the normal 300 ms delay and Node Descriptor probe again; the
   timing-only fast diagnostic policy was removed.

The older 2026-08-09 capture remains useful proof that command-format APS ACKs
must parse according to `ack_format`, but its ACK cannot be used as proof of
unique-key possession.

A fresh first-attempt ZiGate v3.23 acceptance capture now closes this gate:

- the router associated once as `0x7329`, with no Leave or second Association
  Request in the capture;
- frame 4885 is the router's NWK-secured `Verify-Key` with APS frame control
  `0x41`, no APS auxiliary security header, and no APS MIC;
- ZiGate returned the command-format APS ACK in frame 4889 and an APS-secured
  successful `Confirm-Key` in frame 4891, 31.566 ms after `Verify-Key`;
- ZiGate's sanitized diagnostic counters recorded one completed hash
  validation and one successful Confirm-Key send;
- `TELINK_JOIN_METRICS` recorded one Request-Key, one Verify-Key, one
  authenticated successful Confirm-Key, TCLK completion at 1.971 s, full
  commissioning at 1.982 s, and a zero legacy Verify-Key APS security counter.

The capture contains 7,481 packets over 209.336444 seconds and has SHA-256
`fd649cafadb9aa94526fe4aab24d5ce8fd6ecf5a7680ffeb722b541a78409b91`.

The older 2026-08-09 capture also shows the `Simple_Desc_req` for
`0x83a6`/endpoint 1
(frame 1001) being received and APS-acknowledged while no `Simple_Desc_rsp`
(`0x8004`) ever reaches the coordinator, which is the z2m timeout on cluster
32772. Host tests prove the ZDO dispatcher always produces that response, so
the loss is below ZDO. Three transmit-side defects fixed alongside are directly
implicated:

- unicast ZDP frames requested **no** APS acknowledgement, so a lost response
  had no APS retransmission and no delivery failure signal (R22 §2.4.1.2
  requires the acknowledgement);
- a duplicate incoming data frame was dropped **without** regenerating its
  acknowledgement, so a coordinator retransmission was answered with silence;
  and
- APS retransmissions were sent to `0xFFFF` instead of the original unicast
  destination.

All three are host-tested only. The remaining gate is a post-flash capture:
whether the plug now stays joined past 15 s and whether `Simple_Desc_rsp`
appears on air.

The same trace also rules out the Node Descriptor parser. The coordinator's
`Node_Desc_rsp` for TSN 97 is well formed, advertises stack-compliance revision
22, and passes the host parser test. It arrived on air 45 ms after the request
but did not reach the pending ZDO response slot; the request was retried at the
exact 1.5-second timeout boundary. That timing is a receive-delivery loss, not
a parse rejection.

Live RAM diagnostics on the TB-04 then proved the lower-layer mechanism. Over
397,519 valid received frames, the old eight-slot interrupt queue reported
17,685 overflows, while `dma_incomplete` remained zero. Invalid-length and
invalid-CRC outcomes also consumed bounded queue slots even though the MAC
discarded them immediately. The fix therefore stays above the RF registers:

- invalid outcomes are counted but never occupy the interrupt queue;
- the queue has 16 measured slots and moves only slot indices during
  overload, avoiding the old 129-byte volatile copies in interrupt context;
- local unicast traffic outranks broadcasts, unattributable queued ACKs, and
  provably foreign traffic, so stale channel traffic cannot evict a new ZDO
  response;
- expected transmit ACKs remain on the synchronous polled ACK path and do not
  depend on the interrupt queue; and
- HAL and MAC queue overflow, eviction, and high-water counters are exported
  separately through `TELINK_JOIN_METRICS`.

`overflow - evicted` is the number of arrivals dropped outright. The
post-flash ZiGate gate requires that value to stay at zero for both data
queues, no command-event overflow, and delivery of the first Node/Simple
Descriptor responses without a timeout retry.

## Current capability boundary

The TLSR8258 backend provides active/passive/energy scan, association, data
request polling, unicast TX/RX, CSMA-CA, ACK retries, software ACK generation,
mandatory timing, and crash-safe security persistence. Home Assistant ZHA has
verified commissioning, TCLK exchange, interview, reporting, reset resume,
secured rejoin, and router join/relay setup.

Router restart, maintenance traffic, and NWK relay are hardware-verified.
Child-parent support is software-complete and has partial hardware evidence,
but remains behind the clean corrected-image sniffer gate described above.
SWire RAM/flash inspection is restart-intrusive on the tested programmer and
must be performed only after stopping an acceptance capture. Full coordinator
support is not advertised.
