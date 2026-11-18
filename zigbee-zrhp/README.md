# zigbee-zrhp

Shared Rust implementation of **ZRHP v1**, the zigbee-rs host ↔ Network
Co-Processor protocol specified in
[`docs/book/src/advanced/host-protocol.md`](../docs/book/src/advanced/host-protocol.md).
It is used unchanged by the ESP32-H2 NCP firmware, the ESP32-S3 embedded hub
(esp-idf-svc `std`) and desktop tooling.

* `#![no_std]`, no allocation, **no dependencies**, `#![deny(unsafe_code)]`.
* Sans-IO and poll-style: the caller passes a monotonic time in
  microseconds and moves bytes; no embassy/tokio dependency.
* Fixed memory selected with const generics.

Validation level: **host-tested** (all JSON vectors byte-exact, normative ARQ
traces, link property tests). Builds for `riscv32imac-unknown-none-elf`. No
hardware gate has been run.

## Layers

| Module | Spec | What it provides |
|---|---|---|
| `crc` | 5.2 | CRC-32C (`Crc32c`, `crc32c`) |
| `cobs` | 5.1 | minimal COBS `encode`/`Encoder`, `decode`/`decode_in_place`, bounded byte-at-a-time `Accumulator<N>` |
| `frame` | 5.2–5.5 | `Header`, `FrameType`, `parse`, `encode_wire` (COBS + CRC straight into the TX buffer), `AckBody`, `ResetBody`, `RejectBody`, `session_epoch` |
| `link` | 5.5–5.13 | `Link<TXW, RXW>` state machine: handshake/cookies/epoch, SR-ARQ (window, SACK, FIFO loss inference, `nbad`, RTO + Karn), per-endpoint ordering and credits, priority scheduler with anti-starvation, keepalive, link timeout, counters |
| `queue` | 5.10 | `TxSource` trait, `TxQueue<N>`, `EndpointQueues<N>` |
| `msg` | 6–7 | `MsgHeader`, `Batch` (atomic tiling check), `BatchWriter`, `Response<T>`, `ids`, `home_endpoint`, `TidAllocator`, typed bodies for every v1 command and event in `msg::{system, network, data, security, tables, backup, diag, firmware, log, gp}` |
| `records` | 9.3–9.4 | `Record` TLVs (NODE … GP_SINK, END), `StreamCrc`, `verify_stream`, `RestoreCounters` |
| `codec` | — | `Wire` trait, `Reader`/`Writer`, zero-copy `List`, `Bytes8`, `Bytes16`, `Rest` |

## Link API

```rust
use zigbee_zrhp::frame::{ResetReason, TX_BUFFER_LEN};
use zigbee_zrhp::link::{Link, LinkConfig, LinkEvent};
use zigbee_zrhp::msg::Batch;
use zigbee_zrhp::queue::EndpointQueues;

// ESP32-H2 NCP: 16 TX + 16 RX slots (22.5 KB). `new` is const, so the
// link can also live in a `static` (e.g. a StaticCell).
let mut link = Link::<16, 16>::new(LinkConfig::ncp(), rng_nonzero_u32());
let mut queues = EndpointQueues::<2048>::new();
let mut tx = [0u8; TX_BUFFER_LEN];

link.open(now_us(), ResetReason::PowerOn);
loop {
    // 1. Receive: feed bytes, drain events after every call.
    let mut off = 0;
    while off < rx.len() {
        off += link.push_rx(&rx[off..], now_us());
        while let Some(ev) = link.poll_event() {
            match ev {
                LinkEvent::Established(_) => { /* NCP: queue EVT_NCP_READY first */ }
                LinkEvent::Data { endpoint, payload } => {
                    match Batch::parse(payload) {
                        Ok(batch) => for m in batch.iter() { /* dispatch m */ },
                        Err(_) => { /* count msg_bad_batch, EVT_PROTOCOL_ERROR */ }
                    }
                }
                LinkEvent::TxFailed { payload, reason, .. } => { /* fail each message (LINK_RESET / LINK_REJECTED) */ }
                LinkEvent::PeerReset { .. } | LinkEvent::LinkDown | LinkEvent::LinkFailed => {}
            }
        }
    }
    // 2. Transmit: retransmissions, control frames, then scheduled DATA.
    while let Some(n) = link.poll_transmit(now_us(), &mut queues, &mut tx) {
        uart_write(&tx[..n]);
    }
    // 3. Sleep until new input or the next timer.
    wait_until(link.next_deadline());
}
```

`LinkConfig::{ncp, desktop_host, embedded_host}` carry the spec defaults
(window 16 / 32 / 8, credits of 5.10, ack delay 1 ms, keepalive 500 ms,
timeout 3 s, RTO 100 ms initial, 1 s max, 10 retransmissions).

Delivered `Data` payloads borrow the link's receive slot until the next call
on the link. With `auto_credit = true` (default) the slot's credit is
returned then; set it to `false` and call `release_credit` when a consumer
holds frames longer.

## Message API

```rust
use zigbee_zrhp::msg::{BatchWriter, Kind, network::NwkPermitJoin};
let mut buf = [0u8; 64];
let mut w = BatchWriter::new(&mut buf);
w.request(260, &NwkPermitJoin { duration_s: 254, target: 0xFFFC, flags: 1 })?;
// → 00 0501 0401 0400 fe fcff 01
```

Decoding is zero-copy: `RawMessage::decode::<T>()` for requests/events,
`RawMessage::decode_response::<T>()` for `Response { status, body: Option<T> }`.
Trailing bytes are ignored (spec 6.6). Lists (`List<'a, T>`) are either
views of received bytes or borrowed slices for sending.

## Tests

```text
cargo test -p zigbee-zrhp
cargo clippy -p zigbee-zrhp --all-targets -- -D warnings
cargo build -p zigbee-zrhp --target riscv32imac-unknown-none-elf --release
python3 docs/book/src/advanced/host-protocol-vectors/generate.py --check
```
