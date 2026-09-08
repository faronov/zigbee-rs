# Introduction

`zigbee-rs` is a heap-free, `no_std`, pure-Rust Zigbee PRO stack with reusable
sleepy-sensor, router, and coordinator application frontends.

This book describes the current `experiment/r22-bdb-complete` worktree. The
normative target is Zigbee Core R22 (`05-3474-22`) with BDB 3.0.1
(`16-02828-012`). The book source is authoritative for the branch. GitHub
Pages deploys only after a push to `main` or `master`, so this revision is not
public on Pages until the branch is merged and deployed.

## What is shared

The protocol and lifecycle path is:

```text
application profile
        |
sensor-sed / router-app
        |
zigbee-runtime::ZigbeeNode
        |
BDB -> ZCL/ZDO -> APS -> NWK -> MAC
        |
platform radio backend
```

The stack supplies:

- IEEE 802.15.4 MAC integration;
- NWK routing and security;
- APS security, binding, groups, and fragmentation;
- ZDO discovery and device management;
- ZCL clusters, reporting, and OTA transport;
- BDB commissioning;
- crash-safe security-state and child-table persistence;
- reusable finite application lifecycles.

## What varies by target

```text
application/profile  device behavior, clusters, measurement mapping
        ↓
product              identity, layout, persistence, bootloader/OTA
        ↓
board                physical wiring and fitted hardware
        ↓
platform/chip HAL    clocks, GPIO, buses, timers, flash, radio
```

The short example `main.rs` is the composition root that wires these layers.
Moving an existing product behavior to another MCU should change board and
platform adapters, not copy the Zigbee or application state machines.

## Static embedded design

The embedded path uses:

- concrete generic types;
- fixed-capacity and `heapless` storage;
- explicit ownership bundles;
- narrow capability traits;
- product-owned linker and persistence boundaries.

It does not require devicetree, Kconfig, a heap allocator, runtime hardware
discovery, or a broad platform “god trait.” Public application composition has
no trait objects. One internal outlined `dyn Future` path in `zigbee-runtime`
controls TC32 code size without allocation; it is not a platform API.

## Validation language

This book distinguishes:

- **protocol implementation** — the code path exists and unsupported
  operations fail explicitly;
- **host-tested** — portable behavior passed host tests;
- **cross-compiled/build-tested** — the target image and named layout/symbol
  gates passed;
- **exact-image HIL/packet capture** — that exact byte image ran on silicon
  or was observed over the air;
- **certified** — an external Zigbee certification result.

A release image that reaches a sleep instruction is not proof of current
consumption or correct wake restoration. A compiled flash journal is not proof
of power-loss safety on that controller. Platform guides state the remaining
hardware gates explicitly.

Host tests and release builds are evidence, not a Zigbee certification claim;
the role and platform chapters keep remaining hardware/interoperability gates
explicit. The consolidated [R22 / BDB status](reference/conformance.md)
separates implemented behavior from those open gates.

The exact 2026-09-06 firmware measurements are build/layout-tested. Hardware
results described in this book are prior path-level evidence unless an
exact-image rerun is explicitly identified.

Start with [Architecture](getting-started/architecture.md), then run the
[Quick Start](getting-started/quickstart.md). Exact toolchains, measurements,
and build commands are in the repository's
[`BUILD.md`](https://github.com/faronov/zigbee-rs/blob/experiment/r22-bdb-complete/BUILD.md).
