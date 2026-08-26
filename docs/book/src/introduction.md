# Introduction

`zigbee-rs` is a heap-free, `no_std`, pure-Rust Zigbee PRO stack with reusable
sleepy-sensor, router, and coordinator application frontends.

This book describes the current `experiment/zephyr-app-model` worktree. The
book source is authoritative for the branch. GitHub Pages deploys only after a
push to `main` or `master`, so this revision is not public on Pages until the
branch is merged and deployed.

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
platform/chip HAL + MacDriver
        ↓
board resources and fitted wiring
        ↓
product identity/profile/policy/storage/linker/OTA
        ↓
short composition root
```

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

- **compiles** — target code builds;
- **host-tested** — portable behavior passed host tests;
- **hardware-tested** — a named primitive or path ran on silicon;
- **production path proven** — the complete product flow passed its stated
  hardware acceptance.

A release image that reaches a sleep instruction is not proof of current
consumption or correct wake restoration. A compiled flash journal is not proof
of power-loss safety on that controller. Platform guides state the remaining
hardware gates explicitly.

Start with [Architecture](getting-started/architecture.md), then run the
[Quick Start](getting-started/quickstart.md). Exact toolchains, measurements,
and build commands are in the repository's
[`BUILD.md`](https://github.com/faronov/zigbee-rs/blob/experiment/zephyr-app-model/BUILD.md).
