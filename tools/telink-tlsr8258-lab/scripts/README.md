# TLSR8258 lab helper

`tlsr8258.sh` owns the legacy laboratory build modes plus SWire flashing,
memory dumps, and experimental tc32 `probe-rs` commands.

From the repository root, prefer the public wrapper:

```sh
./scripts/tlsr8258.sh build diag-beacon
./scripts/tlsr8258.sh flash diag-assoc
./scripts/tlsr8258.sh build diag-pm
./scripts/tlsr8258.sh build lab-sensor
```

The helper uses tc32-45 from `.toolchains/tc32-stage2-tc32-45` by default.
Override `TC32_TOOLCHAIN`, `TELINK_PORT`, `TLSRPGM`, `TLSR_DEBUG`, or
`PROBE_RS` when the local tools use different paths.

Live SRAM diagnostics are still available directly:

```sh
tools/telink-tlsr8258-lab/scripts/tlsr8258.sh dump-boot 32
tools/telink-tlsr8258-lab/scripts/tlsr8258.sh dump-mode 64
tools/telink-tlsr8258-lab/scripts/tlsr8258.sh probe-debug diag-assoc
```

These commands belong to the lab and are not part of the production sensor or
router workflow.
