# Mock sensor compatibility alias

`mock-sensor` now delegates to the finite
[`mock-sleepy-sensor`](../mock-sleepy-sensor/) demo. It no longer contains a
separate scan/associate/application implementation.

```bash
cd examples/mock-sensor
cargo +nightly-2026-03-23 run --locked
```

New host sensor work should use `mock-sleepy-sensor` and explicit
`SensorSedParts`.
