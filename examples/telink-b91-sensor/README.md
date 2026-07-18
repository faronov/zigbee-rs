# Telink B91 scaffold

This directory is retained as a hardware/application scaffold only. The
repository does **not** currently contain a Telink B91 `RadioPhy` or
`MacDriver` implementation, and this package is intentionally excluded from
the firmware CI matrix.

The old example incorrectly selected the TLSR8258 backend through the
ambiguous `zigbee-mac/telink` feature and claimed that B91 FFI support existed.
That backend is TLSR8258-specific and cannot run on the B91 RISC-V target.

Before this example can become buildable, it needs:

1. A B91 radio HAL proven against the official Telink SDK register and IRQ
   behavior.
2. A `RadioPhy` plus mandatory platform time, delay, and secure-entropy
   services.
3. Host conformance tests and B91 hardware evidence for scan, association,
   polling, ACK timing, security, persistence, and sleep.

Use `examples/telink-tlsr8258-sensor` for the currently supported pure-Rust
Telink implementation.
