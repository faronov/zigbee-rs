# TLSR8258 helper script

`tlsr8258.sh` wraps the local tc32 Rust toolchain, Telink SWire flashing, SRAM
marker reads, and the experimental tc32-enabled `probe-rs` commands.

Run commands from the repository root:

```sh
examples/telink-tlsr8258-sensor/scripts/tlsr8258.sh build sensor
examples/telink-tlsr8258-sensor/scripts/tlsr8258.sh flash sensor
examples/telink-tlsr8258-sensor/scripts/tlsr8258.sh dump-mode 48
```

## Defaults

The script currently defaults to:

```sh
TC32_TOOLCHAIN=/tmp/tc32-stage2-tc32-42/extracted/tc32-stage2-x86_64-apple-darwin
TELINK_PORT=/dev/cu.usbserial-1410
TLSRPGM=$HOME/TLSRPGM/TlsrPgm.py
TLSR_DEBUG=$HOME/zboss_opensource/tlsr_debug.py
PROBE_RS=/tmp/probe-rs-tc32-25521051175/probe-rs
PROBE_RS_PROBE=sws:$TELINK_PORT
```

Override any of them in the environment if the toolchain or serial device lives
elsewhere.

## Modes

- `sensor`: default firmware. Scans channel 15, associates, announces, and polls.
- `diag-beacon`: minimal MAC/RF diagnostic. Sends beacon requests and reports if
  it receives a beacon.
- `diag-assoc`: association diagnostic path.

Examples:

```sh
examples/telink-tlsr8258-sensor/scripts/tlsr8258.sh flash diag-beacon
examples/telink-tlsr8258-sensor/scripts/tlsr8258.sh flash sensor
```

## SRAM markers

The firmware reserves a 1 KiB debug block at the top of SRAM:

```text
boot markers: 0x0084FC00
mode markers: 0x0084FD00
```

Use `dump-boot` and `dump-mode` for normal live debugging:

```sh
examples/telink-tlsr8258-sensor/scripts/tlsr8258.sh dump-boot 32
examples/telink-tlsr8258-sensor/scripts/tlsr8258.sh dump-mode 64
```

These commands intentionally do not pass `--activate` to `tlsr_debug.py`; they
read without resetting or stopping the CPU. Use `dump-activate` only when the
chip is already stuck and a non-invasive read cannot attach.

Useful marker values:

```text
0xD1A600B0  diag-beacon loop is running
0xBEAC0001  diag-beacon received and parsed a beacon
0x53E50000  sensor loop started
0x53E50001  sensor scan attempt started
0x53E50010  sensor found a beacon
0x53E5C000  association succeeded
0x53E50021  poll received a MAC frame
0x53E5BEEF  received frame was handled by the sensor path
```

## probe-rs

The custom tc32 `probe-rs` build supports Telink SWS selectors. The helper uses:

```sh
PROBE_RS_PROBE=sws:/dev/cu.usbserial-1410
```

RTT attach commands:

```sh
examples/telink-tlsr8258-sensor/scripts/tlsr8258.sh probe-list-rtt sensor
examples/telink-tlsr8258-sensor/scripts/tlsr8258.sh probe-attach sensor
```

Debugger commands:

```sh
examples/telink-tlsr8258-sensor/scripts/tlsr8258.sh probe-debug sensor
examples/telink-tlsr8258-sensor/scripts/tlsr8258.sh probe-gdb sensor
```

At the time this note was written, `TlsrPgm.py` worked on the local USB SWS
programmer, while `probe-rs attach --probe sws:/dev/cu.usbserial-1410` timed out
during target attach. The commands are kept in the helper so the same workflow
can be retried when the SWS attach sequence is fixed.
