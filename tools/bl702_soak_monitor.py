#!/usr/bin/env python3
"""LAB ONLY: classify XT-ZB1 loader power-cut soak resets from the UART.

Reads the UART of a board running the `bl702-ota-soak` image
(examples/bl702-sensor/src/bin/ota_soak.rs) from a serial port, or replays
a captured log.

On the XT-ZB1 bench the board is powered through its USB-UART adapter, so
a power cut also removes the serial port. The monitor treats the port
disappearing as the cut (never as silence), waits for it to re-enumerate
and reopens it with DTR and RTS released. It writes `~ port lost` and
`~ port back` markers into the log, so a replay classifies the same way.

Each cut is classified by the last thing the board printed before it:

  cut-boot     after power-up, before any line was seen
  cut-loader   while the loader validated the slots
  cut-copy     while the loader erased or copied the image
  cut-install  after the copy, before the application started
  cut-app      in the application, before it started re-staging
  cut-staging  while the application re-staged the secondary slot
  cut-rearmed  after the request, before the reset

A loader banner without a cut is a `cycle` after `soak: re-armed`, else an
unexpected `reset-<phase>`; reopening the port may cause one, so these are
reported, not failures.

The run fails on any loader error, `soak: FAIL`, a changed loader or
security journal CRC, a loader watchdog reset, no soak application line
within `--recover` seconds after the port returns, or no output for
`--stall` seconds while the port is present (a possible brick: stop
cutting and investigate).

    tools/bl702_soak_monitor.py --port /dev/cu.usbserial-1430 --log soak.log
    tools/bl702_soak_monitor.py --replay soak.log
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from collections import Counter
from dataclasses import dataclass, field

BANNER = "bl702-boot: v"
PORT_LOST = "~ port lost"
PORT_BACK = "~ port back"
FAILURES = (
    "soak: FAIL",
    "no bootable image",
    "installed copy corrupt",
    "flash error",
    "flash config error",
    "status log full",
    "bad configuration",
    "update rejected",
    "halted",
)
CUT_KINDS = (
    "cut-boot", "cut-loader", "cut-copy", "cut-install", "cut-app",
    "cut-staging", "cut-rearmed",
)


@dataclass
class Monitor:
    """Feed it `(seconds, line)`; it keeps the counts and failures."""

    recover: float = 60.0
    counts: Counter = field(default_factory=Counter)
    failures: list[str] = field(default_factory=list)
    copy_seconds: list[float] = field(default_factory=list)
    crcs: str | None = None
    armed: bool = False
    phase: str | None = None
    copy_start: float | None = None
    deadline: float | None = None
    last_banner: str = ""

    def feed(self, now: float, line: str) -> str | None:
        """Process one line; returns a cut or reset classification."""
        line = line.strip("\r\n\x00 ")
        if not line:
            return None
        if line.startswith(PORT_LOST):
            return self._cut()
        if line.startswith(PORT_BACK):
            self.phase = "boot"
            self.copy_start = None
            if self.armed:
                self.deadline = now + self.recover
            return None
        verdict = None
        if BANNER in line:
            verdict = self._reset()
            self.last_banner = line[line.index(BANNER):]
            self.phase = "loader"
            self.copy_start = None
        if "watchdog reset" in line:
            self.counts["watchdog"] += 1
            self.failures.append(f"loader watchdog reset after {self.last_banner}")
        for marker in FAILURES:
            if marker in line:
                self.failures.append(line)
        if "installing update" in line:
            self.phase = "copy"
            self.copy_start = now
        elif set(line) == {"."} and self.phase in ("boot", "loader"):
            # Copy progress seen without its start (port reopened mid-copy).
            self.phase = "copy"
        if "update installed" in line:
            self.phase = "install"
            if self.copy_start is not None:
                self.copy_seconds.append(now - self.copy_start)
        if "bl702-boot: app " in line:
            self.phase = "app"
        if "soak: " in line and self.phase in ("boot", "loader", "install"):
            self.phase = "app"
        if "soak: crc " in line:
            self.armed = True
            self.deadline = None
            crcs = line[line.index("soak: crc ") + 10:]
            if self.crcs is None:
                self.crcs = crcs
            elif crcs != self.crcs:
                self.failures.append(f"CRC changed: {self.crcs} -> {crcs}")
        if "soak: re-staging" in line:
            self.phase = "staging"
        if "soak: re-armed" in line:
            self.phase = "re-armed"
        return verdict

    def check(self, now: float) -> None:
        """Fail if the soak application did not return after a power-up."""
        if self.deadline is not None and now > self.deadline:
            self.failures.append(
                f"no soak application within {self.recover:.0f} s after power-up "
                f"(last phase {self.phase})")
            self.deadline = None

    def _cut(self) -> str | None:
        phase, self.phase, self.deadline = self.phase, "off", None
        if not self.armed or phase in (None, "off"):
            return None
        kind = "cut-rearmed" if phase == "re-armed" else f"cut-{phase}"
        self.counts[kind] += 1
        return kind

    def _reset(self) -> str | None:
        if not self.armed or self.phase in (None, "off", "boot"):
            return None
        kind = "cycle" if self.phase == "re-armed" else f"reset-{self.phase}"
        self.counts[kind] += 1
        return kind

    def cuts(self) -> int:
        return sum(self.counts[kind] for kind in CUT_KINDS)

    def summary(self) -> str:
        parts = [f"{kind[4:]} {self.counts[kind]}" for kind in CUT_KINDS]
        text = f"cuts {self.cuts()} ({', '.join(parts)}); cycles {self.counts['cycle']}"
        resets = sum(n for kind, n in self.counts.items() if kind.startswith("reset-"))
        if resets:
            text += f"; other resets {resets}"
        if self.copy_seconds:
            text += f"; copy {min(self.copy_seconds):.1f}..{max(self.copy_seconds):.1f} s"
        if self.crcs:
            text += f"; {self.crcs}"
        return text


def replay(path: str) -> Monitor:
    monitor = Monitor()
    with open(path, encoding="utf-8", errors="replace") as log:
        for number, line in enumerate(log):
            # Logs written by `watch` start with a wall-clock timestamp.
            stamp, _, rest = line.partition(" ")
            try:
                now, line = float(stamp), rest
            except ValueError:
                now = float(number)
            monitor.feed(now, line)
    return monitor


def open_port(port: str, baud: int):
    import serial  # pyserial

    uart = serial.Serial()
    uart.port, uart.baudrate, uart.timeout = port, baud, 0.2
    # Released before opening; RTS drives the XT-ZB1 reset.
    uart.dtr = False
    uart.rts = False
    uart.open()
    return uart


def watch(port: str, baud: int, log_path: str | None, stall: float,
          recover: float, stop_at: int) -> Monitor:
    import serial  # pyserial

    monitor = Monitor(recover=recover)
    log = open(log_path, "a", encoding="utf-8") if log_path else None

    def record(now: float, line: str) -> None:
        if log:
            log.write(f"{time.time():.3f} {line.rstrip()}\n")
            log.flush()
        failures = len(monitor.failures)
        verdict = monitor.feed(now, line)
        if verdict:
            print(f"[{monitor.cuts():3d}] {verdict:<12} {monitor.summary()}", flush=True)
        for failure in monitor.failures[failures:]:
            print(f"!!! FAILURE: {failure}", flush=True)

    uart = None
    last = time.monotonic()
    warned = False
    try:
        while True:
            now = time.monotonic()
            if uart is None:
                if not os.path.exists(port):
                    time.sleep(0.05)
                    continue
                try:
                    uart = open_port(port, baud)
                except (OSError, serial.SerialException):
                    time.sleep(0.05)
                    continue
                last, warned = time.monotonic(), False
                record(last, PORT_BACK)
                print("~ port back", flush=True)
                continue
            try:
                raw = uart.readline()
            except (OSError, serial.SerialException):
                uart.close()
                uart = None
                record(time.monotonic(), PORT_LOST)
                print(f"~ port lost (power cut)  {monitor.summary()}", flush=True)
                continue
            now = time.monotonic()
            failures = len(monitor.failures)
            monitor.check(now)
            for failure in monitor.failures[failures:]:
                print(f"!!! FAILURE: {failure}", flush=True)
            if not raw:
                if now - last > stall and not warned:
                    print(f"!!! STALL: port present but no output for {stall:.0f} s; "
                          "possible brick, stop cutting", flush=True)
                    monitor.failures.append(f"no output for {stall:.0f} s while powered")
                    warned = True
                continue
            last, warned = now, False
            record(now, raw.decode("utf-8", errors="replace"))
            if stop_at and monitor.cuts() >= stop_at and monitor.phase == "app":
                print(f"=== {monitor.cuts()} cuts reached and the soak is running: "
                      "tell the user to STOP", flush=True)
                stop_at = 0
    except KeyboardInterrupt:
        pass
    finally:
        if uart is not None:
            uart.close()
        if log:
            log.close()
    return monitor


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="UART device of the XT-ZB1 (GPIO14 TX)")
    source.add_argument("--replay", help="classify a captured log instead")
    parser.add_argument("--baud", type=int, default=2_000_000)
    parser.add_argument("--log", help="append the timestamped UART log here")
    parser.add_argument("--stall", type=float, default=30.0)
    parser.add_argument("--recover", type=float, default=60.0)
    parser.add_argument("--min-cuts", type=int, default=50)
    args = parser.parse_args()

    if args.replay:
        monitor = replay(args.replay)
    else:
        monitor = watch(args.port, args.baud, args.log, args.stall, args.recover,
                        args.min_cuts)
    print(monitor.summary())
    for failure in monitor.failures:
        print(f"FAILURE: {failure}")
    if monitor.failures:
        return 1
    if monitor.cuts() < args.min_cuts or monitor.counts["cut-copy"] == 0:
        print(f"not enough cuts: need {args.min_cuts}, including some during the copy")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
