#!/usr/bin/env python3
"""LAB ONLY: classify XT-ZB1 loader power-cut soak resets from the UART.

Reads the UART of a board running the `bl702-ota-soak` image
(examples/bl702-sensor/src/bin/ota_soak.rs) from a serial port, or replays
a captured log, and decides for every loader banner why the previous run
ended:

  cycle        the soak re-armed itself and reset (no cut)
  cut-copy     power cut while the loader erased or copied the image
  cut-install  cut after the copy, before the application started
  cut-loader   cut while the loader validated the slots
  cut-app      cut in the application before it started re-staging
  cut-staging  cut while the application re-staged the secondary slot
  watchdog     the loader watchdog fired (a failure in this test)

It fails on any loader error, `soak: FAIL`, a changed loader or security
journal CRC, a watchdog reset, or a stall: no UART output for
`--stall` seconds while powered. A stall means a possible brick; stop
cutting and investigate.

    tools/bl702_soak_monitor.py --port /dev/cu.usbserial-1430 --log soak.log
    tools/bl702_soak_monitor.py --replay soak.log
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter
from dataclasses import dataclass, field

BANNER = "bl702-boot: v"
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
CUT_KINDS = ("cut-copy", "cut-install", "cut-loader", "cut-app", "cut-staging")


@dataclass
class Monitor:
    """Feed it `(seconds, line)`; it keeps the counts and failures."""

    counts: Counter = field(default_factory=Counter)
    failures: list[str] = field(default_factory=list)
    copy_seconds: list[float] = field(default_factory=list)
    crcs: str | None = None
    phase: str | None = None
    copy_start: float | None = None
    last_banner: str = ""

    def feed(self, now: float, line: str) -> str | None:
        """Process one line; returns the reset classification at a banner."""
        line = line.strip("\r\n\x00 ")
        if not line:
            return None
        verdict = None
        if BANNER in line:
            verdict = self._reset()
            self.last_banner = line[line.index(BANNER):]
            self.phase = "loader"
        if "watchdog reset" in line:
            self.counts["watchdog"] += 1
            self.failures.append(f"loader watchdog reset after {self.last_banner}")
        for marker in FAILURES:
            if marker in line:
                self.failures.append(line)
        if "installing update" in line:
            self.phase = "copy"
            self.copy_start = now
        if "update installed" in line:
            self.phase = "install"
            if self.copy_start is not None:
                self.copy_seconds.append(now - self.copy_start)
        if "bl702-boot: app " in line:
            self.phase = "app"
        if "soak: crc " in line:
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

    def _reset(self) -> str | None:
        if self.phase is None:
            return None
        kind = {
            "re-armed": "cycle",
            "copy": "cut-copy",
            "install": "cut-install",
            "loader": "cut-loader",
            "app": "cut-app",
            "staging": "cut-staging",
        }[self.phase]
        self.counts[kind] += 1
        self.copy_start = None
        return kind

    def cuts(self) -> int:
        return sum(self.counts[kind] for kind in CUT_KINDS)

    def summary(self) -> str:
        parts = [f"{kind} {self.counts[kind]}" for kind in ("cycle", *CUT_KINDS)]
        text = f"cuts {self.cuts()}: " + ", ".join(parts)
        if self.copy_seconds:
            text += (
                f"; copy {min(self.copy_seconds):.1f}..{max(self.copy_seconds):.1f} s"
            )
        if self.crcs:
            text += f"; {self.crcs}"
        return text


def replay(path: str) -> Monitor:
    monitor = Monitor()
    with open(path, encoding="utf-8", errors="replace") as log:
        for number, line in enumerate(log):
            monitor.feed(float(number), line)
    return monitor


def watch(port: str, baud: int, log_path: str | None, stall: float) -> Monitor:
    import serial  # pyserial

    monitor = Monitor()
    log = open(log_path, "a", encoding="utf-8") if log_path else None
    last = time.monotonic()
    warned = False
    try:
        with serial.Serial(port, baud, timeout=1) as uart:
            while True:
                raw = uart.readline()
                now = time.monotonic()
                if not raw:
                    if now - last > stall and not warned:
                        print(f"\n!!! STALL: no output for {stall:.0f} s; "
                              "possible brick, stop cutting", flush=True)
                        warned = True
                    continue
                last, warned = now, False
                line = raw.decode("utf-8", errors="replace")
                if log:
                    log.write(f"{time.time():.3f} {line.rstrip()}\n")
                    log.flush()
                verdict = monitor.feed(now, line)
                if verdict:
                    print(f"[{monitor.cuts():3d}] {verdict:<12} {monitor.summary()}",
                          flush=True)
                if monitor.failures:
                    print(f"!!! FAILURE: {monitor.failures[-1]}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
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
    parser.add_argument("--min-cuts", type=int, default=50)
    args = parser.parse_args()

    if args.replay:
        monitor = replay(args.replay)
    else:
        monitor = watch(args.port, args.baud, args.log, args.stall)
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
