#!/usr/bin/env python3
"""Assemble documentation, publishing firmware only when every required job passes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import shutil


CHIPS = ("ESP32-C6", "ESP32-H2")
RESULTS = ("success", "failure", "cancelled", "skipped")


def replace_slot(template: str, name: str, content: str) -> str:
    start = f"<!-- {name}_START -->"
    end = f"<!-- {name}_END -->"
    if template.count(start) != 1 or template.count(end) != 1:
        raise ValueError(f"expected one {name} template slot")
    before, rest = template.split(start)
    _, after = rest.split(end)
    return before + start + "\n" + content + "\n" + end + after


def prepare(
    book: Path,
    flasher: Path,
    firmware: Path,
    output: Path,
    revision: str,
    run_id: int,
    results: dict[str, str],
) -> None:
    if re.fullmatch(r"[0-9a-f]{40}", revision) is None or run_id <= 0:
        raise ValueError("expected a full commit SHA and positive workflow run ID")
    if set(results) != {"esp32c6", "esp32h2", "bl702"}:
        raise ValueError("all three firmware job results are required")
    if any(result not in RESULTS for result in results.values()):
        raise ValueError("unknown firmware job result")
    if output.exists():
        raise ValueError("refusing to reuse a site directory with potentially stale firmware")
    if not (book / "index.html").is_file() or (book / "flasher").exists():
        raise ValueError("expected a built book without the reserved flasher directory")

    available = all(result == "success" for result in results.values())
    files = [f"{chip.lower().replace('-', '')}-sensor.bin" for chip in CHIPS]
    if available:
        for name in files:
            if not (firmware / name).is_file() or (firmware / name).stat().st_size == 0:
                raise ValueError(f"qualified firmware artifact is missing or empty: {name}")

    run_url = f"https://github.com/faronov/zigbee-rs/actions/runs/{run_id}"
    status = (
        "Prebuilt firmware passed the required CI checks."
        if available else
        "Prebuilt firmware is unavailable: required firmware checks did not all pass."
    )
    status += (
        f' Revision <code>{revision[:7]}</code>. '
        f'<a href="{run_url}">View CI results</a>. '
        "Build checks do not replace hardware qualification."
    )
    button = (
        '<esp-web-install-button manifest="./manifest.json">'
        '<button slot="activate">Install Zigbee Sensor</button>'
        "</esp-web-install-button>"
        if available else
        '<button disabled>Prebuilt firmware unavailable</button>'
    )
    template = replace_slot(flasher.read_text(), "PREBUILT_INSTALL", button)
    template = replace_slot(template, "PREBUILT_STATUS", status)
    manifest = {
        "name": "Zigbee-RS Sensor Firmware",
        "version": revision,
        "builds": [
            {"chipFamily": chip, "parts": [{"path": f"./firmware/{name}", "offset": 0}]}
            for chip, name in zip(CHIPS, files)
        ] if available else [],
    }
    metadata = {
        "revision": revision,
        "workflow_run": run_url,
        "firmware_available": available,
        "firmware_jobs": results,
    }
    shutil.copytree(book, output)
    destination = output / "flasher"
    destination.mkdir()
    (destination / "index.html").write_text(template)
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (destination / "status.json").write_text(json.dumps(metadata, indent=2) + "\n")
    if available:
        (destination / "firmware").mkdir()
        for name in files:
            shutil.copyfile(firmware / name, destination / "firmware" / name)
    print(f"Site assembled: revision={revision}, firmware_available={available}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("book", "flasher", "firmware", "output"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    parser.add_argument("--revision", required=True)
    parser.add_argument("--run-id", type=int, required=True)
    for chip in ("esp32c6", "esp32h2", "bl702"):
        parser.add_argument(f"--{chip}-result", choices=RESULTS, required=True)
    args = parser.parse_args()
    try:
        prepare(
            args.book, args.flasher, args.firmware, args.output,
            args.revision, args.run_id,
            {chip: getattr(args, f"{chip}_result") for chip in ("esp32c6", "esp32h2", "bl702")},
        )
    except (OSError, ValueError) as error:
        parser.exit(1, f"Cannot assemble Pages site: {error}\n")


if __name__ == "__main__":
    main()
