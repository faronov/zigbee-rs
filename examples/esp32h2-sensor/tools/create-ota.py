#!/usr/bin/env python3
"""Run the shared ESP32 OTA packager for the ESP32-H2 sensor."""

from __future__ import annotations

import pathlib
import runpy
import sys

if not any(arg == "--chip" or arg.startswith("--chip=") for arg in sys.argv[1:]):
    sys.argv[1:1] = ["--chip", "esp32h2"]

runpy.run_path(
    str(pathlib.Path(__file__).resolve().parents[3] / "tools/create-esp32-ota.py"),
    run_name="__main__",
)
