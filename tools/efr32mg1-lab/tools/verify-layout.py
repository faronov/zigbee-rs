#!/usr/bin/env python3
"""Run the canonical EFR32MG1 bootloader-safe ELF layout verifier."""

import runpy
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
runpy.run_path(
    str(ROOT / "examples" / "efr32mg1-sensor" / "tools" / "verify-layout.py"),
    run_name="__main__",
)
