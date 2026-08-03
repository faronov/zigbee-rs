#!/usr/bin/env bash
set -euo pipefail

BFLB_MCU_TOOL="${BFLB_MCU_TOOL:-bflb-mcu-tool}"

TOOL_PATH="$(command -v "$BFLB_MCU_TOOL" || true)"
if [[ -z "$TOOL_PATH" ]]; then
    echo "bflb-mcu-tool not found; install version 1.10.0 with pip" >&2
    exit 1
fi

SHEBANG="$(head -n 1 "$TOOL_PATH")"
if [[ "$SHEBANG" == "#!"* ]]; then
    read -r -a INTERPRETER <<<"${SHEBANG#\#!}"
    if [[ "${INTERPRETER[0]##*/}" == "env" ]]; then
        PYTHON="$(command -v "${INTERPRETER[1]}" || true)"
    else
        PYTHON="${INTERPRETER[0]}"
    fi
else
    PYTHON="${PYTHON:-python3}"
fi

if [[ -z "${PYTHON:-}" || ! -x "$PYTHON" ]]; then
    echo "cannot resolve the Python interpreter used by $TOOL_PATH" >&2
    exit 1
fi

TOOL_INFO="$("$PYTHON" - <<'PY'
import importlib.metadata
import os

import bflb_mcu_tool

print(
    importlib.metadata.version("bflb-mcu-tool"),
    os.path.dirname(bflb_mcu_tool.__file__),
    sep="\t",
)
PY
)"
IFS=$'\t' read -r TOOL_VERSION PACKAGE_ROOT <<<"$TOOL_INFO"

if [[ "$TOOL_VERSION" != "1.10.0" ]]; then
    echo "bflb-mcu-tool 1.10.0 is required, found ${TOOL_VERSION:-unknown}" >&2
    exit 1
fi

if [[ ! -d "$PACKAGE_ROOT/chips/bl702" ]]; then
    echo "BL702 support files are missing from $PACKAGE_ROOT" >&2
    exit 1
fi

# bflb-mcu-tool writes generated settings back into its installed package.
# In version 1.10.0, --pllclk=Manual does not restore pll_clk/bclk_div after
# an earlier 144 MHz build, so a reused installation can silently emit the
# wrong boot clock. Run from an isolated copy with all mutable INI files
# removed; the tool then recreates them from its pristine .conf templates.
ISOLATED_ROOT="$(mktemp -d -t bl702-bflb-tool.XXXXXX)"
trap 'rm -rf "$ISOLATED_ROOT"' EXIT

mkdir -p "$ISOLATED_ROOT/bflb_mcu_tool/chips"
cp "$PACKAGE_ROOT"/*.py "$ISOLATED_ROOT/bflb_mcu_tool/"
for directory in core libs utils; do
    cp -R "$PACKAGE_ROOT/$directory" "$ISOLATED_ROOT/bflb_mcu_tool/"
done
cp -R "$PACKAGE_ROOT/chips/bl702" "$ISOLATED_ROOT/bflb_mcu_tool/chips/"

rm -f \
    "$ISOLATED_ROOT/bflb_mcu_tool/chips/bl702/eflash_loader/eflash_loader_cfg.ini" \
    "$ISOLATED_ROOT/bflb_mcu_tool/chips/bl702/img_create_mcu/efuse_bootheader_cfg.ini" \
    "$ISOLATED_ROOT/bflb_mcu_tool/chips/bl702/img_create_mcu/img_create_cfg.ini"

PYTHONPATH="$ISOLATED_ROOT${PYTHONPATH:+:$PYTHONPATH}" \
    "$PYTHON" -m bflb_mcu_tool "$@"
