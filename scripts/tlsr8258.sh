#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
action="${1:-}"
mode="${2:-sensor}"

usage() {
    cat >&2 <<'EOF'
usage: scripts/tlsr8258.sh <check|build|flash> [sensor|sensor-retention|sensor-retention-10s|router|diag-assoc|diag-beacon|diag-smoke|diag-pm|diag-retention|lab-sensor]

Production firmware:
  sensor       polling end-device sensor with hardware AES
  sensor-retention       feature-gated 250 ms LOW32K proof
  sensor-retention-10s   feature-gated 10 s steady-state LOW32K proof
  router       always-on join/relay router with hardware AES

Hardware lab:
  diag-assoc   association and MAC stress
  diag-beacon  raw beacon-request/RX diagnostics
  diag-smoke   startup and radio smoke test
  diag-pm      full-SRAM SUSPEND power-management test
  diag-retention reset-on-wake LOW32K restoration test
  lab-sensor   legacy manual-stack sensor regression
EOF
    exit 2
}

[[ -n "$action" ]] || usage

case "$action:$mode" in
    check:sensor|build:sensor|flash:sensor|\
    check:runtime-sensor|build:runtime-sensor|flash:runtime-sensor)
        exec "$ROOT_DIR/tools/tlsr8258-firmware.sh" "$action" \
            examples/telink-tlsr8258-sensor telink-tlsr8258-sensor
        ;;
    check:sensor-retention|build:sensor-retention|flash:sensor-retention)
        exec "$ROOT_DIR/tools/tlsr8258-firmware.sh" "$action" \
            examples/telink-tlsr8258-sensor telink-tlsr8258-sensor retention-proof
        ;;
    check:sensor-retention-10s|build:sensor-retention-10s|flash:sensor-retention-10s)
        exec "$ROOT_DIR/tools/tlsr8258-firmware.sh" "$action" \
            examples/telink-tlsr8258-sensor telink-tlsr8258-sensor retention-proof-10s
        ;;
    check:router|build:router|flash:router|\
    check:runtime-router|build:runtime-router|flash:runtime-router)
        exec "$ROOT_DIR/tools/tlsr8258-firmware.sh" "$action" \
            examples/telink-tlsr8258-router telink-tlsr8258-router
        ;;
    check:lab-sensor|build:lab-sensor|flash:lab-sensor)
        exec "$ROOT_DIR/tools/telink-tlsr8258-lab/scripts/tlsr8258.sh" "$action" sensor
        ;;
    check:diag-assoc|build:diag-assoc|flash:diag-assoc|\
    check:diag-beacon|build:diag-beacon|flash:diag-beacon|\
    check:diag-smoke|build:diag-smoke|flash:diag-smoke|\
    check:diag-pm|build:diag-pm|flash:diag-pm|\
    check:diag-retention|build:diag-retention|flash:diag-retention)
        exec "$ROOT_DIR/tools/telink-tlsr8258-lab/scripts/tlsr8258.sh" "$action" "$mode"
        ;;
    *)
        # Preserve the debugger/dump utilities owned by the hardware lab.
        exec "$ROOT_DIR/tools/telink-tlsr8258-lab/scripts/tlsr8258.sh" "$@"
        ;;
esac
