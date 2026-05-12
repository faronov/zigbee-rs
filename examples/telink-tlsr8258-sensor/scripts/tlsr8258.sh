#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
EXAMPLE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

DEFAULT_TC32_TOOLCHAIN="/tmp/tc32-rust-toolchain-macos-amd64"
TC32_TOOLCHAIN="${TC32_TOOLCHAIN:-$DEFAULT_TC32_TOOLCHAIN}"
CARGO_BIN="${CARGO_BIN:-$TC32_TOOLCHAIN/bin/cargo}"
LLVM_OBJCOPY="${LLVM_OBJCOPY:-$TC32_TOOLCHAIN/llvm/bin/llvm-objcopy}"

DEFAULT_CARGO_HOME="${HOME}/.cargo"
CARGO_HOME="${CARGO_HOME:-$DEFAULT_CARGO_HOME}"

TLSRPGM="${TLSRPGM:-$HOME/TLSRPGM/TlsrPgm.py}"
TLSR_DEBUG="${TLSR_DEBUG:-$HOME/zboss_opensource/tlsr_debug.py}"
TELINK_PORT="${TELINK_PORT:-/dev/cu.usbserial-1410}"

TARGET_DIR="${EXAMPLE_DIR}/target/tc32-unknown-none-elf/release"
ELF_PATH="${TARGET_DIR}/telink-tlsr8258-sensor"
BIN_PATH="${TARGET_DIR}/telink-tlsr8258-sensor.bin"

DBG_BOOT_BASE="0x00848400"
DBG_MODE_BASE="0x00848500"

MODE_NAME="sensor"
declare -a CARGO_FEATURE_ARGS=()

usage() {
    cat <<'EOF'
Usage:
  scripts/tlsr8258.sh check [sensor|diag-assoc|diag-beacon]
  scripts/tlsr8258.sh build [sensor|diag-assoc|diag-beacon]
  scripts/tlsr8258.sh flash [sensor|diag-assoc|diag-beacon]
  scripts/tlsr8258.sh dump-boot [word-count]
  scripts/tlsr8258.sh dump-mode [word-count]
  scripts/tlsr8258.sh dump <address> [word-count]

Environment overrides:
  TC32_TOOLCHAIN  Path to tc32-stage2 toolchain root
  CARGO_BIN       Cargo binary to use instead of $TC32_TOOLCHAIN/bin/cargo
  LLVM_OBJCOPY    llvm-objcopy binary to emit the flashable .bin
  CARGO_HOME      Cargo home for registry/cache
  TLSRPGM         Path to TlsrPgm.py
  TLSR_DEBUG      Path to tlsr_debug.py
  TELINK_PORT     Serial device used by both flasher and debugger
EOF
}

require_file() {
    local path="$1"
    local name="$2"
    if [[ ! -e "$path" ]]; then
        echo "Missing ${name}: ${path}" >&2
        exit 1
    fi
}

resolve_mode() {
    local mode="${1:-sensor}"
    case "$mode" in
        sensor)
            MODE_NAME="sensor"
            CARGO_FEATURE_ARGS=()
            ;;
        diag-assoc)
            MODE_NAME="diag-assoc"
            CARGO_FEATURE_ARGS=(--no-default-features --features diag-assoc)
            ;;
        diag-beacon)
            MODE_NAME="diag-beacon"
            CARGO_FEATURE_ARGS=(--no-default-features --features diag-beacon)
            ;;
        *)
            echo "Unsupported mode: ${mode}" >&2
            usage >&2
            exit 1
            ;;
    esac
}

run_cargo() {
    local cargo_subcommand="$1"
    shift
    require_file "$CARGO_BIN" "tc32 cargo"
    (
        local -a cargo_args=("$cargo_subcommand" --release)
        if (( ${#CARGO_FEATURE_ARGS[@]} > 0 )); then
            cargo_args+=("${CARGO_FEATURE_ARGS[@]}")
        fi
        cargo_args+=("$@")
        cd "$EXAMPLE_DIR"
        env CARGO_HOME="$CARGO_HOME" "$CARGO_BIN" "${cargo_args[@]}"
    )
}

emit_bin() {
    require_file "$LLVM_OBJCOPY" "llvm-objcopy"
    require_file "$ELF_PATH" "ELF image"
    "$LLVM_OBJCOPY" -O binary "$ELF_PATH" "$BIN_PATH"
}

cmd_check() {
    resolve_mode "${1:-sensor}"
    run_cargo check
}

cmd_build() {
    resolve_mode "${1:-sensor}"
    if ! run_cargo rustc -- -C lto=no -C opt-level=1; then
        cat >&2 <<'EOF'
tc32-stage2-tc32-31 is currently unstable for optimized TLSR8258 release builds.
The validated workaround for this example is:
  cargo rustc --release -- -C lto=no -C opt-level=1
EOF
        exit 1
    fi
    emit_bin
    echo "Mode: ${MODE_NAME}"
    echo "ELF:  ${ELF_PATH}"
    echo "BIN:  ${BIN_PATH}"
}

cmd_flash() {
    resolve_mode "${1:-sensor}"
    cmd_build "$MODE_NAME"
    require_file "$TLSRPGM" "TlsrPgm.py"
    python3 "$TLSRPGM" -p "$TELINK_PORT" -t 500 -a 200 -m we 0 "$BIN_PATH"
}

cmd_dump() {
    local address="${1:?dump requires an address}"
    local words="${2:-16}"
    require_file "$TLSR_DEBUG" "tlsr_debug.py"
    python3 "$TLSR_DEBUG" -p "$TELINK_PORT" --activate read "$address" "$words"
}

cmd_dump_boot() {
    cmd_dump "$DBG_BOOT_BASE" "${1:-16}"
}

cmd_dump_mode() {
    cmd_dump "$DBG_MODE_BASE" "${1:-20}"
}

main() {
    local command="${1:-}"
    shift || true

    case "$command" in
        check) cmd_check "$@" ;;
        build) cmd_build "$@" ;;
        flash) cmd_flash "$@" ;;
        dump) cmd_dump "$@" ;;
        dump-boot) cmd_dump_boot "$@" ;;
        dump-mode) cmd_dump_mode "$@" ;;
        -h|--help|help|"") usage ;;
        *)
            echo "Unsupported command: ${command}" >&2
            usage >&2
            exit 1
            ;;
    esac
}

main "$@"
