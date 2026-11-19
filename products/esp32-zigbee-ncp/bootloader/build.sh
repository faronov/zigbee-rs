#!/usr/bin/env bash
# Build the NCP bootloaders with ESP-IDF v5.5.1 (A/B rollback enabled).
# Usage: IDF_PATH=~/esp/esp-idf-v5.5.1 ./build.sh
# Needs the IDF v5.5.1 Python env and the esp-14.2.0_20241119 RISC-V GCC
# (`tools/idf_tools.py install riscv32-esp-elf`).
set -euo pipefail
: "${IDF_PATH:?set IDF_PATH to an ESP-IDF v5.5.1 checkout}"
if ! command -v idf.py >/dev/null; then
  IDF_TOOLS="${IDF_TOOLS_PATH:-$HOME/.espressif}"
  PY="$IDF_TOOLS/python_env/idf5.5_py3.11_env/bin/python"
  while IFS='=' read -r key value; do
    export "$key=${value//\$PATH/$PATH}"
  done < <(cd "$IDF_PATH" && "$PY" tools/idf_tools.py export --format key-value)
  export PATH="$IDF_TOOLS/tools/riscv32-esp-elf/esp-14.2.0_20241119/riscv32-esp-elf/bin:$PATH"
fi
[[ "$(cd "$IDF_PATH" && git describe --tags 2>/dev/null)" == v5.5.1 ]] ||
  { echo "ESP-IDF v5.5.1 required" >&2; exit 1; }
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/bin"
mkdir -p "$OUT"
for variant in esp32h2:4MB:4mb esp32h2:2MB:2mb esp32c6:4MB:4mb; do
  IFS=: read -r chip size csv <<<"$variant"
  work="$(mktemp -d)"
  cp -R "$HERE/project/." "$work/"
  cp "$HERE/../partitions/esp32h2-ncp-$csv.csv" "$work/partitions.csv"
  {
    cat "$HERE/project/sdkconfig.defaults"
    echo "CONFIG_PARTITION_TABLE_CUSTOM_FILENAME=\"partitions.csv\""
    echo "CONFIG_ESPTOOLPY_FLASHSIZE_${size}=y"
  } >"$work/sdkconfig.defaults"
  (cd "$work" && idf.py -DIDF_TARGET="$chip" bootloader >/dev/null)
  cp "$work/build/bootloader/bootloader.bin" "$OUT/bootloader-$chip-$csv.bin"
  grep -E "CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE|CONFIG_ESPTOOLPY_FLASHSIZE=|CONFIG_ESPTOOLPY_FLASHMODE=|CONFIG_ESPTOOLPY_FLASHFREQ=|CONFIG_ESP32[A-Z0-9]*_REV_MIN_FULL" \
    "$work/sdkconfig" >"$OUT/bootloader-$chip-$csv.config"
  rm -rf "$work"
done
(cd "$OUT" && shasum -a 256 ./*.bin >SHA256SUMS)
cat "$OUT/SHA256SUMS"
