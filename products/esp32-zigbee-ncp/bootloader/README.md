# NCP second-stage bootloader (A/B rollback)

ESP-IDF **v5.5.1** bootloader built with
`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` for the NCP partition tables
(`../partitions/esp32h2-ncp-{4mb,2mb}.csv`, partition table at `0x8000`).
Decision N6 (amended 2026-11-18): NCP firmware rollback lives on the H2 —
two application slots, no NCP images on the host.

Rollback protocol (spec 7.9, 8.9):

1. `FW_*` stages the inactive slot; `FW_ACTIVATE` writes an `otadata`
   entry `NEW` for it and the NCP reboots.
2. The bootloader turns `NEW` into `PENDING_VERIFY` and boots the new image
   once. The image marks the attempt and starts the 10-minute trial window.
3. `FW_CONFIRM` from the host writes `VALID`.
4. Any reset before that — a crash before `main`, the watchdog, a host
   `SYS_RESET`, the window expiring (the image rolls back itself) — makes the
   bootloader mark the entry `ABORTED` and boot the previous slot.

The application-side checks (`src/fw.rs` `check_boot`/`roll_back`) also
work with the espflash-bundled bootloader (no rollback); this bootloader adds
the case where the new image never reaches its own boot check.
`src/fw_tests.rs` models the v5.5.1 selection step
(`idf_rollback_bootloader`) and checks the transitions.

| File | Chip | Flash | SHA-256 |
|---|---|---|---|
| `bin/bootloader-esp32h2-4mb.bin` | ESP32-H2 (min rev v0.0) | 4 MB, DIO 48 MHz | see `bin/SHA256SUMS` |
| `bin/bootloader-esp32h2-2mb.bin` | ESP32-H2 | 2 MB, DIO 48 MHz | |
| `bin/bootloader-esp32c6-4mb.bin` | ESP32-C6 (lab) | 4 MB, DIO 80 MHz | |

Flash at offset `0x0` (H2/C6). Rebuild: `IDF_PATH=~/esp/esp-idf-v5.5.1
./build.sh` (the `.config` files list the relevant resolved Kconfig values).
Builds are not bit-reproducible across machines (paths/timestamps); the
committed binaries are the ones tested.

Validation: host-modelled only. Hardware gate: activate an image on the
Thread BR H2, reset without `FW_CONFIRM`, and observe the previous slot boot
with the network retained (H12).
