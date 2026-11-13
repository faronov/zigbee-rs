# STM32WB FUS helper (SRAM shim)

A 2.7 KiB CPU1 program that runs from SRAM1 and performs one coprocessor
FUS step. It is the "Option B" path for an STM32WB55 board **without a
debug probe**, such as the P-NUCLEO-WB55 USB dongle (MB1293C). The ROM USB
DFU bootloader (ST AN3156) can write SRAM and jump to it, but it has no
FUS commands. hwlab's `stm32dfu` plugin loads this helper and then leaves
DFU into it.

It never erases or programs flash and never touches option bytes. Images
are staged below the secure start (SFSA) by the DFU plugin's
`flash-program`. Only the FUS writes the secure area.

## Build

```sh
cd tools/stm32wb-fus-shim
cargo build --release
arm-none-eabi-objcopy -O binary \
  target/thumbv7em-none-eabihf/release/stm32wb-fus-shim stm32wb-fus-shim.bin
```

The image is linked at `0x2000_8000`. Its vector table holds SP =
`0x2002_F000` and a Thumb reset vector. There is no `.data` or `.bss`; the
linker script asserts this.

## Host protocol

| Address | Direction | Content |
|---------|-----------|---------|
| `0x2000_8000` | host → device | the image |
| `0x2002_F000` | host → device | parameter block |
| `0x2002_F100` | device → host | result block, 0x200 bytes |

The **parameter block** is little-endian `u32` words:

| Offset | Field | Value |
|--------|-------|-------|
| 0x0 | magic | `0x50484D31` |
| 0x4 | op | see the table below |
| 0x8 | timeout | ms, clamped to 1 s .. 600 s |

The helper clears the magic before acting, so a stale block never repeats
a command.

| op | Name | Action |
|----|------|--------|
| 1 | INFO | Boot CPU2 and record the ready event and device info. Under the FUS it also sends one FUS_GET_STATE. |
| 2 | ENTER_FUS | If the wireless stack runs, send FUS_GET_STATE twice. The stack then reboots into the FUS, which is a system reset. |
| 3 | UPGRADE | From an idle FUS, send FUS_FW_UPGRADE, then poll. A busy state or 15 s must pass before an idle state counts as done. |
| 4 | DELETE | From an idle FUS, send FUS_FW_DELETE, then poll, as for UPGRADE. |
| 5 | START_WS | From an idle FUS, send FUS_START_WS and wait for the reset. FUS_GET_STATE is never sent after it. |
| 6 | POLL | Send FUS_GET_STATE until the state is idle, not-running or error. |

The **result block** is little-endian `u32` words at these offsets:

| Offset | Field |
|--------|-------|
| 0x00 | magic `0x52534D31` |
| 0x04 | op |
| 0x08 | phase: 1 start, 2 CPU2 booted, 3 ready, 4 command sent, 5 polling, 6 waiting for reset, 0xD0 done, 0xEE failed |
| 0x0C | error (0 = none) |
| 0x10 | PWR_CR4 at entry |
| 0x14 | RCC_CSR at entry |
| 0x18 | FLASH_SFR |
| 0x1C | FLASH_SRRVR |
| 0x20 | FLASH_IPCCBR |
| 0x24 | HCLK in Hz |
| 0x28 | mode: 0 = fresh mailbox, 1 = reused the bootloader's mailbox |
| 0x2C | ready code: 0 = wireless stack, 1 = FUS, 0xFF = none |
| 0x30 | ignored events |
| 0x34 | command status bytes |
| 0x38 | last state, as state \| error << 8 |
| 0x3C | number of state changes |
| 0x40 | elapsed ms |
| 0x44 | device-info pointer |
| 0x80 | 0x60 bytes of the device info table |
| 0xE0 | history of state changes, as `u8` (state, error) pairs |

The error codes are in `src/main.rs`.

When it finishes, the helper shows the result on the LEDs and then
requests a system reset:

- blue while working;
- green on success;
- red on error.

With SW2 still on BOOT0, the reset re-enters the ROM bootloader. The host
then reads the result block with DFU UPLOAD and runs the next step. The
FUS's own resets (upgrade, delete, start) also land in the bootloader.
The host runs INFO or POLL again until the FUS reports idle with the
expected versions.

The mailbox layout and command sequences match hwlab's SWD driver
(`plugins/stlink/stm32/fus.go`, ADR 0039). That driver took the
STM32WB5MM-DK from FUS 1.1.1 to 1.2.0 and installed MAC 1.24.0.
