# STM32WB FUS sequencer (flash-resident)

A ~4.3 KiB CPU1 program that updates the STM32WB coprocessor (CPU2) on a
board **without a debug probe**, such as the P-NUCLEO-WB55 USB dongle
(MB1293C). It does the following, in order:

1. FUS upgrade to 1.2.0;
2. delete the old wireless stack;
3. install the 802.15.4 MAC 1.24.0;
4. start the MAC.

The ROM USB DFU bootloader (ST AN3156) can program flash but has no FUS
commands. hwlab's `stm32dfu` plugin programs this sequencer at
`0x0800_0000` and stages the coprocessor images. On each normal boot (SW2
away from BOOT0) the sequencer runs **one** step of the update, decided
from what CPU2 reports. The FUS's own resets re-run it, so it continues
by itself until it reaches a stop.

The mailbox layout and command sequences match hwlab's SWD driver
(`plugins/stlink/stm32/fus.go`, ADR 0039). That driver updated the
STM32WB5MM-DK and the Nucleo-WB55 the same way.

## Why it is in flash, not SRAM

The first version ran from SRAM1, loaded with DFU `memory-write` and
started with DFU leave. On hardware that path does not work:

- The ROM bootloader boots CPU2 (the factory BLE stack) on every reset.
- It resets the IPCC when it leaves DFU.
- After that, CPU2 never answered a system command, even after re-arming
  C2CR/C2MR.

A program that starts from a normal boot sees C2BOOT clear and can open a
fresh mailbox. hwlab's SWD driver uses the same path.

## Safety

- **No erasing.** The sequencer never erases flash and never writes
  option bytes.
- **Log page only.** It programs only erased double words inside its log
  page (`0x080C_0000`, 4 KiB), and only below the secure start (SFSA).
  It takes HSEM 2 for every write and waits for CPU2's flash semaphore
  (HSEM 7).
- **Staged images are checked first.** It sends `FUS_FW_UPGRADE` only
  when the staged image's CRC-32 is right for the running FUS:
  - FUS 0.5.3 needs ST's special "for FUS 0.5.3" image;
  - other FUS versions below 1.2.0 need `FUS_fw_1_2_0`.
- **MAC install conditions.** It installs the MAC only when all of these
  hold:
  - the FUS is at least 1.2.0;
  - no wireless stack is installed;
  - the FUS staging area is erased;
  - the MAC image at `0x080E_1000` has the expected CRC-32.
- **Bounded retries.** The log caps every action: 48 boots, 6 FUS-entry
  attempts, 3 FUS upgrades, 2 deletes, 2 installs, 3 stack starts and
  2 first-start resets. Reaching a cap stops the sequencer.
- **Image placement.** All staging, erasing and image writing is done
  over DFU by the host, under the plugin's backup and SFSA checks. The FUS
  installs a stack where it was staged. The MAC must be at `0x080E_1000`
  so that SFSA becomes 0xE1, which the router layout
  (`products/stm32wb-router`) requires.

## Decision table (one per boot)

| CPU2 reports | Action or stop |
|--------------|----------------|
| wireless stack, MAC 1.24, SFSA 0xE1 | **Done** |
| any other wireless stack | EnterFus: FUS_GET_STATE ×2; the stack reboots into the FUS |
| FUS in error with ERR_UNKNOWN (first start) | Reset |
| FUS < 1.2.0, matching image staged | UpgradeFus |
| FUS < 1.2.0, no matching image | stop **NeedFusImage** |
| FUS ≥ 1.2.0, MAC 1.24 installed, SFSA 0xE1 | StartWs |
| FUS ≥ 1.2.0, any other stack | DeleteStack |
| FUS ≥ 1.2.0, no stack, FUS image still staged or MAC not staged | stop **WaitMacStaging** |
| FUS ≥ 1.2.0, no stack, MAC staged | InstallMac |

The table is `decide()` in `src/lib.rs`, with host tests.

## LEDs (dongle: blue PA4, green PB0, red PB1)

| Pattern | Meaning |
|---------|---------|
| all three for 0.3 s | a boot started |
| blue blinking (250 ms) | working |
| green blinking (500 ms) | WaitMacStaging: phase 1 done; stage the MAC over DFU |
| green steady | Done: MAC 1.24 runs, SFSA 0xE1 |
| red steady | any other stop or a fault; read the log |

## Log

The log page holds 512 records of 8 bytes:

- `w0 = 0x5A | kind << 8 | arg << 16`;
- `w1 = value`.

The record kinds are Boot, Ready, FusVersion, StackVersion, State, Action,
Stop and Error. Action records are written **before** the action runs, so
they count attempts that ended in a reset.

To decode a DFU dump of the page:

```sh
./decode-log.py dump.bin           # dump read from 0x080C0000
```

## Host procedure (dongle)

1. **Stage phase 1** (SW2 on BOOT0, device in DFU):
   - program the sequencer at `0x0800_0000`;
   - check that the log page is erased;
   - stage the FUS image at `0x080C_3000`;
   - read everything back and check the SHA-256 values.
2. **Run phase 1:** set SW2 to normal and press RESET. It ends on green
   blinking: FUS 1.2.0 and no stack.
3. **Stage the MAC** (SW2 on BOOT0, RESET, device in DFU):
   - read the log;
   - check that SFSA ≥ 0xF4, so the secure start is above the MAC's end at
     `0x080F_3174`;
   - take a new backup;
   - erase `0x080C_3000..0x080C_B000`;
   - stage the MAC at `0x080E_1000` and verify it.
4. **Run phase 2:** set SW2 to normal and press RESET. It ends on green
   steady: MAC 1.24.0 runs and SFSA is 0xE1.
5. **Program the router** (SW2 on BOOT0, RESET): check SFSA 0xE1 and
   program the router image. The router image replaces the sequencer at
   `0x0800_0000`.

## Build

```sh
cd tools/stm32wb-fus-shim
cargo build --release
arm-none-eabi-objcopy -O binary \
  target/thumbv7em-none-eabihf/release/stm32wb-fus-shim stm32wb-fus-sequencer.bin
cargo test --lib --target "$(rustc -vV | sed -n 's/^host: //p')"
```

The linker script (`shim.x`) places the image at `0x0800_0000` with SP at
`0x2003_0000`. It asserts that there is no `.data` or `.bss`.
