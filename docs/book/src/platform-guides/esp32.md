# ESP32-C6 / ESP32-H2

The ESP products share a board flash resource, product profile/storage/OTA
implementation, and the platform-independent sleepy-sensor lifecycle.

## Layering

```text
environmental OTA profile
        ↓
products/esp32-zigbee-devkit (identity, policy, partitions, OTA)
        ↓
boards/esp32-zigbee-devkit (physical flash resource)
        ↓
esp-hal + esp-radio + EspRadioPhy
        ↓
zigbee-mac SoftMacCore (EspSoftMac)
```

The example root owns chip startup, radio resources and button/LED/temperature
adapters. The product supplies the shared interrupt-driven time driver and
single-future executor, button wake and optional retained PMU sleep.
`SensorApp` owns commissioning, parent polling, reporting, persistence
checkpoints, and OTA-first event routing.

Both targets use the Rust `esp-radio` IEEE 802.15.4 backend, with the existing
Espressif PHY binaries underneath; the Zigbee stack is not ESP-IDF/ZBOSS.
Local, version-pinned HAL/radio sources live in `vendor/`; their
`LOCAL-PATCHES.md` files record provenance and sleep changes.

The MAC is the shared `SoftMacCore` over `EspRadioPhy` (`EspSoftMac`):
scan, association, polling, CSMA-CA backoff and retries are the common
implementation. The radio supplies raw TX, hardware auto-ACK with frame-pending
control and a software Imm-Ack wait (`type Tx = AckWaitTx`; esp-radio has no
usable CCA/retry engine), and 10 dBm through
`RadioPhy::DEFAULT_TX_POWER_DBM`. This migration is build-, layout-, symbol-
and host-tested only; the hardware evidence below predates it.

## Node roles

Each example crate has one bin per role, following the
[role template](../advanced/soft-mac-porting.md#role-template-one-example-crate-per-board):

| bin | feature | app | MAC | model |
|---|---|---|---|---|
| `esp32c6-sensor` / `esp32h2-sensor` | `sed` (default) | `sensor-sed-app` | `EspSoftMac` (`EndDevice`) | `ESP32-{C6,H2}-Sensor` |
| `esp32c6-ed` / `esp32h2-ed` | `ed` | `AlwaysOnEndDeviceApp` | `EspSoftMac` (`EndDevice`) | `ESP32-{C6,H2}-ED` |
| `esp32c6-router` / `esp32h2-router` | `router` | `ParentRouterApp` | `EspRouterMac` = `SoftMacCore<EspRadioPhy, Parent<8, 8, 0>>` | `ESP32-{C6,H2}-Router` |

```bash
cargo +nightly-2026-08-01 build --release --locked -Z build-std=core,alloc \
  --no-default-features --features ed --bin esp32c6-ed
cargo +nightly-2026-08-01 build --release --locked -Z build-std=core,alloc \
  --no-default-features --features router --bin esp32c6-router
```

The ED and router images expose only Basic and Identify (the DevKit has no
fitted sensor besides the on-chip temperature, which stays with the SED
image). They have no OTA client. The product enables their modules through
its `roles` feature; the SED-only modules (policy, wake, OTA) sit behind
`sed`, so the existing SED images do not change.

**Status LED (`status-led`, mains roles only).** All three boards (C6
DevKitC-1, H2 DevKitM-1, Waveshare H2-Zero) fit a WS2812-class RGB LED with
DIN on GPIO8; a plain GPIO level cannot light it. The router image
(`--no-default-features --features router`) always enables `status-led`;
the ED takes it on request (`--features ed,status-led`). Such an image
drives the LED from RMT channel 0:

- `router_app::StatusIndicator` (shared, host tested) maps the
  `RouterStatus` lifecycle, the NIB permit-join flag and OTA events to an
  indication: error > identify > OTA > permit join > joined > searching >
  booting. Joined is shown for `joined_hold_ms`, then the LED goes dark.
- The product (`esp32_zigbee_devkit_product::status_led`) owns the palette:
  dim white booting, amber 1 Hz searching, green for 30 s after joining,
  blue 0.5 Hz permit join, magenta 2 Hz identify, cyan OTA flash, red
  error; no channel above 24/255.
- The board (`esp32_zigbee_devkit::status_led`) owns GPIO8 and the RMT
  backend: 16 MHz RMT ticks, 0.31/0.88 µs bit halves, a 300 µs latch, one
  25-symbol frame that fits a single channel memory block. A frame is
  written only when the colour changes (about 330 µs busy wait, interrupts
  enabled).
- The colour byte order, the bit encoding and the ns-to-tick timing come
  from the platform-neutral `zigbee-ws2812` crate in
  [zigbee-rs-drivers](https://github.com/faronov/zigbee-rs-drivers)
  (`drivers/ws2812`, git dependency pinned to an exact rev, pulled in only
  by `status-led`). It also has an SPI byte-pattern encoder for future
  nRF52/EFR32/BL702 backends.
- The byte order is a board fact (`esp32_zigbee_devkit::BOARD`, fixed at
  build time): the Espressif devkits take GRB, the Waveshare ESP32-H2-Zero
  takes RGB. Build H2-Zero images with the `waveshare-h2-zero` feature,
  e.g. `--no-default-features --features router,waveshare-h2-zero`; a GRB
  image on the H2-Zero shows green as red. The lab-only
  `status-led-selftest` feature shows red, green and blue for 3 s each at
  start (logged), to check the order on a new board; it is never part of
  a shipped image. Both were confirmed on an H2-Zero (red, green, blue,
  joined green, identify magenta).

It is default-on only for the router (about 3.8 KB of app image, 32 B of
`.bss`). The ED keeps it opt-in, and sleepy images reject it at build time
and keep GPIO8 isolated; without it the ED logs status on the console only. The
router has no OTA client, so the OTA indication does not occur there yet.

**Frame Pending.** The radio auto-ACKs in hardware, and the core is too late
to influence that ACK, so the router keeps the core's `PENDING` capacity at
0. The `esp-parent` feature of `zigbee-mac` enables the local esp-radio patch
`ieee802154-ack-pending`: in the RX_DONE interrupt, before the ACK is sent,
a hook looks up the source of a MAC Data Request in the
`zigbee_mac::ack_pending` table (16 entries, short or extended addresses)
and sets the ACK's Frame Pending bit. The core fills the table through
`RadioPhy::set_ack_frame_pending`. A full table fails the request with
`PhyError::Hardware` instead of dropping a child silently.
`EspRadioPhy::parent_stats()` counts Data Requests, FP ACKs, raw
transmissions, TX errors, acknowledged transmissions and missing ACKs; the
router logs them every 60 s.

Every ESP image enables the local esp-radio patch
`ieee802154-tx-when-rx-full`. Upstream esp-radio refuses to transmit while
its receive queue is full, and nothing else drains that queue. On the C6 a
router relaying a broadcast burst then failed RREQ forwarding and link status
with `RouteError`. An always-on end device whose parent never acknowledged
its polls stopped being able to send any Beacon or Association Request.

A sleepy end device drops the frames its radio queued while it was idle.
esp-radio keeps the receiver on between operations, so between two polls the
SED's 20-frame queue filled with broadcasts it was not receiving. The ACK
window of the next poll then met that backlog instead of the ACK, and every
poll was reported unacknowledged. `EspRadioPhy` therefore clears the queue,
inside the critical section that esp-radio already takes, before each
acknowledged transmission and before suspend, but only while
`macRxOnWhenIdle` is false. Routers and always-on end devices keep every
frame.
All images
convert RSSI to the IEEE 0–255 LQI scale (`zigbee_mac::lqi`) for neighbour
and link-cost decisions. The radio's raw LQI byte (about 7–11) maps every
router to link cost 7, which made end-device rejoin parent selection reject
every beacon ("none suitable as parent").

**Persistence.** The role images write a role tag (ROUTER `0x5201`,
END_DEVICE `0x5202`) in the security journal. The SED image writes none, so
its journal records are unchanged. The router keeps its child and APS tables
in `zbnv` at `0x3F0000..0x3F4000`, below the security journal at
`0x3FE000`. After a ZHA leave, the router and ED rejoin when the BOOT button
is pressed or after 5 minutes. Remove the device from the network before
flashing the SED image back.

## Application parts and policy

Both images provide a concrete
`esp32_zigbee_devkit_product::ota_transport::OtaTransport` paired with an
`WithOta` profile.

| build selection (both chips) | fast wait | slow wait | slow parent poll |
|---|---|---|---|
| default | `Active` | `Active` | 30 s |
| `--features light-sleep` (experimental) | `Active` | retained `Idle` | 5 s |

Neither SED image shows a status: the C6 has no LED adapter, and the H2
sensor's active-low GPIO8 output cannot light the board's WS2812. It is
kept unchanged so the validated image stays byte-identical; the
`status-led` driver above is for the mains roles only.

`Active`/`Active` describes the radio/SoC policy, not a continuously spinning
CPU. The executor uses `WFI` while its root future is pending. SYSTIMER alarm 0
wakes expired Embassy timers and is rearmed for the next deadline; the
check-to-idle transition masks interrupts to avoid losing a wakeup. The HAL
monotonic counter is not reset at startup.

The default is **CPU idle only**, with radio RX enabled. Button and long-press
waits now use GPIO interrupts rather than a 10 ms polling timer.

The opt-in path stops a quiet MAC, disables the PHY and enters actual PMU
light sleep, retaining CPU state, RAM, flash power and digital/register
domains. GPIO9 uses digital low-level wake, not RTC EXT1. After wake, measured
LP-timer time corrects only the missing SYSTIMER ticks, overdue Embassy alarms
are serviced, and PHY/RX are restored before the application continues.
This is not reset-on-wake deep sleep, and it does not promise minimum-current
operation while so many domains remain retained.

Queued/raced RX, active TX/auto-ACK and pending interrupts veto suspension.
The application services that activity and checks the idle policy again.
Commissioning/TCLK, APS acknowledgements, queued protocol responses, rejoin,
interview/identify and active OTA keep short, radio-on waits. Sub-20 ms waits
remain active; a rejected PMU request is reported as rejection, not as sleep.
Pending UART transmission also vetoes entry. A native USB session observed
since boot keeps the device radio-on with an explicit warning: the existing
auto console does not own a sleep-safe USB suspend/restore protocol. For power
measurements, cold-boot from a power-only supply (or use the separate UART
console). Unplugging native USB without resetting is not enough; its console
selection latch is deliberately preserved.
Unexpected wake/restoration failures are fatal rather than silently falling
back to WFI. Five-second polling leaves margin under a typical parent queue
lifetime, but still needs verification with the actual coordinator/parent.

**Hardware acceptance is pending:** timer/button and simultaneous wakes,
rejected sleep, repeated sleep/wake, parent loss/rejoin, downlink commands,
reports, OTA, long-press reset, and measured current/temperature on both chips.
No hardware was flashed by this change. Defaults and the web-flasher image
remain radio-on until qualification; do not infer battery suitability from
successful compilation.

The reported temperature is the on-chip sensor reading, not ambient
temperature. Real self-heating is possible. No guessed temperature offset is
applied. Both chips select ESP-IDF's default −10…80 °C TSENS range (DAC 15,
offset 0) before every sample, verify it by read-back, and subtract the
per-chip sign-magnitude `TEMP_CALIB` eFuse delta exactly as ESP-IDF v5.5
does (`products/esp32-zigbee-devkit/src/chip_temperature.rs`). Earlier C6
builds used esp-hal's hard-coded offset −1 without programming the DAC,
which over-reports by 27.88 °C when the DAC is in the default range. C6,
like H2, powers its temperature sensor only around samples. Accuracy against
a reference thermometer is still a hardware gate, separate from measuring the
effect of sleep.

OTA cluster events reach `OtaTransport` before generic application handling.
When the image is ready, `SensorApp` checkpoints Zigbee security state before
the product writer changes `otadata` and resets.

### ESP32-H2 thermometer SED (`thermometer`, `quiet`)

`esp32h2-thermometer` is a temperature-only Sleepy End Device for battery and
current measurements on boards whose status hardware it does not drive (the
Waveshare ESP32-H2-Zero; its WS2812 on GPIO8 stays dark and untouched).
Endpoint 1 carries Basic, Identify, Poll Control (0x0020) and
Temperature Measurement (0x0402) fed by the eFuse-calibrated die sensor.
Every policy value — identity, reporting defaults, poll/fast-poll periods,
sampling cadence, Poll Control limits, rejoin backoff, the Active-time
bound, the oscillator power-down and the USB pad policy — comes from
`examples/esp32h2-sensor/thermometer.toml` through `build.rs` (override per
build with `H2_THERMO_<KEY>` or `H2_THERMO_CONFIG`).

```bash
cd examples/esp32h2-sensor
cargo +nightly-2026-08-01 build --release --locked -Z build-std=core,alloc \
  --bin esp32h2-thermometer --features thermometer,light-sleep,quiet
```

- `quiet` sets `log` to `max_level_off`; the bin links no console.
- `reporting-persist` (enabled by `thermometer`) journals the coordinator's
  Configure Reporting in the product's two-sector `0x3F4000..0x3F6000`
  window (keyed by extended PAN ID, CRC + commit marker, alternating
  sectors) and restores it after a reset on the same network. About 4.5 KB
  of flash: journal scan/commit/erase ≈2.4 KB, snapshot capture/encode
  ≈0.6 KB, record parse ≈0.4 KB, the rest flash-window glue.
- Poll Control: periodic Check-in to the bound client, Check-in Response
  fast poll bounded by `fast_poll_timeout_max_ms`, Fast Poll Stop, and
  Set Long/Short Poll Interval range-checked against the profile. With
  `reporting-persist`, CheckInInterval, FastPollTimeout and the
  Long/Short Poll Interval a coordinator changed are journalled in the
  same snapshot (same window and extended-PAN binding) and restored after a
  reset only if they still fit the profile's limits.
- Keep-awake limits and the anomaly record: `thermometer.toml` sets
  `max_active_ms`, `max_ota_awake_ms`, `max_identify_awake_ms`,
  `max_fast_poll_ms` and `missed_report_grace_ms` (see the power guide).
  `anomaly-record` (enabled by `thermometer`) journals the record in the
  two-sector `0x3F6000..0x3F8000` window only when it changes (at most
  `anomaly_max_writes_per_boot` writes per boot) and serves it from a
  minimal Diagnostics (0x0B05) server on the endpoint, each attribute with
  its standard data type:
  `0x0000` (`NumberOfResets`, uint16) = total anomalies;
  `0x0001` (`PersistentMemoryWrites`, uint16) = summary: bits 0-7 last
  reason code, bits 8-15 reasons seen (bit `code - 1`);
  `0x0100` (`MacRxBcast`, uint32) = last cause word (below);
  `0x0101`/`0x0102` (`MacTxBcast`/`MacRxUcast`, uint32) = per-reason
  counts, one saturating byte per reason, reasons 1-4 / 5-8, LE.
  Reason codes: 1 pending work, 2 OTA, 3 Identify, 4 fast-poll windows,
  5 missed report, 6 parent-loss rejoin, 7 unplanned reset of the previous
  boot, 8 light sleep disabled. The standard IDs are reused with this
  meaning; `0x0000` reports on change only. Read them from ZHA
  (Manage Zigbee device → Clusters → Diagnostic → Read attribute).
- Cause word (reasons 7 and 8): bits 28-31 kind, 0-27 payload. Kind 1 wake
  error, payload = variant << 24 | detail (1 unsupported depth, 2 sleep not
  configured, 3 GPIO wake config, 4 HAL sleep error: 1 UnsupportedClock,
  2 UnsupportedConfiguration, 4 InvalidDuration, 5 InvalidCounter, and
  InvalidCalibration as 6 RTC slow-clock period 0, 7 RC_FAST calibration
  timeout, 8 RC_FAST result out of range (3 = unspecified calibration
  failure, written only by older firmware); 5 unexpected PMU wake cause, 6 repeated
  PMU rejection with the reject cause, 7 radio wake failure); kind 2 panic,
  payload = flash-code offset from `0x4200_0000` of the first return
  address on the stack (the panicking function's caller; 0 if none),
  resolve with `addr2line -f -e <elf> 0x4200_0000+offset` against the exact
  ELF; kind 3 profile failure;
  kind 4 hardware reset without a software cause, payload = the raw ROM
  SoC reset reason (e.g. 0x0F brownout, 0x07-0x12 watchdogs). Software,
  USB and debugger resets (0x03, 0x05, 0x0C, 0x15, 0x16, 0x18 CPU0_JTAG)
  are planned and never produce kind 4; the code is read from the ROM, not
  through esp-hal's `SocResetReason`, which has no 0x18 variant.
- Sleep/reset resilience: before a software reset the cause is left in four
  words of RTC fast memory (`.rtc_fast.persistent`, survives resets, not
  power-on); the next boot notes it as reason 7 and logs one `[boot]` line
  in non-quiet builds. A failed light sleep is retried as an active wait;
  after `sleep_failure_limit` consecutive failures the boot stays awake and
  records reason 8 with the wake error. After `rapid_reset_limit` boots in a
  row shorter than `rapid_reset_window_ms`, the next boot keeps light sleep
  off and records reason 8 once (cause = carried reset cause).
- Light sleep stops XTAL and RC_FAST (`sleep_xtal_off`,
  `sleep_rc_fast_off`) and applies ESP-IDF's light-sleep analog settings.
  The H2 PMU sequence is the repository's own RAM-resident
  `esp32h2-sleep` crate (`esp32h2-sleep/README.md`), a port of
  ESP-IDF v5.5.1 that replaces the H2 entry of the vendor esp-hal backport
  (`vendor/esp-hal/LOCAL-PATCHES.md`; C6 still uses the backport). With
  `sleep_mode = "retained"` (default) CPU, top, modem and flash domains
  stay powered. `sleep_mode = "top_pd"` (`--features top-pd`,
  **experimental**) powers TOP, CPU and modem off and retains ~5.3 KiB of
  registers in RAM, aiming at the datasheet ~25 µA light-sleep floor; the
  BOOT button then wakes through EXT1. With TOP off the PMU leaves
  `dig_pad_slp_sel` clear and IO_MUX/GPIO lose power, so the isolated pads
  are also latched with the always-on `LP_AON_GPIO_HOLD0` pad hold (below).
  A TOP power-down also resets the USB Serial/JTAG (the host sees a
  detach and the start-of-frame latch behind the USB veto is cleared), and
  ~17 ms wakes are too short for the host to enumerate again. The
  thermometer therefore takes no PMU sleep during `usb_host_wait_ms` after
  boot (`SensorWake::with_usb_host_wait`); a host seen in that window keeps
  the device awake and reflashable for the power cycle, in either mode, and
  `top_pd` refuses `usb_host_wait_ms = 0` at build time. Images before this
  gate (e.g. `1b0a72c`, `c53ca91`, `a8f849b5`) can lose that race and then
  drop off USB even with a host attached; they are recovered through ROM
  download mode (BOOT + plug), where hwlab `9c75594` or later attaches the
  SPI flash through the ROM first (before it, JEDEC read `ffffff`). The gate
  is hardware-proven on the H2-Zero (`90ef338`, image `d93a205e`): a plain
  replug without BOOT keeps a stable USB Serial/JTAG port, hwlab reads the
  chip, and the device rejoins.
  `tools/esp32h2-sleep-ram-check.py` checks in CI that the sleep/restore
  path never reaches flash.
- CPU clock: the ESP-IDF bootloader leaves the H2 CPU on FLASH_PLL
  (64 MHz from BBPLL, source 3). Earlier HAL code rejected that source, so
  every sleep failed with `UnsupportedClock` (cause `0x14000001`, reason 8)
  and the device drew a constant ~26 mA. The HAL now saves source and
  dividers, sleeps on XTAL and restores FLASH_PLL after recalibrating BBPLL,
  as IDF's `rtc_clk_cpu_freq_set_config` does. The code from the sleep
  request to the clock restore runs entirely from RAM/ROM.
- RC_FAST calibration: H2 ECO2+ (wafer v1.2 on the test board) measures
  RC_FAST through the REF_TICK /32 divider, which needs
  `PCR.ctrl_tick_conf.fosc_tick_num = 255` (IDF `REG_FOSC_TICK_NUM`). The
  reset value 7 made every sleep fail with `InvalidCalibration`; the HAL
  now sets it at init and before each RC_FAST calibration, and switches
  RC_FAST/RC32K back off afterwards when it had to enable them. XTAL32K and
  RC32K are no longer kept running (the slow clock is RC_SLOW).
- Boot self-test: `sleep_selftest = true` (non-`quiet` builds with
  `light-sleep`, e.g. `H2_THERMO_SLEEP_SELFTEST=true cargo build --release
  --bin esp32h2-thermometer --features thermometer,light-sleep`) runs
  `sleep_selftest_count` timer-only light sleeps of `sleep_selftest_ms`
  through the production backend before the radio starts, even with USB
  attached, and logs chip revision, `ctrl_tick_conf`, the slow period and
  per sleep OK/cause word, the RTC-measured duration and the wake cause.

- Poll-response window: `poll_response_wait_ms` (default 32, range
  10..=1000) bounds how long a data poll listens after an ACK with Frame
  Pending. 32 ms covers 802.15.4 macMaxFrameTotalWaitTime (1986 symbols,
  31.8 ms); the ESP backend default stays 250 ms for association. A frame
  the parent sends later stays queued there and is fetched by the next poll;
  pending APS/ZDO work keeps the 250 ms fast poll. On the WSTK capture of
  `573f44f` each report spent ~250 ms in RX after an empty FP=1 ACK
  (~6 mC of the ~8.2 mC report event); `014d77b` (32 ms window) still held
  the CPU awake for the 250 ms until the poll that fetches the report's APS
  ACK. Since then a wait for APS ACKs only light-sleeps at the fast-poll
  depth (`fast_poll_light_sleep`), keeping the 250 ms cadence and the
  fetching poll; ZDO responses, key exchanges, rejoins and fragmented
  transfers still wait awake.
- OTA: an active session (query, download, verify, activation wait) holds
  the fast poll awake, bounded by `max_ota_awake_ms`. A NO_IMAGE_AVAILABLE
  or failed Query Next Image Response, or the runtime's 120 s response
  timeout, ends the session and the slow poll resumes at once; the former
  fixed 120 s `OTA_KEEP_AWAKE_MS` tail (~213 mC on the `014d77b` capture,
  triggered by ZHA's Image Notify) is gone. Device_annce retries are
  `announce_retry_ms`/`announce_retries` in `thermometer.toml`.
- Sleep-floor lab: `esp32h2-floor-lab` (`--no-default-features --features
  floor-lab`, `floor_lab.toml`) starts no radio. With a USB host it dumps
  the PMU sleep, pad, USB and clock registers before and after one light
  sleep; without a host it disables the USB pad and cycles light sleep
  (LED DIN untouched, isolated, driven low) and an LP-timer deep sleep, so a
  power trace separates board quiescent current from chip sleep
  configuration. Light and deep sleep go through `esp32h2-sleep`
  (`light_mode = "retained"` or, with `--features floor-lab,top-pd`,
  `"top_pd"`); deep sleep is timer wake only. Production images do not
  link deep sleep. The measurements below used the earlier esp-hal
  backport path. On the H2-Zero (WSTK AEM, 3.3 V) the lab measured light sleep
  1.10 mA (= the production floor), 0.97 mA with GPIO8 isolated, 1.18 mA
  with GPIO8 driven low and 0.73 mA in deep sleep.
- **Known board limit (Waveshare ESP32-H2-Zero):** ~0.73 mA flows even in
  deep sleep, so it is board quiescent current, not chip configuration. The
  WS2812 sits on the unswitched 3V3 rail and the LDO may be back-fed from
  VMCU; the two cannot be told apart without rework, and no rework is
  planned (the parts are too small). Battery estimates for this board must
  include it. Firmware work on the light-sleep configuration is measured as
  **B − D** (lab phase B, light sleep with GPIO8 isolated, minus phase D,
  deep sleep), which cancels the board current: ≈240 µA at `22674da`
  against ~80 µA expected from the datasheet light/deep difference.
- Sleep pad isolation: `sleep_isolate_gpio_mask` (bit n = GPIOn) lists the
  pads the thermometer does not use. Once at boot they are switched to
  GPIO function, output/input/pulls disabled in both the active and the
  sleep pad configuration, and `SLP_SEL` set, like ESP-IDF's
  `esp_sleep_config_gpio_isolate()` + `esp_sleep_enable_gpio_switch()` but
  limited to an explicit mask. The default `0x02407DFF` covers GPIO0-8,
  10-14, 22 and 25 on the H2-Zero (GPIO8 = WS2812 DIN, isolated like lab
  phase B). The build refuses GPIO9 (BOOT/Poll Control button), the flash
  pads GPIO15-20 and USB GPIO26/27 (also checked in
  `products/esp32-zigbee-devkit::pad_isolation`), and UART0 GPIO23/24
  unless `quiet`. A driver that claims a pad later reconfigures it. After
  isolating, the same pads are latched with the per-pad hold
  (`LP_AON_GPIO_HOLD0`, ESP-IDF `gpio_ll_hold_en`), the only pad state that
  survives TOP power-down; without it the `top_pd` floor on the H2-Zero was
  1.00-1.02 mA instead of the retained 0.80 mA (`1b0a72c`, WSTK AEM), with
  it 0.895 mA (`90ef338`), still above retained (status below). The
  hold survives every reset except power-on, so an image that drives one of
  these pads must release its hold bit first (`esp-hal` does not).
- HP_SLEEP PMU parity with ESP-IDF v5.5.1: the H2 init writes the
  `PMU_HP_SLEEP_*_CONFIG_DEFAULT` values IDF sets once and its sleep path
  never rewrites (HP APB clock gates closed, the dedicated sleep memory and
  logic regulators off), and light sleep gates the HP function clocks
  (`icg_func = 0`, IDF `get_sleep_clock_icg_flags()`). The reset values
  kept both sleep regulators on and every clock gate open. The modem ICG
  code IDF leaves unconfigured on H2 is not written. Effect on B − D is a
  **hardware gate**.

Status: join, interview, temperature reporting, the anomaly record and
retained light sleep with XTAL/RC_FAST off are lab-tested on one H2 board
through the esp-hal backport path (WSTK AEM, `61b0f8a`: polls every
29.75 s at ~557 µC, a 0.802 mA sleep floor and ~823 µA average; 0.73 mA of
it is the H2-Zero board limit above). The retained mode through
`esp32h2-sleep` (`3aa3ffa`, quiet image `694fb656`, WSTK AEM, 1 h from a
VMCU power cycle) matches it: the restored-state fast window ends ~103 s
after power-on, then polls every ~29.8 s at a median ~511 µC above the
floor, reports ~1.6-1.9 mC, a 0.80 mA floor (0.801-0.805 mA in four quiet
windows across the hour, peaks ≤0.99 mA) and ~824 µA average after the
fast window (~848 µA over the whole hour including it). Diagnostics read
`0x0000` = 5 and `0x0001` = `0xE008` before and after, so no keep-awake
limit fired. `top_pd` (`1b0a72c`) is lab-tested
on the WSTK AEM for 1 h: no boot loop, every 29.8 s poll answered after
TOP/modem power-down, reports delivered, polls ~514 µC and reports ~2.0 mC
(as retained), but a 1.023 mA floor and ~1,057 µA average because the pad
isolation was not held. With the pad hold and the USB gate (`90ef338`,
image `d93a205e`, WSTK AEM, 1 h) `top_pd` again ran without a boot loop,
kept a regular 29.76 s poll with retained-size pulses, delivered reports (ZHA's 900 s maximum) and
answered Diagnostics reads (`0x0000` = 5 and `0x0001` = `0xE008` before
and after, so no keep-awake limit fired), with polls at a median ~524 µC
above the floor and reports ~1.7-2.0 mC, but a 0.895 mA floor (stable
over the hour, against 0.802 mA retained) and ~929 µA average. On this
board `top_pd` therefore costs ~93 µA more floor than retained: its chip
excess above the 0.73 mA board limit is ~165 µA against ~72 µA retained,
and its quiet windows peak at ~1.4 mA against ≤0.95 mA retained (not
yet characterised). Retained stays the default;
`top_pd` remains experimental until that excess is explained. A
tens-of-uA floor needs a different board; on the H2-Zero the target is
B − D near 80 µA. Reporting persistence across a power cycle and Poll
Control check-in remain **hardware gates**.

## Product-owned flash layout

The checked 4 MiB table is:

```text
0x000000..0x008000  second-stage bootloader
0x008000..0x008C00  partition table
0x009000..0x00B000  redundant otadata sectors
0x010000..0x200000  ota_0
0x200000..0x3F0000  ota_1
0x3F0000..0x400000  zbnv
0x3F0000..0x3F2000  router child table within zbnv (router image only)
0x3F2000..0x3F4000  router APS table within zbnv (router image only)
0x3F4000..0x3F6000  reporting journal within zbnv (reporting-persist)
0x3F6000..0x3F8000  anomaly journal within zbnv (anomaly-record)
0x3FE000..0x400000  security journal within zbnv
```

The board crate exposes raw physical flash. The product checks the table,
bounds writes, selects the inactive slot, owns `otadata`, and constructs the
security journal.

If the on-device table is missing or incompatible, or the running application
cannot be identified safely, startup fails explicitly.
These OTA-capable images always advertise the OTA client cluster and do not
pretend an unsafe staging path exists.

The writer:

- identifies the executing slot from the live flash MMU translation, not the
  preferred `otadata` entry, which can remain stale after bootloader fallback;
- stages only in the other slot and rejects missing or changed running-image
  evidence;
- erases staging sectors lazily;
- pads the final 4-byte write with `0xFF`;
- validates segment count, lengths, load ranges, mapping alignment, descriptor,
  chip/revision compatibility, entry address, XOR checksum, exact image length,
  and appended SHA-256;
- rechecks staged flash before activation;
- writes one redundant `otadata` entry in the `ESP_OTA_IMG_NEW` state for
  activation, and refuses to stage while the running image is unconfirmed,
  because the other slot is then the rollback target.

### Pending verification and rollback

The espflash-bundled ESP-IDF bootloader is built without
`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`. It still honours the entry states
and sequence numbers, so rollback is application-driven
(`esp32_zigbee_devkit_product::ota::{check_boot, confirm_boot, roll_back}` and
the `ota_boot` policy):

1. Before persistence opens, `ota_boot::begin()` checks `otadata`. The first
   boot of a `NEW` image programs a boot-attempt mark into the entry's unused
   `seq_label` bytes, which only clears bits and needs no erase.
2. A fresh join, a secured rejoin, or an inbound decrypted ZCL frame confirms
   the image: its entry is rewritten as `VALID`.
3. A silent resume has no radio exchange. It confirms only after the image has
   run for 15 minutes without a failed rejoin, commissioning or wake.
4. A boot that finds its own mark without confirmation (crash, watchdog or
   power loss), the 15-minute deadline without evidence, a reached
   secure-rejoin limit (before the application wipes network state), or a
   security-store failure selects the previous slot. A `VALID` entry with a
   higher sequence is written to the spare sector first, then the abandoned
   entry is marked `ABORTED`, and the chip resets.
5. If the bootloader itself fell back from an invalid new image, the running
   slot is recorded as `VALID`.

Confirmation erases and rewrites the active sector. A power loss inside that
window boots the previous image. Any deliberate reset before confirmation also
rolls back.

`EspFirmwareWriter::new()` returns an initialization error if its flash
implementation cannot supply running-slot and image-compatibility evidence.
The standard sensor profile propagates this error instead of guessing a slot.

These checks cover unsigned plaintext image structure and integrity, not
secure-boot signatures, anti-rollback (version downgrade) policy, or program
correctness. The
offline size checker and OTA packager share the same Python structural
validator; on-device revision compatibility is checked by the writer rather
than guessed by offline tooling.

## Build and flash

Use the fixed `nightly-2026-08-01` and `espflash 4.5.0`:

```bash
cd examples/esp32c6-sensor
cargo +nightly-2026-08-01 build --release --locked -Z build-std=core,alloc

cd ../esp32h2-sensor
cargo +nightly-2026-08-01 build --release --locked -Z build-std=core,alloc
```

To build the retained-sleep variant in either example directory, add
`--features light-sleep`. This feature does not change partitions, OTA identity
or network persistence. CI links both opt-in images and runs the real-image
OTA corpus separately from the default images. Size reports measure the
artifacts without enforcing artificial regression budgets; physical OTA-slot
and other layout checks remain mandatory.

Documentation can be published after its core/code checks pass even when a
required firmware job fails a build, physical limit, or other check. In that
case the site includes no prebuilt ESP binaries, its firmware manifest has no
builds, and one-click installation is
disabled with a link to the failing CI run. The existing ESP32-C6, ESP32-H2 and
BL702 jobs must all succeed before prebuilt ESP firmware is published.
Exceeding a former regression budget no longer fails those jobs; binaries and
one-click installation can become available when all remaining checks pass.
The flasher's custom-file mode remains an explicit user-supplied operation,
not a qualification of that file or automatic flashing.

The configured runner installs the product partition table, writes `ota_0`,
and clears `otadata`:

```bash
cargo +nightly-2026-08-01 run --release --locked -Z build-std=core,alloc
```

Back up commissioned state before the first partition-table migration.

Local integrated snapshot (2026-09-10), not remote CI or hardware results:

| image | application | former regression budget | merged flash | Zigbee OTA v1 | static `.data + .bss` |
|---|---:|---:|---:|---:|---:|
| ESP32-C6 default | 381,056 | 368,640 | 446,592 | 381,122 | 53,796 |
| ESP32-C6 `light-sleep` | 392,080 | 368,640 | 457,616 | 392,146 | 53,992 |
| ESP32-H2 default | 365,920 | 356,352 | 431,456 | 365,986 | 53,448 |
| ESP32-H2 `light-sleep` | 376,640 | 356,352 | 442,176 | 376,706 | 53,696 |

Each physical OTA slot is 2,031,616 B.
The RAM totals include `.data.wifi` (80 B on C6 and 84 B on H2), but exclude
RAM-resident code. Linked default/light stack regions are 390,696/390,504 B
for C6 and 201,312/200,640 B for H2, not measured high-water marks.
The applications passed structural/layout and host flash-mock staging,
verification, and activation checks. The hardware results below are earlier
path evidence, not exact-image reruns.
All four snapshots exceeded their then-enforced regression budgets:
default/light C6 by 12,416/23,440 B and H2 by 9,568/20,288 B. Those historical
failures remain recorded, but the former budgets are no longer build blockers.
The physical slot and other image/layout checks remain mandatory.
These sizes include the combined idle, OTA, reporting, and runtime changes;
their differences from earlier snapshots are not isolated per-fix costs.

## OTA packaging

From the selected example:

```bash
tools/create-ota.py 2 --toolchain nightly-2026-08-01
```

For the retained-sleep variant, use a separate output directory and explicitly
pass its feature again; a previous feature-selected build does not select the
OTA build's features:

```bash
tools/create-ota.py 2 target/ota-light-sleep \
  --toolchain nightly-2026-08-01 --features light-sleep
```

The tool builds the locked application with `ESP32_OTA_VERSION=2`, creates the Zigbee
OTA container, and updates a `zigpy_local` index. C6 and H2 use distinct image
types, so a server cannot offer one chip's image to the other.
Default and retained-sleep variants share the same chip-specific OTA identity:
do not publish both variants with the same version to one server index.
`--elf` packages an already selected image without rebuilding and cannot be
combined with `--features`.

Ordinary product host tests leave the artifact-dependent tests ignored.
Each CI firmware build explicitly enables them with `--include-ignored`,
using its freshly generated application and the shared Python/Rust validation
corpus alongside size reporting and physical image/layout checks. These
exercise flash mocks, not the physical device.

## Hardware validation

### ESP32-H2

An earlier 4 MiB ESP32-H2 revision 1.2 image demonstrated:

- migration to the security journal;
- secure reset/resume and reporting;
- complete v1→v2 ZHA OTA transfer;
- verification, activation, reboot into `ota_1`;
- retained IEEE address, PAN, parent, network credentials, and counters.

Fresh factory-reset commissioning and long-duration power behavior remain
separate gates. Pending verification and confirmation were exercised on
hardware only by the signed TEST-ONLY runs below and in the
[bootloader results](../advanced/bootloader.md#esp32-h2-signed-zha-ota-2026-11-10);
rollback is host-tested only. Neither current H2 variant above has hardware execution
evidence. Removing artificial size budgets does not close those hardware gates.

### ESP32-C6

Earlier C6 hardware runs demonstrated commissioning/reporting and OTA transfer
to 18.3% before intentional cancellation. The former regression budgets are
not live build gates.

On 2026-11-13 a signed TEST-ONLY sensor (`light-sleep,signed-ota`) completed
a ZHA upgrade from 0.2.0 to 0.2.1 on one DevKit
([results](../advanced/bootloader.md#esp32-c6-signed-zha-ota-2026-11-13)):

- about 43 min download of a 419,640 B container; two parent-loss rejoins
  during the download kept `0xFA41` and the staged data;
- verification, `otadata` switch, reboot into `ota_1`;
- `PendingVerify`, then `running image confirmed` after resuming without a
  rejoin;
- forward-only outgoing NWK frame counter (93,187 before, 113,664 after).

An earlier attempt was aborted by zigpy when the parent was reflashed. A
second failed because downlink to the device was missing for about 15 min
after a parent change; that recovered without intervention, and the cause
was not established. Sleep current, power loss during staging or install,
and rollback remain open. Wrong-key and downgrade refusal were shown on the
H2 only.

Observations from the run, tracked separately:

- ZHA shows the IEEE address byte-reversed (`88:96:f6:fe:ff:f9:55:62` for
  eFuse MAC `60:55:F9:F6:96:88`); the H2 shows the same pattern.
- Rejoin picks the shallowest parent and does not avoid the parent that
  just failed.
- A MAC `NoAck` to the parent is logged as `ZCL frame send failed:
  NoShortAddress`.

Role images on an ESP32-C6 DevKit (hwlab, ZHA coordinator, channel 15):

| role | result | hwlab sessions |
|---|---|---|
| router | joined ZHA as a Router, interview complete, 14–15 neighbours with LQI, 4–5 routes; ran more than 16 minutes without a restart. A router on another platform routed through it. Counters at the end: 982 raw TX attempts, 90 refused by the radio or never completed (the core retried them as a busy channel), 47 acknowledged TX with 25 missing ACKs (mostly one neighbour that never acknowledges); 0 Data Requests, so the Frame Pending hook has not yet run on hardware | `2026-11-01T103101Z-esp32c6-router-v2`, `2026-11-01T104229Z-esp32c6-router-v2-run` |
| always-on ED | fresh join, Transport-Key, ED Timeout accepted, ZHA interview (`ESP32-C6-ED`, EndDevice); a system reset resumed from the journal without a rejoin | `2026-11-01T110710Z-esp32c6-ed-v3-join` |
| SED (light sleep) | fresh join and interview (`ESP32-C6-Sensor`, EndDevice); humidity/temperature reports and ZHA attribute reads through 5 s polls; no failed-poll rejoin in 18 minutes; a system reset resumed from the journal | `2026-11-01T113839Z-esp32c6-sed-ls-v4-join`, `2026-11-01T115656Z-esp32c6-sed-ls-v4-reset-resume` |

The router run used the image from before the receive-queue discard. That
change passes `false` on router and ED paths, and those images were rebuilt
and checked but not rerun on hardware.

The `0 Data Requests` above was not only a lack of children. Until the
`ieee802154-fcf-offsets` patch (see `vendor/esp-radio/LOCAL-PATCHES.md`),
the ACK-request bit was read from the wrong FCF octet. As a result, no
received frame took the auto-ACK path. The Frame Pending hook never ran, and
the receive path restarted RX during the hardware ACK turnaround.

ESP32-H2 router (4 MiB DevKit, ZHA channel 15, joined as `0xA17F`), with the
`ieee802154-fcf-offsets` patch:

| check | result | hwlab sessions |
|---|---|---|
| before the patch | a ZBT-1 (EFR32MG21) router picked the H2 as parent. Association succeeded, but the H2 never acknowledged the first Data Request (`Join failed: DeviceNotFound`); the H2 counted `data-req 0` | `2026-11-03T061048Z-h2assoc-repro2` |
| joins through the H2 | 3 of 3 fresh ZBT-1 joins associated on the first attempt via `0xA17F` (`0x9420`, `0x9421`, `0x9422`). Each received Transport-Key on the first poll, then ZDO descriptors, Basic/Identify reads and TCLK Verify/Confirm-Key. The H2 counters reached `data-req 3 fp-acks 3` with `tx-err 0` | `2026-11-03T063725Z-h2assoc-fix-verify1`, `2026-11-03T064553Z-h2assoc-fix-verify2`, `2026-11-03T065319Z-h2assoc-fix-verify3` |
| Identify | a ZHA Identify (5 s) toggled `identifying` true→false | `2026-11-03T065319Z-h2assoc-fix-verify3` |
| reset/resume | after a SoC reset the H2 resumed with 15 neighbours and child `0x9422` restored; the child stayed online without rejoining | `2026-11-03T070153Z-h2assoc-fix-resume`, `2026-11-03T070425Z-h2assoc-fix-resume2` |

Open hardware gates:

- The C6 router uses the same patch but has not been rerun on hardware.
- No sleepy child has polled an ESP router for a long run, so Frame Pending
  delivery of queued indirect data is still unproven on hardware.
- No packet sniffer has confirmed the turnaround timing of individual ACKs.
- Light sleep is vetoed while the native USB Serial/JTAG session is
  attached. Sleep power needs a cold boot from a power-only supply.
- Light-sleep current in normal operation has not been measured. An H2
  SED that had lost end-to-end delivery through its parent was measured
  stuck in `Active` at ~26.4 mA (WSTK AEM); the shared recovery changes
  (bounded Active time, staged rejoin with slept backoff) target that
  defect and still need an AEM rerun.
- The H2 ED and sensor role images are build-, layout- and symbol-checked
  only.
