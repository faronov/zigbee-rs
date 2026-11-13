# STM32WB5MM-DK peripheral lab

Standalone diagnostics for the fitted parts of the ST STM32WB5MM-DK
(MB1292). Each binary proves one peripheral of `boards/stm32wb5mm-dk`
independently of the Zigbee router; none starts the CPU2 coprocessor or
touches the router's journals. These are not production firmware.

## Binaries

| Binary | Part | Pass criterion |
|---|---|---|
| `stm32wb5mm-dk-diag-rgb` | TLC59731 RGB LED (EasySet on PA7, select PH1) | Every write completes; red, green, blue, white, off in turn |
| `stm32wb5mm-dk-diag-lcd` | SSD1315 128x64 OLED on SPI1 | Init and 30 full flushes succeed, LED writes interleave on the shared PA7; the border, text and counter are readable |
| `stm32wb5mm-dk-diag-sensors` | I2C3: STTS22H, ISM330DHCX, VL53L0X | All three addresses ACK; plausible temperature, ~1 g total acceleration, a ToF range or an explicit out-of-range status |
| `stm32wb5mm-dk-diag-touch` | TSC touch key (G6) with shield (G4), buttons B1/B2 | Stable baseline; the count drops clearly while touched; B1/B2 read low while pressed |
| `stm32wb5mm-dk-diag-mic` | MP23DB01HP PDM microphone on SAI1 A, ring DMA on DMA1 CH2 | Only the configured byte lane carries data; bursts complete without overruns; the level rises with sound |

Build from this directory so `.cargo/config.toml` selects the target and
the product's linker scripts (the application region only):

```bash
cargo build --release --bin stm32wb5mm-dk-diag-sensors
```

Flash through HWLab only (`target flash program` on the ST-LINK probe),
then read the VCOM log (USART1, 115200 8N1). Lines are tagged
`[DK][<part>]`.

## Results on hardware (STM32WB5MM-DK, FUS 1.2.0, MAC 1.24.0)

| Binary | Result |
|---|---|
| rgb | All writes complete. The colours have not been checked by eye: no camera on the bench |
| lcd | Init 7.25 ms; full flush 1.205 ms, 30/30 Ok with LED writes in between. The image has not been checked by eye |
| sensors | ACK at 0x29, 0x38, 0x6B. STTS22H 20.1 °C; ISM330DHCX x=-34 y=-1018 z=19 mg; VL53L0X init Ok (33.8 ms budget), status 2 (no target in range) |
| touch | Baseline ~2450 counts, ±2 noise, 0.7 ms per acquisition; B1/B2 read released. A touch needs a person at the bench |
| mic | MSB-first byte lane 1; 0 overruns at 2.5 ms CPU per 1024-word chunk. Quiet room ~-73 dBFS; a 1 kHz tone from a nearby speaker ~-57.6 dBFS (+16 dB) for exactly its duration |

## Ownership notes

- SPI1 MOSI (PA7) is also the TLC59731 data line. `Spi1Bus` owns both and
  deselects the LCD before each LED write, so one owner serialises them.
- The microphone must stay clocked: its DC settles over seconds after power-up.
  The driver keeps SAI1 and circular DMA running and the reader drops stale
  data with `clear()` before each burst.
- The full-rate byte CIC costs about 50 % CPU if run continuously, so
  consumers meter short bursts (for example 8 chunks, about 41 ms, once per second).
