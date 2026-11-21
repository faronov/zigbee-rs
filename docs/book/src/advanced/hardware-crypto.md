# Hardware Crypto

Zigbee security needs only the forward AES-128 block permutation: CCM\*
(NWK/APS frames, Green Power), AES-MMO and HMAC-MMO (key-transport and
key-load keys, Verify-Key hashes, install codes). Every production image runs
these on the chip's AES engine. Software AES (the RustCrypto `aes` fixslice
core behind `SoftwareAes128`) remains only for host tests and the mock
simulators.

## One path for every AES user

```text
NWK/APS CCM*, AES-MMO, install codes, Trust Center Verify-Key, Green Power
        |
ForwardAesProvider::forward_cipher(key)   <- implemented by the MAC/PHY
        |
platform HardwareAes128 / engine driver   <- owns the peripheral (token)
```

- `PlatformServices: ForwardAesProvider`, so the MAC is the provider. The
  protocol crates call only the `*_with(provider, ..)` functions.
- The Trust Center table (`TrustCenterTable::handle_verify_key`,
  `provision_install_code`) and `BdbLayer::set_install_code` take the provider
  as well. Before this, coordinator and NCP images linked the software core
  through these two paths even when their CCM\* ran on hardware (ESP32 NCP:
  5.8 KB of flash).
- The software wrappers (`ccm_star_encrypt`, `derive_key_transport_key`, ...)
  are host/test conveniences. Production code must not call them; the CI
  symbol gates below catch it if it does.

## Engine contract

Each backend follows the same rules:

- The chip HAL (or the MAC backend, where the radio driver lives) owns the
  peripheral through a one-time token. The composition root constructs the
  engine and calls `install_aes_engine(..)`.
- Installation runs a known-answer test. A failure stops the image before
  Zigbee starts; there is no fall-back to software.
- Every hardware wait is bounded. Errors (timeout, engine error flag, lost
  clock, ownership conflict) propagate as `Err`; the stack drops the frame.
  Partial output is never published.
- If the engine's clock or power domain may have been lost, the engine
  revalidates with the KAT before it returns any result.
- A host model of the register protocol passes `zigbee_crypto::kat::run_all`
  (feature `kat`): FIPS-197/SP 800-38A blocks, IEEE 802.15.4-2006 Annex C.2.1
  and C.2.3 CCM\* frames (reference CCM\* with MIC-64), Zigbee `M = 4` CCM\*
  frames through the production path, and the install-code AES-MMO vector.

## Platform status

Validation levels: *build* = all CI images build and pass the symbol gate;
*host* = engine model passes the KATs and fail-closed tests; *silicon* =
startup KAT passed on the chip; *ZHA* = secured join (unique TCLK +
Verify-Key), interview, reports and reset/resume proven with this engine.
Rows marked ZHA for nRF, ESP, TLSR8258 and EFR32MG1 refer to the earlier lab
runs recorded in their platform guides.

| Platform | Engine | Ownership notes | Status |
|---|---|---|---|
| nRF52840 / nRF52833 | ECB | CPU only; radio CCM block unused | ZHA |
| ESP32-H2 / C6 / C5 | AES accelerator | CPU only | ZHA |
| ESP32 NCP | AES accelerator | Trust Center path moved to the provider | build |
| TLSR8258 | AES-128 ECB | CPU only | ZHA |
| BL702 (sensor, ED, router) | SEC_ENG AES | CPU only; the parent PHY delegates to the wrapped radio's engine | silicon KAT + secured ZHA join (sensor); silent reset/resume pending |
| EFR32MG1 | CRYPTO | Gecko Bootloader may gate CRYPTO: clock restore re-runs the KAT | ZHA |
| EFR32MG21 / ZBT-1 | Secure Engine (HSE) mailbox, `AES_ENCRYPT` ECB | mailbox shared with the Gecko Bootloader API (sequential only); a timed-out command disables the engine until reset | host, build |
| EFR32MG22 | CRYPTOACC (BA411E) with descriptor DMA | clocks shared with the CRYPTOACC TRNG through a holder count | ZHA |
| EFR32MG24 | Secure Engine (HSE) mailbox | mailbox and disable flag shared with the SE TRNG | host, build |
| STM32WB55 | AES1 (CPU1) | AES2 belongs to CPU2; AES1 has no HSEM and CPU2 never uses it (MAC requests are unsecured) | ZHA |
| CC2340R5 | LAES | radio (LRF) does not use it; registers reloaded per block | host, build |
| CC26x2 / CC13x2 | CRYPTO | see the CC26x2 guide | see guide |
| PHY6222 / PHY6252 | LL AES block, single (ECB) mode | BLE LL not used; clock gated per block; completion is a fixed cycle wait until the status bit is measured | host, build |
| Mock simulators | software | host only | — |

No supported chip lacks a usable AES engine, so no production image keeps
software AES.

## CI symbol gates

Each platform's build job runs `llvm-nm --demangle` on every production and
role image and fails when the image contains `aes::soft`, `SoftwareAes128`
(or, for the NCP, `SoftwareAesProvider`), or when the platform's hardware
backend symbol is missing.

## Size

Replacing the software core with a register driver shrinks every image:
about 0.8–2.9 KB of flash per image, and 5.8 KB for the ESP32 NCP. RAM grows
only where the engine needs a static DMA area (EFR32 Series 2: 96–116 B) or
an owner slot (PHY62x2 End Device, CC2340: 8–16 B). The per-image tables are in
the platform guides.
