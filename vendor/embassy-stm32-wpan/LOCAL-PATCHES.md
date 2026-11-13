# Local STM32WB MAC ABI patches

Baseline: `embassy-rs/embassy` commit
`7419ec0700f23058e90285a56c62ecd2c0a39b87`, directory `embassy-stm32-wpan`.
The upstream MIT/Apache-2.0 licenses remain applicable. The only manifest
change rewrites the sibling `path` dependencies to the same git revision so
the crate can live outside the embassy tree; the zigbee-rs STM32WB manifests
select this copy with `[patch."https://github.com/embassy-rs/embassy"]`.

Layout reference: ST `802_15_4_mac_types.h` from
`STMicroelectronics/stm32-mw-wpan` commit
`1a66948ee871e7665712aa53dd3cbb9a71c4985a`
(`mac_802_15_4/core/inc`), compiled with `arm-none-eabi-gcc -mcpu=cortex-m4`.
CPU2 runs `stm32wb5x_Mac_802_15_4_fw.bin` 1.24.0 on the board this was proven
on. `stm32wb-hal/src/st_abi.rs` asserts every size and public field offset of
the MAC SAP structures against that reference at compile time.

## Patches

- `src/net/typedefs.rs`: `MacAddress` (`MAC_addr_t`) is `#[repr(C)]`;
  `PanDescriptor` (`MAC_PAN_Desc_t`) is `#[repr(C)]` and gains ST's trailing
  `a_stuffing: [u8; 2]`. Upstream's descriptor was 22 octets instead of 24, so
  `ScanConfirm` was 173 octets instead of CPU2's 185 and the beacon-notify
  `addr_list`/`bsn`/`sdu_length` fields were read two octets early.
- `src/net/commands.rs`: `StartRequest` (`MAC_startReq_t`) gains the missing
  `coord_realign_key_id_mode` field; upstream placed every later field one
  octet early and sent 34 octets instead of 35.
- `src/wb/sub/mac.rs`: `read()` acknowledges a notification that fails to
  parse before returning the error. Upstream returned `Err` without releasing
  the IPCC MAC notification channel, so the first size mismatch (the 185-octet
  scan confirm) wedged the mailbox permanently. Parsing and writing are split
  into `parse`/`write_packet` helpers; behaviour on success is unchanged.

Upstream `main` (checked while vendoring) still has the `PanDescriptor`
defect. Drop this copy once an embassy release carries equivalent fixes and
`st_abi.rs` passes against it.

Hardware evidence: before the patch the coprocessor answered the first active
scan with a 185-octet confirm and the CPU1 mailbox stalled (reset loop); with
the patch the STM32WB5MM-DK parsed nine PAN 0xDFE9 beacons, associated, and
was interviewed by ZHA as a router.
