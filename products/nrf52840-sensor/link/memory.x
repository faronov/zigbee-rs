MEMORY
{
  /* nRF52840: 1 MiB flash, 256 KiB RAM. */
  /* Product-owned: the last 8 KiB (pages 254-255) are reserved for the */
  /* crash-safe security journal (see src/storage.rs). Do not shrink this */
  /* reservation without verifying the journal's sector-rollover margin. */
  FLASH : ORIGIN = 0x00000000, LENGTH = 1016K
  RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
