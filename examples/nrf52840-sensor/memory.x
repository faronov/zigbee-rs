MEMORY
{
  /* nRF52840 : 1 MB Flash, 256 KB RAM */
  /* Last 8 KB (pages 254-255) reserved for NV storage */
  FLASH : ORIGIN = 0x00000000, LENGTH = 1016K
  RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
