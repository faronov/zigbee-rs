MEMORY
{
    /* Bare-metal boot at 0x0 (no bootloader) for JLink flashing. */
    FLASH : ORIGIN = 0x00000000, LENGTH = 256K
    RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}
