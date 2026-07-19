MEMORY
{
    /* Bare-metal boot at 0x0 (no bootloader) for JLink flashing. */
    /* 4 KiB at 0x3E000 is reserved for application NV; 0x3F000 remains unused. */
    FLASH : ORIGIN = 0x00000000, LENGTH = 248K
    RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}
