MEMORY
{
    /* Bare-metal boot at 0x0 (no bootloader).
       For Gecko Bootloader: change to ORIGIN=0x4000, LENGTH=240K
       and flash bootloader at 0x0000-0x3FFF. */
    FLASH : ORIGIN = 0x00000000, LENGTH = 256K
    RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}
