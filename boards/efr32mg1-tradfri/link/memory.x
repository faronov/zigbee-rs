MEMORY
{
    /* Preserve the resident Gecko bootloader at 0x0000..0x3FFF. */
    /* Reserve 0x37000..0x38FFF for the crash-safe security journal. */
    /* Reserve 0x39000..0x39FFF for two Rust application-NV pages. */
    /* Preserve the native firmware's 24 KiB NVM3 at 0x3A000..0x3FFFF. */
    FLASH : ORIGIN = 0x00004000, LENGTH = 0x00033000
    /* EFR32MG1P132F256 exposes 31 KiB of SRAM, not 32 KiB. */
    RAM   : ORIGIN = 0x20000000, LENGTH = 0x00007C00
}
