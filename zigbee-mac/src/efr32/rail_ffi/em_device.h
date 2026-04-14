/*
 * Minimal em_device.h for EFR32MG1P232F256GM48
 * Just enough defines for RAIL library headers.
 */
#ifndef EM_DEVICE_H
#define EM_DEVICE_H

/* Silicon Labs series/config identifiers */
#define _SILICON_LABS_32B_SERIES          1
#define _SILICON_LABS_32B_SERIES_1_CONFIG 1
#define _SILICON_LABS_EFR32_RADIO_TYPE    _SILICON_LABS_EFR32_RADIO_2G4HZ
#define _SILICON_LABS_EFR32_RADIO_2G4HZ  1
#define _SILICON_LABS_EFR32_RADIO_SUBGHZ 2
#define _SILICON_LABS_EFR32_RADIO_DUALBAND 3

/* EFR32MG1P identification */
#define EFR32MG1P232F256GM48 1
#define _EFR32_MIGHTY_FAMILY 1

/* Flash/RAM sizes */
#define FLASH_SIZE   0x00040000UL  /* 256 KB */
#define SRAM_SIZE    0x00008000UL  /* 32 KB */
#define FLASH_BASE   0x00000000UL
#define SRAM_BASE    0x20000000UL

/* Cortex-M4 */
#define __CM4_REV              0x0001
#define __MPU_PRESENT          1
#define __FPU_PRESENT          1
#define __VTOR_PRESENT         1
#define __NVIC_PRIO_BITS       3

/* Interrupt numbers (for NVIC) */
typedef enum {
    FRC_PRI_IRQn  = 4,
    FRC_IRQn      = 5,
    RAC_SEQ_IRQn  = 6,
    RAC_RSM_IRQn  = 7,
    BUFC_IRQn     = 8,
    MODEM_IRQn    = 9,
    AGC_IRQn      = 10,
    PROTIMER_IRQn = 11,
} IRQn_Type;

/* NVIC functions (minimal) */
static inline void NVIC_ClearPendingIRQ(IRQn_Type irq) {
    volatile uint32_t *icpr = (volatile uint32_t *)0xE000E280;
    *icpr = (1UL << (uint32_t)irq);
}

static inline void NVIC_EnableIRQ(IRQn_Type irq) {
    volatile uint32_t *iser = (volatile uint32_t *)0xE000E100;
    *iser = (1UL << (uint32_t)irq);
}

static inline void NVIC_DisableIRQ(IRQn_Type irq) {
    volatile uint32_t *icer = (volatile uint32_t *)0xE000E180;
    *icer = (1UL << (uint32_t)irq);
}

static inline void NVIC_SetPriority(IRQn_Type irq, uint32_t priority) {
    volatile uint8_t *ipr = (volatile uint8_t *)0xE000E400;
    ipr[(uint32_t)irq] = (uint8_t)(priority << (8 - __NVIC_PRIO_BITS));
}

static inline void __NVIC_ClearPendingIRQ(IRQn_Type irq) { NVIC_ClearPendingIRQ(irq); }
static inline void __NVIC_EnableIRQ(IRQn_Type irq) { NVIC_EnableIRQ(irq); }
static inline void __NVIC_DisableIRQ(IRQn_Type irq) { NVIC_DisableIRQ(irq); }

/* Interrupt enable/disable */
static inline void __enable_irq(void)  { __asm volatile ("cpsie i" ::: "memory"); }
static inline void __disable_irq(void) { __asm volatile ("cpsid i" ::: "memory"); }

#endif /* EM_DEVICE_H */
