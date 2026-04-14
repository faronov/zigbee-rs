/*
 * rail_shim.c — Minimal RAIL integration shim for EFR32MG1P
 *
 * Provides:
 * 1. Stub implementations for emlib functions RAIL depends on
 * 2. Wrapper functions callable from Rust via FFI
 * 3. RAIL event callback that signals Rust
 *
 * Only links the IEEE 802.15.4 subset of RAIL — BLE/WMBUS/HADM stubs are empty.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "rail.h"
#include "rail_ieee802154.h"

/* ── Hardware register definitions (no emlib headers needed) ─── */
#define CMU_BASE            0x400E4000UL
#define CMU_CTRL            (*(volatile uint32_t*)(CMU_BASE + 0x000))
#define CMU_STATUS          (*(volatile uint32_t*)(CMU_BASE + 0x01C))
#define CMU_HFCLKSEL        (*(volatile uint32_t*)(CMU_BASE + 0x058))
#define CMU_OSCENCMD        (*(volatile uint32_t*)(CMU_BASE + 0x060))
#define CMU_RADIOCLKEN0     (*(volatile uint32_t*)(CMU_BASE + 0x0C8))
#define CMU_HFXOSTEADYSTATE (*(volatile uint32_t*)(CMU_BASE + 0x014))

#define SYSTEM_CHIPREV_ADDR 0x0FE081FCUL

/* ── RAIL state ──────────────────────────────────────────────── */
static uint8_t rail_state_buf[512] __attribute__((aligned(4)));
static RAIL_Handle_t rail_handle = NULL;
static RAIL_Config_t rail_cfg;

/* Callbacks from Rust */
extern void rust_rail_event_callback(uint32_t events_lo, uint32_t events_hi);

/* RAIL event handler */
static void rail_event_cb(RAIL_Handle_t handle, RAIL_Events_t events) {
    (void)handle;
    rust_rail_event_callback((uint32_t)(events & 0xFFFFFFFF),
                             (uint32_t)((events >> 32) & 0xFFFFFFFF));
}

/* ── Public API (called from Rust via FFI) ───────────────────── */

RAIL_Handle_t rail_shim_init(void) {
    memset(&rail_cfg, 0, sizeof(rail_cfg));
    rail_cfg.eventsCallback = rail_event_cb;

    /* Disable interrupts during RAIL init to prevent premature IRQ firing */
    __asm volatile ("cpsid i" ::: "memory");

    rail_handle = RAIL_Init(&rail_cfg, NULL);
    if (!rail_handle) {
        __asm volatile ("cpsie i" ::: "memory");
        return NULL;
    }

    /* Set up TX FIFO — RAIL needs this before any TX/RX operations */
    static uint8_t tx_fifo[256] __attribute__((aligned(4)));
    RAIL_SetTxFifo(rail_handle, tx_fifo, 0, sizeof(tx_fifo));

    /* Re-enable interrupts */
    __asm volatile ("cpsie i" ::: "memory");

    return rail_handle;
}

uint8_t rail_shim_config_2p4ghz(void) {
    if (!rail_handle) return 0xFF;
    __asm volatile ("cpsid i" ::: "memory");
    uint8_t r = (uint8_t)RAIL_IEEE802154_Config2p4GHzRadio(rail_handle);
    __asm volatile ("cpsie i" ::: "memory");
    return r;
}

uint8_t rail_shim_ieee802154_init(void) {
    if (!rail_handle) return 0xFF;

    /* Enable RX/TX events BEFORE IEEE802154 init */
    RAIL_ConfigEvents(rail_handle,
        RAIL_EVENTS_ALL,
        RAIL_EVENT_TX_PACKET_SENT
        | RAIL_EVENT_RX_PACKET_RECEIVED
        | RAIL_EVENT_RX_FRAME_ERROR
        | RAIL_EVENT_RX_FIFO_OVERFLOW
        | RAIL_EVENT_TX_UNDERFLOW
        | RAIL_EVENT_CAL_NEEDED);

    RAIL_IEEE802154_Config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.addresses = NULL;
    cfg.ackConfig.enable = false;
    cfg.ackConfig.ackTimeout = 672; /* Default for 802.15.4 */
    cfg.timings.idleToRx = 100;
    cfg.timings.txToRx = 182;
    cfg.timings.idleToTx = 100;
    cfg.timings.rxToTx = 192;
    cfg.framesMask = RAIL_IEEE802154_ACCEPT_STANDARD_FRAMES;
    cfg.promiscuousMode = true;
    cfg.isPanCoordinator = false;
    cfg.defaultFramePendingInOutgoingAcks = false;
    return (uint8_t)RAIL_IEEE802154_Init(rail_handle, &cfg);
}

uint8_t rail_shim_start_rx(uint16_t channel) {
    if (!rail_handle) return 0xFF;
    return (uint8_t)RAIL_StartRx(rail_handle, channel, NULL);
}

uint8_t rail_shim_start_tx(uint16_t channel) {
    if (!rail_handle) return 0xFF;
    return (uint8_t)RAIL_StartTx(rail_handle, channel, 0, NULL);
}

uint16_t rail_shim_rx_get(uint8_t *buf, uint16_t max_len) {
    if (!rail_handle) return 0;
    RAIL_RxPacketInfo_t info;
    RAIL_RxPacketHandle_t pkt = RAIL_GetRxPacketInfo(rail_handle,
                                    RAIL_RX_PACKET_HANDLE_NEWEST, &info);
    if (pkt == RAIL_RX_PACKET_HANDLE_INVALID) return 0;
    uint16_t len = info.packetBytes;
    if (len > max_len) len = max_len;
    RAIL_CopyRxPacket(buf, &info);
    RAIL_ReleaseRxPacket(rail_handle, pkt);
    return len;
}

int8_t rail_shim_get_rssi(void) {
    if (!rail_handle) return -128;
    return (int8_t)(RAIL_GetRssi(rail_handle, false) / 4);
}

uint16_t rail_shim_tx_load(const uint8_t *data, uint16_t len) {
    if (!rail_handle) return 0xFFFF;
    return RAIL_WriteTxFifo(rail_handle, data, len, true);
}

RAIL_Handle_t rail_shim_get_handle(void) {
    return rail_handle;
}

/* ── emlib stubs ─────────────────────────────────────────────── */
/* These are minimal implementations — just enough for RAIL to work */

void CMU_ClockEnable(uint32_t clock, bool enable) {
    (void)clock;
    (void)enable;
    /* Radio clocks are already enabled by Rust init */
}

uint32_t CMU_ClockFreqGet(uint32_t clock) {
    (void)clock;
    return 38400000; /* HFXO = 38.4 MHz */
}

uint32_t CMU_ClockSelectGet(uint32_t clock) {
    (void)clock;
    return 4; /* cmuSelect_HFXO */
}

void CMU_OscillatorEnable(uint32_t osc, bool enable, bool wait) {
    (void)osc; (void)enable; (void)wait;
    /* HFXO already enabled by Rust init */
}

uint32_t CORE_EnterAtomic(void) {
    uint32_t primask;
    __asm volatile ("MRS %0, primask" : "=r" (primask));
    __asm volatile ("CPSID i" ::: "memory");
    return primask;
}

void CORE_ExitAtomic(uint32_t state) {
    if (state == 0) {
        __asm volatile ("CPSIE i" ::: "memory");
    }
}

uint32_t CORE_EnterCritical(void) {
    return CORE_EnterAtomic();
}

void CORE_ExitCritical(uint32_t state) {
    CORE_ExitAtomic(state);
}

void EMU_DCDCLnRcoBandSet(uint32_t band) { (void)band; }

void GPIO_PinModeSet(uint32_t port, uint32_t pin, uint32_t mode, uint32_t out) {
    (void)port; (void)pin; (void)mode; (void)out;
}

void LDMA_StartTransfer(int ch, void *xfer, void *desc) {
    (void)ch; (void)xfer; (void)desc;
}

bool LDMA_TransferDone(int ch) {
    (void)ch;
    return true;
}

int PRS_GetFreeChannel(uint32_t type) {
    (void)type;
    return 0;
}

void PRS_SourceSignalSet(uint32_t ch, uint32_t source, uint32_t signal, uint32_t edge) {
    (void)ch; (void)source; (void)signal; (void)edge;
}

void __attribute__((weak)) PTI_AuxdataOutput(uint32_t data) { (void)data; }

/* Power manager stubs */
void sl_power_manager_subscribe_em_transition_event(void *h) { (void)h; }
void sl_power_manager_unsubscribe_em_transition_event(void *h) { (void)h; }
void sli_power_manager_update_em_requirement(uint32_t from, uint32_t to) {
    (void)from; (void)to;
}
void sli_power_manager_debug_log_em_requirement(uint32_t em, void *req, const char *s) {
    (void)em; (void)req; (void)s;
}

/* Sleeptimer stubs */
uint32_t sl_sleeptimer_get_tick_count(void) { return 0; }
uint32_t sl_sleeptimer_get_timer_frequency(void) { return 32768; }
int32_t sl_sleeptimer_start_timer(void *h, uint32_t t, void *cb, void *d, uint8_t p, uint16_t f) {
    (void)h; (void)t; (void)cb; (void)d; (void)p; (void)f;
    return 0;
}
int32_t sl_sleeptimer_stop_timer(void *handle) { (void)handle; return 0; }
uint32_t sl_sleeptimer_tick_to_ms(uint32_t tick) { return tick * 1000 / 32768; }

/* CMU internal stubs */
void sli_em_cmu_HFClockSelectHFRCO(void) {}
void sli_em_cmu_HFClockSelectHFXO(void) {}

/* System stubs */
typedef struct { uint8_t major; uint8_t minor; uint8_t family; uint8_t patch; } SYSTEM_ChipRevision_TypeDef;
void SYSTEM_ChipRevisionGet(SYSTEM_ChipRevision_TypeDef *rev) {
    volatile uint32_t *devinfo = (volatile uint32_t *)0x0FE081FCUL;
    uint32_t val = *devinfo;
    rev->family = (val >> 16) & 0xFF;
    rev->major = (val >> 8) & 0xFF;
    rev->minor = val & 0xFF;
    rev->patch = 0;
}

uint32_t SystemHFClockGet(void) { return 38400000; }
uint32_t SystemHFXOClockGet(void) { return 38400000; }
uint32_t SystemLFRCOClockGet(void) { return 32768; }
uint32_t SystemLFXOClockGet(void) { return 32768; }
uint32_t SystemULFRCOClockGet(void) { return 1000; }

/* BLE/HADM/WMBUS stubs — never called for 802.15.4 */
void RFHAL_ConfigBleHadm(void *a, void *b) { (void)a; (void)b; }
void RFHAL_ConfigHadmAntenna(void *a) { (void)a; }
void RFHAL_EnableBleHadm(void *a, void *b) { (void)a; (void)b; }
void RFHAL_LoadHadmCompTables(void *a) { (void)a; }
void RFHAL_PrepareNextHadmStep(void *a) { (void)a; }
void RFHAL_WMBUS_Config(void *a) { (void)a; }

/* RAIL assert callback */
void RAILCb_AssertFailed(RAIL_Handle_t handle, RAIL_AssertErrorCodes_t code) {
    (void)handle;
    /* Store error code in a known RAM location for debugging */
    volatile uint32_t *err = (volatile uint32_t *)0x20007FF0;
    *err = 0xDEAD0000 | code;
    while(1);
}

/* ── IRQ vector name wrappers ────────────────────────────────── */
/* cortex-m-rt uses short names (FRC_PRI, FRC, etc.) but RAIL library
 * exports handlers with _IRQHandler suffix. Bridge them here. */

extern void FRC_PRI_IRQHandler(void);
extern void FRC_IRQHandler(void);
extern void RAC_SEQ_IRQHandler(void);
extern void BUFC_IRQHandler(void);
extern void MODEM_IRQHandler(void);
extern void AGC_IRQHandler(void);
extern void PROTIMER_IRQHandler(void);

void FRC_PRI(void) { FRC_PRI_IRQHandler(); }
void FRC(void)     { FRC_IRQHandler(); }
void RAC_SEQ(void) { RAC_SEQ_IRQHandler(); }
void BUFC(void)    { BUFC_IRQHandler(); }
void MODEM(void)   { MODEM_IRQHandler(); }
void AGC(void)     { AGC_IRQHandler(); }
void PROTIMER(void){ PROTIMER_IRQHandler(); }
