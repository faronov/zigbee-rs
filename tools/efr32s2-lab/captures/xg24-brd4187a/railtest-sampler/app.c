/* Lab-only RAIL transition sampler (zigbee-rs xG24 capture, not shipped).
 *
 * Write a non-zero mode to `zrs_sampler.request` over SWD. The main loop
 * then starts one RAIL transmission of the frame RAILtest has loaded
 * (`setTxPayload`/`setTxLength`) and samples radio registers in a tight
 * loop, recording an entry only when one of them changes, until the RSM
 * has been back in OFF/RX for a while or the buffer is full. Read
 * `zrs_sampler` back over SWD.
 */
#include <stdint.h>
#include <string.h>
#include "app.h"
#include "rail.h"
#include "em_device.h"

extern RAIL_Handle_t railHandle;
extern uint16_t channel;

#define NREGMAX 16
#define NENT 400
/* Default set: the PA power-up path, ~7 us per sample. */
static const uint32_t regs_default[] = {
  0xB802000C, /* RAC.STATUS */
  0xB802016C, /* RAC.TX */
  0xB80201C8, /* RAC.PACTRL */
  0xB80200B8, /* RAC.PAENCTRL */
  0xB80200AC, /* RAC.RFSTATUS */
  0xB80141B0, /* MODEM.PADEBUG */
  0xB80140DC, /* MODEM.RAMPLEV */
};

typedef struct {
  uint32_t cycles;
  uint32_t v[NREGMAX];
} entry_t;

typedef struct {
  volatile uint32_t magic;
  volatile uint32_t request;
  volatile uint32_t count;
  volatile uint32_t status;
  volatile uint32_t nreg;
  volatile uint32_t max_iter;
  volatile uint32_t quiet_lim;
  volatile uint32_t iters;
  uint32_t regs[NREGMAX];
  entry_t e[NENT];
} sampler_t;

__attribute__((used)) sampler_t zrs_sampler = { .magic = 0x5A525332u };

void app_init(void)
{
  memcpy(zrs_sampler.regs, regs_default, sizeof regs_default);
  zrs_sampler.nreg = sizeof regs_default / sizeof regs_default[0];
  zrs_sampler.max_iter = 3000u;
  zrs_sampler.quiet_lim = 1500u;
}

void app_process_action(void)
{
  if (zrs_sampler.request == 0u) {
    return;
  }
  uint32_t nreg = zrs_sampler.nreg;
  if (nreg > NREGMAX) {
    nreg = NREGMAX;
  }
  uint32_t last[NREGMAX];
  uint32_t n = 0;
  for (uint32_t i = 0; i < NREGMAX; i++) {
    last[i] = 0xFFFFFFFFu;
  }
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  zrs_sampler.status = RAIL_StartTx(railHandle, channel, RAIL_TX_OPTIONS_DEFAULT, NULL);
  uint32_t t0 = DWT->CYCCNT;
  uint32_t quiet = 0;
  uint32_t it = 0;
  while (n < NENT && quiet < zrs_sampler.quiet_lim && it < zrs_sampler.max_iter) {
    uint32_t v[NREGMAX];
    for (uint32_t i = 0; i < nreg; i++) {
      v[i] = *(volatile uint32_t *)zrs_sampler.regs[i];
    }
    int changed = 0;
    for (uint32_t i = 0; i < nreg; i++) {
      if (v[i] != last[i]) {
        changed = 1;
        last[i] = v[i];
      }
    }
    it++;
    if (changed) {
      zrs_sampler.e[n].cycles = DWT->CYCCNT - t0;
      memcpy(zrs_sampler.e[n].v, v, nreg * 4u);
      n++;
      quiet = 0;
    } else {
      quiet++;
    }
  }
  zrs_sampler.iters = it;
  zrs_sampler.count = n;
  zrs_sampler.request = 0u;
}
