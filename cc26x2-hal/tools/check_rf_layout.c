/* Re-derive the RF core structure offsets pinned in src/rf/cmd.rs from the
 * TI headers, as GCC/clang lay them out for the Cortex-M4F.
 *
 * Needs a SimpleLink Low Power F2 SDK checkout (not vendored):
 *   clang --target=thumbv7em-none-eabihf -fsyntax-only \
 *     -I<sdk>/source/ti/devices/cc13x2_cc26x2 tools/check_rf_layout.c
 * A layout mismatch fails the _Static_assert.
 */
#include <stddef.h>
#include <stdint.h>
#include <driverlib/rf_common_cmd.h>
#include <driverlib/rf_ieee_cmd.h>
#include <driverlib/rf_ieee_mailbox.h>

#define AT(t, f, o) _Static_assert(offsetof(t, f) == (o), #t "." #f)
#define SIZE(t, n) _Static_assert(sizeof(t) == (n), "sizeof " #t)

AT(rfc_CMD_RADIO_SETUP_PA_t, mode, 14);
AT(rfc_CMD_RADIO_SETUP_PA_t, config, 16);
AT(rfc_CMD_RADIO_SETUP_PA_t, txPower, 18);
AT(rfc_CMD_RADIO_SETUP_PA_t, pRegOverride, 20);
AT(rfc_CMD_RADIO_SETUP_PA_t, pRegOverrideTx20, 28);
SIZE(rfc_CMD_RADIO_SETUP_PA_t, 32);
AT(rfc_CMD_SYNC_START_RAT_t, rat0, 16);
SIZE(rfc_CMD_SYNC_START_RAT_t, 20);
SIZE(rfc_CMD_FS_POWERDOWN_t, 16);
AT(rfc_CMD_IEEE_RX_t, channel, 14);
AT(rfc_CMD_IEEE_RX_t, rxConfig, 15);
AT(rfc_CMD_IEEE_RX_t, pRxQ, 16);
AT(rfc_CMD_IEEE_RX_t, pOutput, 20);
AT(rfc_CMD_IEEE_RX_t, frameFiltOpt, 24);
AT(rfc_CMD_IEEE_RX_t, frameTypes, 26);
AT(rfc_CMD_IEEE_RX_t, ccaOpt, 27);
AT(rfc_CMD_IEEE_RX_t, ccaRssiThr, 28);
AT(rfc_CMD_IEEE_RX_t, numExtEntries, 30);
AT(rfc_CMD_IEEE_RX_t, numShortEntries, 31);
AT(rfc_CMD_IEEE_RX_t, pExtEntryList, 32);
AT(rfc_CMD_IEEE_RX_t, pShortEntryList, 36);
AT(rfc_CMD_IEEE_RX_t, localExtAddr, 40);
AT(rfc_CMD_IEEE_RX_t, localShortAddr, 48);
AT(rfc_CMD_IEEE_RX_t, localPanID, 50);
AT(rfc_CMD_IEEE_RX_t, endTrigger, 55);
AT(rfc_CMD_IEEE_RX_t, endTime, 56);
/* sizeof is 64 (trailing u64 alignment padding); the RF core reads up to endTime. */
AT(rfc_CMD_IEEE_CSMA_t, randomState, 14);
AT(rfc_CMD_IEEE_CSMA_t, macMaxBE, 16);
AT(rfc_CMD_IEEE_CSMA_t, csmaConfig, 18);
AT(rfc_CMD_IEEE_CSMA_t, BE, 20);
AT(rfc_CMD_IEEE_CSMA_t, lastRssi, 22);
AT(rfc_CMD_IEEE_CSMA_t, endTrigger, 23);
AT(rfc_CMD_IEEE_CSMA_t, lastTimeStamp, 24);
AT(rfc_CMD_IEEE_CSMA_t, endTime, 28);
SIZE(rfc_CMD_IEEE_CSMA_t, 32);
AT(rfc_CMD_IEEE_TX_t, txOpt, 14);
AT(rfc_CMD_IEEE_TX_t, payloadLen, 15);
AT(rfc_CMD_IEEE_TX_t, pPayload, 16);
SIZE(rfc_CMD_IEEE_TX_t, 24);
AT(rfc_CMD_IEEE_RX_ACK_t, seqNo, 14);
AT(rfc_CMD_IEEE_RX_ACK_t, endTrigger, 15);
AT(rfc_CMD_IEEE_RX_ACK_t, endTime, 16);
AT(rfc_CMD_IEEE_ED_SCAN_t, channel, 14);
AT(rfc_CMD_IEEE_ED_SCAN_t, ccaRssiThr, 16);
AT(rfc_CMD_IEEE_ED_SCAN_t, maxRssi, 18);
AT(rfc_CMD_IEEE_ED_SCAN_t, endTrigger, 19);
AT(rfc_CMD_IEEE_ED_SCAN_t, endTime, 20);
SIZE(rfc_CMD_IEEE_MOD_SRC_MATCH_t, 4);
AT(rfc_CMD_CHANGE_PA_t, pRegOverride, 4);
AT(rfc_ieeeRxOutput_t, beaconTimeStamp, 12);
