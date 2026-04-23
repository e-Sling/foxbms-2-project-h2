/**
 * @file    can_cbs_rx_dhvc-state.c
 * @author  CELLSIUS Project H2
 * @brief   CAN Rx callback for DHVC State
 */

/*========== Includes =======================================================*/
#include "bms.h"
#include "can_cbs_rx.h"
#include "can_cfg_rx-message-definitions.h"
#include "can_helper.h"

#include <stdint.h>

/*========== Macros and Definitions =========================================*/
/** @{
 * defines for the state request signal data
 */
#define CANRX_DHVC_STATE_ALLOW_HV_START_BIT (19u)
#define CANRX_DHVC_STATE_ALLOW_HV_LENGTH    (CAN_BIT)
#define CANRX_DHVC_STATE_CRC_START_BIT      (48u)
#define CANRX_DHVC_STATE_CRC_LENGTH         (8u)
/** @} */

/*========== Static Constant and Variable Definitions =======================*/

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/
/**
 * @brief sets Allow HV
 * @param[in] messageData contents of the dhvc state message
 */
static void CANRX_SetAllowHV(uint64_t messageData);

/*========== Static Function Implementations ================================*/

static void CANRX_SetAllowHV(uint64_t messageData) {
    uint64_t signalData = 0u;
    CAN_RxGetSignalDataFromMessageData(
        messageData,
        CANRX_DHVC_STATE_ALLOW_HV_START_BIT,
        CANRX_DHVC_STATE_ALLOW_HV_LENGTH,
        &signalData,
        CANRX_DHVC_STATE_ENDIANNESS);

    BMS_SetAllowHV((bool)signalData);
}

/*========== Extern Function Implementations ================================*/
extern uint32_t CANRX_DhvcState(
    CAN_MESSAGE_PROPERTIES_s message,
    const uint8_t *const kpkCanData,
    const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(message.id == CANRX_DHVC_STATE_ID);
    FAS_ASSERT(message.idType == CANRX_DHVC_STATE_ID_TYPE);
    FAS_ASSERT(message.dlc == CANRX_DHVC_STATE_DLC);
    FAS_ASSERT(message.endianness == CANRX_DHVC_STATE_ENDIANNESS);
    FAS_ASSERT(kpkCanData != NULL_PTR);
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    uint64_t messageData = 0u;
    /* Get message as big endian for CRC calculation */
    CAN_RxGetMessageDataFromCanData(&messageData, kpkCanData, CAN_BIG_ENDIAN);
    uint8_t crc =
        Compute_RX_CRC8H2F((uint8_t *)&messageData, CANRX_DHVC_STATE_CRC_START_BIT / 8u, CRC8H2F_INITIAL_VALUE);

    CAN_RxGetMessageDataFromCanData(&messageData, kpkCanData, CANRX_DHVC_STATE_ENDIANNESS);
    uint64_t crc_received = 0u;
    CAN_RxGetSignalDataFromMessageData(
        messageData,
        CANRX_DHVC_STATE_CRC_START_BIT,
        CANRX_DHVC_STATE_CRC_LENGTH,
        &crc_received,
        CANRX_DHVC_STATE_ENDIANNESS);

    if (crc == (uint8_t)crc_received) {
        /* Set Allow HV */
        CANRX_SetAllowHV(messageData);

        /* Save tick from this message */
        BMS_SetLastDHVCTick();
    }

    return 0u;
}
