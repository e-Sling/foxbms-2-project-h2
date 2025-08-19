/**
 * @file    can_cbs_rx_ecu-state.c
 * @author  CELLSIUS Project H2
 * @brief   CAN Rx callback for ECU State
 */

/*========== Includes =======================================================*/
#include "bms.h"
#include "can_cbs_rx.h"
#include "can_cfg_rx-message-definitions.h"
#include "can_helper.h"

#include <stdint.h>

/*========== Macros and Definitions =========================================*/
/**
 * @brief   CAN state request update time
 * @details When a new CAN state request is received, it leads to an update
 *          of #DATA_BLOCK_STATE_REQUEST_s::stateRequestViaCan if one of the
 *          following conditions is met:
 *
 *             - The new request is different than the old request.
 *             - The old request is older than the time span set in this define.
 */
#define CANRX_CAN_REQUEST_UPDATE_TIME_ms (3000u)

/** @{
 * defines for the state request signal data
 */
#define CANRX_ECU_STATE_FAULT_DISARM_START_BIT (33u)
#define CANRX_ECU_STATE_FAULT_DISARM_LENGTH    (CAN_BIT)
#define CANRX_ECU_STATE_CRC_START_BIT          (48u)
#define CANRX_ECU_STATE_CRC_LENGTH             (8u)
/** @} */

/*========== Static Constant and Variable Definitions =======================*/

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/
/**
 * @brief   sets the fault disarm flag
 * @param[in] messageData contents of the ecu state message
 */
static void CANRX_SetFaultDisarmFlag(uint64_t messageData);

/*========== Static Function Implementations ================================*/

static void CANRX_SetFaultDisarmFlag(uint64_t messageData) {
    uint64_t signalData = 0u;
    CAN_RxGetSignalDataFromMessageData(
        messageData,
        CANRX_ECU_STATE_FAULT_DISARM_START_BIT,
        CANRX_ECU_STATE_FAULT_DISARM_LENGTH,
        &signalData,
        CANRX_BMS_STATE_REQUEST_ENDIANNESS);

    BMS_SetFaultDisarmFlag((bool)signalData);
}

/*========== Extern Function Implementations ================================*/
extern uint32_t CANRX_EcuStateRequest(
    CAN_MESSAGE_PROPERTIES_s message,
    const uint8_t *const kpkCanData,
    const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(message.id == CANRX_ECU_STATE_REQUEST_ID);
    FAS_ASSERT(message.idType == CANRX_ECU_STATE_REQUEST_ID_TYPE);
    FAS_ASSERT(message.dlc == CANRX_ECU_STATE_REQUEST_DLC);
    FAS_ASSERT(message.endianness == CANRX_ECU_STATE_REQUEST_ENDIANNESS);
    FAS_ASSERT(kpkCanData != NULL_PTR);
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    uint64_t messageData = 0u;
    CAN_RxGetMessageDataFromCanData(&messageData, kpkCanData, CANRX_ECU_STATE_REQUEST_ENDIANNESS);

    /* Set Fault Disarm Flag */
    CANRX_SetFaultDisarmFlag(messageData);

    return 0u;
}
