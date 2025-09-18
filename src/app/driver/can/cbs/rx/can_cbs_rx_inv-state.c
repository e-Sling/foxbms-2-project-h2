/**
 * @file    can_cbs_rx_inv-state.c
 * @author  CELLSIUS Project H2
 * @brief   CAN Rx callback for Inverter State
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
#define CANRX_INV_STATE_PRECHARGE_ALLOWED_START_BIT (20u)
#define CANRX_INV_STATE_PRECHARGE_ALLOWED_LENGTH    (CAN_BIT)
#define CANRX_INV_STATE_DIRECT_CONNECT_START_BIT    (21u)
#define CANRX_INV_STATE_DIRECT_CONNECT_LENGTH       (CAN_BIT)
#define CANRX_INV_STATE_CRC_START_BIT               (56u)
#define CANRX_INV_STATE_CRC_LENGTH                  (8u)
/** @} */

/*========== Static Constant and Variable Definitions =======================*/

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/
/**
 * @brief   sets the precharge allowed flag
 * @param[in] messageData contents of the inv state message
 */
static void CANRX_SetPrechargeAllowedFlag(uint64_t messageData);

/**
 * @brief   sets the direct connect flag
 * @param[in] messageData contents of the inv state message
 */
static void CANRX_SetDirectConnectFlag(uint64_t messageData);

/*========== Static Function Implementations ================================*/

static void CANRX_SetPrechargeAllowedFlag(uint64_t messageData) {
    uint64_t signalData = 0u;
    CAN_RxGetSignalDataFromMessageData(
        messageData,
        CANRX_INV_STATE_PRECHARGE_ALLOWED_START_BIT,
        CANRX_INV_STATE_PRECHARGE_ALLOWED_LENGTH,
        &signalData,
        CANRX_INV_STATE_ENDIANNESS);

    BMS_SetPrechargeAllowedFlag((bool)signalData);
}

static void CANRX_SetDirectConnectFlag(uint64_t messageData) {
    uint64_t signalData = 0u;
    CAN_RxGetSignalDataFromMessageData(
        messageData,
        CANRX_INV_STATE_DIRECT_CONNECT_START_BIT,
        CANRX_INV_STATE_DIRECT_CONNECT_LENGTH,
        &signalData,
        CANRX_INV_STATE_ENDIANNESS);

    BMS_SetDirectConnectFlag((bool)signalData);
}

/*========== Extern Function Implementations ================================*/
extern uint32_t CANRX_InverterState(
    CAN_MESSAGE_PROPERTIES_s message,
    const uint8_t *const kpkCanData,
    const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(message.id == CANRX_INV_STATE_ID);
    FAS_ASSERT(message.idType == CANRX_INV_STATE_ID_TYPE);
    FAS_ASSERT(message.dlc == CANRX_INV_STATE_DLC);
    FAS_ASSERT(message.endianness == CANRX_INV_STATE_ENDIANNESS);
    FAS_ASSERT(kpkCanData != NULL_PTR);
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    uint64_t messageData = 0u;
    /* Get message as big endian for CRC calculation */
    CAN_RxGetMessageDataFromCanData(&messageData, kpkCanData, CAN_BIG_ENDIAN);
    uint8_t crc =
        Compute_RX_CRC8H2F((uint8_t *)&messageData, CANRX_INV_STATE_CRC_START_BIT / 8u, CRC8H2F_INITIAL_VALUE);

    CAN_RxGetMessageDataFromCanData(&messageData, kpkCanData, CANRX_INV_STATE_ENDIANNESS);
    uint64_t crc_received = 0u;
    CAN_RxGetSignalDataFromMessageData(
        messageData,
        CANRX_INV_STATE_CRC_START_BIT,
        CANRX_INV_STATE_CRC_LENGTH,
        &crc_received,
        CANRX_INV_STATE_ENDIANNESS);

    if (crc == (uint8_t)crc_received) {
        /* Set Precharge Allowed Flag */
        CANRX_SetPrechargeAllowedFlag(messageData);

        /* Set Direct Connect Flag */
        CANRX_SetDirectConnectFlag(messageData);

        /* Save tick from this message */
        BMS_SetLastInverterTick();
    }

    return 0u;
}
