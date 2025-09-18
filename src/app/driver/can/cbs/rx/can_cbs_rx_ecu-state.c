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
/** @{
 * defines for the state request signal data
 */
#define CANRX_ECU_STATE_FLIGHTMODE_START_BIT   (32u)
#define CANRX_ECU_STATE_FLIGHTMODE_LENGTH      (CAN_BIT)
#define CANRX_ECU_STATE_FAULT_DISARM_START_BIT (33u)
#define CANRX_ECU_STATE_FAULT_DISARM_LENGTH    (CAN_BIT)
#define CANRX_ECU_STATE_CRC_START_BIT          (48u)
#define CANRX_ECU_STATE_CRC_LENGTH             (8u)
/** @} */

/*========== Static Constant and Variable Definitions =======================*/

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/
/**
 * @brief sets Flightmode
 * @param[in] messageData contents of the ecu state message
 */
static void CANRX_SetFlightmode(uint64_t messageData);

/**
 * @brief   sets the fault disarm flag
 * @param[in] messageData contents of the ecu state message
 */
static void CANRX_SetFaultDisarmFlag(uint64_t messageData);

/*========== Static Function Implementations ================================*/

static void CANRX_SetFlightmode(uint64_t messageData) {
    uint64_t signalData = 0u;
    CAN_RxGetSignalDataFromMessageData(
        messageData,
        CANRX_ECU_STATE_FLIGHTMODE_START_BIT,
        CANRX_ECU_STATE_FLIGHTMODE_LENGTH,
        &signalData,
        CANRX_ECU_STATE_ENDIANNESS);

    BMS_SetFlightmode((bool)signalData);
}

static void CANRX_SetFaultDisarmFlag(uint64_t messageData) {
    uint64_t signalData = 0u;
    CAN_RxGetSignalDataFromMessageData(
        messageData,
        CANRX_ECU_STATE_FAULT_DISARM_START_BIT,
        CANRX_ECU_STATE_FAULT_DISARM_LENGTH,
        &signalData,
        CANRX_ECU_STATE_ENDIANNESS);

    BMS_SetFaultDisarmFlag((bool)signalData);
}

/*========== Extern Function Implementations ================================*/
extern uint32_t CANRX_EcuState(
    CAN_MESSAGE_PROPERTIES_s message,
    const uint8_t *const kpkCanData,
    const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(message.id == CANRX_ECU_STATE_ID);
    FAS_ASSERT(message.idType == CANRX_ECU_STATE_ID_TYPE);
    FAS_ASSERT(message.dlc == CANRX_ECU_STATE_DLC);
    FAS_ASSERT(message.endianness == CANRX_ECU_STATE_ENDIANNESS);
    FAS_ASSERT(kpkCanData != NULL_PTR);
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    uint64_t messageData = 0u;
    /* Get message as big endian for CRC calculation */
    CAN_RxGetMessageDataFromCanData(&messageData, kpkCanData, CAN_BIG_ENDIAN);
    uint8_t crc =
        Compute_RX_CRC8H2F((uint8_t *)&messageData, CANRX_ECU_STATE_CRC_START_BIT / 8u, CRC8H2F_INITIAL_VALUE);

    CAN_RxGetMessageDataFromCanData(&messageData, kpkCanData, CANRX_ECU_STATE_ENDIANNESS);
    uint64_t crc_received = 0u;
    CAN_RxGetSignalDataFromMessageData(
        messageData,
        CANRX_ECU_STATE_CRC_START_BIT,
        CANRX_ECU_STATE_CRC_LENGTH,
        &crc_received,
        CANRX_ECU_STATE_ENDIANNESS);

    if (crc == (uint8_t)crc_received) {
        /* Set Flightmode */
        CANRX_SetFlightmode(messageData);

        /* Set Fault Disarm Flag */
        CANRX_SetFaultDisarmFlag(messageData);
    }

    return 0u;
}
