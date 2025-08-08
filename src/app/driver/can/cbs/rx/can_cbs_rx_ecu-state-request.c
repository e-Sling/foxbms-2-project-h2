/**
 *
 * @copyright &copy; 2010 - 2025, Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * We kindly request you to use one or more of the following phrases to refer to
 * foxBMS in your hardware, software, documentation or advertising materials:
 *
 * - "This product uses parts of foxBMS&reg;"
 * - "This product includes parts of foxBMS&reg;"
 * - "This product is derived from foxBMS&reg;"
 *
 */

/**
 * @file    can_cbs_rx_ecu-state-request.c
 * @author  foxBMS Team
 * @date    2021-07-28 (date of creation)
 * @updated 2025-03-31 (date of last update)
 * @version v1.9.0
 * @ingroup DRIVERS
 * @prefix  CANRX
 *
 * @brief   CAN driver Rx callback implementation
 * @details CAN Rx callback for command message
 */

/*========== Includes =======================================================*/
#include "bms_cfg.h"

/* AXIVION Next Codeline Generic-LocalInclude: 'can_cbs_rx.h' declares the
 * prototype for the callback 'CANRX_EcuStateRequest' */
#include "bms.h"
#include "can_cbs_rx.h"
#include "can_cfg_rx-message-definitions.h"
#include "can_helper.h"
#include "diag.h"
#include "os.h"
#include "sys_mon.h"

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
 * @param[in] messageData contents of the ecu state request message
 */
static void CANRX_SetFaultDisarmFlag(uint64_t messageData);

/*========== Static Function Implementations ================================*/

static void CANRX_SetFaultDisarmFlag(uint64_t messageData) {
    /* AXIVION Routine Generic-MissingParameterAssert: messageData: parameter accepts whole range */
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

    /* Leon: Read CRC? */

    return 0u;
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
extern void TEST_CANRX_ClearAllPersistentFlags(uint64_t messageData) {
    CANRX_ClearAllPersistentFlags(messageData);
}
extern void TEST_CANRX_HandleModeRequest(uint64_t messageData, const CAN_SHIM_s *const kpkCanShim) {
    CANRX_HandleModeRequest(messageData, kpkCanShim);
}
extern void TEST_CANRX_HandleBalancingRequest(uint64_t messageData) {
    CANRX_HandleBalancingRequest(messageData);
}
extern void TEST_CANRX_SetBalancingThreshold(uint64_t messageData) {
    CANRX_SetBalancingThreshold(messageData);
}
#endif
