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
 * @file    can_cbs_tx_pack_voltage_min-max-avg.c
 * @author  foxBMS Team
 * @date    2021-04-20 (date of creation)
 * @updated 2025-03-31 (date of last update)
 * @version v1.9.0
 * @ingroup DRIVERS
 * @prefix  CANTX
 *
 * @brief   CAN driver Tx callback implementation
 * @details CAN Tx callback for min/max/avg values
 */

/*========== Includes =======================================================*/
#include "bms.h"
/* AXIVION Next Codeline Generic-LocalInclude: 'can_cbs_tx_cyclic.h' declares
 * the prototype for the callback 'CANTX_PackMinimumMaximumValues' */
#include "can_cbs_tx_cyclic.h"
#include "can_cfg_tx-cyclic-message-definitions.h"
#include "can_helper.h"
#include "foxmath.h"

#include <math.h>
#include <stdint.h>

/*========== Macros and Definitions =========================================*/
/**
 * Configuration of the signals
 */
#define CANTX_SIGNAL_MAXIMUM_CELL_VOLTAGE_START_BIT (0u)
#define CANTX_SIGNAL_MAXIMUM_CELL_VOLTAGE_LENGTH    (16u)
#define CANTX_SIGNAL_MINIMUM_CELL_VOLTAGE_START_BIT (16u)
#define CANTX_SIGNAL_MINIMUM_CELL_VOLTAGE_LENGTH    (16u)
#define CANTX_SIGNAL_AVERAGE_CELL_VOLTAGE_START_BIT (32u)
#define CANTX_SIGNAL_AVERAGE_CELL_VOLTAGE_LENGTH    (16u)

/*========== Static Constant and Variable Definitions =======================*/

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/
/**
 * @brief   Adds the data to the message about the pack values
 * @param   kpkCanShim const pointer to CAN shim
 * @param   pMessageData message data of the CAN message
 */
static void CANTX_BuildVoltageMinMaxAvgMessage(const CAN_SHIM_s *const kpkCanShim, uint64_t *pMessageData);

/*========== Static Function Implementations ================================*/
static void CANTX_BuildVoltageMinMaxAvgMessage(const CAN_SHIM_s *const kpkCanShim, uint64_t *pMessageData) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);
    FAS_ASSERT(pMessageData != NULL_PTR);

    /* maximum cell voltage */
    uint64_t signalData = (uint64_t)kpkCanShim->pTableMinMax->maximumCellVoltage_mV[BS_STRING0];
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_SIGNAL_MAXIMUM_CELL_VOLTAGE_START_BIT,
        CANTX_SIGNAL_MAXIMUM_CELL_VOLTAGE_LENGTH,
        signalData,
        CANTX_PACK_VOLTAGE_MIN_MAX_AVG_ENDIANNESS);
    /* minimum cell voltage */
    signalData = (uint64_t)kpkCanShim->pTableMinMax->minimumCellVoltage_mV[BS_STRING0];
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_SIGNAL_MINIMUM_CELL_VOLTAGE_START_BIT,
        CANTX_SIGNAL_MINIMUM_CELL_VOLTAGE_LENGTH,
        signalData,
        CANTX_PACK_VOLTAGE_MIN_MAX_AVG_ENDIANNESS);
    /* average cell voltage */
    signalData = (uint64_t)kpkCanShim->pTableMinMax->averageCellVoltage_mV[BS_STRING0];
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_SIGNAL_AVERAGE_CELL_VOLTAGE_START_BIT,
        CANTX_SIGNAL_AVERAGE_CELL_VOLTAGE_LENGTH,
        signalData,
        CANTX_PACK_VOLTAGE_MIN_MAX_AVG_ENDIANNESS);
}

/*========== Extern Function Implementations ================================*/
extern uint32_t CANTX_VoltageMinMaxAvgValues(
    CAN_MESSAGE_PROPERTIES_s message,
    uint8_t *pCanData,
    uint8_t *pMuxId,
    const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(message.id == CANTX_PACK_VOLTAGE_MIN_MAX_AVG_ID);
    FAS_ASSERT(message.idType == CANTX_PACK_VOLTAGE_MIN_MAX_AVG_ID_TYPE);
    FAS_ASSERT(message.dlc == CAN_FOXBMS_MESSAGES_DEFAULT_DLC);
    FAS_ASSERT(message.endianness == CANTX_PACK_VOLTAGE_MIN_MAX_AVG_ENDIANNESS);
    FAS_ASSERT(pCanData != NULL_PTR);
    FAS_ASSERT(pMuxId == NULL_PTR); /* pMuxId is not used here, therefore has to be NULL_PTR */
    FAS_ASSERT(kpkCanShim != NULL_PTR);
    uint64_t messageData = 0u;

    DATA_READ_DATA(kpkCanShim->pTableMinMax);

    CANTX_BuildVoltageMinMaxAvgMessage(kpkCanShim, &messageData);

    /* now copy data in the buffer that will be used to send data */
    CAN_TxSetCanDataWithMessageData(messageData, pCanData, message.endianness);

    return 0u;
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
extern int16_t TEST_CANTX_GetPackMaximumVoltage(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_GetPackMaximumVoltage(kpkCanShim);
}
extern int16_t TEST_CANTX_GetPackMinimumVoltage(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_GetPackMinimumVoltage(kpkCanShim);
}
extern int16_t TEST_CANTX_GetPackMaximumTemperature(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_GetPackMaximumTemperature(kpkCanShim);
}
extern int16_t TEST_CANTX_GetPackMinimumTemperature(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_GetPackMinimumTemperature(kpkCanShim);
}
extern uint64_t TEST_CANTX_CalculatePackMaximumTemperature(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_CalculatePackMaximumTemperature(kpkCanShim);
}
extern uint64_t TEST_CANTX_CalculatePackMinimumTemperature(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_CalculatePackMinimumTemperature(kpkCanShim);
}
extern void TEST_CANTX_BuildPackMinimumMaximumValuesMessage(
    const CAN_SHIM_s *const kpkCanShim,
    uint64_t *pMessageData) {
    CANTX_BuildPackMinimumMaximumValuesMessage(kpkCanShim, pMessageData);
}
#endif
