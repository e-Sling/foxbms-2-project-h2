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
 * @file    can_cbs_tx_diagnostic_flags.c
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
#include "diag.h"
#include "foxmath.h"
#include "sys_mon.h"

#include <math.h>
#include <stdint.h>

/*========== Macros and Definitions =========================================*/
/**
 * Configuration of the signals
 */
#define CANTX_DIAG_FLAG_LENGTH (CAN_BIT)

#define CANTX_DIAG_EMERGENCY_SHUTOFF_START_BIT                (0u)
#define CANTX_DIAG_SYSTEM_MONITORING_ERROR_START_BIT          (1u)
#define CANTX_DIAG_PRECHARGE_VOLTAGE_ERROR_START_BIT          (2u)
#define CANTX_DIAG_PRECHARGE_CURRENT_ERROR_START_BIT          (3u)
#define CANTX_DIAG_MCU_DIE_TEMPERATURE_ERROR_START_BIT        (4u)
#define CANTX_DIAG_CAN_TIMING_ERROR_START_BIT                 (5u)
#define CANTX_DIAG_PACK_OVERCURRENT_CHARGE_ERROR_START_BIT    (6u)
#define CANTX_DIAG_PACK_OVERCURRENT_DISCHARGE_ERROR_START_BIT (7u)
#define CANTX_DIAG_ALERT_FLAG_START_BIT                       (8u)
#define CANTX_DIAG_NVRAM_CRC_ERROR_START_BIT                  (9u)
#define CANTX_DIAG_CLAMP_30C_ERROR_START_BIT                  (10u)

#define CANTX_DIAG_BASE_CELL_TEMP_MEAS_TIMEOUT_START_BIT (11u)
#define CANTX_DIAG_AFE_CELL_TEMP_MEAS_START_BIT          (12u)
#define CANTX_DIAG_AFE_CELL_VOLTAGE_MEAS_START_BIT       (13u)

/*========== Static Constant and Variable Definitions =======================*/

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/
/**
 * @brief   get a boolean for if any timing error (current or recorded) occurred
 * @param   kpkCanShim  const pointer to CAN shim
 * @return  returns if there has been any timing violations
 */
static bool CANTX_AnySysMonTimingIssueDetected(const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief   Adds the data to the message about the pack values
 * @param   kpkCanShim const pointer to CAN shim
 * @param   pMessageData message data of the CAN message
 */
static void CANTX_BuildDiagnosticFlagsMessage(const CAN_SHIM_s *const kpkCanShim, uint64_t *pMessageData);

/*========== Static Function Implementations ================================*/
static bool CANTX_AnySysMonTimingIssueDetected(const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);
    SYSM_TIMING_VIOLATION_RESPONSE_s recordedTimingViolations = {false, false, false, false, false, false};
    SYSM_GetRecordedTimingViolations(&recordedTimingViolations);

    const bool anyTimingViolation =
        (recordedTimingViolations.recordedViolationAny ||
         kpkCanShim->pTableErrorState->taskEngineTimingViolationError ||
         kpkCanShim->pTableErrorState->task1msTimingViolationError ||
         kpkCanShim->pTableErrorState->task10msTimingViolationError ||
         kpkCanShim->pTableErrorState->task100msTimingViolationError ||
         kpkCanShim->pTableErrorState->task100msAlgoTimingViolationError);

    return anyTimingViolation;
}

static void CANTX_BuildDiagnosticFlagsMessage(const CAN_SHIM_s *const kpkCanShim, uint64_t *pMessageData) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);
    FAS_ASSERT(pMessageData != NULL_PTR);

    /* Emergency shutoff */
    uint64_t data = CAN_ConvertBooleanToInteger(BMS_IsTransitionToErrorStateActive());
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_EMERGENCY_SHUTOFF_START_BIT, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* sys mon error */
    data = CAN_ConvertBooleanToInteger(CANTX_AnySysMonTimingIssueDetected(kpkCanShim));
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_SYSTEM_MONITORING_ERROR_START_BIT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Precharge voltage */
    data = 0u; /* No precharge error detected */
    for (uint8_t s = 0u; s < BS_NR_OF_STRINGS; s++) {
        if (kpkCanShim->pTableErrorState->prechargeAbortedDueToVoltage[s] == true) {
            data = 1u;
        }
    }
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_PRECHARGE_VOLTAGE_ERROR_START_BIT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Precharge current */
    data = 0u; /* No precharge error detected */
    for (uint8_t s = 0u; s < BS_NR_OF_STRINGS; s++) {
        if (kpkCanShim->pTableErrorState->prechargeAbortedDueToCurrent[s] == true) {
            data = 1u;
        }
    }
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_PRECHARGE_CURRENT_ERROR_START_BIT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: MCU die temperature */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->mcuDieTemperatureViolationError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_MCU_DIE_TEMPERATURE_ERROR_START_BIT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Can timing */
    data = kpkCanShim->pTableErrorState->stateRequestTimingViolationError;
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_CAN_TIMING_ERROR_START_BIT, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Overcurrent pack charge */
    data = kpkCanShim->pTableMsl->packChargeOvercurrent;
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_PACK_OVERCURRENT_CHARGE_ERROR_START_BIT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Overcurrent pack discharge */
    data = kpkCanShim->pTableMsl->packDischargeOvercurrent;
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_PACK_OVERCURRENT_DISCHARGE_ERROR_START_BIT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Alert flag */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->alertFlagSetError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_ALERT_FLAG_START_BIT, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: NVRAM CRC */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->framReadCrcError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_NVRAM_CRC_ERROR_START_BIT, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Clamp 30C */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->supplyVoltageClamp30cError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_CLAMP_30C_ERROR_START_BIT, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Base Cell Temp Measurement Timeout */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->baseCellTemperatureMeasurementTimeoutError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_BASE_CELL_TEMP_MEAS_TIMEOUT_START_BIT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: AFE Cell Temperature Measurement */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->afeCellTemperatureInvalidError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_AFE_CELL_TEMP_MEAS_START_BIT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: AFE Cell Voltage Measurement */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->afeCellVoltageInvalidError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_AFE_CELL_VOLTAGE_MEAS_START_BIT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);
}

/*========== Extern Function Implementations ================================*/
extern uint32_t CANTX_DiagnosticFlags(
    CAN_MESSAGE_PROPERTIES_s message,
    uint8_t *pCanData,
    uint8_t *pMuxId,
    const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(message.id == CANTX_DIAGNOSTIC_ID);
    FAS_ASSERT(message.idType == CANTX_DIAGNOSTIC_ID_TYPE);
    FAS_ASSERT(message.dlc == CAN_FOXBMS_MESSAGES_DEFAULT_DLC);

    FAS_ASSERT(message.endianness == CANTX_DIAGNOSTIC_ENDIANNESS);
    FAS_ASSERT(pCanData != NULL_PTR);
    FAS_ASSERT(pMuxId == NULL_PTR); /* pMuxId is not used here, therefore has to be NULL_PTR */
    FAS_ASSERT(kpkCanShim != NULL_PTR);
    uint64_t messageData = 0u;

    DATA_READ_DATA(kpkCanShim->pTableErrorState, kpkCanShim->pTableMsl);

    CANTX_BuildDiagnosticFlagsMessage(kpkCanShim, &messageData);

    /* now copy data in the buffer that will be used to send data */
    CAN_TxSetCanDataWithMessageData(messageData, pCanData, message.endianness);

    return 0u;
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
#endif
