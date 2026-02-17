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
#define CANTX_DIAG_FLAG_LENGTH        (CAN_BIT)
#define CANTX_DIAG_ERROR_LEVEL_LENGTH (3u)

#define CANTX_DIAG_VOLTAGE_ERROR             (0u)
#define CANTX_DIAG_TEMPERATURE_ERROR         (3u)
#define CANTX_DIAG_OVERCURRENT_CHARGE        (6u)
#define CANTX_DIAG_OVERCURRENT_DISCHARGE     (7u)
#define CANTX_DIAG_DEEP_DISCHARGE_DETECTED   (8u)
#define CANTX_DIAG_PLAUSIBILITY_PACK_VOLTAGE (9u)
#define CANTX_DIAG_PLAUSIBILITY_CELL_VOLTAGE (10u)
#define CANTX_DIAG_PLAUSIBILITY_CELL_TEMP    (11u)
#define CANTX_DIAG_PRECHARGE_VOLTAGE         (12u)
#define CANTX_DIAG_PRECHARGE_CURRENT         (13u)

#define CANTX_DIAG_CONTACTOR_FEEDBACK     (15u)
#define CANTX_DIAG_CURRENT_SENSOR         (16u)
#define CANTX_DIAG_CURRENT_ON_OPEN_STRING (17u)
#define CANTX_DIAG_AFE_COMMUNICATION      (18u)
#define CANTX_DIAG_AFE_OPEN_WIRE          (19u)
#define CANTX_DIAG_SYSTEM_CAUTION         (20u)
#define CANTX_DIAG_SYSTEM_WARNING         (21u)

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

    const uint8_t latch = BMS_GetLatchedShutdownBits();

    /* Voltage ErrorLevel */
    uint64_t data = (uint64_t)CAN_ConvertFlagstoErrorLevel(
        kpkCanShim->pTableErrorState->plausibilityCheckCellVoltageSpreadError[BS_STRING0],
        kpkCanShim->pTableMsl->underVoltage[BS_STRING0],
        kpkCanShim->pTableRsl->underVoltage[BS_STRING0],
        kpkCanShim->pTableMol->underVoltage[BS_STRING0],
        kpkCanShim->pTableMol->overVoltage[BS_STRING0],
        kpkCanShim->pTableRsl->overVoltage[BS_STRING0],
        kpkCanShim->pTableMsl->overVoltage[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_VOLTAGE_ERROR, CANTX_DIAG_ERROR_LEVEL_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Temperature ErrorLevel */
    data = (uint64_t)CAN_ConvertFlagstoErrorLevel(
        kpkCanShim->pTableErrorState->plausibilityCheckCellTemperatureSpreadError[BS_STRING0],
        kpkCanShim->pTableMsl->undertemperatureDischarge[BS_STRING0] ||
            kpkCanShim->pTableMsl->undertemperatureCharge[BS_STRING0],
        kpkCanShim->pTableRsl->undertemperatureDischarge[BS_STRING0] ||
            kpkCanShim->pTableRsl->undertemperatureCharge[BS_STRING0],
        kpkCanShim->pTableMol->undertemperatureDischarge[BS_STRING0] ||
            kpkCanShim->pTableMol->undertemperatureCharge[BS_STRING0],
        kpkCanShim->pTableMol->overtemperatureDischarge[BS_STRING0] ||
            kpkCanShim->pTableMol->overtemperatureCharge[BS_STRING0],
        kpkCanShim->pTableRsl->overtemperatureDischarge[BS_STRING0] ||
            kpkCanShim->pTableRsl->overtemperatureCharge[BS_STRING0],
        kpkCanShim->pTableMsl->overtemperatureDischarge[BS_STRING0] ||
            kpkCanShim->pTableMsl->overtemperatureCharge[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_TEMPERATURE_ERROR, CANTX_DIAG_ERROR_LEVEL_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: System Warning */
    data = CAN_ConvertBooleanToInteger(
        CANTX_AnySysMonTimingIssueDetected(kpkCanShim) || kpkCanShim->pTableErrorState->alertFlagSetError ||
        kpkCanShim->pTableErrorState->framReadCrcError || kpkCanShim->pTableErrorState->supplyVoltageClamp30cError ||
        kpkCanShim->pTableErrorState->currentMeasurementTimeoutError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->currentMeasurementInvalidError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->powerMeasurementInvalidError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->mcuSbcFinError || kpkCanShim->pTableErrorState->mcuSbcRstbError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_SYSTEM_WARNING, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: System Caution */
    data = CAN_ConvertBooleanToInteger(
        kpkCanShim->pTableErrorState->canRxQueueFullError || kpkCanShim->pTableErrorState->canTxQueueFullError ||
        kpkCanShim->pTableErrorState->afeCellVoltageInvalidError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->afeCellTemperatureInvalidError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->baseCellVoltageMeasurementTimeoutError ||
        kpkCanShim->pTableErrorState->redundancy0CellVoltageMeasurementTimeoutError ||
        kpkCanShim->pTableErrorState->baseCellTemperatureMeasurementTimeoutError ||
        kpkCanShim->pTableErrorState->redundancy0CellTemperatureMeasurementTimeoutError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_SYSTEM_CAUTION, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Overcurrent charge */
    data = CAN_ConvertBooleanToInteger(
        kpkCanShim->pTableMsl->cellChargeOvercurrent[BS_STRING0] || kpkCanShim->pTableMsl->packChargeOvercurrent);
    if (latch & SHUTDOWNBIT_OVERCURRENT_CHARGE)
        data = 1u;
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_OVERCURRENT_CHARGE, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Overcurrent discharge */
    data = CAN_ConvertBooleanToInteger(
        kpkCanShim->pTableMsl->cellDischargeOvercurrent[BS_STRING0] || kpkCanShim->pTableMsl->packDischargeOvercurrent);
    if (latch & SHUTDOWNBIT_OVERCURRENT_DISCHARGE)
        data = 1u;
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_OVERCURRENT_DISCHARGE, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Deep discharge */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->deepDischargeDetectedError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_DEEP_DISCHARGE_DETECTED, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Current on open string */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->currentOnOpenStringDetectedError[BS_STRING0]);
    if (latch & SHUTDOWNBIT_CURRENT_ON_OPEN_STRING)
        data = 1u;
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_CURRENT_ON_OPEN_STRING, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: AFE Communication */
    data = CAN_ConvertBooleanToInteger(
        kpkCanShim->pTableErrorState->afeCommunicationSpiError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->afeCommunicationCrcError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->afeSlaveMultiplexerError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->afeConfigurationError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_AFE_COMMUNICATION, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: AFE Open Wire */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->openWireDetectedError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_AFE_OPEN_WIRE, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Current sensor */
    data = CAN_ConvertBooleanToInteger(
        kpkCanShim->pTableErrorState->currentSensorNotRespondingError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->currentSensorVoltage1TimeoutError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->currentSensorVoltage2TimeoutError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->currentSensorVoltage3TimeoutError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->currentSensorPowerTimeoutError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->currentSensorCoulombCounterTimeoutError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->currentSensorEnergyCounterTimeoutError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_CURRENT_SENSOR, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Contactor feedback */
    data = CAN_ConvertBooleanToInteger(
        kpkCanShim->pTableErrorState->contactorInNegativePathOfStringFeedbackError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->contactorInPositivePathOfStringFeedbackError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->prechargeContactorFeedbackError[BS_STRING0] ||
        kpkCanShim->pTableErrorState->mainContactorFeedbackError[BS_STRING0]);
    if (latch & SHUTDOWNBIT_CONTACTOR_FEEDBACK)
        data = 1u;
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_CONTACTOR_FEEDBACK, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: Plausibility Pack Voltage */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->plausibilityCheckPackVoltageError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_PLAUSIBILITY_PACK_VOLTAGE, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: Plausibility Cell Voltage */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->plausibilityCheckCellVoltageError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_PLAUSIBILITY_CELL_VOLTAGE, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: Plausibility Cell Temperature */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->plausibilityCheckCellTemperatureError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_PLAUSIBILITY_CELL_TEMP, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: Precharge voltage */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->prechargeAbortedDueToVoltage[BS_STRING0]);
    if (latch & SHUTDOWNBIT_PRECHARGE_VOLTAGE)
        data = 1u;
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_PRECHARGE_VOLTAGE, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: Precharge current */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->prechargeAbortedDueToCurrent[BS_STRING0]);
    if (latch & SHUTDOWNBIT_PRECHARGE_CURRENT)
        data = 1u;
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_PRECHARGE_CURRENT, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);
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

    DATA_READ_DATA(kpkCanShim->pTableErrorState, kpkCanShim->pTableMsl, kpkCanShim->pTableRsl, kpkCanShim->pTableMol);

    CANTX_BuildDiagnosticFlagsMessage(kpkCanShim, &messageData);

    /* now copy data in the buffer that will be used to send data */
    CAN_TxSetCanDataWithMessageData(messageData, pCanData, message.endianness);

    return 0u;
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
#endif
