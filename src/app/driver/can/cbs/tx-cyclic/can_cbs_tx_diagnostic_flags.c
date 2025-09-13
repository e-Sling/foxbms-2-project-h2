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

/* Diagnostics shown on HMI */

#define CANTX_DIAG_VOLTAGE_ERROR              (0u)
#define CANTX_DIAG_TEMPERATURE_ERROR          (3u)
#define CANTX_DIAG_PACK_OVERCURRENT_CHARGE    (6u)
#define CANTX_DIAG_PACK_OVERCURRENT_DISCHARGE (7u)
#define CANTX_DIAG_DEEP_DISCHARGE_DETECTED    (8u)
#define CANTX_DIAG_AFE_SPI                    (9u)
#define CANTX_DIAG_PLAUSIBILITY_PACK_VOLTAGE  (10u)
#define CANTX_DIAG_PLAUSIBILITY_CELL_VOLTAGE  (11u)
#define CANTX_DIAG_PLAUSIBILITY_CELL_TEMP     (12u)
#define CANTX_DIAG_PRECHARGE_VOLTAGE          (15u)
#define CANTX_DIAG_PRECHARGE_CURRENT          (16u)
#define CANTX_DIAG_DIRECTCONNECT_ABORT        (17u)

/************** Additional diagnostic flags **************/

#define CANTX_DIAG_EMERGENCY_SHUTOFF             (18u)
#define CANTX_DIAG_SYSTEM_MONITORING             (19u)
#define CANTX_DIAG_ALERT_FLAG                    (20u)
#define CANTX_DIAG_FRAM_READ_CRC                 (21u)
#define CANTX_DIAG_SUPPLY_VOLTAGE_CLAMP_30C_LOST (22u)
#define CANTX_DIAG_CURRENT_ON_OPEN_STRING        (23u)

/* AFE diagnostic flags */
#define CANTX_DIAG_AFE_CRC       (24u)
#define CANTX_DIAG_AFE_MUX       (25u)
#define CANTX_DIAG_AFE_CONFIG    (26u)
#define CANTX_DIAG_AFE_OPEN_WIRE (27u)

/* CAN diagnostic flags */
#define CANTX_DIAG_CAN_TIMING        (28u)
#define CANTX_DIAG_CAN_RX_QUEUE_FULL (29u)
#define CANTX_DIAG_CAN_TX_QUEUE_FULL (30u)

/* Isabellenhuette diagnostic flags */
#define CANTX_DIAG_CURRENT_SENSOR_RESPONDING                (31u)
#define CANTX_DIAG_CURRENT_SENSOR_V1_MEASUREMENT_TIMEOUT    (32u)
#define CANTX_DIAG_CURRENT_SENSOR_V2_MEASUREMENT_TIMEOUT    (33u)
#define CANTX_DIAG_CURRENT_SENSOR_V3_MEASUREMENT_TIMEOUT    (34u)
#define CANTX_DIAG_CURRENT_SENSOR_POWER_MEASUREMENT_TIMEOUT (35u)
#define CANTX_DIAG_CURRENT_SENSOR_CC_RESPONDING             (36u)
#define CANTX_DIAG_CURRENT_SENSOR_EC_RESPONDING             (37u)

/* Contactor diagnostic flags */
#define CANTX_DIAG_STRING_MINUS_CONTACTOR_FEEDBACK     (38u)
#define CANTX_DIAG_STRING_PLUS_CONTACTOR_FEEDBACK      (39u)
#define CANTX_DIAG_STRING_PRECHARGE_CONTACTOR_FEEDBACK (40u)
#define CANTX_DIAG_STRING_MAIN_CONTACTOR_FEEDBACK      (41u)

/* Measurement diagnostic flags */
#define CANTX_DIAG_AFE_CELL_VOLTAGE_MEAS_ERROR                      (42u)
#define CANTX_DIAG_AFE_CELL_TEMPERATURE_MEAS_ERROR                  (43u)
#define CANTX_DIAG_BASE_CELL_VOLTAGE_MEASUREMENT_TIMEOUT            (44u)
#define CANTX_DIAG_REDUNDANCY0_CELL_VOLTAGE_MEASUREMENT_TIMEOUT     (45u)
#define CANTX_DIAG_BASE_CELL_TEMPERATURE_MEASUREMENT_TIMEOUT        (46u)
#define CANTX_DIAG_REDUNDANCY0_CELL_TEMPERATURE_MEASUREMENT_TIMEOUT (47u)
#define CANTX_DIAG_CURRENT_MEASUREMENT_TIMEOUT                      (48u)
#define CANTX_DIAG_CURRENT_MEASUREMENT_ERROR                        (49u)
#define CANTX_DIAG_POWER_MEASUREMENT_ERROR                          (50u)

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
    data |= (uint64_t)CAN_ConvertFlagstoErrorLevel(
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

    /* Emergency shutoff */
    data = CAN_ConvertBooleanToInteger(BMS_IsTransitionToErrorStateActive());
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_EMERGENCY_SHUTOFF, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: System monitoring */
    data = CAN_ConvertBooleanToInteger(CANTX_AnySysMonTimingIssueDetected(kpkCanShim));
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_SYSTEM_MONITORING, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Can timing */
    data = kpkCanShim->pTableErrorState->stateRequestTimingViolationError;
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_CAN_TIMING, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Overcurrent pack charge */
    data = kpkCanShim->pTableMsl->packChargeOvercurrent;
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_PACK_OVERCURRENT_CHARGE, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Overcurrent pack discharge */
    data = kpkCanShim->pTableMsl->packDischargeOvercurrent;
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_PACK_OVERCURRENT_DISCHARGE, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Alert flag */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->alertFlagSetError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_ALERT_FLAG, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Info: FRAM CRC */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->framReadCrcError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_FRAM_READ_CRC, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Supply voltage clamp 30C */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->supplyVoltageClamp30cError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_SUPPLY_VOLTAGE_CLAMP_30C_LOST,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Deep discharge */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->deepDischargeDetectedError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_DEEP_DISCHARGE_DETECTED, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Current on open string */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->currentOnOpenStringDetectedError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_CURRENT_ON_OPEN_STRING, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: AFE SPI */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->afeCommunicationSpiError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_AFE_SPI, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: AFE CRC */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->afeCommunicationCrcError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_AFE_CRC, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: AFE MUX */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->afeSlaveMultiplexerError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_AFE_MUX, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: AFE CONFIG */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->afeConfigurationError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_AFE_CONFIG, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: AFE Open Wire */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->openWireDetectedError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_AFE_OPEN_WIRE, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: CAN Rx */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->canRxQueueFullError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_CAN_RX_QUEUE_FULL, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: CAN Tx */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->canTxQueueFullError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_CAN_TX_QUEUE_FULL, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Current sensor responding */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->currentSensorNotRespondingError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_CURRENT_SENSOR_RESPONDING, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Current sensor V1 measurement timeout */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->currentSensorVoltage1TimeoutError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_CURRENT_SENSOR_V1_MEASUREMENT_TIMEOUT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Current sensor V2 measurement timeout */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->currentSensorVoltage2TimeoutError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_CURRENT_SENSOR_V2_MEASUREMENT_TIMEOUT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Current sensor V3 measurement timeout */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->currentSensorVoltage3TimeoutError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_CURRENT_SENSOR_V3_MEASUREMENT_TIMEOUT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Current sensor power measurement timeout */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->currentSensorPowerTimeoutError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_CURRENT_SENSOR_POWER_MEASUREMENT_TIMEOUT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Current sensor CC responding */
    data =
        CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->currentSensorCoulombCounterTimeoutError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_CURRENT_SENSOR_CC_RESPONDING,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Current sensor EC responding */
    data =
        CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->currentSensorEnergyCounterTimeoutError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_CURRENT_SENSOR_EC_RESPONDING,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: String minus contactor feedback */
    data = CAN_ConvertBooleanToInteger(
        kpkCanShim->pTableErrorState->contactorInNegativePathOfStringFeedbackError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_STRING_MINUS_CONTACTOR_FEEDBACK,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: String plus contactor feedback */
    data = CAN_ConvertBooleanToInteger(
        kpkCanShim->pTableErrorState->contactorInPositivePathOfStringFeedbackError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_STRING_PLUS_CONTACTOR_FEEDBACK,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: String precharge contactor feedback */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->prechargeContactorFeedbackError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_STRING_PRECHARGE_CONTACTOR_FEEDBACK,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: String main contactor feedback */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->mainContactorFeedbackError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_STRING_MAIN_CONTACTOR_FEEDBACK,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

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
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_PRECHARGE_VOLTAGE, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: Precharge current */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->prechargeAbortedDueToCurrent[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_PRECHARGE_CURRENT, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Direct connect abort */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->directConnectAborted[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_DIRECTCONNECT_ABORT, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: AFE Cell Voltage Measurement */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->afeCellVoltageInvalidError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_AFE_CELL_VOLTAGE_MEAS_ERROR, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: AFE Cell Temperature Measurement */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->afeCellTemperatureInvalidError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_AFE_CELL_TEMPERATURE_MEAS_ERROR,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: Base Cell Voltage Timeout */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->baseCellVoltageMeasurementTimeoutError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_BASE_CELL_VOLTAGE_MEASUREMENT_TIMEOUT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: Redundancy0 Cell Voltage Timeout */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->redundancy0CellVoltageMeasurementTimeoutError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_REDUNDANCY0_CELL_VOLTAGE_MEASUREMENT_TIMEOUT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: Base Cell Temperature Timeout */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->baseCellTemperatureMeasurementTimeoutError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_BASE_CELL_TEMPERATURE_MEASUREMENT_TIMEOUT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Warning: Redundancy0 Cell Temperature Timeout */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->redundancy0CellTemperatureMeasurementTimeoutError);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_DIAG_REDUNDANCY0_CELL_TEMPERATURE_MEASUREMENT_TIMEOUT,
        CANTX_DIAG_FLAG_LENGTH,
        data,
        CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Current Measurement Timeout */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->currentMeasurementTimeoutError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_CURRENT_MEASUREMENT_TIMEOUT, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Current Measurement Error */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->currentMeasurementInvalidError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_CURRENT_MEASUREMENT_ERROR, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);

    /* Error: Power Measurement Error */
    data = CAN_ConvertBooleanToInteger(kpkCanShim->pTableErrorState->powerMeasurementInvalidError[BS_STRING0]);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, CANTX_DIAG_POWER_MEASUREMENT_ERROR, CANTX_DIAG_FLAG_LENGTH, data, CANTX_BMS_STATE_ENDIANNESS);
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
