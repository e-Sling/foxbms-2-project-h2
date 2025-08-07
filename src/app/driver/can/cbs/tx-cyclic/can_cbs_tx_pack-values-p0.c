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
 * @file    can_cbs_tx_pack-values-p0.c
 * @author  foxBMS Team
 * @date    2021-07-21 (date of creation)
 * @updated 2025-03-31 (date of last update)
 * @version v1.9.0
 * @ingroup DRIVERS
 * @prefix  CANTX
 *
 * @brief   CAN driver Tx callback implementation
 * @details CAN Tx callback for pack value and string value messages
 */

/*========== Includes =======================================================*/
/* AXIVION Next Codeline Generic-LocalInclude: 'can_cbs_tx_cyclic.h' declares
 * the prototype for the callback 'CANTX_PackValuesP0' */
#include "can_cbs_tx_cyclic.h"
#include "can_cfg_tx-cyclic-message-definitions.h"
#include "can_helper.h"
#include "foxmath.h"

#include <math.h>
#include <stdint.h>

/*========== Macros and Definitions =========================================*/
/** @{
 * defines of the battery voltage signal
*/
#define CANTX_PACK_P0_BATTERY_VOLTAGE_START_BIT    (0u)
#define CANTX_PACK_P0_BATTERY_VOLTAGE_LENGTH       (16u)
#define CANTX_MINIMUM_VALUE_BATTERY_VOLTAGE_SIGNAL (-1638400.0f)
#define CANTX_MAXIMUM_VALUE_BATTERY_VOLTAGE_SIGNAL (1638300.0f)
/** @} */

/** @{
 * configuration of the battery voltage signal
*/
static const CAN_SIGNAL_TYPE_s cantx_signalBatteryVoltage = {
    CANTX_PACK_P0_BATTERY_VOLTAGE_START_BIT,
    CANTX_PACK_P0_BATTERY_VOLTAGE_LENGTH,
    UNIT_CONVERSION_FACTOR_100_FLOAT,
    CAN_SIGNAL_OFFSET_0,
    CANTX_MINIMUM_VALUE_BATTERY_VOLTAGE_SIGNAL,
    CANTX_MAXIMUM_VALUE_BATTERY_VOLTAGE_SIGNAL};
/** @} */

/** @{
 * defines of the average State of Energy (SoE) signal
*/
#define CANTX_PACK_P0_AVERAGE_SOE_START_BIT (16u)
#define CANTX_PACK_P0_AVERAGE_SOE_LENGTH    (10u)
#define CANTX_MINIMUM_VALUE_PERCENT_SIGNALS (0.0f)
#define CANTX_MAXIMUM_VALUE_PERCENT_SIGNALS (102.3f)

/** @{
 * configuration of the average SoE signal
*/
static const CAN_SIGNAL_TYPE_s cantx_signalAverageSoe = {
    CANTX_PACK_P0_AVERAGE_SOE_START_BIT,
    CANTX_PACK_P0_AVERAGE_SOE_LENGTH,
    UNIT_CONVERSION_FACTOR_1_10_TH_FLOAT,
    CAN_SIGNAL_OFFSET_0,
    CANTX_MINIMUM_VALUE_PERCENT_SIGNALS,
    CANTX_MAXIMUM_VALUE_PERCENT_SIGNALS};
/** @} */

/** @{
 * defines of the maximum discharge power signal
*/
#define CANTX_SIGNAL_MAXIMUM_DISCHARGE_POWER_START_BIT     (26u)
#define CANTX_SIGNAL_MAXIMUM_DISCHARGE_POWER_LENGTH        (12u)
#define CANTX_MINIMUM_VALUE_MAXIMUM_DISCHARGE_POWER_SIGNAL (0.0f)
#define CANTX_MAXIMUM_VALUE_MAXIMUM_DISCHARGE_POWER_SIGNAL (409500.0f)
/** @} */

/** @{
 * configuration of the maximum discharge power signal
*/
static const CAN_SIGNAL_TYPE_s cantx_signalMaximumDischargePower = {
    CANTX_SIGNAL_MAXIMUM_DISCHARGE_POWER_START_BIT,
    CANTX_SIGNAL_MAXIMUM_DISCHARGE_POWER_LENGTH,
    UNIT_CONVERSION_FACTOR_100_FLOAT,
    CAN_SIGNAL_OFFSET_0,
    CANTX_MINIMUM_VALUE_MAXIMUM_DISCHARGE_POWER_SIGNAL,
    CANTX_MAXIMUM_VALUE_MAXIMUM_DISCHARGE_POWER_SIGNAL};
/** @} */

/** @{
 * defines of the maximum charge power signal
*/
#define CANTX_SIGNAL_MAXIMUM_CHARGE_POWER_START_BIT     (38u)
#define CANTX_SIGNAL_MAXIMUM_CHARGE_POWER_LENGTH        (12u)
#define CANTX_MINIMUM_VALUE_MAXIMUM_CHARGE_POWER_SIGNAL (0.0f)
#define CANTX_MAXIMUM_VALUE_MAXIMUM_CHARGE_POWER_SIGNAL (409500.0f)
/** @} */

/** @{
 * configuration of the maximum charge power signal
*/
static const CAN_SIGNAL_TYPE_s cantx_signalMaximumChargePower = {
    CANTX_SIGNAL_MAXIMUM_CHARGE_POWER_START_BIT,
    CANTX_SIGNAL_MAXIMUM_CHARGE_POWER_LENGTH,
    UNIT_CONVERSION_FACTOR_100_FLOAT,
    CAN_SIGNAL_OFFSET_0,
    CANTX_MINIMUM_VALUE_MAXIMUM_CHARGE_POWER_SIGNAL,
    CANTX_MAXIMUM_VALUE_MAXIMUM_CHARGE_POWER_SIGNAL};
/** @} */

#define CANTX_SIGNAL_PACK_VALUES_CRC_START_BIT (56u)
#define CANTX_SIGNAL_PACK_VALUES_CRC_LENGTH    (8u)

/*========== Static Constant and Variable Definitions =======================*/

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/
/**
 * @brief   calculates the return value of the battery voltage
 * @param   kpkCanShim const pointer to CAN shim
 * @return  returns the return value of the battery voltage
 */
static uint64_t CANTX_CalculateBatteryVoltage(const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief   calculates the average State of Energy (SoE) of the pack
 * @param   kpkCanShim const pointer to CAN shim
 * @return  returns the return value of average State of Energy (SoE) of the pack
 */
static uint64_t CANTX_CalculateAverageSoE(const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief   calculates the maximum discharge power of the pack
 * @param   kpkCanShim const pointer to CAN shim
 * @return  returns the return value of maximum discharge power of the pack
 */
static uint64_t CANTX_CalculateMaxDischargePower(const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief   calculates the maximum charge power of the pack
 * @param   kpkCanShim const pointer to CAN shim
 * @return  returns the return value of maximum charge power of the pack
 */
static uint64_t CANTX_CalculateMaxChargePower(const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief   builds the PackP0 message
 * @param   kpkCanShim const pointer to CAN shim
 * @param   pMessageData message data of the CAN message
 */
static void CANTX_BuildP0Message(const CAN_SHIM_s *const kpkCanShim, uint64_t *pMessageData);

/*========== Static Function Implementations ================================*/
static uint64_t CANTX_CalculateBatteryVoltage(const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    /* Battery voltage */
    float_t signalData = kpkCanShim->pTablePackValues->batteryVoltage_mV;
    CAN_TxPrepareSignalData(&signalData, cantx_signalBatteryVoltage);
    uint64_t data = (uint64_t)signalData;
    return data;
}

static uint64_t CANTX_CalculateAverageSoE(const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    /* Average SoE */
    float_t signalData = kpkCanShim->pTableSoe->averageSoe_perc[BS_STRING0];
    CAN_TxPrepareSignalData(&signalData, cantx_signalAverageSoe);
    uint64_t data = (uint64_t)signalData;
    return data;
}

static uint64_t CANTX_CalculateMaxDischargePower(const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    /* maximum charge power = discharge_current_A * battery_voltage_V */
    float_t signalData =
        ((float_t)kpkCanShim->pTableSof->recommendedContinuousPackDischargeCurrent_mA *
         UNIT_CONVERSION_FACTOR_1_1000_TH_FLOAT) *
        ((float_t)kpkCanShim->pTablePackValues->batteryVoltage_mV * UNIT_CONVERSION_FACTOR_1_1000_TH_FLOAT);
    CAN_TxPrepareSignalData(&signalData, cantx_signalMaximumDischargePower);
    uint64_t data = (uint64_t)signalData;
    return data;
}

static uint64_t CANTX_CalculateMaxChargePower(const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    /* maximum charge power = charge_current_A * battery_voltage_V */
    float_t signalData =
        ((float_t)kpkCanShim->pTableSof->recommendedContinuousPackChargeCurrent_mA *
         UNIT_CONVERSION_FACTOR_1_1000_TH_FLOAT) *
        ((float_t)kpkCanShim->pTablePackValues->batteryVoltage_mV * UNIT_CONVERSION_FACTOR_1_1000_TH_FLOAT);
    CAN_TxPrepareSignalData(&signalData, cantx_signalMaximumChargePower);
    uint64_t data = (uint64_t)signalData;
    return data;
}

static void CANTX_BuildP0Message(const CAN_SHIM_s *const kpkCanShim, uint64_t *pMessageData) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);
    FAS_ASSERT(pMessageData != NULL_PTR);

    /* Battery voltage */
    uint64_t data = CANTX_CalculateBatteryVoltage(kpkCanShim);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        cantx_signalBatteryVoltage.bitStart,
        cantx_signalBatteryVoltage.bitLength,
        data,
        CANTX_PACK_VALUES_P0_ENDIANNESS);

    /* Average SoE */
    data = CANTX_CalculateAverageSoE(kpkCanShim);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        cantx_signalAverageSoe.bitStart,
        cantx_signalAverageSoe.bitLength,
        data,
        CANTX_PACK_VALUES_P0_ENDIANNESS);

    /* Maximum discharge power */
    data = CANTX_CalculateMaxDischargePower(kpkCanShim);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        cantx_signalMaximumDischargePower.bitStart,
        cantx_signalMaximumDischargePower.bitLength,
        data,
        CANTX_PACK_VALUES_P0_ENDIANNESS);

    /* Maximum charge power */
    data = CANTX_CalculateMaxChargePower(kpkCanShim);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        cantx_signalMaximumChargePower.bitStart,
        cantx_signalMaximumChargePower.bitLength,
        data,
        CANTX_PACK_VALUES_P0_ENDIANNESS);

    /* Cellsius: CRC */
    data = Compute_CRC8H2F((uint8_t *)pMessageData, CANTX_SIGNAL_PACK_VALUES_CRC_START_BIT / 8u, CRC8H2F_INITIAL_VALUE);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        CANTX_SIGNAL_PACK_VALUES_CRC_START_BIT,
        CANTX_SIGNAL_PACK_VALUES_CRC_LENGTH,
        data,
        CANTX_PACK_VALUES_P0_ENDIANNESS);
}

/*========== Extern Function Implementations ================================*/
extern uint32_t CANTX_PackValuesP0(
    CAN_MESSAGE_PROPERTIES_s message,
    uint8_t *pCanData,
    uint8_t *pMuxId,
    const CAN_SHIM_s *const kpkCanShim) {
    /* pMuxId is not used here, therefore has to be NULL_PTR */
    FAS_ASSERT(pMuxId == NULL_PTR);

    FAS_ASSERT(message.id == CANTX_PACK_VALUES_P0_ID);
    FAS_ASSERT(message.idType == CANTX_PACK_VALUES_P0_ID_TYPE);
    FAS_ASSERT(message.dlc == CAN_FOXBMS_MESSAGES_DEFAULT_DLC);
    FAS_ASSERT(message.endianness == CANTX_PACK_VALUES_P0_ENDIANNESS);
    FAS_ASSERT(pCanData != NULL_PTR);
    FAS_ASSERT(kpkCanShim != NULL_PTR);
    uint64_t messageData = 0u;

    /* Read database entry */
    DATA_READ_DATA(kpkCanShim->pTablePackValues, kpkCanShim->pTableSoe, kpkCanShim->pTableSof);

    /* build message from data */
    CANTX_BuildP0Message(kpkCanShim, &messageData);

    /* now copy data in the buffer that will be used to send data */
    CAN_TxSetCanDataWithMessageData(messageData, pCanData, message.endianness);

    return 0u;
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
extern uint64_t TEST_CANTX_CalculateBatteryVoltage(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_CalculateBatteryVoltage(kpkCanShim);
}
extern uint64_t TEST_CANTX_CalculateBusVoltage(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_CalculateBusVoltage(kpkCanShim);
}
extern uint64_t TEST_CANTX_CalculatePower(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_CalculatePower(kpkCanShim);
}
extern uint64_t TEST_CANTX_CalculateCurrent(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_CalculateCurrent(kpkCanShim);
}
extern void TEST_CANTX_BuildP0Message(const CAN_SHIM_s *const kpkCanShim, uint64_t *pMessageData) {
    CANTX_BuildP0Message(kpkCanShim, pMessageData);
}
#endif
