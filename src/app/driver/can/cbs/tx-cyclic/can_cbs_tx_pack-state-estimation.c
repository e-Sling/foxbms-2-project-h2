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
 * @file    can_cbs_tx_pack-state-estimation.c
 * @author  foxBMS Team
 * @date    2021-07-21 (date of creation)
 * @updated 2025-03-31 (date of last update)
 * @version v1.9.0
 * @ingroup DRIVERS
 * @prefix  CANTX
 *
 * @brief   CAN driver Tx callback implementation
 * @details CAN Tx callback for state estimation messages
 */

/*========== Includes =======================================================*/
#include "bms.h"
/* AXIVION Next Codeline Generic-LocalInclude: 'can_cbs_tx_cyclic.h' declares
 * the prototype for the callback 'CANTX_PackStateEstimation' */
#include "can_cbs_tx_cyclic.h"
#include "can_cfg_tx-cyclic-message-definitions.h"
#include "can_helper.h"
#include "foxmath.h"

#include <float.h>
#include <math.h>
#include <stdint.h>

/*========== Macros and Definitions =========================================*/
#define CANTX_100_PERCENT_FLOAT (100.0f)

#define CANTX_SIGNAL_MINIMUM_SOC_START_BIT (0u)
#define CANTX_SIGNAL_MINIMUM_SOC_LENGTH    (10u)
#define CANTX_SIGNAL_MAXIMUM_SOC_START_BIT (10u)
#define CANTX_SIGNAL_MAXIMUM_SOC_LENGTH    (10u)
#define CANTX_SIGNAL_AVERAGE_SOC_START_BIT (20u)
#define CANTX_SIGNAL_AVERAGE_SOC_LENGTH    (10u)
#define CANTX_SIGNAL_MINIMUM_SOE_START_BIT (30u)
#define CANTX_SIGNAL_MINIMUM_SOE_LENGTH    (10u)
#define CANTX_SIGNAL_MAXIMUM_SOE_START_BIT (40u)
#define CANTX_SIGNAL_MAXIMUM_SOE_LENGTH    (10u)

#define CANTX_MINIMUM_VALUE_PERCENT_SIGNALS (0.0f)
#define CANTX_MAXIMUM_VALUE_PERCENT_SIGNALS (102.3f)

/** @{
 * configuration of the minimum soc signal
*/
static const CAN_SIGNAL_TYPE_s cantx_signalMinimumSoc = {
    CANTX_SIGNAL_MINIMUM_SOC_START_BIT,
    CANTX_SIGNAL_MINIMUM_SOC_LENGTH,
    UNIT_CONVERSION_FACTOR_1_10_TH_FLOAT,
    CAN_SIGNAL_OFFSET_0,
    CANTX_MINIMUM_VALUE_PERCENT_SIGNALS,
    CANTX_MAXIMUM_VALUE_PERCENT_SIGNALS};
/** @} */

/** @{
 * configuration of the maximum soc signal
*/
static const CAN_SIGNAL_TYPE_s cantx_signalMaximumSoc = {
    CANTX_SIGNAL_MAXIMUM_SOC_START_BIT,
    CANTX_SIGNAL_MAXIMUM_SOC_LENGTH,
    UNIT_CONVERSION_FACTOR_1_10_TH_FLOAT,
    CAN_SIGNAL_OFFSET_0,
    CANTX_MINIMUM_VALUE_PERCENT_SIGNALS,
    CANTX_MAXIMUM_VALUE_PERCENT_SIGNALS};
/** @} */

/** @{
 * configuration of the average soc signal
*/
static const CAN_SIGNAL_TYPE_s cantx_signalAverageSoc = {
    CANTX_SIGNAL_AVERAGE_SOC_START_BIT,
    CANTX_SIGNAL_AVERAGE_SOC_LENGTH,
    UNIT_CONVERSION_FACTOR_1_10_TH_FLOAT,
    CAN_SIGNAL_OFFSET_0,
    CANTX_MINIMUM_VALUE_PERCENT_SIGNALS,
    CANTX_MAXIMUM_VALUE_PERCENT_SIGNALS};
/** @} */

/** @{
 * configuration of the minimum soe signal
*/
static const CAN_SIGNAL_TYPE_s cantx_signalMinimumSoe = {
    CANTX_SIGNAL_MINIMUM_SOE_START_BIT,
    CANTX_SIGNAL_MINIMUM_SOC_LENGTH,
    UNIT_CONVERSION_FACTOR_1_10_TH_FLOAT,
    CAN_SIGNAL_OFFSET_0,
    CANTX_MINIMUM_VALUE_PERCENT_SIGNALS,
    CANTX_MAXIMUM_VALUE_PERCENT_SIGNALS};
/** @} */

/** @{
 * configuration of the maximum soe signal
*/
static const CAN_SIGNAL_TYPE_s cantx_signalMaximumSoe = {
    CANTX_SIGNAL_MAXIMUM_SOE_START_BIT,
    CANTX_SIGNAL_MAXIMUM_SOE_LENGTH,
    UNIT_CONVERSION_FACTOR_1_10_TH_FLOAT,
    CAN_SIGNAL_OFFSET_0,
    CANTX_MINIMUM_VALUE_PERCENT_SIGNALS,
    CANTX_MAXIMUM_VALUE_PERCENT_SIGNALS};
/** @} */

/*========== Static Constant and Variable Definitions =======================*/

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/
/**
 * @brief Calculates the return value of the maximum SOC
 * @return Returns the return value of the maximum SOC
 */
static uint64_t CANTX_CalculateMaximumPackSoc(const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief Calculates the return value of the minimum SOC
 * @return Returns the return value of the minimum SOC
 */
static uint64_t CANTX_CalculateMinimumPackSoc(const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief Calculates the return value of the average SOC
 * @return Returns the return value of the average SOC
 */
static uint64_t CANTX_CalculateAveragePackSoc(const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief Calculates the return value of the maximum SOE
 * @return Returns the return value of the maximum SOE
 */
static uint64_t CANTX_CalculateMaximumPackSoe(const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief Calculates the return value of the minimum SOE
 * @return Returns the return value of the minimum SOE
 */
static uint64_t CANTX_CalculateMinimumPackSoe(const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief Builds the CAN message form signal data
 */
static void CANTX_BuildPackStateEstimationMessage(const CAN_SHIM_s *const kpkCanShim, uint64_t *pMessageData);

/*========== Static Function Implementations ================================*/
static uint64_t CANTX_CalculateMaximumPackSoc(const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    /* Cellsius: Get maximum SOC percentage, only one string */
    float_t signalData = kpkCanShim->pTableSoc->maximumSoc_perc[BS_STRING0];

    CAN_TxPrepareSignalData(&signalData, cantx_signalMaximumSoc);
    uint64_t data = (uint64_t)signalData;
    return data;
}

static uint64_t CANTX_CalculateMinimumPackSoc(const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    /* Cellsius: Get minimum SOC percentage, only one string */
    float_t signalData = kpkCanShim->pTableSoc->minimumSoc_perc[BS_STRING0];

    CAN_TxPrepareSignalData(&signalData, cantx_signalMinimumSoc);
    uint64_t data = (uint64_t)signalData;
    return data;
}

static uint64_t CANTX_CalculateAveragePackSoc(const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    /* Cellsius: Get average SOC percentage, only one string */
    float_t signalData = kpkCanShim->pTableSoc->averageSoc_perc[BS_STRING0];

    CAN_TxPrepareSignalData(&signalData, cantx_signalAverageSoc);
    uint64_t data = (uint64_t)signalData;
    return data;
}

static uint64_t CANTX_CalculateMaximumPackSoe(const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    /* Cellsius: Get maximum SOE percentage, only one string */
    float_t signalData = kpkCanShim->pTableSoe->maximumSoe_perc[BS_STRING0];

    CAN_TxPrepareSignalData(&signalData, cantx_signalMaximumSoe);
    uint64_t data = (uint64_t)signalData;
    return data;
}

static uint64_t CANTX_CalculateMinimumPackSoe(const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    /* Cellsius: Get minimum SOE percentage, only one string */
    float_t signalData = kpkCanShim->pTableSoe->minimumSoe_perc[BS_STRING0];

    CAN_TxPrepareSignalData(&signalData, cantx_signalMinimumSoe);
    uint64_t data = (uint64_t)signalData;
    return data;
}

static void CANTX_BuildPackStateEstimationMessage(const CAN_SHIM_s *const kpkCanShim, uint64_t *pMessageData) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);
    FAS_ASSERT(pMessageData != NULL_PTR);

    /* minimum SOC */
    uint64_t data = CANTX_CalculateMinimumPackSoc(kpkCanShim);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        cantx_signalMinimumSoc.bitStart,
        cantx_signalMinimumSoc.bitLength,
        data,
        CANTX_PACK_STATE_ESTIMATION_ENDIANNESS);
    /* maximum SOC */
    data = CANTX_CalculateMaximumPackSoc(kpkCanShim);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        cantx_signalMaximumSoc.bitStart,
        cantx_signalMaximumSoc.bitLength,
        data,
        CANTX_PACK_STATE_ESTIMATION_ENDIANNESS);
    /* average SOC */
    data = CANTX_CalculateAveragePackSoc(kpkCanShim);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        cantx_signalAverageSoc.bitStart,
        cantx_signalAverageSoc.bitLength,
        data,
        CANTX_PACK_STATE_ESTIMATION_ENDIANNESS);
    /* minimum SOE*/
    data = CANTX_CalculateMinimumPackSoe(kpkCanShim);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        cantx_signalMinimumSoe.bitStart,
        cantx_signalMinimumSoe.bitLength,
        data,
        CANTX_PACK_STATE_ESTIMATION_ENDIANNESS);
    /* maximum SOE */
    data = CANTX_CalculateMaximumPackSoe(kpkCanShim);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        cantx_signalMaximumSoe.bitStart,
        cantx_signalMaximumSoe.bitLength,
        data,
        CANTX_PACK_STATE_ESTIMATION_ENDIANNESS);
}

/*========== Extern Function Implementations ================================*/
extern uint32_t CANTX_PackStateEstimation(
    CAN_MESSAGE_PROPERTIES_s message,
    uint8_t *pCanData,
    uint8_t *pMuxId,
    const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(message.id == CANTX_PACK_STATE_ESTIMATION_ID);
    FAS_ASSERT(message.idType == CANTX_PACK_STATE_ESTIMATION_ID_TYPE);
    FAS_ASSERT(message.dlc == CAN_FOXBMS_MESSAGES_DEFAULT_DLC);
    FAS_ASSERT(message.endianness == CANTX_PACK_STATE_ESTIMATION_ENDIANNESS);
    FAS_ASSERT(pCanData != NULL_PTR);
    FAS_ASSERT(pMuxId == NULL_PTR); /* pMuxId is not used here, therefore has to be NULL_PTR */
    FAS_ASSERT(kpkCanShim != NULL_PTR);
    uint64_t messageData = 0u;

    DATA_READ_DATA(kpkCanShim->pTableSoc, kpkCanShim->pTableSoe);

    /* build CAN message */
    CANTX_BuildPackStateEstimationMessage(kpkCanShim, &messageData);
    /* now copy data in the buffer that will be used to send data */
    CAN_TxSetCanDataWithMessageData(messageData, pCanData, message.endianness);

    return 0u;
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
extern uint64_t TEST_CANTX_CalculateMaximumPackSoc(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_CalculateMaximumPackSoc(kpkCanShim);
}
extern float_t TEST_CANTX_GetMaximumStringSoc(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_GetMaximumStringSoc(kpkCanShim);
}
extern uint64_t TEST_CANTX_CalculateMinimumPackSoc(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_CalculateMinimumPackSoc(kpkCanShim);
}
extern float_t TEST_CANTX_GetMinimumStringSoc(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_GetMinimumStringSoc(kpkCanShim);
}
extern uint64_t TEST_CANTX_CalculateMaximumPackSoe(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_CalculateMaximumPackSoe(kpkCanShim);
}
extern float_t TEST_CANTX_GetMaximumStringSoe(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_GetMaximumStringSoe(kpkCanShim);
}
extern uint64_t TEST_CANTX_CalculateMinimumPackSoe(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_CalculateMinimumPackSoe(kpkCanShim);
}
extern float_t TEST_CANTX_GetMinimumStringSoe(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_GetMinimumStringSoe(kpkCanShim);
}
extern uint64_t TEST_CANTX_CalculatePackSoh(void) {
    return CANTX_CalculatePackSoh();
}
extern uint64_t TEST_CANTX_CalculatePackEnergy(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_CalculatePackEnergy(kpkCanShim);
}
extern float_t TEST_CANTX_GetStringEnergy(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_GetStringEnergy(kpkCanShim);
}
extern void TEST_CANTX_BuildPackStateEstimationMessage(const CAN_SHIM_s *const kpkCanShim, uint64_t *pMessageData) {
    CANTX_BuildPackStateEstimationMessage(kpkCanShim, pMessageData);
}

#endif
