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
 * @file    can_cbs_tx_pack-limits.c
 * @author  foxBMS Team
 * @date    2021-07-21 (date of creation)
 * @updated 2025-03-31 (date of last update)
 * @version v1.9.0
 * @ingroup DRIVERS
 * @prefix  CANTX
 *
 * @brief   CAN driver Tx callback implementation
 * @details CAN Tx callback for state messages
 */

/*========== Includes =======================================================*/
#include "battery_cell_cfg.h"

/* AXIVION Next Codeline Generic-LocalInclude: 'can_cbs_tx_cyclic.h' declares
 * the prototype for the callback 'CANTX_PackLimits' */
#include "can_cbs_tx_cyclic.h"
#include "can_cfg_tx-cyclic-message-definitions.h"
#include "can_helper.h"
#include "foxmath.h"

#include <math.h>
#include <stdint.h>

/*========== Macros and Definitions =========================================*/
/** @{
 * defines of the maximum discharge current signal
*/
#define CANTX_SIGNAL_MAXIMUM_DISCHARGE_CURRENT_START_BIT     (0u)
#define CANTX_SIGNAL_MAXIMUM_DISCHARGE_CURRENT_LENGTH        (12u)
#define CANTX_MINIMUM_VALUE_MAXIMUM_DISCHARGE_CURRENT_SIGNAL (0.0f)
#define CANTX_MAXIMUM_VALUE_MAXIMUM_DISCHARGE_CURRENT_SIGNAL (1023750.0f)
#define CANTX_FACTOR_MAXIMUM_DISCHARGE_CURRENT               (250.0f)
/** @} */

/** @{
 * configuration of the maximum discharge current signal
*/
static const CAN_SIGNAL_TYPE_s cantx_signalMaximumDischargeCurrent = {
    CANTX_SIGNAL_MAXIMUM_DISCHARGE_CURRENT_START_BIT,
    CANTX_SIGNAL_MAXIMUM_DISCHARGE_CURRENT_LENGTH,
    CANTX_FACTOR_MAXIMUM_DISCHARGE_CURRENT,
    CAN_SIGNAL_OFFSET_0,
    CANTX_MINIMUM_VALUE_MAXIMUM_DISCHARGE_CURRENT_SIGNAL,
    CANTX_MAXIMUM_VALUE_MAXIMUM_DISCHARGE_CURRENT_SIGNAL};
/** @} */

/** @{
 * defines of the maximum charge current signal
*/
#define CANTX_SIGNAL_MAXIMUM_CHARGE_CURRENT_START_BIT     (12u)
#define CANTX_SIGNAL_MAXIMUM_CHARGE_CURRENT_LENGTH        (12u)
#define CANTX_MINIMUM_VALUE_MAXIMUM_CHARGE_CURRENT_SIGNAL (0.0f)
#define CANTX_MAXIMUM_VALUE_MAXIMUM_CHARGE_CURRENT_SIGNAL (1023750.0f)
#define CANTX_FACTOR_MAXIMUM_CHARGE_CURRENT               (250.0f)
/** @} */

/** @{
 * configuration of the maximum charge current signal
*/
static const CAN_SIGNAL_TYPE_s cantx_signalMaximumChargeCurrent = {
    CANTX_SIGNAL_MAXIMUM_CHARGE_CURRENT_START_BIT,
    CANTX_SIGNAL_MAXIMUM_CHARGE_CURRENT_LENGTH,
    CANTX_FACTOR_MAXIMUM_CHARGE_CURRENT,
    CAN_SIGNAL_OFFSET_0,
    CANTX_MINIMUM_VALUE_MAXIMUM_CHARGE_CURRENT_SIGNAL,
    CANTX_MAXIMUM_VALUE_MAXIMUM_CHARGE_CURRENT_SIGNAL};
/** @} */

/*========== Static Constant and Variable Definitions =======================*/
/**
 * @brief   Calculates the return value of the maximum discharge current
 * @param   kpkCanShim const pointer to CAN shim
 * @return  Returns the return value of the maximum discharge current
 */
static uint64_t CANTX_CalculateMaximumDischargeCurrent(const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief   Calculates the return value of the maximum charge current
 * @param   kpkCanShim const pointer to CAN shim
 * @return  Returns the return value of the maximum charge current
 */
static uint64_t CANTX_CalculateMaximumChargeCurrent(const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief   Adds the data to the message
 * @param   pMessageData message data of the CAN message
 * @param   kpkCanShim const pointer to CAN shim
 */
static void CANTX_BuildPackLimitsMessage(uint64_t *pMessageData, const CAN_SHIM_s *const kpkCanShim);

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/

/*========== Static Function Implementations ================================*/
static uint64_t CANTX_CalculateMaximumDischargeCurrent(const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    /* maximum discharge current */
    float_t signalData = (float_t)kpkCanShim->pTableSof->recommendedContinuousPackDischargeCurrent_mA;
    CAN_TxPrepareSignalData(&signalData, cantx_signalMaximumDischargeCurrent);
    uint64_t data = (uint64_t)signalData;
    return data;
}

static uint64_t CANTX_CalculateMaximumChargeCurrent(const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    /* maximum charge current */
    float_t signalData = (float_t)kpkCanShim->pTableSof->recommendedContinuousPackChargeCurrent_mA;
    CAN_TxPrepareSignalData(&signalData, cantx_signalMaximumChargeCurrent);
    uint64_t data = (uint64_t)signalData;
    return data;
}

static void CANTX_BuildPackLimitsMessage(uint64_t *pMessageData, const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(pMessageData != NULL_PTR);
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    /* Maximum discharge current */
    uint64_t data = CANTX_CalculateMaximumDischargeCurrent(kpkCanShim);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        cantx_signalMaximumDischargeCurrent.bitStart,
        cantx_signalMaximumDischargeCurrent.bitLength,
        data,
        CANTX_PACK_LIMITS_ENDIANNESS);
    /* Maximum charge current */
    data = CANTX_CalculateMaximumChargeCurrent(kpkCanShim);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        cantx_signalMaximumChargeCurrent.bitStart,
        cantx_signalMaximumChargeCurrent.bitLength,
        data,
        CANTX_PACK_LIMITS_ENDIANNESS);
}

/*========== Extern Function Implementations ================================*/
extern uint32_t CANTX_PackLimits(
    CAN_MESSAGE_PROPERTIES_s message,
    uint8_t *pCanData,
    uint8_t *pMuxId,
    const CAN_SHIM_s *const kpkCanShim) {
    /* pMuxId is not used here, therefore has to be NULL_PTR */
    FAS_ASSERT(pMuxId == NULL_PTR);
    FAS_ASSERT(message.id == CANTX_PACK_LIMITS_ID);
    FAS_ASSERT(message.idType == CANTX_PACK_LIMITS_ID_TYPE);
    FAS_ASSERT(message.dlc == CANTX_PACK_LIMITS_DLC);
    FAS_ASSERT(message.endianness == CANTX_PACK_LIMITS_ENDIANNESS);
    FAS_ASSERT(pCanData != NULL_PTR);
    FAS_ASSERT(kpkCanShim != NULL_PTR);
    uint64_t messageData = 0u;

    DATA_READ_DATA(kpkCanShim->pTableSof, can_kShim.pTablePackValues);

    CANTX_BuildPackLimitsMessage(&messageData, kpkCanShim);

    /* now copy data in the buffer that will be used to send data */
    CAN_TxSetCanDataWithMessageData(messageData, pCanData, message.endianness);

    return 0u;
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
extern uint64_t TEST_CANTX_CalculateMaximumDischargeCurrent(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_CalculateMaximumDischargeCurrent(kpkCanShim);
}
extern uint64_t TEST_CANTX_CalculateMaximumChargeCurrent(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_CalculateMaximumChargeCurrent(kpkCanShim);
}
extern uint64_t TEST_CANTX_CalculateMaximumDischargePower(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_CalculateMaximumDischargePower(kpkCanShim);
}
extern uint64_t TEST_CANTX_CalculateMaximumChargePower(const CAN_SHIM_s *const kpkCanShim) {
    return CANTX_CalculateMaximumChargePower(kpkCanShim);
}
extern uint64_t TEST_CANTX_CalculateMinimumBatteryVoltage(void) {
    return CANTX_CalculateMinimumBatteryVoltage();
}
extern uint64_t TEST_CANTX_CalculateMaximumBatteryVoltage(void) {
    return CANTX_CalculateMaximumBatteryVoltage();
}
extern void TEST_CANTX_BuildPackLimitsMessage(uint64_t *pMessageData, const CAN_SHIM_s *const kpkCanShim) {
    CANTX_BuildPackLimitsMessage(pMessageData, kpkCanShim);
}

#endif
