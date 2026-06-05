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
 * @file    can_cbs_tx_can-diagnostics.c
 * @author  CELLSIUS Project H2
 * @brief   CAN Tx callback for CAN receive diagnostic counters
 */

/*========== Includes =======================================================*/
#include "general.h"

#include "can.h"
/* AXIVION Next Codeline Generic-LocalInclude: 'can_cbs_tx_cyclic.h' declares
 * the prototype for the callback 'CANTX_CanDiagnosticCounters' */
#include "can_cbs_tx_cyclic.h"
#include "can_cfg_tx-cyclic-message-definitions.h"

#include <stdint.h>

/*========== Macros and Definitions =========================================*/

#define CANTX_CAN_DIAGNOSTIC_MUX_VALIDITY_COUNTERS (0u)
#define CANTX_CAN_DIAGNOSTIC_MUX_RX_HEALTH         (1u)
#define CANTX_CAN_DIAGNOSTIC_MUX_MAX               (2u)

#define CANTX_CAN_DIAGNOSTIC_MUX_BYTE          (0u)
#define CANTX_CAN_DIAGNOSTIC_COUNTER_0_LSB     (1u)
#define CANTX_CAN_DIAGNOSTIC_COUNTER_1_LSB     (3u)
#define CANTX_CAN_DIAGNOSTIC_COUNTER_2_LSB     (5u)
#define CANTX_CAN_DIAGNOSTIC_RESERVED_BYTE     (7u)
#define CANTX_CAN_DIAGNOSTIC_BYTE_SHIFT        (8u)
#define CANTX_CAN_DIAGNOSTIC_LOWER_16_BIT_MASK (0xFFFFu)

/*========== Static Constant and Variable Definitions =======================*/

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/

/**
 * @brief   Copies one 16-bit diagnostic counter into a CAN payload.
 * @param   pCanData   payload of CAN frame
 * @param   lsbIndex   payload byte for counter LSB
 * @param   counter    counter value
 */
static void CANTX_SetCounter16(uint8_t *pCanData, uint8_t lsbIndex, uint32_t counter);

/*========== Static Function Implementations ================================*/

static void CANTX_SetCounter16(uint8_t *pCanData, uint8_t lsbIndex, uint32_t counter) {
    FAS_ASSERT(pCanData != NULL_PTR);
    FAS_ASSERT((lsbIndex + 1u) < CAN_DEFAULT_DLC);

    const uint16_t counter16 = (uint16_t)(counter & CANTX_CAN_DIAGNOSTIC_LOWER_16_BIT_MASK);
    pCanData[lsbIndex]       = (uint8_t)(counter16 & UINT8_MAX);
    pCanData[lsbIndex + 1u]  = (uint8_t)(counter16 >> CANTX_CAN_DIAGNOSTIC_BYTE_SHIFT);
}

/*========== Extern Function Implementations ================================*/

extern uint32_t CANTX_CanDiagnosticCounters(
    CAN_MESSAGE_PROPERTIES_s message,
    uint8_t *pCanData,
    uint8_t *pMuxId,
    const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(message.id == CANTX_CAN_DIAGNOSTIC_COUNTERS_ID);
    FAS_ASSERT(message.idType == CANTX_CAN_DIAGNOSTIC_COUNTERS_ID_TYPE);
    FAS_ASSERT(message.dlc == CAN_FOXBMS_MESSAGES_DEFAULT_DLC);
    FAS_ASSERT(message.endianness == CANTX_CAN_DIAGNOSTIC_COUNTERS_ENDIANNESS);
    FAS_ASSERT(pCanData != NULL_PTR);
    FAS_ASSERT(pMuxId != NULL_PTR);
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    (void)kpkCanShim;

    CAN_DIAGNOSTIC_COUNTERS_s counters = {0u};
    CAN_GetDiagnosticCounters(&counters);

    for (uint8_t i = 0u; i < CAN_DEFAULT_DLC; i++) {
        pCanData[i] = 0u;
    }

    if (*pMuxId >= CANTX_CAN_DIAGNOSTIC_MUX_MAX) {
        *pMuxId = CANTX_CAN_DIAGNOSTIC_MUX_VALIDITY_COUNTERS;
    }

    pCanData[CANTX_CAN_DIAGNOSTIC_MUX_BYTE]      = *pMuxId;
    pCanData[CANTX_CAN_DIAGNOSTIC_RESERVED_BYTE] = 0u;

    /* Byte 0 selects the counter group; bytes 1-6 contain three little-endian uint16 counters. */
    if (*pMuxId == CANTX_CAN_DIAGNOSTIC_MUX_VALIDITY_COUNTERS) {
        CANTX_SetCounter16(pCanData, CANTX_CAN_DIAGNOSTIC_COUNTER_0_LSB, counters.ecuStateValid);
        CANTX_SetCounter16(pCanData, CANTX_CAN_DIAGNOSTIC_COUNTER_1_LSB, counters.ecuStateCrcInvalid);
        CANTX_SetCounter16(pCanData, CANTX_CAN_DIAGNOSTIC_COUNTER_2_LSB, counters.dhvcStateValid);
    } else {
        CANTX_SetCounter16(pCanData, CANTX_CAN_DIAGNOSTIC_COUNTER_0_LSB, counters.dhvcStateCrcInvalid);
        CANTX_SetCounter16(pCanData, CANTX_CAN_DIAGNOSTIC_COUNTER_1_LSB, counters.rxDataLost);
        CANTX_SetCounter16(pCanData, CANTX_CAN_DIAGNOSTIC_COUNTER_2_LSB, counters.rxQueueFull);
    }

    (*pMuxId)++;
    if (*pMuxId >= CANTX_CAN_DIAGNOSTIC_MUX_MAX) {
        *pMuxId = CANTX_CAN_DIAGNOSTIC_MUX_VALIDITY_COUNTERS;
    }

    return 0u;
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
#endif
