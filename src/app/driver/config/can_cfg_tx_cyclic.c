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
 * @file    can_cfg_tx_cyclic.c
 * @author  foxBMS Team
 * @date    2019-12-04 (date of creation)
 * @updated 2025-03-31 (date of last update)
 * @version v1.9.0
 * @ingroup DRIVERS_CONFIGURATION
 * @prefix  CANTX
 *
 * @brief   Configuration for the CAN module
 * @details The CAN bus settings and the transmitted messages and their
 *          reception handling are to be specified here.
 */

/*========== Includes =======================================================*/
#include "can_cfg.h"

#include "can_cbs_tx_cyclic.h"
#include "can_cfg_tx-cyclic-message-definitions.h"
#include "database.h"
#include "ftask.h"

#include <stdint.h>

/*========== Macros and Definitions =========================================*/

/*========== Static Constant and Variable Definitions =======================*/

/** Multiplexer values @{*/
static uint8_t cantx_cellVoltagesMux               = 0u;
static uint8_t cantx_cellTemperaturesMux           = 0u;
static uint8_t cantx_stringStateMux                = 0u;
static uint8_t cantx_stringValuesP0Mux             = 0u;
static uint8_t cantx_stringValuesP1Mux             = 0u;
static uint8_t cantx_stringMinimumMaximumValuesMux = 0u;
static uint8_t cantx_stringStateEstimationMux      = 0u;
/**@}*/

/*========== Extern Constant and Variable Definitions =======================*/

/** registry of CAN TX messages */
const CAN_TX_MESSAGE_TYPE_s can_txMessages[] = {
    {CAN_NODE, CANTX_BMS_STATE_MESSAGE, &CANTX_BmsState, NULL_PTR},
    {CAN_NODE, CANTX_BMS_STATE_DETAILS_MESSAGE, &CANTX_BmsStateDetails, NULL_PTR},
    {CAN_NODE, CANTX_CELL_VOLTAGES_MESSAGE, &CANTX_CellVoltages, &cantx_cellVoltagesMux},
    {CAN_NODE, CANTX_CELL_TEMPERATURES_MESSAGE, &CANTX_CellTemperatures, &cantx_cellTemperaturesMux},
    {CAN_NODE, CANTX_PACK_LIMITS_MESSAGE, &CANTX_PackLimits, NULL_PTR},
    {CAN_NODE, CANTX_PACK_MINIMUM_MAXIMUM_VALUES_MESSAGE, &CANTX_PackMinimumMaximumValues, NULL_PTR},
    {CAN_NODE, CANTX_PACK_STATE_ESTIMATION_MESSAGE, &CANTX_PackStateEstimation, NULL_PTR},
    {CAN_NODE, CANTX_PACK_VALUES_P0_MESSAGE, &CANTX_PackValuesP0, NULL_PTR},
    {CAN_NODE, CANTX_PACK_VALUES_P1_MESSAGE, &CANTX_PackValuesP1, NULL_PTR},
    {CAN_NODE, CANTX_STRING_STATE_MESSAGE, &CANTX_StringState, &cantx_stringStateMux},
    {CAN_NODE,
     CANTX_STRING_MINIMUM_MAXIMUM_VALUES_MESSAGE,
     &CANTX_StringMinimumMaximumValues,
     &cantx_stringMinimumMaximumValuesMux},
    {CAN_NODE, CANTX_STRING_STATE_ESTIMATION_MESSAGE, &CANTX_StringStateEstimation, &cantx_stringStateEstimationMux},
    {CAN_NODE, CANTX_STRING_VALUES_P0_MESSAGE, &CANTX_StringValuesP0, &cantx_stringValuesP0Mux},
    {CAN_NODE, CANTX_STRING_VALUES_P1_MESSAGE, &CANTX_StringValuesP1, &cantx_stringValuesP1Mux}};

/** length of CAN message arrays */
const uint8_t can_txMessagesLength = sizeof(can_txMessages) / sizeof(can_txMessages[0]);

/*========== Static Function Prototypes =====================================*/

/*========== Static Function Implementations ================================*/

/*========== Extern Function Implementations ================================*/

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
#endif
