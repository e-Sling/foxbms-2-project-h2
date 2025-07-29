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
 * @file    bms.c
 * @author  foxBMS Team
 * @date    2020-02-24 (date of creation)
 * @updated 2025-03-31 (date of last update)
 * @version v1.9.0
 * @ingroup ENGINE
 * @prefix  BMS
 *
 * @brief   Bms driver implementation
 * @details Implements the state machine that controls the BMS
 *
 */

/*========== Includes =======================================================*/
#include "bms.h"

#include "battery_cell_cfg.h"

#include "afe.h"
#include "bal.h"
#include "can_cbs_tx_cyclic.h"
#include "database.h"
#include "diag.h"
#include "foxmath.h"
#include "imd.h"
#include "led.h"
#include "meas.h"
#include "os.h"
#include "soa.h"
#include "sps.h"

#include <stdbool.h>
#include <stdint.h>

/*========== Macros and Definitions =========================================*/
/** default value for unset "active delay time" */
#define BMS_NO_ACTIVE_DELAY_TIME_ms (UINT32_MAX)

/**
 * Saves the last state and the last substate
 */
#define BMS_SAVE_LAST_STATES()                \
    bms_state.lastState    = bms_state.state; \
    bms_state.lastSubstate = bms_state.substate

/*========== Static Constant and Variable Definitions =======================*/

/**
 * contains the state of the bms state machine
 */
static BMS_STATE_s bms_state = {
    .timer                             = 0,
    .stateRequest                      = BMS_STATE_NO_REQUEST,
    .state                             = BMS_STATEMACH_UNINITIALIZED,
    .substate                          = BMS_ENTRY,
    .lastState                         = BMS_STATEMACH_UNINITIALIZED,
    .lastSubstate                      = BMS_ENTRY,
    .triggerentry                      = 0u,
    .ErrRequestCounter                 = 0u,
    .initFinished                      = STD_NOT_OK,
    .counter                           = 0u,
    .OscillationTimeout                = 0u,
    .prechargeTryCounter               = 0u,
    .powerPath                         = BMS_POWER_PATH_OPEN,
    .closedStrings                     = {0u},
    .closedPrechargeContactors         = {0u},
    .numberOfClosedStrings             = 0u,
    .deactivatedStrings                = {0},
    .firstClosedString                 = 0u,
    .stringOpenTimeout                 = 0u,
    .nextStringClosedTimer             = 0u,
    .stringCloseTimeout                = 0u,
    .nextState                         = BMS_STATEMACH_STANDBY,
    .restTimer_10ms                    = BS_RELAXATION_PERIOD_10ms,
    .currentFlowState                  = BMS_RELAXATION,
    .remainingDelay_ms                 = BMS_NO_ACTIVE_DELAY_TIME_ms,
    .minimumActiveDelay_ms             = BMS_NO_ACTIVE_DELAY_TIME_ms,
    .transitionToErrorState            = false,
    .timeAboveContactorBreakCurrent_ms = 0u,
    .stringToBeOpened                  = 0u,
    .contactorToBeOpened               = CONT_UNDEFINED,
};

/** local copies of database tables */
/**@{*/
/* static DATA_BLOCK_MIN_MAX_s bms_tableMinMax         = {.header.uniqueId = DATA_BLOCK_ID_MIN_MAX};
static DATA_BLOCK_OPEN_WIRE_s bms_tableOpenWire     = {.header.uniqueId = DATA_BLOCK_ID_OPEN_WIRE_BASE}; */
static DATA_BLOCK_PACK_VALUES_s bms_tablePackValues = {.header.uniqueId = DATA_BLOCK_ID_PACK_VALUES};
/**@}*/

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/

/**
 * @brief       checks the state requests that are made.
 * @details     This function checks the validity of the state requests. The
 *              results of the checked is returned immediately.
 * @param[in]   statereq    state request to be checked
 * @return      result of the state request that was made
 */
static BMS_RETURN_TYPE_e BMS_CheckStateRequest(BMS_STATE_REQUEST_e statereq);

/**
 * @brief   transfers the current state request to the state machine.
 * @details This function takes the current state request from #bms_state
 *          transfers it to the state machine. It resets the value from
 *          #bms_state to #BMS_STATE_NO_REQUEST
 * @return  current state request
 */
static BMS_STATE_REQUEST_e BMS_TransferStateRequest(void);

/**
 * @brief   re-entrance check of SYS state machine trigger function
 * @details This function is not re-entrant and should only be called time- or
 *          event-triggered. It increments the triggerentry counter from the
 *          state variable ltc_state. It should never be called by two
 *          different processes, so if it is the case, triggerentry should
 *          never be higher than 0 when this function is called.
 * @return  retval  0 if no further instance of the function is active, 0xff
 *          else
 */
static uint8_t BMS_CheckReEntrance(void);

/**
 * @brief   Checks the state requests made to the BMS state machine.
 * @details Checks of the state request in the database and sets this value as
 *          return value.
 * @return  requested state
 */
static uint8_t BMS_CheckCanRequests(void);

/**
 * @brief   Checks all the error flags from diagnosis module with a severity of
 *          #DIAG_FATAL_ERROR
 * @details Checks all the error flags from diagnosis module with a severity of
 *          #DIAG_FATAL_ERROR. Furthermore, sets parameter minimumActiveDelay_ms
 *          of bms_state variable.
 * @return  true if error flag is set, otherwise false
 */
static bool BMS_IsAnyFatalErrorFlagSet(void);

/**
 * @brief   Checks if any error flag is set and handles delay until contactors
 *          need to be opened.
 * @details Checks all the diagnosis entries with severity of #DIAG_FATAL_ERROR
 *          and handles the configured delay until the contactors need to be
 *          opened. The shortest delay is used, if multiple errors are active at
 *          once.
 * @return  #STD_NOT_OK if error detected and delay time elapsed, otherwise #STD_OK
 */
static STD_RETURN_TYPE_e BMS_IsBatterySystemStateOkay(void);

/**
 * @brief   Checks if the contactor feedback for a specific contactor is valid
 *          need to be opened.
 * @details Reads error flag database entry and checks if the feedback for this
 *          specific contactor is valid or not.
 * @return  true if no error detected feedback is valid, otherwise false
 */
static bool BMS_IsContactorFeedbackValid(uint8_t stringNumber, CONT_TYPE_e contactorType);

/**
 * @brief       Checks if the current limitations are violated
 * @param[in]   stringNumber    string addressed
 * @param[in]   pPackValues     pointer to pack values database entry
 * @return      #STD_OK if the current limitations are NOT violated, else
 *              #STD_NOT_OK (type: #STD_RETURN_TYPE_e)
 */
static STD_RETURN_TYPE_e BMS_CheckPrecharge(uint8_t stringNumber, const DATA_BLOCK_PACK_VALUES_s *pPackValues);

/*========== Static Function Implementations ================================*/

static BMS_RETURN_TYPE_e BMS_CheckStateRequest(BMS_STATE_REQUEST_e statereq) {
    if (statereq == BMS_STATE_ERROR_REQUEST) {
        return BMS_OK;
    }

    if (bms_state.stateRequest == BMS_STATE_NO_REQUEST) {
        /* init only allowed from the uninitialized state */
        if (statereq == BMS_STATE_INIT_REQUEST) {
            if (bms_state.state == BMS_STATEMACH_UNINITIALIZED) {
                return BMS_OK;
            } else {
                return BMS_ALREADY_INITIALIZED;
            }
        } else {
            return BMS_ILLEGAL_REQUEST;
        }
    } else {
        return BMS_REQUEST_PENDING;
    }
}

static uint8_t BMS_CheckReEntrance(void) {
    uint8_t retval = 0;
    OS_EnterTaskCritical();
    if (!bms_state.triggerentry) {
        bms_state.triggerentry++;
    } else {
        retval = 0xFF; /* multiple calls of function */
    }
    OS_ExitTaskCritical();
    return retval;
}

static BMS_STATE_REQUEST_e BMS_TransferStateRequest(void) {
    BMS_STATE_REQUEST_e retval = BMS_STATE_NO_REQUEST;

    OS_EnterTaskCritical();
    retval                 = bms_state.stateRequest;
    bms_state.stateRequest = BMS_STATE_NO_REQUEST;
    OS_ExitTaskCritical();
    return retval;
}

static uint8_t BMS_CheckCanRequests(void) {
    uint8_t retVal                     = BMS_REQ_ID_NOREQ;
    DATA_BLOCK_STATE_REQUEST_s request = {.header.uniqueId = DATA_BLOCK_ID_STATE_REQUEST};

    DATA_READ_DATA(&request);

    if (request.stateRequestViaCan == BMS_REQ_ID_STANDBY) {
        retVal = BMS_REQ_ID_STANDBY;
    } else if (request.stateRequestViaCan == BMS_REQ_ID_NORMAL) {
        retVal = BMS_REQ_ID_NORMAL;
    } else if (request.stateRequestViaCan == BMS_REQ_ID_CHARGE) {
        retVal = BMS_REQ_ID_CHARGE;
    } else if (request.stateRequestViaCan == BMS_REQ_ID_NOREQ) {
        retVal = BMS_REQ_ID_NOREQ;
    } else {
        /* invalid or no request, default to BMS_REQ_ID_NOREQ (already set) */
    }

    return retVal;
}

static STD_RETURN_TYPE_e BMS_CheckPrecharge(uint8_t stringNumber, const DATA_BLOCK_PACK_VALUES_s *pPackValues) {
    STD_RETURN_TYPE_e retVal = STD_NOT_OK;
    /* make sure that we do not access the arrays in the database
       tables out of bounds */
    FAS_ASSERT(stringNumber < BS_NR_OF_STRINGS);
    FAS_ASSERT(pPackValues != NULL_PTR);

    /* Only check precharging if current value and voltages are valid */
    if ((pPackValues->invalidStringCurrent[stringNumber] == 0u) &&
        (pPackValues->invalidStringVoltage[stringNumber] == 0u) && (pPackValues->invalidHvBusVoltage == 0u)) {
        /* Only current not current direction is checked */
        const int32_t current_mA                = MATH_AbsInt32_t(pPackValues->stringCurrent_mA[stringNumber]);
        const int64_t cont_prechargeVoltDiff_mV = MATH_AbsInt64_t(
            (int64_t)pPackValues->stringVoltage_mV[stringNumber] - (int64_t)pPackValues->highVoltageBusVoltage_mV);

        /* Check if precharging has been successful */
        if ((cont_prechargeVoltDiff_mV < BMS_PRECHARGE_VOLTAGE_THRESHOLD_mV) &&
            (current_mA < BMS_PRECHARGE_CURRENT_THRESHOLD_mA)) {
            retVal = STD_OK;
            (void)DIAG_Handler(DIAG_ID_PRECHARGE_ABORT_REASON_VOLTAGE, DIAG_EVENT_OK, DIAG_STRING, stringNumber);
            (void)DIAG_Handler(DIAG_ID_PRECHARGE_ABORT_REASON_CURRENT, DIAG_EVENT_OK, DIAG_STRING, stringNumber);
        } else {
            if (cont_prechargeVoltDiff_mV >= BMS_PRECHARGE_VOLTAGE_THRESHOLD_mV) {
                /* Voltage difference too large */
                (void)DIAG_Handler(
                    DIAG_ID_PRECHARGE_ABORT_REASON_VOLTAGE, DIAG_EVENT_NOT_OK, DIAG_STRING, stringNumber);
                (void)DIAG_Handler(DIAG_ID_PRECHARGE_ABORT_REASON_CURRENT, DIAG_EVENT_OK, DIAG_STRING, stringNumber);
            }
            if (current_mA >= BMS_PRECHARGE_CURRENT_THRESHOLD_mA) {
                /* Current flow too high */
                (void)DIAG_Handler(
                    DIAG_ID_PRECHARGE_ABORT_REASON_CURRENT, DIAG_EVENT_NOT_OK, DIAG_STRING, stringNumber);
                (void)DIAG_Handler(DIAG_ID_PRECHARGE_ABORT_REASON_VOLTAGE, DIAG_EVENT_OK, DIAG_STRING, stringNumber);
            }
            retVal = STD_NOT_OK;
        }
    }
    return retVal;
}

static bool BMS_IsAnyFatalErrorFlagSet(void) {
    bool fatalErrorActive = false;

    for (uint16_t entry = 0u; entry < diag_device.numberOfFatalErrors; entry++) {
        const STD_RETURN_TYPE_e diagnosisState =
            DIAG_GetDiagnosisEntryState(diag_device.pFatalErrorLinkTable[entry]->id);
        if (STD_NOT_OK == diagnosisState) {
            /* Fatal error detected -> get delay of this error until contactors shall be opened */
            const uint32_t kDelay_ms = DIAG_GetDelay(diag_device.pFatalErrorLinkTable[entry]->id);
            /* Check if delay of detected failure is smaller than the delay of a previously detected failure */
            if (bms_state.minimumActiveDelay_ms > kDelay_ms) {
                bms_state.minimumActiveDelay_ms = kDelay_ms;
            }
            fatalErrorActive = true;
        }
    }
    return fatalErrorActive;
}

static STD_RETURN_TYPE_e BMS_IsBatterySystemStateOkay(void) {
    STD_RETURN_TYPE_e retVal          = STD_OK; /* is set to STD_NOT_OK if error detected */
    static uint32_t previousTimestamp = 0u;
    uint32_t timestamp                = OS_GetTickCount();

    /* Check if any fatal error is detected */
    const bool isErrorActive = BMS_IsAnyFatalErrorFlagSet();

    /** Check if a fatal error has been detected previously. If yes, check delay */
    if (bms_state.transitionToErrorState == true) {
        /* Decrease active delay since last call */
        const uint32_t timeSinceLastCall_ms = timestamp - previousTimestamp;
        if (timeSinceLastCall_ms <= bms_state.remainingDelay_ms) {
            bms_state.remainingDelay_ms -= timeSinceLastCall_ms;
        } else {
            bms_state.remainingDelay_ms = 0u;
        }

        /* Check if delay from a new error is shorter then active delay from
         * previously detected error in BMS state machine */
        if (bms_state.remainingDelay_ms >= bms_state.minimumActiveDelay_ms) {
            bms_state.remainingDelay_ms = bms_state.minimumActiveDelay_ms;
        }
    } else {
        /* Delay is not active, check if it should be activated */
        if (isErrorActive == true) {
            bms_state.transitionToErrorState = true;
            bms_state.remainingDelay_ms      = bms_state.minimumActiveDelay_ms;
        }
    }

    /** Set previous timestamp for next call */
    previousTimestamp = timestamp;

    /* Check if bms state machine should switch to error state. This is the case
     * if the delay is activated and the remaining delay is down to 0 */
    if ((bms_state.transitionToErrorState == true) && (bms_state.remainingDelay_ms == 0u)) {
        retVal = STD_NOT_OK;
    }

    return retVal;
}

static bool BMS_IsContactorFeedbackValid(uint8_t stringNumber, CONT_TYPE_e contactorType) {
    FAS_ASSERT(stringNumber < BS_NR_OF_STRINGS);
    FAS_ASSERT(contactorType != CONT_UNDEFINED);
    bool feedbackValid = false;
    /* Read latest error flags from database */
    DATA_BLOCK_ERROR_STATE_s tableErrorFlags = {.header.uniqueId = DATA_BLOCK_ID_ERROR_STATE};
    DATA_READ_DATA(&tableErrorFlags);
    /* Check if contactor feedback is valid */
    switch (contactorType) {
        case CONT_PLUS:
            if (tableErrorFlags.contactorInPositivePathOfStringFeedbackError[stringNumber] == false) {
                feedbackValid = true;
            }
            break;
        case CONT_MINUS:
            if (tableErrorFlags.contactorInNegativePathOfStringFeedbackError[stringNumber] == false) {
                feedbackValid = true;
            }
            break;
        case CONT_PRECHARGE:
            if (tableErrorFlags.prechargeContactorFeedbackError[stringNumber] == false) {
                feedbackValid = true;
            }
            break;
        case CONT_MAIN:
            if (tableErrorFlags.mainContactorFeedbackError[stringNumber] == false) {
                feedbackValid = true;
            }
            break;
        default:
            /* CONT_UNDEFINED already prevent via assert */
            break;
    }
    return feedbackValid;
}

/*========== Extern Function Implementations ================================*/

extern STD_RETURN_TYPE_e BMS_GetInitializationState(void) {
    return bms_state.initFinished;
}

extern BMS_STATEMACH_e BMS_GetState(void) {
    return bms_state.state;
}

extern BMS_STATEMACH_SUB_e BMS_GetSubstate(void) {
    return bms_state.substate;
}

BMS_RETURN_TYPE_e BMS_SetStateRequest(BMS_STATE_REQUEST_e statereq) {
    BMS_RETURN_TYPE_e retVal = BMS_OK;

    OS_EnterTaskCritical();
    retVal = BMS_CheckStateRequest(statereq);

    if (retVal == BMS_OK) {
        bms_state.stateRequest = statereq;
    }
    OS_ExitTaskCritical();

    return retVal;
}

void BMS_Trigger(void) {
    BMS_STATE_REQUEST_e statereq          = BMS_STATE_NO_REQUEST;
    DATA_BLOCK_SYSTEM_STATE_s systemState = {.header.uniqueId = DATA_BLOCK_ID_SYSTEM_STATE};
    uint32_t timestamp                    = OS_GetTickCount();
    static uint32_t nextOpenWireCheck     = 0;
    STD_RETURN_TYPE_e retVal              = STD_NOT_OK;
    static uint8_t stringNumber           = 0u;
    /*     static uint8_t nextStringNumber             = 0u; */
    CONT_ELECTRICAL_STATE_TYPE_e contactorState = CONT_SWITCH_UNDEFINED;
    bool contactorFeedbackValid                 = false;
    STD_RETURN_TYPE_e contRetVal                = STD_NOT_OK;

    if (bms_state.state != BMS_STATEMACH_UNINITIALIZED) {
        /* BMS_GetMeasurementValues();
        BMS_UpdateBatterySystemState(&bms_tablePackValues);
        SOA_CheckVoltages(&bms_tableMinMax);
        SOA_CheckTemperatures(&bms_tableMinMax, &bms_tablePackValues);
        SOA_CheckCurrent(&bms_tablePackValues);
        SOA_CheckSlaveTemperatures();
        BMS_CheckOpenSenseWire(); */
        CONT_CheckFeedback();
    }
    /* Check re-entrance of function */
    if (BMS_CheckReEntrance() > 0u) {
        return;
    }

    if (bms_state.nextStringClosedTimer > 0u) {
        bms_state.nextStringClosedTimer--;
    }
    if (bms_state.stringOpenTimeout > 0u) {
        bms_state.stringOpenTimeout--;
    }

    if (bms_state.stringCloseTimeout > 0u) {
        bms_state.stringCloseTimeout--;
    }

    if (bms_state.OscillationTimeout > 0u) {
        bms_state.OscillationTimeout--;
    }

    if (bms_state.timer > 0u) {
        if ((--bms_state.timer) > 0u) {
            bms_state.triggerentry--;
            return; /* handle state machine only if timer has elapsed */
        }
    }

    /****Happens every time the state machine is triggered**************/
    switch (bms_state.state) {
        /****************************UNINITIALIZED****************************/
        case BMS_STATEMACH_UNINITIALIZED:
            /* waiting for Initialization Request */
            statereq = BMS_TransferStateRequest();
            if (statereq == BMS_STATE_INIT_REQUEST) {
                BMS_SAVE_LAST_STATES();
                bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                bms_state.state    = BMS_STATEMACH_INITIALIZATION;
                bms_state.substate = BMS_ENTRY;
            } else if (statereq == BMS_STATE_NO_REQUEST) {
                /* no actual request pending */
            } else {
                bms_state.ErrRequestCounter++; /* illegal request pending */
            }
            break;

        /****************************INITIALIZATION***************************/
        case BMS_STATEMACH_INITIALIZATION:
            BMS_SAVE_LAST_STATES();
            /* Reset ALERT mode flag */
            DIAG_Handler(DIAG_ID_ALERT_MODE, DIAG_EVENT_OK, DIAG_SYSTEM, 0u);
            bms_state.initFinished = STD_OK;
            bms_state.timer        = BMS_STATEMACH_LONGTIME;
            bms_state.state        = BMS_STATEMACH_INITIALIZED;
            bms_state.substate     = BMS_ENTRY;
            break;

        /****************************INITIALIZED******************************/
        case BMS_STATEMACH_INITIALIZED:
            BMS_SAVE_LAST_STATES();
            if (IMD_RequestInsulationMeasurement() == IMD_ILLEGAL_REQUEST) {
                /* Initialization of IMD device not finished yet -> wait until this is finished before moving on */
                bms_state.timer = BMS_STATEMACH_LONGTIME;
            } else {
                bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                bms_state.state    = BMS_STATEMACH_IDLE;
                bms_state.substate = BMS_ENTRY;
            }
            break;

        /****************************IDLE*************************************/
        case BMS_STATEMACH_IDLE:
            BMS_SAVE_LAST_STATES();

            if (bms_state.substate == BMS_ENTRY) {
                DATA_READ_DATA(&systemState);
                systemState.bmsCanState = BMS_CAN_STATE_IDLE;
                DATA_WRITE_DATA(&systemState);
                bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                bms_state.substate = BMS_CHECK_ERROR_FLAGS;
                break;
            } else if (bms_state.substate == BMS_CHECK_ERROR_FLAGS) {
                if (BMS_IsBatterySystemStateOkay() == STD_NOT_OK) {
                    bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_ERROR;
                    bms_state.substate  = BMS_ENTRY;
                    break;
                } else {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_CHECK_STATE_REQUESTS;
                    break;
                }
            } else if (bms_state.substate == BMS_CHECK_STATE_REQUESTS) {
                /* Cellsius: Go to standby without request */
                if (true) {
                    bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_STANDBY;
                    bms_state.substate  = BMS_ENTRY;
                    break;
                } else {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_CHECK_ERROR_FLAGS;
                }
                break;
            }
            break;

        /****************************OPEN CONTACTORS**************************/
        case BMS_STATEMACH_OPEN_CONTACTORS:
            BMS_SAVE_LAST_STATES();

            if (bms_state.substate == BMS_ENTRY) {
                BAL_SetStateRequest(BAL_STATE_NO_BALANCING_REQUEST);
                /* Check if the error reason is the loss of supply voltage clamp 30C */
                if (DIAG_GetDiagnosisEntryState(DIAG_ID_SUPPLY_VOLTAGE_CLAMP_30C_LOST) == STD_NOT_OK) {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_HANDLE_SUPPLY_VOLTAGE_30C_LOSS;
                } else {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_OPEN_ALL_PRECHARGE_CONTACTORS;
                }
                break;
            } else if (bms_state.substate == BMS_OPEN_ALL_PRECHARGE_CONTACTORS) {
                /* Precharge contactors can always be opened as the precharge
                 * resistor limits the maximum current */
                CONT_OpenAllPrechargeContactors();

                /* Regular string opening - Open one string after another,
                      * starting with highest string index */
                stringNumber       = BS_NR_OF_STRINGS - 1u;
                bms_state.substate = BMS_OPEN_MAIN_CONTACTOR;
                bms_state.timer    = BMS_TIME_WAIT_AFTER_OPENING_PRECHARGE;
                break;
            } else if (bms_state.substate == BMS_OPEN_MAIN_CONTACTOR) {
                bms_state.contactorToBeOpened = CONT_MAIN;
                /* Cellsius: Open main contactor */
                CONT_OpenContactor(bms_state.stringToBeOpened, bms_state.contactorToBeOpened);
                bms_state.timer    = BMS_WAIT_TIME_AFTER_OPENING_STRING_CONTACTOR;
                bms_state.substate = BMS_CHECK_MAIN_CONTACTOR;
            } else if (bms_state.substate == BMS_CHECK_MAIN_CONTACTOR) {
                /* Cellsius: Check if main contactor has been opened correctly */
                contactorState = CONT_GetContactorState(bms_state.stringToBeOpened, bms_state.contactorToBeOpened);
                contactorFeedbackValid =
                    BMS_IsContactorFeedbackValid(bms_state.stringToBeOpened, bms_state.contactorToBeOpened);
                /* If we want to open the contactors because of a feedback
                 * error for this contactor, the statement will never be true.
                 * Thus, also continue if a feedback error for this contactor
                 * is detected as we are not able to get a valid feedback
                 * information at this point */
                if ((contactorState == CONT_SWITCH_OFF) || (contactorFeedbackValid == false)) {
                    /* Main contactor opened correctly. Reset state variables used for opening */
                    bms_state.contactorToBeOpened = CONT_UNDEFINED;
                    /* All contactors opened -> prepare to leave state BMS_STATEMACH_OPEN_CONTACTORS */
                    bms_state.substate = BMS_OPEN_STRINGS_EXIT;
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                } else {
                    /* String not opened, re-issue closing request */
                    CONT_OpenContactor(bms_state.stringToBeOpened, bms_state.contactorToBeOpened);
                    bms_state.timer = BMS_STATEMACH_SHORTTIME;
                }
            } else if (bms_state.substate == BMS_HANDLE_SUPPLY_VOLTAGE_30C_LOSS) {
                CONT_OpenAllContactors();
                SPS_SwitchOffAllGeneralIoChannels();
                bms_state.substate = BMS_OPEN_STRINGS_EXIT;
                bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                break;
            } else if (bms_state.substate == BMS_OPEN_STRINGS_EXIT) {
                if (bms_state.nextState == BMS_STATEMACH_STANDBY) {
                    /* Opening due to STANDBY request -> switch to BMS_STATEMACH_STANDBY */
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.state    = BMS_STATEMACH_STANDBY;
                    bms_state.substate = BMS_ENTRY;
                    break;
                } else {
                    /* Opening due to detected error -> switch to BMS_STATEMACH_ERROR */
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.state    = BMS_STATEMACH_ERROR;
                    bms_state.substate = BMS_ENTRY;
                }
            } else {
                FAS_ASSERT(FAS_TRAP);
            }
            break;

        /****************************STANDBY**********************************/
        case BMS_STATEMACH_STANDBY:
            BMS_SAVE_LAST_STATES();
            if (bms_state.substate == BMS_ENTRY) {
                BAL_SetStateRequest(BAL_STATE_ALLOW_BALANCING_REQUEST);
#if BS_STANDBY_PERIODIC_OPEN_WIRE_CHECK == TRUE
                nextOpenWireCheck = timestamp + BS_STANDBY_OPEN_WIRE_PERIOD_ms;
#endif /* BS_STANDBY_PERIODIC_OPEN_WIRE_CHECK == TRUE */
                bms_state.timer    = BMS_STATEMACH_MEDIUM_TIME;
                bms_state.substate = BMS_CHECK_ERROR_FLAGS_INTERLOCK;
                DATA_READ_DATA(&systemState);
                systemState.bmsCanState = BMS_CAN_STATE_STANDBY;
                DATA_WRITE_DATA(&systemState);
                break;
            } else if (bms_state.substate == BMS_CHECK_ERROR_FLAGS_INTERLOCK) {
                if (BMS_IsBatterySystemStateOkay() == STD_NOT_OK) {
                    bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_ERROR;
                    bms_state.substate  = BMS_ENTRY;
                    break;
                } else {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_INTERLOCK_CHECKED;
                    break;
                }
            } else if (bms_state.substate == BMS_INTERLOCK_CHECKED) {
                bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                bms_state.substate = BMS_CHECK_ERROR_FLAGS;
                break;
            } else if (bms_state.substate == BMS_CHECK_ERROR_FLAGS) {
                if (BMS_IsBatterySystemStateOkay() == STD_NOT_OK) {
                    bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_ERROR;
                    bms_state.substate  = BMS_ENTRY;
                    break;
                } else {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_CHECK_STATE_REQUESTS;
                    break;
                }
            } else if (bms_state.substate == BMS_CHECK_STATE_REQUESTS) {
                if (BMS_CheckCanRequests() == BMS_REQ_ID_NORMAL) {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.state    = BMS_STATEMACH_PRECHARGE;
                    bms_state.substate = BMS_ENTRY;
                    break;
                }
                if (BMS_CheckCanRequests() == BMS_REQ_ID_CHARGE) {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.state    = BMS_STATEMACH_CHARGE;
                    bms_state.substate = BMS_ENTRY;
                    break;
                } else {
#if BS_STANDBY_PERIODIC_OPEN_WIRE_CHECK == TRUE
                    if (nextOpenWireCheck <= timestamp) {
                        MEAS_RequestOpenWireCheck();
                        nextOpenWireCheck = timestamp + BS_STANDBY_OPEN_WIRE_PERIOD_ms;
                    }
#endif /* BS_STANDBY_PERIODIC_OPEN_WIRE_CHECK == TRUE */
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_CHECK_ERROR_FLAGS;
                    break;
                }
            } else {
                FAS_ASSERT(FAS_TRAP);
            }
            break;

        /****************************PRECHARGE********************************/
        case BMS_STATEMACH_PRECHARGE:
            BMS_SAVE_LAST_STATES();

            if (bms_state.substate == BMS_ENTRY) {
                BAL_SetStateRequest(BAL_STATE_NO_BALANCING_REQUEST);
                DATA_READ_DATA(&systemState);
                systemState.bmsCanState = BMS_CAN_STATE_PRECHARGE;
                DATA_WRITE_DATA(&systemState);
                /* Cellsius: On this branch, just precharge and main contactor are used */
                bms_state.prechargeTryCounter = 0u;
                if (bms_state.OscillationTimeout == 0u) {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_PRECHARGE_CLOSE_PRECHARGE;
                } else if (BMS_IsBatterySystemStateOkay() == STD_NOT_OK) {
                    /* If precharge re-enter timeout not elapsed, wait (and check errors while waiting) */
                    bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_ERROR;
                    bms_state.substate  = BMS_ENTRY;
                    break;
                }
                break;
            } else if (bms_state.substate == BMS_PRECHARGE_CLOSE_PRECHARGE) {
                bms_state.OscillationTimeout                      = BMS_OSCILLATION_TIMEOUT;
                contRetVal                                        = CONT_ClosePrecharge(bms_state.firstClosedString);
                bms_state.closedPrechargeContactors[stringNumber] = 1u;
                if (contRetVal == STD_OK) {
                    bms_state.timer    = BMS_TIME_WAIT_AFTER_CLOSING_PRECHARGE;
                    bms_state.substate = BMS_CHECK_ERROR_FLAGS_CLOSING_PRECHARGE;
                } else {
                    bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_ERROR;
                    bms_state.substate  = BMS_ENTRY;
                }
                break;
            } else if (bms_state.substate == BMS_CHECK_ERROR_FLAGS_CLOSING_PRECHARGE) {
                if (BMS_IsBatterySystemStateOkay() == STD_NOT_OK) {
                    bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_ERROR;
                    bms_state.substate  = BMS_ENTRY;
                    break;
                } else {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_CHECK_STATE_REQUESTS;
                    break;
                }
            } else if (bms_state.substate == BMS_CHECK_STATE_REQUESTS) {
                if (BMS_CheckCanRequests() == BMS_REQ_ID_STANDBY) {
                    bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_STANDBY;
                    bms_state.substate  = BMS_ENTRY;
                    break;
                } else {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_PRECHARGE_CHECK_VOLTAGES;
                }
            } else if (bms_state.substate == BMS_PRECHARGE_CHECK_VOLTAGES) {
                contactorState = CONT_GetContactorState(bms_state.firstClosedString, CONT_PRECHARGE);
                retVal         = BMS_CheckPrecharge(bms_state.firstClosedString, &bms_tablePackValues);
                /* Check if precharge contactor is closed and precharge is finished */
                if ((contactorState == CONT_SWITCH_ON) && (retVal == STD_OK)) {
                    /* Cellsius: Successfully precharged. Close MAIN contactor */
                    CONT_CloseContactor(bms_state.firstClosedString, CONT_MAIN);
                    bms_state.stringCloseTimeout = BMS_STRING_CLOSE_TIMEOUT;
                    bms_state.timer              = BMS_WAIT_TIME_AFTER_CLOSING_STRING_CONTACTOR;
                    bms_state.substate           = BMS_CHECK_CLOSE_MAIN_CONTACTOR_PRECHARGE_STATE;
                    break;
                } else {
                    /* Precharging failed. Open precharge contactor. */
                    contRetVal = CONT_OpenPrecharge(bms_state.firstClosedString);
                    /* Check if retry limit has been reached */
                    if (bms_state.prechargeTryCounter < (BMS_PRECHARGE_TRIES - 1u)) {
                        bms_state.closedPrechargeContactors[stringNumber] = 0u;
                        if (contRetVal == STD_OK) {
                            bms_state.timer    = BMS_TIME_WAIT_AFTER_PRECHARGE_FAIL;
                            bms_state.substate = BMS_PRECHARGE_CLOSE_PRECHARGE;
                            bms_state.prechargeTryCounter++;
                        } else {
                            bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                            bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                            bms_state.nextState = BMS_STATEMACH_ERROR;
                            bms_state.substate  = BMS_ENTRY;
                        }
                        break;
                    } else {
                        bms_state.closedPrechargeContactors[stringNumber] = 0u;
                        bms_state.timer                                   = BMS_STATEMACH_SHORTTIME;
                        bms_state.state                                   = BMS_STATEMACH_OPEN_CONTACTORS;
                        bms_state.nextState                               = BMS_STATEMACH_ERROR;
                        bms_state.substate                                = BMS_ENTRY;
                        break;
                    }
                }
            } else if (bms_state.substate == BMS_CHECK_CLOSE_MAIN_CONTACTOR_PRECHARGE_STATE) {
                contactorState = CONT_GetContactorState(bms_state.firstClosedString, CONT_MAIN);
                if (contactorState == CONT_SWITCH_ON) {
                    bms_state.closedStrings[bms_state.firstClosedString] = 1u;
                    bms_state.numberOfClosedStrings++;
                    bms_state.timer    = BMS_WAIT_TIME_AFTER_CLOSING_STRING_CONTACTOR;
                    bms_state.substate = BMS_CHECK_ERROR_FLAGS_PRECHARGE_CLOSING_STRINGS;
                    break;
                } else if (bms_state.stringCloseTimeout == 0u) {
                    /* String takes too long to close */
                    bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_ERROR;
                    bms_state.substate  = BMS_ENTRY;
                    break;
                } else {
                    /* String not closed, re-issue closing request */
                    CONT_CloseContactor(bms_state.firstClosedString, CONT_MAIN);
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_CHECK_ERROR_FLAGS_PRECHARGE_FIRST_STRING;
                    break;
                }
            } else if (bms_state.substate == BMS_CHECK_ERROR_FLAGS_PRECHARGE_FIRST_STRING) {
                if (BMS_IsBatterySystemStateOkay() == STD_NOT_OK) {
                    bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_ERROR;
                    bms_state.substate  = BMS_ENTRY;
                    break;
                } else {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_CHECK_CLOSE_MAIN_CONTACTOR_PRECHARGE_STATE;
                    break;
                }
            } else if (bms_state.substate == BMS_CHECK_ERROR_FLAGS_PRECHARGE_CLOSING_STRINGS) {
                /* Always make one error check after the first string was closed successfully */
                if (BMS_IsBatterySystemStateOkay() == STD_NOT_OK) {
                    bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_ERROR;
                    bms_state.substate  = BMS_ENTRY;
                    break;
                } else {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_PRECHARGE_OPEN_PRECHARGE;
                    break;
                }
            } else if (bms_state.substate == BMS_PRECHARGE_OPEN_PRECHARGE) {
                contRetVal = CONT_OpenPrecharge(bms_state.firstClosedString);
                if (contRetVal == STD_OK) {
                    bms_state.closedPrechargeContactors[stringNumber] = 0u;
                    bms_state.timer                                   = BMS_TIME_WAIT_AFTER_OPENING_PRECHARGE;
                    bms_state.substate                                = BMS_PRECHARGE_CHECK_OPEN_PRECHARGE;
                } else {
                    bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_ERROR;
                    bms_state.substate  = BMS_ENTRY;
                }
                break;
            } else if (bms_state.substate == BMS_PRECHARGE_CHECK_OPEN_PRECHARGE) {
                contactorState = CONT_GetContactorState(bms_state.firstClosedString, CONT_PRECHARGE);
                if (contactorState == CONT_SWITCH_OFF) {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.state    = BMS_STATEMACH_NORMAL;
                    bms_state.substate = BMS_ENTRY;
                    break;
                } else if (bms_state.stringCloseTimeout == 0u) {
                    /* Precharge contactor takes too long to open */
                    bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_ERROR;
                    bms_state.substate  = BMS_ENTRY;
                    break;
                } else {
                    /* Precharge contactor not opened, re-issue open request */
                    CONT_OpenPrecharge(bms_state.firstClosedString);
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_CHECK_ERROR_FLAGS_PRECHARGE_FIRST_STRING;
                    break;
                }
            } else {
                FAS_ASSERT(FAS_TRAP);
            }
            break;

        /****************************NORMAL**************************************/
        case BMS_STATEMACH_NORMAL:
            BMS_SAVE_LAST_STATES();

            if (bms_state.substate == BMS_ENTRY) {
#if BS_NORMAL_PERIODIC_OPEN_WIRE_CHECK == TRUE
                nextOpenWireCheck = timestamp + BS_NORMAL_OPEN_WIRE_PERIOD_ms;
#endif /* BS_NORMAL_PERIODIC_OPEN_WIRE_CHECK == TRUE */
                DATA_READ_DATA(&systemState);
                systemState.bmsCanState = BMS_CAN_STATE_NORMAL;
                DATA_WRITE_DATA(&systemState);
                bms_state.timer                 = BMS_STATEMACH_SHORTTIME;
                bms_state.substate              = BMS_CHECK_ERROR_FLAGS;
                bms_state.nextStringClosedTimer = 0u;
                break;
            } else if (bms_state.substate == BMS_CHECK_ERROR_FLAGS) {
                if (BMS_IsBatterySystemStateOkay() == STD_NOT_OK) {
                    bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_ERROR;
                    bms_state.substate  = BMS_ENTRY;
                    break;
                } else {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_CHECK_STATE_REQUESTS;
                    break;
                }
            } else if (bms_state.substate == BMS_CHECK_STATE_REQUESTS) {
                if (BMS_CheckCanRequests() == BMS_REQ_ID_STANDBY) {
                    bms_state.timer     = BMS_STATEMACH_SHORTTIME;
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_STANDBY;
                    bms_state.substate  = BMS_ENTRY;
                    break;
                } else {
#if BS_NORMAL_PERIODIC_OPEN_WIRE_CHECK == TRUE
                    if (nextOpenWireCheck <= timestamp) {
                        MEAS_RequestOpenWireCheck();
                        nextOpenWireCheck = timestamp + BS_NORMAL_OPEN_WIRE_PERIOD_ms;
                    }
#endif /* BS_NORMAL_PERIODIC_OPEN_WIRE_CHECK == TRUE */
                    bms_state.timer = BMS_STATEMACH_SHORTTIME;
                    /* bms_state.substate = BMS_NORMAL_CLOSE_NEXT_STRING; */
                    /* Cellsius: Only one string, directly go to Check Error */
                    bms_state.substate = BMS_CHECK_ERROR_FLAGS;
                    break;
                }
            } else {
                FAS_ASSERT(FAS_TRAP);
            }
            break;

        /****************************ERROR*************************************/
        case BMS_STATEMACH_ERROR:
            BMS_SAVE_LAST_STATES();

            if (bms_state.substate == BMS_ENTRY) {
                /* Set BMS System state to error */
                DATA_READ_DATA(&systemState);
                systemState.bmsCanState = BMS_CAN_STATE_ERROR;
                DATA_WRITE_DATA(&systemState);
                /* Deactivate balancing */
                BAL_SetStateRequest(BAL_STATE_NO_BALANCING_REQUEST);
                /* Change LED toggle frequency to indicate an error */
                LED_SetToggleTime(LED_ERROR_OPERATION_ON_OFF_TIME_ms);
                /* Set timer for next open wire check */
                nextOpenWireCheck = timestamp + AFE_ERROR_OPEN_WIRE_PERIOD_ms;
                /* Switch to next substate */
                bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                bms_state.substate = BMS_CHECK_ERROR_FLAGS;
                break;
            } else if (bms_state.substate == BMS_CHECK_ERROR_FLAGS) {
                if (DIAG_IsAnyFatalErrorSet() == true) {
                    /* we stay already in requested state */
                    if (nextOpenWireCheck <= timestamp) {
                        /* Perform open-wire check periodically */
                        /* MEAS_RequestOpenWireCheck(); */ /*TODO: check with strings */
                        nextOpenWireCheck = timestamp + AFE_ERROR_OPEN_WIRE_PERIOD_ms;
                    }
                } else {
                    /* No error detected anymore - reset fatal error related variables */
                    bms_state.minimumActiveDelay_ms  = BMS_NO_ACTIVE_DELAY_TIME_ms;
                    bms_state.minimumActiveDelay_ms  = BMS_NO_ACTIVE_DELAY_TIME_ms;
                    bms_state.transitionToErrorState = false;
                    /* Check for STANDBY request */
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_CHECK_STATE_REQUESTS;
                    break;
                }
            } else if (bms_state.substate == BMS_CHECK_STATE_REQUESTS) {
                /* Cellsius: Go to standby without request */
                if (true) {
                    /* Activate balancing again */
                    BAL_SetStateRequest(BAL_STATE_ALLOW_BALANCING_REQUEST);
                    /* Set LED frequency to normal operation as we leave error
                       state subsequently */
                    LED_SetToggleTime(LED_NORMAL_OPERATION_ON_OFF_TIME_ms);

                    /* Verify that all contactors are opened and switch to
                     * STANDBY state afterwards */
                    bms_state.state     = BMS_STATEMACH_OPEN_CONTACTORS;
                    bms_state.nextState = BMS_STATEMACH_STANDBY;
                    bms_state.substate  = BMS_ENTRY;
                    break;
                } else {
                    bms_state.timer    = BMS_STATEMACH_SHORTTIME;
                    bms_state.substate = BMS_CHECK_ERROR_FLAGS;
                    break;
                }
            } else {
                /* invalid state -> this should never be reached */
                FAS_ASSERT(FAS_TRAP);
            }
            break;
        default:
            /* invalid state */
            FAS_ASSERT(FAS_TRAP);
            break;

        /****************************CHARGE********************************/
        case BMS_STATEMACH_CHARGE:
            BMS_SAVE_LAST_STATES();

            /* Leon: to implement*/

            if (bms_state.substate == BMS_ENTRY) {
                BAL_SetStateRequest(BAL_STATE_NO_BALANCING_REQUEST);
                DATA_READ_DATA(&systemState);
                systemState.bmsCanState = BMS_CAN_STATE_CHARGE;
                DATA_WRITE_DATA(&systemState);
                break;
            }

            /* TODO: Different precharge sequence. Implemented in this state */
            break;
    } /* end switch (bms_state.state) */

    /* Send an asynchronous bms state message if the state or substate changed*/
    if ((bms_state.state != bms_state.lastState) || (bms_state.substate != bms_state.lastSubstate)) {
        CANTX_TransmitBmsState();
    }

    bms_state.triggerentry--;
    bms_state.counter++;
}

extern BMS_CURRENT_FLOW_STATE_e BMS_GetBatterySystemState(void) {
    return bms_state.currentFlowState;
}

extern BMS_CURRENT_FLOW_STATE_e BMS_GetCurrentFlowDirection(int32_t current_mA) {
    /* AXIVION Routine Generic-MissingParameterAssert: current_mA: parameter accepts whole range */
    BMS_CURRENT_FLOW_STATE_e retVal = BMS_DISCHARGING;

    if (BS_POSITIVE_DISCHARGE_CURRENT == true) {
        if (current_mA >= BS_REST_CURRENT_mA) {
            retVal = BMS_DISCHARGING;
        } else if (current_mA <= -BS_REST_CURRENT_mA) {
            retVal = BMS_CHARGING;
        } else {
            retVal = BMS_AT_REST;
        }
    } else {
        if (current_mA <= -BS_REST_CURRENT_mA) {
            retVal = BMS_DISCHARGING;
        } else if (current_mA >= BS_REST_CURRENT_mA) {
            retVal = BMS_CHARGING;
        } else {
            retVal = BMS_AT_REST;
        }
    }
    return retVal;
}

extern bool BMS_IsStringClosed(uint8_t stringNumber) {
    FAS_ASSERT(stringNumber < BS_NR_OF_STRINGS);
    bool retval = false;
    if (bms_state.closedStrings[stringNumber] == 1u) {
        retval = true;
    }
    return retval;
}

extern bool BMS_IsStringPrecharging(uint8_t stringNumber) {
    FAS_ASSERT(stringNumber < BS_NR_OF_STRINGS);
    bool retval = false;
    if (bms_state.closedPrechargeContactors[stringNumber] == 1u) {
        retval = true;
    }
    return retval;
}

extern uint8_t BMS_GetNumberOfConnectedStrings(void) {
    return bms_state.numberOfClosedStrings;
}

extern bool BMS_IsTransitionToErrorStateActive(void) {
    return bms_state.transitionToErrorState;
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
extern BMS_RETURN_TYPE_e TEST_BMS_CheckStateRequest(BMS_STATE_REQUEST_e statereq) {
    return BMS_CheckStateRequest(statereq);
}
extern BMS_STATE_REQUEST_e TEST_BMS_TransferStateRequest(void) {
    return BMS_TransferStateRequest();
}
extern uint8_t TEST_BMS_CheckReEntrance(void) {
    return BMS_CheckReEntrance();
}
extern uint8_t TEST_BMS_CheckCanRequests(void) {
    return BMS_CheckCanRequests();
}
extern bool TEST_BMS_IsAnyFatalErrorFlagSet(void) {
    return BMS_IsAnyFatalErrorFlagSet();
}
extern STD_RETURN_TYPE_e TEST_BMS_IsBatterySystemStateOkay(void) {
    return BMS_IsBatterySystemStateOkay();
}
extern bool TEST_BMS_IsContactorFeedbackValid(uint8_t stringNumber, CONT_TYPE_e contactorType) {
    return BMS_IsContactorFeedbackValid(stringNumber, contactorType);
}
extern void TEST_BMS_GetMeasurementValues(void) {
    BMS_GetMeasurementValues();
}
extern void TEST_BMS_CheckOpenSenseWire(void) {
    BMS_CheckOpenSenseWire();
}
extern STD_RETURN_TYPE_e TEST_BMS_CheckPrecharge(uint8_t stringNumber, DATA_BLOCK_PACK_VALUES_s *pPackValues) {
    return BMS_CheckPrecharge(stringNumber, pPackValues);
}
extern uint8_t TEST_BMS_GetHighestString(BMS_CONSIDER_PRECHARGE_e precharge, DATA_BLOCK_PACK_VALUES_s *pPackValues) {
    return BMS_GetHighestString(precharge, pPackValues);
}
extern uint8_t TEST_BMS_GetClosestString(BMS_CONSIDER_PRECHARGE_e precharge, DATA_BLOCK_PACK_VALUES_s *pPackValues) {
    return BMS_GetClosestString(precharge, pPackValues);
}

extern uint8_t TEST_BMS_GetLowestString(BMS_CONSIDER_PRECHARGE_e precharge, DATA_BLOCK_PACK_VALUES_s *pPackValues) {
    return BMS_GetLowestString(precharge, pPackValues);
}
extern int32_t TEST_BMS_GetStringVoltageDifference(uint8_t string, DATA_BLOCK_PACK_VALUES_s *pPackValues) {
    return BMS_GetStringVoltageDifference(string, pPackValues);
}
extern int32_t TEST_BMS_GetAverageStringCurrent(DATA_BLOCK_PACK_VALUES_s *pPackValues) {
    return BMS_GetAverageStringCurrent(pPackValues);
}
extern void TEST_BMS_UpdateBatterySystemState(DATA_BLOCK_PACK_VALUES_s *pPackValues) {
    BMS_UpdateBatterySystemState(pPackValues);
}

#endif
