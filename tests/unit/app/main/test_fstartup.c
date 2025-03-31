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
 * @file    test_fstartup.c
 * @author  foxBMS Team
 * @date    2020-04-01 (date of creation)
 * @updated 2025-03-31 (date of last update)
 * @version v1.9.0
 * @ingroup UNIT_TEST_IMPLEMENTATION
 * @prefix  TEST
 *
 * @brief   Tests for the MCU startup routine of the TMS570Lx43xx
 * @details TODO
 *
 */

/*========== Includes =======================================================*/
#include "unity.h"
#include "mock_esm.h"

#include "HL_hal_stdtypes.h"

/* clang-format off */
#include "MockHL_sys_common.h"
#include "MockHL_hal_stdtypes.h"
#include "MockHL_system.h"
#include "MockHL_sys_vim.h"
#include "MockHL_sys_core.h"
#include "MockHL_reg_esm.h"
#include "MockHL_esm.h"
#include "MockHL_sys_mpu.h"
#include "MockHL_errata_SSWF021_45.h"
/* clang-format on */

#include "fstartup.h"
#include "fstd_types.h"
#include "test_assert_helper.h"

/*========== Unit Testing Framework Directives ==============================*/

TEST_INCLUDE_PATH("../../tests/unit/app/main/helper")
TEST_INCLUDE_PATH("C:/ti/Hercules/HALCoGen/v04.07.01/drivers/TMS570LC4357ZWT/SYSTEM570v000")

/*========== Definitions and Implementations for Unit Test ==================*/

esmBASE_t dummy   = {.SR1 = {0u, 0u, 0u}};
esmBASE_t *esmREG = &dummy;
extern void __TI_auto_init(void) {
}

/*========== Setup and Teardown =============================================*/
void setUp(void) {
}

void tearDown(void) {
}

/*========== Test Cases =====================================================*/

void test_c_int00(void) {
}
