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
 * @file    battery_cell_cfg.c
 * @author  foxBMS Team
 * @date    2020-10-08 (date of creation)
 * @updated 2025-03-31 (date of last update)
 * @version v1.9.0
 * @ingroup BATTERY_CELL_CONFIGURATION
 * @prefix  BC
 *
 * @brief   Configuration of the battery cell
 * @details This files contains battery cell lookup tables
 *
 */

/*========== Includes =======================================================*/
#include "battery_cell_cfg.h"

#include <stdint.h>

/*========== Macros and Definitions =========================================*/

/*========== Static Constant and Variable Definitions =======================*/

/*========== Extern Constant and Variable Definitions =======================*/

/* SOC Lookup table in 1% steps starting with 100%. Created from Molicel P50B datasheet. */
const BC_LUT_s bc_stateOfChargeLookupTable[] = {
    {4169, 100.0f}, {4132, 99.0f}, {4110, 98.0f}, {4095, 97.0f}, {4084, 96.0f}, {4076, 95.0f}, {4069, 94.0f},
    {4064, 93.0f},  {4059, 92.0f}, {4054, 91.0f}, {4050, 90.0f}, {4046, 89.0f}, {4041, 88.0f}, {4034, 87.0f},
    {4026, 86.0f},  {4016, 85.0f}, {4005, 84.0f}, {3994, 83.0f}, {3982, 82.0f}, {3970, 81.0f}, {3957, 80.0f},
    {3944, 79.0f},  {3932, 78.0f}, {3919, 77.0f}, {3906, 76.0f}, {3896, 75.0f}, {3885, 74.0f}, {3875, 73.0f},
    {3865, 72.0f},  {3856, 71.0f}, {3847, 70.0f}, {3839, 69.0f}, {3832, 68.0f}, {3824, 67.0f}, {3816, 66.0f},
    {3807, 65.0f},  {3797, 64.0f}, {3787, 63.0f}, {3778, 62.0f}, {3770, 61.0f}, {3761, 60.0f}, {3751, 59.0f},
    {3740, 58.0f},  {3730, 57.0f}, {3720, 56.0f}, {3710, 55.0f}, {3700, 54.0f}, {3687, 53.0f}, {3675, 52.0f},
    {3662, 51.0f},  {3650, 50.0f}, {3638, 49.0f}, {3625, 48.0f}, {3612, 47.0f}, {3597, 46.0f}, {3581, 45.0f},
    {3568, 44.0f},  {3555, 43.0f}, {3542, 42.0f}, {3532, 41.0f}, {3522, 40.0f}, {3513, 39.0f}, {3503, 38.0f},
    {3492, 37.0f},  {3473, 36.0f}, {3457, 35.0f}, {3443, 34.0f}, {3431, 33.0f}, {3416, 32.0f}, {3400, 31.0f},
    {3384, 30.0f},  {3368, 29.0f}, {3352, 28.0f}, {3335, 27.0f}, {3316, 26.0f}, {3297, 25.0f}, {3277, 24.0f},
    {3256, 23.0f},  {3235, 22.0f}, {3214, 21.0f}, {3194, 20.0f}, {3173, 19.0f}, {3149, 18.0f}, {3124, 17.0f},
    {3102, 16.0f},  {3077, 15.0f}, {3048, 14.0f}, {3017, 13.0f}, {2985, 12.0f}, {2953, 11.0f}, {2920, 10.0f},
    {2888, 9.0f},   {2854, 8.0f},  {2819, 7.0f},  {2782, 6.0f},  {2741, 5.0f},  {2696, 4.0f},  {2635, 3.0f},
    {2557, 2.0f},   {2447, 1.0f},  {2325, 0.0f},
};

/* SOE Lookup table in 1% steps starting with 100%. Created from Molicel P50B datasheet. */
const BC_LUT_s bc_stateOfEnergyLookupTable[] = {
    {4169, 100.0f}, {4137, 99.0f}, {4115, 98.0f}, {4101, 97.0f}, {4091, 96.0f}, {4082, 95.0f}, {4075, 94.0f},
    {4069, 93.0f},  {4065, 92.0f}, {4060, 91.0f}, {4056, 90.0f}, {4052, 89.0f}, {4048, 88.0f}, {4045, 87.0f},
    {4041, 86.0f},  {4034, 85.0f}, {4027, 84.0f}, {4018, 83.0f}, {4010, 82.0f}, {4000, 81.0f}, {3990, 80.0f},
    {3979, 79.0f},  {3968, 78.0f}, {3956, 77.0f}, {3945, 76.0f}, {3934, 75.0f}, {3922, 74.0f}, {3911, 73.0f},
    {3901, 72.0f},  {3891, 71.0f}, {3882, 70.0f}, {3872, 69.0f}, {3864, 68.0f}, {3855, 67.0f}, {3847, 66.0f},
    {3840, 65.0f},  {3833, 64.0f}, {3826, 63.0f}, {3819, 62.0f}, {3811, 61.0f}, {3801, 60.0f}, {3792, 59.0f},
    {3784, 58.0f},  {3776, 57.0f}, {3768, 56.0f}, {3759, 55.0f}, {3750, 54.0f}, {3740, 53.0f}, {3730, 52.0f},
    {3720, 51.0f},  {3711, 50.0f}, {3701, 49.0f}, {3689, 48.0f}, {3677, 47.0f}, {3665, 46.0f}, {3654, 45.0f},
    {3642, 44.0f},  {3630, 43.0f}, {3617, 42.0f}, {3603, 41.0f}, {3588, 40.0f}, {3574, 39.0f}, {3561, 38.0f},
    {3548, 37.0f},  {3536, 36.0f}, {3527, 35.0f}, {3517, 34.0f}, {3507, 33.0f}, {3497, 32.0f}, {3482, 31.0f},
    {3464, 30.0f},  {3449, 29.0f}, {3435, 28.0f}, {3422, 27.0f}, {3405, 26.0f}, {3389, 25.0f}, {3372, 24.0f},
    {3355, 23.0f},  {3337, 22.0f}, {3318, 21.0f}, {3297, 20.0f}, {3276, 19.0f}, {3253, 18.0f}, {3230, 17.0f},
    {3208, 16.0f},  {3185, 15.0f}, {3161, 14.0f}, {3132, 13.0f}, {3106, 12.0f}, {3079, 11.0f}, {3045, 10.0f},
    {3009, 9.0f},   {2971, 8.0f},  {2932, 7.0f},  {2893, 6.0f},  {2852, 5.0f},  {2808, 4.0f},  {2759, 3.0f},
    {2703, 2.0f},   {2622, 1.0f},  {2501, 0.0f},
};

uint16_t bc_stateOfChargeLookupTableLength = sizeof(bc_stateOfChargeLookupTable) / sizeof(BC_LUT_s);
uint16_t bc_stateOfEnergyLookupTableLength = sizeof(bc_stateOfEnergyLookupTable) / sizeof(BC_LUT_s);

/*========== Static Function Prototypes =====================================*/

/*========== Static Function Implementations ================================*/

/*========== Extern Function Implementations ================================*/

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
#endif
