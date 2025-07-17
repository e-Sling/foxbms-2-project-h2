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
 * @file    rnd_r_155mf52a2_103f3470.c
 * @author  foxBMS Team
 * @date    2020-12-14 (date of creation)
 * @updated 2020-02-23 (date of last update)
 * @version v1.9.0
 * @ingroup DRIVERS
 * @prefix  TS
 *
 * @brief   Resistive divider used for measuring temperature
 * @details TODO
 */

/*========== Includes =======================================================*/
#include "rnd_r_155mf52a2_103f3470.h"

#include "foxmath.h"

/*========== Macros and Definitions =========================================*/

/**
 * temperature-resistance LUT for RND 155MF52A2_103F3470 NTC
 */
typedef struct {
    int16_t temperature_ddegC; /*!< temperature in &deg;C */
    float resistance_Ohm;      /*!< resistance in ohms */
} R_155MF52A2_103F3470_LUT_s;

/*========== Static Constant and Variable Definitions =======================*/

/* clang-format off */
/**
 * LUT filled from higher resistance to lower resistance
 */
static const R_155MF52A2_103F3470_LUT_s r_155mf52a2_103f3470_LUT[] = {
        { -500, 384109},
        { -490, 361304},
        { -480, 340231},
        { -470, 320673},
        { -460, 302456},
        { -450, 285438},
        { -440, 269502},
        { -430, 254552},
        { -420, 240506},
        { -410, 227295},
        { -400, 214860},
        { -390, 203146},
        { -380, 192108},
        { -370, 181705},
        { -360, 171897},
        { -350, 162649},
        { -340, 153930},
        { -330, 145710},
        { -320, 137958},
        { -310, 130650},
        { -300, 123760},
        { -290, 117263},
        { -280, 111137},
        { -270, 105362},
        { -260, 99915},
        { -250, 94780},
        { -240, 89936},
        { -230, 85367},
        { -220, 81056},
        { -210, 76989},
        { -200, 73150},
        { -190, 69526},
        { -180, 66105},
        { -170, 62873},
        { -160, 59819},
        { -150, 56934},
        { -140, 54206},
        { -130, 51626},
        { -120, 49185},
        { -110, 46876},
        { -100, 44690},
        {  -90, 42619},
        {  -80, 40658},
        {  -70, 38799},
        {  -60, 37037},
        {  -50, 35365},
        {  -40, 33780},
        {  -30, 32275},
        {  -20, 30847},
        {  -10, 29490},
        {   00, 28110},
        {   10, 26975},
        {   20, 25811},
        {   30, 24703},
        {   40, 23649},
        {   50, 22646},
        {   60, 21691},
        {   70, 20782},
        {   80, 19916},
        {   90, 19091},
        {  100, 18305},
        {  110, 17555},
        {  120, 16840},
        {  130, 16158},
        {  140, 15507},
        {  150, 14886},
        {  160, 14292},
        {  170, 13726},
        {  180, 13185},
        {  190, 12669},
        {  200, 12175},
        {  210, 11703},
        {  220, 11252},
        {  230, 10820},
        {  240, 10408},
        {  250, 10000.00},
        {  260, 9635},
        {  270, 9273},
        {  280, 8927},
        {  290, 8596},
        {  300, 8278},
        {  310, 7974},
        {  320, 7683},
        {  330, 7403},
        {  340, 7136},
        {  350, 6879},
        {  360, 6633},
        {  370, 6397},
        {  380, 6171},
        {  390, 5953},
        {  400, 5745},
        {  410, 5545},
        {  420, 5353},
        {  430, 5168},
        {  440, 4991},
        {  450, 4821},
        {  460, 4657},
        {  470, 4500},
        {  480, 4349},
        {  490, 4203},
        {  500, 4064},
        {  510, 3929},
        {  520, 3800},
        {  530, 3675},
        {  540, 3556},
        {  550, 3440},
        {  560, 3329},
        {  570, 3222},
        {  580, 3120},
        {  590, 3020},
        {  600, 2925},
        {  610, 2833},
        {  620, 2744},
        {  630, 2658},
        {  640, 2576},
        {  650, 2496},
        {  660, 2420},
        {  670, 2346},
        {  680, 2274},
        {  690, 2205},
        {  700, 2139},
        {  710, 2074},
        {  720, 2012},
        {  730, 1953},
        {  740, 1895},
        {  750, 1839},
        {  760, 1785},
        {  770, 1733},
        {  780, 1682},
        {  790, 1634},
        {  800, 1587},
        {  810, 1541},
        {  820, 1497},
        {  830, 1454},
        {  840, 1413},
        {  850, 1374},
        {  860, 1335},
        {  870, 1298},
        {  880, 1262},
        {  890, 1227},
        {  900, 1193},
        {  910, 1161},
        {  920, 1129},
        {  930, 1098},
        {  940, 1069},
        {  950, 1040},
        {  960, 1013},
        {  970, 986},
        {  980, 960},
        {  990, 935},
        { 1000, 911},
};
/* clang-format on */

/** size of #r_155mf52a2_103f3470_LUT */
static const uint16_t sizeLUT = sizeof(r_155mf52a2_103f3470_LUT) / sizeof(R_155MF52A2_103F3470_LUT_s);

/*========== Extern Constant and Variable Definitions =======================*/
/**
 * Defines for calculating the ADC voltage on the ends of the operating range. (voltage devider)
 * The ADC voltage is calculated with the following formula:
 *
 * Vadc = ((Vsupply * Rntc) / (R + Rntc))
 *
 * Depending on the position of the NTC in the voltage resistor (R1/R2),
 * different Rntc values are used for the calculation.
 * @{
 */
#if R_155MF52A2_103F3470_POSITION_IN_RESISTOR_DIVIDER_IS_R1 == TRUE
#define R_155MF52A2_103F3470_ADC_VOLTAGE_VMAX_V                       \
    (float)((R_155MF52A2_103F3470_RESISTOR_DIVIDER_SUPPLY_VOLTAGE_V * \
             r_155mf52a2_103f3470_LUT[sizeLUT - 1].resistance_Ohm) /  \
            (r_155mf52a2_103f3470_LUT[sizeLUT - 1].resistance_Ohm +   \
             R_155MF52A2_103F3470_RESISTOR_DIVIDER_RESISTANCE_R1_R2_Ohm))
#define R_155MF52A2_103F3470_ADC_VOLTAGE_VMIN_V                                                                     \
    (float)((R_155MF52A2_103F3470_RESISTOR_DIVIDER_SUPPLY_VOLTAGE_V * r_155mf52a2_103f3470_LUT[0].resistance_Ohm) / \
            (r_155mf52a2_103f3470_LUT[0].resistance_Ohm + R_155MF52A2_103F3470_RESISTOR_DIVIDER_RESISTANCE_R1_R2_Ohm))
#else /*R_155MF52A2_103F3470_POSITION_IN_RESISTOR_DIVIDER_IS_R1 == FALSE */
#define R_155MF52A2_103F3470_ADC_VOLTAGE_VMIN_V                       \
    (float)((R_155MF52A2_103F3470_RESISTOR_DIVIDER_SUPPLY_VOLTAGE_V * \
             r_155mf52a2_103f3470_LUT[sizeLUT - 1].resistance_Ohm) /  \
            (r_155mf52a2_103f3470_LUT[sizeLUT - 1].resistance_Ohm +   \
             R_155MF52A2_103F3470_RESISTOR_DIVIDER_RESISTANCE_R1_R2_Ohm))
#define R_155MF52A2_103F3470_ADC_VOLTAGE_VMAX_V                                                                     \
    (float)((R_155MF52A2_103F3470_RESISTOR_DIVIDER_SUPPLY_VOLTAGE_V * r_155mf52a2_103f3470_LUT[0].resistance_Ohm) / \
            (r_155mf52a2_103f3470_LUT[0].resistance_Ohm + R_155MF52A2_103F3470_RESISTOR_DIVIDER_RESISTANCE_R1_R2_Ohm))
#endif
/**@}*/

/*========== Static Function Prototypes =====================================*/

/*========== Static Function Implementations ================================*/

/*========== Extern Function Implementations ================================*/

extern int16_t R_155MF52A2_103F3470_GetTempFromLUT(uint16_t adcVoltage_mV) {
    int16_t temperature_ddegC = 0.0;
    float resistance_Ohm      = 0.0;
    float adcVoltage_V        = adcVoltage_mV / 1000.0; /* Convert mV to V */

    /* Check for valid ADC measurements to prevent undefined behavior */
    if (adcVoltage_V > R_155MF52A2_103F3470_ADC_VOLTAGE_VMAX_V) {
        /* Invalid measured ADC voltage -> sensor out of operating range or disconnected/shorted */
        temperature_ddegC = INT16_MIN;
    } else if (adcVoltage_V < R_155MF52A2_103F3470_ADC_VOLTAGE_VMIN_V) {
        /* Invalid measured ADC voltage -> sensor out of operating range or shorted/disconnected */
        temperature_ddegC = INT16_MAX;
    } else {
        /* Calculate NTC resistance based on measured ADC voltage */
#if R_155MF52A2_103F3470_POSITION_IN_RESISTOR_DIVIDER_IS_R1 == TRUE

        /* R1 = R2*((Vsupply/Vadc)-1) */
        resistance_Ohm = R_155MF52A2_103F3470_RESISTOR_DIVIDER_RESISTANCE_R1_R2_Ohm *
                         ((R_155MF52A2_103F3470_RESISTOR_DIVIDER_SUPPLY_VOLTAGE_V / adcVoltage_V) - 1);
#else  /* R_155MF52A2_103F3470_POSITION_IN_RESISTOR_DIVIDER_IS_R1 == FALSE */

        /* R2 = R1*(V2/(Vsupply-Vadc)) */
        resistance_Ohm = R_155MF52A2_103F3470_RESISTOR_DIVIDER_RESISTANCE_R1_R2_Ohm *
                         (adcVoltage_V / (R_155MF52A2_103F3470_RESISTOR_DIVIDER_SUPPLY_VOLTAGE_V - adcVoltage_V));
#endif /* R_155MF52A2_103F3470_POSITION_IN_RESISTOR_DIVIDER_IS_R1 */

        /* Variables for interpolating LUT value */
        uint16_t between_high = 0;
        uint16_t between_low  = 0;
        for (uint16_t i = 1; i < sizeLUT; i++) {
            if (resistance_Ohm < r_155mf52a2_103f3470_LUT[i].resistance_Ohm) {
                between_low  = i + 1u;
                between_high = i;
            }
        }

        /* Interpolate between LUT vales, but do not extrapolate LUT! */
        if (!(((between_high == 0u) && (between_low == 0u)) || /* measured resistance > maximum LUT resistance */
              (between_low > sizeLUT))) {                      /* measured resistance < minimum LUT resistance */
            temperature_ddegC = (int16_t)MATH_LinearInterpolation(
                r_155mf52a2_103f3470_LUT[between_low].resistance_Ohm,
                r_155mf52a2_103f3470_LUT[between_low].temperature_ddegC,
                r_155mf52a2_103f3470_LUT[between_high].resistance_Ohm,
                r_155mf52a2_103f3470_LUT[between_high].temperature_ddegC,
                resistance_Ohm);
        }
    }

    /* Return temperature based on measured NTC resistance */
    return temperature_ddegC;
}

/**
 * Interpolation methode of degree 9
 * look @Matlab script /Berechnungen/02_BMS/RND_Temperatursensor_Interpolationspolynom.m
 * the average error = 0.06, the maximum error = 0.33 at 100degC
 *
 * settings:
 *      R1_IsNTC = false;
 *      R_notNTC = 10000;
 *      Vsupply = 3;
 *      n = 9
 *
 * */
extern int16_t R_155MF52A2_103F3470_GetTempFromPolynomial(uint16_t adcVoltage_mV) {
    float temperature_degC = 0.0;
    float vadc_V           = adcVoltage_mV / 1000.0;
    float vadc2            = vadc_V * vadc_V;
    float vadc3            = vadc2 * vadc_V;
    float vadc4            = vadc3 * vadc_V;
    float vadc5            = vadc4 * vadc_V;
    float vadc6            = vadc5 * vadc_V;
    float vadc7            = vadc6 * vadc_V;
    float vadc8            = vadc7 * vadc_V;
    float vadc9            = vadc8 * vadc_V;

    temperature_degC = -(4.1f * vadc9) + (56.3f * vadc8) - (332.7f * vadc7) + (1105.8f * vadc6) - (2274.8f * vadc5) +
                       (3011.2f * vadc4) - (2594.6f * vadc3) + (1452.9f * vadc2) - (0554.8f * vadc_V) + 178.7f;

    return (int16_t)(temperature_degC * 10.0f); /* Convert deg into deci &deg;C */
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
#endif
