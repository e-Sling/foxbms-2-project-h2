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
 * @file    can_helper.c
 * @author  foxBMS Team
 * @date    2021-04-22 (date of creation)
 * @updated 2025-03-31 (date of last update)
 * @version v1.9.0
 * @ingroup DRIVERS
 * @prefix  CAN
 *
 * @brief   Helper functions for the CAN module
 * @details TODO
 *
 *
 */

/*========== Includes =======================================================*/
#include "can_helper.h"

#include "database.h"

#include <math.h>
#include <stdint.h>

/*========== Macros and Definitions =========================================*/
/** Plausibility checking can signals lengths */
#define CAN_SIGNAL_MAX_SIZE (64u)

/** bitmask for extraction of one byte */
#define CAN_MESSAGE_BIT_MASK_ONE_BYTE (0xFFu)

/** length of one CAN byte in bit */
#define CAN_BYTE_LENGTH (8u)

/*========== Static Constant and Variable Definitions =======================*/

/** To convert big endian start bit to usual little endian representation (from 0 as LSB to 63 as MSB) */
static const uint8_t can_bigEndianTable[CAN_SIGNAL_MAX_SIZE] = {
    56u, 57u, 58u, 59u, 60u, 61u, 62u, 63u, 48u, 49u, 50u, 51u, 52u, 53u, 54u, 55u, 40u, 41u, 42u, 43u, 44u, 45u,
    46u, 47u, 32u, 33u, 34u, 35u, 36u, 37u, 38u, 39u, 24u, 25u, 26u, 27u, 28u, 29u, 30u, 31u, 16u, 17u, 18u, 19u,
    20u, 21u, 22u, 23u, 8u,  9u,  10u, 11u, 12u, 13u, 14u, 15u, 0u,  1u,  2u,  3u,  4u,  5u,  6u,  7u};

/* Cellsius: CRC table */
static const uint8_t CRC8H2F_TABLE[256] = {
    0x00u, 0x2fu, 0x5eu, 0x71u, 0xbcu, 0x93u, 0xe2u, 0xcdu, 0x57u, 0x78u, 0x09u, 0x26u, 0xebu, 0xc4u, 0xb5u, 0x9au,
    0xaeu, 0x81u, 0xf0u, 0xdfu, 0x12u, 0x3du, 0x4cu, 0x63u, 0xf9u, 0xd6u, 0xa7u, 0x88u, 0x45u, 0x6au, 0x1bu, 0x34u,
    0x73u, 0x5cu, 0x2du, 0x02u, 0xcfu, 0xe0u, 0x91u, 0xbeu, 0x24u, 0x0bu, 0x7au, 0x55u, 0x98u, 0xb7u, 0xc6u, 0xe9u,
    0xddu, 0xf2u, 0x83u, 0xacu, 0x61u, 0x4eu, 0x3fu, 0x10u, 0x8au, 0xa5u, 0xd4u, 0xfbu, 0x36u, 0x19u, 0x68u, 0x47u,
    0xe6u, 0xc9u, 0xb8u, 0x97u, 0x5au, 0x75u, 0x04u, 0x2bu, 0xb1u, 0x9eu, 0xefu, 0xc0u, 0x0du, 0x22u, 0x53u, 0x7cu,
    0x48u, 0x67u, 0x16u, 0x39u, 0xf4u, 0xdbu, 0xaau, 0x85u, 0x1fu, 0x30u, 0x41u, 0x6eu, 0xa3u, 0x8cu, 0xfdu, 0xd2u,
    0x95u, 0xbau, 0xcbu, 0xe4u, 0x29u, 0x06u, 0x77u, 0x58u, 0xc2u, 0xedu, 0x9cu, 0xb3u, 0x7eu, 0x51u, 0x20u, 0x0fu,
    0x3bu, 0x14u, 0x65u, 0x4au, 0x87u, 0xa8u, 0xd9u, 0xf6u, 0x6cu, 0x43u, 0x32u, 0x1du, 0xd0u, 0xffu, 0x8eu, 0xa1u,
    0xe3u, 0xccu, 0xbdu, 0x92u, 0x5fu, 0x70u, 0x01u, 0x2eu, 0xb4u, 0x9bu, 0xeau, 0xc5u, 0x08u, 0x27u, 0x56u, 0x79u,
    0x4du, 0x62u, 0x13u, 0x3cu, 0xf1u, 0xdeu, 0xafu, 0x80u, 0x1au, 0x35u, 0x44u, 0x6bu, 0xa6u, 0x89u, 0xf8u, 0xd7u,
    0x90u, 0xbfu, 0xceu, 0xe1u, 0x2cu, 0x03u, 0x72u, 0x5du, 0xc7u, 0xe8u, 0x99u, 0xb6u, 0x7bu, 0x54u, 0x25u, 0x0au,
    0x3eu, 0x11u, 0x60u, 0x4fu, 0x82u, 0xadu, 0xdcu, 0xf3u, 0x69u, 0x46u, 0x37u, 0x18u, 0xd5u, 0xfau, 0x8bu, 0xa4u,
    0x05u, 0x2au, 0x5bu, 0x74u, 0xb9u, 0x96u, 0xe7u, 0xc8u, 0x52u, 0x7du, 0x0cu, 0x23u, 0xeeu, 0xc1u, 0xb0u, 0x9fu,
    0xabu, 0x84u, 0xf5u, 0xdau, 0x17u, 0x38u, 0x49u, 0x66u, 0xfcu, 0xd3u, 0xa2u, 0x8du, 0x40u, 0x6fu, 0x1eu, 0x31u,
    0x76u, 0x59u, 0x28u, 0x07u, 0xcau, 0xe5u, 0x94u, 0xbbu, 0x21u, 0x0eu, 0x7fu, 0x50u, 0x9du, 0xb2u, 0xc3u, 0xecu,
    0xd8u, 0xf7u, 0x86u, 0xa9u, 0x64u, 0x4bu, 0x3au, 0x15u, 0x8fu, 0xa0u, 0xd1u, 0xfeu, 0x33u, 0x1cu, 0x6du, 0x42u};

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/

/**
 * @brief   Convert bit start in big endian case.
 *          In the big endian case for CAN, the bit start is set
 *          to the MSB and the bit numbering is not directly usable.
 *          This functions converts the bit start to set it to
 *          the LSB of the signal and with the usual bit numbering.
 * @param   bitStart    bit start in big endian format
 * @param   bitLength   signal length
 * @return  bit start position converted to little endian format
 */
static uint64_t CAN_ConvertBitStartBigEndian(uint64_t bitStart, uint64_t bitLength);

/*========== Static Function Implementations ================================*/

static uint64_t CAN_ConvertBitStartBigEndian(uint64_t bitStart, uint64_t bitLength) {
    /* A valid message has to start before the end of the message */
    FAS_ASSERT(bitStart < CAN_SIGNAL_MAX_SIZE);
    /* The longest message may be CAN_SIGNAL_MAX_SIZE long */
    FAS_ASSERT(bitLength <= CAN_SIGNAL_MAX_SIZE);
    /* A signal must contain at least one bit */
    FAS_ASSERT(bitLength > 0u);

    /**
     * Example: big endian, bitStart = 53, bitLength = 13
     * For big endian, bitStart corresponds to MSB.
     * First convert |07 06 05 04 03 02 01 00| to |63 62 61 60 59 58 57 56|
     *               |15 14 13 12 11 10 09 08|    |55 54 53 52 51 50 49 48|
     *               |23 22 21 20 19 18 17 16|    |47 46 45 44 43 42 41 40|
     *               |31 30 29 28 27 26 25 24|    |39 38 37 36 35 34 33 32|
     *               |39 38 37 36 35 34 33 32|    |31 30 29 28 27 26 25 24|
     *               |47 46 45 44 43 42 41 40|    |23 22 21 20 19 18 17 16|
     *               |55 54 53 52 51 50 49 48|    |15 14 13 12 11 10 09 08|
     *               |63 62 61 60 59 58 57 56|    |07 06 05 04 03 02 01 00|
     * to get MSB position in the usual bit representation (from 0 as LSB to 63 as MSB).
     * In the example, 53 must be converted to 13.
     */
    uint64_t position = can_bigEndianTable[bitStart];

    /**
     * Usual bit position of MSB of signal is now available.
     * Now subtract signal length to get LSB position.
     * In the example, bitStart is converted from 53 to 0 for the length of 13 bits.
     * This corresponds to the usual data access in the little endian case, from bit 0 to bit 12,
     * where bitStart corresponds to LSB.
     */
    position = position - (bitLength - 1u);

    /* a valid message has to start before the end of the message */
    FAS_ASSERT(position < CAN_SIGNAL_MAX_SIZE);
    /* Check for a plausible message length (sum of start bit and length shall
       not be larger than 64, otherwise it will not fit into the message) */
    FAS_ASSERT((position + bitLength) <= CAN_SIGNAL_MAX_SIZE);

    return position;
}

/*========== Extern Function Implementations ================================*/

extern void CAN_TxPrepareSignalData(float_t *pSignal, CAN_SIGNAL_TYPE_s signalProperties) {
    FAS_ASSERT(pSignal != NULL_PTR);

    /* Check min/max limits */
    if (*pSignal > signalProperties.max) {
        *pSignal = signalProperties.max;
    } else if (*pSignal < signalProperties.min) {
        *pSignal = signalProperties.min;
    } else {
        ; /* no action on *pSignal required */
    }

    /* Apply offset */
    *pSignal = *pSignal + signalProperties.offset;

    /* Apply factor */
    *pSignal = *pSignal / signalProperties.factor;
}

extern void CAN_RxConvertRawSignalData(
    float_t *pSignalConverted,
    float_t signalRaw,
    CAN_SIGNAL_TYPE_s signalProperties) {
    FAS_ASSERT(pSignalConverted != NULL_PTR);
    /* Apply offset and factor */
    *pSignalConverted = (signalRaw * signalProperties.factor) - signalProperties.offset;
}

extern void CAN_TxSetMessageDataWithSignalData(
    uint64_t *pMessage,
    uint64_t bitStart,
    uint8_t bitLength,
    uint64_t canSignal,
    CAN_ENDIANNESS_e endianness) {
    /* AXIVION Routine Generic-MissingParameterAssert: canSignal: parameter accepts whole range */
    FAS_ASSERT(pMessage != NULL_PTR);
    FAS_ASSERT((endianness == CAN_BIG_ENDIAN) || (endianness == CAN_LITTLE_ENDIAN));
    /* The longest message may be CAN_SIGNAL_MAX_SIZE long */
    FAS_ASSERT(bitLength <= CAN_SIGNAL_MAX_SIZE);
    /* A signal must contain at least one bit */
    FAS_ASSERT(bitLength > 0u);
    /* Signal start can not be outside of message data */
    FAS_ASSERT(bitStart < CAN_SIGNAL_MAX_SIZE);

    uint64_t position = bitStart;

    if (endianness == CAN_BIG_ENDIAN) {
        position = CAN_ConvertBitStartBigEndian(bitStart, bitLength);
    }

    /* A valid message has to start before the end of the message */
    FAS_ASSERT(position < CAN_SIGNAL_MAX_SIZE);
    /* Check for a plausible message length (sum of start bit and length shall
       not be larger than 64, otherwise it will not fit into the message) */
    FAS_ASSERT((position + bitLength) <= CAN_SIGNAL_MAX_SIZE);

    /* Prepare a mask and assemble message */
    uint64_t mask = UINT64_MAX;

    if (bitLength == CAN_SIGNAL_MAX_SIZE) {
        /* Since a bit-shift of 64 on a 64bit integer is undefined, set to desired 0 */
        mask = 0u;
    } else {
        mask <<= bitLength;
    }
    mask = ~mask;

    *pMessage |= (canSignal & mask) << position;
}

extern void CAN_TxSetCanDataWithMessageData(uint64_t message, uint8_t *pCanData, CAN_ENDIANNESS_e endianness) {
    /* AXIVION Routine Generic-MissingParameterAssert: message: parameter accepts whole range */
    FAS_ASSERT(pCanData != NULL_PTR);

    /* Swap byte order if necessary */
    if (endianness == CAN_BIG_ENDIAN) {
        pCanData[CAN_BYTE_0_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_7_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
        pCanData[CAN_BYTE_1_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_6_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
        pCanData[CAN_BYTE_2_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_5_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
        pCanData[CAN_BYTE_3_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_4_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
        pCanData[CAN_BYTE_4_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_3_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
        pCanData[CAN_BYTE_5_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_2_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
        pCanData[CAN_BYTE_6_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_1_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
        pCanData[CAN_BYTE_7_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_0_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
    } else if (endianness == CAN_LITTLE_ENDIAN) {
        pCanData[CAN_BYTE_0_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_0_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
        pCanData[CAN_BYTE_1_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_1_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
        pCanData[CAN_BYTE_2_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_2_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
        pCanData[CAN_BYTE_3_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_3_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
        pCanData[CAN_BYTE_4_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_4_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
        pCanData[CAN_BYTE_5_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_5_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
        pCanData[CAN_BYTE_6_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_6_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
        pCanData[CAN_BYTE_7_POSITION] =
            (uint8_t)(((message) >> (CAN_BYTE_7_POSITION * CAN_BYTE_LENGTH)) & CAN_MESSAGE_BIT_MASK_ONE_BYTE);
    } else {
        /* Endianness must be big or little */
        FAS_ASSERT(FAS_TRAP);
    }
}

extern void CAN_RxGetSignalDataFromMessageData(
    uint64_t message,
    uint64_t bitStart,
    uint8_t bitLength,
    uint64_t *pCanSignal,
    CAN_ENDIANNESS_e endianness) {
    /* AXIVION Routine Generic-MissingParameterAssert: message: parameter accepts whole range */
    FAS_ASSERT(pCanSignal != NULL_PTR);
    FAS_ASSERT((endianness == CAN_BIG_ENDIAN) || (endianness == CAN_LITTLE_ENDIAN));
    /* The longest message may be CAN_SIGNAL_MAX_SIZE long */
    FAS_ASSERT(bitLength <= CAN_SIGNAL_MAX_SIZE);
    /* A signal must contain at least one bit */
    FAS_ASSERT(bitLength > 0u);
    /* Signal start can not be outside of message data */
    FAS_ASSERT(bitStart < CAN_SIGNAL_MAX_SIZE);
    uint64_t position = bitStart;

    if (endianness == CAN_BIG_ENDIAN) {
        position = CAN_ConvertBitStartBigEndian(bitStart, bitLength);
    }

    /* A valid message has to start before the end of the message */
    FAS_ASSERT(position < CAN_SIGNAL_MAX_SIZE);
    /* Check for a plausible message length (sum of start bit and length shall
       not be larger than 64, otherwise it will not fit into the message) */
    FAS_ASSERT((position + bitLength) <= CAN_SIGNAL_MAX_SIZE);

    /* Prepare a mask and assemble message */
    uint64_t mask = UINT64_MAX;

    if (bitLength == CAN_SIGNAL_MAX_SIZE) {
        /* Since a bit-shift of 64 on a 64bit integer is undefined, set to desired 0 */
        mask = 0u;
    } else {
        mask <<= bitLength;
    }
    mask = ~mask;

    *pCanSignal = (message >> position) & mask;
}

extern void CAN_RxGetMessageDataFromCanData(
    uint64_t *pMessage,
    const uint8_t *const kpkCanData,
    CAN_ENDIANNESS_e endianness) {
    FAS_ASSERT(pMessage != NULL_PTR);
    FAS_ASSERT(kpkCanData != NULL_PTR);
    /* Swap byte order if necessary */
    if (endianness == CAN_BIG_ENDIAN) {
        *pMessage = ((((uint64_t)kpkCanData[CAN_BYTE_0_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_7_POSITION * CAN_BYTE_LENGTH)) |
                    ((((uint64_t)kpkCanData[CAN_BYTE_1_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_6_POSITION * CAN_BYTE_LENGTH)) |
                    ((((uint64_t)kpkCanData[CAN_BYTE_2_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_5_POSITION * CAN_BYTE_LENGTH)) |
                    ((((uint64_t)kpkCanData[CAN_BYTE_3_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_4_POSITION * CAN_BYTE_LENGTH)) |
                    ((((uint64_t)kpkCanData[CAN_BYTE_4_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_3_POSITION * CAN_BYTE_LENGTH)) |
                    ((((uint64_t)kpkCanData[CAN_BYTE_5_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_2_POSITION * CAN_BYTE_LENGTH)) |
                    ((((uint64_t)kpkCanData[CAN_BYTE_6_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_1_POSITION * CAN_BYTE_LENGTH)) |
                    ((((uint64_t)kpkCanData[CAN_BYTE_7_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_0_POSITION * CAN_BYTE_LENGTH));
    } else if (endianness == CAN_LITTLE_ENDIAN) {
        *pMessage = ((((uint64_t)kpkCanData[CAN_BYTE_0_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_0_POSITION * CAN_BYTE_LENGTH)) |
                    ((((uint64_t)kpkCanData[CAN_BYTE_1_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_1_POSITION * CAN_BYTE_LENGTH)) |
                    ((((uint64_t)kpkCanData[CAN_BYTE_2_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_2_POSITION * CAN_BYTE_LENGTH)) |
                    ((((uint64_t)kpkCanData[CAN_BYTE_3_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_3_POSITION * CAN_BYTE_LENGTH)) |
                    ((((uint64_t)kpkCanData[CAN_BYTE_4_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_4_POSITION * CAN_BYTE_LENGTH)) |
                    ((((uint64_t)kpkCanData[CAN_BYTE_5_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_5_POSITION * CAN_BYTE_LENGTH)) |
                    ((((uint64_t)kpkCanData[CAN_BYTE_6_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_6_POSITION * CAN_BYTE_LENGTH)) |
                    ((((uint64_t)kpkCanData[CAN_BYTE_7_POSITION]) & CAN_MESSAGE_BIT_MASK_ONE_BYTE)
                     << (CAN_BYTE_7_POSITION * CAN_BYTE_LENGTH));
    } else {
        /* Endianness must be big or little */
        FAS_ASSERT(FAS_TRAP);
    }
}

extern uint8_t CAN_ConvertBooleanToInteger(bool input) {
    uint8_t returnValue = 0u;
    if (input == true) {
        returnValue = 1u;
    }
    return returnValue;
}

/* Initial value variable, poly: 0x2F, xor value: 0xFF */
extern uint8_t Compute_CRC8H2F(const uint8_t *Crc_DataPtr, uint32_t Crc_Length, uint8_t Crc_StartValue8H2F) {
    uint8_t crc = Crc_StartValue8H2F;

    for (uint32_t i = 0; i < Crc_Length; ++i) {
        uint8_t data = Crc_DataPtr[i] ^ crc;
        crc          = CRC8H2F_TABLE[data];
    }

    return (crc ^ 0xFF);
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
extern uint64_t TEST_CAN_ConvertBitStartBigEndian(uint64_t bitStart, uint64_t bitLength) {
    uint64_t result = CAN_ConvertBitStartBigEndian(bitStart, bitLength);
    return result;
}
#endif
