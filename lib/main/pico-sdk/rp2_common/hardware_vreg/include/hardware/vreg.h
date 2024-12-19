/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_VREG_H
#define _HARDWARE_VREG_H

#include "pico.h"

#if PICO_RP2040
#include "hardware/structs/vreg_and_chip_reset.h"
#else
#include "hardware/structs/powman.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** \file vreg.h
 *  \defgroup hardware_vreg hardware_vreg
 *
 * \brief Voltage Regulation API
 *
 */

/** Possible voltage values that can be applied to the regulator
 */
enum vreg_voltage {
#if !PICO_RP2040
    VREG_VOLTAGE_0_55 = 0b00000,
    VREG_VOLTAGE_0_60 = 0b00001,
    VREG_VOLTAGE_0_65 = 0b00010,
    VREG_VOLTAGE_0_70 = 0b00011,
    VREG_VOLTAGE_0_75 = 0b00100,
    VREG_VOLTAGE_0_80 = 0b00101,
#endif
    VREG_VOLTAGE_0_85 = 0b00110,    ///< 0.85 V
    VREG_VOLTAGE_0_90 = 0b00111,    ///< 0.90 V
    VREG_VOLTAGE_0_95 = 0b01000,    ///< 0.95 V
    VREG_VOLTAGE_1_00 = 0b01001,    ///< 1.00 V
    VREG_VOLTAGE_1_05 = 0b01010,    ///< 1.05 V
    VREG_VOLTAGE_1_10 = 0b01011,    ///< 1.10 V
    VREG_VOLTAGE_1_15 = 0b01100,    ///< 1.15 V
    VREG_VOLTAGE_1_20 = 0b01101,    ///< 1.20 V
    VREG_VOLTAGE_1_25 = 0b01110,    ///< 1.25 V
    VREG_VOLTAGE_1_30 = 0b01111,    ///< 1.30 V
#if !PICO_RP2040
    // Above this point you will need to set POWMAN_VREG_CTRL_DISABLE_VOLTAGE_LIMIT
    VREG_VOLTAGE_1_35 = 0b10000,
    VREG_VOLTAGE_1_40 = 0b10001,
    VREG_VOLTAGE_1_50 = 0b10010,
    VREG_VOLTAGE_1_60 = 0b10011,
    VREG_VOLTAGE_1_65 = 0b10100,
    VREG_VOLTAGE_1_70 = 0b10101,
    VREG_VOLTAGE_1_80 = 0b10110,
    VREG_VOLTAGE_1_90 = 0b10111,
    VREG_VOLTAGE_2_00 = 0b11000,
    VREG_VOLTAGE_2_35 = 0b11001,
    VREG_VOLTAGE_2_50 = 0b11010,
    VREG_VOLTAGE_2_65 = 0b11011,
    VREG_VOLTAGE_2_80 = 0b11100,
    VREG_VOLTAGE_3_00 = 0b11101,
    VREG_VOLTAGE_3_15 = 0b11110,
    VREG_VOLTAGE_3_30 = 0b11111,
#endif

    // Note the "max" here assumes that VREG_CTRL_DISABLE_VOLTAGE_LIMIT is not set
    VREG_VOLTAGE_MIN = VREG_VOLTAGE_0_85,      ///< Always the minimum possible voltage
    VREG_VOLTAGE_DEFAULT = VREG_VOLTAGE_1_10,  ///< Default voltage on power up.
    VREG_VOLTAGE_MAX = VREG_VOLTAGE_1_30,      ///< Always the maximum possible voltage
};


/*! \brief  Set voltage
 *  \ingroup hardware_vreg
 *
 * \param voltage  The voltage (from enumeration \ref vreg_voltage) to apply to the voltage regulator
 **/
void vreg_set_voltage(enum vreg_voltage voltage);


/*! \brief  Enable use of voltages beyond the safe range of operation
 *  \ingroup hardware_vreg
 *
 * This allows voltages beyond VREG_VOLTAGE_MAX to be used, on platforms where
 * they are available (e.g. RP2350). Attempting to set a higher voltage
 * without first calling this function will result in a voltage of
 * VREG_VOLTAGE_MAX.
 **/
void vreg_disable_voltage_limit(void);

#ifdef __cplusplus
}
#endif

#endif
