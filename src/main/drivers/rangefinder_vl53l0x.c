/*
 * This file is part of INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 * =================================================================
 * Substantial part of VL53L0X API is derived from Pololu VL53L0X library.
 * The following applies to code derived from Pololu libraries:
 *
 * Copyright (c) 2017 Pololu Corporation.  For more information, see
 *
 * https://www.pololu.com/
 * https://forum.pololu.com/
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * =================================================================
 *
 * Most of the functionality of this library is based on the VL53L0X
 * API provided by ST (STSW-IMG005), and some of the explanatory
 * comments are quoted or paraphrased from the API source code, API
 * user manual (UM2039), and the VL53L0X datasheet.
 *
 * The following applies to source code reproduced or derived from
 * the API:
 *
 * -----------------------------------------------------------------
 *
 * Copyright © 2016, STMicroelectronics International N.V.  All
 * rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * - Neither the name of STMicroelectronics nor the
 * names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior
 * written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 * NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 * IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_VL53L0X)

#include "build/build_config.h"

#include "drivers/time.h"
#include "drivers/bus_i2c.h"

#include "drivers/rangefinder.h"
#include "drivers/rangefinder_vl53l0x.h"

#include "build/debug.h"

#ifndef RANGEFINDER_VL53L0X_INSTANCE
#define RANGEFINDER_VL53L0X_INSTANCE I2C_DEVICE
#endif

#define VL53L0X_MAX_RANGE_CM                250
#define VL53L0X_DETECTION_CONE_DECIDEGREES  900

#define VL53L0X_I2C_ADDRESS     0x29

/** @defgroup VL53L0X_DefineRegisters_group Define Registers
 *  @brief List of all the defined registers
 *  @{
 */
#define VL53L0X_REG_SYSRANGE_START                          0x00
#define VL53L0X_REG_SYSTEM_THRESH_HIGH                      0x0C
#define VL53L0X_REG_SYSTEM_THRESH_LOW                       0x0E
#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG                  0x01
#define VL53L0X_REG_SYSTEM_RANGE_CONFIG                     0x09
#define VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD          0x04
#define VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO            0x0A
#define VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH                 0x84
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR                  0x0B

/* Result registers */
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS                 0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS                     0x14

#define VL53L0X_REG_RESULT_CORE_PAGE  1
#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN   0xBC
#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN    0xC0
#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF   0xD0
#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF    0xD4
#define VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF             0xB6

/* Algo register */
#define VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM       0x28

#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS                0x8a

/* Check Limit registers */
#define VL53L0X_REG_MSRC_CONFIG_CONTROL                     0x60

#define VL53L0X_REG_PRE_RANGE_CONFIG_MIN_SNR                0x27
#define VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW        0x56
#define VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH       0x57
#define VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT      0x64

#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_SNR              0x67
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW      0x47
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH     0x48
#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT   0x44


#define VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI        0x61
#define VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO        0x62

/* PRE RANGE registers */
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD           0x50
#define VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI      0x51
#define VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO      0x52

#define VL53L0X_REG_SYSTEM_HISTOGRAM_BIN                    0x81
#define VL53L0X_REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT   0x33
#define VL53L0X_REG_HISTOGRAM_CONFIG_READOUT_CTRL           0x55

#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD         0x70
#define VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI    0x71
#define VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO    0x72
#define VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS   0x20

#define VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP              0x46


#define VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N             0xbf
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID                 0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID              0xc2

#define VL53L0X_REG_OSC_CALIBRATE_VAL                       0xf8

#define VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH               0x32
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0        0xB0
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_1        0xB1
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_2        0xB2
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_3        0xB3
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_4        0xB4
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_5        0xB5

#define VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT       0xB6
#define VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD     0x4E /* 0x14E */
#define VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET        0x4F /* 0x14F */
#define VL53L0X_REG_POWER_MANAGEMENT_GO1_POWER_FORCE        0x80

#define VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV       0x89

#define VL53L0X_REG_ALGO_PHASECAL_LIM                       0x30 /* 0x130 */
#define VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT            0x30

#define setTimeout(timeout)                 {timeoutValueMs = timeout;}
#define getTimeout()                        (timeoutValueMs)
#define startTimeout()                      (timeoutStartMs = millis())
#define checkTimeoutExpired()               (timeoutValueMs > 0 && ((uint16_t)millis() - timeoutStartMs) > timeoutValueMs)
#define decodeVcselPeriod(reg_val)          (((reg_val) + 1) << 1)
#define encodeVcselPeriod(period_pclks)     (((period_pclks) >> 1) - 1)
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

typedef enum {
    VcselPeriodPreRange,
    VcselPeriodFinalRange
} vcselPeriodType_e;

typedef enum {
    MEASUREMENT_START,
    MEASUREMENT_WAIT,
    MEASUREMENT_READ,
} measurementSteps_e;

typedef struct {
    bool tcc;
    bool msrc;
    bool dss;
    bool pre_range;
    bool final_range;
} sequenceStepEnables_t;

typedef struct {
    uint16_t pre_range_vcsel_period_pclks;
    uint16_t final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks;
    uint16_t pre_range_mclks;
    uint16_t final_range_mclks;
    uint32_t msrc_dss_tcc_us;
    uint16_t pre_range_us;
    uint16_t final_range_us;
} sequenceStepTimeouts_t;

static int32_t lastMeasurementCm = RANGEFINDER_OUT_OF_RANGE;
static bool lastMeasurementIsNew = false;
static uint8_t stopVariable = 0;    // StopVariable field of VL53L0X_DevData_t structure in ST's API
static bool isInitialized = false;
static bool isResponding = true;
static uint32_t measurementTimingBudgetUs;
static timeMs_t timeoutStartMs = 0;
static timeMs_t timeoutValueMs = 0;
static measurementSteps_e measSteps = MEASUREMENT_START;

static uint8_t readReg(uint8_t reg)
{
    uint8_t byte;
    isResponding = i2cRead(RANGEFINDER_VL53L0X_INSTANCE, VL53L0X_I2C_ADDRESS, reg, 1, &byte);
    return byte;
}

static uint16_t readReg16(uint8_t reg)
{
    uint8_t bytes[2];
    isResponding = i2cRead(RANGEFINDER_VL53L0X_INSTANCE, VL53L0X_I2C_ADDRESS, reg, 2, &bytes[0]);
    return ((uint16_t)bytes[0] << 8) |
           ((uint16_t)bytes[1] << 0);
}

static void readMulti(uint8_t reg, uint8_t * dst, uint8_t count)
{
    isResponding = i2cRead(RANGEFINDER_VL53L0X_INSTANCE, VL53L0X_I2C_ADDRESS, reg, count, dst);
}

static void writeReg(uint8_t reg, uint8_t value)
{
    isResponding = i2cWrite(RANGEFINDER_VL53L0X_INSTANCE, VL53L0X_I2C_ADDRESS, reg, value);
}

static void writeReg16(uint8_t reg, uint16_t value)
{
    uint8_t bytes[2];
    bytes[0] = (value >> 8) & 0xFF;
    bytes[1] = (value >> 0) & 0xFF;
    isResponding = i2cWriteBuffer(RANGEFINDER_VL53L0X_INSTANCE, VL53L0X_I2C_ADDRESS, reg, 2, &bytes[0]);
}

static void writeReg32(uint8_t reg, uint16_t value)
{
    uint8_t bytes[4];
    bytes[0] = (value >> 24) & 0xFF;
    bytes[1] = (value >> 16) & 0xFF;
    bytes[2] = (value >>  8) & 0xFF;
    bytes[3] = (value >>  0) & 0xFF;
    isResponding = i2cWriteBuffer(RANGEFINDER_VL53L0X_INSTANCE, VL53L0X_I2C_ADDRESS, reg, 4, &bytes[0]);
}

static void writeMulti(uint8_t reg, uint8_t const * src, uint8_t count)
{
    isResponding = i2cWriteBuffer(RANGEFINDER_VL53L0X_INSTANCE, VL53L0X_I2C_ADDRESS, reg, count, (uint8_t *)src);
}


// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t decodeTimeout(uint16_t reg_val)
{
    // format: "(LSByte * 2^MSByte) + 1"
    return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    else {
        return 0;
    }
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool setSignalRateLimit(float limit_Mcps)
{
    if (limit_Mcps < 0 || limit_Mcps > 511.99) {
        return false;
    }

    // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    writeReg16(VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
    return true;
}

// Get the return signal rate limit check value in MCPS
float getSignalRateLimit(void)
{
    return (float)readReg16(VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// based on VL53L0X_perform_single_ref_calibration()
bool performSingleRefCalibration(uint8_t vhv_init_byte)
{
    writeReg(VL53L0X_REG_SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

    startTimeout();
    while ((readReg(VL53L0X_REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (checkTimeoutExpired()) {
            return false;
        }
    }

    writeReg(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    writeReg(VL53L0X_REG_SYSRANGE_START, 0x00);

    return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void getSequenceStepEnables(sequenceStepEnables_t * enables)
{
    uint8_t sequence_config = readReg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG);

    enables->tcc          = (sequence_config >> 4) & 0x1;
    enables->dss          = (sequence_config >> 3) & 0x1;
    enables->msrc         = (sequence_config >> 2) & 0x1;
    enables->pre_range    = (sequence_config >> 6) & 0x1;
    enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t getVcselPulsePeriod(vcselPeriodType_e type)
{
    if (type == VcselPeriodPreRange) {
        return decodeVcselPeriod(readReg(VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD));
    }
    else if (type == VcselPeriodFinalRange) {
        return decodeVcselPeriod(readReg(VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    }
    else {
        return 255;
    }
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void getSequenceStepTimeouts(sequenceStepEnables_t const * enables, sequenceStepTimeouts_t * timeouts)
{
    timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

    timeouts->msrc_dss_tcc_mclks = readReg(VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

    timeouts->pre_range_mclks = decodeTimeout(readReg16(VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->pre_range_us = timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

    timeouts->final_range_mclks = decodeTimeout(readReg16(VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables->pre_range) {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool setMeasurementTimingBudget(uint32_t budget_us)
{
    sequenceStepEnables_t enables;
    sequenceStepTimeouts_t timeouts;

    const uint16_t StartOverhead      = 1320; // note that this is different than the value in get_
    const uint16_t EndOverhead        = 960;
    const uint16_t MsrcOverhead       = 660;
    const uint16_t TccOverhead        = 590;
    const uint16_t DssOverhead        = 690;
    const uint16_t PreRangeOverhead   = 660;
    const uint16_t FinalRangeOverhead = 550;

    uint32_t const MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget) { return false; }

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables(&enables);
    getSequenceStepTimeouts(&enables, &timeouts);

    if (enables.tcc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss) {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range) {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range) {
        used_budget_us += FinalRangeOverhead;

        // "Note that the final range timeout is determined by the timing
        // budget and the sum of all other timeouts within the sequence.
        // If there is no room for the final range timeout, then an error
        // will be set. Otherwise the remaining time will be applied to
        // the final range."

        if (used_budget_us > budget_us) {
            // "Requested timeout too big."
            return false;
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t final_range_timeout_mclks =
            timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range) {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        writeReg16(VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));

        // set_sequence_step_timeout() end

        measurementTimingBudgetUs = budget_us; // store for internal reuse
    }

    return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t getMeasurementTimingBudget(void)
{
    sequenceStepEnables_t enables;
    sequenceStepTimeouts_t timeouts;

    const uint16_t StartOverhead     = 1910; // note that this is different than the value in set_
    const uint16_t EndOverhead        = 960;
    const uint16_t MsrcOverhead       = 660;
    const uint16_t TccOverhead        = 590;
    const uint16_t DssOverhead        = 690;
    const uint16_t PreRangeOverhead   = 660;
    const uint16_t FinalRangeOverhead = 550;

    // "Start and end overhead times always present"
    uint32_t budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables(&enables);
    getSequenceStepTimeouts(&enables, &timeouts);

    if (enables.tcc) {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss) {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc) {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range) {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range) {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    measurementTimingBudgetUs = budget_us; // store for internal reuse
    return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
bool setVcselPulsePeriod(vcselPeriodType_e type, uint8_t period_pclks)
{
    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

    sequenceStepEnables_t enables;
    sequenceStepTimeouts_t timeouts;

    getSequenceStepEnables(&enables);
    getSequenceStepTimeouts(&enables, &timeouts);

    // "Apply specific settings for the requested clock period"
    // "Re-calculate and apply timeouts, in macro periods"

    // "When the VCSEL period for the pre or final range is changed,
    // the corresponding timeout must be read from the device using
    // the current VCSEL period, then the new VCSEL period can be
    // applied. The timeout then must be written back to the device
    // using the new VCSEL period.
    //
    // For the MSRC timeout, the same applies - this timeout being
    // dependant on the pre-range vcsel period."


    if (type == VcselPeriodPreRange) {
        // "Set phase check limits"
        switch (period_pclks) {
            case 12:
                writeReg(VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
                break;

            case 14:
                writeReg(VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
                break;

            case 16:
                writeReg(VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
                break;

            case 18:
                writeReg(VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
                break;

            default:
                // invalid period
                return false;
        }
        writeReg(VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

        // apply new VCSEL period
        writeReg(VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

        uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

        writeReg16(VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));

        // set_sequence_step_timeout() end

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

        uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

        writeReg(VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

        // set_sequence_step_timeout() end
    }
    else if (type == VcselPeriodFinalRange) {
        switch (period_pclks) {
            case 8:
                writeReg(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
                writeReg(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                writeReg(VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
                writeReg(VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
                writeReg(0xFF, 0x01);
                writeReg(VL53L0X_REG_ALGO_PHASECAL_LIM, 0x30);
                writeReg(0xFF, 0x00);
                break;

            case 10:
                writeReg(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
                writeReg(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                writeReg(VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                writeReg(VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
                writeReg(0xFF, 0x01);
                writeReg(VL53L0X_REG_ALGO_PHASECAL_LIM, 0x20);
                writeReg(0xFF, 0x00);
                break;

            case 12:
                writeReg(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
                writeReg(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                writeReg(VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                writeReg(VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
                writeReg(0xFF, 0x01);
                writeReg(VL53L0X_REG_ALGO_PHASECAL_LIM, 0x20);
                writeReg(0xFF, 0x00);
                break;

            case 14:
                writeReg(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
                writeReg(VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                writeReg(VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                writeReg(VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
                writeReg(0xFF, 0x01);
                writeReg(VL53L0X_REG_ALGO_PHASECAL_LIM, 0x20);
                writeReg(0xFF, 0x00);
                break;

            default:
                // invalid period
                return false;
        }

        // apply new VCSEL period
        writeReg(VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

        if (enables.pre_range) {
            new_final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        writeReg16(VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks));
        // set_sequence_step_timeout end
    }
    else {
        // invalid type
        return false;
    }

    // "Finally, the timing budget must be re-applied"
    setMeasurementTimingBudget(measurementTimingBudgetUs);

    // "Perform the phase calibration. This is needed after changing on vcsel period."
    // VL53L0X_perform_phase_calibration() begin

    uint8_t sequence_config = readReg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG);
    writeReg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
    performSingleRefCalibration(0x0);
    writeReg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, sequence_config);

    // VL53L0X_perform_phase_calibration() end

    return true;
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool getSpadInfo(uint8_t * count, bool * type_is_aperture)
{
    uint8_t tmp;

    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);

    writeReg(0xFF, 0x06);
    writeReg(0x83, readReg(0x83) | 0x04);
    writeReg(0xFF, 0x07);
    writeReg(0x81, 0x01);

    writeReg(0x80, 0x01);

    writeReg(0x94, 0x6b);
    writeReg(0x83, 0x00);

    startTimeout();
    while (readReg(0x83) == 0x00)
    {
        if (checkTimeoutExpired()) {
            return false;
        }
    }

    writeReg(0x83, 0x01);
    tmp = readReg(0x92);

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    writeReg(0x81, 0x00);
    writeReg(0xFF, 0x06);
    writeReg(0x83, readReg(0x83)  & ~0x04);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x01);

    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    return true;
}

static void vl53l0x_Init(void)
{
    uint8_t byte;
    isInitialized = false;

    // VL53L0X_DataInit() begin
    // Switch sensor to 2.8V mode
    byte = readReg(VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
    writeReg(VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, byte | 0x01);

    // Set I2C standard mode
    writeReg(0x88, 0x00);

    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    stopVariable = readReg(0x91);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    byte = readReg(VL53L0X_REG_MSRC_CONFIG_CONTROL);
    writeReg(VL53L0X_REG_MSRC_CONFIG_CONTROL, byte | 0x12);

    // set final range signal rate limit to 0.25 MCPS (million counts per second)
    setSignalRateLimit(0.25);

    writeReg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    // VL53L0X_StaticInit() begin

    uint8_t spad_count;
    bool spad_type_is_aperture;
    if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) {
        // Init failed
        return;
    }

    // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
    // the API, but the same data seems to be more easily readable from
    // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
    uint8_t ref_spad_map[6];
    readMulti(VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // VL53L0X_set_reference_spads() begin (assume NVM values are valid)
    writeReg(0xFF, 0x01);
    writeReg(VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    writeReg(VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    writeReg(0xFF, 0x00);
    writeReg(VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++)
    {
        if (i < first_spad_to_enable || spads_enabled == spad_count)
        {
            // This bit is lower than the first one that should be enabled, or
            // (reference_spad_count) bits have already been enabled, so zero this bit
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        }
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
        {
            spads_enabled++;
        }
    }

    writeMulti(VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_load_tuning_settings() begin
    // DefaultTuningSettings from vl53l0x_tuning.h

    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);

    writeReg(0xFF, 0x00);
    writeReg(0x09, 0x00);
    writeReg(0x10, 0x00);
    writeReg(0x11, 0x00);

    writeReg(0x24, 0x01);
    writeReg(0x25, 0xFF);
    writeReg(0x75, 0x00);

    writeReg(0xFF, 0x01);
    writeReg(0x4E, 0x2C);
    writeReg(0x48, 0x00);
    writeReg(0x30, 0x20);

    writeReg(0xFF, 0x00);
    writeReg(0x30, 0x09);
    writeReg(0x54, 0x00);
    writeReg(0x31, 0x04);
    writeReg(0x32, 0x03);
    writeReg(0x40, 0x83);
    writeReg(0x46, 0x25);
    writeReg(0x60, 0x00);
    writeReg(0x27, 0x00);
    writeReg(0x50, 0x06);
    writeReg(0x51, 0x00);
    writeReg(0x52, 0x96);
    writeReg(0x56, 0x08);
    writeReg(0x57, 0x30);
    writeReg(0x61, 0x00);
    writeReg(0x62, 0x00);
    writeReg(0x64, 0x00);
    writeReg(0x65, 0x00);
    writeReg(0x66, 0xA0);

    writeReg(0xFF, 0x01);
    writeReg(0x22, 0x32);
    writeReg(0x47, 0x14);
    writeReg(0x49, 0xFF);
    writeReg(0x4A, 0x00);

    writeReg(0xFF, 0x00);
    writeReg(0x7A, 0x0A);
    writeReg(0x7B, 0x00);
    writeReg(0x78, 0x21);

    writeReg(0xFF, 0x01);
    writeReg(0x23, 0x34);
    writeReg(0x42, 0x00);
    writeReg(0x44, 0xFF);
    writeReg(0x45, 0x26);
    writeReg(0x46, 0x05);
    writeReg(0x40, 0x40);
    writeReg(0x0E, 0x06);
    writeReg(0x20, 0x1A);
    writeReg(0x43, 0x40);

    writeReg(0xFF, 0x00);
    writeReg(0x34, 0x03);
    writeReg(0x35, 0x44);

    writeReg(0xFF, 0x01);
    writeReg(0x31, 0x04);
    writeReg(0x4B, 0x09);
    writeReg(0x4C, 0x05);
    writeReg(0x4D, 0x04);

    writeReg(0xFF, 0x00);
    writeReg(0x44, 0x00);
    writeReg(0x45, 0x20);
    writeReg(0x47, 0x08);
    writeReg(0x48, 0x28);
    writeReg(0x67, 0x00);
    writeReg(0x70, 0x04);
    writeReg(0x71, 0x01);
    writeReg(0x72, 0xFE);
    writeReg(0x76, 0x00);
    writeReg(0x77, 0x00);

    writeReg(0xFF, 0x01);
    writeReg(0x0D, 0x01);

    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x01);
    writeReg(0x01, 0xF8);

    writeReg(0xFF, 0x01);
    writeReg(0x8E, 0x01);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    // -- VL53L0X_SetGpioConfig() begin

    writeReg(VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    writeReg(VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, readReg(VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
    writeReg(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    measurementTimingBudgetUs = getMeasurementTimingBudget();

    // "Disable MSRC and TCC by default"
    // MSRC = Minimum Signal Rate Check
    // TCC = Target CentreCheck
    // -- VL53L0X_SetSequenceStepEnable() begin

    writeReg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // "Recalculate timing budget"
    setMeasurementTimingBudget(measurementTimingBudgetUs);

    // -- VL53L0X_perform_vhv_calibration() begin
    writeReg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!performSingleRefCalibration(0x40)) {
        return;
    }

    // -- VL53L0X_perform_phase_calibration() begin

    writeReg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!performSingleRefCalibration(0x00)) {
        return;
    }

    // "restore the previous Sequence Config"
    writeReg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // Apply range parameters
#if 1   // LONG RANGE
    setSignalRateLimit(0.25f);
    setVcselPulsePeriod(VcselPeriodPreRange, 18);
    setVcselPulsePeriod(VcselPeriodFinalRange, 14);
#endif

    // Set timeout
    setTimeout(100);

    // We are initialized
    isInitialized = true;
}

void vl53l0x_Update(void)
{
    if (!isInitialized) {
        return;
    }

#if 1   // SINGLE MEASUREMENT MODE
    switch (measSteps) {
        case MEASUREMENT_START:
            // Initiate new measurement
            writeReg(0x80, 0x01);
            writeReg(0xFF, 0x01);
            writeReg(0x00, 0x00);
            writeReg(0x91, stopVariable);
            writeReg(0x00, 0x01);
            writeReg(0xFF, 0x00);
            writeReg(0x80, 0x00);
            writeReg(VL53L0X_REG_SYSRANGE_START, 0x01);

            startTimeout();
            measSteps = MEASUREMENT_WAIT;
            break;

        case MEASUREMENT_WAIT:
            // Wait for data and read the measurement
            if (readReg(VL53L0X_REG_SYSRANGE_START) & 0x01) {
                if (checkTimeoutExpired()) {
                    lastMeasurementCm = RANGEFINDER_OUT_OF_RANGE;
                    measSteps = MEASUREMENT_START;
                }
            }
            else {
                startTimeout();
                measSteps = MEASUREMENT_READ;
            }
            break;

        case MEASUREMENT_READ:
            if ((readReg(VL53L0X_REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
                if (checkTimeoutExpired()) {
                    lastMeasurementCm = RANGEFINDER_OUT_OF_RANGE;
                    measSteps = MEASUREMENT_START;
                }
            }
            else {
                // assumptions: Linearity Corrective Gain is 1000 (default);
                uint16_t raw = readReg16(VL53L0X_REG_RESULT_RANGE_STATUS + 10);
                writeReg(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

                lastMeasurementCm = raw / 10;
                lastMeasurementIsNew = true;
                measSteps = MEASUREMENT_START;
            }
            break;
    }
#else   // CONTINUOUS MEASUREMENT MODE
    switch (measSteps) {
        case MEASUREMENT_START:
            // Initiate new measurement
            writeReg(0x80, 0x01);
            writeReg(0xFF, 0x01);
            writeReg(0x00, 0x00);
            writeReg(0x91, stopVariable);
            writeReg(0x00, 0x01);
            writeReg(0xFF, 0x00);
            writeReg(0x80, 0x00);

            uint16_t osc_calibrate_val = readReg16(VL53L0X_REG_OSC_CALIBRATE_VAL);

            if (osc_calibrate_val != 0) {
                writeReg32(VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD, 50 * osc_calibrate_val);
            }
            else {
                writeReg32(VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD, 50);
            }

            writeReg(VL53L0X_REG_SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK

            startTimeout();
            measSteps = MEASUREMENT_READ;
            break;

        case MEASUREMENT_READ:
        default:
            if ((readReg(VL53L0X_REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
                if (checkTimeoutExpired()) {
                    lastMeasurementCm = RANGEFINDER_OUT_OF_RANGE;

                    // Restart timeout
                    startTimeout();
                    measSteps = MEASUREMENT_READ;
                }
            }
            else {
                // assumptions: Linearity Corrective Gain is 1000 (default);
                uint16_t raw = readReg16(VL53L0X_REG_RESULT_RANGE_STATUS + 10);
                writeReg(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

                lastMeasurementCm = raw / 10;

                // Restart timeout
                startTimeout();
                measSteps = MEASUREMENT_READ;
            }
            break;
    }
#endif
}

int32_t vl53l0x_GetDistance(void)
{
    if (isResponding && isInitialized) {
        if (lastMeasurementIsNew) {
            lastMeasurementIsNew = false;
            return lastMeasurementCm < VL53L0X_MAX_RANGE_CM ? lastMeasurementCm : RANGEFINDER_OUT_OF_RANGE;
        }
        else {
            return RANGEFINDER_NO_NEW_DATA;
        }
    }
    else {
        return RANGEFINDER_HARDWARE_FAILURE;
    }
}

bool vl53l0xDetect(rangefinderDev_t *dev)
{
    uint8_t model_id = readReg(VL53L0X_REG_IDENTIFICATION_MODEL_ID);

    if (isResponding && model_id == 0xEE) {
        dev->delayMs = RANGEFINDER_VL53L0X_TASK_PERIOD_MS;
        dev->maxRangeCm = VL53L0X_MAX_RANGE_CM;
        dev->detectionConeDeciDegrees = VL53L0X_DETECTION_CONE_DECIDEGREES;
        dev->detectionConeExtendedDeciDegrees = VL53L0X_DETECTION_CONE_DECIDEGREES;

        dev->init = &vl53l0x_Init;
        dev->update = &vl53l0x_Update;
        dev->read = &vl53l0x_GetDistance;

        return true;
    }
    else {
        return false;
    }
}
#endif
