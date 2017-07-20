/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_SRF10)

#include "build/build_config.h"


#include "drivers/time.h"
#include "drivers/bus_i2c.h"

#include "drivers/rangefinder.h"
#include "drivers/rangefinder_srf10.h"

#ifndef SRF10_I2C_INSTANCE
#define SRF10_I2C_INSTANCE I2CDEV_1
#endif

// Technical specification is at: http://robot-electronics.co.uk/htm/srf10tech.htm
#define SRF10_MAX_RANGE_CM 600 // 6m, from SFR10 spec sheet
// see http://www.robot-electronics.co.uk/htm/sonar_faq.htm for cone angles
// FAQ states 55 degrees, conservatively reduced to 50 degrees here
#define SRF10_DETECTION_CONE_DECIDEGREES 500
#define SRF10_DETECTION_CONE_EXTENDED_DECIDEGREES 500

// from spec sheet, when range set to 1100cm
// speed of sound is 340m/s, echo time for 1100cm is 65ms
#define SRF10_MinimumFiringIntervalFor1100cmRangeMs 65
// echo time for 600cm is 36ms, round this up to 40
#define SRF10_MinimumFiringIntervalFor600cmRangeMs 40

// SRF10 hardware constants
#define SRF10_Address 0xE0
#define SRF10_AddressI2C (SRF10_Address>>1) // the I2C 7 bit address

#define SRF10_READ_SoftwareRevision 0x00
#define SRF10_READ_Unused 0x01 // (read value returned is 0x80)
#define SRF10_READ_Unused_ReturnValue 0x80
#define SRF10_READ_RangeHighByte 0x02
#define SRF10_READ_RangeLowByte 0x03

#define SRF10_WRITE_CommandRegister 0x00
#define SRF10_WRITE_MaxGainRegister 0x01 //(default gain value is 16)
#define SRF10_WRITE_RangeRegister 0x02 //(default range value is 255)

#define SRF10_COMMAND_InitiateRangingInches 0x50
#define SRF10_COMMAND_InitiateRangingCm 0x51
#define SRF10_COMMAND_InitiateRangingMicroSeconds 0x53

#define SRF10_COMMAND_SetGain_40 0x00
#define SRF10_COMMAND_SetGain_100 0x06
#define SRF10_COMMAND_SetGain_200 0x09
#define SRF10_COMMAND_SetGain_300 0x0B
#define SRF10_COMMAND_SetGain_400 0x0D
#define SRF10_COMMAND_SetGain_500 0x0E
#define SRF10_COMMAND_SetGain_600 0x0F
#define SRF10_COMMAND_SetGain_700 0x10 // default and maximum
#define SRF10_COMMAND_SetGain_Max 0x10 // default

#define SRF10_COMMAND_ChangeAddress1 0xA0
#define SRF10_COMMAND_ChangeAddress2 0xAA
#define SRF10_COMMAND_ChangeAddress3 0xA5

// The range is (RangeRegister + 1) * 43mm
#define SRF10_RangeValue43mm 0
#define SRF10_RangeValue86mm 1
#define SRF10_RangeValue1m 24
#define SRF10_RangeValue4m 93
#define SRF10_RangeValue6m 139 // maximum range
#define SRF10_RangeValue11m 0xFF // exceeds actual maximum range

STATIC_UNIT_TESTED volatile int32_t srf10measurementCm = RANGEFINDER_OUT_OF_RANGE;
static int16_t minimumFiringIntervalMs;
static uint32_t timeOfLastMeasurementMs;
static bool isSensorResponding = true;

#ifdef UNIT_TEST
bool i2cWrite(uint8_t addr_, uint8_t reg, uint8_t data) {UNUSED(addr_); UNUSED(reg); UNUSED(data); return false;}
bool i2cRead(uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf) {UNUSED(addr_); UNUSED(reg);UNUSED(len); UNUSED(buf); return false;}
#endif

static bool i2c_srf10_send_command(uint8_t command)
{
    isSensorResponding = i2cWrite(SRF10_I2C_INSTANCE, SRF10_AddressI2C, SRF10_WRITE_CommandRegister, command);
    return isSensorResponding;
}

static bool i2c_srf10_send_byte(uint8_t i2cRegister, uint8_t val)
{
    isSensorResponding = i2cWrite(SRF10_I2C_INSTANCE, SRF10_AddressI2C, i2cRegister, val);
    return isSensorResponding;
}

static uint8_t i2c_srf10_read_byte(uint8_t i2cRegister)
{
    uint8_t byte;
    isSensorResponding = i2cRead(SRF10_I2C_INSTANCE, SRF10_AddressI2C, i2cRegister, 1, &byte);
    return byte;
}

static void srf10_init(void)
{
    i2c_srf10_send_byte(SRF10_WRITE_MaxGainRegister, SRF10_COMMAND_SetGain_Max);
    i2c_srf10_send_byte(SRF10_WRITE_RangeRegister, SRF10_RangeValue6m);
    // initiate first ranging command
    i2c_srf10_send_command(SRF10_COMMAND_InitiateRangingCm);
    timeOfLastMeasurementMs = millis();
}

/*
 * Start a range reading
 * Called periodically by the scheduler
 */
static void srf10_start_reading(void)
{
    // check if there is a measurement outstanding, 0xFF is returned if no measurement
    const uint8_t revision = i2c_srf10_read_byte(SRF10_READ_SoftwareRevision);
    if (revision != 0xFF) {
        // there is a measurement
        const uint8_t lowByte = i2c_srf10_read_byte(SRF10_READ_RangeLowByte);
        const uint8_t highByte = i2c_srf10_read_byte(SRF10_READ_RangeHighByte);
        srf10measurementCm =  highByte << 8 | lowByte;
        if (srf10measurementCm > SRF10_MAX_RANGE_CM) {
            srf10measurementCm = RANGEFINDER_OUT_OF_RANGE;
        }
    }
    const timeMs_t timeNowMs = millis();
    if (timeNowMs > timeOfLastMeasurementMs + minimumFiringIntervalMs) {
        // measurement repeat interval should be greater than minimumFiringIntervalMs
        // to avoid interference between connective measurements.
        timeOfLastMeasurementMs = timeNowMs;
        i2c_srf10_send_command(SRF10_COMMAND_InitiateRangingCm);
    }
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
static int32_t srf10_get_distance(void)
{
    if (isSensorResponding) {
        return srf10measurementCm;
    }
    else {
        return RANGEFINDER_HARDWARE_FAILURE;
    }
}

bool srf10Detect(rangefinderDev_t *dev)
{
    const uint8_t value = i2c_srf10_read_byte(SRF10_READ_Unused);
    if (value == SRF10_READ_Unused_ReturnValue) {
        dev->delayMs = SRF10_MinimumFiringIntervalFor600cmRangeMs + 10; // set up the SRF10 hardware for a range of 6m + margin of 10ms
        dev->maxRangeCm = SRF10_MAX_RANGE_CM;
        dev->detectionConeDeciDegrees = SRF10_DETECTION_CONE_DECIDEGREES;
        dev->detectionConeExtendedDeciDegrees = SRF10_DETECTION_CONE_EXTENDED_DECIDEGREES;

        dev->init = &srf10_init;
        dev->update = &srf10_start_reading;
        dev->read = &srf10_get_distance;

        return true;
    }
    else {
        return false;
    }
}
#endif
