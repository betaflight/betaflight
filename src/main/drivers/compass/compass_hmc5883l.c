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

#include <math.h>

#include "platform.h"

#ifdef USE_MAG_HMC5883

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/time.h"
#include "drivers/nvic.h"
#include "drivers/io.h"
#include "drivers/exti.h"
#include "drivers/bus_i2c.h"
#include "drivers/light_led.h"

#include "drivers/logging.h"

#include "drivers/sensor.h"
#include "drivers/compass/compass.h"

#include "drivers/compass/compass_hmc5883l.h"

//#define DEBUG_MAG_DATA_READY_INTERRUPT

// HMC5883L, default address 0x1E
// NAZE Target connections
// PB12 connected to MAG_DRDY on rev4 hardware
// PC14 connected to MAG_DRDY on rev5 hardware

/* CTRL_REGA: Control Register A
 * Read Write
 * Default value: 0x10
 * 7:5  0   These bits must be cleared for correct operation.
 * 4:2 DO2-DO0: Data Output Rate Bits
 *             DO2 |  DO1 |  DO0 |   Minimum Data Output Rate (Hz)
 *            ------------------------------------------------------
 *              0  |  0   |  0   |            0.75
 *              0  |  0   |  1   |            1.5
 *              0  |  1   |  0   |            3
 *              0  |  1   |  1   |            7.5
 *              1  |  0   |  0   |           15 (default)
 *              1  |  0   |  1   |           30
 *              1  |  1   |  0   |           75
 *              1  |  1   |  1   |           Not Used
 * 1:0 MS1-MS0: Measurement Configuration Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0   |  Normal
 *              0  |  1   |  Positive Bias
 *              1  |  0   |  Negative Bias
 *              1  |  1   |  Not Used
 *
 * CTRL_REGB: Control RegisterB
 * Read Write
 * Default value: 0x20
 * 7:5 GN2-GN0: Gain Configuration Bits.
 *             GN2 |  GN1 |  GN0 |   Mag Input   | Gain       | Output Range
 *                 |      |      |  Range[Ga]    | [LSB/mGa]  |
 *            ------------------------------------------------------
 *              0  |  0   |  0   |  �0.88Ga      |   1370     | 0xF800?0x07FF (-2048:2047)
 *              0  |  0   |  1   |  �1.3Ga (def) |   1090     | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  0   |  �1.9Ga       |   820      | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  1   |  �2.5Ga       |   660      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  0   |  �4.0Ga       |   440      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  1   |  �4.7Ga       |   390      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  0   |  �5.6Ga       |   330      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  1   |  �8.1Ga       |   230      | 0xF800?0x07FF (-2048:2047)
 *                               |Not recommended|
 *
 * 4:0 CRB4-CRB: 0 This bit must be cleared for correct operation.
 *
 * _MODE_REG: Mode Register
 * Read Write
 * Default value: 0x02
 * 7:2  0   These bits must be cleared for correct operation.
 * 1:0 MD1-MD0: Mode Select Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0   |  Continuous-Conversion Mode.
 *              0  |  1   |  Single-Conversion Mode
 *              1  |  0   |  Negative Bias
 *              1  |  1   |  Sleep Mode
 */

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03

#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16f)       // X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16f)       // Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08f)       // Z axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0f / 390.0f)    // Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0f / 390.0f)    // High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

static float magGain[3] = { 1.0f, 1.0f, 1.0f };

static const hmc5883Config_t *hmc5883Config = NULL;

#ifdef USE_MAG_DATA_READY_SIGNAL

static IO_t intIO;
static extiCallbackRec_t hmc5883_extiCallbackRec;

static void hmc5883_extiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);
#ifdef DEBUG_MAG_DATA_READY_INTERRUPT
    // Measure the delta between calls to the interrupt handler
    // currently should be around 65/66 milli seconds / 15hz output rate
    static uint32_t lastCalledAt = 0;
    static int32_t callDelta = 0;

    uint32_t now = millis();
    callDelta = now - lastCalledAt;

    //UNUSED(callDelta);
    debug[0] = callDelta;

    lastCalledAt = now;
#endif
}
#endif

static void hmc5883lConfigureDataReadyInterruptHandling(void)
{
#ifdef USE_MAG_DATA_READY_SIGNAL

    if (!(hmc5883Config->intTag)) {
        return;
    }
    intIO = IOGetByTag(hmc5883Config->intTag);
#ifdef ENSURE_MAG_DATA_READY_IS_HIGH
    uint8_t status = IORead(intIO);
    if (!status) {
        return;
    }
#endif

    EXTIHandlerInit(&hmc5883_extiCallbackRec, hmc5883_extiHandler);
    EXTIConfig(intIO, &hmc5883_extiCallbackRec, NVIC_PRIO_MAG_INT_EXTI, EXTI_Trigger_Rising);
    EXTIEnable(intIO, true);
#endif
}

static bool hmc5883lRead(magDev_t* magDev)
{
    uint8_t buf[6];

    bool ack = i2cRead(MAG_I2C_INSTANCE, MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);
    if (!ack) {
        magDev->magADCRaw[X] = 0;
        magDev->magADCRaw[Y] = 0;
        magDev->magADCRaw[Z] = 0;
        return false;
    }
    // During calibration, magGain is 1.0, so the read returns normal non-calibrated values.
    // After calibration is done, magGain is set to calculated gain values.
    magDev->magADCRaw[X] = (int16_t)(buf[0] << 8 | buf[1]) * magGain[X];
    magDev->magADCRaw[Z] = (int16_t)(buf[2] << 8 | buf[3]) * magGain[Z];
    magDev->magADCRaw[Y] = (int16_t)(buf[4] << 8 | buf[5]) * magGain[Y];

    return true;
}

#define INITIALISATION_MAX_READ_FAILURES 5
static bool hmc5883lInit(magDev_t* magDev)
{
    int32_t xyz_total[3] = { 0, 0, 0 }; // 32 bit totals so they won't overflow.
    bool bret = true;           // Error indicator

    delay(50);
    i2cWrite(MAG_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_CONFA, 0x18 + HMC_POS_BIAS);   // Reg A DOR = 0x18 + MS1, MS0 set to pos bias
    // Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
    // The new gain setting is effective from the second measurement and on.
    i2cWrite(MAG_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_CONFB, 0x80); // Set the Gain to 4Ga (7:5->100)
    delay(100);
    hmc5883lRead(magDev);

    int validSamples1 = 0;
    int failedSamples1 = 0;
    int saturatedSamples1 = 0;
    while (validSamples1 < 10 && failedSamples1 < INITIALISATION_MAX_READ_FAILURES) { // Collect 10 samples
        i2cWrite(MAG_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_MODE, 1);
        delay(70);
        if (hmc5883lRead(magDev)) { // Get the raw values in case the scales have already been changed.
            // Detect saturation.
            if (-4096 >= MIN(magDev->magADCRaw[X], MIN(magDev->magADCRaw[Y], magDev->magADCRaw[Z]))) {
                ++saturatedSamples1;
                ++failedSamples1;
            } else {
                ++validSamples1;
                // Since the measurements are noisy, they should be averaged rather than taking the max.
                xyz_total[X] += magDev->magADCRaw[X];
                xyz_total[Y] += magDev->magADCRaw[Y];
                xyz_total[Z] += magDev->magADCRaw[Z];

            }
        } else {
            ++failedSamples1;
        }
        LED1_TOGGLE;
    }

    // Apply the negative bias. (Same gain)
    i2cWrite(MAG_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_CONFA, 0x18 + HMC_NEG_BIAS);   // Reg A DOR = 0x18 + MS1, MS0 set to negative bias.
    int validSamples2 = 0;
    int failedSamples2 = 0;
    int saturatedSamples2 = 0;
    while (validSamples2 < 10 && failedSamples2 < INITIALISATION_MAX_READ_FAILURES) { // Collect 10 samples
        i2cWrite(MAG_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_MODE, 1);
        delay(70);
        if (hmc5883lRead(magDev)) { // Get the raw values in case the scales have already been changed.
            // Detect saturation.
            if (-4096 >= MIN(magDev->magADCRaw[X], MIN(magDev->magADCRaw[Y], magDev->magADCRaw[Z]))) {
                ++saturatedSamples2;
                ++failedSamples2;
            } else {
                ++validSamples2;
                // Since the measurements are noisy, they should be averaged.
                xyz_total[X] -= magDev->magADCRaw[X];
                xyz_total[Y] -= magDev->magADCRaw[Y];
                xyz_total[Z] -= magDev->magADCRaw[Z];
            }
        } else {
            ++failedSamples2;
        }
        LED1_TOGGLE;
    }

    if (failedSamples1 >= INITIALISATION_MAX_READ_FAILURES || failedSamples2 >= INITIALISATION_MAX_READ_FAILURES) {
        addBootlogEvent4(BOOT_EVENT_HMC5883L_READ_OK_COUNT, BOOT_EVENT_FLAGS_NONE, validSamples1, validSamples2);
        addBootlogEvent4(BOOT_EVENT_HMC5883L_READ_FAILED, BOOT_EVENT_FLAGS_WARNING, failedSamples1, failedSamples2);
        bret = false;
    }
    if (saturatedSamples1 > 0 || saturatedSamples2 > 0) {
        addBootlogEvent4(BOOT_EVENT_HMC5883L_SATURATION, BOOT_EVENT_FLAGS_WARNING, saturatedSamples1, saturatedSamples2);
    }

    if (bret) {
        magGain[X] = fabsf(440.0f * HMC58X3_X_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[X]);
        magGain[Y] = fabsf(440.0f * HMC58X3_Y_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[Y]);
        magGain[Z] = fabsf(440.0f * HMC58X3_Z_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[Z]);
    } else {
        // Something went wrong so get a best guess
        magGain[X] = 1.0f;
        magGain[Y] = 1.0f;
        magGain[Z] = 1.0f;
    }

    // leave test mode
    i2cWrite(MAG_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_CONFA, 0x78);   // Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 75Hz ; normal measurement mode
    i2cWrite(MAG_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_CONFB, 0x20);   // Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    i2cWrite(MAG_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_MODE, 0x00);    // Mode register             -- 000000 00    continuous Conversion Mode
    delay(100);

    hmc5883lConfigureDataReadyInterruptHandling();

    return bret;
}

#define DETECTION_MAX_RETRY_COUNT   5
bool hmc5883lDetect(magDev_t* magDev, const hmc5883Config_t *hmc5883ConfigToUse)
{
    hmc5883Config = hmc5883ConfigToUse;

    for (int retryCount = 0; retryCount < DETECTION_MAX_RETRY_COUNT; retryCount++) {
        uint8_t sig = 0;
        bool ack = i2cRead(MAG_I2C_INSTANCE, MAG_ADDRESS, 0x0A, 1, &sig);
        if (ack && sig == 'H') {
            magDev->init = hmc5883lInit;
            magDev->read = hmc5883lRead;

            return true;
        }

        delay(10);
    }

    return false;
}
#endif
