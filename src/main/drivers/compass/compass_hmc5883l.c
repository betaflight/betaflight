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

#if defined(USE_MAG_HMC5883) || defined(USE_MAG_SPI_HMC5883)

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"

#include "compass_hmc5883l.h"

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
 *              0  |  0  |  Normal
 *              0  |  1  |  Positive Bias
 *              1  |  0  |  Negative Bias
 *              1  |  1  |  Not Used
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
 *              0  |  0  |  Continuous-Conversion Mode.
 *              0  |  1  |  Single-Conversion Mode
 *              1  |  0  |  Negative Bias
 *              1  |  1  |  Sleep Mode
 */

#define HMC5883_MAG_I2C_ADDRESS     0x1E
#define HMC5883_DEVICE_ID           0x48

#define HMC58X3_REG_CONFA           0x00
#define HMC58X3_REG_CONFB           0x01
#define HMC58X3_REG_MODE            0x02
#define HMC58X3_REG_DATA            0x03
#define HMC58X3_REG_IDA             0x0A

#define HMC_CONFA_NORMAL            0x00
#define HMC_CONFA_POS_BIAS          0x01
#define HMC_CONFA_NEG_BIAS          0x02
#define HMC_CONFA_DOR_15HZ          0X10
#define HMC_CONFA_8_SAMLES          0X60
#define HMC_CONFB_GAIN_2_5GA        0X60
#define HMC_CONFB_GAIN_1_3GA        0X20
#define HMC_MODE_CONTINOUS          0X00
#define HMC_MODE_SINGLE             0X01

#define HMC58X3_X_SELF_TEST_GAUSS   (+1.16f)            // X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS   (+1.16f)            // Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS   (+1.08f)            // Z axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT         (243.0f / 390.0f)   // Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT        (575.0f / 390.0f)   // High limit when gain is 5.

#ifdef USE_MAG_DATA_READY_SIGNAL

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

static void hmc5883lConfigureDataReadyInterruptHandling(magDev_t* mag)
{
#ifdef USE_MAG_DATA_READY_SIGNAL
    if (mag->magIntExtiTag == IO_TAG_NONE) {
        return;
    }

    const IO_t magIntIO = IOGetByTag(mag->magIntExtiTag);

#ifdef ENSURE_MAG_DATA_READY_IS_HIGH
    uint8_t status = IORead(magIntIO);
    if (!status) {
        return;
    }
#endif

#if defined (STM32F7)
    IOInit(magIntIO, OWNER_COMPASS_EXTI, 0);
    EXTIHandlerInit(&mag->exti, hmc5883_extiHandler);
    EXTIConfig(magIntIO, &gmag->exti, NVIC_PRIO_MPU_INT_EXTI, IO_CONFIG(GPIO_MODE_INPUT,0,GPIO_NOPULL));
    EXTIEnable(magIntIO, true);
#else
    IOInit(magIntIO, OWNER_COMPASS_EXTI, 0);
    IOConfigGPIO(magIntIO, IOCFG_IN_FLOATING);
    EXTIHandlerInit(&mag->exti, hmc5883_extiHandler);
    EXTIConfig(magIntIO, &mag->exti, NVIC_PRIO_MAG_INT_EXTI, EXTI_Trigger_Rising);
    EXTIEnable(magIntIO, true);
#endif
#else
    UNUSED(mag);
#endif
}

#ifdef USE_MAG_SPI_HMC5883
static void hmc5883SpiInit(busDevice_t *busdev)
{
    IOHi(busdev->busdev_u.spi.csnPin); // Disable

    IOInit(busdev->busdev_u.spi.csnPin, OWNER_COMPASS_CS, 0);
    IOConfigGPIO(busdev->busdev_u.spi.csnPin, IOCFG_OUT_PP);

    spiSetDivisor(busdev->busdev_u.spi.instance, SPI_CLOCK_STANDARD);
}
#endif

static int16_t parseMag(uint8_t *raw, int16_t gain) {
  int ret = (int16_t)(raw[0] << 8 | raw[1]) * gain / 256;
  return constrain(ret, INT16_MIN, INT16_MAX);
}

static bool hmc5883lRead(magDev_t *mag, int16_t *magData)
{
    uint8_t buf[6];

    busDevice_t *busdev = &mag->busdev;

    bool ack = busReadRegisterBuffer(busdev, HMC58X3_REG_DATA, buf, 6);

    if (!ack) {
        return false;
    }
    // During calibration, magGain is 1.0, so the read returns normal non-calibrated values.
    // After calibration is done, magGain is set to calculated gain values.

    magData[X] = parseMag(buf + 0, mag->magGain[X]);
    magData[Z] = parseMag(buf + 2, mag->magGain[Z]);
    magData[Y] = parseMag(buf + 4, mag->magGain[Y]);

    return true;
}

static bool hmc5883lInit(magDev_t *mag)
{
    enum {
        polPos,
        polNeg
    };

    busDevice_t *busdev = &mag->busdev;

    int16_t magADC[3];
    int i;
    int32_t xyz_total[3] = { 0, 0, 0 }; // 32 bit totals so they won't overflow.
    bool bret = true;           // Error indicator

    mag->magGain[X] = 256;
    mag->magGain[Y] = 256;
    mag->magGain[Z] = 256;

    delay(50);

    busWriteRegister(busdev, HMC58X3_REG_CONFA, HMC_CONFA_DOR_15HZ | HMC_CONFA_POS_BIAS);   // Reg A DOR = 0x010 + MS1, MS0 set to pos bias

    // Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
    // The new gain setting is effective from the second measurement and on.

    busWriteRegister(busdev, HMC58X3_REG_CONFB, HMC_CONFB_GAIN_2_5GA); // Set the Gain to 2.5Ga (7:5->011)

    delay(100);

    hmc5883lRead(mag, magADC);

    for (int polarity = polPos; polarity <= polNeg; polarity++) {
        switch(polarity) {
        case polPos:
            busWriteRegister(busdev, HMC58X3_REG_CONFA, HMC_CONFA_DOR_15HZ | HMC_CONFA_POS_BIAS);   // Reg A DOR = 0x010 + MS1, MS0 set to pos bias
            break;
        case polNeg:
            busWriteRegister(busdev, HMC58X3_REG_CONFA, HMC_CONFA_DOR_15HZ | HMC_CONFA_NEG_BIAS);   // Reg A DOR = 0x010 + MS1, MS0 set to negative bias.
            break;
        }
        for (i = 0; i < 10; i++) {  // Collect 10 samples
            busWriteRegister(busdev, HMC58X3_REG_MODE, HMC_MODE_SINGLE);
            delay(20);
            hmc5883lRead(mag, magADC);       // Get the raw values in case the scales have already been changed.

            // Since the measurements are noisy, they should be averaged rather than taking the max.

            xyz_total[X] += ((polarity == polPos) ? 1 : -1) * magADC[X];
            xyz_total[Y] += ((polarity == polPos) ? 1 : -1) * magADC[Y];
            xyz_total[Z] += ((polarity == polPos) ? 1 : -1) * magADC[Z];

            // Detect saturation.
            if (-4096 >= MIN(magADC[X], MIN(magADC[Y], magADC[Z]))) {
                bret = false;
                break;              // Breaks out of the for loop.  No sense in continuing if we saturated.
            }
            LED1_TOGGLE;
        }
    }

    mag->magGain[X] = (int)(660.0f * HMC58X3_X_SELF_TEST_GAUSS * 2.0f * 10.0f * 256.0f) / xyz_total[X];
    mag->magGain[Y] = (int)(660.0f * HMC58X3_Y_SELF_TEST_GAUSS * 2.0f * 10.0f * 256.0f) / xyz_total[Y];
    mag->magGain[Z] = (int)(660.0f * HMC58X3_Z_SELF_TEST_GAUSS * 2.0f * 10.0f * 256.0f) / xyz_total[Z];

    // leave test mode

    busWriteRegister(busdev, HMC58X3_REG_CONFA, HMC_CONFA_8_SAMLES | HMC_CONFA_DOR_15HZ | HMC_CONFA_NORMAL);    // Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    busWriteRegister(busdev, HMC58X3_REG_CONFB, HMC_CONFB_GAIN_1_3GA);                                          // Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    busWriteRegister(busdev, HMC58X3_REG_MODE, HMC_MODE_CONTINOUS);                                             // Mode register             -- 000000 00    continuous Conversion Mode

    delay(100);

    if (!bret) {                // Something went wrong so get a best guess
        mag->magGain[X] = 256;
        mag->magGain[Y] = 256;
        mag->magGain[Z] = 256;
    }

    hmc5883lConfigureDataReadyInterruptHandling(mag);
    return true;
}

bool hmc5883lDetect(magDev_t* mag)
{
    busDevice_t *busdev = &mag->busdev;

    uint8_t sig = 0;

#ifdef USE_MAG_SPI_HMC5883
    if (busdev->bustype == BUSTYPE_SPI) {
        hmc5883SpiInit(&mag->busdev);
    }
#endif

#ifdef USE_MAG_HMC5883
    if (busdev->bustype == BUSTYPE_I2C && busdev->busdev_u.i2c.address == 0) {
        busdev->busdev_u.i2c.address = HMC5883_MAG_I2C_ADDRESS;
    }
#endif

    bool ack = busReadRegisterBuffer(&mag->busdev, HMC58X3_REG_IDA, &sig, 1);

    if (!ack || sig != HMC5883_DEVICE_ID) {
        return false;
    }

    mag->init = hmc5883lInit;
    mag->read = hmc5883lRead;

    return true;
}
#endif
