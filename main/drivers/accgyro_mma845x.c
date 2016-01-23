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

#include "system.h"
#include "gpio.h"
#include "bus_i2c.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mma845x.h"

// MMA8452QT, Standard address 0x1C
// ACC_INT2 routed to PA5

#define MMA8452_ADDRESS     0x1C

#define MMA8452_DEVICE_SIGNATURE    0x2A
#define MMA8451_DEVICE_SIGNATURE    0x1A

#define MMA8452_STATUS              0x00
#define MMA8452_OUT_X_MSB           0x01
#define MMA8452_WHO_AM_I            0x0D
#define MMA8452_XYZ_DATA_CFG        0x0E
#define MMA8452_HP_FILTER_CUTOFF    0x0F
#define MMA8452_CTRL_REG1           0x2A
#define MMA8452_CTRL_REG2           0x2B
#define MMA8452_CTRL_REG3           0x2C
#define MMA8452_CTRL_REG4           0x2D
#define MMA8452_CTRL_REG5           0x2E

#define MMA8452_FS_RANGE_8G         0x02
#define MMA8452_FS_RANGE_4G         0x01
#define MMA8452_FS_RANGE_2G         0x00

#define MMA8452_HPF_CUTOFF_LV1      0x00
#define MMA8452_HPF_CUTOFF_LV2      0x01
#define MMA8452_HPF_CUTOFF_LV3      0x02
#define MMA8452_HPF_CUTOFF_LV4      0x03

#define MMA8452_CTRL_REG2_B7_ST     0x80
#define MMA8452_CTRL_REG2_B6_RST    0x40
#define MMA8452_CTRL_REG2_B4_SMODS1 0x10
#define MMA8452_CTRL_REG2_B3_SMODS0 0x08
#define MMA8452_CTRL_REG2_B2_SLPE   0x04
#define MMA8452_CTRL_REG2_B1_MODS1  0x02
#define MMA8452_CTRL_REG2_B0_MODS0  0x01

#define MMA8452_CTRL_REG2_MODS_LP   0x03
#define MMA8452_CTRL_REG2_MODS_HR   0x02
#define MMA8452_CTRL_REG2_MODS_LNLP 0x01
#define MMA8452_CTRL_REG2_MODS_NOR  0x00

#define MMA8452_CTRL_REG3_IPOL          0x02
#define MMA8452_CTRL_REG4_INT_EN_DRDY   0x01

#define MMA8452_CTRL_REG1_LNOISE        0x04
#define MMA8452_CTRL_REG1_ACTIVE        0x01

static uint8_t device_id;

static void mma8452Init(void);
static bool mma8452Read(int16_t *accelData);

bool mma8452Detect(acc_t *acc)
{
    bool ack = false;
    uint8_t sig = 0;

    ack = i2cRead(MMA8452_ADDRESS, MMA8452_WHO_AM_I, 1, &sig);
    if (!ack || (sig != MMA8452_DEVICE_SIGNATURE && sig != MMA8451_DEVICE_SIGNATURE))
        return false;

    acc->init = mma8452Init;
    acc->read = mma8452Read;
    device_id = sig;
    return true;
}

static inline void mma8451ConfigureInterrupt(void)
{
#ifdef NAZE
    // PA5 - ACC_INT2 output on NAZE rev3/4 hardware
    // NAZE rev.5 hardware has PA5 (ADC1_IN5) on breakout pad on bottom of board
    // OLIMEXINO - The PA5 pin is wired up to LED1, if you need to use an mma8452 on an Olimexino use a different pin and provide support in code.

    gpio_config_t gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.pin = Pin_5;
    gpio.speed = Speed_2MHz;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(GPIOA, &gpio);
#endif

    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG3, MMA8452_CTRL_REG3_IPOL); // Interrupt polarity (active HIGH)
    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG4, MMA8452_CTRL_REG4_INT_EN_DRDY); // Enable DRDY interrupt (unused by this driver)
    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG5, 0); // DRDY routed to INT2
}

static void mma8452Init(void)
{

    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG1, 0); // Put device in standby to configure stuff
    i2cWrite(MMA8452_ADDRESS, MMA8452_XYZ_DATA_CFG, MMA8452_FS_RANGE_8G);
    i2cWrite(MMA8452_ADDRESS, MMA8452_HP_FILTER_CUTOFF, MMA8452_HPF_CUTOFF_LV4);
    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG2, MMA8452_CTRL_REG2_MODS_HR | MMA8452_CTRL_REG2_MODS_HR << 3); // High resolution measurement in both sleep and active modes

    mma8451ConfigureInterrupt();

    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG1, MMA8452_CTRL_REG1_LNOISE | MMA8452_CTRL_REG1_ACTIVE); // Turn on measurements, low noise at max scale mode, Data Rate 800Hz. LNoise mode makes range +-4G.

    acc_1G = 256;
}

static bool mma8452Read(int16_t *accelData)
{
    uint8_t buf[6];

    if (!i2cRead(MMA8452_ADDRESS, MMA8452_OUT_X_MSB, 6, buf)) {
        return false;
    }

    accelData[0] = ((int16_t)((buf[0] << 8) | buf[1]) >> 2) / 4;
    accelData[1] = ((int16_t)((buf[2] << 8) | buf[3]) >> 2) / 4;
    accelData[2] = ((int16_t)((buf[4] << 8) | buf[5]) >> 2) / 4;

    return true;
}
