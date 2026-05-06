/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_ACCGYRO_SCS3304

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_scs3304.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

// 10 MHz max SPI frequency
#define SCS3304_MAX_SPI_CLK_HZ 10000000

#define SCS3304_CHIP_ID 0x6B

// equivalent to 70 mdps/LSB, as specified in SCS3304 datasheet section 4.1, symbol G_So
#define SCS3304_GYRO_SCALE_2000DPS 0.070f

// SCS3304 register configuration values
typedef enum {
    SCS3304_VAL_INT1_CTRL = 0x02,             // enable gyro data ready interrupt pin 1
    SCS3304_VAL_INT2_CTRL = 0x02,             // enable gyro data ready interrupt pin 2
    SCS3304_VAL_CTRL1_XL_ODR833 = 0x07,       // accelerometer 833hz output data rate (gyro/8)
    SCS3304_VAL_CTRL1_XL_ODR1667 = 0x08,      // accelerometer 1666hz output data rate (gyro/4)
    SCS3304_VAL_CTRL1_XL_ODR3332 = 0x09,      // accelerometer 3332hz output data rate (gyro/2)
    SCS3304_VAL_CTRL1_XL_ODR3333 = 0x0A,      // accelerometer 6664hz output data rate (gyro/1)
    SCS3304_VAL_CTRL1_XL_8G = 0x03,           // accelerometer 8G scale
    SCS3304_VAL_CTRL1_XL_16G = 0x01,          // accelerometer 16G scale
    SCS3304_VAL_CTRL1_XL_LPF1 = 0x00,         // accelerometer output from LPF1
    SCS3304_VAL_CTRL1_XL_LPF2 = 0x01,         // accelerometer output from LPF2
    SCS3304_VAL_CTRL2_G_ODR6664 = 0x0A,       // gyro 6664hz output data rate
    SCS3304_VAL_CTRL2_G_2000DPS = 0x03,       // gyro 2000dps scale
    SCS3304_VAL_CTRL3_C_BDU = BIT(6),         // (bit 6) output registers are not updated until MSB and LSB have been read (prevents MSB from being updated while burst reading LSB/MSB)
    SCS3304_VAL_CTRL3_C_H_LACTIVE = 0,        // (bit 5) interrupt pins active high
    SCS3304_VAL_CTRL3_C_PP_OD = 0,            // (bit 4) interrupt pins push/pull
    SCS3304_VAL_CTRL3_C_SIM = 0,              // (bit 3) SPI 4-wire interface mode
    SCS3304_VAL_CTRL3_C_IF_INC = BIT(2),      // (bit 2) auto-increment address for burst reads
    SCS3304_VAL_CTRL4_C_I2C_DISABLE = BIT(2), // (bit 2) disable I2C interface
    SCS3304_VAL_CTRL4_C_LPF1_SEL_G = BIT(1),  // (bit 1) enable gyro LPF1
    SCS3301_VAL_CTRL6_C_BIT4 = 0,             // (bit 4) recommend
    SCS3301_VAL_CTRL6_C_FTYPE_297HZ = 0x00,   // (bits 2:0) gyro LPF1 cutoff 297hz
    SCS3301_VAL_CTRL6_C_FTYPE_223HZ = 0x01,   // (bits 2:0) gyro LPF1 cutoff 223hz
    SCS3301_VAL_CTRL6_C_FTYPE_154HZ = 0x02,   // (bits 2:0) gyro LPF1 cutoff 154hz
    SCS3301_VAL_CTRL6_C_FTYPE_470HZ = 0x03,
    SCS3304_VAL_CTRL9_XL_DEVICE_CONF = BIT(1),// (bit 1) recommend
} scs3304ConfigValues_e;

// SCS3304 register configuration bit masks
typedef enum {
    SCS3304_MASK_CTRL3_C = 0x7C,         // 0b01111100
    SCS3304_MASK_CTRL3_C_RESET = BIT(0), // 0b00000001
    SCS3304_MASK_CTRL4_C = 0x06,         // 0b00000110
    SCS3304_MASK_CTRL6_C = 0x17,         // 0b00010111  
    SCS3304_MASK_CTRL9_XL = 0x02,        // 0b00000010
} scs3304ConfigMasks_e;

uint8_t scs3304Detect(const extDevice_t *dev)
{
    uint8_t chipID = 0;

    if (busReadRegisterBuffer(dev, SCS3304_REG_WHO_AM_I, &chipID, 1)) {
        if (chipID == SCS3304_CHIP_ID) {
            return SCS3304_SPI;
        }
    }

    return MPU_NONE;
}

static void scs3304WriteRegister(const extDevice_t *dev, scs3304Register_e registerID, uint8_t value, unsigned delayMs)
{
    busWriteRegister(dev, registerID, value);
    if (delayMs) {
        delay(delayMs);
    }
}

static void scs3304WriteRegisterBits(const extDevice_t *dev, scs3304Register_e registerID, scs3304ConfigMasks_e mask, uint8_t value, unsigned delayMs)
{
    uint8_t newValue;
    if (busReadRegisterBuffer(dev, registerID, &newValue, 1)) {
        delayMicroseconds(2);
        newValue = (newValue & ~mask) | value;
        scs3304WriteRegister(dev, registerID, newValue, delayMs);
    }
}

static void scs3304Config(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // Reset the device (wait 100ms before continuing config)
    scs3304WriteRegisterBits(dev, SCS3304_REG_CTRL3_C, SCS3304_MASK_CTRL3_C_RESET, BIT(0), 100);

    // Configure interrupt pin 1 for gyro data ready only
    scs3304WriteRegister(dev, SCS3304_REG_INT1_CTRL, SCS3304_VAL_INT1_CTRL, 1);

    // Disable interrupt pin 2
    scs3304WriteRegister(dev, SCS3304_REG_INT2_CTRL, SCS3304_VAL_INT2_CTRL, 1);

    // Configure the accelerometer
    // 833hz ODR, 16G scale, use LPF1 output
    scs3304WriteRegister(dev, SCS3304_REG_CTRL1_XL, (SCS3304_VAL_CTRL1_XL_ODR833 << 4) | (SCS3304_VAL_CTRL1_XL_16G << 2) | (SCS3304_VAL_CTRL1_XL_LPF1 << 1), 1);

    // Configure the gyro
    // 6664hz ODR, 2000dps scale
    scs3304WriteRegister(dev, SCS3304_REG_CTRL2_G, (SCS3304_VAL_CTRL2_G_ODR6664 << 4) | (SCS3304_VAL_CTRL2_G_2000DPS << 2), 1);

    // Configure control register 3
    // latch LSB/MSB during reads; set interrupt pins active high; set interrupt pins push/pull; set 4-wire SPI; enable auto-increment burst reads
    scs3304WriteRegisterBits(dev, SCS3304_REG_CTRL3_C, SCS3304_MASK_CTRL3_C, (SCS3304_VAL_CTRL3_C_BDU | SCS3304_VAL_CTRL3_C_H_LACTIVE | SCS3304_VAL_CTRL3_C_PP_OD | SCS3304_VAL_CTRL3_C_SIM | SCS3304_VAL_CTRL3_C_IF_INC), 1);

    // Configure control register 4
    // disable I2C interface; enable gyro LPF1
    scs3304WriteRegisterBits(dev, SCS3304_REG_CTRL4_C, SCS3304_MASK_CTRL4_C, (SCS3304_VAL_CTRL4_C_I2C_DISABLE | SCS3304_VAL_CTRL4_C_LPF1_SEL_G), 1);

    // Configure control register 6
    // set gyro LPF1 cutoff to 297hz
    scs3304WriteRegisterBits(dev, SCS3304_REG_CTRL6_C, SCS3304_MASK_CTRL6_C, (SCS3301_VAL_CTRL6_C_BIT4 | SCS3301_VAL_CTRL6_C_FTYPE_297HZ), 1);

    // Configure control register 9
    // device conf
    scs3304WriteRegisterBits(dev, SCS3304_REG_CTRL9_XL, SCS3304_MASK_CTRL9_XL, SCS3304_VAL_CTRL9_XL_DEVICE_CONF, 1);
}

static void scs3304IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, scs3304ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}

static void scs3304SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    scs3304Config(gyro);

    scs3304IntExtiInit(gyro);

    spiSetClkDivisor(dev, spiCalculateDivider(SCS3304_MAX_SPI_CLK_HZ));
}

static void scs3304SpiAccInit(accDev_t *acc)
{
    // sensor is configured during gyro init
    acc->acc_1G = 512 * 4;   // 16G sensor scale
}

bool scs3304SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != SCS3304_SPI) {
        return false;
    }

    acc->initFn = scs3304SpiAccInit;
    acc->readFn = scs3304AccRead;

    return true;
}

bool scs3304SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != SCS3304_SPI) {
        return false;
    }

    gyro->initFn = scs3304SpiGyroInit;
    gyro->readFn = scs3304GyroRead;
    gyro->scale = SCS3304_GYRO_SCALE_2000DPS;

    return true;
}
#endif // USE_ACCGYRO_SCS3304
