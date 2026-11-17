/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if ENABLE_MAG_IST8308

#include "common/axis.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"
#include "compass_ist8308.h"

/*
 * iSentek IST8308 3-axis AMR magnetometer (I2C only, up to 400 kHz).
 *
 * Slave address select:
 *         CAD1  |  CAD0  |  Address
 *    ------------------------------
 *         VSS   |  VSS   |  0x0C
 *         VSS   |  VDD   |  0x0D
 *         VDD   |  VSS   |  0x0E
 *         VDD   |  VDD   |  0x0F
 * If CAD1 and CAD0 are floating, the I2C address is 0x0C.
 *
 * Unlike the IST8310 the measurement registers start at 0x10 (STAT) and the
 * mode is selected in CNTL2 (0x31). The device is run in continuous mode;
 * STAT and the six data bytes are fetched in a single burst so DRDY and the
 * sample are always consistent.
 */

#define IST8308_MAG_I2C_ADDRESS         0x0C

#define IST8308_REG_WAI                 0x00
#define IST8308_WAI_VALID               0x08

#define IST8308_REG_STAT                0x10
#define IST8308_STAT_DRDY               0x01
#define IST8308_STAT_DOR                0x02    // data overrun

#define IST8308_REG_DATA_XL             0x11    // X/Y/Z, little endian, 0x11..0x16

#define IST8308_REG_CNTL1               0x30    // noise suppression filter
#define IST8308_REG_CNTL2               0x31    // operating mode
#define IST8308_CNTL2_STANDBY           0x00
#define IST8308_CNTL2_SINGLE            0x01
#define IST8308_CNTL2_CONT_10HZ         0x02
#define IST8308_CNTL2_CONT_20HZ         0x04
#define IST8308_CNTL2_CONT_50HZ         0x06
#define IST8308_CNTL2_CONT_100HZ        0x08
#define IST8308_CNTL2_CONT_200HZ        0x0A

#define IST8308_REG_CNTL3               0x32
#define IST8308_CNTL3_SRST              0x01    // soft reset, self clearing
#define IST8308_CNTL3_DRDY_POL_HIGH     0x04
#define IST8308_CNTL3_DRDY_EN           0x08

#define IST8308_REG_CNTL4               0x34    // dynamic range
#define IST8308_CNTL4_RANGE_500UT       0x00    // 6.6 LSB/uT, 0.15 uT/LSB
#define IST8308_CNTL4_RANGE_200UT       0x01    // 13.2 LSB/uT, 0.075 uT/LSB

#define IST8308_REG_OSRCNTL             0x41    // over sampling ratio
#define IST8308_OSR_XZ_16               0x04
#define IST8308_OSR_Y_16                (0x04 << 3)

#define IST8308_SOFT_RESET_DELAY_MS     20      // POR max is 50ms after power-up; soft reset completes well within 20ms

#define IST8308_READ_LEN                7       // STAT + 3 x int16

static bool ist8308Init(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;

    busDeviceRegister(dev);

    bool ack = busWriteRegister(dev, IST8308_REG_CNTL3, IST8308_CNTL3_SRST);
    delay(IST8308_SOFT_RESET_DELAY_MS);

    // OSR 16 on all axes; ODR up to 200Hz requires OSR <= 8, 100Hz is fine with 16
    ack = ack && busWriteRegister(dev, IST8308_REG_OSRCNTL, IST8308_OSR_XZ_16 | IST8308_OSR_Y_16);
    // +/-500uT range so that motor/battery fields close to the sensor don't saturate
    ack = ack && busWriteRegister(dev, IST8308_REG_CNTL4, IST8308_CNTL4_RANGE_500UT);
    ack = ack && busWriteRegister(dev, IST8308_REG_CNTL2, IST8308_CNTL2_CONT_100HZ);

    magDev->magOdrHz = 100;

    return ack;
}

static bool ist8308Read(magDev_t *magDev, int16_t *magData)
{
    extDevice_t *dev = &magDev->dev;

    static uint8_t buf[IST8308_READ_LEN];
    static bool pendingRead = true;

    if (pendingRead) {
        if (busReadRegisterBufferStart(dev, IST8308_REG_STAT, buf, sizeof(buf))) {
            pendingRead = false;
        }
        return false;
    }

    bool error = false;
    if (busBusy(dev, &error)) {
        return false;
    }

    // Transfer has completed, issue a new one on the next call
    pendingRead = true;

    if (error) {
        // Transfer failed (e.g. NACK), buf holds no new sample
        return false;
    }

    if (!(buf[0] & IST8308_STAT_DRDY)) {
        // No new sample since the last read
        return false;
    }

    const int16_t x = (int16_t)(buf[2] << 8 | buf[1]);
    const int16_t y = (int16_t)(buf[4] << 8 | buf[3]);
    const int16_t z = (int16_t)(buf[6] << 8 | buf[5]);

    // 0.15uT/LSB = 1.5mG/LSB at +/-500uT range; max |raw| ~3330 so fits int16 after scaling.
    // Datasheet axes (X+, Y+, Z+ up) form a left-handed frame; invert Y to make it
    // right-handed with Z up, matching the IST8310 driver orientation.
    magData[X] =  (x * 3) / 2;
    magData[Y] = -(y * 3) / 2;
    magData[Z] =  (z * 3) / 2;

    return true;
}

bool ist8308Detect(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;

    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = IST8308_MAG_I2C_ADDRESS;
    }

    uint8_t wai = 0;
    const bool ack = busReadRegisterBuffer(dev, IST8308_REG_WAI, &wai, 1);

    if (ack && wai == IST8308_WAI_VALID) {
        magDev->init = ist8308Init;
        magDev->read = ist8308Read;
        return true;
    }

    return false;
}

#endif // ENABLE_MAG_IST8308
