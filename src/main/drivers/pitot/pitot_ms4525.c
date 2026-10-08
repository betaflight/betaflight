/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_PITOT_MS4525

#include "drivers/bus_i2c.h"
#include "drivers/pitot/pitot_ms4525.h"

// MS4525DO: register-less device, a plain 4-byte read returns
// [status:2 | pressure:14][temperature:11 | unused:5]. The -DS5AI001DP part
// (bidirectional +/-1 psi, 10-90% transfer function) is the ASPD-4525 module.
#define MS4525_PSI_RANGE     2.0f      // -1 .. +1 psi
#define MS4525_PSI_MIN       (-1.0f)
#define MS4525_OUT_MIN       1638.3f   // 0.10 * 16383
#define MS4525_OUT_SPAN      13106.4f  // 0.80 * 16383
#define PSI_TO_PASCAL        6894.757f

static bool ms4525Read(pitotDev_t *pitot, float *diffPressurePa, float *temperatureK)
{
    uint8_t buf[4];
    const i2cDevice_e device = pitot->dev.bus->busType_u.i2c.device;
    const uint8_t address = pitot->dev.busType_u.i2c.address;

    if (!i2cRead(device, address, 0xFF, sizeof(buf), buf)) {
        return false;
    }

    const uint8_t status = buf[0] >> 6;
    if (status == 2 || status == 3) {   // 2 = stale data, 3 = fault
        return false;
    }

    const uint16_t pressureRaw = (uint16_t)(((buf[0] & 0x3F) << 8) | buf[1]);
    const uint16_t temperatureRaw = (uint16_t)((buf[2] << 3) | (buf[3] >> 5));

    const float psi = (pressureRaw - MS4525_OUT_MIN) / MS4525_OUT_SPAN * MS4525_PSI_RANGE + MS4525_PSI_MIN;
    *diffPressurePa = psi * PSI_TO_PASCAL;
    *temperatureK = (temperatureRaw * (200.0f / 2047.0f)) - 50.0f + 273.15f;

    return true;
}

bool ms4525Detect(pitotDev_t *pitot)
{
    extDevice_t *dev = &pitot->dev;

    if (dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = MS4525_I2C_ADDR;
    }

    uint8_t buf[4];
    if (!i2cRead(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, 0xFF, sizeof(buf), buf)) {
        return false;
    }

    busDeviceRegister(dev);
    pitot->read = ms4525Read;

    return true;
}

#endif // USE_PITOT_MS4525
