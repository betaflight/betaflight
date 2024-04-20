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

#include "platform.h"

#include "drivers/io.h"
#include "drivers/time.h"

#include "bus_i2c_utils.h"

// Clock period in us during unstick transfer
#define UNSTICK_CLK_US 10  // 100Khz
// Allow 500us for clock stretch to complete during unstick
#define UNSTICK_CLK_STRETCH (500 / UNSTICK_CLK_US)

// wait for SCL to return high (clock stretching)
static bool i2cUnstick_waitStretch(IO_t scl, int timeout)
{
    bool sclVal;
    while (!(sclVal = IORead(scl)) && timeout) {
        delayMicroseconds(UNSTICK_CLK_US);
        timeout--;
    }
    return sclVal;
}

// generate clock pulses + STOP on I2C bus
// this should get all devices into idle state
// return true if bus seems to be idle (both SCL and SDA are high)
bool i2cUnstick(IO_t scl, IO_t sda)
{
    // output value first to prevent glitch after switching direction
    IOHi(scl);
    IOHi(sda);

    // bus was probably in I2C (alternate function) mode
    IOConfigGPIO(scl, IOCFG_OUT_OD);
    IOConfigGPIO(sda, IOCFG_OUT_OD);

    // Clock out, with SDA high:
    // 7 data bits, 1 READ bit, 1 cycle for the ACK
    for (int i = 0; i < (7 + 1 + 1); i++) {
        // Wait for any clock stretching to finish
        i2cUnstick_waitStretch(scl, UNSTICK_CLK_STRETCH);
        // Pull low
        IOLo(scl); // Set bus low
        delayMicroseconds(UNSTICK_CLK_US / 2);
        IOHi(scl); // Set bus high
        delayMicroseconds(UNSTICK_CLK_US / 2);
    }
    // slave may be still stretching after last pulse
    i2cUnstick_waitStretch(scl, UNSTICK_CLK_STRETCH);

    // Generate a stop condition in case there was none
    // SCL low pulse to switch SDA low
    IOLo(scl);
    delayMicroseconds(UNSTICK_CLK_US / 2);
    IOLo(sda);
    delayMicroseconds(UNSTICK_CLK_US / 2);
    IOHi(scl);
    delayMicroseconds(UNSTICK_CLK_US / 2);
     // SDA rising edge = STOP
    IOHi(sda);
    // check that both SCL and SDA are high
    delayMicroseconds(UNSTICK_CLK_US / 2);  // time for SDA to return high
    bool ok = IORead(scl) && IORead(sda);
    // I2C ping are left in GPIO mode
    return ok;
}
