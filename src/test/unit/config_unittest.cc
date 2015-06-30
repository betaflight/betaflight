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

#include <stdint.h>
#include <stddef.h>

extern "C" {
    #include "platform.h"
    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/color.h"
    #include "flight/pid.h"
    #include "drivers/sensor.h"
    #include "drivers/timer.h"
    #include "drivers/accgyro.h"
    #include "drivers/pwm_rx.h"
    #include "drivers/serial.h"
    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/gyro.h"
    #include "sensors/battery.h"
    #include "sensors/boardalignment.h"
    #include "io/rc_controls.h"
    #include "io/escservo.h"
    #include "io/gimbal.h"
    #include "io/gps.h"
    #include "io/serial.h"
    #include "io/ledstrip.h"
    #include "flight/mixer.h"
    #include "flight/imu.h"
    #include "flight/navigation.h"
    #include "flight/failsafe.h"
    #include "telemetry/telemetry.h"
    #include "config/config.h"
    #include "config/config_profile.h"
    #include "config/config_master.h"
    #include "config/parameter_group.h"
    #include "platform.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

pgRegistry_t __pg_registry[50] =
{
    {
        .base = &masterConfig,
        .size = sizeof(masterConfig),
        .pgn = 0,
        .format = 0,
    },
    { nullptr, 0, 0, 0 },
};

TEST(configTest, resetEEPROM)
{
    ledStripInit(masterConfig.ledConfigs, masterConfig.colors);
    resetEEPROM();
}

TEST(configTest, modify)
{
    ledStripInit(masterConfig.ledConfigs, masterConfig.colors);
    resetEEPROM();

    masterConfig.looptime = 123;
    writeEEPROM();

    EXPECT_EQ(123, masterConfig.looptime);

    masterConfig.looptime = 456;
    readEEPROM();
    EXPECT_EQ(123, masterConfig.looptime);
}
