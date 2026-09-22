/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3
 * of the License, or (at your option) any later version.
 */

extern "C" {
#include "platform.h"

#include "fc/runtime_config.h"
#include "msp/msp_reboot.h"

void beeperConfirmationBeeps(uint8_t) {}
}

#include "gtest/gtest.h"

TEST(MspRebootTest, AllowsRebootWhileDisarmed)
{
    armingFlags = 0;

    EXPECT_TRUE(mspRebootIsAllowed());
}

TEST(MspRebootTest, RejectsRebootWhileArmed)
{
    armingFlags = ARMED;

    EXPECT_FALSE(mspRebootIsAllowed());
}
