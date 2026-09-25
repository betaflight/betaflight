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

#include <csetjmp>

extern "C" {
#include "platform.h"

#include "drivers/system.h"
#include "fc/runtime_config.h"
#include "msp/msp_reboot.h"

void beeperConfirmationBeeps(uint8_t) {}
void testMspDirectReboot(void);
void testMspDelayedReboot(void);

static std::jmp_buf resetJump;
static unsigned shutdownCount;
static unsigned resetCount;
void motorShutdown(void) { shutdownCount++; }
void systemReset(void) { resetCount++; std::longjmp(resetJump, 1); }
void systemResetToBootloader(bootloaderRequestType_e) { resetCount++; std::longjmp(resetJump, 1); }
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

TEST(MspRebootTest, DirectSerialCallbackDoesNotShutDownArmedMotors)
{
    armingFlags = ARMED;
    shutdownCount = resetCount = 0;
    if (setjmp(resetJump) == 0) {
        testMspDirectReboot();
    }
    EXPECT_EQ(0u, shutdownCount);
    EXPECT_EQ(0u, resetCount);
}

TEST(MspRebootTest, DelayedDispatchDoesNotShutDownArmedMotors)
{
    armingFlags = ARMED;
    shutdownCount = resetCount = 0;
    if (setjmp(resetJump) == 0) {
        testMspDelayedReboot();
    }
    EXPECT_EQ(0u, shutdownCount);
    EXPECT_EQ(0u, resetCount);
}

TEST(MspRebootTest, DirectSerialCallbackRebootsWhileDisarmed)
{
    armingFlags = 0;
    shutdownCount = resetCount = 0;
    if (setjmp(resetJump) == 0) {
        testMspDirectReboot();
    }
    EXPECT_EQ(1u, shutdownCount);
    EXPECT_EQ(1u, resetCount);
}
