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
#include <stdbool.h>

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    #include "drivers/resource.h"
    #include "drivers/io_def.h"
    #include "drivers/io.h"
    #include "common/utils.h"
    #include "io/armhook.h"
}

typedef struct testData_s {
    uint8_t configuredIOs[2];
} testData_t;

static testData_t testData;

TEST(ArmHookTest, InitializeWithoutIOSetup)
{
    memset(&testData, 0, sizeof(testData));

    armHookInit();

    EXPECT_EQ(false, armHookIsEnabled());
}

TEST(ArmHookTest, Initialize)
{
    memset(&testData, 0, sizeof(testData));
    testData.configuredIOs[0] = IO_TAG(SBFC_SWITCH_PIN);
    testData.configuredIOs[1] = IO_TAG(SBFC_CONNECTION_STATE_PIN);
    armHookInit();

    EXPECT_EQ(true, armHookIsEnabled());
}

TEST(ArmHookTest, Initialize2)
{
    memset(&testData, 0, sizeof(testData));
    testData.configuredIOs[0] = IO_TAG(SBFC_SWITCH_PIN);
    armHookInit();

    EXPECT_EQ(false, armHookIsEnabled());
}

TEST(ArmHookTest, Process)
{
    memset(&testData, 0, sizeof(testData));
    testData.configuredIOs[0] = IO_TAG(SBFC_SWITCH_PIN);
    testData.configuredIOs[1] = IO_TAG(SBFC_CONNECTION_STATE_PIN);
    armHookInit();

    EXPECT_EQ(true, armHookIsEnabled());

    armHookProcess(0);
}

// STUBS
extern "C" {
    void IOConfigGPIO(IO_t io, ioConfig_t cfg) { UNUSED(io); UNUSED(cfg); }
    IO_t IOGetByTag(ioTag_t tag)
    {
        for (unsigned i = 0; i < ARRAYLEN(testData.configuredIOs); i++) {
            if (testData.configuredIOs[i] == 0) {
                continue;
            }

            if (tag == testData.configuredIOs[i]) {
                return (IO_t)1;
            }
        }

        return NULL;
    }

    void IOInit(IO_t io, resourceOwner_e owner, uint8_t index) { UNUSED(io); UNUSED(owner); UNUSED(index); }
    bool IORead(IO_t io) { UNUSED(io); return false; }
    void IOWrite(IO_t io, bool hi) { UNUSED(io); UNUSED(hi); }
    uint8_t armingFlags = 0;
}
