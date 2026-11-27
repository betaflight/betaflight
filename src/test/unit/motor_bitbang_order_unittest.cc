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

// motorWriteAll() with bitbang DShot: the order of the motor device calls in
// one update, and what that order means for a bitbang driver that defers an
// update while the previous frame still owns the line.
//
// The device here is a fake. It follows the rules of the STM32 bitbang driver
// (src/platform/STM32/dshot_bitbang.c): updateInit() decides whether this
// update is deferred, write(), telemetryWait() and updateComplete() do nothing
// for a deferred update, and decodeTelemetry() reads the capture buffer only
// when the update was not deferred. motorWriteAll() itself is the real code.

#include <stdint.h>
#include <string.h>
#include <string>
#include <vector>

extern "C" {
#include "platform.h"

#include "drivers/dshot.h"
#include "drivers/dshot_bitbang.h"
#include "drivers/motor.h"
#include "drivers/motor_impl.h"
#include "drivers/pwm_output.h"

#include "fc/rc_controls.h"

#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

PG_REGISTER(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 0);
PG_REGISTER(flight3DConfig_t, flight3DConfig, PG_MOTOR_3D_CONFIG, 0);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

const unsigned motorCount = 4;

std::vector<std::string> calls;

// The line and the capture buffer of the fake port.
bool lineBusy;            // the previous frame is still being sent or captured
int framesSent;           // frames that went out
int captureFrame;         // frame whose reply is in the capture buffer, 0 = none
bool captureComplete;     // the capture buffer holds a whole reply
std::vector<int> decoded; // frames whose reply was decoded, in order
int partialReads;         // decodes of a capture that was still being written

bool deferred;            // this update's decision, made in updateInit()

void fakeUpdateInit(void)
{
    calls.push_back("updateInit");
    deferred = lineBusy;
}

void fakeWrite(uint8_t index, float value)
{
    UNUSED(index);
    UNUSED(value);
    calls.push_back("write");
}

bool fakeTelemetryWait(void)
{
    calls.push_back("telemetryWait");
    return deferred;
}

void fakeUpdateComplete(void)
{
    calls.push_back("updateComplete");
    if (deferred) {
        return;
    }
    framesSent++;
    // The new frame goes out now; its reply is captured into the buffer after
    // the frame, so for the rest of this update the buffer still holds the
    // previous reply.
    lineBusy = true;
}

bool fakeDecodeTelemetry(void)
{
    calls.push_back("decodeTelemetry");
    if (deferred) {
        return true;
    }
    if (captureFrame != 0) {
        if (!captureComplete) {
            partialReads++;
        }
        decoded.push_back(captureFrame);
        captureFrame = 0;
    }
    return true;
}

bool fakeEnable(void)
{
    return true;
}

void fakeDisable(void)
{
}

const motorVTable_t fakeVTable = {
    .postInit = NULL,
    .convertExternalToMotor = NULL,
    .convertMotorToExternal = NULL,
    .enable = fakeEnable,
    .disable = fakeDisable,
    .isMotorEnabled = NULL,
    .telemetryWait = fakeTelemetryWait,
    .decodeTelemetry = fakeDecodeTelemetry,
    .updateInit = fakeUpdateInit,
    .write = fakeWrite,
    .writeInt = NULL,
    .updateComplete = fakeUpdateComplete,
    .shutdown = NULL,
    .isMotorIdle = NULL,
    .getMotorIO = NULL,
    .releaseMotorIO = NULL,
    .reinstateMotorIO = NULL,
    .requestTelemetry = NULL,
};

// The reply to the last frame starts arriving: the buffer is being written.
void replyStarts(void)
{
    captureFrame = framesSent;
    captureComplete = false;
    lineBusy = true;
}

// The reply has been captured in full and the line is free again.
void replyEnds(void)
{
    captureComplete = true;
    lineBusy = false;
}

void update(void)
{
    float values[motorCount] = { 0 };
    motorWriteAll(values);
}

class MotorBitbangOrder : public ::testing::Test {
protected:
    void SetUp() override
    {
        calls.clear();
        decoded.clear();
        lineBusy = false;
        framesSent = 0;
        captureFrame = 0;
        captureComplete = false;
        partialReads = 0;
        deferred = false;

        motorConfigMutable()->dev.motorProtocol = MOTOR_PROTOCOL_DSHOT600;
        float outputLow, outputHigh, disarm, deadbandHigh, deadbandLow;
        motorInitEndpoints(motorConfig(), 1.0f, &outputLow, &outputHigh, &disarm, &deadbandHigh, &deadbandLow);
        motorDevInit(motorCount);
        motorEnable();
        ASSERT_TRUE(motorIsEnabled());
    }
};

} // namespace

TEST_F(MotorBitbangOrder, UpdateDecidesBeforeItDecodes)
{
    update();

    const std::vector<std::string> expected = {
        "updateInit",
        "write", "write", "write", "write",
        "telemetryWait",
        "updateComplete",
        "decodeTelemetry",
    };
    EXPECT_EQ(expected, calls);
}

TEST_F(MotorBitbangOrder, LateUpdateDefersAndTheCaptureIsDecodedByTheNextUpdate)
{
    // Frame 1 goes out and its reply is captured in full.
    update();
    ASSERT_EQ(1, framesSent);
    replyStarts();
    replyEnds();

    // Frame 2 goes out and decodes the reply to frame 1.
    update();
    ASSERT_EQ(2, framesSent);
    EXPECT_EQ(std::vector<int>({ 1 }), decoded);

    // A late loop: the update arrives while the reply to frame 2 is still
    // being captured. Nothing is sent and the half-written buffer is not read.
    replyStarts();
    update();
    EXPECT_EQ(2, framesSent);
    EXPECT_EQ(std::vector<int>({ 1 }), decoded);

    // The capture ends. The next update sends frame 3 and decodes the reply to
    // frame 2: it was skipped once, not lost.
    replyEnds();
    update();
    EXPECT_EQ(3, framesSent);
    EXPECT_EQ(std::vector<int>({ 1, 2 }), decoded);
    EXPECT_EQ(0, partialReads);
}

TEST_F(MotorBitbangOrder, EveryReplyIsDecodedOnceOverManyLateUpdates)
{
    // Every other update arrives while the previous reply is still being
    // captured, the worst steady case for a loop that runs late.
    for (int i = 0; i < 20; i++) {
        update();
        replyStarts();
        update();       // late: deferred
        replyEnds();
    }
    update();

    ASSERT_EQ(21, framesSent);
    std::vector<int> expected;
    for (int frame = 1; frame <= 20; frame++) {
        expected.push_back(frame);
    }
    EXPECT_EQ(expected, decoded);
    EXPECT_EQ(0, partialReads);
}

// STUBS

extern "C" {

bool useDshotTelemetry = true;

bool featureIsEnabled(uint32_t mask)
{
    UNUSED(mask);
    return false;
}

bool isDshotBitbangActive(const motorDevConfig_t *motorDevConfig)
{
    UNUSED(motorDevConfig);
    return true;
}

bool dshotBitbangDevInit(motorDevice_t *device, const motorDevConfig_t *motorDevConfig)
{
    UNUSED(motorDevConfig);
    device->vTable = &fakeVTable;
    return true;
}

bool dshotPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorDevConfig)
{
    UNUSED(device);
    UNUSED(motorDevConfig);
    return false;
}

bool motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorDevConfig, uint16_t idlePulse)
{
    UNUSED(device);
    UNUSED(motorDevConfig);
    UNUSED(idlePulse);
    return false;
}

void dshotInitEndpoints(const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3dHigh, float *deadbandMotor3dLow)
{
    UNUSED(motorConfig);
    UNUSED(outputLimit);
    *outputLow = *outputHigh = *disarm = *deadbandMotor3dHigh = *deadbandMotor3dLow = 0;
}

void analogInitEndpoints(const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3dHigh, float *deadbandMotor3dLow)
{
    UNUSED(motorConfig);
    UNUSED(outputLimit);
    *outputLow = *outputHigh = *disarm = *deadbandMotor3dHigh = *deadbandMotor3dLow = 0;
}

timeMs_t millis(void)
{
    return 0;
}

void delayMicroseconds(timeUs_t us)
{
    UNUSED(us);
}

uint16_t getBatteryVoltage(void)
{
    return 0;
}

}
