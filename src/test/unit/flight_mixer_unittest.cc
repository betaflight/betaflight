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

extern "C" {
    #include "platform.h"
    #include "flight/mixer.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(MixerInitTest, LoadsQuadXMotorMix)
{
    motorMixer_t customMixers[MAX_SUPPORTED_MOTORS] = {};

    // mixerLoadMix takes a zero-based index into the one-based mixer table.
    mixerLoadMix(MIXER_QUADX - 1, customMixers);

    const motorMixer_t expected[] = {
        { 1.0f, -1.0f,  1.0f, -1.0f },
        { 1.0f, -1.0f, -1.0f,  1.0f },
        { 1.0f,  1.0f,  1.0f,  1.0f },
        { 1.0f,  1.0f, -1.0f, -1.0f },
    };

    for (int i = 0; i < QUAD_MOTOR_COUNT; i++) {
        EXPECT_FLOAT_EQ(expected[i].throttle, customMixers[i].throttle);
        EXPECT_FLOAT_EQ(expected[i].roll, customMixers[i].roll);
        EXPECT_FLOAT_EQ(expected[i].pitch, customMixers[i].pitch);
        EXPECT_FLOAT_EQ(expected[i].yaw, customMixers[i].yaw);
    }
}

TEST(MixerInitTest, ClearsMotorEntriesForMixerWithoutPreset)
{
    motorMixer_t customMixers[MAX_SUPPORTED_MOTORS];
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        customMixers[i] = { 1.0f, 2.0f, 3.0f, 4.0f };
    }

    mixerLoadMix(MIXER_CUSTOM - 1, customMixers);

    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        EXPECT_FLOAT_EQ(0.0f, customMixers[i].throttle);
    }
}

TEST(MixerInitTest, RecognizesFixedWingModes)
{
    EXPECT_TRUE(mixerModeIsFixedWing(MIXER_FLYING_WING));
    EXPECT_TRUE(mixerModeIsFixedWing(MIXER_AIRPLANE));
    EXPECT_TRUE(mixerModeIsFixedWing(MIXER_CUSTOM_AIRPLANE));
    EXPECT_FALSE(mixerModeIsFixedWing(MIXER_QUADX));
}
