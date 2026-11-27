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

#include <stdint.h>
#include <stdbool.h>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"
    #include "common/maths.h"
    #include "common/utils.h"
    #include "config/feature.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"
    #include "flight/failsafe.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"
    #include "rx/rx.h"

    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode = 0;
    uint8_t armingFlags = 0;
    uint16_t flightModeFlags = 0;

    PG_RESET_TEMPLATE(featureConfig_t, featureConfig,
        .enabledFeatures = 0
    );

    PG_REGISTER(flight3DConfig_t, flight3DConfig, PG_MOTOR_3D_CONFIG, 0);
    PG_REGISTER(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 0);

    void rxFrameCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static uint32_t nowUs;

static void sendFrame(int16_t rssiPercent, int16_t linkQualityPercent)
{
    static const uint16_t channels[SIMULATOR_MAX_RC_CHANNELS] = { 1500, 1500, 1000, 1500 };
    rxUpdateUdpLinkStatus(rssiPercent, linkQualityPercent);
    rxUpdateUdpChannels(channels, ARRAYLEN(channels));
}

// One RX check 10 ms after the last; RSSI smoothing is off in this test
static void check(void)
{
    nowUs += 10000;
    rxFrameCheck(nowUs, 10000);
    updateRSSI(nowUs);
}

static void start(uint8_t rssiChannel)
{
    featureConfigMutable()->enabledFeatures = FEATURE_RX_UDP;
    featureInit();
    rxConfigMutable()->rssi_channel = rssiChannel;
    rxConfigMutable()->rssi_scale = 100;
    rssiSource = RSSI_SOURCE_NONE;
    linkQualitySource = LQ_SOURCE_NONE;
    rxInit();
    nowUs = 1000000;
}

TEST(RxUdpUnittest, OldPacketsKeepTheOldSources)
{
    start(0);
    for (int i = 0; i < 10; i++) {
        sendFrame(-1, -1);
        check();
    }
    EXPECT_EQ(RSSI_SOURCE_NONE, rssiSource);
    EXPECT_EQ(LQ_SOURCE_NONE, linkQualitySource);
}

TEST(RxUdpUnittest, TakesRssiAndLinkQuality)
{
    start(0);
    sendFrame(80, 95);
    check();
    check();
    EXPECT_EQ(RSSI_SOURCE_RX_PROTOCOL, rssiSource);
    EXPECT_EQ(LQ_SOURCE_RX_PROTOCOL_UDP, linkQualitySource);
    EXPECT_EQ(80, getRssiPercent());
    EXPECT_EQ(95, rxGetLinkQualityPercent());

    sendFrame(30, 40);
    check();
    check();
    EXPECT_EQ(30, getRssiPercent());
    EXPECT_EQ(40, rxGetLinkQualityPercent());
}

TEST(RxUdpUnittest, RssiChannelWins)
{
    start(16);
    sendFrame(80, 95);
    check();
    EXPECT_EQ(RSSI_SOURCE_RX_CHANNEL, rssiSource);
    EXPECT_EQ(LQ_SOURCE_RX_PROTOCOL_UDP, linkQualitySource);
}

TEST(RxUdpUnittest, DropsToZeroWhenFramesStop)
{
    start(0);
    sendFrame(80, 95);
    check();
    for (int i = 0; i < 49; i++) {
        check();
    }
    EXPECT_EQ(80, getRssiPercent());
    EXPECT_EQ(95, rxGetLinkQualityPercent());
    for (int i = 0; i < 3; i++) {
        check();
    }
    EXPECT_EQ(0, getRssiPercent());
    EXPECT_EQ(0, rxGetLinkQualityPercent());
}

// STUBS

extern "C" {
    void failsafeOnValidDataFailed() {}
    void failsafeOnValidDataReceived() {}
    void failsafeOnRxSuspend(uint32_t ) {}
    void failsafeOnRxResume(void) {}
    bool failsafeIsActive(void) { return false; }
    bool failsafeIsReceivingRxData(void) { return true; }
    uint32_t failsafeFailurePeriodMs(void) { return 400; }

    uint32_t micros(void) { return nowUs; }
    uint32_t millis(void) { return nowUs / 1000; }

    bool isPPMDataBeingReceived(void) { return false; }
    bool isPWMDataBeingReceived(void) { return false; }
    void resetPPMDataReceivedState(void) {}
    bool rxMspFrameComplete(void) { return false; }

    void crsfRxInit(const rxConfig_t *, rxRuntimeState_t *) {}
    void ibusInit(const rxConfig_t *, rxRuntimeState_t *) {}
    void jetiExBusInit(const rxConfig_t *, rxRuntimeState_t *) {}
    void sbusInit(const rxConfig_t *, rxRuntimeState_t *) {}
    void spektrumInit(const rxConfig_t *, rxRuntimeState_t *) {}
    void sumdInit(const rxConfig_t *, rxRuntimeState_t *) {}
    void sumhInit(const rxConfig_t *, rxRuntimeState_t *) {}
    void xBusInit(const rxConfig_t *, rxRuntimeState_t *) {}
    void rxMspInit(const rxConfig_t *, rxRuntimeState_t *) {}
    void rxPwmInit(const rxConfig_t *, rxRuntimeState_t *) {}
    void setArmingDisabled(armingDisableFlags_e flag) { UNUSED(flag); }
    void unsetArmingDisabled(armingDisableFlags_e flag) { UNUSED(flag); }
    bool taskUpdateRxMainInProgress(void) { return false; }

    float pt1FilterGain(float, float) { return 1.0f; }
    void pt1FilterInit(pt1Filter_t *, float) {}
    void pt1FilterUpdateCutoff(pt1Filter_t *, float) {}
    float pt1FilterApply(pt1Filter_t *, float input) { return input; }

    void pinioBoxTaskControl(void) {}
}
