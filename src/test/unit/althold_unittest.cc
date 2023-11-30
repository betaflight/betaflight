#include "unittest_macros.h"
#include "gtest/gtest.h"


extern "C" {

    #include "sensors/acceleration.h"
    FAST_DATA_ZERO_INIT acc_t acc;

    #include "build/debug.h"
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;

    #include "fc/runtime_config.h"
    uint8_t armingFlags = 0;
    uint8_t stateFlags = 0;
    uint16_t flightModeFlags = 0;

    #include "flight/imu.h"
    void imuTransformVectorBodyToEarth(t_fp_vector * v)
    {
        (void)v;
    }

    #include "drivers/time.h"
    uint32_t millisRW;
    uint32_t millis() {
        return millisRW;
    }

    #include "sensors/barometer.h"
    baro_t baro;

    #include "flight/position.h"
    int32_t getEstimatedAltitudeCm(void)
    {
        return 0;
    }
    int16_t getEstimatedVario(void)
    {
        return 0;
    }

    // other
    float getRcDeflection(int)
    {
        return 0;
    }

    bool baroIsCalibrated(void)
    {
        return 1;
    }

    bool sensors(uint32_t)
    {
        return 1;
    }

    bool failsafeIsActive(void)
    {
        return 0;
    }

    #include "flight/alt_hold.h"

    extern altHoldState_s altHoldState;

    void altHoldReset(altHoldState_s* altHoldState);
    void altHoldProcessTransitions(altHoldState_s* altHoldState);
    void altHoldInit(altHoldState_s* altHoldState);
    void altHoldUpdate(altHoldState_s* altHoldState);
}


TEST(AltholdUnittest, altHoldTransitionsTest)
{
    altHoldInit(&altHoldState);
    EXPECT_EQ(altHoldState.altHoldEnabled, false);

    flightModeFlags |= ALTHOLD_MODE;
    millisRW = 42;
    altHoldUpdate(&altHoldState);
    EXPECT_EQ(altHoldState.altHoldEnabled, true);
    EXPECT_EQ(altHoldState.enterTime, 42);
    EXPECT_EQ(altHoldState.exitTime, 0);
    EXPECT_TRUE(getAltHoldThrottleFactor(0) < 0.01f);

    millisRW = 56;
    flightModeFlags ^= ALTHOLD_MODE;
    altHoldUpdate(&altHoldState);
    EXPECT_EQ(altHoldState.altHoldEnabled, false);
    EXPECT_EQ(altHoldState.exitTime, 56);


    flightModeFlags |= ALTHOLD_MODE;
    millisRW = 64;
    altHoldUpdate(&altHoldState);
    EXPECT_EQ(altHoldState.altHoldEnabled, true);
    EXPECT_EQ(altHoldState.enterTime, 64);
    EXPECT_EQ(altHoldState.exitTime, 0);
    EXPECT_TRUE(getAltHoldThrottleFactor(0) < 0.01f);

    millisRW = 64 + 0.3f * ALTHOLD_ENTER_PERIOD;
    altHoldUpdate(&altHoldState);
    EXPECT_TRUE(ABS(getAltHoldThrottleFactor(0) - 0.3f) < 0.01f);

    millisRW = 64 + 0.9f * ALTHOLD_ENTER_PERIOD;
    altHoldUpdate(&altHoldState);
    EXPECT_TRUE(ABS(getAltHoldThrottleFactor(0) - 0.9f) < 0.01f);

    millisRW = 64 + 1.4f * ALTHOLD_ENTER_PERIOD;
    altHoldUpdate(&altHoldState);
    EXPECT_TRUE(ABS(getAltHoldThrottleFactor(0) - 1.0f) < 0.01f);


    millisRW = 10042;
    flightModeFlags ^= ALTHOLD_MODE;
    altHoldUpdate(&altHoldState);
    EXPECT_TRUE(ABS(getAltHoldThrottleFactor(1.5f) - 1.0f) < 0.01f);

    millisRW = 10042 + 0.3f * ALTHOLD_MAX_EXIT_PERIOD;
    altHoldUpdate(&altHoldState);
    EXPECT_TRUE(ABS(getAltHoldThrottleFactor(1.5f) - 0.7f) < 0.01f);

    millisRW = 10042 + 0.5f * ALTHOLD_MAX_EXIT_PERIOD;
    altHoldState.throttle = 0.5f;
    getAltHoldThrottleFactor(0.5f);
    altHoldUpdate(&altHoldState);
    EXPECT_TRUE(ABS(getAltHoldThrottleFactor(0.5f)) < 0.01f);
}

TEST(AltholdUnittest, altHoldTransitionsTestUnfinishedExitEnter)
{
    altHoldInit(&altHoldState);
    EXPECT_EQ(altHoldState.altHoldEnabled, false);

    flightModeFlags |= ALTHOLD_MODE;
    millisRW = 42;
    altHoldUpdate(&altHoldState);
    EXPECT_EQ(altHoldState.altHoldEnabled, true);
    EXPECT_EQ(altHoldState.enterTime, 42);
    EXPECT_EQ(altHoldState.exitTime, 0);
    EXPECT_TRUE(getAltHoldThrottleFactor(0) < 0.01f);

    millisRW += ALTHOLD_ENTER_PERIOD * 2;
    altHoldUpdate(&altHoldState);
    EXPECT_TRUE(ABS(getAltHoldThrottleFactor(0) - 1.0f) < 0.01f);

    flightModeFlags ^= ALTHOLD_MODE;
    altHoldUpdate(&altHoldState);
    millisRW += 0.5f * ALTHOLD_MAX_EXIT_PERIOD;
    altHoldUpdate(&altHoldState);
    EXPECT_TRUE(ABS(getAltHoldThrottleFactor(1.5f) - 0.5f) < 0.01f);

    flightModeFlags |= ALTHOLD_MODE;
    millisRW += 1;
    altHoldUpdate(&altHoldState);
    EXPECT_TRUE(ABS(getAltHoldThrottleFactor(1.5f) - 0.5f) < 0.01f);
}
