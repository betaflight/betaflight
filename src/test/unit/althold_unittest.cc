#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {

    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "sensors/acceleration.h"
    FAST_DATA_ZERO_INIT acc_t acc;
    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);

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
    float getAltitude(void) { return 0.0f; }
    PG_REGISTER(positionConfig_t, positionConfig, PG_POSITION, 0);

    #include "fc/rc_controls.h"
    float rcCommand[4];

    #include "rx/rx.h"
    void parseRcChannels(const char *input, rxConfig_t *rxConfig)
    {
        UNUSED(input);
        UNUSED(rxConfig);
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
    void altHoldReset();
    void altHoldProcessTransitions();
    void altHoldInit();
    void altHoldUpdate();
    void pt2FilterInit(pt2Filter_t *altHoldDeltaLpf, float) {
        UNUSED(altHoldDeltaLpf);
    }
    float pt2FilterGain(float, float) {
        return 0.0f;
    }
    float pt2FilterApply(pt2Filter_t *altHoldDeltaLpf, float) {
        UNUSED(altHoldDeltaLpf);
        return 0.0f;
    }

}


TEST(AltholdUnittest, altHoldTransitionsTest)
{
    altHoldInit();
    EXPECT_EQ(altHoldState.altHoldActive, false);

    flightModeFlags |= ALTHOLD_MODE;
    millisRW = 42;
    altHoldUpdate();
    EXPECT_EQ(altHoldState.altHoldActive, true);

    millisRW = 56;
    flightModeFlags ^= ALTHOLD_MODE;
    altHoldUpdate();
    EXPECT_EQ(altHoldState.altHoldActive, false);

    flightModeFlags |= ALTHOLD_MODE;
    millisRW = 64;
    altHoldUpdate();
    EXPECT_EQ(altHoldState.altHoldActive, true);

//     millisRW = 64 + 0.3f * ALTHOLD_ENTER_PERIOD;
//     altHoldUpdate();
// 
//     millisRW = 64 + 0.9f * ALTHOLD_ENTER_PERIOD;
//     altHoldUpdate();
// 
//     millisRW = 64 + 1.4f * ALTHOLD_ENTER_PERIOD;
//     altHoldUpdate();
// 
//     millisRW = 10042;
//     flightModeFlags ^= ALTHOLD_MODE;
//     altHoldUpdate();
// 
//     millisRW = 10042 + 0.3f * ALTHOLD_MAX_ENTER_PERIOD;
//     altHoldUpdate();
// 
//     millisRW = 10042 + 0.5f * ALTHOLD_MAX_ENTER_PERIOD;
//     altHoldState.throttleOut = 0.5f;
//     altHoldUpdate();
}

TEST(AltholdUnittest, altHoldTransitionsTestUnfinishedExitEnter)
{
    altHoldInit();
    EXPECT_EQ(altHoldState.altHoldActive, false);

    flightModeFlags |= ALTHOLD_MODE;
    millisRW = 42;
    altHoldUpdate();
    EXPECT_EQ(altHoldState.altHoldActive, true);
//     EXPECT_EQ(altHoldState.enterTime, 42);
//     EXPECT_EQ(altHoldState.exitTime, 0);
// 
//     millisRW += ALTHOLD_ENTER_PERIOD * 2;
//     altHoldUpdate();
// 
//     flightModeFlags ^= ALTHOLD_MODE;
//     altHoldUpdate();
//     millisRW += 0.5f * ALTHOLD_MAX_ENTER_PERIOD;
//     altHoldUpdate();
// 
//     flightModeFlags |= ALTHOLD_MODE;
//     millisRW += 1;
//     altHoldUpdate();
}
