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

/*
 * Based on https://github.com/ExpressLRS/ExpressLRS
 * Thanks to AlessandroAU, original creator of the ExpressLRS project.
 */

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

extern "C" {
    #include "platform.h"

    #include "build/atomic.h"

    #include "drivers/io.h"
    #include "common/filter.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx_spi.h"
    #include "pg/rx_spi_expresslrs.h"

    #include "rx/rx_spi.h"
    #include "rx/expresslrs.h"
    #include "rx/expresslrs_impl.h"

    #include "drivers/rx/rx_sx127x.h"
    #include "drivers/rx/rx_sx1280.h"

    extern uint8_t fhssSequence[ELRS_NR_SEQUENCE_ENTRIES];
    extern uint16_t crc14tab[ELRS_CRC_LEN];

    extern elrsReceiver_t receiver;
    static const elrsReceiver_t empty = elrsReceiver_t();

    static rxRuntimeState_t config = rxRuntimeState_t();
    static rxSpiExtiConfig_t extiConfig;
    static const rxSpiConfig_t injectedConfig = {
        .extiIoTag = IO_TAG(PA0),
    };
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

//make clean test_rx_spi_expresslrs_unittest
TEST(RxSpiExpressLrsUnitTest, TestCrc14)
{
    uint16_t expectedCrc14Tab[ELRS_CRC_LEN] = {
        0,28247,62201,40110,52133,42482,14684,22283,
        38730,63773,26035,3044,23791,12984,44566,49217,
        11924,16579,56429,45626,58673,35686,6088,31135,
        47582,55177,19239,9584,29307,7212,32898,61141,
        29567,7464,33158,61393,47322,54925,18979,9332,
        58421,35426,5836,30875,12176,16839,56681,45886,
        24043,13244,44818,49477,38478,63513,25783,2784,
        51873,42230,14424,22031,260,28499,62461,40362,
        51369,42750,14928,21511,780,27995,61941,40866,
        24547,12724,44314,49997,37958,64017,26303,2280,
        58941,34922,5316,31379,11672,17359,57185,45366,
        29047,7968,33678,60889,47826,54405,18475,9852,
        48086,54657,18735,10104,28787,7716,33418,60637,
        11420,17099,56933,45106,59193,35182,5568,31639,
        38210,64277,26555,2540,24295,12464,44062,49737,
        520,27743,61681,40614,51629,43002,15188,21763,
        37202,65285,25515,3580,23287,13472,43022,50777,
        1560,26703,62689,39606,52669,41962,16196,20755,
        49094,53649,19775,9064,29795,6708,34458,59597,
        10380,18139,55925,46114,58153,36222,4560,32647,
        57901,35962,4308,32387,10632,18399,56177,46374,
        30055,6960,34718,59849,48834,53397,19515,8812,
        52409,41710,15936,20503,1820,26955,62949,39858,
        23539,13732,43274,51037,36950,65025,25263,3320,
        23035,14252,43778,50517,37470,64521,24743,3824,
        52913,41190,15432,21023,1300,27459,63469,39354,
        30575,6456,34198,60353,48330,53917,20019,8292,
        57381,36466,4828,31883,11136,17879,55673,46894,
        10884,17619,55421,46634,57633,36726,5080,32143,
        48590,54169,20279,8544,30315,6204,33938,60101,
        1040,27207,63209,39102,53173,41442,15692,21275,
        37722,64781,24995,4084,22783,13992,43526,50257
    };

    generateCrc14Table();
    for (int i = 0; i < ELRS_CRC_LEN; i++) {
        EXPECT_EQ(expectedCrc14Tab[i], crc14tab[i]);
    }
}

TEST(RxSpiExpressLrsUnitTest, TestFHSSTable)
{
    const uint8_t UID[6] = {1, 2, 3, 4, 5, 6};

    const uint8_t expectedSequence[2][ELRS_NR_SEQUENCE_ENTRIES] = {
        {
            40, 43, 39, 18, 52, 19, 36, 7, 1, 12,
            71, 5, 42, 46, 50, 28, 49, 33, 76, 51, 
            60, 70, 47, 61, 0, 55, 72, 37, 53, 25,
            3, 11, 41, 13, 35, 27, 9, 75, 48, 77, 
            73, 74, 69, 58, 14, 31, 10, 59, 66, 4, 
            78, 17, 44, 54, 29, 57, 21, 64, 22, 67, 
            62, 56, 15, 79, 6, 34, 23, 30, 32, 2, 
            68, 8, 63, 65, 45, 20, 24, 26, 16, 38, 
            40, 8, 52, 29, 57, 10, 6, 26, 19, 75, 
            21, 24, 1, 9, 50, 32, 69, 67, 2, 59, 
            28, 48, 77, 60, 41, 49, 68, 4, 5, 3, 
            44, 78, 58, 31, 16, 62, 35, 45, 73, 11, 
            33, 46, 42, 36, 64, 7, 34, 53, 17, 25, 
            37, 38, 54, 55, 15, 76, 18, 43, 23, 12, 
            39, 51, 22, 79, 74, 63, 27, 66, 65, 47, 
            70, 0, 30, 61, 13, 56, 14, 72, 71, 20, 
            40, 71, 68, 12, 57, 45, 10, 53, 21, 15, 
            69, 26, 54, 55, 73, 47, 35, 77, 1, 31, 
            20, 0, 38, 76, 5, 60, 6, 79, 3, 16, 
            50, 17, 52, 62, 18, 46, 28, 39, 29, 51, 
            43, 34, 49, 56, 32, 61, 74, 58, 25, 44, 
            2, 19, 65, 4, 13, 67, 11, 30, 66, 64, 
            36, 24, 75, 33, 59, 7, 41, 70, 48, 14, 
            42, 37, 8, 23, 78, 63, 22, 9, 72, 27
        },
        {
            20, 37, 1, 3, 7, 26, 36, 29, 15, 35, 
            33, 24, 10, 34, 13, 31, 22, 9, 28, 23, 
            17, 38, 6, 27, 0, 32, 11, 5, 18, 25, 
            2, 4, 12, 19, 16, 8, 30, 14, 21, 39, 
            20, 2, 14, 7, 13, 33, 32, 28, 21, 11, 
            25, 17, 22, 9, 3, 4, 0, 31, 35, 38, 
            10, 34, 26, 39, 36, 6, 19, 16, 30, 27, 
            15, 24, 18, 1, 23, 37, 29, 8, 12, 5, 
            20, 19, 24, 29, 27, 2, 22, 14, 0, 3, 
            23, 13, 12, 35, 4, 25, 38, 18, 33, 36, 
            21, 16, 5, 31, 9, 32, 11, 1, 6, 7, 
            10, 15, 26, 34, 39, 37, 28, 17, 30, 8, 
            20, 7, 4, 24, 19, 16, 8, 13, 15, 10, 
            14, 36, 34, 0, 17, 12, 28, 21, 39, 22, 
            3, 2, 32, 33, 27, 6, 37, 18, 31, 38, 
            23, 25, 26, 30, 9, 1, 35, 5, 11, 29, 
            20, 1, 35, 22, 0, 10, 11, 27, 18, 37, 
            21, 31, 9, 19, 30, 17, 5, 38, 29, 36, 
            3, 2, 25, 34, 23, 6, 15, 4, 16, 26, 
            12, 24, 14, 13, 39, 8, 32, 7, 28, 33, 
            20, 36, 13, 5, 39, 37, 15, 8, 9, 4, 
            22, 12, 1, 6, 32, 25, 17, 18, 27, 28, 
            23, 19, 26, 3, 38, 16, 2, 34, 14, 30, 
            10, 11, 7, 0, 35, 24, 21, 33, 31, 29 
        }
    };

    fhssGenSequence(UID, ISM2400);
    for (int i = 0; i < ELRS_NR_SEQUENCE_ENTRIES; i++) {
        EXPECT_EQ(expectedSequence[0][i], fhssSequence[i]);
    }

    fhssGenSequence(UID, FCC915);
    for (int i = 0; i < ELRS_NR_SEQUENCE_ENTRIES; i++) {
        EXPECT_EQ(expectedSequence[1][i], fhssSequence[i]);
    }
}

TEST(RxSpiExpressLrsUnitTest, TestInitUnbound)
{
    const uint8_t bindUID[6] = {0, 1, 2, 3, 4, 5};

    receiver = empty;
    expressLrsSpiInit(&injectedConfig, &config, &extiConfig);

    //check initialization of elrsReceiver_t
    EXPECT_TRUE(receiver.inBindingMode);
    EXPECT_EQ(IO_NONE, receiver.resetPin);
    EXPECT_EQ(IO_NONE, receiver.busyPin);
    for (int i = 0; i < 6; i++) {
        EXPECT_EQ(bindUID[i], receiver.UID[i]);
    }
    EXPECT_EQ(0, receiver.nonceRX);
    EXPECT_EQ(0, receiver.freqOffset);
    EXPECT_EQ(0, receiver.lastValidPacketMs);

    const uint32_t initialFrequencies[7] = {
        FREQ_HZ_TO_REG_VAL_900(433920000), 
        FREQ_HZ_TO_REG_VAL_900(921500000), 
        FREQ_HZ_TO_REG_VAL_900(433925000), 
        FREQ_HZ_TO_REG_VAL_900(866425000),
        FREQ_HZ_TO_REG_VAL_900(866425000),
        FREQ_HZ_TO_REG_VAL_900(915500000), 
        FREQ_HZ_TO_REG_VAL_24(2440400000)
    };

    for (int i = 0; i < 7; i++) {
        receiver = empty;
        rxExpressLrsSpiConfigMutable()->domain = i;
        expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
        EXPECT_EQ(initialFrequencies[i], receiver.currentFreq);
    }

    // for unbound we need to initialize in 50HZ mode
    receiver = empty;
    rxExpressLrsSpiConfigMutable()->rateIndex = 1;
    rxExpressLrsSpiConfigMutable()->domain = FCC915;
    expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
    EXPECT_EQ(airRateConfig[0][2].index, receiver.modParams->index);
    EXPECT_EQ(airRateConfig[0][2].enumRate, receiver.modParams->enumRate);
    EXPECT_EQ(airRateConfig[0][2].bw, receiver.modParams->bw);
    EXPECT_EQ(airRateConfig[0][2].sf, receiver.modParams->sf);
    EXPECT_EQ(airRateConfig[0][2].cr, receiver.modParams->cr);
    EXPECT_EQ(airRateConfig[0][2].interval, receiver.modParams->interval);
    EXPECT_EQ(airRateConfig[0][2].tlmInterval, receiver.modParams->tlmInterval);
    EXPECT_EQ(airRateConfig[0][2].fhssHopInterval, receiver.modParams->fhssHopInterval);
    EXPECT_EQ(airRateConfig[0][2].preambleLen, receiver.modParams->preambleLen);

    receiver = empty;
    rxExpressLrsSpiConfigMutable()->rateIndex = 1;
    rxExpressLrsSpiConfigMutable()->domain = ISM2400;
    expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
    EXPECT_EQ(airRateConfig[1][3].index, receiver.modParams->index);
    EXPECT_EQ(airRateConfig[1][3].enumRate, receiver.modParams->enumRate);
    EXPECT_EQ(airRateConfig[1][3].bw, receiver.modParams->bw);
    EXPECT_EQ(airRateConfig[1][3].sf, receiver.modParams->sf);
    EXPECT_EQ(airRateConfig[1][3].cr, receiver.modParams->cr);
    EXPECT_EQ(airRateConfig[1][3].interval, receiver.modParams->interval);
    EXPECT_EQ(airRateConfig[1][3].tlmInterval, receiver.modParams->tlmInterval);
    EXPECT_EQ(airRateConfig[1][3].fhssHopInterval, receiver.modParams->fhssHopInterval);
    EXPECT_EQ(airRateConfig[1][3].preambleLen, receiver.modParams->preambleLen);

    //check switch mode
    receiver = empty;
    rxExpressLrsSpiConfigMutable()->switchMode = SM_HYBRID;
    expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
    EXPECT_EQ(16, config.channelCount);
    receiver = empty;
    rxExpressLrsSpiConfigMutable()->switchMode = SM_HYBRID_WIDE;
    expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
    EXPECT_EQ(16, config.channelCount);
}

TEST(RxSpiExpressLrsUnitTest, TestInitBound)
{
    const uint8_t validUID[6] = {0, 0, 1, 2, 3, 4};
    receiver = empty;
    memcpy(rxExpressLrsSpiConfigMutable()->UID, validUID, 6);
    
    // check mod params
    for (int i = 0; i < ELRS_RATE_MAX; i++) {
        receiver = empty;
        rxExpressLrsSpiConfigMutable()->rateIndex = i;
        rxExpressLrsSpiConfigMutable()->domain = FCC915;
        expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
        EXPECT_EQ(airRateConfig[0][i].index, receiver.modParams->index);
        EXPECT_EQ(airRateConfig[0][i].enumRate, receiver.modParams->enumRate);
        EXPECT_EQ(airRateConfig[0][i].bw, receiver.modParams->bw);
        EXPECT_EQ(airRateConfig[0][i].sf, receiver.modParams->sf);
        EXPECT_EQ(airRateConfig[0][i].cr, receiver.modParams->cr);
        EXPECT_EQ(airRateConfig[0][i].interval, receiver.modParams->interval);
        EXPECT_EQ(airRateConfig[0][i].tlmInterval, receiver.modParams->tlmInterval);
        EXPECT_EQ(airRateConfig[0][i].fhssHopInterval, receiver.modParams->fhssHopInterval);
        EXPECT_EQ(airRateConfig[0][i].preambleLen, receiver.modParams->preambleLen);

        receiver = empty;
        rxExpressLrsSpiConfigMutable()->rateIndex = i;
        rxExpressLrsSpiConfigMutable()->domain = ISM2400;
        expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
        EXPECT_EQ(airRateConfig[1][i].index, receiver.modParams->index);
        EXPECT_EQ(airRateConfig[1][i].enumRate, receiver.modParams->enumRate);
        EXPECT_EQ(airRateConfig[1][i].bw, receiver.modParams->bw);
        EXPECT_EQ(airRateConfig[1][i].sf, receiver.modParams->sf);
        EXPECT_EQ(airRateConfig[1][i].cr, receiver.modParams->cr);
        EXPECT_EQ(airRateConfig[1][i].interval, receiver.modParams->interval);
        EXPECT_EQ(airRateConfig[1][i].tlmInterval, receiver.modParams->tlmInterval);
        EXPECT_EQ(airRateConfig[1][i].fhssHopInterval, receiver.modParams->fhssHopInterval);
        EXPECT_EQ(airRateConfig[1][i].preambleLen, receiver.modParams->preambleLen);
    }

    expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
    EXPECT_FALSE(receiver.inBindingMode);
    for (int i = 0; i < 6; i++) {
        EXPECT_EQ(validUID[i], receiver.UID[i]);
    }
}

TEST(RxSpiExpressLrsUnitTest, TestLQCalc)
{
    lqReset();
    for (int i = 1; i <= 100; i++) {
        lqNewPeriod();
        lqIncrease();
        EXPECT_EQ(i, lqGet());
    }
    lqNewPeriod();
    lqIncrease();
    EXPECT_EQ(100, lqGet());
    for (int i = 99; i >= 0; i--) {
        lqNewPeriod();
        EXPECT_EQ(i, lqGet());
    }
    lqNewPeriod();
    EXPECT_EQ(0, lqGet());
    lqReset();
    lqNewPeriod();
    EXPECT_EQ(0, lqGet());
    lqIncrease();
    EXPECT_EQ(1, lqGet());
}

TEST(RxSpiExpressLrsUnitTest, Test1bSwitchDecode)
{
    EXPECT_EQ(1000, convertSwitch1b(0));
    EXPECT_EQ(2000, convertSwitch1b(1));
    EXPECT_EQ(2000, convertSwitch1b(2));
    EXPECT_EQ(2000, convertSwitch1b(255));
}

TEST(RxSpiExpressLrsUnitTest, Test3bSwitchDecode)
{
    EXPECT_EQ(1000, convertSwitch3b(0));
    EXPECT_EQ(1275, convertSwitch3b(1));
    EXPECT_EQ(1425, convertSwitch3b(2));
    EXPECT_EQ(1575, convertSwitch3b(3));
    EXPECT_EQ(1725, convertSwitch3b(4));
    EXPECT_EQ(2000, convertSwitch3b(5));
    EXPECT_EQ(1500, convertSwitch3b(6));
    EXPECT_EQ(1500, convertSwitch3b(7));
    EXPECT_EQ(1500, convertSwitch3b(8));
    EXPECT_EQ(1500, convertSwitch3b(123));
    EXPECT_EQ(1500, convertSwitch3b(255));
}

TEST(RxSpiExpressLrsUnitTest, Test4bSwitchDecode)
{
    EXPECT_EQ(1000, convertSwitchNb(0, 15));
    EXPECT_EQ(1066, convertSwitchNb(1, 15));
    EXPECT_EQ(1133, convertSwitchNb(2, 15));
    EXPECT_EQ(1200, convertSwitchNb(3, 15));
    EXPECT_EQ(1266, convertSwitchNb(4, 15));
    EXPECT_EQ(1333, convertSwitchNb(5, 15));
    EXPECT_EQ(1400, convertSwitchNb(6, 15));
    EXPECT_EQ(1466, convertSwitchNb(7, 15));
    EXPECT_EQ(1533, convertSwitchNb(8, 15));
    EXPECT_EQ(1600, convertSwitchNb(9, 15));
    EXPECT_EQ(1666, convertSwitchNb(10, 15));
    EXPECT_EQ(1733, convertSwitchNb(11, 15));
    EXPECT_EQ(1800, convertSwitchNb(12, 15));
    EXPECT_EQ(1866, convertSwitchNb(13, 15));
    EXPECT_EQ(1933, convertSwitchNb(14, 15));
    EXPECT_EQ(2000, convertSwitchNb(15, 15));
    EXPECT_EQ(1500, convertSwitchNb(16, 15));
    EXPECT_EQ(1500, convertSwitchNb(255, 15));
}

TEST(RxSpiExpressLrsUnitTest, TestAnalogDecode)
{
    EXPECT_EQ(988, convertAnalog(172));
    EXPECT_EQ(1500, convertAnalog(992));
    EXPECT_EQ(2012, convertAnalog(1811));
}

// STUBS

extern "C" {

    uint8_t systemState;
    int16_t *debug;
    uint8_t debugMode;

    rssiSource_e rssiSource;
    linkQualitySource_e linkQualitySource;
    void setRssi(uint16_t , rssiSource_e ) {}
    void setRssiDirect(uint16_t , rssiSource_e ) {}

    uint32_t micros(void) { return 0; }
    uint32_t millis(void) { return 0; }

    bool IORead(IO_t ) { return true; }
    IO_t IOGetByTag(ioTag_t ) { return (IO_t)1; }
    void IOHi(IO_t ) {}
    void IOLo(IO_t ) {}

    void saveConfigAndNotify(void) {}

    void rxSpiCommonIOInit(const rxSpiConfig_t *) {}
    void rxSpiLedBlinkRxLoss(rx_spi_received_e ) {}
    void rxSpiLedBlinkBind(void) {}
    bool rxSpiCheckBindRequested(bool)
    {
        return false;
    }
    bool rxSpiExtiConfigured(void) { return true; }

    bool sx1280IsBusy(void) { return false; }
    void sx1280Config(const sx1280LoraBandwidths_e , const sx1280LoraSpreadingFactors_e , const sx1280LoraCodingRates_e , const uint32_t , const uint8_t , const bool ) {}
    void sx1280StartReceiving(void) {}
    void sx1280ISR(void) {}
    bool rxSpiGetExtiState(void) { return false; }
    void sx1280HandleFromTock(void) {}
    bool sx1280HandleFromTick(void) { return false; }
    void sx1280TransmitData(const uint8_t *, const uint8_t ) {}
    void sx1280ReceiveData(uint8_t *, const uint8_t ) {}
    void sx1280SetFrequencyReg(const uint32_t ) {}
    void sx1280GetLastPacketStats(int8_t *rssi, int8_t *snr) 
    {
        *rssi = 0;
        *snr = 0;
    }
    void sx1280AdjustFrequency(int32_t , const uint32_t ) {}
    bool sx1280Init(IO_t , IO_t ) { return true; }

    void sx127xConfig(const sx127xBandwidth_e , const sx127xSpreadingFactor_e , const sx127xCodingRate_e , const uint32_t , const uint8_t , const bool ) {}
    void sx127xStartReceiving(void) {}
    uint8_t sx127xISR(uint32_t *timestamp)
    {
        *timestamp = 0;
        return 0;
    }
    void sx127xTransmitData(const uint8_t *, const uint8_t ) {}
    void sx127xReceiveData(uint8_t *, const uint8_t ) {}
    void sx127xSetFrequencyReg(const uint32_t ) {}
    void sx127xGetLastPacketStats(int8_t *rssi, int8_t *snr) 
    {
        *rssi = 0;
        *snr = 0;
    }
    void sx127xAdjustFrequency(int32_t , const uint32_t ) {}
    bool sx127xInit(IO_t , IO_t ) { return true; }

    int scaleRange(int x, int srcFrom, int srcTo, int destFrom, int destTo) {
        long int a = ((long int) destTo - (long int) destFrom) * ((long int) x - (long int) srcFrom);
        long int b = (long int) srcTo - (long int) srcFrom;
        return (a / b) + destFrom;
    }

    void expressLrsInitialiseTimer(TIM_TypeDef *, elrsReceiver_t *) {}
    void expressLrsTimerEnableIRQs(void) {}
    void expressLrsUpdateTimerInterval(uint16_t ) {}
    void expressLrsUpdatePhaseShift(int32_t ) {}
    void expressLrsTimerIncreaseFrequencyOffset(void) {}
    void expressLrsTimerDecreaseFrequencyOffset(void) {}
    void expressLrsTimerResetFrequencyOffset(void) {}
    void expressLrsTimerStop(void) {}
    void expressLrsTimerResume(void) {}
    bool expressLrsTimerIsRunning(void) { return true; }
    void expressLrsTimerDebug(void) {}

    int32_t simpleLPFilterUpdate(simpleLowpassFilter_t *, int32_t ) { return 0; }
    void simpleLPFilterInit(simpleLowpassFilter_t *, int32_t , int32_t ) {}
    void dbgPinHi(int ) {}
    void dbgPinLo(int ) {}

    void initTelemetry(void) {}
    bool getNextTelemetryPayload(uint8_t *, uint8_t **) { return false; }

    void setTelemetryDataToTransmit(const uint8_t , uint8_t* , const uint8_t ) {}
    bool isTelemetrySenderActive(void) { return false; }
    void getCurrentTelemetryPayload(uint8_t *, uint8_t *, uint8_t **) {}
    void confirmCurrentTelemetryPayload(const bool ) {}
    void updateTelemetryRate(const uint16_t , const uint8_t , const uint8_t ) {}
}
