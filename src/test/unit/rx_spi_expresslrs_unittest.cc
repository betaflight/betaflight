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

static void printFhssSequence(uint8_t *seq)
{
    for (int i = 0; i < ELRS_NR_SEQUENCE_ENTRIES; i++) {
        printf("%d, ", seq[i]);
        if (i % 10 == 9) {
            printf("\n");
        }
    }
    printf("\n\n");
}

TEST(RxSpiExpressLrsUnitTest, TestFHSSTable)
{
    const uint8_t UID[6] = {1, 2, 3, 4, 5, 6};

    const uint8_t expectedSequence[2][ELRS_NR_SEQUENCE_ENTRIES] = {
        {
            41, 39, 72, 45, 76, 48, 3, 79, 55, 66,
            68, 13, 65, 20, 15, 43, 21, 54, 46, 37,
            60, 29, 50, 40, 34, 61, 69, 64, 10, 70,
            16, 2, 0, 38, 36, 30, 47, 8, 59, 35,
            4, 71, 73, 11, 9, 51, 32, 1, 18, 12,
            22, 26, 49, 25, 6, 7, 77, 62, 42, 57,
            53, 74, 14, 31, 28, 75, 78, 23, 24, 27,
            17, 44, 67, 33, 19, 5, 52, 56, 58, 63,
            41, 24, 21, 5, 58, 3, 70, 76, 59, 67,
            15, 78, 26, 10, 2, 27, 7, 23, 48, 31,
            72, 13, 56, 45, 51, 50, 44, 54, 39, 53,
            22, 71, 9, 47, 55, 36, 49, 43, 4, 40,
            52, 25, 60, 28, 34, 74, 6, 29, 62, 14,
            8, 0, 19, 17, 79, 46, 42, 77, 37, 18,
            66, 38, 30, 75, 32, 63, 1, 33, 69, 12,
            61, 57, 35, 65, 11, 64, 20, 73, 68, 16,
            41, 75, 76, 17, 26, 22, 25, 60, 53, 43,
            72, 50, 38, 24, 79, 77, 49, 33, 15, 8,
            14, 59, 42, 64, 1, 30, 29, 35, 39, 56,
            40, 36, 74, 18, 47, 73, 62, 13, 61, 58,
            44, 71, 0, 37, 19, 16, 48, 63, 65, 20,
            69, 54, 52, 70, 31, 3, 12, 21, 57, 10,
            5, 7, 28, 55, 27, 6, 11, 9, 78, 45,
            68, 66, 2, 67, 51, 32, 34, 23, 4, 46
        },
        {
            21, 7, 12, 6, 11, 27, 35, 9, 18, 28,
            13, 1, 24, 32, 30, 38, 14, 5, 2, 23,
            37, 26, 17, 3, 25, 29, 39, 36, 0, 4,
            33, 10, 16, 31, 34, 22, 8, 19, 15, 20,
            21, 33, 11, 13, 26, 6, 1, 7, 2, 0,
            9, 28, 19, 18, 23, 10, 29, 14, 25, 3,
            30, 15, 35, 38, 5, 39, 22, 8, 20, 24,
            34, 27, 37, 36, 31, 12, 4, 16, 32, 17,
            21, 20, 30, 37, 11, 5, 26, 3, 18, 32,
            35, 13, 38, 9, 15, 2, 16, 34, 22, 29,
            7, 24, 10, 39, 28, 8, 31, 1, 23, 0,
            12, 36, 19, 4, 27, 17, 6, 25, 33, 14,
            21, 8, 32, 16, 15, 20, 30, 29, 5, 7,
            31, 33, 28, 9, 36, 34, 13, 1, 0, 12,
            35, 26, 25, 39, 22, 19, 11, 2, 37, 23,
            10, 14, 6, 3, 17, 27, 18, 38, 4, 24,
            21, 12, 25, 32, 9, 8, 6, 28, 38, 17,
            2, 31, 10, 16, 20, 24, 13, 22, 39, 29,
            26, 5, 7, 18, 19, 11, 14, 30, 35, 4,
            37, 15, 0, 34, 33, 23, 27, 1, 36, 3,
            21, 36, 5, 31, 17, 32, 10, 34, 1, 3,
            7, 2, 28, 38, 13, 4, 22, 29, 0, 23,
            20, 26, 25, 16, 30, 11, 35, 6, 19, 24,
            8, 18, 33, 15, 37, 12, 14, 39, 27, 9
        }
    };

    fhssGenSequence(UID, ISM2400);
    printFhssSequence(fhssSequence);
    for (int i = 0; i < ELRS_NR_SEQUENCE_ENTRIES; i++) {
        EXPECT_EQ(expectedSequence[0][i], fhssSequence[i]);
    }

    fhssGenSequence(UID, FCC915);
    printFhssSequence(fhssSequence);
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
        FREQ_HZ_TO_REG_VAL_900(434420000), 
        FREQ_HZ_TO_REG_VAL_900(922100000), 
        FREQ_HZ_TO_REG_VAL_900(434450000), 
        FREQ_HZ_TO_REG_VAL_900(867783300),
        FREQ_HZ_TO_REG_VAL_900(866949900),
        FREQ_HZ_TO_REG_VAL_900(916100000), 
        FREQ_HZ_TO_REG_VAL_24(2441400000)
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
    rxExpressLrsSpiConfigMutable()->switchMode = SM_WIDE;
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
    void sx1280SetOutputPower(const int8_t ) {}

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

    void setTelemetryDataToTransmit(const uint8_t , uint8_t* ) {}
    bool isTelemetrySenderActive(void) { return false; }
    uint8_t getCurrentTelemetryPayload(uint8_t * ) { return 0; }
    void confirmCurrentTelemetryPayload(const bool ) {}
    void updateTelemetryRate(const uint16_t , const uint8_t , const uint8_t ) {}

    void meanAccumulatorAdd(meanAccumulator_t * , const int8_t ) {};
    int8_t meanAccumulatorCalc(meanAccumulator_t * , const int8_t defaultValue) { return defaultValue; };
    void meanAccumulatorInit(meanAccumulator_t * ) {};
}
