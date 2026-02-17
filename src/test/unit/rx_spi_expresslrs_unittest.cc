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
    extern uint16_t seqCount;
    extern uint16_t crc14tab[ELRS_CRC_LEN];

    extern elrsReceiver_t receiver;
    static const elrsReceiver_t empty = elrsReceiver_t();

    static rxRuntimeState_t config = rxRuntimeState_t();
    static rxSpiExtiConfig_t extiConfig;
    static const rxSpiConfig_t injectedConfig = {
        .rx_spi_protocol = 0,
        .rx_spi_id = 0,
        .rx_spi_rf_channel_count = 0,
        .csnTag = IO_TAG_NONE,
        .spibus = 0,
        .bindIoTag = IO_TAG_NONE,
        .ledIoTag = IO_TAG_NONE,
        .ledInversion = 0,
        .extiIoTag = IO_TAG_NONE,
    };
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// set to dump generted sequences during test
const bool PRINT_FHSS_SEQUENCES = false;

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
    for (int i = 0; i < seqCount; i++) {
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
    const uint32_t seed = elrsUidToSeed(UID);

    const uint8_t expectedSequence[2][ELRS_NR_SEQUENCE_ENTRIES] = {
        {
            40, 60, 23, 37, 16, 55, 0, 63, 13, 59,
            66, 20, 26, 18, 33, 53, 24, 54, 17, 64,
            7, 51, 62, 47, 41, 69, 22, 29, 57, 6,
            52, 4, 34, 38, 31, 25, 32, 78, 79, 61,
            21, 10, 36, 9, 27, 65, 8, 12, 67, 45,
            28, 73, 15, 43, 56, 3, 5, 44, 76, 49,
            58, 30, 35, 1, 68, 50, 39, 72, 75, 77,
            46, 2, 74, 11, 71, 19, 14, 70, 42, 48,
            40, 56, 47, 32, 39, 11, 41, 43, 52, 62,
            74, 61, 54, 21, 2, 15, 37, 78, 34, 36,
            24, 57, 23, 0, 60, 79, 58, 25, 12, 35,
            77, 51, 50, 69, 3, 59, 46, 76, 5, 38,
            8, 31, 14, 4, 64, 29, 49, 65, 18, 28,
            22, 53, 72, 44, 45, 33, 9, 26, 16, 67,
            68, 70, 66, 75, 1, 7, 55, 71, 27, 73,
            20, 13, 30, 48, 42, 63, 6, 19, 10, 17,
            40, 34, 36, 61, 56, 35, 53, 14, 64, 66,
            30, 50, 62, 48, 78, 32, 49, 10, 70, 31,
            17, 26, 37, 6, 74, 46, 69, 51, 18, 39,
            71, 42, 29, 11, 38, 9, 16, 41, 68, 20,
            43, 4, 54, 27, 19, 57, 22, 13, 1, 44,
            76, 58, 45, 21, 24, 33, 12, 7, 55, 60,
            77, 73, 67, 5, 65, 23, 15, 2, 63, 72,
            47, 0, 79, 52, 25, 3, 59, 28, 75, 8,
        },
        {
            20, 39, 24, 6, 4, 38, 28, 37, 5, 3,
            10, 16, 23, 9, 17, 27, 22, 36, 29, 19,
            7, 15, 12, 35, 30, 13, 2, 32, 26, 34,
            31, 8, 25, 33, 0, 18, 1, 11, 14, 21,
            20, 30, 3, 18, 25, 35, 19, 17, 29, 2,
            6, 38, 34, 0, 26, 36, 21, 8, 1, 37,
            32, 11, 31, 28, 9, 22, 23, 10, 4, 15,
            16, 13, 5, 7, 12, 27, 24, 39, 33, 14,
            20, 1, 34, 30, 28, 2, 29, 26, 23, 3,
            13, 38, 11, 17, 31, 12, 6, 25, 14, 5,
            7, 33, 16, 32, 35, 21, 24, 27, 10, 19,
            18, 36, 39, 8, 0, 22, 9, 37, 4, 15,
            20, 32, 4, 12, 33, 15, 23, 21, 16, 18,
            30, 11, 1, 39, 27, 6, 22, 5, 31, 2,
            28, 26, 36, 9, 34, 13, 0, 8, 19, 3,
            14, 7, 37, 38, 25, 10, 24, 29, 17, 35,
            20, 22, 23, 30, 19, 8, 9, 18, 13, 24,
            28, 39, 0, 2, 21, 37, 6, 36, 11, 38,
            27, 3, 1, 32, 4, 26, 25, 34, 29, 10,
            15, 14, 17, 31, 12, 35, 7, 16, 33, 5,
            20, 13, 28, 7, 26, 14, 18, 32, 24, 15,
            8, 4, 23, 5, 35, 1, 11, 39, 17, 30,
            3, 19, 0, 37, 33, 2, 27, 25, 12, 38,
            36, 29, 9, 16, 6, 21, 31, 34, 22, 10,
        }
    };
    UNUSED(expectedSequence);

    fhssGenSequence(seed, ISM2400);
    if (PRINT_FHSS_SEQUENCES) {
        printFhssSequence(fhssSequence);
    }
    for (int i = 0; i < ELRS_NR_SEQUENCE_ENTRIES; i++) {
        EXPECT_EQ(expectedSequence[0][i], fhssSequence[i]);
    }

    fhssGenSequence(seed, FCC915);
    if (PRINT_FHSS_SEQUENCES) {
        printFhssSequence(fhssSequence);
    }
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
        FREQ_HZ_TO_REG_VAL_900(921499940),
        FREQ_HZ_TO_REG_VAL_900(433775000),
        FREQ_HZ_TO_REG_VAL_900(867424930),
        FREQ_HZ_TO_REG_VAL_900(866424930),
        FREQ_HZ_TO_REG_VAL_900(915499940),
        FREQ_HZ_TO_REG_VAL_24(2440399900)
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
    EXPECT_EQ(airRateConfig[1][5].index, receiver.modParams->index);
    EXPECT_EQ(airRateConfig[1][5].enumRate, receiver.modParams->enumRate);
    EXPECT_EQ(airRateConfig[1][5].bw, receiver.modParams->bw);
    EXPECT_EQ(airRateConfig[1][5].sf, receiver.modParams->sf);
    EXPECT_EQ(airRateConfig[1][5].cr, receiver.modParams->cr);
    EXPECT_EQ(airRateConfig[1][5].interval, receiver.modParams->interval);
    EXPECT_EQ(airRateConfig[1][5].tlmInterval, receiver.modParams->tlmInterval);
    EXPECT_EQ(airRateConfig[1][5].fhssHopInterval, receiver.modParams->fhssHopInterval);
    EXPECT_EQ(airRateConfig[1][5].preambleLen, receiver.modParams->preambleLen);

    //check switch mode
    receiver = empty;
    expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
    EXPECT_EQ(16, config.channelCount);
}

TEST(RxSpiExpressLrsUnitTest, TestInitBound)
{
    const uint8_t validUID[6] = {0, 0, 1, 2, 3, 4};
    receiver = empty;
    memcpy(rxExpressLrsSpiConfigMutable()->UID, validUID, 6);

    // check mod params
    for (int i = 0; i < ELRS_RATE_MAX_900; i++) {
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
    }

    for (int i = 0; i < ELRS_RATE_MAX_24; i++) {
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
    void sx1280Config(const uint8_t /* bw */, const uint8_t /* sfbt */, const uint8_t /* cr */,
        const uint32_t /* freq */, const uint8_t /* preambleLength */, const bool /* iqInverted */,
        const uint32_t /* flrcSyncWord */, const uint16_t /* flrcCrcSeed */, const bool /* isFlrc */) {}
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
    void sx1280AdjustFrequency(int32_t *, const uint32_t ) {}
    bool sx1280Init(IO_t , IO_t ) { return true; }
    void sx1280SetOutputPower(const int8_t ) {}

    void sx127xConfig(const uint8_t /* bw */, const uint8_t /* sfbt */, const uint8_t /* cr */,
        const uint32_t /* freq */, const uint8_t /* preambleLength */, const bool /* iqInverted */,
        const uint32_t /* flrcSyncWord */, const uint16_t /* flrcCrcSeed */, const bool /* isFlrc */) {}
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
    void sx127xAdjustFrequency(int32_t *, const uint32_t ) {}
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
    uint8_t getCurrentTelemetryPayload(uint8_t *, uint8_t ) { return 0; }
    void confirmCurrentTelemetryPayload(const bool ) {}
    void updateTelemetryRate(const uint16_t , const uint8_t , const uint8_t ) {}

    void meanAccumulatorAdd(meanAccumulator_t * , const int8_t ) {};
    int8_t meanAccumulatorCalc(meanAccumulator_t * , const int8_t defaultValue) { return defaultValue; };
    void meanAccumulatorInit(meanAccumulator_t * ) {};
}
