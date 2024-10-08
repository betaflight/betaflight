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

#include <limits.h>

#include <math.h>

extern "C" {
    #include "platform.h"
    #include "target.h"
    #include "build/version.h"
    #include "io/gps.h"
    #include "cli/cli.h"
    #include "cli/settings.h"
    #include "common/printf.h"
    #include "common/maths.h"
    #include "common/gps_conversion.h"
    #include "config/feature.h"
    #include "drivers/buf_writer.h"
    #include "drivers/vtx_common.h"
    #include "config/config.h"
    #include "fc/rc_adjustments.h"
    #include "fc/runtime_config.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/servos.h"
    #include "io/beeper.h"
    #include "io/ledstrip.h"
    #include "io/serial.h"
    #include "io/vtx.h"
    #include "io/vtx_control.h"
    #include "msp/msp.h"
    #include "msp/msp_box.h"
    #include "osd/osd.h"
    #include "pg/pg.h"
    #include "pg/gps_rescue.h"
    #include "pg/pg_ids.h"
    #include "pg/beeper.h"
    #include "pg/gps.h"
    #include "pg/rx.h"
    #include "rx/rx.h"
    #include "scheduler/scheduler.h"
    #include "sensors/battery.h"
    #include "sensors/gyro.h"

    // rc/rx.h
    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];

    void cliSet(const char *cmdName, char *cmdline);
    int cliGetSettingIndex(char *name, uint8_t length);
    void *cliGetValuePointer(const clivalue_t *value);

    void cliHelp(const char *, char *);
    void cliAux(const char *, char *);
    void printAux(
        dumpFlags_t, const modeActivationCondition_t *, const modeActivationCondition_t *, const char *
    );

    const clivalue_t valueTable[] = {
        { .name = "array_unit_test",   .type = VAR_INT8  | MODE_ARRAY  | MASTER_VALUE, .config = { .array = { .length = 3}},                     .pgn = PG_RESERVED_FOR_TESTING_1, .offset = 0 },
        { .name = "str_unit_test",     .type = VAR_UINT8 | MODE_STRING | MASTER_VALUE, .config = { .string = { 0, 16, 0 }},                      .pgn = PG_RESERVED_FOR_TESTING_1, .offset = 0 },
        { .name = "wos_unit_test",     .type = VAR_UINT8 | MODE_STRING | MASTER_VALUE, .config = { .string = { 0, 16, STRING_FLAGS_WRITEONCE }}, .pgn = PG_RESERVED_FOR_TESTING_1, .offset = 0 },
    };
    const uint16_t valueTableEntryCount = ARRAYLEN(valueTable);
    const lookupTableEntry_t lookupTables[] = {};
    const char * const lookupTableOsdDisplayPortDevice[] = {};
    const char * const buildKey = NULL;
    const char * const releaseName = NULL;

    bufWriter_t cliWriterDesc;
    extern bufWriter_t *cliWriter;
    extern bufWriter_t *cliErrorWriter;
    void dummyBufWriter(void *, void *, int);

    PG_REGISTER(osdConfig_t, osdConfig, PG_OSD_CONFIG, 0);
    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER(ledStripConfig_t, ledStripConfig, PG_LED_STRIP_CONFIG, 0);
    PG_REGISTER(ledStripStatusModeConfig_t, ledStripStatusModeConfig, PG_LED_STRIP_STATUS_MODE_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(pilotConfig_t, pilotConfig, PG_PILOT_CONFIG, 0);
    PG_REGISTER_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges, PG_ADJUSTMENT_RANGE_CONFIG, 0);
    PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
    PG_REGISTER_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);
    PG_REGISTER_ARRAY(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);
    PG_REGISTER_ARRAY(servoMixer_t, MAX_SERVO_RULES, customServoMixers, PG_SERVO_MIXER, 0);
    PG_REGISTER(beeperConfig_t, beeperConfig, PG_BEEPER_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(serialConfig_t, serialConfig, PG_SERIAL_CONFIG, 0);
    PG_REGISTER_ARRAY(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs, PG_RX_CHANNEL_RANGE_CONFIG, 0);
    PG_REGISTER_ARRAY(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs, PG_RX_FAILSAFE_CHANNEL_CONFIG, 0);
    PG_REGISTER(pidConfig_t, pidConfig, PG_PID_CONFIG, 0);
    PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    PG_REGISTER(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 0);

    PG_REGISTER_WITH_RESET_FN(int8_t, unitTestData, PG_RESERVED_FOR_TESTING_1, 0);
}

#include <vector>
#include "unittest_macros.h"
#include "gtest/gtest.h"

using namespace std;

const bool PRINT_TEST_DATA = false;


TEST(CLIUnittest, TestCliSetArray)
{
    char *str = (char *)"array_unit_test    =   123,  -3  , 1";
    cliSet("", str);

    const uint16_t index = cliGetSettingIndex(str, 15);
    EXPECT_LT(index, valueTableEntryCount);

    const clivalue_t val = valueTable[index];
    int8_t *data = (int8_t *)cliGetValuePointer(&val);

    if (PRINT_TEST_DATA) {
        printf("\n===============================\n");
        for(int i = 0; i < val.config.array.length; i++){
            printf("data[%d] = %d\n", i, data[i]);
        }
        printf("\n===============================\n");
    }

    EXPECT_EQ(123, data[0]);
    EXPECT_EQ( -3, data[1]);
    EXPECT_EQ(  1, data[2]);
}

TEST(CLIUnittest, TestCliSetStringNoFlags)
{
    char *str = (char *)"str_unit_test    =   SAMPLE"; 
    cliSet("", str);

    const uint16_t index = cliGetSettingIndex(str, 13);
    EXPECT_LT(index, valueTableEntryCount);

    const clivalue_t val = valueTable[index];
    uint8_t *data = (uint8_t *)cliGetValuePointer(&val);

    if (PRINT_TEST_DATA) {
        printf("\n===============================\n");
        for(int i = 0; i < val.config.string.maxlength && data[i] != 0; i++){
            printf("data[%d] = %d (%c)\n", i, data[i], data[i]);
        }
        printf("\n===============================\n");
    }

    EXPECT_EQ('S', data[0]);
    EXPECT_EQ('A', data[1]);
    EXPECT_EQ('M', data[2]);
    EXPECT_EQ('P', data[3]);
    EXPECT_EQ('L', data[4]);
    EXPECT_EQ('E', data[5]);
    EXPECT_EQ(0,   data[6]);
}

TEST(CLIUnittest, TestCliSetStringWriteOnce)
{
    char *str1 = (char *)"wos_unit_test    =   SAMPLE"; 
    char *str2 = (char *)"wos_unit_test    =   ELPMAS"; 
    cliSet("", str1);

    const uint16_t index = cliGetSettingIndex(str1, 13);
    EXPECT_LT(index, valueTableEntryCount);

    const clivalue_t val = valueTable[index];

    uint8_t *data = (uint8_t *)cliGetValuePointer(&val);
    if (PRINT_TEST_DATA) {
        printf("\n===============================\n");
        for(int i = 0; i < val.config.string.maxlength && data[i] != 0; i++){
            printf("data[%d] = %d (%c)\n", i, data[i], data[i]);
        }
        printf("\n===============================\n");
    }
    EXPECT_EQ('S', data[0]);
    EXPECT_EQ('A', data[1]);
    EXPECT_EQ('M', data[2]);
    EXPECT_EQ('P', data[3]);
    EXPECT_EQ('L', data[4]);
    EXPECT_EQ('E', data[5]);
    EXPECT_EQ(0,   data[6]);

    cliSet("", str2);

    EXPECT_EQ('S', data[0]);
    EXPECT_EQ('A', data[1]);
    EXPECT_EQ('M', data[2]);
    EXPECT_EQ('P', data[3]);
    EXPECT_EQ('L', data[4]);
    EXPECT_EQ('E', data[5]);
    EXPECT_EQ(0,   data[6]);

    cliSet("", str1);

    EXPECT_EQ('S', data[0]);
    EXPECT_EQ('A', data[1]);
    EXPECT_EQ('M', data[2]);
    EXPECT_EQ('P', data[3]);
    EXPECT_EQ('L', data[4]);
    EXPECT_EQ('E', data[5]);
    EXPECT_EQ(0,   data[6]);
}

static uint8_t data[1000];
static vector<string> outLines;

class CliWriteTest : public ::testing::Test
{

protected:
    static void SetUpTestCase() {}

    virtual void SetUp() {
        bufWriterInit(&cliWriterDesc, data, ARRAYLEN(data), &dummyBufWriter, NULL);
        cliWriter = cliErrorWriter = &cliWriterDesc;
    }

    virtual void TearDown() {
        cliWriter = NULL;
        outLines.clear();
    }

    static void dummyBufWriter(void *, void *_data, int size) {
        uint8_t *data = (uint8_t *)_data;
        ostringstream stream;
        for (int i = 0; i < size; i++) {
            stream << (char)data[i];
        }
        outLines.push_back(stream.str());
    }

    void clear() {
        outLines.clear();
    }
};

class AuxCliWriteTest : public CliWriteTest {
protected:
    virtual void SetUp() {
        CliWriteTest::SetUp();

        for (uint8_t i = 0; i <= MAX_AUX_CHANNEL_COUNT; i++) {
            memset(modeActivationConditionsMutable(i), 0, sizeof(modeActivationCondition_t));
        }
    }
};

// Help tests
TEST_F(CliWriteTest, HelpAll)
{
    const char cmd[] = "help";
    char args[] = "";
    cliHelp(cmd, args);
    EXPECT_LT(100, outLines.size());
}

TEST_F(CliWriteTest, HelpFindByName)
{
    const char cmd[] = "help";
    char args[] = "aux";
    cliHelp(cmd, args);
    vector<string> expected = {
        "aux", " - configure modes", "\r\n\t<index> <mode> <aux> <start> <end> <logic>", "\r\n",
    };
    EXPECT_EQ(expected, outLines);
}

TEST_F(CliWriteTest, HelpSearchByDescription)
{
    const char cmd[] = "help";
    char args[] = "reb";
    cliHelp(cmd, args);
    vector<string> expected = {
        "bl", " - reboot into bootloader", "\r\n\t[rom]", "\r\n",
        "defaults", " - reset to defaults and reboot", "\r\n\t{nosave}", "\r\n",
        "exit", " - exit command line interface and reboot (default)", "\r\n\t[noreboot]", "\r\n",
        "save", " - save and reboot (default)", "\r\n\t[noreboot]", "\r\n",
    };
    EXPECT_EQ(expected, outLines);
}
// End of help tests

// Aux tests
TEST_F(AuxCliWriteTest, PrintAux_Default)
{
    const char heading[] = "aux";
    printAux(DUMP_MASTER, modeActivationConditions(0), NULL, heading);
    vector<string> expected = {
        "\r\n# ", "aux", "\r\n",
        "aux 0 0 0 900 900 0 0", "\r\n",
        "aux 1 0 0 900 900 0 0", "\r\n",
        "aux 2 0 0 900 900 0 0", "\r\n",
        "aux 3 0 0 900 900 0 0", "\r\n",
        "aux 4 0 0 900 900 0 0", "\r\n",
        "aux 5 0 0 900 900 0 0", "\r\n",
        "aux 6 0 0 900 900 0 0", "\r\n",
        "aux 7 0 0 900 900 0 0", "\r\n",
        "aux 8 0 0 900 900 0 0", "\r\n",
        "aux 9 0 0 900 900 0 0", "\r\n",
        "aux 10 0 0 900 900 0 0", "\r\n",
        "aux 11 0 0 900 900 0 0", "\r\n",
        "aux 12 0 0 900 900 0 0", "\r\n",
        "aux 13 0 0 900 900 0 0", "\r\n",
        "aux 14 0 0 900 900 0 0", "\r\n",
        "aux 15 0 0 900 900 0 0", "\r\n",
        "aux 16 0 0 900 900 0 0", "\r\n",
        "aux 17 0 0 900 900 0 0", "\r\n",
        "aux 18 0 0 900 900 0 0", "\r\n",
        "aux 19 0 0 900 900 0 0", "\r\n",
    };
    EXPECT_EQ(expected, outLines);
}

TEST_F(AuxCliWriteTest, PrintAux_DiffAll)
{
    modeActivationCondition_t modifiedConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT] = {
        [0] = {
            .modeId = BOXANGLE,
            .auxChannelIndex = 7,
            .range = { .startStep = 0, .endStep = 12 },
            .modeLogic = MODELOGIC_OR,
            .linkedTo = BOXARM
        },
    };
    const char heading[] = "aux";
    printAux(DO_DIFF, modifiedConditions, modeActivationConditions(0), heading);
    vector<string> expected = {
        "\r\n# ", "aux", "\r\n",
        "aux 0 1 7 900 1200 0 0", "\r\n",
    };
    EXPECT_EQ(expected, outLines);
}

TEST_F(AuxCliWriteTest, Show)
{
    const char cmd[] = "aux";
    char args[] = "";
    cliAux(cmd, args);
    vector<string> expected = {
        "aux 0 0 0 900 900 0 0", "\r\n",
        "aux 1 0 0 900 900 0 0", "\r\n",
        "aux 2 0 0 900 900 0 0", "\r\n",
        "aux 3 0 0 900 900 0 0", "\r\n",
        "aux 4 0 0 900 900 0 0", "\r\n",
        "aux 5 0 0 900 900 0 0", "\r\n",
        "aux 6 0 0 900 900 0 0", "\r\n",
        "aux 7 0 0 900 900 0 0", "\r\n",
        "aux 8 0 0 900 900 0 0", "\r\n",
        "aux 9 0 0 900 900 0 0", "\r\n",
        "aux 10 0 0 900 900 0 0", "\r\n",
        "aux 11 0 0 900 900 0 0", "\r\n",
        "aux 12 0 0 900 900 0 0", "\r\n",
        "aux 13 0 0 900 900 0 0", "\r\n",
        "aux 14 0 0 900 900 0 0", "\r\n",
        "aux 15 0 0 900 900 0 0", "\r\n",
        "aux 16 0 0 900 900 0 0", "\r\n",
        "aux 17 0 0 900 900 0 0", "\r\n",
        "aux 18 0 0 900 900 0 0", "\r\n",
        "aux 19 0 0 900 900 0 0", "\r\n",
    };
    EXPECT_EQ(expected, outLines);

    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        auto condition = modeActivationConditions(i);
        EXPECT_EQ(0, condition->auxChannelIndex);
    }
}

TEST_F(AuxCliWriteTest, NotEnoughArgs)
{
    const char cmd[] = "aux";
    char args[] = "0 1 ";
    cliAux(cmd, args);
    vector<string> expected = {
        "###ERROR IN ", "aux", ": ", "INVALID ARGUMENT COUNT###", "\r\n"
    };
    EXPECT_EQ(expected, outLines);

    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        auto condition = modeActivationConditions(i);
        EXPECT_EQ(0, condition->auxChannelIndex);
    }
}

TEST_F(AuxCliWriteTest, IndexNotANumber)
{
    const char cmd[] = "aux";
    char args[] = "a";
    cliAux(cmd, args);
    vector<string> expected = {
        "###ERROR IN ", "aux", ": ", "INDEX IS NOT A NUMBER###", "\r\n"
    };
    EXPECT_EQ(expected, outLines);

    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        auto condition = modeActivationConditions(i);
        EXPECT_EQ(0, condition->auxChannelIndex);
    }
}

TEST_F(AuxCliWriteTest, ChannelIndexOutOfRange)
{
    const char cmd[] = "aux";
    char args[] = "0 0 14 1800 2100 0 0";
    cliAux(cmd, args);
    vector<string> expected = {
        "###ERROR IN ", "aux", ": ", "CHANNEL_INDEX NOT BETWEEN 0 AND 13###", "\r\n"
    };
    EXPECT_EQ(expected, outLines);

    auto condition = modeActivationConditions(0);
    EXPECT_EQ(0, condition->auxChannelIndex);
    EXPECT_EQ(0, condition->range.startStep);
    EXPECT_EQ(0, condition->range.endStep);
}

TEST_F(AuxCliWriteTest, ChannelEndRangeOutOfRange)
{
    const char cmd[] = "aux";
    char args[] = "0 0 13 1800 2101 0 0";
    cliAux(cmd, args);
    vector<string> expected = {
        "###ERROR IN ", "aux", ": ", "CHANNEL_RANGE.END NOT BETWEEN 900 AND 2100###", "\r\n"
    };
    EXPECT_EQ(expected, outLines);

    auto condition = modeActivationConditions(0);
    EXPECT_EQ(0, condition->auxChannelIndex);
    EXPECT_EQ(0, condition->range.startStep);
    EXPECT_EQ(0, condition->range.endStep);
}

TEST_F(AuxCliWriteTest, SetCondition)
{
    const char cmd[] = "aux";
    char args[] = "0 1 2 900 1200 0 0";
    cliAux(cmd, args);
    vector<string> expected = {
        "aux 0 1 2 900 1200 0 0", "\r\n",
    };
    EXPECT_EQ(expected, outLines);

    auto condition = modeActivationConditions(0);
    EXPECT_EQ(BOXANGLE, condition->modeId);
    EXPECT_EQ(2, condition->auxChannelIndex);
    EXPECT_EQ(0, condition->range.startStep);
    EXPECT_EQ(12, condition->range.endStep);
    EXPECT_EQ(MODELOGIC_OR, condition->modeLogic);
    EXPECT_EQ(BOXARM, condition->linkedTo);
    for (int i = 1; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        auto condition = modeActivationConditions(i);
        EXPECT_EQ(0, condition->auxChannelIndex);
    }
}

TEST_F(AuxCliWriteTest, BackwardCompat)
{
    const char cmd[] = "aux";
    char args[] = "19 1 2 900 1200 ";
    cliAux(cmd, args);
    vector<string> expected = {
        "aux 19 1 2 900 1200 0 0", "\r\n",
    };
    EXPECT_EQ(expected, outLines);

    auto condition = modeActivationConditions(19);
    EXPECT_EQ(BOXANGLE, condition->modeId);
    EXPECT_EQ(2, condition->auxChannelIndex);
    EXPECT_EQ(0, condition->range.startStep);
    EXPECT_EQ(12, condition->range.endStep);
    EXPECT_EQ(MODELOGIC_OR, condition->modeLogic);
    EXPECT_EQ(BOXARM, condition->linkedTo);
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT - 1; i++) {
        auto condition = modeActivationConditions(i);
        EXPECT_EQ(0, condition->auxChannelIndex);
    }
}

// End of aux tests

// STUBS
extern "C" {

int16_t debug[8];
float motor_disarmed[MAX_SUPPORTED_MOTORS];

uint16_t batteryWarningVoltage;
uint8_t useHottAlarmSoundPeriod (void) { return 0; }
const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000, 400000}; // see baudRate_e

uint8_t debugMode;
int32_t schedLoopStartCycles;
int32_t taskGuardCycles;

uint32_t micros(void) {return 0;}

int32_t getAmperage(void)
{
    return 100;
}

uint16_t getBatteryVoltage(void)
{
    return 42;
}

batteryState_e getBatteryState(void)
{
    return BATTERY_OK;
}

uint8_t calculateBatteryPercentageRemaining(void)
{
    return 67;
}

uint8_t getMotorCount()
{
    return 4;
}

size_t getEEPROMStorageSize()
{
    return 0;
}


void setPrintfSerialPort(struct serialPort_s) {}

static const box_t boxes[] = {
  { "ARM", 0, 0 },
  { "ANGLE", 1, 1 }
};
const box_t *findBoxByPermanentId(uint8_t permanentId) { return &boxes[permanentId]; }
const box_t *findBoxByBoxId(boxId_e boxId) { return &boxes[boxId]; }

int8_t unitTestDataArray[3];

void pgResetFn_unitTestData(int8_t *)
{}

uint32_t getBeeperOffMask(void) { return 0; }
uint32_t getPreferredBeeperOffMask(void) { return 0; }

void beeper(beeperMode_e) {}
void beeperSilence(void) {}
void beeperConfirmationBeeps(uint8_t) {}
void beeperWarningBeeps(uint8_t) {}
void beeperUpdate(timeUs_t) {}
uint32_t getArmingBeepTimeMicros(void) {return 0;}
beeperMode_e beeperModeForTableIndex(int) {return BEEPER_SILENCE;}
uint32_t beeperModeMaskForTableIndex(int idx) {UNUSED(idx); return 0;}
const char *beeperNameForTableIndex(int) {return NULL;}
int beeperTableEntryCount(void) {return 0;}
bool isBeeperOn(void) {return false;}
void beeperOffSetAll(uint8_t) {}
void setBeeperOffMask(uint32_t) {}
void setPreferredBeeperOffMask(uint32_t) {}

void beeperOffSet(uint32_t) {}
void beeperOffClear(uint32_t) {}
void beeperOffClearAll(void) {}
bool parseColor(int, const char *) {return false; }
bool resetEEPROM(void) { return true; }
void mixerResetDisarmedMotors(void) {}

typedef enum {
    DUMMY
} pageId_e;

void dashboardShowFixedPage(pageId_e){}
void dashboardUpdate(timeUs_t) {}

bool parseLedStripConfig(int, const char *){return false; }
const char rcChannelLetters[] = "AERT12345678abcdefgh";

void parseRcChannels(const char *, rxConfig_t *){}
void mixerLoadMix(int, motorMixer_t *) {}
bool setModeColor(ledModeIndex_e, int, int) { return false; }
float motorConvertFromExternal(uint16_t) { return 1.0; }
void motorShutdown(void) { }
uint8_t getCurrentPidProfileIndex(void){ return 1; }
uint8_t getCurrentControlRateProfileIndex(void){ return 1; }
void changeControlRateProfile(uint8_t) {}
void resetAllRxChannelRangeConfigurations(rxChannelRangeConfig_t *) {}
void writeEEPROM() {}
serialPortConfig_t *serialFindPortConfigurationMutable(serialPortIdentifier_e) {return NULL; }
baudRate_e lookupBaudRateIndex(uint32_t){return BAUD_9600; }
serialPortUsage_t *findSerialPortUsageByIdentifier(serialPortIdentifier_e){ return NULL; }
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) { return NULL; }
const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e) { return NULL; }
void serialPassthrough(serialPort_t *, serialPort_t *, serialConsumer *, serialConsumer *) {}
uint32_t millis(void) { return 0; }
uint8_t getBatteryCellCount(void) { return 1; }
void servoMixerLoadMix(int) {}
const char * getBatteryStateString(void){ return "_getBatteryStateString_"; }

uint32_t stackTotalSize(void) { return 0x4000; }
uint32_t stackHighMem(void) { return 0x80000000; }
uint16_t getEEPROMConfigSize(void) { return 1024; }

uint8_t __config_start = 0x00;
uint8_t __config_end = 0x10;
uint16_t averageSystemLoadPercent = 0;

timeDelta_t getTaskDeltaTimeUs(taskId_e){ return 0; }
uint16_t currentRxIntervalUs = 9000;

/*const char *armingDisableFlagNames[]= {
"DUMMYDISABLEFLAGNAME"
};*/

void getTaskInfo(taskId_e, taskInfo_t *) {}
void getCheckFuncInfo(cfCheckFuncInfo_t *) {}
void schedulerResetTaskMaxExecutionTime(taskId_e) {}
void schedulerResetCheckFunctionMaxExecutionTime(void) {}

const char * const targetName = "UNITTEST";
const char* const buildDate = "Jan 01 2017";
const char * const buildTime = "00:00:00";
const char * const shortGitRevision = "MASTER";

//uint32_t serialRxBytesWaiting(const serialPort_t *) {return 0;}
//uint8_t serialRead(serialPort_t *){return 0;}

//void serialWriteBufShim(void *, const uint8_t *, int) {}
//void setArmingDisabled(armingDisableFlags_e) {}

void waitForSerialPortToFinishTransmitting(serialPort_t *) {}
void systemResetToBootloader(void) {}
void resetConfig(void) {}
void systemReset(void) {}
void writeUnmodifiedConfigToEEPROM(void) {}

void changePidProfile(uint8_t) {}
bool serialIsPortAvailable(serialPortIdentifier_e) { return false; }
void generateLedConfig(ledConfig_t *, char *, size_t) {}
//bool isSerialTransmitBufferEmpty(const serialPort_t *) {return true; }
//void serialWrite(serialPort_t *, uint8_t ch) { printf("%c", ch);}

//void serialSetCtrlLineStateCb(serialPort_t *, void (*)(void *, uint16_t ), void *) {}
void serialSetCtrlLineStateDtrPin(serialPort_t *, ioTag_t ) {}
void serialSetCtrlLineState(serialPort_t *, uint16_t ) {}

//void serialSetBaudRateCb(serialPort_t *, void (*)(serialPort_t *context, uint32_t baud), serialPort_t *) {}
void rescheduleTask(taskId_e, timeDelta_t){}
void schedulerSetNextStateTime(timeDelta_t ){}
char *getBoardName(void) { return NULL; }
char *getManufacturerId(void) { return NULL; }
bool boardInformationIsSet(void) { return true; }

bool setBoardName(char *newBoardName) { UNUSED(newBoardName); return true; };
bool setManufacturerId(char *newManufacturerId) { UNUSED(newManufacturerId); return true; };
bool persistBoardInformation(void) { return true; };

void activeAdjustmentRangeReset(void) {}
// void analyzeModeActivationConditions(void) {}
// bool isModeActivationConditionConfigured(const modeActivationCondition_t *, const modeActivationCondition_t *) { return false; }

void delay(uint32_t) {}
displayPort_t *osdGetDisplayPort(osdDisplayPortDevice_e *) { return NULL; }
mcuTypeId_e getMcuTypeId(void) { return MCU_TYPE_UNKNOWN; }
uint16_t getCurrentRxRateHz(void) { return 0; }
uint16_t getAverageSystemLoadPercent(void) { return 0; }
bool getRxRateValid(void) { return false; }
}
