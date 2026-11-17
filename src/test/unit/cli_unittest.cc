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
#include <string>

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
    #include "io/serial_feature_map.h"
    #include "io/vtx.h"
    #include "msp/msp.h"
    #include "msp/msp_box.h"
    #include "osd/osd.h"
    #include "pg/pg.h"
    #include "pg/gps_rescue.h"
    #include "pg/pg_ids.h"
    #include "pg/beeper.h"
    #include "pg/gps.h"
    #include "pg/pilot.h"
    #include "pg/rx.h"
    #include "rx/rx.h"
    #include "scheduler/scheduler.h"
    #include "sensors/battery.h"
    #include "sensors/gyro.h"

    bool gpsNmeaFlushPending(void);

    void cliSet(const char *cmdName, char *cmdline);
    void cliHelp(const char *cmdName, char *cmdline);
    int cliGetSettingIndex(char *name, uint8_t length);
    void *cliGetValuePointer(const clivalue_t *value);

    const clivalue_t valueTable[] = {
        { .name = "array_unit_test",   .type = VAR_INT8  | MODE_ARRAY  | MASTER_VALUE, .config = { .array = { .length = 3}},                     .pgn = PG_RESERVED_FOR_TESTING_1, .offset = 0 },
        { .name = "str_unit_test",     .type = VAR_UINT8 | MODE_STRING | MASTER_VALUE, .config = { .string = { 0, 16, 0 }},                      .pgn = PG_RESERVED_FOR_TESTING_1, .offset = 0 },
        { .name = "wos_unit_test",     .type = VAR_UINT8 | MODE_STRING | MASTER_VALUE, .config = { .string = { 0, 16, STRING_FLAGS_WRITEONCE }}, .pgn = PG_RESERVED_FOR_TESTING_1, .offset = 0 },
    };
    const uint16_t valueTableEntryCount = ARRAYLEN(valueTable);
    const lookupTableEntry_t lookupTables[] = {};
    static const char *stubIdentifierName(int) { return "NONE"; }
    static bool stubIdentifierValue(const char *, int *value) { *value = SERIAL_PORT_NONE; return true; }
    static const char *stubIdentifierNameAt(unsigned index) { return index == 0 ? "NONE" : NULL; }
    const identifierLookupEntry_t identifierLookups[IDENTIFIER_LOOKUP_COUNT] = {
        [IDENTIFIER_LOOKUP_SERIAL_PORT] = { stubIdentifierName, stubIdentifierValue, stubIdentifierNameAt },
    };
    const char * const lookupTableOsdDisplayPortDevice[] = {};
    const char * const buildKey = NULL;
    const char * const releaseName = NULL;

    gyro_t gyro;

    PG_REGISTER(osdConfig_t, osdConfig, PG_OSD_CONFIG, 0);
    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER(ledStripConfig_t, ledStripConfig, PG_LED_STRIP_CONFIG, 0);
    PG_REGISTER(ledStripStatusModeConfig_t, ledStripStatusModeConfig, PG_LED_STRIP_STATUS_MODE_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(pilotConfig_t, pilotConfig, PG_PILOT_CONFIG, 0);
    PG_REGISTER_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges, PG_ADJUSTMENT_RANGE_CONFIG, 0);
    PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions, PG_MODE_ACTIVATION_PROFILE, 0);
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

#include "unittest_macros.h"
#include "gtest/gtest.h"

const bool PRINT_TEST_DATA = false;

// Verifies that cliSet correctly parses and stores an array-type setting
// with mixed positive and negative integer values.
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

// Verifies that cliSet correctly writes a string value into a string-type setting
// without any special flags (e.g. write-once protection).
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

// Verifies that a write-once string setting ignores subsequent writes after
// the initial value is set, preserving the original value on re-assignment.
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

// serial_feature_map is referenced by printSerial(); the CLI unit tests do not
// exercise it, so stubs are sufficient.
uint8_t serialDefaultPortBaud(serialBaudClass_e baudClass)
{
    (void)baudClass;
    return BAUD_115200;
}

uint8_t serialSynthesizePortBaud(serialPortIdentifier_e identifier, serialBaudClass_e baudClass)
{
    (void)identifier;
    (void)baudClass;
    return BAUD_115200;
}

uint32_t serialSynthesizeFunctionMask(serialPortIdentifier_e identifier)
{
    (void)identifier;
    return 0;
}

unsigned serialGetPortClaims(serialPortIdentifier_e identifier, serialPortClaim_t *claims, unsigned maxClaims)
{
    (void)identifier;
    (void)claims;
    (void)maxClaims;
    return 0;
}

uint32_t serialFunctionMaskFromLegacy(uint32_t legacyMask)
{
    return legacyMask;
}

bool serialApplyFunctionMask(serialPortIdentifier_e identifier, uint32_t mask)
{
    (void)identifier;
    (void)mask;
    return true;
}

void serialApplyPortBaud(serialPortIdentifier_e identifier, serialBaudClass_e baudClass, uint8_t baudIndex)
{
    (void)identifier;
    (void)baudClass;
    (void)baudIndex;
}

const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT] = {
    SERIAL_PORT_USB_VCP,
    SERIAL_PORT_USART1,
    SERIAL_PORT_USART2,
    SERIAL_PORT_USART3,
    SERIAL_PORT_UART4,
    SERIAL_PORT_UART5,
    SERIAL_PORT_SOFTSERIAL1,
    SERIAL_PORT_SOFTSERIAL2,
};

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

static const box_t boxes[] = { { "DUMMYBOX", 0, 0 } };
const box_t *findBoxByPermanentId(uint8_t) { return &boxes[0]; }
const box_t *findBoxByBoxId(boxId_e) { return &boxes[0]; }

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
bool resetEEPROM(configResetReason_e, uint16_t) { return true; }
void bufWriterFlush(bufWriter_t *) {}
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
baudRate_e lookupBaudRateIndex(uint32_t){return BAUD_9600; }
serialPortUsage_t *findSerialPortUsageByIdentifier(serialPortIdentifier_e){ return NULL; }
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) { return NULL; }
void serialPassthrough(serialPort_t *, serialPort_t *, serialConsumer *, serialConsumer *) {}
uint32_t millis(void) { return 0; }
uint8_t getBatteryCellCount(void) { return 1; }
void servoMixerLoadMix(int) {}
const char * getBatteryStateString(void){ return "_getBatteryStateString_"; }
uint32_t getCycleCounter(void) { return 0; }
uint32_t clockMicrosToCycles(uint32_t micros) { return micros; }

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
void schedulerIgnoreTaskExecTime(void) {}

const char * const targetName = "UNITTEST";
const char * const buildDate = "Jan 01 2017";
const char * const buildTime = "00:00:00";
const char * const shortGitRevision = "MASTER";

//uint32_t serialRxBytesWaiting(const serialPort_t *) {return 0;}
//uint8_t serialRead(serialPort_t *){return 0;}

// Keep the CLI output, so that tests can examine the messages.
static char cliOutput[256];
static unsigned cliOutputLen = 0;

static void cliOutputReset(void)
{
    cliOutputLen = 0;
    cliOutput[0] = '\0';
}

void bufWriterAppend(bufWriter_t *, uint8_t ch)
{
    printf("%c", ch);
    if (cliOutputLen < sizeof(cliOutput) - 1) {
        cliOutput[cliOutputLen++] = ch;
        cliOutput[cliOutputLen] = '\0';
    }
}
//void serialWriteBufShim(void *, const uint8_t *, int) {}
void bufWriterInit(bufWriter_t *, uint8_t *, int, bufWrite_t, void *) { }
//void setArmingDisabled(armingDisableFlags_e) {}

void waitForSerialPortToFinishTransmitting(serialPort_t *) {}
void systemResetToBootloader(void) {}
void resetConfig(void) {}
void systemReset(void) {}
void writeUnmodifiedConfigToEEPROM(void) {}

void changePidProfile(uint8_t) {}
void changeBatteryProfile(uint8_t) {}
uint8_t getCurrentBatteryProfileIndex(void) { return 0; }
bool serialIsPortAvailable(serialPortIdentifier_e) { return false; }
int findSerialPortIndexByIdentifier(serialPortIdentifier_e) { return -1; }
void generateLedConfig(ledConfig_t *, char *, size_t) {}
//bool isSerialTransmitBufferEmpty(const serialPort_t *) {return true; }
//void serialWrite(serialPort_t *, uint8_t ch) { printf("%c", ch);}

//void serialSetCtrlLineStateCb(serialPort_t *, void (*)(void *, uint16_t ), void *) {}
void serialSetCtrlLineStateDtrPin(serialPort_t *, ioTag_t ) {}
void serialSetCtrlLineState(serialPort_t *, uint16_t ) {}

serialPortIdentifier_e findSerialPortByName(const char* portName, int (*cmp)(const char *portName, const char *candidate))
{
    UNUSED(portName);
    UNUSED(cmp);
    return SERIAL_PORT_NONE;
}

const char* serialName(serialPortIdentifier_e identifier, const char* notFound)
{
    UNUSED(identifier);
    return notFound;
}

//void serialSetBaudRateCb(serialPort_t *, void (*)(serialPort_t *context, uint32_t baud), serialPort_t *) {}
void rescheduleTask(taskId_e, timeDelta_t){}
void schedulerSetNextStateTime(timeDelta_t ){}
char *getBoardName(void) { return NULL; }
char *getManufacturerId(void) { return NULL; }
bool boardInformationIsSet(void) { return true; }

bool setBoardName(char *newBoardName) { UNUSED(newBoardName); return true; };
bool setManufacturerId(char *newManufacturerId) { UNUSED(newManufacturerId); return true; };
bool persistBoardInformation(void) { return true; };

const char * const *sensorHardwareNames(sensorIndex_e, int *count) { if (count) *count = 0; return NULL; }
sensorIndex_e sensorIndexFromName(const char *) { return SENSOR_INDEX_COUNT; }
const char *sensorTypeName(sensorIndex_e) { return NULL; }

void activeAdjustmentRangeReset(void) {}
void analyzeModeActivationConditions(void) {}
bool isModeActivationConditionConfigured(const modeActivationCondition_t *, const modeActivationCondition_t *) { return false; }

void delay(uint32_t) {}
displayPort_t *osdGetDisplayPort(osdDisplayPortDevice_e *) { return NULL; }
const char *getMcuTypeName(void) { return targetName; }
float getCurrentRxRateHz(void) { return 0; }
uint16_t getAverageSystemLoadPercent(void) { return 0; }
bool getRxRateValid(void) { return false; }
const uint16_t *getBuildOptions(unsigned *count) { *count = 0; return NULL; }
}

// Verifies cliGetSettingByName returns "name = value" for an array setting.
TEST(CLIUnittest, TestGetSettingByNameArray)
{
    // First set the array to known values
    char *str = (char *)"array_unit_test = 10,-20,30";
    cliSet("", str);

    char buf[128];
    memset(buf, 0, sizeof(buf));
    int written = cliGetSettingByName("array_unit_test", buf, sizeof(buf));

    EXPECT_GT(written, 0);
    ASSERT_LT((size_t)written, sizeof(buf));
    buf[written] = '\0';
    EXPECT_STREQ("array_unit_test = 10,-20,30", buf);
}

// Verifies cliGetSettingByName returns "name = value" for a string setting.
TEST(CLIUnittest, TestGetSettingByNameString)
{
    char *str = (char *)"str_unit_test = HELLO";
    cliSet("", str);

    char buf[128];
    memset(buf, 0, sizeof(buf));
    int written = cliGetSettingByName("str_unit_test", buf, sizeof(buf));

    EXPECT_GT(written, 0);
    ASSERT_LT((size_t)written, sizeof(buf));
    buf[written] = '\0';
    EXPECT_STREQ("str_unit_test = HELLO", buf);
}

// Verifies cliGetSettingByName returns -1 for a nonexistent setting.
TEST(CLIUnittest, TestGetSettingByNameNotFound)
{
    char buf[128];
    int written = cliGetSettingByName("nonexistent_setting", buf, sizeof(buf));
    EXPECT_EQ(-1, written);
}

// Verifies cliGetSettingByName truncates output when buffer is small.
TEST(CLIUnittest, TestGetSettingByNameSmallBuffer)
{
    char *str = (char *)"array_unit_test = 1,2,3";
    cliSet("", str);

    char buf[10];
    memset(buf, 0, sizeof(buf));
    int written = cliGetSettingByName("array_unit_test", buf, sizeof(buf));

    EXPECT_LT(written, 0); // truncated output must return error
}

// The reply is "name = value", always longer than the name alone, so a buffer
// sized from the requested name -- all a read request carries -- can never hold
// it. MSP2_CLI_SETTING sized its scratch buffer exactly that way, so every read
// over MSP answered MSP_RESULT_ERROR while writes appeared to work (there the
// request carries the value too, so it happens to fit). Pins the caller contract.
TEST(CLIUnittest, TestGetSettingByNameRejectsBufferSizedToNameOnly)
{
    const char *name = "array_unit_test";
    char buf[128];
    memset(buf, 0, sizeof(buf));

    EXPECT_EQ(-1, cliGetSettingByName(name, buf, strlen(name) + 1));
}

// Verifies cliGetSettingByName returns 0 when buffer length is zero.
TEST(CLIUnittest, TestGetSettingByNameZeroBuffer)
{
    char buf[1];
    int written = cliGetSettingByName("array_unit_test", buf, 0);
    EXPECT_EQ(0, written);
}

#ifdef USE_MSP_CLI_COMMAND
// Reboot/mode-switch/passthrough commands must be refused: cliExecuteCommand runs them
// against a NULL-vTable port, so reaching waitForSerialPortToFinishTransmitting() would fault.
TEST(CLIUnittest, TestCliExecuteCommandRefusesDangerous)
{
    char buf[64];
    EXPECT_EQ(CLI_COMMAND_REFUSED, cliExecuteCommand("bl", buf, sizeof(buf)));
    EXPECT_EQ(CLI_COMMAND_REFUSED, cliExecuteCommand("msc", buf, sizeof(buf)));
    EXPECT_EQ(CLI_COMMAND_REFUSED, cliExecuteCommand("serialpassthrough 1", buf, sizeof(buf)));
    EXPECT_EQ(CLI_COMMAND_REFUSED, cliExecuteCommand("defaults", buf, sizeof(buf)));
    EXPECT_FALSE(cliMode);
}

// 'exit' is a no-op (never reboots), and bad arguments are rejected.
TEST(CLIUnittest, TestCliExecuteCommandGuards)
{
    char buf[64];
    EXPECT_EQ(0, cliExecuteCommand("exit", buf, sizeof(buf)));
    EXPECT_EQ(CLI_COMMAND_REFUSED, cliExecuteCommand(NULL, buf, sizeof(buf)));
    EXPECT_EQ(CLI_COMMAND_REFUSED, cliExecuteCommand("version", buf, 0));

    cliMode = true; // already in a CLI session -> must refuse to avoid clobbering state
    EXPECT_EQ(CLI_COMMAND_REFUSED, cliExecuteCommand("version", buf, sizeof(buf)));
    cliMode = false;
}

// An inline comment must not be read as the 'nosave' argument: processCharacter() strips the
// comment before dispatch, so 'defaults' would run in its saving form and reboot.
TEST(CLIUnittest, TestCliExecuteCommandRefusesDefaultsWithCommentedNosave)
{
    char buf[64];
    EXPECT_EQ(CLI_COMMAND_REFUSED, cliExecuteCommand("defaults # nosave", buf, sizeof(buf)));
    EXPECT_EQ(CLI_COMMAND_REFUSED, cliExecuteCommand("defaults #nosave", buf, sizeof(buf)));
    EXPECT_EQ(CLI_COMMAND_REFUSED, cliExecuteCommand("defaults // nosave", buf, sizeof(buf)));
    EXPECT_FALSE(cliMode);
}

// Persisting or wiping the config while armed must be refused: writeEEPROM() blocks the
// flight loop, and this path is reachable from a telemetry link in flight.
TEST(CLIUnittest, TestCliExecuteCommandRefusesWhenArmed)
{
    char buf[64];
    armingFlags |= ARMED;
    EXPECT_EQ(CLI_COMMAND_REFUSED, cliExecuteCommand("save", buf, sizeof(buf)));
    EXPECT_EQ(CLI_COMMAND_REFUSED, cliExecuteCommand("defaults nosave", buf, sizeof(buf)));
    armingFlags &= ~ARMED;
    EXPECT_FALSE(cliMode);
}

// atoi("-1") gives -1. A check of only the upper limit let 'aux -1' write before the start
// of the array. A negative index must give the index range error.
TEST(CLIUnittest, TestCliAuxRejectsNegativeIndex)
{
    static_assert(MAX_MODE_ACTIVATION_CONDITION_COUNT == 20, "update the aux index tests");
    char buf[64];

    // Without the other arguments, the command clears the entry with memset()
    cliOutputReset();
    cliExecuteCommand("aux -1", buf, sizeof(buf));
    EXPECT_NE(nullptr, strstr(cliOutput, "INDEX NOT BETWEEN"));

    cliOutputReset();
    cliExecuteCommand("aux -1 0 1 1700 2100 0 0", buf, sizeof(buf));
    EXPECT_NE(nullptr, strstr(cliOutput, "INDEX NOT BETWEEN"));

    cliOutputReset();
    cliExecuteCommand("aux 20 0 1 1700 2100 0 0", buf, sizeof(buf));
    EXPECT_NE(nullptr, strstr(cliOutput, "INDEX NOT BETWEEN"));

    // The last index is valid
    cliOutputReset();
    cliExecuteCommand("aux 19 0 1 1700 2100 0 0", buf, sizeof(buf));
    EXPECT_EQ(nullptr, strstr(cliOutput, "INDEX NOT BETWEEN"));
    EXPECT_EQ(1, modeActivationConditions(19)->auxChannelIndex);

    memset(modeActivationConditionsMutable(19), 0, sizeof(modeActivationCondition_t));
    EXPECT_FALSE(cliMode);
}

// Same problem as 'aux': 'adjrange -1' wrote before the start of the array.
TEST(CLIUnittest, TestCliAdjrangeRejectsNegativeIndex)
{
    static_assert(MAX_ADJUSTMENT_RANGE_COUNT == 30, "update the adjrange index tests");
    char buf[64];

    // Without the other arguments, the command clears the entry with memset()
    cliOutputReset();
    cliExecuteCommand("adjrange -1", buf, sizeof(buf));
    EXPECT_NE(nullptr, strstr(cliOutput, "INDEX NOT BETWEEN"));

    cliOutputReset();
    cliExecuteCommand("adjrange -1 0 1 1700 2100 1 2", buf, sizeof(buf));
    EXPECT_NE(nullptr, strstr(cliOutput, "INDEX NOT BETWEEN"));

    cliOutputReset();
    cliExecuteCommand("adjrange 30 0 1 1700 2100 1 2", buf, sizeof(buf));
    EXPECT_NE(nullptr, strstr(cliOutput, "INDEX NOT BETWEEN"));

    // The last index is valid
    cliOutputReset();
    cliExecuteCommand("adjrange 29 0 1 1700 2100 1 2", buf, sizeof(buf));
    EXPECT_EQ(nullptr, strstr(cliOutput, "INDEX NOT BETWEEN"));
    EXPECT_EQ(1, adjustmentRanges(29)->auxChannelIndex);
    EXPECT_EQ(2, adjustmentRanges(29)->auxSwitchChannelIndex);

    memset(adjustmentRangesMutable(29), 0, sizeof(adjustmentRange_t));
    EXPECT_FALSE(cliMode);
}
#endif

// Verifies cliSetSettingByName sets an array value and round-trips via get.
TEST(CLIUnittest, TestSetSettingByNameArray)
{
    bool result = cliSetSettingByName("array_unit_test = 50,-60,70");
    EXPECT_TRUE(result);

    // Verify the values were stored
    const uint16_t index = cliGetSettingIndex((char *)"array_unit_test", 15);
    EXPECT_LT(index, valueTableEntryCount);
    int8_t *data = (int8_t *)cliGetValuePointer(&valueTable[index]);
    EXPECT_EQ(50,  data[0]);
    EXPECT_EQ(-60, data[1]);
    EXPECT_EQ(70,  data[2]);
}

// Verifies cliSetSettingByName sets a string value.
TEST(CLIUnittest, TestSetSettingByNameString)
{
    bool result = cliSetSettingByName("str_unit_test = WORLD");
    EXPECT_TRUE(result);

    char buf[128];
    memset(buf, 0, sizeof(buf));
    int written = cliGetSettingByName("str_unit_test", buf, sizeof(buf));
    EXPECT_GT(written, 0);
    ASSERT_LT((size_t)written, sizeof(buf));
    buf[written] = '\0';
    EXPECT_STREQ("str_unit_test = WORLD", buf);
}

// Verifies cliSetSettingByName returns false for missing equals sign.
TEST(CLIUnittest, TestSetSettingByNameNoEquals)
{
    bool result = cliSetSettingByName("array_unit_test 1,2,3");
    EXPECT_FALSE(result);
}

// Verifies cliSetSettingByName returns false for nonexistent setting.
TEST(CLIUnittest, TestSetSettingByNameNotFound)
{
    bool result = cliSetSettingByName("nonexistent_setting = 42");
    EXPECT_FALSE(result);
}

// Verifies cliSetSettingByName rejects a partial array (fewer elements than length).
TEST(CLIUnittest, TestSetSettingByNamePartialArray)
{
    // First set to known values
    cliSetSettingByName("array_unit_test = 10,20,30");

    // Attempt partial update with only 1 of 3 elements
    bool result = cliSetSettingByName("array_unit_test = 99");
    EXPECT_FALSE(result);

    // Original values should be unchanged
    const uint16_t index = cliGetSettingIndex((char *)"array_unit_test", 15);
    EXPECT_LT(index, valueTableEntryCount);
    int8_t *data = (int8_t *)cliGetValuePointer(&valueTable[index]);
    EXPECT_EQ(10, data[0]);
    EXPECT_EQ(20, data[1]);
    EXPECT_EQ(30, data[2]);
}

// Verifies cliSetSettingByName returns false when value is empty.
TEST(CLIUnittest, TestSetSettingByNameEmptyValue)
{
    bool result = cliSetSettingByName("array_unit_test = ");
    EXPECT_FALSE(result);
}

// Verifies cliSetSettingByName handles extra whitespace around '='.
TEST(CLIUnittest, TestSetSettingByNameWhitespace)
{
    bool result = cliSetSettingByName("array_unit_test   =   7, -8 , 9");
    EXPECT_TRUE(result);

    const uint16_t index = cliGetSettingIndex((char *)"array_unit_test", 15);
    EXPECT_LT(index, valueTableEntryCount);
    int8_t *data = (int8_t *)cliGetValuePointer(&valueTable[index]);
    EXPECT_EQ(7,  data[0]);
    EXPECT_EQ(-8, data[1]);
    EXPECT_EQ(9,  data[2]);
}

// Verifies cliGetSettingInfoByName returns metadata for an array setting.
TEST(CLIUnittest, TestGetSettingInfoByNameArray)
{
    char buf[512];
    memset(buf, 0, sizeof(buf));
    int totalLen = 0;

    int written = cliGetSettingInfoByName("array_unit_test", 0, buf, sizeof(buf), &totalLen);

    EXPECT_GT(written, 0);
    EXPECT_GT(totalLen, 0);
    ASSERT_LT((size_t)written, sizeof(buf));
    buf[written] = '\0';

    // Should contain PGN, type, and length info
    EXPECT_TRUE(strstr(buf, "pgn=") != NULL);
    EXPECT_TRUE(strstr(buf, "type=int8[]") != NULL);
    EXPECT_TRUE(strstr(buf, "length=3") != NULL);
    EXPECT_TRUE(strstr(buf, "default=") != NULL);
}

// Verifies cliGetSettingInfoByName returns metadata for a string setting.
TEST(CLIUnittest, TestGetSettingInfoByNameString)
{
    char buf[512];
    memset(buf, 0, sizeof(buf));
    int totalLen = 0;

    int written = cliGetSettingInfoByName("str_unit_test", 0, buf, sizeof(buf), &totalLen);

    EXPECT_GT(written, 0);
    ASSERT_LT((size_t)written, sizeof(buf));
    buf[written] = '\0';

    EXPECT_TRUE(strstr(buf, "type=string") != NULL);
    EXPECT_TRUE(strstr(buf, "maxlength=16") != NULL);
}

// Verifies cliGetSettingInfoByName returns -1 for nonexistent setting.
TEST(CLIUnittest, TestGetSettingInfoByNameNotFound)
{
    char buf[128];
    int totalLen = 0;
    int written = cliGetSettingInfoByName("nonexistent_setting", 0, buf, sizeof(buf), &totalLen);
    EXPECT_EQ(-1, written);
}

// Verifies the windowed offset feature of cliGetSettingInfoByName.
TEST(CLIUnittest, TestGetSettingInfoByNameOffset)
{
    char fullBuf[512];
    memset(fullBuf, 0, sizeof(fullBuf));
    int totalLen = 0;
    int fullWritten = cliGetSettingInfoByName("array_unit_test", 0, fullBuf, sizeof(fullBuf), &totalLen);
    EXPECT_GT(fullWritten, 0);

    // Read with an offset of 5 bytes
    char offsetBuf[512];
    memset(offsetBuf, 0, sizeof(offsetBuf));
    int totalLen2 = 0;
    int offsetWritten = cliGetSettingInfoByName("array_unit_test", 5, offsetBuf, sizeof(offsetBuf), &totalLen2);
    EXPECT_GT(offsetWritten, 0);
    EXPECT_EQ(totalLen, totalLen2); // total size should be the same regardless of offset

    // The offset buffer should match the full buffer starting at position 5
    ASSERT_GE(offsetWritten, 0);
    ASSERT_LE(5 + offsetWritten, fullWritten);
    EXPECT_EQ(0, memcmp(fullBuf + 5, offsetBuf, offsetWritten));
}

// Verifies that cliHelp does not dereference NULL when searching commands whose
// description field is NULL (e.g. flash_read, flash_write, play_sound).
// Prior to the fix, strcasestr(NULL, cmdline) was called for these entries,
// which is undefined behaviour and causes a SIGSEGV on hosted targets.
TEST(CLIUnittest, TestCliHelpNullDescription)
{
    // "address" appears in the *args* of flash_read/flash_write but not in their names,
    // and their descriptions are NULL — this is the exact case that triggered the UB.
    cliHelp("help", (char *)"address");

    // If we reach here without a crash/SIGSEGV the null guard is working.
    SUCCEED();
}

static bool feedGpsBytes(const uint8_t *bytes, size_t count)
{
    bool published = false;
    for (size_t i = 0; i < count; ++i) {
        published |= gpsNewFrame(bytes[i]);
    }
    return published;
}

static void makeAllystarFrame(uint8_t *frame, int32_t north, int32_t east, int32_t down, uint32_t sAcc, int32_t heading = 1234567, uint32_t iTow = 45337000)
{
    memset(frame, 0, 44);
    frame[0] = 0xf1;
    frame[1] = 0xd9;
    frame[2] = 0x01;
    frame[3] = 0x12;
    frame[4] = 36;
    const uint32_t values[] = {iTow, (uint32_t)north, (uint32_t)east, (uint32_t)down, 250, 200, (uint32_t)heading, sAcc, 0};
    for (unsigned i = 0; i < 9; ++i) {
        for (unsigned j = 0; j < 4; ++j) {
            frame[6 + i * 4 + j] = values[i] >> (8 * j);
        }
    }
    uint8_t a = 0, b = 0;
    for (unsigned i = 2; i < 42; ++i) {
        a += frame[i];
        b += a;
    }
    frame[42] = a;
    frame[43] = b;
}

static bool feedTestSentence(const char *sentence)
{
    return feedGpsBytes((const uint8_t *)sentence, strlen(sentence));
}

static void resetAllystarTest(bool enabled)
{
    gpsConfigMutable()->provider = GPS_NMEA;
    gpsConfigMutable()->gps_nmea_allystar = enabled;
    gpsConfigMutable()->gps_uart = SERIAL_PORT_NONE;
    gpsInit();
    memset(&gpsSol, 0, sizeof(gpsSol));
    gpsData.now = 100;
    gpsData.lastNavSolTs = enabled ? 45318900 : 8900;
    if (enabled) {
        uint8_t timeFrame[24] = {0xf1, 0xd9, 0x01, 0x05, 16, 0};
        timeFrame[6 + 10] = 18;
        timeFrame[6 + 1] = 6; // Valid reference GPS time and leap seconds.
        uint8_t a = 0, b = 0;
        for (unsigned i = 2; i < 22; ++i) {
            a += timeFrame[i];
            b += a;
        }
        timeFrame[22] = a;
        timeFrame[23] = b;
        feedGpsBytes(timeFrame, sizeof(timeFrame));
    }
}

static std::string nmeaSentence(const std::string &body);

TEST(CLIUnittest, AllystarGgaWaitsForMatchingVelocity)
{
    resetAllystarTest(true);
    EXPECT_FALSE(feedTestSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,*4F\r\n"));
}

TEST(CLIUnittest, AllystarFreshVelnedPublishesGgaWithNedAndAccuracy)
{
    resetAllystarTest(true);
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, -67, 17);
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
    EXPECT_TRUE(feedTestSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,*4F\r\n"));
    EXPECT_EQ(123, gpsSol.velned.velN);
    EXPECT_EQ(-45, gpsSol.velned.velE);
    EXPECT_EQ(-67, gpsSol.velned.velD);
    EXPECT_TRUE(gpsSol.velDValid);
    EXPECT_EQ(170u, gpsSol.acc.sAcc);
    EXPECT_FALSE(feedTestSentence("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n"));
    EXPECT_EQ(170u, gpsSol.acc.sAcc);
}

TEST(CLIUnittest, AllystarUsesVelocityImmediatelyBeforeGgaAtDifferentRates)
{
    for (const unsigned intervalMs : {1000U, 500U, 333U, 200U, 100U}) {
        resetAllystarTest(true);
        EXPECT_FALSE(feedTestSentence(nmeaSentence("GPGGA,123518.00,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,").c_str()));
        gpsData.now += 50;
        ASSERT_TRUE(gpsNmeaFlushPending());
        gpsData.now += intervalMs - 1;
        uint8_t frame[44];
        makeAllystarFrame(frame, 123, -45, -67, 17, 1234567, 45336000 + intervalMs);
        EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
        gpsData.now++;
        char timestamp[16];
        snprintf(timestamp, sizeof(timestamp), "1235%02u.%03u", 18U + intervalMs / 1000U, intervalMs % 1000U);
        const auto gga = nmeaSentence(std::string("GPGGA,") + timestamp + ",4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,");
        ASSERT_TRUE(feedTestSentence(gga.c_str()));
        EXPECT_FALSE(gpsSol.velNEInvalid) << "interval=" << intervalMs;
        EXPECT_TRUE(gpsSol.velDValid) << "interval=" << intervalMs;
        EXPECT_EQ(123, gpsSol.velned.velN) << "interval=" << intervalMs;
        EXPECT_EQ(intervalMs, gpsSol.navIntervalMs);
    }
}

TEST(CLIUnittest, AllystarSignedHeadingWrapsToCompassCourse)
{
    struct Case { int32_t heading; uint16_t course; };
    for (const Case &test : {Case{-9000000, 2700}, Case{-18000000, 1800},
                             Case{9000000, 900}, Case{27000000, 2700}, Case{36000000, 0}}) {
        resetAllystarTest(true);
        uint8_t frame[44];
        makeAllystarFrame(frame, 0, -200, 0, 17, test.heading);
        EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
        ASSERT_TRUE(feedTestSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,*4F\r\n"));
        EXPECT_EQ(test.course, gpsSol.groundCourse) << "heading=" << test.heading;
    }
}

TEST(CLIUnittest, AllystarStaleFrameFallsBackToRmcWithoutVerticalAccuracy)
{
    resetAllystarTest(true);
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, 67, 17);
    feedGpsBytes(frame, sizeof(frame));
    gpsData.now += 251;
    EXPECT_FALSE(feedTestSentence("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n"));
    EXPECT_FALSE(feedTestSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,*4F\r\n"));
    gpsData.now += 50;
    ASSERT_TRUE(gpsNmeaFlushPending());
    EXPECT_FALSE(gpsSol.velDValid);
    EXPECT_EQ(0, gpsSol.velned.velD);
    EXPECT_EQ(UINT32_MAX, gpsSol.acc.sAcc);
    EXPECT_NE(0, gpsSol.velned.velN);
}

TEST(CLIUnittest, GenericNmeaDoesNotDecodeAllystar)
{
    resetAllystarTest(false);
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, 67, 17);
    feedGpsBytes(frame, sizeof(frame));
    EXPECT_TRUE(feedTestSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,*4F\r\n"));
    EXPECT_EQ(0u, gpsSol.acc.sAcc);
    EXPECT_FALSE(gpsSol.velDValid);
}


static std::string nmeaSentence(const std::string &body)
{
    unsigned checksum = 0;
    for (unsigned char c : body) checksum ^= c;
    char suffix[8];
    snprintf(suffix, sizeof(suffix), "*%02X\r\n", checksum);
    return "$" + body + suffix;
}

static std::string nmeaGga(const char *quality = "4", const char *sats = "30")
{
    return nmeaSentence(std::string("GNGGA,123519.00,4807.038,N,01131.000,E,") + quality + "," + sats + ",1.0,545.4,M,46.9,M,,");
}

TEST(CLIUnittest, NmeaGgaMustBeValidatedBeforeChangingFix)
{
    resetAllystarTest(false);
    DISABLE_STATE(GPS_FIX);
    auto valid = nmeaGga();
    ASSERT_TRUE(feedTestSentence(valid.c_str()));
    ASSERT_TRUE(STATE(GPS_FIX));
    ASSERT_EQ(30, gpsSol.numSat);
    auto invalid = nmeaGga("0", "00");
    invalid[invalid.size() - 4] = invalid[invalid.size() - 4] == '0' ? '1' : '0';
    EXPECT_FALSE(feedTestSentence(invalid.c_str()));
    EXPECT_TRUE(STATE(GPS_FIX));
    EXPECT_EQ(30, gpsSol.numSat);
    EXPECT_FALSE(feedTestSentence("$GNGGA,123519.00,4807.038,N,01131.000,E,0,"));
    EXPECT_TRUE(STATE(GPS_FIX));
    EXPECT_TRUE(feedTestSentence(nmeaGga("0", "00").c_str()));
    EXPECT_FALSE(STATE(GPS_FIX));
    EXPECT_EQ(0, gpsSol.numSat);
}

TEST(CLIUnittest, NmeaScalesFieldsWithMissingDecimalDigits)
{
    resetAllystarTest(false);
    ASSERT_TRUE(feedTestSentence(nmeaSentence("GPGGA,123519.00,4807.038,N,01131.000,E,1,08,1.0,545,M,46.9,M,,").c_str()));
    EXPECT_EQ(54500, gpsSol.llh.altCm);
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPRMC,123519.00,A,4807.038,N,01131.000,E,22,84,230394,003.1,W").c_str()));
    EXPECT_EQ(1131, gpsSol.groundSpeed);
    EXPECT_EQ(840, gpsSol.groundCourse);
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.8,1,1.50").c_str()));
    EXPECT_EQ(180, gpsSol.dop.pdop);
    EXPECT_EQ(100, gpsSol.dop.hdop);
    EXPECT_EQ(150, gpsSol.dop.vdop);
    ASSERT_TRUE(feedTestSentence(nmeaSentence("GPGGA,123519.00,4807.038,N,01131.000,E,1,08,1.0,-12,M,46.9,M,,").c_str()));
    EXPECT_EQ(-1200, gpsSol.llh.altCm);
    ASSERT_TRUE(feedTestSentence(nmeaSentence("GPGGA,123519.00,4807.038,N,01131.000,E,1,08,1.0,545.49,M,46.9,M,,").c_str()));
    EXPECT_EQ(54540, gpsSol.llh.altCm);
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPRMC,123519.00,A,4807.038,N,01131.000,E,22.49,84.49,230394,003.1,W").c_str()));
    EXPECT_EQ(1152, gpsSol.groundSpeed);
    EXPECT_EQ(844, gpsSol.groundCourse);
}

TEST(CLIUnittest, NmeaRejectsAltitudeAndDopThatOverflowStorage)
{
    resetAllystarTest(false);
    ASSERT_TRUE(feedTestSentence(nmeaGga().c_str()));
    ASSERT_EQ(54540, gpsSol.llh.altCm);
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPGGA,123519.00,4807.038,N,01131.000,E,1,08,1.0,42949673,M,46.9,M,,").c_str()));
    EXPECT_EQ(54540, gpsSol.llh.altCm);
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPGGA,123519.00,4807.038,N,01131.000,E,1,08,1.0,-21474837,M,46.9,M,,").c_str()));
    EXPECT_EQ(54540, gpsSol.llh.altCm);
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.80,1.00,1.50").c_str()));
    ASSERT_EQ(180, gpsSol.dop.pdop);
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,42949673,1,1.50").c_str()));
    EXPECT_EQ(180, gpsSol.dop.pdop);
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,655.35,1,1.50").c_str()));
    ASSERT_EQ(UINT16_MAX, gpsSol.dop.pdop);
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,655.36,1,1.50").c_str()));
    EXPECT_EQ(UINT16_MAX, gpsSol.dop.pdop);
    ASSERT_TRUE(feedTestSentence(nmeaSentence("GPGGA,123519.00,4807.038,N,01131.000,E,1,08,1.0,-21474836.4,M,46.9,M,,").c_str()));
    EXPECT_EQ(-2147483640, gpsSol.llh.altCm);
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPGGA,123519.00,4807.038,N,01131.000,E,1,08,1.0,-21474836.5,M,46.9,M,,").c_str()));
    EXPECT_EQ(-2147483640, gpsSol.llh.altCm);
}

TEST(CLIUnittest, NmeaRecoversAfterInterruptedChecksum)
{
    resetAllystarTest(false);
    EXPECT_FALSE(feedTestSentence("$GNGGA,12*"));
    EXPECT_TRUE(feedTestSentence(nmeaGga().c_str()));
    EXPECT_TRUE(STATE(GPS_FIX));
    EXPECT_EQ(30, gpsSol.numSat);
}

TEST(CLIUnittest, NmeaRejectsIncompleteAndOversizeFields)
{
    resetAllystarTest(false);
    ASSERT_TRUE(feedTestSentence(nmeaGga().c_str()));
    const int32_t altitude = gpsSol.llh.altCm;
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GNGGA,123519.00,4807.038,N,01131.000,E,0").c_str()));
    EXPECT_TRUE(STATE(GPS_FIX));
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GNGGA,123519.000000000000000000,4807.038,N,01131.000,E,0,00,1.0,1.0,M,0,M,,").c_str()));
    EXPECT_TRUE(STATE(GPS_FIX));
    EXPECT_EQ(altitude, gpsSol.llh.altCm);
    auto missing = nmeaSentence("GNGGA,123519.00,,,,,4,30,1.0,,M,0,M,,");
    EXPECT_FALSE(feedTestSentence(missing.c_str()));
    EXPECT_TRUE(feedTestSentence(nmeaGga().c_str()));
}

TEST(CLIUnittest, NmeaRejectsMalformedChecksumAndRecovers)
{
    resetAllystarTest(false);
    ASSERT_TRUE(feedTestSentence(nmeaGga().c_str()));
    for (const char *tail : {"*0\r\n", "*GG\r\n", "*000\r\n", "*00,\r\n"}) {
        auto bad = std::string("$GNGGA,123519.00,4807.038,N,01131.000,E,0,00,1.0,545.4,M,46.9,M,,") + tail;
        EXPECT_FALSE(feedTestSentence(bad.c_str()));
        EXPECT_TRUE(STATE(GPS_FIX));
    }
    EXPECT_TRUE(feedTestSentence(nmeaGga().c_str()));
}

TEST(CLIUnittest, NmeaRejectedFieldsCannotLeakIntoNextSentence)
{
    resetAllystarTest(false);
    ASSERT_TRUE(feedTestSentence(nmeaGga().c_str()));
    auto bad = nmeaSentence("GPGGA,123519.00,4807.038,N,01131.000,E,4,12,1.0,999.9,M,0,M,,");
    bad[bad.size()-4] = bad[bad.size()-4] == '0' ? '1' : '0';
    EXPECT_FALSE(feedTestSentence(bad.c_str()));
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPGGA,123519.00,4807.038,N,01131.000,E,4,30,1.0").c_str()));
    EXPECT_EQ(54540, gpsSol.llh.altCm);
    auto rmc = nmeaSentence("GPRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
    EXPECT_FALSE(feedTestSentence(rmc.c_str()));
    EXPECT_EQ(844, gpsSol.groundCourse);
    auto invalid = nmeaSentence("GPRMC,123519.00,A,4807.038,N,01131.000,E,099.9,123.4,230394,003.1,W");
    invalid[invalid.size()-4] = invalid[invalid.size()-4] == '0' ? '1' : '0';
    EXPECT_FALSE(feedTestSentence(invalid.c_str()));
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPRMC,123519.00,A,4807.038,N,01131.000,E,001.0").c_str()));
    EXPECT_EQ(844, gpsSol.groundCourse);
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPRMC,123520.00,A,4807.038,N,01131.000,E,010.0,045.0,230394,003.1,W").c_str()));
    EXPECT_EQ(514, gpsSol.groundSpeed);
    EXPECT_EQ(450, gpsSol.groundCourse);
    EXPECT_TRUE(feedTestSentence(nmeaSentence("GPGGA,123520.00,4907.038,N,01231.000,E,4,28,1.0,123.4,M,0,M,,").c_str()));
    EXPECT_EQ(491173000, gpsSol.llh.lat);
    EXPECT_EQ(12340, gpsSol.llh.altCm);
    EXPECT_EQ(28, gpsSol.numSat);
}


TEST(CLIUnittest, NmeaInterleavedMessagesCommitOnlyAfterValidation)
{
    resetAllystarTest(false);
    auto gga = nmeaSentence("GPGGA,123519.00,4807.038,N,01131.000,E,4,30,1.0,545.4,M,46.9,M,,");
    ASSERT_TRUE(feedTestSentence(gga.c_str()));
    EXPECT_EQ(481173000, gpsSol.llh.lat);
    EXPECT_EQ(115166666, gpsSol.llh.lon);
    EXPECT_EQ(54540, gpsSol.llh.altCm);
    auto rmc = nmeaSentence("GNRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
    EXPECT_FALSE(feedTestSentence(rmc.c_str()));
    EXPECT_EQ(1152, gpsSol.groundSpeed);
    EXPECT_EQ(844, gpsSol.groundCourse);
    auto gsa = nmeaSentence("GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.80,1.00,1.50");
    EXPECT_FALSE(feedTestSentence(gsa.c_str()));
    EXPECT_EQ(180, gpsSol.dop.pdop);
    EXPECT_EQ(100, gpsSol.dop.hdop);
    EXPECT_EQ(150, gpsSol.dop.vdop);
    auto gsv = nmeaSentence("GPGSV,1,1,02,01,45,123,40,02,30,100,35");
    EXPECT_FALSE(feedTestSentence(gsv.c_str()));
    EXPECT_EQ(2, GPS_numCh);
    EXPECT_EQ(1, GPS_svinfo[0].svid);
    EXPECT_EQ(40, GPS_svinfo[0].cno);
    auto bad = nmeaSentence("GPGSV,1,1,01,09,45,123,50");
    bad[bad.size()-4] = bad[bad.size()-4] == '0' ? '1' : '0';
    EXPECT_FALSE(feedTestSentence(bad.c_str()));
    EXPECT_EQ(2, GPS_numCh);
    EXPECT_EQ(1, GPS_svinfo[0].svid);
    EXPECT_EQ(40, GPS_svinfo[0].cno);
}

TEST(CLIUnittest, NmeaRejectsInvalidRequiredFieldValues)
{
    resetAllystarTest(false);
    ASSERT_TRUE(feedTestSentence(nmeaGga().c_str()));
    const auto lat = gpsSol.llh.lat;
    for (const char *body : {
        "GNGGA,12x519.00,4807.038,N,01131.000,E,4,30,1.0,545.4,M,46.9,M,,",
        "GNGGA,123519.00,broken,N,01131.000,E,4,30,1.0,545.4,M,46.9,M,,",
        "GNGGA,123519.00,4807.038,X,01131.000,E,4,30,1.0,545.4,M,46.9,M,,",
        "GNGGA,123519.00,4807.038,N,01131.000,E,4,XX,1.0,545.4,M,46.9,M,,",
        "GNGGA,123519.00,4807.038,N,01131.000,E,4,30,1.0,bad,M,46.9,M,,",
        "GNGGA,123519.00,4807.038,N,01131.000,E,4,30,1.0,123.,M,46.9,M,,",
        "GNGGA,123519.00,9999.999,N,01131.000,E,4,30,1.0,545.4,M,46.9,M,,",
        "GNGGA,256099.00,4807.038,N,01131.000,E,4,30,1.0,545.4,M,46.9,M,,"
    }) {
        EXPECT_FALSE(feedTestSentence(nmeaSentence(body).c_str()));
        EXPECT_TRUE(STATE(GPS_FIX));
        EXPECT_EQ(lat, gpsSol.llh.lat);
    }
    EXPECT_TRUE(feedTestSentence(nmeaGga().c_str()));
}

TEST(CLIUnittest, NmeaBurstRecoversAfterRepeatedDamage)
{
    resetAllystarTest(false);
    const auto good = nmeaGga();
    const auto rmc = nmeaSentence("GPRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
    unsigned publications = 0;
    for (unsigned i = 0; i < 2000; i++) {
        EXPECT_FALSE(feedTestSentence("$GNGGA,123519.00,4807.038,N,01131.000,E,0,*"));
        publications += feedTestSentence(good.c_str());
        feedTestSentence(rmc.c_str());
        EXPECT_TRUE(STATE(GPS_FIX));
    }
    EXPECT_EQ(2000u, publications);
    EXPECT_EQ(30, gpsSol.numSat);
    EXPECT_EQ(844, gpsSol.groundCourse);
}

TEST(CLIUnittest, NmeaRejectsMalformedRmcAndGsvFields)
{
    resetAllystarTest(false);
    feedTestSentence(nmeaSentence("GPRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W").c_str());
    ASSERT_EQ(844, gpsSol.groundCourse);
    feedTestSentence(nmeaSentence("GPRMC,123519.00,X,4807.038,N,01131.000,E,022.4,123.4,230394,003.1,W").c_str());
    EXPECT_EQ(844, gpsSol.groundCourse);
    feedTestSentence(nmeaSentence("GPGSV,1,1,02,01,45,123,40,02,30,100,35").c_str());
    ASSERT_EQ(2, GPS_numCh);
    feedTestSentence(nmeaSentence("GPGSV,1,1,xx,01,45,123,40").c_str());
    EXPECT_EQ(2, GPS_numCh);
    feedTestSentence(nmeaSentence("GPGSV,1,1,,").c_str());
    EXPECT_EQ(2, GPS_numCh);
}

TEST(CLIUnittest, AllystarRmcBeforeGgaPublishesOnceWithCurrentPosition)
{
    resetAllystarTest(true);
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, 67, 17);
    feedGpsBytes(frame, sizeof(frame));
    EXPECT_FALSE(feedTestSentence("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n"));
    EXPECT_TRUE(feedTestSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,*4F\r\n"));
    EXPECT_EQ(123, gpsSol.velned.velN);
}

TEST(CLIUnittest, AllystarInvalidRmcDoesNotPublishFallback)
{
    resetAllystarTest(true);
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPRMC,123519,V,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W").c_str()));
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GNGGA,123519,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,").c_str()));
    gpsData.now += 50;
    ASSERT_TRUE(gpsNmeaFlushPending());
}

TEST(CLIUnittest, AllystarFallbackClearsStale3dSpeed)
{
    resetAllystarTest(true);
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, 67, 17);
    feedGpsBytes(frame, sizeof(frame));
    ASSERT_TRUE(feedTestSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,*4F\r\n"));
    ASSERT_EQ(250, gpsSol.speed3d);
    feedTestSentence("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    gpsData.now += 251;
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPRMC,123520,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W").c_str()));
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPGGA,123520,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,").c_str()));
    gpsData.now += 50;
    ASSERT_TRUE(gpsNmeaFlushPending());
    EXPECT_EQ(gpsSol.groundSpeed, gpsSol.speed3d);
    EXPECT_FALSE(gpsSol.velDValid);
}

TEST(CLIUnittest, AllystarGgaWithoutVelocityPublishesAndMarksRetainedSpeedInvalid)
{
    resetAllystarTest(true);
    gpsSol.speed3d = 1234;
    gpsSol.velned.velN = 4321;
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GNGGA,123519,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,").c_str()));
    gpsData.now += 50;
    ASSERT_TRUE(gpsNmeaFlushPending());
    EXPECT_EQ(1234, gpsSol.speed3d);
    EXPECT_TRUE(gpsSol.velNEInvalid);
    EXPECT_FALSE(gpsSol.velDValid);
    EXPECT_TRUE(feedTestSentence(nmeaSentence("GNGGA,123520,,,,,0,00,1.0,,M,0,M,,").c_str()));
    EXPECT_FALSE(STATE(GPS_FIX));
}

TEST(CLIUnittest, AllystarDoesNotFuseRecentRmcFromDifferentEpoch)
{
    resetAllystarTest(true);
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W").c_str()));
    gpsData.now += 100;
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPGGA,123520,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,").c_str()));
    gpsData.now += 50;
    ASSERT_TRUE(gpsNmeaFlushPending());
    EXPECT_TRUE(gpsSol.velNEInvalid);
    EXPECT_FALSE(gpsSol.velDValid);
}

TEST(CLIUnittest, AllystarRmcFallbackScalesIntegerSpeedAndCourse)
{
    resetAllystarTest(true);
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPRMC,123519.00,A,4807.038,N,01131.000,E,22,84,230394,003.1,W").c_str()));
    ASSERT_FALSE(feedTestSentence(nmeaSentence("GPGGA,123519.00,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,").c_str()));
    gpsData.now += 50;
    ASSERT_TRUE(gpsNmeaFlushPending());
    EXPECT_FALSE(gpsSol.velNEInvalid);
    EXPECT_EQ(1131, gpsSol.groundSpeed);
    EXPECT_EQ(840, gpsSol.groundCourse);
    EXPECT_NEAR(118, gpsSol.velned.velN, 3);
    EXPECT_NEAR(1125, gpsSol.velned.velE, 3);
}

TEST(CLIUnittest, AllystarDoesNotFuseOneSecondOldRmc)
{
    resetAllystarTest(true);
    gpsData.lastNavSolTs = 0;
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPGGA,123518,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,").c_str()));
    gpsData.now += 50;
    ASSERT_TRUE(gpsNmeaFlushPending());
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPRMC,123518,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W").c_str()));
    gpsData.now += 1000;
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPGGA,123519,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,").c_str()));
    gpsData.now += 50;
    ASSERT_TRUE(gpsNmeaFlushPending());
    EXPECT_EQ(1000, gpsSol.navIntervalMs);
    EXPECT_TRUE(gpsSol.velNEInvalid);
}

TEST(CLIUnittest, AllystarDoesNotFuseOneSecondOldVelned)
{
    resetAllystarTest(true);
    gpsData.lastNavSolTs = 0;
    ASSERT_FALSE(feedTestSentence(nmeaSentence("GPGGA,123518,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,").c_str()));
    gpsData.now += 50;
    ASSERT_TRUE(gpsNmeaFlushPending());
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, 67, 17);
    feedGpsBytes(frame, sizeof(frame));
    gpsData.now += 1000;
    EXPECT_FALSE(feedTestSentence(nmeaSentence("GPGGA,123519,4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,").c_str()));
    gpsData.now += 50;
    ASSERT_TRUE(gpsNmeaFlushPending());
    EXPECT_FALSE(gpsSol.velDValid);
    EXPECT_TRUE(gpsSol.velNEInvalid);
}

static std::string allystarGgaAt(const char *epoch)
{
    return nmeaSentence(std::string("GPGGA,") + epoch + ",4807.038,N,01131.000,E,1,08,1.0,545.4,M,46.9,M,,");
}

static std::string allystarRmcAt(const char *epoch)
{
    return nmeaSentence(std::string("GPRMC,") + epoch + ",A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
}

TEST(CLIUnittest, AllystarAllSentenceOrdersPreferMatchingVelnedAndPublishOnce)
{
    const unsigned orders[][3] = {{0,1,2}, {0,2,1}, {1,0,2}, {1,2,0}, {2,0,1}, {2,1,0}};
    for (const auto &order : orders) {
        resetAllystarTest(true);
        uint8_t frame[44];
        makeAllystarFrame(frame, 123, -45, -67, 17);
        unsigned publications = 0;
        for (unsigned item : order) {
            if (item == 0) publications += feedTestSentence(allystarGgaAt("123519").c_str());
            if (item == 1) publications += feedTestSentence(allystarRmcAt("123519").c_str());
            if (item == 2) publications += feedGpsBytes(frame, sizeof(frame));
            gpsData.now += 10;
        }
        EXPECT_EQ(1u, publications);
        EXPECT_EQ(123, gpsSol.velned.velN);
        EXPECT_EQ(-67, gpsSol.velned.velD);
        EXPECT_EQ(170u, gpsSol.acc.sAcc);
        gpsData.now += 50;
        EXPECT_FALSE(gpsNmeaFlushPending());
        EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
        EXPECT_FALSE(feedTestSentence(allystarGgaAt("123519").c_str()));
    }
}

TEST(CLIUnittest, AllystarMatchingRmcFallbackWaitsForDeadlineInEitherOrder)
{
    for (bool ggaFirst : {false, true}) {
        resetAllystarTest(true);
        const auto gga = allystarGgaAt("123519");
        const auto rmc = allystarRmcAt("123519");
        EXPECT_FALSE(feedTestSentence((ggaFirst ? gga : rmc).c_str()));
        EXPECT_FALSE(feedTestSentence((ggaFirst ? rmc : gga).c_str()));
        gpsData.now += 49;
        EXPECT_FALSE(gpsNmeaFlushPending());
        gpsData.now++;
        ASSERT_TRUE(gpsNmeaFlushPending());
        EXPECT_FALSE(gpsSol.velNEInvalid);
        EXPECT_FALSE(gpsSol.velDValid);
        EXPECT_EQ(844, gpsSol.groundCourse);
        EXPECT_EQ(UINT32_MAX, gpsSol.acc.sAcc);
        EXPECT_FALSE(feedTestSentence(rmc.c_str()));
        EXPECT_FALSE(feedTestSentence(gga.c_str()));
    }
}

TEST(CLIUnittest, AllystarFinalVelnedChecksumBytePublishesPendingGga)
{
    resetAllystarTest(true);
    EXPECT_FALSE(feedTestSentence(allystarGgaAt("123519").c_str()));
    gpsData.now += 49;
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, -67, 17);
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame) - 1));
    EXPECT_TRUE(gpsNewFrame(frame[sizeof(frame) - 1]));
    EXPECT_EQ(123, gpsSol.velned.velN);
    EXPECT_TRUE(gpsSol.velDValid);
}

TEST(CLIUnittest, AllystarPreservesMillisecondNavigationInterval)
{
    resetAllystarTest(true);
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, -67, 17, 1234567, 45337000);
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
    ASSERT_TRUE(feedTestSentence(allystarGgaAt("123519.000").c_str()));
    gpsData.now += 333;
    makeAllystarFrame(frame, 123, -45, -67, 17, 1234567, 45337333);
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
    ASSERT_TRUE(feedTestSentence(allystarGgaAt("123519.333").c_str()));
    EXPECT_EQ(333, gpsSol.navIntervalMs);
}

TEST(CLIUnittest, AllystarCentisecondGgaMatchesMillisecondVelned)
{
    resetAllystarTest(true);
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, -67, 17, 1234567, 45337333);
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
    ASSERT_TRUE(feedTestSentence(allystarGgaAt("123519.33").c_str()));
    EXPECT_EQ(123, gpsSol.velned.velN);
    EXPECT_TRUE(gpsSol.velDValid);
}

TEST(CLIUnittest, AllystarCentisecondGgaMatchesMillisecondRmcFallback)
{
    resetAllystarTest(true);
    EXPECT_FALSE(feedTestSentence(allystarRmcAt("123519.333").c_str()));
    EXPECT_FALSE(feedTestSentence(allystarGgaAt("123519.33").c_str()));
    gpsData.now += 50;
    ASSERT_TRUE(gpsNmeaFlushPending());
    EXPECT_FALSE(gpsSol.velNEInvalid);
    EXPECT_EQ(844, gpsSol.groundCourse);
    EXPECT_FALSE(gpsSol.velDValid);
}

TEST(CLIUnittest, AllystarMillisecondGgaRequiresExactVelnedEpoch)
{
    resetAllystarTest(true);
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, -67, 17, 1234567, 45337334);
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
    EXPECT_FALSE(feedTestSentence(allystarGgaAt("123519.333").c_str()));
    gpsData.now += 50;
    ASSERT_TRUE(gpsNmeaFlushPending());
    EXPECT_TRUE(gpsSol.velNEInvalid);
    EXPECT_FALSE(gpsSol.velDValid);
}

TEST(CLIUnittest, AllystarCentisecondGgaRejectsPrevious100msVelocityEpoch)
{
    resetAllystarTest(true);
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, -67, 17, 1234567, 45337233);
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
    EXPECT_FALSE(feedTestSentence(allystarRmcAt("123519.233").c_str()));
    EXPECT_FALSE(feedTestSentence(allystarGgaAt("123519.33").c_str()));
    gpsData.now += 50;
    ASSERT_TRUE(gpsNmeaFlushPending());
    EXPECT_TRUE(gpsSol.velNEInvalid);
    EXPECT_FALSE(gpsSol.velDValid);
}

TEST(CLIUnittest, AllystarPendingGgaExpiresWithoutMoreSerialBytes)
{
    resetAllystarTest(true);
    EXPECT_FALSE(feedTestSentence(allystarGgaAt("123519").c_str()));
    gpsData.now += 49;
    EXPECT_FALSE(gpsNmeaFlushPending());
    gpsData.now++;
    EXPECT_TRUE(gpsNmeaFlushPending());
    EXPECT_TRUE(gpsSol.velNEInvalid);
    EXPECT_FALSE(gpsNmeaFlushPending());
}

TEST(CLIUnittest, AllystarLateMatchingVelnedDoesNotRepublishExpiredEpoch)
{
    resetAllystarTest(true);
    EXPECT_FALSE(feedTestSentence(allystarGgaAt("123519").c_str()));
    gpsData.now += 50;
    ASSERT_TRUE(gpsNmeaFlushPending());
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, -67, 17);
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
    EXPECT_TRUE(gpsSol.velNEInvalid);
    EXPECT_FALSE(feedTestSentence(allystarGgaAt("123519").c_str()));
    gpsData.now += 50;
    EXPECT_FALSE(gpsNmeaFlushPending());
}

TEST(CLIUnittest, AllystarRecentWrongEpochVelnedDoesNotMatch)
{
    resetAllystarTest(true);
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, -67, 17, 1234567, 45336999);
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
    EXPECT_FALSE(feedTestSentence(allystarGgaAt("123519").c_str()));
    gpsData.now += 50;
    EXPECT_TRUE(gpsNmeaFlushPending());
    EXPECT_TRUE(gpsSol.velNEInvalid);
}

TEST(CLIUnittest, AllystarCachedVelocityFreshnessIncludes250Milliseconds)
{
    for (unsigned age : {250U, 251U}) {
        resetAllystarTest(true);
        uint8_t frame[44];
        makeAllystarFrame(frame, 123, -45, -67, 17);
        EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
        gpsData.now += age;
        EXPECT_EQ(age == 250, feedTestSentence(allystarGgaAt("123519").c_str()));
        if (age > 250) {
            gpsData.now += 50;
            ASSERT_TRUE(gpsNmeaFlushPending());
            EXPECT_TRUE(gpsSol.velNEInvalid);
        } else {
            EXPECT_TRUE(gpsSol.velDValid);
        }
    }
}

TEST(CLIUnittest, AllystarFixLossImmediatelyClearsPendingPosition)
{
    resetAllystarTest(true);
    EXPECT_FALSE(feedTestSentence(allystarGgaAt("123519").c_str()));
    EXPECT_TRUE(feedTestSentence(nmeaSentence("GPGGA,123520,,,,,0,00,1.0,,M,0,M,,").c_str()));
    EXPECT_FALSE(STATE(GPS_FIX));
    gpsData.now += 50;
    EXPECT_FALSE(gpsNmeaFlushPending());
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, -67, 17);
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
    EXPECT_FALSE(STATE(GPS_FIX));
}

TEST(CLIUnittest, AllystarFixLossKeepsEpochWatermarkAndAcceptsNewerFix)
{
    for (uint32_t delayedOffset : {0U, 50U}) {
        resetAllystarTest(true);
        uint8_t frame[44];
        makeAllystarFrame(frame, 123, -45, -67, 17);
        EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
        ASSERT_TRUE(feedTestSentence(allystarGgaAt("123519.000").c_str()));
        ASSERT_TRUE(STATE(GPS_FIX));
        gpsData.now += 100;
        ASSERT_TRUE(feedTestSentence(nmeaSentence("GPGGA,123519.100,,,,,0,00,1.0,,M,0,M,,").c_str()));
        ASSERT_FALSE(STATE(GPS_FIX));
        makeAllystarFrame(frame, 456, -45, -67, 17, 1234567, 45337000 + delayedOffset);
        EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
        EXPECT_FALSE(feedTestSentence(allystarGgaAt(delayedOffset ? "123519.050" : "123519.000").c_str()));
        EXPECT_FALSE(STATE(GPS_FIX));
        gpsData.now += 50;
        EXPECT_FALSE(gpsNmeaFlushPending());
        EXPECT_FALSE(STATE(GPS_FIX));
        makeAllystarFrame(frame, 789, -45, -67, 17, 1234567, 45337200);
        EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
        ASSERT_TRUE(feedTestSentence(allystarGgaAt("123519.200").c_str()));
        EXPECT_TRUE(STATE(GPS_FIX));
        EXPECT_EQ(789, gpsSol.velned.velN);
    }
}

TEST(CLIUnittest, AllystarUntimedFixLossKeepsPriorEpochWatermark)
{
    resetAllystarTest(true);
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, -67, 17);
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
    ASSERT_TRUE(feedTestSentence(allystarGgaAt("123519.000").c_str()));
    ASSERT_TRUE(feedTestSentence(nmeaSentence("GNGGA,,,,,,0,,,,,,,,").c_str()));
    ASSERT_FALSE(STATE(GPS_FIX));
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
    EXPECT_FALSE(feedTestSentence(allystarGgaAt("123519.000").c_str()));
    gpsData.now += 50;
    EXPECT_FALSE(gpsNmeaFlushPending());
    EXPECT_FALSE(STATE(GPS_FIX));
}

TEST(CLIUnittest, AllystarEpochWatermarkExpiresAfterReceiverTimeout)
{
    resetAllystarTest(true);
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, -67, 17);
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
    ASSERT_TRUE(feedTestSentence(allystarGgaAt("123519.000").c_str()));
    ASSERT_TRUE(feedTestSentence(nmeaSentence("GPGGA,123519.100,,,,,0,00,1.0,,M,0,M,,").c_str()));
    gpsData.now += 2501; // GPS_TIMEOUT_MS is 2500.
    makeAllystarFrame(frame, 456, -45, -67, 17, 1234567, 45336000);
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
    ASSERT_TRUE(feedTestSentence(allystarGgaAt("123518.000").c_str()));
    EXPECT_TRUE(STATE(GPS_FIX));
    EXPECT_EQ(456, gpsSol.velned.velN);
}

TEST(CLIUnittest, AllystarMidnightRollsForwardAndRejectsPriorEpoch)
{
    resetAllystarTest(true);
    uint8_t frame[44];
    makeAllystarFrame(frame, 123, -45, -67, 17, 1234567, 86417900);
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
    EXPECT_TRUE(feedTestSentence(allystarGgaAt("235959.900").c_str()));
    gpsData.now += 100;
    makeAllystarFrame(frame, 456, -45, -67, 17, 1234567, 18000);
    EXPECT_FALSE(feedGpsBytes(frame, sizeof(frame)));
    EXPECT_TRUE(feedTestSentence(allystarGgaAt("000000.000").c_str()));
    EXPECT_EQ(456, gpsSol.velned.velN);
    EXPECT_EQ(100, gpsSol.navIntervalMs);
    EXPECT_FALSE(feedTestSentence(allystarGgaAt("235959.900").c_str()));
    gpsData.now += 50;
    EXPECT_FALSE(gpsNmeaFlushPending());
}

TEST(CLIUnittest, AllystarInvalidVelocityDoesNotLeakToOtherProviders)
{
    resetAllystarTest(true);
    gpsSol.velNEInvalid = true;
    gpsSol.vel3dInvalid = true;
    EXPECT_FALSE(gpsHorizontalVelocityUsable());
    EXPECT_FALSE(gpsThreeDimensionalSpeedUsable());
    gpsConfigMutable()->provider = GPS_UBLOX;
    EXPECT_TRUE(gpsHorizontalVelocityUsable());
    EXPECT_TRUE(gpsThreeDimensionalSpeedUsable());
}

TEST(CLIUnittest, NmeaRejectsOutOfRangeRmcSpeedAndCourse)
{
    resetAllystarTest(false);
    feedTestSentence(nmeaSentence("GPRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W").c_str());
    const auto speed = gpsSol.groundSpeed;
    const auto course = gpsSol.groundCourse;
    feedTestSentence(nmeaSentence("GPRMC,123519.00,A,4807.038,N,01131.000,E,999999999999,084.4,230394,003.1,W").c_str());
    EXPECT_EQ(speed, gpsSol.groundSpeed);
    feedTestSentence(nmeaSentence("GPRMC,123519.00,A,4807.038,N,01131.000,E,022.4,361.0,230394,003.1,W").c_str());
    EXPECT_EQ(course, gpsSol.groundCourse);
    feedTestSentence(nmeaSentence("GPRMC,123519.00,A,4807.038,N,01131.000,E,1274.0,360.000,230394,003.1,W").c_str());
    EXPECT_EQ(65534, gpsSol.groundSpeed);
    EXPECT_EQ(3600, gpsSol.groundCourse);
    feedTestSentence(nmeaSentence("GPRMC,123519.00,A,4807.038,N,01131.000,E,1274.01,084.4,230394,003.1,W").c_str());
    EXPECT_EQ(3600, gpsSol.groundCourse);
    feedTestSentence(nmeaSentence("GPRMC,123519.00,A,4807.038,N,01131.000,E,022.4,360.001,230394,003.1,W").c_str());
    EXPECT_EQ(3600, gpsSol.groundCourse);
}

TEST(CLIUnittest, NmeaNoFixCanOmitTimeAndSatelliteCount)
{
    resetAllystarTest(false);
    ASSERT_TRUE(feedTestSentence(nmeaGga().c_str()));
    ASSERT_TRUE(STATE(GPS_FIX));
    const uint16_t intervalBefore = gpsSol.navIntervalMs;
    EXPECT_TRUE(feedTestSentence(nmeaSentence("GNGGA,,,,,,0,,,,,,,,").c_str()));
    EXPECT_FALSE(STATE(GPS_FIX));
    EXPECT_EQ(intervalBefore, gpsSol.navIntervalMs);
}
