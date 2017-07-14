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
#include <stdio.h>

#include <limits.h>

#include <math.h>

extern "C" {
    #include "platform.h"
    #include "target.h"
    #include "fc/runtime_config.h"
    #include "fc/fc_msp.h"
    #include "config/parameter_group.h"
    #include "config/feature.h"
    #include "config/parameter_group_ids.h"
    #include "sensors/battery.h"
    #include "drivers/buf_writer.h"
    #include "flight/mixer.h"
    #include "flight/servos.h"
    #include "flight/pid.h"
    #include "io/ledstrip.h"
    #include "io/serial.h"
    #include "io/osd.h"
    #include "fc/settings.h"
    #include "rx/rx.h"
    #include "io/beeper.h"
    #include "fc/rc_adjustments.h"
    #include "scheduler/scheduler.h"
    #include "fc/runtime_config.h"
    #include "build/version.h"
    #include "fc/config.h"
    #include "drivers/buf_writer.h"
    #include "fc/cli.h"

    void cliSet(char *cmdline);
    void cliGet(char *cmdline);
    void *getValuePointer(const clivalue_t *value);

    const clivalue_t valueTable[] = {
        { "array_unit_test",             VAR_INT8  | MODE_ARRAY | MASTER_VALUE, .config.array.length = 3, PG_RESERVED_FOR_TESTING_1, 0 }
    };
    const uint16_t valueTableEntryCount = ARRAYLEN(valueTable);
    const lookupTableEntry_t lookupTables[] = {};
  

    PG_REGISTER_WITH_RESET_FN(osdConfig_t, osdConfig, PG_OSD_CONFIG, 0);
    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER_WITH_RESET_FN(ledStripConfig_t, ledStripConfig, PG_LED_STRIP_CONFIG, 0);
    PG_REGISTER_WITH_RESET_TEMPLATE(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges, PG_ADJUSTMENT_RANGE_CONFIG, 0);
    PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions, PG_MODE_ACTIVATION_PROFILE, 0);
    PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
    PG_REGISTER_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);
    PG_REGISTER_ARRAY_WITH_RESET_FN(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);
    PG_REGISTER_ARRAY(servoMixer_t, MAX_SERVO_RULES, customServoMixers, PG_SERVO_MIXER, 0);
    PG_REGISTER_WITH_RESET_TEMPLATE(featureConfig_t, featureConfig, PG_FEATURE_CONFIG, 0);
    PG_REGISTER(beeperConfig_t, beeperConfig, PG_BEEPER_CONFIG, 0);
    PG_REGISTER_WITH_RESET_FN(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER_WITH_RESET_FN(serialConfig_t, serialConfig, PG_SERIAL_CONFIG, 0);
    PG_REGISTER_ARRAY_WITH_RESET_FN(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs, PG_RX_CHANNEL_RANGE_CONFIG, 0);
    PG_REGISTER_ARRAY_WITH_RESET_FN(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs, PG_RX_FAILSAFE_CHANNEL_CONFIG, 0);
    PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 0);

    PG_REGISTER_WITH_RESET_FN(int8_t, unitTestData, PG_RESERVED_FOR_TESTING_1, 0); 
}

#include "unittest_macros.h"
#include "gtest/gtest.h"
TEST(CLIUnittest, TestCliSet)
{

    cliSet((char *)"array_unit_test    =   123,  -3  , 1");

    const clivalue_t cval = {
        .name = "array_unit_test",
        .type = MODE_ARRAY | MASTER_VALUE | VAR_INT8,
        .pgn = PG_RESERVED_FOR_TESTING_1,
        .offset = 0
    };

    printf("\n===============================\n");
    int8_t *data = (int8_t *)getValuePointer(&cval);
    for(int i=0; i<3; i++){
        printf("data[%d] = %d\n", i, data[i]);
    }
    printf("\n===============================\n");


    EXPECT_EQ(123, data[0]);
    EXPECT_EQ( -3, data[1]);
    EXPECT_EQ(  1, data[2]);


    //cliGet((char *)"osd_item_vbat");
    //EXPECT_EQ(false, false);
}

// STUBS
extern "C" {

float motor_disarmed[MAX_SUPPORTED_MOTORS];

uint16_t batteryWarningVoltage;
uint8_t useHottAlarmSoundPeriod (void) { return 0; }
const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000, 400000}; // see baudRate_e

uint32_t micros(void) {return 0;}

int32_t getAmperage(void) {
    return 100;
}

uint16_t getBatteryVoltage(void) {
    return 42;
}

batteryState_e getBatteryState(void) {
    return BATTERY_OK;
}

uint8_t calculateBatteryPercentageRemaining(void) {
    return 67;
}

uint8_t getMotorCount() {
    return 4;
}


void setPrintfSerialPort(struct serialPort_s *serialPort) { UNUSED(serialPort); }

void tfp_printf(const char * expectedFormat, ...) {
    va_list args;

    va_start(args, expectedFormat);
    vprintf(expectedFormat, args);
    va_end(args);
}


void tfp_format(void *putp, void (*putf) (void *, char), const char * expectedFormat, va_list va) {
    UNUSED(putp);
    UNUSED(putf);
    vprintf(expectedFormat, va);
}



static const box_t boxes[] = {
    { 0, "DUMMYBOX", 0 }
};

const box_t *findBoxByPermanentId(uint8_t permanentId) {
	UNUSED(permanentId);
	return &boxes[0];
}
const box_t *findBoxByBoxId(boxId_e boxId) {
	UNUSED(boxId);
        return &boxes[0];
}

const pidConfig_t pgResetTemplate_pidConfig = { 0 };
const systemConfig_t pgResetTemplate_systemConfig = { .debug_mode = 0,0};
const mixerConfig_t pgResetTemplate_mixerConfig = { 0, false }; 
const featureConfig_t pgResetTemplate_featureConfig = { 0 };

int8_t unitTestDataArray[3];

void pgResetFn_unitTestData(int8_t *ptr) {
    ptr = &unitTestDataArray[0];
}

void pgResetFn_osdConfig(osdConfig_t *config) { UNUSED(config); }
void pgResetFn_rxConfig(rxConfig_t *config) { UNUSED(config); }
void pgResetFn_serialConfig(serialConfig_t *config) { UNUSED(config); }
void pgResetFn_rxChannelRangeConfigs(rxChannelRangeConfig_t *config) { UNUSED(config); }
void pgResetFn_rxFailsafeChannelConfigs(rxFailsafeChannelConfig_t *config) { UNUSED(config); }


void pgResetFn_ledStripConfig(ledStripConfig_t *ledStripConfig) { UNUSED(ledStripConfig); }
void pgResetFn_mixerConfig(mixerConfig_t *mixerConfig) { UNUSED(mixerConfig); }
void pgResetFn_servoParams(servoParam_t *servoParams) { UNUSED(servoParams); }


uint32_t getBeeperOffMask(void) { return 0; }
uint32_t getPreferredBeeperOffMask(void) { return 0; }

void beeper(beeperMode_e mode) {UNUSED(mode);}
void beeperSilence(void) {}
void beeperConfirmationBeeps(uint8_t beepCount) {UNUSED(beepCount);}
void beeperWarningBeeps(uint8_t beepCount) {UNUSED(beepCount);}
void beeperUpdate(timeUs_t currentTimeUs) {UNUSED(currentTimeUs);}
uint32_t getArmingBeepTimeMicros(void) {return 0;}
beeperMode_e beeperModeForTableIndex(int idx) {UNUSED(idx); return BEEPER_SILENCE;}
const char *beeperNameForTableIndex(int idx) {UNUSED(idx); return NULL;}
int beeperTableEntryCount(void) {return 0;}
bool isBeeperOn(void) {return false;}
void beeperOffSetAll(uint8_t beeperCount) { UNUSED(beeperCount); }
void setBeeperOffMask(uint32_t mask) { UNUSED(mask);}
void setPreferredBeeperOffMask(uint32_t mask) { UNUSED(mask);}

void beeperOffSet(uint32_t mask) { UNUSED(mask); }
void beeperOffClear(uint32_t mask) {UNUSED(mask); }
void beeperOffClearAll(void) {}
bool parseColor(int index, const char *colorConfig) { UNUSED(index); UNUSED(colorConfig); return false; }
void resetEEPROM(void) {}
void bufWriterFlush(bufWriter_t *b) { UNUSED(b); }
void mixerResetDisarmedMotors(void) {}
void gpsEnablePassthrough(struct serialPort_s *gpsPassthroughPort) { UNUSED(gpsPassthroughPort); }
bool parseLedStripConfig(int ledIndex, const char *config){ UNUSED(ledIndex); UNUSED(config); return false; }
const char rcChannelLetters[] = "AERT12345678abcdefgh";
void parseRcChannels(const char *input, rxConfig_t *rxConfig){ UNUSED(input), UNUSED(rxConfig); }
void mixerLoadMix(int index, motorMixer_t *customMixers) { UNUSED(index); UNUSED(customMixers); }
bool setModeColor(ledModeIndex_e modeIndex, int modeColorIndex, int colorIndex) {  UNUSED(modeIndex); UNUSED(modeColorIndex); UNUSED(colorIndex); return false; }
float convertExternalToMotor(uint16_t externalValue){ UNUSED(externalValue); return 1.0; }
uint8_t getCurrentPidProfileIndex(void){ return 1; }
uint8_t getCurrentControlRateProfileIndex(void){ return 1; }
void changeControlRateProfile(uint8_t controlRateProfileIndex) {UNUSED(controlRateProfileIndex);}
void resetAllRxChannelRangeConfigurations(rxChannelRangeConfig_t *rxChannelRangeConfig) { UNUSED(rxChannelRangeConfig); }
void writeEEPROM() {}
serialPortConfig_t *serialFindPortConfiguration(serialPortIdentifier_e identifier) { UNUSED(identifier); return NULL; }
baudRate_e lookupBaudRateIndex(uint32_t baudRate){ UNUSED(baudRate); return BAUD_9600; }
serialPortUsage_t *findSerialPortUsageByIdentifier(serialPortIdentifier_e identifier){ UNUSED(identifier); return NULL; }
serialPort_t *openSerialPort(serialPortIdentifier_e identifier, serialPortFunction_e function, serialReceiveCallbackPtr rxCallback, uint32_t baudrate, portMode_t mode, portOptions_t options) { 
	UNUSED(identifier); UNUSED(function); UNUSED(rxCallback); UNUSED(baudrate); UNUSED(mode); UNUSED(options); 
	return NULL;
}
void serialSetBaudRate(serialPort_t *instance, uint32_t baudRate) { UNUSED(instance); UNUSED(baudRate); }
void serialSetMode(serialPort_t *instance, portMode_t mode) { UNUSED(instance); UNUSED(mode); }
void serialPassthrough(serialPort_t *left, serialPort_t *right, serialConsumer *leftC, serialConsumer *rightC) { UNUSED(left); UNUSED(right); UNUSED(leftC); UNUSED(rightC); }
uint32_t millis(void) { return 0; }
uint8_t getBatteryCellCount(void) { return 1; }
void servoMixerLoadMix(int index) { UNUSED(index); }
const char * getBatteryStateString(void){ return "_getBatteryStateString_"; }

uint32_t stackTotalSize(void) { return 0x4000; }
uint32_t stackHighMem(void) { return 0x80000000; }
uint16_t getEEPROMConfigSize(void) { return 1024; }

uint8_t __config_start = 0x00;
uint8_t __config_end = 0x10;
uint16_t averageSystemLoadPercent = 0;

timeDelta_t getTaskDeltaTime(cfTaskId_e taskId){ UNUSED(taskId); return 0; }
armingDisableFlags_e getArmingDisableFlags(void) { return ARMING_DISABLED_NO_GYRO; }

const char *armingDisableFlagNames[]= {
"DUMMYDISABLEFLAGNAME"
};

void getTaskInfo(cfTaskId_e taskId, cfTaskInfo_t *taskInfo){ UNUSED(taskId); UNUSED(taskInfo); }
void getCheckFuncInfo(cfCheckFuncInfo_t *checkFuncInfo){ UNUSED(checkFuncInfo); }

const char * const targetName = "UNITTEST";
const char* const buildDate = "Jan 01 2017";
const char * const buildTime = "00:00:00";
const char * const shortGitRevision = "MASTER";

uint32_t serialRxBytesWaiting(const serialPort_t *instance) { UNUSED(instance); return 0;}
uint8_t serialRead(serialPort_t *instance){ UNUSED(instance); return 0;}

void bufWriterAppend(bufWriter_t *b, uint8_t ch){ UNUSED(b); printf("%c", ch); }
//void setPrintfSerialPort(struct serialPort_s *serialPort){ UNUSED(serialPort); }
void serialWriteBufShim(void *instance, const uint8_t *data, int count){ UNUSED(instance); UNUSED(data); UNUSED(count); }
bufWriter_t *bufWriterInit(uint8_t *b, int total_size, bufWrite_t writer, void *p) { UNUSED(b); UNUSED(total_size); UNUSED(writer); UNUSED(p); return NULL; }
void schedulerSetCalulateTaskStatistics(bool calculateTaskStatistics) { UNUSED(calculateTaskStatistics); }
void setArmingDisabled(armingDisableFlags_e flag) { UNUSED(flag); }


void waitForSerialPortToFinishTransmitting(serialPort_t *serialPort) { UNUSED(serialPort); }
void stopPwmAllMotors(void) {}
void systemResetToBootloader(void) {}
void resetConfigs(void) {}
void systemReset(void) {}

void changePidProfile(uint8_t pidProfileIndex) { UNUSED(pidProfileIndex); }
bool serialIsPortAvailable(serialPortIdentifier_e identifier) { UNUSED(identifier); return false; }
void generateLedConfig(ledConfig_t *ledConfig, char *ledConfigBuffer, size_t bufferSize){ UNUSED(ledConfig); UNUSED(ledConfigBuffer); UNUSED(bufferSize); }

bool isSerialTransmitBufferEmpty(const serialPort_t *instance){ UNUSED(instance); return true; }

void serialWrite(serialPort_t *instance, uint8_t ch) { printf("%c", ch); UNUSED(instance); }





}
