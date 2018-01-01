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
    #include "build/version.h"
    #include "pg/pg.h"
    #include "config/feature.h"
    #include "pg/pg_ids.h"
    #include "drivers/buf_writer.h"
    #include "drivers/vtx_common.h"
    #include "fc/config.h"
    #include "fc/rc_adjustments.h"
    #include "fc/runtime_config.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/servos.h"
    #include "interface/cli.h"
    #include "interface/msp.h"
    #include "interface/msp_box.h"
    #include "interface/settings.h"
    #include "io/beeper.h"
    #include "io/ledstrip.h"
    #include "io/osd.h"
    #include "io/serial.h"
    #include "io/vtx.h"
    #include "pg/beeper.h"
    #include "rx/rx.h"
    #include "scheduler/scheduler.h"
    #include "sensors/battery.h"

    void cliSet(char *cmdline);
    void cliGet(char *cmdline);

    const clivalue_t valueTable[] = {
        { "array_unit_test",             VAR_INT8  | MODE_ARRAY | MASTER_VALUE, .config.array.length = 3, PG_RESERVED_FOR_TESTING_1, 0 }
    };
    const uint16_t valueTableEntryCount = ARRAYLEN(valueTable);
    const lookupTableEntry_t lookupTables[] = {};


    PG_REGISTER(osdConfig_t, osdConfig, PG_OSD_CONFIG, 0);
    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER(ledStripConfig_t, ledStripConfig, PG_LED_STRIP_CONFIG, 0);
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
    int8_t *data = (int8_t *)cliGetValuePointer(&cval);
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


void setPrintfSerialPort(struct serialPort_s) {}

void tfp_printf(const char * expectedFormat, ...) {
    va_list args;

    va_start(args, expectedFormat);
    vprintf(expectedFormat, args);
    va_end(args);
}


void tfp_format(void *, void (*) (void *, char), const char * expectedFormat, va_list va) {
    vprintf(expectedFormat, va);
}

static const box_t boxes[] = { { 0, "DUMMYBOX", 0 } };
const box_t *findBoxByPermanentId(uint8_t) { return &boxes[0]; }
const box_t *findBoxByBoxId(boxId_e) { return &boxes[0]; }

int8_t unitTestDataArray[3];

void pgResetFn_unitTestData(int8_t *ptr) {
    ptr = &unitTestDataArray[0];
}

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
void resetEEPROM(void) {}
void bufWriterFlush(bufWriter_t *) {}
void mixerResetDisarmedMotors(void) {}
void gpsEnablePassthrough(struct serialPort_s *) {}
bool parseLedStripConfig(int, const char *){return false; }
const char rcChannelLetters[] = "AERT12345678abcdefgh";

void parseRcChannels(const char *, rxConfig_t *){}
void mixerLoadMix(int, motorMixer_t *) {}
bool setModeColor(ledModeIndex_e, int, int) { return false; }
float convertExternalToMotor(uint16_t ){ return 1.0; }
uint8_t getCurrentPidProfileIndex(void){ return 1; }
uint8_t getCurrentControlRateProfileIndex(void){ return 1; }
void changeControlRateProfile(uint8_t) {}
void resetAllRxChannelRangeConfigurations(rxChannelRangeConfig_t *) {}
void writeEEPROM() {}
serialPortConfig_t *serialFindPortConfiguration(serialPortIdentifier_e) {return NULL; }
baudRate_e lookupBaudRateIndex(uint32_t){return BAUD_9600; }
serialPortUsage_t *findSerialPortUsageByIdentifier(serialPortIdentifier_e){ return NULL; }
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) { return NULL; }
void serialSetBaudRate(serialPort_t *, uint32_t) {}
void serialSetMode(serialPort_t *, portMode_e) {}
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

timeDelta_t getTaskDeltaTime(cfTaskId_e){ return 0; }
armingDisableFlags_e getArmingDisableFlags(void) { return ARMING_DISABLED_NO_GYRO; }

const char *armingDisableFlagNames[]= {
"DUMMYDISABLEFLAGNAME"
};

void getTaskInfo(cfTaskId_e, cfTaskInfo_t *) {}
void getCheckFuncInfo(cfCheckFuncInfo_t *) {}

const char * const targetName = "UNITTEST";
const char* const buildDate = "Jan 01 2017";
const char * const buildTime = "00:00:00";
const char * const shortGitRevision = "MASTER";

uint32_t serialRxBytesWaiting(const serialPort_t *) {return 0;}
uint8_t serialRead(serialPort_t *){return 0;}

void bufWriterAppend(bufWriter_t *, uint8_t ch){ printf("%c", ch); }
void serialWriteBufShim(void *, const uint8_t *, int) {}
bufWriter_t *bufWriterInit(uint8_t *, int, bufWrite_t, void *) {return NULL;}
void schedulerSetCalulateTaskStatistics(bool) {}
void setArmingDisabled(armingDisableFlags_e) {}

void waitForSerialPortToFinishTransmitting(serialPort_t *) {}
void stopPwmAllMotors(void) {}
void systemResetToBootloader(void) {}
void resetConfigs(void) {}
void systemReset(void) {}

void changePidProfile(uint8_t) {}
bool serialIsPortAvailable(serialPortIdentifier_e) { return false; }
void generateLedConfig(ledConfig_t *, char *, size_t) {}
bool isSerialTransmitBufferEmpty(const serialPort_t *) {return true; }
void serialWrite(serialPort_t *, uint8_t ch) { printf("%c", ch);}


}
