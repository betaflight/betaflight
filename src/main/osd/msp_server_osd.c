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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "build/build_config.h"
#include "build/debug.h"
#include <platform.h>

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "common/axis.h"
#include "common/utils.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/streambuf.h"

#include "drivers/adc.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/video_textscreen.h"

#include "fc/rc_controls.h" // FIXME virtual current sensor needs it
#include "sensors/battery.h"

#include "io/serial.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"


#include "build/version.h"
#include "scheduler/scheduler.h"

#include "osd/config.h"
#include "osd/osd.h"
#include "osd/osd_serial.h"
#include "osd/msp_server_osd.h"

#define MAX_VOLTAGE_METERS 4 // FIXME move this

extern uint16_t cycleTime;

static const char * const flightControllerIdentifier = CLEANFLIGHT_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.
static const char * const boardIdentifier = TARGET_BOARD_IDENTIFIER;

void mspRebootFn(mspPort_t *msp)
{
    waitForSerialPortToFinishTransmitting(msp->port);  // TODO - postpone reboot, allow all modules to react
    systemReset();
}

void mspApplyVideoConfigurationFn(mspPort_t *msp)
{
    waitForSerialPortToFinishTransmitting(msp->port);
    osdApplyConfiguration();
}


int mspServerCommandHandler(mspPacket_t *cmd, mspPacket_t *reply)
{
    sbuf_t *src = &cmd->buf;
    sbuf_t *dst = &reply->buf;
    int len = sbufBytesRemaining(src);

    switch (cmd->cmd) {
        case MSP_API_VERSION:
            sbufWriteU8(dst, MSP_PROTOCOL_VERSION);

            sbufWriteU8(dst, API_VERSION_MAJOR);
            sbufWriteU8(dst, API_VERSION_MINOR);
            break;

        case MSP_FC_VARIANT:
            sbufWriteData(dst, flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
            break;

        case MSP_FC_VERSION:
            sbufWriteU8(dst, FC_VERSION_MAJOR);
            sbufWriteU8(dst, FC_VERSION_MINOR);
            sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
            break;

        case MSP_BOARD_INFO:
            sbufWriteData(dst, boardIdentifier, BOARD_IDENTIFIER_LENGTH);
            sbufWriteU16(dst, 0);  // hardware revision
            sbufWriteU8(dst, 1);  // 0 == FC, 1 == OSD
            break;

        case MSP_BUILD_INFO:
            sbufWriteData(dst, buildDate, BUILD_DATE_LENGTH);
            sbufWriteData(dst, buildTime, BUILD_TIME_LENGTH);
            sbufWriteData(dst, shortGitRevision, GIT_SHORT_REVISION_LENGTH);
            break;

            // DEPRECATED - Use MSP_API_VERSION
        case MSP_IDENT:
            sbufWriteU8(dst, MW_VERSION);
            sbufWriteU8(dst, 0); // mixer mode
            sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
            sbufWriteU32(dst, CAP_DYNBALANCE); // "capability"
            break;

        case MSP_STATUS_EX:
        case MSP_STATUS:
            sbufWriteU16(dst, cycleTime);
#ifdef USE_I2C
            sbufWriteU16(dst, i2cGetErrorCounter());
#else
            sbufWriteU16(dst, 0);
#endif
            sbufWriteU16(dst, 0); // sensors
            sbufWriteU32(dst, 0); // flight mode flags
            sbufWriteU8(dst, 0);  // profile index
            if(cmd->cmd == MSP_STATUS_EX) {
                sbufWriteU16(dst, averageSystemLoadPercent);
            }
            break;

        case MSP_DEBUG:
            // output some useful QA statistics
            // debug[x] = ((hse_value / 1000000) * 1000) + (SystemCoreClock / 1000000);         // XX0YY [crystal clock : core clock]

            for (int i = 0; i < DEBUG16_VALUE_COUNT; i++)
                sbufWriteU16(dst, debug[i]);      // 4 variables are here for general monitoring purpose
            break;

        case MSP_UID:
            sbufWriteU32(dst, U_ID_0);
            sbufWriteU32(dst, U_ID_1);
            sbufWriteU32(dst, U_ID_2);
            break;

        case MSP_VOLTAGE_METER_CONFIG:
            for (int i = 0; i < MAX_VOLTAGE_METERS; i++) {
                // FIXME update for multiple voltage sources  i.e.  use `i` and support at least OSD VBAT, OSD 12V, OSD 5V
                sbufWriteU8(dst, batteryConfig()->vbatscale);
                sbufWriteU8(dst, batteryConfig()->vbatmincellvoltage);
                sbufWriteU8(dst, batteryConfig()->vbatmaxcellvoltage);
                sbufWriteU8(dst, batteryConfig()->vbatwarningcellvoltage);
            }
            break;

        case MSP_CURRENT_METER_CONFIG:
            sbufWriteU16(dst, batteryConfig()->currentMeterScale);
            sbufWriteU16(dst, batteryConfig()->currentMeterOffset);
            sbufWriteU8(dst, batteryConfig()->currentMeterType);
            sbufWriteU16(dst, batteryConfig()->batteryCapacity);
            break;

        case MSP_CF_SERIAL_CONFIG:
            for (int i = 0; i < serialGetAvailablePortCount(); i++) {
                if (!serialIsPortAvailable(serialConfig()->portConfigs[i].identifier)) {
                    continue;
                };
                sbufWriteU8(dst, serialConfig()->portConfigs[i].identifier);
                sbufWriteU16(dst, serialConfig()->portConfigs[i].functionMask);
                sbufWriteU8(dst, serialConfig()->portConfigs[i].baudRates[BAUDRATE_MSP_SERVER]);
                sbufWriteU8(dst, serialConfig()->portConfigs[i].baudRates[BAUDRATE_MSP_CLIENT]);
                sbufWriteU8(dst, serialConfig()->portConfigs[i].baudRates[BAUDRATE_RESERVED1]);
                sbufWriteU8(dst, serialConfig()->portConfigs[i].baudRates[BAUDRATE_RESERVED2]);
            }
            break;

        case MSP_BF_BUILD_INFO:
            sbufWriteData(dst, buildDate, 11); // MMM DD YYYY as ascii, MMM = Jan/Feb... etc
            sbufWriteU32(dst, 0); // future exp
            sbufWriteU32(dst, 0); // future exp
            break;

        case MSP_DATAFLASH_SUMMARY: // FIXME update GUI and remove this.
            sbufWriteU8(dst, 0); // FlashFS is neither ready nor supported
            sbufWriteU32(dst, 0);
            sbufWriteU32(dst, 0);
            sbufWriteU32(dst, 0);
            break;

        case MSP_BATTERY_STATES:
            // write out battery states, once for each battery
            sbufWriteU8(dst, (uint8_t)getBatteryState() == BATTERY_NOT_PRESENT ? 0 : 1); // battery connected - 0 not connected, 1 connected
            sbufWriteU8(dst, (uint8_t)constrain(vbat, 0, 255));
            sbufWriteU16(dst, (uint16_t)constrain(mAhDrawn, 0, 0xFFFF)); // milliamp hours drawn from battery
            break;

        case MSP_CURRENT_METERS:
            // write out amperage, once for each current meter.
            sbufWriteU16(dst, (uint16_t)constrain(amperage * 10, 0, 0xFFFF)); // send amperage in 0.001 A steps. Negative range is truncated to zero
            break;

        case MSP_VOLTAGE_METERS:
            // write out voltage, once for each meter.
            for (int i = 0; i < 3; i++) {
                // FIXME hack that needs cleanup, see issue #2221
                // This works for now, but the vbat scale also changes the 12V and 5V readings.
                switch(i) {
                    case 0:
                        sbufWriteU8(dst, (uint8_t)constrain(vbat, 0, 255));
                        break;
                    case 1:
                        sbufWriteU8(dst, (uint8_t)constrain(batteryAdcToVoltage(adcGetChannel(ADC_POWER_12V)), 0, 255));
                        break;
                    case 2:
                        sbufWriteU8(dst, (uint8_t)constrain(batteryAdcToVoltage(adcGetChannel(ADC_POWER_5V)), 0, 255));
                        break;
                }
            }
            break;
        case MSP_OSD_VIDEO_CONFIG:
            sbufWriteU8(dst, osdVideoConfig()->videoMode); // 0 = NTSC, 1 = PAL
            break;

        case MSP_RESET_CONF:
            resetEEPROM();
            readEEPROM();
            break;

        case MSP_EEPROM_WRITE:
            writeEEPROM();
            readEEPROM();
            break;

        case MSP_SET_VOLTAGE_METER_CONFIG: {
            uint8_t i = sbufReadU8(src);
            if (i >= MAX_VOLTAGE_METERS) {
                return -1;
            }
            // FIXME use `i`, see MSP_VOLTAGE_METER_CONFIG
            batteryConfig()->vbatscale = sbufReadU8(src);               // actual vbatscale as intended
            batteryConfig()->vbatmincellvoltage = sbufReadU8(src);      // vbatlevel_warn1 in MWC2.3 GUI
            batteryConfig()->vbatmaxcellvoltage = sbufReadU8(src);      // vbatlevel_warn2 in MWC2.3 GUI
            batteryConfig()->vbatwarningcellvoltage = sbufReadU8(src);  // vbatlevel when buzzer starts to alert
            break;
        }

        case MSP_SET_CURRENT_METER_CONFIG:
            batteryConfig()->currentMeterScale = sbufReadU16(src);
            batteryConfig()->currentMeterOffset = sbufReadU16(src);
            batteryConfig()->currentMeterType = sbufReadU8(src);
            batteryConfig()->batteryCapacity = sbufReadU16(src);
            break;

        case MSP_SET_CF_SERIAL_CONFIG: {
            int portConfigSize = sizeof(uint8_t) + sizeof(uint16_t) + (sizeof(uint8_t) * 4);

            if (len % portConfigSize != 0)
                return -1;

            while (sbufBytesRemaining(src) >= portConfigSize) {
                uint8_t identifier = sbufReadU8(src);

                serialPortConfig_t *portConfig = serialFindPortConfiguration(identifier);
                if (!portConfig)
                    return -1;

                portConfig->identifier = identifier;
                portConfig->functionMask = sbufReadU16(src);
                portConfig->baudRates[BAUDRATE_MSP_SERVER] = sbufReadU8(src);
                portConfig->baudRates[BAUDRATE_MSP_CLIENT] = sbufReadU8(src);
                portConfig->baudRates[BAUDRATE_RESERVED1] = sbufReadU8(src);
                portConfig->baudRates[BAUDRATE_RESERVED2] = sbufReadU8(src);
            }
            break;
        }

        case MSP_REBOOT:
            mspPostProcessFn = mspRebootFn;
            break;

        case MSP_SET_OSD_VIDEO_CONFIG:
            osdVideoConfig()->videoMode = sbufReadU8(src);
            mspPostProcessFn = mspApplyVideoConfigurationFn;
            break;

        default:
            // we do not know how to handle the message
            return 0;
    }
    return 1;     // message was handled successfully
}

void mspInit(void)
{
}
