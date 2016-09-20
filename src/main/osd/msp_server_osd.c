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
#include "config/feature.h"

#include "common/axis.h"
#include "common/utils.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/streambuf.h"
#include "common/filter.h"
#include "common/pilot.h"

#include "drivers/adc.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/video_textscreen.h"
#include "drivers/video.h"

#include "sensors/voltage.h"
#include "sensors/battery.h"

#include "io/serial.h"
#include "io/transponder_ir.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"


#include "build/version.h"
#include "scheduler/scheduler.h"

#include "osd/config.h"
#include "osd/osd_element.h"
#include "osd/osd.h"
#include "osd/osd_serial.h"
#include "osd/osd_screen.h"
#include "osd/msp_server_osd.h"
#include "../sensors/amperage.h"

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
            sbufWriteU8(dst, 1);  // 0 == FC, 1 == OSD, 2 == FC with OSD
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
            sbufWriteU16(dst, averageSystemLoadPercent);
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
                sbufWriteU8(dst, voltageMeterConfig(i)->vbatscale);
                sbufWriteU8(dst, voltageMeterConfig(i)->vbatresdivval);
                sbufWriteU8(dst, voltageMeterConfig(i)->vbatresdivmultiplier);
            }
            break;

        case MSP_AMPERAGE_METER_CONFIG:
            for (int i = 0; i < MAX_AMPERAGE_METERS; i++) {
                sbufWriteU16(dst, amperageMeterConfig(i)->scale);
                sbufWriteU16(dst, amperageMeterConfig(i)->offset);
            }
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

        case MSP_BATTERY_STATES: {
            amperageMeter_t *amperageMeter = getAmperageMeter(batteryConfig()->amperageMeterSource);

            sbufWriteU8(dst, (uint8_t)getBatteryState() == BATTERY_NOT_PRESENT ? 0 : 1); // battery connected - 0 not connected, 1 connected
            sbufWriteU8(dst, (uint8_t)constrain(vbat, 0, 255));
            sbufWriteU16(dst, (uint16_t)constrain(amperageMeter->mAhDrawn, 0, 0xFFFF)); // milliamp hours drawn from battery
            break;
        }

        case MSP_CURRENT_METERS:
            for (int i = 0; i < MAX_AMPERAGE_METERS; i++) {
                amperageMeter_t *meter = getAmperageMeter(i);
                // write out amperage, once for each current meter.
                sbufWriteU16(dst, (uint16_t)constrain(meter->amperage * 10, 0, 0xFFFF)); // send amperage in 0.001 A steps. Negative range is truncated to zero
                sbufWriteU32(dst, meter->mAhDrawn);
            }
            break;
        case MSP_VOLTAGE_METERS:
            // write out voltage, once for each meter.
            for (int i = 0; i < MAX_VOLTAGE_METERS; i++) {
                uint16_t voltage = getVoltageMeter(i)->vbat;
                sbufWriteU8(dst, (uint8_t)constrain(voltage, 0, 255));
            }
            break;

        case MSP_PILOT: {
            uint8_t *callsign = pilotConfig()->callsign;
            sbufWriteU8(dst, strlen((char *)callsign));
            sbufWriteString(dst, (char *)pilotConfig()->callsign);
            break;
        }
        case MSP_SET_PILOT: {
            uint8_t callsignMessageBytesRemaining = sbufReadU8(src);
            uint8_t *callsign = pilotConfig()->callsign;
            uint8_t callsignBytesToRemaining = MIN(callsignMessageBytesRemaining, CALLSIGN_LENGTH);

            while(sbufBytesRemaining(src) && callsignMessageBytesRemaining--) {
                uint8_t c = sbufReadU8(src);
                if (callsignBytesToRemaining > 0) {
                    callsignBytesToRemaining--;
                    *callsign++ = c;
                }
            };
            *callsign = 0;
            break;
        }
        case MSP_OSD_VIDEO_CONFIG:
            sbufWriteU8(dst, osdVideoConfig()->videoMode); // 0 = NTSC, 1 = PAL
            break;

        case MSP_OSD_VIDEO_STATUS:
            sbufWriteU8(dst, osdState.videoMode);
            sbufWriteU8(dst, osdState.cameraConnected);
            sbufWriteU8(dst, osdTextScreen.width);
            sbufWriteU8(dst, osdTextScreen.height);
            break;

        case MSP_OSD_ELEMENT_SUMMARY: {
            for (int i = 0; i < osdSupportedElementIdsCount; i++) {
                sbufWriteU16(dst, osdSupportedElementIds[i]);
            }
            break;
        }

        case MSP_OSD_LAYOUT_CONFIG:
            sbufWriteU8(dst, MAX_OSD_ELEMENT_COUNT);
            for (int i = 0; i < MAX_OSD_ELEMENT_COUNT; i++) {
                element_t *element = &osdElementConfig()->elements[i];

                // use 16bit to allow for current and future element IDs
                sbufWriteU16(dst, element->id);

                // use 16bit to allow for current future flags
                sbufWriteU16(dst, element->flags);

                sbufWriteU8(dst, element->x);
                sbufWriteU8(dst, element->y);
            }
            break;

        case MSP_SET_OSD_LAYOUT_CONFIG: {
            uint8_t elementIndex = sbufReadU8(src);
            if (elementIndex >= MAX_OSD_ELEMENT_COUNT) {
                return -1;
            }

            element_t *element = &osdElementConfig()->elements[elementIndex];

            element->id = sbufReadU16(src);
            element->flags = sbufReadU16(src);
            element->x = sbufReadU8(src);
            element->y = sbufReadU8(src);;
            break;
        }

        case MSP_SET_OSD_VIDEO_CONFIG:
            osdVideoConfig()->videoMode = sbufReadU8(src);
            mspPostProcessFn = mspApplyVideoConfigurationFn;
            break;

        case MSP_OSD_CHAR_WRITE: {
            uint8_t address = sbufReadU8(src);

            osdSetFontCharacter(address, src);
            break;
        }

        case MSP_RESET_CONF:
            resetEEPROM();
            readEEPROM();
            break;

        case MSP_EEPROM_WRITE:
            writeEEPROM();
            readEEPROM();
            break;

        case MSP_SET_VOLTAGE_METER_CONFIG: {
            int index = sbufReadU8(src);

            if (index >= MAX_VOLTAGE_METERS) {
                return -1;
            }

            voltageMeterConfig(index)->vbatscale = sbufReadU8(src);
            voltageMeterConfig(index)->vbatresdivval = sbufReadU8(src);
            voltageMeterConfig(index)->vbatresdivmultiplier = sbufReadU8(src);
            break;
        }

        case MSP_SET_AMPERAGE_METER_CONFIG: {
            int index = sbufReadU8(src);

            if (index >= MAX_AMPERAGE_METERS) {
                return -1;
            }

            amperageMeterConfig(index)->scale = sbufReadU16(src);
            amperageMeterConfig(index)->offset = sbufReadU16(src);
            break;
        }

        case MSP_SET_BATTERY_CONFIG:
            batteryConfig()->vbatmincellvoltage = sbufReadU8(src);      // vbatlevel_warn1 in MWC2.3 GUI
            batteryConfig()->vbatmaxcellvoltage = sbufReadU8(src);      // vbatlevel_warn2 in MWC2.3 GUI
            batteryConfig()->vbatwarningcellvoltage = sbufReadU8(src);  // vbatlevel when buzzer starts to alert
            batteryConfig()->batteryCapacity = sbufReadU16(src);
            batteryConfig()->amperageMeterSource = sbufReadU8(src);
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

        case MSP_FEATURE:
            sbufWriteU32(dst, featureMask());
            break;

        case MSP_SET_FEATURE:
            featureClearAll();
            featureSet(sbufReadU32(src)); // features bitmap
            break;

        case MSP_TRANSPONDER_CONFIG:
#ifdef TRANSPONDER
            sbufWriteU8(dst, 1); //Transponder supported
            sbufWriteData(dst, transponderConfig()->data, sizeof(transponderConfig()->data));
#else
            sbufWriteU8(dst, 0); // Transponder not supported
#endif
            break;

#ifdef TRANSPONDER
        case MSP_SET_TRANSPONDER_CONFIG:
            if (len != sizeof(transponderConfig()->data))
                return -1;
            sbufReadData(src, transponderConfig()->data, sizeof(transponderConfig()->data));
            transponderUpdateData(transponderConfig()->data);
            break;
#endif

        case MSP_REBOOT:
            mspPostProcessFn = mspRebootFn;
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
