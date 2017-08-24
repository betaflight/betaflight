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
 Created by Marcin Baliniak
 some functions based on MinimOSD

 OSD-CMS separation by jflyper
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"

#ifdef USE_MAX7456
#define DEFAULT_OSD_DEVICE OSD_DEVICE_MAX7456
#else
#define DEFAULT_OSD_DEVICE OSD_DEVICE_NONE
#endif

#if defined(USE_OPENTCO) || defined(USE_MAX7456)  || defined(USE_OSD_SLAVE)

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_osd.h"

#include "common/maths.h"
#include "common/printf.h"
#include "common/typeconversion.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/display.h"
#include "drivers/max7456_symbols.h"
#include "drivers/time.h"
#include "drivers/vtx_common.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/osd.h"
#include "io/vtx_rtc6705.h"
#include "io/vtx_control.h"
#include "io/vtx_string.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/altitude.h"
#include "flight/navigation.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "rx/rx.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/esc_sensor.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#define VIDEO_BUFFER_CHARS_PAL    480

const char * const osdTimerSourceNames[] = {
    "ON TIME  ",
    "TOTAL ARM",
    "LAST ARM "
};

// Blink control

static bool blinkState = true;
static bool showVisualBeeper = false;

static uint32_t blinkBits[(OSD_ITEM_COUNT + 31)/32];
#define SET_BLINK(item) (blinkBits[(item) / 32] |= (1 << ((item) % 32)))
#define CLR_BLINK(item) (blinkBits[(item) / 32] &= ~(1 << ((item) % 32)))
#define IS_BLINK(item) (blinkBits[(item) / 32] & (1 << ((item) % 32)))
#define BLINK(item) (IS_BLINK(item) && blinkState)

// Things in both OSD and CMS

#define IS_HI(X)  (rcData[X] > 1750)
#define IS_LO(X)  (rcData[X] < 1250)
#define IS_MID(X) (rcData[X] > 1250 && rcData[X] < 1750)

static timeUs_t flyTime = 0;
static uint8_t statRssi;

typedef struct statistic_s {
    int16_t max_speed;
    int16_t min_voltage; // /10
    int16_t max_current; // /10
    int16_t min_rssi;
    int16_t max_altitude;
    int16_t max_distance;
    timeUs_t armed_time;
} statistic_t;

static statistic_t stats;

uint32_t resumeRefreshAt = 0;
#define REFRESH_1S    1000 * 1000

static uint8_t armState;
static uint8_t osdCurrentElementIndex;

displayPort_t *osdDisplayPort = NULL;

#define AH_SYMBOL_COUNT 9

#define AH_SIDEBAR_WIDTH_POS 7
#define AH_SIDEBAR_HEIGHT_POS 3

static const char compassBar[] = {
  SYM_HEADING_W,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE,
  SYM_HEADING_N,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE,
  SYM_HEADING_E,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE,
  SYM_HEADING_S,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE,
  SYM_HEADING_W,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE,
  SYM_HEADING_N,
  SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE
};

PG_REGISTER_WITH_RESET_FN(osdConfig_t, osdConfig, PG_OSD_CONFIG, 1);

#ifdef USE_ESC_SENSOR
static escSensorData_t *escData;
#endif

/**
 * Gets the correct altitude symbol for the current unit system
 */
static char osdGetMetersToSelectedUnitSymbol()
{
    switch (osdConfig()->units) {
    case OSD_UNIT_IMPERIAL:
        return SYM_FT;
    default:
        return SYM_M;
    }
}

/**
 * Gets average battery cell voltage in 0.01V units.
 */
static int osdGetBatteryAverageCellVoltage(void)
{
    return (getBatteryVoltage() * 10) / getBatteryCellCount();
}

static char osdGetBatterySymbol(int cellVoltage)
{
    if (getBatteryState() == BATTERY_CRITICAL) {
        return SYM_MAIN_BATT; // FIXME: currently the BAT- symbol, ideally replace with a battery with exclamation mark
    } else {
        /* Calculate a symbol offset using cell voltage over full cell voltage range */
        int symOffset = scaleRange(cellVoltage, batteryConfig()->vbatmincellvoltage * 10, batteryConfig()->vbatmaxcellvoltage * 10, 0, 7);
        return SYM_BATT_EMPTY - constrain(symOffset, 0, 6);
    }
}

/**
 * Converts altitude based on the current unit system.
 * @param meters Value in meters to convert
 */
static int32_t osdGetMetersToSelectedUnit(int32_t meters)
{
    switch (osdConfig()->units) {
    case OSD_UNIT_IMPERIAL:
        return (meters * 328) / 100; // Convert to feet / 100
    default:
        return meters;               // Already in metre / 100
    }
}

static void osdFormatAltitudeString(char * buff, int altitude, bool pad)
{
    const int alt = osdGetMetersToSelectedUnit(altitude);
    int altitudeIntergerPart = abs(alt / 100);
    if (alt < 0) {
        altitudeIntergerPart *= -1;
    }
    tfp_sprintf(buff, pad ? "%4d.%01d%c" : "%d.%01d%c", altitudeIntergerPart, abs((alt % 100) / 10), osdGetMetersToSelectedUnitSymbol());
}

static void osdFormatPID(char * buff, const char * label, const pid8_t * pid)
{
    tfp_sprintf(buff, "%s %3d %3d %3d", label, pid->P, pid->I, pid->D);
}

static uint8_t osdGetHeadingIntoDiscreteDirections(int heading, int directions)
{
    // Split input heading 0..359 into sectors 0..(directions-1), but offset
    // by half a sector so that sector 0 gets centered around heading 0.
    heading = (heading * 2 + 360 / directions) % 720;
    heading = heading / (360 * 2 / directions);

    return heading;
}

static uint8_t osdGetDirectionSymbolFromHeading(int heading)
{
    heading = osdGetHeadingIntoDiscreteDirections(heading, 16);

    // Now heading has a heading with Up=0, Right=4, Down=8 and Left=12
    // Our symbols are Down=0, Right=4, Up=8 and Left=12
    // There're 16 arrow symbols. Transform it.
    heading = 16 - heading;
    heading = (heading + 8) % 16;

    return SYM_ARROW_SOUTH + heading;
}

static char osdGetTimerSymbol(osd_timer_source_e src)
{
    switch (src) {
    case OSD_TIMER_SRC_ON:
        return SYM_ON_M;
    case OSD_TIMER_SRC_TOTAL_ARMED:
    case OSD_TIMER_SRC_LAST_ARMED:
        return SYM_FLY_M;
    default:
        return ' ';
    }
}

static timeUs_t osdGetTimerValue(osd_timer_source_e src)
{
    switch (src) {
    case OSD_TIMER_SRC_ON:
        return micros();
    case OSD_TIMER_SRC_TOTAL_ARMED:
        return flyTime;
    case OSD_TIMER_SRC_LAST_ARMED:
        return stats.armed_time;
    default:
        return 0;
    }
}

STATIC_UNIT_TESTED void osdFormatTime(char * buff, osd_timer_precision_e precision, timeUs_t time)
{
    int seconds = time / 1000000;
    const int minutes = seconds / 60;
    seconds = seconds % 60;

    switch (precision) {
    case OSD_TIMER_PREC_SECOND:
    default:
        tfp_sprintf(buff, "%02d:%02d", minutes, seconds);
        break;
    case OSD_TIMER_PREC_HUNDREDTHS:
        {
            const int hundredths = (time / 10000) % 100;
            tfp_sprintf(buff, "%02d:%02d.%02d", minutes, seconds, hundredths);
            break;
        }
    }
}

STATIC_UNIT_TESTED void osdFormatTimer(char *buff, bool showSymbol, int timerIndex)
{
    const uint16_t timer = osdConfig()->timers[timerIndex];
    const uint8_t src = OSD_TIMER_SRC(timer);

    if (showSymbol) {
        *(buff++) = osdGetTimerSymbol(src);
    }

    osdFormatTime(buff, OSD_TIMER_PRECISION(timer), osdGetTimerValue(src));
}

static uint8_t osdIncElementIndex(uint8_t elementIndex) {
    // this makes sure that we do not enter an endless loop
    // in case there are no visible items
    uint8_t max_inc = OSD_ITEM_COUNT-1;

    while (max_inc) {
        // next item
        elementIndex++;
        max_inc--;

        // make sure not to exceed maximum
        if (elementIndex >= OSD_ITEM_COUNT) {
            elementIndex = 0;
        }

        // check for visibility
        if (osdConfig()->item[elementIndex].flags & OSD_FLAG_VISIBLE) {
            // found next visible item
            return elementIndex;
        }
    }

    // no osd item is visible, return invalid item
    return OSD_ITEM_COUNT;
}

// this will convert from relative positioning (i.e. measured from center pos)
// to absolute positioning (based on display height and width)
STATIC_UNIT_TESTED void osdConvertToAbsolutePosition(uint8_t item, int8_t *pos_x, int8_t *pos_y) {
    // output display dimensions
    uint8_t maxX  = osdDisplayPort->colCount - 1;
    uint8_t maxY = osdDisplayPort->rowCount - 1;

    // fetch origin positio
    uint8_t origin = ((uint8_t)(osdConfig()->item[item].flags) & OSD_FLAG_ORIGIN_MASK);

    // start with center
    int8_t tmpX = maxX;
    int8_t tmpY = maxY;


    // add offsets based on origin
    if (origin & OSD_FLAG_ORIGIN_E) {
        // move east
        tmpX += maxX;
    }

    if (origin & OSD_FLAG_ORIGIN_W) {
        // move west
        tmpX -= maxX;
    }

    if (origin & OSD_FLAG_ORIGIN_N) {
        // move north
        tmpY -= maxY;
    }

    if (origin & OSD_FLAG_ORIGIN_S) {
        // move south
        tmpY += maxY;
    }

    // rescale (round to next full int)
    tmpX = (tmpX + 1) / 2;
    tmpY = (tmpY + 1) / 2;

    // add offset
    tmpX += osdConfig()->item[item].x;
    tmpY += osdConfig()->item[item].y;

    // make sure to return valid x/y positions \in [0..max]
    *pos_x = constrain(tmpX, 0, maxX);
    *pos_y = constrain(tmpY, 0, maxY);
}

static void osdDrawSingleElement(uint8_t item)
{
    char buff[OSD_ELEMENT_BUFFER_LENGTH];
    int8_t elemPosX;
    int8_t elemPosY;

    if (item == OSD_ITEM_COUNT) {
        // no osd items are visible
        displayClearScreen(osdDisplayPort);
        return;
    }

    // fetch absolute positions
    osdConvertToAbsolutePosition(item, &elemPosX, &elemPosY);

    switch (item) {
    case OSD_RSSI_VALUE:
        {
            uint16_t osdRssi = rssi * 100 / 1024; // change range
            if (osdRssi >= 100)
                osdRssi = 99;

            tfp_sprintf(buff, "%c%2d", SYM_RSSI, osdRssi);
            break;
        }

    case OSD_MAIN_BATT_VOLTAGE:
        buff[0] = osdGetBatterySymbol(osdGetBatteryAverageCellVoltage());
        tfp_sprintf(buff + 1, "%d.%1d%c", getBatteryVoltage() / 10, getBatteryVoltage() % 10, SYM_VOLT);
        break;

    case OSD_CURRENT_DRAW:
        {
            const int32_t amperage = getAmperage();
            tfp_sprintf(buff, "%3d.%02d%c", abs(amperage) / 100, abs(amperage) % 100, SYM_AMP);
            break;
        }

    case OSD_MAH_DRAWN:
        tfp_sprintf(buff, "%4d%c", getMAhDrawn(), SYM_MAH);
        break;

#ifdef GPS
    case OSD_GPS_SATS:
        tfp_sprintf(buff, "%c%2d", 0x1f, gpsSol.numSat);
        break;

    case OSD_GPS_SPEED:
        // FIXME ideally we want to use SYM_KMH symbol but it's not in the font any more, so we use K.
        tfp_sprintf(buff, "%3dK", CM_S_TO_KM_H(gpsSol.groundSpeed));
        break;

    case OSD_GPS_LAT:
    case OSD_GPS_LON:
        {
            int32_t val;
            if (item == OSD_GPS_LAT) {
                buff[0] = SYM_ARROW_EAST;
                val = gpsSol.llh.lat;
            } else {
                buff[0] = SYM_ARROW_SOUTH;
                val = gpsSol.llh.lon;
            }

            // add sign
            if (val < 0) {
                val = -val;
                buff[1] = '-';
            }else{
                buff[1] = ' ';
            }
            // add whole degree number
            tfp_sprintf(buff + 1, "%3d", val / GPS_DEGREES_DIVIDER);
            // add '.'
            buff[5] = '.';
            // add fraction
            val = val - (val * GPS_DEGREES_DIVIDER);
            tfp_sprintf(&buff[6], "%d", val);
            // convert to fixed length string by filling the rest with spaces
            // gps data maximum is -DDD.1234567 = 12chars
            // start at the end
            uint8_t pos = 12;
            // new end of string:
            buff[pos--] = 0;
            // fill with spaces
            while ((pos > 0) && (buff[pos])) {
                buff[pos--] = ' ';
            }
            // remove old eof marker
            buff[pos] = ' ';
            break;
        }

    case OSD_HOME_DIST:
        if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
            int32_t distance = osdGetMetersToSelectedUnit(GPS_distanceToHome);
            tfp_sprintf(buff, "%5d%c", distance, osdGetMetersToSelectedUnitSymbol());
        } else {
            // We use this symbol when we don't have a FIX
            buff[0] = SYM_COLON;
            buff[1] = 0;
        }
        break;

    case OSD_HOME_DIR:
        if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
            if (GPS_distanceToHome > 0) {
                const int h = GPS_directionToHome - DECIDEGREES_TO_DEGREES(attitude.values.yaw);
                buff[0] = osdGetDirectionSymbolFromHeading(h);
            } else {
                // We don't have a HOME symbol in the font, by now we use this
                buff[0] = SYM_THR1;
            }

        } else {
            // We use this symbol when we don't have a FIX
            buff[0] = SYM_COLON;
        }

        buff[1] = 0;

        break;

#endif // GPS

    case OSD_COMPASS_BAR:
    {
        int16_t h = DECIDEGREES_TO_DEGREES(attitude.values.yaw);

        h = osdGetHeadingIntoDiscreteDirections(h, 16);

        memcpy(buff, compassBar + h, 9);
        buff[9]=0;
        break;
    }

    case OSD_ALTITUDE:
        {
            osdFormatAltitudeString(buff, getEstimatedAltitude(), true);
            break;
        }

    case OSD_ITEM_TIMER_1:
    case OSD_ITEM_TIMER_2:
        {
            const int timer = item - OSD_ITEM_TIMER_1;
            osdFormatTimer(buff, true, timer);
            break;
        }

    case OSD_FLYMODE:
        {
            char *p = "ACRO";

            if (isAirmodeActive())
                p = " AIR";

            if (FLIGHT_MODE(FAILSAFE_MODE))
                p = "!FS!";
            else if (FLIGHT_MODE(ANGLE_MODE))
                p = "STAB";
            else if (FLIGHT_MODE(HORIZON_MODE))
                p = " HOR";

            strcpy(buff, p);
            break;
        }

    case OSD_CRAFT_NAME:
        if (strlen(pilotConfig()->name) == 0)
            strcpy(buff, "   CRAFT_NAME   ");
        else {
            memset(buff, ' ', MAX_NAME_LENGTH);
            for (unsigned int i = 0; i < MAX_NAME_LENGTH; i++) {
                if (pilotConfig()->name[i] == 0){
                    break;
                }
                buff[i] = toupper((unsigned char)pilotConfig()->name[i]);
            }
            buff[MAX_NAME_LENGTH-1] = 0;
        }
        break;

    case OSD_THROTTLE_POS:
        buff[0] = SYM_THR;
        buff[1] = SYM_THR1;
        tfp_sprintf(buff + 2, "%3d", (constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN));
        break;

#if defined(VTX_COMMON)
    case OSD_VTX_CHANNEL:
        {
            uint8_t band=0, channel=0;
            vtxCommonGetBandAndChannel(&band,&channel);

            uint8_t power = 0;
            vtxCommonGetPowerIndex(&power);

            const char vtxBandLetter = vtx58BandLetter[band];
            const char *vtxChannelName = vtx58ChannelNames[channel];
            tfp_sprintf(buff, "%c:%s:%d", vtxBandLetter, vtxChannelName, power);
            break;
        }
#endif

    case OSD_CROSSHAIRS:
        buff[0] = SYM_AH_CENTER_LINE;
        buff[1] = SYM_AH_CENTER;
        buff[2] = SYM_AH_CENTER_LINE_RIGHT;
        buff[3] = 0;
        break;

    case OSD_ARTIFICIAL_HORIZON:
        {
            // Get pitch and roll limits in tenths of degrees
            const int maxPitch = osdConfig()->ahMaxPitch * 10;
            const int maxRoll = osdConfig()->ahMaxRoll * 10;

            const int rollAngle = constrain(attitude.values.roll, -maxRoll, maxRoll);
            int pitchAngle = constrain(attitude.values.pitch, -maxPitch, maxPitch);

            // Convert pitchAngle to y compensation value
            // (maxPitch / 25) divisor matches previous settings of fixed divisor of 8 and fixed max AHI pitch angle of 20.0 degrees
            pitchAngle = ((pitchAngle * 25) / maxPitch) - 41; // 41 = 4 * AH_SYMBOL_COUNT + 5

            // clear the area before writing the new horizon characters
            displayFillRegion(osdDisplayPort, elemPosX - 4, elemPosY, 9, 9, ' ');

            // if not visible: abort after clearing here
            if (!(osdConfig()->item[item].flags & OSD_FLAG_VISIBLE)){
                return;
            }

            for (int x = -4; x <= 4; x++) {
                const int y = ((-rollAngle * x) / 64) - pitchAngle;
                if (y >= 0 && y <= 81) {
                    displayWriteChar(osdDisplayPort, elemPosX + x, elemPosY + (y / AH_SYMBOL_COUNT), (SYM_AH_BAR9_0 + (y % AH_SYMBOL_COUNT)));
                }
            }

            return;
        }

    case OSD_HORIZON_SIDEBARS:
        {
            // Draw AH sides
            const int8_t hudwidth = AH_SIDEBAR_WIDTH_POS;
            const int8_t hudheight = AH_SIDEBAR_HEIGHT_POS;
            for (int y = -hudheight; y <= hudheight; y++) {
                if (!(osdConfig()->item[item].flags & OSD_FLAG_VISIBLE)){
                    // clear sidebars
                    displayWriteChar(osdDisplayPort, elemPosX - hudwidth, elemPosY + y, ' ');
                    displayWriteChar(osdDisplayPort, elemPosX + hudwidth, elemPosY + y, ' ');
                } else {
                    // show sidebars
                    displayWriteChar(osdDisplayPort, elemPosX - hudwidth, elemPosY + y, SYM_AH_DECORATION);
                    displayWriteChar(osdDisplayPort, elemPosX + hudwidth, elemPosY + y, SYM_AH_DECORATION);
                }
            }

            // AH level indicators
            if (!(osdConfig()->item[item].flags & OSD_FLAG_VISIBLE)){
                // not visible -> clear
                displayWriteChar(osdDisplayPort, elemPosX - hudwidth + 1, elemPosY, ' ');
                displayWriteChar(osdDisplayPort, elemPosX + hudwidth - 1, elemPosY, ' ');
            } else {
                // render bars
                displayWriteChar(osdDisplayPort, elemPosX - hudwidth + 1, elemPosY, SYM_AH_LEFT);
                displayWriteChar(osdDisplayPort, elemPosX + hudwidth - 1, elemPosY, SYM_AH_RIGHT);
            }

            return;
        }

    case OSD_ROLL_PIDS:
        {
            const pidProfile_t *pidProfile = currentPidProfile;
            osdFormatPID(buff, "ROL", &pidProfile->pid[PID_ROLL]);
            break;
        }

    case OSD_PITCH_PIDS:
        {
            const pidProfile_t *pidProfile = currentPidProfile;
            osdFormatPID(buff, "PIT", &pidProfile->pid[PID_PITCH]);
            break;
        }

    case OSD_YAW_PIDS:
        {
            const pidProfile_t *pidProfile = currentPidProfile;
            osdFormatPID(buff, "YAW", &pidProfile->pid[PID_YAW]);
            break;
        }

    case OSD_POWER:
        tfp_sprintf(buff, "%4dW", getAmperage() * getBatteryVoltage() / 1000);
        break;

    case OSD_PIDRATE_PROFILE:
        {
            const uint8_t pidProfileIndex = getCurrentPidProfileIndex();
            const uint8_t rateProfileIndex = getCurrentControlRateProfileIndex();
            tfp_sprintf(buff, "%d-%d", pidProfileIndex + 1, rateProfileIndex + 1);
            break;
        }

    case OSD_WARNINGS:
        /* Show common reason for arming being disabled */
        if (IS_RC_MODE_ACTIVE(BOXARM) && isArmingDisabled()) {
            const armingDisableFlags_e flags = getArmingDisableFlags();
            for (int i = 0; i < NUM_ARMING_DISABLE_FLAGS; i++) {
                if (flags & (1 << i)) {
                    // center warning flag name
                    tfp_sprintf(buff, "           ");
                    int name_len = strlen(armingDisableFlagNames[i]);
                    tfp_sprintf(buff + 11/2 - name_len/2, "%s", armingDisableFlagNames[i]);
                    break;
                }
            }
            break;
        }

        /* Show warning if battery is not fresh */
        if (!ARMING_FLAG(WAS_EVER_ARMED) && (getBatteryState() == BATTERY_OK)
              && getBatteryAverageCellVoltage() < batteryConfig()->vbatfullcellvoltage) {
            tfp_sprintf(buff, "BATT NOT FULL");
            break;
        }

        /* Show battery state warning */
        switch (getBatteryState()) {
        case BATTERY_WARNING:
            tfp_sprintf(buff, "LOW BATTERY");
            break;

        case BATTERY_CRITICAL:
            tfp_sprintf(buff, " LAND NOW  ");
            break;

        default:
            /* Show visual beeper if battery is OK */
            if (showVisualBeeper) {
                tfp_sprintf(buff, "  * * * *  ");
                showVisualBeeper = false;
            } else {
                tfp_sprintf(buff, "           ");
            }
            break;
        }
        break;

    case OSD_AVG_CELL_VOLTAGE:
        {
            const int cellV = osdGetBatteryAverageCellVoltage();
            buff[0] = osdGetBatterySymbol(cellV);
            tfp_sprintf(buff + 1, "%d.%02d%c", cellV / 100, cellV % 100, SYM_VOLT);
            break;
        }

    case OSD_DEBUG:
        tfp_sprintf(buff, "DBG %5d %5d %5d %5d", debug[0], debug[1], debug[2], debug[3]);
        break;

    case OSD_PITCH_ANGLE:
    case OSD_ROLL_ANGLE:
        {
            const int angle = (item == OSD_PITCH_ANGLE) ? attitude.values.pitch : attitude.values.roll;
            tfp_sprintf(buff, "%c%02d.%01d", angle < 0 ? '-' : ' ', abs(angle / 10), abs(angle % 10));
            break;
        }

    case OSD_MAIN_BATT_USAGE:
        {
            //Set length of indicator bar
            #define MAIN_BATT_USAGE_STEPS 11 // Use an odd number so the bar can be centralised.

            //Calculate constrained value
            float value = constrain(batteryConfig()->batteryCapacity - getMAhDrawn(), 0, batteryConfig()->batteryCapacity);

            //Calculate mAh used progress
            uint8_t mAhUsedProgress = ceil((value / (batteryConfig()->batteryCapacity / MAIN_BATT_USAGE_STEPS)));

            //Create empty battery indicator bar
            buff[0] = SYM_PB_START;
            for (uint8_t i = 1; i <= MAIN_BATT_USAGE_STEPS; i++) {
                if (i <= mAhUsedProgress)
                    buff[i] = SYM_PB_FULL;
                else
                    buff[i] = SYM_PB_EMPTY;
            }
            buff[MAIN_BATT_USAGE_STEPS+1] = SYM_PB_CLOSE;

            if (mAhUsedProgress > 0 && mAhUsedProgress < MAIN_BATT_USAGE_STEPS) {
                buff[1+mAhUsedProgress] = SYM_PB_END;
            }

            buff[MAIN_BATT_USAGE_STEPS+2] = 0;

            break;
        }

    case OSD_DISARMED:
        if (!ARMING_FLAG(ARMED)) {
            tfp_sprintf(buff, "DISARMED");
        } else {
            tfp_sprintf(buff, "        ");
        }
        break;

    case OSD_NUMERICAL_HEADING:
        {
            const int heading = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
            tfp_sprintf(buff, "%c%03d", osdGetDirectionSymbolFromHeading(heading), heading);
            break;
        }

    case OSD_NUMERICAL_VARIO:
        {
            const int verticalSpeed = osdGetMetersToSelectedUnit(getEstimatedVario());
            const char directionSymbol = verticalSpeed < 0 ? SYM_ARROW_SOUTH : SYM_ARROW_NORTH;
            tfp_sprintf(buff, "%c%01d.%01d", directionSymbol, abs(verticalSpeed / 100), abs((verticalSpeed % 100) / 10));
            break;
        }
#ifdef USE_ESC_SENSOR
    case OSD_ESC_TMP:
        tfp_sprintf(buff, "%3d%c", escData == NULL ? 0 : escData->temperature, SYM_TEMP_C);
        break;

    case OSD_ESC_RPM:
        tfp_sprintf(buff, "%5d", escData == NULL ? 0 : escData->rpm);
        break;
#endif

    default:
        return;
    }

    if (BLINK(item)) {
        // this item should be invisible right now because it blinks
        // fill with spaces to clear
        displayFillRegion(osdDisplayPort, elemPosX, elemPosY, strlen(buff), 1, ' ');
    } else {
        // send prepared string to display at given position
        displayWrite(osdDisplayPort, elemPosX, elemPosY, buff);
    }
}

void pgResetFn_osdConfig(osdConfig_t *osdConfig)
{
    // default positions are attached to origin points
    // this way the ui looks similar no matter what the current screen resolution is
    OSD_INIT(osdConfig, OSD_DEBUG            ,   1,  0, OSD_FLAG_ORIGIN_NW | OSD_FLAG_VISIBLE);

    OSD_INIT(osdConfig, OSD_RSSI_VALUE       ,  -6,  1, OSD_FLAG_ORIGIN_N | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_MAIN_BATT_VOLTAGE,   2,  1, OSD_FLAG_ORIGIN_N | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_AVG_CELL_VOLTAGE ,   2,  2, OSD_FLAG_ORIGIN_N | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_DISARMED         ,   4,  4, OSD_FLAG_ORIGIN_N | OSD_FLAG_VISIBLE);

    OSD_INIT(osdConfig, OSD_ESC_TMP          ,   4,  2, OSD_FLAG_ORIGIN_N | ESC_DEFAULT_VISIBILITY);
    OSD_INIT(osdConfig, OSD_ESC_RPM          ,   5,  2, OSD_FLAG_ORIGIN_N | ESC_DEFAULT_VISIBILITY);

    OSD_INIT(osdConfig, OSD_GPS_SPEED        ,  -3, -1, OSD_FLAG_ORIGIN_E | GPS_DEFAULT_VISIBILITY);
    OSD_INIT(osdConfig, OSD_GPS_SATS         ,   5,  1, OSD_FLAG_ORIGIN_N | GPS_DEFAULT_VISIBILITY);
    OSD_INIT(osdConfig, OSD_GPS_LAT          ,   1,  2, OSD_FLAG_ORIGIN_NW | GPS_DEFAULT_VISIBILITY);
    OSD_INIT(osdConfig, OSD_GPS_LON          ,   4,  2, OSD_FLAG_ORIGIN_N | GPS_DEFAULT_VISIBILITY);
    OSD_INIT(osdConfig, OSD_HOME_DIST        ,   1,  2, OSD_FLAG_ORIGIN_C | GPS_DEFAULT_VISIBILITY);
    OSD_INIT(osdConfig, OSD_HOME_DIR         ,   0,  2, OSD_FLAG_ORIGIN_C | GPS_DEFAULT_VISIBILITY);

    OSD_INIT(osdConfig, OSD_ITEM_TIMER_1     ,  -7,  1, OSD_FLAG_ORIGIN_NE | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_ITEM_TIMER_2     ,   1,  1, OSD_FLAG_ORIGIN_NW | OSD_FLAG_VISIBLE);

    OSD_INIT(osdConfig, OSD_ALTITUDE         ,  -6,  0, OSD_FLAG_ORIGIN_E | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_NUMERICAL_HEADING,  -6,  2, OSD_FLAG_ORIGIN_E | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_NUMERICAL_VARIO  ,  -6,  1, OSD_FLAG_ORIGIN_E | OSD_FLAG_VISIBLE);

    OSD_INIT(osdConfig, OSD_VTX_CHANNEL      ,  -4, -4, OSD_FLAG_ORIGIN_SE | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_PIDRATE_PROFILE  ,  -4, -5, OSD_FLAG_ORIGIN_SE | OSD_FLAG_VISIBLE);

    OSD_INIT(osdConfig, OSD_FLYMODE          ,  -1, -5, OSD_FLAG_ORIGIN_S | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_CRAFT_NAME       ,  -4, -4, OSD_FLAG_ORIGIN_S | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_POWER            ,   1, -5, OSD_FLAG_ORIGIN_S | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_MAIN_BATT_USAGE  ,  -6, -3, OSD_FLAG_ORIGIN_S | OSD_FLAG_VISIBLE);

    OSD_INIT(osdConfig, OSD_CURRENT_DRAW     ,   1,  0, OSD_FLAG_ORIGIN_SW | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_MAH_DRAWN        ,   1, -1, OSD_FLAG_ORIGIN_SW | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_ROLL_PIDS        ,   7,  -2, OSD_FLAG_ORIGIN_SW | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_PITCH_PIDS       ,   7,  -1, OSD_FLAG_ORIGIN_SW | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_YAW_PIDS         ,   5,  -0, OSD_FLAG_ORIGIN_SW | OSD_FLAG_VISIBLE);

    OSD_INIT(osdConfig, OSD_PITCH_ANGLE      ,   1,  1, OSD_FLAG_ORIGIN_W | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_ROLL_ANGLE       ,   1,  2, OSD_FLAG_ORIGIN_W | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_THROTTLE_POS     ,   1,  0, OSD_FLAG_ORIGIN_W | OSD_FLAG_VISIBLE);

    OSD_INIT(osdConfig, OSD_WARNINGS         ,  -5,  3, OSD_FLAG_ORIGIN_C | OSD_FLAG_VISIBLE);
    OSD_INIT(osdConfig, OSD_COMPASS_BAR      ,  -4,  1, OSD_FLAG_ORIGIN_C | OSD_FLAG_VISIBLE);

    // Crosshair uses 3 chars, from center offset 1 to the left
    OSD_INIT(osdConfig, OSD_CROSSHAIRS       , -1,  0, OSD_FLAG_ORIGIN_C | OSD_FLAG_VISIBLE);
    // AH top center of region is 4 to the left
    OSD_INIT(osdConfig, OSD_ARTIFICIAL_HORIZON ,  0, -4, OSD_FLAG_ORIGIN_C | HORIZON_DEFAULT_VISIBILITY);
    // Horizon is centered
    OSD_INIT(osdConfig, OSD_HORIZON_SIDEBARS ,  0,  0, OSD_FLAG_ORIGIN_C | HORIZON_DEFAULT_VISIBILITY);

    osdConfig->enabled_stats[OSD_STAT_MAX_SPEED]       = true;
    osdConfig->enabled_stats[OSD_STAT_MIN_BATTERY]     = true;
    osdConfig->enabled_stats[OSD_STAT_MIN_RSSI]        = true;
    osdConfig->enabled_stats[OSD_STAT_MAX_CURRENT]     = true;
    osdConfig->enabled_stats[OSD_STAT_USED_MAH]        = true;
    osdConfig->enabled_stats[OSD_STAT_MAX_ALTITUDE]    = false;
    osdConfig->enabled_stats[OSD_STAT_BLACKBOX]        = true;
    osdConfig->enabled_stats[OSD_STAT_END_BATTERY]     = false;
    osdConfig->enabled_stats[OSD_STAT_MAX_DISTANCE]    = false;
    osdConfig->enabled_stats[OSD_STAT_BLACKBOX_NUMBER] = true;
    osdConfig->enabled_stats[OSD_STAT_TIMER_1]         = false;
    osdConfig->enabled_stats[OSD_STAT_TIMER_2]         = true;

    osdConfig->units = OSD_UNIT_METRIC;

    osdConfig->timers[OSD_TIMER_1] = OSD_TIMER(OSD_TIMER_SRC_ON, OSD_TIMER_PREC_SECOND, 10);
    osdConfig->timers[OSD_TIMER_2] = OSD_TIMER(OSD_TIMER_SRC_TOTAL_ARMED, OSD_TIMER_PREC_SECOND, 10);

    osdConfig->rssi_alarm = 20;
    osdConfig->cap_alarm  = 2200;
    osdConfig->alt_alarm  = 100; // meters or feet depend on configuration

    osdConfig->ahMaxPitch = 20; // 20 degrees
    osdConfig->ahMaxRoll = 40; // 40 degrees

    osdConfig->device = DEFAULT_OSD_DEVICE;
}

static void osdDrawLogo(int x, int y)
{
    // display logo and help
    int fontOffset = 160;
    for (int row = 0; row < 4; row++) {
        for (int column = 0; column < 24; column++) {
            if (fontOffset <= SYM_END_OF_FONT)
                displayWriteChar(osdDisplayPort, x + column, y + row, fontOffset++);
        }
    }
}

void osdInit(displayPort_t *osdDisplayPortToUse)
{
    if ((!feature(FEATURE_OSD)) || (!osdDisplayPortToUse)) {
        // no osd hw active
        return;
    }

    //BUILD_BUG_ON(OSD_POS_MAX != OSD_POS(31,31));

    osdDisplayPort = osdDisplayPortToUse;
#ifdef CMS
    cmsDisplayPortRegister(osdDisplayPort);
#endif

    osdCurrentElementIndex = 0;

    armState = ARMING_FLAG(ARMED);

    memset(blinkBits, 0, sizeof(blinkBits));

    displayClearScreen(osdDisplayPort);

    displayReloadProfile(osdDisplayPort);

    displayEnableFeature(osdDisplayPort, DISPLAY_FEATURE_ENABLE);

    osdDrawLogo(3, 1);

    char string_buffer[30];
    tfp_sprintf(string_buffer, "V%s", FC_VERSION_STRING);
    displayWrite(osdDisplayPort, 20, 6, string_buffer);
#ifdef CMS
    displayWrite(osdDisplayPort, 7, 8,  CMS_STARTUP_HELP_TEXT1);
    displayWrite(osdDisplayPort, 11, 9, CMS_STARTUP_HELP_TEXT2);
    displayWrite(osdDisplayPort, 11, 10, CMS_STARTUP_HELP_TEXT3);
#endif

    displayResync(osdDisplayPort);

    resumeRefreshAt = micros() + (4 * REFRESH_1S);
}

void osdUpdateAlarms(void)
{
    // This is overdone?

    int32_t alt = osdGetMetersToSelectedUnit(getEstimatedAltitude()) / 100;

    if (statRssi < osdConfig()->rssi_alarm)
        SET_BLINK(OSD_RSSI_VALUE);
    else
        CLR_BLINK(OSD_RSSI_VALUE);

    if (getBatteryState() == BATTERY_OK) {
        CLR_BLINK(OSD_WARNINGS);
        CLR_BLINK(OSD_MAIN_BATT_VOLTAGE);
        CLR_BLINK(OSD_AVG_CELL_VOLTAGE);
    } else {
        SET_BLINK(OSD_WARNINGS);
        SET_BLINK(OSD_MAIN_BATT_VOLTAGE);
        SET_BLINK(OSD_AVG_CELL_VOLTAGE);
    }

    if (STATE(GPS_FIX) == 0)
        SET_BLINK(OSD_GPS_SATS);
    else
        CLR_BLINK(OSD_GPS_SATS);

    for (uint32_t i = 0; i < OSD_TIMER_COUNT; i++) {
        const uint16_t timer = osdConfig()->timers[i];
        const timeUs_t time = osdGetTimerValue(OSD_TIMER_SRC(timer));
        const timeUs_t alarmTime = OSD_TIMER_ALARM(timer) * 60000000; // convert from minutes to us
        if (alarmTime != 0 && time >= alarmTime)
            SET_BLINK(OSD_ITEM_TIMER_1 + i);
        else
            CLR_BLINK(OSD_ITEM_TIMER_1 + i);
    }

    if (getMAhDrawn() >= osdConfig()->cap_alarm) {
        SET_BLINK(OSD_MAH_DRAWN);
        SET_BLINK(OSD_MAIN_BATT_USAGE);
    } else {
        CLR_BLINK(OSD_MAH_DRAWN);
        CLR_BLINK(OSD_MAIN_BATT_USAGE);
    }

    if (alt >= osdConfig()->alt_alarm)
        SET_BLINK(OSD_ALTITUDE);
    else
        CLR_BLINK(OSD_ALTITUDE);
}

void osdResetAlarms(void)
{
    CLR_BLINK(OSD_RSSI_VALUE);
    CLR_BLINK(OSD_MAIN_BATT_VOLTAGE);
    CLR_BLINK(OSD_WARNINGS);
    CLR_BLINK(OSD_GPS_SATS);
    CLR_BLINK(OSD_MAH_DRAWN);
    CLR_BLINK(OSD_ALTITUDE);
    CLR_BLINK(OSD_AVG_CELL_VOLTAGE);
    CLR_BLINK(OSD_MAIN_BATT_USAGE);
    CLR_BLINK(OSD_ITEM_TIMER_1);
    CLR_BLINK(OSD_ITEM_TIMER_2);
}

static void osdResetStats(void)
{
    stats.max_current  = 0;
    stats.max_speed    = 0;
    stats.min_voltage  = 500;
    stats.max_current  = 0;
    stats.min_rssi     = 99;
    stats.max_altitude = 0;
    stats.max_distance = 0;
    stats.armed_time   = 0;
}

static void osdUpdateStats(void)
{
    int16_t value = 0;
#ifdef GPS
    value = CM_S_TO_KM_H(gpsSol.groundSpeed);
#endif
    if (stats.max_speed < value)
        stats.max_speed = value;

    if (stats.min_voltage > getBatteryVoltage())
        stats.min_voltage = getBatteryVoltage();

    value = getAmperage() / 100;
    if (stats.max_current < value)
        stats.max_current = value;

    if (stats.min_rssi > statRssi)
        stats.min_rssi = statRssi;

    if (stats.max_altitude < getEstimatedAltitude())
        stats.max_altitude = getEstimatedAltitude();

#ifdef GPS
    if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME) && (stats.max_distance < GPS_distanceToHome)) {
            stats.max_distance = GPS_distanceToHome;
    }
#endif
}

#ifdef BLACKBOX
static void osdGetBlackboxStatusString(char * buff)
{
    bool storageDeviceIsWorking = false;
    uint32_t storageUsed = 0;
    uint32_t storageTotal = 0;

    switch (blackboxConfig()->device) {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        storageDeviceIsWorking = sdcard_isInserted() && sdcard_isFunctional() && (afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_READY);
        if (storageDeviceIsWorking) {
            storageTotal = sdcard_getMetadata()->numBlocks / 2000;
            storageUsed = storageTotal - (afatfs_getContiguousFreeSpace() / 1024000);
        }
        break;
#endif

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        storageDeviceIsWorking = flashfsIsReady();
        if (storageDeviceIsWorking) {
            const flashGeometry_t *geometry = flashfsGetGeometry();
            storageTotal = geometry->totalSize / 1024;
            storageUsed = flashfsGetOffset() / 1024;
        }
        break;
#endif

    default:
        storageDeviceIsWorking = true;
    }

    if (storageDeviceIsWorking) {
        const uint16_t storageUsedPercent = (storageUsed * 100) / storageTotal;
        tfp_sprintf(buff, "%d%%", storageUsedPercent);
    } else {
        tfp_sprintf(buff, "FAULT");
    }
}
#endif

static void osdDisplayStatisticLabel(uint8_t y, const char * text, const char * value)
{
    displayWrite(osdDisplayPort, 2, y, text);
    displayWrite(osdDisplayPort, 20, y, ":");
    displayWrite(osdDisplayPort, 22, y, value);
}

static void osdShowStats(void)
{
    uint8_t top = 2;
    char buff[10];

    displayClearScreen(osdDisplayPort);
    displayWrite(osdDisplayPort, 2, top++, "  --- STATS ---");

    if (osdConfig()->enabled_stats[OSD_STAT_TIMER_1]) {
        osdFormatTimer(buff, false, OSD_TIMER_1);
        osdDisplayStatisticLabel(top++, osdTimerSourceNames[OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_1])], buff);
    }

    if (osdConfig()->enabled_stats[OSD_STAT_TIMER_2]) {
        osdFormatTimer(buff, false, OSD_TIMER_2);
        osdDisplayStatisticLabel(top++, osdTimerSourceNames[OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_2])], buff);
    }

    if (osdConfig()->enabled_stats[OSD_STAT_MAX_SPEED] && STATE(GPS_FIX)) {
        itoa(stats.max_speed, buff, 10);
        osdDisplayStatisticLabel(top++, "MAX SPEED", buff);
    }

    if (osdConfig()->enabled_stats[OSD_STAT_MAX_DISTANCE]) {
        tfp_sprintf(buff, "%d%c", osdGetMetersToSelectedUnit(stats.max_distance), osdGetMetersToSelectedUnitSymbol());
        osdDisplayStatisticLabel(top++, "MAX DISTANCE", buff);
    }

    if (osdConfig()->enabled_stats[OSD_STAT_MIN_BATTERY]) {
        tfp_sprintf(buff, "%d.%1d%c", stats.min_voltage / 10, stats.min_voltage % 10, SYM_VOLT);
        osdDisplayStatisticLabel(top++, "MIN BATTERY", buff);
    }

    if (osdConfig()->enabled_stats[OSD_STAT_END_BATTERY]) {
        tfp_sprintf(buff, "%d.%1d%c", getBatteryVoltage() / 10, getBatteryVoltage() % 10, SYM_VOLT);
        osdDisplayStatisticLabel(top++, "END BATTERY", buff);
    }

    if (osdConfig()->enabled_stats[OSD_STAT_MIN_RSSI]) {
        itoa(stats.min_rssi, buff, 10);
        strcat(buff, "%");
        osdDisplayStatisticLabel(top++, "MIN RSSI", buff);
    }

    if (batteryConfig()->currentMeterSource != CURRENT_METER_NONE) {
        if (osdConfig()->enabled_stats[OSD_STAT_MAX_CURRENT]) {
            itoa(stats.max_current, buff, 10);
            strcat(buff, "A");
            osdDisplayStatisticLabel(top++, "MAX CURRENT", buff);
        }

        if (osdConfig()->enabled_stats[OSD_STAT_USED_MAH]) {
            tfp_sprintf(buff, "%d%c", getMAhDrawn(), SYM_MAH);
            osdDisplayStatisticLabel(top++, "USED MAH", buff);
        }
    }

    if (osdConfig()->enabled_stats[OSD_STAT_MAX_ALTITUDE]) {
        osdFormatAltitudeString(buff, stats.max_altitude, false);
        osdDisplayStatisticLabel(top++, "MAX ALTITUDE", buff);
    }

#ifdef BLACKBOX
    if (osdConfig()->enabled_stats[OSD_STAT_BLACKBOX] && blackboxConfig()->device && blackboxConfig()->device != BLACKBOX_DEVICE_SERIAL) {
        osdGetBlackboxStatusString(buff);
        osdDisplayStatisticLabel(top++, "BLACKBOX", buff);
    }

    if (osdConfig()->enabled_stats[OSD_STAT_BLACKBOX_NUMBER] && blackboxConfig()->device && blackboxConfig()->device != BLACKBOX_DEVICE_SERIAL) {
        itoa(blackboxGetLogNumber(), buff, 10);
        osdDisplayStatisticLabel(top++, "BB LOG NUM", buff);
    }
#endif

    /* Reset time since last armed here to ensure this timer is at zero when back at "main" OSD screen */
    stats.armed_time = 0;
}

static void osdShowArmed(void)
{
    displayClearScreen(osdDisplayPort);
    displayWrite(osdDisplayPort, 12, 7, "ARMED");
}

STATIC_UNIT_TESTED void osdRefresh(timeUs_t currentTimeUs)
{
    static timeUs_t lastTimeUs = 0;

    // detect arm/disarm
    if (armState != ARMING_FLAG(ARMED)) {
        if (ARMING_FLAG(ARMED)) {
            osdResetStats();
            osdShowArmed();
            resumeRefreshAt = currentTimeUs + (REFRESH_1S / 2);
        } else {
            osdShowStats();
            resumeRefreshAt = currentTimeUs + (60 * REFRESH_1S);
        }

        armState = ARMING_FLAG(ARMED);
    }

    statRssi = scaleRange(rssi, 0, 1024, 0, 100);

    osdUpdateStats();

    if (ARMING_FLAG(ARMED)) {
        timeUs_t deltaT = currentTimeUs - lastTimeUs;
        flyTime += deltaT;
        stats.armed_time += deltaT;
    }
    lastTimeUs = currentTimeUs;

    if (resumeRefreshAt) {
        if (cmp32(currentTimeUs, resumeRefreshAt) < 0) {
            // in timeout period, check sticks for activity to resume display.
            if (IS_HI(THROTTLE) || IS_HI(PITCH)) {
                displayClearScreen(osdDisplayPort);
                resumeRefreshAt = 0;
            }

            displayHeartbeat(osdDisplayPort);
            return;
        } else {
            displayClearScreen(osdDisplayPort);
            resumeRefreshAt = 0;
        }
    } else {
        blinkState = (currentTimeUs / 200000) % 2;

#ifdef USE_ESC_SENSOR
        if (feature(FEATURE_ESC_SENSOR)) {
            escData = getEscSensorData(ESC_SENSOR_COMBINED);
        }
#endif

#ifdef CMS
        if (!displayIsGrabbed(osdDisplayPort)) {
            osdUpdateAlarms();

            if (! IS_RC_MODE_ACTIVE(BOXOSD)) {
                // draw single element
                osdDrawSingleElement(osdCurrentElementIndex);
                osdCurrentElementIndex = osdIncElementIndex(osdCurrentElementIndex);
            }

            displayHeartbeat(osdDisplayPort);
#ifdef OSD_CALLS_CMS
        } else {
            cmsUpdate(currentTimeUs);
#endif
        }
#endif
    }
}

void osdRedraw()
{
    // force a full redraw
    displayClearScreen(osdDisplayPort);
}

void osdReloadProfile()
{
    displayReloadProfile(osdDisplayPort);
    osdRedraw();
}

int osdRowCount()
{
    return osdDisplayPort->rowCount;
}

int osdColCount()
{
    return osdDisplayPort->colCount;
}

/*
 * Called periodically by the scheduler
 */
void osdUpdate(timeUs_t currentTimeUs)
{
    //static uint32_t counter = 0;

    if (osdConfig()->device == OSD_DEVICE_NONE) {
        // osd not active -> return
        return;
    }

    if (isBeeperOn()) {
        showVisualBeeper = true;
    }

#ifdef MAX7456_DMA_CHANNEL_TX
    // don't touch buffers if DMA transaction is in progress
    if (displayIsTransferInProgress(osdDisplayPort)) {
        return;
    }
#endif // MAX7456_DMA_CHANNEL_TX

#ifdef USE_SLOW_MSP_DISPLAYPORT_RATE_WHEN_UNARMED
    static uint32_t idlecounter = 0;
    if (!ARMING_FLAG(ARMED)) {
        if (idlecounter++ % 4 != 0) {
            return;
        }
    }
#endif

    // fill screen element by element (one each call)
    osdRefresh(currentTimeUs);

#ifdef CMS
    // do not allow ARM if we are in menu
    if (displayIsGrabbed(osdDisplayPort)) {
        setArmingDisabled(ARMING_DISABLED_OSD_MENU);
    } else {
        unsetArmingDisabled(ARMING_DISABLED_OSD_MENU);
    }
#endif
}

#endif
