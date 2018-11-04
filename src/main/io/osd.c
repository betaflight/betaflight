/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
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

#ifdef USE_OSD

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/typeconversion.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/display.h"
#include "drivers/flash.h"
#include "drivers/max7456_symbols.h"
#include "drivers/sdcard.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/core.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/position.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/osd.h"
#include "io/vtx_string.h"
#include "io/vtx.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/esc_sensor.h"
#include "sensors/sensors.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#define VIDEO_BUFFER_CHARS_PAL    480
#define FULL_CIRCLE 360

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
static float osdGForce = 0;

typedef struct statistic_s {
    timeUs_t armed_time;
    int16_t max_speed;
    int16_t min_voltage; // /10
    int16_t max_current; // /10
    uint8_t min_rssi;
    int32_t max_altitude;
    int16_t max_distance;
    float max_g_force;
    int16_t max_esc_temp;
    int32_t max_esc_rpm;
    uint8_t min_link_quality;
} statistic_t;

static statistic_t stats;

timeUs_t resumeRefreshAt = 0;
#define REFRESH_1S    1000 * 1000

static uint8_t armState;
static bool lastArmState;

static displayPort_t *osdDisplayPort;

#ifdef USE_ESC_SENSOR
static escSensorData_t *escDataCombined;
#endif

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

static const uint8_t osdElementDisplayOrder[] = {
    OSD_MAIN_BATT_VOLTAGE,
    OSD_RSSI_VALUE,
    OSD_CROSSHAIRS,
    OSD_HORIZON_SIDEBARS,
    OSD_ITEM_TIMER_1,
    OSD_ITEM_TIMER_2,
    OSD_REMAINING_TIME_ESTIMATE,
    OSD_FLYMODE,
    OSD_THROTTLE_POS,
    OSD_VTX_CHANNEL,
    OSD_CURRENT_DRAW,
    OSD_MAH_DRAWN,
    OSD_CRAFT_NAME,
    OSD_ALTITUDE,
    OSD_ROLL_PIDS,
    OSD_PITCH_PIDS,
    OSD_YAW_PIDS,
    OSD_POWER,
    OSD_PIDRATE_PROFILE,
    OSD_WARNINGS,
    OSD_AVG_CELL_VOLTAGE,
    OSD_DEBUG,
    OSD_PITCH_ANGLE,
    OSD_ROLL_ANGLE,
    OSD_MAIN_BATT_USAGE,
    OSD_DISARMED,
    OSD_NUMERICAL_HEADING,
    OSD_NUMERICAL_VARIO,
    OSD_COMPASS_BAR,
    OSD_ANTI_GRAVITY,
    OSD_MOTOR_DIAG,
    OSD_FLIP_ARROW,
#ifdef USE_RTC_TIME
    OSD_RTC_DATETIME,
#endif
#ifdef USE_OSD_ADJUSTMENTS
    OSD_ADJUSTMENT_RANGE,
#endif
#ifdef USE_ADC_INTERNAL
    OSD_CORE_TEMPERATURE,
#endif
#ifdef USE_RX_LINK_QUALITY_INFO
    OSD_LINK_QUALITY,
#endif
};

PG_REGISTER_WITH_RESET_FN(osdConfig_t, osdConfig, PG_OSD_CONFIG, 3);

/**
 * Gets the correct altitude symbol for the current unit system
 */
static char osdGetMetersToSelectedUnitSymbol(void)
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
        // Calculate a symbol offset using cell voltage over full cell voltage range
        const int symOffset = scaleRange(cellVoltage, batteryConfig()->vbatmincellvoltage * 10, batteryConfig()->vbatmaxcellvoltage * 10, 0, 7);
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

#if defined(USE_ADC_INTERNAL) || defined(USE_ESC_SENSOR)
STATIC_UNIT_TESTED int osdConvertTemperatureToSelectedUnit(int tempInDegreesCelcius)
{
    switch (osdConfig()->units) {
    case OSD_UNIT_IMPERIAL:
        return lrintf(((tempInDegreesCelcius * 9.0f) / 5) + 32);
    default:
        return tempInDegreesCelcius;
    }
}

static char osdGetTemperatureSymbolForSelectedUnit(void)
{
    switch (osdConfig()->units) {
    case OSD_UNIT_IMPERIAL:
        return 'F';
    default:
        return 'C';
    }
}
#endif

static void osdFormatAltitudeString(char * buff, int32_t altitudeCm)
{
    const int alt = osdGetMetersToSelectedUnit(altitudeCm) / 10;

    tfp_sprintf(buff, "%5d %c", alt, osdGetMetersToSelectedUnitSymbol());
    buff[5] = buff[4];
    buff[4] = '.';
}

static void osdFormatPID(char * buff, const char * label, const pidf_t * pid)
{
    tfp_sprintf(buff, "%s %3d %3d %3d", label, pid->P, pid->I, pid->D);
}

static uint8_t osdGetHeadingIntoDiscreteDirections(int heading, unsigned directions)
{
    heading += FULL_CIRCLE;  // Ensure positive value

    // Split input heading 0..359 into sectors 0..(directions-1), but offset
    // by half a sector so that sector 0 gets centered around heading 0.
    // We multiply heading by directions to not loose precision in divisions
    // In this way each segment will be a FULL_CIRCLE length
    int direction = (heading * directions + FULL_CIRCLE / 2) /  FULL_CIRCLE; // scale with rounding
    direction %= directions; // normalize

    return direction; // return segment number
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

STATIC_UNIT_TESTED void osdFormatTimer(char *buff, bool showSymbol, bool usePrecision, int timerIndex)
{
    const uint16_t timer = osdConfig()->timers[timerIndex];
    const uint8_t src = OSD_TIMER_SRC(timer);

    if (showSymbol) {
        *(buff++) = osdGetTimerSymbol(src);
    }

    osdFormatTime(buff, (usePrecision ? OSD_TIMER_PRECISION(timer) : OSD_TIMER_PREC_SECOND), osdGetTimerValue(src));
}

#ifdef USE_GPS
static void osdFormatCoordinate(char *buff, char sym, int32_t val)
{
    // latitude maximum integer width is 3 (-90).
    // longitude maximum integer width is 4 (-180).
    // We show 7 decimals, so we need to use 12 characters:
    // eg: s-180.1234567z   s=symbol, z=zero terminator, decimal separator  between 0 and 1

    static const int coordinateMaxLength = 13;//12 for the number (4 + dot + 7) + 1 for the symbol

    buff[0] = sym;
    const int32_t integerPart = val / GPS_DEGREES_DIVIDER;
    const int32_t decimalPart = labs(val % GPS_DEGREES_DIVIDER);
    const int written = tfp_sprintf(buff + 1, "%d.%07d", integerPart, decimalPart);
    // pad with blanks to coordinateMaxLength
    for (int pos = 1 + written; pos < coordinateMaxLength; ++pos) {
        buff[pos] = SYM_BLANK;
    }
    buff[coordinateMaxLength] = '\0';
}
#endif // USE_GPS

#ifdef USE_RTC_TIME
static bool osdFormatRtcDateTime(char *buffer)
{
    dateTime_t dateTime;
    if (!rtcGetDateTime(&dateTime)) {
        buffer[0] = '\0';

        return false;
    }

    dateTimeFormatLocalShort(buffer, &dateTime);

    return true;
}
#endif

static void osdFormatMessage(char *buff, size_t size, const char *message)
{
    memset(buff, SYM_BLANK, size);
    if (message) {
        memcpy(buff, message, strlen(message));
    }
    // Ensure buff is zero terminated
    buff[size - 1] = '\0';
}

void osdStatSetState(uint8_t statIndex, bool enabled)
{
    if (enabled) {
        osdConfigMutable()->enabled_stats |= (1 << statIndex);
    } else {
        osdConfigMutable()->enabled_stats &= ~(1 << statIndex);
    }
}

bool osdStatGetState(uint8_t statIndex)
{
    return osdConfig()->enabled_stats & (1 << statIndex);
}

void osdWarnSetState(uint8_t warningIndex, bool enabled)
{
    if (enabled) {
        osdConfigMutable()->enabledWarnings |= (1 << warningIndex);
    } else {
        osdConfigMutable()->enabledWarnings &= ~(1 << warningIndex);
    }
}

bool osdWarnGetState(uint8_t warningIndex)
{
    return osdConfig()->enabledWarnings & (1 << warningIndex);
}

static bool osdDrawSingleElement(uint8_t item)
{
    if (!VISIBLE(osdConfig()->item_pos[item]) || BLINK(item)) {
        return false;
    }

    uint8_t elemPosX = OSD_X(osdConfig()->item_pos[item]);
    uint8_t elemPosY = OSD_Y(osdConfig()->item_pos[item]);
    char buff[OSD_ELEMENT_BUFFER_LENGTH] = "";

    switch (item) {
    case OSD_FLIP_ARROW: 
        {
            const int angleR = attitude.values.roll / 10;
            const int angleP = attitude.values.pitch / 10; // still gotta update all angleR and angleP pointers.
            if (isFlipOverAfterCrashActive()) {
                if (angleP > 0 && ((angleR > 175 && angleR < 180) || (angleR > -180 && angleR < -175))) {
                    buff[0] = SYM_ARROW_SOUTH;
                } else if (angleP > 0 && angleR > 0 && angleR < 175) {
                    buff[0] = (SYM_ARROW_EAST + 2);
                } else if (angleP > 0 && angleR < 0 && angleR > -175) {
                    buff[0] = (SYM_ARROW_WEST + 2);
                } else if (angleP <= 0 && ((angleR > 175 && angleR < 180) || (angleR > -180 && angleR < -175))) {
                    buff[0] = SYM_ARROW_NORTH;
                } else if (angleP <= 0 && angleR > 0 && angleR < 175) {
                    buff[0] = (SYM_ARROW_NORTH + 2);
                } else if (angleP <= 0 && angleR < 0 && angleR > -175) {
                    buff[0] = (SYM_ARROW_SOUTH + 2);
                }
            } else {
                buff[0] = ' ';
            }
            buff[1] = '\0';
            break;
        }
    case OSD_RSSI_VALUE:
        {
            uint16_t osdRssi = getRssi() * 100 / 1024; // change range
            if (osdRssi >= 100) {
                osdRssi = 99;
            }

            tfp_sprintf(buff, "%c%2d", SYM_RSSI, osdRssi);
            break;
        }

#ifdef USE_RX_LINK_QUALITY_INFO
    case OSD_LINK_QUALITY:
        {
            // change range to 0-9 (two sig. fig. adds little extra value, also reduces screen estate)
            uint8_t osdLinkQuality = rxGetLinkQuality() * 10 / LINK_QUALITY_MAX_VALUE;
            if (osdLinkQuality >= 10) {
                osdLinkQuality = 9;
            }

            tfp_sprintf(buff, "%1d", osdLinkQuality);
            break;
        }
#endif

    case OSD_MAIN_BATT_VOLTAGE:
        buff[0] = osdGetBatterySymbol(osdGetBatteryAverageCellVoltage());
        tfp_sprintf(buff + 1, "%2d.%1d%c", getBatteryVoltage() / 10, getBatteryVoltage() % 10, SYM_VOLT);
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

#ifdef USE_GPS
    case OSD_GPS_SATS:
        tfp_sprintf(buff, "%c%c%2d", SYM_SAT_L, SYM_SAT_R, gpsSol.numSat);
        break;

    case OSD_GPS_SPEED:
        // FIXME ideally we want to use SYM_KMH symbol but it's not in the font any more, so we use K (M for MPH)
        switch (osdConfig()->units) {
        case OSD_UNIT_IMPERIAL:
            tfp_sprintf(buff, "%3dM", CM_S_TO_MPH(gpsSol.groundSpeed));
            break;
        default:
            tfp_sprintf(buff, "%3dK", CM_S_TO_KM_H(gpsSol.groundSpeed));
            break;
        }
        break;

    case OSD_GPS_LAT:
        // The SYM_LAT symbol in the actual font contains only blank, so we use the SYM_ARROW_NORTH
        osdFormatCoordinate(buff, SYM_ARROW_NORTH, gpsSol.llh.lat);
        break;

    case OSD_GPS_LON:
        // The SYM_LON symbol in the actual font contains only blank, so we use the SYM_ARROW_EAST
        osdFormatCoordinate(buff, SYM_ARROW_EAST, gpsSol.llh.lon);
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

    case OSD_HOME_DIST:
        if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
            const int32_t distance = osdGetMetersToSelectedUnit(GPS_distanceToHome);
            tfp_sprintf(buff, "%d%c", distance, osdGetMetersToSelectedUnitSymbol());
        } else {
            // We use this symbol when we don't have a FIX
            buff[0] = SYM_COLON;
            // overwrite any previous distance with blanks
            memset(buff + 1, SYM_BLANK, 6);
            buff[7] = '\0';
        }
        break;

#endif // GPS

    case OSD_COMPASS_BAR:
        memcpy(buff, compassBar + osdGetHeadingIntoDiscreteDirections(DECIDEGREES_TO_DEGREES(attitude.values.yaw), 16), 9);
        buff[9] = 0;
        break;

    case OSD_ALTITUDE:
        osdFormatAltitudeString(buff, getEstimatedAltitudeCm());
        break;

    case OSD_ITEM_TIMER_1:
    case OSD_ITEM_TIMER_2:
        osdFormatTimer(buff, true, true, item - OSD_ITEM_TIMER_1);
        break;

    case OSD_REMAINING_TIME_ESTIMATE:
        {
            const int mAhDrawn = getMAhDrawn();

            if (mAhDrawn <= 0.1 * osdConfig()->cap_alarm) {  // also handles the mAhDrawn == 0 condition
                tfp_sprintf(buff, "--:--");
            } else if (mAhDrawn > osdConfig()->cap_alarm) {
                tfp_sprintf(buff, "00:00");
            } else {
                const int remaining_time = (int)((osdConfig()->cap_alarm - mAhDrawn) * ((float)flyTime) / mAhDrawn);
                osdFormatTime(buff, OSD_TIMER_PREC_SECOND, remaining_time);
            }
            break;
        }

    case OSD_FLYMODE:
        {
            // Note that flight mode display has precedence in what to display.
            //  1. FS
            //  2. GPS RESCUE
            //  3. ANGLE, HORIZON, ACRO TRAINER
            //  4. AIR
            //  5. ACRO

            if (FLIGHT_MODE(FAILSAFE_MODE)) {
                strcpy(buff, "!FS!");
            } else if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
                strcpy(buff, "RESC");
            } else if (FLIGHT_MODE(HEADFREE_MODE)) {
                strcpy(buff, "HEAD");
            } else if (FLIGHT_MODE(ANGLE_MODE)) {
                strcpy(buff, "STAB");
            } else if (FLIGHT_MODE(HORIZON_MODE)) {
                strcpy(buff, "HOR ");
            } else if (IS_RC_MODE_ACTIVE(BOXACROTRAINER)) {
                strcpy(buff, "ATRN");
            } else if (airmodeIsEnabled()) {
                strcpy(buff, "AIR ");
            } else {
                strcpy(buff, "ACRO");
            }

            break;
        }

    case OSD_ANTI_GRAVITY:
        {
            if (pidOsdAntiGravityActive()) {
                strcpy(buff, "AG");
            }

            break;
        }
		
    case OSD_MOTOR_DIAG:
        if(areMotorsRunning()) {
            int maxIdx = 0;
            int i = 0;
            for(; i < getMotorCount(); i++) {
                if(motor[i] > motor[maxIdx]) {
                    maxIdx = i;
                }
                buff[i] =  0x88 - scaleRange(motor[i], motorOutputLow, motorOutputHigh, 0, 8);
            }
            buff[i] = '\0';
        }
        break;

    case OSD_CRAFT_NAME:
        // This does not strictly support iterative updating if the craft name changes at run time. But since the craft name is not supposed to be changing this should not matter, and blanking the entire length of the craft name string on update will make it impossible to configure elements to be displayed on the right hand side of the craft name.
        //TODO: When iterative updating is implemented, change this so the craft name is only printed once whenever the OSD 'flight' screen is entered.

        if (strlen(pilotConfig()->name) == 0) {
            strcpy(buff, "CRAFT_NAME");
        } else {
            unsigned i;
            for (i = 0; i < MAX_NAME_LENGTH; i++) {
                if (pilotConfig()->name[i]) {
                    buff[i] = toupper((unsigned char)pilotConfig()->name[i]);
                } else {
                    break;
                }    
            }    
            buff[i] = '\0';
        }

        break;

    case OSD_THROTTLE_POS:
        buff[0] = SYM_THR;
        buff[1] = SYM_THR1;
        tfp_sprintf(buff + 2, "%3d", (constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN));
        break;

#if defined(USE_VTX_COMMON)
    case OSD_VTX_CHANNEL:
        {
            const char vtxBandLetter = vtx58BandLetter[vtxSettingsConfig()->band];
            const char *vtxChannelName = vtx58ChannelNames[vtxSettingsConfig()->channel];
            uint8_t vtxPower = vtxSettingsConfig()->power;
            const vtxDevice_t *vtxDevice = vtxCommonDevice();
            if (vtxDevice && vtxSettingsConfig()->lowPowerDisarm) {
                vtxCommonGetPowerIndex(vtxDevice, &vtxPower);
            }
            tfp_sprintf(buff, "%c:%s:%1d", vtxBandLetter, vtxChannelName, vtxPower);
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
            if (maxPitch > 0) {
                pitchAngle = ((pitchAngle * 25) / maxPitch);
            }
            pitchAngle -= 41; // 41 = 4 * AH_SYMBOL_COUNT + 5

            for (int x = -4; x <= 4; x++) {
                const int y = ((-rollAngle * x) / 64) - pitchAngle;
                if (y >= 0 && y <= 81) {
                    displayWriteChar(osdDisplayPort, elemPosX + x, elemPosY + (y / AH_SYMBOL_COUNT), (SYM_AH_BAR9_0 + (y % AH_SYMBOL_COUNT)));
                }
            }

            return true;
        }

    case OSD_HORIZON_SIDEBARS:
        {
            // Draw AH sides
            const int8_t hudwidth = AH_SIDEBAR_WIDTH_POS;
            const int8_t hudheight = AH_SIDEBAR_HEIGHT_POS;
            for (int y = -hudheight; y <= hudheight; y++) {
                displayWriteChar(osdDisplayPort, elemPosX - hudwidth, elemPosY + y, SYM_AH_DECORATION);
                displayWriteChar(osdDisplayPort, elemPosX + hudwidth, elemPosY + y, SYM_AH_DECORATION);
            }

            // AH level indicators
            displayWriteChar(osdDisplayPort, elemPosX - hudwidth + 1, elemPosY, SYM_AH_LEFT);
            displayWriteChar(osdDisplayPort, elemPosX + hudwidth - 1, elemPosY, SYM_AH_RIGHT);

            return true;
        }

    case OSD_G_FORCE:
        {
            const int gForce = lrintf(osdGForce * 10);
            tfp_sprintf(buff, "%01d.%01dG", gForce / 10, gForce % 10);
            break;
        }

    case OSD_ROLL_PIDS:
        osdFormatPID(buff, "ROL", &currentPidProfile->pid[PID_ROLL]);
        break;

    case OSD_PITCH_PIDS:
        osdFormatPID(buff, "PIT", &currentPidProfile->pid[PID_PITCH]);
        break;

    case OSD_YAW_PIDS:
        osdFormatPID(buff, "YAW", &currentPidProfile->pid[PID_YAW]);
        break;

    case OSD_POWER:
        tfp_sprintf(buff, "%4dW", getAmperage() * getBatteryVoltage() / 1000);
        break;

    case OSD_PIDRATE_PROFILE:
        tfp_sprintf(buff, "%d-%d", getCurrentPidProfileIndex() + 1, getCurrentControlRateProfileIndex() + 1);
        break;

    case OSD_WARNINGS:
        {

#define OSD_WARNINGS_MAX_SIZE 11
#define OSD_FORMAT_MESSAGE_BUFFER_SIZE (OSD_WARNINGS_MAX_SIZE + 1)

            STATIC_ASSERT(OSD_FORMAT_MESSAGE_BUFFER_SIZE <= sizeof(buff), osd_warnings_size_exceeds_buffer_size);

            const batteryState_e batteryState = getBatteryState();
            const timeUs_t currentTimeUs = micros();

            static timeUs_t armingDisabledUpdateTimeUs;
            static unsigned armingDisabledDisplayIndex;

            // Cycle through the arming disabled reasons
            if (osdWarnGetState(OSD_WARNING_ARMING_DISABLE)) {
                if (IS_RC_MODE_ACTIVE(BOXARM) && isArmingDisabled()) {
                    const armingDisableFlags_e armSwitchOnlyFlag = 1 << (ARMING_DISABLE_FLAGS_COUNT - 1);
                    armingDisableFlags_e flags = getArmingDisableFlags();

                    // Remove the ARMSWITCH flag unless it's the only one
                    if ((flags & armSwitchOnlyFlag) && (flags != armSwitchOnlyFlag)) {
                        flags -= armSwitchOnlyFlag;
                    }

                    // Rotate to the next arming disabled reason after a 0.5 second time delay
                    // or if the current flag is no longer set
                    if ((currentTimeUs - armingDisabledUpdateTimeUs > 5e5) || !(flags & (1 << armingDisabledDisplayIndex))) {
                        if (armingDisabledUpdateTimeUs == 0) {
                            armingDisabledDisplayIndex = ARMING_DISABLE_FLAGS_COUNT - 1;
                        }
                        armingDisabledUpdateTimeUs = currentTimeUs;

                        do {
                            if (++armingDisabledDisplayIndex >= ARMING_DISABLE_FLAGS_COUNT) {
                                armingDisabledDisplayIndex = 0;
                            }
                        } while (!(flags & (1 << armingDisabledDisplayIndex)));
                    }

                    osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, armingDisableFlagNames[armingDisabledDisplayIndex]);
                    break;
                } else {
                    armingDisabledUpdateTimeUs = 0;
                }
            }

#ifdef USE_DSHOT
            if (isTryingToArm() && !ARMING_FLAG(ARMED)) {
                int armingDelayTime = (getLastDshotBeaconCommandTimeUs() + DSHOT_BEACON_GUARD_DELAY_US - currentTimeUs) / 1e5;
                if (armingDelayTime < 0) {
                    armingDelayTime = 0;
                }
                if (armingDelayTime >= (DSHOT_BEACON_GUARD_DELAY_US / 1e5 - 5)) {
                    osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, " BEACON ON"); // Display this message for the first 0.5 seconds
                } else {
                    char armingDelayMessage[OSD_FORMAT_MESSAGE_BUFFER_SIZE];
                    tfp_sprintf(armingDelayMessage, "ARM IN %d.%d", armingDelayTime / 10, armingDelayTime % 10);
                    osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, armingDelayMessage);
                }
                break;
            }
#endif
            if (osdWarnGetState(OSD_WARNING_FAIL_SAFE) && failsafeIsActive()) {
                osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "FAIL SAFE");
                break;
            }

            if (osdWarnGetState(OSD_WARNING_BATTERY_CRITICAL) && batteryState == BATTERY_CRITICAL) {
                osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, " LAND NOW");
                break;
            }

            // Show warning if in HEADFREE flight mode
            if (FLIGHT_MODE(HEADFREE_MODE)) {
                osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "HEADFREE");
                break;
            }

#ifdef USE_ADC_INTERNAL
            const int16_t coreTemperature = getCoreTemperatureCelsius();
            if (osdWarnGetState(OSD_WARNING_CORE_TEMPERATURE) && coreTemperature >= osdConfig()->core_temp_alarm) {
                char coreTemperatureWarningMsg[OSD_FORMAT_MESSAGE_BUFFER_SIZE];
                tfp_sprintf(coreTemperatureWarningMsg, "CORE: %3d%c", osdConvertTemperatureToSelectedUnit(coreTemperature), osdGetTemperatureSymbolForSelectedUnit());

                osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, coreTemperatureWarningMsg);

                break;
            }
#endif

#ifdef USE_ESC_SENSOR
            // Show warning if we lose motor output, the ESC is overheating or excessive current draw
            if (featureIsEnabled(FEATURE_ESC_SENSOR) && osdWarnGetState(OSD_WARNING_ESC_FAIL)) {
                char escWarningMsg[OSD_FORMAT_MESSAGE_BUFFER_SIZE];
                unsigned pos = 0;
                
                const char *title = "ESC";

                // center justify message
                while (pos < (OSD_WARNINGS_MAX_SIZE - (strlen(title) + getMotorCount())) / 2) {
                    escWarningMsg[pos++] = ' ';
                }

                strcpy(escWarningMsg + pos, title);
                pos += strlen(title);

                unsigned i = 0;
                unsigned escWarningCount = 0;
                while (i < getMotorCount() && pos < OSD_FORMAT_MESSAGE_BUFFER_SIZE - 1) {
                    escSensorData_t *escData = getEscSensorData(i);
                    const char motorNumber = '1' + i;
                    // if everything is OK just display motor number else R, T or C
                    char warnFlag = motorNumber;
                    if (ARMING_FLAG(ARMED) && osdConfig()->esc_rpm_alarm != ESC_RPM_ALARM_OFF && calcEscRpm(escData->rpm) <= osdConfig()->esc_rpm_alarm) {
                        warnFlag = 'R';
                    }
                    if (osdConfig()->esc_temp_alarm != ESC_TEMP_ALARM_OFF && escData->temperature >= osdConfig()->esc_temp_alarm) {
                        warnFlag = 'T';
                    }
                    if (ARMING_FLAG(ARMED) && osdConfig()->esc_current_alarm != ESC_CURRENT_ALARM_OFF && escData->current >= osdConfig()->esc_current_alarm) {
                        warnFlag = 'C';
                    }

                    escWarningMsg[pos++] = warnFlag;

                    if (warnFlag != motorNumber) {
                        escWarningCount++;
                    }

                    i++;
                }

                escWarningMsg[pos] = '\0';

                if (escWarningCount > 0) {
                    osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, escWarningMsg);
                    break;
                }
            }
#endif

            // Warn when in flip over after crash mode
            if (osdWarnGetState(OSD_WARNING_CRASH_FLIP) && isFlipOverAfterCrashActive()) {
                osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "CRASH FLIP");
                break;
            }

#ifdef USE_LAUNCH_CONTROL
            // Warn when in launch control mode
            if (osdWarnGetState(OSD_WARNING_LAUNCH_CONTROL) && isLaunchControlActive()) {
                if (sensors(SENSOR_ACC)) {
                    char launchControlMsg[OSD_FORMAT_MESSAGE_BUFFER_SIZE];
                    const int pitchAngle = constrain((attitude.raw[FD_PITCH] - accelerometerConfig()->accelerometerTrims.raw[FD_PITCH]) / 10, -90, 90);
                    tfp_sprintf(launchControlMsg, "LAUNCH %d", pitchAngle);
                    osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, launchControlMsg);
                } else {
                    osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "LAUNCH");
                }
                break;
            }
#endif

            if (osdWarnGetState(OSD_WARNING_BATTERY_WARNING) && batteryState == BATTERY_WARNING) {
                osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "LOW BATTERY");
                break;
            }

#ifdef USE_RC_SMOOTHING_FILTER
            // Show warning if rc smoothing hasn't initialized the filters
            if (osdWarnGetState(OSD_WARNING_RC_SMOOTHING) && ARMING_FLAG(ARMED) && !rcSmoothingInitializationComplete()) {
                osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "RCSMOOTHING");
                break;
            }
#endif

            // Show warning if battery is not fresh
            if (osdWarnGetState(OSD_WARNING_BATTERY_NOT_FULL) && !ARMING_FLAG(WAS_EVER_ARMED) && (getBatteryState() == BATTERY_OK)
                  && getBatteryAverageCellVoltage() < batteryConfig()->vbatfullcellvoltage) {
                osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "BATT < FULL");
                break;
            }

            // Visual beeper
            if (osdWarnGetState(OSD_WARNING_VISUAL_BEEPER) && showVisualBeeper) {
                osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, "  * * * *");
                break;
            }

            osdFormatMessage(buff, OSD_FORMAT_MESSAGE_BUFFER_SIZE, NULL);
            break;
        }

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
            // Set length of indicator bar
            #define MAIN_BATT_USAGE_STEPS 11 // Use an odd number so the bar can be centered.

            // Calculate constrained value
            const float value = constrain(batteryConfig()->batteryCapacity - getMAhDrawn(), 0, batteryConfig()->batteryCapacity);

            // Calculate mAh used progress
            const uint8_t mAhUsedProgress = ceilf((value / (batteryConfig()->batteryCapacity / MAIN_BATT_USAGE_STEPS)));

            // Create empty battery indicator bar
            buff[0] = SYM_PB_START;
            for (int i = 1; i <= MAIN_BATT_USAGE_STEPS; i++) {
                buff[i] = i <= mAhUsedProgress ? SYM_PB_FULL : SYM_PB_EMPTY;
            }
            buff[MAIN_BATT_USAGE_STEPS + 1] = SYM_PB_CLOSE;
            if (mAhUsedProgress > 0 && mAhUsedProgress < MAIN_BATT_USAGE_STEPS) {
                buff[1 + mAhUsedProgress] = SYM_PB_END;
            }
            buff[MAIN_BATT_USAGE_STEPS+2] = '\0';
            break;
        }

    case OSD_DISARMED:
        if (!ARMING_FLAG(ARMED)) {
            tfp_sprintf(buff, "DISARMED");
        } else {
            if (!lastArmState) {  // previously disarmed - blank out the message one time
                tfp_sprintf(buff, "        ");
            }
        }
        break;

    case OSD_NUMERICAL_HEADING:
        {
            const int heading = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
            tfp_sprintf(buff, "%c%03d", osdGetDirectionSymbolFromHeading(heading), heading);
            break;
        }
#ifdef USE_VARIO
    case OSD_NUMERICAL_VARIO:
        {
            const int verticalSpeed = osdGetMetersToSelectedUnit(getEstimatedVario());
            const char directionSymbol = verticalSpeed < 0 ? SYM_ARROW_SOUTH : SYM_ARROW_NORTH;
            tfp_sprintf(buff, "%c%01d.%01d", directionSymbol, abs(verticalSpeed / 100), abs((verticalSpeed % 100) / 10));
            break;
        }
#endif

#ifdef USE_ESC_SENSOR
    case OSD_ESC_TMP:
        if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
            tfp_sprintf(buff, "%3d%c", osdConvertTemperatureToSelectedUnit(escDataCombined->temperature), osdGetTemperatureSymbolForSelectedUnit());
        }
        break;

    case OSD_ESC_RPM:
        if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
            tfp_sprintf(buff, "%5d", escDataCombined == NULL ? 0 : calcEscRpm(escDataCombined->rpm));
        }
        break;
#endif

#ifdef USE_RTC_TIME
    case OSD_RTC_DATETIME:
        osdFormatRtcDateTime(&buff[0]);
        break;
#endif

#ifdef USE_OSD_ADJUSTMENTS
    case OSD_ADJUSTMENT_RANGE:
        if (getAdjustmentsRangeName()) {
            tfp_sprintf(buff, "%s: %3d", getAdjustmentsRangeName(), getAdjustmentsRangeValue());
        }
        break;
#endif

#ifdef USE_ADC_INTERNAL
    case OSD_CORE_TEMPERATURE:
        tfp_sprintf(buff, "%3d%c", osdConvertTemperatureToSelectedUnit(getCoreTemperatureCelsius()), osdGetTemperatureSymbolForSelectedUnit());
        break;
#endif

#ifdef USE_BLACKBOX
    case OSD_LOG_STATUS:
        if (!isBlackboxDeviceWorking()) {
            tfp_sprintf(buff, "L-");
        } else if (isBlackboxDeviceFull()) {
            tfp_sprintf(buff, "L>");
        } else {
            tfp_sprintf(buff, "L%d", blackboxGetLogNumber());
        }
        break;
#endif

    default:
        return false;
    }

    displayWrite(osdDisplayPort, elemPosX, elemPosY, buff);

    return true;
}

static void osdDrawElements(void)
{
    displayClearScreen(osdDisplayPort);

    // Hide OSD when OSDSW mode is active
    if (IS_RC_MODE_ACTIVE(BOXOSD)) {
        return;
    }

    osdGForce = 0.0f;
    if (sensors(SENSOR_ACC)) {
        // only calculate the G force if the element is visible or the stat is enabled
        if (VISIBLE(osdConfig()->item_pos[OSD_G_FORCE]) || osdStatGetState(OSD_STAT_MAX_G_FORCE)) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                const float a = accAverage[axis];
                osdGForce += a * a;
            }
            osdGForce = sqrtf(osdGForce) * acc.dev.acc_1G_rec;
        }
        osdDrawSingleElement(OSD_ARTIFICIAL_HORIZON);
        osdDrawSingleElement(OSD_G_FORCE);
    }


    for (unsigned i = 0; i < sizeof(osdElementDisplayOrder); i++) {
        osdDrawSingleElement(osdElementDisplayOrder[i]);
    }

#ifdef USE_GPS
    if (sensors(SENSOR_GPS)) {
        osdDrawSingleElement(OSD_GPS_SATS);
        osdDrawSingleElement(OSD_GPS_SPEED);
        osdDrawSingleElement(OSD_GPS_LAT);
        osdDrawSingleElement(OSD_GPS_LON);
        osdDrawSingleElement(OSD_HOME_DIST);
        osdDrawSingleElement(OSD_HOME_DIR);
    }
#endif // GPS

#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        osdDrawSingleElement(OSD_ESC_TMP);
        osdDrawSingleElement(OSD_ESC_RPM);
    }
#endif

#ifdef USE_BLACKBOX
    if (IS_RC_MODE_ACTIVE(BOXBLACKBOX)) {
        osdDrawSingleElement(OSD_LOG_STATUS);
    }
#endif
}

void pgResetFn_osdConfig(osdConfig_t *osdConfig)
{
    // Position elements near centre of screen and disabled by default
    for (int i = 0; i < OSD_ITEM_COUNT; i++) {
        osdConfig->item_pos[i] = OSD_POS(10, 7);
    }

    // Always enable warnings elements by default
    osdConfig->item_pos[OSD_WARNINGS] = OSD_POS(9, 10) | VISIBLE_FLAG;

    // Default to old fixed positions for these elements
    osdConfig->item_pos[OSD_CROSSHAIRS]         = OSD_POS(13, 6);
    osdConfig->item_pos[OSD_ARTIFICIAL_HORIZON] = OSD_POS(14, 2);
    osdConfig->item_pos[OSD_HORIZON_SIDEBARS]   = OSD_POS(14, 6);

    // Enable the default stats
    osdConfig->enabled_stats = 0; // reset all to off and enable only a few initially
    osdStatSetState(OSD_STAT_MAX_SPEED, true);
    osdStatSetState(OSD_STAT_MIN_BATTERY, true);
    osdStatSetState(OSD_STAT_MIN_RSSI, true);
    osdStatSetState(OSD_STAT_MAX_CURRENT, true);
    osdStatSetState(OSD_STAT_USED_MAH, true);
    osdStatSetState(OSD_STAT_BLACKBOX, true);
    osdStatSetState(OSD_STAT_BLACKBOX_NUMBER, true);
    osdStatSetState(OSD_STAT_TIMER_2, true);

    osdConfig->units = OSD_UNIT_METRIC;

    // Enable all warnings by default
    for (int i=0; i < OSD_WARNING_COUNT; i++) {
        osdWarnSetState(i, true);
    }

    osdConfig->timers[OSD_TIMER_1] = OSD_TIMER(OSD_TIMER_SRC_ON, OSD_TIMER_PREC_SECOND, 10);
    osdConfig->timers[OSD_TIMER_2] = OSD_TIMER(OSD_TIMER_SRC_TOTAL_ARMED, OSD_TIMER_PREC_SECOND, 10);

    osdConfig->rssi_alarm = 20;
    osdConfig->cap_alarm  = 2200;
    osdConfig->alt_alarm  = 100; // meters or feet depend on configuration
    osdConfig->esc_temp_alarm = ESC_TEMP_ALARM_OFF; // off by default
    osdConfig->esc_rpm_alarm = ESC_RPM_ALARM_OFF; // off by default
    osdConfig->esc_current_alarm = ESC_CURRENT_ALARM_OFF; // off by default
    osdConfig->core_temp_alarm = 70; // a temperature above 70C should produce a warning, lockups have been reported above 80C

    osdConfig->ahMaxPitch = 20; // 20 degrees
    osdConfig->ahMaxRoll = 40; // 40 degrees
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
    if (!osdDisplayPortToUse) {
        return;
    }

    STATIC_ASSERT(OSD_POS_MAX == OSD_POS(31,31), OSD_POS_MAX_incorrect);

    osdDisplayPort = osdDisplayPortToUse;
#ifdef USE_CMS
    cmsDisplayPortRegister(osdDisplayPort);
#endif

    armState = ARMING_FLAG(ARMED);

    memset(blinkBits, 0, sizeof(blinkBits));

    displayClearScreen(osdDisplayPort);

    osdDrawLogo(3, 1);

    char string_buffer[30];
    tfp_sprintf(string_buffer, "V%s", FC_VERSION_STRING);
    displayWrite(osdDisplayPort, 20, 6, string_buffer);
#ifdef USE_CMS
    displayWrite(osdDisplayPort, 7, 8,  CMS_STARTUP_HELP_TEXT1);
    displayWrite(osdDisplayPort, 11, 9, CMS_STARTUP_HELP_TEXT2);
    displayWrite(osdDisplayPort, 11, 10, CMS_STARTUP_HELP_TEXT3);
#endif

#ifdef USE_RTC_TIME
    char dateTimeBuffer[FORMATTED_DATE_TIME_BUFSIZE];
    if (osdFormatRtcDateTime(&dateTimeBuffer[0])) {
        displayWrite(osdDisplayPort, 5, 12, dateTimeBuffer);
    }
#endif

    displayResync(osdDisplayPort);

    resumeRefreshAt = micros() + (4 * REFRESH_1S);
}

bool osdInitialized(void)
{
    return osdDisplayPort;
}

void osdUpdateAlarms(void)
{
    // This is overdone?

    int32_t alt = osdGetMetersToSelectedUnit(getEstimatedAltitudeCm()) / 100;

    if (getRssiPercent() < osdConfig()->rssi_alarm) {
        SET_BLINK(OSD_RSSI_VALUE);
    } else {
        CLR_BLINK(OSD_RSSI_VALUE);
    }

    // Determine if the OSD_WARNINGS should blink
    if (getBatteryState() != BATTERY_OK
           && (osdWarnGetState(OSD_WARNING_BATTERY_CRITICAL) || osdWarnGetState(OSD_WARNING_BATTERY_WARNING))
#ifdef USE_DSHOT
           && (!isTryingToArm())
#endif
       ) {
        SET_BLINK(OSD_WARNINGS);
    } else {
        CLR_BLINK(OSD_WARNINGS);
    }

    if (getBatteryState() == BATTERY_OK) {
        CLR_BLINK(OSD_MAIN_BATT_VOLTAGE);
        CLR_BLINK(OSD_AVG_CELL_VOLTAGE);
    } else {
        SET_BLINK(OSD_MAIN_BATT_VOLTAGE);
        SET_BLINK(OSD_AVG_CELL_VOLTAGE);
    }

    if (STATE(GPS_FIX) == 0) {
        SET_BLINK(OSD_GPS_SATS);
    } else {
        CLR_BLINK(OSD_GPS_SATS);
    }

    for (int i = 0; i < OSD_TIMER_COUNT; i++) {
        const uint16_t timer = osdConfig()->timers[i];
        const timeUs_t time = osdGetTimerValue(OSD_TIMER_SRC(timer));
        const timeUs_t alarmTime = OSD_TIMER_ALARM(timer) * 60000000; // convert from minutes to us
        if (alarmTime != 0 && time >= alarmTime) {
            SET_BLINK(OSD_ITEM_TIMER_1 + i);
        } else {
            CLR_BLINK(OSD_ITEM_TIMER_1 + i);
        }
    }

    if (getMAhDrawn() >= osdConfig()->cap_alarm) {
        SET_BLINK(OSD_MAH_DRAWN);
        SET_BLINK(OSD_MAIN_BATT_USAGE);
        SET_BLINK(OSD_REMAINING_TIME_ESTIMATE);
    } else {
        CLR_BLINK(OSD_MAH_DRAWN);
        CLR_BLINK(OSD_MAIN_BATT_USAGE);
        CLR_BLINK(OSD_REMAINING_TIME_ESTIMATE);
    }

    if (alt >= osdConfig()->alt_alarm) {
        SET_BLINK(OSD_ALTITUDE);
    } else {
        CLR_BLINK(OSD_ALTITUDE);
    }

#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        // This works because the combined ESC data contains the maximum temperature seen amongst all ESCs
        if (osdConfig()->esc_temp_alarm != ESC_TEMP_ALARM_OFF && escDataCombined->temperature >= osdConfig()->esc_temp_alarm) {
            SET_BLINK(OSD_ESC_TMP);
        } else {
            CLR_BLINK(OSD_ESC_TMP);
        }
    }
#endif
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
    CLR_BLINK(OSD_REMAINING_TIME_ESTIMATE);
    CLR_BLINK(OSD_ESC_TMP);
}

static void osdResetStats(void)
{
    stats.max_current  = 0;
    stats.max_speed    = 0;
    stats.min_voltage  = 500;
    stats.min_rssi     = 99; // percent
    stats.max_altitude = 0;
    stats.max_distance = 0;
    stats.armed_time   = 0;
    stats.max_g_force  = 0;
    stats.max_esc_temp = 0;
    stats.max_esc_rpm  = 0;
    stats.min_link_quality = 99; // percent
}

static void osdUpdateStats(void)
{
    int16_t value = 0;
#ifdef USE_GPS
    switch (osdConfig()->units) {
    case OSD_UNIT_IMPERIAL:
        value = CM_S_TO_MPH(gpsSol.groundSpeed);
        break;
    default:
        value = CM_S_TO_KM_H(gpsSol.groundSpeed);
        break;
    }
#endif
    if (stats.max_speed < value) {
        stats.max_speed = value;
    }

    value = getBatteryVoltage();
    if (stats.min_voltage > value) {
        stats.min_voltage = value;
    }

    value = getAmperage() / 100;
    if (stats.max_current < value) {
        stats.max_current = value;
    }

    value = getRssiPercent();
    if (stats.min_rssi > value) {
        stats.min_rssi = value;
    }

    int32_t altitudeCm = getEstimatedAltitudeCm();
    if (stats.max_altitude < altitudeCm) {
        stats.max_altitude = altitudeCm;
    }

    if (stats.max_g_force < osdGForce) {
        stats.max_g_force = osdGForce;
    }

#ifdef USE_RX_LINK_QUALITY_INFO
    value = rxGetLinkQualityPercent();
    if (stats.min_link_quality > value) {
        stats.min_link_quality = value;
    }
#endif

#ifdef USE_GPS
    if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
        value = GPS_distanceToHome;

        if (stats.max_distance < GPS_distanceToHome) {
            stats.max_distance = GPS_distanceToHome;
        }
    }
#endif
#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        value = escDataCombined->temperature;
        if (stats.max_esc_temp < value) {
            stats.max_esc_temp = value;
        }
        value = calcEscRpm(escDataCombined->rpm);
        if (stats.max_esc_rpm < value) {
            stats.max_esc_rpm = value;
        }
    }
#endif
}

#ifdef USE_BLACKBOX

static void osdGetBlackboxStatusString(char * buff)
{
    bool storageDeviceIsWorking = isBlackboxDeviceWorking();
    uint32_t storageUsed = 0;
    uint32_t storageTotal = 0;

    switch (blackboxConfig()->device) {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        if (storageDeviceIsWorking) {
            storageTotal = sdcard_getMetadata()->numBlocks / 2000;
            storageUsed = storageTotal - (afatfs_getContiguousFreeSpace() / 1024000);
        }
        break;
#endif

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        if (storageDeviceIsWorking) {
            const flashGeometry_t *geometry = flashfsGetGeometry();
            storageTotal = geometry->totalSize / 1024;
            storageUsed = flashfsGetOffset() / 1024;
        }
        break;
#endif

    default:
        break;
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

/*
 * Test if there's some stat enabled
 */
static bool isSomeStatEnabled(void)
{
    return (osdConfig()->enabled_stats != 0);
}

// *** IMPORTANT ***
// The order of the OSD stats as displayed on-screen must match the osd_stats_e enumeration.
// This is because the fields are presented in the configurator in the order of the enumeration
// and we want the configuration order to match the on-screen display order.  If you change the
// display order you *must* update the osd_stats_e enumeration to match. Additionally the
// changes to the stats display order *must* be implemented in the configurator otherwise the
// stats selections will not be populated correctly and the settings will become corrupted.

static void osdShowStats(uint16_t endBatteryVoltage)
{
    uint8_t top = 2;
    char buff[OSD_ELEMENT_BUFFER_LENGTH];

    displayClearScreen(osdDisplayPort);
    displayWrite(osdDisplayPort, 2, top++, "  --- STATS ---");

    if (osdStatGetState(OSD_STAT_RTC_DATE_TIME)) {
        bool success = false;
#ifdef USE_RTC_TIME
        success = osdFormatRtcDateTime(&buff[0]);
#endif
        if (!success) {
            tfp_sprintf(buff, "NO RTC");
        }

        displayWrite(osdDisplayPort, 2, top++, buff);
    }

    if (osdStatGetState(OSD_STAT_TIMER_1)) {
        osdFormatTimer(buff, false, (OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_1]) == OSD_TIMER_SRC_ON ? false : true), OSD_TIMER_1);
        osdDisplayStatisticLabel(top++, osdTimerSourceNames[OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_1])], buff);
    }

    if (osdStatGetState(OSD_STAT_TIMER_2)) {
        osdFormatTimer(buff, false, (OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_2]) == OSD_TIMER_SRC_ON ? false : true), OSD_TIMER_2);
        osdDisplayStatisticLabel(top++, osdTimerSourceNames[OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_2])], buff);
    }

    if (osdStatGetState(OSD_STAT_MAX_SPEED) && STATE(GPS_FIX)) {
        itoa(stats.max_speed, buff, 10);
        osdDisplayStatisticLabel(top++, "MAX SPEED", buff);
    }

    if (osdStatGetState(OSD_STAT_MAX_DISTANCE)) {
        tfp_sprintf(buff, "%d%c", osdGetMetersToSelectedUnit(stats.max_distance), osdGetMetersToSelectedUnitSymbol());
        osdDisplayStatisticLabel(top++, "MAX DISTANCE", buff);
    }

    if (osdStatGetState(OSD_STAT_MIN_BATTERY)) {
        tfp_sprintf(buff, "%d.%1d%c", stats.min_voltage / 10, stats.min_voltage % 10, SYM_VOLT);
        osdDisplayStatisticLabel(top++, "MIN BATTERY", buff);
    }

    if (osdStatGetState(OSD_STAT_END_BATTERY)) {
        tfp_sprintf(buff, "%d.%1d%c", endBatteryVoltage / 10, endBatteryVoltage % 10, SYM_VOLT);
        osdDisplayStatisticLabel(top++, "END BATTERY", buff);
    }

    if (osdStatGetState(OSD_STAT_BATTERY)) {
        tfp_sprintf(buff, "%d.%1d%c", getBatteryVoltage() / 10, getBatteryVoltage() % 10, SYM_VOLT);
        osdDisplayStatisticLabel(top++, "BATTERY", buff);
    }

    if (osdStatGetState(OSD_STAT_MIN_RSSI)) {
        itoa(stats.min_rssi, buff, 10);
        strcat(buff, "%");
        osdDisplayStatisticLabel(top++, "MIN RSSI", buff);
    }

    if (batteryConfig()->currentMeterSource != CURRENT_METER_NONE) {
        if (osdStatGetState(OSD_STAT_MAX_CURRENT)) {
            itoa(stats.max_current, buff, 10);
            strcat(buff, "A");
            osdDisplayStatisticLabel(top++, "MAX CURRENT", buff);
        }

        if (osdStatGetState(OSD_STAT_USED_MAH)) {
            tfp_sprintf(buff, "%d%c", getMAhDrawn(), SYM_MAH);
            osdDisplayStatisticLabel(top++, "USED MAH", buff);
        }
    }

    if (osdStatGetState(OSD_STAT_MAX_ALTITUDE)) {
        osdFormatAltitudeString(buff, stats.max_altitude);
        osdDisplayStatisticLabel(top++, "MAX ALTITUDE", buff);
    }

#ifdef USE_BLACKBOX
    if (osdStatGetState(OSD_STAT_BLACKBOX) && blackboxConfig()->device && blackboxConfig()->device != BLACKBOX_DEVICE_SERIAL) {
        osdGetBlackboxStatusString(buff);
        osdDisplayStatisticLabel(top++, "BLACKBOX", buff);
    }

    if (osdStatGetState(OSD_STAT_BLACKBOX_NUMBER) && blackboxConfig()->device && blackboxConfig()->device != BLACKBOX_DEVICE_SERIAL) {
        itoa(blackboxGetLogNumber(), buff, 10);
        osdDisplayStatisticLabel(top++, "BB LOG NUM", buff);
    }
#endif

    if (osdStatGetState(OSD_STAT_MAX_G_FORCE) && sensors(SENSOR_ACC)) {
        const int gForce = lrintf(stats.max_g_force * 10);
        tfp_sprintf(buff, "%01d.%01dG", gForce / 10, gForce % 10);
        osdDisplayStatisticLabel(top++, "MAX G-FORCE", buff);
    }

#ifdef USE_ESC_SENSOR
    if (osdStatGetState(OSD_STAT_MAX_ESC_TEMP)) {
        tfp_sprintf(buff, "%3d%c", osdConvertTemperatureToSelectedUnit(stats.max_esc_temp), osdGetTemperatureSymbolForSelectedUnit());
        osdDisplayStatisticLabel(top++, "MAX ESC TEMP", buff);
    }

    if (osdStatGetState(OSD_STAT_MAX_ESC_RPM)) {
        itoa(stats.max_esc_rpm, buff, 10);
        osdDisplayStatisticLabel(top++, "MAX ESC RPM", buff);
    }
#endif

#ifdef USE_RX_LINK_QUALITY_INFO
    if (osdStatGetState(OSD_STAT_MIN_LINK_QUALITY)) {
        itoa(stats.min_link_quality, buff, 10);
        strcat(buff, "%");
        osdDisplayStatisticLabel(top++, "MIN LINK", buff);
    }
#endif
}

static void osdShowArmed(void)
{
    displayClearScreen(osdDisplayPort);
    displayWrite(osdDisplayPort, 12, 7, "ARMED");
}

STATIC_UNIT_TESTED void osdRefresh(timeUs_t currentTimeUs)
{
    static timeUs_t lastTimeUs = 0;
    static bool osdStatsEnabled = false;
    static bool osdStatsVisible = false;
    static timeUs_t osdStatsRefreshTimeUs;
    static uint16_t endBatteryVoltage;

    // detect arm/disarm
    if (armState != ARMING_FLAG(ARMED)) {
        if (ARMING_FLAG(ARMED)) {
            osdStatsEnabled = false;
            osdStatsVisible = false;
            osdResetStats();
            osdShowArmed();
            resumeRefreshAt = currentTimeUs + (REFRESH_1S / 2);
        } else if (isSomeStatEnabled()
                   && (!(getArmingDisableFlags() & ARMING_DISABLED_RUNAWAY_TAKEOFF)
                       || !VISIBLE(osdConfig()->item_pos[OSD_WARNINGS]))) { // suppress stats if runaway takeoff triggered disarm and WARNINGS element is visible
            osdStatsEnabled = true;
            resumeRefreshAt = currentTimeUs + (60 * REFRESH_1S);
            endBatteryVoltage = getBatteryVoltage();
        }

        armState = ARMING_FLAG(ARMED);
    }


    if (ARMING_FLAG(ARMED)) {
        osdUpdateStats();
        timeUs_t deltaT = currentTimeUs - lastTimeUs;
        flyTime += deltaT;
        stats.armed_time += deltaT;
    } else if (osdStatsEnabled) {  // handle showing/hiding stats based on OSD disable switch position
        if (displayIsGrabbed(osdDisplayPort)) {
            osdStatsEnabled = false;
            resumeRefreshAt = 0;
            stats.armed_time = 0;
        } else {
            if (IS_RC_MODE_ACTIVE(BOXOSD) && osdStatsVisible) {
                osdStatsVisible = false;
                displayClearScreen(osdDisplayPort);
            } else if (!IS_RC_MODE_ACTIVE(BOXOSD)) {
                if (!osdStatsVisible) {
                    osdStatsVisible = true;
                    osdStatsRefreshTimeUs = 0;
                }
                if (currentTimeUs >= osdStatsRefreshTimeUs) {
                    osdStatsRefreshTimeUs = currentTimeUs + REFRESH_1S;
                    osdShowStats(endBatteryVoltage);
                }
            }
        }
    }
    lastTimeUs = currentTimeUs;

    if (resumeRefreshAt) {
        if (cmp32(currentTimeUs, resumeRefreshAt) < 0) {
            // in timeout period, check sticks for activity to resume display.
            if (IS_HI(THROTTLE) || IS_HI(PITCH)) {
                resumeRefreshAt = currentTimeUs;
            }
            displayHeartbeat(osdDisplayPort);
            return;
        } else {
            displayClearScreen(osdDisplayPort);
            resumeRefreshAt = 0;
            osdStatsEnabled = false;
            stats.armed_time = 0;
        }
    }

    blinkState = (currentTimeUs / 200000) % 2;

#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        escDataCombined = getEscSensorData(ESC_SENSOR_COMBINED);
    }
#endif

#ifdef USE_CMS
    if (!displayIsGrabbed(osdDisplayPort)) {
        osdUpdateAlarms();
        osdDrawElements();
        displayHeartbeat(osdDisplayPort);
#ifdef OSD_CALLS_CMS
    } else {
        cmsUpdate(currentTimeUs);
#endif
    }
#endif
    lastArmState = ARMING_FLAG(ARMED);
}

/*
 * Called periodically by the scheduler
 */
void osdUpdate(timeUs_t currentTimeUs)
{
    static uint32_t counter = 0;

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

    // redraw values in buffer
#ifdef USE_MAX7456
#define DRAW_FREQ_DENOM 5
#else
#define DRAW_FREQ_DENOM 10 // MWOSD @ 115200 baud (
#endif
#define STATS_FREQ_DENOM    50

    if (counter % DRAW_FREQ_DENOM == 0) {
        osdRefresh(currentTimeUs);
        showVisualBeeper = false;
    } else {
        // rest of time redraw screen 10 chars per idle so it doesn't lock the main idle
        displayDrawScreen(osdDisplayPort);
    }
    ++counter;

#ifdef USE_CMS
    // do not allow ARM if we are in menu
    if (displayIsGrabbed(osdDisplayPort)) {
        setArmingDisabled(ARMING_DISABLED_OSD_MENU);
    } else {
        unsetArmingDisabled(ARMING_DISABLED_OSD_MENU);
    }
#endif
}

#endif // USE_OSD
