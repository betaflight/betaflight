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
    *****************************************
    Instructions for adding new OSD Elements:
    *****************************************

    First add the new element to the osd_items_e enumeration in osd/osd.h. The
    element must be added to the end just before OSD_ITEM_COUNT.

    Next add the element to the osdElementDisplayOrder array defined in this file.
    If the element needs special runtime conditional processing then it should be added
    to the osdAddActiveElements() function instead.

    Create the function to "draw" the element.
    ------------------------------------------
    It should be named like "osdElementSomething()" where the "Something" describes
    the element. The drawing function should only render the dynamic portions of the
    element. If the element has static (unchanging) portions then those should be
    rendered in the background function. The exception to this is elements that are
    expected to blink (have a warning associated). In this case the entire element
    must be handled in the main draw function and you can't use the background capability.

    Add the mapping from the element ID added in the first step to the function
    created in the third step to the osdElementDrawFunction array.

    Create the function to draw the element's static (background) portion.
    ---------------------------------------------------------------------
    If an element has static (unchanging) portions then create a function to draw only those
    parts. It should be named like "osdBackgroundSomething()" where the "Something" matches
    the related element function.

    Add the mapping for the element ID to the background drawing function to the
    osdElementBackgroundFunction array.

    You should also add a corresponding entry to the file: cms_menu_osd.c

    Accelerometer reqirement:
    -------------------------
    If the new element utilizes the accelerometer, add it to the osdElementsNeedAccelerometer() function.

    Finally add a CLI parameter for the new element in cli/settings.c.
    CLI parameters should be added before line #endif // end of #ifdef USE_OSD
*/

/*
    *********************
    OSD element variants:
    *********************

    Each element can have up to 4 display variants. "Type 1" is always the default and every
    every element has an implicit type 1 variant even if no additional options exist. The
    purpose is to allow the user to choose a different element display or rendering style to
    fit their needs. Like displaying GPS coordinates in a different format, displaying a voltage
    with a different number of decimal places, etc. The purpose is NOT to display unrelated
    information in different variants of the element. For example it would be inappropriate
    to use variants to display RSSI for one type and link quality for another. In this case
    they should be separate elements. Remember that element variants are mutually exclusive
    and only one type can be displayed at a time. So they shouldn't be used in cases where
    the user would want to display different types at the same time - like in the above example
    where the user might want to display both RSSI and link quality at the same time.

    As variants are added to the firmware, support must also be included in the Configurator.

    The following lists the variants implemented so far (please update this as variants are added):

    OSD_ALTITUDE
        type 1: Altitude with one decimal place
        type 2: Altitude with no decimal (whole number only)

    OSD_GPS_LON
    OSD_GPS_LAT
        type 1: Decimal representation with 7 digits
        type 2: Decimal representation with 4 digits
        type 3: Degrees, minutes, seconds
        type 4: Open location code (Google Plus Code)

    OSD_MAIN_BATT_USAGE
        type 1: Graphical bar showing remaining battery (shrinks as used)
        type 2: Graphical bar showing battery used (grows as used)
        type 3: Numeric % of remaining battery
        type 4: Numeric % or used battery

    VTX_CHANNEL
        type 1: Contains Band:Channel:Power:Pit
        type 2: Contains only Power
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

#include "cli/settings.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/typeconversion.h"
#include "common/utils.h"
#include "common/unit.h"
#include "common/filter.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/display.h"
#include "drivers/dshot.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"
#include "drivers/vtx_common.h"

#include "drivers/pinio.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/gps_lap_timer.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/position.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/gps.h"
#include "io/vtx.h"

#include "osd/osd.h"
#include "osd/osd_elements.h"
#include "osd/osd_warnings.h"

#include "pg/motor.h"
#include "pg/pilot.h"
#include "pg/stats.h"

#include "rx/rx.h"

#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/rangefinder.h"

#ifdef USE_GPS_PLUS_CODES
// located in lib/main/google/olc
#include "olc.h"
#endif

#define AH_SYMBOL_COUNT 9
#define AH_SIDEBAR_WIDTH_POS 7
#define AH_SIDEBAR_HEIGHT_POS 3

// Stick overlay size
#define OSD_STICK_OVERLAY_WIDTH 7
#define OSD_STICK_OVERLAY_HEIGHT 5
#define OSD_STICK_OVERLAY_SPRITE_HEIGHT 3
#define OSD_STICK_OVERLAY_VERTICAL_POSITIONS (OSD_STICK_OVERLAY_HEIGHT * OSD_STICK_OVERLAY_SPRITE_HEIGHT)

#define FULL_CIRCLE 360
#define EFFICIENCY_MINIMUM_SPEED_CM_S 100
#define EFFICIENCY_CUTOFF_HZ 0.5f

static pt1Filter_t batteryEfficiencyFilt;

#define MOTOR_STOPPED_THRESHOLD_RPM 1000

#define SINE_25_DEG 0.422618261740699f

#ifdef USE_OSD_STICK_OVERLAY
typedef struct radioControls_s {
    uint8_t left_vertical;
    uint8_t left_horizontal;
    uint8_t right_vertical;
    uint8_t right_horizontal;
} radioControls_t;

static const radioControls_t radioModes[4] = {
    { PITCH,    YAW,    THROTTLE,   ROLL }, // Mode 1
    { THROTTLE, YAW,    PITCH,      ROLL }, // Mode 2
    { PITCH,    ROLL,   THROTTLE,   YAW  }, // Mode 3
    { THROTTLE, ROLL,   PITCH,      YAW  }, // Mode 4
};
#endif

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

static unsigned activeOsdElementCount = 0;
static uint8_t activeOsdElementArray[OSD_ITEM_COUNT];
static bool backgroundLayerSupported = false;

// Blink control
#define OSD_BLINK_FREQUENCY_HZ 2
static bool blinkState = true;
static uint32_t blinkBits[(OSD_ITEM_COUNT + 31) / 32];
#define SET_BLINK(item) (blinkBits[(item) / 32] |= (1 << ((item) % 32)))
#define CLR_BLINK(item) (blinkBits[(item) / 32] &= ~(1 << ((item) % 32)))
#define IS_BLINK(item) (blinkBits[(item) / 32] & (1 << ((item) % 32)))
#define BLINK(item) (IS_BLINK(item) && blinkState)

// Current element and render status
static osdElementParms_t activeElement;
static bool displayPendingForeground;
static bool displayPendingBackground;
static char elementBuff[OSD_ELEMENT_BUFFER_LENGTH];

// Return whether element is a SYS element and needs special handling
#define IS_SYS_OSD_ELEMENT(item) (item >= OSD_SYS_GOGGLE_VOLTAGE) && (item <= OSD_SYS_FAN_SPEED)

enum {UP, DOWN};

static int osdDisplayWrite(osdElementParms_t *element, uint8_t x, uint8_t y, uint8_t attr, const char *s)
{
    if (IS_BLINK(element->item)) {
        attr |= DISPLAYPORT_BLINK;
    }

    return displayWrite(element->osdDisplayPort, x, y, attr, s);
}

static int osdDisplayWriteChar(osdElementParms_t *element, uint8_t x, uint8_t y, uint8_t attr, char c)
{
    char buf[2];

    buf[0] = c;
    buf[1] = 0;

    return osdDisplayWrite(element, x, y, attr, buf);
}

#if defined(USE_ESC_SENSOR) || defined(USE_DSHOT_TELEMETRY)
typedef int (*getEscRpmOrFreqFnPtr)(int i);

static int getEscRpm(int i)
{
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        return lrintf(getDshotRpm(i));
    }
#endif
#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        return lrintf(erpmToRpm(getEscSensorData(i)->rpm));
    }
#endif
    return 0;
}

static int getEscRpmFreq(int i)
{
    return getEscRpm(i) / 60;
}

static void renderOsdEscRpmOrFreq(getEscRpmOrFreqFnPtr escFnPtr, osdElementParms_t *element)
{
    static uint8_t motor = 0;
    const int rpm = MIN((*escFnPtr)(motor),99999);

    tfp_sprintf(element->buff, "%d", rpm);
    element->elemOffsetY = motor;

    if (++motor == getMotorCount()) {
        motor = 0;
    } else {
        element->rendered = false;
    }
}
#endif

int osdConvertTemperatureToSelectedUnit(int tempInDegreesCelcius)
{
    switch (osdConfig()->units) {
    case UNIT_IMPERIAL:
        return lrintf(((tempInDegreesCelcius * 9.0f) / 5) + 32);
    default:
        return tempInDegreesCelcius;
    }
}

static void osdFormatAltitudeString(char * buff, int32_t altitudeCm, osdElementType_e variantType)
{
    static const struct {
        uint8_t decimals;
        bool asl;
    } variantMap[] = {
        [OSD_ELEMENT_TYPE_1] = { 1, false },
        [OSD_ELEMENT_TYPE_2] = { 0, false },
        [OSD_ELEMENT_TYPE_3] = { 1, true },
        [OSD_ELEMENT_TYPE_4] = { 0, true },
    };

    int32_t alt = altitudeCm;
#ifdef USE_GPS
    if (variantMap[variantType].asl) {
        alt = getAltitudeAsl();
    }
#endif
    unsigned decimalPlaces = variantMap[variantType].decimals;
    const char unitSymbol = osdGetMetersToSelectedUnitSymbol();

    osdPrintFloat(buff, SYM_ALTITUDE, osdGetMetersToSelectedUnit(alt) / 100.0f, "", decimalPlaces, true, unitSymbol);
}

#ifdef USE_GPS
static void osdFormatCoordinate(char *buff, gpsCoordinateType_e coordinateType, osdElementType_e variantType)
{
    int32_t gpsValue = 0;
    const char leadingSymbol = (coordinateType == GPS_LONGITUDE) ? SYM_LON : SYM_LAT;

    if (STATE(GPS_FIX_EVER)) {  // don't display interim coordinates until we get the first position fix
        gpsValue = (coordinateType == GPS_LONGITUDE) ? gpsSol.llh.lon : gpsSol.llh.lat;
    }

    const int degreesPart = abs(gpsValue) / GPS_DEGREES_DIVIDER;
    int fractionalPart = abs(gpsValue) % GPS_DEGREES_DIVIDER;

    switch (variantType) {
#ifdef USE_GPS_PLUS_CODES
#define PLUS_CODE_DIGITS 11
    case OSD_ELEMENT_TYPE_4: // Open Location Code
        {
            *buff++ = SYM_SAT_L;
            *buff++ = SYM_SAT_R;
            if (STATE(GPS_FIX_EVER)) {
                OLC_LatLon location;
                location.lat = (double)gpsSol.llh.lat / GPS_DEGREES_DIVIDER;
                location.lon = (double)gpsSol.llh.lon / GPS_DEGREES_DIVIDER;
                OLC_Encode(&location, PLUS_CODE_DIGITS, buff);
            } else {
                memset(buff, SYM_HYPHEN, PLUS_CODE_DIGITS + 1);
                buff[8] = '+';
                buff[PLUS_CODE_DIGITS + 1] = '\0';
            }
            break;
        }
#endif // USE_GPS_PLUS_CODES

    case OSD_ELEMENT_TYPE_3: // degree, minutes, seconds style. ddd^mm'ss.00"W
        {
            char trailingSymbol;
            *buff++ = leadingSymbol;

            const int minutes = fractionalPart * 60 / GPS_DEGREES_DIVIDER;
            const int fractionalMinutes =  fractionalPart * 60 % GPS_DEGREES_DIVIDER;
            const int seconds = fractionalMinutes * 60 / GPS_DEGREES_DIVIDER;
            const int tenthSeconds = (fractionalMinutes * 60 % GPS_DEGREES_DIVIDER) * 10 / GPS_DEGREES_DIVIDER;

            if (coordinateType == GPS_LONGITUDE) {
                trailingSymbol = (gpsValue < 0) ? 'W' : 'E';
            } else {
                trailingSymbol = (gpsValue < 0) ? 'S' : 'N';
            }
            tfp_sprintf(buff, "%u%c%02u%c%02u.%u%c%c", degreesPart, SYM_GPS_DEGREE, minutes, SYM_GPS_MINUTE, seconds, tenthSeconds, SYM_GPS_SECOND, trailingSymbol);
            break;
        }

    case OSD_ELEMENT_TYPE_2:
        fractionalPart /= 1000;
        FALLTHROUGH;

    case OSD_ELEMENT_TYPE_1:
    default:
        *buff++ = leadingSymbol;
        if (gpsValue < 0) {
            *buff++ = SYM_HYPHEN;
        }
        tfp_sprintf(buff, (variantType == OSD_ELEMENT_TYPE_1 ? "%u.%07u" : "%u.%04u"), degreesPart, fractionalPart);
        break;
    }
}
#endif // USE_GPS

void osdFormatDistanceString(char *ptr, int distance, char leadingSymbol)
{
    const float convertedDistance = osdGetMetersToSelectedUnit(distance);
    char unitSymbol;
    char unitSymbolExtended;
    int unitTransition;

    switch (osdConfig()->units) {
    case UNIT_IMPERIAL:
        unitTransition = 5280;
        unitSymbol = SYM_FT;
        unitSymbolExtended = SYM_MILES;
        break;
    default:
        unitTransition = 1000;
        unitSymbol = SYM_M;
        unitSymbolExtended = SYM_KM;
        break;
    }

    unsigned decimalPlaces;
    float displayDistance;
    char displaySymbol;
    if (convertedDistance < unitTransition) {
        decimalPlaces = 0;
        displayDistance = convertedDistance;
        displaySymbol = unitSymbol;
    } else {
        displayDistance = convertedDistance / unitTransition;
        displaySymbol = unitSymbolExtended;
        if (displayDistance >= 10) { // >= 10 miles or km - 1 decimal place
            decimalPlaces = 1;
        } else {                     // < 10 miles or km - 2 decimal places
            decimalPlaces = 2;
        }
    }
    osdPrintFloat(ptr, leadingSymbol, displayDistance, "", decimalPlaces, false, displaySymbol);
}

static void osdFormatPID(char * buff, const char * label, uint8_t axis)
{
    tfp_sprintf(buff, "%s %3d %3d %3d %3d %3d", label,
        currentPidProfile->pid[axis].P,
        currentPidProfile->pid[axis].I,
        currentPidProfile->pid[axis].D,
        currentPidProfile->d_max[axis],
        currentPidProfile->pid[axis].F);
}

#ifdef USE_RTC_TIME
bool osdFormatRtcDateTime(char *buffer)
{
    dateTime_t dateTime;
    if (!rtcGetDateTime(&dateTime)) {
        buffer[0] = '\0';

        return false;
    }

    switch (activeElement.type) {
    case OSD_ELEMENT_TYPE_2:
        tfp_sprintf(buffer, "%02d.%02d %02d:%02d", dateTime.month, dateTime.day, dateTime.hours, dateTime.minutes);
        break;
    case OSD_ELEMENT_TYPE_1:
    default:
        dateTimeFormatLocalShort(buffer, &dateTime);
        break;
    }

    return true;
}
#endif

void osdFormatTime(char * buff, osd_timer_precision_e precision, timeUs_t time)
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
    case OSD_TIMER_PREC_TENTHS:
        {
            const int tenths = (time / 100000) % 10;
            tfp_sprintf(buff, "%02d:%02d.%01d", minutes, seconds, tenths);
            break;
        }
    }
}

static char osdGetTimerSymbol(osd_timer_source_e src)
{
    switch (src) {
    case OSD_TIMER_SRC_ON:
        return SYM_ON_M;
    case OSD_TIMER_SRC_TOTAL_ARMED:
    case OSD_TIMER_SRC_LAST_ARMED:
        return SYM_FLY_M;
    case OSD_TIMER_SRC_ON_OR_ARMED:
        return ARMING_FLAG(ARMED) ? SYM_FLY_M : SYM_ON_M;
    case OSD_TIMER_SRC_LAUNCH_TIME:
        return 'L';
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
        return osdFlyTime;
    case OSD_TIMER_SRC_LAST_ARMED: {
        statistic_t *stats = osdGetStats();
        return stats->armed_time;
    }
    case OSD_TIMER_SRC_ON_OR_ARMED:
        return ARMING_FLAG(ARMED) ? osdFlyTime : micros();
    case OSD_TIMER_SRC_LAUNCH_TIME:
        return osdLaunchTime;
    default:
        return 0;
    }
}

void osdFormatTimer(char *buff, bool showSymbol, bool usePrecision, int timerIndex)
{
    const uint16_t timer = osdConfig()->timers[timerIndex];
    const uint8_t src = OSD_TIMER_SRC(timer);

    if (showSymbol) {
        *(buff++) = osdGetTimerSymbol(src);
    }

    osdFormatTime(buff, (usePrecision ? OSD_TIMER_PRECISION(timer) : OSD_TIMER_PREC_SECOND), osdGetTimerValue(src));
}

static char osdGetBatterySymbol(int cellVoltage)
{
    if (getBatteryState() == BATTERY_CRITICAL) {
        return SYM_MAIN_BATT; // FIXME: currently the BAT- symbol, ideally replace with a battery with exclamation mark
    } else {
        // Calculate a symbol offset using cell voltage over full cell voltage range
        const int symOffset = scaleRange(cellVoltage, batteryConfig()->vbatmincellvoltage, batteryConfig()->vbatmaxcellvoltage, 0, 8);
        return SYM_BATT_EMPTY - constrain(symOffset, 0, 6);
    }
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

/**
 * Converts altitude based on the current unit system.
 * @param meters Value in meters to convert
 */
float osdGetMetersToSelectedUnit(int32_t meters)
{
    switch (osdConfig()->units) {
    case UNIT_IMPERIAL:
        return meters * 3.28084f;       // Convert to feet
    default:
        return meters;                  // Already in meters
    }
}

/**
 * Gets the correct altitude symbol for the current unit system
 */
char osdGetMetersToSelectedUnitSymbol(void)
{
    switch (osdConfig()->units) {
    case UNIT_IMPERIAL:
        return SYM_FT;
    default:
        return SYM_M;
    }
}

/**
 * Converts speed based on the current unit system.
 * @param value in cm/s to convert
 */
int32_t osdGetSpeedToSelectedUnit(int32_t value)
{
    switch (osdConfig()->units) {
    case UNIT_IMPERIAL:
    case UNIT_BRITISH:
        return CM_S_TO_MPH(value);
    default:
        return CM_S_TO_KM_H(value);
    }
}

/**
 * Gets the correct speed symbol for the current unit system
 */
char osdGetSpeedToSelectedUnitSymbol(void)
{
    switch (osdConfig()->units) {
    case UNIT_IMPERIAL:
    case UNIT_BRITISH:
        return SYM_MPH;
    default:
        return SYM_KPH;
    }
}

MAYBE_UNUSED static char osdGetVarioToSelectedUnitSymbol(void)
{
    switch (osdConfig()->units) {
    case UNIT_IMPERIAL:
        return SYM_FTPS;
    default:
        return SYM_MPS;
    }
}

#if defined(USE_ADC_INTERNAL) || defined(USE_ESC_SENSOR) || defined(USE_DSHOT_TELEMETRY)
char osdGetTemperatureSymbolForSelectedUnit(void)
{
    switch (osdConfig()->units) {
    case UNIT_IMPERIAL:
        return SYM_F;
    default:
        return SYM_C;
    }
}
#endif

// *************************
// Element drawing functions
// *************************

#ifdef USE_RANGEFINDER
static void osdElementLidarDist(osdElementParms_t *element)
{
    int16_t dist = rangefinderGetLatestAltitude();

    if (dist > 0) {

        tfp_sprintf(element->buff, "RF:%3d", dist);

    } else {

        tfp_sprintf(element->buff, "RF:---");
    }
}
#endif

#ifdef USE_OSD_ADJUSTMENTS
static void osdElementAdjustmentRange(osdElementParms_t *element)
{
    const char *name = getAdjustmentsRangeName();
    if (name) {
        tfp_sprintf(element->buff, "%s: %3d", name, getAdjustmentsRangeValue());
    }
}
#endif // USE_OSD_ADJUSTMENTS

static void osdElementAltitude(osdElementParms_t *element)
{
    bool haveBaro = false;
    bool haveGps = false;
#ifdef USE_BARO
    haveBaro = sensors(SENSOR_BARO);
#endif // USE_BARO
#ifdef USE_GPS
    haveGps = sensors(SENSOR_GPS) && STATE(GPS_FIX);
#endif // USE_GPS
    int32_t alt = osdGetMetersToSelectedUnit(getEstimatedAltitudeCm()) / 100;

    if ((alt >= osdConfig()->alt_alarm) && ARMING_FLAG(ARMED)) {
        element->attr = DISPLAYPORT_SEVERITY_CRITICAL;
    }

    if (haveBaro || haveGps) {
        osdFormatAltitudeString(element->buff, getEstimatedAltitudeCm(), element->type);
    } else {
        element->buff[0] = SYM_ALTITUDE;
        element->buff[1] = SYM_HYPHEN; // We use this symbol when we don't have a valid measure
        element->buff[2] = '\0';
    }
}

#ifdef USE_ACC
static void osdElementAngleRollPitch(osdElementParms_t *element)
{
    const float angle = ((element->item == OSD_PITCH_ANGLE) ? attitude.values.pitch : attitude.values.roll) / 10.0f;
    osdPrintFloat(element->buff, (element->item == OSD_PITCH_ANGLE) ? SYM_PITCH : SYM_ROLL, fabsf(angle), ((angle < 0) ? "-%02u" : " %02u"), 1, true, SYM_NONE);
}
#endif

static void osdElementAntiGravity(osdElementParms_t *element)
{
    if (pidOsdAntiGravityActive()) {
        strcpy(element->buff, "AG");
    }
}

#ifdef USE_ACC

static void osdElementArtificialHorizon(osdElementParms_t *element)
{
    static int x = -4;
    // Get pitch and roll limits in tenths of degrees
    const int maxPitch = osdConfig()->ahMaxPitch * 10;
    const int maxRoll = osdConfig()->ahMaxRoll * 10;
    const int ahSign = osdConfig()->ahInvert ? -1 : 1;
    const int rollAngle = constrain(attitude.values.roll * ahSign, -maxRoll, maxRoll);
    int pitchAngle = constrain(attitude.values.pitch * ahSign, -maxPitch, maxPitch);
    // Convert pitchAngle to y compensation value
    // (maxPitch / 25) divisor matches previous settings of fixed divisor of 8 and fixed max AHI pitch angle of 20.0 degrees
    if (maxPitch > 0) {
        pitchAngle = ((pitchAngle * 25) / maxPitch);
    }
    pitchAngle -= 41; // 41 = 4 * AH_SYMBOL_COUNT + 5

    const int y = ((-rollAngle * x) / 64) - pitchAngle;
    if (y >= 0 && y <= 81) {
        element->elemOffsetX = x;
        element->elemOffsetY = y / AH_SYMBOL_COUNT;

        tfp_sprintf(element->buff, "%c", (SYM_AH_BAR9_0 + (y % AH_SYMBOL_COUNT)));
    } else {
        element->drawElement = false;  // element does not need to be rendered
    }

    if (x == 4) {
        // Rendering is complete, so prepare to start again
        x = -4;
    } else {
        // Rendering not yet complete
        element->rendered = false;
        x++;
    }
}

static void osdElementUpDownReference(osdElementParms_t *element)
{
// Up/Down reference feature displays reference points on the OSD at Zenith and Nadir
    const float earthUpinBodyFrame[3] = {-rMat.m[2][0], -rMat.m[2][1], -rMat.m[2][2]}; //transforum the up vector to the body frame

    if (fabsf(earthUpinBodyFrame[2]) < SINE_25_DEG && fabsf(earthUpinBodyFrame[1]) < SINE_25_DEG) {
        float thetaB; // pitch from body frame to zenith/nadir
        float psiB; // psi from body frame to zenith/nadir
        char *symbol[2] = {"U", "D"}; // character buffer
        int direction;

        if (attitude.values.pitch > 0.0f){ //nose down
            thetaB = -earthUpinBodyFrame[2]; // get pitch w/re to nadir (use small angle approx for sine)
            psiB = -earthUpinBodyFrame[1]; // calculate the yaw w/re to nadir (use small angle approx for sine)
            direction = DOWN;
        } else { // nose up
            thetaB = earthUpinBodyFrame[2]; // get pitch w/re to zenith (use small angle approx for sine)
            psiB = earthUpinBodyFrame[1]; // calculate the yaw w/re to zenith (use small angle approx for sine)
            direction = UP;
        }
        element->elemOffsetX = lrintf(scaleRangef(psiB, -M_PIf / 4, M_PIf / 4, -14, 14));
        element->elemOffsetY = lrintf(scaleRangef(thetaB, -M_PIf / 4, M_PIf / 4, -8, 8));

        tfp_sprintf(element->buff, "%c", symbol[direction]);
    }
}
#endif // USE_ACC

static void osdElementAverageCellVoltage(osdElementParms_t *element)
{
    const int cellV = getBatteryAverageCellVoltage();
    const batteryState_e batteryState = getBatteryState();

    switch (batteryState) {
    case BATTERY_WARNING:
        element->attr = DISPLAYPORT_SEVERITY_WARNING;
        break;
    case BATTERY_CRITICAL:
        element->attr = DISPLAYPORT_SEVERITY_CRITICAL;
        break;
    default:
        break;
    }

    osdPrintFloat(element->buff, osdGetBatterySymbol(cellV), cellV / 100.0f, "", 2, false, SYM_VOLT);
}

static void osdElementCompassBar(osdElementParms_t *element)
{
    memcpy(element->buff, compassBar + osdGetHeadingIntoDiscreteDirections(DECIDEGREES_TO_DEGREES(attitude.values.yaw), 16), 9);
    element->buff[9] = 0;
}

//display custom message from MSPv2
static void osdElementCustomMsg(osdElementParms_t *element)
{
    int msgIndex = element->item - OSD_CUSTOM_MSG0;
    if (msgIndex < 0 || msgIndex >= OSD_CUSTOM_MSG_COUNT || pilotConfig()->message[msgIndex][0] == '\0') {
        tfp_sprintf(element->buff, "CUSTOM_MSG%d", msgIndex + 1);
    } else {
        strncpy(element->buff, pilotConfig()->message[msgIndex], MAX_NAME_LENGTH);
        element->buff[MAX_NAME_LENGTH] = 0;   // terminate maximum-length string
    }
}

#ifdef USE_ADC_INTERNAL
static void osdElementCoreTemperature(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "C%c%3d%c", SYM_TEMPERATURE, osdConvertTemperatureToSelectedUnit(getCoreTemperatureCelsius()), osdGetTemperatureSymbolForSelectedUnit());
}
#endif // USE_ADC_INTERNAL

static void osdBackgroundCameraFrame(osdElementParms_t *element)
{
    static enum {TOP, MIDDLE, BOTTOM} renderPhase = TOP;
    const uint8_t xpos = element->elemPosX;
    const uint8_t ypos = element->elemPosY;
    const uint8_t width = constrain(osdConfig()->camera_frame_width, OSD_CAMERA_FRAME_MIN_WIDTH, OSD_CAMERA_FRAME_MAX_WIDTH);
    const uint8_t height = constrain(osdConfig()->camera_frame_height, OSD_CAMERA_FRAME_MIN_HEIGHT, OSD_CAMERA_FRAME_MAX_HEIGHT);

    if (renderPhase != BOTTOM) {
        // Rendering not yet complete
        element->rendered = false;
    }

    if (renderPhase == MIDDLE) {
        static uint8_t i = 1;

        osdDisplayWriteChar(element, xpos, ypos + i, DISPLAYPORT_SEVERITY_NORMAL, SYM_STICK_OVERLAY_VERTICAL);
        osdDisplayWriteChar(element, xpos + width - 1, ypos + i, DISPLAYPORT_SEVERITY_NORMAL, SYM_STICK_OVERLAY_VERTICAL);

        element->drawElement = false;  // element already drawn

        if (++i == height) {
            i = 1;
            renderPhase = BOTTOM;
        }
    } else {
        element->buff[0] = SYM_STICK_OVERLAY_CENTER;
        for (uint8_t i = 1; i < (width - 1); i++) {
            element->buff[i] = SYM_STICK_OVERLAY_HORIZONTAL;
        }
        element->buff[width - 1] = SYM_STICK_OVERLAY_CENTER;
        element->buff[width] = 0;  // string terminator

        if (renderPhase == TOP) {
            renderPhase = MIDDLE;
        } else {
            element->elemOffsetY = height - 1;
            renderPhase = TOP;
        }
    }
}

static void toUpperCase(char* dest, const char* src, unsigned int maxSrcLength)
{
    unsigned int i;
    for (i = 0; i < maxSrcLength && src[i]; i++) {
            dest[i] = toupper((unsigned char)src[i]);
    }
    dest[i] = '\0';
}

static void osdBackgroundCraftName(osdElementParms_t *element)
{
    if (strlen(pilotConfig()->craftName) == 0) {
        strcpy(element->buff, "CRAFT_NAME");
    } else {
        toUpperCase(element->buff, pilotConfig()->craftName, MAX_NAME_LENGTH);
    }
}

#ifdef USE_ACC
static void osdElementCrashFlipArrow(osdElementParms_t *element)
{
    int rollAngle = attitude.values.roll / 10;
    const int pitchAngle = attitude.values.pitch / 10;
    if (abs(rollAngle) > 90) {
        rollAngle = (rollAngle < 0 ? -180 : 180) - rollAngle;
    }

    if ((isCrashFlipModeActive() || (!ARMING_FLAG(ARMED) && !isUpright())) && !((imuConfig()->small_angle < 180 && isUpright()) || (rollAngle == 0 && pitchAngle == 0))) {
        element->attr = DISPLAYPORT_SEVERITY_INFO;
        if (abs(pitchAngle) < 2 * abs(rollAngle) && abs(rollAngle) < 2 * abs(pitchAngle)) {
            if (pitchAngle > 0) {
                if (rollAngle > 0) {
                    element->buff[0] = SYM_ARROW_WEST + 2;
                } else {
                    element->buff[0] = SYM_ARROW_EAST - 2;
                }
            } else {
                if (rollAngle > 0) {
                    element->buff[0] = SYM_ARROW_WEST - 2;
                } else {
                    element->buff[0] = SYM_ARROW_EAST + 2;
                }
            }
        } else {
            if (abs(pitchAngle) > abs(rollAngle)) {
                if (pitchAngle > 0) {
                    element->buff[0] = SYM_ARROW_SOUTH;
                } else {
                    element->buff[0] = SYM_ARROW_NORTH;
                }
            } else {
                if (rollAngle > 0) {
                    element->buff[0] = SYM_ARROW_WEST;
                } else {
                    element->buff[0] = SYM_ARROW_EAST;
                }
            }
        }
        element->buff[1] = '\0';
    }
}
#endif // USE_ACC

static void osdElementCrosshairs(osdElementParms_t *element)
{
    element->buff[0] = SYM_AH_CENTER_LINE;
    element->buff[1] = SYM_AH_CENTER;
    element->buff[2] = SYM_AH_CENTER_LINE_RIGHT;
    element->buff[3] = 0;
}

static void osdElementCurrentDraw(osdElementParms_t *element)
{
    const float amperage = fabsf(getAmperage() / 100.0f);
    osdPrintFloat(element->buff, SYM_NONE, amperage, "%3u", 2, false, SYM_AMP);
}

static void osdElementDebug(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "DBG %5d %5d %5d %5d", debug[0], debug[1], debug[2], debug[3]);
}

static void osdElementDebug2(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "D2  %5d %5d %5d %5d", debug[4], debug[5], debug[6], debug[7]);
}

static void osdElementDisarmed(osdElementParms_t *element)
{
    if (!ARMING_FLAG(ARMED)) {
        tfp_sprintf(element->buff, "DISARMED");
    }
}

static void osdBackgroundPilotName(osdElementParms_t *element)
{
    if (strlen(pilotConfig()->pilotName) == 0) {
        strcpy(element->buff, "PILOT_NAME");
    } else {
        toUpperCase(element->buff, pilotConfig()->pilotName, MAX_NAME_LENGTH);
    }
}

#ifdef USE_PERSISTENT_STATS
static void osdElementTotalFlights(osdElementParms_t *element)
{
    const int32_t total_flights = statsConfig()->stats_total_flights;
    tfp_sprintf(element->buff, "#%d", total_flights);
}
#endif

#ifdef USE_PROFILE_NAMES
static void osdElementRateProfileName(osdElementParms_t *element)
{
    if (strlen(currentControlRateProfile->profileName) == 0) {
        tfp_sprintf(element->buff, "RATE_%u", getCurrentControlRateProfileIndex() + 1);
    } else {
        toUpperCase(element->buff, currentControlRateProfile->profileName, MAX_PROFILE_NAME_LENGTH);
    }
}

static void osdElementPidProfileName(osdElementParms_t *element)
{
    if (strlen(currentPidProfile->profileName) == 0) {
        tfp_sprintf(element->buff, "PID_%u", getCurrentPidProfileIndex() + 1);
    } else {
        toUpperCase(element->buff, currentPidProfile->profileName, MAX_PROFILE_NAME_LENGTH);
    }
}
#endif

#ifdef USE_OSD_PROFILES
static void osdElementOsdProfileName(osdElementParms_t *element)
{
    uint8_t profileIndex = getCurrentOsdProfileIndex();

    if (strlen(osdConfig()->profile[profileIndex - 1]) == 0) {
        tfp_sprintf(element->buff, "OSD_%u", profileIndex);
    } else {
        toUpperCase(element->buff, osdConfig()->profile[profileIndex - 1], OSD_PROFILE_NAME_LENGTH);
    }
}
#endif

#if defined(USE_ESC_SENSOR) ||  defined(USE_DSHOT_TELEMETRY)

static void osdElementEscTemperature(osdElementParms_t *element)
{
#if defined(USE_ESC_SENSOR)
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        tfp_sprintf(element->buff, "E%c%3d%c", SYM_TEMPERATURE, osdConvertTemperatureToSelectedUnit(osdEscDataCombined->temperature), osdGetTemperatureSymbolForSelectedUnit());
    } else
#endif
#if defined(USE_DSHOT_TELEMETRY)
    {
        uint32_t osdEleIx = tfp_sprintf(element->buff, "E%c", SYM_TEMPERATURE);

        for (uint8_t k = 0; k < getMotorCount(); k++) {
            if ((dshotTelemetryState.motorState[k].telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_TEMPERATURE)) != 0) {
                osdEleIx += tfp_sprintf(element->buff + osdEleIx, "%3d%c",
                    osdConvertTemperatureToSelectedUnit(dshotTelemetryState.motorState[k].telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE]),
                    osdGetTemperatureSymbolForSelectedUnit());
            } else {
                osdEleIx += tfp_sprintf(element->buff + osdEleIx, "  0%c", osdGetTemperatureSymbolForSelectedUnit());
            }
        }
    }
#else
    {}
#endif
}

static void osdElementEscRpm(osdElementParms_t *element)
{
    renderOsdEscRpmOrFreq(&getEscRpm,element);
}

static void osdElementEscRpmFreq(osdElementParms_t *element)
{
    renderOsdEscRpmOrFreq(&getEscRpmFreq,element);
}

#endif

static void osdElementFlymode(osdElementParms_t *element)
{
    // Note that flight mode display has precedence in what to display.
    //  1. FS
    //  2. GPS RESCUE
    //  3. PASSTHRU
    //  4. HEAD, POSHOLD, ALTHOLD, ANGLE, HORIZON, ACRO TRAINER
    //  5. AIR
    //  6. ACRO

    if (FLIGHT_MODE(FAILSAFE_MODE)) {
        strcpy(element->buff, "!FS!");
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        strcpy(element->buff, "RESC");
    } else if (FLIGHT_MODE(HEADFREE_MODE)) {
        strcpy(element->buff, "HEAD");
    } else if (FLIGHT_MODE(PASSTHRU_MODE)) {
        strcpy(element->buff, "PASS");
    } else if (FLIGHT_MODE(POS_HOLD_MODE)) {
        strcpy(element->buff, "POSH");
    } else if (FLIGHT_MODE(ALT_HOLD_MODE)) {
        strcpy(element->buff, "ALTH");
    } else if (FLIGHT_MODE(ANGLE_MODE)) {
        strcpy(element->buff, "ANGL");
    } else if (FLIGHT_MODE(HORIZON_MODE)) {
        strcpy(element->buff, "HOR ");
    } else if (IS_RC_MODE_ACTIVE(BOXACROTRAINER)) {
        strcpy(element->buff, "ATRN");
#ifdef USE_CHIRP
    // the additional check for pidChirpIsFinished() is to have visual feedback for user that don't have warnings enabled in their goggles
    } else if (FLIGHT_MODE(CHIRP_MODE) && !pidChirpIsFinished()) {
        strcpy(element->buff, "CHIR");
#endif
    } else if (isAirmodeEnabled()) {
        strcpy(element->buff, "AIR ");
    } else {
        strcpy(element->buff, "ACRO");
    }
}

static void osdElementReadyMode(osdElementParms_t *element)
{
    if (IS_RC_MODE_ACTIVE(BOXREADY) && !ARMING_FLAG(ARMED)) {
        strcpy(element->buff, "READY");
    }
}

#ifdef USE_ACC
static void osdElementGForce(osdElementParms_t *element)
{
    osdPrintFloat(element->buff, SYM_NONE, osdGForce, "", 1, true, 'G');
}
#endif // USE_ACC

#ifdef USE_GPS
static void osdElementGpsFlightDistance(osdElementParms_t *element)
{
    if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
        osdFormatDistanceString(element->buff, GPS_distanceFlownInCm / 100, SYM_TOTAL_DISTANCE);
    } else {
        // We use this symbol when we don't have a FIX
        tfp_sprintf(element->buff, "%c%c", SYM_TOTAL_DISTANCE, SYM_HYPHEN);
    }
}

static void osdElementGpsHomeDirection(osdElementParms_t *element)
{
    if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
        if (GPS_distanceToHome > 0) {
            int direction = GPS_directionToHome;
#ifdef USE_GPS_LAP_TIMER
            // Override the "home" point to the start/finish location if the lap timer is running
            if (gpsLapTimerData.timerRunning) {
                direction = lrintf(gpsLapTimerData.dirToPoint * 0.1f); // Convert from centidegree to degree and round to nearest
            }
#endif
            element->buff[0] = osdGetDirectionSymbolFromHeading(DECIDEGREES_TO_DEGREES(direction - attitude.values.yaw));
        } else {
            element->buff[0] = SYM_OVER_HOME;
        }

    } else {
        // We use this symbol when we don't have a FIX
        element->buff[0] = SYM_HYPHEN;
    }

    element->buff[1] = 0;
}

static void osdElementGpsHomeDistance(osdElementParms_t *element)
{
    if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
        int distance = GPS_distanceToHome;
#ifdef USE_GPS_LAP_TIMER
        // Change the "home" point to the start/finish location if the lap timer is running
        if (gpsLapTimerData.timerRunning) {
            distance = lrintf(gpsLapTimerData.distToPointCM * 0.01f); // Round to nearest natural number
        }
#endif
        osdFormatDistanceString(element->buff, distance, SYM_HOMEFLAG);
    } else {
        element->buff[0] = SYM_HOMEFLAG;
        // We use this symbol when we don't have a FIX
        element->buff[1] = SYM_HYPHEN;
        element->buff[2] = '\0';
    }
}

static void osdElementGpsCoordinate(osdElementParms_t *element)
{
    const gpsCoordinateType_e coordinateType = (element->item == OSD_GPS_LON) ? GPS_LONGITUDE : GPS_LATITUDE;
    osdFormatCoordinate(element->buff, coordinateType, element->type);
    if (STATE(GPS_FIX_EVER) && !STATE(GPS_FIX)) {
        SET_BLINK(element->item); // blink if we had a fix but have since lost it
    } else {
        CLR_BLINK(element->item);
    }
}

static void osdElementGpsSats(osdElementParms_t *element)
{
    if ((STATE(GPS_FIX) == 0) || (gpsSol.numSat < GPS_MIN_SAT_COUNT) ) {
        element->attr = DISPLAYPORT_SEVERITY_CRITICAL;
    }
#ifdef USE_GPS_RESCUE
    else if ((gpsSol.numSat < gpsRescueConfig()->minSats) && gpsRescueIsConfigured()) {
        element->attr = DISPLAYPORT_SEVERITY_WARNING;
    }
#endif
    else {
        element->attr = DISPLAYPORT_SEVERITY_NORMAL;
    }

    if (!gpsIsHealthy()) {
        tfp_sprintf(element->buff, "%c%cNC", SYM_SAT_L, SYM_SAT_R);
    } else {
        int pos = tfp_sprintf(element->buff, "%c%c%2d", SYM_SAT_L, SYM_SAT_R, gpsSol.numSat);
        if (osdConfig()->gps_sats_show_pdop) { // add on the GPS module PDOP estimate
            element->buff[pos++] = ' ';
            osdPrintFloat(element->buff + pos, SYM_NONE, gpsSol.dop.pdop / 100.0f, "", 1, true, SYM_NONE);
        }
    }
}

static void osdElementGpsSpeed(osdElementParms_t *element)
{
    if (STATE(GPS_FIX)) {
        tfp_sprintf(element->buff, "%c%3d%c", SYM_SPEED, osdGetSpeedToSelectedUnit(gpsConfig()->gps_use_3d_speed ? gpsSol.speed3d : gpsSol.groundSpeed), osdGetSpeedToSelectedUnitSymbol());
    } else {
        tfp_sprintf(element->buff, "%c%c%c", SYM_SPEED, SYM_HYPHEN, osdGetSpeedToSelectedUnitSymbol());
    }
}

static void osdElementEfficiency(osdElementParms_t *element)
{
    int efficiency = 0;
    if (sensors(SENSOR_GPS) && ARMING_FLAG(ARMED) && STATE(GPS_FIX) && gpsSol.groundSpeed >= EFFICIENCY_MINIMUM_SPEED_CM_S) {
        const float speed = (float)osdGetSpeedToSelectedUnit(gpsSol.groundSpeed);
        const float mAmperage = (float)getAmperage() * 10.f; // Current in mA
        efficiency = lrintf(pt1FilterApply(&batteryEfficiencyFilt, (mAmperage / speed)));
    }

    const char unitSymbol = osdConfig()->units == UNIT_IMPERIAL ? SYM_MILES : SYM_KM;
    if (efficiency > 0 && efficiency <= 9999) {
        tfp_sprintf(element->buff, "%4d%c/%c", efficiency, SYM_MAH, unitSymbol);
    } else {
        tfp_sprintf(element->buff, "----%c/%c", SYM_MAH, unitSymbol);
    }
}
#endif // USE_GPS

#ifdef USE_GPS_LAP_TIMER
static void osdFormatLapTime(osdElementParms_t *element, uint32_t timeMs, uint8_t symbol)
{
    timeMs += 5;  // round to nearest centisecond (+/- 5ms)
    uint32_t seconds = timeMs / 1000;
    uint32_t decimals = (timeMs % 1000) / 10;
    tfp_sprintf(element->buff, "%c%3u.%02u", symbol, seconds, decimals);
}

static void osdElementGpsLapTimeCurrent(osdElementParms_t *element)
{
    if (gpsLapTimerData.timerRunning) {
        osdFormatLapTime(element, gpsSol.time - gpsLapTimerData.timeOfLastLap, SYM_TOTAL_DISTANCE);
    } else {
        osdFormatLapTime(element, 0, SYM_TOTAL_DISTANCE);
    }
}

static void osdElementGpsLapTimePrevious(osdElementParms_t *element)
{
    osdFormatLapTime(element, gpsLapTimerData.previousLaps[0], SYM_PREV_LAP_TIME);
}

static void osdElementGpsLapTimeBest3(osdElementParms_t *element)
{
    osdFormatLapTime(element, gpsLapTimerData.best3Consec, SYM_CHECKERED_FLAG);
}
#endif // GPS_LAP_TIMER

static void osdBackgroundHorizonSidebars(osdElementParms_t *element)
{
    static bool renderLevel = false;
    static int8_t y = -AH_SIDEBAR_HEIGHT_POS;
    // Draw AH sides
    const int8_t hudwidth = AH_SIDEBAR_WIDTH_POS;
    const int8_t hudheight = AH_SIDEBAR_HEIGHT_POS;

    if (renderLevel) {
        // AH level indicators
        osdDisplayWriteChar(element, element->elemPosX - hudwidth + 1, element->elemPosY, DISPLAYPORT_SEVERITY_NORMAL, SYM_AH_LEFT);
        osdDisplayWriteChar(element, element->elemPosX + hudwidth - 1, element->elemPosY, DISPLAYPORT_SEVERITY_NORMAL, SYM_AH_RIGHT);
        renderLevel = false;
    } else {
        osdDisplayWriteChar(element, element->elemPosX - hudwidth, element->elemPosY + y, DISPLAYPORT_SEVERITY_NORMAL, SYM_AH_DECORATION);
        osdDisplayWriteChar(element, element->elemPosX + hudwidth, element->elemPosY + y, DISPLAYPORT_SEVERITY_NORMAL, SYM_AH_DECORATION);

        if (y == hudheight) {
            // Rendering is complete, so prepare to start again
            y = -hudheight;
            // On next pass render the level markers
            renderLevel = true;
        } else {
            y++;
        }
        // Rendering not yet complete
        element->rendered = false;
    }

    element->drawElement = false;  // element already drawn
}

#ifdef USE_RX_LINK_QUALITY_INFO
static void osdElementLinkQuality(osdElementParms_t *element)
{
    uint16_t osdLinkQuality = 0;

    if (rxGetLinkQualityPercent() < osdConfig()->link_quality_alarm) {
        element->attr = DISPLAYPORT_SEVERITY_CRITICAL;
    }

    if (linkQualitySource == LQ_SOURCE_RX_PROTOCOL_CRSF) { // 0-99
        osdLinkQuality = rxGetLinkQuality();
        const uint8_t osdRfMode = rxGetRfMode();
        tfp_sprintf(element->buff, "%c%1d:%2d", SYM_LINK_QUALITY, osdRfMode, osdLinkQuality);
    } else if (linkQualitySource == LQ_SOURCE_RX_PROTOCOL_GHST) { // 0-100
        osdLinkQuality = rxGetLinkQuality();
        tfp_sprintf(element->buff, "%c%2d", SYM_LINK_QUALITY, osdLinkQuality);
    } else { // 0-9
        osdLinkQuality = rxGetLinkQuality() * 10 / LINK_QUALITY_MAX_VALUE;
        if (osdLinkQuality >= 10) {
            osdLinkQuality = 9;
        }
        tfp_sprintf(element->buff, "%c%1d", SYM_LINK_QUALITY, osdLinkQuality);
    }
}
#endif // USE_RX_LINK_QUALITY_INFO

#ifdef USE_RX_LINK_UPLINK_POWER
static void osdElementTxUplinkPower(osdElementParms_t *element)
{
    const uint16_t osdUplinkTxPowerMw = rxGetUplinkTxPwrMw();
    if (osdUplinkTxPowerMw < 1000) {
        tfp_sprintf(element->buff, "%c%3dMW", SYM_RSSI, osdUplinkTxPowerMw);
    } else {
        osdPrintFloat(element->buff, SYM_RSSI, osdUplinkTxPowerMw / 1000.0f, "", 1, false, 'W');
    }
}
#endif // USE_RX_LINK_UPLINK_POWER

#ifdef USE_BLACKBOX
static void osdElementLogStatus(osdElementParms_t *element)
{
    if (IS_RC_MODE_ACTIVE(BOXBLACKBOX)) {
        if (!isBlackboxDeviceWorking()) {
            tfp_sprintf(element->buff, "%c!", SYM_BBLOG);
        } else if (isBlackboxDeviceFull()) {
            tfp_sprintf(element->buff, "%c>", SYM_BBLOG);
        } else {
            int32_t logNumber = blackboxGetLogNumber();
            if (logNumber >= 0) {
                tfp_sprintf(element->buff, "%c%d", SYM_BBLOG, logNumber);
            } else {
                tfp_sprintf(element->buff, "%c", SYM_BBLOG);
            }
        }
    }
}
#endif // USE_BLACKBOX

static void osdElementMahDrawn(osdElementParms_t *element)
{
    const int mAhDrawn = getMAhDrawn();

    if (mAhDrawn >= osdConfig()->cap_alarm) {
        element->attr = DISPLAYPORT_SEVERITY_CRITICAL;
    }

    tfp_sprintf(element->buff, "%4d%c", mAhDrawn, SYM_MAH);
}

static void osdElementWattHoursDrawn(osdElementParms_t *element)
{
    const int mAhDrawn = getMAhDrawn();
    const float wattHoursDrawn = getWhDrawn();

    if (mAhDrawn >= osdConfig()->cap_alarm) {
        element->attr = DISPLAYPORT_SEVERITY_CRITICAL;
    }

    if (wattHoursDrawn < 1.0f) {
        tfp_sprintf(element->buff, "%3dMWH", lrintf(wattHoursDrawn * 1000));
    } else {
        int wattHourWholeNumber = (int)wattHoursDrawn;
        int wattHourDecimalValue = (int)((wattHoursDrawn - wattHourWholeNumber) * 100);

        tfp_sprintf(element->buff, wattHourDecimalValue >= 10 ? "%3d.%2dWH" : "%3d.0%1dWH", wattHourWholeNumber, wattHourDecimalValue);
    }
}

static void osdElementMainBatteryUsage(osdElementParms_t *element)
{
    // Set length of indicator bar
    #define MAIN_BATT_USAGE_STEPS 11 // Use an odd number so the bar can be centered.
    const int mAhDrawn = getMAhDrawn();
    const int usedCapacity = getMAhDrawn();
    int displayBasis = usedCapacity;

    if (mAhDrawn >= osdConfig()->cap_alarm) {
        element->attr = DISPLAYPORT_SEVERITY_CRITICAL;
    }

    switch (element->type) {
    case OSD_ELEMENT_TYPE_3:  // mAh remaining percentage (counts down as battery is used)
        displayBasis = constrain(batteryConfig()->batteryCapacity - usedCapacity, 0, batteryConfig()->batteryCapacity);
        FALLTHROUGH;

    case OSD_ELEMENT_TYPE_4:  // mAh used percentage (counts up as battery is used)
        {
            int displayPercent = 0;
            if (batteryConfig()->batteryCapacity) {
                displayPercent = constrain(lrintf(100.0f * displayBasis / batteryConfig()->batteryCapacity), 0, 100);
            }
            tfp_sprintf(element->buff, "%c%d%%", SYM_MAH, displayPercent);
            break;
        }

    case OSD_ELEMENT_TYPE_2:  // mAh used graphical progress bar (grows as battery is used)
        displayBasis = constrain(batteryConfig()->batteryCapacity - usedCapacity, 0, batteryConfig()->batteryCapacity);
        FALLTHROUGH;

    case OSD_ELEMENT_TYPE_1:  // mAh remaining graphical progress bar (shrinks as battery is used)
    default:
        {
            uint8_t remainingCapacityBars = 0;

            if (batteryConfig()->batteryCapacity) {
                const float batteryRemaining = constrain(batteryConfig()->batteryCapacity - displayBasis, 0, batteryConfig()->batteryCapacity);
                remainingCapacityBars = ceilf((batteryRemaining / (batteryConfig()->batteryCapacity / MAIN_BATT_USAGE_STEPS)));
            }

            // Create empty battery indicator bar
            element->buff[0] = SYM_PB_START;
            for (int i = 1; i <= MAIN_BATT_USAGE_STEPS; i++) {
                element->buff[i] = i <= remainingCapacityBars ? SYM_PB_FULL : SYM_PB_EMPTY;
            }
            element->buff[MAIN_BATT_USAGE_STEPS + 1] = SYM_PB_CLOSE;
            if (remainingCapacityBars > 0 && remainingCapacityBars < MAIN_BATT_USAGE_STEPS) {
                element->buff[1 + remainingCapacityBars] = SYM_PB_END;
            }
            element->buff[MAIN_BATT_USAGE_STEPS+2] = '\0';
            break;
        }
    }
}

static void osdElementMainBatteryVoltage(osdElementParms_t *element)
{
    unsigned decimalPlaces;
    const float batteryVoltage = getBatteryVoltage() / 100.0f;
    batteryState_e batteryState = getBatteryState();

    switch (batteryState) {
    case BATTERY_WARNING:
        element->attr = DISPLAYPORT_SEVERITY_WARNING;
        break;
    case BATTERY_CRITICAL:
        element->attr = DISPLAYPORT_SEVERITY_CRITICAL;
        break;
    default:
        break;
    }

    if (batteryVoltage >= 10) { // if voltage is 10v or more then display only 1 decimal place
        decimalPlaces = 1;
    } else {
        decimalPlaces = 2;
    }
    osdPrintFloat(element->buff, osdGetBatterySymbol(getBatteryAverageCellVoltage()), batteryVoltage, "", decimalPlaces, true, SYM_VOLT);
}

static void osdElementMotorDiagnostics(osdElementParms_t *element)
{
    int i = 0;
    const bool motorsRunning = areMotorsRunning();
    for (; i < getMotorCount(); i++) {
        if (motorsRunning) {
            element->buff[i] =  0x88 - scaleRange(motor[i], getMotorOutputLow(), getMotorOutputHigh(), 0, 8);
#if defined(USE_ESC_SENSOR) || defined(USE_DSHOT_TELEMETRY)
            if (getEscRpm(i) < MOTOR_STOPPED_THRESHOLD_RPM) {
                // Motor is not spinning properly. Mark as Stopped
                element->buff[i] = 'S';
            }
#endif
        } else {
            element->buff[i] =  0x88;
        }
    }
    element->buff[i] = '\0';
}

static void osdElementNumericalHeading(osdElementParms_t *element)
{
    const int heading = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
    tfp_sprintf(element->buff, "%c%03d", osdGetDirectionSymbolFromHeading(heading), heading);
}

#ifdef USE_VARIO
static void osdElementNumericalVario(osdElementParms_t *element)
{
    bool haveBaro = false;
    bool haveGps = false;
#ifdef USE_BARO
    haveBaro = sensors(SENSOR_BARO);
#endif // USE_BARO
#ifdef USE_GPS
    haveGps = sensors(SENSOR_GPS) && STATE(GPS_FIX);
#endif // USE_GPS
    if (haveBaro || haveGps) {
        const float verticalSpeed = osdGetMetersToSelectedUnit(getEstimatedVario()) / 100.0f;
        const char directionSymbol = verticalSpeed < 0 ? SYM_ARROW_SMALL_DOWN : SYM_ARROW_SMALL_UP;
        osdPrintFloat(element->buff, directionSymbol, fabsf(verticalSpeed), "", 1, true, osdGetVarioToSelectedUnitSymbol());
    } else {
        // We use this symbol when we don't have a valid measure
        element->buff[0] = SYM_HYPHEN;
        element->buff[1] = '\0';
    }
}
#endif // USE_VARIO

static void osdElementPidRateProfile(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "%d-%d", getCurrentPidProfileIndex() + 1, getCurrentControlRateProfileIndex() + 1);
}

static void osdElementPidsPitch(osdElementParms_t *element)
{
    osdFormatPID(element->buff, "PIT", PID_PITCH);
}

static void osdElementPidsRoll(osdElementParms_t *element)
{
    osdFormatPID(element->buff, "ROL", PID_ROLL);
}

static void osdElementPidsYaw(osdElementParms_t *element)
{
    osdFormatPID(element->buff, "YAW", PID_YAW);
}

static void osdElementPower(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "%4dW", getAmperage() * getBatteryVoltage() / 10000);
}

static void osdElementRcChannels(osdElementParms_t *element)
{
    static uint8_t channel = 0;

    if (osdConfig()->rcChannels[channel] >= 0) {
        // Translate (1000, 2000) to (-1000, 1000)
        int data = scaleRange(rcData[osdConfig()->rcChannels[channel]], PWM_RANGE_MIN, PWM_RANGE_MAX, -1000, 1000);
        // Opt for the simplest formatting for now.
        // Decimal notation can be added when tfp_sprintf supports float among fancy options.
        tfp_sprintf(element->buff, "%5d", data);
        element->elemOffsetY = channel;
    }

    if (++channel == OSD_RCCHANNELS_COUNT) {
        channel = 0;
    } else {
        element->rendered = false;
    }
}

static void osdElementRemainingTimeEstimate(osdElementParms_t *element)
{
    const int mAhDrawn = getMAhDrawn();

    if (mAhDrawn >= osdConfig()->cap_alarm) {
        element->attr = DISPLAYPORT_SEVERITY_CRITICAL;
    }

    if (mAhDrawn <= 0.1f * osdConfig()->cap_alarm) {  // also handles the mAhDrawn == 0 condition
        tfp_sprintf(element->buff, "--:--");
    } else if (mAhDrawn > osdConfig()->cap_alarm) {
        tfp_sprintf(element->buff, "00:00");
    } else {
        const int remaining_time = (int)((osdConfig()->cap_alarm - mAhDrawn) * ((float)osdFlyTime) / mAhDrawn);
        osdFormatTime(element->buff, OSD_TIMER_PREC_SECOND, remaining_time);
    }
}

static void osdElementRssi(osdElementParms_t *element)
{
    uint16_t osdRssi = getRssi() * 100 / 1024; // change range
    if (osdRssi >= 100) {
        osdRssi = 99;
    }

    if (getRssiPercent() < osdConfig()->rssi_alarm) {
        element->attr = DISPLAYPORT_SEVERITY_CRITICAL;
    }

    tfp_sprintf(element->buff, "%c%2d", SYM_RSSI, osdRssi);
}

#ifdef USE_RTC_TIME
static void osdElementRtcTime(osdElementParms_t *element)
{
    osdFormatRtcDateTime(&element->buff[0]);
}
#endif // USE_RTC_TIME

#ifdef USE_RX_RSSI_DBM
static void osdElementRssiDbm(osdElementParms_t *element)
{
    const int8_t antenna = getActiveAntenna();
    const int16_t osdRssiDbm = getRssiDbm();
    static bool diversity = false;

    if (osdRssiDbm < osdConfig()->rssi_dbm_alarm) {
        element->attr = DISPLAYPORT_SEVERITY_CRITICAL;
    }

    if (antenna || diversity) {
        diversity = true;
        tfp_sprintf(element->buff, "%c%3d:%d", SYM_RSSI, osdRssiDbm, antenna + 1);
    } else {
        tfp_sprintf(element->buff, "%c%3d", SYM_RSSI, osdRssiDbm);
    }
}
#endif // USE_RX_RSSI_DBM

#ifdef USE_RX_RSNR
static void osdElementRsnr(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "%c%3d", SYM_RSSI, getRsnr());
}
#endif // USE_RX_RSNR

#ifdef USE_OSD_STICK_OVERLAY
static void osdBackgroundStickOverlay(osdElementParms_t *element)
{
    static enum {VERT, HORZ} renderPhase = VERT;

    if (renderPhase == VERT) {
        static uint8_t y = 0;
        tfp_sprintf(element->buff, "%c", SYM_STICK_OVERLAY_VERTICAL);
        element->elemOffsetX = ((OSD_STICK_OVERLAY_WIDTH - 1) / 2);
        element->elemOffsetY = y;

        y++;

        if (y == (OSD_STICK_OVERLAY_HEIGHT - 1) / 2) {
            // Skip over horizontal
            y++;
        }

        if (y == OSD_STICK_OVERLAY_HEIGHT) {
            y = 0;
            renderPhase = HORZ;
        }

        element->rendered = false;
    } else {
        for (uint8_t i = 0; i < OSD_STICK_OVERLAY_WIDTH; i++) {
            element->buff[i] = SYM_STICK_OVERLAY_HORIZONTAL;
        }
        element->buff[((OSD_STICK_OVERLAY_WIDTH - 1) / 2)] = SYM_STICK_OVERLAY_CENTER;
        element->buff[OSD_STICK_OVERLAY_WIDTH] = 0;  // string terminator

        element->elemOffsetY = ((OSD_STICK_OVERLAY_HEIGHT - 1) / 2);

        renderPhase = VERT;
    }
}

static void osdElementStickOverlay(osdElementParms_t *element)
{
    // Now draw the cursor
    rc_alias_e vertical_channel, horizontal_channel;

    if (element->item == OSD_STICK_OVERLAY_LEFT) {
        vertical_channel = radioModes[osdConfig()->overlay_radio_mode-1].left_vertical;
        horizontal_channel = radioModes[osdConfig()->overlay_radio_mode-1].left_horizontal;
    } else {
        vertical_channel = radioModes[osdConfig()->overlay_radio_mode-1].right_vertical;
        horizontal_channel = radioModes[osdConfig()->overlay_radio_mode-1].right_horizontal;
    }

    const uint8_t cursorX = scaleRange(constrain(rcData[horizontal_channel], PWM_RANGE_MIN, PWM_RANGE_MAX - 1), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, OSD_STICK_OVERLAY_WIDTH);
    const uint8_t cursorY = OSD_STICK_OVERLAY_VERTICAL_POSITIONS - 1 - scaleRange(constrain(rcData[vertical_channel], PWM_RANGE_MIN, PWM_RANGE_MAX - 1), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, OSD_STICK_OVERLAY_VERTICAL_POSITIONS);
    const char cursor = SYM_STICK_OVERLAY_SPRITE_HIGH + (cursorY % OSD_STICK_OVERLAY_SPRITE_HEIGHT);

    tfp_sprintf(element->buff, "%c", cursor);
    element->elemOffsetX = cursorX;
    element->elemOffsetY = cursorY / OSD_STICK_OVERLAY_SPRITE_HEIGHT;
}
#endif // USE_OSD_STICK_OVERLAY

static void osdElementThrottlePosition(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "%c%3d", SYM_THR, calculateThrottlePercent());
}

static void osdElementTimer(osdElementParms_t *element)
{
    for (int i = 0; i < OSD_TIMER_COUNT; i++) {
        const uint16_t timer = osdConfig()->timers[i];
        const timeUs_t time = osdGetTimerValue(OSD_TIMER_SRC(timer));
        const timeUs_t alarmTime = OSD_TIMER_ALARM(timer) * 60000000; // convert from minutes to us
        if (alarmTime != 0 && time >= alarmTime) {
            element->attr = DISPLAYPORT_SEVERITY_CRITICAL;
        }
    }

    osdFormatTimer(element->buff, true, true, element->item - OSD_ITEM_TIMER_1);
}

#ifdef USE_VTX_COMMON
static void osdElementVtxChannel(osdElementParms_t *element)
{
    const vtxDevice_t *vtxDevice = vtxCommonDevice();
    uint8_t band = vtxSettingsConfigMutable()->band;
    uint8_t channel = vtxSettingsConfig()->channel;
    if (band == 0) {
        /* Direct frequency set is used */
        vtxCommonLookupBandChan(vtxDevice, vtxSettingsConfig()->freq, &band, &channel);
    }
    const char vtxBandLetter = vtxCommonLookupBandLetter(vtxDevice, band);
    const char *vtxChannelName = vtxCommonLookupChannelName(vtxDevice, channel);
    unsigned vtxStatus = 0;
    uint8_t vtxPower = vtxSettingsConfig()->power;
    if (vtxDevice) {
        vtxCommonGetStatus(vtxDevice, &vtxStatus);

        if (vtxSettingsConfig()->lowPowerDisarm) {
            vtxCommonGetPowerIndex(vtxDevice, &vtxPower);
        }
    }
    const char *vtxPowerLabel = vtxCommonLookupPowerName(vtxDevice, vtxPower);

    char vtxStatusIndicator = '\0';
    if (IS_RC_MODE_ACTIVE(BOXVTXCONTROLDISABLE)) {
        vtxStatusIndicator = 'D';
    } else if (vtxStatus & VTX_STATUS_PIT_MODE) {
        vtxStatusIndicator = 'P';
    }

switch (element->type) {
    case OSD_ELEMENT_TYPE_2:
            tfp_sprintf(element->buff, "%s", vtxPowerLabel);
        break;

    default:
        if (vtxStatus & VTX_STATUS_LOCKED) {
            tfp_sprintf(element->buff, "-:-:-:L");
        } else if (vtxStatusIndicator) {
            tfp_sprintf(element->buff, "%c:%s:%s:%c", vtxBandLetter, vtxChannelName, vtxPowerLabel, vtxStatusIndicator);
        } else {
            tfp_sprintf(element->buff, "%c:%s:%s", vtxBandLetter, vtxChannelName, vtxPowerLabel);
        }
        break;
    }
}
#endif // USE_VTX_COMMON

static void osdElementAuxValue(osdElementParms_t *element)
{
    tfp_sprintf(element->buff, "%c%d", osdConfig()->aux_symbol, osdAuxValue);
}

static void osdElementWarnings(osdElementParms_t *element)
{
    bool elementBlinking = false;
    renderOsdWarning(element->buff, &elementBlinking, &element->attr);
    if (elementBlinking) {
        SET_BLINK(OSD_WARNINGS);
    } else {
        CLR_BLINK(OSD_WARNINGS);
    }

#ifdef USE_CRAFTNAME_MSGS
    // Injects data into the CraftName variable for systems which limit
    // the available MSP data field in their OSD.
    if (osdConfig()->osd_craftname_msgs == true) {
        // if warning is not set, or blink is off, then display LQ & RSSI
        if (blinkState || (strlen(element->buff) == 0)) {
#ifdef USE_RX_LINK_QUALITY_INFO
            // replicate the LQ functionality without the special font symbols
            uint16_t osdLinkQuality = 0;
            if (linkQualitySource == LQ_SOURCE_RX_PROTOCOL_CRSF) { // 0-99
                osdLinkQuality = rxGetLinkQuality();
#ifdef USE_RX_RSSI_DBM
                const uint8_t osdRfMode = rxGetRfMode();
                tfp_sprintf(element->buff, "LQ %2d:%03d %3d", osdRfMode, osdLinkQuality, getRssiDbm());
            } else if (linkQualitySource == LQ_SOURCE_RX_PROTOCOL_GHST) { // 0-100
                osdLinkQuality = rxGetLinkQuality();
                tfp_sprintf(element->buff, "LQ %03d %3d", osdLinkQuality, getRssiDbm());
#endif
            } else { // 0-9
                osdLinkQuality = rxGetLinkQuality() * 10 / LINK_QUALITY_MAX_VALUE;
                if (osdLinkQuality >= 10) {
                    osdLinkQuality = 9;
                }
                tfp_sprintf(element->buff, "LQ %1d", osdLinkQuality);
            }
#endif // USE_RX_LINK_QUALITY_INFO
        }
        strncpy(pilotConfigMutable()->craftName, element->buff, MAX_NAME_LENGTH - 1);
    }
#endif // USE_CRAFTNAME_MSGS
}

#ifdef USE_MSP_DISPLAYPORT
static void osdElementSys(osdElementParms_t *element)
{
    UNUSED(element);

    // Nothing to render for a system element
}
#endif

// Define the order in which the elements are drawn.
// Elements positioned later in the list will overlay the earlier
// ones if their character positions overlap
// Elements that need special runtime conditional processing should be added
// to osdAddActiveElements()

static const uint8_t osdElementDisplayOrder[] = {
    OSD_MAIN_BATT_VOLTAGE,
    OSD_RSSI_VALUE,
    OSD_CROSSHAIRS,
    OSD_HORIZON_SIDEBARS,
    OSD_UP_DOWN_REFERENCE,
    OSD_ITEM_TIMER_1,
    OSD_ITEM_TIMER_2,
    OSD_REMAINING_TIME_ESTIMATE,
    OSD_FLYMODE,
    OSD_THROTTLE_POS,
    OSD_VTX_CHANNEL,
    OSD_CURRENT_DRAW,
    OSD_MAH_DRAWN,
    OSD_WATT_HOURS_DRAWN,
    OSD_CRAFT_NAME,
    OSD_CUSTOM_MSG0,
    OSD_CUSTOM_MSG1,
    OSD_CUSTOM_MSG2,
    OSD_CUSTOM_MSG3,
    OSD_ALTITUDE,
    OSD_ROLL_PIDS,
    OSD_PITCH_PIDS,
    OSD_YAW_PIDS,
    OSD_POWER,
    OSD_PIDRATE_PROFILE,
    OSD_WARNINGS,
    OSD_AVG_CELL_VOLTAGE,
    OSD_DEBUG,
    OSD_DEBUG2,
    OSD_PITCH_ANGLE,
    OSD_ROLL_ANGLE,
    OSD_MAIN_BATT_USAGE,
    OSD_DISARMED,
    OSD_NUMERICAL_HEADING,
    OSD_READY_MODE,
#ifdef USE_VARIO
    OSD_NUMERICAL_VARIO,
#endif
    OSD_COMPASS_BAR,
    OSD_ANTI_GRAVITY,
#ifdef USE_BLACKBOX
    OSD_LOG_STATUS,
#endif
    OSD_MOTOR_DIAG,
#ifdef USE_ACC
    OSD_FLIP_ARROW,
#endif
    OSD_PILOT_NAME,
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
#ifdef USE_RX_LINK_UPLINK_POWER
    OSD_TX_UPLINK_POWER,
#endif
#ifdef USE_RX_RSSI_DBM
    OSD_RSSI_DBM_VALUE,
#endif
#ifdef USE_RX_RSNR
    OSD_RSNR_VALUE,
#endif
#ifdef USE_OSD_STICK_OVERLAY
    OSD_STICK_OVERLAY_LEFT,
    OSD_STICK_OVERLAY_RIGHT,
#endif
#ifdef USE_PROFILE_NAMES
    OSD_RATE_PROFILE_NAME,
    OSD_PID_PROFILE_NAME,
#endif
#ifdef USE_OSD_PROFILES
    OSD_PROFILE_NAME,
#endif
    OSD_RC_CHANNELS,
    OSD_CAMERA_FRAME,
#ifdef USE_PERSISTENT_STATS
    OSD_TOTAL_FLIGHTS,
#endif
    OSD_AUX_VALUE,
#ifdef USE_OSD_HD
    OSD_SYS_GOGGLE_VOLTAGE,
    OSD_SYS_VTX_VOLTAGE,
    OSD_SYS_BITRATE,
    OSD_SYS_DELAY,
    OSD_SYS_DISTANCE,
    OSD_SYS_LQ,
    OSD_SYS_GOGGLE_DVR,
    OSD_SYS_VTX_DVR,
    OSD_SYS_WARNINGS,
    OSD_SYS_VTX_TEMP,
    OSD_SYS_FAN_SPEED,
#endif
#ifdef USE_RANGEFINDER
    OSD_LIDAR_DIST,
#endif
};

// Define the mapping between the OSD element id and the function to draw it

const osdElementDrawFn osdElementDrawFunction[OSD_ITEM_COUNT] = {
    [OSD_CAMERA_FRAME]            = NULL,  // only has background. Added first so it's the lowest "layer" and doesn't cover other elements
    [OSD_RSSI_VALUE]              = osdElementRssi,
    [OSD_MAIN_BATT_VOLTAGE]       = osdElementMainBatteryVoltage,
    [OSD_CROSSHAIRS]              = osdElementCrosshairs,  // only has background, but needs to be over other elements (like artificial horizon)
#ifdef USE_ACC
    [OSD_ARTIFICIAL_HORIZON]      = osdElementArtificialHorizon,
    [OSD_UP_DOWN_REFERENCE]       = osdElementUpDownReference,
#endif
    [OSD_HORIZON_SIDEBARS]        = NULL,  // only has background
    [OSD_ITEM_TIMER_1]            = osdElementTimer,
    [OSD_ITEM_TIMER_2]            = osdElementTimer,
    [OSD_FLYMODE]                 = osdElementFlymode,
    [OSD_CRAFT_NAME]              = NULL,  // only has background
    [OSD_CUSTOM_MSG0]             = osdElementCustomMsg,
    [OSD_CUSTOM_MSG1]             = osdElementCustomMsg,
    [OSD_CUSTOM_MSG2]             = osdElementCustomMsg,
    [OSD_CUSTOM_MSG3]             = osdElementCustomMsg,
    [OSD_THROTTLE_POS]            = osdElementThrottlePosition,
#ifdef USE_VTX_COMMON
    [OSD_VTX_CHANNEL]             = osdElementVtxChannel,
#endif
    [OSD_CURRENT_DRAW]            = osdElementCurrentDraw,
    [OSD_MAH_DRAWN]               = osdElementMahDrawn,
    [OSD_WATT_HOURS_DRAWN]        = osdElementWattHoursDrawn,
#ifdef USE_GPS
    [OSD_GPS_SPEED]               = osdElementGpsSpeed,
    [OSD_GPS_SATS]                = osdElementGpsSats,
#endif
    [OSD_ALTITUDE]                = osdElementAltitude,
    [OSD_ROLL_PIDS]               = osdElementPidsRoll,
    [OSD_PITCH_PIDS]              = osdElementPidsPitch,
    [OSD_YAW_PIDS]                = osdElementPidsYaw,
    [OSD_POWER]                   = osdElementPower,
    [OSD_PIDRATE_PROFILE]         = osdElementPidRateProfile,
    [OSD_WARNINGS]                = osdElementWarnings,
    [OSD_AVG_CELL_VOLTAGE]        = osdElementAverageCellVoltage,
    [OSD_READY_MODE]              = osdElementReadyMode,
#ifdef USE_GPS
    [OSD_GPS_LON]                 = osdElementGpsCoordinate,
    [OSD_GPS_LAT]                 = osdElementGpsCoordinate,
#endif
    [OSD_DEBUG]                   = osdElementDebug,
    [OSD_DEBUG2]                  = osdElementDebug2,
#ifdef USE_ACC
    [OSD_PITCH_ANGLE]             = osdElementAngleRollPitch,
    [OSD_ROLL_ANGLE]              = osdElementAngleRollPitch,
#endif
    [OSD_MAIN_BATT_USAGE]         = osdElementMainBatteryUsage,
    [OSD_DISARMED]                = osdElementDisarmed,
#ifdef USE_GPS
    [OSD_HOME_DIR]                = osdElementGpsHomeDirection,
    [OSD_HOME_DIST]               = osdElementGpsHomeDistance,
#endif
    [OSD_NUMERICAL_HEADING]       = osdElementNumericalHeading,
#ifdef USE_VARIO
    [OSD_NUMERICAL_VARIO]         = osdElementNumericalVario,
#endif
    [OSD_COMPASS_BAR]             = osdElementCompassBar,
#if defined(USE_DSHOT_TELEMETRY) || defined(USE_ESC_SENSOR)
    [OSD_ESC_TMP]                 = osdElementEscTemperature,
    [OSD_ESC_RPM]                 = osdElementEscRpm,
#endif
    [OSD_REMAINING_TIME_ESTIMATE] = osdElementRemainingTimeEstimate,
#ifdef USE_RTC_TIME
    [OSD_RTC_DATETIME]            = osdElementRtcTime,
#endif
#ifdef USE_OSD_ADJUSTMENTS
    [OSD_ADJUSTMENT_RANGE]        = osdElementAdjustmentRange,
#endif
#ifdef USE_ADC_INTERNAL
    [OSD_CORE_TEMPERATURE]        = osdElementCoreTemperature,
#endif
    [OSD_ANTI_GRAVITY]            = osdElementAntiGravity,
#ifdef USE_ACC
    [OSD_G_FORCE]                 = osdElementGForce,
#endif
    [OSD_MOTOR_DIAG]              = osdElementMotorDiagnostics,
#ifdef USE_BLACKBOX
    [OSD_LOG_STATUS]              = osdElementLogStatus,
#endif
#ifdef USE_ACC
    [OSD_FLIP_ARROW]              = osdElementCrashFlipArrow,
#endif
#ifdef USE_RX_LINK_QUALITY_INFO
    [OSD_LINK_QUALITY]            = osdElementLinkQuality,
#endif
#ifdef USE_RX_LINK_UPLINK_POWER
    [OSD_TX_UPLINK_POWER]         = osdElementTxUplinkPower,
#endif
#ifdef USE_GPS
    [OSD_FLIGHT_DIST]             = osdElementGpsFlightDistance,
#endif
#ifdef USE_OSD_STICK_OVERLAY
    [OSD_STICK_OVERLAY_LEFT]      = osdElementStickOverlay,
    [OSD_STICK_OVERLAY_RIGHT]     = osdElementStickOverlay,
#endif
    [OSD_PILOT_NAME]              = NULL,  // only has background
#if defined(USE_DSHOT_TELEMETRY) || defined(USE_ESC_SENSOR)
    [OSD_ESC_RPM_FREQ]            = osdElementEscRpmFreq,
#endif
#ifdef USE_PROFILE_NAMES
    [OSD_RATE_PROFILE_NAME]       = osdElementRateProfileName,
    [OSD_PID_PROFILE_NAME]        = osdElementPidProfileName,
#endif
#ifdef USE_OSD_PROFILES
    [OSD_PROFILE_NAME]            = osdElementOsdProfileName,
#endif
#ifdef USE_RX_RSSI_DBM
    [OSD_RSSI_DBM_VALUE]          = osdElementRssiDbm,
#endif
#ifdef USE_RX_RSNR
    [OSD_RSNR_VALUE]              = osdElementRsnr,
#endif
    [OSD_RC_CHANNELS]             = osdElementRcChannels,
#ifdef USE_GPS
    [OSD_EFFICIENCY]              = osdElementEfficiency,
#endif
#ifdef USE_GPS_LAP_TIMER
    [OSD_GPS_LAP_TIME_CURRENT]    = osdElementGpsLapTimeCurrent,
    [OSD_GPS_LAP_TIME_PREVIOUS]   = osdElementGpsLapTimePrevious,
    [OSD_GPS_LAP_TIME_BEST3]      = osdElementGpsLapTimeBest3,
#endif // GPS_LAP_TIMER
#ifdef USE_PERSISTENT_STATS
    [OSD_TOTAL_FLIGHTS]           = osdElementTotalFlights,
#endif
    [OSD_AUX_VALUE]               = osdElementAuxValue,
#ifdef USE_MSP_DISPLAYPORT
    [OSD_SYS_GOGGLE_VOLTAGE]      = osdElementSys,
    [OSD_SYS_VTX_VOLTAGE]         = osdElementSys,
    [OSD_SYS_BITRATE]             = osdElementSys,
    [OSD_SYS_DELAY]               = osdElementSys,
    [OSD_SYS_DISTANCE]            = osdElementSys,
    [OSD_SYS_LQ]                  = osdElementSys,
    [OSD_SYS_GOGGLE_DVR]          = osdElementSys,
    [OSD_SYS_VTX_DVR]             = osdElementSys,
    [OSD_SYS_WARNINGS]            = osdElementSys,
    [OSD_SYS_VTX_TEMP]            = osdElementSys,
    [OSD_SYS_FAN_SPEED]           = osdElementSys,
#endif
#ifdef USE_RANGEFINDER
    [OSD_LIDAR_DIST]              = osdElementLidarDist,
#endif
};

// Define the mapping between the OSD element id and the function to draw its background (static part)
// Only necessary to define the entries that actually have a background function

const osdElementDrawFn osdElementBackgroundFunction[OSD_ITEM_COUNT] = {
    [OSD_CAMERA_FRAME]            = osdBackgroundCameraFrame,
    [OSD_HORIZON_SIDEBARS]        = osdBackgroundHorizonSidebars,
    [OSD_CRAFT_NAME]              = osdBackgroundCraftName,
#ifdef USE_OSD_STICK_OVERLAY
    [OSD_STICK_OVERLAY_LEFT]      = osdBackgroundStickOverlay,
    [OSD_STICK_OVERLAY_RIGHT]     = osdBackgroundStickOverlay,
#endif
    [OSD_PILOT_NAME]              = osdBackgroundPilotName,
};

static void osdAddActiveElement(osd_items_e element)
{
    if (VISIBLE(osdElementConfig()->item_pos[element])) {
        activeOsdElementArray[activeOsdElementCount++] = element;
    }
}

// Examine the elements and build a list of only the active (enabled)
// ones to speed up rendering.

void osdAddActiveElements(void)
{
    activeOsdElementCount = 0;

#ifdef USE_ACC
    if (sensors(SENSOR_ACC)) {
        osdAddActiveElement(OSD_ARTIFICIAL_HORIZON);
        osdAddActiveElement(OSD_G_FORCE);
        osdAddActiveElement(OSD_UP_DOWN_REFERENCE);
    }
#endif

    for (unsigned i = 0; i < sizeof(osdElementDisplayOrder); i++) {
        osdAddActiveElement(osdElementDisplayOrder[i]);
    }

#ifdef USE_GPS
    if (sensors(SENSOR_GPS)) {
        osdAddActiveElement(OSD_GPS_SATS);
        osdAddActiveElement(OSD_GPS_SPEED);
        osdAddActiveElement(OSD_GPS_LAT);
        osdAddActiveElement(OSD_GPS_LON);
        osdAddActiveElement(OSD_HOME_DIST);
        osdAddActiveElement(OSD_HOME_DIR);
        osdAddActiveElement(OSD_FLIGHT_DIST);
        osdAddActiveElement(OSD_EFFICIENCY);
    }
#endif // GPS

#if defined(USE_DSHOT_TELEMETRY) || defined(USE_ESC_SENSOR)
    if ((featureIsEnabled(FEATURE_ESC_SENSOR)) || useDshotTelemetry) {
        osdAddActiveElement(OSD_ESC_TMP);
        osdAddActiveElement(OSD_ESC_RPM);
        osdAddActiveElement(OSD_ESC_RPM_FREQ);
    }
#endif

#ifdef USE_GPS_LAP_TIMER
    if (sensors(SENSOR_GPS)) {
        osdAddActiveElement(OSD_GPS_LAP_TIME_CURRENT);
        osdAddActiveElement(OSD_GPS_LAP_TIME_PREVIOUS);
        osdAddActiveElement(OSD_GPS_LAP_TIME_BEST3);
    }
#endif // GPS_LAP_TIMER

#ifdef USE_PERSISTENT_STATS
    osdAddActiveElement(OSD_TOTAL_FLIGHTS);
#endif
}

static bool osdDrawSingleElement(displayPort_t *osdDisplayPort, uint8_t item)
{
    // By default mark the element as rendered in case it's in the off blink state
    activeElement.rendered = true;

    if (!osdElementDrawFunction[item]) {
        // Element has no drawing function
        return true;
    }
    if (!osdDisplayPort->useDeviceBlink && BLINK(item)) {
        return true;
    }

    uint8_t elemPosX = OSD_X(osdElementConfig()->item_pos[item]);
    uint8_t elemPosY = OSD_Y(osdElementConfig()->item_pos[item]);

    activeElement.item = item;
    activeElement.elemPosX = elemPosX;
    activeElement.elemPosY = elemPosY;
    activeElement.elemOffsetX = 0;
    activeElement.elemOffsetY = 0;
    activeElement.type = OSD_TYPE(osdElementConfig()->item_pos[item]);
    activeElement.buff = elementBuff;
    activeElement.osdDisplayPort = osdDisplayPort;
    activeElement.drawElement = true;
    activeElement.attr = DISPLAYPORT_SEVERITY_NORMAL;

    // Call the element drawing function
    if (IS_SYS_OSD_ELEMENT(item)) {
        displaySys(osdDisplayPort, elemPosX, elemPosY, (displayPortSystemElement_e)(item - OSD_SYS_GOGGLE_VOLTAGE + DISPLAYPORT_SYS_GOGGLE_VOLTAGE));
    } else {
        osdElementDrawFunction[item](&activeElement);
        if (activeElement.drawElement) {
            displayPendingForeground = true;
        }
    }

    return activeElement.rendered;
}

static bool osdDrawSingleElementBackground(displayPort_t *osdDisplayPort, uint8_t item)
{
    if (!osdElementBackgroundFunction[item]) {
        // Element has no background drawing function
        return true;
    }

    uint8_t elemPosX = OSD_X(osdElementConfig()->item_pos[item]);
    uint8_t elemPosY = OSD_Y(osdElementConfig()->item_pos[item]);

    activeElement.item = item;
    activeElement.elemPosX = elemPosX;
    activeElement.elemPosY = elemPosY;
    activeElement.elemOffsetX = 0;
    activeElement.elemOffsetY = 0;
    activeElement.type = OSD_TYPE(osdElementConfig()->item_pos[item]);
    activeElement.buff = elementBuff;
    activeElement.osdDisplayPort = osdDisplayPort;
    activeElement.drawElement = true;
    activeElement.rendered = true;
    activeElement.attr = DISPLAYPORT_SEVERITY_NORMAL;

    // Call the element background drawing function
    osdElementBackgroundFunction[item](&activeElement);
    if (activeElement.drawElement) {
        displayPendingBackground = true;
    }

    return activeElement.rendered;
}

static uint8_t activeElementNumber = 0;

bool osdIsRenderPending(void)
{
    return displayPendingForeground | displayPendingBackground;
}

uint8_t osdGetActiveElement(void)
{
    return activeElementNumber;
}

uint8_t osdGetActiveElementCount(void)
{
    return activeOsdElementCount;
}

// Return true if there is more to display
bool osdDisplayActiveElement(void)
{
    if (activeElementNumber >= activeOsdElementCount) {
        return false;
    }

    // If there's a previously drawn background string to be displayed, do that
    if (displayPendingBackground) {
        osdDisplayWrite(&activeElement,
                        activeElement.elemPosX + activeElement.elemOffsetX,
                        activeElement.elemPosY + activeElement.elemOffsetY,
                        activeElement.attr, activeElement.buff);

        activeElement.buff[0] = '\0';

        displayPendingBackground = false;

        return displayPendingForeground;
    }

    // If there's a previously drawn foreground string to be displayed, do that
    if (displayPendingForeground) {
        osdDisplayWrite(&activeElement,
                        activeElement.elemPosX + activeElement.elemOffsetX,
                        activeElement.elemPosY + activeElement.elemOffsetY,
                        activeElement.attr, activeElement.buff);

        activeElement.buff[0] = '\0';

        displayPendingForeground = false;
    }

    return false;
}

// Return true if there are more elements to draw
bool osdDrawNextActiveElement(displayPort_t *osdDisplayPort)
{
    static bool backgroundRendered = false;

    if (activeElementNumber >= activeOsdElementCount) {
        activeElementNumber = 0;
        return false;
    }

    uint8_t item = activeOsdElementArray[activeElementNumber];

    if (!backgroundLayerSupported && osdElementBackgroundFunction[item] && !backgroundRendered) {
        // If the background layer isn't supported then we
        // have to draw the element's static layer as well.
        backgroundRendered = osdDrawSingleElementBackground(osdDisplayPort, item);

        // After the background always come back to check for foreground
        return true;
    }

    // Only advance to the next element if rendering is complete
    if (osdDrawSingleElement(osdDisplayPort, item)) {
        // If rendering is complete then advance to the next element

        // Prepare to render the background of the next element
        backgroundRendered = false;

        if (++activeElementNumber >= activeOsdElementCount) {
            activeElementNumber = 0;
            return false;
        }
    }

    return true;
}

#ifdef USE_SPEC_PREARM_SCREEN
bool osdDrawSpec(displayPort_t *osdDisplayPort)
{
    static enum {RPM, POLES, MIXER, THR, MOTOR, BAT, VER} specState = RPM;
    static int currentRow;

    const uint8_t midRow = osdDisplayPort->rows / 2;
    const uint8_t midCol = osdDisplayPort->cols / 2;

    char buff[OSD_ELEMENT_BUFFER_LENGTH] = "";

    int len = 0;

    switch (specState) {
    default:
    case RPM:
        currentRow = midRow - 3;
#ifdef USE_RPM_LIMIT
        {
            const bool rpmLimitActive = mixerConfig()->rpm_limit > 0 && isMotorProtocolBidirDshot();
            if (rpmLimitActive) {
                len = tfp_sprintf(buff, "RPM LIMIT ON  %d", mixerConfig()->rpm_limit_value);
            } else {
                len = tfp_sprintf(buff, "%s", "RPM LIMIT OFF");
            }
            displayWrite(osdDisplayPort, midCol - (len / 2), currentRow++, DISPLAYPORT_SEVERITY_NORMAL, buff);

            if (rpmLimitActive) {
                specState = POLES;
            } else {
                specState = THR;
            }
        }
        break;

    case POLES:
        len = tfp_sprintf(buff, "KV %d   POLES %d", motorConfig()->kv, motorConfig()->motorPoleCount);
        displayWrite(osdDisplayPort, midCol - (len / 2), currentRow++, DISPLAYPORT_SEVERITY_NORMAL, buff);

        specState = MIXER;
        break;

    case MIXER:
        len = tfp_sprintf(buff, "%d  %d  %d", mixerConfig()->rpm_limit_p, mixerConfig()->rpm_limit_i, mixerConfig()->rpm_limit_d);
        displayWrite(osdDisplayPort, midCol - (len / 2), currentRow++, DISPLAYPORT_SEVERITY_NORMAL, buff);

        specState = THR;
        break;

    case THR:
#endif // #USE_RPM_LIMIT
        len = tfp_sprintf(buff, "THR LIMIT %s", lookupTableThrottleLimitType[currentControlRateProfile->throttle_limit_type]);
        if (currentControlRateProfile->throttle_limit_type != THROTTLE_LIMIT_TYPE_OFF) {
            len = tfp_sprintf(buff, "%s %d", buff, currentControlRateProfile->throttle_limit_percent);
        }
        displayWrite(osdDisplayPort, midCol - (len / 2), currentRow++, DISPLAYPORT_SEVERITY_NORMAL, buff);

        specState = MOTOR;
        break;

    case MOTOR:
        len = tfp_sprintf(buff, "MOTOR LIMIT %d", currentPidProfile->motor_output_limit);
        displayWrite(osdDisplayPort, midCol - (len / 2), currentRow++, DISPLAYPORT_SEVERITY_NORMAL, buff);

        specState = BAT;
        break;

    case BAT:
        {
            const float batteryVoltage = getBatteryVoltage() / 100.0f;
            len = osdPrintFloat(buff, osdGetBatterySymbol(getBatteryAverageCellVoltage()), batteryVoltage, "", 2, true, SYM_VOLT);
            displayWrite(osdDisplayPort, midCol - (len / 2), currentRow++, DISPLAYPORT_SEVERITY_NORMAL, buff);
        }

        specState = VER;
        break;

    case VER:
        len = strlen(FC_VERSION_STRING);
        displayWrite(osdDisplayPort, midCol - (len / 2), currentRow++, DISPLAYPORT_SEVERITY_NORMAL, FC_VERSION_STRING);

        specState = RPM;

        return true;
    }

    return false;
}
#endif // USE_SPEC_PREARM_SCREEN

void osdDrawActiveElementsBackground(displayPort_t *osdDisplayPort)
{
    if (backgroundLayerSupported) {
        displayLayerSelect(osdDisplayPort, DISPLAYPORT_LAYER_BACKGROUND);
        displayClearScreen(osdDisplayPort, DISPLAY_CLEAR_WAIT);
        for (unsigned i = 0; i < activeOsdElementCount; i++) {
            while (!osdDrawSingleElementBackground(osdDisplayPort, activeOsdElementArray[i]));
        }
        displayLayerSelect(osdDisplayPort, DISPLAYPORT_LAYER_FOREGROUND);
    }
}

void osdElementsInit(bool backgroundLayerFlag)
{
    backgroundLayerSupported = backgroundLayerFlag;
    activeOsdElementCount = 0;
    pt1FilterInitLPF(&batteryEfficiencyFilt, EFFICIENCY_CUTOFF_HZ, 1.0f / osdConfig()->framerate_hz);
}

void osdSyncBlink(timeUs_t currentTimeUs)
{
    const int period = 1000000/OSD_BLINK_FREQUENCY_HZ;

    blinkState = ((currentTimeUs % period) < (period >> 1));
}

void osdResetAlarms(void)
{
    memset(blinkBits, 0, sizeof(blinkBits));
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

#ifdef USE_RX_RSSI_DBM
    if (getRssiDbm() < osdConfig()->rssi_dbm_alarm) {
        SET_BLINK(OSD_RSSI_DBM_VALUE);
    } else {
        CLR_BLINK(OSD_RSSI_DBM_VALUE);
    }
#endif

#ifdef USE_RX_LINK_QUALITY_INFO
    if (rxGetLinkQualityPercent() < osdConfig()->link_quality_alarm) {
        SET_BLINK(OSD_LINK_QUALITY);
    } else {
        CLR_BLINK(OSD_LINK_QUALITY);
    }
#endif // USE_RX_LINK_QUALITY_INFO

    if (getBatteryState() == BATTERY_OK) {
        CLR_BLINK(OSD_MAIN_BATT_VOLTAGE);
        CLR_BLINK(OSD_AVG_CELL_VOLTAGE);
    } else {
        SET_BLINK(OSD_MAIN_BATT_VOLTAGE);
        SET_BLINK(OSD_AVG_CELL_VOLTAGE);
    }

#ifdef USE_GPS
    if ((STATE(GPS_FIX) == 0) || (gpsSol.numSat < GPS_MIN_SAT_COUNT)
#ifdef USE_GPS_RESCUE
            || ((gpsSol.numSat < gpsRescueConfig()->minSats) && gpsRescueIsConfigured())
#endif
            ) {
        SET_BLINK(OSD_GPS_SATS);
    } else {
        CLR_BLINK(OSD_GPS_SATS);
    }
#endif //USE_GPS

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

    if ((alt >= osdConfig()->alt_alarm) && ARMING_FLAG(ARMED)) {
        SET_BLINK(OSD_ALTITUDE);
    } else {
        CLR_BLINK(OSD_ALTITUDE);
    }

#ifdef USE_GPS
    if (sensors(SENSOR_GPS) && ARMING_FLAG(ARMED) && STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
        if (osdConfig()->distance_alarm && GPS_distanceToHome >= osdConfig()->distance_alarm) {
            SET_BLINK(OSD_HOME_DIST);
        } else {
            CLR_BLINK(OSD_HOME_DIST);
        }
    } else {
        CLR_BLINK(OSD_HOME_DIST);
    }
#endif

#if defined(USE_ESC_SENSOR) || defined(USE_DSHOT_TELEMETRY)
    bool blink = false;

#if defined(USE_ESC_SENSOR)
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        // This works because the combined ESC data contains the maximum temperature seen amongst all ESCs
        blink = osdConfig()->esc_temp_alarm != ESC_TEMP_ALARM_OFF && osdEscDataCombined->temperature >= osdConfig()->esc_temp_alarm;
    } else
#endif
#if defined(USE_DSHOT_TELEMETRY)
    {
        if (osdConfig()->esc_temp_alarm != ESC_TEMP_ALARM_OFF) {
            for (uint32_t k = 0; !blink && (k < getMotorCount()); k++) {
                blink = (dshotTelemetryState.motorState[k].telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_TEMPERATURE)) != 0 &&
                    dshotTelemetryState.motorState[k].telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE] >= osdConfig()->esc_temp_alarm;
            }
        }
    }
#else
    {}
#endif

    if (blink) {
        SET_BLINK(OSD_ESC_TMP);
    } else {
        CLR_BLINK(OSD_ESC_TMP);
    }
#endif
}

#ifdef USE_ACC
static bool osdElementIsActive(osd_items_e element)
{
    for (unsigned i = 0; i < activeOsdElementCount; i++) {
        if (activeOsdElementArray[i] == element) {
            return true;
        }
    }
    return false;
}

// Determine if any active elements need the ACC
bool osdElementsNeedAccelerometer(void)
{
    return osdElementIsActive(OSD_ARTIFICIAL_HORIZON) ||
           osdElementIsActive(OSD_PITCH_ANGLE) ||
           osdElementIsActive(OSD_ROLL_ANGLE) ||
           osdElementIsActive(OSD_G_FORCE) ||
           osdElementIsActive(OSD_FLIP_ARROW) ||
           osdElementIsActive(OSD_UP_DOWN_REFERENCE);
}

#endif // USE_ACC

#endif // USE_OSD
