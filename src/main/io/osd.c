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
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include <string.h>
#include "platform.h"

#include "debug.h"
#include "version.h"
#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "common/printf.h"
#include "common/typeconversion.h"

#include "drivers/gpio.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/compass.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"
#include "drivers/light_ws2811strip.h"
#include "drivers/sound_beeper.h"
#include "drivers/max7456.h"
#include "drivers/max7456_symbols.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/flashfs.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/beeper.h"
#include "io/osd.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#define IS_HI(X)  (rcData[X] > 1750)
#define IS_LO(X)  (rcData[X] < 1250)
#define IS_MID(X) (rcData[X] > 1250 && rcData[X] < 1750)

//key definiotion because API provide menu navigation over MSP/GUI app - not used NOW
#define KEY_ENTER   0
#define KEY_UP      1
#define KEY_DOWN    2
#define KEY_LEFT    3
#define KEY_RIGHT   4
#define KEY_ESC     5

//osd current screen - to reduce long lines ;-)
#define OSD_cfg masterConfig.osdProfile
#define curr_profile masterConfig.profile[masterConfig.current_profile_index]

uint16_t refreshTimeout = 0;

#define VISIBLE_FLAG  0x0800
#define BLINK_FLAG    0x0400
uint8_t blinkState = 1;

#define OSD_POS(x,y)  (x | (y << 5))
#define OSD_X(x)      (x & 0x001F)
#define OSD_Y(x)      ((x >> 5) & 0x001F)
#define VISIBLE(x)    (x & VISIBLE_FLAG)
#define BLINK(x)      ((x & BLINK_FLAG) && blinkState)
#define BLINK_OFF(x)  (x & ~BLINK_FLAG)

extern uint8_t RSSI; // TODO: not used?

static uint16_t flyTime = 0;
uint8_t statRssi;

statistic_t stats;

#define BUTTON_TIME   2
#define BUTTON_PAUSE  5
#define REFRESH_1S    12

#define LEFT_MENU_COLUMN  1
#define RIGHT_MENU_COLUMN 23
#define MAX_MENU_ITEMS    (max7456GetRowsCount() - 2)

uint8_t osdRows;

//type of elements
typedef enum
{
    OME_Label,
    OME_Back,
    OME_OSD_Exit,
    OME_Submenu,
    OME_Bool,
    OME_INT8,
    OME_UINT8,
    OME_UINT16,
    OME_INT16,
    OME_FLOAT, //only up to 255 value and cant be 2.55 or 25.5, just for PID's
    //wlasciwosci elementow
    OME_VISIBLE,
    OME_POS,
    OME_TAB,
    OME_END,
} OSD_MenuElement;

//local variable to detect arm/disarm and show statistic etc
uint8_t armState;
uint8_t featureBlackbox = 0;
uint8_t featureLedstrip = 0;

#if defined(VTX) || defined(USE_RTC6705)
uint8_t featureVtx = 0, vtxBand, vtxChannel;
#endif // VTX || USE_RTC6705

// We are in menu flag
bool inMenu = false;

typedef void (* OSDMenuFuncPtr)(void *data);

void osdUpdate(uint8_t guiKey);
void osdOpenMenu(void);
void osdExitMenu(void * ptr);
void osdMenuBack(void);
void osdEditElement(void *ptr);
void osdEraseFlash(void *ptr);
void osdUpdateMaxRows(void);
void osdChangeScreen(void * ptr);
void osdDrawElements(void);
void osdDrawSingleElement(uint8_t item);

typedef struct
{
    char *text;
    OSD_MenuElement type;
    OSDMenuFuncPtr func;
    void *data;
} OSD_Entry;

typedef struct
{
    uint8_t *val;
    uint8_t min;
    uint8_t max;
    uint8_t step;
} OSD_UINT8_t;

typedef struct
{
    int8_t *val;
    int8_t min;
    int8_t max;
    int8_t step;
} OSD_INT8_t;

typedef struct
{
    int16_t *val;
    int16_t min;
    int16_t max;
    int16_t step;
} OSD_INT16_t;

typedef struct
{
    uint16_t *val;
    uint16_t min;
    uint16_t max;
    uint16_t step;
} OSD_UINT16_t;

typedef struct
{
    uint8_t *val;
    uint8_t min;
    uint8_t max;
    uint8_t step;
    uint16_t multipler;
} OSD_FLOAT_t;

typedef struct
{
    uint8_t *val;
    uint8_t max;
    const char * const *names;
} OSD_TAB_t;

OSD_Entry *menuStack[10]; //tab to save menu stack
uint8_t menuStackHistory[10]; //current position in menu stack
uint8_t menuStackIdx = 0;

OSD_Entry *currentMenu;
OSD_Entry *nextPage = NULL;

int8_t currentMenuPos = 0;
uint8_t currentMenuIdx = 0;
uint16_t *currentElement = NULL;

OSD_Entry menuAlarms[];
OSD_Entry menuOsdLayout[];
OSD_Entry menuOsdActiveElems[];
OSD_Entry menuOsdElemsPositions[];
OSD_Entry menuFeatures[];
OSD_Entry menuBlackbox[];
#ifdef LED_STRIP
OSD_Entry menuLedstrip[];
#endif // LED_STRIP
#if defined(VTX) || defined(USE_RTC6705)
OSD_Entry menu_vtx[];
#endif // VTX || USE_RTC6705
OSD_Entry menuImu[];
OSD_Entry menuPid[];
OSD_Entry menuRc[];
OSD_Entry menuRateExpo[];
OSD_Entry menuMisc[];

OSD_Entry menuMain[] =
{
    {"----MAIN MENU----", OME_Label, NULL, NULL},
    {"SCREEN LAYOUT", OME_Submenu, osdChangeScreen, &menuOsdLayout[0]},
    {"ALARMS", OME_Submenu, osdChangeScreen, &menuAlarms[0]},
    {"CFG. IMU", OME_Submenu, osdChangeScreen, &menuImu[0]},
    {"FEATURES", OME_Submenu, osdChangeScreen, &menuFeatures[0]},
    {"SAVE & EXIT", OME_OSD_Exit, osdExitMenu, (void*)1},
    {"EXIT", OME_OSD_Exit, osdExitMenu, (void*)0},
    {NULL,OME_END, NULL, NULL}
};

OSD_Entry menuFeatures[] =
{
    {"----- FEATURES -----", OME_Label, NULL, NULL},
    {"BLACKBOX", OME_Submenu, osdChangeScreen, &menuBlackbox[0]},
#ifdef LED_STRIP
    {"LED STRIP", OME_Submenu, osdChangeScreen, &menuLedstrip[0]},
#endif // LED_STRIP
#if defined(VTX) || defined(USE_RTC6705)
    {"VTX", OME_Submenu, osdChangeScreen, &menu_vtx[0]},
#endif // VTX || USE_RTC6705
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

OSD_UINT8_t entryBlackboxRateDenom = {&masterConfig.blackbox_rate_denom,1,32,1};

OSD_Entry menuBlackbox[] =
{
    {"--- BLACKBOX ---", OME_Label, NULL, NULL},
    {"ENABLED", OME_Bool, NULL, &featureBlackbox},
    {"RATE DENOM", OME_UINT8, NULL, &entryBlackboxRateDenom},
#ifdef USE_FLASHFS
    {"ERASE FLASH", OME_Submenu, osdEraseFlash, NULL},
#endif // USE_FLASHFS
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

#ifdef LED_STRIP
//local variable to keep color value
uint8_t ledColor;

static const char * const LED_COLOR_NAMES[] = {
    "   BLACK   ",
    "   WHITE   ",
    "   RED     ",
    "   ORANGE  ",
    "   YELLOW  ",
    " LIME GREEN",
    "   GREEN   ",
    " MINT GREEN",
    "   CYAN    ",
    " LIGHT BLUE",
    "   BLUE    ",
    "DARK VIOLET",
    "   MAGENTA ",
    "  DEEP PINK"
};

//find first led with color flag and restore color index
//after saving all leds with flags color will have color set in OSD
void getLedColor(void)
{
    for (int ledIndex = 0; ledIndex < LED_MAX_STRIP_LENGTH; ledIndex++) {
        const ledConfig_t *ledConfig = &masterConfig.ledConfigs[ledIndex];

        int fn = ledGetFunction(ledConfig);

        if (fn == LED_FUNCTION_COLOR) {
            ledColor = ledGetColor(ledConfig);
            break;
        }
    }
}

//udate all leds with flag color
void applyLedColor(void * ptr)
{
    UNUSED(ptr);
    for (int ledIndex = 0; ledIndex < LED_MAX_STRIP_LENGTH; ledIndex++) {
        ledConfig_t *ledConfig = &masterConfig.ledConfigs[ledIndex];
        if (ledGetFunction(ledConfig) == LED_FUNCTION_COLOR)
            *ledConfig = DEFINE_LED(ledGetX(ledConfig), ledGetY(ledConfig), ledColor, ledGetDirection(ledConfig), ledGetFunction(ledConfig), ledGetOverlay(ledConfig), 0);
    }
    updateLedStrip();
}

OSD_TAB_t entryLed = {&ledColor, 13, &LED_COLOR_NAMES[0]};

OSD_Entry menuLedstrip[] =
{
    {"--- LED TRIP ---", OME_Label, NULL, NULL},
    {"ENABLED", OME_Bool, NULL, &featureLedstrip},
    {"LED COLOR", OME_TAB, applyLedColor, &entryLed},
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};
#endif // LED_STRIP

#if defined(VTX) || defined(USE_RTC6705)
static const char * const vtxBandNames[] = {
    "BOSCAM A",
    "BOSCAM B",
    "BOSCAM E",
    "FATSHARK",
    "RACEBAND",
};

OSD_TAB_t entryVtxBand = {&vtxBand,4,&vtxBandNames[0]};
OSD_UINT8_t entryVtxChannel =  {&vtxChannel, 1, 8, 1};

#ifdef VTX
OSD_UINT8_t entryVtxMode =  {&masterConfig.vtx_mode, 0, 2, 1};
OSD_UINT16_t entryVtxMhz =  {&masterConfig.vtx_mhz, 5600, 5950, 1};
#endif // VTX

OSD_Entry menu_vtx[] =
{
    {"--- VTX ---", OME_Label, NULL, NULL},
    {"ENABLED", OME_Bool, NULL, &featureVtx},
#ifdef VTX
    {"VTX MODE", OME_UINT8, NULL, &entryVtxMode},
    {"VTX MHZ", OME_UINT16, NULL, &entryVtxMhz},
#endif // VTX
    {"BAND", OME_TAB, NULL, &entryVtxBand},
    {"CHANNEL", OME_UINT8, NULL, &entryVtxChannel},
#ifdef USE_RTC6705
    {"LOW POWER", OME_Bool, NULL, &masterConfig.vtx_power},
#endif // USE_RTC6705
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};
#endif // VTX || USE_RTC6705

OSD_UINT16_t entryMinThrottle = {&masterConfig.escAndServoConfig.minthrottle, 1020, 1300, 10};
OSD_UINT8_t entryGyroSoftLpfHz = {&masterConfig.gyro_soft_lpf_hz, 0, 255, 1};
OSD_UINT16_t entryDtermLpf = {&masterConfig.profile[0].pidProfile.dterm_lpf_hz, 0, 500, 5};
OSD_UINT16_t entryYawLpf = {&masterConfig.profile[0].pidProfile.yaw_lpf_hz, 0, 500, 5};
OSD_UINT16_t entryYawPLimit = {&masterConfig.profile[0].pidProfile.yaw_p_limit, 100, 500, 5};
OSD_UINT8_t entryVbatScale = {&masterConfig.batteryConfig.vbatscale, 1, 250, 1};
OSD_UINT8_t entryVbatMaxCell = {&masterConfig.batteryConfig.vbatmaxcellvoltage, 10, 50, 1};

OSD_Entry menuMisc[]=
{
    {"----- MISC -----", OME_Label, NULL, NULL},
    {"GYRO LOWPASS", OME_UINT8, NULL, &entryGyroSoftLpfHz},
    {"DTERM LPF", OME_UINT16, NULL, &entryDtermLpf},
    {"YAW LPF", OME_UINT16, NULL, &entryYawLpf},
    {"YAW P LIMIT", OME_UINT16, NULL, &entryYawPLimit},
    {"MINTHROTTLE", OME_UINT16, NULL, &entryMinThrottle},
    {"VBAT SCALE", OME_UINT8, NULL, &entryVbatScale},
    {"VBAT CELL MAX", OME_UINT8, NULL, &entryVbatMaxCell},
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

OSD_UINT8_t entryPidProfile = {&masterConfig.current_profile_index, 0, MAX_PROFILE_COUNT, 1};

OSD_Entry menuImu[] =
{
    {"-----CFG. IMU-----", OME_Label, NULL, NULL},
    {"PID", OME_Submenu, osdChangeScreen, &menuPid[0]},
    {"PID PROFILE", OME_UINT8, NULL, &entryPidProfile},
    {"RATE & RXPO", OME_Submenu, osdChangeScreen, &menuRateExpo[0]},
    {"RC PREVIEW", OME_Submenu, osdChangeScreen, &menuRc[0]},
    {"MISC", OME_Submenu, osdChangeScreen, &menuMisc[0]},
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

uint8_t tempPid[4][3];

static OSD_UINT8_t entryRollP = {&tempPid[PIDROLL][0], 10, 150, 1};
static OSD_UINT8_t entryRollI = {&tempPid[PIDROLL][1], 1, 150, 1};
static OSD_UINT8_t entryRollD = {&tempPid[PIDROLL][2], 0, 150, 1};

static OSD_UINT8_t entryPitchP = {&tempPid[PIDPITCH][0], 10, 150, 1};
static OSD_UINT8_t entryPitchI = {&tempPid[PIDPITCH][1], 1, 150, 1};
static OSD_UINT8_t entryPitchD = {&tempPid[PIDPITCH][2], 0, 150, 1};

static OSD_UINT8_t entryYawP = {&tempPid[PIDYAW][0], 10, 150, 1};
static OSD_UINT8_t entryYawI = {&tempPid[PIDYAW][1], 1, 150, 1};
static OSD_UINT8_t entryYawD = {&tempPid[PIDYAW][2], 0, 150, 1};

OSD_Entry menuPid[] =
{
    {"------- PID -------", OME_Label, NULL, NULL},
    {"ROLL P", OME_UINT8, NULL, &entryRollP},
    {"ROLL I", OME_UINT8, NULL, &entryRollI},
    {"ROLL D", OME_UINT8, NULL, &entryRollD},

    {"PITCH P", OME_UINT8, NULL, &entryPitchP},
    {"PITCH I", OME_UINT8, NULL, &entryPitchI},
    {"PITCH D", OME_UINT8, NULL, &entryPitchD},

    {"YAW P", OME_UINT8, NULL, &entryYawP},
    {"YAW I", OME_UINT8, NULL, &entryYawI},
    {"YAW D", OME_UINT8, NULL, &entryYawD},

    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

controlRateConfig_t rateProfile;

static OSD_FLOAT_t entryRollRate = {&rateProfile.rates[0], 0, 250, 1, 10};
static OSD_FLOAT_t entryPitchRate = {&rateProfile.rates[1], 0, 250, 1, 10};
static OSD_FLOAT_t entryYawRate = {&rateProfile.rates[2], 0, 250, 1, 10};
static OSD_FLOAT_t entryRcRate = {&rateProfile.rcRate8, 0, 200, 1, 10};
static OSD_FLOAT_t entryRcExpo = {&rateProfile.rcExpo8, 0, 100, 1, 10};
static OSD_FLOAT_t entryRcExpoYaw = {&rateProfile.rcYawExpo8, 0, 100, 1, 10};
static OSD_FLOAT_t extryTpaEntry = {&rateProfile.dynThrPID, 0, 70, 1, 10};
static OSD_UINT16_t entryTpaBreak = {&rateProfile.tpa_breakpoint, 1100, 1800, 10};
static OSD_FLOAT_t entryPSetpoint = {&masterConfig.profile[0].pidProfile.ptermSRateWeight, 0, 100, 1, 10};
static OSD_FLOAT_t entryDSetpoint = {&masterConfig.profile[0].pidProfile.dtermSetpointWeight, 0, 255, 1, 10};

OSD_Entry menuRateExpo[] =
{
    {"----RATE & EXPO----", OME_Label, NULL, NULL},
    {"ROLL RATE", OME_FLOAT, NULL, &entryRollRate},
    {"PITCH RATE", OME_FLOAT, NULL, &entryPitchRate},
    {"YAW RATE", OME_FLOAT, NULL, &entryYawRate},
    {"RC RATE", OME_FLOAT, NULL, &entryRcRate},
    {"RC EXPO", OME_FLOAT, NULL, &entryRcExpo},
    {"RC YAW EXPO", OME_FLOAT, NULL, &entryRcExpoYaw},
    {"THR. PID ATT.", OME_FLOAT, NULL, &extryTpaEntry},
    {"TPA BREAKPOINT", OME_UINT16, NULL, &entryTpaBreak},
    {"PTERM SRATE RATIO", OME_FLOAT, NULL, &entryPSetpoint},
    {"D SETPOINT", OME_FLOAT, NULL, &entryDSetpoint},
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

static OSD_INT16_t entryRcRoll = {&rcData[ROLL], 1, 2500, 0};
static OSD_INT16_t entryRcPitch = {&rcData[PITCH], 1, 2500, 0};
static OSD_INT16_t entryRcThrottle = {&rcData[THROTTLE], 1, 2500, 0};
static OSD_INT16_t entryRcYaw = {&rcData[YAW], 1, 2500, 0};
static OSD_INT16_t entryRcAux1 = {&rcData[AUX1], 1, 2500, 0};
static OSD_INT16_t entryRcAux2 = {&rcData[AUX2], 1, 2500, 0};
static OSD_INT16_t entryRcAux3 = {&rcData[AUX3], 1, 2500, 0};
static OSD_INT16_t entryRcAux4 = {&rcData[AUX4], 1, 2500, 0};

OSD_Entry menuRc[] =
{
    {"---- RC PREVIEW ----", OME_Label, NULL, NULL},
    {"ROLL", OME_INT16, NULL, &entryRcRoll},
    {"PITCH", OME_INT16, NULL, &entryRcPitch},
    {"THROTTLE", OME_INT16, NULL, &entryRcThrottle},
    {"YAW", OME_INT16, NULL, &entryRcYaw},
    {"AUX1", OME_INT16, NULL, &entryRcAux1},
    {"AUX2", OME_INT16, NULL, &entryRcAux2},
    {"AUX3", OME_INT16, NULL, &entryRcAux3},
    {"AUX4", OME_INT16, NULL, &entryRcAux4},
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

OSD_Entry menuOsdLayout[] =
{
    {"---SCREEN LAYOUT---", OME_Label, NULL, NULL},
    {"ACTIVE ELEM.", OME_Submenu, osdChangeScreen, &menuOsdActiveElems[0]},
    {"POSITION", OME_Submenu, osdChangeScreen, &menuOsdElemsPositions[0]},
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

OSD_UINT8_t entryAlarmRssi = {&OSD_cfg.rssi_alarm, 5, 90, 5};
OSD_UINT16_t entryAlarmCapacity = {&OSD_cfg.cap_alarm, 50, 30000, 50};
OSD_UINT16_t enryAlarmFlyTime = {&OSD_cfg.time_alarm, 1, 200, 1};
OSD_UINT16_t entryAlarmAltitude = {&OSD_cfg.alt_alarm, 1, 200, 1};

OSD_Entry menuAlarms[] =
{
    {"------ ALARMS ------", OME_Label, NULL, NULL},
    {"RSSI", OME_UINT8, NULL, &entryAlarmRssi},
    {"MAIN BATT.", OME_UINT16, NULL, &entryAlarmCapacity},
    {"FLY TIME", OME_UINT16, NULL, &enryAlarmFlyTime},
    {"MAX ALTITUDE", OME_UINT16, NULL, &entryAlarmAltitude},
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

OSD_Entry menuOsdElemsPositions[] =
{
    {"---POSITION---", OME_Label, NULL, NULL},
    {"RSSI", OME_POS, osdEditElement, &masterConfig.osdProfile.item_pos[OSD_RSSI_VALUE]},
    {"MAIN BATTERY", OME_POS, osdEditElement, &masterConfig.osdProfile.item_pos[OSD_MAIN_BATT_VOLTAGE]},
    {"UPTIME", OME_POS, osdEditElement, &masterConfig.osdProfile.item_pos[OSD_ONTIME]},
    {"FLY TIME", OME_POS, osdEditElement, &masterConfig.osdProfile.item_pos[OSD_FLYTIME]},
    {"FLY MODE", OME_POS, osdEditElement, &masterConfig.osdProfile.item_pos[OSD_FLYMODE]},
    {"NAME", OME_POS, osdEditElement, &masterConfig.osdProfile.item_pos[OSD_CRAFT_NAME]},
    {"THROTTLE POS", OME_POS, osdEditElement, &masterConfig.osdProfile.item_pos[OSD_THROTTLE_POS]},
#ifdef VTX
    {"VTX CHAN", OME_POS, osdEditElement, &masterConfig.osdProfile.item_pos[OSD_VTX_CHANNEL]},
#endif // VTX
    {"CURRENT (A)", OME_POS, osdEditElement, &masterConfig.osdProfile.item_pos[OSD_CURRENT_DRAW]},
    {"USED MAH", OME_POS, osdEditElement, &masterConfig.osdProfile.item_pos[OSD_MAH_DRAWN]},
#ifdef GPS
    {"GPS SPEED", OME_POS, osdEditElement, &masterConfig.osdProfile.item_pos[OSD_GPS_SPEED]},
    {"GPS SATS.", OME_POS, osdEditElement, &masterConfig.osdProfile.item_pos[OSD_GPS_SATS]},
#endif // GPS
    {"ALTITUDE", OME_POS, osdEditElement, &masterConfig.osdProfile.item_pos[OSD_ALTITUDE]},
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

OSD_Entry menuOsdActiveElems[] =
{
    {" --ACTIV ELEM.-- ", OME_Label, NULL, NULL},
    {"RSSI", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_RSSI_VALUE]},
    {"MAIN BATTERY", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_MAIN_BATT_VOLTAGE]},
    {"HORIZON", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_ARTIFICIAL_HORIZON]},
    {"HORIZON SIDEBARS", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_HORIZON_SIDEBARS]},
    {"UPTIME", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_ONTIME]},
    {"FLY TIME", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_FLYTIME]},
    {"FLY MODE", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_FLYMODE]},
    {"NAME", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_CRAFT_NAME]},
    {"THROTTLE POS", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_THROTTLE_POS]},
#ifdef VTX
    {"VTX CHAN", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_VTX_CHANNEL]},
#endif // VTX
    {"CURRENT (A)", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_CURRENT_DRAW]},
    {"USED MAH", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_MAH_DRAWN]},
#ifdef GPS
    {"GPS SPEED", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_GPS_SPEED]},
    {"GPS SATS.", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_GPS_SATS]},
#endif // GPS
    {"ALTITUDE", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_ALTITUDE]},
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

void resetOsdConfig(osd_profile_t *osdProfile)
{
    osdProfile->item_pos[OSD_RSSI_VALUE] = OSD_POS(22, 0) | VISIBLE_FLAG;
    osdProfile->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(12, 0) | VISIBLE_FLAG;
    osdProfile->item_pos[OSD_ARTIFICIAL_HORIZON] = OSD_POS(8, 6) | VISIBLE_FLAG;
    osdProfile->item_pos[OSD_HORIZON_SIDEBARS] = OSD_POS(8, 6) | VISIBLE_FLAG;
    osdProfile->item_pos[OSD_ONTIME] = OSD_POS(22, 11) | VISIBLE_FLAG;
    osdProfile->item_pos[OSD_FLYTIME] = OSD_POS(22, 12) | VISIBLE_FLAG;
    osdProfile->item_pos[OSD_FLYMODE] = OSD_POS(12, 11) | VISIBLE_FLAG;
    osdProfile->item_pos[OSD_CRAFT_NAME] = OSD_POS(12, 12);
    osdProfile->item_pos[OSD_THROTTLE_POS] = OSD_POS(1, 4);
    osdProfile->item_pos[OSD_VTX_CHANNEL] = OSD_POS(8, 6);
    osdProfile->item_pos[OSD_CURRENT_DRAW] = OSD_POS(1, 3);
    osdProfile->item_pos[OSD_MAH_DRAWN] = OSD_POS(15, 3);
    osdProfile->item_pos[OSD_GPS_SPEED] = OSD_POS(2, 2);
    osdProfile->item_pos[OSD_GPS_SATS] = OSD_POS(2, 12);
    osdProfile->item_pos[OSD_ALTITUDE] = OSD_POS(1, 5);

    osdProfile->rssi_alarm = 20;
    osdProfile->cap_alarm = 2200;
    osdProfile->time_alarm = 10; // in minutes
    osdProfile->alt_alarm = 100; // meters or feet depend on configuration

    osdProfile->video_system = 0;
}

void osdInit(void)
{
    char x, string_buffer[30];

    armState = ARMING_FLAG(ARMED);

    max7456Init(masterConfig.osdProfile.video_system);

    max7456ClearScreen();

    // display logo and help
    x = 160;
    for (int i = 1; i < 5; i++) {
        for (int j = 3; j < 27; j++) {
            if (x != 255)
                max7456WriteChar(j, i, x++);
        }
    }

    sprintf(string_buffer, "BF VERSION: %s", FC_VERSION_STRING);
    max7456Write(5, 6, string_buffer);
    max7456Write(7, 7, "MENU: THRT MID");
    max7456Write(13, 8, "YAW RIGHT");
    max7456Write(13, 9, "PITCH UP");
    max7456RefreshAll();

    refreshTimeout = 4 * REFRESH_1S;
}

void osdUpdateAlarms(void)
{
    int32_t alt = BaroAlt / 100;

    statRssi = rssi * 100 / 1024;

    if (statRssi < OSD_cfg.rssi_alarm)
        OSD_cfg.item_pos[OSD_RSSI_VALUE] |= BLINK_FLAG;
    else
        OSD_cfg.item_pos[OSD_RSSI_VALUE] &= ~BLINK_FLAG;

    if (vbat <= (batteryWarningVoltage - 1))
        OSD_cfg.item_pos[OSD_MAIN_BATT_VOLTAGE] |= BLINK_FLAG;
    else
        OSD_cfg.item_pos[OSD_MAIN_BATT_VOLTAGE] &= ~BLINK_FLAG;

    if (STATE(GPS_FIX) == 0)
        OSD_cfg.item_pos[OSD_GPS_SATS] |= BLINK_FLAG;
    else
        OSD_cfg.item_pos[OSD_GPS_SATS] &= ~BLINK_FLAG;

    if (flyTime / 60 >= OSD_cfg.time_alarm && ARMING_FLAG(ARMED))
        OSD_cfg.item_pos[OSD_FLYTIME] |= BLINK_FLAG;
    else
        OSD_cfg.item_pos[OSD_FLYTIME] &= ~BLINK_FLAG;

    if (mAhDrawn >= OSD_cfg.cap_alarm)
        OSD_cfg.item_pos[OSD_MAH_DRAWN] |= BLINK_FLAG;
    else
        OSD_cfg.item_pos[OSD_MAH_DRAWN] &= ~BLINK_FLAG;

    if (masterConfig.osdProfile.units == OSD_UNIT_IMPERIAL) {
        alt = (alt * 328) / 100; // Convert to feet
    }

    if (alt >= OSD_cfg.alt_alarm)
        OSD_cfg.item_pos[OSD_ALTITUDE] |= BLINK_FLAG;
    else
        OSD_cfg.item_pos[OSD_ALTITUDE] &= ~BLINK_FLAG;
}

void osdResetAlarms(void)
{
    OSD_cfg.item_pos[OSD_RSSI_VALUE] &= ~BLINK_FLAG;
    OSD_cfg.item_pos[OSD_MAIN_BATT_VOLTAGE] &= ~BLINK_FLAG;
    OSD_cfg.item_pos[OSD_GPS_SATS] &= ~BLINK_FLAG;
    OSD_cfg.item_pos[OSD_FLYTIME] &= ~BLINK_FLAG;
    OSD_cfg.item_pos[OSD_MAH_DRAWN] &= ~BLINK_FLAG;
}

uint8_t osdHandleKey(uint8_t key)
{
    uint8_t res = BUTTON_TIME;
    OSD_Entry *p;

    if (!currentMenu)
        return res;

    if (key == KEY_ESC) {
        osdMenuBack();
        return BUTTON_PAUSE;
    }

    if (key == KEY_DOWN) {
        if (currentMenuPos < currentMenuIdx)
            currentMenuPos++;
        else {
            if (nextPage) // we have more pages
            {
                max7456ClearScreen();
                p = nextPage;
                nextPage = currentMenu;
                currentMenu = (OSD_Entry *)p;
                currentMenuPos = 0;
                osdUpdateMaxRows();
            }
            currentMenuPos = 0;
        }
    }

    if (key == KEY_UP) {
        currentMenuPos--;

        if ((currentMenu + currentMenuPos)->type == OME_Label && currentMenuPos > 0)
            currentMenuPos--;

        if (currentMenuPos == -1 || (currentMenu + currentMenuPos)->type == OME_Label) {
            if (nextPage) {
                max7456ClearScreen();
                p = nextPage;
                nextPage = currentMenu;
                currentMenu = (OSD_Entry *)p;
                currentMenuPos = 0;
                osdUpdateMaxRows();
            }
            currentMenuPos = currentMenuIdx;
        }
    }

    if (key == KEY_DOWN || key == KEY_UP)
        return res;

    p = currentMenu + currentMenuPos;

    switch (p->type) {
        case OME_POS:
            if (key == KEY_RIGHT) {
                uint32_t address = (uint32_t)p->data;
                uint16_t *val;

                val = (uint16_t *)address;
                if (!(*val & VISIBLE_FLAG)) // no submenu for hidden elements
                    break;
            }
        case OME_Submenu:
        case OME_OSD_Exit:
            if (p->func && key == KEY_RIGHT) {
                p->func(p->data);
                res = BUTTON_PAUSE;
            }
            break;
        case OME_Back:
            osdMenuBack();
            res = BUTTON_PAUSE;
            break;
        case OME_Bool:
            if (p->data) {
                uint8_t *val = p->data;
                if (key == KEY_RIGHT)
                    *val = 1;
                else
                    *val = 0;
            }
            break;
        case OME_VISIBLE:
            if (p->data) {
                uint32_t address = (uint32_t)p->data;
                uint16_t *val;

                val = (uint16_t *)address;

                if (key == KEY_RIGHT)
                    *val |= VISIBLE_FLAG;
                else
                    *val %= ~VISIBLE_FLAG;
            }
            break;
        case OME_UINT8:
        case OME_FLOAT:
            if (p->data) {
                OSD_UINT8_t *ptr = p->data;
                if (key == KEY_RIGHT) {
                    if (*ptr->val < ptr->max)
                        *ptr->val += ptr->step;
                }
                else {
                    if (*ptr->val > ptr->min)
                        *ptr->val -= ptr->step;
                }
            }
            break;
        case OME_TAB:
            if (p->type == OME_TAB) {
                OSD_TAB_t *ptr = p->data;

                if (key == KEY_RIGHT) {
                    if (*ptr->val < ptr->max)
                        *ptr->val += 1;
                }
                else {
                    if (*ptr->val > 0)
                        *ptr->val -= 1;
                }
                if (p->func)
                    p->func(p->data);
            }
            break;
        case OME_INT8:
            if (p->data) {
                OSD_INT8_t *ptr = p->data;
                if (key == KEY_RIGHT) {
                    if (*ptr->val < ptr->max)
                        *ptr->val += ptr->step;
                }
                else {
                    if (*ptr->val > ptr->min)
                        *ptr->val -= ptr->step;
                }
            }
            break;
        case OME_UINT16:
            if (p->data) {
                OSD_UINT16_t *ptr = p->data;
                if (key == KEY_RIGHT) {
                    if (*ptr->val < ptr->max)
                        *ptr->val += ptr->step;
                }
                else {
                    if (*ptr->val > ptr->min)
                        *ptr->val -= ptr->step;
                }
            }
            break;
        case OME_INT16:
            if (p->data) {
                OSD_INT16_t *ptr = p->data;
                if (key == KEY_RIGHT) {
                    if (*ptr->val < ptr->max)
                        *ptr->val += ptr->step;
                }
                else {
                    if (*ptr->val > ptr->min)
                        *ptr->val -= ptr->step;
                }
            }
            break;
        case OME_Label:
        case OME_END:
            break;
    }
    return res;
}

void osdUpdateMaxRows(void)
{
    OSD_Entry *ptr;

    currentMenuIdx = 0;
    for (ptr = currentMenu; ptr->type != OME_END; ptr++)
        currentMenuIdx++;

    if (currentMenuIdx > MAX_MENU_ITEMS)
        currentMenuIdx = MAX_MENU_ITEMS;

    currentMenuIdx--;
}

void osdMenuBack(void)
{
    uint8_t i;

    // becasue pids and rates meybe stored in profiles we need some thicks to manipulate it
    // hack to save pid profile
    if (currentMenu == &menuPid[0]) {
        for (i = 0; i < 3; i++) {
            curr_profile.pidProfile.P8[i] = tempPid[i][0];
            curr_profile.pidProfile.I8[i] = tempPid[i][1];
            curr_profile.pidProfile.D8[i] = tempPid[i][2];
        }

        curr_profile.pidProfile.P8[PIDLEVEL] = tempPid[3][0];
        curr_profile.pidProfile.I8[PIDLEVEL] = tempPid[3][1];
        curr_profile.pidProfile.D8[PIDLEVEL] = tempPid[3][2];
    }

    // hack - save rate config for current profile
    if (currentMenu == &menuRateExpo[0])
        memcpy(&masterConfig.profile[masterConfig.current_profile_index].controlRateProfile[masterConfig.profile[masterConfig.current_profile_index].activeRateProfile], &rateProfile, sizeof(controlRateConfig_t));

    if (menuStackIdx) {
        max7456ClearScreen();
        menuStackIdx--;
        nextPage = NULL;
        currentMenu = menuStack[menuStackIdx];
        currentMenuPos = menuStackHistory[menuStackIdx];

        osdUpdateMaxRows();
    }
    else
        osdOpenMenu();
}

void simple_ftoa(int32_t value, char *floatString)
{
    uint8_t k;
    // np. 3450

    itoa(100000 + value, floatString, 10); // Create string from abs of integer value

    // 103450

    floatString[0] = floatString[1];
    floatString[1] = floatString[2];
    floatString[2] = '.';

    // 03.450
    // usuwam koncowe zera i kropke
    for (k = 5; k > 1; k--)
        if (floatString[k] == '0' || floatString[k] == '.')
            floatString[k] = 0;
        else
            break;

    // oraz zero wiodonce
    if (floatString[0] == '0')
        floatString[0] = ' ';
}

void osdDrawMenu(void)
{
    uint8_t i = 0;
    OSD_Entry *p;
    char buff[10];
    uint8_t top = (osdRows - currentMenuIdx) / 2 - 1;
    if (!currentMenu)
        return;

    if ((currentMenu + currentMenuPos)->type == OME_Label) // skip label
        currentMenuPos++;

    for (p = currentMenu; p->type != OME_END; p++) {
        if (currentMenuPos == i)
            max7456Write(LEFT_MENU_COLUMN, i + top, " >");
        else
            max7456Write(LEFT_MENU_COLUMN, i + top, "  ");
        max7456Write(LEFT_MENU_COLUMN + 2, i + top, p->text);

        switch (p->type) {
            case OME_POS: {
                uint32_t address = (uint32_t)p->data;
                uint16_t *val;

                val = (uint16_t *)address;
                if (!(*val & VISIBLE_FLAG))
                    break;
            }
            case OME_Submenu:
                max7456Write(RIGHT_MENU_COLUMN, i + top, ">");
                break;
            case OME_Bool:
                if (p->data) {
                    if (*((uint8_t *)(p->data)))
                        max7456Write(RIGHT_MENU_COLUMN, i + top, "YES");
                    else
                        max7456Write(RIGHT_MENU_COLUMN, i + top, "NO ");
                }
                break;
            case OME_TAB: {
                OSD_TAB_t *ptr = p->data;
                max7456Write(RIGHT_MENU_COLUMN - 5, i + top, (char *)ptr->names[*ptr->val]);
                break;
            }
            case OME_VISIBLE:
                if (p->data) {
                    uint32_t address = (uint32_t)p->data;
                    uint16_t *val;

                    val = (uint16_t *)address;

                    if (VISIBLE(*val))
                        max7456Write(RIGHT_MENU_COLUMN, i + top, "YES");
                    else
                        max7456Write(RIGHT_MENU_COLUMN, i + top, "NO ");
                }
                break;
            case OME_UINT8:
                if (p->data) {
                    OSD_UINT8_t *ptr = p->data;
                    itoa(*ptr->val, buff, 10);
                    max7456Write(RIGHT_MENU_COLUMN, i + top, "     ");
                    max7456Write(RIGHT_MENU_COLUMN, i + top, buff);
                }
                break;
            case OME_INT8:
                if (p->data) {
                    OSD_INT8_t *ptr = p->data;
                    itoa(*ptr->val, buff, 10);
                    max7456Write(RIGHT_MENU_COLUMN, i + top, "     ");
                    max7456Write(RIGHT_MENU_COLUMN, i + top, buff);
                }
                break;
            case OME_UINT16:
                if (p->data) {
                    OSD_UINT16_t *ptr = p->data;
                    itoa(*ptr->val, buff, 10);
                    max7456Write(RIGHT_MENU_COLUMN, i + top, "     ");
                    max7456Write(RIGHT_MENU_COLUMN, i + top, buff);
                }
                break;
            case OME_INT16:
                if (p->data) {
                    OSD_UINT16_t *ptr = p->data;
                    itoa(*ptr->val, buff, 10);
                    max7456Write(RIGHT_MENU_COLUMN, i + top, "     ");
                    max7456Write(RIGHT_MENU_COLUMN, i + top, buff);
                }
                break;
            case OME_FLOAT:
                if (p->data) {
                    OSD_FLOAT_t *ptr = p->data;
                    simple_ftoa(*ptr->val * ptr->multipler, buff);
                    max7456Write(RIGHT_MENU_COLUMN - 1, i + top, "      ");
                    max7456Write(RIGHT_MENU_COLUMN - 1, i + top, buff);
                }
                break;
            case OME_OSD_Exit:
            case OME_Label:
            case OME_END:
            case OME_Back:
                break;
        }
        i++;

        if (i == MAX_MENU_ITEMS) // max per page
        {
            nextPage = currentMenu + i;
            if (nextPage->type == OME_END)
                nextPage = NULL;
            break;
        }
    }
}

void osdResetStats(void)
{
    stats.max_current = 0;
    stats.max_speed = 0;
    stats.min_voltage = 500;
    stats.max_current = 0;
    stats.min_rssi = 99;
}

void osdUpdateStats(void)
{
    int16_t value;

    value = GPS_speed * 36 / 1000;
    if (stats.max_speed < value)
        stats.max_speed = value;

    if (stats.min_voltage > vbat)
        stats.min_voltage = vbat;

    value = amperage / 100;
    if (stats.max_current < value)
        stats.max_current = value;

    if (stats.min_rssi > statRssi)
        stats.min_rssi = statRssi;
}

void osdShowStats(void)
{
    uint8_t top = 2;
    char buff[10];

    max7456ClearScreen();
    max7456Write(2, top++, "  --- STATS ---");

    if (STATE(GPS_FIX)) {
        max7456Write(2, top, "MAX SPEED        :");
        itoa(stats.max_speed, buff, 10);
        max7456Write(22, top++, buff);
    }

    max7456Write(2, top, "MIN BATTERY      :");
    sprintf(buff, "%d.%1dV", stats.min_voltage / 10, stats.min_voltage % 10);
    max7456Write(22, top++, buff);

    max7456Write(2, top, "MIN RSSI         :");
    itoa(stats.min_rssi, buff, 10);
    strcat(buff, "%");
    max7456Write(22, top++, buff);

    if (feature(FEATURE_CURRENT_METER)) {
        max7456Write(2, top, "MAX CURRENT      :");
        itoa(stats.max_current, buff, 10);
        strcat(buff, "A");
        max7456Write(22, top++, buff);

        max7456Write(2, top, "USED MAH         :");
        itoa(mAhDrawn, buff, 10);
        strcat(buff, "\x07");
        max7456Write(22, top++, buff);
    }
    refreshTimeout = 60 * REFRESH_1S;
}

// called when motors armed
void osdArmMotors(void)
{
    max7456ClearScreen();
    max7456Write(12, 7, "ARMED");
    refreshTimeout = REFRESH_1S / 2;
    osdResetStats();
}

void updateOsd(void)
{
    static uint32_t counter;
#ifdef MAX7456_DMA_CHANNEL_TX
    // don't touch buffers if DMA transaction is in progress
    if (max7456DmaInProgres())
        return;
#endif // MAX7456_DMA_CHANNEL_TX

    // redraw values in buffer
    if (counter++ % 5 == 0)
        osdUpdate(0);
    else // rest of time redraw screen 10 chars per idle to don't lock the main idle
        max7456DrawScreen();

    // do not allow ARM if we are in menu
    if (inMenu)
        DISABLE_ARMING_FLAG(OK_TO_ARM);
}

void osdUpdate(uint8_t guiKey)
{
    static uint8_t rcDelay = BUTTON_TIME;
    static uint8_t last_sec = 0;
    uint8_t key = 0, sec;

    // detect enter to menu
    if (IS_MID(THROTTLE) && IS_HI(YAW) && IS_HI(PITCH) && !ARMING_FLAG(ARMED))
        osdOpenMenu();

    // detect arm/disarm
    if (armState != ARMING_FLAG(ARMED)) {
        if (ARMING_FLAG(ARMED))
            osdArmMotors(); // reset statistic etc
        else
            osdShowStats(); // show statistic

        armState = ARMING_FLAG(ARMED);
    }

    osdUpdateStats();

    sec = millis() / 1000;

    if (ARMING_FLAG(ARMED) && sec != last_sec) {
        flyTime++;
        last_sec = sec;
    }

    if (refreshTimeout) {
        if (IS_HI(THROTTLE) || IS_HI(PITCH)) // hide statistics
            refreshTimeout = 1;
        refreshTimeout--;
        if (!refreshTimeout)
            max7456ClearScreen();
        return;
    }

    blinkState = (millis() / 200) % 2;

    if (inMenu) {
        if (rcDelay) {
            rcDelay--;
        }
        else if (IS_HI(PITCH)) {
            key = KEY_UP;
            rcDelay = BUTTON_TIME;
        }
        else if (IS_LO(PITCH)) {
            key = KEY_DOWN;
            rcDelay = BUTTON_TIME;
        }
        else if (IS_LO(ROLL)) {
            key = KEY_LEFT;
            rcDelay = BUTTON_TIME;
        }
        else if (IS_HI(ROLL)) {
            key = KEY_RIGHT;
            rcDelay = BUTTON_TIME;
        }
        else if ((IS_HI(YAW) || IS_LO(YAW)) && currentMenu != menuRc) // this menu is used to check transmitter signals so can exit using YAW
        {
            key = KEY_ESC;
            rcDelay = BUTTON_TIME;
        }

        if (guiKey)
            key = guiKey;

        if (key && !currentElement) {
            rcDelay = osdHandleKey(key);
            return;
        }
        if (currentElement) // edit position of element
        {
            if (key) {
                if (key == KEY_ESC) {
                    // exit
                    osdMenuBack();
                    rcDelay = BUTTON_PAUSE;
                    *currentElement &= ~BLINK_FLAG;
                    currentElement = NULL;
                    return;
                }
                else {
                    uint8_t x, y;
                    x = OSD_X(*currentElement);
                    y = OSD_Y(*currentElement);
                    switch (key) {
                        case KEY_UP:
                            y--;
                            break;
                        case KEY_DOWN:
                            y++;
                            break;
                        case KEY_RIGHT:
                            x++;
                            break;
                        case KEY_LEFT:
                            x--;
                            break;
                    }

                    *currentElement &= 0xFC00;
                    *currentElement |= OSD_POS(x, y);
                    max7456ClearScreen();
                }
            }
            osdDrawElements();
        }
        else
            osdDrawMenu();
    }
    else {
        osdUpdateAlarms();
        osdDrawElements();
    }
}

void osdChangeScreen(void *ptr)
{
    uint8_t i;
    if (ptr) {
        max7456ClearScreen();
        // hack - save profile to temp
        if (ptr == &menuPid[0]) {
            for (i = 0; i < 3; i++) {
                tempPid[i][0] = curr_profile.pidProfile.P8[i];
                tempPid[i][1] = curr_profile.pidProfile.I8[i];
                tempPid[i][2] = curr_profile.pidProfile.D8[i];
            }
            tempPid[3][0] = curr_profile.pidProfile.P8[PIDLEVEL];
            tempPid[3][1] = curr_profile.pidProfile.I8[PIDLEVEL];
            tempPid[3][2] = curr_profile.pidProfile.D8[PIDLEVEL];
        }

        if (ptr == &menuRateExpo[0])
            memcpy(&rateProfile, &masterConfig.profile[masterConfig.current_profile_index].controlRateProfile[masterConfig.profile[masterConfig.current_profile_index].activeRateProfile], sizeof(controlRateConfig_t));

        menuStack[menuStackIdx] = currentMenu;
        menuStackHistory[menuStackIdx] = currentMenuPos;
        menuStackIdx++;

        currentMenu = (OSD_Entry *)ptr;
        currentMenuPos = 0;
        osdUpdateMaxRows();
    }
}

#ifdef USE_FLASHFS
void osdEraseFlash(void *ptr)
{
    UNUSED(ptr);

    max7456ClearScreen();
    max7456Write(5, 3, "ERASING FLASH...");
    max7456RefreshAll();

    flashfsEraseCompletely();
    while (!flashfsIsReady()) {
        delay(100);
    }
    max7456ClearScreen();
    max7456RefreshAll();
}
#endif // USE_FLASHFS

void osdEditElement(void *ptr)
{
    uint32_t address = (uint32_t)ptr;

    // zsave position on menu stack
    menuStack[menuStackIdx] = currentMenu;
    menuStackHistory[menuStackIdx] = currentMenuPos;
    menuStackIdx++;

    currentElement = (uint16_t *)address;

    *currentElement |= BLINK_FLAG;
    max7456ClearScreen();
}

void osdExitMenu(void *ptr)
{
    max7456ClearScreen();
    max7456Write(5, 3, "RESTARTING IMU...");
    max7456RefreshAll();
    stopMotors();
    stopPwmAllMotors();
    delay(200);

    if (ptr) {
        // save local variables to configuration
        if (featureBlackbox)
            featureSet(FEATURE_BLACKBOX);
        else
            featureClear(FEATURE_BLACKBOX);

        if (featureLedstrip)
            featureSet(FEATURE_LED_STRIP);
        else
            featureClear(FEATURE_LED_STRIP);
#if defined(VTX) || defined(USE_RTC6705)
        if (featureVtx)
            featureSet(FEATURE_VTX);
        else
            featureClear(FEATURE_VTX);
#endif // VTX || USE_RTC6705

#ifdef VTX
        masterConfig.vtxBand = vtxBand;
        masterConfig.vtx_channel = vtxChannel - 1;
#endif // VTX

#ifdef USE_RTC6705
        masterConfig.vtx_channel = vtxBand * 8 + vtxChannel - 1;
#endif // USE_RTC6705

        saveConfigAndNotify();
    }

    systemReset();
}

void osdOpenMenu(void)
{
    if (inMenu)
        return;

    if (feature(FEATURE_LED_STRIP))
        featureLedstrip = 1;

    if (feature(FEATURE_BLACKBOX))
        featureBlackbox = 1;

#if defined(VTX) || defined(USE_RTC6705)
    if (feature(FEATURE_VTX))
        featureVtx = 1;
#endif // VTX || USE_RTC6705

#ifdef VTX
    vtxBand = masterConfig.vtxBand;
    vtxChannel = masterConfig.vtx_channel + 1;
#endif // VTX

#ifdef USE_RTC6705
    vtxBand = masterConfig.vtx_channel / 8;
    vtxChannel = masterConfig.vtx_channel % 8 + 1;
#endif // USE_RTC6705

    osdRows = max7456GetRowsCount();
    inMenu = true;
    refreshTimeout = 0;
    max7456ClearScreen();
    currentMenu = &menuMain[0];
    osdResetAlarms();
    osdChangeScreen(currentMenu);
#ifdef LED_STRIP
    getLedColor();
#endif // LED_STRIP
}

void osdDrawElementPositioningHelp(void)
{
    max7456Write(OSD_X(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]), OSD_Y(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]), "---  HELP --- ");
    max7456Write(OSD_X(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]), OSD_Y(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]) + 1, "USE ROLL/PITCH");
    max7456Write(OSD_X(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]), OSD_Y(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]) + 2, "TO MOVE ELEM. ");
    max7456Write(OSD_X(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]), OSD_Y(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]) + 3, "              ");
    max7456Write(OSD_X(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]), OSD_Y(OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON]) + 4, "YAW - EXIT    ");
}

void osdDrawElements(void)
{
    max7456ClearScreen();

    if (currentElement)
        osdDrawElementPositioningHelp();
    else if (sensors(SENSOR_ACC) || inMenu)
        osdDrawSingleElement(OSD_ARTIFICIAL_HORIZON);

    osdDrawSingleElement(OSD_MAIN_BATT_VOLTAGE);
    osdDrawSingleElement(OSD_RSSI_VALUE);
    osdDrawSingleElement(OSD_FLYTIME);
    osdDrawSingleElement(OSD_ONTIME);
    osdDrawSingleElement(OSD_FLYMODE);
    osdDrawSingleElement(OSD_THROTTLE_POS);
    osdDrawSingleElement(OSD_VTX_CHANNEL);
    osdDrawSingleElement(OSD_CURRENT_DRAW);
    osdDrawSingleElement(OSD_MAH_DRAWN);
    osdDrawSingleElement(OSD_CRAFT_NAME);
    osdDrawSingleElement(OSD_ALTITUDE);

#ifdef GPS
    if (sensors(SENSOR_GPS) || inMenu) {
        osdDrawSingleElement(OSD_GPS_SATS);
        osdDrawSingleElement(OSD_GPS_SPEED);
    }
#endif // GPS
}

#define AH_MAX_PITCH 200 // Specify maximum AHI pitch value displayed. Default 200 = 20.0 degrees
#define AH_MAX_ROLL 400  // Specify maximum AHI roll value displayed. Default 400 = 40.0 degrees
#define AH_SIDEBAR_WIDTH_POS 7
#define AH_SIDEBAR_HEIGHT_POS 3
extern uint16_t rssi; // FIXME dependency on mw.c

void osdDrawSingleElement(uint8_t item)
{
    if (!VISIBLE(OSD_cfg.item_pos[item]) || BLINK(OSD_cfg.item_pos[item]))
        return;

    uint8_t elemPosX = OSD_X(OSD_cfg.item_pos[item]);
    uint8_t elemPosY = OSD_Y(OSD_cfg.item_pos[item]);
    char buff[32];

    switch(item) {
        case OSD_RSSI_VALUE:
        {
            uint16_t osdRssi = rssi * 100 / 1024; // change range
            if (osdRssi >= 100)
                osdRssi = 99;

            buff[0] = SYM_RSSI;
            sprintf(buff + 1, "%d", osdRssi);
            break;
        }

        case OSD_MAIN_BATT_VOLTAGE:
        {
            buff[0] = SYM_BATT_5;
            sprintf(buff + 1, "%d.%1dV", vbat / 10, vbat % 10);
            break;
        }

        case OSD_CURRENT_DRAW:
        {
            buff[0] = SYM_AMP;
            sprintf(buff + 1, "%d.%02d", abs(amperage) / 100, abs(amperage) % 100);
            break;
        }

        case OSD_MAH_DRAWN:
        {
            buff[0] = SYM_MAH;
            sprintf(buff + 1, "%d", mAhDrawn);
            break;
        }

#ifdef GPS
        case OSD_GPS_SATS:
        {
            buff[0] = 0x1f;
            sprintf(buff + 1, "%d", GPS_numSat);
            break;
        }

        case OSD_GPS_SPEED:
        {
            sprintf(buff, "%d", GPS_speed * 36 / 1000);
            break;
        }
#endif // GPS

        case OSD_ALTITUDE:
        {
            int32_t alt = BaroAlt; // Metre x 100
            char unitSym = 0xC;    // m

            if (!VISIBLE(OSD_cfg.item_pos[OSD_ALTITUDE]) || BLINK(OSD_cfg.item_pos[OSD_ALTITUDE]))
                return;

            if (masterConfig.osdProfile.units == OSD_UNIT_IMPERIAL) {
                alt = (alt * 328) / 100; // Convert to feet x 100
                unitSym = 0xF;           // ft
            }

            sprintf(buff, "%c%d.%01d%c", alt < 0 ? '-' : ' ', abs(alt / 100), abs((alt % 100) / 10), unitSym);
            break;
        }

        case OSD_ONTIME:
        {
            uint32_t seconds = micros() / 1000000;
            buff[0] = SYM_ON_M;
            sprintf(buff + 1, "%02d:%02d", seconds / 60, seconds % 60);
            break;
        }

        case OSD_FLYTIME:
        {
            buff[0] = SYM_FLY_M;
            sprintf(buff + 1, "%02d:%02d", flyTime / 60, flyTime % 60);
            break;
        }

        case OSD_FLYMODE:
        {
            char *p = "ACRO";

            if (isAirmodeActive())
                p = "AIR";

            if (FLIGHT_MODE(FAILSAFE_MODE))
                p = "!FS";
            else if (FLIGHT_MODE(ANGLE_MODE))
                p = "STAB";
            else if (FLIGHT_MODE(HORIZON_MODE))
                p = "HOR";

            max7456Write(elemPosX, elemPosY, p);
            return;
        }

        case OSD_CRAFT_NAME:
        {
            if (strlen(masterConfig.name) == 0)
                strcpy(buff, "CRAFT_NAME");
            else {
                for (uint8_t i = 0; i < MAX_NAME_LENGTH; i++) {
                    buff[i] = toupper((unsigned char)masterConfig.name[i]);
                    if (masterConfig.name[i] == 0)
                        break;
                }
            }

            break;
        }

        case OSD_THROTTLE_POS:
        {
            buff[0] = SYM_THR;
            buff[1] = SYM_THR1;
            sprintf(buff + 2, "%d", (constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN));
            break;
        }

#ifdef VTX
        case OSD_VTX_CHANNEL:
        {
            sprintf(buff, "CH:%d", current_vtx_channel % CHANNELS_PER_BAND + 1);
            break;
        }
#endif // VTX

        case OSD_ARTIFICIAL_HORIZON:
        {
            uint8_t *screenBuffer = max7456GetScreenBuffer();
            uint16_t position = 194;

            int rollAngle = attitude.values.roll;
            int pitchAngle = attitude.values.pitch;

            if (maxScreenSize == VIDEO_BUFFER_CHARS_PAL)
                position += 30;


            if (pitchAngle > AH_MAX_PITCH)
                pitchAngle = AH_MAX_PITCH;
            if (pitchAngle < -AH_MAX_PITCH)
                pitchAngle = -AH_MAX_PITCH;
            if (rollAngle > AH_MAX_ROLL)
                rollAngle = AH_MAX_ROLL;
            if (rollAngle < -AH_MAX_ROLL)
                rollAngle = -AH_MAX_ROLL;

            for (uint8_t x = 0; x <= 8; x++) {
                if (x == 4)
                    x = 5;
                int y = (rollAngle * (4 - x)) / 64;
                y -= pitchAngle / 8;
                y += 41;
                if (y >= 0 && y <= 81) {
                    uint16_t pos = position - 7 + LINE * (y / 9) + 3 - 4 * LINE + x;
                    screenBuffer[pos] = (SYM_AH_BAR9_0 + (y % 9));
                }
            }

            screenBuffer[position - 1] = (SYM_AH_CENTER_LINE);
            screenBuffer[position + 1] = (SYM_AH_CENTER_LINE_RIGHT);
            screenBuffer[position] = (SYM_AH_CENTER);

            osdDrawSingleElement(OSD_HORIZON_SIDEBARS);
            return;
        }

        case OSD_HORIZON_SIDEBARS:
        {
            uint8_t *screenBuffer = max7456GetScreenBuffer();
            uint16_t position = 194;

            if (maxScreenSize == VIDEO_BUFFER_CHARS_PAL)
                position += 30;

            // Draw AH sides
            int8_t hudwidth = AH_SIDEBAR_WIDTH_POS;
            int8_t hudheight = AH_SIDEBAR_HEIGHT_POS;
            for (int8_t x = -hudheight; x <= hudheight; x++) {
                screenBuffer[position - hudwidth + (x * LINE)] = (SYM_AH_DECORATION);
                screenBuffer[position + hudwidth + (x * LINE)] = (SYM_AH_DECORATION);
            }

            // AH level indicators
            screenBuffer[position - hudwidth + 1] = (SYM_AH_LEFT);
            screenBuffer[position + hudwidth - 1] = (SYM_AH_RIGHT);

            return;
        }

        default:
            return;
    }

    max7456Write(elemPosX, elemPosY, buff);
}
