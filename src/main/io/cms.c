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
 OSD-CMS separation by jflyper
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#include "build/version.h"

#ifdef CMS

#include "drivers/system.h"

#include "common/typeconversion.h"

#include "io/cms_types.h"
#include "io/canvas.h"

#include "io/flashfs.h"
#include "io/osd.h"
#include "io/display.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#include "build/debug.h"

// Configuration Menu System forwards

void cmsOpenMenu(void);
void cmsExitMenu(void *);
void cmsChangeScreen(void *);
void cmsMenuBack(void);
void cmsEraseFlash(void *);

screenFnVTable_t *pScreenFnVTable = NULL;

uint8_t cmsRows;
uint8_t cmsCols;
uint16_t cmsBuftime;
uint16_t cmsBufsize;
uint16_t cmsBatchsize;

#define CMS_UPDATE_INTERVAL 50 // msec

//#define MAX_MENU_ITEMS    (cmsGetRowsCount() - 2)
#define MAX_MENU_ITEMS    (cmsRows - 2)

// XXX LEFT_MENU_COLUMN and RIGHT_MENU_COLUMN must be adjusted
// dynamically depending on size of the active output device,
// or statically to accomodate sizes of all supported devices.
//
// Device characteristics
// OLED
//   21 cols x 8 rows
//     128x64 with 5x7 (6x8) : 21 cols x 8 rows
// MAX7456 (PAL)
//   30 cols x 16 rows
// MAX7456 (NTSC)
//   30 cols x 13 rows
// HoTT Telemetry Screen
//   21 cols x 8 rows
//

#define LEFT_MENU_COLUMN  1
#define RIGHT_MENU_COLUMN (cmsCols - 7)

bool cmsScreenCleared;
OSD_Entry *currentMenu;
OSD_Entry *nextPage = NULL;

int8_t currentCursorPos = 0;
int8_t lastCursorPos;
uint8_t currentMenuIdx = 0;
uint16_t *currentElement = NULL;

void cmsGetDevParam(uint8_t *pRows, uint8_t *pCols, uint16_t *pBuftime, uint16_t *pBufsize)
{
    pScreenFnVTable->getDevParam(pRows, pCols, pBuftime, pBufsize);
}

void cmsScreenClear(void)
{
    pScreenFnVTable->clear();
    cmsScreenCleared = true;
    lastCursorPos = -1;
}

void cmsScreenBegin(void)
{
    pScreenFnVTable->begin();
    pScreenFnVTable->clear();
}

void cmsScreenEnd(void)
{
    pScreenFnVTable->end();
}

int cmsScreenWrite(uint8_t x, uint8_t y, char *s)
{
    return pScreenFnVTable->write(x, y, s);
}

void cmsScreenHeartBeat(void)
{
    if (pScreenFnVTable->heartbeat)
        pScreenFnVTable->heartbeat();
}

void cmsScreenResync(void)
{
    if (pScreenFnVTable->resync)
        pScreenFnVTable->resync();

    pScreenFnVTable->getDevParam(&cmsRows, &cmsCols, &cmsBuftime, &cmsBufsize);
}

void cmsScreenInit(void)
{
#ifdef OSD
pScreenFnVTable = osdCmsInit();
#endif

#ifdef CANVAS
pScreenFnVTable = canvasInit();
#endif

#ifdef OLEDCMS
pScreenFnVTable = displayCMSInit();
#endif
}

//
// Lots of things not separated yet.
//

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

#define BUTTON_TIME   250 // msec
#define BUTTON_PAUSE  500 // msec

//uint8_t armState;
uint8_t featureBlackbox = 0;
uint8_t featureLedstrip = 0;

#if defined(VTX) || defined(USE_RTC6705)
uint8_t featureVtx = 0, vtxBand, vtxChannel;
#endif // VTX || USE_RTC6705

OSD_Entry *menuStack[10]; //tab to save menu stack
uint8_t menuStackHistory[10]; //current position in menu stack
uint8_t menuStackIdx = 0;

#ifdef OSD
OSD_Entry menuAlarms[];
OSD_Entry menuOsdLayout[];
OSD_Entry menuOsdActiveElems[];
#endif

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
    {"--- MAIN MENU ---", OME_Label, NULL, NULL, 0},
    {"CFG. IMU", OME_Submenu, cmsChangeScreen, &menuImu[0], 0},
    {"FEATURES", OME_Submenu, cmsChangeScreen, &menuFeatures[0], 0},
#ifdef OSD
    {"SCR LAYOUT", OME_Submenu, cmsChangeScreen, &menuOsdLayout[0], 0},
    {"ALARMS", OME_Submenu, cmsChangeScreen, &menuAlarms[0], 0},
#endif
    {"SAVE&REBOOT", OME_OSD_Exit, cmsExitMenu, (void*)1, 0},
    {"EXIT", OME_OSD_Exit, cmsExitMenu, (void*)0, 0},
    {NULL,OME_END, NULL, NULL, 0}
};

OSD_Entry menuFeatures[] =
{
    {"--- FEATURES ---", OME_Label, NULL, NULL, 0},
    {"BLACKBOX", OME_Submenu, cmsChangeScreen, &menuBlackbox[0], 0},
#if defined(VTX) || defined(USE_RTC6705)
    {"VTX", OME_Submenu, cmsChangeScreen, &menu_vtx[0], 0},
#endif // VTX || USE_RTC6705
#ifdef LED_STRIP
    {"LED STRIP", OME_Submenu, cmsChangeScreen, &menuLedstrip[0], 0},
#endif // LED_STRIP
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

OSD_UINT8_t entryBlackboxRateDenom = {&masterConfig.blackbox_rate_denom,1,32,1};

OSD_Entry menuBlackbox[] =
{
    {"--- BLACKBOX ---", OME_Label, NULL, NULL, 0},
    {"ENABLED", OME_Bool, NULL, &featureBlackbox, 0},
    {"RATE DENOM", OME_UINT8, NULL, &entryBlackboxRateDenom, 0},
#ifdef USE_FLASHFS
    {"ERASE FLASH", OME_Submenu, cmsEraseFlash, NULL, 0},
#endif // USE_FLASHFS
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

#ifdef LED_STRIP
//local variable to keep color value
uint8_t ledColor;

static const char * const LED_COLOR_NAMES[] = {
    "BLACK   ",
    "WHITE   ",
    "RED     ",
    "ORANGE  ",
    "YELLOW  ",
    "LIME GRN",
    "GREEN   ",
    "MINT GRN",
    "CYAN    ",
    "LT BLUE ",
    "BLUE    ",
    "DK VIOLT",
    "MAGENTA ",
    "DEEP PNK"
};

//find first led with color flag and restore color index
//after saving all leds with flags color will have color set in OSD
static void getLedColor(void)
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
static void applyLedColor(void * ptr)
{
    UNUSED(ptr);
    for (int ledIndex = 0; ledIndex < LED_MAX_STRIP_LENGTH; ledIndex++) {
        ledConfig_t *ledConfig = &masterConfig.ledConfigs[ledIndex];
        if (ledGetFunction(ledConfig) == LED_FUNCTION_COLOR)
            *ledConfig = DEFINE_LED(ledGetX(ledConfig), ledGetY(ledConfig), ledColor, ledGetDirection(ledConfig), ledGetFunction(ledConfig), ledGetOverlay(ledConfig), 0);
    }
}

OSD_TAB_t entryLed = {&ledColor, 13, &LED_COLOR_NAMES[0]};

OSD_Entry menuLedstrip[] =
{
    {"--- LED STRIP ---", OME_Label, NULL, NULL, 0},
    {"ENABLED", OME_Bool, NULL, &featureLedstrip, 0},
    {"LED COLOR", OME_TAB, applyLedColor, &entryLed, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
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
    {"--- VTX ---", OME_Label, NULL, NULL, 0},
    {"ENABLED", OME_Bool, NULL, &featureVtx, 0},
#ifdef VTX
    {"VTX MODE", OME_UINT8, NULL, &entryVtxMode, 0},
    {"VTX MHZ", OME_UINT16, NULL, &entryVtxMhz, 0},
#endif // VTX
    {"BAND", OME_TAB, NULL, &entryVtxBand, 0},
    {"CHANNEL", OME_UINT8, NULL, &entryVtxChannel, 0},
#ifdef USE_RTC6705
    {"LOW POWER", OME_Bool, NULL, &masterConfig.vtx_power, 0},
#endif // USE_RTC6705
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};
#endif // VTX || USE_RTC6705

OSD_UINT16_t entryMinThrottle = {&masterConfig.motorConfig.minthrottle, 1020, 1300, 10};
OSD_UINT8_t entryGyroSoftLpfHz = {&masterConfig.gyro_soft_lpf_hz, 0, 255, 1};
OSD_UINT16_t entryDtermLpf = {&masterConfig.profile[0].pidProfile.dterm_lpf_hz, 0, 500, 5};
OSD_UINT16_t entryYawLpf = {&masterConfig.profile[0].pidProfile.yaw_lpf_hz, 0, 500, 5};
OSD_UINT16_t entryYawPLimit = {&masterConfig.profile[0].pidProfile.yaw_p_limit, 100, 500, 5};
OSD_UINT8_t entryVbatScale = {&masterConfig.batteryConfig.vbatscale, 1, 250, 1};
OSD_UINT8_t entryVbatMaxCell = {&masterConfig.batteryConfig.vbatmaxcellvoltage, 10, 50, 1};

OSD_Entry menuMisc[]=
{
    {"--- MISC ---", OME_Label, NULL, NULL, 0},
    {"GYRO LPF", OME_UINT8, NULL, &entryGyroSoftLpfHz, 0},
    {"DTERM LPF", OME_UINT16, NULL, &entryDtermLpf, 0},
    {"YAW LPF", OME_UINT16, NULL, &entryYawLpf, 0},
    {"YAW P LIM", OME_UINT16, NULL, &entryYawPLimit, 0},
    {"MIN THR", OME_UINT16, NULL, &entryMinThrottle, 0},
    {"VBAT SCALE", OME_UINT8, NULL, &entryVbatScale, 0},
    {"VBAT CLMAX", OME_UINT8, NULL, &entryVbatMaxCell, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

OSD_UINT8_t entryPidProfile = {&masterConfig.current_profile_index, 0, MAX_PROFILE_COUNT, 1};

OSD_Entry menuImu[] =
{
    {"--- CFG. IMU ---", OME_Label, NULL, NULL, 0},
    {"PID PROF", OME_UINT8, NULL, &entryPidProfile, 0},
    {"PID", OME_Submenu, cmsChangeScreen, &menuPid[0], 0},
    {"RATE&RXPO", OME_Submenu, cmsChangeScreen, &menuRateExpo[0], 0},
    {"RC PREV", OME_Submenu, cmsChangeScreen, &menuRc[0], 0},
    {"MISC", OME_Submenu, cmsChangeScreen, &menuMisc[0], 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
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
    {"--- PID ---", OME_Label, NULL, NULL, 0},
    {"ROLL P", OME_UINT8, NULL, &entryRollP, 0},
    {"ROLL I", OME_UINT8, NULL, &entryRollI, 0},
    {"ROLL D", OME_UINT8, NULL, &entryRollD, 0},

    {"PITCH P", OME_UINT8, NULL, &entryPitchP, 0},
    {"PITCH I", OME_UINT8, NULL, &entryPitchI, 0},
    {"PITCH D", OME_UINT8, NULL, &entryPitchD, 0},

    {"YAW P", OME_UINT8, NULL, &entryYawP, 0},
    {"YAW I", OME_UINT8, NULL, &entryYawI, 0},
    {"YAW D", OME_UINT8, NULL, &entryYawD, 0},

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

controlRateConfig_t rateProfile;

static OSD_FLOAT_t entryRollRate = {&rateProfile.rates[0], 0, 250, 1, 10};
static OSD_FLOAT_t entryPitchRate = {&rateProfile.rates[1], 0, 250, 1, 10};
static OSD_FLOAT_t entryYawRate = {&rateProfile.rates[2], 0, 250, 1, 10};
static OSD_FLOAT_t entryRcRate = {&rateProfile.rcRate8, 0, 200, 1, 10};
static OSD_FLOAT_t entryRcYawRate = {&rateProfile.rcYawRate8, 0, 200, 1, 10};
static OSD_FLOAT_t entryRcExpo = {&rateProfile.rcExpo8, 0, 100, 1, 10};
static OSD_FLOAT_t entryRcExpoYaw = {&rateProfile.rcYawExpo8, 0, 100, 1, 10};
static OSD_FLOAT_t extryTpaEntry = {&rateProfile.dynThrPID, 0, 70, 1, 10};
static OSD_UINT16_t entryTpaBreak = {&rateProfile.tpa_breakpoint, 1100, 1800, 10};
static OSD_FLOAT_t entryPSetpoint = {&masterConfig.profile[0].pidProfile.setpointRelaxRatio, 0, 100, 1, 10};
static OSD_FLOAT_t entryDSetpoint = {&masterConfig.profile[0].pidProfile.dtermSetpointWeight, 0, 255, 1, 10};

OSD_Entry menuRateExpo[] =
{
    {"--- RATE&EXPO ---", OME_Label, NULL, NULL, 0},
    {"RC RATE", OME_FLOAT, NULL, &entryRcYawRate, 0},
    {"RC YAW RATE", OME_FLOAT, NULL, &entryRcRate, 0},
    {"ROLL SUPER", OME_FLOAT, NULL, &entryRollRate, 0},
    {"PITCH SUPER", OME_FLOAT, NULL, &entryPitchRate, 0},
    {"YAW SUPER", OME_FLOAT, NULL, &entryYawRate, 0},
    {"RC EXPO", OME_FLOAT, NULL, &entryRcExpo, 0},
    {"RC YAW EXPO", OME_FLOAT, NULL, &entryRcExpoYaw, 0},
    {"THR PID ATT", OME_FLOAT, NULL, &extryTpaEntry, 0},
    {"TPA BRKPT", OME_UINT16, NULL, &entryTpaBreak, 0},
    {"D SETPT", OME_FLOAT, NULL, &entryDSetpoint, 0},
    {"D SETPT TRN", OME_FLOAT, NULL, &entryPSetpoint, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
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
    {"--- RC PREV ---", OME_Label, NULL, NULL, 0},
    {"ROLL", OME_Poll_INT16, NULL, &entryRcRoll, 0},
    {"PITCH", OME_Poll_INT16, NULL, &entryRcPitch, 0},
    {"THROTTLE", OME_Poll_INT16, NULL, &entryRcThrottle, 0},
    {"YAW", OME_Poll_INT16, NULL, &entryRcYaw, 0},
    {"AUX1", OME_Poll_INT16, NULL, &entryRcAux1, 0},
    {"AUX2", OME_Poll_INT16, NULL, &entryRcAux2, 0},
    {"AUX3", OME_Poll_INT16, NULL, &entryRcAux3, 0},
    {"AUX4", OME_Poll_INT16, NULL, &entryRcAux4, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};


bool cmsInMenu = false;

void cmsUpdateMaxRows(void)
{
    OSD_Entry *ptr;

    currentMenuIdx = 0;
    for (ptr = currentMenu; ptr->type != OME_END; ptr++)
        currentMenuIdx++;

    if (currentMenuIdx > MAX_MENU_ITEMS)
        currentMenuIdx = MAX_MENU_ITEMS;

    currentMenuIdx--;
}

uint16_t cmsHandleKey(uint8_t key)
{
    uint16_t res = BUTTON_TIME;
    OSD_Entry *p;

    if (!currentMenu)
        return res;

    if (key == KEY_ESC) {
        cmsMenuBack();
        return BUTTON_PAUSE;
    }

    if (key == KEY_DOWN) {
        if (currentCursorPos < currentMenuIdx) {
            currentCursorPos++;
        } else {
            if (nextPage) // we have more pages
            {
                cmsScreenClear();
                p = nextPage;
                nextPage = currentMenu;
                currentMenu = (OSD_Entry *)p;
                currentCursorPos = 0;
                cmsUpdateMaxRows();
            } else {
                currentCursorPos = 0;
            }
        }
    }

    if (key == KEY_UP) {
        currentCursorPos--;

        if ((currentMenu + currentCursorPos)->type == OME_Label && currentCursorPos > 0)
            currentCursorPos--;

        if (currentCursorPos == -1 || (currentMenu + currentCursorPos)->type == OME_Label) {
            if (nextPage) {
                cmsScreenClear();
                p = nextPage;
                nextPage = currentMenu;
                currentMenu = (OSD_Entry *)p;
                currentCursorPos = 0;
                cmsUpdateMaxRows();
            } else {
                currentCursorPos = currentMenuIdx;
            }
        }
    }

    if (key == KEY_DOWN || key == KEY_UP)
        return res;

    p = currentMenu + currentCursorPos;

    switch (p->type) {
        case OME_Submenu:
        case OME_OSD_Exit:
            if (p->func && key == KEY_RIGHT) {
                p->func(p->data);
                res = BUTTON_PAUSE;
            }
            break;
        case OME_Back:
            cmsMenuBack();
            res = BUTTON_PAUSE;
            break;
        case OME_Bool:
            if (p->data) {
                uint8_t *val = p->data;
                if (key == KEY_RIGHT)
                    *val = 1;
                else
                    *val = 0;
                SET_PRINTVALUE(p);
            }
            break;
        case OME_VISIBLE:
#ifdef OSD
            if (p->data) {
                uint32_t address = (uint32_t)p->data;
                uint16_t *val;

                val = (uint16_t *)address;

                if (key == KEY_RIGHT)
                    *val |= VISIBLE_FLAG;
                else
                    *val %= ~VISIBLE_FLAG;
                SET_PRINTVALUE(p);
            }
#endif
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
                SET_PRINTVALUE(p);
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
                SET_PRINTVALUE(p);
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
                SET_PRINTVALUE(p);
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
                SET_PRINTVALUE(p);
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
                SET_PRINTVALUE(p);
            }
            break;
        case OME_Poll_INT16:
        case OME_Label:
        case OME_END:
            break;
    }
    return res;
}

static void cmsFtoa(int32_t value, char *floatString)
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

void cmsPad(char *buf, int size)
{
    int i;

    for (i = 0 ; i < size ; i++) {
        if (buf[i] == 0)
            break;
    }

    for ( ; i < size ; i++) {
        buf[i] = ' ';
    }

    buf[size] = 0;
}

int cmsDrawMenuEntry(OSD_Entry *p, uint8_t row, bool drawPolled)
{
    char buff[10];
    int cnt = 0;

    switch (p->type) {
    case OME_Submenu:
        if (IS_PRINTVALUE(p))  {
            cnt = cmsScreenWrite(RIGHT_MENU_COLUMN, row, ">");
            CLR_PRINTVALUE(p);
        }
        break;
    case OME_Bool:
        if (IS_PRINTVALUE(p) && p->data) {
            if (*((uint8_t *)(p->data))) {
                cnt = cmsScreenWrite(RIGHT_MENU_COLUMN, row, "YES");
            } else {
                cnt = cmsScreenWrite(RIGHT_MENU_COLUMN, row, "NO ");
            }
            CLR_PRINTVALUE(p);
        }
        break;
    case OME_TAB: {
        if (IS_PRINTVALUE(p)) {
            OSD_TAB_t *ptr = p->data;
            //cnt = cmsScreenWrite(RIGHT_MENU_COLUMN - 5, row, (char *)ptr->names[*ptr->val]);
            cnt = cmsScreenWrite(RIGHT_MENU_COLUMN, row, (char *)ptr->names[*ptr->val]);
            CLR_PRINTVALUE(p);
        }
        break;
    }
    case OME_VISIBLE:
#ifdef OSD
        if (IS_PRINTVALUE(p) && p->data) {
            uint32_t address = (uint32_t)p->data;
            uint16_t *val;

            val = (uint16_t *)address;

            if (VISIBLE(*val)) {
                cnt = cmsScreenWrite(RIGHT_MENU_COLUMN, row, "YES");
            } else {
                cnt = cmsScreenWrite(RIGHT_MENU_COLUMN, row, "NO ");
            }
            CLR_PRINTVALUE(p);
        }
#endif
        break;
    case OME_UINT8:
        if (IS_PRINTVALUE(p) && p->data) {
            OSD_UINT8_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cmsPad(buff, 5);
            //cnt = cmsScreenWrite(RIGHT_MENU_COLUMN, row, "     ");
            cnt = cmsScreenWrite(RIGHT_MENU_COLUMN, row, buff);
            CLR_PRINTVALUE(p);
        }
        break;
    case OME_INT8:
        if (IS_PRINTVALUE(p) && p->data) {
            OSD_INT8_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cmsPad(buff, 5);
            cnt = cmsScreenWrite(RIGHT_MENU_COLUMN, row, buff);
            CLR_PRINTVALUE(p);
        }
        break;
    case OME_UINT16:
        if (IS_PRINTVALUE(p) && p->data) {
            OSD_UINT16_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cmsPad(buff, 5);
            cnt = cmsScreenWrite(RIGHT_MENU_COLUMN, row, buff);
            CLR_PRINTVALUE(p);
        }
        break;
    case OME_INT16:
        if (IS_PRINTVALUE(p) && p->data) {
            OSD_UINT16_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cmsPad(buff, 5);
            cnt = cmsScreenWrite(RIGHT_MENU_COLUMN, row, buff);
            CLR_PRINTVALUE(p);
        }
        break;
    case OME_Poll_INT16:
        if (p->data && drawPolled) {
            OSD_UINT16_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cmsPad(buff, 5);
            cnt = cmsScreenWrite(RIGHT_MENU_COLUMN, row, buff);
            // PRINTVALUE not cleared on purpose
        }
        break;
    case OME_FLOAT:
        if (IS_PRINTVALUE(p) && p->data) {
            OSD_FLOAT_t *ptr = p->data;
            cmsFtoa(*ptr->val * ptr->multipler, buff);
            cmsPad(buff, 5);
            // XXX One char left ???
            cnt = cmsScreenWrite(RIGHT_MENU_COLUMN - 1, row, buff);
            CLR_PRINTVALUE(p);
        }
        break;
    case OME_OSD_Exit:
    case OME_Label:
    case OME_END:
    case OME_Back:
        break;
    }

    return cnt;
}

void cmsDrawMenu(uint32_t currentTime)
{
    UNUSED(currentTime);

    uint8_t i;
    OSD_Entry *p;
    uint8_t top = (cmsRows - currentMenuIdx) / 2 - 1;

    // Polled (dynamic) value display denominator.
    // XXX Need to denom based on absolute time
    static uint8_t pollDenom = 0;
    bool drawPolled = (++pollDenom % 8 == 0);

    int16_t cnt = 0;

    if (!currentMenu)
        return;

    if (cmsScreenCleared) {
        for (p = currentMenu, i= 0; p->type != OME_END; p++, i++) {
            SET_PRINTLABEL(p);
            SET_PRINTVALUE(p);
        }

        if (i > MAX_MENU_ITEMS) // max per page
        {
            nextPage = currentMenu + MAX_MENU_ITEMS;
            if (nextPage->type == OME_END)
                nextPage = NULL;
        }

        cmsScreenCleared = false;
    }

    // Cursor manipulation

    if ((currentMenu + currentCursorPos)->type == OME_Label) // skip label
        currentCursorPos++;

    if (lastCursorPos >= 0 && currentCursorPos != lastCursorPos) {
        cnt += cmsScreenWrite(LEFT_MENU_COLUMN, lastCursorPos + top, "  ");
    }

    if (lastCursorPos != currentCursorPos) {
        cnt += cmsScreenWrite(LEFT_MENU_COLUMN, currentCursorPos + top, " >");
        lastCursorPos = currentCursorPos;
    }

    // Print text labels
    for (i = 0, p = currentMenu; i < MAX_MENU_ITEMS && p->type != OME_END; i++, p++) {
        if (IS_PRINTLABEL(p)) {
            cnt += cmsScreenWrite(LEFT_MENU_COLUMN + 2, i + top, p->text);
            CLR_PRINTLABEL(p);
            if (cnt > cmsBatchsize)
                return;
        }
    }

    // Print values

    // XXX Polled values at latter positions in the list may not be
    // XXX printed if the cnt exceeds cmsBatchsize in the middle of the list.

    for (i = 0, p = currentMenu; i < MAX_MENU_ITEMS && p->type != OME_END; i++, p++) {
        if (IS_PRINTVALUE(p)) {
            cnt += cmsDrawMenuEntry(p, top + i, drawPolled);
            if (cnt > cmsBatchsize)
                return;
        }
    }
}

void cmsChangeScreen(void *ptr)
{
    uint8_t i;
    if (ptr) {
        // hack - save profile to temp
        if (ptr == &menuPid[0]) {
            for (i = 0; i < 3; i++) {
                tempPid[i][0] = masterConfig.profile[masterConfig.current_profile_index].pidProfile.P8[i];
                tempPid[i][1] = masterConfig.profile[masterConfig.current_profile_index].pidProfile.I8[i];
                tempPid[i][2] = masterConfig.profile[masterConfig.current_profile_index].pidProfile.D8[i];
            }
            tempPid[3][0] = masterConfig.profile[masterConfig.current_profile_index].pidProfile.P8[PIDLEVEL];
            tempPid[3][1] = masterConfig.profile[masterConfig.current_profile_index].pidProfile.I8[PIDLEVEL];
            tempPid[3][2] = masterConfig.profile[masterConfig.current_profile_index].pidProfile.D8[PIDLEVEL];
        }

        if (ptr == &menuRateExpo[0])
            memcpy(&rateProfile, &masterConfig.profile[masterConfig.current_profile_index].controlRateProfile[masterConfig.profile[masterConfig.current_profile_index].activeRateProfile], sizeof(controlRateConfig_t));

        menuStack[menuStackIdx] = currentMenu;
        menuStackHistory[menuStackIdx] = currentCursorPos;
        menuStackIdx++;

        currentMenu = (OSD_Entry *)ptr;
        currentCursorPos = 0;
        cmsScreenClear();
        cmsUpdateMaxRows();
    }
}

void cmsMenuBack(void)
{
    uint8_t i;

    // becasue pids and rates meybe stored in profiles we need some thicks to manipulate it
    // hack to save pid profile
    if (currentMenu == &menuPid[0]) {
        for (i = 0; i < 3; i++) {
            masterConfig.profile[masterConfig.current_profile_index].pidProfile.P8[i] = tempPid[i][0];
            masterConfig.profile[masterConfig.current_profile_index].pidProfile.I8[i] = tempPid[i][1];
            masterConfig.profile[masterConfig.current_profile_index].pidProfile.D8[i] = tempPid[i][2];
        }

        masterConfig.profile[masterConfig.current_profile_index].pidProfile.P8[PIDLEVEL] = tempPid[3][0];
        masterConfig.profile[masterConfig.current_profile_index].pidProfile.I8[PIDLEVEL] = tempPid[3][1];
        masterConfig.profile[masterConfig.current_profile_index].pidProfile.D8[PIDLEVEL] = tempPid[3][2];
    }

    // hack - save rate config for current profile
    if (currentMenu == &menuRateExpo[0])
        memcpy(&masterConfig.profile[masterConfig.current_profile_index].controlRateProfile[masterConfig.profile[masterConfig.current_profile_index].activeRateProfile], &rateProfile, sizeof(controlRateConfig_t));

    if (menuStackIdx) {
        cmsScreenClear();
        menuStackIdx--;
        nextPage = NULL;
        currentMenu = menuStack[menuStackIdx];
        currentCursorPos = menuStackHistory[menuStackIdx];

        cmsUpdateMaxRows();
    }
}

void cmsOpenMenu(void)
{
    if (cmsInMenu)
        return;

    featureLedstrip = feature(FEATURE_LED_STRIP) ? 1 : 0;
    featureBlackbox = feature(FEATURE_BLACKBOX) ? 1 : 0;

#if defined(VTX) || defined(USE_RTC6705)
    featureVtx = feature(FEATURE_VTX) ? 1 : 0;
#endif // VTX || USE_RTC6705

#ifdef VTX
    vtxBand = masterConfig.vtxBand;
    vtxChannel = masterConfig.vtx_channel + 1;
#endif // VTX

#ifdef USE_RTC6705
    vtxBand = masterConfig.vtx_channel / 8;
    vtxChannel = masterConfig.vtx_channel % 8 + 1;
#endif // USE_RTC6705

#ifdef LED_STRIP
    getLedColor();
#endif // LED_STRIP

    cmsGetDevParam(&cmsRows, &cmsCols, &cmsBuftime, &cmsBufsize);

    cmsBatchsize = (cmsBuftime < CMS_UPDATE_INTERVAL) ? cmsBufsize : (cmsBufsize * CMS_UPDATE_INTERVAL) / cmsBuftime;

    cmsInMenu = true;
    cmsScreenBegin();
    currentMenu = &menuMain[0];
    cmsChangeScreen(currentMenu);
}

void cmsExitMenu(void *ptr)
{
    if (ptr) {
        cmsScreenClear();

        cmsScreenWrite(5, 3, "RESTARTING IMU...");
        cmsScreenResync(); // Was max7456RefreshAll(); why at this timing?

        stopMotors();
        stopPwmAllMotors();
        delay(200);

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

    cmsInMenu = false;

    cmsScreenEnd();

    if (ptr)
        systemReset();
}

void cmsUpdate(uint32_t currentTime)
{
    static int16_t rcDelay = BUTTON_TIME;
    static uint32_t lastCalled = 0;
    static uint32_t lastCmsHeartBeat = 0;

    uint8_t key = 0;

    // Detect menu invocation
    if (IS_MID(THROTTLE) && IS_LO(YAW) && IS_HI(PITCH) && !ARMING_FLAG(ARMED)) {
        // XXX Double enter!?
        cmsOpenMenu();
        rcDelay = BUTTON_PAUSE;    // Tends to overshoot if BUTTON_TIME
        lastCalled = currentTime;
        return;
    }

    if (cmsInMenu) {
        if (rcDelay > 0) {
            rcDelay -= currentTime - lastCalled;
            debug[0] = rcDelay;
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

        lastCalled = currentTime;

        if (key && !currentElement) {
            rcDelay = cmsHandleKey(key);
            return;
        }

        cmsDrawMenu(currentTime);

        if (currentTime > lastCmsHeartBeat + 500) {
            // Heart beat for external CMS display device @ 500msec
            // (Timeout @ 1000msec)
            cmsScreenHeartBeat();
            lastCmsHeartBeat = currentTime;
        }
    }
}

void cmsHandler(uint32_t unusedTime)
{
    UNUSED(unusedTime);

    if (pScreenFnVTable == NULL)
        return;

    static uint32_t lastCalled = 0;
    uint32_t now = millis();

    if (now - lastCalled >= CMS_UPDATE_INTERVAL) {
        cmsUpdate(now);
        lastCalled = now;
    }

    // do not allow ARM if we are in menu
    if (cmsInMenu)
        DISABLE_ARMING_FLAG(OK_TO_ARM);
}

void cmsInit(void)
{
    cmsScreenInit();
}

// Does this belong here?

#ifdef USE_FLASHFS
void cmsEraseFlash(void *ptr)
{
    UNUSED(ptr);

    cmsScreenClear();
    cmsScreenWrite(5, 3, "ERASING FLASH...");
    cmsScreenResync(); // Was max7456RefreshAll(); Why at this timing?

    flashfsEraseCompletely();
    while (!flashfsIsReady()) {
        delay(100);
    }

    cmsScreenClear();
    cmsScreenResync(); // Was max7456RefreshAll(); wedges during heavy SPI?
}
#endif // USE_FLASHFS

#ifdef OSD
//
// OSD specific menu pages and items
// XXX Should be part of the osd.c, or new osd_cms.c.
//
OSD_Entry menuOsdLayout[] =
{
    {"---SCREEN LAYOUT---", OME_Label, NULL, NULL, 0},
    {"ACTIVE ELEM.", OME_Submenu, cmsChangeScreen, &menuOsdActiveElems[0], 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

OSD_UINT8_t entryAlarmRssi = {&masterConfig.osdProfile.rssi_alarm, 5, 90, 5};
OSD_UINT16_t entryAlarmCapacity = {&masterConfig.osdProfile.cap_alarm, 50, 30000, 50};
OSD_UINT16_t enryAlarmFlyTime = {&masterConfig.osdProfile.time_alarm, 1, 200, 1};
OSD_UINT16_t entryAlarmAltitude = {&masterConfig.osdProfile.alt_alarm, 1, 200, 1};

OSD_Entry menuAlarms[] =
{
    {"--- ALARMS ---", OME_Label, NULL, NULL, 0},
    {"RSSI", OME_UINT8, NULL, &entryAlarmRssi, 0},
    {"MAIN BAT", OME_UINT16, NULL, &entryAlarmCapacity, 0},
    {"FLY TIME", OME_UINT16, NULL, &enryAlarmFlyTime, 0},
    {"MAX ALT", OME_UINT16, NULL, &entryAlarmAltitude, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

OSD_Entry menuOsdActiveElems[] =
{
    {"--- ACTIV ELEM ---", OME_Label, NULL, NULL, 0},
    {"RSSI", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_RSSI_VALUE], 0},
    {"MAIN BATTERY", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_MAIN_BATT_VOLTAGE], 0},
    {"HORIZON", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_ARTIFICIAL_HORIZON], 0},
    {"HORIZON SIDEBARS", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_HORIZON_SIDEBARS], 0},
    {"UPTIME", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_ONTIME], 0},
    {"FLY TIME", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_FLYTIME], 0},
    {"FLY MODE", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_FLYMODE], 0},
    {"NAME", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_CRAFT_NAME], 0},
    {"THROTTLE", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_THROTTLE_POS], 0},
#ifdef VTX
    {"VTX CHAN", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_VTX_CHANNEL]},
#endif // VTX
    {"CURRENT (A)", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_CURRENT_DRAW], 0},
    {"USED MAH", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_MAH_DRAWN], 0},
#ifdef GPS
    {"GPS SPEED", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_GPS_SPEED], 0},
    {"GPS SATS.", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_GPS_SATS], 0},
#endif // GPS
    {"ALTITUDE", OME_VISIBLE, NULL, &masterConfig.osdProfile.item_pos[OSD_ALTITUDE], 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};
#endif

#endif // CMS
