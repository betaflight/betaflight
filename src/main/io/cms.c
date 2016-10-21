#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#include "build/version.h"

#include "drivers/system.h"

#include "io/cms_types.h"
#include "io/canvas.h"

#include "io/flashfs.h"
#include "io/osd.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#include "build/debug.h"

// Configuration Menu System forwards

uint8_t cmsHandleKey(uint8_t);
void cmsUpdateMaxRows(void);
void cmsOpenMenu(void);
void cmsExitMenu(void * ptr);
void cmsChangeScreen(void * ptr);
void cmsMenuBack(void);
void cmsDrawMenu(void);
void cmsGetSize(uint8_t *, uint8_t *);
void cmsEraseFlash(void *ptr);

screenFnVTable_t *pScreenFnVTable;

// Force draw all elements if true
bool cmsScreenCleared;

// Function vector may be good here.

uint8_t cmsRows;
uint8_t cmsCols;

void cmsGetSize(uint8_t *pRows, uint8_t *pCols)
{
    pScreenFnVTable->getsize(pRows, pCols);
}

void cmsScreenClear(void)
{
    pScreenFnVTable->clear();
    cmsScreenCleared = true;
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

void cmsScreenWrite(uint8_t x, uint8_t y, char *s)
{
    pScreenFnVTable->write(x, y, s);
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

    pScreenFnVTable->getsize(&cmsRows, &cmsCols);
}

void cmsScreenInit(void)
{
#ifdef OSD
pScreenFnVTable = osdCmsInit();
#endif

#ifdef CANVAS
pScreenFnVTable = canvasInit();
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

#define curr_profile masterConfig.profile[masterConfig.current_profile_index]

//osd current screen - to reduce long lines ;-)
#define OSD_cfg masterConfig.osdProfile

#if 0
uint16_t refreshTimeout = 0;

#define VISIBLE_FLAG  0x0800
#define BLINK_FLAG    0x0400
bool blinkState = true;

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
#endif

#define BUTTON_TIME   2
#define BUTTON_PAUSE  5

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
// Right column size be 5 chars??? (now 7)

#define LEFT_MENU_COLUMN  1
#define RIGHT_MENU_COLUMN 23

//#define MAX_MENU_ITEMS    (cmsGetRowsCount() - 2)
#define MAX_MENU_ITEMS    (cmsRows - 2)

//uint8_t armState;
uint8_t featureBlackbox = 0;
uint8_t featureLedstrip = 0;

#if defined(VTX) || defined(USE_RTC6705)
uint8_t featureVtx = 0, vtxBand, vtxChannel;
#endif // VTX || USE_RTC6705

OSD_Entry *menuStack[10]; //tab to save menu stack
uint8_t menuStackHistory[10]; //current position in menu stack
uint8_t menuStackIdx = 0;

OSD_Entry *currentMenu;
OSD_Entry *nextPage = NULL;

int8_t currentMenuPos = 0;
int8_t lastMenuPos;
uint8_t currentMenuIdx = 0;
uint16_t *currentElement = NULL;

#ifdef OSD
OSD_Entry menuAlarms[];
OSD_Entry menuOsdLayout[];
OSD_Entry menuOsdActiveElems[];
#if 0 // Not supported yet (or drop GUI position editing)
OSD_Entry menuOsdElemsPositions[];
#endif
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
    {"----MAIN MENU----", OME_Label, NULL, NULL, true},
#ifdef OSD
    {"SCREEN LAYOUT", OME_Submenu, cmsChangeScreen, &menuOsdLayout[0], true},
    {"ALARMS", OME_Submenu, cmsChangeScreen, &menuAlarms[0], true},
#endif
    {"CFG. IMU", OME_Submenu, cmsChangeScreen, &menuImu[0], true},
    {"FEATURES", OME_Submenu, cmsChangeScreen, &menuFeatures[0], true},
    {"SAVE & EXIT", OME_OSD_Exit, cmsExitMenu, (void*)1, true},
    {"EXIT", OME_OSD_Exit, cmsExitMenu, (void*)0, true},
    {NULL,OME_END, NULL, NULL, true}
};

OSD_Entry menuFeatures[] =
{
    {"----- FEATURES -----", OME_Label, NULL, NULL, true},
    {"BLACKBOX", OME_Submenu, cmsChangeScreen, &menuBlackbox[0], true},
#ifdef LED_STRIP
    {"LED STRIP", OME_Submenu, cmsChangeScreen, &menuLedstrip[0], true},
#endif // LED_STRIP
#if defined(VTX) || defined(USE_RTC6705)
    {"VTX", OME_Submenu, cmsChangeScreen, &menu_vtx[0], true},
#endif // VTX || USE_RTC6705
    {"BACK", OME_Back, NULL, NULL, true},
    {NULL, OME_END, NULL, NULL, true}
};

OSD_UINT8_t entryBlackboxRateDenom = {&masterConfig.blackbox_rate_denom,1,32,1};

OSD_Entry menuBlackbox[] =
{
    {"--- BLACKBOX ---", OME_Label, NULL, NULL, true},
    {"ENABLED", OME_Bool, NULL, &featureBlackbox, true},
    {"RATE DENOM", OME_UINT8, NULL, &entryBlackboxRateDenom, true},
#ifdef USE_FLASHFS
    {"ERASE FLASH", OME_Submenu, cmsEraseFlash, NULL, true},
#endif // USE_FLASHFS
    {"BACK", OME_Back, NULL, NULL, true},
    {NULL, OME_END, NULL, NULL, true}
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
    {"--- LED TRIP ---", OME_Label, NULL, NULL, true},
    {"ENABLED", OME_Bool, NULL, &featureLedstrip, true},
    {"LED COLOR", OME_TAB, applyLedColor, &entryLed, true},
    {"BACK", OME_Back, NULL, NULL, true},
    {NULL, OME_END, NULL, NULL, true}
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
    {"--- VTX ---", OME_Label, NULL, NULL, true},
    {"ENABLED", OME_Bool, NULL, &featureVtx, true},
#ifdef VTX
    {"VTX MODE", OME_UINT8, NULL, &entryVtxMode, true},
    {"VTX MHZ", OME_UINT16, NULL, &entryVtxMhz, true},
#endif // VTX
    {"BAND", OME_TAB, NULL, &entryVtxBand, true},
    {"CHANNEL", OME_UINT8, NULL, &entryVtxChannel, true},
#ifdef USE_RTC6705
    {"LOW POWER", OME_Bool, NULL, &masterConfig.vtx_power, true},
#endif // USE_RTC6705
    {"BACK", OME_Back, NULL, NULL, true},
    {NULL, OME_END, NULL, NULL, true}
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
    {"----- MISC -----", OME_Label, NULL, NULL, true},
    {"GYRO LOWPASS", OME_UINT8, NULL, &entryGyroSoftLpfHz, true},
    {"DTERM LPF", OME_UINT16, NULL, &entryDtermLpf, true},
    {"YAW LPF", OME_UINT16, NULL, &entryYawLpf, true},
    {"YAW P LIMIT", OME_UINT16, NULL, &entryYawPLimit, true},
    {"MINTHROTTLE", OME_UINT16, NULL, &entryMinThrottle, true},
    {"VBAT SCALE", OME_UINT8, NULL, &entryVbatScale, true},
    {"VBAT CELL MAX", OME_UINT8, NULL, &entryVbatMaxCell, true},
    {"BACK", OME_Back, NULL, NULL, true},
    {NULL, OME_END, NULL, NULL, true}
};

OSD_UINT8_t entryPidProfile = {&masterConfig.current_profile_index, 0, MAX_PROFILE_COUNT, 1};

OSD_Entry menuImu[] =
{
    {"-----CFG. IMU-----", OME_Label, NULL, NULL, true},
    {"PID", OME_Submenu, cmsChangeScreen, &menuPid[0], true},
    {"PID PROFILE", OME_UINT8, NULL, &entryPidProfile, true},
    {"RATE & RXPO", OME_Submenu, cmsChangeScreen, &menuRateExpo[0], true},
    {"RC PREVIEW", OME_Submenu, cmsChangeScreen, &menuRc[0], true},
    {"MISC", OME_Submenu, cmsChangeScreen, &menuMisc[0], true},
    {"BACK", OME_Back, NULL, NULL, true},
    {NULL, OME_END, NULL, NULL, true}
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
    {"------- PID -------", OME_Label, NULL, NULL, true},
    {"ROLL P", OME_UINT8, NULL, &entryRollP, true},
    {"ROLL I", OME_UINT8, NULL, &entryRollI, true},
    {"ROLL D", OME_UINT8, NULL, &entryRollD, true},

    {"PITCH P", OME_UINT8, NULL, &entryPitchP, true},
    {"PITCH I", OME_UINT8, NULL, &entryPitchI, true},
    {"PITCH D", OME_UINT8, NULL, &entryPitchD, true},

    {"YAW P", OME_UINT8, NULL, &entryYawP, true},
    {"YAW I", OME_UINT8, NULL, &entryYawI, true},
    {"YAW D", OME_UINT8, NULL, &entryYawD, true},

    {"BACK", OME_Back, NULL, NULL, true},
    {NULL, OME_END, NULL, NULL, true}
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
static OSD_FLOAT_t entryPSetpoint = {&masterConfig.profile[0].pidProfile.setpointRelaxRatio, 0, 100, 1, 10};
static OSD_FLOAT_t entryDSetpoint = {&masterConfig.profile[0].pidProfile.dtermSetpointWeight, 0, 255, 1, 10};

OSD_Entry menuRateExpo[] =
{
    {"----RATE & EXPO----", OME_Label, NULL, NULL, true},
    {"ROLL RATE", OME_FLOAT, NULL, &entryRollRate, true},
    {"PITCH RATE", OME_FLOAT, NULL, &entryPitchRate, true},
    {"YAW RATE", OME_FLOAT, NULL, &entryYawRate, true},
    {"RC RATE", OME_FLOAT, NULL, &entryRcRate, true},
    {"RC EXPO", OME_FLOAT, NULL, &entryRcExpo, true},
    {"RC YAW EXPO", OME_FLOAT, NULL, &entryRcExpoYaw, true},
    {"THR PID ATT", OME_FLOAT, NULL, &extryTpaEntry, true},
    {"TPA BRKPT", OME_UINT16, NULL, &entryTpaBreak, true},
    {"D SETPT", OME_FLOAT, NULL, &entryDSetpoint, true},
    {"D SETPT TRNS", OME_FLOAT, NULL, &entryPSetpoint, true},
    {"BACK", OME_Back, NULL, NULL, true},
    {NULL, OME_END, NULL, NULL, true}
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
    {"---- RC PREVIEW ----", OME_Label, NULL, NULL, true},
    {"ROLL", OME_Poll_INT16, NULL, &entryRcRoll, true},
    {"PITCH", OME_Poll_INT16, NULL, &entryRcPitch, true},
    {"THROTTLE", OME_Poll_INT16, NULL, &entryRcThrottle, true},
    {"YAW", OME_Poll_INT16, NULL, &entryRcYaw, true},
    {"AUX1", OME_Poll_INT16, NULL, &entryRcAux1, true},
    {"AUX2", OME_Poll_INT16, NULL, &entryRcAux2, true},
    {"AUX3", OME_Poll_INT16, NULL, &entryRcAux3, true},
    {"AUX4", OME_Poll_INT16, NULL, &entryRcAux4, true},
    {"BACK", OME_Back, NULL, NULL, true},
    {NULL, OME_END, NULL, NULL, true}
};


bool cmsInMenu = false;

//
// CMS specific functions
//

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

uint8_t cmsHandleKey(uint8_t key)
{
    uint8_t res = BUTTON_TIME;
    OSD_Entry *p;

    if (!currentMenu)
        return res;

    if (key == KEY_ESC) {
        cmsMenuBack();
        return BUTTON_PAUSE;
    }

    if (key == KEY_DOWN) {
        if (currentMenuPos < currentMenuIdx) {
            currentMenuPos++;
        } else {
            if (nextPage) // we have more pages
            {
                cmsScreenClear();
                p = nextPage;
                nextPage = currentMenu;
                currentMenu = (OSD_Entry *)p;
                currentMenuPos = 0;
                lastMenuPos = -1;
                cmsUpdateMaxRows();
            } else {
                currentMenuPos = 0;
            }
        }
    }

    if (key == KEY_UP) {
        currentMenuPos--;

        if ((currentMenu + currentMenuPos)->type == OME_Label && currentMenuPos > 0)
            currentMenuPos--;

        if (currentMenuPos == -1 || (currentMenu + currentMenuPos)->type == OME_Label) {
            if (nextPage) {
                cmsScreenClear();
                p = nextPage;
                nextPage = currentMenu;
                currentMenu = (OSD_Entry *)p;
                currentMenuPos = 0;
                lastMenuPos = -1;
                cmsUpdateMaxRows();
            } else {
                currentMenuPos = currentMenuIdx;
                // lastMenuPos = -1;
            }
        }
    }

    if (key == KEY_DOWN || key == KEY_UP)
        return res;

    p = currentMenu + currentMenuPos;

    switch (p->type) {
        case OME_POS:
#if 0
#ifdef OSD
            if (key == KEY_RIGHT) {
                uint32_t address = (uint32_t)p->data;
                uint16_t *val;

                val = (uint16_t *)address;
                if (!(*val & VISIBLE_FLAG)) // no submenu for hidden elements
                    break;
            }
#endif
#endif
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
                p->changed = true;
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
                p->changed = true;
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
                p->changed = true;
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
                p->changed = true;
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
                p->changed = true;
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
                p->changed = true;
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
                p->changed = true;
            }
            break;
        case OME_Poll_INT16:
        case OME_Label:
        case OME_END:
            break;
    }
    return res;
}

static void simple_ftoa(int32_t value, char *floatString)
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

void cmsDrawMenu(void)
{
    uint8_t i = 0;
    OSD_Entry *p;
    char buff[10];
    uint8_t top = (cmsRows - currentMenuIdx) / 2 - 1;

    // XXX Need denom based on absolute time?
    static uint8_t pollDenom = 0;
    bool drawPolled = (++pollDenom % 8 == 0);

    if (!currentMenu)
        return;

    if ((currentMenu + currentMenuPos)->type == OME_Label) // skip label
        currentMenuPos++;

    if (lastMenuPos >= 0 && currentMenuPos != lastMenuPos)
        cmsScreenWrite(LEFT_MENU_COLUMN, lastMenuPos + top, "  ");

    for (p = currentMenu; p->type != OME_END; p++) {

        if (currentMenuPos == i && lastMenuPos != currentMenuPos) {
            cmsScreenWrite(LEFT_MENU_COLUMN, i + top, " >");
            lastMenuPos = currentMenuPos;
        }

        if (cmsScreenCleared)
            cmsScreenWrite(LEFT_MENU_COLUMN + 2, i + top, p->text);

        switch (p->type) {
            case OME_POS:; // Semi-colon required to add an empty statement
#ifdef OSD
                uint32_t address = (uint32_t)p->data;
                uint16_t *val;

                val = (uint16_t *)address;
                if (!(*val & VISIBLE_FLAG))
                    break;
#endif

            case OME_Submenu:
                if (cmsScreenCleared)
                    cmsScreenWrite(RIGHT_MENU_COLUMN, i + top, ">");
                break;
            case OME_Bool:
                if ((p->changed || cmsScreenCleared) && p->data) {
                    if (*((uint8_t *)(p->data))) {
                        cmsScreenWrite(RIGHT_MENU_COLUMN, i + top, "YES");
                    } else {
                        cmsScreenWrite(RIGHT_MENU_COLUMN, i + top, "NO ");
                    }
                    p->changed = false;
                }
                break;
            case OME_TAB: {
                if (p->changed || cmsScreenCleared) {
                    OSD_TAB_t *ptr = p->data;
                    cmsScreenWrite(RIGHT_MENU_COLUMN - 5, i + top, (char *)ptr->names[*ptr->val]);
                    p->changed = false;
                }
                break;
            }
            case OME_VISIBLE:
#ifdef OSD
                if ((p->changed || cmsScreenCleared) && p->data) {
                    uint32_t address = (uint32_t)p->data;
                    uint16_t *val;

                    val = (uint16_t *)address;

                    if (VISIBLE(*val)) {
                        cmsScreenWrite(RIGHT_MENU_COLUMN, i + top, "YES");
                    } else {
                        cmsScreenWrite(RIGHT_MENU_COLUMN, i + top, "NO ");
                    }
                    p->changed = false;
                }
#endif
                break;
            case OME_UINT8:
                if ((p->changed || cmsScreenCleared) && p->data) {
                    OSD_UINT8_t *ptr = p->data;
                    itoa(*ptr->val, buff, 10);
                    cmsScreenWrite(RIGHT_MENU_COLUMN, i + top, "     ");
                    cmsScreenWrite(RIGHT_MENU_COLUMN, i + top, buff);
                    p->changed = false;
                }
                break;
            case OME_INT8:
                if ((p->changed || cmsScreenCleared) && p->data) {
                    OSD_INT8_t *ptr = p->data;
                    itoa(*ptr->val, buff, 10);
                    cmsScreenWrite(RIGHT_MENU_COLUMN, i + top, "     ");
                    cmsScreenWrite(RIGHT_MENU_COLUMN, i + top, buff);
                    p->changed = false;
                }
                break;
            case OME_UINT16:
                if ((p->changed || cmsScreenCleared) && p->data) {
                    OSD_UINT16_t *ptr = p->data;
                    itoa(*ptr->val, buff, 10);
                    cmsScreenWrite(RIGHT_MENU_COLUMN, i + top, "     ");
                    cmsScreenWrite(RIGHT_MENU_COLUMN, i + top, buff);
                    p->changed = false;
                }
                break;
            case OME_INT16:
                if ((p->changed || cmsScreenCleared) && p->data) {
                    OSD_UINT16_t *ptr = p->data;
                    itoa(*ptr->val, buff, 10);
                    cmsScreenWrite(RIGHT_MENU_COLUMN, i + top, "     ");
                    cmsScreenWrite(RIGHT_MENU_COLUMN, i + top, buff);
                    p->changed = false;
                }
                break;
            case OME_Poll_INT16:
                if (p->data && drawPolled) {
                    OSD_UINT16_t *ptr = p->data;
                    itoa(*ptr->val, buff, 10);
                    cmsScreenWrite(RIGHT_MENU_COLUMN, i + top, "     ");
                    cmsScreenWrite(RIGHT_MENU_COLUMN, i + top, buff);
                }
                break;
            case OME_FLOAT:
                if ((p->changed || cmsScreenCleared) && p->data) {
                    OSD_FLOAT_t *ptr = p->data;
                    simple_ftoa(*ptr->val * ptr->multipler, buff);
                    cmsScreenWrite(RIGHT_MENU_COLUMN - 1, i + top, "      ");
                    cmsScreenWrite(RIGHT_MENU_COLUMN - 1, i + top, buff);
                    p->changed = false;
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
    cmsScreenCleared = false;
}

void cmsChangeScreen(void *ptr)
{
    uint8_t i;
    if (ptr) {
        cmsScreenClear();
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
        lastMenuPos = -1; // XXX this?
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
        cmsScreenClear();
        menuStackIdx--;
        nextPage = NULL;
        currentMenu = menuStack[menuStackIdx];
        currentMenuPos = menuStackHistory[menuStackIdx];
        lastMenuPos = -1;

        cmsUpdateMaxRows();
    }
    else {
        cmsOpenMenu();
    }
}

void cmsOpenMenu(void)
{
    if (cmsInMenu)
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

#ifdef LED_STRIP
    getLedColor();
#endif // LED_STRIP

    // cmsRows = cmsGetRowsCount();
    cmsGetSize(&cmsRows, &cmsCols);
    cmsInMenu = true;
    cmsScreenBegin();
    cmsScreenClear();
    currentMenu = &menuMain[0];
    cmsChangeScreen(currentMenu);
}

void cmsExitMenu(void *ptr)
{
    cmsScreenClear();

    cmsScreenWrite(5, 3, "RESTARTING IMU...");
    cmsScreenResync(); // Was max7456RefreshAll(); why at this timing?

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

    cmsInMenu = false;

    cmsScreenEnd();

    systemReset();
}

void cmsUpdate(uint32_t currentTime)
{
    static uint8_t rcDelay = BUTTON_TIME;
    uint8_t key = 0;
    static uint32_t lastCmsHeartBeat = 0;

    // detect enter to menu
    if (IS_MID(THROTTLE) && IS_HI(YAW) && IS_HI(PITCH) && !ARMING_FLAG(ARMED)) {
        // XXX Double enter!?
        cmsOpenMenu();
    }

    if (cmsInMenu) {
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

        // XXX Element position adjustment is hard to separate.
        // XXX May need to drop it upon real separation.
        // XXX Don't know if this still works

        if (key && !currentElement) {
            rcDelay = cmsHandleKey(key);
            return;
        }

        cmsDrawMenu();

        if (currentTime > lastCmsHeartBeat + 500) {
            // Heart beat for external CMS display device @ 500msec
            // (Timeout @ 1000msec)
            cmsScreenHeartBeat();
            lastCmsHeartBeat = currentTime;
        }
    }
}

void cmsHandler(uint32_t currentTime)
{
    static uint32_t counter = 0;

    if (counter++ % 5 == 0) {
        cmsUpdate(currentTime);
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
// XXX Should be part of the osd.c, or new osd_csm.c.
//
OSD_Entry menuOsdLayout[] =
{
    {"---SCREEN LAYOUT---", OME_Label, NULL, NULL, true},
    {"ACTIVE ELEM.", OME_Submenu, cmsChangeScreen, &menuOsdActiveElems[0], true},
#if 0
    {"POSITION", OME_Submenu, cmsChangeScreen, &menuOsdElemsPositions[0], true},
#endif
    {"BACK", OME_Back, NULL, NULL, true},
    {NULL, OME_END, NULL, NULL, true}
};

OSD_UINT8_t entryAlarmRssi = {&OSD_cfg.rssi_alarm, 5, 90, 5};
OSD_UINT16_t entryAlarmCapacity = {&OSD_cfg.cap_alarm, 50, 30000, 50};
OSD_UINT16_t enryAlarmFlyTime = {&OSD_cfg.time_alarm, 1, 200, 1};
OSD_UINT16_t entryAlarmAltitude = {&OSD_cfg.alt_alarm, 1, 200, 1};

OSD_Entry menuAlarms[] =
{
    {"------ ALARMS ------", OME_Label, NULL, NULL, true},
    {"RSSI", OME_UINT8, NULL, &entryAlarmRssi, true},
    {"MAIN BATT.", OME_UINT16, NULL, &entryAlarmCapacity, true},
    {"FLY TIME", OME_UINT16, NULL, &enryAlarmFlyTime, true},
    {"MAX ALTITUDE", OME_UINT16, NULL, &entryAlarmAltitude, true},
    {"BACK", OME_Back, NULL, NULL, true},
    {NULL, OME_END, NULL, NULL, true}
};

#if 0 // Not supported yet (or drop support for GUI position editing)
OSD_Entry menuOsdElemsPositions[] =
{
    {"---POSITION---", OME_Label, NULL, NULL, true},
    {"RSSI", OME_POS, osdEditElement, &OSD_cfg.item_pos[OSD_RSSI_VALUE], true},
    {"MAIN BATTERY", OME_POS, osdEditElement, &OSD_cfg.item_pos[OSD_MAIN_BATT_VOLTAGE], true},
    {"UPTIME", OME_POS, osdEditElement, &OSD_cfg.item_pos[OSD_ONTIME], true},
    {"FLY TIME", OME_POS, osdEditElement, &OSD_cfg.item_pos[OSD_FLYTIME], true},
    {"FLY MODE", OME_POS, osdEditElement, &OSD_cfg.item_pos[OSD_FLYMODE], true},
    {"NAME", OME_POS, osdEditElement, &OSD_cfg.item_pos[OSD_CRAFT_NAME], true},
    {"THROTTLE", OME_POS, osdEditElement, &OSD_cfg.item_pos[OSD_THROTTLE_POS], true},

#ifdef VTX
    {"VTX CHAN", OME_POS, osdEditElement, &OSD_cfg.item_pos[OSD_VTX_CHANNEL], true},
#endif // VTX
    {"CURRENT (A)", OME_POS, osdEditElement, &OSD_cfg.item_pos[OSD_CURRENT_DRAW], true},
    {"USED MAH", OME_POS, osdEditElement, &OSD_cfg.item_pos[OSD_MAH_DRAWN], true},
#ifdef GPS
    {"GPS SPEED", OME_POS, osdEditElement, &OSD_cfg.item_pos[OSD_GPS_SPEED], true},
    {"GPS SATS.", OME_POS, osdEditElement, &OSD_cfg.item_pos[OSD_GPS_SATS], true},
#endif // GPS
    {"ALTITUDE", OME_POS, osdEditElement, &OSD_cfg.item_pos[OSD_ALTITUDE], true},
    {"BACK", OME_Back, NULL, NULL, true},
    {NULL, OME_END, NULL, NULL, true}
};
#endif

OSD_Entry menuOsdActiveElems[] =
{
    {" --ACTIV ELEM.-- ", OME_Label, NULL, NULL, true},
    {"RSSI", OME_VISIBLE, NULL, &OSD_cfg.item_pos[OSD_RSSI_VALUE], true},
    {"MAIN BATTERY", OME_VISIBLE, NULL, &OSD_cfg.item_pos[OSD_MAIN_BATT_VOLTAGE], true},
    {"HORIZON", OME_VISIBLE, NULL, &OSD_cfg.item_pos[OSD_ARTIFICIAL_HORIZON], true},
    {"HORIZON SIDEBARS", OME_VISIBLE, NULL, &OSD_cfg.item_pos[OSD_HORIZON_SIDEBARS], true},
    {"UPTIME", OME_VISIBLE, NULL, &OSD_cfg.item_pos[OSD_ONTIME], true},
    {"FLY TIME", OME_VISIBLE, NULL, &OSD_cfg.item_pos[OSD_FLYTIME], true},
    {"FLY MODE", OME_VISIBLE, NULL, &OSD_cfg.item_pos[OSD_FLYMODE], true},
    {"NAME", OME_VISIBLE, NULL, &OSD_cfg.item_pos[OSD_CRAFT_NAME], true},
    {"THROTTLE", OME_VISIBLE, NULL, &OSD_cfg.item_pos[OSD_THROTTLE_POS], true},
#ifdef VTX
    {"VTX CHAN", OME_VISIBLE, NULL, &OSD_cfg.item_pos[OSD_VTX_CHANNEL]},
#endif // VTX
    {"CURRENT (A)", OME_VISIBLE, NULL, &OSD_cfg.item_pos[OSD_CURRENT_DRAW], true},
    {"USED MAH", OME_VISIBLE, NULL, &OSD_cfg.item_pos[OSD_MAH_DRAWN], true},
#ifdef GPS
    {"GPS SPEED", OME_VISIBLE, NULL, &OSD_cfg.item_pos[OSD_GPS_SPEED], true},
    {"GPS SATS.", OME_VISIBLE, NULL, &OSD_cfg.item_pos[OSD_GPS_SATS], true},
#endif // GPS
    {"ALTITUDE", OME_VISIBLE, NULL, &OSD_cfg.item_pos[OSD_ALTITUDE], true},
    {"BACK", OME_Back, NULL, NULL, true},
    {NULL, OME_END, NULL, NULL, true}
};
#endif
