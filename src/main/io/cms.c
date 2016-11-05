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
#define CMS_MENU_DEBUG // For external menu content creators

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#include "build/version.h"

#ifdef CMS

#include "drivers/system.h"

#include "common/typeconversion.h"

#include "io/cms.h"
#include "io/cms_types.h"

#ifdef CANVAS
#include "io/canvas.h"
#endif

#ifdef USE_FLASHFS
#include "io/flashfs.h"
#endif

#ifdef OSD
#include "io/osd.h"
#endif

#ifdef USE_DASHBOARD
#include "io/dashboard.h"
#endif

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#include "build/debug.h"

// External menu contents
#include "io/cms_imu.h"
#include "io/cms_blackbox.h"
#include "io/cms_vtx.h"
#ifdef OSD
#include "io/cms_osd.h"
#endif
#include "io/cms_ledstrip.h"

// Forwards
long cmsx_InfoInit(void);
long cmsx_FeatureRead(void);
long cmsx_FeatureWriteback(void);

// Device management

#ifndef CMS_MAX_DEVICE
#define CMS_MAX_DEVICE 4
#endif

cmsDeviceInitFuncPtr cmsDeviceInitFunc[CMS_MAX_DEVICE];
int cmsDeviceCount;
int cmsCurrentDevice = -1;
int cmsLastDevice = -1;

bool cmsDeviceRegister(cmsDeviceInitFuncPtr func)
{
    if (cmsDeviceCount == CMS_MAX_DEVICE)
        return false;

    cmsDeviceInitFunc[cmsDeviceCount++] = func;

    return true;
}

cmsDeviceInitFuncPtr cmsDeviceSelectCurrent(void)
{
    if (cmsDeviceCount == 0)
        return NULL;

    if (cmsCurrentDevice < 0)
        cmsCurrentDevice = 0;

    return cmsDeviceInitFunc[cmsCurrentDevice];
}

cmsDeviceInitFuncPtr cmsDeviceSelectNext(void)
{
    if (cmsDeviceCount == 0)
        return NULL;

    cmsCurrentDevice = (cmsCurrentDevice + 1) % cmsDeviceCount; // -1 Okay

    return cmsDeviceInitFunc[cmsCurrentDevice];
}

#define CMS_UPDATE_INTERVAL 50 // msec

void cmsScreenInit(displayPort_t *pDisp, cmsDeviceInitFuncPtr cmsDeviceInitFunc)
{
    cmsDeviceInitFunc(pDisp);
}


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
#define RIGHT_MENU_COLUMN(p) ((p)->cols - 8)
#define MAX_MENU_ITEMS(p)    ((p)->rows - 2)

displayPort_t currentDisplay;

bool cmsInMenu = false;

CMS_Menu menuMain;
CMS_Menu *currentMenu;          // Points to top entry of the current page

// XXX Does menu backing support backing into second page???

CMS_Menu *menuStack[10];         // Stack to save menu transition
uint8_t menuStackHistory[10];    // cursorRow in a stacked menu
uint8_t menuStackIdx = 0;

OSD_Entry *pageTop;              // Points to top entry of the current page
OSD_Entry *pageTopAlt;           // Only 2 pages are allowed (for now)
uint8_t maxRow;                  // Max row in the current page

int8_t entryPos;                 // Absolute position of the cursor
int8_t cursorRow;                // Position of the cursor relative to pageTop

// Broken menu substitution

char menuErrLabel[21 + 1];

OSD_Entry menuErrEntries[] = {
    { "BROKEN MENU", OME_Label, NULL, NULL, 0 },
    { menuErrLabel, OME_String, NULL, NULL, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu menuErr = {
    "MENU CONTENT BROKEN",
    OME_MENU,
    NULL,
    NULL,
    NULL,
    menuErrEntries,
};

// Stick/key detection

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

void cmsUpdateMaxRow(displayPort_t *instance)
{
    OSD_Entry *ptr;

    maxRow = 0;
    for (ptr = pageTop; ptr->type != OME_END; ptr++)
        maxRow++;

    if (maxRow > MAX_MENU_ITEMS(instance))
        maxRow = MAX_MENU_ITEMS(instance);

    maxRow--;
}

static void cmsFormatFloat(int32_t value, char *floatString)
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
    // Keep the first decimal place
    for (k = 5; k > 3; k--)
        if (floatString[k] == '0' || floatString[k] == '.')
            floatString[k] = 0;
        else
            break;

    // oraz zero wiodonce
    if (floatString[0] == '0')
        floatString[0] = ' ';
}

void cmsPadToSize(char *buf, int size)
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

int cmsDrawMenuEntry(displayPort_t *pDisplay, OSD_Entry *p, uint8_t row, bool drawPolled)
{
    char buff[10];
    int cnt = 0;

    switch (p->type) {
    case OME_String:
        if (IS_PRINTVALUE(p) && p->data) {
            cnt = displayWrite(pDisplay, RIGHT_MENU_COLUMN(pDisplay), row, p->data);
            CLR_PRINTVALUE(p);
        }
        break;
    case OME_Submenu:
        if (IS_PRINTVALUE(p))  {
            cnt = displayWrite(pDisplay, RIGHT_MENU_COLUMN(pDisplay), row, ">");
            CLR_PRINTVALUE(p);
        }
        break;
    case OME_Bool:
        if (IS_PRINTVALUE(p) && p->data) {
            if (*((uint8_t *)(p->data))) {
                cnt = displayWrite(pDisplay, RIGHT_MENU_COLUMN(pDisplay), row, "YES");
            } else {
                cnt = displayWrite(pDisplay, RIGHT_MENU_COLUMN(pDisplay), row, "NO ");
            }
            CLR_PRINTVALUE(p);
        }
        break;
    case OME_TAB: {
        if (IS_PRINTVALUE(p)) {
            OSD_TAB_t *ptr = p->data;
            //cnt = displayWrite(pDisplay, RIGHT_MENU_COLUMN(pDisplay) - 5, row, (char *)ptr->names[*ptr->val]);
            cnt = displayWrite(pDisplay, RIGHT_MENU_COLUMN(pDisplay), row, (char *)ptr->names[*ptr->val]);
            CLR_PRINTVALUE(p);
        }
        break;
    }
#ifdef OSD
    case OME_VISIBLE:
        if (IS_PRINTVALUE(p) && p->data) {
            uint32_t address = (uint32_t)p->data;
            uint16_t *val;

            val = (uint16_t *)address;

            if (VISIBLE(*val)) {
                cnt = displayWrite(pDisplay, RIGHT_MENU_COLUMN(pDisplay), row, "YES");
            } else {
                cnt = displayWrite(pDisplay, RIGHT_MENU_COLUMN(pDisplay), row, "NO ");
            }
            CLR_PRINTVALUE(p);
        }
        break;
#endif
    case OME_UINT8:
        if (IS_PRINTVALUE(p) && p->data) {
            OSD_UINT8_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cmsPadToSize(buff, 5);
            cnt = displayWrite(pDisplay, RIGHT_MENU_COLUMN(pDisplay), row, buff);
            CLR_PRINTVALUE(p);
        }
        break;
    case OME_INT8:
        if (IS_PRINTVALUE(p) && p->data) {
            OSD_INT8_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cmsPadToSize(buff, 5);
            cnt = displayWrite(pDisplay, RIGHT_MENU_COLUMN(pDisplay), row, buff);
            CLR_PRINTVALUE(p);
        }
        break;
    case OME_UINT16:
        if (IS_PRINTVALUE(p) && p->data) {
            OSD_UINT16_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cmsPadToSize(buff, 5);
            cnt = displayWrite(pDisplay, RIGHT_MENU_COLUMN(pDisplay), row, buff);
            CLR_PRINTVALUE(p);
        }
        break;
    case OME_INT16:
        if (IS_PRINTVALUE(p) && p->data) {
            OSD_UINT16_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cmsPadToSize(buff, 5);
            cnt = displayWrite(pDisplay, RIGHT_MENU_COLUMN(pDisplay), row, buff);
            CLR_PRINTVALUE(p);
        }
        break;
    case OME_Poll_INT16:
        if (p->data && drawPolled) {
            OSD_UINT16_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cmsPadToSize(buff, 5);
            cnt = displayWrite(pDisplay, RIGHT_MENU_COLUMN(pDisplay), row, buff);
            // PRINTVALUE not cleared on purpose
        }
        break;
    case OME_FLOAT:
        if (IS_PRINTVALUE(p) && p->data) {
            OSD_FLOAT_t *ptr = p->data;
            cmsFormatFloat(*ptr->val * ptr->multipler, buff);
            cmsPadToSize(buff, 5);
            cnt = displayWrite(pDisplay, RIGHT_MENU_COLUMN(pDisplay) - 1, row, buff); // XXX One char left ???
            CLR_PRINTVALUE(p);
        }
        break;
    case OME_OSD_Exit:
    case OME_Label:
    case OME_END:
    case OME_Back:
        break;
    case OME_MENU:
#ifdef CMS_MENU_DEBUG
        // Shouldn't happen. Notify creator of this menu content.
        cnt = displayWrite(pDisplay, RIGHT_MENU_COLUMN(pDisplay), row, "BADENT");
#endif
        break;
    }

    return cnt;
}

void cmsDrawMenu(displayPort_t *pDisplay)
{
    uint8_t i;
    OSD_Entry *p;
    uint8_t top = (pDisplay->rows - maxRow) / 2 - 1;

    // Polled (dynamic) value display denominator.
    // XXX Need to denom based on absolute time
    static uint8_t pollDenom = 0;
    bool drawPolled = (++pollDenom % 8 == 0);

    uint32_t room = displayTxBytesFree(pDisplay);

    if (!pageTop)
        return;

    if (pDisplay->cleared) {
        for (p = pageTop, i= 0; p->type != OME_END; p++, i++) {
            SET_PRINTLABEL(p);
            SET_PRINTVALUE(p);
        }

        if (i > MAX_MENU_ITEMS(pDisplay)) // max per page
        {
            pageTopAlt = pageTop + MAX_MENU_ITEMS(pDisplay);
            if (pageTopAlt->type == OME_END)
                pageTopAlt = NULL;
        }

        pDisplay->cleared = false;
    }

    // Cursor manipulation

    while ((pageTop + cursorRow)->type == OME_Label) // skip label
        cursorRow++;

    if (pDisplay->cursorRow >= 0 && cursorRow != pDisplay->cursorRow) {
        room -= displayWrite(pDisplay, LEFT_MENU_COLUMN, pDisplay->cursorRow + top, "  ");
    }

    if (room < 30)
        return;

    if (pDisplay->cursorRow != cursorRow) {
        room -= displayWrite(pDisplay, LEFT_MENU_COLUMN, cursorRow + top, " >");
        pDisplay->cursorRow = cursorRow;
    }

    if (room < 30)
        return;

    // Print text labels
    for (i = 0, p = pageTop; i < MAX_MENU_ITEMS(pDisplay) && p->type != OME_END; i++, p++) {
        if (IS_PRINTLABEL(p)) {
            room -= displayWrite(pDisplay, LEFT_MENU_COLUMN + 2, i + top, p->text);
            CLR_PRINTLABEL(p);
            if (room < 30)
                return;
        }
    }

    // Print values

    // XXX Polled values at latter positions in the list may not be
    // XXX printed if not enough room in the middle of the list.

    for (i = 0, p = pageTop; i < MAX_MENU_ITEMS(pDisplay) && p->type != OME_END; i++, p++) {
        if (IS_PRINTVALUE(p)) {
            room -= cmsDrawMenuEntry(pDisplay, p, top + i, drawPolled);
            if (room < 30)
                return;
        }
    }
}

long cmsMenuChange(displayPort_t *pDisplay, void *ptr)
{
    CMS_Menu *pMenu = (CMS_Menu *)ptr;

    if (pMenu) {
#ifdef CMS_MENU_DEBUG
        if (pMenu->GUARD_type != OME_MENU) {
            // ptr isn't pointing to a CMS_Menu.
            if (pMenu->GUARD_type < OME_MENU) {
                strncpy(menuErrLabel, pMenu->GUARD_text, 21);
            } else {
                strncpy(menuErrLabel, "LABEL UNKNOWN", 21);
            }
            pMenu = &menuErr;
        }
#endif

        // Stack the current menu and move to a new menu.
        // The (pMenu == curretMenu) case occurs when reopening for display sw

        if (pMenu != currentMenu) {
            menuStack[menuStackIdx] = currentMenu;
            cursorRow += pageTop - currentMenu->entries; // Convert cursorRow to absolute value
            menuStackHistory[menuStackIdx] = cursorRow;
            menuStackIdx++;

            currentMenu = (CMS_Menu *)ptr;
            cursorRow = 0;

            if (pMenu->onEnter)
                pMenu->onEnter();
        }

        pageTop = currentMenu->entries;

        displayClear(pDisplay);
        cmsUpdateMaxRow(pDisplay);
    }

    return 0;
}

long cmsMenuBack(displayPort_t *pDisplay)
{
    if (currentMenu->onExit)
        currentMenu->onExit();

    if (menuStackIdx) {
        displayClear(pDisplay);
        menuStackIdx--;
        currentMenu = menuStack[menuStackIdx];
        cursorRow = menuStackHistory[menuStackIdx];
        pageTop = currentMenu->entries; // Temporary for cmsUpdateMaxRow()
        cmsUpdateMaxRow(pDisplay);
        if (cursorRow > maxRow) {
            pageTopAlt = currentMenu->entries;
            pageTop = pageTopAlt + maxRow + 1;
            cursorRow -= (maxRow + 1);
            cmsUpdateMaxRow(pDisplay);
        }
    }

    return 0;
}

void cmsMenuOpen(void)
{
    cmsDeviceInitFuncPtr initfunc;

    if (!cmsInMenu) {
        // New open
        cmsInMenu = true;
        DISABLE_ARMING_FLAG(OK_TO_ARM);
        initfunc = cmsDeviceSelectCurrent(); 
        currentMenu = &menuMain;
    } else {
        // Switch display
        displayClose(&currentDisplay);
        initfunc = cmsDeviceSelectNext();
    }

    if (!initfunc)
        return;

    cmsScreenInit(&currentDisplay, initfunc);
    displayOpen(&currentDisplay);
    cmsMenuChange(&currentDisplay, currentMenu);
}

void cmsTraverseGlobalExit(CMS_Menu *pMenu)
{
    OSD_Entry *p;

    for (p = pMenu->entries; p->type != OME_END ; p++)
        if (p->type == OME_Submenu)
            cmsTraverseGlobalExit(p->data);

    if (pMenu->onGlobalExit)
        pMenu->onGlobalExit();
}

long cmsMenuExit(displayPort_t *pDisplay, void *ptr)
{
    if (ptr) {
        displayClear(pDisplay);

        displayWrite(pDisplay, 5, 3, "REBOOTING...");
        displayResync(pDisplay); // Was max7456RefreshAll(); why at this timing?

        stopMotors();
        stopPwmAllMotors();
        delay(200);

        cmsTraverseGlobalExit(&menuMain);

        if (currentMenu->onExit)
            currentMenu->onExit();
    }

    cmsInMenu = false;

    displayClose(pDisplay);
    currentMenu = NULL;

    if (ptr)
        systemReset();

    ENABLE_ARMING_FLAG(OK_TO_ARM);

    return 0;
}

uint16_t cmsHandleKey(displayPort_t *pDisplay, uint8_t key)
{
    uint16_t res = BUTTON_TIME;
    OSD_Entry *p;

    if (!currentMenu)
        return res;

    if (key == KEY_ESC) {
        cmsMenuBack(pDisplay);
        return BUTTON_PAUSE;
    }

    if (key == KEY_DOWN) {
        if (cursorRow < maxRow) {
            cursorRow++;
        } else {
            if (pageTopAlt) { // we have another page
                displayClear(pDisplay);
                p = pageTopAlt;
                pageTopAlt = pageTop;
                pageTop = (OSD_Entry *)p;
                cmsUpdateMaxRow(pDisplay);
            }
            cursorRow = 0;    // Goto top in any case
        }
    }

    if (key == KEY_UP) {
        cursorRow--;

        if ((pageTop + cursorRow)->type == OME_Label && cursorRow > 0)
            cursorRow--;

        if (cursorRow == -1 || (pageTop + cursorRow)->type == OME_Label) {
            if (pageTopAlt) {
                displayClear(pDisplay);
                p = pageTopAlt;
                pageTopAlt = pageTop;
                pageTop = (OSD_Entry *)p;
                cmsUpdateMaxRow(pDisplay);
            }
            cursorRow = maxRow;    // Goto bottom in any case
        }
    }

    if (key == KEY_DOWN || key == KEY_UP)
        return res;

    p = pageTop + cursorRow;

    switch (p->type) {
        case OME_Submenu:
        case OME_OSD_Exit:
            if (p->func && key == KEY_RIGHT) {
                p->func(pDisplay, p->data);
                res = BUTTON_PAUSE;
            }
            break;
        case OME_Back:
            cmsMenuBack(pDisplay);
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
#ifdef OSD
        case OME_VISIBLE:
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
            break;
#endif
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
                    p->func(pDisplay, p->data);
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
        case OME_String:
            break;
        case OME_Poll_INT16:
        case OME_Label:
        case OME_END:
            break;
        case OME_MENU:
            // Shouldn't happen
            break;
    }
    return res;
}

void cmsUpdate(displayPort_t *pDisplay, uint32_t currentTime)
{
    static int16_t rcDelay = BUTTON_TIME;
    static uint32_t lastCalled = 0;
    static uint32_t lastCmsHeartBeat = 0;

    uint8_t key = 0;

    if (!cmsInMenu) {
        // Detect menu invocation
        if (IS_MID(THROTTLE) && IS_LO(YAW) && IS_HI(PITCH) && !ARMING_FLAG(ARMED)) {
            cmsMenuOpen();
            rcDelay = BUTTON_PAUSE;    // Tends to overshoot if BUTTON_TIME
        }
    } else {
        if (rcDelay > 0) {
            rcDelay -= (currentTime - lastCalled);
        }
        else if (IS_MID(THROTTLE) && IS_LO(YAW) && IS_HI(PITCH) && !ARMING_FLAG(ARMED)) {
            // Double enter = display switching
            cmsMenuOpen();
            rcDelay = BUTTON_PAUSE;
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
        else if ((IS_HI(YAW) || IS_LO(YAW)) && currentMenu != &cmsx_menuRc) // this menu is used to check transmitter signals so can't exit using YAW
        {
            key = KEY_ESC;
            rcDelay = BUTTON_TIME;
        }

        //lastCalled = currentTime;

        if (key) {
            rcDelay = cmsHandleKey(&currentDisplay, key);
            return;
        }

        cmsDrawMenu(pDisplay);

        if (currentTime > lastCmsHeartBeat + 500) {
            // Heart beat for external CMS display device @ 500msec
            // (Timeout @ 1000msec)
            displayHeartbeat(&currentDisplay);
            lastCmsHeartBeat = currentTime;
        }
    }
    lastCalled = currentTime;
}

void cmsHandler(uint32_t currentTime)
{
    if (cmsDeviceCount < 0)
        return;

    static uint32_t lastCalled = 0;
    const uint32_t now = currentTime / 1000;

    if (now - lastCalled >= CMS_UPDATE_INTERVAL) {
        cmsUpdate(&currentDisplay, now);
        lastCalled = now;
    }
}

void cmsInit(void)
{
    //cmsx_InfoInit();
}

//
// Menu contents
//

// Info

static char infoGitRev[GIT_SHORT_REVISION_LENGTH];
static char infoTargetName[] = __TARGET__;

#include "msp/msp_protocol.h" // XXX for FC identification... not available elsewhere

OSD_Entry menuInfoEntries[] = {
    { "--- INFO ---", OME_Label, NULL, NULL, 0 },
    { "FWID", OME_String, NULL, BETAFLIGHT_IDENTIFIER, 0 },
    { "FWVER", OME_String, NULL, FC_VERSION_STRING, 0 },
    { "GITREV", OME_String, NULL, infoGitRev, 0 },
    { "TARGET", OME_String, NULL, infoTargetName, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu menuInfo = {
    "MENUINFO",
    OME_MENU,
    cmsx_InfoInit,
    NULL,
    NULL,
    menuInfoEntries,
};

long cmsx_InfoInit(void)
{
    for (int i = 0 ; i < GIT_SHORT_REVISION_LENGTH ; i++) {
        if (shortGitRevision[i] >= 'a' && shortGitRevision[i] <= 'f')
            infoGitRev[i] = shortGitRevision[i] - 'a' + 'A';
        else
            infoGitRev[i] = shortGitRevision[i];
    }

    return 0;
}

// Features

OSD_Entry menuFeaturesEntries[] =
{
    {"--- FEATURES ---", OME_Label, NULL, NULL, 0},
    {"BLACKBOX", OME_Submenu, cmsMenuChange, &cmsx_menuBlackbox, 0},
#if defined(VTX) || defined(USE_RTC6705)
    {"VTX", OME_Submenu, cmsMenuChange, &cmsx_menuVtx, 0},
#endif // VTX || USE_RTC6705
#ifdef LED_STRIP
    {"LED STRIP", OME_Submenu, cmsMenuChange, &cmsx_menuLedstrip, 0},
#endif // LED_STRIP
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu menuFeatures = {
    "MENUFEATURES",
    OME_MENU,
    NULL,
    NULL,
    NULL,
    menuFeaturesEntries,
};

// Main

OSD_Entry menuMainEntries[] =
{
    {"--- MAIN MENU ---", OME_Label, NULL, NULL, 0},
    {"CFG&IMU", OME_Submenu, cmsMenuChange, &cmsx_menuImu, 0},
    {"FEATURES", OME_Submenu, cmsMenuChange, &menuFeatures, 0},
#ifdef OSD
    {"SCR LAYOUT", OME_Submenu, cmsMenuChange, &cmsx_menuOsdLayout, 0},
    {"ALARMS", OME_Submenu, cmsMenuChange, &cmsx_menuAlarms, 0},
#endif
    {"FC&FW INFO", OME_Submenu, cmsMenuChange, &menuInfo, 0},
    {"FC&FW INFO1", OME_Submenu, cmsMenuChange, &menuInfo, 0},
    {"FC&FW INFO2", OME_Submenu, cmsMenuChange, &menuInfo, 0},
    {"SAVE&REBOOT", OME_OSD_Exit, cmsMenuExit, (void*)1, 0},
    {"EXIT", OME_OSD_Exit, cmsMenuExit, (void*)0, 0},
    {NULL,OME_END, NULL, NULL, 0}
};

CMS_Menu menuMain = {
    "MENUMAIN",
    OME_MENU,
    NULL,
    NULL,
    NULL,
    menuMainEntries,
};

#if 0
long cmsx_FeatureRead(void)
{
    cmsx_Blackbox_FeatureRead();

#ifdef LED_STRIP
    cmsx_Ledstrip_FeatureRead();
    cmsx_Ledstrip_ConfigRead();
#endif

#if defined(VTX) || defined(USE_RTC6705)
    cmsx_Vtx_FeatureRead();
    cmsx_Vtx_ConfigRead();
#endif // VTX || USE_RTC6705

    return 0;
}

long cmsx_FeatureWriteback(void)
{
    cmsx_Blackbox_FeatureWriteback();

#ifdef LED_STRIP
    cmsx_Ledstrip_FeatureWriteback();
#endif

#if defined(VTX) || defined(USE_RTC6705)
    cmsx_Vtx_FeatureWriteback();
    cmsx_Vtx_ConfigWriteback();
#endif // VTX || USE_RTC6705

    saveConfigAndNotify();

    return 0;
}
#endif

#endif // CMS
