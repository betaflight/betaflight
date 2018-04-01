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
 Original OSD code created by Marcin Baliniak
 OSD-CMS separation by jflyper
 CMS-displayPort separation by jflyper and martinbudden
 */

//#define CMS_PAGE_DEBUG // For multi-page/menu debugging
//#define CMS_MENU_DEBUG // For external menu content creators

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_menu_builtin.h"
#include "cms/cms_types.h"

#include "common/maths.h"
#include "common/typeconversion.h"

#include "drivers/system.h"
#include "drivers/time.h"

// For rcData, stopAllMotors, stopPwmAllMotors
#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

// For 'ARM' related
#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"

// For VISIBLE*
#include "io/osd.h"
#include "io/rcdevice_cam.h"

#include "rx/rx.h"

// DisplayPort management

#ifndef CMS_MAX_DEVICE
#define CMS_MAX_DEVICE 4
#endif

displayPort_t *pCurrentDisplay;

static displayPort_t *cmsDisplayPorts[CMS_MAX_DEVICE];
static int cmsDeviceCount;
static int cmsCurrentDevice = -1;

bool cmsDisplayPortRegister(displayPort_t *pDisplay)
{
    if (cmsDeviceCount == CMS_MAX_DEVICE)
        return false;

    cmsDisplayPorts[cmsDeviceCount++] = pDisplay;

    return true;
}

static displayPort_t *cmsDisplayPortSelectCurrent(void)
{
    if (cmsDeviceCount == 0)
        return NULL;

    if (cmsCurrentDevice < 0)
        cmsCurrentDevice = 0;

    return cmsDisplayPorts[cmsCurrentDevice];
}

static displayPort_t *cmsDisplayPortSelectNext(void)
{
    if (cmsDeviceCount == 0)
        return NULL;

    cmsCurrentDevice = (cmsCurrentDevice + 1) % cmsDeviceCount; // -1 Okay

    return cmsDisplayPorts[cmsCurrentDevice];
}

#define CMS_UPDATE_INTERVAL_US  50000   // Interval of key scans (microsec)
#define CMS_POLL_INTERVAL_US   100000   // Interval of polling dynamic values (microsec)

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
// Spektrum SRXL Telemtry Textgenerator
//   13 cols x 9 rows, top row printed as a Bold Heading
//   Needs the "smallScreen" adaptions



#define NORMAL_SCREEN_MIN_COLS 18      // Less is a small screen
static bool    smallScreen;
static uint8_t leftMenuColumn;
static uint8_t rightMenuColumn;
static uint8_t maxMenuItems;
static uint8_t linesPerMenuItem;

bool cmsInMenu = false;

typedef struct cmsCtx_s {
    const CMS_Menu *menu;         // menu for this context
    uint8_t page;                 // page in the menu
    int8_t cursorRow;             // cursorRow in the page
} cmsCtx_t;

static cmsCtx_t menuStack[10];
static uint8_t menuStackIdx = 0;

static int8_t pageCount;         // Number of pages in the current menu
static OSD_Entry *pageTop;       // First entry for the current page
static uint8_t pageMaxRow;       // Max row in the current page

static cmsCtx_t currentCtx;

#ifdef CMS_MENU_DEBUG // For external menu content creators

static char menuErrLabel[21 + 1] = "RANDOM DATA";

static OSD_Entry menuErrEntries[] = {
    { "BROKEN MENU", OME_Label, NULL, NULL, 0 },
    { menuErrLabel, OME_Label, NULL, NULL, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu menuErr = {
    "MENUERR",
    OME_MENU,
    NULL,
    NULL,
    NULL,
    menuErrEntries,
};
#endif

#ifdef CMS_PAGE_DEBUG
#define cmsPageDebug() { \
    debug[0] = pageCount; \
    debug[1] = currentCtx.page; \
    debug[2] = pageMaxRow; \
    debug[3] = currentCtx.cursorRow; } struct _dummy
#else
#define cmsPageDebug()
#endif

static void cmsUpdateMaxRow(displayPort_t *instance)
{
    UNUSED(instance);
    pageMaxRow = 0;

    for (const OSD_Entry *ptr = pageTop; ptr->type != OME_END; ptr++) {
        pageMaxRow++;
    }

    if (pageMaxRow > maxMenuItems) {
        pageMaxRow = maxMenuItems;
    }

    pageMaxRow--;
}

static uint8_t cmsCursorAbsolute(displayPort_t *instance)
{
    UNUSED(instance);
    return currentCtx.cursorRow + currentCtx.page * maxMenuItems;
}

static void cmsPageSelect(displayPort_t *instance, int8_t newpage)
{
    currentCtx.page = (newpage + pageCount) % pageCount;
    pageTop = &currentCtx.menu->entries[currentCtx.page * maxMenuItems];
    cmsUpdateMaxRow(instance);
    displayClearScreen(instance);
}

static void cmsPageNext(displayPort_t *instance)
{
    cmsPageSelect(instance, currentCtx.page + 1);
}

static void cmsPagePrev(displayPort_t *instance)
{
    cmsPageSelect(instance, currentCtx.page - 1);
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

// CMS on OSD legacy was to use LEFT aligned values, not the RIGHT way ;-)
#define CMS_OSD_RIGHT_ALIGNED_VALUES

#ifndef CMS_OSD_RIGHT_ALIGNED_VALUES

// Pad buffer to the left, i.e. align left
static void cmsPadRightToSize(char *buf, int size)
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
#endif

// Pad buffer to the left, i.e. align right
static void cmsPadLeftToSize(char *buf, int size)
{
    int i,j;
    int len = strlen(buf);

    for (i = size - 1, j = size - len ; i - j >= 0 ; i--) {
      buf[i] = buf[i - j];
    }

    for ( ; i >= 0 ; i--) {
      buf[i] = ' ';
    }

    buf[size] = 0;
}

static void cmsPadToSize(char *buf, int size)
{
    // Make absolutely sure the string terminated.
    buf[size] = 0x00,

#ifdef CMS_OSD_RIGHT_ALIGNED_VALUES
    cmsPadLeftToSize(buf, size);
#else
    smallScreen ? cmsPadLeftToSize(buf, size) : cmsPadRightToSize(buf, size);
#endif
}

static int cmsDrawMenuItemValue(displayPort_t *pDisplay, char *buff, uint8_t row, uint8_t maxSize)
{
    int colpos;
    int cnt;

    cmsPadToSize(buff, maxSize);
#ifdef CMS_OSD_RIGHT_ALIGNED_VALUES
    colpos = rightMenuColumn - maxSize;
#else
    colpos = smallScreen ? rightMenuColumn - maxSize : rightMenuColumn;
#endif
    cnt = displayWrite(pDisplay, colpos, row, buff);
    return cnt;
}

static int cmsDrawMenuEntry(displayPort_t *pDisplay, OSD_Entry *p, uint8_t row)
{
    #define CMS_DRAW_BUFFER_LEN 12
    #define CMS_NUM_FIELD_LEN 5

    char buff[CMS_DRAW_BUFFER_LEN +1]; // Make room for null terminator.
    int cnt = 0;

    if (smallScreen) {
        row++;
    }

    switch (p->type) {
    case OME_String:
        if (IS_PRINTVALUE(p) && p->data) {
            strncpy(buff, p->data, CMS_DRAW_BUFFER_LEN);
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_DRAW_BUFFER_LEN);
            CLR_PRINTVALUE(p);
        }
        break;

    case OME_Submenu:
    case OME_Funcall:
        if (IS_PRINTVALUE(p))  {

            buff[0]= 0x0;

            if ((p->type == OME_Submenu) && p->func && (p->flags & OPTSTRING)) {

                // Special case of sub menu entry with optional value display.

                char *str = ((CMSMenuOptFuncPtr)p->func)();
                strncpy( buff, str, CMS_DRAW_BUFFER_LEN);
            }
            strncat(buff, ">", CMS_DRAW_BUFFER_LEN);

            row = smallScreen  ? row - 1  : row;
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, strlen(buff));
            CLR_PRINTVALUE(p);
        }
        break;

    case OME_Bool:
        if (IS_PRINTVALUE(p) && p->data) {
            if (*((uint8_t *)(p->data))) {
              strcpy(buff, "YES");
            } else {
              strcpy(buff, "NO ");
            }

            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, 3);
            CLR_PRINTVALUE(p);
        }
        break;

    case OME_TAB:
        if (IS_PRINTVALUE(p)) {
            OSD_TAB_t *ptr = p->data;
            char * str = (char *)ptr->names[*ptr->val];
            strncpy(buff, str, CMS_DRAW_BUFFER_LEN);
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_DRAW_BUFFER_LEN);
            CLR_PRINTVALUE(p);
        }
        break;

#ifdef USE_OSD
    case OME_VISIBLE:
        if (IS_PRINTVALUE(p) && p->data) {
            uint16_t *val = (uint16_t *)p->data;

            if (VISIBLE(*val)) {
                strcpy(buff, "YES");
            } else {
              strcpy(buff, "NO ");
            }
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, 3);
            CLR_PRINTVALUE(p);
        }
        break;
#endif

    case OME_UINT8:
        if (IS_PRINTVALUE(p) && p->data) {
            OSD_UINT8_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
            CLR_PRINTVALUE(p);
        }
        break;

    case OME_INT8:
        if (IS_PRINTVALUE(p) && p->data) {
            OSD_INT8_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
            CLR_PRINTVALUE(p);
        }
        break;

    case OME_UINT16:
        if (IS_PRINTVALUE(p) && p->data) {
            OSD_UINT16_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
            CLR_PRINTVALUE(p);
        }
        break;

    case OME_INT16:
        if (IS_PRINTVALUE(p) && p->data) {
            OSD_UINT16_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
            CLR_PRINTVALUE(p);
        }
        break;

    case OME_FLOAT:
        if (IS_PRINTVALUE(p) && p->data) {
            OSD_FLOAT_t *ptr = p->data;
            cmsFormatFloat(*ptr->val * ptr->multipler, buff);
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
            CLR_PRINTVALUE(p);
        }
        break;

    case OME_Label:
        if (IS_PRINTVALUE(p) && p->data) {
            // A label with optional string, immediately following text
            cnt = displayWrite(pDisplay, leftMenuColumn + 1 + (uint8_t)strlen(p->text), row, p->data);
            CLR_PRINTVALUE(p);
        }
        break;

    case OME_OSD_Exit:
    case OME_END:
    case OME_Back:
        break;

    case OME_MENU:
        // Fall through
    default:
#ifdef CMS_MENU_DEBUG
        // Shouldn't happen. Notify creator of this menu content
#ifdef CMS_OSD_RIGHT_ALIGNED_VALUES
        cnt = displayWrite(pDisplay, rightMenuColumn - 6, row, "BADENT");
#else.
        cnt = displayWrite(pDisplay, rightMenuColumn, row, "BADENT");
#endif
#endif
        break;
    }

    return cnt;
}

static void cmsDrawMenu(displayPort_t *pDisplay, uint32_t currentTimeUs)
{
    if (!pageTop)
        return;

    uint8_t i;
    OSD_Entry *p;
    uint8_t top = smallScreen ? 1 : (pDisplay->rows - pageMaxRow)/2;

    // Polled (dynamic) value display denominator.

    bool drawPolled = false;
    static uint32_t lastPolledUs = 0;

    if (currentTimeUs > lastPolledUs + CMS_POLL_INTERVAL_US) {
        drawPolled = true;
        lastPolledUs = currentTimeUs;
    }

    uint32_t room = displayTxBytesFree(pDisplay);

    if (pDisplay->cleared) {
        for (p = pageTop, i= 0; p->type != OME_END; p++, i++) {
            SET_PRINTLABEL(p);
            SET_PRINTVALUE(p);
        }
        pDisplay->cleared = false;
    } else if (drawPolled) {
        for (p = pageTop ; p <= pageTop + pageMaxRow ; p++) {
            if (IS_DYNAMIC(p))
                SET_PRINTVALUE(p);
        }
    }

    // Cursor manipulation

    while ((pageTop + currentCtx.cursorRow)->type == OME_Label) // skip label
        currentCtx.cursorRow++;

    cmsPageDebug();

    if (pDisplay->cursorRow >= 0 && currentCtx.cursorRow != pDisplay->cursorRow) {
        room -= displayWrite(pDisplay, leftMenuColumn, top + pDisplay->cursorRow * linesPerMenuItem, " ");
    }

    if (room < 30)
        return;

    if (pDisplay->cursorRow != currentCtx.cursorRow) {
        room -= displayWrite(pDisplay, leftMenuColumn, top + currentCtx.cursorRow * linesPerMenuItem, ">");
        pDisplay->cursorRow = currentCtx.cursorRow;
    }

    if (room < 30)
        return;

    // Print text labels
    for (i = 0, p = pageTop; i < maxMenuItems && p->type != OME_END; i++, p++) {
        if (IS_PRINTLABEL(p)) {
            uint8_t coloff = leftMenuColumn;
            coloff += (p->type == OME_Label) ? 0 : 1;
            room -= displayWrite(pDisplay, coloff, top + i * linesPerMenuItem, p->text);
            CLR_PRINTLABEL(p);
            if (room < 30)
                return;
        }

    // Print values

    // XXX Polled values at latter positions in the list may not be
    // XXX printed if not enough room in the middle of the list.

        if (IS_PRINTVALUE(p)) {
            room -= cmsDrawMenuEntry(pDisplay, p, top + i * linesPerMenuItem);
            if (room < 30)
                return;
        }
    }
}

static void cmsMenuCountPage(displayPort_t *pDisplay)
{
    UNUSED(pDisplay);
    const OSD_Entry *p;
    for (p = currentCtx.menu->entries; p->type != OME_END; p++);
    pageCount = (p - currentCtx.menu->entries - 1) / maxMenuItems + 1;
}

STATIC_UNIT_TESTED long cmsMenuBack(displayPort_t *pDisplay); // Forward; will be resolved after merging

long cmsMenuChange(displayPort_t *pDisplay, const void *ptr)
{
    const CMS_Menu *pMenu = (const CMS_Menu *)ptr;

    if (!pMenu) {
        return 0;
    }

#ifdef CMS_MENU_DEBUG
    if (pMenu->GUARD_type != OME_MENU) {
        // ptr isn't pointing to a CMS_Menu.
        if (pMenu->GUARD_type <= OME_MAX) {
            strncpy(menuErrLabel, pMenu->GUARD_text, sizeof(menuErrLabel) - 1);
        } else {
            strncpy(menuErrLabel, "LABEL UNKNOWN", sizeof(menuErrLabel) - 1);
        }
        pMenu = &menuErr;
    }
#endif

    if (pMenu != currentCtx.menu) {
        // Stack the current menu and move to a new menu.

        menuStack[menuStackIdx++] = currentCtx;

        currentCtx.menu = pMenu;
        currentCtx.cursorRow = 0;

        if (pMenu->onEnter && (pMenu->onEnter() == MENU_CHAIN_BACK)) {
            return cmsMenuBack(pDisplay);
        }

        cmsMenuCountPage(pDisplay);
        cmsPageSelect(pDisplay, 0);
    } else {
        // The (pMenu == curretMenu) case occurs when reopening for display cycling
        // currentCtx.cursorRow has been saved as absolute; convert it back to page + relative

        int8_t cursorAbs = currentCtx.cursorRow;
        currentCtx.cursorRow = cursorAbs % maxMenuItems;
        cmsMenuCountPage(pDisplay);
        cmsPageSelect(pDisplay, cursorAbs / maxMenuItems);
    }

    cmsPageDebug();

    return 0;
}

STATIC_UNIT_TESTED long cmsMenuBack(displayPort_t *pDisplay)
{
    // Let onExit function decide whether to allow exit or not.

    if (currentCtx.menu->onExit && currentCtx.menu->onExit(pageTop + currentCtx.cursorRow) < 0) {
        return -1;
    }

    if (!menuStackIdx) {
        return 0;
    }

    currentCtx = menuStack[--menuStackIdx];

    cmsMenuCountPage(pDisplay);
    cmsPageSelect(pDisplay, currentCtx.page);

    cmsPageDebug();

    return 0;
}

STATIC_UNIT_TESTED void cmsMenuOpen(void)
{
    if (!cmsInMenu) {
        // New open
        pCurrentDisplay = cmsDisplayPortSelectCurrent();
        if (!pCurrentDisplay)
            return;
        cmsInMenu = true;
        currentCtx = (cmsCtx_t){ &menuMain, 0, 0 };
        setArmingDisabled(ARMING_DISABLED_CMS_MENU);
    } else {
        // Switch display
        displayPort_t *pNextDisplay = cmsDisplayPortSelectNext();
        if (pNextDisplay != pCurrentDisplay) {
            // DisplayPort has been changed.
            // Convert cursorRow to absolute value
            currentCtx.cursorRow = cmsCursorAbsolute(pCurrentDisplay);
            displayRelease(pCurrentDisplay);
            pCurrentDisplay = pNextDisplay;
        } else {
            return;
        }
    }
    displayGrab(pCurrentDisplay); // grab the display for use by the CMS

    if ( pCurrentDisplay->cols < NORMAL_SCREEN_MIN_COLS) {
      smallScreen       = true;
      linesPerMenuItem  = 2;
      leftMenuColumn    = 0;
      rightMenuColumn   = pCurrentDisplay->cols;
      maxMenuItems      = (pCurrentDisplay->rows) / linesPerMenuItem;
    } else {
      smallScreen       = false;
      linesPerMenuItem  = 1;
      leftMenuColumn    = 2;
#ifdef CMS_OSD_RIGHT_ALIGNED_VALUES
      rightMenuColumn   = pCurrentDisplay->cols - 2;
#else
      rightMenuColumn   = pCurrentDisplay->cols - CMS_DRAW_BUFFER_LEN;
#endif
      maxMenuItems      = pCurrentDisplay->rows - 2;
    }

    cmsMenuChange(pCurrentDisplay, currentCtx.menu);
}

static void cmsTraverseGlobalExit(const CMS_Menu *pMenu)
{
    for (const OSD_Entry *p = pMenu->entries; p->type != OME_END ; p++) {
        if (p->type == OME_Submenu) {
            cmsTraverseGlobalExit(p->data);
        }
    }

}

long cmsMenuExit(displayPort_t *pDisplay, const void *ptr)
{
    int exitType = (int)ptr;
    switch (exitType) {
    case CMS_EXIT_SAVE:
    case CMS_EXIT_SAVEREBOOT:

        cmsTraverseGlobalExit(&menuMain);

        if (currentCtx.menu->onExit)
            currentCtx.menu->onExit((OSD_Entry *)NULL); // Forced exit

        saveConfigAndNotify();
        break;

    case CMS_EXIT:
        break;
    }

    cmsInMenu = false;

    displayRelease(pDisplay);
    currentCtx.menu = NULL;

    if (exitType == CMS_EXIT_SAVEREBOOT) {
        displayClearScreen(pDisplay);
        displayWrite(pDisplay, 5, 3, "REBOOTING...");

        displayResync(pDisplay); // Was max7456RefreshAll(); why at this timing?

        stopMotors();
        stopPwmAllMotors();
        delay(200);

        systemReset();
    }

    unsetArmingDisabled(ARMING_DISABLED_CMS_MENU);

    return 0;
}

// Stick/key detection and key codes

#define IS_HI(X)  (rcData[X] > 1750)
#define IS_LO(X)  (rcData[X] < 1250)
#define IS_MID(X) (rcData[X] > 1250 && rcData[X] < 1750)

#define KEY_NONE    0
#define KEY_UP      1
#define KEY_DOWN    2
#define KEY_LEFT    3
#define KEY_RIGHT   4
#define KEY_ESC     5
#define KEY_MENU    6

#define BUTTON_TIME   250 // msec
#define BUTTON_PAUSE  500 // msec

STATIC_UNIT_TESTED uint16_t cmsHandleKey(displayPort_t *pDisplay, uint8_t key)
{
    uint16_t res = BUTTON_TIME;
    OSD_Entry *p;

    if (!currentCtx.menu)
        return res;

    if (key == KEY_MENU) {
        cmsMenuOpen();
        return BUTTON_PAUSE;
    }

    if (key == KEY_ESC) {
        cmsMenuBack(pDisplay);
        return BUTTON_PAUSE;
    }

    if (key == KEY_DOWN) {
        if (currentCtx.cursorRow < pageMaxRow) {
            currentCtx.cursorRow++;
        } else {
            cmsPageNext(pDisplay);
            currentCtx.cursorRow = 0;    // Goto top in any case
        }
    }

    if (key == KEY_UP) {
        currentCtx.cursorRow--;

        // Skip non-title labels
        if ((pageTop + currentCtx.cursorRow)->type == OME_Label && currentCtx.cursorRow > 0)
            currentCtx.cursorRow--;

        if (currentCtx.cursorRow == -1 || (pageTop + currentCtx.cursorRow)->type == OME_Label) {
            // Goto previous page
            cmsPagePrev(pDisplay);
            currentCtx.cursorRow = pageMaxRow;
        }
    }

    if (key == KEY_DOWN || key == KEY_UP)
        return res;

    p = pageTop + currentCtx.cursorRow;

    switch (p->type) {
        case OME_Submenu:
            if (key == KEY_RIGHT) {
                cmsMenuChange(pDisplay, p->data);
                res = BUTTON_PAUSE;
            }
            break;

        case OME_Funcall:;
            long retval;
            if (p->func && key == KEY_RIGHT) {
                retval = p->func(pDisplay, p->data);
                if (retval == MENU_CHAIN_BACK)
                    cmsMenuBack(pDisplay);
                res = BUTTON_PAUSE;
            }
            break;

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

#ifdef USE_OSD
        case OME_VISIBLE:
            if (p->data) {
                uint16_t *val = (uint16_t *)p->data;

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
                if (p->func) {
                    p->func(pDisplay, p);
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
                if (p->func) {
                    p->func(pDisplay, p);
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
                SET_PRINTVALUE(p);
                if (p->func) {
                    p->func(pDisplay, p);
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
                SET_PRINTVALUE(p);
                if (p->func) {
                    p->func(pDisplay, p);
                }
            }
            break;

        case OME_String:
            break;

        case OME_Label:
        case OME_END:
            break;

        case OME_MENU:
            // Shouldn't happen
            break;
    }
    return res;
}

uint16_t cmsHandleKeyWithRepeat(displayPort_t *pDisplay, uint8_t key, int repeatCount)
{
    uint16_t ret = 0;

    for (int i = 0 ; i < repeatCount ; i++) {
        ret = cmsHandleKey(pDisplay, key);
    }

    return ret;
}

void cmsUpdate(uint32_t currentTimeUs)
{
#ifdef USE_RCDEVICE
    if(rcdeviceInMenu) {
        return ;
    }
#endif

    static int16_t rcDelayMs = BUTTON_TIME;
    static int holdCount = 1;
    static int repeatCount = 1;
    static int repeatBase = 0;

    static uint32_t lastCalledMs = 0;
    static uint32_t lastCmsHeartBeatMs = 0;

    const uint32_t currentTimeMs = currentTimeUs / 1000;

    if (!cmsInMenu) {
        // Detect menu invocation
        if (IS_MID(THROTTLE) && IS_LO(YAW) && IS_HI(PITCH) && !ARMING_FLAG(ARMED)) {
            cmsMenuOpen();
            rcDelayMs = BUTTON_PAUSE;    // Tends to overshoot if BUTTON_TIME
        }
    } else {
        //
        // Scan 'key' first
        //

        uint8_t key = KEY_NONE;

        if (IS_MID(THROTTLE) && IS_LO(YAW) && IS_HI(PITCH) && !ARMING_FLAG(ARMED)) {
            key = KEY_MENU;
        }
        else if (IS_HI(PITCH)) {
            key = KEY_UP;
        }
        else if (IS_LO(PITCH)) {
            key = KEY_DOWN;
        }
        else if (IS_LO(ROLL)) {
            key = KEY_LEFT;
        }
        else if (IS_HI(ROLL)) {
            key = KEY_RIGHT;
        }
        else if (IS_HI(YAW) || IS_LO(YAW))
        {
            key = KEY_ESC;
        }

        if (key == KEY_NONE) {
            // No 'key' pressed, reset repeat control
            holdCount = 1;
            repeatCount = 1;
            repeatBase = 0;
        } else {
            // The 'key' is being pressed; keep counting
            ++holdCount;
        }

        if (rcDelayMs > 0) {
            rcDelayMs -= (currentTimeMs - lastCalledMs);
        } else if (key) {
            rcDelayMs = cmsHandleKeyWithRepeat(pCurrentDisplay, key, repeatCount);

            // Key repeat effect is implemented in two phases.
            // First phldase is to decrease rcDelayMs reciprocal to hold time.
            // When rcDelayMs reached a certain limit (scheduling interval),
            // repeat rate will not raise anymore, so we call key handler
            // multiple times (repeatCount).
            //
            // XXX Caveat: Most constants are adjusted pragmatically.
            // XXX Rewrite this someday, so it uses actual hold time instead
            // of holdCount, which depends on the scheduling interval.

            if (((key == KEY_LEFT) || (key == KEY_RIGHT)) && (holdCount > 20)) {

                // Decrease rcDelayMs reciprocally

                rcDelayMs /= (holdCount - 20);

                // When we reach the scheduling limit,

                if (rcDelayMs <= 50) {

                    // start calling handler multiple times.

                    if (repeatBase == 0)
                        repeatBase = holdCount;

                    repeatCount = repeatCount + (holdCount - repeatBase) / 5;

                    if (repeatCount > 5) {
                        repeatCount= 5;
                    }
                }
            }
        }

        cmsDrawMenu(pCurrentDisplay, currentTimeUs);

        if (currentTimeMs > lastCmsHeartBeatMs + 500) {
            // Heart beat for external CMS display device @ 500msec
            // (Timeout @ 1000msec)
            displayHeartbeat(pCurrentDisplay);
            lastCmsHeartBeatMs = currentTimeMs;
        }
    }

    // Some key (command), notably flash erase, takes too long to use the
    // currentTimeMs to be used as lastCalledMs (freezes CMS for a minute or so
    // if used).
    lastCalledMs = millis();
}

void cmsHandler(timeUs_t currentTimeUs)
{
    if (cmsDeviceCount < 0)
        return;



    static timeUs_t lastCalledUs = 0;

    if (currentTimeUs >= lastCalledUs + CMS_UPDATE_INTERVAL_US) {
        lastCalledUs = currentTimeUs;
        cmsUpdate(currentTimeUs);
    }
}

void cmsInit(void)
{
    cmsDeviceCount = 0;
    cmsCurrentDevice = -1;
}

#endif // CMS
