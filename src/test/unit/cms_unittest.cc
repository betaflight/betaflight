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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <limits.h>

#include <math.h>

#define USE_BARO

extern "C" {
    #include "platform.h"
    #include "target.h"
    #include "cms/cms.h"
    #include "cms/cms_types.h"
    #include "fc/runtime_config.h"
    void cmsMenuOpen(void);
    long cmsMenuBack(displayPort_t *pDisplay);
    uint16_t cmsHandleKey(displayPort_t *pDisplay, uint8_t key);
    extern CMS_Menu *currentMenu;    // Points to top entry of the current page
}

#include "unittest_macros.h"
#include "unittest_displayport.h"
#include "gtest/gtest.h"

TEST(CMSUnittest, TestCmsDisplayPortRegister)
{
    cmsInit();
    displayPort_t *displayPort = displayPortTestInit();
    for (int ii = 0; ii < CMS_MAX_DEVICE; ++ii) {
        const bool registered = cmsDisplayPortRegister(displayPort);
        EXPECT_EQ(true, registered);
    }
    const bool registered = cmsDisplayPortRegister(displayPort);
    EXPECT_EQ(false, registered);
}

TEST(CMSUnittest, TestCmsMenuOpen)
{
    cmsInit();
    displayPort_t *displayPort = displayPortTestInit();
    displayGrab(displayPort);
    cmsDisplayPortRegister(displayPort);

    cmsMenuOpen();
}

TEST(CMSUnittest, TestCmsMenuExit0)
{
    cmsInit();
    displayPort_t *displayPort = displayPortTestInit();
    cmsDisplayPortRegister(displayPort);

    cmsMenuOpen();
    long exit = cmsMenuExit(displayPort, (void*)0);
    EXPECT_EQ(0, exit);
}

TEST(CMSUnittest, TestCmsMenuExit1)
{
    cmsInit();
    displayPort_t *displayPort = displayPortTestInit();
    cmsDisplayPortRegister(displayPort);

    cmsMenuOpen();
    long exit = cmsMenuExit(displayPort, (void*)0);
    EXPECT_EQ(0, exit);
}

TEST(CMSUnittest, TestCmsMenuBack)
{
    cmsInit();
    displayPort_t *displayPort = displayPortTestInit();
    cmsDisplayPortRegister(displayPort);

    cmsMenuOpen();
    long exit = cmsMenuBack(displayPort);
    EXPECT_EQ(0, exit);
}

TEST(CMSUnittest, TestCmsMenuKey)
{
#define KEY_ENTER   0
#define KEY_UP      1
#define KEY_DOWN    2
#define KEY_LEFT    3
#define KEY_RIGHT   4
#define KEY_ESC     5
#define BUTTON_TIME   250 // msec
#define BUTTON_PAUSE  500 // msec
    cmsInit();
    displayPort_t *displayPort = &testDisplayPort;
    cmsDisplayPortRegister(displayPort);

    cmsMenuOpen();
    uint16_t result = cmsHandleKey(displayPort, KEY_ESC);
    EXPECT_EQ(BUTTON_PAUSE, result);
}
// STUBS

extern "C" {
static OSD_Entry menuMainEntries[] =
{
    {"-- MAIN MENU --", OME_Label, NULL, NULL, 0},
    {"SAVE&REBOOT", OME_OSD_Exit, cmsMenuExit, (void*)1, 0},
    {"EXIT", OME_OSD_Exit, cmsMenuExit, (void*)0, 0},
    {NULL, OME_END, NULL, NULL, 0}
};
CMS_Menu menuMain = {
#ifdef CMS_MENU_DEBUG
    "MENUMAIN",
    OME_MENU,
#endif
    NULL,
    NULL,
    menuMainEntries,
};
uint8_t armingFlags;
int16_t debug[4];
int16_t rcData[18];
void delay(uint32_t) {}
uint32_t micros(void) { return 0; }
uint32_t millis(void) { return 0; }
void saveConfigAndNotify(void) {}
void stopMotors(void) {}
void stopPwmAllMotors(void) {}
void systemReset(void) {}
void setArmingDisabled(armingDisableFlags_e flag) { UNUSED(flag); }
void unsetArmingDisabled(armingDisableFlags_e flag) { UNUSED(flag); }
}
