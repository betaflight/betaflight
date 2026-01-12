/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_FB_OSD

#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/fb_osd_impl.h"
#include "drivers/osd.h"

#include "config/config.h"

#include "fc/runtime_config.h"

#include "io/displayport_fb_osd.h"

#include "osd/osd.h"

#include "pg/displayport_profiles.h"
//#include "pg/fb_osd.h"
#include "pg/vcd.h"

static displayPort_t fbOsdDisplayPort;
static vcdProfile_t const *fbOsdVcdProfile;
static bool fbOsdDeviceDetected;

static int grab(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int release(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int clearScreen(displayPort_t *displayPort, displayClearOption_e options)
{
    UNUSED(displayPort);
    UNUSED(options);
    fbOsdClearScreen();
    return 0;
}

// Return true if drawScreen still in progress
static bool drawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return fbOsdDrawScreen();
}

static int screenSize(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0; // Not used in BF. Could return 30*16 if appropriate.
}

static int writeString(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, const char *text)
{
    UNUSED(displayPort);
    fbOsdWrite(x, y, attr, text);
    return 0;
}

static int writeChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
{
    UNUSED(displayPort);
    fbOsdWriteChar(x, y, attr, c);
    return 0;
}

static bool isTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return fbOsdBufferInUse();
}

static bool isSynced(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return true; // N/A
}

static void redraw(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    if (!ARMING_FLAG(ARMED)) {
        fbOsdRefreshAll();
    }
}

static int heartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    // (Re)Initialize at startup or if error state detected.
    return fbOsdReInitIfRequired(false);
}

static uint32_t txBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

static bool layerSupported(displayPort_t *displayPort, displayPortLayer_e layer)
{
    UNUSED(displayPort);
    return fbOsdLayerSupported(layer);
}

static bool layerSelect(displayPort_t *displayPort, displayPortLayer_e layer)
{
    UNUSED(displayPort);
    return fbOsdLayerSelect(layer);
}

static bool layerCopy(displayPort_t *displayPort, displayPortLayer_e destLayer, displayPortLayer_e sourceLayer)
{
    UNUSED(displayPort);
    return fbOsdLayerCopy(destLayer, sourceLayer);
}

static bool writeFontCharacter(displayPort_t *displayPort, uint16_t addr, const osdCharacter_t *chr)
{
    UNUSED(displayPort);
    return fbOsdWriteFontCharacter(addr, (const uint8_t *)chr);
}

static void fontUpdateCompletion(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    fbOsdFontUpdateCompletion();
}


static bool checkReady(displayPort_t *displayPort, bool rescan)
{
    UNUSED(displayPort);
    UNUSED(rescan);
    if (!fbOsdDeviceDetected) { // haven't yet returned OK from fbOsdInit function...
        // Try to initialize the device
        if (fbOsdInit(NULL /* fbOsdConfig() */, fbOsdVcdProfile) != FB_OSD_INIT_OK) {
            return false;
        }
        // At this point the device has been initialized and detected
        fbOsdDeviceDetected = true;
        fbOsdDisplayPort.rows = fbOsdGetRowsCount() + displayPortProfileFbOsd()->rowAdjust;
        redraw(&fbOsdDisplayPort);
    }

    return true;
}

static void setBackgroundType(displayPort_t *displayPort, displayPortBackground_e backgroundType)
{
    UNUSED(displayPort);
    fbOsdSetBackgroundType(backgroundType);
}

static bool drawOsdItem(displayPort_t *displayPort, uint8_t elemPosX, uint8_t elemPosY, uint8_t /* osd_items_e */ item, bool isBackground)
{
    UNUSED(displayPort);
    return fbOsdDrawItem((osd_items_e)item, elemPosX, elemPosY, isBackground);
}

static void redrawBackground(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    fbOsdRedrawBackground();
}

static const displayPortVTable_t fbOsdVTable = {
    .grab = grab,
    .release = release,
    .clearScreen = clearScreen,
    .drawScreen = drawScreen,
    .screenSize = screenSize,
    .writeString = writeString,
    .writeChar = writeChar,
    .isTransferInProgress = isTransferInProgress,
    .heartbeat = heartbeat,
    .redraw = redraw,
    .isSynced = isSynced,
    .txBytesFree = txBytesFree,
    .layerSupported = layerSupported,
    .layerSelect = layerSelect,
    .layerCopy = layerCopy,
    .writeFontCharacter = writeFontCharacter,
    .fontUpdateCompletion = fontUpdateCompletion,
    .checkReady = checkReady,
    .setBackgroundType = setBackgroundType,
    .drawOsdItem = drawOsdItem,
    .redrawBackground = redrawBackground,
};

bool fbOsdDisplayPortInit(const vcdProfile_t *vcdProfile, displayPort_t **displayPort)
{
    fbOsdVcdProfile = vcdProfile;

    fbOsdInitStatus_e initResult = fbOsdInit(NULL /* fbOsdConfig() */ , fbOsdVcdProfile);

    if (initResult == FB_OSD_INIT_NOT_CONFIGURED) {
        // fb device IO pins are not defined. We either don't have
        // it on board or either the configuration for it has
        // not been set.
        *displayPort = NULL;
        return false;
    }

    uint8_t displayRows;

    switch(vcdProfile->video_system) {
    default:
    case VIDEO_SYSTEM_PAL:
        displayRows = VIDEO_LINES_PAL;
        break;

    case VIDEO_SYSTEM_NTSC:
        displayRows = VIDEO_LINES_NTSC;
        break;

    case VIDEO_SYSTEM_AUTO:
        displayRows = fbOsdGetRowsCount();
        break;
    }

    *displayPort = &fbOsdDisplayPort;

    // (compare with) max7456 allows row and col adjust to be changed in settings in order
    // to reduce the character grid, so can reduce rows by up to 3, cols by up to 6
    fbOsdDisplayPort.rows = displayRows + displayPortProfileFbOsd()->rowAdjust;
    fbOsdDisplayPort.cols = 30 + displayPortProfileFbOsd()->colAdjust;

    displayInit(&fbOsdDisplayPort, &fbOsdVTable, DISPLAYPORT_DEVICE_TYPE_FBOSD);

    fbOsdDeviceDetected = initResult == FB_OSD_INIT_OK;
    // could be FB_OSD_INIT_INITIALISING, in which case
    // fb device IO pins are defined, but it's not fully up and running.
    // Delay full initialization to checkReady() with 'rescan' enabled.

    return true;
}
#endif // USE_FB_OSD
