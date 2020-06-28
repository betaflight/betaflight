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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_MAX7456

#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/max7456.h"
#include "drivers/osd.h"

#include "config/config.h"

#include "io/displayport_max7456.h"

#include "osd/osd.h"

#include "pg/displayport_profiles.h"
#include "pg/max7456.h"
#include "pg/vcd.h"

displayPort_t max7456DisplayPort;

static int grab(displayPort_t *displayPort)
{
    // FIXME this should probably not have a dependency on the OSD or OSD slave code
    UNUSED(displayPort);
#ifdef USE_OSD
    resumeRefreshAt = 0;
#endif

    return 0;
}

static int release(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    return 0;
}

static int clearScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    max7456Invert(displayPortProfileMax7456()->invert);
    max7456Brightness(displayPortProfileMax7456()->blackBrightness, displayPortProfileMax7456()->whiteBrightness);

    max7456ClearScreen();

    return 0;
}

static int drawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    max7456DrawScreen();

    return 0;
}

static int screenSize(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return maxScreenSize;
}

static int writeString(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, const char *s)
{
    UNUSED(displayPort);
    UNUSED(attr);

    max7456Write(x, y, s);

    return 0;
}

static int writeChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
{
    UNUSED(displayPort);
    UNUSED(attr);

    max7456WriteChar(x, y, c);

    return 0;
}

static bool isTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return max7456DmaInProgress();
}

static bool isSynced(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return max7456BuffersSynced();
}

static void resync(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    max7456RefreshAll();
    displayPort->rows = max7456GetRowsCount() + displayPortProfileMax7456()->rowAdjust;
    displayPort->cols = 30 + displayPortProfileMax7456()->colAdjust;
}

static int heartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static uint32_t txBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

static bool layerSupported(displayPort_t *displayPort, displayPortLayer_e layer)
{
    UNUSED(displayPort);
    return max7456LayerSupported(layer);
}

static bool layerSelect(displayPort_t *displayPort, displayPortLayer_e layer)
{
    UNUSED(displayPort);
    return max7456LayerSelect(layer);
}

static bool layerCopy(displayPort_t *displayPort, displayPortLayer_e destLayer, displayPortLayer_e sourceLayer)
{
    UNUSED(displayPort);
    return max7456LayerCopy(destLayer, sourceLayer);
}

static bool writeFontCharacter(displayPort_t *instance, uint16_t addr, const osdCharacter_t *chr)
{
    UNUSED(instance);

    return max7456WriteNvm(addr, (const uint8_t *)chr);
}

static bool isReady(displayPort_t *instance)
{
    UNUSED(instance);

    if (!max7456IsDeviceDetected()) {
        // Try to initialize the device
        if (max7456Init(max7456Config(), vcdProfile(), systemConfig()->cpu_overclock) != MAX7456_INIT_OK) {
            return false;
        }
        // At this point the device has been initialized and detected
        resync(&max7456DisplayPort);
    }
    return true;
}

static const displayPortVTable_t max7456VTable = {
    .grab = grab,
    .release = release,
    .clearScreen = clearScreen,
    .drawScreen = drawScreen,
    .screenSize = screenSize,
    .writeString = writeString,
    .writeChar = writeChar,
    .isTransferInProgress = isTransferInProgress,
    .heartbeat = heartbeat,
    .resync = resync,
    .isSynced = isSynced,
    .txBytesFree = txBytesFree,
    .layerSupported = layerSupported,
    .layerSelect = layerSelect,
    .layerCopy = layerCopy,
    .writeFontCharacter = writeFontCharacter,
    .isReady = isReady,
};

displayPort_t *max7456DisplayPortInit(const vcdProfile_t *vcdProfile)
{
    switch (max7456Init(max7456Config(), vcdProfile, systemConfig()->cpu_overclock)) {
    case MAX7456_INIT_NOT_FOUND:
        // MAX7456 IO pins are defined, but we could not get a reply
        // from it at this time. Delay full initialization to
        // isReady()
        displayInit(&max7456DisplayPort, &max7456VTable);
        break;
    case MAX7456_INIT_OK:
        // MAX7456 configured and detected
        displayInit(&max7456DisplayPort, &max7456VTable);
        resync(&max7456DisplayPort);
        break;
    case MAX7456_INIT_NOT_CONFIGURED:
        // MAX7456 IO pins are not defined. We either don't have
        // it on board or either the configuration for it has
        // not been set.
        return NULL;
    }
    return &max7456DisplayPort;
}
#endif // USE_MAX7456
