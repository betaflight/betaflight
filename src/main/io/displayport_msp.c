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
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_MSP_DISPLAYPORT

#include "cli/cli.h"

#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/osd.h"

#include "io/displayport_msp.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

#include "osd/osd.h"

#include "pg/vcd.h"

static displayPort_t mspDisplayPort;
static serialPortIdentifier_e displayPortSerial;

static int output(displayPort_t *displayPort, uint8_t cmd, uint8_t *buf, int len)
{
    UNUSED(displayPort);

    return mspSerialPush(displayPortSerial, cmd, buf, len, MSP_DIRECTION_REPLY, MSP_V1);
}

static int heartbeat(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { MSP_DP_HEARTBEAT };

    // heartbeat is used to:
    // a) ensure display is not released by MW OSD software
    // b) prevent OSD Slave boards from displaying a 'disconnected' status.
    output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));

    return 0;
}

static int grab(displayPort_t *displayPort)
{
    return heartbeat(displayPort);
}

static int release(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { MSP_DP_RELEASE };

    return output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));
}

static int clearScreen(displayPort_t *displayPort, displayClearOption_e options)
{
    UNUSED(options);

    uint8_t subcmd[] = { MSP_DP_CLEAR_SCREEN };

    return output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));
}

static bool drawScreen(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { MSP_DP_DRAW_SCREEN };
    output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));

    return 0;
}

static int screenSize(const displayPort_t *displayPort)
{
    return displayPort->rows * displayPort->cols;
}

static int writeString(displayPort_t *displayPort, uint8_t col, uint8_t row, uint8_t attr, const char *string)
{
#define MSP_OSD_MAX_STRING_LENGTH 30 // FIXME move this
    uint8_t buf[MSP_OSD_MAX_STRING_LENGTH + 4];

    int len = strlen(string);
    if (len >= MSP_OSD_MAX_STRING_LENGTH) {
        len = MSP_OSD_MAX_STRING_LENGTH;
    }

    buf[0] = MSP_DP_WRITE_STRING;
    buf[1] = row;
    buf[2] = col;
    buf[3] = displayPortProfileMsp()->fontSelection[attr & (DISPLAYPORT_SEVERITY_COUNT - 1)] & DISPLAYPORT_MSP_ATTR_FONT;

    if (attr & DISPLAYPORT_BLINK) {
        buf[3] |= DISPLAYPORT_MSP_ATTR_BLINK;
    }

    memcpy(&buf[4], string, len);

    return output(displayPort, MSP_DISPLAYPORT, buf, len + 4);
}

static int writeSys(displayPort_t *displayPort, uint8_t col, uint8_t row, displayPortSystemElement_e systemElement)
{
    uint8_t syscmd[4];

    syscmd[0] = MSP_DP_SYS;
    syscmd[1] = row;
    syscmd[2] = col;
    syscmd[3] = systemElement;

    return output(displayPort, MSP_DISPLAYPORT, syscmd, sizeof(syscmd));
}

static int writeChar(displayPort_t *displayPort, uint8_t col, uint8_t row, uint8_t attr, uint8_t c)
{
    char buf[2];

    buf[0] = c;
    buf[1] = 0;
    return writeString(displayPort, col, row, attr, buf);
}

static bool isTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

static bool isSynced(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return true;
}

static void redraw(displayPort_t *displayPort)
{
    drawScreen(displayPort);
}

static uint32_t txBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return mspSerialTxBytesFree();
}

static const displayPortVTable_t mspDisplayPortVTable = {
    .grab = grab,
    .release = release,
    .clearScreen = clearScreen,
    .drawScreen = drawScreen,
    .screenSize = screenSize,
    .writeSys = writeSys,
    .writeString = writeString,
    .writeChar = writeChar,
    .isTransferInProgress = isTransferInProgress,
    .heartbeat = heartbeat,
    .redraw = redraw,
    .isSynced = isSynced,
    .txBytesFree = txBytesFree,
    .layerSupported = NULL,
    .layerSelect = NULL,
    .layerCopy = NULL,
};

displayPort_t *displayPortMspInit(void)
{
    displayInit(&mspDisplayPort, &mspDisplayPortVTable, DISPLAYPORT_DEVICE_TYPE_MSP);

    if (displayPortProfileMsp()->useDeviceBlink) {
        mspDisplayPort.useDeviceBlink = true;
    }

#ifndef USE_OSD_SD
    if (vcdProfile()->video_system != VIDEO_SYSTEM_HD) {
        vcdProfileMutable()->video_system = VIDEO_SYSTEM_HD;
    }
#endif
#ifndef USE_OSD_HD
    if (vcdProfile()->video_system == VIDEO_SYSTEM_HD) {
        vcdProfileMutable()->video_system = VIDEO_SYSTEM_AUTO;
    }
#endif

    if (vcdProfile()->video_system == VIDEO_SYSTEM_HD) {
        mspDisplayPort.rows = osdConfig()->canvas_rows;
        mspDisplayPort.cols = osdConfig()->canvas_cols;
    } else {
        const uint8_t displayRows = (vcdProfile()->video_system == VIDEO_SYSTEM_PAL) ? VIDEO_LINES_PAL : VIDEO_LINES_NTSC;
        mspDisplayPort.rows = displayRows + displayPortProfileMsp()->rowAdjust;
        mspDisplayPort.cols = OSD_SD_COLS + displayPortProfileMsp()->colAdjust;
    }

    redraw(&mspDisplayPort);

    return &mspDisplayPort;
}

void displayPortMspSetSerial(serialPortIdentifier_e serialPort)
{
    displayPortSerial = serialPort;
}

serialPortIdentifier_e displayPortMspGetSerial(void) {
    return displayPortSerial;
}
#endif // USE_MSP_DISPLAYPORT
