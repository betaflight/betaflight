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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RCDEVICE

#include "io/rcdevice.h"

#include "pg/vcd.h"

#include "rcdevice_osd.h"

#define VIDEO_BUFFER_CHARS_PAL 480

static uint8_t columnCount = 30;

static runcamDevice_t runcamOSDDevice;
runcamDevice_t *osdDevice = &runcamOSDDevice;

static uint8_t video_system;
static uint16_t maxScreenSize = VIDEO_BUFFER_CHARS_PAL;

#ifdef USE_PARTICLE_DRAW
#define MAX_CHARS2UPDATE 20
static uint8_t screenBuffer[VIDEO_BUFFER_CHARS_PAL + 40]; // For faster writes
                                                          // we use memcpy so we
                                                          // need some space to
                                                          // don't overwrite
                                                          // buffer
static uint8_t shadowBuffer[VIDEO_BUFFER_CHARS_PAL];
static bool rcdeviceOSDLock = false;
#endif

bool rcdeviceOSDInit(const vcdProfile_t *vcdProfile)
{
    if (!runcamDeviceInit(osdDevice)) {
        return false;
    }

    if ((osdDevice->info.features & RCDEVICE_PROTOCOL_FEATURE_DISPLAYP_PORT) == 0) {
        return false;
    }

    // get screen column count
    runcamDeviceSettingDetail_t settingDetail;
    if (!runcamDeviceGetSettingDetail(osdDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &settingDetail)) {
        return false;
    }

    columnCount = settingDetail.value;

    video_system = vcdProfile->video_system;
    if (video_system == VIDEO_SYSTEM_AUTO) {
        // fetch current video mode from device
        runcamDeviceSettingDetail_t settingDetail;
        if (!runcamDeviceGetSettingDetail(osdDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_TV_MODE, &settingDetail)) {
            return false;
        }
        video_system = settingDetail.value;
    } else {
        // set video system
        runcamDeviceWriteSettingResponse_t response;
        uint8_t tvMode = 0;
        if (video_system == VIDEO_SYSTEM_NTSC) {
            tvMode = 0;
        } else if (video_system == VIDEO_SYSTEM_PAL) {
            tvMode = 1;
        }

        if (!runcamDeviceWriteSetting(osdDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_TV_MODE, &tvMode, sizeof(uint8_t), &response)) {
            return false;
        }

        if (response.resultCode) {
            return false;
        }
    }

    // user bf charset
    uint8_t charsetID = 0;
    runcamDeviceWriteSettingResponse_t updateCharsetResp;
    if (!runcamDeviceWriteSetting(osdDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_CHARSET, &charsetID, sizeof(uint8_t), &updateCharsetResp) || updateCharsetResp.resultCode != 0) {
        return false;
    }

#ifdef USE_PARTICLE_DRAW
    memset(shadowBuffer, 2, VIDEO_BUFFER_CHARS_PAL);
#endif

    // fill screen with ' '
    rcdeviceOSDClearScreen(NULL);
    return true;
}

int rcdeviceOSDGrab(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    // osdResetAlarms();
    // resumeRefreshAt = 0;
    return 0;
}

int rcdeviceOSDRelease(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

int rcdeviceOSDDrawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);

#ifdef USE_PARTICLE_DRAW
    static uint16_t pos = 0;
    int k = 0;

    if (!rcdeviceOSDLock) {
        rcdeviceOSDLock = true;

        uint8_t data[60];
        uint8_t dataLen = 0;
        for (k = 0; k < MAX_CHARS2UPDATE; k++) {
            if (screenBuffer[pos] != shadowBuffer[pos]) {
                shadowBuffer[pos] = screenBuffer[pos];
                uint8_t x = pos % columnCount;
                uint8_t y = pos / columnCount;
                data[dataLen++] = x;
                data[dataLen++] = y;
                data[dataLen++] = screenBuffer[pos];
            }

            if (++pos >= maxScreenSize) {
                pos = 0;
                break;
            }
        }
        runcamDeviceDispWriteChars(osdDevice, data, dataLen);

        rcdeviceOSDLock = false;
    }
#endif
    return 0;
}

int rcdeviceOSDWriteString(displayPort_t *displayPort, uint8_t x, uint8_t y, const char *buff)
{
    UNUSED(displayPort);
#if !defined(USE_PARTICLE_DRAW)
    runcamDeviceDispWriteHorizontalString(osdDevice, x, y, buff);
#else
    for (int xpos = x; *buff && xpos < columnCount; xpos++) {
        screenBuffer[y * columnCount + xpos] = *buff++;
    }
#endif
    return 0;
}

int rcdeviceOSDWriteChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t c)
{
    UNUSED(displayPort);
#if !defined(USE_PARTICLE_DRAW)
    runcamDeviceDispWriteChar(osdDevice, x, y, c);
#else
    screenBuffer[y * columnCount + x] = c;
#endif

    return 0;
}

int rcdeviceOSDClearScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);

#if defined(USE_PARTICLE_DRAW)
    memset(screenBuffer, 0x20, sizeof(screenBuffer));
#else
    runcamDeviceDispFillRegion(osdDevice, 0, 0, 255, 255, ' ');
#endif

    return 0;
}

bool rcdeviceOSDIsTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

bool rcdeviceOSDIsSynced(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return true;
}

int rcdeviceOSDHeartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

void rcdeviceOSDResync(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    if (video_system == VIDEO_SYSTEM_PAL) {
        displayPort->rows = RCDEVICE_PROTOCOL_OSD_VIDEO_LINES_PAL;
    } else {
        displayPort->rows = RCDEVICE_PROTOCOL_OSD_VIDEO_LINES_NTSC;
    }

    displayPort->cols = columnCount;
    maxScreenSize = displayPort->rows * displayPort->cols;
}

uint32_t rcdeviceOSDTxBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return INT32_MAX;
}

int rcdeviceScreenSize(const displayPort_t *displayPort)
{
    return displayPort->rows * displayPort->cols;
}

#endif
