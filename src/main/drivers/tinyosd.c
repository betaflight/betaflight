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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tinyosd.h"

#include "common/maths.h"
#include "common/time.h"
#include "io/osd.h"
#include "io/displayport_tinyosd.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "drivers/time.h"
#include "drivers/vcd.h"
#include "drivers/opentco.h"
#include "sensors/gyroanalyse.h"

uint16_t tinyOSD_maxScreenSize = TINYOSD_VIDEO_BUFFER_CHARS_PAL;

static uint8_t  video_system;

static sbuf_t tinyOSDStreamBuffer;
static sbuf_t *sbuf = &tinyOSDStreamBuffer;

static void tinyOSDSetRegister(uint8_t reg, uint8_t value);

int tinyOSDGrab(displayPort_t * displayPort)
{
    UNUSED(displayPort);
    osdResetAlarms();
    resumeRefreshAt = 0;
    return 0;
}

int tinyOSDRelease(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

void tinyOSDReInit(void)
{
    static bool firstInit = true;

    if (video_system == VIDEO_SYSTEM_AUTO) {
        // fetch video mode from tinyOSD FIXME
        video_system = VIDEO_SYSTEM_NTSC;
    } else {
        // set video system
        tinyOSDSetRegister(OPENTCO_OSD_REGISTER_VIDEO_FORMAT, video_system);
    }

    // FIXME: feth this info from device
    if (video_system == VIDEO_SYSTEM_PAL) {
        tinyOSD_maxScreenSize = TINYOSD_VIDEO_BUFFER_CHARS_PAL;
    } else {              // NTSC
        tinyOSD_maxScreenSize = TINYOSD_VIDEO_BUFFER_CHARS_NTSC;
    }

    // enable osd
    tinyOSDSetRegister(OPENTCO_OSD_REGISTER_STATUS, 1);

    if (firstInit)
    {
        tinyOSDRefreshAll();
        firstInit = false;
    }
}


bool tinyOSDInit(const vcdProfile_t *pVcdProfile)
{
    // Setup values to write to registers
    video_system = pVcdProfile->video_system;
    //hosRegValue = 32 - pVcdProfile->h_offset;
    //vosRegValue = 16 - pVcdProfile->v_offset;

    // open serial port
    if (!opentcoInit(FUNCTION_TINYOSD)) {
        return false;
    }

    // fill whole screen on device with ' '
    tinyOSDClearScreen(NULL);

    return true;
}

// fill the whole screen with spaces
int tinyOSDClearScreen(displayPort_t *displayPort)
{
    tinyOSDFillRegion(displayPort, 0, 0, 255, 255, ' ');
    return 0;
}

int tinyOSDFillRegion(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t value)
{
    UNUSED(displayPort);

    // start frame
    opentcoInitializeFrame(sbuf, OPENTCO_DEVICE_OSD, OPENTCO_OSD_COMMAND_FILL_REGION);

    // start coordinates
    sbufWriteU8(sbuf, x);
    sbufWriteU8(sbuf, y);

    // width and height of region
    sbufWriteU8(sbuf, width);
    sbufWriteU8(sbuf, height);

    // fill value
    sbufWriteU8(sbuf, value);

    // send
    opentcoSendFrame(sbuf);

    // done
    return 0;
}

//uint8_t* tinyOSDGetScreenBuffer(void) {
//    return NULL;
//}

static void tinyOSDDrawSticks(void) {
    // start frame
    opentcoInitializeFrame(sbuf, OPENTCO_DEVICE_OSD, OPENTCO_OSD_COMMAND_SPECIAL);

    // automatic calc of frame length
    uint8_t *lengthPtr = sbufPtr(sbuf);
    sbufWriteU8(sbuf, 0);

    // set subcommand
    sbufWriteU8(sbuf, OPENTCO_OSD_COMMAND_SPECIAL_SUB_STICKSTATUS);

    // add stick status
    sbufWriteU8(sbuf, TINYOSD_STICKSIZE_X/2 + (TINYOSD_STICKSIZE_X/2 * rcCommand[ROLL]) / 500.0f);
    sbufWriteU8(sbuf, TINYOSD_STICKSIZE_Y/2 - (TINYOSD_STICKSIZE_Y/2 * rcCommand[PITCH]) / 500.0f);
    // throttle is 1000 - 2000, rescale to match out STICK_Y resolution
    sbufWriteU8(sbuf, TINYOSD_STICKSIZE_Y - (TINYOSD_STICKSIZE_Y * (rcCommand[THROTTLE]-1000.0f))/1000.0f);
    sbufWriteU8(sbuf, TINYOSD_STICKSIZE_X/2 - (TINYOSD_STICKSIZE_X/2 * rcCommand[YAW]) / 500.0f);
    // add arming status
    sbufWriteU8(sbuf, armingFlags);

    // add automatic length
    *lengthPtr = sbufPtr(sbuf) - lengthPtr - 1;

    // send
    opentcoSendFrame(sbuf);
}

static void tinyOSDDrawSpectrum(uint8_t axis) {
    // start frame
    opentcoInitializeFrame(sbuf, OPENTCO_DEVICE_OSD, OPENTCO_OSD_COMMAND_SPECIAL);

    // automatic calc of frame length
    uint8_t *lengthPtr = sbufPtr(sbuf);
    sbufWriteU8(sbuf, 0);

    // set subcommand
    sbufWriteU8(sbuf, OPENTCO_OSD_COMMAND_SPECIAL_SUB_SPECTRUM);

    // add axis
    sbufWriteU8(sbuf, axis);

    // fetch gyro bin data
    const gyroFftData_t *gyro_fft = gyroFftData(axis);

    // use simple LPF for rescaling to max
    // y[i] = y[i-1] + (a) * ( x[i] - y[i-1] )
    static float maxgyro[3];
    maxgyro[axis] = maxgyro[axis] + 0.01 * (gyro_fft->maxVal - maxgyro[axis]);

    for(uint32_t i=0; i < GYRO_FFT_BIN_COUNT; i++) {
        // make sure to limit this to 255.0 in any case
        uint8_t bin_value = MIN(255.0, ((255.0 * gyro_fft->bin2[i]) / maxgyro[axis]));
        // add to streambuffer
        sbufWriteU8(sbuf, bin_value);
    }

    // add arming status
    sbufWriteU8(sbuf, armingFlags);

    // add automatic length
    *lengthPtr = sbufPtr(sbuf) - lengthPtr - 1;

    // send
    opentcoSendFrame(sbuf);
}

int tinyOSDWriteChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t c)
{
    UNUSED(displayPort);

    // start frame
    opentcoInitializeFrame(sbuf, OPENTCO_DEVICE_OSD, OPENTCO_OSD_COMMAND_WRITE);

    // add x/y start coordinate
    sbufWriteU8(sbuf, x);
    sbufWriteU8(sbuf, y);

    // add char
    sbufWriteU8(sbuf, c);

    // send
    opentcoSendFrame(sbuf);

    // done
    return 0;
}

int tinyOSDWriteString(displayPort_t *displayPort, uint8_t x, uint8_t y, const char *buff)
{
    UNUSED(displayPort);

    // start frame
    // FIXME: add vertical mode
    // opentcoInitializeFrame(sbuf, OPENTCO_DEVICE_OSD, OPENTCO_OSD_COMMAND_WRITE_BUFFER_V);
    opentcoInitializeFrame(sbuf, OPENTCO_DEVICE_OSD, OPENTCO_OSD_COMMAND_WRITE_BUFFER_H);

    // automatic calc of frame length
    uint8_t *lengthPtr = sbufPtr(sbuf);
    sbufWriteU8(sbuf, 0);

    // add x/y start coordinate
    sbufWriteU8(sbuf, x);
    sbufWriteU8(sbuf, y);

    // add string
    sbufWriteString(sbuf, buff);

    // add automatic length
    *lengthPtr = sbufPtr(sbuf) - lengthPtr - 1;

    // send
    opentcoSendFrame(sbuf);

    // done
    return 0;
}


void tinyOSDSetRegister(uint8_t reg, uint8_t value)
{
    // start frame
    opentcoInitializeFrame(sbuf, OPENTCO_DEVICE_OSD, OPENTCO_OSD_COMMAND_SET_REGISTER);

    // add register and value
    sbufWriteU8(sbuf, reg);
    sbufWriteU8(sbuf, value);

    // send
    opentcoSendFrame(sbuf);
}

int tinyOSDDrawScreen(displayPort_t *displayPortProfile)
{
    UNUSED(displayPortProfile);
    return 0;
}

bool tinyOSDIsTransferInProgress(const displayPort_t *displayPort){
    UNUSED(displayPort);
    return false;
}

int tinyOSDHeartbeat(displayPort_t *displayPort){
    UNUSED(displayPort);

    static timeUs_t tinyOSDNextCycleTimeSticks;
    static timeUs_t tinyOSDNextCycleTimeSpectrum;

    // draw additional data at a fixed framerate
    // note: only draw a single item every iteration
    timeUs_t currentTimeUs = micros();
    if (currentTimeUs >= tinyOSDNextCycleTimeSticks) {
        // draw stick overlay
        tinyOSDDrawSticks();
        tinyOSDNextCycleTimeSticks = currentTimeUs + TINYOSD_CYCLETIME_US_STICKS;
    } else if (currentTimeUs >= tinyOSDNextCycleTimeSpectrum) {
        // draw spectrum overlay
        tinyOSDDrawSpectrum(0); // FIXME: add axis round robin
        tinyOSDNextCycleTimeSpectrum = currentTimeUs + TINYOSD_CYCLETIME_US_SPECTRUM;
    }

    return 0;
}

// This funcktion refresh all and should not be used when copter is armed
void tinyOSDRefreshAll(void)
{
    // FIXME: with successive osd item drawing this has to be handled differently
}

void tinyOSDResync(displayPort_t *displayPort)
{
    if (video_system == VIDEO_SYSTEM_PAL) {
        displayPort->rowCount = TINYOSD_VIDEO_LINES_PAL;
    } else {
        displayPort->rowCount = TINYOSD_VIDEO_LINES_NTSC;
    }

    displayPort->colCount = TINYOSD_VIDEO_COLS;
}

uint32_t tinyOSDTxBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

void tinyOSDWriteNvm(uint8_t char_address, const uint8_t *font_data)
{
    UNUSED(char_address);
    UNUSED(font_data);
}

