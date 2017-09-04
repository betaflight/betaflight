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

#include "opentco_osd.h"
#include "opentco.h"

#include "common/maths.h"
#include "common/time.h"
#include "io/osd.h"
#include "io/displayport_opentco.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "drivers/time.h"
#include "drivers/vcd.h"
#include "drivers/opentco.h"
#include "sensors/gyroanalyse.h"

//uint16_t opentcoOSDMaxScreenSize = OPENTCO_OSD_VIDEO_BUFFER_CHARS_PAL;

static opentcoDevice_t OSDDevice;
static opentcoDevice_t *device = &OSDDevice;

static uint8_t  video_system;

//static void opentcoOSDSetRegister(uint8_t reg, uint16_t value);
void opentcoOSDRefreshAll(void);

int opentcoOSDGrab(displayPort_t * displayPort)
{
    UNUSED(displayPort);
    osdResetAlarms();
    resumeRefreshAt = 0;
    return 0;
}

int opentcoOSDRelease(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

int opentcoOSDFillRegion(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t value)
{
    UNUSED(displayPort);

    // start frame
    opentcoInitializeFrame(device, OPENTCO_OSD_COMMAND_FILL_REGION);

    // start coordinates
    sbufWriteU8(device->sbuf, x);
    sbufWriteU8(device->sbuf, y);

    // width and height of region
    sbufWriteU8(device->sbuf, width);
    sbufWriteU8(device->sbuf, height);

    // fill value
    sbufWriteU8(device->sbuf, value);

    // send
    opentcoSendFrame(device);

    // done
    return 0;
}

// fill the whole screen with spaces
int opentcoOSDClearScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    opentcoOSDFillRegion(displayPort, 0, 0, 255, 255, ' ');
    return 0;
}

static void opentcoOSDQuerySupportedFeatures()
{
    uint16_t opentcoFeatures;

    // fetch available and acitvated features
    opentcoReadRegisterUint16(device, OPENTCO_OSD_REGISTER_SUPPORTED_FEATURES,  &opentcoFeatures);

    // opentco features do not necessarily have to match
    // with betaflight features -> convert!
    uint16_t displayFeature = 0;
    if (opentcoFeatures & OPENTCO_OSD_FEATURE_ENABLE) displayFeature |= DISPLAY_FEATURE_ENABLE;
    if (opentcoFeatures & OPENTCO_OSD_FEATURE_INVERT) displayFeature |= DISPLAY_FEATURE_INVERT;
    if (opentcoFeatures & OPENTCO_OSD_FEATURE_BRIGHTNESS) displayFeature |= DISPLAY_FEATURE_BRIGHTNESS;
    if (opentcoFeatures & OPENTCO_OSD_FEATURE_RENDER_CROSSHAIR) displayFeature |= DISPLAY_FEATURE_RENDER_CROSSHAIR;
    if (opentcoFeatures & OPENTCO_OSD_FEATURE_RENDER_LOGO) displayFeature |= DISPLAY_FEATURE_RENDER_LOGO;
    if (opentcoFeatures & OPENTCO_OSD_FEATURE_RENDER_PILOTLOGO) displayFeature |= DISPLAY_FEATURE_RENDER_PILOTLOGO;
    if (opentcoFeatures & OPENTCO_OSD_FEATURE_RENDER_SPECTRUM) displayFeature |= DISPLAY_FEATURE_RENDER_SPECTRUM;
    if (opentcoFeatures & OPENTCO_OSD_FEATURE_RENDER_STICKS) displayFeature |= DISPLAY_FEATURE_RENDER_STICKS;

    // store
    displayPortProfileMutable()->supportedFeatures = displayFeature;

    // disable any unsupported features:
    displayPortProfileMutable()->enabledFeatures &=  displayFeature;
}

bool opentcoOSDInit(const vcdProfile_t *pVcdProfile)
{
    // Setup values to write to registers
    video_system = pVcdProfile->video_system;
    //hosRegValue = 32 - pVcdProfile->h_offset;
    //vosRegValue = 16 - pVcdProfile->v_offset;

    // configure device:
    device->id = OPENTCO_DEVICE_OSD;

    // open serial port
    if (!opentcoInit(device)) {
        // no device found
        return false;
    }

    // start with disabled features (switch osd off!)
    displayPortProfileMutable()->enabledFeatures = 0;

    if (video_system == VIDEO_SYSTEM_AUTO) {
        // fetch current video mode from opentco OSD
        uint16_t tmp;
        opentcoReadRegisterUint16(device, OPENTCO_OSD_REGISTER_VIDEO_FORMAT, &tmp);
        video_system = tmp;
    } else {
        // set video system
        opentcoWriteRegisterUint16(device, OPENTCO_OSD_REGISTER_VIDEO_FORMAT, video_system);
    }

    // try to enable all enabled osd features
    opentcoOSDQuerySupportedFeatures();

    // fill whole screen on device with ' '
    opentcoOSDClearScreen(NULL);

    return true;
}

static void opentcoOSDDrawSticks(void) {
    // start frame
    opentcoInitializeFrame(device, OPENTCO_OSD_COMMAND_SPECIAL);

    // automatic calc of frame length
    uint8_t *lengthPtr = sbufPtr(device->sbuf);
    sbufWriteU8(device->sbuf, 0);

    // set subcommand
    sbufWriteU8(device->sbuf, OPENTCO_OSD_COMMAND_SPECIAL_SUB_STICKSTATUS);

    // add stick status
    sbufWriteU8(device->sbuf, OPENTCO_OSD_STICKSIZE_X/2 + (OPENTCO_OSD_STICKSIZE_X/2 * rcCommand[ROLL]) / 500.0f);
    sbufWriteU8(device->sbuf, OPENTCO_OSD_STICKSIZE_Y/2 - (OPENTCO_OSD_STICKSIZE_Y/2 * rcCommand[PITCH]) / 500.0f);
    // throttle is 1000 - 2000, rescale to match out STICK_Y resolution
    sbufWriteU8(device->sbuf, OPENTCO_OSD_STICKSIZE_Y - (OPENTCO_OSD_STICKSIZE_Y * (rcCommand[THROTTLE]-1000.0f))/1000.0f);
    sbufWriteU8(device->sbuf, OPENTCO_OSD_STICKSIZE_X/2 - (OPENTCO_OSD_STICKSIZE_X/2 * rcCommand[YAW]) / 500.0f);
    // add arming status
    sbufWriteU8(device->sbuf, armingFlags);

    // add automatic length
    *lengthPtr = sbufPtr(device->sbuf) - lengthPtr - 1;

    // send
    opentcoSendFrame(device);
}

static void opentcoOSDDrawSpectrum(uint8_t axis) {
    // start frame
    opentcoInitializeFrame(device, OPENTCO_OSD_COMMAND_SPECIAL);

    // automatic calc of frame length
    uint8_t *lengthPtr = sbufPtr(device->sbuf);
    sbufWriteU8(device->sbuf, 0);

    // set subcommand
    sbufWriteU8(device->sbuf, OPENTCO_OSD_COMMAND_SPECIAL_SUB_SPECTRUM);

    // add axis
    sbufWriteU8(device->sbuf, axis);

    // fetch gyro bin data
    const gyroFftData_t *gyro_fft = gyroFftData(axis);

    // use simple LPF for rescaling to max
    // y[i] = y[i-1] + (a) * ( x[i] - y[i-1] )
    static float maxgyro[3];
    maxgyro[axis] = maxgyro[axis] + 0.01f * (gyro_fft->maxVal - maxgyro[axis]);

    for(int i=0; i < GYRO_FFT_BIN_COUNT; i++) {
        // make sure to limit this to 255.0 in any case
        uint8_t bin_value = MIN(255.0f, ((255.0f * gyro_fft->bin2[i]) / maxgyro[axis]));
        // add to streambuffer
        sbufWriteU8(device->sbuf, bin_value);
    }

    // add arming status
    sbufWriteU8(device->sbuf, armingFlags);

    // add automatic length
    *lengthPtr = sbufPtr(device->sbuf) - lengthPtr - 1;

    // send
    opentcoSendFrame(device);
}

int opentcoOSDWriteChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t c)
{
    UNUSED(displayPort);

    // start frame
    opentcoInitializeFrame(device, OPENTCO_OSD_COMMAND_WRITE);

    // add x/y start coordinate
    sbufWriteU8(device->sbuf, x);
    sbufWriteU8(device->sbuf, y);

    // add char
    sbufWriteU8(device->sbuf, c);

    // send
    opentcoSendFrame(device);

    // done
    return 0;
}

int opentcoOSDWriteString(displayPort_t *displayPort, uint8_t x, uint8_t y, const char *buff)
{
    UNUSED(displayPort);

    // start frame
    // FIXME: add vertical mode
    // opentcoInitializeFrame(device->sbuf, OPENTCO_DEVICE_OSD, OPENTCO_OSD_COMMAND_WRITE_BUFFER_V);
    opentcoInitializeFrame(device, OPENTCO_OSD_COMMAND_WRITE_BUFFER_H);

    // automatic calc of frame length
    uint8_t *lengthPtr = sbufPtr(device->sbuf);
    sbufWriteU8(device->sbuf, 0);

    // add x/y start coordinate
    sbufWriteU8(device->sbuf, x);
    sbufWriteU8(device->sbuf, y);

    // add string
    sbufWriteString(device->sbuf, buff);

    // add automatic length
    *lengthPtr = sbufPtr(device->sbuf) - lengthPtr - 1;

    // send
    opentcoSendFrame(device);

    // done
    return 0;
}

void opentcoOSDSetRegister(uint8_t reg, uint16_t value)
{
    opentcoWriteRegisterUint16(device, reg, value);
}

static void opentcoOSDSendEnabledFeatures()
{
    // enable features, convert bf display features to opentco:
    uint16_t displayFeature = displayPortProfile()->enabledFeatures;
    uint16_t opentcoFeature = 0;
    if (displayFeature & DISPLAY_FEATURE_ENABLE) opentcoFeature |= OPENTCO_OSD_FEATURE_ENABLE;
    if (displayFeature & DISPLAY_FEATURE_INVERT) opentcoFeature |= OPENTCO_OSD_FEATURE_INVERT;
    if (displayFeature & DISPLAY_FEATURE_BRIGHTNESS) opentcoFeature |= OPENTCO_OSD_FEATURE_BRIGHTNESS;
    if (displayFeature & DISPLAY_FEATURE_RENDER_CROSSHAIR) opentcoFeature |= OPENTCO_OSD_FEATURE_RENDER_CROSSHAIR;
    if (displayFeature & DISPLAY_FEATURE_RENDER_LOGO) opentcoFeature |= OPENTCO_OSD_FEATURE_RENDER_LOGO;
    if (displayFeature & DISPLAY_FEATURE_RENDER_PILOTLOGO) opentcoFeature |= OPENTCO_OSD_FEATURE_RENDER_PILOTLOGO;
    if (displayFeature & DISPLAY_FEATURE_RENDER_SPECTRUM) opentcoFeature |= OPENTCO_OSD_FEATURE_RENDER_SPECTRUM;
    if (displayFeature & DISPLAY_FEATURE_RENDER_STICKS) opentcoFeature |= OPENTCO_OSD_FEATURE_RENDER_STICKS;

    // activate
    opentcoOSDSetRegister(OPENTCO_OSD_REGISTER_STATUS, opentcoFeature);
}

int opentcoOSDReloadProfile (displayPort_t * displayPort)
{
    UNUSED(displayPort);

    opentcoOSDSendEnabledFeatures();

    // set brightness
    opentcoOSDSetRegister(OPENTCO_OSD_REGISTER_BRIGHTNESS_BLACK, displayPortProfile()->blackBrightness);
    opentcoOSDSetRegister(OPENTCO_OSD_REGISTER_BRIGHTNESS_WHITE, displayPortProfile()->whiteBrightness);

    return 0;
}

int opentcoOSDDrawScreen(displayPort_t *displayPortProfile)
{
    UNUSED(displayPortProfile);
    return 0;
}

bool opentcoOSDIsTransferInProgress(const displayPort_t *displayPort){
    UNUSED(displayPort);
    return false;
}

int opentcoOSDHeartbeat(displayPort_t *displayPort){
    UNUSED(displayPort);

    static timeUs_t opentcoOSDNextCycleTime;
    static uint8_t opentcoOSDNextCycleItem = 0;

    // draw additional data at a fixed framerate
    // note: only draw a single item every iteration
    timeUs_t currentTimeUs = micros();
    if (currentTimeUs >= opentcoOSDNextCycleTime) {
        switch (++opentcoOSDNextCycleItem)
        {
        default:
        case 0:
            // send enabled features
            opentcoOSDSendEnabledFeatures();
            break;

        case 1:
            // draw stick overlay?
            if (displayPortProfile()->enabledFeatures & DISPLAY_FEATURE_RENDER_STICKS) {
                opentcoOSDDrawSticks();
            }
            break;

        case 2:
            // draw spectrum overlay?
            if (displayPortProfile()->enabledFeatures & DISPLAY_FEATURE_RENDER_SPECTRUM) {
                opentcoOSDDrawSpectrum(0); // FIXME: add axis round robin
            }
            // last item -> return to zero
            opentcoOSDNextCycleItem = 0;
            break;
        }

        // store next time
        opentcoOSDNextCycleTime = currentTimeUs + OPENTCO_OSD_CYCLETIME_US;
    }

    return 0;
}

void opentcoOSDResync(displayPort_t *displayPort)
{
    if (video_system == VIDEO_SYSTEM_PAL) {
        displayPort->rowCount = OPENTCO_OSD_VIDEO_LINES_PAL;
    } else {
        displayPort->rowCount = OPENTCO_OSD_VIDEO_LINES_NTSC;
    }

    displayPort->colCount = OPENTCO_OSD_VIDEO_COLS;
}

uint32_t opentcoOSDTxBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

void opentcoOSDWriteNvm(uint8_t char_address, const uint8_t *font_data)
{
    UNUSED(char_address);
    UNUSED(font_data);
}
