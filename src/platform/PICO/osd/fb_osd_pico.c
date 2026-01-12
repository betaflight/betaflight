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
#include <string.h>

#include "platform.h"

#ifdef USE_FB_OSD

#include "config/config_streamer.h"
#include "drivers/fb_osd_impl.h"
#include "drivers/osd.h"
#include "drivers/time.h"
#include "font_betaflight.h"
#include "osd_pico.h"

static uint8_t fontDataMagic[] = {'p', 'f', 'n', 't'};

static bool inNTSCrange(int n)
{
    const int ntscHsyncsMin = 253;
    const int ntscHsyncsMax = 254;
    return n >= ntscHsyncsMin - 1 && n <= ntscHsyncsMax + 1;
}

static bool inPALrange(int n)
{
    const int palHsyncs = 305;
    return n >= palHsyncs - 1 && n <= palHsyncs + 1;
}

static void fbOsdLoadFont(void)
{
    // Retrieve font data from flash if present. Otherwise defaults to baked-in font in font_betaflight.c.
    uint8_t *ptr = (uint8_t *)&__fontdata_start;
    if (0 != memcmp(ptr, fontDataMagic, 4)) {
        bprintf("FONT did not detect font data in flash");
        return;
    }

    ptr += 4;
    bprintf("FONT loading font from flash into ram");
    memcpy(fontData, ptr, FONTDATA_LENGTH);
    bprintf("FONT loaded font from flash into ram");
}

fbOsdInitStatus_e fbOsdInit(const struct fbOsdConfig_s *fbOsdConfig, const struct vcdProfile_s *vcdProfile)
{
    UNUSED(fbOsdConfig);

    static bool first = true;
    const int repeatTarget = 15;
    static int repeatCount;
    static int lastRange; // 1 = NTSC range, -1 = PAL range, 0 = neither

    static int lastHSyncs = -1;

    videoSystem_e videoSystem = vcdProfile->video_system;

    if (first) {
        fbOsdLoadFont();
        if (!osdPioStartDetection()) {
            return FB_OSD_INIT_NOT_CONFIGURED;
        }

        first = false;
        bprintf("fbOsdInit vcdProfile %p video system %d", vcdProfile, videoSystem);
        return FB_OSD_INIT_INITIALISING;
    }
    
    int hSyncs = osdPioCountHSyncs();

    
#ifdef PICO_TRACE
    static int count;
    count++;
    if (false) { // ((count % 100017 == 0) || (hSyncs && hSyncs != lastHSyncs)) {
        bprintf("OSD %d detected %d hSyncs", count, hSyncs);
    }
#else
    UNUSED(lastHSyncs);
#endif

    if (!hSyncs) {
        // Invalid - maybe tried to read again too quickly.
        return FB_OSD_INIT_INITIALISING;
    }

#ifdef PICO_TRACE    
    if (count % 7 == 0 && hSyncs != lastHSyncs) {
        bprintf("OSD %d detected %d hSyncs", count, hSyncs);
    }
#endif

    // While composite source is warming up, might expect to see hSyncs increasing over a period of seconds
    // from zero to a stable number.
    int range = inNTSCrange(hSyncs) ? 1 : inPALrange(hSyncs) ? -1 : 0;
    if (range && (lastRange == range)) {
        repeatCount++;
        if (0 == (repeatCount % 4)) {
            bprintf("OSD repeat %d of %d (%s)", repeatCount, hSyncs, range == 1 ? "NTSC" : range == -1 ? "PAL" : "neither PAL nor NTSC");
        }
    } else {
        repeatCount = 0;
    }
    
    if (repeatCount == repeatTarget) {
        bool initOk;
        if (range == 1) {
            bprintf("OSD seen %d of %d (NTSC)", repeatCount, hSyncs);
            if (videoSystem == VIDEO_SYSTEM_PAL) {
                bprintf("*** Warning: detected NTSC sync pattern but starting as PAL due to configuration ***");
                initOk = osdPioStartPAL();
            } else {
                initOk = osdPioStartNTSC();
            }
        } else {
            bprintf("OSD seen %d of %d (PAL)", repeatCount, hSyncs);
            if (videoSystem == VIDEO_SYSTEM_NTSC) {
                bprintf("*** Warning: detected PAL sync pattern but starting as NTSC due to configuration ***");
                initOk = osdPioStartNTSC();
            } else {
                initOk = osdPioStartPAL();
            }
        }

        return initOk ? FB_OSD_INIT_OK : FB_OSD_INIT_NOT_CONFIGURED;
    }
    
    lastHSyncs = hSyncs;
    lastRange = range;
    return FB_OSD_INIT_INITIALISING;
}

bool fbOsdReInitIfRequired(bool forceStallCheck)
{
    UNUSED(forceStallCheck);
    return false;
}


// Limit time taken for an individual call to fbOsdDrawScreen.
// NB not a true limit, we are allowed to start a new operation if time has not gone past this,
// so it might end up being exceeded by the length of the longest individual operation.

// set this low to diagnose long operations
// #define DRAWSCREEN_TIME_LIMIT_US 5

#define DRAWSCREEN_TIME_LIMIT_US 15

// Return true if not complete.
bool fbOsdDrawScreen(void)
{
    // Try to spend no more than DRAWSCREEN_TIME_LIMIT_US on each iteration, will keep
    // on calling in here until we return false for "all done".
    return osdPioRenderScreenUntil(micros() + DRAWSCREEN_TIME_LIMIT_US);
}

// Write a character into the font data storage (in RAM)
bool fbOsdWriteFontCharacter(uint8_t char_address, const uint8_t *font_data)
{
    uint8_t bitConv[] = {0b10, 0b00, 0b11, 0b00};
    // MCM format 00 = black, 01 = transparent, 10 = white, 11 = transparent
    // -> FB format 10 = black, 11 = white, 00 = transparent
    uint8_t *p = fontData + 54*char_address;
    for (int i=0; i<54; ++i) {
        uint8_t c = *font_data++;
        uint8_t d = bitConv[c&0x3] << 6;
        d |= bitConv[(c>>2)&0x3] << 4;
        d |= bitConv[(c>>4)&0x3] << 2;
        d |= bitConv[c>>6];
        *p++ = d;
    }

    return true;
}

void fbOsdFontUpdateCompletion(void)
{
    // The font has been updated in place (in RAM). Copy out of RAM back into flash.
    // Borrow the streamer functions from config.
    // __fontdata_start will be on a flash page size boundary (256 byte aligned)
    bprintf("FONT fbOsdFontUpdateCompletion start");
    config_streamer_t streamer = {
        .address = (uintptr_t)&__fontdata_start,
    };
    config_streamer_write(&streamer, fontDataMagic, 4);
    config_streamer_write(&streamer, (const uint8_t *)fontData, FONTDATA_LENGTH);
    config_streamer_flush(&streamer);
    bprintf("FONT fbOsdFontUpdateCompletion end");
}
    
uint8_t fbOsdGetRowsCount(void)
{
    return osdPioRowsCount();
}

void fbOsdWrite(uint8_t x, uint8_t y, uint8_t attr, const char *text)
{
    // *** TODO possibly implement some attr behaviours
    UNUSED(attr);
    osdPioWrite(x, y, text);
}

void fbOsdWriteChar(uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
{
    // *** TODO possibly implement some attr behaviours
    UNUSED(attr);
    osdPioWriteChar(x, y, c);
}

void fbOsdClearScreen(void)
{
    osdPioClearCharBuffer();
}

void fbOsdRefreshAll(void)
{
    fbOsdReInitIfRequired(true);
    while (fbOsdDrawScreen()) ; // Call draw function until transfer is completed.
}

bool fbOsdBufferInUse(void)
{
    return !osdPioBufferAvailable();
}

bool fbOsdLayerSupported(displayPortLayer_e layer)
{
    return layer == DISPLAYPORT_LAYER_FOREGROUND;
}

bool fbOsdLayerSelect(displayPortLayer_e layer)
{
    return fbOsdLayerSupported(layer); // no action
}

bool fbOsdLayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer)
{
    UNUSED(destLayer);
    UNUSED(sourceLayer);
    return false;
}

void fbOsdSetBackgroundType(displayPortBackground_e backgroundType)
{
    // Not currently supporting different background types.
    UNUSED(backgroundType);
}

bool fbOsdDrawItem(osd_items_e item, uint8_t elemPosX, uint8_t elemPosY, bool isBackground)
{
    return isBackground ? osdPioDrawBackgroundItem(item, elemPosX, elemPosY) : osdPioDrawForegroundItem(item, elemPosX, elemPosY);
}

void fbOsdRedrawBackground(void)
{
    osdPioRedrawBackground();
}

#endif // USE_FB_OSD
