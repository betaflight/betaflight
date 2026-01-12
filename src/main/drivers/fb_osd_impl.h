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

#pragma once

#include <stdint.h>

#include "drivers/display.h"
#include "osd/osd.h"
#include "pg/vcd.h"

typedef enum {
    FB_OSD_INIT_OK = 0,              // IO defined and fb device was detected
    FB_OSD_INIT_INITIALISING = -1,   // IO defined, but fb device not fully up and running (e.g. syncs not yet correctly detected)
    FB_OSD_INIT_NOT_CONFIGURED = -2, // No fb IO defined, which means either the we don't have it or it's not properly configured
} fbOsdInitStatus_e;

// fbOsdConfig not currently used, we could put e.g. horizontal and vertical offsets in here.
struct fbOsdConfig_s; 

// Per platform implementation of framebuffer OSD.
fbOsdInitStatus_e fbOsdInit(const struct fbOsdConfig_s *fbOsdConfig, const struct vcdProfile_s *vcdProfile);
bool fbOsdReInitIfRequired(bool forceStallCheck);
bool fbOsdDrawScreen(void);
bool fbOsdWriteFontCharacter(uint8_t char_address, const uint8_t *font_data);
uint8_t fbOsdGetRowsCount(void);
void fbOsdWrite(uint8_t x, uint8_t y, uint8_t attr, const char *text);
void fbOsdWriteChar(uint8_t x, uint8_t y, uint8_t attr, uint8_t c);
void fbOsdClearScreen(void);
void fbOsdRefreshAll(void);
bool fbOsdBufferInUse(void);
bool fbOsdLayerSupported(displayPortLayer_e layer);
bool fbOsdLayerSelect(displayPortLayer_e layer);
bool fbOsdLayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer);
void fbOsdSetBackgroundType(displayPortBackground_e backgroundType);
bool fbOsdDrawItem(osd_items_e item, uint8_t elemPosX, uint8_t elemPosY, bool isBackground);
void fbOsdRedrawBackground(void);
void fbOsdFontUpdateCompletion(void);

// Not currently required / implemented
// void fbOsdHardwareReset(void);
// void fbOsdPreinit(const struct fbOsdConfig_s *fbOsdConfig);
// void fbOsdInvert(bool invert);
// void fbOsdBrightness(uint8_t black, uint8_t white);
// bool fbOsdBuffersSynced(void);
// bool fbOsdIsDeviceDetected(void);
