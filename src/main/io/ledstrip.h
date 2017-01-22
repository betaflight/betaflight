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

#pragma once

#include "config/parameter_group.h"

#define LED_MAX_STRIP_LENGTH           32
#define LED_CONFIGURABLE_COLOR_COUNT   16
#define LED_MODE_COUNT                  6
#define LED_DIRECTION_COUNT             6
#define LED_BASEFUNCTION_COUNT          7
#define LED_OVERLAY_COUNT               6
#define LED_SPECIAL_COLOR_COUNT        11

#define LED_POS_OFFSET                  0
#define LED_FUNCTION_OFFSET             8
#define LED_OVERLAY_OFFSET             12
#define LED_COLOR_OFFSET               18
#define LED_DIRECTION_OFFSET           22
#define LED_PARAMS_OFFSET              28

#define LED_POS_BITCNT                  8
#define LED_FUNCTION_BITCNT             4
#define LED_OVERLAY_BITCNT              6
#define LED_COLOR_BITCNT                4
#define LED_DIRECTION_BITCNT            6
#define LED_PARAMS_BITCNT               4

#define LED_FLAG_OVERLAY_MASK ((1 << LED_OVERLAY_BITCNT) - 1)
#define LED_FLAG_DIRECTION_MASK ((1 << LED_DIRECTION_BITCNT) - 1)

#define LED_MOV_POS(pos) ((pos) << LED_POS_OFFSET)
#define LED_MOV_FUNCTION(func) ((func) << LED_FUNCTION_OFFSET)
#define LED_MOV_OVERLAY(overlay) ((overlay) << LED_OVERLAY_OFFSET)
#define LED_MOV_COLOR(colorId) ((colorId) << LED_COLOR_OFFSET)
#define LED_MOV_DIRECTION(direction) ((direction) << LED_DIRECTION_OFFSET)
#define LED_MOV_PARAMS(param) ((param) << LED_PARAMS_OFFSET)

#define LED_BIT_MASK(len) ((1 << (len)) - 1)

#define LED_POS_MASK LED_MOV_POS(((1 << LED_POS_BITCNT) - 1))
#define LED_FUNCTION_MASK LED_MOV_FUNCTION(((1 << LED_FUNCTION_BITCNT) - 1))
#define LED_OVERLAY_MASK LED_MOV_OVERLAY(LED_FLAG_OVERLAY_MASK)
#define LED_COLOR_MASK LED_MOV_COLOR(((1 << LED_COLOR_BITCNT) - 1))
#define LED_DIRECTION_MASK LED_MOV_DIRECTION(LED_FLAG_DIRECTION_MASK)
#define LED_PARAMS_MASK LED_MOV_PARAMS(((1 << LED_PARAMS_BITCNT) - 1))

#define LED_FLAG_OVERLAY(id) (1 << (id))
#define LED_FLAG_DIRECTION(id) (1 << (id))

#define LED_X_BIT_OFFSET 4
#define LED_Y_BIT_OFFSET 0
#define LED_XY_MASK      0x0F
#define CALCULATE_LED_XY(x, y) ((((x) & LED_XY_MASK) << LED_X_BIT_OFFSET) | (((y) & LED_XY_MASK) << LED_Y_BIT_OFFSET))

typedef enum {
    LED_MODE_ORIENTATION = 0,
    LED_MODE_HEADFREE,
    LED_MODE_HORIZON,
    LED_MODE_ANGLE,
    LED_MODE_MAG,
    LED_MODE_BARO,
    LED_SPECIAL
} ledModeIndex_e;

typedef enum {
    LED_SCOLOR_DISARMED = 0,
    LED_SCOLOR_ARMED,
    LED_SCOLOR_ANIMATION,
    LED_SCOLOR_BACKGROUND,
    LED_SCOLOR_BLINKBACKGROUND,
    LED_SCOLOR_GPSNOSATS,
    LED_SCOLOR_GPSNOLOCK,
    LED_SCOLOR_GPSLOCKED
} ledSpecialColorIds_e;

typedef enum {
    LED_DIRECTION_NORTH = 0,
    LED_DIRECTION_EAST,
    LED_DIRECTION_SOUTH,
    LED_DIRECTION_WEST,
    LED_DIRECTION_UP,
    LED_DIRECTION_DOWN
} ledDirectionId_e;

typedef enum {
    LED_FUNCTION_COLOR,
    LED_FUNCTION_FLIGHT_MODE,
    LED_FUNCTION_ARM_STATE,
    LED_FUNCTION_BATTERY,
    LED_FUNCTION_RSSI,
    LED_FUNCTION_GPS,
    LED_FUNCTION_THRUST_RING,
} ledBaseFunctionId_e;

typedef enum {
    LED_OVERLAY_THROTTLE,
    LED_OVERLAY_LARSON_SCANNER,
    LED_OVERLAY_BLINK,
    LED_OVERLAY_LANDING_FLASH,
    LED_OVERLAY_INDICATOR,
    LED_OVERLAY_WARNING,
} ledOverlayId_e;

typedef struct modeColorIndexes_s {
    uint8_t color[LED_DIRECTION_COUNT];
} modeColorIndexes_t;

typedef struct specialColorIndexes_s {
    uint8_t color[LED_SPECIAL_COLOR_COUNT];
} specialColorIndexes_t;

typedef uint32_t ledConfig_t;

typedef struct ledCounts_s {
    uint8_t count;
    uint8_t ring;
    uint8_t larson;
    uint8_t ringSeqLen;
} ledCounts_t;

typedef struct ledStripConfig_s {
    ledConfig_t ledConfigs[LED_MAX_STRIP_LENGTH];
    hsvColor_t colors[LED_CONFIGURABLE_COLOR_COUNT];
    modeColorIndexes_t modeColors[LED_MODE_COUNT];
    specialColorIndexes_t specialColors;
    uint8_t ledstrip_visual_beeper; // suppress LEDLOW mode if beeper is on
} ledStripConfig_t;

PG_DECLARE(ledStripConfig_t, ledStripConfig);

#define DEFINE_LED(x, y, col, dir, func, ol, params) (LED_MOV_POS(CALCULATE_LED_XY(x, y)) | LED_MOV_COLOR(col) | LED_MOV_DIRECTION(dir) | LED_MOV_FUNCTION(func) | LED_MOV_OVERLAY(ol) | LED_MOV_PARAMS(params))

static inline uint8_t ledGetXY(const ledConfig_t *lcfg)         { return ((*lcfg >> LED_POS_OFFSET) & LED_BIT_MASK(LED_POS_BITCNT)); }
static inline uint8_t ledGetX(const ledConfig_t *lcfg)          { return ((*lcfg >> (LED_POS_OFFSET + LED_X_BIT_OFFSET)) & LED_XY_MASK); }
static inline uint8_t ledGetY(const ledConfig_t *lcfg)          { return ((*lcfg >> (LED_POS_OFFSET + LED_Y_BIT_OFFSET)) & LED_XY_MASK); }
static inline uint8_t ledGetFunction(const ledConfig_t *lcfg)   { return ((*lcfg >> LED_FUNCTION_OFFSET) & LED_BIT_MASK(LED_FUNCTION_BITCNT)); }
static inline uint8_t ledGetOverlay(const ledConfig_t *lcfg)    { return ((*lcfg >> LED_OVERLAY_OFFSET) & LED_BIT_MASK(LED_OVERLAY_BITCNT)); }
static inline uint8_t ledGetColor(const ledConfig_t *lcfg)      { return ((*lcfg >> LED_COLOR_OFFSET) & LED_BIT_MASK(LED_COLOR_BITCNT)); }
static inline uint8_t ledGetDirection(const ledConfig_t *lcfg)  { return ((*lcfg >> LED_DIRECTION_OFFSET) & LED_BIT_MASK(LED_DIRECTION_BITCNT)); }
static inline uint8_t ledGetParams(const ledConfig_t *lcfg)     { return ((*lcfg >> LED_PARAMS_OFFSET) & LED_BIT_MASK(LED_PARAMS_BITCNT)); }

static inline bool ledGetOverlayBit(const ledConfig_t *lcfg, int id) { return ((ledGetOverlay(lcfg) >> id) & 1); }
static inline bool ledGetDirectionBit(const ledConfig_t *lcfg, int id) { return ((ledGetDirection(lcfg) >> id) & 1); }

bool parseColor(int index, const char *colorConfig);

bool parseLedStripConfig(int ledIndex, const char *config);
void generateLedConfig(ledConfig_t *ledConfig, char *ledConfigBuffer, size_t bufferSize);
void reevaluateLedConfig(void);

void ledStripInit(void);
void ledStripEnable(void);
void ledStripUpdate(timeUs_t currentTimeUs);

bool setModeColor(ledModeIndex_e modeIndex, int modeColorIndex, int colorIndex);
