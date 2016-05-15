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

#define LED_MAX_STRIP_LENGTH           32
#define LED_CONFIGURABLE_COLOR_COUNT   16
#define LED_MODE_COUNT                  6
#define LED_DIRECTION_COUNT             6
#define LED_FUNCTION_COUNT             10
#define LED_SPECIAL_COLOR_COUNT         8

#define LED_DIRECTION_BIT_OFFSET 0
#define LED_FUNCTION_BIT_OFFSET LED_DIRECTION_COUNT

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

#define LED_FLAG_DIRECTION(directionId) (1 << (LED_DIRECTION_BIT_OFFSET + (directionId)))
// generate direction bit, used in initializers
#define LED_FLAG_DIRECTION_MASK (((1 << LED_DIRECTION_COUNT) - 1) << LED_DIRECTION_BIT_OFFSET)

typedef enum {
    LED_FUNCTION_INDICATOR,
    LED_FUNCTION_WARNING,
    LED_FUNCTION_FLIGHT_MODE,
    LED_FUNCTION_ARM_STATE,
    LED_FUNCTION_THROTTLE,
    LED_FUNCTION_THRUST_RING,
    LED_FUNCTION_COLOR,
    LED_FUNCTION_GPS,
    LED_FUNCTION_RSSI,
    LED_FUNCTION_BLINK,
} ledFunctionId_e;

#define LED_FLAG_FUNCTION(functionId) (1 << (LED_FUNCTION_BIT_OFFSET + (functionId)))
// generate direction bit, used in initializers
#define LED_FLAG_FUNCTION_MASK (((1 << LED_FUNCTION_COUNT) - 1) << LED_FUNCTION_BIT_OFFSET)

typedef struct modeColorIndexes_s {
    uint8_t color[LED_DIRECTION_COUNT];
} modeColorIndexes_t;

typedef struct specialColorIndexes_s {
    uint8_t color[LED_SPECIAL_COLOR_COUNT];
} specialColorIndexes_t;

typedef struct ledConfig_s {
    uint8_t xy;     // see LED_X/Y_MASK defines
    uint8_t color;  // see colors (config_master)
    uint16_t flags; // see LED_FLAG_FUNCTION + LED_FLAG_DIRECTION
} ledConfig_t;

#define LED_X_BIT_OFFSET 4
#define LED_Y_BIT_OFFSET 0
#define LED_XY_MASK      0x0F

static inline int ledGetX(const ledConfig_t *lcfg) { return (lcfg->xy >> LED_X_BIT_OFFSET) & LED_XY_MASK; }
static inline int ledGetY(const ledConfig_t *lcfg) { return (lcfg->xy >> LED_Y_BIT_OFFSET) & LED_XY_MASK; }
static inline void ledSetXY(ledConfig_t *lcfg, int x, int y) {
    lcfg->xy = ((x & LED_XY_MASK) << LED_X_BIT_OFFSET) | ((y & LED_XY_MASK) << LED_Y_BIT_OFFSET);
}
#define CALCULATE_LED_XY(x, y) ((((x) & LED_XY_MASK) << LED_X_BIT_OFFSET) | (((y) & LED_XY_MASK) << LED_Y_BIT_OFFSET))

extern uint8_t ledCount;
extern uint8_t ledRingCount;

PG_DECLARE_ARR(ledConfig_t, LED_MAX_STRIP_LENGTH, ledConfigs);
PG_DECLARE_ARR(hsvColor_t, LED_CONFIGURABLE_COLOR_COUNT, colors);
PG_DECLARE_ARR(modeColorIndexes_t, LED_MODE_COUNT, modeColors);
PG_DECLARE_ARR(specialColorIndexes_t, 1, specialColors);

void ledStripInit(void);

bool parseLedStripConfig(int ledIndex, const char *config);
void updateLedStrip(void);
void updateLedRing(void);

void applyDefaultLedStripConfig(void);
void generateLedConfig(int ledIndex, char *ledConfigBuffer, size_t bufferSize);

bool parseColor(int index, const char *colorConfig);
void applyDefaultColors(void);

void ledStripInit(void);
void ledStripEnable(void);
void reevalulateLedConfig(void);

bool setModeColor(ledModeIndex_e modeIndex, int modeColorIndex, int colorIndex);

extern uint16_t rssi; // FIXME dependency on mw.c

