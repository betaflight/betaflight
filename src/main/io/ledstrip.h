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

#pragma once

#include "common/color.h"
#include "common/time.h"

#include "drivers/io_types.h"
#include "drivers/light_ws2811strip.h"

#include "pg/pg.h"

#define TASK_LEDSTRIP_RATE_HZ 60

#define LED_CONFIGURABLE_COLOR_COUNT     16
#define LED_FCSTATE_COUNT                 (LED_FCSTATE_BARO + 1)
#define LED_DIRECTION_COUNT               6
#define LED_BASEFUNCTION_COUNT           10
#define LED_OVERLAY_COUNT                 7
#define LED_FCSTATE_SPECIAL_COLOR_COUNT  11

#define LED_POS_OFFSET                  0
#define LED_FUNCTION_OFFSET             8
#define LED_OVERLAY_OFFSET             12
#define LED_COLOR_OFFSET               22
#define LED_DIRECTION_OFFSET           26

#define LED_POS_BITCNT                  8
#define LED_FUNCTION_BITCNT             4
#define LED_OVERLAY_BITCNT              10
#define LED_COLOR_BITCNT                4
#define LED_DIRECTION_BITCNT            6

#define LED_MOV_POS(pos) ((pos) << LED_POS_OFFSET)
#define LED_MOV_FUNCTION(func) ((func) << LED_FUNCTION_OFFSET)
#define LED_MOV_OVERLAY(overlay) ((overlay) << LED_OVERLAY_OFFSET)
#define LED_MOV_COLOR(colorId) ((colorId) << LED_COLOR_OFFSET)
#define LED_MOV_DIRECTION(direction) ((direction) << LED_DIRECTION_OFFSET)

#define LED_BIT_MASK(len) ((1 << (len)) - 1)

#define LED_POS_MASK LED_MOV_POS(((1 << LED_POS_BITCNT) - 1))
#define LED_FUNCTION_MASK LED_MOV_FUNCTION(((1 << LED_FUNCTION_BITCNT) - 1))
#define LED_OVERLAY_MASK LED_MOV_OVERLAY(((1 << LED_OVERLAY_BITCNT) - 1))
#define LED_COLOR_MASK LED_MOV_COLOR(((1 << LED_COLOR_BITCNT) - 1))
#define LED_DIRECTION_MASK LED_MOV_DIRECTION(((1 << LED_DIRECTION_BITCNT) - 1))

#define LED_FLAG_OVERLAY(id) (1 << (id))
#define LED_FLAG_DIRECTION(id) (1 << (id))

#define LED_X_BIT_OFFSET 4
#define LED_Y_BIT_OFFSET 0
#define LED_XY_MASK      0x0F
#define CALCULATE_LED_XY(x, y) ((((x) & LED_XY_MASK) << LED_X_BIT_OFFSET) | (((y) & LED_XY_MASK) << LED_Y_BIT_OFFSET))

#define LED_TARGET_UPDATE_US 20

typedef enum {
    COLOR_BLACK = 0,
    COLOR_WHITE,
    COLOR_RED,
    COLOR_ORANGE,
    COLOR_YELLOW,
    COLOR_LIME_GREEN,
    COLOR_GREEN,
    COLOR_MINT_GREEN,
    COLOR_CYAN,
    COLOR_LIGHT_BLUE,
    COLOR_BLUE,
    COLOR_DARK_VIOLET,
    COLOR_MAGENTA,
    COLOR_DEEP_PINK,
    COLOR_COUNT
} colorId_e;

typedef enum {
    LED_FCSTATE_ORIENTATION = 0,
    LED_FCSTATE_HEADFREE,
    LED_FCSTATE_HORIZON,
    LED_FCSTATE_ANGLE,
    LED_FCSTATE_MAG,
    LED_FCSTATE_BARO,
    LED_FCSTATE_SPECIAL,
    LED_FCSTATE_AUX_CHANNEL
} ledFCStates_e;

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
    LED_FUNCTION_GPS_BAR,
    LED_FUNCTION_BATTERY_BAR,
    LED_FUNCTION_ALTITUDE
} ledBaseFunctionId_e;

typedef enum {
    LED_OVERLAY_THROTTLE,
    LED_OVERLAY_RAINBOW,
    LED_OVERLAY_LARSON_SCANNER,
    LED_OVERLAY_BLINK,
    LED_OVERLAY_VTX,
    LED_OVERLAY_INDICATOR,
    LED_OVERLAY_WARNING
} ledOverlayId_e;

typedef enum {
    LED_STRIP_MODE_RACE = 0,
    LED_STRIP_MODE_BEACON,
#ifdef USE_LED_STRIP_DETAILED_MODE
    LED_STRIP_MODE_DETAILED,
#endif
    LED_STRIP_MODE_COUNT
} ledStripMode_e;

typedef struct modeColorIndexes_s {
    uint8_t color[LED_DIRECTION_COUNT];
} modeColorIndexes_t;

typedef struct specialColorIndexes_s {
    uint8_t color[LED_FCSTATE_SPECIAL_COLOR_COUNT];
} specialColorIndexes_t;

typedef uint32_t ledConfig_t;

typedef struct ledCounts_s {
    uint8_t count;
    uint8_t ring;
    uint8_t larson;
    uint8_t ringSeqLen;
} ledCounts_t;

typedef struct ledStripConfig_s {
    uint8_t ledstrip_visual_beeper;
    ioTag_t ioTag;
    ledStripFormatRGB_e ledstrip_grb_rgb;
    ledStripMode_e ledstrip_mode;
    colorId_e ledstrip_race_color;
    colorId_e ledstrip_beacon_color;
    uint16_t ledstrip_beacon_period_ms;
    uint8_t ledstrip_beacon_percent;
    uint8_t ledstrip_beacon_armed_only;
    colorId_e ledstrip_visual_beeper_color;
    uint8_t ledstrip_brightness;
    uint16_t ledstrip_rainbow_delta;
    uint16_t ledstrip_rainbow_freq;
} ledStripConfig_t;

PG_DECLARE(ledStripConfig_t, ledStripConfig);

#if defined(USE_LED_STRIP_DETAILED_MODE)
typedef struct ledStripDetailedModeConfig_s {
    ledConfig_t ledConfigs[LED_STRIP_MAX_LENGTH];
    hsvColor_t colors[LED_CONFIGURABLE_COLOR_COUNT];
    modeColorIndexes_t modeColors[LED_FCSTATE_COUNT];
    specialColorIndexes_t specialColors;
    uint8_t ledstrip_aux_channel;
} ledStripDetailedModeConfig_t;

PG_DECLARE(ledStripDetailedModeConfig_t, ledStripDetailedModeConfig);
#endif

#define LF(name) LED_FUNCTION_ ## name
#define LO(name) LED_FLAG_OVERLAY(LED_OVERLAY_ ## name)
#define LD(name) LED_FLAG_DIRECTION(LED_DIRECTION_ ## name)
#define DEFINE_LED(x, y, col, dir, func, ol) (LED_MOV_POS(CALCULATE_LED_XY(x, y)) | LED_MOV_COLOR(col) | LED_MOV_DIRECTION(dir) | LED_MOV_FUNCTION(func) | LED_MOV_OVERLAY(ol))

static inline uint8_t ledGetXY(const ledConfig_t *lcfg)         { return ((*lcfg >> LED_POS_OFFSET) & LED_BIT_MASK(LED_POS_BITCNT)); }
static inline uint8_t ledGetX(const ledConfig_t *lcfg)          { return ((*lcfg >> (LED_POS_OFFSET + LED_X_BIT_OFFSET)) & LED_XY_MASK); }
static inline uint8_t ledGetY(const ledConfig_t *lcfg)          { return ((*lcfg >> (LED_POS_OFFSET + LED_Y_BIT_OFFSET)) & LED_XY_MASK); }
static inline uint8_t ledGetFunction(const ledConfig_t *lcfg)   { return ((*lcfg >> LED_FUNCTION_OFFSET) & LED_BIT_MASK(LED_FUNCTION_BITCNT)); }
static inline uint8_t ledGetOverlay(const ledConfig_t *lcfg)    { return ((*lcfg >> LED_OVERLAY_OFFSET) & LED_BIT_MASK(LED_OVERLAY_BITCNT)); }
static inline uint8_t ledGetColor(const ledConfig_t *lcfg)      { return ((*lcfg >> LED_COLOR_OFFSET) & LED_BIT_MASK(LED_COLOR_BITCNT)); }
static inline uint8_t ledGetDirection(const ledConfig_t *lcfg)  { return ((*lcfg >> LED_DIRECTION_OFFSET) & LED_BIT_MASK(LED_DIRECTION_BITCNT)); }

static inline bool ledGetOverlayBit(const ledConfig_t *lcfg, int id) { return ((ledGetOverlay(lcfg) >> id) & 1); }
static inline bool ledGetDirectionBit(const ledConfig_t *lcfg, int id) { return ((ledGetDirection(lcfg) >> id) & 1); }

bool parseColor(int index, const char *colorConfig);

bool parseLedStripConfig(int ledIndex, const char *config);
void generateLedConfig(ledConfig_t *ledConfig, char *ledConfigBuffer, size_t bufferSize);
void reevaluateLedConfig(void);

void ledStripInit(void);
void ledStripEnable(void);
void ledStripDisable(void);
void ledStripUpdate(timeUs_t currentTimeUs);

bool setModeColor(ledFCStates_e fcStatusIndex, int modeColorIndex, int colorIndex);

void applyDefaultLedStripConfig(ledConfig_t *ledConfig);
void applyDefaultColors(hsvColor_t *colors);
void applyDefaultModeColors(modeColorIndexes_t *modeColors);
void applyDefaultSpecialColors(specialColorIndexes_t *specialColors);

void updateRequiredOverlay(void);

uint8_t getledStripMode(void);
void setledStripMode(uint8_t profile);
