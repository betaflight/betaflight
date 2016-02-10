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

#define MAX_LED_STRIP_LENGTH 32
#define CONFIGURABLE_COLOR_COUNT 16

#define LED_X_BIT_OFFSET 4
#define LED_Y_BIT_OFFSET 0

#define LED_XY_MASK (0x0F)

#define GET_LED_X(ledConfig) ((ledConfig->xy >> LED_X_BIT_OFFSET) & LED_XY_MASK)
#define GET_LED_Y(ledConfig) ((ledConfig->xy >> LED_Y_BIT_OFFSET) & LED_XY_MASK)

#define CALCULATE_LED_X(x) ((x & LED_XY_MASK) << LED_X_BIT_OFFSET)
#define CALCULATE_LED_Y(y) ((y & LED_XY_MASK) << LED_Y_BIT_OFFSET)


#define CALCULATE_LED_XY(x,y) (CALCULATE_LED_X(x) | CALCULATE_LED_Y(y))

typedef enum {
    LED_DISABLED = 0,
    LED_DIRECTION_NORTH      = (1 << 0),
    LED_DIRECTION_EAST       = (1 << 1),
    LED_DIRECTION_SOUTH      = (1 << 2),
    LED_DIRECTION_WEST       = (1 << 3),
    LED_DIRECTION_UP         = (1 << 4),
    LED_DIRECTION_DOWN       = (1 << 5),
    LED_FUNCTION_INDICATOR   = (1 << 6),
    LED_FUNCTION_WARNING     = (1 << 7),
    LED_FUNCTION_FLIGHT_MODE = (1 << 8),
    LED_FUNCTION_ARM_STATE   = (1 << 9),
    LED_FUNCTION_THROTTLE    = (1 << 10),
    LED_FUNCTION_THRUST_RING = (1 << 11),
    LED_FUNCTION_COLOR       = (1 << 12),
} ledFlag_e;

#define LED_DIRECTION_BIT_OFFSET 0
#define LED_DIRECTION_MASK ( \
    LED_DIRECTION_NORTH | \
    LED_DIRECTION_EAST | \
    LED_DIRECTION_SOUTH | \
    LED_DIRECTION_WEST | \
    LED_DIRECTION_UP | \
    LED_DIRECTION_DOWN \
)
#define LED_FUNCTION_BIT_OFFSET 6
#define LED_FUNCTION_MASK ( \
    LED_FUNCTION_INDICATOR | \
    LED_FUNCTION_WARNING | \
    LED_FUNCTION_FLIGHT_MODE | \
    LED_FUNCTION_ARM_STATE | \
    LED_FUNCTION_THROTTLE | \
    LED_FUNCTION_THRUST_RING | \
    LED_FUNCTION_COLOR \
)


typedef struct ledConfig_s {
    uint8_t xy;     // see LED_X/Y_MASK defines
    uint8_t color;  // see colors (config_master)
    uint16_t flags; // see ledFlag_e
} ledConfig_t;

extern uint8_t ledCount;
extern uint8_t ledsInRingCount;



bool parseLedStripConfig(uint8_t ledIndex, const char *config);
void updateLedStrip(void);
void updateLedRing(void);

void applyDefaultLedStripConfig(ledConfig_t *ledConfig);
void generateLedConfig(uint8_t ledIndex, char *ledConfigBuffer, size_t bufferSize);

bool parseColor(uint8_t index, const char *colorConfig);
void applyDefaultColors(hsvColor_t *colors, uint8_t colorCount);

void ledStripEnable(void);
void reevalulateLedConfig(void);
