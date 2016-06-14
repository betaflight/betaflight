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


#define MAX7456_COLUMN_COUNT 30

#define MAX7456_PAL_ROW_COUNT 16
#define MAX7456_PAL_CHARACTER_COUNT (MAX7456_COLUMN_COUNT * MAX7456_PAL_ROW_COUNT)

#define MAX7456_NTSC_ROW_COUNT 13
#define MAX7456_NTSC_CHARACTER_COUNT (MAX7456_COLUMN_COUNT * MAX7456_NTSC_ROW_COUNT)

typedef struct max7456State_s {
    bool los;
    uint32_t losCounter;
    uint32_t frameCounter;

    volatile bool vSyncDetected;
    volatile bool hSyncDetected;
} max7456State_t;

extern max7456State_t max7456State;

void max7456_hardwareReset(void);
void max7456_init(videoMode_e videoMode);
void max7456_extiConfigure(
    const extiConfig_t *losExtiConfig,
    const extiConfig_t *vSyncExtiConfig,
    const extiConfig_t *hSyncExtiConfig
);
void max7456_resetFont(void);
void max7456_updateLOSState(void);

//
// These methods talk to the hardware directly, ignoring the OSD screen buffer.
// TODO Delete unused methods.
//

uint8_t max7456_readStatus(void);
bool max7456_isOSDEnabled(void);

void max7456_clearScreen(void);
void max7456_clearScreenAtNextVSync(void);

void max7456_showFont(void);

void max7465_printAt(uint8_t x, uint8_t y, char *message);

void max7456_setCharacterAtPosition(uint8_t x, uint8_t y, uint8_t c);
void max7456_setCharacterAtCursor(uint8_t c);

void max7456_fillScreen(void);

//
// Text Screen API
//

textScreen_t *max7456_getTextScreen(void);
void max7456_writeScreen(textScreen_t *textScreen, char *screenBuffer);
