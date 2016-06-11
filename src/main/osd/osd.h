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

typedef struct osdFontConfig_s {
    uint16_t fontVersion;
} osdFontConfig_t;

PG_DECLARE(osdFontConfig_t, osdFontConfig);

typedef struct osdVideoConfig_s {
    uint8_t videoMode;
    // other settings like horizontal/vertical offsets, sync modes, etc probably belong here in the future.
} osdVideoConfig_t;

PG_DECLARE(osdVideoConfig_t, osdVideoConfig);

extern const uint8_t *asciiToFontMapping;

extern textScreen_t osdTextScreen;
extern char textScreenBuffer[];

//
// OSD API
//

void osdInit(void);
void osdApplyConfiguration(void);
void osdUpdate(void);
void osdSetTextScreen(textScreen_t *textScreen);
void osdClearScreen(void);
void osdResetCursor(void);
void osdSetCursor(uint8_t x, uint8_t y);
void osdPrint(char *message);
void osdPrintAt(uint8_t x, uint8_t y, char *message);
void osdSetRawCharacterAtPosition(uint8_t x, uint8_t y, char c);

//
// To be implemented by hardware specific OSD code using hardware drivers.
//
void osdHardwareInit(void);
void osdHardwareApplyConfiguration(void);
void osdHardwareUpdate(void);
void osdHardwareCheck(void);
void osdHardwareDrawLogo(void);
bool osdIsCameraConnected(void);
void osdHardwareDisplayMotor(uint8_t x, uint8_t y, uint8_t percent);
