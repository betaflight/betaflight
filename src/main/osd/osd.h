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

#define MAX_OSD_ELEMENT_COUNT 32

typedef struct osdElementConfig_s {
    element_t elements[MAX_OSD_ELEMENT_COUNT];
} osdElementConfig_t;

PG_DECLARE(osdElementConfig_t, osdElementConfig);

typedef struct osdVideoConfig_s {
    uint8_t videoMode;
    // other settings like horizontal/vertical offsets, sync modes, etc probably belong here in the future.
} osdVideoConfig_t;

PG_DECLARE(osdVideoConfig_t, osdVideoConfig);

typedef struct osdState_s {
    videoMode_e videoMode;
    bool cameraConnected;
} osdState_t;

extern osdState_t osdState;

//
// OSD API
//

void osdInit(void);
void osdApplyConfiguration(void);
void osdUpdate(void);

//
// To be implemented by hardware specific OSD code using hardware drivers.
//
void osdHardwareInit(void);
void osdHardwareApplyConfiguration(videoMode_e videoMode);
void osdHardwareUpdate(void);
void osdHardwareCheck(void);
void osdHardwareDrawLogo(void);
bool osdIsCameraConnected(void);
void osdHardwareDisplayMotor(uint8_t x, uint8_t y, uint8_t percent);
