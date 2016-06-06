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

void max7456_hardwareReset(void);
void max7456_init(void);

void max7456_resetFont(void);

uint8_t max7456_readStatus(void);
bool max7456_isOSDEnabled(void);

void max7456_clearScreen(void);
void max7456_clearScreenAtNextVSync(void);

void max7456_showFont(void);

void max7465_print(uint8_t x, uint8_t y, char *message);

void max7456_setCharacterAtPosition(uint8_t x, uint8_t y, uint8_t c);


void max7456_fillScreen(void);
