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

//#define ENABLE_DEBUG_OLED_PAGE

typedef enum {
    PAGE_WELCOME,
    PAGE_ARMED,
    PAGE_BATTERY,
    PAGE_SENSORS,
    PAGE_RX,
    PAGE_PROFILE,
#ifdef GPS
    PAGE_GPS,
#endif
#ifdef ENABLE_DEBUG_OLED_PAGE
    PAGE_DEBUG,
#endif
} pageId_e;

void updateDisplay(void);

void displayShowFixedPage(pageId_e pageId);

void displayEnablePageCycling(void);
void displayDisablePageCycling(void);
void displayResetPageCycling(void);
void displaySetNextPageChangeAt(uint32_t futureMicros);
