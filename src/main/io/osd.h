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

#define MAX_MENU_ROWS 8
#define MAX_MENU_COLS 3

typedef struct {
    const char* title;
    uint8_t x_pos;
} col_t;

typedef struct {
    const char* title;
    uint8_t     y_pos;
    void (*update)(bool increase, uint8_t col);
    void (*print)(uint16_t pos, uint8_t col);
} row_t;

typedef struct {
    const char* title;
    uint8_t     cols_number;
    uint8_t     rows_number;
    col_t       cols[MAX_MENU_COLS];
    row_t       rows[MAX_MENU_ROWS];
} page_t;

void updateOsd(void);
void osdInit(void);
