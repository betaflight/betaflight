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

#include "pg/pg.h"
#include "drivers/display.h"

#define CRSF_DISPLAY_PORT_ROWS_MAX 8
#define CRSF_DISPLAY_PORT_COLS_MAX 32

typedef struct crsfDisplayPortRow_s {
    char data[CRSF_DISPLAY_PORT_COLS_MAX];
    bool pendingTransport;
} crsfDisplayPortRow_t;

typedef struct crsfDisplayPortScreen_s {
    crsfDisplayPortRow_t rows[CRSF_DISPLAY_PORT_ROWS_MAX];
    bool reset;
} crsfDisplayPortScreen_t;

PG_DECLARE(displayPortProfile_t, displayPortProfileCrsf);

struct displayPort_s;
struct displayPort_s *displayPortCrsfInit(void);
crsfDisplayPortScreen_t *crsfDisplayPortScreen(void);
void crsfDisplayPortMenuOpen(void);
void crsfDisplayPortMenuExit(void);
void crsfDisplayPortRefresh(void);
int crsfDisplayPortNextRow(void);
