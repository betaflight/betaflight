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

#include <stdint.h>
#include <stdbool.h>

#include "drivers/io_types.h"

#include "pg/pg.h"

#if defined(USE_RESOURCE_MGMT)

typedef struct {
    const uint8_t owner;
    pgn_t pgn;
    uint8_t stride;
    uint8_t offset;
    const uint8_t maxIndex;
} resourceValue_t;

typedef enum {
    CLEARED = 0,
    ASSIGNED = 1,

    PARSE_ERROR = 10,

    INVALID_RESOURSE_NAME = 11,
    RESOURCE_INDEX_OUT_OF_RANGE = 12,
} resourceApplyResult_t;


bool resource_strToPin(char *ptr, ioTag_t *tag);
ioTag_t *resource_getIoTag(const resourceValue_t value, uint8_t index);
void resource_assign(uint8_t resourceIndex, uint8_t index, ioTag_t newTag);
resourceApplyResult_t resource_apply(const char *args);

void resource_applyDefaults(void);
#endif // USE_RESOURCE_MGMT
