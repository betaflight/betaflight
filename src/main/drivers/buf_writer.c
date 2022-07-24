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

#include "platform.h"

#include "buf_writer.h"

void bufWriterInit(bufWriter_t *b, uint8_t *data, int size, bufWrite_t writer, void *arg)
{
    bufWriter_t *buf = (bufWriter_t *)b;
    buf->writer = writer;
    buf->arg = arg;
    buf->at = 0;
    buf->capacity = size;
    buf->data = data;
}

void bufWriterAppend(bufWriter_t *b, uint8_t ch)
{
    b->data[b->at++] = ch;
    if (b->at >= b->capacity) {
        bufWriterFlush(b);
    }
}

void bufWriterFlush(bufWriter_t *b)
{
    if (b->at != 0) {
        b->writer(b->arg, b->data, b->at);
        b->at = 0;
    }
}
