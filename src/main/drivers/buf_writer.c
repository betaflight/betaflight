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

#include <stdint.h>

#include "buf_writer.h"

bufWriter_t *bufWriterInit(uint8_t *b, int total_size, bufWrite_t writer, void *arg)
{
    bufWriter_t *buf = (bufWriter_t *)b;
    buf->writer = writer;
    buf->arg = arg;
    buf->at = 0;
    buf->capacity = total_size - sizeof(*buf);

    return buf;
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
