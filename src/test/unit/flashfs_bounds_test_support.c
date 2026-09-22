/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3
 * of the License, or (at your option) any later version.
 */

#include "platform.h"

#include "io/flashfs.h"

unsigned int testFlashfsReadLength(uint32_t volumeSize, uint32_t address, unsigned int requestedLength)
{
    return flashfsReadLength(volumeSize, address, requestedLength);
}
