/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3
 * of the License, or (at your option) any later version.
 */

#pragma once

#include <stdbool.h>

#include "fc/runtime_config.h"

static inline bool mspRebootIsAllowed(void)
{
    return !ARMING_FLAG(ARMED);
}
