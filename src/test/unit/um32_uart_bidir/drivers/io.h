/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include "drivers/io_types.h"
#include "drivers/resource.h"
#define IO_TAG(pin) (pin)
IO_t IOGetByTag(ioTag_t tag);
void IOConfigGPIO(IO_t io, ioConfig_t config);
void IOConfigGPIOAF(IO_t io, ioConfig_t config, uint8_t af);
void IOInit(IO_t io, resourceOwner_e owner, uint8_t index);
