/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/runtime.h"

void __weak hard_assertion_failure(void) {
    panic("Hard assert");
}
