/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
// Host tests execute interrupt callbacks explicitly on one thread.
#define ATOMIC_BLOCK(priority) for (int once = 1; once; once = 0)
