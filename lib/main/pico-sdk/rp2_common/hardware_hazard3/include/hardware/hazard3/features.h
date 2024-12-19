/*
 * Copyright (c) 2024 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_HAZARD3_FEATURES_H
#define _HARDWARE_HAZARD3_FEATURES_H

#include "pico.h"

/** \file hardware/hazard3/features.h
 *  \addtogroup hardware_hazard3
 *
 * \brief Sets macros for supported Hazard3 custom extensions (features) based on PICO_PLATFORM macros
 *
 */

// Feature detection macros for Hazard3 custom extensions
#if PICO_RP2350
// Version 1.0 of these four extensions
// (encoded as major * 100 + minor)
#define __hazard3_extension_xh3power 100
#define __hazard3_extension_xh3bextm 100
#define __hazard3_extension_xh3irq 100
#define __hazard3_extension_xh3pmpm 100
#endif

#endif
