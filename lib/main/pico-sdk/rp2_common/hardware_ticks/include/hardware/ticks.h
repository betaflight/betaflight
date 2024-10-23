/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_TICKS_H
#define _HARDWARE_TICKS_H

/** \file hardware/tick.h
 *  \defgroup hardware_ticks hardware_ticks
 *
 * \brief Hardware Tick API
 * 
 * \if rp2040_specific
 * RP2040 only has one tick generator, and it is part of the watchdog hardware.
 * \endif
 * 
 * \if rp2350_specific
 * The RP2350 has a dedicated Tick block that is used to supply ticks to TIMER0, TIMER1, 
 * RISC-V platform timer, Arm Cortex-M33 0 timer, Arm Cortex-M33 1 timer and the WATCHDOG block.
 * \endif
 */

#include "pico.h"
#if !PICO_RP2040
#include "hardware/structs/ticks.h"
#else
#include "hardware/watchdog.h"
/*! \brief Tick generator numbers on RP2040 (used as typedef \ref tick_gen_num_t)
 *  \ingroup hardware_ticks
 *
 * RP2040 only has one tick generator, and it is part of the watchdog hardware
 */
typedef enum tick_gen_num_rp2040 {
    TICK_WATCHDOG = 0,
    TICK_COUNT
} tick_gen_num_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_HARDWARE_TICKS, Enable/disable assertions in the hardware_ticks module, type=bool, default=0, group=hardware_ticks
#ifndef PARAM_ASSERTIONS_ENABLED_HARDWARE_TICKS
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_TICKS 0
#endif

/*! \brief Start a tick generator
 *  \ingroup hardware_ticks
 *
 * \param tick The tick generator number
 * \param cycles The number of clock cycles per tick
 */
void tick_start(tick_gen_num_t tick, uint cycles);


/*! \brief Stop a tick generator
 *  \ingroup hardware_ticks
 *
 * \param tick The tick generator number
 */
void tick_stop(tick_gen_num_t tick);

/*! \brief Check if a tick genererator is currently running
 *  \ingroup hardware_ticks
 *
 * \param tick The tick generator number
 * \return true if the specific ticker is running.
 */
bool tick_is_running(tick_gen_num_t tick);

#ifdef __cplusplus
}
#endif

#endif
