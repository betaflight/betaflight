/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico.h"
#include "hardware/vreg.h"

void vreg_set_voltage(enum vreg_voltage voltage) {
#if PICO_RP2040

    hw_write_masked(
        &vreg_and_chip_reset_hw->vreg,
        ((uint)voltage) << VREG_AND_CHIP_RESET_VREG_VSEL_LSB,
        VREG_AND_CHIP_RESET_VREG_VSEL_BITS
    );

#elif PICO_RP2350

    hw_set_bits(&powman_hw->vreg_ctrl, POWMAN_PASSWORD_BITS | POWMAN_VREG_CTRL_UNLOCK_BITS);

    // Wait for any prior change to finish before making a new change
    while (powman_hw->vreg & POWMAN_VREG_UPDATE_IN_PROGRESS_BITS)
        tight_loop_contents();

    hw_write_masked(
        &powman_hw->vreg,
        POWMAN_PASSWORD_BITS | ((uint)voltage << POWMAN_VREG_VSEL_LSB),
        POWMAN_PASSWORD_BITS | POWMAN_VREG_VSEL_BITS
    );
    while (powman_hw->vreg & POWMAN_VREG_UPDATE_IN_PROGRESS_BITS)
        tight_loop_contents();

#else
    panic_unsupported();
#endif
}

void vreg_disable_voltage_limit(void) {
#if PICO_RP2040
    // The voltage limit can't be disabled on RP2040 (was implemented by
    // hardwiring the LDO controls)
    return;
#elif PICO_RP2350
    hw_set_bits(&powman_hw->vreg_ctrl, POWMAN_PASSWORD_BITS | POWMAN_VREG_CTRL_DISABLE_VOLTAGE_LIMIT_BITS);
#else
    panic_unsupported();
#endif
}
