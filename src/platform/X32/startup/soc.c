/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "x32m7xx.h"

/* ---- Section attributes ---- */
#define BOOT_CODE   __attribute__((used, section(".boot_text")))

/* ---- Linker-script symbols ---- */
extern uint32_t __image_copy_load_start__;
extern uint32_t __image_copy_start__;
extern uint32_t __image_copy_end__;

extern uint32_t __tcm_vector_start__;
extern uint32_t __tcm_vector_end__;

extern uint32_t __itcm_text_load_start__;
extern uint32_t __itcm_text_start__;
extern uint32_t __itcm_text_end__;

extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;

extern uint32_t _sbss;
extern uint32_t _ebss;

extern uint32_t __app_data_load_start__;
extern uint32_t __app_data_start__;
extern uint32_t __app_data_end__;

extern uint32_t __app_bss_start__;
extern uint32_t __app_bss_end__;

extern uint32_t _app_estack;

/* SystemInit() is declared by the device headers. */
extern int main(int argc, char * argv[]);

/* ======================================================================
 * Boot-stage helpers -- copied nowhere, run directly from FLASH
 * ====================================================================== */

BOOT_CODE static void boot_copy_words(volatile uint32_t *dst, volatile const uint32_t *src, volatile uint32_t *end)
{
    while (dst < end)
    {
        *dst++ = *src++;
    }
}

BOOT_CODE static void boot_zero_words(volatile uint32_t *start, volatile uint32_t *end)
{
    while (start < end)
    {
        *start++ = 0U;
    }
}

BOOT_CODE static void relocate_runtime_image(void)
{
    volatile uint32_t *src = &__image_copy_load_start__;
    volatile uint32_t *dst = &__image_copy_start__;
    uint32_t *end = &__image_copy_end__;

    if (dst != src)
    {
        boot_copy_words(dst, src, end);
    }
    dst[15] = src[250];
}

/* ======================================================================
 * lowlevel_init -- called from reset (startup.S)
 *
 * Equivalent to u-boot's lowlevel_init plus cache enable.
 * Cortex-M7 has no CP15, so SCB helpers are used instead.
 * ====================================================================== */

BOOT_CODE static void cache_init(void)
{
    /* Invalidate before enabling to avoid stale cache state. */
    // SCB_InvalidateICache();
    // SCB_InvalidateDCache();
    // SCB_EnableICache();
    // SCB_EnableDCache();
}

BOOT_CODE void lowlevel_init(void)
{
    /* Interrupts are already disabled by reset in startup.S. */
    SystemInit();

    /* Keep boot running on default clock; main() switches PLL later. */
    cache_init();
}

/* ======================================================================
 * _main -- relocate runtime image, switch execution context, jump to app
 *
 * This mirrors U-Boot's relocate_code() at a high level:
 *   - copy one contiguous runtime image into its execution address
 *   - initialize runtime data sections that live outside the image
 *   - switch VTOR/MSP and branch into the relocated program
 * ====================================================================== */

BOOT_CODE void _main(void)
{
    /* Relocate one contiguous ITCM runtime image, like U-Boot's image copy. */
    relocate_runtime_image();

    /* Data sections outside the main runtime image still need dedicated init. */
    boot_copy_words(&_sdata, &_sidata, &_edata);
    boot_zero_words(&_sbss, &_ebss);
    boot_copy_words(&__app_data_start__,
                    &__app_data_load_start__,
                    &__app_data_end__);
    boot_zero_words(&__app_bss_start__, &__app_bss_end__);

    /* Switch exception entry to the relocated runtime vector table. */
    SCB->VTOR = (uint32_t)(uintptr_t)&__tcm_vector_start__;
    __DSB();
    __ISB();

    /* Switch to the application runtime stack in DTCM. */
    __set_MSP((uint32_t)(uintptr_t)&_app_estack);
    __DSB();
    __ISB();

    /* Runtime image now lives in ITCM/DTCM, so VTOR/MSP switch only needs
     * DSB/ISB ordering here; no extra D-cache clean is required. */
    __enable_fault_irq();   /* cpsie f -- clear FAULTMASK (defensive) */
    __enable_irq();         /* cpsie i -- clear PRIMASK               */
    __asm volatile ("bx %0" :: "r"(main));

    while (1) {}
}
