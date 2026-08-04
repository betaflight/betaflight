/*
 * Interrupt handlers for the FSBL stub. Only SysTick is wired up — HAL_Init
 * starts a 1 ms tick that HAL_XSPI_Command and friends use for timeout
 * accounting. Everything else stays defaulted to the spin-loops in the
 * startup file (the FSBL doesn't enable any interrupts beyond SysTick).
 */

#include "main.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
}
