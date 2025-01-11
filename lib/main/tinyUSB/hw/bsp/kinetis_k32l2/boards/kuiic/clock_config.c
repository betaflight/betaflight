#include "clock_config.h"
#include "fsl_clock.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
// extern uint32_t SystemCoreClock;

/*******************************************************************************
 * Variables for BOARD_BootClockRUN configuration
 ******************************************************************************/
const mcglite_config_t mcgliteConfig_BOARD_BootClockRUN = {
    .outSrc          = kMCGLITE_ClkSrcHirc,  /* MCGOUTCLK source is HIRC */
    .irclkEnableMode = kMCGLITE_IrclkEnable, /* MCGIRCLK enabled, MCGIRCLK disabled in STOP mode */
    .ircs            = kMCGLITE_Lirc8M,      /* Slow internal reference (LIRC) 8 MHz clock selected */
    .fcrdiv          = kMCGLITE_LircDivBy1,  /* Low-frequency Internal Reference Clock Divider: divided by 1 */
    .lircDiv2        = kMCGLITE_LircDivBy1,  /* Second Low-frequency Internal Reference Clock Divider: divided by 1 */
    .hircEnableInNotHircMode = true,         /* HIRC source is enabled */
};
const sim_clock_config_t simConfig_BOARD_BootClockRUN = {
    .er32kSrc = SIM_OSC32KSEL_LPO_CLK,       /* OSC32KSEL select: LPO clock */
    .clkdiv1  = 0x10000U,                    /* SIM_CLKDIV1 - OUTDIV1: /1, OUTDIV4: /2 */
};

/*******************************************************************************
 * Code for BOARD_BootClockRUN configuration
 ******************************************************************************/
void BOARD_BootClockRUN(void)
{
  /* Set the system clock dividers in SIM to safe value. */
  CLOCK_SetSimSafeDivs();
  /* Set MCG to HIRC mode. */
  CLOCK_SetMcgliteConfig(&mcgliteConfig_BOARD_BootClockRUN);
  /* Set the clock configuration in SIM module. */
  CLOCK_SetSimConfig(&simConfig_BOARD_BootClockRUN);
  /* Set SystemCoreClock variable. */
  SystemCoreClock = BOARD_BOOTCLOCKRUN_CORE_CLOCK;
}
