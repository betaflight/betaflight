#ifndef BOARD_H
#define BOARD_H

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
#define LED_PORT      0
#define LED_PIN       29
#define LED_STATE_ON  0

// WAKE button
#define BUTTON_PORT   0
#define BUTTON_PIN    24

// IOCON pin mux
#define IOCON_PIO_DIGITAL_EN          0x80u   /*!< Enables digital function */
#define IOCON_PIO_FUNC1               0x01u   /*!< Selects pin function 1 */
#define IOCON_PIO_FUNC7               0x07u   /*!< Selects pin function 7 */
#define IOCON_PIO_INPFILT_OFF       0x0100u   /*!< Input filter disabled */
#define IOCON_PIO_INV_DI              0x00u   /*!< Input function is not inverted */
#define IOCON_PIO_MODE_INACT          0x00u   /*!< No addition pin function */
#define IOCON_PIO_OPENDRAIN_DI        0x00u   /*!< Open drain is disabled */
#define IOCON_PIO_SLEW_STANDARD       0x00u   /*!< Standard mode, output slew rate control is enabled */

/****************************************************************
name: BOARD_BootClockFROHF96M
outputs:
- {id: SYSTICK_clock.outFreq, value: 96 MHz}
- {id: System_clock.outFreq, value: 96 MHz}
settings:
- {id: SYSCON.MAINCLKSELA.sel, value: SYSCON.fro_hf}
sources:
- {id: SYSCON.fro_hf.outFreq, value: 96 MHz}
******************************************************************/
static inline void BootClockFROHF96M(void) {
  /*!< Set up the clock sources */
  /*!< Set up FRO */
  POWER_DisablePD(kPDRUNCFG_PD_FRO_EN); /*!< Ensure FRO is on  */
  CLOCK_AttachClk(kFRO12M_to_MAIN_CLK); /*!< Switch to FRO 12MHz first to ensure we can change voltage without
                                             accidentally being below the voltage for current speed */
  POWER_SetVoltageForFreq(96000000U); /*!< Set voltage for the one of the fastest clock outputs: System clock output */
  CLOCK_SetFLASHAccessCyclesForFreq(96000000U); /*!< Set FLASH wait states for core */

  CLOCK_SetupFROClocking(96000000U); /*!< Set up high frequency FRO output to selected frequency */

  /*!< Set up dividers */
  CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false);     /*!< Set AHBCLKDIV divider to value 1 */
  CLOCK_SetClkDiv(kCLOCK_DivSystickClk, 0U, true);  /*!< Reset SYSTICKCLKDIV divider counter and halt it */
  CLOCK_SetClkDiv(kCLOCK_DivSystickClk, 1U, false); /*!< Set SYSTICKCLKDIV divider to value 1 */

  /*!< Set up clock selectors - Attach clocks to the peripheries */
  CLOCK_AttachClk(kFRO_HF_to_MAIN_CLK); /*!< Switch MAIN_CLK to FRO_HF */
  /*!< Set SystemCoreClock variable. */
  SystemCoreClock = 96000000U;
}

#endif
