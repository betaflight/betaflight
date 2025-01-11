#ifndef BOARD_H
#define BOARD_H

#define LED_PORT              1
#define LED_PIN               24
#define LED_STATE_ON          0

// Wake up Switch
#define BUTTON_PORT           0
#define BUTTON_PIN            16
#define BUTTON_STATE_ACTIVE   0

/* System oscillator rate and RTC oscillator rate */
const uint32_t OscRateIn = 12000000;
const uint32_t ExtRateIn = 0;

/* Pin muxing table, only items that need changing from their default pin
   state are in this table. Not every pin is mapped. */
/* IOCON pin definitions for pin muxing */
typedef struct {
  uint32_t port : 8;			/* Pin port */
  uint32_t pin : 8;			/* Pin number */
  uint32_t modefunc : 16;		/* Function and mode */
} PINMUX_GRP_T;

static const PINMUX_GRP_T pinmuxing[] = {
    {0,  3, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)}, // USB VBUS
    {0,  6, (IOCON_FUNC1 | IOCON_MODE_INACT)},		/* PIO0_6 used for USB_CONNECT */

    {0, 18, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)}, // UART0 RX
    {0, 19, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)}, // UART0 TX
};

/* Setup system clocking */
static inline void Chip_SetupXtalClocking(void) {
  volatile int i;

  /* Powerup main oscillator */
  Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSOSC_PD);

  /* Wait 200us for OSC to be stablized, no status
     indication, dummy wait. */
  for (i = 0; i < 0x100; i++) {}

  /* Set system PLL input to main oscillator */
  Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_MAINOSC);

  /* Power down PLL to change the PLL divider ratio */
  Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);

  /* Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 4 = 48MHz
     MSEL = 3 (this is pre-decremented), PSEL = 1 (for P = 2)
     FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 4 = 48MHz
     FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range) */
  Chip_Clock_SetupSystemPLL(3, 1);

  /* Powerup system PLL */
  Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);

  /* Wait for PLL to lock */
  while (!Chip_Clock_IsSystemPLLLocked()) {}

  /* Set system clock divider to 1 */
  Chip_Clock_SetSysClockDiv(1);

  /* Setup FLASH access to 3 clocks */
  Chip_FMC_SetFLASHAccess(FLASHTIM_50MHZ_CPU);

  /* Set main clock source to the system PLL. This will drive 48MHz
     for the main clock and 48MHz for the system clock */
  Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);

  /* Set USB PLL input to main oscillator */
  Chip_Clock_SetUSBPLLSource(SYSCTL_PLLCLKSRC_MAINOSC);
  /* Setup USB PLL  (FCLKIN = 12MHz) * 4 = 48MHz
     MSEL = 3 (this is pre-decremented), PSEL = 1 (for P = 2)
     FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 4 = 48MHz
     FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range) */
  Chip_Clock_SetupUSBPLL(3, 1);

  /* Powerup USB PLL */
  Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_USBPLL_PD);

  /* Wait for PLL to lock */
  while (!Chip_Clock_IsUSBPLLLocked()) {}
}

static inline void Chip_USB_Init(void) {
  /* enable USB main clock */
  Chip_Clock_SetUSBClockSource(SYSCTL_USBCLKSRC_PLLOUT, 1);
  /* Enable AHB clock to the USB block and USB RAM. */
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_USB);
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_USBRAM);
  /* power UP USB Phy */
  Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_USBPAD_PD);
}

#endif
