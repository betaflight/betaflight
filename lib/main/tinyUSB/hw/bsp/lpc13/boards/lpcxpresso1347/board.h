#ifndef BOARD_H
#define BOARD_H

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
#define LED_PORT      0
#define LED_PIN       7

// Joytick Down if connected to LPCXpresso Base board
#define BUTTON_PORT   1
#define BUTTON_PIN    20

//static const struct {
//  uint8_t port;
//  uint8_t pin;
//} buttons[] =
//{
//    {1, 22 }, // Joystick up
//    {1, 20 }, // Joystick down
//    {1, 23 }, // Joystick left
//    {1, 21 }, // Joystick right
//    {1, 19 }, // Joystick press
//    {0, 1  }, // SW3
//};

/* System oscillator rate and RTC oscillator rate */
const uint32_t OscRateIn = 12000000;
const uint32_t ExtRateIn = 0;

/* Pin muxing table, only items that need changing from their default pin
   state are in this table. */
static const PINMUX_GRP_T pinmuxing[] = {
    {0, 1,  (IOCON_FUNC1 | IOCON_RESERVED_BIT_7 | IOCON_MODE_INACT)},  /* PIO0_1 used for CLKOUT */
    {0, 2,  (IOCON_FUNC1 | IOCON_RESERVED_BIT_7 | IOCON_MODE_PULLUP)},  /* PIO0_2 used for SSEL */
    {0, 3,  (IOCON_FUNC1 | IOCON_RESERVED_BIT_7 | IOCON_MODE_INACT)},  /* PIO0_3 used for USB_VBUS */
    {0, 6,  (IOCON_FUNC1 | IOCON_RESERVED_BIT_7 | IOCON_MODE_INACT)},  /* PIO0_6 used for USB_CONNECT */
    {0, 8,  (IOCON_FUNC1 | IOCON_RESERVED_BIT_7 | IOCON_MODE_INACT)},  /* PIO0_8 used for MISO0 */
    {0, 9,  (IOCON_FUNC1 | IOCON_RESERVED_BIT_7 | IOCON_MODE_INACT)},  /* PIO0_9 used for MOSI0 */
    {0, 11, (IOCON_FUNC2 | IOCON_ADMODE_EN | IOCON_FILT_DIS)},  /* PIO0_11 used for AD0 */
    {0, 18, (IOCON_FUNC1 | IOCON_RESERVED_BIT_7 | IOCON_MODE_INACT)},  /* PIO0_18 used for RXD */
    {0, 19, (IOCON_FUNC1 | IOCON_RESERVED_BIT_7 | IOCON_MODE_INACT)},  /* PIO0_19 used for TXD */
    {1, 29, (IOCON_FUNC1 | IOCON_RESERVED_BIT_7 | IOCON_MODE_INACT)},  /* PIO1_29 used for SCK0 */
};

#endif
