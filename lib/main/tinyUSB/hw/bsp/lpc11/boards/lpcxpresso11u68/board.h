#ifndef BOARD_H
#define BOARD_H

#define LED_PORT              2
#define LED_PIN               17
#define LED_STATE_ON          0

// Wake up Switch
#define BUTTON_PORT           0
#define BUTTON_PIN            16
#define BUTTON_STATE_ACTIVE   0

/* System oscillator rate and RTC oscillator rate */
const uint32_t OscRateIn = 12000000;
const uint32_t RTCOscRateIn = 32768;

/* Pin muxing table, only items that need changing from their default pin
   state are in this table. Not every pin is mapped. */
static const PINMUX_GRP_T pinmuxing[] = {
    {0, 3,  (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)}, // USB VBUS
    {0, 18, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)}, // UART0 RX
    {0, 19, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)}, // UART0 TX
    {2, 0,  (IOCON_FUNC1 | IOCON_MODE_INACT)}, // XTALIN
    {2, 1,  (IOCON_FUNC1 | IOCON_MODE_INACT)}, // XTALOUT
};

#endif
