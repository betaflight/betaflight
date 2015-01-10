# Board - ChebuzzF3

The ChebuzzF3 is a daugter board which connects to the bottom of an STM32F3Discovery board and provides pin headers and ports for various FC connections.

All connections were traced using a multimeter and then verified against the TauLabs source code using the revision linked.

https://github.com/TauLabs/TauLabs/blob/816760dec2a20db7fb9ec1a505add240e696c31f/flight/targets/flyingf3/board-info/board_hw_defs.c

## Connections

Board orientation.

These notes assume that when the board is placed with the header pins facing up, the bottom right of the board is next to the 8 sets of INPUT pin headers.
Inner means between the two rows of header sockets, outer means between the left/right board edges and the header sockets.


### SPI2 / External SPI

sclk GPIOB 13
miso GPIOB 14
mosi GPIOB 15


There are 4 pins, labelled CS1-4 next to a label that reads Ext SPI.  The 3rd pin is connected to the flash chip on
the bottom right inner of the board.  The other pins on the flash chip are wired up to PB3/4/5

### SPI3 / SPI

sclk GPIOB 3
miso GPIOB 4
mosi GPIOB 5

ssel 1 GPIOB 10 / Ext SPI CS1
ssel 2 GPIOB 11 / Ext SPI CS2
ssel 3 GPIOB 12 / Ext SPI CS3 - wired up to Slave Select of M25P16 15MBitFlash chip
ssel 4 GPIOB 13 / Ext SPI CS4 - not usable since it is used for SPI2 sclk

### RC Input

INPUT
PA8 / CH1 - TIM1_CH1
PB8 / CH2 - TIM16_CH1
PB9 / CH3 - TIM17_CH1
PC6 / CH4 - TIM8_CH1
PC7 / CH5 - TIM8_CH2
PC8 / CH6 - TIM8_CH3
PF9 / CH7 - TIM15_CH1
PF10 / CH8 - TIM15_CH2

### PWM Outputs

OUTPUT
PD12 / CH1 - TIM4_CH1
PD13 / CH2 - TIM4_CH2
PD14 / CH3 - TIM4_CH3
PD15 / CH4 - TIM4_CH4
PA1 / CH5 - TIM2_CH2
PA2 / CH6 - TIM2_CH3
PA3 / CH7 - TIM2_CH4
PB0 / CH8 - TIM3_CH3
PB1 / CH9 - TIM3_CH4
PA4 / CH10 - TIM3_CH2 

### Other ports

There is space for a MS5611 pressure sensor at the top left inner of the board.

There is an I2C socket on the left outer of the board which connects to a PCA9306 I2C level shifter directly opposite (inner).
The PCA9306 is not populated on some boards and thus the I2C socket is unusable.

There is a CAN socket on the top right outer of the board which connects to a MAX3015 CAN Tranceiver.
The MAX3015 is not populated on some boards and thus the CAN socket is unusable.

There are some solder pads labelled Ext 1-4 at the top right inner of the board.

GPIOE 6 / PE6 / Ext 1
GPIOD 3 / PD3 / Ext 2
GPIOD 4 / PD4 / Ext 3
GPIOB 3 / PB3 / Ext 4

There are some solder pads labelled ADC0-3 & Diff Press at the top left inner of the board
They are connected to the ADC socket at the top left outer of the board

PC3 / Diff Press - ADC12_IN9 (Differential Pressure)
PC2 / ADC2 - ADC12_IN8
PC1 / ADC1 - ADC12_IN7
PC0 / ADC0 - ADC12_IN6

There is space for a MPXV5004/MPVZ5004 differential pressure sensor, if populated it's analog pin connects to PC3.

There are sockets for 5 UARTs labelled USART1-5.

There is a socket labelled RX_IN.

GPIOD 2 / PD2 / RX_IN
