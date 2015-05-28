# Board - Paris Air Hero 32 / Acro Naze 32 Mini

This board uses the same firmware as the Naze32 board.

## Sensors

MPU6500 via SPI interface.

## Ports

6 x 3pin ESC / Servo outputs
1 x 8pin JST connector (PPM/PWM/UART2)
1 x 4pin JST connector (UART3/I2C)

## Pinouts

The 10 pin RC I/O connector has the following pinouts when used in RX_PPM/RX_SERIAL mode.

From right to left when looking at the socket from the edge of the board.

| Pin | Function       | Notes                            |
| --- | -------------- | -------------------------------- |
| 1   | Ground         |                                  |
| 2   | +5V            |                                  |
| 3   | RX_PPM         | Enable `feature RX_PPM`          | 
| 4   | RSSI_ADC       | Enable `feature RSSI_ADC`.  Connect to the output of a PWM-RSSI conditioner, 0v-3.3v input | 
| 5   | USART2 TX      |                                  | 
| 6   | USART2 RX      | Built-in inverter                | 
| 7   | LED_STRIP      | Enable `feature LED_STRIP`       |
| 8   | unused         |                                  |

When SOFTSERIAL is enabled, LED_STRIP and CURRENT_METER are unavailable, but one SoftSerial port is made available to use instead.

| Pin | Function       | Notes                            |
| --- | -------------- | -------------------------------- |
| 7   | SOFTSERIAL1 RX | Enable `feature SOFTSERIAL`      |
| 8   | SOFTSERIAL1 TX |                                  |


## Serial Ports

| Value | Identifier   | RX         | TX                 | Notes                                                                                       |
| ----- | ------------ | ---------- | ------------------ | ------------------------------------------------------------------------------------------- |
| 1     | USART1       | RX  / PA10 | TX  / PA9 / TELEM  | TELEM output is always inverted (for FrSky). Internally connected to USB port via CP2102 IC |
| 2     | USART2       | RC4 / PA3  | RC3 / PA2          |                                                                                             |
| 3     | USART3       | F3  / PB11 | F2  / PB10         | Flex port is configured as UART3 when port is configured                                    |
| 4     | SOFTSERIAL1  | RC5 / PA6  | RC6 / PA7          |                                                                                             |

