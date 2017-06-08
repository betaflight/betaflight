# Board - Paris Air Hero 32 / Acro Naze 32 Mini

This is the AIR3 PARIS Sirius AirHERO 32 F3 board from MultiWiiCopter
Source: http://www.multiwiicopter.com/products/inav-air3-fixed-wing

## Sensors

MPU6500 via SPI interface.
BMP280 via SPI interface

## Ports

6 x 3pin ESC / Servo outputs
1 x 8pin JST connector (PPM/PWM/UART2)
1 x 4pin JST connector (UART3)

## I2C bus

I2C bus is made available with a special target - AIRHEROF3_QUAD. This target limits motor outputs to 4 and adds I2C bus at M5/M6 connectors.

## Pinouts

The 10 pin RC I/O connector has the following pinouts when used in RX_PPM/RX_SERIAL mode.

From right to left when looking at the socket from the edge of the board.

| Pin | Function       | Notes                            |
| --- | -------------- | -------------------------------- |
| 1   | Ground         |                                  |
| 2   | +5V            |                                  |
| 3   | RX_PPM         | Enable `feature RX_PPM`          | 
| 4   | AIRSPEED       | Airspeed sensor (3.3V max)       | 
| 5   | USART2 TX      |                                  | 
| 6   | USART2 RX      |                                  | 
| 7   | SS1 RX         | Enable `feature SOFT_SERIAL`     |
| 8   | SS1 TX         |                                  |


## Serial Ports

| Value | Identifier   | RX         | TX                 | Notes                                                                                       |
| ----- | ------------ | ---------- | ------------------ | ------------------------------------------------------------------------------------------- |
| 1     | USART1       | RX  / PA10 | TX  / PA9          | Internally connected to USB port via CP2102 IC                                              |
| 2     | USART2       | RC4 / PA3  | RC3 / PA2          |                                                                                             |
| 3     | USART3       | F3  / PB11 | F2  / PB10         |                                                                                             |

