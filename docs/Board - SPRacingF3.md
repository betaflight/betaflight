# Board - SPRacingF3

The Seriously Pro Racing MOF3 board (SPRacingF3) is the first board designed specifically for INAV.

Full details available on the website, here:

http://seriouslypro.com/spracingf3

## Hardware Features

* No compromise I/O. Use all the features all the time; e.g. Connect your OSD + SmartPort + SBus + GPS + LED Strip + Battery Monitoring + HC-SR04 + 8 motors - all at the same time!
* On-board high-capacity black box flight log recorder - optimize your tuning and see the results of your setup without guesswork. (Acro and Deluxe)
* Next-generation STM32 F3 processor with hardware floating point unit for efficient flight calculations and faster ARM-Cortex M4 core.
* Stackable design - perfect for integrating with OSDs and power distribution boards.
* 16 PWM I/O lines for ESCs, Servos and legacy receivers. 8 available on standard pin headers. 8 via side mounted connectors.
* Supports SBus, SumH, SumD, Spektrum1024/2048, XBus, PPM, PWM receivers. No external inverters required (built-in).
* Dedicated output for programmable LEDs - great for orientation, racing and night flying.
* Dedicated I2C port for connection of OLED display without needing flight battery.
* Battery monitoring ports for voltage and current.
* Buzzer port for audible warnings and notifications.
* Solder pads in addition to connectors for HC-SR04, PPM, RSSI, Current, GPIO, LED Strip, 3.3v,
* Developer friendly debugging port (SWD) and boot mode selection, unbrickable bootloader.
* Symmetrical design for a super tidy wiring.
* Wire up using using pin headers, JST-SH sockets or solder pads. Use either right-angled or straight pin-headers.
* Barometer mounted on the bottom of the board for easy wind isolation.

## Serial Ports

| Value | Identifier   | RX           | TX           | 5v Tolerant | Notes                                                                                       |
| ----- | ------------ | ------------ | ------------ | ----------- | ------------------------------------------------------------------------------------------- |
| 1     | USART1       | PA10         | PA9          | YES         | Internally connected to USB port via CP2102 IC.  Also available on a USART1 JST connector and on through hole pins. |
| 2     | USART2       | PA15         | PA14         | YES         | Available on USART2 JST port only. |
| 3     | USART3       | PB11 / IO2_3 | PB10 / IO2_4 | NO          | Available on IO_2, USART3 JST port and through hole pins. |

* You cannot use SWD and USART2 at the same time.
* You may encounter flashing problems if you have something connected to the USART1 RX/TX pins.   Power other devices of and/or disconnect them.

## Pinouts

Full pinout details are available in the manual, here:

http://seriouslypro.com/spracingf3#manual

### IO_1

The 8 pin IO_1 connector has the following pinouts when used in RX_PARALLEL_PWM mode.

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | Ground         |                                              |
| 2   | VCC_IN         | Voltage as-supplied by BEC.                  |
| 3   | RC_CH1         | |
| 4   | RC_CH2         | |
| 5   | RC_CH5         | |
| 6   | RC_CH6         | |
| 7   | LED_STRIP      | Enable `feature LED_STRIP`                   |
| 8   | VCC            | 3.3v output for LOW CURRENT application only |

When RX_PPM/RX_SERIAL is used the IO_1 pinout is as follows.

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | Ground         |                                              |
| 2   | VCC_IN         | Voltage as-supplied by BEC.                  |
| 3   | RX_PPM         | Enable `feature RX_PPM`                      |
| 4   | GPIO           |                                              |
| 5   | SoftSerial1_RX |                                              |
| 6   | SoftSerial1_TX |                                              |
| 7   | LED_STRIP      | Enable `feature LED_STRIP`                   |
| 8   | VCC            | 3.3v output for LOW CURRENT application only |

### IO_2

The 8 pin IO_2 connector has the following pinouts when used in RX_PARALLEL_PWM mode.

| Pin | Function          | Notes                                        |
| --- | ----------------- | -------------------------------------------- |
| 1   | Ground            |                                              |
| 2   | VCC_IN            | Voltage as-supplied by BEC.                  |
| 3   | RC_CH3            |                                              |
| 4   | RC_CH4            |                                              |
| 5   | RC_CH7/HC-SR04_TRIG |                                              |
| 6   | RC_CH8/HC-SR04_ECHO |                                              |
| 7   | ADC_1             | Current Sensor                               |
| 8   | ADC_2             | RSSI                                         |

When RX_PPM/RX_SERIAL is used the IO_2 pinout is as follows.

| Pin | Function                  | Notes                                        |
| --- | ------------------------- | -------------------------------------------- |
| 1   | Ground                    |                                              |
| 2   | VCC_IN                    | Voltage as-supplied by BEC.                  |
| 3   | RX_SERIAL                 | UART3 RX                                     |
| 4   |                           | UART3_TX                                     |
| 5   | HC-SR04_TRIG/SoftSerial2_RX | Enable `feature SOFTSERIAL` or HC-SR04 rangefinder     |
| 6   | HC-SR04_ECHO/SoftSerial2_TX | Enable `feature SOFTSERIAL` or HC-SR04 rangefinder     |
| 7   | ADC_1                     | Current Sensor                               |
| 8   | ADC_2                     | RSSI                                         |

### UART1/2/3

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | Ground         |                                              |
| 2   | VCC_IN         | Voltage as-supplied by BEC.                  |
| 3   | TXD            |                                              |
| 4   | RXD            |                                              |

### I2C

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | Ground         |                                              |
| 2   | 5.0v           | Voltage as-supplied by BEC OR USB, always on |
| 3   | SCL            |                                              |
| 4   | SDA            |                                              |

### SWD

The port cannot be used at the same time as UART2.

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | Ground         |                                              |
| 2   | NRST           | Voltage as-supplied by BEC OR USB, always on |
| 3   | SWDIO          |                                              |
| 4   | SWDCLK         |                                              |
