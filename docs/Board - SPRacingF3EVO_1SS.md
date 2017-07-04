# Board - Seriously Pro SP Racing F3 EVO

The Seriously Pro Racing F3 Evo board (SPRacingF3EVO) is the evolution of the first board designed specifically for Cleanflight.

Purchasing boards directly from SeriouslyPro / SP Racing and official retailers helps fund Cleanflight development, it's the reason the Seriously Pro boards exist!  Official retailers are always listed on the SeriouslyPro.com website.

Full details available on the website, here:

http://seriouslypro.com/spracingf3evo

## Hardware Features

* Next-generation STM32 F3 processor with hardware floating point unit for efficient flight calculations and faster ARM-Cortex M4 core.
* MicroSD-Card socket for black box flight log recorder - optimize your tuning and see the results of your setup without guesswork.
* Race transponder built in - just turn up at a race and have your lap times recorded.
* Features the latest Accelerometer, Gyro and Mag/Compass and Baro/Altitude sensor technology.
* Wire up using using pin headers for all major connections for excellent crash-durability.  Use either right-angled or straight pin-headers.
* No compromise I/O. Use all the features all the time; e.g. Connect your USB + OSD + SmartPort + SBus + GPS + LED Strip + Battery Monitoring + 8 motors - all at the same time!
* 8 PWM output lines for ESCs and Servos. Arranged for easy wiring on standard pin headers.
* Supports direct connection of SBus, SumH, SumD, Spektrum1024/2048, XBus receivers. No external inverters required (built-in).
* Supports direct connection of 3.3v Spektrum Satellite receivers via 3 pin through-hole JST-ZH connector.
* Dedicated PPM receiver input.
* 3 Serial Ports - NOT shared with the USB socket.
* Telemetry port
* Micro USB socket. 
* Dedicated output for programmable LEDs - great for orientation, racing and night flying. (Currently mutually exclusive with the Transponder).
* Dedicated I2C port for connection of OLED display without needing flight battery.
* Battery monitoring for voltage and current.
* RSSI monitoring (analogue or PWM).
* Buzzer port for audible warnings and notifications.
* Developer friendly debugging port (SWD) and boot mode selection, unbrickable bootloader.
* Symmetrical design for a super tidy wiring.
* JST-SH sockets only for I2C, UART2 and SWD.  UART2 also on through-hole pins.
* Flashing via USB or serial port.
* Stackable design - perfect for integrating with OSDs and power distribution boards.
* Standard mounting - 36x36mm with standard 30.5mm mounting holes.
* LEDs for 3v, 5v and Status for easy diagnostics.
* Copper-etched Cleanflight logo.

## Serial Ports

| Value | Identifier   | RX           | TX           | 5v Tolerant | Notes                                                                                       |
| ----- | ------------ | ------------ | ------------ | ----------- | ------------------------------------------------------------------------------------------- |
| 1     | USART1       | PA10         | PA9          | YES         | 2 through-hole pins. Use for connecting to OSD/GPS/BlueTooth. |
| 2     | USART2       | PA15         | PA14 / SWCLK | YES         | JST socket and PPM header. Use to connect to RX. |
| 3     | USART3       | PB11 / AF7   | PB10 / AF7   | NO          | Available on 4 through-hole pins. 3.3V signals only ! Use for GPS, Spektrum Satellite RX, SmartPort Telemetry, HoTT telemetry, etc. |

* You cannot use SWD and USART2 at the same time.
* When using a Serial RX receiver the TXD (T2) pin cannot be used for telemetry. Use UART3 TXD instead.
* One Software serial is supported in th SPRacingF3EVO_1SS version, see table below
* Windows DFU Flashing requires Zadig (see configurator)

### SoftSerial

| Pin | Function       | Notes                            |
| --- | -------------- | -------------------------------- |
| 7   | SOFTSERIAL1 RX | SoftSerial disables Servo 5,6    |
| 8   | SOFTSERIAL1 TX |                                  |

## Pinouts

Full pinout details are available in the manual, here:

http://seriouslypro.com/files/SPRacingF3EVO-Manual-latest.pdf

### IO_1

The 6 pin IO_1 connector has the following pinouts when used in RX_SERIAL mode.

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | Ground         |                                              |
| 2   | VCC_IN         | Voltage as-supplied by BEC.                  |
| 3   | RX_SERIAL      | Enable `feature RX_SERIAL` |
| 4   | | |
| 5   | +V BATTERY     | Voltage as-supplied by Battery.              |
| 6   | -V BATTERY     | Voltage as-supplied by Battery.              |

When RX_PPM is used the IO_1 pinout is as follows.

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | Ground         |                                              |
| 2   | VCC_IN         | Voltage as-supplied by BEC.                  |
| 3   | RX_PPM         | Enable `feature RX_PPM`                      |
| 4   | TELEMETRY      | Enable `feature TELEMETRY`                   |
| 5   | +V BATTERY     | Voltage as-supplied by Battery.              |
| 6   | -V BATTERY     | Voltage as-supplied by Battery.              |

### IO_2

When TRANSPONDER is used and the IR solder pads are shorted, the 6 pin IO_2 pinout is as follows.

| Pin | Function                  | Notes                                        |
| --- | ------------------------- | -------------------------------------------- |
| 1   | IR-                       | Short leg of the IR LED                      |
| 2   | IR+                       | Long leg of the IR LED                       |
| 3   | CURRENT                   | Current Sensor                               |
| 4   | RSSI                      | RSSI (PWM or Analog - select by solder pads) |
| 5   | BUZZER+                   | 5V Source                                    |
| 6   | BUZZER-                   | Buzzer signal                                |

When LEDSTRIP is used and the LED solder pads are shorted, the 6 pin IO_2 pinout is as follows.

| Pin | Function                  | Notes                                        |
| --- | ------------------------- | -------------------------------------------- |
| 1   | | |
| 2   | LEDSTRIP                  | WS2812 Ledstrip data                         |
| 3   | CURRENT                   | Current Sensor                               |
| 4   | RSSI                      | RSSI (PWM or Analog - select by solder pads) |
| 5   | BUZZER+                   | 5V Source                                    |
| 6   | BUZZER-                   | Buzzer signal                                |

### UART1

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 3   | TXD            |                                              |
| 4   | RXD            |                                              |

### UART2/3

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | Ground         |                                              |
| 2   | VCC_IN         | Voltage as-supplied by BEC.                  |
| 3   | TXD            |                                              |
| 4   | RXD            |                                              |

### Spektrum Satellite

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 3   | 3.3V           |                                              |
| 2   | Ground         |                                              |
| 1   | RXD            |                                              |

### I2C

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | Ground         |                                              |
| 2   | 5.0v           | Voltage as-supplied by BEC OR USB, always on |
| 3   | SCL            | 3.3V signals only                            |
| 4   | SDA            | 3.3V signals only                            |

### SWD

The port cannot be used at the same time as UART2.

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | Ground         |                                              |
| 2   | NRST           |                                              |
| 3   | SWDIO          |                                              |
| 4   | SWDCLK         |                                              |



