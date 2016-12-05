# Board - Seriously Pro SP Racing F3 MINI

The Seriously Pro Racing F3 Mini (SPRacingF3Mini) is the second flight controller board designed specifically for Cleanflight.  It is the only FC that supports all the latest Cleanflight features and the ONLY mini board with a full feature set that can also be mounted in mini frames as well as in standard 36x36mm mounting holes.  With the new transponder feature it is the only truly race ready board available.

It's full feature set, size and weight makes it perfect for incorporating in sub-250g drones to comply with USA drone regulations.

Purchasing boards directly from SeriouslyPro / SP Racing and official retailers helps fund Cleanflight development, it's the reason the Seriously Pro boards exist!  Official retailers are always listed on the SeriouslyPro.com website.

Full details available on the website, here:

http://seriouslypro.com/spracingf3mini

## Hardware Features

* Next-generation STM32 F3 processor with hardware floating point unit for efficient flight calculations and faster ARM-Cortex M4 core.
* MicroSD-Card socket for black box flight log recorder - optimize your tuning and see the results of your setup without guesswork.
* Race transponder built in - just turn up at a race and have your lap times recorded.
* Onboard regulator (BEC) for powering the FC, receiver and small servos.
* Features the latest Accelerometer, Gyro and Mag/Compass and Baro/Altitude sensor technology.
* 2 Buttons for binding Spektrum Satellite receivers, activating USB bootloader mode or resetting the configuration.  More future uses coming!
* Wire up using using pin headers for all major connections for excellent crash-durability.  Use either right-angled or straight pin-headers.
* No compromise I/O. Use all the features all the time; e.g. Connect your USB + OSD + SmartPort + SBus + GPS + LED Strip + Battery Monitoring + Sonar + 8 motors - all at the same time!
* 8 PWM output lines for ESCs and Servos. Arranged for easy wiring on standard pin headers.
* Supports direct connection of SBus, SumH, SumD, Spektrum1024/2048, XBus receivers. No external inverters required (built-in).
* Supports direct connection of 3.3v Spektrum Satellite receivers via 3 pin through-hole JST-ZH connector.
* Supports direct connection of 1-5 channel Parallel PWM receivers *1. 
* Dedicated PPM receiver input.
* 3 Serial Ports - NOT shared with the USB socket.
* Telemetry port (via pin header or USART2 JST-SH socket).
* Micro USB socket. 
* Dedicated output for programmable LEDs - great for orientation, racing and night flying. (Currently mutually exclusive with the Transponder).
* Dedicated I2C port for connection of OLED display without needing flight battery.
* Battery monitoring for voltage and current.
* RSSI monitoring (analog or PWM).
* Buzzer port for audible warnings and notifications.
* Developer friendly debugging port (SWD) and boot mode selection, unbrickable bootloader.
* Symmetrical design for a super tidy wiring.
* JST-SH sockets only for I2C/UART2 and SWD.
* Barometer mounted on the bottom of the board for easy wind isolation.
* Flashing via USB or serial port.
* Stackable design - perfect for integrating with OSDs and power distribution boards.
* Modular design - the core of the board measures 36x22mm but has the ability to be mounted using standard 30.5mm mounting holes.
* LEDs for 3v, 5v and Status for easy diagnostics.
* Copper-etched Cleanflight and #RB logos.


*1 - PWM receiver must use 3.3v outputs.  Works only in multirotor mode.  Uses motor outputs 5-8 and PPM pin as RC 1-5 inputs.

## Pinouts

Full pinout details are available in the manual, here:

http://seriouslypro.com/spracingf3mini#manual

#### Main Section

The main section is the square part of the board with the 30.5mm mounting holes.

### Left Side IO (Front to Back)

| Pin | Function                     | Notes                                        |
| --- | ---------------------------- | -------------------------------------------- |
| 1   | RX3                          | Square Pad                                   |
| 2   | TX3                          | Round Pad                                    |
| 3   | PWM8 / SoftSerial 1 RX / RC4 | Square Pad                                   |
| 4   | PWM7 / SoftSerial 1 TX / RC3 | Square Pad                                   |

To the left of both PWM7 and PWM8 there are 2 more pins - left to right: GND, VIN, PWM7/8.
To the right of RX3 there are two more though holes.  Use RX3 and the 2 holes to attach a JST-ZH connector for a Spektrum Satellite 3v receiver.

### Right Side IO (Front to Back)

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | RSSI           | Round Pad / PWM                              |
| 2   | CURRENT        | Round Pad                                    |
| 3   | PWM6 / RC2     | Square Pad                                   |
| 4   | PWM5 / RC1     | Square Pad                                   |
| 5   | T1             | Round Pad                                    |
| 6   | R1             | Round Pad                                    |
| 7   | 5v             | Round Pad                                    |
| 8   | GND            | Round Pad                                    |
| 9   | 5v             | Round Pad                                    |
| 10  | PPM            | Square Pad                                   |

To the right of both PWM5 and PWM6 there are 2 more pins - left to right: PWM5/6, VIN, GND.

Pins 8/9/10 allow a standard 3 pin cable to be attached for PPM receivers.
Pins 8/7/6 allow a standard 3 pin cable to be attached for SBus/SerialRX receivers (5v).

### Top IO (Left to Right)

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | BATTERY -      | Round Pad                                    |
| 2   | BATTERY +      | Square Pad                                   |

25V Absolute Maximum.  No reverse polarity protection!

### Bottom Left IO (Left to Right)

For Telemetry / LED / IR.

A - Bottom Row (Towards board edge)
B - Top Row (Towards board center)

| Pin | Row | Function       | Notes                                        |
| --- | --- | -------------- | -------------------------------------------- |
| 1   | A   | TELEMETRY / T2 | Square Pad                                   |
| 2   | A   | GND            | Round Pad                                    |
| 3   | B   | VIN            | Square Pad (Suface)                          |
| 4   | B   | LED_STRIP/IR   | Round Pad                                    |

### Bottom Right IO (Left to Right)

For Buzzer.

A - Bottom Row (Towards board edge)
B - Top Row (Towards board center)

| Pin | Row | Function       | Notes                                        |
| --- | --- | -------------- | -------------------------------------------- |
| 1   | A   | BUZZER -       | Round Pad                                    |
| 2   | A   | GND            | Square Pad                                   |
| 3   | B   | 5v             | Round Pad (For Buzzer)                       |
| 4   | B   | 3.3v           | Square Pad (Surface)                         |

### Bottom Center IO (Left to Right)

For ESC / Servos.

A - Bottom Row (Towards board edge)
B - Middle Row 
C - Top Row (Towards board center)

| Pin | Row | Function       | Notes                                        |
| --- | --- | -------------- | -------------------------------------------- |
| 1   | A   | GND            | Round Pad                                    |
| 2   | A   | GND            | Round Pad                                    |
| 3   | A   | GND            | Round Pad                                    |
| 4   | A   | GND            | Round Pad                                    |
| 1   | B   | VIN            | Round Pad                                    |
| 2   | B   | VIN            | Round Pad                                    |
| 3   | B   | VIN            | Round Pad                                    |
| 4   | B   | VIN            | Round Pad                                    |
| 1   | C   | PWM1           | Square Pad (Suface)                          |
| 2   | C   | PWM2           | Round Pad                                    |
| 3   | C   | PWM3           | Round Pad                                    |
| 4   | C   | PWM4           | Round Pad                                    |

### Bottom IO (Underside, left to right)

| Pad  | Function       | Notes                                        |
| ---- | -------------- | -------------------------------------------- |
| TRIG | SONAR TRIGGER  | Also Switch B, 3.3v signal only              |
| ECHO | SONAR ECHO     | Also Switch A, 3.3v signal only              |

### UART2

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | Ground         |                                              |
| 2   | 5v             | Voltage as-supplied by BEC OR USB, always on |
| 3   | TXD            |                                              |
| 4   | RXD            |                                              |

### I2C

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | Ground         |                                              |
| 2   | 5v             | Voltage as-supplied by BEC OR USB, always on |
| 3   | SCL            |                                              |
| 4   | SDA            |                                              |

### SWD

The port cannot be used at the same time as UART2.

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | Ground         |                                              |
| 2   | NRST           |                                              |
| 3   | SWDIO          |                                              |
| 4   | SWDCLK         |                                              |


### Transponder Section.

The transponder section of the board is where up to two IR leds (supplied) can be connected for the race timing transponder system.  It can left attached, stacked above or below the main board or detached and connected via a cable, as appropriate.

The transponder section has a small enable jumper which must be bridged with solder before using the IR transponder feature. 

#### Bottom Left and Bottom Right Transponder IO (Left to Right)

For LED / IR.  The 4 pins arranged in a square by the mounting holes of the transponder board have the same pinouts on the left and right hand sides of the board.  They are the same so that a 4 pin cable can be attached either side when relocating the transponder section.

A - Bottom Row (Towards board edge)
B - Top Row (Towards board center)

| Pin | Row | Function       | Notes                                        |
| --- | --- | -------------- | -------------------------------------------- |
| 1   | A   | N/C            | Square Pad                                   |
| 2   | A   | GND            | Round Pad                                    |
| 3   | B   | 5v             | Round Pad                                    |
| 4   | B   | LED_STRIP/IR   | Round Pad                                    |

### Bottom Left and Bottom Right IR

There are two, two pin holes on the left and right of the transponder section either side of some surface mount components. They are for connecting up to two IR leds.
A - Bottom Row (Towards board edge)
B - Top Row (Towards board center)

| Pin | Row | Function       | Notes                                        |
| --- | --- | -------------- | -------------------------------------------- |
| 1   | A   | IR+            | Round Pad                                    |
| 2   | B   | IR-            | Square Pad                                   |

Note: The silk screen on early boards have IR+ and IR- transposed.  Connect as above regardless of silkscreen.

### Switch Section.

On the bottom of the switch section there are 4 pads.  The switch section can be relocated and reconnected to the main section via a 4 way cable.

| Pad On Switch Section | Pad on main section |
| --------------------- | ------------------- |
| A                     | ECHO                |
| B                     | TRIG                |
| GND                   | Any GND             |
| 3v3                   | Any 3.3v            |

