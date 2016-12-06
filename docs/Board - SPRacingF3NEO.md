# Board - Seriously Pro SP Racing F3 NEO

The Seriously Pro Racing F3 NEO board (SPRacingF3NEO) is a full-featured board designed specifically for quadcopters.  The NEO FC/PDB features the latest gyro technology and stacks with an OSD/VTX board.  The stack makes for an extremely compact and easy to install solution with the minimal of solder connections and external cables required.  It's perfect for beginners and experienced pilots.  The acc/gyro is an ICM20602, which is newer than you'll find in all other F3 boards and most F4 boards as of December 2016.

Purchasing boards directly from SeriouslyPro / SP Racing and official retailers helps fund Cleanflight development, it's the reason the Seriously Pro boards exist!  Official retailers are always listed on the SeriouslyPro.com website.

Full details available on the website, here:

http://seriouslypro.com/spracingf3neo

## Hardware Features

The NEO comprises of two boards.

1. NEO FC/PDB - FC/PDB/AMPERAGE MONITORING/LC FILTER/12V BEC/5V BEC/ETC
2. NEO OSD/VTX - OSD/VTX/BUTTON/ANTENNA/ETC

The height of the combined stack is just 15mm high.

### NEO FC/PDB board.

* Next-generation STM32 F3 processor with hardware floating point unit for efficient flight calculations and faster ARM-Cortex M4 core.
* Features the latest accelerometer and gyro technology (ICM20602) via SPI connection.
* MicroSD-Card socket for black box flight log recorder - optimize your tuning and see the results of your setup without guesswork.
* Built in amperage/current monitoring sensor, 110A.
* Telemetry support (FrSky, SmartPort/S.PORT, IBus, etc).
* 5 Serial ports.  e.g Receiver + Telemetry + 3 Spare - None shared with the USB.
* Built-in LC-Filter for clearer video.
* Race transponder built in - just turn up at a race and have your lap times recorded.
* Boot button for easy DFU/USB flashing.
* Wire up using solder pads for ESC connections and cable or header pins for receiver.
* 6 PWM output lines for ESCs and Servos. 4 are arranged for easy wiring on each corner of the board. (The 4 main outputs support DSHOT *).
* Supports direct connection of SBus, SumH, SumD, Spektrum1024/2048, XBus/IBus receivers. No external inverters required (built-in).
* Supports direct connection of 3.3v Spektrum Satellite receivers via 3 pin through-hole JST-ZH connector.
* Supports PPM receivers.
* Micro USB socket. 
* Dedicated output for programmable LEDs - great for orientation, racing and night flying.
* Dedicated I2C/UART3/UART5 port for connection of OLED display, GPS receivers, external MAG/BARO without needing flight battery.
* Voltage monitoring for battery, 12v and 5v supplies.
* Analog RSSI monitoring.
* Buzzer port for audible warnings and notifications.
* Developer friendly debugging port (SWD) unbrickable bootloader.
* Symmetrical design for a super tidy wiring, just 4 wires per corner to each ESC and you're done!
* JST-SH sockets only for I2C/UART3/UART4 and SWD.  UART1 available on through-hole stacking pins and broken out on OSD/VTX board.
* 12 position connector for stacking the OSD/VTX board. (SPI/CS1/CS2/VSYNC/HSYNC/VTX ENABLE/BUTTON),
* Through-hole solder pads AND solder-less connections for receiver via PicoBlade connector.
* Through-hole solder pads for Transponder IR LED.
* Solder pads for RSSI.
* Solder pads for LED Strip.
* Solder pads for ESC power. (2 by each corner)
* Solder pads for ESC signal and ground. (2 by each corner).
* Solder pads for 2x additional PWM outputs (e.g. for pan / tilt servos).
* Direct connection for XT60 socket. (Through hole).
* Flashing via USB or serial port.
* Standard 30.5mm mounting holes, board fits in most 36x36x mounting spaces.
* Board is 45x50mm with corner cutouts to allow frame clearance.
* LEDs for 3v (Blue), 5v (Green) and 12v (Yellow) supplies.
* Two status LEDs (Red, White) for easy diagnostics/flight mode indications.
* Supplied with cable for no-solder connection to FrSky XSR receivers.
* Supplied with receiver cable for no-solder connection to most other receviers.
* Supplied with JST-ZH socket for connection to 3.3v spektrum satellite receivers.
* Cleanflight logo.
* SPRacing logo.

* Note: DSHOT support is coming to a future Cleanflight release.

### NEO VTX/OSD board.

* OSD with customisable layout.
* VTX with 25/200mw output.
* Button for changing VTX channel/band/rf power/power/etc.
* Configurable via cleanflight configurator, via the NEO USB socket.
* U.FL socket for antenna connection, gold plated.
* Display of amperage/current, voltage, RSSI, flight modes, on duration, armed duration, 5v, 12v, callsign, motors, etc.
* Available without VTX module, so users can use external VTX or alternative compatible VTX modules.
* VTX can be turned off remotely (e.g via transmitter).  Great for team races or in-pit configuration.
* Solder pads for 5V, 5V switched, 3.3V, 12V, Video IN/OUT, Audio, GND.
* Breakout 2.54mm pitch though-holes for connecting to UART1 on the NEO - e.g. for Bluetooth/Wifi modules.
* Picoplade connector for no-solder connection to external VTX, with 12V/VIDEO/AUDIO/5V switched/GND signals.
* 2.54mm Through hole pads for connecting Buzzer (controlled via NEO).
* 2.54mm Through hole pads for connecting 12V devices.
* Though hole pads for JST-ZH sockets (supplied) connecting CAMERA an EXTERNAL VTX boards.
* Solder pads for PCB side-mount antenna JACK connection (when not using U.FL connector). 
* CAMERA and EXTERNAL VTX sockets power selectable between 5V and 12V.
* Hole for accessing the boot button (via paperclip) on the NEO below it.
* Board is 36x36mm with standard 30.5mm mounting holes.
* 12 position connector for stacking above the NEO.
* Illuminating white LED.
* Blue LED for 3v3 power.
* Cleanflight logos.
* SPRacing logo.


## Pinouts

Full pinout details are available in the manual, here:

http://seriouslypro.com/files/SPRacingF3NEO-Manual-latest.pdf

### Spektrum Satellite

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 3   | 3.3V           |                                              |
| 2   | Ground         |                                              |
| 1   | RXD            |                                              |

### Receiver

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | UART2 RX       | Serial RX or PPM                             |
| 2   | UART2 TX       | FrSky Telemetry                              |
| 3   | UART5 TX       | SmartPort/S.PORT telemetry                   |
| 4   | 5.0v           | Voltage as-supplied by BEC or USB, always on |
| 5   | Ground         |                                              |


### I2C/UART3/UART4

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | Ground         |                                              |
| 2   | 5.0v           | Voltage as-supplied by BEC OR USB, always on |
| 3   | SCL            | 3.3V signals only                            |
| 4   | SDA            | 3.3V signals only                            |
| 5   | UART3 RXD      | 3.3V signals only                            |
| 6   | UART3 TXD      | 3.3V signals only                            |
| 7   | UART4 RXD      | 3.3V or 5V signals OK                        |
| 8   | UART4 TXD      | 3.3V or 5V signals OK                        |

### SWD

| Pin | Function       | Notes                                        |
| --- | -------------- | -------------------------------------------- |
| 1   | Ground         |                                              |
| 2   | NRST           |                                              |
| 3   | SWDIO          |                                              |
| 4   | SWDCLK         |                                              |

