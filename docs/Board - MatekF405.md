
# Board - MATEKSYS F405 family

## Matek F405-OSD

![Matek F405-OSD](http://www.mateksys.com/downloads/FC/MATEKF405-OSD.JPG)

## Matek F405-AIO

![Matek F405-AIO](http://www.mateksys.com/wp-content/uploads/2017/06/F405-AIO_2.jpg)

## Description
F405+ICM20602, w/ Betaflight OSD & SD Card Slot

F405-AIO (only) exposes I2C pads for compass / baro etc.

## MCU, Sensors and Features

### Hardware
* MCU: STM32F405RGT6
* IMU: ICM-20602(SPI)
* OSD: BetaFlight OSD w/ AT7456E chip
* Compass & Baro: no
* VCP: Yes
* Hardware UARTS: 1, 2, 3, 4, 5
* Blackbox: Micro SD Card
* PPM/UART Shared:  UART2-RX
* Battery Voltage Sensor: Yes
* Current Sensor: F405-AIO, otherwise FCHUB-6S option
* Integrated Voltage Regulator: F405-AIO, otherwise FCHUB-6S option
* Brushed Motor Mosfets: No
* Buttons: BOOT button
* 6 PWM / DShot outputs
* WS2812 Led Strip : Yes
* Beeper : Yes

### Features
* 32K Gyro ICM-20602
* Support Gyro sample rate 32K & PID Loop 16K
* Built in inverter for SBUS input (UART2-RX)
* 6x DShot outputs without conflict
* SD Card Slot
* VCP, UART1, UART2, UART3, UART4 & UART5
* w/ Anti-vibration Standoffs
* I2C on F405-AIO

## USB

This board uses STM32 VCP and does _not_ use a UART when USB is connected. STM32 VCP drivers might be required on some operating systems.

Flashing requires DFU mode and STM32 DFU drivers. On Linux, the configurator or `dfu-util` work with a `udev` rule.

````
# DFU (Internal bootloader for STM32 MCUs)
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0664", GROUP="plugdev"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ENV{ID_MM_DEVICE_IGNORE}="1"
````

On Windows, it may be necessary to use the [Zadig](http://zadig.akeo.ie) tool to install the WinUSB driver.

## Manufacturers and Distributors
Matek Systems http://www.mateksys.com/?portfolio=f405-osd and http://www.mateksys.com/?portfolio=f405-aio

## Designers
Matek Systems www.mateksys.com

## Maintainers
* Hardware: Matek Systems

## FAQ & Known Issues

I2C requires that the WS2812 led strip is moved to S5, thus WS2812 is probably only usable on quadcopters.

Setup Guide Matek F405: http://f405.mateksys.com

Setup Guide Matek F405-AIO: http://www.mateksys.com/?portfolio=f405-aio

Rcgroups Thread Matek F405: https://www.rcgroups.com/forums/showthread.php?2889298-MATEKSYS-Flight-Controller-F405-OSD-32K-Gyro-5xUARTs-SD-Slot

Rcgroups Thread Matek F405-AIO: https://www.rcgroups.com/forums/showthread.php?2912273-Matek-Flight-Controller-F405-AIO
