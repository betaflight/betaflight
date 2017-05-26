# Board - MotoLab

The MOTOLAB build target supports the STM32F3-based boards provided by MotoLab.

At present this includes the TornadoFC, CycloneFC and MotoF3.

The CycloneFC and TornadoFC are described here:

http://www.rcgroups.com/forums/showpost.php?p=32330479&postcount=2

The MotoF3 is described here:

http://www.rcgroups.com/forums/showpost.php?p=28508139&postcount=3

All boards use the STM32F303 microcontroller and have the following features:

* 256K bytes of flash memory
* Floating point math coprocessor
* Three hardware serial port UARTs
* USB using the built-in USB phy that does not interfere with any hardware UART
* Stable voltage regulation
* High-voltage/high-current buzzer/LED output
* Serial LED interface
* Low-pass filtered VBAT input with 1/10 divider ratio
* Low-pass filtered PWM or analog RSSI input
* 8 short-circuit protected PWM outputs, with 5V buffering on the TornadoFC
* On-board 4S-compatible switching regulator (CycloneFC and MotoF3)
* Direct mounting option for a Pololu switching regulator for up to 6S lipo operation (TornadoFC)
* Pass-through programming and configuration of BLHeli-based ESCs using Cleanflight Configurator

The MotoF3 also provides built-in power distribution for four ESCs, an on-board buzzer, and 2Mbyte SPI flash for data logging.

# Flashing

MotoLab boards all use the built-in USB interface on the STM32F3 microcontroller. New versions of Cleanflight can be installed using Cleanflight Configurator through the USB interface. Installation of new firmware is described here:

[USB Flashing](USB%20Flashing.md)

Firmware installation using Windows operating system is complicated because the default device driver for the STM32 USB interface in DFU programming mode is not compatible with the Configurator flash tool. The required DFU mode driver must be manually installed. The default driver for the USB port in normal (VCP) mode is also required for connection to Configurator. Additional details on the drivers and their installion is provided here:

http://www.rcgroups.com/forums/showthread.php?t=2537379

