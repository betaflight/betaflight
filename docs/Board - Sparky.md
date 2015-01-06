# Board - Sparky

The Sparky is a very low cost and very powerful board.

* 3 hardware serial ports.
* Built-in serial port inverters which allows S.BUS receivers to be used without external inverters.
* USB (can be used at the same time as the serial ports).
* 10 PWM outputs.
* Dedicated PPM/SerialRX input pin.
* MPU9150 I2C Acc/Gyro/Mag
* Baro

# Status

Flyable!

Tested with revision 1 board. 

## TODO
* Baro - detection works but sending bad readings, disabled for now.
* LED Strip
* ADC
* Sonar
* Display (via Flex port)
* SoftSerial - though having 3 hardware serial ports makes it a little redundant.
* Airplane PWM mappings.

# Flashing

## Via Device Firmware Upload (DFU, USB)

These instructions are for dfu-util, tested using dfu-util 0.7 for OSX from the OpenTX project.

http://www.open-tx.org/2013/07/15/dfu-util-07-for-mac-taranis-flashing-utility/

A binary file is required for DFU, not a .hex file.  If one is not included in the release then build one as follows.

```
make TARGET=SPARKY clean
make TARGET=SPARKY binary
```

Put the device into DFU mode by powering on the sparky with the bootloader pins temporarily bridged.  The only light that should come on is the blue PWR led.

Run 'dfu-util -l' to make sure the device is listed, as below.

```
$ dfu-util -l
dfu-util 0.7

Copyright 2005-2008 Weston Schmidt, Harald Welte and OpenMoko Inc.
Copyright 2010-2012 Tormod Volden and Stefan Schmidt
This program is Free Software and has ABSOLUTELY NO WARRANTY
Please report bugs to dfu-util@lists.gnumonks.org

Found DFU: [0483:df11] devnum=0, cfg=1, intf=0, alt=0, name="@Internal Flash  /0x08000000/128*0002Kg"
Found DFU: [0483:df11] devnum=0, cfg=1, intf=0, alt=1, name="@Option Bytes  /0x1FFFF800/01*016 e"
```

Then flash the binary as below.

```
dfu-util -D obj/cleanflight_SPARKY.bin --alt 0 -R -s 0x08000000
```

The output should be similar to this:

```
dfu-util 0.7

Copyright 2005-2008 Weston Schmidt, Harald Welte and OpenMoko Inc.
Copyright 2010-2012 Tormod Volden and Stefan Schmidt
This program is Free Software and has ABSOLUTELY NO WARRANTY
Please report bugs to dfu-util@lists.gnumonks.org

Opening DFU capable USB device... ID 0483:df11
Run-time device DFU version 011a
Found DFU: [0483:df11] devnum=0, cfg=1, intf=0, alt=0, name="@Internal Flash  /0x08000000/128*0002Kg"
Claiming USB DFU Interface...
Setting Alternate Setting #0 ...
Determining device status: state = dfuERROR, status = 10
dfuERROR, clearing status
Determining device status: state = dfuIDLE, status = 0
dfuIDLE, continuing
DFU mode device DFU version 011a
Device returned transfer size 2048
No valid DFU suffix signature
Warning: File has no DFU suffix
DfuSe interface name: "Internal Flash  "
Downloading to address = 0x08000000, size = 76764
......................................
File downloaded successfully
can't detach
Resetting USB to switch back to runtime mode

```

## Via SWD

On the bottom of the board there is an SWD header socket onto switch a JST-SH connector can be soldered.
Once you have SWD connected you can use the st-link or j-link tools to flash a binary.

See Sparky schematic for CONN2 pinouts.

## TauLabs bootloader

Flashing cleanflight will erase the TauLabs bootloader, this is not a problem and can easily be restored using the st flashloader tool.

# Serial Ports

| Value | Identifier   | RX        | TX         | Notes                                                                                       |
| ----- | ------------ | --------- | ---------- | ------------------------------------------------------------------------------------------- |
| 1     | USB VCP      | RX (USB)  | TX (USB)   |  |
| 2     | USART1       | RX / PB7  | TX / PB6   | Conn1 / Flexi Port. |
| 3     | USART2       | RX / PA3  | PWM6 / PA2 | On RX is on INPUT header.  Best port for Serial RX input |
| 4     | USART3       | RX / PB11 | TX / PB10  | RX/TX is on one end of the 6-pin header about the PWM outputs. |

USB VCP *can* be used at the same time as other serial ports (unlike Naze32).

All USART ports all support automatic hardware inversion which allows direct connection of serial rx receivers like the FrSky X4RSB - no external inverter needed.


