# Board - Sparky

The Sparky is a very low cost and very powerful board.

* 3 hardware serial ports.
* Built-in serial port inverters which allows S.BUS receivers to be used without external inverters.
* USB (can be used at the same time as the serial ports).
* 10 PWM outputs.
* Dedicated PPM/SerialRX input pin.
* MPU9150 I2C Acc/Gyro/Mag
* Baro

Tested with revision 1 & 2 boards.

## TODO

* Rangefinder
* Display (via Flex port)
* SoftSerial - though having 3 hardware serial ports makes it a little redundant.
* Airplane PWM mappings.

# Voltage and current monitoring (ADC support)

Voltage monitoring is possible when enabled via PWM9 pin and current can be monitored via PWM8 pin. The voltage divider and current sensor need to be connected externally. The vbatscale cli parameter need to be adjusted to fit the sensor specification. For more details regarding the sensor hardware you can check here: https://github.com/TauLabs/TauLabs/wiki/User-Guide:-Battery-Configuration

# Flashing

## Via Device Firmware Upload (DFU, USB) - Windows

These instructions are for flashing the Sparky board under Windows using DfuSE.
Credits go to Thomas Shue (Full video of the below steps can be found here: https://www.youtube.com/watch?v=I4yHiRVRY94)

Required Software:
DfuSE Version 3.0.2 (latest version 3.0.4 causes errors): http://code.google.com/p/multipilot32/downloads/detail?name=DfuSe.rar
STM VCP Driver 1.4.0: http://www.st.com/web/en/catalog/tools/PF257938

A binary file is required for DFU, not a .hex file.  If one is not included in the release then build one as follows.

```
Unpack DfuSE and the STM VCP Drivers into a folder on your Hardrive
Download the latest Sparky release (inav_SPARKY.hex) from:
https://github.com/iNavFlight/inav/releases and store it on your Hardrive

In your DfuSE folder go to BIN and start DfuFileMgr.exe
Select: "I want to GENERATE a DFUfile from S19,HEX or BIN files" press OK
Press: "S19 or Hex.."
Go to the folder where you saved the inav_SPARKY.hex file, select it  and press open
(you might need to change the filetype in the DfuSE explorer window to "hex Files (*.hex)" to be able to see the file)
Press: "Generate" and select the .dfu output file and location
If all worked well you should see " Success for 'Image for lternate Setting 00 (ST..)'!"

```

Put the device into DFU mode by powering on the sparky with the bootloader pins temporarily bridged.  The only light that should come on is the blue PWR led.

Check the windows device manager to make sure the board is recognized correctly.
It should show up as "STM Device in DFU mode" under Universal Serial Bus Controllers

If it shows up as "STMicroelectronics Virtual COM" under Ports (COM & LPT) instead then the board is not in DFU mode. Disconnect the board, short the bootloader pins again while connecting the board.

If the board shows up as "STM 32 Bootloader" device in the device manager, the drivers need to be updated manually.
Select the device in the device manager, press "update drivers", select "manual update drivers" and choose the location where you extracted the STM VCP Drivers, select "let me choose which driver to install". You shoud now be able to select either the STM32 Bootloader driver or the STM in DFU mode driver. Select the later and install.


Then flash the binary as below.

```
In your DfuSE folder go to BIN and start DfuSeDemo.exe
Select the Sparky Board (STM in DFU Mode) from the Available DFU and compatible HID Devices drop down list
Press "Choose.." at the bootom of the window and select the .dfu file created in the previous step
"File correctly loaded" should appear in the status bar
Press "Upgrade" and confirm with "Yes"
The status bar will show the upload progress and confirm that the upload is complete at the end

```

Disconnect and reconnect the board from USB and continue to configure it via the INAV configurator as per normal


## Via Device Firmware Upload (DFU, USB) - Mac OS X / Linux

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
dfu-util -D obj/inav_SPARKY.bin --alt 0 -R -s 0x08000000
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
On Linux you might want to take care that the modemmanager isn't trying to use your sparky as modem getting it into bootloader mode while doing so. In doubt you probably want to uninstall it. It could also be good idea to get udev fixed. It looks like teensy did just that -> http://www.pjrc.com/teensy/49-teensy.rules (untested)

To make a full chip erase you can use a file created by
```
dd if=/dev/zero of=zero.bin bs=1 count=262144
```
This can be used by dfu-util.

## Via SWD

On the bottom of the board there is an SWD header socket onto switch a JST-SH connector can be soldered.
Once you have SWD connected you can use the st-link or j-link tools to flash a binary.

See Sparky schematic for CONN2 pinouts.

## TauLabs bootloader

Flashing INAV will erase the TauLabs bootloader, this is not a problem and can easily be restored using the st flashloader tool.

# Serial Ports

| Value | Identifier   | RX        | TX         | Notes                                                          |
| ----- | ------------ | --------- | ---------- | -------------------------------------------------------------- |
| 1     | USB VCP      | RX (USB)  | TX (USB)   |                                                                |
| 2     | USART1       | RX / PB7  | TX / PB6   | Conn1 / Flexi Port.                                            |
| 3     | USART2       | RX / PA3  | PWM6 / PA2 | On RX is on INPUT header.  Best port for Serial RX input       |
| 4     | USART3       | RX / PB11 | TX / PB10  | RX/TX is on one end of the 6-pin header about the PWM outputs. |

USB VCP *can* be used at the same time as other serial ports (unlike Naze32).

All USART ports all support automatic hardware inversion which allows direct connection of serial rx receivers like the FrSky X4RSB - no external inverter needed.


# Battery Monitoring Connections

| Pin  | Signal | Function        |
| ---- | ------ | --------------- |
| PWM9 | PA4    | Battery Voltage |
| PWM8 | PA7    | Current Meter   |

## Voltage Monitoring

The Sparky has no battery divider cricuit, PWM9 has an inline 10k resistor which has to be factored into the resistor calculations.
The divider circuit should eventally create a voltage between 0v and 3.3v (MAX) at the MCU input pin.

WARNING: Double check the output of your voltage divider using a voltmeter *before* connecting to the FC.

### Example Circuit

For a 3Cell battery divider the following circuit works:

`Battery (+) ---< R1 >--- PWM9 ---< R2 >--- Battery (-)`

* R1 = 8k2 (Grey Red Red)
* R2 = 2k0 (Red Black Red)

This gives a 2.2k for an 11.2v battery.  The `vbat_scale` for this divider should be set around `52`.

## Current Monitoring

Connect a current sensor to PWM8/PA7 that gives a range between 0v and 3.3v out (MAX).
