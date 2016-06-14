# Board - MotoLab

The MOTOLAB build target supports the STM32F3-based boards provided by MotoLab.

At present this includes the TornadoFC and MotoF3. The TornadoFC is described here:

http://www.rcgroups.com/forums/showthread.php?t=2473157

The MotoF3 documentation will be provided when the board is available.

Both boards use the STM32F303 microcontroller and have the following features:

* 256K bytes of flash memory
* Floating point math coprocessor
* Three hardware serial port UARTs
* USB using the built-in USB phy that does not interfere with any hadware UART
* Stable voltage regulation
* High-current buzzer/LED output
* Serial LED interface
* Low-pass filtered VBAT input with 1/10 divider ratio
* 8 short-circuit protected PWM outputs, with 5V buffering on the TornadoFC
* On-board 6S-compatible switching regulator (MotoF3)
* Direct mounting option for a Pololu switching regulator for up to 6S lipo operation (TornadoFC)


# Flashing

The MotoLab boards use the internal DFU USB interface on the STM32F3 microcontroller which is not compatible with the INAV configurator flashing tool.

Instead, on Windows you can use the Impulse Flashing Utility from ImpulseRC, available here:

http://www.warpquad.com/ImpulseFlash.zip

Download and unzip the program. Start the program, plug in the USB on the target board, and drag and drop the intended binary file onto the program icon. The program will put the STM32F3 into bootloader mode automatically and load the binary file to the flash.

For programming on Linux, use the dfu-util program which is installed by default on Ubuntu-based systems. Connect the boot pins on the board and plug in the USB.

Verify that the system identifies the DFU device with this command:
```
dfu-util -l
```

The output should list a "Found DFU" device, something like this:
```
dfu-util 0.5

(C) 2005-2008 by Weston Schmidt, Harald Welte and OpenMoko Inc.
(C) 2010-2011 Tormod Volden (DfuSe support)
This program is Free Software and has ABSOLUTELY NO WARRANTY

dfu-util does currently only support DFU version 1.0

Found DFU: [0483:df11] devnum=0, cfg=1, intf=0, alt=0, name="@Internal Flash  /0x08000000/128*0002Kg"
Found DFU: [0483:df11] devnum=0, cfg=1, intf=0, alt=1, name="@Option Bytes  /0x1FFFF800/01*016 e"
```

Use this command to load the binary file to the flash memory on the board:
```
dfu-util --alt 0 -s 0x08000000 -D <binfile>
```

The output should look something like this:
```
dfu-util 0.5

(C) 2005-2008 by Weston Schmidt, Harald Welte and OpenMoko Inc.
(C) 2010-2011 Tormod Volden (DfuSe support)
This program is Free Software and has ABSOLUTELY NO WARRANTY

dfu-util does currently only support DFU version 1.0

Opening DFU USB device... ID 0483:df11
Run-time device DFU version 011a
Found DFU: [0483:df11] devnum=0, cfg=1, intf=0, alt=0, name="@Internal Flash  /0x08000000/128*0002Kg"
Claiming USB DFU Interface...
Setting Alternate Setting #0 ...
Determining device status: state = dfuDNLOAD-IDLE, status = 0
aborting previous incomplete transfer
Determining device status: state = dfuIDLE, status = 0
dfuIDLE, continuing
DFU mode device DFU version 011a
Device returned transfer size 2048
No valid DFU suffix signature
Warning: File has no DFU suffix
DfuSe interface name: "Internal Flash  "
```

A binary file is required for the Impulse flashing Utility and dfu-util. The binary file can be built as follows:
```
make TARGET=MOTOLAB clean
make TARGET=MOTOLAB binary
```

To completely erase the flash, create an all-zero file with this command on linux:
```
dd if=/dev/zero of=zero.bin bs=1 count=262144
```

## Todo

Pinout documentation
