# Blackbox flight data recorder

![Rendered flight log frame](Screenshots/blackbox-screenshot-1.jpg)

## Introduction

This feature transmits your flight data information on every control loop iteration over a serial port to an external
logging device to be recorded, or to a dataflash chip which is present on some flight controllers.

After your flight, you can process the resulting logs on your computer to either turn them into CSV (comma-separated
values) or render your flight log as a video using the tools `blackbox_decode` and `blackbox_render`. Those tools can be
found in this repository:

https://github.com/cleanflight/blackbox-tools

You can also view your flight logs using your web browser with the interactive log viewer:

https://github.com/cleanflight/blackbox-log-viewer

## Logged data
The blackbox records flight data on every iteration of the flight control loop. It records the current time in
microseconds, P, I and D corrections for each axis, your RC command stick positions (after applying expo curves),
gyroscope data, accelerometer data (after your configured low-pass filtering), barometer readings, 3-axis magnetometer
readings, raw VBAT and current measurements, and the command being sent to each motor speed controller. This is all
stored without any approximation or loss of precision, so even quite subtle problems should be detectable from the fight
data log.

GPS data is logged whenever new GPS data is available. Although the CSV decoder will decode this data, the video
renderer does not yet show any of the GPS information (this will be added later).

## Supported configurations

The maximum data rate that can be recorded to the flight log is fairly restricted, so anything that increases the load
can cause the flight log to drop frames and contain errors.

The Blackbox was developed and tested on a quadcopter. It has also been tested on a tricopter. It should work on
hexacopters or octocopters, but as they transmit more information to the flight log (due to having more motors), the 
number of dropped frames may increase. The `blackbox_render` tool only supports tri and quadcopters (please send me 
flight logs from other craft, and I can add support for them!)

Cleanflight's `looptime` setting will decide how many times per second an update is saved to the flight log. The
software was developed on a craft with a looptime of 2400. Any looptime smaller than this will put more strain on the
data rate. The default looptime on Cleanflight is 3500. 

If you're using a looptime of 2300 or smaller, you will probably need to reduce the sampling rate in the Blackbox
settings, or increase your logger's baudrate to 250000. See the later section on configuring the Blackbox feature for
details.

## Hardware

There are two options for storing your flight logs. You can either transmit the log data over a serial port to an 
external logging device like the [OpenLog serial data logger][] to be recorded to a microSDHC card, or if you have a
compatible flight controller you can store the logs on the onboard dataflash storage instead.

### OpenLog serial data logger

The OpenLog ships with standard OpenLog 3 firmware installed. However, in order to reduce the number of dropped frames,
it should be reflashed with the [OpenLog Light firmware][] or the special [OpenLog Blackbox firmware][] (this needs to
be version 2.0 or higher to allow configuration of baud rates). 

The Blackbox variant of the firmware ensures that the OpenLog is using the correct settings, and defaults to 115200
baud. If you are using a looptime of 2500 or smaller, you should set the baud rate to 250000 instead to eliminate
dropped frames.

You can find the Blackbox version of the OpenLog firmware [here](https://github.com/cleanflight/blackbox-firmware), 
along with instructions for installing it onto your OpenLog.

[OpenLog serial data logger]: https://www.sparkfun.com/products/9530
[OpenLog Light firmware]: https://github.com/sparkfun/OpenLog/tree/master/firmware/OpenLog_v3_Light
[OpenLog Blackbox firmware]: https://github.com/cleanflight/blackbox-firmware

#### microSDHC

Your choice of microSDHC card is very important to the performance of the system. The OpenLog relies on being able to
make many small writes to the card with minimal delay, which not every card is good at. A faster SD-card speed rating is
not a guarantee of better performance.

##### microSDHC cards known to have poor performance

 - Generic 4GB Class 4 microSDHC card - the rate of missing frames is about 1%, and is concentrated around the most
   interesting parts of the log!
 - Sandisk Ultra 32GB (unlike the smaller 16GB version, this version has poor write latency)

##### microSDHC cards known to have good performance

 - Transcend 16GB Class 10 UHS-I microSDHC (typical error rate < 0.1%)
 - Sandisk Extreme 16GB Class 10 UHS-I microSDHC (typical error rate < 0.1%)
 - Sandisk Ultra 16GB (it performs only half as well as the Extreme in theory, but still very good)

You should format any card you use with the [SD Association's special formatting tool][] , as it will give the OpenLog
the best chance of writing at high speed. You must format it with either FAT, or with FAT32 (recommended).

[SD Association's special formatting tool]: https://www.sdcard.org/downloads/formatter_4/

#### OpenLog configuration

Power up the OpenLog with a microSD card inside, wait 10 seconds or so, then power it down and plug the microSD card
into your computer. You should find a "CONFIG.TXT" file on the card. You should see the baud rate that the OpenLog has
been configured to. You probably want this to be set to either 115200 (the default) or 250000 (for craft with looptimes
smaller than 2500).

Save the file and put the card back into your OpenLog, it should use those settings from now on.

If your OpenLog didn't write a CONFIG.TXT file, create a CONFIG.TXT file with these contents and store it in the root
of the MicroSD card:

```
115200
baud
```

If you are using the official OpenLog Light firmware, use this configuration instead:

```
115200,26,0,0,1,0,1
baud,escape,esc#,mode,verb,echo,ignoreRX
```

#### Serial port

A hardware serial port is required to connect the OpenLog to your flight controller (such as `serial_port_1` on the
Naze32, the two-pin Tx/Rx header in the center of the board). The Blackbox can not be used with softserial ports as they
are too slow.

Connect the "TX" pin of the serial port you've chosen to the OpenLog's "RXI" pin. Don't connect the serial port's RX
pin to the OpenLog.

#### Protection

The OpenLog can be wrapped in black electrical tape or heat-shrink in order to insulate it from conductive frames (like
carbon fiber), but this makes its status LEDs impossible to see. I recommend wrapping it with some clear heatshrink
tubing instead.

![OpenLog installed](Wiring/blackbox-installation-1.jpg "OpenLog installed with double-sided tape, SDCard slot pointing outward")

### Onboard dataflash storage
Some flight controllers have an onboard SPI NOR flash chip which can be used to store flight logs instead of using an 
OpenLog.

The full version of the Naze32 and the CC3D have an onboard "m25p16" 2 megayte dataflash storage chip. This is a small
chip with 8 fat legs, which can be found at the base of the Naze32's direction arrow. This chip is not present on the
"Acro" version of the Naze32.

These chips are also supported:

* Micron/ST M25P16 - 16 Mbit
* Micron N25Q064 - 64 Mbit
* Winbond W25Q64 - 64 Mbit

## Enabling the Blackbox (CLI)
In the [Cleanflight Configurator][] , enter the CLI tab. Enable the Blackbox feature by typing in `feature BLACKBOX` and
pressing enter. Now choose the device that you want to log to:

### OpenLog serial data logger
Enter `set blackbox_device=0` to switch to logging to a serial port (this is the default).

You then need to let Cleanflight know which of [your serial ports][] you connected the OpenLog to. A 115200 baud port
is required (such as `serial_port_1` on the Naze32, the two-pin Tx/Rx header in the center of the board).

You can use the GUI to configure a port for the Blackbox feature on the Ports tab.

### Onboard dataflash
Enter `set blackbox_device=1` to switch to logging to an onboard dataflash chip, if your flight controller has one.

[your serial ports]: https://github.com/cleanflight/cleanflight/blob/master/docs/Serial.md
[Cleanflight Configurator]: https://chrome.google.com/webstore/detail/cleanflight-configurator/enacoimjcgeinfnnnpajinjgmkahmfgb?hl=en

## Configuring the Blackbox

If you are using a short looptime like 2500 or smaller, try switching your OpenLog to 250000 baud (instead of the 
default of 115200) and set that baud rate on the Blackbox's port in the Confgurator.

The Blackbox currently provides two settings (`blackbox_rate_num` and `blackbox_rate_denom`) that allow you to control 
the rate at which data is logged. These two together form a fraction (`blackbox_rate_num / blackbox_rate_denom`) which
decides what portion of the flight controller's control loop iterations should be logged. The default is 1/1 which logs 
every iteration.

If you're using a slower MicroSD card, you may need to reduce your logging rate to reduce the number of corrupted
logged frames that `blackbox_decode` complains about. A rate of 1/2 is likely to work for most craft.

You can change the logging rate settings by entering the CLI tab in the [Cleanflight Configurator][] and using the `set`
command, like so:

```
set blackbox_rate_num = 1
set blackbox_rate_denom = 2
```

The data rate for my quadcopter using a looptime of 2400 and a rate of 1/1 is about 10.25kB/s. This allows about 18
days of flight logs to fit on my OpenLog's 16GB MicroSD card, which ought to be enough for anybody :).

If you're logging to an onboard dataflash chip instead of an OpenLog, be aware that the 2MB of storage space it offers
is pretty small. At the default 1/1 logging rate, and a 2400 looptime, this is only enough for about 3 minutes of
flight. This could be long enough for you to investigate some flying problem with your craft, but you may want to reduce
the logging rate in order to extend your recording time.

To maximize your recording time, you could drop the rate way down to 1/32 (the smallest possible rate) which would
result in a logging rate of about 10-20Hz and about 650 bytes/second of data. At that logging rate, the 2MB flash chip
can store around 50 minutes of flight data, though the level of detail is severely reduced and you could not diagnose
flight problems like vibration or PID setting issues.

## Usage

The Blackbox starts recording data as soon as you arm your craft, and stops when you disarm.

If your craft has a buzzer attached, you can use Cleanflight's arming beep to synchronize your Blackbox log with your
flight video. Cleanflight's arming beep is a "long, short" pattern. The beginning of the first long beep will be shown 
as a blue line in the flight data log, which you can sync against your recorded audio track.

You should wait a few seconds after disarming your craft to allow the Blackbox to finish saving its data.

### Usage - OpenLog
Each time the OpenLog is power-cycled, it begins a fresh new log file. If you arm and disarm several times without
cycling the power (recording several flights), those logs will be combined together into one file. The command line
tools will ask you to pick which one of these flights you want to display/decode.

Don't insert or remove the SD card while the OpenLog is powered up.

### Usage - Dataflash chip
After your flights, you can use the [Cleanflight Configurator][] to download the contents of the dataflash to your
computer. Go to the "dataflash" tab and click the "save flash to file..." button. Saving the log can take 2 or 3
minutes.

![Dataflash tab in Configurator](Screenshots/blackbox-dataflash.png)

After downloading the log, be sure to erase the chip to make it ready for reuse by clicking the "erase flash" button.

If you try to start recording a new flight when the dataflash is already full, Blackbox logging will be disabled and
nothing will be recorded.

## Converting logs to CSV or PNG
After your flights, you'll have a series of flight log files with a .TXT extension. You'll need to decode these with
the `blackbox_decode` tool to create CSV (comma-separated values) files for analysis, or render them into a series of PNG
frames with `blackbox_render` tool, which you could then convert into a video using another software package.

You'll find those tools along with instructions for using them in this repository:

https://github.com/cleanflight/blackbox-tools

You can also view your .TXT flight log files interactively using your web browser with the Cleanflight Blackbox Explorer
tool:

https://github.com/cleanflight/blackbox-log-viewer

This allows you to scroll around a graphed version of your log and examine your log in detail.