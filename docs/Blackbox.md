# Blackbox flight data recorder

![Rendered flight log frame](Screenshots/blackbox-screenshot-1.jpg)

## Introduction

This feature transmits your flight data information on every control loop iteration over a serial port to an external
logging device to be recorded.

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
readings, raw VBAT measurements, and the command being sent to each motor speed controller. This is all stored without
any approximation or loss of precision, so even quite subtle problems should be detectable from the fight data log.

Currently, the blackbox attempts to log GPS data whenever new GPS data is available, but this has not been tested yet.
The CSV decoder and video renderer do not yet show any of the GPS data (though this will be added). If you have a working
GPS, please send in your logs so I can get the decoding implemented.

The data rate for my quadcopter using a looptime of 2400 is about 10.25kB/s. This allows about 18 days of flight logs
to fit on a 16GB MicroSD card, which ought to be enough for anybody :).

## Supported configurations

The maximum data rate for the flight log is fairly restricted, so anything that increases the load can cause the flight
log to drop frames and contain errors.

The Blackbox was developed and tested on a quadcopter. It has also been tested on a tricopter. It should work on
hexacopters or octocopters, but as they transmit more information to the flight log (due to having more motors), the 
number of dropped frames may increase. The `blackbox_render` tool only supports tri and quadcopters (please send me 
flight logs from other craft, and I can add support for them!)

Cleanflight's `looptime` setting will decide how many times per second an update is saved to the flight log. The
software was developed on a craft with a looptime of 2400. Any looptime smaller than this will put more strain on the
data rate. The default looptime on Cleanflight is 3500. If you're using a looptime of 2000 or smaller, you will probably
need to reduce the sampling rate in the Blackbox settings, see the later section on configuring the Blackbox feature for
details.

## Hardware

The blackbox software is designed to be used with an [OpenLog serial data logger][] and a microSDHC card. You need a
little prep to get the OpenLog ready for use, so here are the details:

### Firmware

The OpenLog ships with standard OpenLog 3 firmware installed. However, in order to reduce the number of dropped frames,
it should be reflashed with the [OpenLog Light firmware][] or the special [OpenLog Blackbox firmware][] . The Blackbox
variant of the firmware ensures that the OpenLog is running at the correct baud-rate and does away for the need for a 
`CONFIG.TXT` file to set up the OpenLog.

You can find the Blackbox version of the OpenLog firmware [here](https://github.com/cleanflight/blackbox-firmware), 
along with instructions for installing it onto your OpenLog.

[OpenLog serial data logger]: https://www.sparkfun.com/products/9530
[OpenLog Light firmware]: https://github.com/sparkfun/OpenLog/tree/master/firmware/OpenLog_v3_Light
[OpenLog Blackbox firmware]: https://github.com/cleanflight/blackbox-firmware

### microSDHC

Your choice of microSDHC card is very important to the performance of the system. The OpenLog relies on being able to
make many small writes to the card with minimal delay, which not every card is good at. A faster SD-card speed rating is
not a guarantee of better performance.

#### microSDHC cards known to have poor performance

 - Generic 4GB Class 4 microSDHC card - the rate of missing frames is about 1%, and is concentrated around the most
   interesting parts of the log!

#### microSDHC cards known to have good performance

 - Transcend 16GB Class 10 UHS-I microSDHC (typical error rate < 0.1%)
 - Sandisk Extreme 16GB Class 10 UHS-I microSDHC (typical error rate < 0.1%)

You should format any card you use with the [SD Association's special formatting tool][] , as it will give the OpenLog
the best chance of writing at high speed. You must format it with either FAT, or with FAT32 (recommended).

[SD Association's special formatting tool]: https://www.sdcard.org/downloads/formatter_4/

### OpenLog configuration

This section applies only if you are using the OpenLog or OpenLog Light original firmware on the OpenLog. If you flashed
it with the special OpenLog Blackbox firmware, you don't need to configure it further.

Power up the OpenLog with a microSD card inside, wait 10 seconds or so, then power it down and plug the microSD card
into your computer. You should find a "CONFIG.TXT" file on the card. Edit it in a text editor to set the first number
(baud) to 115200. Set esc# to 0, mode to 0, and echo to 0. Save the file and put the card back into your OpenLog, it
should use those settings from now on.

If your OpenLog didn't write a CONFIG.TXT file, create a CONFIG.TXT file with these contents and store it in the root
of the MicroSD card:

```
115200,26,0,0,1,0,1
baud,escape,esc#,mode,verb,echo,ignoreRX
```

## Enabling this feature (CLI)

Enable the Blackbox feature by typing in `feature BLACKBOX` and pressing enter. You also need to assign the Blackbox to
one of [your serial ports][] . A 115200 baud port is required (such as serial_port_1 on the Naze32, the two-pin Tx/Rx
header in the center of the board).

For example, use `set serial_port_1_scenario=11` to switch the main serial port to MSP, CLI, Blackbox and GPS Passthrough.

Enter `save`. Your configuration should be saved and the flight controller will reboot. You're ready to go!

[your serial ports]: https://github.com/cleanflight/cleanflight/blob/master/docs/Serial.md
[Cleanflight Configurator]: https://chrome.google.com/webstore/detail/cleanflight-configurator/enacoimjcgeinfnnnpajinjgmkahmfgb?hl=en

## Configuring this feature

The Blackbox currently provides two settings (`blackbox_rate_num` and `blackbox_rate_denom`) that allow you to control 
the rate at which data is logged. These two together form a fraction (`blackbox_rate_num / blackbox_rate_denom`) which
decides what portion of the flight controller's control loop iterations should be logged. The default is 1/1 which logs 
every iteration.

If you are using a short looptime like 2000 or faster, or you're using a slower MicroSD card, you will need to reduce
this rate to reduce the number of corrupted logged frames. A rate of 1/2 is likely to work for most craft.

You can change these settings by entering the CLI tab in the Cleanflight Configurator and using the `set` command, like so:

```
set blackbox_rate_num = 1
set blackbox_rate_denom = 2
```

### Serial port

Connect the "TX" pin of the serial port you've chosen to the OpenLog's "RXI" pin. Don't connect the serial port's RX
pin to the OpenLog.

### Protection

The OpenLog can be wrapped in black electrical tape or heat-shrink in order to insulate it from conductive frames (like
carbon fiber), but this makes its status LEDs impossible to see. I recommend wrapping it with some clear heatshrink
tubing instead.

![OpenLog installed](Wiring/blackbox-installation-1.jpg "OpenLog installed with double-sided tape, SDCard slot pointing outward")

## Usage

The Blackbox starts recording data as soon as you arm your craft, and stops when you disarm. Each time the OpenLog is
power-cycled, it begins a fresh new log file. If you arm and disarm several times without cycling the power (recording
several flights), those logs will be combined together into one file. The command line tools will ask you to pick which
one of these flights you want to display/decode.

If your craft has a buzzer attached, a short beep will be played when you arm. You can later use this beep to
synchronize your recorded flight video with the rendered flight data log (the beep is shown as a blue line in the flight
data log, which you can sync against the beep in your recorded audio track).

The OpenLog requires a couple of seconds of delay after connecting the battery before it's ready to record, so don't
arm your craft immediately after connecting the battery (you'll probably be waiting for the flight controller to become
ready during that time anyway!)

You should also wait a few seconds after disarming the quad to allow the OpenLog to finish saving its data.

Don't insert or remove the SD card while the OpenLog is powered up.

## Converting logs to CSV or PNG
After your flights, you'll have a series of files labeled "LOG00001.TXT" etc. on the microSD card. You'll need to
decode these with the `blackbox_decode` tool to create a CSV (comma-separated values) file for analysis, or render them
into a series of PNG frames with `blackbox_render` tool, which you could then convert into a video using another 
software package.

You'll find those tools along with instructions for using them in this repository:

https://github.com/cleanflight/blackbox-tools

You can also view your .TXT flight log files interactively using your web browser with the Cleanflight Blackbox Explorer
tool:

https://github.com/cleanflight/blackbox-log-viewer

This allows you to scroll around a graphed version of your log and examine your log in detail.