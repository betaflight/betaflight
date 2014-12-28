# Blackbox flight data recorder

![Rendered flight log frame](http://i.imgur.com/FBphB8c.jpg)

## Introduction
This feature transmits your flight data information on every control loop iteration over a serial port to an external
logging device to be recorded.

After your flight, you can process the resulting logs on your computer to either turn them into CSV (comma-separated
values) or render your flight log as a video using the tools `blackbox_decode` and `blackbox_render` . Those tools can be
found in this repository:

https://github.com/thenickdude/blackbox

## Logged data
The blackbox records flight data on every iteration of the flight control loop. It records the current time in
microseconds, P, I and D corrections for each axis, your RC command stick positions (after applying expo curves),
gyroscope data, accelerometer data (after your configured low-pass filtering), and the command being sent to each motor
speed controller. This is all stored without any approximation or loss of precision, so even quite subtle problems
should be detectable from the fight data log.

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

The Blackbox feature is currently available on the Naze32 and Naze32Pro targets. It may work on other targets, but you 
will need to enable those manually in `/src/main/target/xxx/target.h` by adding a `#define BLACKBOX` statement and 
recompile Cleanflight.

## Hardware
The blackbox software is designed to be used with an [OpenLog serial data logger][] and a microSDHC card. You need a
little prep to get the OpenLog ready for use, so here are the details:

### Firmware
The OpenLog should be flashed with the [OpenLog Lite firmware][] or the special [OpenLog Blackbox variant][] using the Arduino IDE
in order to minimise dropped frames (target the "Arduino Uno"). The Blackbox variant of the firmware ensures that the
OpenLog is running at the correct baud-rate and does away for the need for a `CONFIG.TXT` file to set up the OpenLog. 

If you decide to use the OpenLog Lite firmware instead, note that the .hex file for OpenLog Lite currently in the
OpenLog repository is out of date with respect to their .ino source file, so you'll need to build it yourself after
adding the [required libraries][] to your Arduino libraries directory.

To flash the firmware, you'll need to use an FTDI programmer like the [FTDI Basic Breakout][] along with some way of
switching the Tx and Rx pins over (since the OpenLog has them switched) like the [FTDI crossover][].

[OpenLog serial data logger]: https://www.sparkfun.com/products/9530
[OpenLog Lite firmware]: https://github.com/sparkfun/OpenLog/tree/master/firmware/OpenLog_v3_Light
[OpenLog Blackbox variant]: https://github.com/thenickdude/blackbox/tree/blackbox/tools/blackbox/OpenLog_v3_Blackbox
[Required libraries]: https://code.google.com/p/beta-lib/downloads/detail?name=SerialLoggerBeta20120108.zip&can=4&q=
[FTDI Basic Breakout]: https://www.sparkfun.com/products/9716
[FTDI crossover]: https://www.sparkfun.com/products/10660

### Serial port
Connect the "TX" pin of the serial port you've chosen to the OpenLog's "RXI" pin. Don't connect the serial port's RX
pin to the OpenLog.

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
This section applies only if you are using the OpenLog or OpenLog Lite original firmware on the OpenLog. If you flashed
it with the special OpenLog Blackbox firmware, you don't need to configure it further.

Power up the OpenLog with a microSD card inside, wait 10 seconds or so, then power it down and plug the microSD card
into your computer. You should find a "CONFIG.TXT" file on the card. Edit it in a text editor to set the first number
(baud) to 115200. Set esc# to 0, mode to 0, and echo to 0. Save the file and put the card back into your OpenLog, it
should use those settings from now on.

If your OpenLog didn't write a CONFIG.TXT file, you can [use this one instead][].

[use this one instead]: https://raw.githubusercontent.com/thenickdude/blackbox/blackbox/tools/blackbox/OpenLog_v3_Blackbox/CONFIG.TXT

### Protection
The OpenLog can be wrapped in black electrical tape in order to insulate it from conductive frames (like carbon fiber),
but this makes its status LEDs impossible to see. I recommend wrapping it with some clear heatshrink tubing instead.

![OpenLog installed](http://i.imgur.com/jYyZ0oC.jpg "OpenLog installed with double-sided tape, SDCard slot pointing outward")

## Enabling this feature
In the [Cleanflight Configurator][], open up the CLI tab. Enable the Blackbox feature by typing in `feature BLACKBOX`
and pressing enter. You also need to assign the Blackbox to one of [your serial ports][] . A 115200 baud port is
required (such as serial_port_1 on the Naze32, the two-pin Tx/Rx header in the center of the board).

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

## Usage
The Blackbox starts recording data as soon as you arm your craft, and stops when you disarm. Each time the OpenLog is
power-cycled, it begins a fresh new log file. If you arm and disarm several times without cycling the power (recording
several flights), those logs will be combined together into one file. The command line tools will ask you to pick which
one of these flights you want to display/decode.

The OpenLog requires a couple of seconds of delay after powerup before it's ready to record, so don't arm your craft
immediately after connecting the battery (you'll probably be waiting for the flight controller to become ready during
that time anyway!)

You should also wait a few seconds after disarming the quad to allow the OpenLog to finish saving its data.

Don't insert or remove the SD card while the OpenLog is powered up.

After your flights, you'll have a series of files labeled "LOG00001.TXT" etc. on the microSD card. You'll need to
decode these with the `blackbox_decode` tool to create a CSV (comma-separated values) file, or render it into a series of
PNG frames with `blackbox_render` which you could convert into a video using another software package.

### Using the blackbox_decode tool
This tool converts a flight log binary file into CSV format. Typical usage (from the command line) would be like:

```bash
blackbox_decode LOG00001.TXT
```

That'll decode the log to `LOG00001.01.csv` and print out some statistics about the log. If you're using Windows, you
can drag and drop your log files onto `blackbox_decode` and they'll all be decoded.

Use the `--help` option to show more details:

```text
Blackbox flight log decoder by Nicholas Sherlock

Usage:
     blackbox_decode [options] <input logs>

Options:
   --help           This page
   --index <num>    Choose the log from the file that should be decoded (or omit to decode all)
   --limits         Print the limits and range of each field
   --stdout         Write log to stdout instead of to a file
   --debug          Show extra debugging information
   --raw            Don't apply predictions to fields (show raw field deltas)
```

### Using the blackbox_render tool
This tool converts a flight log binary file into a series of transparent PNG images that you could overlay onto your
flight video using a video editor (like [DaVinci Resolve][]). Typical usage (from the command line) would be like:

```bash
blackbox_render LOG00001.TXT
```

This will create PNG files at 30 fps into a new directory called `LOG00001.01` next to the log file.

Use the `--help` option to show more details:

```text
Blackbox flight log renderer by Nicholas Sherlock

Usage:
     blackbox_render [options] <logfilename.txt>

Options:
   --help                 This page
   --index <num>          Choose which log from the file should be rendered
   --width <px>           Choose the width of the image (default 1920)
   --height <px>          Choose the height of the image (default 1080)
   --fps                  FPS of the resulting video (default 30)
   --prefix <filename>    Set the prefix of the output frame filenames
   --start <x:xx>         Begin the log at this time offset (default 0:00)
   --end <x:xx>           End the log at this time offset
   --[no-]draw-pid-table  Show table with PIDs and gyros (default on)
   --[no-]draw-craft      Show craft drawing (default on)
   --[no-]draw-sticks     Show RC command sticks (default on)
   --[no-]draw-time       Show frame number and time in bottom right (default on)
   --[no-]plot-motor      Draw motors on the upper graph (default on)
   --[no-]plot-pid        Draw PIDs on the lower graph (default off)
   --[no-]plot-gyro       Draw gyroscopes on the lower graph (default on)
   --smoothing-pid <n>    Smoothing window for the PIDs (default 4)
   --smoothing-gyro <n>   Smoothing window for the gyroscopes (default 2)
   --smoothing-motor <n>  Smoothing window for the motors (default 2)
   --prop-style <name>    Style of propeller display (pie/blades, default pie)
   --gapless              Fill in gaps in the log with straight lines
```

(At least on Windows) if you just want to render a log file using the defaults, you can drag and drop a log onto the
blackbox_render program and it'll start generating the PNGs immediately.

[DaVinci Resolve]: https://www.blackmagicdesign.com/products/davinciresolve