# Blackbox flight data recorder port for Cleanflight

![Rendered flight log frame](http://i.imgur.com/FBphB8c.jpg)

WARNING - This firmware is experimental, and may cause your craft to suddenly fall out of the sky. This port to
Cleanflight has only had 3 test flights! No warranty is offered: if your craft breaks, you get to keep both pieces.

## Introduction
This is a modified version of Cleanflight which adds a flight data recorder function ("Blackbox"). Flight data
information is transmitted over the flight controller's serial port on every control loop iteration to an external
logging device to be recorded.

After your flight, you can process the resulting logs on your computer to either turn them into CSV (comma-separated
values) or render your flight log as a video.

This is a port of my Blackbox feature for Baseflight, so please follow the instructions there for usage details (just
use the Cleanflight firmware from this repository instead of the Baseflight firmware):

https://github.com/thenickdude/blackbox

Instructions which are specific to Cleanflight are included here.

## Installation of firmware
Before installing the new firmware onto your Naze32, back up your configuration: Connect to your flight controller
using the [Cleanflight Configurator][] , open up the CLI tab and enter "dump" into the box at the bottom and press enter.
Copy all of the text that results and paste it into a text document somewhere for safe-keeping.

Click the disconnect button, then on the main page choose the Firmware Flasher option. Tick the box for "Full Chip
Erase" (warning, this will erase all your settings!). Click the "Load firmware (local)" button, and select the file `cleanflight_NAZE.hex`
from the `obj/` directory. Click the "Flash Firmware" button and wait for it to complete.

Now you need to reload your configuration: Go to the CLI tab and paste in the dump that you saved earlier and press
enter, it should execute and restore your settings.

Before you leave the CLI tab, enable the Blackbox feature by typing in `feature BLACKBOX` and pressing enter. You also
need to assign the Blackbox to one of [your serial ports][]. Because it requires a 115200 baud port, the best choice on the
Naze32 to use is serial_port_1, which is the two-pin Tx/Rx header in the centre of the board.

Use `set serial_port_1_scenario=10` to switch the port to Blackbox-only, or `set serial_port_1_scenario=11` to switch it
to MSP, CLI, Blackbox and GPS Passthrough (probably the most useful configuration, since this is the port connected to
USB and you'll still want to access the CLI over it).

Enter `save`. Your configuration should be saved and the flight controller will reboot. You're ready to go!

If you ever need to disable the Blackbox (say, for example, to switch to using the serial port for an OSD instead), you
can either reflash the stock firmware using the Configurator, or you can just turn off the Blackbox feature
by entering `feature -BLACKBOX` on the CLI tab.

[your serial ports]: https://github.com/cleanflight/cleanflight/blob/master/docs/Serial.md
[Cleanflight Configurator]: https://chrome.google.com/webstore/detail/cleanflight-configurator/enacoimjcgeinfnnnpajinjgmkahmfgb?hl=en

## License

This project is licensed under GPLv3.