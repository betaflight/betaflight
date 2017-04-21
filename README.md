# INAV

![INAV](http://static.rcgroups.net/forums/attachments/6/1/0/3/7/6/a9088858-102-inav.png)

[![Join the chat at https://gitter.im/iNavFlight/inav](https://badges.gitter.im/iNavFlight/inav.svg)](https://gitter.im/iNavFlight/inav?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge) ![Travis CI status](https://travis-ci.org/iNavFlight/inav.svg?branch=master)

Clean-code version of baseflight flight-controller with improved navigational capabilities - flight controllers are used to fly multi-rotor craft and fixed wing craft.

## Important: PID values and scaling

Starting at 22-06-2016 INAV uses the same scaling for PIDs as Cleanflight/Betaflight LuxFloat and MWRewrite PID controllers. That means the following:

* PIDs from CF/BF can be used in INAV, no need to retune for INAV
* INAV uses the same PID defaults that Cleanflight and Betaflight
* Current INAV tunes can be converted to new using [this guide](https://github.com/iNavFlight/inav/wiki/PID-conversion-from-pre-1.2-to-1.2). This applies to all INAV 1.1
* [Conversion spreadsheet](https://docs.google.com/spreadsheets/d/133vfzz6_38W5nUmoRNuP7ZX9V1E-8IG6x0FxuxkBuQg/edit?usp=sharing)

## Features

* Multi-color RGB LED Strip support (each LED can be a different color using variable length WS2811 Addressable RGB strips - use for Orientation Indicators, Low Battery Warning, Flight Mode Status, etc)
* Oneshot ESC support.
* Blackbox flight recorder logging (to onboard flash or external SD card).
* Support for additional targets that use the STM32F3 processors (baseflight only supports STM32F1).
* Support for the Seriously Pro Racing F3 board (STM32F303, I2C sensors, large flash, excellent I/O.)
* Support for the TauLabs Sparky board (STM32F303, I2C sensors, based board with acc/gyro/compass and baro, ~$35)
* Support for the OpenPilot CC3D board. (STM32F103, board, SPI acc/gyro, ~$20)
* Support for the CJMCU nano quadcopter board.
* Support for developer breakout boards: (Port103R, EUSTM32F103RC, Olimexino, STM32F3Discovery).
* Support for more than 8 RC channels - (e.g. 16 Channels via FrSky X4RSB SBus).
* Support for N-Position switches via flexible channel ranges - not just 3 like baseflight or 3/6 in MultiWii
* Lux's new PID (uses float values internally, resistant to looptime variation).
* Simultaneous Bluetooth configuration and OSD.
* Better PWM and PPM input and failsafe detection than baseflight.
* Better FrSky Telemetry than baseflight.
* LTM Telemetry.
* Smartport Telemetry.
* RSSI via ADC - Uses ADC to read PWM RSSI signals, tested with FrSky D4R-II and X8R.
* OLED Displays - Display information on: Battery voltage, profile, rate profile, version, sensors, RC, etc.
* In-flight manual PID tuning and rate adjustment.
* Rate profiles and in-flight selection of them.
* Graupner PPM failsafe.
* Graupner HoTT telemetry.
* Multiple simultaneous telemetry providers.
* Configurable serial ports for Serial RX, Telemetry, MSP, GPS - Use most devices on any port, softserial too.
* And many more minor bug fixes.

For a list of features, changes and some discussion please review the thread on RCGroups forums and consult the documentation.

http://www.rcgroups.com/forums/showthread.php?t=2495732

## Installation

See: https://github.com/iNavFlight/inav/blob/master/docs/Installation.md

## Documentation

There is lots of documentation here: https://github.com/iNavFlight/inav/tree/master/docs

If what you need is not covered then refer to the [Cleanflight documentation](https://github.com/cleanflight/cleanflight/tree/master/docs). If you still can't find what you need then visit the Gitter room or the IRC Channel at #inavflight

## Gitter Support and Developers Channel

There's a dedicated Gitter room here:
https://gitter.im/iNavFlight/inav

There's a dedicated INAV IRC channel on the Freenode IRC network. Users and some of the developers are there, and it is a helpful and friendly community - but there are two important things to keep in mind: First and most importantly, please go ahead and ask if you have questions, but make sure you wait around long enough for a reply. Next, sometimes people are out flying, asleep or at work and can't answer immediately, even though they are present in the channel. This is how IRC works: Many people stay logged in, even though they are not actively participating in the discussion all the time. Have a seat, grab a drink and hang around if it's a quiet time of day.

irc://irc.freenode.net/#inavflight

## Configuration Tool

To configure INAV you should use the [INAV-configurator GUI tool](https://chrome.google.com/webstore/detail/inav-configurator/fmaidjmgkdkpafmbnmigkpdnpdhopgel) (Windows/OSX/Linux).

The source for it is here:

https://github.com/iNavFlight/inav-configurator

If you rather just want to use Cleanflight configurator you can download from here:
https://chrome.google.com/webstore/detail/cleanflight-configurator/enacoimjcgeinfnnnpajinjgmkahmfgb


## Contributing

Contributions are welcome and encouraged.  You can contribute in many ways:

* Documentation updates and corrections.
* How-To guides - received help?  help others!
* Bug fixes.
* New features.
* Telling us your ideas and suggestions.

The best place to start is the IRC channel on freenode (see above), drop in, say hi. Next place is the github issue tracker:

https://github.com/iNavFlight/inav/issues

https://github.com/iNavFlight/inav-configurator/issues

Before creating new issues please check to see if there is an existing one, search first otherwise you waste peoples time when they could be coding instead!

## Developers

Please refer to the development section in the [docs/development](https://github.com/iNavFlight/inav/tree/master/docs/development) folder.


## INAV Releases
https://github.com/iNavFlight/inav/releases
