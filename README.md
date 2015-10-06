# Cleanflight

![Cleanflight](docs/assets/cleanflight/cleanflight-logo-light-wide-1-240px.jpg)

Clean-code version of baseflight flight-controller - flight controllers are used to fly multi-rotor craft and fixed wing craft.

This fork differs from baseflight in that it attempts to use modern software development practices which result in:

1. greater reliability through code robustness and automated testing. 
2. easier maintenance through code cleanliness.
3. easier to develop new features. 
4. easier to re-use code though code de-coupling and modularisation.

The MultiWii software, from which baseflight originated, violates many good software development best-practices. Hopefully this fork will go some way to address them. If you see any bad code in this fork please immediately raise an issue so it can be fixed, or better yet submit a pull request.

## Additional Features

Cleanflight also has additional features not found in baseflight.

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
* MSP Telemetry.
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

For a list of features, changes and some discussion please review the thread on MultiWii forums and consult the documentation.

http://www.multiwii.com/forum/viewtopic.php?f=23&t=5149

## Installation

See: https://github.com/cleanflight/cleanflight/blob/master/docs/Installation.md 

## Documentation

There is lots of documentation here: https://github.com/cleanflight/cleanflight/tree/master/docs 

If what you need is not covered then refer to the baseflight documentation. If you still can't find what you need then visit the #cleanflight on the Freenode IRC network

## IRC Support and Developers Channel

There's a dedicated IRC channel here:

irc://irc.freenode.net/#cleanflight

If you are using windows and don't have an IRC client installed then take a look at HydraIRC - here: http://hydrairc.com/

Etiquette: Don't ask to ask and please wait around long enough for a reply - sometimes people are out flying, asleep or at work and can't answer immediately.

## Videos

There is a dedicated Cleanflight youtube channel which has progress update videos, flight demonstrations, instructions and other related videos.

https://www.youtube.com/playlist?list=PL6H1fAj_XUNVBEcp8vbMH2DrllZAGWkt8

Please subscribe and '+1' the videos if you find them useful.

## Configuration Tool

To configure Cleanflight you should use the Cleanflight-configurator GUI tool (Windows/OSX/Linux) that can be found here:

https://chrome.google.com/webstore/detail/cleanflight-configurator/enacoimjcgeinfnnnpajinjgmkahmfgb

The source for it is here:

https://github.com/cleanflight/cleanflight-configurator

## Contributing

Contributions are welcome and encouraged.  You can contribute in many ways:

* Documentation updates and corrections.
* How-To guides - received help?  help others!
* Bug fixes.
* New features.
* Telling us your ideas and suggestions.

The best place to start is the IRC channel on freenode (see above), drop in, say hi. Next place is the github issue tracker:

https://github.com/cleanflight/cleanflight/issues
https://github.com/cleanflight/cleanflight-configurator/issues

Before creating new issues please check to see if there is an existing one, search first otherwise you waste peoples time when they could be coding instead!

## Developers

Please refer to the development section in the `docs/development` folder.

TravisCI is used to run automatic builds

https://travis-ci.org/cleanflight/cleanflight

[![Build Status](https://travis-ci.org/cleanflight/cleanflight.svg?branch=master)](https://travis-ci.org/cleanflight/cleanflight)

## Cleanflight Releases
https://github.com/cleanflight/cleanflight/releases


