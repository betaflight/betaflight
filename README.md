![Betaflight](https://raw.githubusercontent.com/wiki/betaflight/betaflight/images/betaflight/bf_logo.png)

Betaflight is flight controller software (firmware) used to fly multi-rotor craft and fixed wing craft.

This fork differs from Baseflight and Cleanflight in that it focuses on flight performance, leading-edge feature additions, and wide target support.

## News

### Embargo on new targets for Betaflight 4.0

As [announced earlier](https://github.com/betaflight/betaflight#betaflight-40), Betaflight 4.0 will introduce a radically new way to define targets. At the moment, we are making the changes necessary for this, and this requires some changes to how targets are defined in the 'legacy' way. Because of this, any pull request opened against `master` that introduces a new target is likely to be outdated by the time it has been reviewed. This means extra work for the target maintainer and the reviewers, for little or no benefit.

Because of this, the following embargo is put in place:

1. From 30/12/2018 on until the release of Betaflight 4.0 (scheduled for 01/04/2019), no pull requests that introduce new targets against `master` are accepted. Because any such pull requests will be outdated by the time 4.0 is released, they will be closed;

2. During the time that 1. is in place, pull requests adding new targets (in 3.5 format) against the `3.5.x-maintenance` branch will be considered. This is an exception to the rule that all pull requests must be opened against `master`. This exception only applies to pull requests containing no other changes than a new target and a documentation page for the new target. These new targets will be included in upcoming 3.5 maintenance releases;

3. Targets added under the exception in 2. will have to be ported into 4.0 before 4.0 is released. The format that these targets will have in 4.0 is yet to be determined, and instructions for porting the targets will have to be written, but we expect the maintainers of these targets to make themselves available to assist with this task when the time comes. Please follow the news and updates that are posted to https://betaflight.com/ or the [Betaflight GitHub page](https://github.com/betaflight/betaflight) to keep track of when action is required.


### Betaflight 4.0

As you might have learned from the [Betaflight GitHub page](https://github.com/betaflight/betaflight), our next release will be 4.0. Betaflight 4.0 will be the culmination of years of work that started in 2016 with the introduction of remappable resources, and it will drastically change the way how Betaflight is built and distributed. To you as the user, not much in how you download and install the Betaflight firmware will change, but you will get some noticeable improvements:

- we’ll have to spend less time on maintaining and releasing the firmware, meaning that we’ll have more time to work on new and exciting features;
- manufacturers will have an easy way to release custom configurations for all of their boards and ready-to-fly (including RX setup and tuning) craft based on original Betaflight firmware - you will no longer be stuck on using old firmware, or recreating your configuration from scratch;
- the tinkerers amongst you will be able to share Betaflight firmware with your home built improvements amongst your friends without having to build and distribute separate targets for everybody’s board.

*(These changes are planned for F4 and F7, F3’s flash space limitations mean we won’t be able to fit all of this in.)*

We are almost there with the implementation of these changes, but since they are quite complex, and getting ‘almost there’ doesn’t buy us much, we have decided that we need to take more time to complete them, make sure the way users can use the firmware still works as expected, and properly test the new firmware. For this reason we have decided to **postpone the planned release date for Betaflight to 01 April 2019**. We will keep doing monthly releases of Betaflight 3.5 with bugfixes and new / updated targets in the meantime.


To get the latest update from us, you can now also visit our webpage at <https://betaflight.com/>.

In addition to the drastic changes mentioned above, Betaflight 4.0 will have a number of other exciting new features and improvements:

- yet again improved flight performance;
- 'Launch control' mode;
- switchable profiles for the OSD layout.

## Events

| Date  | Event |
| - | - |
| 01 March 2019 | Start of feature freeze / Release Candidate window for Betaflight 4.0 |
| 01 April 2019 | Planned [release](https://github.com/betaflight/betaflight/milestone/20) date for Betaflight 4.0 |

## Features

Betaflight has the following features:

* Multi-color RGB LED strip support (each LED can be a different color using variable length WS2811 Addressable RGB strips - use for Orientation Indicators, Low Battery Warning, Flight Mode Status, Initialization Troubleshooting, etc)
* DShot (150, 300, 600 and 1200), Multishot, and Oneshot (125 and 42) motor protocol support
* Blackbox flight recorder logging (to onboard flash or external microSD card where equipped)
* Support for targets that use the STM32 F7, F4 and F3 processors
* PWM, PPM, and Serial (SBus, SumH, SumD, Spektrum 1024/2048, XBus, etc) RX connection with failsafe detection
* Multiple telemetry protocols (CSRF, FrSky, HoTT smart-port, MSP, etc)
* RSSI via ADC - Uses ADC to read PWM RSSI signals, tested with FrSky D4R-II, X8R, X4R-SB, & XSR
* OSD support & configuration without needing third-party OSD software/firmware/comm devices
* OLED Displays - Display information on: Battery voltage/current/mAh, profile, rate profile, mode, version, sensors, etc
* In-flight manual PID tuning and rate adjustment
* Rate profiles and in-flight selection of them
* Configurable serial ports for Serial RX, Telemetry, ESC telemetry, MSP, GPS, OSD, Sonar, etc - Use most devices on any port, softserial included
* VTX support for Unify Pro and IRC Tramp
* and MUCH, MUCH more.

## Installation & Documentation

See: https://github.com/betaflight/betaflight/wiki

## IRC Support and Developers Channel

There's a dedicated Slack chat channel here:

https://slack.betaflight.com/

Etiquette: Don't ask to ask and please wait around long enough for a reply - sometimes people are out flying, asleep or at work and can't answer immediately.

## Configuration Tool

To configure Betaflight you should use the Betaflight-configurator GUI tool (Windows/OSX/Linux) that can be found here:

https://chrome.google.com/webstore/detail/betaflight-configurator/kdaghagfopacdngbohiknlhcocjccjao

The source for it is here:

https://github.com/betaflight/betaflight-configurator

## Contributing

Contributions are welcome and encouraged.  You can contribute in many ways:

* Documentation updates and corrections.
* How-To guides - received help? Help others!
* Bug reporting & fixes.
* New feature ideas & suggestions.

The best place to start is the IRC channel on gitter (see above), drop in, say hi. Next place is the github issue tracker:

https://github.com/betaflight/betaflight/issues
https://github.com/betaflight/betaflight-configurator/issues

Before creating new issues please check to see if there is an existing one, search first otherwise you waste peoples time when they could be coding instead!

## Developers

Please refer to the development section in the `docs/development` folder.

TravisCI is used to run automatic builds

https://travis-ci.org/betaflight/betaflight

[![Build Status](https://travis-ci.org/betaflight/betaflight.svg?branch=master)](https://travis-ci.org/betaflight/betaflight)

## Betaflight Releases

https://github.com/betaflight/betaflight/releases

## Open Source / Contributors

Betaflight is software that is **open source** and is available free of charge without warranty to all users.

Betaflight is forked from Cleanflight, so thanks goes to all those whom have contributed to Cleanflight and its origins.

Origins for this fork (Thanks!):
* **Alexinparis** (for MultiWii),
* **timecop** (for Baseflight),
* **Dominic Clifton** (for Cleanflight), and
* **Sambas** (for the original STM32F4 port).
* **borisbstyle** (Fork from Cleanflight).

The Betaflight Configurator is forked from Cleanflight Configurator and its origins.

Origins for Betaflight Configurator:
* **Dominic Clifton** (for Cleanflight configurator), and
* **ctn** (for the original Configurator).

Big thanks to current and past contributors:
* Budden, Martin (martinbudden)
* Bardwell, Joshua (joshuabardwell)
* Blackman, Jason (blckmn)
* ctzsnooze
* Höglund, Anders (andershoglund)
* Ledvina, Petr (ledvinap) - **IO code awesomeness!**
* kc10kevin
* Keeble, Gary (MadmanK)
* Keller, Michael (mikeller) - **Configurator brilliance**
* Kravcov, Albert (skaman82) - **Configurator brilliance**
* MJ666
* Nathan (nathantsoi)
* ravnav
* sambas - **bringing us the F4**
* savaga
* Stålheim, Anton (KiteAnton)

And many many others who haven't been mentioned....
