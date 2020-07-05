# Betaflight 4.1 Release Notes

**We're already late, so let's keep this short!**

As you've come to expect, we've packed this release with a whole lot of new flying goodness, new features, and support for a bunch of new targets.

For an extended list of new features see below.


To make sure you get the latest version of your target installed, head over to [this page](https://github.com/betaflight/betaflight-configurator/releases) and make sure you have got the latest version of Betaflight configurator installed **before updating your firmware**.

To get the best out of the flight performance improvements, please read these [tuning tips](https://github.com/betaflight/betaflight/wiki/4.1-Tuning-Notes).

If you are upgrading from an earlier version of Betaflight, please read the following section containing a list of things that you might have to change in your configuration.


We have tried to make this release as bug free as possible. If you still find a **bug**, please report it back to us by opening an **issue [here](https://github.com/betaflight/betaflight/issues)**.

_Have fun_


## Important information when upgrading

- Betaflight 4.1 brings a fundamental change to how targets are downloaded and installed. The latest release of [Betaflight configurator](https://github.com/betaflight/betaflight-configurator/releases), 10.6.0, contains the changes necessary to support this. For this reason **it is important that you update to the version 10.6.0 or newer of Betaflight Configurator (installation instructions [here](https://github.com/betaflight/betaflight-configurator#installation)) in order to get the latest version of your targets installed**;
- if you are using the [Blackbox Explorer](https://github.com/betaflight/blackbox-log-viewer/releases), there is an updated version 3.4.0 to go with Betaflight 4.1 (installation instructions [here](https://github.com/betaflight/blackbox-log-viewer#installation));
- version 1.4.0 of the [Betaflight TX lua scripts](https://github.com/betaflight/betaflight-tx-lua-scripts/releases) has been released. This includes changes to go with Betaflight 4.1 (installation instructions [here](https://github.com/betaflight/betaflight-tx-lua-scripts#installing));
- bidirectional Dshot, which is the basis for RPM based filtering, has been improved and is now available with BLHeli\_32 (from version 32.7 on) and on BLHeli\_S hardware (by using the JESC firmware). Use [these instructions](https://github.com/betaflight/betaflight/wiki/Bidirectional-DSHOT-and-RPM-Filter) to get it set up ([#8554](https://github.com/betaflight/betaflight/pull/8554), [#8779](https://github.com/betaflight/betaflight/pull/8779));
- as you have come to expect, there is a detailed [Tuning Notes](https://github.com/betaflight/betaflight/wiki/4.1-Tuning-Notes) for Betaflight 4.1. Use them, or use the new tuning sliders in the Betaflight configurator 10.6.0 to get your craft tuned. Please **do not paste tuning configurations from previous versions of the firmware**. Some defaults have been changed, and some parameters are used in different ways, so previous tuning settings will not work well with Betaflight 4.1 ([#8623](https://github.com/betaflight/betaflight/pull/8623), [#8736](https://github.com/betaflight/betaflight/pull/8736));
- with the introduction of fully configurable VTX control (VTX tables), after flashing the firmware you now have to load a VTX tables appropriate for your VTX and for the country you are flying in before you can control your VTX through Betaflight. Support for loading VTX tables from files is integrated into Betaflight configurator from version 10.6.0 on. Please see the documentation in the new 'Video Transmitter' tab for instructions on how to find an appropriate VTX table and install it ([#7251](https://github.com/betaflight/betaflight/pull/7251));
- some optimisations were made to the OSD fonts, and some characters were improved. In order to get a properly working OSD with Betaflight 4.1, the font loaded onto the OSD needs to be updated to the latest version (available in configurator 10.6.0 or newer) ([#8390](https://github.com/betaflight/betaflight/pull/8390));
- as announced before the release of Betaflight 4.0, support for F3 based flight controllers has been removed from Betaflight 4.1.


## Major features:

- new and improved feed forward 2.0 ([#8623](https://github.com/betaflight/betaflight/pull/8623), [#8736](https://github.com/betaflight/betaflight/pull/8736));
- reworked bidirectional Dshot ([#8554](https://github.com/betaflight/betaflight/pull/8554), [#8779](https://github.com/betaflight/betaflight/pull/8779));
- dynamic idle management using RPM telemetry ([#8604](https://github.com/betaflight/betaflight/pull/8604));
- fully configurable VTX control with VTX tables ([#7251](https://github.com/betaflight/betaflight/pull/7251)).

 
## Minor features:

- support for the Spektrum SRXL2 serial protocol ([#8606](https://github.com/betaflight/betaflight/pull/8606));
- support for board-specific custom defaults ([#8707](https://github.com/betaflight/betaflight/pull/8707));
- support for arbitrary gyro and mag alignment ([#8474](https://github.com/betaflight/betaflight/pull/8474)).
