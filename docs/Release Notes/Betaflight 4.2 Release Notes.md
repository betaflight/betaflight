# Betaflight 4.2 Release Notes

**Just in time for summer...**

...for most of us, and in time for the end of COVID-19 related lockdowns in a lot of places, we are proud to announce the release of Betaflight 4.2.0!

This is the third release of Betaflight after the switch to our new approach of how we support targets for different boards, and we have put effort into making support for these 'Unified Targets' easier and more seamless than before. We have also managed to convert most of the existing targets to use this new approach. This is enabling us to add support for new targets without additional effort for the developers, and as a result Betaflight is now supporting more different hardware than ever before (currently we are supporting 185 different flight controller models and counting).

As usual, our flight performance wizards have kept busy as well, and have added a bunch of improvements to how Betaflight flies. On top of that, they have improved the defaults, and made the firmware easier to tune than ever before.

One new feature that will be of interest to pilots still forced to stay inside is the new battery sag compensation: This feature results in a more consistent stick feel for the entire duration of the battery. The feedback that we are getting from many of our test pilots is that this is particularly useful for small indoor setups, like whoops or similar. Have a look at the [4.2 Tuning Notes](https://github.com/betaflight/betaflight/wiki/4.2-Tuning-Notes) to find out how to set this up.

Last but not least, we have also reworked how the gyro loop works, and it now runs at a fixed speed that is identical to the native update speed for the model of gyro that is used. This has simplified the setup of loop speeds, and allowed us to add support for a number of new gyro models - hopefully we'll see manufacturers pick this up and start offering exciting new flight controllers with these new gyros. 

For an extended list of new features see below.


To make sure you get the latest version of your target installed, head over to [this page](https://github.com/betaflight/betaflight-configurator/releases) and make sure you have got the latest version of Betaflight configurator installed **before updating your firmware**.

To get the best out of the flight performance improvements, please read the [Tuning Notes](https://github.com/betaflight/betaflight/wiki/4.2-Tuning-Notes).

If you are upgrading from an earlier version of Betaflight, please read the following section containing a list of things that you might have to change in your configuration.


We have tried to make this release as bug free as possible. If you still find a **bug**, please report it back to us by opening an **issue [here](https://github.com/betaflight/betaflight/issues)**.

_Happy flying, enjoy summer_


## Important information when upgrading

- Betaflight 4.2 brings some changes to how targets are downloaded and installed. The [latest release](https://github.com/betaflight/betaflight-configurator/releases) of Betaflight Configurator, 10.7.0, contains the changes necessary to support this. For this reason **it is important that you update to the version 10.7.0 or newer of Betaflight Configurator (installation instructions [here](https://github.com/betaflight/betaflight-configurator#installation)) in order to get the latest version of your targets installed**;
- if you are using the [Blackbox Explorer](https://github.com/betaflight/blackbox-log-viewer/releases), there is an updated version 3.5.0 to go with Betaflight 4.2 (installation instructions [here](https://github.com/betaflight/blackbox-log-viewer#installation));
- version 1.5.0 of the [Betaflight TX lua scripts](https://github.com/betaflight/betaflight-tx-lua-scripts/releases) has been released. This includes changes to go with Betaflight 4.2 (installation instructions [here](https://github.com/betaflight/betaflight-tx-lua-scripts#installing));
- as you have come to expect, there is a detailed [Tuning Notes](https://github.com/betaflight/betaflight/wiki/4.2-Tuning-Notes) for Betaflight 4.2. Use them, or use the improved tuning sliders in the Betaflight configurator 10.7.0 to get your craft tuned. Please **do not paste tuning configurations from previous versions of the firmware**. Some defaults have been changed, and some parameters are used in different ways, so previous tuning settings will not work well with Betaflight 4.1 ([#8623](https://github.com/betaflight/betaflight/pull/8623), [#8736](https://github.com/betaflight/betaflight/pull/8736));
- after installing new firmware or resetting the configuration, the motor output protocol selected by default is now 'disabled'. This means that the correct motor protocol has to be selected before the craft can be armed. But this also removed the limitation on the maximum possible PID loop speed imposed by the pre-selected legacy protocol (OneShot125) in earlier versions, resulting in the maximum possible PID loop speed for each MCU type being pre-selected. Arming will be disabled and a warning displayed in configurator until a motor output protocol has been selected ([#9619](https://github.com/betaflight/betaflight/pull/9619));
- when enabling bidirectional Dshot, the maximum possible PID loop speed for Dshot protocols will be reduced by half to accommodate for the time taken by the ESC to send the RPM data packet to the flight controller. This does not affect Dshot600, but if bidirectional Dshot is used with Dshot300, the maximum PID loop speed is 4 kHz, and 2 kHz for Dshot150 ([#9642](https://github.com/betaflight/betaflight/pull/9642));
- the threshold for activation of the yaw spin recovery feature has had an automatic mode added - this mode sets the threshold to a value that is based on the currently configured maximum yaw rate, with a buffer added on top of it. This new mode is the default for yaw spin threshold, since it provides a setting that is safe to use for most users. Manual configuration of the yaw spin threshold is still possible ([#9455](https://github.com/betaflight/betaflight/pull/9455));
- accelerometer calibration is now required before arming is allowed if any feature is enabled that requires the accelerometer. This is to prevent situations arising where the craft may try to self-level with an uncalibrated accelerometer, leading to unexpected or dangerous results. Arming will be disabled and a warning displayed in configurator until the accelerometer has been calibrated ([#9031](https://github.com/betaflight/betaflight/pull/9031));
- the calculation for the virtual current meter device has been changed to be based on the throttle value used in the mixer, instead of the throttle channel RC input. This means that this value has things like throttle limiting and throttle boost applied, resulting in more accurate prediction of current and consumption. If you are using the virtual current meter device, make sure to calibrate it again after updating the firmware to make sure the calibration is still correct ([#9153](https://github.com/betaflight/betaflight/pull/9153));
- the way that the 'RSSI dBm' value is tracked in the firmware has changed to use actual dBM value in a range of -130 to 0, instead of the previous version's 130 to 0 range. This means that if a custom setting for `osd_rssi_dbm_alarm` is used, this will have to be changed to be the inversion of its previous value (e.g. `60` becomes `-60`) ([#9550](https://github.com/betaflight/betaflight/pull/9550));
- since stick arming is not recommended for multirotor craft because it can lead to accidental disarms, it has been disabled by default. If you want to use stick arming (at your own risk), it has to be enabled by setting `enable_stick_arming` to `on` ([#9183](https://github.com/betaflight/betaflight/pull/9183));
- the `name` CLI command has been removed. Use `set name = <name>` instead ([#8837](https://github.com/betaflight/betaflight/pull/8837)).


## Major Features:

- completely reworked the gyro loop, improved performance and made it always run at the native speed of the gyro ([#9444](https://github.com/betaflight/betaflight/pull/9444));
- added new selectable ACTUAL and QUICK rates models ([#9495](https://github.com/betaflight/betaflight/pull/9495), [#9506](https://github.com/betaflight/betaflight/pull/9506));
- added compensation for sagging battery voltage, resulting in more consistent throttle / PID behaviour over for the entire flight time ([#9561](https://github.com/betaflight/betaflight/pull/9561));
- Added level race mode ('NFE race mode' in Silverware) ([#9481](https://github.com/betaflight/betaflight/pull/9481)).


## Minor Features:

- added the option to display the OSD logo on arming ([#9244](https://github.com/betaflight/betaflight/pull/9244));
- added support for enhanced OSD / CMS devices, made it possible to support highligting of text or symbols ([#9212](https://github.com/betaflight/betaflight/pull/9212));
- added support for FrSkyOSD OSD devices ([#9127](https://github.com/betaflight/betaflight/pull/9127));
- added support for the Redpine RC protocol on devices with an SPI connected CC2500 chip (FrSky SPI) ([#7601](https://github.com/betaflight/betaflight/pull/7601)).
