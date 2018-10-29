## Betaflight 3.4 Release Notes

**Version 3.4.0: The best Betaflight ever!**

We recognise that most of our users just want to two things with new firmware: Install it, and then go fly it. That's why we have spent a lot of time on working out default settings that fly great on most quads. Just install it and try for yourself! To make your craft fly even better, we've added a brand-new, experimental mode to improve the response to stick input by filtering it instead of interpolating. For more info on setting up 3.4 to optimise flight performance, read [these notes](https://github.com/betaflight/betaflight/wiki/Tuning-Tips-for-Betaflight-3.4).

We've also put a lot of effort into optimising the performance of the firmware, especially for boards with an F7 MCU. Now you can go and buy that shiny new F7 board, while your friends still struggle with their boards with F3 and F4. :wink:

For pilots who are into long range flying, we have added the brand new ['GPS Rescue' mode](https://github.com/betaflight/betaflight/wiki/GPS-rescue-mode). It is a simplified version of the 'return to home' mode that exists in other, more navigation oriented firmware, and the great news is that all it needs to work is a GPS, no fiddling to make a compass work required!

Last but not least, we've also added a bunch of new features to improve the convenience of using Betaflight: You can now copy / paste your logs from an SD card or the on board flash chip by [mounting the flight controller as a storage device](https://github.com/betaflight/betaflight/wiki/Mass-Storage-Device-(MSC)-Support), and you can use your flight controller / TX to [emulate a joystick](https://github.com/betaflight/betaflight/wiki/HID-Joystick-Support) with no extra hardware needed, in order to fly on a simulator.

If you are upgrading from an earlier version of Betaflight, please read the following section containing a list of things that you might have to change in your configuration.

We have tried to make this release as bug free as possible. If you still find a **bug**, please report it back to us by opening an **issue [here](https://github.com/betaflight/betaflight/issues)**.

Happy Props!


## Important information when upgrading

- A number of changes and improvements in this release require changes to the Betaflight configurator. These changes have been added to Betaflight configurator 10.3.1 (installation instructions [here](https://github.com/betaflight/betaflight-configurator#installation)). Please update your Betaflight configurator to version 10.3.1. If you are using the Blackbox Log Viewer, there is an updated version 3.1.0 to go with Betaflight 3.4 (installation instructions [here](https://github.com/betaflight/blackbox-log-viewer#installation));
- as part of the overhaul of the various filter stages, and improvements to the PID loop, the default settings have been re-evaluated and updated. The new default values are designed to make optimal use of the new filtering and improved PID loop, and be flyable with (almost) any hardware. Even if your current set up is working ok, it is probably worthwhile to try and only restore the non-filtering / PID loop settings, and have a go with the new default values (store your old `diff` in a safe place, just in case you don't like the new values :wink:). ([#6036](https://github.com/betaflight/betaflight/pull/6036));
- also as part of the filtering overhaul, the names of the debug modes available to log filtering / tuning data have been improved `NOTCH` (gyro data after scaling, before filtering) is now `GYRO_SCALED`, `GYRO` (gyro data after all filtering has been applied) is now `GYRO_FILTERED` ([#6059](https://github.com/betaflight/betaflight/pull/6059));
- the upper limit of `dterm_setpoint_weight` has been increased to 2000 (corresponding to a value of 20 for 'D Setpoint Weight' in Betaflight configurator). This means that pilots wanting a more 'locked in' stick feeling can increase this value beyond the previous maximum of 254. At the same time, an undocumented scaling change was reverted, and the scale is now again how it is described in the Betaflight configurator. If you are using a custom setting for `dterm_setpoint_weight`, divide your value by 1.27 to get the new value that will give you the same feeling as between 3.1.6 and 3.4.0 ([#5945](https://github.com/betaflight/betaflight/pull/5945), [#6052](https://github.com/betaflight/betaflight/pull/6052));
- Dshot beacon configuraton has been changed. Now the `beacon` CLI command can be used analogous to how the `beeper` command is used. This allows for the Dshot beacon to be disabled individually for the conditions that are supported by it (`RX_SET` and `RX_LOST` at the moment). The old way of disabling the Dshot beacon by setting `beeper_dshot_beacon_tone` to `0` is no longer supported. The Dshot beacon is disabled for all conditions by default, if you want to enable it, use `beacon <condition name|ALL>` in CLI ([#5891](https://github.com/betaflight/betaflight/pull/5891), [#6070](https://github.com/betaflight/betaflight/pull/6070));
- in previous versions of the firmware, there was a race condition that could cause Dshot commands (e.g. activation of crash flip) to be ignored by the ESC when the Dshot beacon was active. To prevent this, a timeout has been added to the Dshot beacon that prevents arming for 1.2 seconds after the Dshot beacon was active ([#6079](https://github.com/betaflight/betaflight/pull/6079));
- validation has been added for the RSSI configuration. Unlike in previous versions, it is no longer possible to have multiple sources for RSSI configured simultaneously, since only one can be active at any one time. If you have got more than one of the supported RSSI sources (frame error count / ADC / RX channel) configured, all but the first on this list will be disabled ([#5644](https://github.com/betaflight/betaflight/pull/5644));
- scaling has been added to all RSSI sources. If the RSSI mechanism that you are using does not give you the output range that you want for RSSI, you can now use the `rssi_scale` / `rssi_offset` CLI variables to set the scale and offset for RSSI ([#6001](https://github.com/betaflight/betaflight/pull/6001), [#6032](https://github.com/betaflight/betaflight/pull/6032));
- the functionality of the crash flip mode has been improved: In addition to the existing front / back / left / right spinning of 2 propellers, it now supports spinning only 1 propeller (by moving the roll / pitch stick diagonally), and spinning 2 props that are diagonally opposite (by moving yaw), in order to yaw the overturned craft. The largest stick deflection in any of these directions determines which properllers are spun ([#5163](https://github.com/betaflight/betaflight/pull/5163));
- the setting `moron_threshold` for the acceptable noise limit during gyro calibration was renamed to `gyro_calib_noise_limit`. Additionally, a new setting `gyro_calib_duration` was added. This allows users to configure a longer minimum gyro calibration duration (in 1/10ths of seconds, default: 125). Using a larger setting here will result in reduced gyro drift, which is helpful when flying line of sight ([#5932](https://github.com/betaflight/betaflight/pull/5932));
- unfortunately bugfixes and improvements in the flight controller core functionality have led to an increase of the firmware size, causing it to overflow the available space on a number of F3 based flight controllers. As a result, some features have had to be removed from a number of F3 based flight controllers in order to make the firmware fit into flash. The following targets are affected: BETAFLIGHTF3, COLIBRI\_RACE, FRSKYF3, FURYF3OSD, LUX\_RACE, MIDELICF3, OMNIBUS, RCEXPLORERF3, RG\_SSD\_F3, SPRACINGF3EVO, SPRACINGF3NEO;
- the OSD elements `osd_crosshairs` (crosshairs) and `osd_ah_sbar` (artificial horizon sidebar) have been renamed in CLI to `osd_crosshairs_pos` and `osd_ah_sbar_pos` to make them consistent with the naming of OSD elements. If you are using these elements, please manually change the names in your backup before restoring ([#5534](https://github.com/betaflight/betaflight/pull/5534));
- the range of the `vtx_band` parameter in CLI was extended to start at 0 instead of 1. Setting `vtx_band = 0` allows users of VTX using the SmartPort or Tramp protocols to set the desired frequency directly via the `vtx_freq` parameter. Since direct frequency setting is not supported by the RTC6705 (onboard) VTX driver `vtx_band = 0` does not work for these VTX, and should not be used ([#5465](https://github.com/betaflight/betaflight/pull/5465)).


## Major features:

- Overhauled and improved filtering ([#5391](https://github.com/betaflight/betaflight/pull/5391), [#5458](https://github.com/betaflight/betaflight/pull/5458));
- Optimised and massively improved the performance on F7 ([#5674](https://github.com/betaflight/betaflight/pull/5674));
- Added GPS rescue mode ([#5753](https://github.com/betaflight/betaflight/pull/5753), [#5764](https://github.com/betaflight/betaflight/pull/5764));
- Added support for accessing SD card / onboard flash as USB mass storage device (MSC) ([#5443](https://github.com/betaflight/betaflight/pull/5443), [#5629](https://github.com/betaflight/betaflight/pull/5629), [#5650](https://github.com/betaflight/betaflight/pull/5650));
- Added support for reading RC input as USB joystick (HID) ([#5478](https://github.com/betaflight/betaflight/pull/5478), [#5596](https://github.com/betaflight/betaflight/pull/5596));
- Added support for CMS configuration over CRSF ([#5743](https://github.com/betaflight/betaflight/pull/5743));
- Added support for experimental filter based RC channel smoothing ([#6017](https://github.com/betaflight/betaflight/pull/6017)).


## Minor features:

- Added acro trainer mode ([#5970](https://github.com/betaflight/betaflight/pull/5970));
- Added throttle boost mode ([#5508](https://github.com/betaflight/betaflight/pull/5508));
- Added support for throttle limiting ([#5608](https://github.com/betaflight/betaflight/pull/5608));
- Added PID loop improvements ([#5968](https://github.com/betaflight/betaflight/pull/5968), [#5963](https://github.com/betaflight/betaflight/pull/5963), [#5962](https://github.com/betaflight/betaflight/pull/5962));
- Added support for accelerated [yaw spin recovery](https://github.com/betaflight/betaflight/wiki/Yaw-Spin-Recovery-and-Gyro-Overflow-Detect) ([#5706](https://github.com/betaflight/betaflight/pull/5706));
- Added support for direct adjustment of PID values through an RC channel ([#5584](https://github.com/betaflight/betaflight/pull/5584));
- Added support for multiple overclocking speeds ([#5193](https://github.com/betaflight/betaflight/pull/5193));
- Added MCU temperature monitoring ([#5322](https://github.com/betaflight/betaflight/pull/5322));
- Added paralyse mode ([#5851](https://github.com/betaflight/betaflight/pull/5851));
- Added support for QMC5883L compass ([#5309](https://github.com/betaflight/betaflight/pull/5309));
- Added support for W25M flash chips ([#5722](https://github.com/betaflight/betaflight/pull/5722)).


## New targets:

- Added SPRACINGF7DUAL with dual gyro support ([#5264](https://github.com/betaflight/betaflight/pull/5264)).
