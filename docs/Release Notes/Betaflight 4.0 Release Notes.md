## Betaflight 4.0 Release Notes
  
**It took us a long time, but now it’s here, and it’s got so many new things!**

When we released 3.5 in August last year, the number of different targets supported by Betaflight had reached 150, and maintaining them was becoming a major effort and taking up a lot of our time. It became clear that we had to do something. We had been working on changing the architecture of Betaflight to make it possible to use the same firmware for different flight controllers for some time, and so we decided to complete this change before releasing the next version, and that because of this fundamental change the next version would be 4.0.

As we approached the originally set release date for Betaflight 4.0, we realised that we were not quite ready, and we decided to take another three months, in order to be able to complete the work we had started.

So now, here we are, ‘Unified Targets’, as we called the new ‘one firmware for many flight controllers’ technology, is now a reality in Betaflight 4.0. There is still some work left to be done to add support for flashing Unified Targets with their configurations to Betaflight configurator, but once this is done, we will be able to allow manufacturers to make firmware for any number of boards, or RTF products, available directly in configurator.

As you have come to expect from Betaflight, we have picked up a number of new and exciting improvements to the flight performance, like ESC RPM based filtering, D term management with D\_min, and throttle based dynamic gyro and D term filtering.

And, again as expected we have added some more, non flight related new features like launch control, OSD profiles, and support for on-board SPI attached RX.

For an extended list of new features see below.


To get the best out of the flight performance improvements, please read these [tuning tips](https://github.com/betaflight/betaflight/wiki/4.0-Tuning-Notes).

If you are upgrading from an earlier version of Betaflight, please read the following section containing a list of things that you might have to change in your configuration.

We have tried to make this release as bug free as possible. If you still find a **bug**, please report it back to us by opening an **issue [here](https://github.com/betaflight/betaflight/issues)**.

_Kia kaha_ (stay strong)


## Important information when upgrading

- A number of changes and improvements in this release require changes to the Betaflight configurator. These changes have been added to [Betaflight configurator 10.5.1](https://github.com/betaflight/betaflight-configurator/releases/tag/10.5.1) (installation instructions [here](https://github.com/betaflight/betaflight-configurator#installation)), please update your Betaflight configurator to at least this version;
- if you are using the Blackbox Log Viewer, there is an updated [version 3.3.1](https://github.com/betaflight/blackbox-log-viewer/releases/tag/3.3.1) to go with Betaflight 4.0 (installation instructions [here](https://github.com/betaflight/blackbox-log-viewer#installation)). Please update to at least this version;
- if you are using OpenTX and the Betaflight TX Lua scripts, please update them to at least [version 1.2.0](https://github.com/betaflight/betaflight-tx-lua-scripts/releases/tag/1.2.0) in order to get support for the latest features introduced in Betaflight 4.0;
- there are a number of flight performance related improvements in Betaflight 4.0. Therefore, **pasting back-ups of the tuning settings for older versions of Betaflight into Betaflight 4.0 will most likely result in poor flight performance**! The default settings in Betaflight 4.0 should be reasonably well flyable for most hardware configurations, but users wanting to tune the performance of their craft should read the [4.0 Tuning Notes](https://github.com/betaflight/betaflight/wiki/4.0-Tuning-Notes) to learn everything they need to know about tuning Betaflight 4.0 ([#6432](https://github.com/betaflight/betaflight/pull/6432), [#6943](https://github.com/betaflight/betaflight/pull/6943), [#7078](https://github.com/betaflight/betaflight/pull/7078), [#7264](https://github.com/betaflight/betaflight/pull/7264), [#7271](https://github.com/betaflight/betaflight/pull/7271), [#7304](https://github.com/betaflight/betaflight/pull/7304), [#7373](https://github.com/betaflight/betaflight/pull/7373), [#7538](https://github.com/betaflight/betaflight/pull/7538));
- there was a fix to how the `min_check` 'zero throttle' dead zone is applied. Before the fix, there was an additional, undocumented deadband of the same size as the `min_check` range above the `min_check` range. The fix has not brought a change to the arming behaviour and throttle is still required to be below `min_check` for arming to be possible. However the extra unintended throttle deadband above `min_check` has been eliminated. This will result in more responsive throttle near minimum and a slight increase in throttle resolution. If you desire to retain the same range for the 'zero throttle' dead zone, you need to double the amount of dead zone configured in `min_check` (offset from 1000). Be aware that this will also double the throttle range within which the craft can be armed ([#7463](https://github.com/betaflight/betaflight/pull/7463));
- there is a new 'Stick Overlay' OSD element that shows an overlay of the current stick positions. In order to use it, the font loaded onto the OSD needs to be updated to the latest version (available in configurator 10.5.0 or newer) ([#7476](https://github.com/betaflight/betaflight/pull/7476));
- the functionality of the 'crash flip arrow' OSD element was extended to activate as well when the craft is not in crash flip mode, but `small_angle` is set, and the craft is unarmed and tilted more than `small_angle`. This is to show pilots that they won't be able to arm from the current orientation, and allow them to activate crash flip mode and then right their craft ([#7250](https://github.com/betaflight/betaflight/pull/7250));
- As part of the introduction of Unified Targets, the existing `resource` command was complemented with two new commands for resource management: `timer` and `dma`. Just like `resource` can be used to assign pins to functions, `timer` can be used to assign timers to pins, and `dma` can be used to assign DMA streams to subsystems and pins (if they have a timer assigned). **Important:** Since DMA streams are linked to pins through timers, `timer` assignments for pins have to be made first, before `dma` assignments to these pins can be made ([#5824](https://github.com/betaflight/betaflight/pull/5824), [#6837](https://github.com/betaflight/betaflight/pull/6837), [#7620](https://github.com/betaflight/betaflight/pull/7620));
- The syntax for the `resource`, `timer`, and `dma` commands was consolidated, and as part of this the `resource list` subcommand was renamed into `resource show`. This aligns it with the new `dma show` and `timer show` subcommands ([#7712](https://github.com/betaflight/betaflight/pull/7712));
- the following parameters were renamed to more closely match their function: `p_level` => `angle_level_strength`, `i_level` => `horizon_level_strength`, `d_level` => `horizon_transition` ([#6673](https://github.com/betaflight/betaflight/pull/6673));
- the gyro configuration was unified from being done by gyro hardware model to being generic for all gyro models. This means that for some targets that are used for multiple different boards that come with different gyro models _that are mounted with different orientations_, it is necessary from Betaflight 4.0 on to manually set `gyro_1_sensor_align` (and / or `gyro_2_sensor_align` for boards with a second gyro) to match the gyro orientation on the board that is used. See note [#6761](https://github.com/betaflight/betaflight/pull/6761) for an explanation, and instructions for individual boards. This is a temporary workaround, per-board configurations with correct gyro alignment will be released as Unified Target configurations ([#5868](https://github.com/betaflight/betaflight/pull/5868));
- unfortunately, bugfixes in the flight controller core functionality have led to an increase of the firmware size, causing it to overflow the available space on almost all F3 based flight controllers. As a result, features have had to be removed from all but a few of the currently supported F3 based flight controllers. Some of the affected targets are: AIORACERF3, BETAFLIGHTF3, CHEBUZZF3, CRAZYBEEF3FR, FURYF3, FURYF3OSD, IMPULSERCF3, LUX\_RACE, LUXV2\_RACE, MIDELICF3, OMNIBUS, RACEBASE, RMDO, SIRINFPV, SPRACINGF3, SPRACINGF3MINI, SPRACINGF3NEO, STM32F3DISCOVERY ([#6900](https://github.com/betaflight/betaflight/pull/6900), [#6955](https://github.com/betaflight/betaflight/pull/6955), [#7037](https://github.com/betaflight/betaflight/pull/7037), [#7038](https://github.com/betaflight/betaflight/pull/7038), [#7045](https://github.com/betaflight/betaflight/pull/7045), [#7306](https://github.com/betaflight/betaflight/pull/7306), [#7381](https://github.com/betaflight/betaflight/pull/7381), [#7392](https://github.com/betaflight/betaflight/pull/7392), [#7402](https://github.com/betaflight/betaflight/pull/7402), [#7421](https://github.com/betaflight/betaflight/pull/7421), [#7501](https://github.com/betaflight/betaflight/pull/7501), [#7508](https://github.com/betaflight/betaflight/pull/7508), [#7518](https://github.com/betaflight/betaflight/pull/7518), [#7829](https://github.com/betaflight/betaflight/pull/7829), [#7842](https://github.com/betaflight/betaflight/pull/7842));
- in addition to the above point, the following features had to be removed from all F3 based flight controllers in order to make the firmware fit into flash: Smart Feedforward, support for flashing / configuration of SimonK based ESCs ([#7272](https://github.com/betaflight/betaflight/pull/7272), [#7274](https://github.com/betaflight/betaflight/pull/7274), [#7391](https://github.com/betaflight/betaflight/pull/7391));
- support for status display through LED strips was removed from F3 based flight controllers, in order to be able to reclaim some flash space. In its place, [LED Strip Profiles](https://github.com/betaflight/betaflight/blob/master/docs/LedStrip.md#led-strip-profiles) (except for the 'Status' profile which is not supported on F3) can be used to set LED strips to fixed colours. LED strip profiles are also available on F4 / F7, to enable simple configuration of the LED strip in OSD ([#7485](https://github.com/betaflight/betaflight/pull/7485));
- since the above three measures proved to be insufficient to keep flash space from overflowing on F3 based flight controllers, a system of classifying F3 based flight controllers into a number of 'feature cut' levels, and removing features according from them according to their level were introduced. This was used to reduce the number of features built into most F3 based flight controllers even more ([#7429](https://github.com/betaflight/betaflight/pull/7429)).


## Major features:

- real time ESC RPM feedback, and notch filtering based on motor RPM ([#7264](https://github.com/betaflight/betaflight/pull/7264), [#7271](https://github.com/betaflight/betaflight/pull/7271));
- D term management with D\_min ([#7373](https://github.com/betaflight/betaflight/pull/7373), [#7538](https://github.com/betaflight/betaflight/pull/7538));
- throttle based dynamic gyro and D term filtering ([#6943](https://github.com/betaflight/betaflight/pull/6943));
- launch control ([#6992](https://github.com/betaflight/betaflight/pull/6992));
- switchable OSD profiles ([#6714](https://github.com/betaflight/betaflight/pull/6714));
- SPI attached Spektrum RX ([#7210](https://github.com/betaflight/betaflight/pull/7210));
- Unified Targets ([#5824](https://github.com/betaflight/betaflight/pull/5824), [#6837](https://github.com/betaflight/betaflight/pull/6837), [#7620](https://github.com/betaflight/betaflight/pull/7620)).


## Minor features:

- cascaded dynamic notches ([#7078](https://github.com/betaflight/betaflight/pull/7078));
- thrust linearisation ([#7304](https://github.com/betaflight/betaflight/pull/7304));
- integrated yaw control ([#6432](https://github.com/betaflight/betaflight/pull/6432));
- switchable LED\_STRIP profiles ([#7303](https://github.com/betaflight/betaflight/pull/7303));
- stick overlays in OSD ([#7167](https://github.com/betaflight/betaflight/pull/7167)); 
- profile switching based on battery cell count ([#7516](https://github.com/betaflight/betaflight/pull/7516));
- per profile limiting of maximum motor output ([#7482](https://github.com/betaflight/betaflight/pull/7482));
- support for the Futaba SFHSS protocol on CC2500 (FrSky SPI) hardware ([#6865](https://github.com/betaflight/betaflight/pull/6865));
- EU LBT mode for the FrSky SPI RX ([#7339](https://github.com/betaflight/betaflight/pull/7339));
- support for STM32F765xx based flight controllers ([#6669](https://github.com/betaflight/betaflight/pull/6669)).
- configuration over HoTT telemetry ([#6224](https://github.com/betaflight/betaflight/pull/6224)).
