## Betaflight 3.5 Release Notes

**Wait, there is one more thing we can do to make it better!**

This is what we realised about two months ago, as we were preparing for the release of Betaflight 3.4.0. And this is what led to the 'Feed Forward PID controller' being born. But when we came up with it it was already too late to add it to 3.4.0, and it needed some more refinement before it was ready to go out anyway. So we decided to do Betaflight 3.5.0, a release that focuses on more flight improvements. We put the Feed Forward PID controller into it, we have made the dynamic notch filter a whole lot more awesome, and we made improvements to how anti gravity works. Man, all of these improvements show when you fly it!

To get the best out of the flight performance improvements, please read these [tuning tips](https://github.com/betaflight/betaflight/wiki/3.5-tuning-notes).

If you are upgrading from an earlier version of Betaflight, please read the following section containing a list of things that you might have to change in your configuration.

We have tried to make this release as bug free as possible. If you still find a **bug**, please report it back to us by opening an **issue [here](https://github.com/betaflight/betaflight/issues)**.

We also have a Facebook Group: If you want to talk about Betaflight, ask configuration questions, or just hang out with fellow pilots, you can do this [here](https://www.facebook.com/groups/betaflightgroup/).

Happy Props!


## Important information when upgrading

- A number of changes and improvements in this release require changes to the Betaflight configurator. These changes have been added to [Betaflight configurator 10.4.0](https://github.com/betaflight/betaflight-configurator/releases/tag/10.4.0)(installation instructions [here](https://github.com/betaflight/betaflight-configurator#installation)), please update your Betaflight configurator to at least this version;
- if you are using the Blackbox Log Viewer, there is an updated [version 3.2.0](https://github.com/betaflight/blackbox-log-viewer/releases/tag/3.2.0) to go with Betaflight 3.5 (installation instructions [here](https://github.com/betaflight/blackbox-log-viewer#installation)). Please update to at least version 3.2.0;
- a new 'Feed Forward PID' algorithm has been implemented, replacing setpoint weight ([#6355](https://github.com/betaflight/betaflight/pull/6355)). In addition to this, the dynamic notch filter ([#6411](https://github.com/betaflight/betaflight/pull/6411)) and anti-gravity ([#6220](https://github.com/betaflight/betaflight/pull/6220)) have been optimised for improved flight performance. For all of these changes, default values have been chosen that should result in good flight characteristics for most setups. It is recommended to start testing with default settings, incorporating tuned settings from previous versions if needed, where needed. For more in-depth instructions for tuning Betaflight 3.5, please consult [these notes](https://github.com/betaflight/betaflight/wiki/3.5-tuning-notes).
- unfortunately, bugfixes in the flight controller core functionality have led to an increase of the firmware size, causing it to overflow the available space on a number of F3 based flight controllers. As a result, some features have had to be removed from a number of F3 based flight controllers in order to make the firmware fit into flash. The following targets are affected: CRAZYBEEF3FR, CRAZYBEEF3FS, FRSKYF3, FURYF3, FURYF3OSD, OMNIBUS, SPRACINGF3, SPRACINGF3EVO, SPRACINGF3MINI, SPRACINGF3NEO ([#6497](https://github.com/betaflight/betaflight/pull/6497), [#6501](https://github.com/betaflight/betaflight/pull/6501))


## Major features:

- Added support for feed forward to the PID controller ([#6355](https://github.com/betaflight/betaflight/pull/6355)).
- Improved the performance of the dynamic notch filter ([#6411](https://github.com/betaflight/betaflight/pull/6411)).

## Minor features:

- Improved the performance of anti-gravity ([#6220](https://github.com/betaflight/betaflight/pull/6220));
- Added support for linking of modes ([#6335](https://github.com/betaflight/betaflight/pull/6335));
- Added support for dynamic filter in dual gyro mode ([#6428](https://github.com/betaflight/betaflight/pull/6428)).
