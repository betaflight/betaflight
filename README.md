# Cleanflight

![Cleanflight](docs/assets/cleanflight/cleanflight-logo-light-wide-1-240px.jpg)

Cleanflight is flight controller software for multi-rotor and fixed wings.  The Cleanflight project, and related projects such as Betaflight and iNav are
used on the majority of flight controllers used around the world.  There is no other software used on as many flight-controllers!

* If you're looking for experimental new features and don't mind doing your homework, checkout the [betaflight fork](https://github.com/betaflight/betaflight).
* If you're looking for advanced navigation features then check out the [iNav fork](https://github.com/iNavFlight/inav).
* All other users should use Cleanflight.

Features:

* Awesome flight performance as trusted by the majority of Acrobatic and Racing Drone pilots.
* Support for modern STM32 based processors F1/F3/F4/F7.
* Support for modern accelerometer/gyro/barometer/compass sensors.
* Support for modern ESC technologies DSHOT/ONESHOT and legacy PWM.
* Support for Multi-color RGB LED strip support.
* Advanced on-board telemetry logging (Blackbox).
* Wide support of receivers (SBus/iBus/SumD/SumH/PPM/PWM/CRSF/JetiExBus)
* Wide support of telemetry protocols (FrSky/SmartPort/S.Port/HoTT/iBus/LTM/MavLink/CRSF/SRXL).
* Built-in OSD support & configuration without needing third-party OSD software/firmware/comm devices.
* Support for external OSD slave systems.
* VTX support (RTC6705/Unify Pro(SmartAudio)/IRC Tramp/etc).
* and MUCH, MUCH more.

## Installation & Documentation

* Cleanflight documentation - https://github.com/cleanflight/cleanflight/tree/master/docs
* Betaflight Wiki -  https://github.com/betaflight/betaflight/wiki 

## Support

Your first place for support are the [Cleanflight forums on RCGroups](https://www.rcgroups.com/forums/showthread.php?2249574-Cleanflight-firmware-for-STM32F3-based-FCBs-Check-First-Post-Please!!)

The Github issue tracker is NOT for end-user support.

## IRC Support and Developers Channel

There's a dedicated Cleanflight IRC channel on the Freenode IRC network. Many users and some of the developers frequent there, and it is a helpful and friendly community - but there are two important things to keep in mind: First and most importantly, please go ahead and ask if you have questions, but **make sure you wait around long enough for a reply**. Next, sometimes people are out flying, asleep or at work and can't answer immediately, even though they are present in the channel. This is how IRC works: Many people stay logged in, even though they are not actively participating in the discussion all the time. Have a seat, grab a drink and hang around if it's a quiet time of day.

irc://irc.freenode.net/#cleanflight

If you are using Windows and don't have an IRC client installed, take a look at [HydraIRC](http://hydrairc.com/).

There's a dedicated Slack chat channel for betaflight here:

http://www.betaflight.tk/

Etiquette: Don't ask to ask and please wait around long enough for a reply - sometimes people are out flying, asleep or at work and can't answer immediately.

## Videos

There is a dedicated Cleanflight YouTube channel which has progress update videos, flight demonstrations, instructions and other related videos.

https://www.youtube.com/playlist?list=PL6H1fAj_XUNVBEcp8vbMH2DrllZAGWkt8

Please subscribe and '+1' the videos if you find them useful.

## Configuration Tool

To configure Cleanflight you should use the [Cleanflight-configurator GUI tool](https://chrome.google.com/webstore/detail/cleanflight-configurator/enacoimjcgeinfnnnpajinjgmkahmfgb) (Windows/OSX/Linux).

The source for it is here:

https://github.com/cleanflight/cleanflight-configurator

Note: the configurator auto-updates itself if installed using Chrome, if you need an old version to use old firmware then you can get them here:

https://github.com/cleanflight/cleanflight-configurator/releases


## Contributing

Contributions are welcome and encouraged.  You can contribute in many ways:

* Documentation updates and corrections.
* How-To guides - received help? Help others!
* Bug reporting & fixes.
* New feature ideas & suggestions.

See [CONTRIBUTING.md](CONTRIBUTING.md)

## Developers

Please refer to the development section in the [docs/development](https://github.com/cleanflight/cleanflight/tree/master/docs/development) folder.

TravisCI is used to run automatic builds: https://travis-ci.org/cleanflight/cleanflight

https://travis-ci.org/cleanflight/cleanflight

[![Build Status](https://travis-ci.org/cleanflight/cleanflight.svg?branch=master)](https://travis-ci.org/cleanflight/cleanflight)

## Cleanflight Releases
https://github.com/cleanflight/cleanflight/releases

## Open Source

Cleanflight is software that is **open source** and is available free of charge without warranty to all users.

The license is GPL3.

## Project/Fork History

Cleanflight is forked from Baseflight, which is now dead, all primary development happens in Cleanflight, betaflight and iNav forks.

Cleanflight v2.x -> betaflight -> cleanflight v1.x -> baseflight -> multiwii

## Contributors

Thanks goes to all those whom have contributed to Cleanflight and its origins.

Primary developers:
* Dominic Clifton (hydra) - *cleanflight founder*
* Boris B (borisbstyle) - *betaflight founder*
* digitalentity - *inav founder*
* Martin Budden (martinbudden)
* Jason Blackman (blckmn)

Big thanks to current and past contributors:
* **Alexinparis** (for MultiWii),
* **timecop** (for Baseflight),
* **Sambas** (for the original STM32F4 port).
* Bardwell, Joshua (joshuabardwell)
* ctzsnooze
* Höglund, Anders (andershoglund) 
* Ledvin, Peter (ledvinap) - **IO code awesomeness!**
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
* prodrone - **failsafe work**
* **ctn** - **for the original Configurator**

And many many others who haven't been mentioned....

