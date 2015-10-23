<<<<<<< HEAD
# Cleanflight Configurator

Cleanflight Configurator is a crossplatform configuration tool for the [Cleanflight](http://cleanflight.com/) flight control system.

It runs as an app within Google Chrome and allows you to configure the Cleanflight software running on any [supported Cleanflight target](https://github.com/cleanflight/cleanflight/blob/master/docs/Boards.md).

Various types of aircraft are supported by the tool and by cleanflight, e.g. quadcopters, hexacopters, octocopters and fixed-wing aircraft.

[![available in the Chrome web store](https://developer.chrome.com/webstore/images/ChromeWebStore_Badge_v2_206x58.png)](https://chrome.google.com/webstore/detail/cleanflight-configurator/enacoimjcgeinfnnnpajinjgmkahmfgb)

## Authors

Dominic Clifton/hydra - maintainer of the Cleanflight firmware and configurator. 

Cleanflight Configurator was originally a [fork](#credits) of Baseflight Configurator with support for Cleanflight instead of Baseflight.

This configurator is the only configurator with support for Cleanflight specific features. It will likely require that you run the latest firmware on the flight controller.
If you are experiencing any problems please make sure you are running the [latest firmware version](https://github.com/cleanflight/cleanflight/releases/latest).

## Installation

### Via chrome webstore

1. Visit [Chrome web store](https://chrome.google.com/webstore/detail/cleanflight-configurator/enacoimjcgeinfnnnpajinjgmkahmfgb)
2. Click **+ Free**

Please note - the application will automatically update itself when new versions are released.  Please ensure you maintain configuration backups as described in the Cleanflight documentation.

### Alternative way

1. Clone the repo to any local directory or download it as zip
2. Start Chromium or Google Chrome and go to tools -> extension
3. Check the "Developer mode" checkbox
4. Click on load unpacked extension and point it to the Cleanflight Configurator directory (for example D:/cleanflight-configurator)

## How to use

You can find the Cleanflight Configurator icon in your application tab "Apps"

## Notes

### WebGL

=======
*NOTICE*
========

This code is dead, cTn made the original closed source.  All new development should happen on the cleanflight/development branch.

https://github.com/cleanflight/cleanflight-configurator/tree/development

This copy of the old baseflight code is placed in the cleanflight repository as a courtesy to the Open Source community.

Baseflight Configurator
=======================
Configurator based on chrome.serial API running on Google Chrome/Chromium core

Keep in mind that this configurator is the most up-to-date configurator implementation for Baseflight flight software,
in many cases it requires latest firmware on the flight controller, if you are experiencing any problems,
please make sure you are running the latest version of firmware.

Installation
------------
1. - Visit [Chrome web store](https://chrome.google.com/webstore/detail/baseflight-multiwii-confi/mppkgnedeapfejgfimkdoninnofofigk)
2. - Click <strong>+ Free</strong>

Alternative way
---------------
1. - Clone the repo to any local directory or download it as zip
2. - Start chromium or google chrome and go to tools -> extension
3. - Check the "Developer mode" checkbox
4. - Click on load unpacked extension and point it to the baseflight configurator directory (for example D:/baseflight-configurator)

How to use
-----------
You can find the Baseflight - Configurator icon in your application tab "Apps"

WebGL
-----
>>>>>>> origin/baseflight-configurator-development
Make sure Settings -> System -> "User hardware acceleration when available" is checked to achieve the best performance

### Linux users

1. Dont forget to add your user into dialout group "sudo usermod -aG dialout YOUR_USERNAME" for serial access
2. If you have 3D model animation problems, enable "Override software rendering list" in Chrome flags chrome://flags/#ignore-gpu-blacklist

## Support

If you need help your please use the multiwii or rcgroups forums or visit the IRC channel before raising issues in the issue trackers.

### Issue trackers

For Cleanflight configurator issues raise them here

https://github.com/cleanflight/cleanflight-configurator/issues

For Cleanflight firmware issues raise them here

https://github.com/cleanflight/cleanflight/issues

### IRC Channel

There is an IRC channel for Cleanflight, here: irc://irc.freenode.net/#cleanflight

Support for Baseflight Configurator can also be found on IRC, here: irc://irc.freenode.net/#multiwii

## Technical details

The configurator is based on chrome.serial API running on Google Chrome/Chromium core.

## Developers

We accept clean and reasonable patches, submit them!

## Credits

ctn - primary author and maintainer of Baseflight Configurator from which this project was forked.
