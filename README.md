# Betaflight Configurator

![Betaflight](http://static.rcgroups.net/forums/attachments/6/1/0/3/7/6/a9088900-228-bf_logo.jpg)

Betaflight Configurator is a crossplatform configuration tool for the Betaflight flight control system.

It runs as an app within Google Chrome and allows you to configure the Betaflight software running on any [supported Betaflight target](https://github.com/betaflight/betaflight/tree/master/src/main/target).

Various types of aircraft are supported by the tool and by Betaflight, e.g. quadcopters, hexacopters, octocopters and fixed-wing aircraft.

## Authors

Betaflight Configurator is a [fork](#credits) of the Cleanflight Configurator with support for Betaflight instead of Cleanflight.

This configurator is the only configurator with support for Betaflight specific features. It will likely require that you run the latest firmware on the flight controller.
If you are experiencing any problems please make sure you are running the [latest firmware version](https://github.com/betaflight/betaflight/releases/).

## Installation

### Via chrome webstore

[![available in the Chrome web store](https://developer.chrome.com/webstore/images/ChromeWebStore_Badge_v2_206x58.png)](https://chrome.google.com/webstore/detail/betaflight-configurator/kdaghagfopacdngbohiknlhcocjccjao)

1. Visit [Chrome web store](https://chrome.google.com/webstore/detail/betaflight-configurator/kdaghagfopacdngbohiknlhcocjccjao)
2. Click **+ Free**

Please note - the application will automatically update itself when new versions are released.  Please ensure you maintain configuration backups as described in the Betaflight documentation.

### Alternative way

1. Clone the repo to any local directory or download it as zip
2. Start Chromium or Google Chrome and go to tools -> extension
3. Check the "Developer mode" checkbox
4. Click on load unpacked extension and point it to the Betaflight Configurator directory (for example D:/betaflight-configurator)

## How to use

You can find the Betaflight Configurator icon in your application tab "Apps"

## Notes

### WebGL

Make sure Settings -> System -> "User hardware acceleration when available" is checked to achieve the best performance

### Linux users

Dont forget to add your user into dialout group "sudo usermod -aG dialout YOUR_USERNAME" for serial access

### Linux / MacOSX users

If you have 3D model animation problems, enable "Override software rendering list" in Chrome flags chrome://flags/#ignore-gpu-blacklist

## Support

If you need help your please use the multiwii or rcgroups forums or visit the IRC channel before raising issues in the issue trackers.

### Issue trackers

For Betaflight configurator issues raise them here

https://github.com/betaflight/betaflight-configurator/issues

For Betaflight firmware issues raise them here

https://github.com/betaflight/betaflight/issues

## Technical details

The configurator is based on chrome.serial API running on Google Chrome/Chromium core.

## Developers

We accept clean and reasonable patches, submit them!

## Credits

ctn - primary author and maintainer of Baseflight Configurator from which Cleanflight Configurator project was forked.

Hydra -  author and maintainer of Cleanflight Configurator from which this project was forked.
