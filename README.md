Cleanflight Configurator
========================

Cleanflight configurator is a fork of Baseflight Configurator to support Cleanflight instead of baseflight.

It allows you to configure the Cleanflight software running on any supported cleanflight target.

Cleanflight is available here:

https://github.com/cleanflight/cleanflight

Authors
-------
Dominic Clifton/hydra - maintainer of the Cleanflight firmware and configurator. 

This configurator is the only configurator for cleanflight specific features, it will likely require that you run the latest firmware on the flight controller,
if you are experiencing any problems please make sure you are running the latest version of firmware.

Installation
------------
1. - Visit [Chrome web store](https://chrome.google.com/webstore/detail/cleanflight-configurator/enacoimjcgeinfnnnpajinjgmkahmfgb)
2. - Click <strong>+ Free</strong>

Alternative way
---------------
1. - Clone the repo to any local directory or download it as zip
2. - Start chromium or google chrome and go to tools -> extension
3. - Check the "Developer mode" checkbox
4. - Click on load unpacked extension and point it to the cleanflight configurator directory (for example D:/cleanflight-configurator)

How to use
-----------
You can find the Cleanflight - Configurator icon in your application tab "Apps"

WebGL
-----
Make sure Settings -> System -> "User hardware acceleration when available" is checked to achieve the best performance

Linux users
-----------
1. Dont forget to add your user into dialout group "sudo usermod -aG dialout YOUR_USERNAME" for serial access
2. If you have 3D model animation problems, Enable "Override software rendering list" in chrome flags chrome://flags/#ignore-gpu-blacklist

Support
-------
For non-Cleanflight specific issues please raise them here so that the get fixed in the primary cdoebase.

https://github.com/multiwii/baseflight-configurator/issues

For Cleanflight specific issues raise them here:

https://github.com/hydra/cleanflight-configurator/issues

There is an IRC channel for cleanflight, here: irc://irc.freenode.net/#cleanflight

Support for baseflight configurator can also be found on irc, here: irc://irc.freenode.net/#multiwii

Technical details
-----------------

The configurator is based on chrome.serial API running on Google Chrome/Chromium core.

Developers
----------
We accept clean and reasonable patches, submit them!

Credits
-------
ctn - primary author and maintainer of Baseflight configurator from which this project was forked.
