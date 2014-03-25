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

Linux users
-----------
1. Dont forget to add your user into dialout group "sudo usermod -aG dialout YOUR_USERNAME" for serial access
2. If you have 3D model animation problems, Enable "Override software rendering list" in chrome flags chrome://flags/#ignore-gpu-blacklist

Developers
----------
We accept clean and reasonable patches, always target the <strong>"development"</strong> branch for the pull requests