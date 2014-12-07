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
Make sure Settings -> System -> "User hardware acceleration when available" is checked to achieve the best performance

Linux users
-----------
1. Dont forget to add your user into dialout group "sudo usermod -aG dialout YOUR_USERNAME" for serial access
2. If you have 3D model animation problems, Enable "Override software rendering list" in chrome flags chrome://flags/#ignore-gpu-blacklist

Licensing
---------
Baseflight Configurator is licensed under GPL V3, with the following exceptions:

1. You can't publish any iteration of the application in the chrome web store.
2. Artwork (Image files, 3D model files, etc...) requires authors permission to be used outside of this project.

Developers
----------
We accept clean and reasonable patches, always target the <strong>"development"</strong> branch for the pull requests
