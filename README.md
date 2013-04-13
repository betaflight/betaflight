baseflight / multiwii configurator
==================================
This project was designed to work primarily with baseflight (but should work just fine with anything using the standard MSP)

Configurator is based on chrome.serial API (currently found only in google chrome and chromium web browsers)

I am using Flotr2 as a plotting library (with a few UI tweaks).

Using "webkit" as application wrapper assures full compatibility across operating systems (Windows / Linux/ Mac OS).

Installation
------------
1. - Clone the repo to any local directory or download it as zip
2. - Start chromium or google chrome and go to tools -> extension
3. - Check the "Developer mode" checkbox
4. - Click on load unpacked extension and point it to the baseflight configurator directory (for example D:/baseflight-configurator)
5. - Note: Don't go "inside" configurators directory (just point to the folder)
6. - You are done

How to use
-----------
You can find the baseflight configurator icon in your application tab "Apps"

Application should work "out of the box" on both Windows and Mac OSX

On Linux the situation is a little tricky.
There is an outstanding bug in chromium/google chrome which is preventing setting the correct baud rate via the UI
this bug will be fixed in version 27, but till then you might need to use an 3rd party utility like stty

example command to change the baud rate via stty
stty -F /dev/ttyUSB0 115200