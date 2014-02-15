baseflight configurator
==================================
Configurator is based on chrome.serial API (currently found only in google chrome and chromium web browsers).

I am using Flotr2 as a plotting library (with a few UI tweaks).

Installation
------------
1. - Visit https://chrome.google.com/webstore/detail/baseflight-multiwii-confi/mppkgnedeapfejgfimkdoninnofofigk
2. - Click add to Chrome
3. - You are done

Alternative way

1. - Clone the repo to any local directory or download it as zip
2. - Start chromium or google chrome and go to tools -> extension
3. - Check the "Developer mode" checkbox
4. - Click on load unpacked extension and point it to the baseflight configurator directory (for example D:/baseflight-configurator)
5. - Note: Don't go "inside" configurators directory (just point to the folder)
6. - You are done

How to use
-----------
You can find the baseflight configurator icon in your application tab "Apps"

Linux users
-----------
1. Dont forget to add your user into dialout group "sudo usermod -aG dialout YOUR_USERNAME" for serial access
2. If you have 3D model animation problems, Enable "Override software rendering list" in chrome flags chrome://flags/#ignore-gpu-blacklist

Developers
----------
We accept clean and reasonable patches, always target the "development" branch for the pull requests