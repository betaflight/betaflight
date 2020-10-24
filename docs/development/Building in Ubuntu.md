# Building in Ubuntu

Building for Ubuntu platform is remarkably easy.
This document is tested and based on the latest Ubuntu 20.04 LTS release and can also be used for WSL.

### Clone betaflight repository and install toolchain

    $ sudo apt update
    $ sudo apt upgrade
    $ sudo apt install build-essential
    $ git clone https://github.com/betaflight/betaflight.git
    $ cd betaflight
    $ make arm_sdk_install

### Updating and Rebuilding Firmware

Navigate to your local betaflight repository and use the following steps to pull the latest changes and rebuild your version of betaflight:

    $ git pull
    $ make clean TARGET=MATEKF405
    $ make TARGET=MATEKF405 [OPTIONS=RANGEFINDER] [DEBUG=DBG]

Using the optional OPTIONS parameters you can specify options like RANGEFINDER.
Using the optional DEBUG parameter you can specify the debugger.

You'll see a set of files being compiled, and finally linked, yielding both an ELF and then a HEX.
You can use the Betaflight-Configurator to flash the `obj/betaflight_MATEKF405.hex` file.
Make sure to remove `obj/` and `make clean`, before building again.

### Building Betaflight Configurator

See [Betaflight Configurator Development](https://github.com/betaflight/betaflight-configurator#development) for how to build the Betaflight Configurator.

### Flashing a target with Betaflight Configurator on Ubuntu 20.04

In most Linux distributions the user won't have access to serial interfaces by default. Flashing a target requires configuration of usb for dfu mode. To add this access right type the following command in a terminal:

    $ sudo usermod -a -G dialout $USER
    $ sudo usermod -a -G plugdev $USER
    $ sudo apt-get remove modemmanager
    $ (echo '# DFU (Internal bootloader for STM32 MCUs)'
     echo 'ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0664", GROUP="plugdev"') | sudo tee /etc/udev/rules.d/45-stdfu-permissions.rules > /dev/null

Please log out and log in to active the settings. You should now be able to flash your target using Betaflight Configurator.


Credit goes to K.C. Budd, AKfreak for testing, and pulsar for doing the long legwork that yielded the original content of this document.