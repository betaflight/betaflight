# Building in Ubuntu

Building for Ubuntu platform is remarkably easy with Ubuntu.

## Ubuntu 20.04

### Clone the betaflight repository and install toolchain

```sudo apt update
sudo apt upgrade
sudo apt install build-essential
cd ~\dev
git clone https://github.com/betaflight/betaflight.git
cd betaflight
make arm_sdk_install
```

### Updating and Rebuilding Firmware

Navigate to the local betaflight repository and use the following steps to pull the latest changes and rebuild your version of betaflight:

```cd ~\dev\betaflight
rm -rf obj/
git reset --hard
git pull
make clean TARGET=MATEKF722	
make TARGET=MATEKF722 [OPTIONS=RANGEFINDER]
```

You'll see a set of files being compiled, and finally linked, yielding both an ELF and then a HEX.
You can use the Betaflight-Configurator to flash the `obj/betaflight_MATEKF722.hex` file.
Make sure to remove `obj/` and `make clean`, before building again.

### Building Betaflight Configurator

See [Betaflight Configurator Development](https://github.com/betaflight/betaflight-configurator#development) for how to build the Betaflight Configurator.

### Flashing a target with Betaflight Configurator on Ubuntu 20.04

In most Linux distributions the user won't have access to serial interfaces by default. Flashing a target requires configuration of usb for dfu mode. To add this access right type the following command in a terminal, please log out and log in to take effect:

```sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER
sudo apt-get remove modemmanager
lsusb
(echo '# DFU (Internal bootloader for STM32 MCUs)'
 echo 'ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0664", GROUP="plugdev"') | sudo tee /etc/udev/rules.d/45-stdfu-permissions.rules > /dev/null
```

You should now be able to flash your target using Betaflight Configurator.


Credit goes to K.C. Budd, AKfreak for testing, and pulsar for doing the long legwork that yielded the original content of this document.
