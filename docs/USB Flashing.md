# USB Flashing
Some newer boards with full USB support must be flashed in USB DFU mode. This is a straightforward process in Configurator versions 0.67 and newer. The standard flashing procedure should work successfully with the caveat of some platform specific problems as noted below. The "No reboot sequence" checkbox has no effect as the device will automatically be detected when already in bootloader mode (a DFU device will appear in the connect dropdown if this is the case). The Full chip erase checkbox operates as normal. The baudrate checkbox is ignored as it has no relevance to USB.

### Charging-Only Cables
If you see no signs of life on your host computer when you plug in your board, check your cable with your mobile phone or some other USB device - some charging cables have only the power pins connected. These will power up the board, so the leds light up, but the host computer will not react to the device at all. You need a proper USB cable to connect your board to the Betaflight Configurator.

## Platform Specific: Linux
In order for Betaflight configurator to be able to access serial ports, your account needs to be in the `dialout` group. You can add yourself to this group as below: 
```
sudo usermod -a -G dialout <username>
```
If your logs show `unable to open serial port` you might have skipped this step. 
### Ubuntu
Linux requires udev rules to allow write access to USB devices for users. An example shell command to achieve this on Ubuntu is shown here:
```
(echo '# DFU (Internal bootloader for STM32 MCUs)'
 echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0664", GROUP="plugdev"') | sudo tee /etc/udev/rules.d/45-stdfu-permissions.rules > /dev/null
```

This assigns the device to the plugdev group(a standard group in Ubuntu). To check that your account is in the plugdev group type `groups` in the shell and ensure plugdev is listed. If not you can add yourself as shown (replacing `<username>` with your username):
```
sudo usermod -a -G plugdev <username>
```
### Fedora
If you are using Fedora, you will not need to add your account to the plugdev group. 
Instead use the `uaccess` tag in your udev rule for dfu:
```
(echo '# DFU (Internal bootloader for STM32 MCUs)'
 echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0664", TAG+="uaccess"') | sudo tee /etc/udev/rules.d/45-stdfu-permissions.rules > /dev/null
```

### Connectivity Issues: Modem Manager

If you see your ttyUSB device disappear right after the board is connected, chances are that the ModemManager service (that handles network connectivity for you) thinks it is a GSM modem. If this happens, you can issue the following command to disable the service:
```
sudo systemctl stop ModemManager.service 
```

If your system lacks the systemctl command, use any equivalent command that works on your system to disable services. You can likely add your device ID to a blacklist configuration file to stop ModemManager from touching the device, if you need it for cellural networking, but that is beyond the scope of betaflight documentation.

If you see the ttyUSB device appear and immediately disappear from the list in Betaflight Configurator when you plug in your flight controller via USB, chances are that NetworkManager thinks your board is a GSM modem and hands it off to the ModemManager daemon as the flight controllers are not known to the blacklisted.


## Platform Specific: Windows
Chrome can have problems accessing USB devices on Windows. A driver should be automatically installed by Windows for the ST Device in DFU Mode but this doesn't always allow access for Chrome. The solution is to replace the ST driver with a libusb driver. The easiest way to do that is to download [Zadig](http://zadig.akeo.ie/). 
With the board connected and in bootloader mode (reset it by sending the character R via serial, or simply attempt to flash it with the correct serial port selected in Configurator): 
* Open Zadig
* Choose Options > List All Devices
* Select `STM32 BOOTLOADER` in the device list
* Choose `WinUSB (v6.x.x.x)` in the right hand box
![Zadig Driver Procedure](assets/images/zadig-dfu.png)
* Click Replace Driver
* Restart Chrome (make sure it is completely closed, logout and login if unsure)
* Now the DFU device should be seen by Configurator
