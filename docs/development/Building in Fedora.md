# Building in Fedora 32

    $ sudo dnf check-update
    $ sudo dnf install git clang libblocksruntime-devel
    $ sudo dnf group install "C Development Tools and Libraries"
    $ git clone https://github.com/betaflight/betaflight.git
    $ cd betaflight
    $ make arm_sdk_install
    $ make TARGET=MATEKF411

### Building Configurator in Fedora 32

    $ sudo dnf check-update
    $ sudo dnf install libatomic rpm-build dpkg
    $ sudo dnf module install nodejs:10
    $ sudo npm install -g gulp-cli yarn
    $ yarn install
    $ yarn gulp debug

### Serial permissions

Remove ModemManager.
Add yourself to the dialout group:

    $ sudo dnf remove ModemManager
    $ sudo usermod -aG dialout $(whoami)

Save and reboot after adding the following contents:

    $ sudo nano /etc/udev/rules.d/45-stdfu-permissions.rules

    # Notify ModemManager this device should be ignored
    ACTION!="add|change|move", GOTO="mm_usb_device_blacklist_end"
    SUBSYSTEM!="usb", GOTO="mm_usb_device_blacklist_end"
    ENV{DEVTYPE}!="usb_device", GOTO="mm_usb_device_blacklist_end"
   
    ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", ENV{ID_MM_DEVICE_IGNORE}="1"
   
    LABEL="mm_usb_device_blacklist_end"
   
    #STM32 DFU Access
    SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", TAG+="uaccess"
