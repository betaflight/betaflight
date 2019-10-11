# Building in Ubuntu

Building for Ubuntu platform is remarkably easy. The only trick to understand is that the Ubuntu toolchain,
which they are downstreaming from Debian, is not compatible with Betaflight. We suggest that you take an
alternative PPA from Terry Guo, found here:
https://launchpad.net/~terry.guo/+archive/ubuntu/gcc-arm-embedded

This PPA has several compiler versions and platforms available. For many hardware platforms (notably Naze)
the 4.9.3 compiler will work fine. For some, older compiler 4.8 (notably Sparky) is more appropriate. We
suggest you build with 4.9.3 first, and try to see if you can connect to the CLI or run the Configurator.
If you cannot, please see the section below for further hints on what you might do.

Adjust the ARM_SDK_DIR variable in make/tools.mk file with the correct "gcc-arm-none-eabi-xxx" version.

## Setup GNU ARM Toolchain

Note specifically the last paragraph of Terry's PPA documentation -- Ubuntu carries its own package for
`gcc-arm-none-eabi`, so you'll have to remove it, and then pin the one from the PPA.
For your release, you should first remove any older pacakges (from Debian or Ubuntu directly), introduce
Terry's PPA, and update:
```bash
sudo apt-get remove binutils-arm-none-eabi gcc-arm-none-eabi
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
```

For Ubuntu 14.10 (current release, called Utopic Unicorn), you should pin:
```bash
sudo apt-get install gcc-arm-none-eabi=4.9.3.2014q4-0utopic12
```

For Ubuntu 14.04 (an LTS as of Q1'2015, called Trusty Tahr), you should pin:
```bash
sudo apt-get install gcc-arm-none-eabi=4.9.3.2014q4-0trusty12
```

For Ubuntu 12.04 (previous LTS, called Precise Penguin), you should pin:
```bash
sudo apt-get install gcc-arm-none-eabi=4.9.3.2014q4-0precise12
```

## Building on Ubuntu

After the ARM toolchain from Terry is installed, you should be able to build from source.
```bash
cd src
git clone git@github.com:betaflight/betaflight.git
cd betaflight
make TARGET=NAZE
```

You'll see a set of files being compiled, and finally linked, yielding both an ELF and then a HEX:
```
...
arm-none-eabi-size ./obj/main/betaflight_NAZE.elf
   text    data     bss     dec     hex filename
  97164     320   11080  108564   1a814 ./obj/main/betaflight_NAZE.elf
arm-none-eabi-objcopy -O ihex --set-start 0x8000000 obj/main/betaflight_NAZE.elf obj/betaflight_NAZE.hex
$ ls -la obj/betaflight_NAZE.hex
-rw-rw-r-- 1 pim pim 274258 Jan 12 21:45 obj/betaflight_NAZE.hex
```

You can use the Betaflight-Configurator to flash the `obj/betaflight_NAZE.hex` file.

## Bricked/Bad build?

Users have reported that the 4.9.3 compiler for ARM produces bad builds, for example on the Sparky hardware platform.
It is very likely that using an older compiler would be fine -- Terry happens to have also a 4.8 2014q2 build in his
PPA - to install this, you can fetch the `.deb` directly:
http://ppa.launchpad.net/terry.guo/gcc-arm-embedded/ubuntu/pool/main/g/gcc-arm-none-eabi/

and use `dpkg` to install:
```bash
sudo dpkg -i gcc-arm-none-eabi_4-8-2014q2-0saucy9_amd64.deb
```

Make sure to remove `obj/` and `make clean`, before building again.

## Updating and rebuilding

Navigate to the local betaflight repository and use the following steps to pull the latest changes and rebuild your version of betaflight:

```bash
cd src/betaflight
git reset --hard
git pull
make clean TARGET=NAZE
make
```

Credit goes to K.C. Budd, AKfreak for testing, and pulsar for doing the long legwork that yielded this very short document.
