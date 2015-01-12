# Building in Ubuntu

Building for Ubuntu platform is remarkably easy. The only trick to understand is that the Ubuntu toolchain,
which they are downstreaming from Debian, is not compatible with Cleanflight. We suggest that you take an
alternative PPA from Terry Guo, found here:
 
https://launchpad.net/~terry.guo/+archive/ubuntu/gcc-arm-embedded

## Setup GNU ARM Toolchain

Note specifically the last paragraph of Terry's PPA documentation -- Ubuntu carries its own package for
`gcc-arm-none-eabi`, so you'll have to remove it, and then pin the one from the PPA.
For your release, you should first remove any older pacakges (from Debian or Ubuntu directly), introduce
Terry's PPA, and update:
```
sudo apt-get remove binutils-arm-none-eabi gcc-arm-none-eabi
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
sudo apt-get update
```

For Ubuntu 14.10 (current release, called Utopic Unicorn), you should pin:
```
sudo apt-get install gcc-arm-none-eabi=4.9.3.2014q4-0utopic12
```

For Ubuntu 14.04 (an LTS as of Q1'2015, called Trusty Tahr), you should pin:
```
sudo apt-get install gcc-arm-none-eabi=4.9.3.2014q4-0trusty12
```

For Ubuntu 12.04 (previous LTS, called Precise Penguin), you should pin:
```
sudo apt-get install gcc-arm-none-eabi=4.9.3.2014q4-0precise12
```

## Building on Ubuntu

After the ARM toolchain from Terry is installed, you should be able to build from source.
```
cd src
git clone git@github.com:cleanflight/cleanflight.git
cd cleanflight
make TARGET=NAZE
```

You'll see a set of files being compiled, and finally linked, yielding both an ELF and then a HEX:
```
...
arm-none-eabi-size ./obj/main/cleanflight_NAZE.elf 
   text    data     bss     dec     hex filename
  97164     320   11080  108564   1a814 ./obj/main/cleanflight_NAZE.elf
arm-none-eabi-objcopy -O ihex --set-start 0x8000000 obj/main/cleanflight_NAZE.elf obj/cleanflight_NAZE.hex
$ ls -la obj/cleanflight_NAZE.hex                                                                                                                                                 
-rw-rw-r-- 1 pim pim 274258 Jan 12 21:45 obj/cleanflight_NAZE.hex
```

You can use the Cleanflight-Configurator to flash the ```obj/cleanflight_NAZE.hex``` file.

## Updating and rebuilding

Navigate to the local cleanflight repository and use the following steps to pull the latest changes and rebuild your version of cleanflight:

```
cd src/cleanflight
git reset --hard
git pull
make clean TARGET=NAZE
make
```

## Notes
There are compiler issues with at least the Sparky target.

Credit goes to K.C. Budd for doing the long legwork that yielded this very short document.
