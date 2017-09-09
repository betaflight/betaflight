# Building in Ubuntu

Building for Ubuntu platform is remarkably easy. The only trick to understand is that the Ubuntu toolchain,
which they are downstreaming from Debian, is not compatible with INAV. We suggest that you take an
alternative PPA from Terry Guo, found here:
https://launchpad.net/~terry.guo/+archive/ubuntu/gcc-arm-embedded

This PPA has several compiler versions and platforms available. For many hardware platforms (notably Naze)
the 4.9.3 compiler will work fine. For some, older compiler 4.8 (notably Sparky) is more appropriate. We
suggest you build with 4.9.3 first, and try to see if you can connect to the CLI or run the Configurator.
If you cannot, please see the section below for further hints on what you might do.

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

For Linux Mint 18 (Ubuntu 16, 2016-09-11)
```
sudo apt install git
sudo apt install gcc
sudo apt install gcc-arm-none-eabi


sudo apt-get install libnewlib-arm-none-eabi

cd src
git clone https://github.com/iNavFlight/inav.git
cd inav
make TARGET=NAZE
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

## Install Ruby

Install the Ruby package for your distribution. On Debian based distributions, you should
install the ruby package
```
sudo apt-get install ruby
```

## Installing Ruby 2.4

Since in some cases apt-get will install ruby 1.9 which does not work, you can force ruby 2.4 by

```
sudo apt-get remove ruby
sudo apt-add-repository ppa:brightbox/ruby-ng
sudo apt-get update
apt-get install ruby2.4 ruby2.4-dev
```

## Building on Ubuntu

After the ARM toolchain from Terry is installed, you should be able to build from source.
```
cd src
git clone git@github.com:iNavFlight/inav.git
cd inav
make TARGET=NAZE
```

You'll see a set of files being compiled, and finally linked, yielding both an ELF and then a HEX:
```
...
arm-none-eabi-size ./obj/main/inav_NAZE.elf
   text    data     bss     dec     hex filename
  97164     320   11080  108564   1a814 ./obj/main/inav_NAZE.elf
arm-none-eabi-objcopy -O ihex --set-start 0x8000000 obj/main/inav_NAZE.elf obj/inav_NAZE.hex
$ ls -la obj/inav_NAZE.hex                                                                                                                                                 
-rw-rw-r-- 1 pim pim 274258 Jan 12 21:45 obj/inav_NAZE.hex
```

You can use the INAV-Configurator to flash the ```obj/inav_NAZE.hex``` file.

## Bricked/Bad build?

Users have reported that the 4.9.3 compiler for ARM produces bad builds, for example on the Sparky hardware platform.
It is very likely that using an older compiler would be fine -- Terry happens to have also a 4.8 2014q2 build in his
PPA - to install this, you can fetch the `.deb` directly:
http://ppa.launchpad.net/terry.guo/gcc-arm-embedded/ubuntu/pool/main/g/gcc-arm-none-eabi/

and use `dpkg` to install:
```
sudo dpkg -i gcc-arm-none-eabi_4-8-2014q2-0saucy9_amd64.deb
```

Make sure to remove `obj/` and `make clean`, before building again.

## Updating and rebuilding

Navigate to the local INAV repository and use the following steps to pull the latest changes and rebuild your version of INAV:

```
cd src/inav
git reset --hard
git pull
make clean TARGET=NAZE
make
```

Credit goes to K.C. Budd, AKfreak for testing, and pulsar for doing the long legwork that yielded this very short document.
