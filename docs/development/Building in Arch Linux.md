# Building in Arch Linux

To build BetaFlight, first the ARM toolchain has to be installed:

## Setup GNU ARM Toolchain

```bash
pacman -S arm-none-eabi-gcc arm-none-eabi-newlib arm-none-eabi-binutils
```

For running tests, the `libblocksruntime` package from the AUR also has to be installed:
``` bash
trizen -S libblocksruntime
```

At the time of writing, the repository version of GCC is 9.2.0, unlike 9.2.1 which BetaFlight expects.
If you get the following error, you can override the GCC version in `make/locak.mk` before proceeding:
```bash
$ make
make/tools.mk:301: *** **ERROR** your arm-none-eabi-gcc is '9.2.0', but '9.2.1' is expected. Override with 'GCC_REQUIRED_VERSION' in make/local.mk or run 'make arm_sdk_install' to install the right version automatically in the tools folder of this repo.  Stop.
$ echo "GCC_REQUIRED_VERSION = 9.2.0" >> make/local.mk
```

## Building

After the ARM toolchain is installed, you should be able to build from source.
```bash
cd src
git clone git@github.com:betaflight/betaflight.git
cd betaflight
make MATEKF411
```

You'll see a set of files being compiled, and finally linked, yielding both an ELF and then a HEX:
```
...
$ ls -l obj/
total 1516
-rwxr-xr-x 1 s-ol s-ol  428584 Jan 11 12:04 betaflight_4.2.0_MATEKF411.bin*
-rw-r--r-- 1 s-ol s-ol 1136991 Jan 11 12:04 betaflight_4.2.0_MATEKF411.hex
drwxr-xr-x 3 s-ol s-ol    4096 Jan 11 12:04 main/
```

You can use the Betaflight-Configurator to flash the `obj/betaflight_4.2.0_MATEKF411.hex` file.

## Updating and rebuilding

Navigate to the local betaflight repository and use the following steps to pull the latest changes and rebuild your version of betaflight:

```bash
cd src/betaflight
git reset --hard
git pull
make clean
make
```
