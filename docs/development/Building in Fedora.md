# Building in Fedora

Assuming you already have wget and git available, you should be able to build Cleanflight on a fresh install of Fedora with the following commands (tested on F18, F20 and Ubuntu 14.04):

```
wget http://distribute.atmel.no/tools/opensource/Atmel-ARM-GNU-Toolchain/4.8.4/arm-gnu-toolchain-4.8.4.371-linux.any.x86_64.tar.gz

tar xf arm-gnu-toolchain-4.8.4.371-linux.any.x86_64.tar.gz
export PATH=$PATH:$PWD/arm-none-eabi/bin

git clone https://github.com/cleanflight/cleanflight.git
cd cleanflight
TARGET=NAZE make
```

This will create cleanflight_NAZE.hex in the obj folder.
