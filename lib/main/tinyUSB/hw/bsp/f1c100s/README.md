# BSP support for F1Cx00s boards

This folder contains necessary file and scripts to run TinyUSB examples on F1Cx00s boards.

Currently tested on:

- Lichee Pi Nano (F1C100s)
- [Widora Tiny200 v2 (also called MangoPi-R3c)](https://mangopi.org/tiny200)

## Flashing

There are two options to put your code into the MCU: `flash` and `exec`. Both modes require you to install [xfel](https://github.com/xboot/xfel) tool to your PATH. You must enter FEL mode before any operation can be done. To enter FEL mode, press BOOT button, then press RESET once, and release BOOT button. You will find VID/PID=1f3a:efe8 on your PC.

Exec: `make BOARD=f1c100s exec` will just upload the image to the DDR ram and execute it. It will not touch anything in the SPI flash.

Flash: `make BOARD=f1c100s flash` will write the image to SPI flash, and then reset the chip to execute it.

## TODO

* Add F1C100s to `#if CFG_TUSB_MCU == OPT_MCU_LPC43XX || CFG_TUSB_MCU == OPT_MCU_LPC18XX || CFG_TUSB_MCU == OPT_MCU_MIMXRT1XXX` high speed MCU check in examples (maybe we should extract the logic?)
