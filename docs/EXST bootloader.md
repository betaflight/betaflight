# EXST bootloader

The EXST (External Storage) system requires that the targets' CPU has a small program, called a bootloader, that provides functionality to load firmware from an external storage into the CPU's RAM so that the firmware can be executed.

Example external storage systems include:
* External Flash using Firmware Partition
* File on an SD card.

EXST targets also have the requirement of providing a mechansim to load and save configuration ('eeprom').  options for this include:

* CPU Embedded flash page.
* External Flash using Config Partition
* File on an SD card.

Since external storage systems can become corrupted, and communication errors can occur, Bootloaders should verify correct loading of the firmware from an external storage system, or after transferring via a debugger.  An hash is provided for this purpose.

Bootloaders are not limited to just one storage system. 

Bootloaders may offer additional functionality, such as:
* a means to update the firmware on an external storage medium (e.g. via DFU)
* update one external storage system from another (i.e. update external flash from file on SD card)
* backup, select, swap or erase firmware
* backup, select, swap or erase configuration.

Users should consult the manual that came with their EXST bootloader-equipped flight controller for details on how to use any such features.

The build system provides a way to build files suitable for transferring onto an external storage medium via the standard 'bin'/'hex' make goals.

```
make TARGET=<targetname>
```

This results in the usual `*.bin` and `*.hex` files and also a `*_EXST.elf` file which have been patched to contain information required for bootloader operation.

The `.bin` file is a binary file which should be transferred to the storage medium and is suitable for distribution.
The `.hex` file is a hex file which can be used by tools for uploading to a bootloader and is suitable for distribution.

For developers there is a `_EXST.elf` file which is a standard `.elf` file that has had one section replaced, this file is suitable for uploading to targets using a debugger such as GDB.

For details on the memory sections used refer to the linker files.

## EXST format

The format for EXST targets is as follows:

* Firmware.
* Bootloader block.

The bootloader block is 64 bytes.

For example a 448K EXST `.bin` file is comprised as follows:

| Start Address | End address | Usage            |
| ------------- | ----------- | ---------------- |
| 0x00000       | 0x6FFBF     | Firmware section |
| 0x6FFC0       | 0x6FFFF     | Bootloader block |


## EXST bootloader block

The bootloader block is comprised as follows:


| Start Address | End address | Usage            |
| ------------- | ----------- | ---------------- |
| 0x00          | 0x00        | Block Format     |
| 0x01          | 0x3F        | Block Content    |

Block Formats

| Value | Meaning           |
| ----- | ----------------- |
| 0x00  | Block format 0x00 |

### Block Format 0x00 Content

Note: addresses relative to start of bootloader block

| Start Address | End address | Usage            |
| ------------- | ----------- | ---------------- |
| 0x01          | 0x01        | Checksum/Hash method |
| 0x02          | 0x2F        | Reserved, fill with 0x00 |
| 0x30          | 0x3F        | Checksum value, pad with trailing 0x00 |

Checksum Hash methods

| Value | Meaning          |
| ----- | ---------------- |
| 0x00  | No checksum.     |
| 0x01  | MD5.             |

Checksum locations:

| Start Address | End address | Usage            |
| ------------- | ----------- | ---------------- |
| 0x30          | 0x3F        | MD5 hash of firmware section |

The bootloader should make no attempt to use any reserved area otherwise this prevents it's future use by the firmware.

The bootloader should ensure this block is in RAM when the firmware is loaded.  i.e. copy the entire EXST image.

As the reserved area is under control of the build system, not the bootloader, additional information can stored there for use by the firmware or future bootloaders.  Extreme care must be taken not to break the bootloaders ability to load firmware.  Consultation with the developers for any changes to this area is required.

### Example possible future enhancements for the EXST bootloader block

* Hardware layout identification - to allow the firmware to identify the hardware, such that IO pins and peripherals can be reserved/initialised.
* Alternative hashes/CRCs.
