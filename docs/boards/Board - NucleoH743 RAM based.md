# Board - Nucleo H743 - RAM based image

This target for the Nucleo H743 is loaded entirely into the MCU RAM, thus facilitating quick turnaround development testing, without subjecting the flash storage to wear.

In order to flash / run it, the ST-Link tool available from ST Microelectronics has to be used.

## Board preparation

For the MCU to run the firmware from RAM after a reset, the boot address has to be changed:

- open the 'Option Bytes' dialog:

!['Option Bytes' menu entry](images/NUCLEOH743_RAMBASED_option_bytes.png)

- set the high word of BOOT\_ADD0 to `0x2401`:

![set boot address](images/NUCLEOH743_RAMBASED_boot_address.png)

- click 'Apply'.

## Installation

Since the firmware image is only stored in RAM, this has to be done after every power cycle of the board.

- open the 'Program' dialog:

!['Program' menu entry](images/NUCLEOH743_RAMBASED_program.png)

- click 'Browse', select the 'NUCLEOH743\_RAMBASED' hex image;
- make sure 'Reset after programming' is checked;
- click 'Start' to start programming:

![load / run the program](images/NUCLEOH743_RAMBASED_run.png)

- After programming has completed the firmware will be run.
