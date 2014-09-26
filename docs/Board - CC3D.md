# Board - CC3D

The OpenPilot Copter Control 3D aka CC3D is a board more tuned to Acrobatic flying or GPS based
auto-piloting.  It only has one sensor, the MPU6000 SPI based Accelerometer/Gyro.
It also features a 16mbit SPI based EEPROM chip.  It has 6 ports labelled as inputs (one pin each)
and 6 ports labelled as motor/servo outputs (3 pins each).

If issues are found with this board please report via the github issue tracker.

The board has a USB port directly connected to the processor.  Other boards like the Naze and Flip32
have an on-board USB to uart adapter which connect to the processor's serial port instead.

Currently there is no support for virtual com port functionality on the CC3D which means that cleanflight
does not use the USB socket at all.

# Serial Ports

| Value | Identifier   | Board Markings | Notes                                    |
| ----- | ------------ | -------------- | -----------------------------------------|
| 1     | USART1       | MAIN PORT      | Has a hardware inverter for SBUS         |
| 2     | USART3       | FLEX PORT      |                                          |

Software serial is not supported yet due to timer and pin configuration mappings.

To connect the GUI to the flight controller you need additional hardware attached to the USART1 serial port (by default).

# Flashing

There are two primary ways to get Cleanflight onto a CC3D board.

* Single binary image mode - best mode if you don't want to use OpenPilot.
* OpenPilot Bootloader compatible image mode - best mode if you want to switch between OpenPilot and Cleanflight.

## Single binary image mode.

The entire flash ram on the target processor is flashed with a single image.

## OpenPilot Bootloader compatible image mode.

The initial section of flash ram on the target process is flashed with a bootloader which can then run the code in the
remaining area of flash ram.

The OpenPilot bootloader code also allows the remaining section of flash to be reconfigured and re-flashed by the
OpenPilot Ground Station (GCS) via USB without requiring a USB to uart adapter.

In this mode a USB to uart adapter is still required to connect to via the GUI or CLI.