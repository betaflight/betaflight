# Board - CC3D

The OpenPilot Copter Control 3D aka CC3D is a board more tuned to Acrobatic flying or GPS based
auto-piloting.  It only has one sensor, the MPU6000 SPI based Accelerometer/Gyro.
It also features a 16Mbit SPI based EEPROM chip.  It has 6 ports labeled as inputs (one pin each)
and 6 ports labeled as motor/servo outputs (3 pins each).

If issues are found with this board please report via the [github issue tracker](https://github.com/cleanflight/cleanflight/issues).

The board has a USB port directly connected to the processor.  Other boards like the Naze and Flip32
have an on-board USB to uart adapter which connect to the processor's serial port instead.

The board cannot currently be used for hexacopters/octocopters.

Tricopter & Airplane support is untested, please report success or failure if you try it. 

# Pinouts

The 8 pin RC_Input connector has the following pinouts when used in RX_PPM/RX_SERIAL mode

| Pin | Function  | Notes                            |
| --- | --------- | -------------------------------- |
| 1   | Ground    |                                  |
| 2   | +5V       |                                  |
| 3   | PPM Input | Enable `feature RX_PPM`          | 
| 4   | SoftSerial1 TX | Enable `feature SOFTSERIAL` |
| 5   | SoftSerial1 RX | Enable `feature SOFTSERIAL` |
| 6   | Current   | Enable `feature CURRENT_METER`.  Connect to the output of a current sensor, 0v-3.3v input |
| 7   | Battery Voltage sensor | Enable `feature VBAT`. Connect to main battery using a voltage divider, 0v-3.3v input |
| 8   | RSSI      | Enable `feature RSSI_ADC`.  Connect to the output of a PWM-RSSI conditioner, 0v-3.3v input |

The 6 pin RC_Output connector has the following pinouts when used in RX_PPM/RX_SERIAL mode

| Pin | Function  | Notes |
| --- | ----------| ------|
| 1   | MOTOR 1   |       |
| 2   | MOTOR 2   |       |
| 3   | MOTOR 3   |       |
| 4   | MOTOR 4   |       |
| 5   | LED Strip |       |
| 6   | Unused    |       |

The 8 pin RC_Input connector has the following pinouts when used in RX_PARALLEL_PWM mode

| Pin | Function | Notes |
| --- | ---------| ------|
| 1   | Ground   |       |
| 2   | +5V      |       |
| 3   | Unused   |       | 
| 4   | CH1      |       |
| 5   | CH2      |       |
| 6   | CH3      |       |
| 7   | CH4/Battery Voltage sensor      | CH4 if battery voltage sensor is disabled |
| 8   | CH5/CH4  | CH4 if battery voltage monitor is enabled|

The 6 pin RC_Output connector has the following pinouts when used in RX_PARALLEL_PWM mode

| Pin | Function | Notes |
| --- | ---------| ------|
| 1   | MOTOR 1  |       |
| 2   | MOTOR 2  |       |
| 3   | MOTOR 3  |       |
| 4   | MOTOR 4  |       |
| 5   | Unused   |       |
| 6   | Unused   |       |

# Serial Ports

| Value | Identifier   | Board Markings | Notes                                     |
| ----- | ------------ | -------------- | ------------------------------------------|
| 1     | VCP          | USB PORT       |                                           |
| 2     | USART1       | MAIN PORT      | Connected to an MCU controllable inverter |
| 3     | USART3       | FLEX PORT      |                                           |
| 4     | SoftSerial   | RC connector   | Pins 4 and 5 (Tx and Rx respectively)     |

The SoftSerial port is not available when RX_PARALLEL_PWM is used. The transmission data rate is limited to 19200 baud.

To connect the GUI to the flight controller you just need a USB cable to use the Virtual Com Port (VCP) or you can use UART1 (Main Port).

CLI access is only available via the VCP by default.

# Main Port

The main port has MSP support enabled on it by default.

The main port is connected to an inverter which is automatically enabled as required.  For example, if the main port is used for SBus Serial RX then an external inverter is not required.

# Flex Port

The flex port will be enabled in I2C mode unless USART3 is used.  You can connect external I2C sensors and displays to this port.

You cannot use USART3 and I2C at the same time.

## Flex port pinout

| Pin | Signal             | Notes                   |
| --- | ------------------ | ----------------------- |
| 1   | GND                |                         |
| 2   | VCC unregulated    |                         |
| 3   | I2C SCL / UART3 TX | 3.3v level              |
| 4   | I2C SDA / UART3 RX | 3.3v level (5v tolerant |


# Flashing

There are two primary ways to get Cleanflight onto a CC3D board.

* Single binary image mode - best mode if you don't want to use OpenPilot.
* OpenPilot Bootloader compatible image mode - best mode if you want to switch between OpenPilot and Cleanflight.

## Single binary image mode.

The entire flash ram on the target processor is flashed with a single image.

The image can be flashed by using a USB to UART adapter connected to the main port when the CC3D is put into the STM32 bootloader mode, achieved by powering on the CC3D with the SBL/3.3v pads bridged.  

## OpenPilot Bootloader compatible image mode.

The initial section of flash ram on the target process is flashed with a bootloader which can then run the code in the
remaining area of flash ram.

The OpenPilot bootloader code also allows the remaining section of flash to be reconfigured and re-flashed by the
OpenPilot Ground Station (GCS) via USB without requiring a USB to uart adapter.

