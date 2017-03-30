# Board - BeeRotor F4

![BeeRotor F4 front](https://raw.githubusercontent.com/wiki/betaflight/betaflight/images/boards/beerotorf4/beerotorf4_front.jpg)

![BeeRotor F4 back](https://raw.githubusercontent.com/wiki/betaflight/betaflight/images/boards/beerotorf4/beerotorf4_back.jpg)

## Features

* STM32F405 CPU
* Integrated Accelerometer/Gyro MPU6050 via SPI bus
* Barometer BMP280
* 8 motor outputs
* 3 UART ports (UART1, UART2, UART3)
* External SPI bus
* UART2 and UART3 are equipped with inverter
* SD Card slot for logging
* LED connector
* Buzzer connector
* Integrated OSD

## Currently **NOT** supported

* inverter on UART3
* IR transmitter output

## Radio Receivers

This board does not support Parallel PWM receiver connection. Only SerialRX, PPM and MSP receivers are supported.

SerialRX and PPM receivers should be connected to dedicated _PPM SBUS_ pin on the _RX_ connector. MSP receivers should be connected to one of UARTs configured as MSP.

## USB

This board uses STM32 VCP and _not_ utilizes UART when USB is connected. STM32 VCP drivers might be required!

Flashing requires DFU mode and STM32 DFU drivers. Use [Zadig](http://zadig.akeo.ie) tool to install WinUSB driver on Windows.

## Buzzer / Beeper

5V piezo buzzer should be connected directly to dedicated pins _BUZ +_ and _BUZ -_. No additional hardware is required.

## RSSI monitoring

* Use the dedicated _RSSI_ pin on the _RX_ connector.
* 3.3V tolerant, do not supply 5V

## Current monitoring

* Use the dedicated _SI_ pin on the _PDB_ connector, or the _SI_ pad on the board.
* 3.3V tolerant, do not supply 5V

## Voltage monitoring

* Use the dedicated _SI_ pin on the _PDB_ connector, or the _SV_ pad on the board.
* 3.3V tolerant, do not supply 5V
* _SV_ pin on the _PDB_ connector does not have a voltage divider. It requires a voltage divider on the PDB, or the board will be destroyed! (The _SV_ pad on the board has a voltage divider, use this if your board doesn't.)

## LED Strip

* Use the dedicated _LED_ pin on the LED / buzzer connector.
