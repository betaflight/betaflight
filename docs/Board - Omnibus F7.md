# Board -  Omnibus F7

![Omnibus F7](https://quadmeup.com/wp-content/uploads/2017/07/Omnibus-F7-flight-controller-top-view.jpg)

## Features

* STM32F745 CPU
* Integrated Accelerometer/Gyro MPU6000 and MPU6500 via SPI bus. INAV uses only MPU6000
* 4 motor outputs
* 4 UART ports (UART1, UART2 (RX only), UART3, UART6)
* External I2C bus, pins shared with UART3, can not be used simultaneously
* BLHeli Passthrough
* Integrated BEC
* Buzzer connector
* Integrated OSD

## Known issues

* LED Strips are not working

## Radio Receivers

This board does not support Parallel PWM receiver connection. Only SerialRX, PPM and MSP receivers are supported.

SerialRX and PPM receivers should be connected to dedicated _PPM SBUS_ connector. For SerialRX use UART2

## Motors

This board has only 4 motor/servo outputs. Can be used in airplanes only on Flywing Wings

## Buzzer / Beeper

5V piezo buzzer should be connected directly to dedicated pins _BUZ +_ and _BUZ -_. No additional hardware is required.

## RSSI ADC

* Connected to pin PA0
* 3.3V tolerant, do not supply 5V

## Current Meter ADC

* Connected to pin PC1
* 3.3V tolerant, do not supply 5V
