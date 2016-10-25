# Board - Flip32 F4 and Airbot F4

## Features

* STM32F405 CPU
* Integrated Accelerometer/Gyro MPU6000 via SPI bus
* 6 motor outputs
* 3 UART ports (UART1, UART3, UART6)
* External I2C bus, pins shared with UART3, can not be used simultaneously
* Only UART1 is equipped with inverter
* Onboard 128Mbit (16MB) flash
* BLHeli Passthrough
* Integrated voltage stabilizer
* Buzzer connector

## **NOT** supported

* Sonar
* SoftwareSerial
* ServoTilt
* Channel Forwarding

## Motors

| Motor     | pin   |   Shared with |
| ----      | ----  |   ----        |
| 1         | PB0   |               |
| 2         | PB1   |               |
| 3         | PA3   |               |
| 4         | PA2   |               |
| 5         | PA1   | LED Strip     |
| 6         | PA8   |               |

## USB

This board uses STM32 VCP and _not_ utilizes UART when USB is connected. STM32 VCP drivers might be required!

Flashing requires DFU mode and STM32 DFU drivers. Use [Zadig](http://zadig.akeo.ie) tool to install WinUSB driver on Windows.

## Buzzer / Beeper

5V piezo buzzer should be connected directly to dedicated pins _BUZ +_ and _BUZ -_. No additional hardware is required.

## RSSI ADC

* Connected to pin PA0
* 3.3V tolerant, do not supply 5V

## Current Meter ADC

* Connected to pin PC1
* 3.3V tolerant, do not supply 5V

## Voltage monitoring

* Connected to pin PC2
* Connected to VBAT pins (both are the same) and integrated Voltage Stabilizer (LM7805M)

### Integrated voltage stabilizer

It is integrated with voltage monitoring and always powered when VBAT is connected to battery.
Because this is **Linear Stabilizer**, it has a tendency to overheat, especially on 4S. Because of that,
avoid powering too many devices directly to 5V pins on the board. RX receiver is (and board itself) is rather all
it can do without overeating (150mA on 4S gives 1.5W of waste heat!). OSD, LED Strip and other devices should powered from separate BEC if voltage monitoring is to be enabled.

### LED Strip

Right now, LED strip is not functioning correctly on this target. It is a known bug. When bug will be fixed, LED Strip should be connected to **MOTOR 5** output, not dedicated "LED" connector.
