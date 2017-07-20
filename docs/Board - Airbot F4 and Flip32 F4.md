# Board -  Airbot F4 / Flip32 F4 / Generic F4 Flight Controller from Banggood and Aliexpress

![Airbot F4](https://quadmeup.com/wp-content/uploads/2016/10/Flip32-F4-Flight-Controller-pinout.jpg)

> This target supports both MPU6000 and MPU6500 versions of this board

## Features

* STM32F405 CPU
* Integrated Accelerometer/Gyro MPU6000 or MPU6500 via SPI bus
* 6 motor outputs
* 4 UART ports (UART1, UART3, UART6, Softserial1)
* External I2C bus, pins shared with UART3, can not be used simultaneously
* Only UART1 is equipped with inverter
* Onboard 128Mbit (16MB) flash
* BLHeli Passthrough
* Integrated voltage stabilizer
* Buzzer connector

## **NOT** supported

* HC-SR04 Rangefinder
* ServoTilt
* Channel Forwarding

## Radio Receivers

This board does not support Parallel PWM receiver connection. Only SerialRX, PPM and MSP receivers are supported.

SerialRX and PPM receivers should be connected to dedicated _PPM SBUS_ connector above _Motor 1_. MSP receivers should be connected to one of UARTs configured as MSP.

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

## Integrated voltage stabilizer

It is integrated with voltage monitoring and always powered when VBAT is connected to battery.
Because this is **Linear Stabilizer**, it has a tendency to overheat, especially on 4S. Because of that,
avoid powering too many devices directly to 5V pins on the board. RX receiver is (and board itself) is rather all
it can do without overeating (150mA on 4S gives 1.5W of waste heat!). OSD, LED Strip and other devices should powered from separate BEC if voltage monitoring is to be enabled.

## LED Strip

LED strip is enabled on Motor 5 pin (PA1)

## SoftwareSerial

This board allows for single **SoftwareSerial** port on small soldering pads located next to UART3 pins. 

| Pad   | SoftwareSerial Role   |
| ----  | ----                  |
| CH5   | RX                    |
| CH6   | TX                    |

## FrSky SmartPort using SoftwareSerial

SmartPort telemetry is possible using SoftwareSerial. RX and TX lines have to be bridged using
1kOhm resistor (confirmed working with 100Ohm, 1kOhm and 10kOhm)

```
SmartPort ---> RX (CH5 pad) ---> 1kOhm resistor ---> TX (CH6 pad)
```

* Telemetry has to be inverted with `set telemetry_inversion = ON`
* Port should be configured for _57600bps_
* Tested with FrSky X4R