# Board - OpenPilot Revolution also known as CC3D Revo

## Features

* STM32F405 CPU
* Integrated Accelerometer/Gyro MPU6000 via SPI bus
* Integrated Magnetometer HMC5883
* Integrated Barometer MS5611
* Integrated 433MHz OPlink Modem -> Not supported
* 6 motor outputs
* 3 UART ports (UART1, UART3, UART6)
* External I2C bus, pins shared with UART3, can not be used simultaneously
* Only UART1 is equipped with inverter
* Onboard 128Mbit (16MB) flash
* BLHeli Passthrough

## **NOT** supported

* HC-SR04 Rangefinder
* SoftwareSerial
* ServoTilt
* Channel Forwarding

## USB

This board uses STM32 VCP and _not_ utilizes UART when USB is connected. STM32 VCP drivers might be required!

Flashing requires DFU mode and STM32 DFU drivers. Use [Zadig](http://zadig.akeo.ie) tool to install WinUSB driver on Windows.

## Pinout

Following section is ported from [RaceFlight](https://github.com/rs2k/raceflight/blob/master/docs/Board%20-%20Revo.md) with edits

### RC_Input connector

#### RX_PPM and RX_SERIAL

| Pin | Function  | Notes                            |
| --- | --------- | -------------------------------- |
| 1   | Ground    |                                  |
| 2   | +5V       |                                  |
| 3   |           |                                  |
| 4   |           |                                  |
| 5   | PPM Input | Enable `feature RX_PPM`          |
| 6   |           |                                  |
| 7   | UART6 TX  | |
| 8   | UART6 RX  | |
| 9   |           |                                  |
| 10  |           |                                  |

#### RX_PARALLEL_PWM

| Pin | Function | Notes |
| --- | ---------| ------|
| 1   | Ground   |       |
| 2   | +5V      |       |
| 3   | Unused   |       |
| 4   | Unused   |       |
| 5   | CH1      |       |
| 6   | CH2      |       |
| 7   | CH3      |       |
| 8   | CH4      |       |
| 9   | CH5      |       |
| 10  | CH6      |       |

### RC_Output connector

#### RX_PPM and RX_SERIAL

| Pin | Pin     | Function              | Notes |
| --- | ----    |----------             | ------|
| 1   | PB0     | MOTOR 1               |       |
| 2   | PB1     | MOTOR 2               |       |
| 3   | PA3     | MOTOR 3               |       |
| 4   | PA2     | MOTOR 4               |       |
| 5   | PA1     | MOTOR 5 / LED Strip   |       |
| 6   | PA0     | MOTOR 6 / RSSI_ADC    |       |

## Serial Ports

| Value | Identifier   | Board Markings | Notes                                     |
| ----- | ------------ | -------------- | ------------------------------------------|
| 1     | VCP          | USB PORT       |                                           |
| 2     | UART1        | MAIN PORT      | Connected to an MCU controllable inverter |
| 3     | UART3        | FLEX PORT      |                                           |
| 4     | UART6        | RC connector   |                                           |

The UART6 port is not available when RX_PARALLEL_PWM is used.

### Main Port

The main port is connected to an inverter which is automatically enabled as required.  For example, if the main port is used for SBus Serial RX then an external inverter is not required.

| Pin | Signal             | Notes                   |
| --- | ------------------ | ----------------------- |
| 1   | GND                |                         |
| 2   | VCC unregulated    |                         |
| 3   | UART1 TX           | 3.3v level              |
| 4   | UART1 RX           | 3.3v level (5v tolerant)|

### Flex Port

The flex port will be enabled in I2C mode unless UART3 is used.  You can connect external I2C sensors and displays to this port.

You cannot use USART3 and I2C at the same time.

| Pin | Signal             | Notes                    |
| --- | ------------------ | -----------------------  |
| 1   | GND                |                          |
| 2   | VCC unregulated    |                          |
| 3   | I2C SCL / UART3 TX | 3.3v level               |
| 4   | I2C SDA / UART3 RX | 3.3v level (5v tolerant) |

### Pwr sensor connector

| Pin | Signal             | Notes                    |
| --- | ------------------ | -----------------------  |
| 1   | GND                |                          |
| 2   | VCC unregulated    |                          |
| 3   | Current sensor     | 3.3v max input           |
| 4   | Voltage sensor     | 3.3v max input |

## Buzzer

External buzzer should be wired to PB4 CPU pin using a transistor circuit

## Notes

* At this moment LED Strip is not functional on this target
* Motor out put 5 and 6 needs testing
* Servo output might not be working at the moment of writing this documentation
* I2C might not be working at the moment of writing this documentation
* LED Strip might not be working at the moment of writing this documentation
