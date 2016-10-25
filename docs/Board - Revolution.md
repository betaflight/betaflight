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

* Sonar
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
| 3   | PPM Input | Enable `feature RX_PPM`     |
| 4   | UART6 TX | |
| 5   | UART6 RX    | |
| 6   | Current   | Enable `feature CURRENT_METER`.  Connect to the output of a current sensor, 0v-3.3v input |
| 7   | Battery Voltage sensor | Enable `feature VBAT`. Connect to main battery using a voltage divider, 0v-3.3v input |
| 8   | RSSI      | Enable `feature RSSI_ADC`.  Connect to the output of a PWM-RSSI conditioner, 0v-3.3v input |

#### RX_PARALLEL_PWM

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

### RC_Output connector

#### RX_PPM and RX_SERIAL

| Pin | Pin     | Function              | Notes |
| --- | ----    |----------             | ------|
| 1   | PB0     | MOTOR 1               |       |
| 2   | PB1     | MOTOR 2               |       |
| 3   | PA3     | MOTOR 3               |       |
| 4   | PA2     | MOTOR 4               |       |
| 5   | PA1     | MOTOR 5 / LED Strip   |       |
| 6   | PA8     | MOTOR 6               |       |

## Serial Ports

| Value | Identifier   | Board Markings | Notes                                     |
| ----- | ------------ | -------------- | ------------------------------------------|
| 1     | VCP          | USB PORT       |                                           |
| 2     | UART1        | MAIN PORT      | Connected to an MCU controllable inverter |
| 3     | UART3        | FLEX PORT      |                                           |
| 4     | UART6        | RC connector   | Pins 4 and 5 (Tx and Rx respectively)     |

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

## Buzzer

External buzzer should be wired to PB4 CPU pin using a transistor circuit

## Notes

* At this moment LED Strip is not functional on this target
* Motor out put 5 and 6 needs testing
* Servo output might not be working at the moment of writing this documentation
* I2C might not be working at the moment of writing this documentation
* LED Strip might not be working at the moment of writing this documentation
