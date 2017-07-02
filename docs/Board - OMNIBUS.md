# Board - OMNIBUS F3

## Hardware Features

Refer to the product web page:
[OMNIBUS AIO F3 Flight Control](http://shop.myairbot.com/index.php/flight-control/cleanflight-baseflight/omnibusv11.html)

### Hardware Notes

There are few things to note on how things are connected on the board.

1. VBAT (J4)
This is a battery input to the board, and is also a input to voltage sensor.

2. J11 Power distribution
The RAM is user defined power rail, and all RAM through holes (J6, J7 and J11) are connected together. By connecting 5V or VBAT to RAM at J11, the RAM becomes 5V or VBAT power rail respectively. The VBAT on J11 can also be used to power the Board if necessary.

3. RSSI (J4)
The pin is labelled as RSSI, but it will not be used for RSSI input for a hardware configuration limitation. In this document, the "RSSI" is used to indicate the pin location, not the function.

4. UART1 in boot-loader/DFU mode
The UART1 is scanned during boot-loader/DFU mode, together with USB for possible interaction with a host PC. It is observed that devices that autonomously transmits some data, such as GPS, will prevent the MCU to talk to the USB. It is advised not to connect or disconnect such devices to/from UART1. UART2 is safe from this catch.

## iNav Specific Target Configuration

The first support for the OMNIBUS F3 appeared in BetaFlight.
The OMNIBUS target in iNav has different configuration from the BetaFlight support, to maximize the hardware resource utilization for navigation oriented use cases.

 [PIN CONFIGURATION PIC HERE]

### PWM Outputs

Six PWM outputs (PWM1~PWM6) are supported, but PWM5 and PWM6 is not available when UART3 is in use.
PWM7 and PWM8 are dedicated for I2C; in this document, they are used to indicate the pin location, not the function.

If servos are used on a multirotor mixer (i.e. Tricopter) PWM1 is remapped to servo and motor 1 is moved to PWM2 etc.

Note: Tested only for QUAD-X configuration.

### Hardware UART Ports

PPM/SBUS jumper for J8 is assumed to be configured for PPM (SBUS=R18 removed). With newer boards (the 1.1 Version) you don't have to swap an smd resistor to use SBUS anymore. It just works out of the box.

| UART  | Location | Note              |
|-------|----------|-------------------|
| UART1 |J13       |                   |
| UART2 |J12       |                   |
| UART3 |J22       | PWM5=TX3,PWM6=RX3 |

All UARTs are Serial RX capable.

### I2C

I2C is available on J22 PWM7 and PWM8

|signal | Location   | Alt. Location |
|-------|------------|---------------|
|SCL    | J22 (PWM7) | J3 (SCL)      |
|SDA    | J22 (PWM8) | J3 (SDA)      |

### RANGEFINDER

HC-SR04 rangefinder is supported when NOT using PPM.

|signal | Location   |
|-------|------------|
|TRIG   | J8 (PPM)   |
|ECHO   | J4 (RSSI)  |

5V rangefinder can be connected directly without inline resistors.

### OSD

Integrated OSD is supported.

### RSSI Sensor Input

The RSSI sensor adc is not supported due to the hardware configuration limitation.

## Usage in a Fixed Wing
Due to the way INAV handles PWM outputs the first 2 PWM outputs are reserved for the motor outputs. When using SBUS on UART3 as recommended this leaves only 2 additional outputs for the servos, as output 5 and 6 are blocked by UART3 serial for SBUS and 7 and 8 are used for I2C.

You can free PWM outputs 5 and 6 by simply connecting SBUS up to UART1. For FrSky there is no hardware inverter needed as the F3 chip UARTs can handle this without additional hardware. Just make sure that `sbus_inversion = ON` is set. However, you will not be able to use UART3, e.G. for telemetry.

This allows to control a standard airplane with rudder, ailerons and elevator. If you use flaps or a servo gimbal, you can bypass the FC by connecting it up to the receiver directly. 

The popular x4rsb for example outputs channels 1,2,3 as PWM in addition to SBUS. Since they are shared with the channels on SBUS you need to change the channel mapping to `123AETR4`and ignore the first 3 AUX channels within Cleanflight.
