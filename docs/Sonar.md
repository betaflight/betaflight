# Sonar

A sonar sensor can be used to measure altitude for use with BARO and RANGEFINDER altitude
hold modes.

The sonar sensor is used instead of the pressure sensor (barometer) at low altitudes.
The sonar sensor is only used when the aircraft inclination angle (attitude) is small.

## Hardware

Currently the only supported sensor is the HCSR04 sensor.

## Connections

### Naze/Flip32+

| Mode                            | Trigger       | Echo          | Inline 1k resistors |
| ------------------------------- | ------------- | ------------- | ------------------- |
| Parallel PWM/ADC current sensor | PB8 / Motor 5 | PB9 / Motor 6 | NO (5v tolerant)    |
| PPM/Serial RX                   | PB0 / RC7     | PB1 / RC8     | YES (3.3v input)    |

#### Constraints

Current meter cannot be used in conjunction with Parallel PWM and Sonar.

### Olimexino

| Trigger       | Echo          | Inline 1k resistors |
| ------------- | ------------- | ------------------- |
| PB0 / RC7     | PB1 / RC8     | YES (3.3v input)    |

#### Constraints

Current meter cannot be used in conjunction with Sonar.

### CC3D

| Trigger       | Echo          | Inline 1k resistors |
| ------------- | ------------- | ------------------- |
| PB5           | PB0           | YES (3.3v input)    |

#### Constraints

Sonar cannot be used in conjuction with SoftSerial or Parallel PWM.
