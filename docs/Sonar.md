# Sonar

A sonar sensor can be used to measure altitude for use with BARO and SONAR altitude
hold modes.

The sonar sensor is used instead of the pressure sensor (barometer) at low altitudes (less than about 3.5 meters above the ground).
The sonar sensor is only used when the aircraft inclination angle (attitude) is small (less than 22.5 degrees).

## Hardware

Currently the main supported sensor is the HCSR04 sensor.
The Parallax 28015 single-wire sonar can also be used by connecting 1k resistors to the Trigger and Echo pins, and the other end of the resistors shorted together and to the Sonar module.

```
          1k
TRIGGER--/\/\--\
                \_______ 28015 SONAR
          1k    /
ECHO-----/\/\--/
```


## Connections

### Naze/Flip32+

| Mode                            | Trigger       | Echo          | Inline 1k resistors |
| ------------------------------- | ------------- | ------------- | ------------------- |
| Parallel PWM/ADC current sensor | PB8 / Motor 5 | PB9 / Motor 6 | NO (5v tolerant)    |
| PPM/Serial RX                   | PB0 / RC7     | PB1 / RC8     | YES (3.3v input)    |

#### Constraints

Current meter cannot be used in conjunction with Parallel PWM and Sonar.

### CC3D

| Trigger       | Echo          | Inline 1k resistors |
| ------------- | ------------- | ------------------- |
| PB5 / RC4     | PB0 / RC5     | YES (3.3v input)    |

#### Constraints

Sonar cannot be used in conjuction with SoftSerial or Parallel PWM.

### SPRacingF3

| Trigger       | Echo          | Inline 1k resistors |
| ------------- | ------------- | ------------------- |
| PB0 / RC7     | PB1 / RC8     | YES (3.3v input)    |

#### Constraints

Sonar cannot be used in conjuction with SoftSerial2 or Parallel PWM.

