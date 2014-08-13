# Sonar

A sonar sensor can be used to measure altitude for use with altitude hold modes.

The current sensor takes over from the pressure sensor at low altitudes, but only when
the angle of the aircraft is small.
 
## Hardware

Currently the only supported sensor is the HCSR04 sensor.

## Connections

### Naze/Flip32+

| Mode          | Trigger       | Echo          | Inline 1k resistors |
| ------------- | ------------- | ------------- | ------------------- |
| Parallel PWM  | PB8 / Motor 5 | PB9 / Motor 6 | NO (5v tolerant)    |
| PPM/Serial RX | PB0 / RC7     | PB1 / RC8     | YES (3.3v input)    |

Current meter cannot be used in conjunction with Parallel PWM and Sonar.

### Olimexino

| Trigger       | Echo          | Inline 1k resistors |
| ------------- | ------------- | ------------------- |
| PB0 / RC7     | PB1 / RC8     | YES (3.3v input)    |

Current meter cannot be used in conjunction sonar.
