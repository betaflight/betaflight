# Receivers (RX)

## Parallel PWM

8 channel support, 1 channel per input pin.  On some platforms using parallel input will disable the use of serial ports
and softserial making it hard to use telemetry or gps features.

## PPM (PPM SUM)

12 channels via a single input pin, not as accurate or jitter free as methods that use serial communications.

## MultiWii serial protocol (MSP)

Allows you to use MSP commands as the RC input.  Only 8 channel support to maintain compatibility with MSP.

## Spektrum

12 channels via serial currently supported.

## SUMD

8 channels supported currently, 12 or more is technically possible.

## SBUS

12 channels via serial supported currently.

 
### Configuration

See the Configuration document some some RX configuration examples.

#### PPM/PWM input filtering.

Hardware input filtering can be enabled if you are experiencing interference on the signal sent via your PWM/PPM RX.

Use the `input_filtering_mode` cli command to select a mode.

| Value | Meaning   |
| ----- | --------- |
| 0     | Disabled  |
| 1     | Enabled   |

