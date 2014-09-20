
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

## SBUS

12 channels via serial currently supported.

## SUMD

16 channels via serial currently supported.

## SUMH

8 channels via serial currently supported.

 
### Configuration

See the Configuration document some some RX configuration examples.

For Serial RX enable `RX_SERIAL` and set the `serialrx_provider` cli setting as follows.

| Serial RX Provider | Value |
| ------------------ | ----- |
| SPEKTRUM1024       | 0     |
| SPEKTRUM2048       | 1     |
| SBUS               | 2     |
| SUMD               | 3     |
| SUMH               | 4     |

#### PPM/PWM input filtering.

Hardware input filtering can be enabled if you are experiencing interference on the signal sent via your PWM/PPM RX.

Use the `input_filtering_mode` cli setting to select a mode.

| Value | Meaning   |
| ----- | --------- |
| 0     | Disabled  |
| 1     | Enabled   |

