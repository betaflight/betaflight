# Mixer

Cleanflight supports a number of mixing configurations as well as custom mixing.  Mixer configurations determine how the servos and motors work together to control the aircraft.

## Configuration

To use a built-in mixing configuration, you can use the Chrome configuration GUI.  It includes images of the various mixer types to assist in making the proper connections.  See the Configuration section of the documentation for more information on the GUI.

You can also use the Command Line Interface (CLI) to set the mixer type:

1. Use `mixer list` to see a list of supported mixes
2. Select a mixer.  For example, to select TRI, use `mixer TRI`
3. You must use `save` to preserve your changes

## Supported Mixer Types

| Name          | Description               | Motors         | Servos           |
| ------------- | ------------------------- | -------------- | ---------------- |
| TRI           | Tricopter                 | M1-M3          | S1               |
| QUADP         | Quadcopter-Plus           | M1-M4          | None             |
| QUADX         | Quadcopter-X              | M1-M4          | None             |
| BI            | Bicopter (left/right)     | M1-M2          | S1, S2           |
| GIMBAL        | Gimbal control            | N/A            | S1, S2           |
| Y6            | Y6-copter                 | M1-M6          | None             |
| HEX6          | Hexacopter-Plus           | M1-M6          | None             |
| FLYING_WING   | Fixed wing; elevons       | M1             | S1, S2           |
| Y4            | Y4-copter                 | M1-M4          | None             |
| HEX6X         | Hexacopter-X              | M1-M6          | None             |
| OCTOX8        | Octocopter-X (over/under) | M1-M8          | None             |
| OCTOFLATP     | Octocopter-FlatPlus       | M1-M8          | None             |
| OCTOFLATX     | Octocopter-FlatX          | M1-M8          | None             |
| AIRPLANE      | Fixed wing; Ax2, R, E     | M1             | S1, S2, S3, S4   |
| HELI_120_CCPM |                           |                |                  |
| HELI_90_DEG   |                           |                |                  |
| VTAIL4        | Quadcopter with V-Tail    | M1-M4          | N/A              |
| HEX6H         | Hexacopter-H              | M1-M6          | None             |
| PPM_TO_SERVO  |                           |                |                  |
| DUALCOPTER    | Dualcopter                | M1-M2          | S1, S2           |
| SINGLECOPTER  | Conventional helicopter   | M1             | S1               |
| ATAIL4        | Quadcopter with A-Tail    | M1-M4          | N/A              |
| CUSTOM        | User-defined              |                |                  |


## Servo filtering

A low-pass filter can be enabled for the servos.  It may be useful for avoiding structural modes in the airframe, for example.  Currently it can only be configured via the CLI:

1. Use `set servo_lowpass_freq = nnn` to select the cutoff frequency.  Valid values range from 10 to 400.  This is a fraction of the loop frequency in 1/1000ths. For example, `40` means `0.040`.
2. Use `set servo_lowpass_enable = 1` to enable filtering.

The cutoff frequency can be determined by the following formula:
`Frequency = 1000 * servo_lowpass_freq / looptime`

For example, if `servo_lowpass_freq` is set to 40, and looptime is set to the default of 3500 us, the cutoff frequency will be 11.43 Hz.


