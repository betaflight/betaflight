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

A low-pass filter can be enabled for the servos.  It may be useful for avoiding structural modes in the airframe, for example.  

### Configuration 

Currently it can only be configured via the CLI:

1. Use `set servo_lowpass_freq = nnn` to select the cutoff frequency.  Valid values range from 10 to 400.  This is a fraction of the loop frequency in 1/1000ths. For example, `40` means `0.040`.
2. Use `set servo_lowpass_enable = 1` to enable filtering.

The cutoff frequency can be determined by the following formula:
`Frequency = 1000 * servo_lowpass_freq / looptime`

For example, if `servo_lowpass_freq` is set to 40, and looptime is set to the default of 3500 us, the cutoff frequency will be 11.43 Hz.

### Tuning

One method for tuning the filter cutoff is as follows:

1. Ensure your vehicle can move at least somewhat freely in the troublesome axis.  For example, if you are having yaw oscillations on a tricopter, ensure that the copter is supported in a way that allows it to rotate left and right to at least some degree.  Suspension near the CG is ideal.  Alternatively, you can just fly the vehicle and trigger the problematic condition you are trying to eliminate, although tuning will be more tedious.

2. Tap the vehicle at its end in the axis under evaluation.  Directly commanding the servo in question to move may also be used.  In the tricopter example, tap the end of the tail boom from the side, or command a yaw using your transmitter.

3. If your vehicle oscillates for several seconds or even continues oscillating indefinitely, then the filter cutoff frequency should be reduced. Reduce the value of `servo_lowpass_freq` by half its current value and repeat the previous step.

4. If the oscillations are dampened within roughly a second or are no longer present, then you are done.  Be sure to run `save`.

## Custom Motor Mixing 

Custom motor mixing allows for completely customized motor configurations. Each motor must be defined with a custom mixing table for that motor. The mix must reflect how close each motor is with reference to the CG (Center of Gravity) of the flight controller. A motor closer to the CG of the flight controller will need to travel less distance than a motor further away.  

Steps to configure custom mixer in the CLI:

1. Use `mixer custom` to enable the custom mixing.
2. Use `cmix reset` to erase the any existing custom mixing. 
3. Issue a cmix statement for each motor. 

The cmix statement has the following syntax: `cmix n THROTTLE ROLL PITCH YAW` 

| Mixing table parameter | Definition | 
| ---------------------- | ---------- |
| n	| Motor ordering number |
| THROTTLE	| All motors that are used in this configuration are set to 1.0. Unused set to 0.0. |
| ROLL	| Indicates how much roll authority this motor imparts to the roll of the flight controller. Accepts values nominally from 1.0 to -1.0. |
| PITCH	| Indicates the pitch authority this motor has over the flight controller. Also accepts values nominally from 1.0 to -1.0. |
| YAW	| Indicates the direction of the motor rotation in relationship with the flight controller. 1.0 = CCW -1.0 = CW. |

### Example 1: A KK2.0 wired motor setup 
Here's an example of a X configuration quad, but the motors are still wired using the KK board motor numbering scheme. 

```
KK2.0 Motor Layout

  1CW      2CCW
     \    /
       KK
     /    \
  4CCW     3CW
```

1. Use `mixer custom`
2. Use `cmix reset`
3. Use `cmix 1 1.0,  1.0, -1.0, -1.0` for the Front Left motor. It tells the flight controller the #1 motor is used, provides positive roll, provides negative pitch and is turning CW.  
4. Use `cmix 2 1.0, -1.0, -1.0,  1.0` for the Front Right motor. It still provides a negative pitch authority, but unlike the front left, it provides negative roll authority and turns CCW.
5. Use `cmix 3 1.0, -1.0,  1.0, -1.0` for the Rear Right motor. It has negative roll, provides positive pitch when the speed is increased and turns CW.
6. Use `cmix 4 1.0,  1.0,  1.0,  1.0` for the Rear Left motor. Increasing motor speed imparts positive roll, positive pitch and turns CCW.

### Example 2: A HEX-U Copter 

Here is an example of a U-shaped hex; probably good for herding giraffes in the Sahara. Because the 1 and 6 motors are closer to the roll axis, they impart much more force than the motors mounted twice as far from the FC CG. The effect they have on pitch is the same as the forward motors because they are the same distance from the FC CG. The 2 and 5 motors do not contribute anything to pitch because speeding them up and slowing them down has no effect on the forward/back pitch of the FC. 

``` 
HEX6-U

.4........3. 
............
.5...FC...2. 
............
...6....1...
  
```
|Command| Roll | Pitch | Yaw |
| ----- | ---- | ----- | --- | 
| Use `cmix 1 1.0, -0.5,  1.0, -1.0` | half negative | full positive | CW |
| Use `cmix 2 1.0, -1.0,  0.0,  1.0` | full negative | none | CCW | 
| Use `cmix 3 1.0, -1.0, -1.0, -1.0` | full negative | full negative | CW | 
| Use `cmix 4 1.0,  1.0, -1.0,  1.0` | full positive | full negative | CCW  | 
| Use `cmix 5 1.0,  1.0,  0.0, -1.0` | full positive | none | CW | 
| Use `cmix 6 1.0,  0.5,  1.0,  1.0` | half positive | full positive | CCW | 
