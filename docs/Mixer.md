# Mixer

Cleanflight supports a number of mixing configurations as well as custom mixing.  Mixer configurations determine how the servos and motors work together to control the aircraft.

## Configuration

To use a built-in mixing configuration, you can use the Chrome configuration GUI.  It includes images of the various mixer types to assist in making the proper connections.  See the Configuration section of the documentation for more information on the GUI.

You can also use the Command Line Interface (CLI) to set the mixer type:

1. Use `mixer list` to see a list of supported mixes
2. Select a mixer.  For example, to select TRI, use `mixer TRI`
3. You must use `save` to preserve your changes

## Supported Mixer Types

| Name             | Description               | Motors         | Servos           |
| ---------------- | ------------------------- | -------------- | ---------------- |
| TRI              | Tricopter                 | M1-M3          | S1               |
| QUADP            | Quadcopter-Plus           | M1-M4          | None             |
| QUADX            | Quadcopter-X              | M1-M4          | None             |
| BI               | Bicopter (left/right)     | M1-M2          | S1, S2           |
| GIMBAL           | Gimbal control            | N/A            | S1, S2           |
| Y6               | Y6-copter                 | M1-M6          | None             |
| HEX6             | Hexacopter-Plus           | M1-M6          | None             |
| FLYING\_WING      | Fixed wing; elevons       | M1             | S1, S2           |
| Y4               | Y4-copter                 | M1-M4          | None             |
| HEX6X            | Hexacopter-X              | M1-M6          | None             |
| OCTOX8           | Octocopter-X (over/under) | M1-M8          | None             |
| OCTOFLATP        | Octocopter-FlatPlus       | M1-M8          | None             |
| OCTOFLATX        | Octocopter-FlatX          | M1-M8          | None             |
| AIRPLANE         | Fixed wing; Ax2, R, E     | M1             | S1, S2, S3, S4   |
| HELI\_120\_CCPM    | 3D-capable Helicopter     | M1             | S1, S2, S3, S4   |
| HELI\_90\_DEG      |                           |                |                  |
| VTAIL4           | Quadcopter with V-Tail    | M1-M4          | N/A              |
| HEX6H            | Hexacopter-H              | M1-M6          | None             |
| PPM\_TO\_SERVO     |                           |                |                  |
| DUALCOPTER       | Dualcopter                | M1-M2          | S1, S2           |
| SINGLECOPTER     | Conventional helicopter   | M1             | S1               |
| ATAIL4           | Quadcopter with A-Tail    | M1-M4          | N/A              |
| CUSTOM           | User-defined              |                |                  |
| CUSTOM AIRPLANE  | User-defined airplane     | M1-M2          | S1-S8            |
| CUSTOM TRICOPTER | User-defined tricopter    |                |                  |

## Servo configuration

The cli `servo` command defines the settings for the servo outputs.
The cli mixer `smix` command controls how the mixer maps internal FC data (RC input, PID stabilization output, channel forwarding, etc) to servo outputs.

### Channel Forwarding

Channel Forwarding allows you to forward your AUX channels directly to servos over PWM pins 5-8. You can enable it under features in the GUI or using the cli 
with ``feature CHANNEL_FORWARDING``. This requires you to run PPM or another serial RC protocol, and is currently supported on NAZE and SPRACINGF3 targets.
Note that if you have the led feature enabled on the NAZE target,  AUX1-2 is mapped to PWM13-14 instead. So for instance if you enable this feature on a Naze
AUX1 from your receiver will automatically be forwarded to PWM5 as a servo signal.

### cli `servo`

`servo <min> <max> <middle> <angleMin> <angleMax> <rate> <forwardFromChannel>`

- `<min>`, `<max>` - limit servo travel, in uS

- `<middle>` - mid value when not forwarding, value from servo mixer is added to this.

- `<angleMin>`,  `<angleMax>` - unused

- `<rate>` - scale for value from servo mixer or gimbal input, -100% .. 100%

- `<forwardFromChannel>` - use RC channel value as reference instead of `<middle>`. Servo will follow given RC channel, with possible correction from servo mixer. `<min>`, `<max>` are still honored.


## Servo filtering

A low-pass filter can be enabled for the servos.  It may be useful for avoiding structural modes in the airframe, for example.

### Configuration

Currently it can only be configured via the CLI:

1. Use `set servo_lowpass_freq = nnn` to select the cutoff frequency.  Valid values range from 10Hz to 400Hz, second order filter is used.
2. Use `set servo_lowpass_enable = ON` to enable filtering.

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
2. Use `mmix reset` to erase the any existing custom mixing.
3. Optionally use `mmix load <name>` to start with one of available mixers.
4. Issue a `mmix` statement for each motor.

The `mmix` statement has the following syntax: `mmix n THROTTLE ROLL PITCH YAW`

| Mixing table parameter | Definition |
| ---------------------- | ---------- |
| n	                 | Motor ordering number |
| THROTTLE	         | Indicates how much throttle is mixed for this motor. All values used in current configurations are set to 1.0 (full throttle mixing), but other non-zero values may be used. Unused set to 0.0. |
| ROLL	                 | Indicates how much roll authority this motor imparts to the roll of the flight controller. Accepts values nominally from -1.0 to 1.0. |
| PITCH	                 | Indicates the pitch authority this motor has over the flight controller. Also accepts values nominally from -1.0 to 1.0. |
| YAW	                 | Indicates the direction of the motor rotation in relationship with the flight controller. 1.0 = CCW -1.0 = CW. |

Note: the `mmix` command may show a motor mix that is not active, custom motor mixes are only active for models that use custom mixers.

Note: You have to configure every motor number starting at 0. Your command will be ignored if there was no `mmix` command for the previous motor number (mixer stops on first THROTTLE value that is zero). See example 5.


## Custom Servo Mixing

Custom servo mixing rules can be applied to each servo.  Rules are applied in the order they are defined.

##### `smix`

Prints current servo mixer

Note: the `smix` command may show a servo mix that is not active, custom servo mixes are only active for models that use custom mixers.

##### `smix reset`

Erase custom mixer. Servo reversal in current profile ONLY is erased too.

##### `smix load <name>`

Load servo part of given configuration (`<name>` is from `mixer list`)

##### `smix <rule> <servo> <source> <rate> <speed> <min> <max> <box>`

- `<rule>` is index of rule, used mainly for bookkeeping. Rules are applied in this order, but ordering has no influence on result in current code.

- `<servo>`

| id |  Servo slot |
|----|--------------|
| 0  | GIMBAL PITCH |
| 1  | GIMBAL ROLL |
| 2  | ELEVATOR / SINGLECOPTER\_4 |
| 3  | FLAPPERON 1 (LEFT) / SINGLECOPTER\_1 |
| 4  | FLAPPERON 2 (RIGHT) / BICOPTER\_LEFT / DUALCOPTER\_LEFT / SINGLECOPTER\_2 |
| 5  | RUDDER / BICOPTER\_RIGHT / DUALCOPTER\_RIGHT / SINGLECOPTER\_3 |
| 6  | THROTTLE (Based ONLY on the first motor output) |
| 7  | FLAPS |

Only some `<servo>` channels are connected to output, based on mode. For custom modes:
  - RUDDER for CUSTOM\_TRI
  - ELEVATOR ... FLAPS for CUSTOM\_AIRPLANE
  - no servos for CUSTOM

GIMBAL handling is hard-coded, mmix rule is ignored.

- `<source>`

| id |  Input sources |
|----|-----------------|
| 0  | Stabilized ROLL |
| 1  | Stabilized PITCH |
| 2  | Stabilized YAW |
| 3  | Stabilized THROTTLE (ONLY the first motor output) |
| 4  | RC ROLL |
| 5  | RC PITCH |
| 6  | RC YAW |
| 7  | RC THROTTLE |
| 8  | RC AUX 1 |
| 9  | RC AUX 2 |
| 10 | RC AUX 3 |
| 11 | RC AUX 4 |
| 12 | GIMBAL PITCH |
| 13 | GIMBAL ROLL |

Stabilized ROLL/PITCH/YAW is taken directly from RC command when in PASSTHRU mode.

- `<rate>` is used to scale `<source>`, -100% - 100% is allowed. Note that servo reversal may be applied, see below. Zero `<rate>` will terminate smix table.

- `<speed>` will limit <source> speed when non-zero. This speed is taken per-rule, so you may limit only some sources. Value is maximal change of value per loop (1ms with default configuration)

- `<min>` `<max>` - Value in percentage of full servo range. For symmetrical servo limits (equal distance between mid and min/max), 0% is servo min, 50% is servo center, 100% is max servo position. When mid position is asymmetrical, 0% and 100% limits will be shifted.

- `<box>` rule will be applied only when `<box>` is zero or corresponding SERVOx mode is enabled.

##### `smix reverse`

Print current servo reversal configuration

##### `smix reverse <servo> <source> r|n`

Each `<source>` may be `r`eversed or `n`ormal for given `<servo>`. It is almost equivalent to using negative <rate> in given rule, but `<min>, `<max>` limits are applied to value before reversing.
`smix reverse` works for non-custom mixers too.

e.g. when using the TRI mixer to reverse the tail servo on a tricopter use this:

`smix reverse 5 2 r`

i.e. when mixing rudder servo slot (`5`) using Stabilized YAW input source (`2`) reverse the direction (`r`)

`smix reverse` is a per-profile setting.  So ensure you configure it for your profiles as required.


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
2. Use `mmix reset`
3. Use `mmix 0 1.0,  1.0, -1.0, -1.0` for the Front Left motor. It tells the flight controller the #1 motor is used, provides positive roll, provides negative pitch and is turning CW.
4. Use `mmix 1 1.0, -1.0, -1.0,  1.0` for the Front Right motor. It still provides a negative pitch authority, but unlike the front left, it provides negative roll authority and turns CCW.
5. Use `mmix 2 1.0, -1.0,  1.0, -1.0` for the Rear Right motor. It has negative roll, provides positive pitch when the speed is increased and turns CW.
6. Use `mmix 3 1.0,  1.0,  1.0,  1.0` for the Rear Left motor. Increasing motor speed imparts positive roll, positive pitch and turns CCW.

### Example 2: A HEX-U Copter

Here is an example of a U-shaped hex; probably good for herding giraffes in the Sahara. Because the 1 and 6 motors are closer to the roll axis, they impart much less force than the motors mounted twice as far from the FC CG. The effect they have on pitch is the same as the forward motors because they are the same distance from the FC CG. The 2 and 5 motors do not contribute anything to pitch because speeding them up and slowing them down has no effect on the forward/back pitch of the FC.

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
| Use `mmix 0 1.0, -0.5,  1.0, -1.0` | half negative | full positive | CW |
| Use `mmix 1 1.0, -1.0,  0.0,  1.0` | full negative | none | CCW |
| Use `mmix 2 1.0, -1.0, -1.0, -1.0` | full negative | full negative | CW |
| Use `mmix 3 1.0,  1.0, -1.0,  1.0` | full positive | full negative | CCW  |
| Use `mmix 4 1.0,  1.0,  0.0, -1.0` | full positive | none | CW |
| Use `mmix 5 1.0,  0.5,  1.0,  1.0` | half positive | full positive | CCW |

### Example 3: Custom tricopter

```
mixer CUSTOMTRI
mmix reset
mmix 0 1.000 0.000 1.333 0.000
mmix 1 1.000 -1.000 -0.667 0.000
mmix 2 1.000 1.000 -0.667 0.000
smix reset
smix 0 5 2 100 0 0 100 0
profile 0
smix reverse 5 2 r
profile 1
smix reverse 5 2 r
profile 2
smix reverse 5 2 r

```

### Example 4: Custom Airplane with Differential Thrust
Here is an example of a custom twin engine plane with [Differential Thrust](http://rcvehicles.about.com/od/rcairplanes/ss/RCAirplaneBasic.htm#step8)
Motors take the first 2 pins, the servos take pins as indicated in the [Servo slot] chart above.
Settings bellow have motor yaw influence at "0.3", you can change this number to have more or less differential thrust over the two motors.
Note: You can look at the Motors tab in [Cleanflight Cofigurator](https://chrome.google.com/webstore/detail/cleanflight-configurator/enacoimjcgeinfnnnpajinjgmkahmfgb?hl=en) to see motor and servo outputs.

| Pins | Outputs          |
|------|------------------|
| 1    | Left Engine      |
| 2    | Right Engine     |
| 3    | Pitch / Elevator |
| 4    | Roll / Aileron   |
| 5    | Roll / Aileron   |
| 6    | Yaw / Rudder     |
| 7    | [EMPTY]          |
| 8    | [EMPTY]          |

```
mixer CUSTOMAIRPLANE
mmix reset
mmix 0 1.0 0.0 0.0 0.3   # Left Engine
mmix 1 1.0 0.0 0.0 -0.3  # Right Engine

smix reset
# Rule	Servo	Source	Rate	Speed	Min	Max	Box
smix 0 3 0 100 0 0 100 0  # Roll / Aileron
smix 1 4 0 100 0 0 100 0  # Roll / Aileron
smix 2 5 2 100 0 0 100 0  # Yaw / Rudder
smix 3 2 1 100 0 0 100 0  # Pitch / Elevator

```

### Example 5: Use motor output 0,1,2,4 because your output 3 is broken
For this to work you have to make a dummy mmix for motor 3. We do this by just saying it has 0 impact on yaw, roll and pitch.
```
mixer custom
mmix reset
mmix 0 1.0, -1.0, 1.0, -1.0
mmix 1 1.0, -1.0, -1.0,  1.0
mmix 2 1.0, 1.0, 1.0, 1.0
mmix 3 1.0, 0.0, 0.0, 0.0
mmix 4 1.0, 1.0, -1.0, -1.0
save
```
