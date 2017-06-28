# Mixer

INAV supports a number of mixing configurations as well as custom mixing.  Mixer configurations determine how the servos and motors work together to control the aircraft.

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
| FLYING_WING      | Fixed wing; elevons       | M1             | S1, S2           |
| Y4               | Y4-copter                 | M1-M4          | None             |
| HEX6X            | Hexacopter-X              | M1-M6          | None             |
| OCTOX8           | Octocopter-X (over/under) | M1-M8          | None             |
| OCTOFLATP        | Octocopter-FlatPlus       | M1-M8          | None             |
| OCTOFLATX        | Octocopter-FlatX          | M1-M8          | None             |
| AIRPLANE         | Fixed wing; Ax2, R, E     | M1             | S1, S2, S3, S4   |
| HELI_120_CCPM    |                           |                |                  |
| HELI_90_DEG      |                           |                |                  |
| VTAIL4           | Quadcopter with V-Tail    | M1-M4          | N/A              |
| HEX6H            | Hexacopter-H              | M1-M6          | None             |
| PPM_TO_SERVO     |                           |                |                  |
| DUALCOPTER       | Dualcopter                | M1-M2          | S1, S2           |
| SINGLECOPTER     | Conventional helicopter   | M1             | S1               |
| ATAIL4           | Quadcopter with A-Tail    | M1-M4          | N/A              |
| CUSTOM           | User-defined              |                |                  |
| CUSTOM AIRPLANE  | User-defined airplane     |                |                  |
| CUSTOM TRICOPTER | User-defined tricopter    |                |                  |

## Servo configuration

The cli `servo` command defines the settings for the servo outputs.
The cli mixer `smix` command controllers how the mixer maps internal FC data (RC input, PID stabilisation output, channel forwarding, etc) to servo outputs.

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
2. Use `mmix reset` to erase the any existing custom mixing.
3. Issue a `mmix` statement for each motor.

The mmix statement has the following syntax: `mmix n THROTTLE ROLL PITCH YAW`

| Mixing table parameter | Definition |
| ---------------------- | ---------- |
| n	| Motor ordering number |
| THROTTLE	| All motors that are used in this configuration are set to 1.0. Unused set to 0.0. |
| ROLL	| Indicates how much roll authority this motor imparts to the roll of the flight controller. Accepts values nominally from 1.0 to -1.0. |
| PITCH	| Indicates the pitch authority this motor has over the flight controller. Also accepts values nominally from 1.0 to -1.0. |
| YAW	| Indicates the direction of the motor rotation in relationship with the flight controller. 1.0 = CCW -1.0 = CW. |

Note: the `mmix` command may show a motor mix that is not active, custom motor mixes are only active for models that use custom mixers.

## Custom Servo Mixing

Custom servo mixing rules can be applied to each servo.  Rules are applied in the CLI using `smix`. Rules link flight controller stabilization and receiver signals to physical pwm output pins on the FC board. Currently, pin id's 0 and 1 can only be used for motor outputs. Other pins may or may not work depending on the board you are using.

The mmix statement has the following syntax: `smix n SERVO_ID SIGNAL_SOURCE RATE SPEED`
For example, `smix 0 2 0 100 0` will assign Stabilised Roll to the third pwm pin on the FC board.

| id | Flight Controller Output signal sources |
|----|-----------------|
| 0  | Stabilised ROLL |
| 1  | Stabilised PITCH |
| 2  | Stabilised YAW |
| 3  | Stabilised THROTTLE |
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
| 14 | FEATURE FLAPS |

| id |  Servo Slot Optional Setup |
|----|--------------|
| 0  | GIMBAL PITCH |
| 1  | GIMBAL ROLL |
| 2  | ELEVATOR / SINGLECOPTER_4 |
| 3  | FLAPPERON 1 (LEFT) / SINGLECOPTER_1 |
| 4  | FLAPPERON 2 (RIGHT) / BICOPTER_LEFT / DUALCOPTER_LEFT / SINGLECOPTER_2 |
| 5  | RUDDER / BICOPTER_RIGHT / DUALCOPTER_RIGHT / SINGLECOPTER_3 |
| 6  | THROTTLE (Based ONLY on the first motor output) |
| 7  | FLAPS |

### Servo rule rate

Servo rule rate should be understood as a weight of a rule. To obtain full servo throw without clipping sum of all `smix` rates for a servo should equals `100`. For example, is servo #2 should be driven by sources 0 and 1 (Stablilized Roll and Stablized Pitch) with equal strength, correct rules would be:

```
smix 0 2 0 50 0
smix 1 2 1 50 0
```  

To obtain stronger input of one source, increase rate of this source while decreasing the others. For example, to drive servo #2 in 75% from source 0 and in 25% from source 1, correct rules would be:

```
smix 0 2 0 75 0
smix 1 2 1 25 0
```  

If sum of weights would be bigger than `100`, clipping to servo min and max values might appear.

> Note: the `smix` command may show a servo mix that is not active, custom servo mixes are only active for models that use custom mixers.

### Servo speed

Custom servo mixer allows to define the speed of change for given servo rule. By default, all speeds are set to `0`, that means limiting is _NOT_ applied and rules source is directly written to servo. That mean, if, for example, source (AUX) changes from 1000 to 2000 in one cycle, servo output will also change from 1000 to 2000 in one cycle. In this case, speed is limited only by the servo itself.

If value different than `0` is set as rule speed, speed of change will be lowered accordingly. 

`1 speed = 10 us/s`

**Example speed values**
* 0 = no limiting
* 1 = 10us/s -> full servo sweep (from 1000 to 2000) is performed in 100s 
* 10 = 100us/s -> full sweep (from 1000 to 2000)  is performed in 10s
* 100 = 1000us/s -> full sweep in 1s
* 200 = 2000us/s -> full sweep in 0.5s 

Servo speed might be useful for functions like flaps, landing gear retraction and other where full speed provided for hardware is too much.

## Servo Reversing

Servos are reversed using the `smix reverse` command.

e.g. when using the TRI mixer to reverse the tail servo on a tricopter use this:

`smix reverse 5 2 r`

i.e. when mixing rudder servo slot (`5`) using Stabilised YAW input source (`2`) reverse the direction (`r`)

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
Note: You can look at the Motors tab in [INAV Cofigurator](https://chrome.google.com/webstore/detail/inav-configurator/fmaidjmgkdkpafmbnmigkpdnpdhopgel) to see motor and servo outputs.

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
# Rule	Servo	Source	Rate	Speed	Min	Max
smix 0 3 0 100 0 0 100  # Roll / Aileron
smix 1 4 0 100 0 0 100  # Roll / Aileron
smix 2 5 2 100 0 0 100  # Yaw / Rudder
smix 3 2 1 100 0 0 100  # Pitch / Elevator

```
### Example 5: Custom Airplane with Flaps
Here is an example of a custom single engine plane with flaps: (https://hobbyking.com/en_us/orange-grey-tundra-color.html) and is an easy model to setup using the settings below. This custom mix assumes left and right ailerons are wired together to use the same output but actuate in reverse. Doing so allows you to conserve output pins so that boards like the Omnibus or SP Racing F3 EVO can include flaps output using only 6 pins total. (Currently, motors always take the first 2 pins even if you use smix to set pin #2.)
Note: You can look at the Motors tab in [INAV Cofigurator] to see motor and servo outputs.

| Pins | Outputs          |
|------|------------------|
| 1    | Main Engine      |
| 2    | [EMPTY]          |
| 3    | Pitch / Elevator |
| 4    | Roll / Aileron   |
| 5    | Flaps            |
| 6    | Yaw / Rudder     |
| 7    | [EMPTY]          |
| 8    | [EMPTY]          |

```
# mmix
mmix load customairplane
mmix reset
mmix 0  1.000  0.000  0.000  0.000 # Pin 1

# smix
smix load customairplane
smix reset
smix 0 2 1 100 0 0 100 # Pitch / Stab, Pin 3
smix 1 2 5 100 0 0 100 # Pitch / RC, Pin 3
smix 2 3 0 100 0 0 100 # Roll / Stab, Pin 4
smix 3 3 4 100 0 0 100 # Roll / RC, Pin 4
smix 4 4 9 100 0 0 100 # RC Aux 2 Flaps, Pin 5
smix 5 5 2 100 0 0 100 # Yaw / Stab, Pin 6
smix reverse 3 0 r # REVERSE Stab Roll
smix reverse 3 4 r # REVERSE RC Roll
save

```
