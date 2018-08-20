# Modes

There are various modes that can be toggled on or off.  Modes can be enabled/disabled by stick positions, auxillary receiver channels and other events such as failsafe detection.

| ID | Short Name               | Function                                                             |
| -- | ------------------------ | ------------------------------------------------------------------------------------ |
| 0  | ARM                      | Enables motors and flight stabilisation                                              |
| 1  | ANGLE                    | Legacy auto-level flight mode                                                        |
| 2  | HORIZON                  | Auto-level flight mode                                                               |
| 4  | ANTI GRAVITY             | Prevents dips and rolls on fast throttle changes                                     |
| 5  | MAG                      | Heading lock                                                                         |
| 6  | HEADFREE                 | Head Free - When enabled yaw has no effect on pitch/roll inputs                      |
| 7  | HEADADJ                  | Heading Adjust - Sets a new yaw origin for HEADFREE mode                             |
| 8  | CAMSTAB                  | Camera Stabilisation                                                                 |
| 12 | PASSTHRU                 | Pass roll, yaw, and pitch directly from rx to servos in airplane mix                 |
| 13 | BEEPERON                 | Enable beeping - useful for locating a crashed aircraft                              |
| 15 | LEDLOW                   | Switch off LED\_STRIP output                                                         |
| 17 | CALIB                    | Start in-flight calibration                                                          |
| 19 | OSD                      | Enable/Disable On-Screen-Display (OSD)                                               |
| 20 | TELEMETRY                | Enable telemetry via switch                                                          |
| 23 | SERVO1                   | Servo 1                                                                              |
| 24 | SERVO2                   | Servo 2                                                                              |
| 25 | SERVO3                   | Servo 3                                                                              |
| 26 | BLACKBOX                 | Enable BlackBox logging                                                              |
| 27 | FAILSAFE                 | Enter failsafe stage 2 manually                                                      |
| 28 | AIRMODE                  | Alternative mixer and additional PID logic for more stable copter                    |
| 29 | 3D                       | Enable 3D mode                                                                       |
| 30 | FPV ANGLE MIX            | Apply yaw rotation relative to a FPV camera mounted at a preset angle                |
| 31 | BLACKBOX ERASE           | Erase the contents of the onboard flash log chip (takes > 30 s)                      |
| 32 | CAMERA CONTROL 1         | Control function 1 of the onboard camera (if supported)                              |
| 33 | CAMERA CONTROL 2         | Control function 2 of the onboard camera (if supported)                              |
| 34 | CAMERA CONTROL 3         | Control function 3 of the onboard camera (if supported)                              |
| 35 | FLIP OVER AFTER CRASH    | Reverse the motors to flip over an upside down craft after a crash (DShot required)  |
| 36 | BOXPREARM                | When arming, wait for this switch to be activated before actually arming             |
| 37 | BEEP GPS SATELLITE COUNT | Use a number of beeps to indicate the number of GPS satellites found                 |
| 39 | VTX PIT MODE             | Switch the VTX into pit mode (low output power, if supported)                        |
| 40 | USER1                    | User defined switch 1. Intended to be used to control an arbitrary output with PINIO |
| 41 | USER2                    | User defined switch 2. Intended to be used to control an arbitrary output with PINIO |
| 42 | USER3                    | User defined switch 3. Intended to be used to control an arbitrary output with PINIO |
| 43 | USER4                    | User defined switch 4. Intended to be used to control an arbitrary output with PINIO |
| 44 | PID AUDIO                | Enable output of PID controller state as audio                                       |
| 45 | PARALYZE                 | Permanently disable a crashed craft until it is power cycled                         |
| 46 | GPS RESCUE               | Enable 'GPS Rescue' to return the craft to the location where it was last armed      |
| 47 | ACRO TRAINER             | Enable 'acro trainer' angle limiting in acro mode                                    |

## Auto-leveled flight

The default flight mode does not stabilize the multicopter around the roll and the pitch axes. That is, the multicopter does not level on its own if you center the pitch and roll sticks on the radio. Rather, they work just like the yaw axis: the rate of rotation of each axis is controlled directly by the related stick on the radio, and by leaving them centered the flight controller will just try to keep the multicopter in whatever orientation it's in. This default mode is called "Rate" mode, also sometime called "Acro" (from "acrobatic") or "Manual" mode, and is active whenever no auto-leveled mode is enabled.

If your flight controller is equipped with a 3 axis accelerometer (very likely), then you can enable one of the two available auto leveled flight modes.

## Mode details

### Angle

In this auto-leveled mode the roll and pitch channels control the angle between the relevant axis and the vertical, achieving leveled flight just by leaving the sticks centered.

### Horizon

This hybrid mode works exactly like the previous ANGLE mode with centered roll and pitch sticks (thus enabling auto-leveled flight), then gradually behaves more and more like the default RATE mode as the sticks are moved away from the center position.

### Headfree

In this mode, the "head" of the multicopter is always pointing to the same direction as when the feature was activated. This means that when the multicopter rotates around the Z axis (yaw), the controls will always respond according the same "head" direction.

With this mode it is easier to control the multicopter, even fly it with the physical head towards you since the controls always respond the same. This is a friendly mode to new users of multicopters and can prevent losing the control when you don't know the head direction. 

### Airmode

In the standard mixer / mode, when the roll, pitch and yaw gets calculated and saturates a motor, all motors
will be reduced equally. When motor goes below minimum it gets clipped off.
Say you had your throttle just above minimum and tried to pull a quick roll - since two motors can't go
any lower, you essentially get half the power (half of your PID gain).
If your inputs would asked for more than 100% difference between the high and low motors, the low motors
would get clipped, breaking the symmetry of the motor balance by unevenly reducing the gain.
Airmode will enable full PID correction during zero throttle and give you ability for nice zero throttle
gliding and actobatics. But also the cornering / turns will be much tighter now as there is always maximum
possible correction performed. Airmode can also be enabled to work at all times by always putting it on the
same switch like your arm switch or you can enable/disable it in air. Additional things and benefits: Airmode
will additionally fully enable Iterm at zero throttle. Note that there is still some protection on the ground
when throttle zeroed (below min\check) and roll/pitch sticks centered. This is a basic protection to limit
motors spooling up on the ground. Also the Iterm will be reset above 70% of stick input in acro mode to prevent
quick Iterm windups during finishes of rolls and flips, which will provide much cleaner and more natural stops
of flips and rolls what again opens the ability to have higher I gains for some.
Note that AIRMODE will also overrule motor stop function! It will basically also act as an idle up switch.


## Auxiliary Configuration

Spare auxillary receiver channels can be used to enable/disable modes.  Some modes can only be enabled this way.

Configure your transmitter so that switches or dials (potentiometers) send channel data on channels 5 and upwards (the first 4 channels are usually occupied by the throttle, aileron, rudder, and elevator channels).

_e.g. You can configure a 3 position switch to send 1000 when the switch is low, 1500 when the switch is in the middle and 2000 when the switch is high._

Configure your tx/rx channel limits to use values between 1000 and 2000.  The range used by mode ranges is fixed to 900 to 2100.

When a channel is within a specifed range the corresponding mode is enabled.

Use the GUI configuration tool to allow easy configuration when channel.

### CLI 

There is a CLI command, `aux` that allows auxillary configuration.  It takes 5 arguments as follows:

* AUD range slot number (0 - 39)
* mode id (see mode list above)
* AUX channel index (AUX1 = 0, AUX2 = 1,... etc)
* low position, from 900 to 2100. Should be a multiple of 25.
* high position, from 900 to 2100. Should be a multiple of 25.

If the low and high position are the same then the values are ignored.

e.g.

Configure AUX range slot 0 to enable ARM when AUX1 is withing 1700 and 2100.
 
```
aux 0 0 0 1700 2100
```

You can display the AUX configuration by using the `aux` command with no arguments.
