# Failsafe

Failsafe is a state the flight controller will enter when the radio receiver loses the RC link. Any of these of these conditions will trigger it:

* Ony one of the flight channels (pitch, roll, throttle or yaw) send no pulse
* Any channel signal is outside the range between `rx_min_usec` and `rx_max_usec`
* The FAILSAFE aux mode is activated

If the failsafe happens while the flight controller is disarmed, it only prevent arming. If it happens while armed, the failsafe policy configured in `failsafe_procedure` is engaged. The available policies are:

* __DROP:__ Just kill the motors and disarm (crash the craft).
* __SET-THR:__ Enable an auto-level mode (for multirotor) or enter preconfigured roll/pitch/yaw spiral down (for airplanes) and set the throttle to a predefined value (`failsafe_throttle`) for a predefined time (`failsafe_off_delay`). This is meant to get the craft to a safe-ish landing (or more realistically, a controlled crash), it doesn't require any extra sensor other than gyros and accelerometers.
* __RTH:__ (Return To Home) One of the key features of inav, it automatically navigates the craft back to the point where it was armed and lands it. Similarly to all other automated navigation methods, it requires GPS and barometer in order to be activated, plus compass for multicopters. Please note that, should any required sensor become unavailable during the failsafe (eg. loss of GPS fix), an emergency landing similar to **SET-THR** (but with barometer support) will be performed. An emergency landing would also be performed right when the failsafe is triggered if any required sensor is reported as unavailable.
* __NONE:__ Do nothing. This is a dummy procedure meant to let the craft perform a waypoint mission outside of RC range. Highly unsafe when used with manual flight.

Note that:
* Should the failsafe disarm the flight controller (**DROP**, **SET-THR** after `failsafe_off_delay` or **RTH** with `nav_disarm_on_landing` ON), the flight controller will be disarmed and re-arming will be locked until the signal from the receiver is restored for 30 seconds AND the arming switch is in the OFF position (when an arm switch is in use).

* Prior to starting failsafe it is checked if the throttle position was below `min_throttle` level for the last `failsafe_throttle_low_delay` seconds. If it was, the craft is assumed to be on the ground and is only disarmed. It may be re-armed without a power cycle. This feature can be disabled completely by setting `failsafe_throttle_low_delay` to zero. This is useful for gliders that may fly long with zero throttle.

Some notes about **SAFETY**:
* The failsafe system will be activated regardless of current throttle position. So, when the failsafe intervention is aborted (RC signal restored/failsafe switch set to OFF), the current stick positions will be enforced. Be ready to react quickly.

* The craft may already be on the ground with motors stopped and that motors and props could spin again.  Take care when using `MOTOR_STOP` feature. **Props will spin up without warning**, when armed with `MOTOR_STOP` feature ON (props are not spinning) **_and_** failsafe is activated !

## RX configuration

In order to engage failsafe mode correctly, you must configure your receiver to do one of the following on signal loss:

* Send no signal/pulses over the channels
* Send an invalid signal over the channels (for example, send values lower than `rx_min_usec`)
* Set an aux cannel to engage FAILSAFE flight mode.

and

* Ensure your receiver does not send out channel data that would cause a disarm by switch or sticks to be registered by the FC. This is especially important for those using a switch to arm.


## Failsafe Settings

Failsafe delays are configured in 0.1 second units. Distances are in centimeters (1/100 of a meter)

__Parameters relevant to all failsafe policies__

### `failsafe_procedure`

Selects the failsafe procedure. Valid procedures are **DROP**, **SET-THR**, **RTH** and **NONE**. See above for a detailed description of each one

### `failsafe_delay`

Guard time for failsafe activation when rx channel data is lost or invalid.  This is the amount of time the flight controller waits to see if it begins receiving a valid signal again before activating failsafe.

### `failsafe_recovery_delay`

Guard time for failsafe de-activation after signal is recovered.  This is the amount of time the flight controller waits to see if the signal is consistent before turning off failsafe procedure. Usefull to avoid swithing in and out of failsafe RTH.

### `failsafe_stick_threshold`

This parameter defines recovery from failsafe by stick motion. When set to zero failsafe procedure will be cleared as soon as RC link is recovered. 

When this is set to a non-zero value - failsafe won't clear even if RC link is recovered. You will have to deflect any of Roll/Pitch/Yaw sticks beyond this value to exit failsafe.

One use-case is Failsafe-RTH. When on the edge of radio coverage you may end up entering and exiting RTH if radio link is sporadic - happens a lot with long-range pilots. Setting `failsafe_stick_threshold` to a certain value (i.e. 100) RTH will be initiated on first signal loss and will continue as long as pilots want it to continue. When RC link is solid (based on RSSI etc) pilot will move sticks and regain control.

### `failsafe_throttle_low_delay`

Time throttle level must have been below 'min_throttle' to _only disarm_ instead of _full failsafe procedure_. Set to zero to disable.

### `rx_min_usec`

The lowest channel value considered valid.  e.g. PWM/PPM pulse length

### `rx_max_usec`

The highest channel value considered valid.  e.g. PWM/PPM pulse length


__Parameters relevant to **SET-THR** failsafe policy__

### `failsafe_off_delay`

Delay after failsafe activates before motors finally turn off.  This is the amount of time 'failsafe_throttle' is active.  If you fly at higher altitudes you may need more time to descend safely. Set to zero to keep `failsafe_throttle` active indefinitely.

### `failsafe_throttle`

Throttle level used for landing.  Specify a value that causes the aircraft to descend at about 1M/sec. Default is set to 1000 which should correspond to throttle off.

### `failsafe_fw_roll_angle`

When `SET-THR` failsafe is executed on a fixed-wing craft it's not safe to keep it level - airplane can glide for long distances. 

This parameter defines amount of roll angle (in 1/10 deg units) to execute on failsafe. Negative = LEFT

### `failsafe_fw_pitch_angle`

This parameter defines amount of pitch angle (in 1/10 deg units) to execute on `SET-THR` failsafe for an airplane. Negative = CLIMB

### `failsafe_fw_yaw_rate`

This parameter defines amount of yaw rate (in deg per second units) to execute on `SET-THR` failsafe for an airplane. Negative = LEFT

__Parameters relevant to **RTH** failsafe policy__

### `nav_min_rth_distance`

If the failsafe happens while the craft is within this distance (in cm) from the home position, the home position is considered immediately reached.

### `nav_rth_climb_first`

If ON the craft rises to `nav_rth_altitude` before heading to home position. if OFF the craft climbs on the way to home position.

### `nav_rth_climb_ignore_emerg`

When this option is OFF (default) and when you initiate RTH without GPS fix - aircraft will initiate emergency descent and go down. If you set this option to ON - aircraft will reach the RTH target altitude before initiating emergency descent. This is done for cases where GPS coverage is poor (i.e. in the mountains) - allowing UAV to climb up might improve GPS coverage and allow safe return instead of landing in a place where UAV might be impossible to recover.

### `nav_rth_tail_first`

Only relevant for multirotors. If this is OFF the craft will head to home position head first, if ON it'll be tail first

### `nav_rth_altitude`

The altitude used as reference for the RTH procedure.

### `nav_rth_alt_mode`

How and when to reach `nav_rth_altitude`. Please read [the page on the wiki](https://github.com/iNavFlight/inav/wiki/Navigation-modes#rth-altitude-control-modes) for a description of the available modes.

### `nav_rth_abort_threshold`

If the craft increases its distance from the home position by this amount of centimeters using the position where the failsafe was initially triggered as reference, RTH procedure is aborted and an emergency landing is initiated. It's meant to avoid flyaways due to navigation issues or too strong wind.

### `nav_rth_allow_landing`

Enables landing when home position is reached. If OFF the craft will hover indefinitely over the home position.

### `nav_disarm_on_landing`

Instructs the flight controller to disarm the craft when landing is detected


