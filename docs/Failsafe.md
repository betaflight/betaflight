# Failsafe

There are two types of failsafe:

1. Receiver based failsafe
2. Flight controller based failsafe

Receiver based failsafe is where you, from your transmitter and receiver, configure channels to output desired signals if your receiver detects signal loss and goes to the __failsafe mode__. The idea is that you set throttle and other controls so the aircraft descends in a controlled manner.  See your receiver's documentation for this method.

Flight controller based failsafe is where the flight controller attempts to detect signal loss and/or the __failsafe mode__ of your receiver and upon detection goes to __failsafe stage 1__. The idea is that the flight controller starts using __fallback settings__ for all controls, which are set by you, using the CLI command `rxfail` (see `rxfail` section in rx documentation) or the cleanflight-configurator GUI.

It is possible to use both types at the same time, which may be desirable.  Flight controller failsafe can even help if your receiver signal wires come loose, get damaged or your receiver malfunctions in a way the receiver itself cannot detect.

Alternatively you may configure a transmitter switch to activate failsafe mode. This is useful for fieldtesting the failsafe system and as a **_`PANIC`_** switch when you lose orientation.

## Flight controller failsafe system

This system has two stages.

__Stage 1__ is entered when __a flightchannel__ has an __*invalid pulse length*__, the receiver reports __*failsafe mode*__ or there is __*no signal*__ from the receiver. Fallback settings are applied to __*all channels*__ and a short amount of time is provided to allow for recovery.

__Note:__ Prior to entering __stage 1__, fallback settings are also applied to __*individual AUX channels*__ that have invalid pulses.

__Stage 2__ is entered when your craft is __armed__ and __stage 1__ persists longer then the configured guard time (`failsafe_delay`). All channels will remain at the applied fallback setting unless overruled by the chosen stage 2 procedure (`failsafe_procedure`).

__Stage 2__ is not activated until 5 seconds after the flight controller boots up.  This is to prevent unwanted activation, as in the case of TX/RX gear with long bind procedures, before the RX sends out valid data.

__Stage 2__ can also directly be activated when a transmitter switch that is configured to control the failsafe mode is switched ON (and `failsafe_kill_switch` is set to OFF).

__Stage 2__ will be aborted when it was due to:

* a lost RC signal and the RC signal has recovered.
* a transmitter failsafe switch was set to ON position and the switch is set to OFF position (and `failsafe_kill_switch` is set to OFF).

Note that:
* At the end of the stage 2 procedure, the flight controller will be disarmed and re-arming will be locked until the signal from the receiver is restored for 30 seconds AND the arming switch is in the OFF position (when an arm switch is in use).

* When `failsafe_kill_switch` is set to ON and the rc switch configured for failsafe is set to ON, the craft is instantly disarmed. Re-arming is possible when the signal from the receiver has restored for at least 3 seconds AND the arming switch is in the OFF position (when one is in use). Similar effect can be achieved by setting 'failsafe_throttle' to 1000 and 'failsafe_off_delay' to 0. This is not the prefered method, since the reaction is slower and re-arming will be locked.

* Prior to starting a stage 2 intervention it is checked if the throttle position was below `min_throttle` level for the last `failsafe_throttle_low_delay` seconds. If it was, the craft is assumed to be on the ground and is only disarmed. It may be re-armed without a power cycle. This feature can be disabled completely by 
setting `failsafe_throttle_low_delay` to zero. This is useful for gliders that may fly long with zero throttle.

Some notes about **SAFETY**:
* The failsafe system will be activated regardless of current throttle position. So when the failsafe intervention is aborted (RC signal restored/failsafe switch set to OFF) the current stick position will direct the craft !
* The craft may already be on the ground with motors stopped and that motors and props could spin again - the software does not currently detect if the craft is on the ground.  Take care when using `MOTOR_STOP` feature. **Props will spin up without warning**, when armed with `MOTOR_STOP` feature ON (props are not spinning) **_and_** failsafe is activated !

## Configuration

When configuring the flight controller failsafe, use the following steps:

1.  Configure your receiver to do one of the following:

* Upon signal loss, send no signal/pulses over the channels
* Send an invalid signal over the channels (for example, send values lower than `rx_min_usec`)

and

* Ensure your receiver does not send out channel data that would cause a disarm by switch or sticks to be registered by the FC. This is especially important for those using a switch to arm.

See your receiver's documentation for direction on how to accomplish one of these.

* Configure one of the transmitter switches to activate the failsafe mode.

2.  Set `failsafe_off_delay` to an appropriate value based on how high you fly

3.  Set `failsafe_throttle` to a value that allows the aircraft to descend at approximately one meter per second (default is 1000 which should be throttle off).




These are the basic steps for flight controller failsafe configuration; see Failsafe Settings below for additional settings that may be changed.

## Failsafe Settings

Failsafe delays are configured in 0.1 second steps.

1 step = 0.1sec

1 second = 10 steps

### `failsafe_delay`

Guard time for failsafe activation after signal lost.  This is the amount of time the flight controller waits to see if it begins receiving a valid signal again before activating failsafe.

### `failsafe_recovery_delay`

Guard time for failsafe de-activation after signal is recovered.  This is the amount of time the flight controller waits to see if the signal is consistent before turning off failsafe procedure. Usefull to avoid swithing in and out of failsafe RTH.

### `failsafe_off_delay`

Delay after failsafe activates before motors finally turn off.  This is the amount of time 'failsafe_throttle' is active.  If you fly at higher altitudes you may need more time to descend safely.

### `failsafe_throttle`

Throttle level used for landing.  Specify a value that causes the aircraft to descend at about 1M/sec. Default is set to 1000 which should correspond to throttle off.

### `failsafe_kill_switch`

Configure the rc switched failsafe action: the same action as when the rc link is lost (set to OFF) or disarms instantly (set to ON). Also see above.

### `failsafe_throttle_low_delay`

Time throttle level must have been below 'min_throttle' to _only disarm_ instead of _full failsafe procedure_.

Use standard RX usec values.  See Rx documentation.

### `failsafe_procedure`

* __Drop:__ Just kill the motors and disarm (crash the craft).
* __Land:__ Enable an auto-level mode, center the flight sticks and set the throttle to a predefined value (`failsafe_throttle`) for a predefined time (`failsafe_off_delay`). This should allow the craft to come to a safer landing.
* __RTH:__ Attempt to return and land the drone at the point of launch. GPS and Barometer required for proper operation. If this more is selected and GPS is not available - the drone will be landed immediately.

### `rx_min_usec`

The lowest channel value considered valid.  e.g. PWM/PPM pulse length

### `rx_max_usec`

The highest channel value considered valid.  e.g. PWM/PPM pulse length

The `rx_min_usec` and `rx_max_usec` settings helps detect when your RX stops sending any data, enters failsafe mode or when the RX looses signal.

With a Graupner GR-24 configured for PWM output with failsafe on channels 1-4 set to OFF in the receiver settings then this setting, at its default value, will allow failsafe to be activated.

## Testing

**Bench test the failsafe system before flying - _remove props while doing so_.**

1. Arm the craft.
1. Turn off transmitter or unplug RX.
1. Observe motors spin at configured throttle setting for configured duration.
1. Observe motors turn off after configured duration.
1. Ensure that when you turn on your TX again or reconnect the RX that you cannot re-arm once the motors have stopped.
1. Power cycle the FC.
1. Arm the craft.
1. Turn off transmitter or unplug RX.
1. Observe motors spin at configured throttle setting for configured duration.
1. Turn on TX or reconnect RX.
1. Ensure that your switch positions don't now cause the craft to disarm (otherwise it would fall out of the sky on regained signal).
1. Observe that normal flight behavior is resumed.
1. Disarm.

**Field test the failsafe system.**

1. Perform bench testing first!
1. On a calm day go to an unpopulated area away from buildings or test indoors in a safe controlled environment - e.g. inside a big net.
1. Arm the craft.
1. Hover over something soft (long grass, ferns, heather, foam, etc.).
1. Descend the craft and observe throttle position and record throttle value from your TX channel monitor.  Ideally 1500 should be hover. So your value should be less than 1500.
1. Stop, disarm.
1. Set failsafe throttle to the recorded value.
1. Arm, hover over something soft again.
1. Turn off TX (!)
1. Observe craft descends and motors continue to spin for the configured duration.
1. Observe FC disarms after the configured duration.
1. Remove flight battery.

If craft descends too quickly then increase failsafe throttle setting.

Ensure that the duration is long enough for your craft to land at the altitudes you normally fly at.

Using a configured transmitter switch to activate failsafe mode, instead of switching off your TX, is good primary testing method in addition to the above procedure.
