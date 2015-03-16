# Failsafe

There are two types of failsafe:

1. Receiver based failsafe
2. Flight controller based failsafe

Receiver based failsafe is where you, from your transmitter and receiver, configure channels to output desired signals if your receiver detects signal loss.
The idea is that you set throttle and other controls so the aircraft descends in a controlled manner.  See your receiver's documentation for this method.

Flight controller based failsafe is where the flight controller attempts to detect signal loss from your receiver.

It is possible to use both types at the same time which may be desirable.  Flight controller failsafe can even help if your receiver signal wires come loose, get damaged or your receiver malfunctions in a way the receiver itself cannot detect.

## Flight controller failsafe system 

The failsafe system is not activated until 5 seconds after the flight controller boots up.  This is to prevent failsafe from activating, as in the case of TX/RX gear with long bind procedures, before the RX sends out valid data.

After the failsafe has forced a landing, the flight controller cannot be armed and has to be reset.
 
The failsafe system attempts to detect when your receiver loses signal.  It then attempts to prevent your aircraft from flying away uncontrollably by enabling an auto-level mode and setting the throttle that should allow the craft to come to a safter landing.

The failsafe is activated when:

Either:

a) no valid channel data from the RX is received via Serial RX.

b) the first 4 Parallel PWM/PPM channels do not have valid signals.

And when:

c) the failsafe guard time specified by `failsafe_delay` has elapsed.

Note that:

d) The failsafe system will be activated regardless of current throttle position.

e) The craft may already be on the ground with motors stopped and that motors and props could spin again - the software does not currently detect if the craft is on the ground.  Take care when using MOTOR_STOP feature.

### Testing

Bench test the failsafe system before flying - remove props while doing so.

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
1. Observe that normal flight behaviour is resumed.
1. Disarm.
 
Field test the failsafe system

1. Perform bench testing first!
1. On a calm day go to an un-populated area away from buildings or test indoors in a safe controlled environment - e.g. inside a big net.
1. Arm the craft.
1. Hover over something soft (long grass, ferns, heather, foam, etc).
1. Descend the craft and observe throttle position and record throttle value from your TX channel monitor.  Ideally 1500 should be hover. So your value should be less than 1500.
1. Stop, disarm.
1. Set failsafe thottle to the recorded value.
1. Arm, hover over something soft again.
1. Turn off TX (!)
1. Observe craft descends and motors continue to spin for the configurated duration.
1. Observe FC disarms after the configured duration.
1. Remove flight battery.

If craft descends too quickly then increase failsafe throttle setting.

Ensure that the duration is long enough for your craft to land at the altitudes you normally fly at.



## Configuration

When configuring the flight controller failsafe, use the following steps:

1.  Configure your receiver to do one of the following:

a)  Upon signal loss, send no signal/pulses over the channels

b)  Send an invalid signal over the channels (for example, send values lower than 'failsafe_min_usec')

See your receiver's documentation for direction on how to accomplish one of these.

2.  Set 'failsafe_off_delay' to an appropriate value based on how high you fly

3.  Set 'failsafe_throttle' to a value that allows the aircraft to descend at approximately one meter per second.


These are the basic steps for flight controller failsafe configuration, see Failsafe Settings below for additional settings that may be changed.

##Failsafe Settings

Failsafe delays are configured in 0.1 second steps.

1 step = 0.1sec

1 second = 10 steps

### `failsafe_delay`

Guard time for failsafe activation after signal lost.  This is the amount of time the flight controller waits to see if it begins receiving a valid signal again before activating failsafe.

### `failsafe_off_delay`

Delay after failsafe activates before motors finally turn off.  This is the amount of time 'failsafe_throttle' is active.  If you fly at higher altitudes you may need more time to descend safely.

### `failsafe_throttle`

Throttle level used for landing.  Specify a value that causes the aircraft to descend at about 1M/sec.

Use standard RX usec values.  See RX documentation.

### `failsafe_min_usec`

The shortest PWM/PPM pulse considered valid.

Only valid when using Parallel PWM or PPM receivers.

### `failsafe_max_usec`

The longest PWM/PPM pulse considered valid.

Only valid when using Parallel PWM or PPM receivers.

This setting helps detect when your RX stops sending any data when the RX looses signal.

With a Graupner GR-24 configured for PWM output with failsafe on channels 1-4 set to OFF in the receiver settings
then this setting, at its default value, will allow failsafe to be activated.
