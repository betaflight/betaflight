# Failsafe

There are two types of failsafe:

1. receiver based failsafe
2. flight controller based failsafe

Receiver based failsafe is where you, from your transmitter and receiver, configure channels to output desired signals if your receiver detects signal loss.
The idea is that you set throttle and other controls so the aircraft descends in a controlled manner.  See your receiver's documentation for this method.

Flight controller based failsafe is where the flight controller attempts to detect signal loss from your receiver.

It is possible to use both types at the same time which may be desirable.  Flight controller failsafe can even help if your receiver signal wires come loose, get damaged or your receiver malfunctions in a way the receiver itself cannot detect.

## Flight controller failsafe system 

The failsafe system is not activated until 5 seconds after the flight controller boots up.  This is to prevent failsafe from activating in the case of TX/RX gear with long bind procedures before they send out valid data.

After the failsafe has forced a landing, the flight controller cannot be armed and has to be reset.
 
The failsafe system attempts to detect when your receiver loses signal.  It then attempts to prevent your aircraft from flying away uncontrollably.

The failsafe is activated when:

Either:

a) no valid channel data from the RX is received via Serial RX.

b) the first 4 Parallel PWM/PPM channels do not have valid signals.

And:

c) the failsafe guard time specified by `failsafe_delay` has elapsed. 

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
