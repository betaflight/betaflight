# Failsafe

There are two types of failsafe:

1. receiver based failsafe
2. flight controller based failsafe

Receiver based failsafe is where you, from your transmitter and receiver, configure channels to output desired signals if your receiver detects signal loss.
The idea is that you set throttle and other controls so the aircraft descends in a controlled manner.

Flight controller based failsafe is where the flight controller attempts to detect signal loss from your receiver.

It is possible to use both types at the same time and may be desirable.  Flight controller failsafe can even help if your receiver signal wires come loose, get damaged or your receiver malfunctions in a way the receiver itself cannot detect.

## Flight controller failsafe system 

The failsafe system is not activated until 5 seconds after the flight controller boots up.  This is to prevent failsafe from activating in the case of TX/RX gear with long bind procedures before they send out valid data.

After the failsafe has been forced a landing, the flight controller cannot be armed and has to be reset.
 
The failsafe system attempts to detect when your receiver loses signal.  It then attempts to prevent your aircraft from flying away uncontrollably.

The failsafe is activated when:

Either:
a) no valid channel data from the RX via Serial RX.
b) the first 4 Parallel PWM/PPM channels do not have valid signals.

And:
c) the failsafe guard time specified by `failsafe_delay` has elapsed. 

## Configuration

There are a few settings for it, as below.

Failsafe delays are configured in 0.1 second steps.

1 step = 0.1sec
1 second = 10 steps

### `failsafe_delay`

Guard time for failsafe activation after signal lost.

### `failsafe_off_delay`

Delay after failsafe activates before motors finally turn off.  If you fly high you may need more time.

### `failsafe_throttle`

Throttle level used for landing.  Specify a value that causes the aircraft to descend at about 1M/sec.

Use standard RX usec values.  See Rx documentation.

### `failsafe_min_usec`

The shortest PWM/PPM pulse considered valid.

Only valid when using Parallel PWM or PPM receivers.

### `failsafe_max_usec`

The longest PWM/PPM pulse considered valid.

Only valid when using Parallel PWM or PPM receivers.

This setting helps catch when your RX stops sending any data when the RX looses signal.

With a Graupner GR-24 configured for PWM output with failsafe on channels 1-4 set to OFF in the receiver settings
then this setting, at its default value, will allow failsafe to be activated.
