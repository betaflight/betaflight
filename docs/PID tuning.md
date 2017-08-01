# PID tuning

Every aspect of flight dynamics is controlled by the selected "PID controller". This is an algorithm which is
responsible for reacting to your stick inputs and keeping the craft stable in the air by using the gyroscopes and/or
accelerometers (depending on your flight mode).

The "PIDs" are a set of tuning parameters which control the operation of the PID controller. The optimal PID settings
to use are different on every craft, so if you can't find someone with your exact setup who will share their settings
with you, some trial and error is required to find the best performing PID settings.

A video on how to recognise and correct different flight problems caused by PID settings is available here:

https://www.youtube.com/watch?v=YNzqTGEl2xQ

Basically, the goal of the PID controller is to bring the craft's rotation rate in all three axes to the rate that
you're commanding with your sticks. An error is computed which is the difference between your target rotation rate and
the actual one measured by the gyroscopes, and the controller tries to bring this error to zero.

Note that:

* For fixed wing, a PIFF controller is used. Some documentation is available in the wiki and legacy release notes.
* The iNav Configurator provides conservative example PID settings for various aircraft types. These will require tuning to a particular machine.

## PIDs

**The P term** controls the strength of the correction that is applied to bring the craft toward the target angle or
rotation rate. If the P term is too low, the craft will be difficult to control as it won't respond quickly enough to
keep itself stable. If it is set too high, the craft will rapidly oscillate/shake as it continually overshoots its
target.

**The I term** corrects small, long term errors. If it is set too low, the craft's attitude will slowly drift. If it is
set too high, the craft will oscillate (but with slower oscillations than with P being set too high).

**The D term** attempts to increase system stability by monitoring the rate of change in the error. If the error is rapidly converging to zero, the D term causes the strength of the correction to be backed off in order to avoid overshooting the target.


## TPA and TPA Breakpoint

TPA stands for Throttle PID Attenuation and according to [AlexYork.net](http://blog.alexyorke.net/what-is-tpa/):

> "TPA basically allows an aggressively tuned multi-rotor (one that feels very locked in) to reduce its PID gains when throttle is applied beyond the TPA threshold/breakpoint in order to eliminate fast oscillations.."

Note that TPA is set via CLI or on the PID TUNING tab of the GUI.  tpa_breakpoint is set via CLI

Also note that TPA and tpa_breakpoint may not be used in certain PID Controllers.  Check the description on the individual controller.

TPA applies a PID value reduction in relation to full Throttle. It is used to apply dampening of PID values as full throttle is reached.

**TPA** = % of dampening that will occur at full throttle.

**tpa_breakpoint** = the point in the throttle curve at which TPA will begin to be applied.

An Example: With TPA = 50 (or .5 in the GUI) and tpa_breakpoint = 1500 (assumed throttle range 1000 - 2000)

* At 1500 on the throttle channel, the PIDs will begin to be dampened.
* At 3/4 throttle (1750), PIDs are reduced by approximately 25% (half way between 1500 and 2000 the dampening will be 50% of the total TPA value of 50% in this example)
* At full throttle (2000) the full amount of dampening set in TPA is applied. (50% reduction in this example)
* TPA can lead into increase of rotation rate when more throttle applied.

![tpa example chart](https://cloud.githubusercontent.com/assets/1668170/6053290/655255dc-ac92-11e4-9491-1a58d868c131.png "TPA Example Chart")


**How and Why to use this?**

If you are getting oscillations starting at say 3/4 throttle, set tpa breakpoint = 1750 or lower (remember, this is assuming your throttle range is 1000-2000), and then slowly increase TPA until your oscillations are gone. Usually, you will want tpa breakpoint to start a little sooner then when your oscillations start so you'll want to experiment with the values to reduce/remove the oscillations.

## PID controllers

INAV has a single built-in PID controller. The PID controller scaling
means that Betaflight PIDs should be comparable.

Note that very old INAV versions had more  PID controllers. These have been removed.


## RC rate, Pitch and Roll Rates (P/R rate before they were separated), and Yaw rate

### RC Rate

An overall multiplier on the RC stick inputs for pitch, rol;, and yaw.
