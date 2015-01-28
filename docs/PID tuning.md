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

The P term controls the strength of the correction that is applied to bring the craft toward the target angle or
rotation rate. If the P term is too low, the craft will be difficult to control as it won't respond quickly enough to
keep itself stable. If it is set too high, the craft will rapidly oscillate/shake as it continually overshoots its
target.

The I term corrects small, long term errors. If it is set too low, the craft's attitude will slowly drift. If it is
set too high, the craft will oscillate (but with slower oscillations than with P being set too high).

The D term attempts to increase system stability by monitoring the rate of change in the error. If the error is
changing slowly (so the P and I terms aren't having enough impact on reaching the target) the D term causes an increase
in the correction in order to reach the target sooner. If the error is rapidly converging to zero, the D term causes the
strength of the correction to be backed off in order to avoid overshooting the target.

## PID controllers

Cleanflight has 5 built-in PID controllers which each have different flight behavior. Each controller requires
different PID settings for best performance, so if you tune your craft using one PID controller, those settings will
likely not work well on any of the other controllers.

You can change between PID controllers by running `set pid_controller=n` on the CLI tab of the Cleanflight
Configurator, where `n` is the number of the controller you want to use. Please read these notes first before trying one
out.

### PID controller 0, "MultiWii" (default)

PID Controller 0 is the default controller in Cleanflight, and Cleanflight's default PID settings are tuned to be
middle-of-the-road settings for this controller. It originates from the old MultiWii PID controller from MultiWii 2.2
and earlier.

One of the quirks with this controller is that if you increase the P value for an axis, the maximum rotation rates for
that axis are lowered. Hence you need to crank up the pitch or roll rates if you have higher and higher P values.

In Horizon and Angle modes, this controller uses both the LEVEL "P" and "I" settings in order to tune the 
auto-leveling corrections in a similar way to the way that P and I settings are applied to roll and yaw axes in the acro
flight modes. The LEVEL "D" term is used as a limiter to constrain the maximum correction applied by the LEVEL "P" term.

### PID controller 1, "Rewrite"

PID Controller 1 is a newer PID controller that is derived from the one in MultiWii 2.3 and later. It works better from
all accounts, and fixes some inherent problems in the way the old one worked. From reports, tuning is apparently easier
on controller 1, and it tolerates a wider range of PID values well.

Unlike controller 0, controller 1 allows the user to manipulate PID values to tune reaction and stability without
affecting yaw, roll or pitch rotation rates (which are tuned by the dedicated roll & pitch and yaw rate
settings).

In Angle mode, this controller uses the LEVEL "P" PID setting to decide how strong the auto-level correction should
be.  

In Horizon mode, this controller uses the LEVEL "I" PID setting to decide how much auto-level correction should be
applied. The default Cleanflight setting for "I" will result in virtually no auto-leveling being applied, so that will
need to be increased in order to perform like PID controller 0.

The LEVEL "D" setting is not used by this controller.

### PID controller 2, "Baseflight"

PID Controller 2 is Lux's new floating point PID controller. Both controller 0 and 1 use integer arithmetic, which was
faster in the days of the slower 8-bit MultiWii controllers, but is less precise.

This controller has code that attempts to compensate for variations in the looptime, which should mean that the PIDs
don't have to be retuned when the looptime setting changes. 

There were initially some problems with horizon mode, and sluggishness in acro mode, that were recently fixed by
nebbian in v1.6.0. The autotune feature does not work on this controller, so don't try to autotune it.

Even though PC2 is called "pidBaseflight" in the code, it was never in Baseflight or MultiWii. A better name might have
been pidFloatingPoint, or pidCleanflight. It is the first PID Controller designed for 32-bit processors and not derived
from MultiWii. I believe it was named pidBaseflight because it was to be the first true 32-bit processor native PID
controller, and thus the native Baseflight PC, but Timecop never accepted the code into Baseflight.

The strength of the auto-leveling correction applied during Angle mode is set by the parameter "level_angle" which
is labeled "LEVEL Integral" in the GUI. This can be used to tune the auto-leveling strength in Angle mode compared to
Horizon mode. The default is 5.0.

The strength of the auto-leveling correction applied during Horizon mode is set by the parameter "level_horizon" which
is labeled "LEVEL Integral" in the GUI. The default is 3.0, which makes the Horizon mode apply weaker self-leveling than
the Angle mode. Note: There is currently a bug in the Configurator which shows this parameter divided by 100 (so it
shows as 0.03 rather than 3.0).

The transition between self-leveling and acro behavior in Horizon mode is controlled by the "sensitivity_horizon"
parameter which is labeled "LEVEL Derivative" in the Cleanflight Configurator GUI. This sets the percentage of your
stick travel that should have self-leveling applied to it, so smaller values cause more of the stick area to fly using
only the gyros. The default is 75% 

For example, at a setting of "100" for "sensitivity_horizon", 100% self-leveling strength will be applied at center
stick, 50% self-leveling will be applied at 50% stick, and no self-leveling will be applied at 100% stick. If
sensitivity is decreased to 75, 100% self-leveling will be applied at center stick, 50% will be applied at 63%
stick, and no self-leveling will be applied at 75% stick and onwards.

### PID controller 3, "MultiWii23" (default for the ALIENWIIF1 and ALIENWIIF3 targets)

PID Controller 3 is an direct port of the PID controller from MultiWii 2.3 and later.

The algorithm is handling roll and pitch differently to yaw. Users with problems on yaw authority should try this one.

For the ALIENWII32 targets the gyroscale is removed for even more yaw authority. This will provide best performance on very small multicopters with brushed motors.

### PID controller 4, "MultiWiiHybrid"

PID Controller 4 is an hybrid version of two MultiWii PID controllers. Roll and pitch is using the MultiWii 2.2 algorithm and yaw is using the 2.3 algorithm. 

This PID controller was initialy implemented for testing purposes but is also performing quite well.

For the ALIENWII32 targets the gyroscale is removed for more yaw authority. This will provide best performance on very small multicopters with brushed motors.
