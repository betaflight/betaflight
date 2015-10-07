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

##PIDs

**The P term** controls the strength of the correction that is applied to bring the craft toward the target angle or
rotation rate. If the P term is too low, the craft will be difficult to control as it won't respond quickly enough to
keep itself stable. If it is set too high, the craft will rapidly oscillate/shake as it continually overshoots its
target.

**The I term** corrects small, long term errors. If it is set too low, the craft's attitude will slowly drift. If it is
set too high, the craft will oscillate (but with slower oscillations than with P being set too high).

**The D term** attempts to increase system stability by monitoring the rate of change in the error. If the error is rapidly converging to zero, the D term causes the strength of the correction to be backed off in order to avoid overshooting the target.


##TPA and TPA Breakpoint

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
* TPA can lead into increase of rotation rate when more throttle applied. You can get faster flips and rolls when more throttle applied due to coupling of PID's and rates. Only PID controllers 1 and 2 are using linear TPA implementation, where no rotation rates are affected when TPA is being used.

![tpa example chart](https://cloud.githubusercontent.com/assets/1668170/6053290/655255dc-ac92-11e4-9491-1a58d868c131.png "TPA Example Chart")


**How and Why to use this?**

If you are getting oscillations starting at say 3/4 throttle, set tpa breakpoint = 1750 or lower (remember, this is assuming your throttle range is 1000-2000), and then slowly increase TPA until your oscillations are gone. Usually, you will want tpa breakpoint to start a little sooner then when your oscillations start so you'll want to experiment with the values to reduce/remove the oscillations.

## PID controllers

Cleanflight has 6 built-in PID controllers which each have different flight behavior. Each controller requires
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
be. Note that the default value for P_Level is 90.  This is more than likely too high of a value for most, and will cause the model to be very unstable in Angle Mode, and could result in loss of control.  It is recommended to change this value to 20 before using PID Controller 1 in Angle Mode.

In Horizon mode, this controller uses the LEVEL "I" PID setting to decide how much auto-level correction should be applied. Level "I" term: Strength of horizon auto-level. value of 0.030 in the configurator equals to 3.0 for Level P. Level "D" term: Strength of horizon transition. 0 is more stick travel on level and 255 is more rate mode what means very narrow angle of leveling.


### PID controller 2, "LuxFloat"

PID Controller 2 is Lux's new floating point PID controller. Both controller 0 and 1 use integer arithmetic, which was
faster in the days of the slower 8-bit MultiWii controllers, but is less precise.

This controller has code that attempts to compensate for variations in the looptime, which should mean that the PIDs
don't have to be retuned when the looptime setting changes. 

There were initially some problems with horizon mode, and sluggishness in acro mode, that were recently fixed by
nebbian in v1.6.0.

It is the first PID Controller designed for 32-bit processors and not derived from MultiWii.

The strength of the auto-leveling correction applied during Angle mode is set by the parameter "level_angle" which
is labeled "LEVEL Proportional" in the GUI. This can be used to tune the auto-leveling strength in Angle mode compared to
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

### PID controller 5, "Harakiri"

PID Controller 5 is an port of the PID controller from the Harakiri firmware.

The algorithm is leveraging more floating point math. This PID controller also compensates for different looptimes on roll and pitch. It likely don't need retuning of the PID values when looptime is changing. There are two additional settings which are configurable via the CLI in Harakiri:

        set dterm_cut_hz = 0        [1-50Hz] Cut Off Frequency for D term of main PID controller
                                    (default of 0 equals to 12Hz which was the hardcoded setting in previous Cleanflight versions)
        set pid5_oldyw = 0          [0/1] 0 = multiwii 2.3 yaw (default), 1 = older yaw

The PID controller is flight tested and running well with the default PID settings. If you want do acrobatics start slowly.

Yaw authority is also quite good.


## RC rate, Pitch and Roll Rates (P/R rate before they were separated), and Yaw rate

### RC Rate

An overall multiplier on the RC stick inputs for pitch, rol;, and yaw. 

On PID Controllers 0, and 3-5 can be used to set the "feel" around center stick for small control movements. (RC Expo also affects this).For PID Controllers 1 and 2, this basically sets the baseline stick sensitivity

### Pitch and Roll rates

In PID Controllers 0 and 3-5, the affect of the PID error terms for P and D are gradually lessened as the control sticks are moved away from center, ie 0.3 rate gives a 30% reduction of those terms at full throw, effectively making the stabilizing effect of the PID controller less at stick extremes. This results in faster rotation rates. So for these controllers, you can set center stick sensitivity to control movement with RC rate above, and yet have much faster rotation rates at stick extremes.

For PID Controllers 1 and 2, this is an multiplier on overall stick sensitivity, like RC rate, but for roll and pitch independently. Stablility (to outside factors like turbulence) is not reduced at stick extremes. A zero value is no increase in stick sensitivity over that set by RC rate above. Higher values increases stick sensitivity across the entire stick movement range.

### Yaw Rate

In PID Controllers 0 and 5, it acts as a PID reduction as explained above. In PID Controllers 1-4, it acts as a stick sensitivity multiplier, as explained above.
