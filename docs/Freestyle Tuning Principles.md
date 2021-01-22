# Freestyle Tuning Principles

## Motivation

Provide easy and simple guidelines to set-up Betaflight for freestyle.

While Betaflight benefits from a large and vivid developers community
with frequent updates and new features releases, it has often been
criticized by freestyle pilots for being too complicated to use and hard
to keep up with.

This guide aims to provide simple guiding principles and tune
suggestions to make the best out of Betaflight for freestyle purposes.

## Principles and Attributes

Freestyle is mainly about the footage of a smooth and precise acrobatic
flight.

To achieve such goals a freestyle quad should be tuned with the
following principles in mind:

1.  Optimized for smoothness over low latency and sharp control.

> Freestyle pilots tend to prefer a smoother and "loser"
> quad over an extremely reactive (brain reading feeling) one. Mainly
> because it helps to smooth out micro correction and makes the footage
> look more organic and fluid.

2.  Optimized to behave predictively and with consistency.

> Consistency helps pilots to build muscle memory and get a feel of the
> quad, hence gain confidence and precision. One among the best pilots,
> Mr. Steel is known for running the same setup for several years and he
> very rarely makes changes to it.

With the above principles in mind we can distill three attributes we
should optimize for:


|   |  |
| ------------- | ------------- |
| **Attitude Hold**   | Attitude hold is the ability of the craft to keep it’s trajectory and behave as expected. A quadcopter with good attitude hold has low propwas, does not suffer from woubles or mid to high frequency vibrations and efficiently fights against external forces such as wind and gravity while tracking closely setpoint (stick inputs). Attitude provides **smoothness**.  |
| **Predictability**  | Predictability is the ability of the pilot to estimate how the craft will behave based on the environment conditions and the provided stick inputs. The more predictable is the quad behaving the more  **precision and confidence the pilot will gain**.  |
| **Responsiveness**  | Responsiveness is the ability of the craft to track setpoint (stick inputs) as close as possible. A responsive quad has very **low latency and feels connected**.  |



In theory all three attributes are equally important in practice an
increase of responsiveness might affect the pilot ability to fly
smoothly and consistently.

Therefore it is advised to compromise on responsiveness to keep the
quad behaving in a predictive and consistent manner

## Betaflight Tune 

### VBat Sag Compensation

This feature aims to provide motor response consistency across the
entire flight ([BF doc ref](https://github.com/betaflight/betaflight/wiki/4.2-Tuning-Notes#dynamic-battery-sag-compensation)).
By enabling VBat Sag Compensation the craft will fly more consistently
and predictively.

If you plan to use this feature it's crucial to enable it before
performing the PID tuning.

#### Suggested setting: values for a 5"

|   | **Value** |
| ------------- | ------------- |
| **VBat Sag Period (vbat_sag_lpf_period)**  | 200 (20 second)|
| **VBat compensation**    | 40-60|

     
                       

### PID

PID are at the core of a quad tuning, with PID tuning we can achieve a
good quadcopter attitude.

D is the most important PID term to achieve smooth flying, D helps to
minimize prop was as well as dumpening any quads movement. Freestyle
quads tend to use higher D gains.

To counterbalance a higher than usual D gain P needs to be increased as
well.

A simple approach to tune P and D is to set a desired D gain (e.g. 45)
and slowly increase P as high as possible without producing any bounce
back on flips and rolls (see [UAV Tech video](https://www.youtube.com/watch?v=qK5APBg76AU)).

The I term is generally good enough on default, however if the quad
feels sloppy increasing the gain could improve the overall attitude.

#### Suggested settings: values for a 5"


|   |  **P** | **I** | **D** |
| ------------- | ------------- | ------------- | ------------- |
| **Roll**  | 60-70  | 90-100  | 40-50  |
| **Pitch**  | 06-70  | 90-100  | 40-50  |
| **Yaw**  | 30-40  | 90-100  | 0  |

                    
#### Advanced Considerations

**Equal setpoint tracking latency across axes**

To further improve consistency it's important to properly tune all axes
(Roll, Pitch and Yaw) this will ensure that the setpoint tracking
latency is equal for each axe.

**Motor max out**

Another big factor for consistency is that when the craft is commanded
to make a move, the motors should not max out. If that happens you'll
get completely different responsiveness, and somewhat unpredictably.
This is again a key factor in consistency: having enough power and
authority on all axes.

### Feed Forward

Feed Forward is used to help the quad copter tracking the setpoint
(stick inputs) closer. In other words it reduces the latency between
stick movement and quadcopter movement.

Feed Forward anticipates how the quadcopter will move based on how fast and in which direction the sticks are moved.

Feed Forward is great to increase
responsiveness but makes the craft controll less direct (more variables will affect the rotation rate) hence harder for the pilot to pridict how the quad will behave especially on RC link with low frequency frame rate (e.g. Crossfire).


#### Suggested setting: low or off

**Unless the quadcopter suffers from lag (setpoint tracking
latency) or feels too slopy  it is advised to turn Feed
Forward down or even off.**

Latency per se is not a major problem, if kept under a reasonable amount
the human brain is very good at handling it. If the latency is
consistent you will very quickly get acquainted to it.

### D Min

D Min allows to run higher D gain on not so clean builds by dynamically
increasing D on sharp moves.It has been introduced to run cooler motors,
and have faster stick responsiveness.

D Min can negatively affect consistency as D is no longer constant but
varies depending on how quick the move is. Also, running a lower D
during shallow flight will reduce smoothness.

#### Suggested setting: off

**If your quad allows it (clean build with low noise) disabling D Min
increases the quad flight consistency by keeping D constant and at a
generally higher value.**

### TPA

TPA lowers the D and P gain after a certain throttle threshold. It has
been introduced to address fast oscillations induced by high throttle
motor noises on quads running high PID gains.

Similar to D Min this settings could negatively affect consistency
leading to an increase of rotation rate when more throttle applied ([BF
doc ref](https://github.com/betaflight/betaflight/blob/021921252ce41ab69c60a249e955b69360dac3d5/docs/PID%20tuning.md#tpa-and-tpa-breakpointre%20throttle%20applied.)).

**Increasing the default breakpoint value allows to keep D constant also
around mid throttle .**

#### Suggested setting: values for a 5"

|  | **Rate**  |  **Breakpoint**  |
| ------------- | ------------- | ------------- |
| **TPA**  |  40-50  | 1600- 1750 |


Lower rate and higher breakpoint will increase consistency but
eventually introduce oscillations, carefully tune the settings to
minimize the impact of TPA while avoiding oscillations.

### I term relax and iterm_windup 

I term relax aims to inhibit I during fast manoeuvres by preventing it
from further accumulating avoiding I term induced bounce back on flips
and rolls ([BF doc ref](https://github.com/betaflight/betaflight/wiki/I-Term-Relax-Explained)).

#### Suggested setting: values for a 5"


|  |**Axes**       |  **Type**  | **Cutoff**  |
| ------------- | ------------- | ------------- | ------------- |
| **Iterm Relax**  |  RP (Increment only)   |  Setpoint  |7-12|

        
### Anti gravity

Anti Gravity boosts the I term when fast throttle changes are detected.
It has been introduced to mitigate the craft nose tilt on throttle
changes ([BF doc ref](https://github.com/betaflight/betaflight/wiki/PID-Tuning-Guide#antigravity)).

Anti gravity helps to increase smoothness and hold the attitude on
maneuvers with fast throttle changes like boosts, powerloops, etc.

#### Suggested setting: values for a 5"

|  |**Gain**   | 
| ------------- | ------------- | 
|  **Anti gravity**   | 3.5 - 5   |

### Motor Idle Throttle Value, Dynamic Idle Value & Thrust Linear

Lower than default Motor Idle Throttle Value allows for greater hang
time and cleaner dives. However it comes at the cost of a weaker
attitude hold at zero throttle and increased risk of de-sync.

Dynamic Idle Value and Thrust Linear will help to mitigate those
collateral effects.

Dynamic Idle Value allows to define minimum motor RPM, if set at a
reasonable amount it avoids de-sync due to too low motor RPM.

Thrust Linear helps to boost the PID at low throttle helping the craft
to hold attitude.

#### Suggested setting: values for a 5"

|  |**Value**  | 
| ------------- | ------------- | 
|  **Motor Idle Throttle Value**  | 3% - 4%  |
|   **Dynamic Idle**   | Needs to be computed according to [doc](https://github.com/betaflight/betaflight/wiki/Tuning-Dynamic-Idle#setup---enabling-dynamic-idle). |
|  **Thrust Linear**     | 20-25 |
           

### RC smoothing

Higher than default RC smoothing helps reducing stick input gliches caused by noise in the RC link. 

#### Suggested setting:

|  |**Value**   | 
| ------------- | ------------- | 
|  **RC smoothing**   | 20   |


## Betaflight Filtering

Generally speaking the less the better as filtering introduces latency,
however since latency is not as critical for freestyling a more
conservative approach with a good amount of filtering is advised.

Not enough filtering can negatively affect smoothness and in some cases
even burn motors.

## BLHeli Settings

### PWM

Increasing PWM frequency helps to run smoother motors, and can also
provide greater throttle resolution especially at low values at the cost
of losing some thrust. The higher the PWM Frequency the more smoothness,
control, and throttle resolution you gain.

#### Suggested setting: values for a 5"

|  |**Value**   | 
| ------------- | ------------- | 
| **PWM Frequency**    | 48kHz - 96kHz   |


### Motor Timing

By setting motor timing to higher value it is possible to regain some of
the lost thrust due to running higher PWM Frequency.

#### Suggested setting: values for a 5"


|  |**Value**   | 
| ------------- | ------------- | 
| **Motor Timing**    | 22 - 24   |

