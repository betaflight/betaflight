#Rates, Expo, & TPA

RC Rates (on the RECEIVER tab in the GUI)  and Pitch & Roll and Yaw Rates (on the PID TUNING tab of the GUI);  

What is the difference?

* RC Rates affect the entire contorl input range. To decrease sensitivity at the center of the ranges, RC Expo can be used.
* Pitch & Roll and Yaw Rates target the endpoints of the inputs while having little or no effect on the center of the input ranges.

##RC Rate

RC Rate defines the sensitivity to PITCH and ROLL throughout the entire input range. 
* A lower # reduces the over all responsiveness. 
* A Higher # increases the over all responsiveness. 

If you feel your aircraft is too reactive to your inputs, decrease the value. If you feel it is too sluggish, increase the value. 

##RC Expo

RC Expo defines a smoother, less sensitive, zone at the center of the PITCH and ROLL input range without affecting the ends of the inputs.  
The higher the value, the less sensitive the Pitch and Roll inputs will be when in the center of their range. The lower the value, the more sensitive the Pitch and Roll inputs will be when at the center of their range.
The most sensitive at the center of the inputs is 0 Expo.

##Pitch & Roll and Yaw Rates

These values affect mostly the endpoints of the input.  For example,  If the over all control of the aircraft is good, i.e you have your RC Rates dialed in, but extreme enpoint controls need to be faster or slower, like for aerobatics, then these rates can be used to increase the response withoug effecting the center of the input ranges as much.

##TPA and TPA Breakpoint

* Note that TPA is set via CLI or on the PID TUNING tab of the GUI.  tpa_breakpoint is set via CLI

An Example: With TPA = 50 (or .5 in the GUI) and tpa_breakpoint = 1500

At full throttle, your P value is reduced by 50%
at 3/4 throttle (say 1750), your P value is reduced by 25% (1/2 the differnce of 1500 and 2000 = 1/2 of 50% = 25%)
at half throttle (1500), your P vallue is reduced by ~0

So how do you use this?

If you're getting oscillations starting at say 3/4 throttle, set tpa breakpoint = 1750 or lower (remember, this is assuming your throttle range is 1000-2000), and then slowly increase TPA until your oscillations are gone. Usually, you will want tpa breakpoint to start a little sooner then when your oscillations start so you'll want to experiment with the values to reduce/remove the oscillations.

In theory, this should allow you to bump your PIDs a tiny bit more because now you can focus your PID tune to how you fly and not have to worry about bringing it down some to compensate for high throttle oscillations.
