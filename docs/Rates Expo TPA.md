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


