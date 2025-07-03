Disclaimer: 
	Street League Spec, its partners, and the authors of this software are not responsible for injury or damage to property resulting from the use of this software. Use at your own risk

What is the RPM limiter?
	The RPM limiter seeks to better equalize the performance of drone motors for spec racing purposes. A PID loop is used to limit the drone's maximum rpm to 13k rpm at 100% throttle. The limiter also linearizes the rpm of the motors such that, for example, 50% throttle will result in 50% of 13k rpm, 25% will result in 25% of the max throttle, etc. This means that battery sag will have very little effect on the rpm of the motors throughout the flight, and you will not feel it the same way you typically do. Please ensure voltage alarms configured on your drone.

IMPORTANT - Before/after flashing:
	Before flashing, click auto-detect in the firmware tab in betaflight or select your FC from the dropdown
		- This tells Betaflight which custom defaults it should apply to your quad after flashing.
	When you connect for the first time, you should be prompted to apply custom defaults. You MUST do this or your FC may not work properly.
	If you are not prompted to apply custom defaults, use the custom defaults found on the Betaflight's github 
		- https://github.com/betaflight/unified-targets/tree/master/configs/default

Which HEX do I use?
	- connect your fc to betaflight
	- open the cli tab and type "version"
	- look for board_name: (YOUR BOARD NAME HERE)
	- this will show you which board you should flash
	- if in doubt, check what target your FC's manufacterur recommends in their user manual
	- If you have any questions, please reach out on our discord: https://discord.gg/C4HHYccaqk

How do I activate the boost?
	- We've hijacked the beeper mode in betaflight in order to enable boost without any complicated cli commands
		- Simply open the modes tab and apply an aux channel and range to the beeper mode
	- We also recommend enabling the boost bar in your OSD so you can keep track of your boost usage
		- Open the OSD tab and enable "Battery usage" and leave the drop down on "Graphical remaining"

Optional CLI Commands:
	- if you are having issues launching your drone (E.G. low power when you try to take off), consider lowering your moter idle speed.
		- You can also try the cli command "set rpm_limiter_idle_rpm = 14"
		- This sets the expected idle rpm / 100. For street league a good starting place is 1400 RPM, which is `set rpm_limiter_idle_rpm = 14`.
		THE QUAD WILL SPIN UP TO THIS RPM WHEN ARMED, SO DO NOT SET TO ANY VALUE ABOVE 50!
		

Blackbox setup:
	You can view the rpm averaged accross the 4 motors as well as the P term, I term and D term in the blackbox. Simply `set debug_mode = RPM_LIMITER` in the cli. The values logged as a result are as follows in this order
		Debug 0. Average RPM
		Debug 1. RPM error: Difference between desired RPM limit and smoothed average rpm. RPM limit is rpm_limiter_rpm_limit for linearization off and rpm_limiter_rpm_limit*throttle for linearization on. Positive means overspeed, negative means underspeed
		Debug 2. I term (positive means term is pulling the throttle down)
		Debug 3. D term (positive means term is pulling the throttle down)

Am I spec yet?
	Once you have flashed the HEX, your FC is spec! If you have any issues, please verify they don't also exist on stock betaflight. Then report them here. 
		- https://github.com/StreetLeagueSpec/betaflight