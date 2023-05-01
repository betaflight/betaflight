Disclaimer: 
	Street League, its partners, and the authors of this software are not responsible for injury or damage to property resulting from the use of this software. 

What is the RPM limiter?
	The RPM limiter seeks to better equalize the performance of drone motors for spec racing purposes. A PID loop is used to limit the drone's maximum rpm to 13k rpm at 100% throttle. The limiter also linearizes the rpm of the motors such that, for example, 50% throttle will result in 50% of 13k rpm, 25% will result in 25% of the max throttle, etc. This means that battery sag will have very little effect on the rpm of the motors throughout the flight, and it won't be felt via the throttle. Please ensure voltage alarms are configured on your OSD or radio.

Which HEX do I use?
	- plug your flight controller into your PC
	- Open Betaflight
	- In the top right corner will be displayed a dropdown menu with the COM port the FC is using
	- A COM port label of "COM4 - Betaflight STM32F7x2" indicates the user should flash the STM32F7x2 HEX file.
		- Note: pancake board users have a seperate hex
	- If you have any questions, please reach out on our discord: https://discord.gg/C4HHYccaqk

IMPORTANT - After flashing:
	After flashing, ALWAYS select "Apply Custom Defaults" when connecting for the first time!!!
	If you are not prompted to apply custom defaults, you will need to make sure you either paste a dump from before flashing, or copy your custom defaults from here. 
	https://github.com/betaflight/unified-targets/tree/master/configs/default

