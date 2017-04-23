# Airplane Autotune instructions

Airplane PIFF autotune is inspired by ArduPilot Plane firmware.

Getting a good set of roll/pitch/yaw PIFF parameters for your aircraft is essential for stable flight. To help with this it is highly recommended that you use the AUTOTUNE system described below.

## What AUTOTUNE does

The AUTOTUNE mode is a flight mode that acts on top of normal ANGLE/HORIZON/ACRO mode and uses changes in flight attitude input by the pilot to learn the tuning values for roll, pitch and yaw tuning.

In general pilot needs to activate AUTOTUNE mode while in the air and then fly the plane for a few minutes. While flying the pilot needs to input as many sharp attitude changes as possible so that the autotune code can learn how the aircraft responds and figure out PIFF gains.

## Before flying with AUTOTUNE

Before taking off you need to set up a few parameters for your airplane:

parameter | explanation
--------- | -----------
roll_rate | Maximum roll rate limit for your ariplane. Must not exceed physical limit of your plane
pitch_rate | Maximum pitch rate limit for your ariplane. Must not exceed physical limit of your plane
yaw_rate | Maximum yaw rate limit for your ariplane. Must not exceed physical limit of your plane
fw_p_level | Self-leveling strength. Bigger value means sharper response
fw_i_level | Self-leveling filtering. Usual value for airplanes is 1-5 Hz
max_angle_inclination_rll | Maximum roll angle in [0.1 deg] units
max_angle_inclination_pit | Maximum pitch angle in [0.1 deg] units
tpa_breakpoint | Cruise throttle (expected throttle that you would be flying most of the time)
tpa_rate | Amount of TPS curve to apply (usually should be in range 50-80 for most airplanes)

For most hobby-sized airplanes roll/pitch rate limits should be in range 70-120 deg/s (7-12 for `roll_rate` and `pitch_rate` values). Small and agile flying wings can reach 180-200 deg/s.

Other things to check:

* It's highly recommended that you fly in PASTHROUGH and trim your servo midpoints for stable flight
* Make sure you have center of gravity according to manual to your aircraft
* Check that your failsafe activates correctly (test on the ground with propeller off for safety)

## Flying in AUTOTUNE

Once you are all setup you can take off normally and switch to AUTOTUNE mode once you have gained altitude.

When you engage AUTOTUNE mode a few things will happen:

* The autotune system will immediately setup some default values for your roll, pitch and yaw P and I gains
* The autotune system will monitor requested roll and pitch rates (as determined by your transmitter stick movements). When the demanded roll or pitch rate exceeds certain threshold the autotune system will use the response of the aircraft to learn roll or pitch tuning values
* Every 5 seconds the autotune system will store the snapshot of parameters you have. When you switch out of AUTOTUNE mode the last remembered parameters are restored
* You may find the plane is quite sluggish when you first enter AUTOTUNE. You will find that as the tune progresses this will get better. Make sure your flight area has plenty of room for slow large-radius turns.
* Don't land in AUTOTUNE mode - during landing airplane doesn't reach it full performance which may be read by autotune system as insufficient gains.

The key to a successful autotune is to input rapid movements with the transmitter sticks. You should only do one of either roll or pitch at a time, and you should move the stick rapidly to the maximum deflection.

## Don't stop too early

The more you fly the better it will get. Let autotune analyze how your airplane behaves and figure decent tune for you. Once you feel that airplane is flying good in AUTOTUNE - keep flying well past that point to finalize the tune.

## Completing the tune

Once you have tuned reasonable PIFF parameters with AUTOTUNE you should complete the tune by switching out of AUTOTUNE to ANGLE or PASTHROUGH and landing the airplane.

Note that AUTOTUNE mode doesn't automatically save parameters to EEPROM. You need to disarm and issue a [stick command](Controls.md) to save configuration parameters.
