# Autotune

Autotune helps to automatically tune your multirotor.

WARNING: Autotune is an experimental feature.  Currently enough feedback has been gathered and we do not recommend that anyone uses it until this warning is removed.  Autotune may be replaced by G-Tune, Please see https://github.com/cleanflight/cleanflight/pull/568 for details.

## Configuration.

Autotune only works in HORIZON or ANGLE mode, before using auto-tune it's best you setup so there is as little drift as possible.
Autotuning is best on a full battery in good flying conditions, i.e. no or minimal wind. Autotune does not support pid_controller 2  or higher (pid_controller 0 is the Cleanflight default, pid_controller 1 will work for autotune as well).

Configure a two position switch on your transmitter to activate the AUTOTUNE mode. Autotune may be done in ether one of the both only, HORIZON or  ANGLE mode (will then apply on both modes). 


## Using autotuning

Turn off the autotune switch.  If the autotune switch is on while not armed the warning LED will flash and you cannot arm.

1. Launch the multirotor.

1. Phase 1: ROLL/YAW autotune. 
Turn on/hold the autotune switch on your transmitter for approx 5 seconds.  You can observe roll left/right while a beep code sounds on the beeper, when turning off the autotune switch, PID settings will have been updated for ROLL and YAW.

1. Stay in air and re-align your copter for the following PITCH/YAW autotune.

1. Phase 2: PITCH/YAW autotune.
Turn on/hold the switch again for approx 5 seconds.  You can observe pitch forwards/backwards while a beep code sounds on the beeper, when turning off the autotune switch, PID settings will have been updated for PITCH and YAW.

1. Keep flying and see if it's better.  If it's worse then while still armed flip the switch again to restore previous PIDs that were present prior to arming.  You can do this while still flying or after landing.

1. Land & disarm.  If desired you may verify results via an app while battery power still on. Cutting the power will lose the new unsaved PIDs.

1. If you're happy with the PIDs then disarm (but leave the battery still on).

1. Flip the autotune switch again (copter still under battery power) to save all settings. 
A beeper will sound indicating the settings are saved.

1. Then flip it back (so you can arm again).


# References

* Brad Quick for the initial Autotune algorithm in BradWii.
