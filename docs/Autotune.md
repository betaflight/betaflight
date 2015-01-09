# Autotune

Autotune helps to automatically tune your multirotor.

## Configuration.

Autotune only works in HORIZON or ANGLE mode, before using auto-tune it's best you setup so there is as little drift as possible.
Autotuning is best on a full battery in good flying conditions, i.e. no or minimal wind. Autotune does not support 
pid_controller 2 (pid_controller 0 is the Cleanflight default).

Configure a two position switch on your transmitter to activate the AUTOTUNE and (HORIZON or ANGLE) modes using the auxiliary configuration.
You may find a momentary switch more suitable than a toggle switch.


## Using autotuning

Turn off the autotune switch.  If the autotune switch is on while not armed the warning LED will flash and you cannot arm.

1. Launch the multirotor.

1. Turn on/hold the autotune switch on your transmitter. Observe roll left/right.  A beep code will sound on the beeper.

1. Turn off/release the switch while still flying to stop this phase of tuning. PID settings will have been updated for ROLL/YAW.

1. Turn on/hold the switch again. Observe pitch forwards/backwards.  A beep code will sound on the beeper.

1. Turn off/release the switch while still flying to stop this phase of tuning. PID settings will have been updated for PITCH/YAW.

1. PIDs are updated, fly and see if it's better.

1. If it's worse, flip the switch again to restore previous PIDs that were present prior to arming.

1. Land.

1. Verify results via an app with power still applied if desired. Cutting the power will lose the unsaved PIDs.

1. If you're happy with the PIDs then while disarmed flip the autotune switch again to save all settings then flip it back so you can arm again. A beeper will sound indicating the settings are saved.


# References

* Brad Quick for the initial Autotune algorithm in BradWii.
