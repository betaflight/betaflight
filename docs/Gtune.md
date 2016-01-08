# G-Tune instructions.

The algorithm has been originally developed by Mohammad Hefny (mohammad.hefny@gmail.com):

> http://technicaladventure.blogspot.com/2014/06/zero-pids-tuner-for-multirotors.html
> http://diydrones.com/profiles/blogs/zero-pid-tunes-for-multirotors-part-2
> http://www.multiwii.com/forum/viewtopic.php?f=8&t=5190

The G-Tune functionality for Cleanflight is ported from the Harakiri firmware.

## Safety preamble: _Use at your own risk_

The implementation you have here is quite different and just for adjusting the P values of ROLL/PITCH/YAW in Acro mode.
When flying in Acro mode (yaw tune in other modes possible as well - see below) you can activate G-Tune with an AUX box (switch) while the copter is armed.

It will start tuning the wanted / possible axis (see below) in a predefined range (see below).

After activation you will probably notice nothing! That means G-Tune will not start shaking your copter, you will have to do it (or simply fly and let it work).

The G-Tune is based on the gyro error so it is only active when you give no RC input (that would be an additional error). So if you just roll only pitch and yaw are tuned. If you stop rolling G-Tune will wait ca. 450ms to let the axis settle and then start tuning that axis again. All axis are treated independently.

The easiest way to tune all axis at once is to do some air-jumps with the copter in Acro (RC centered and G-Tune activated... of course..).

You can set a too high P for the axis as default in the GUI, when the copter starts shaking the wobbles will be detected and P tuned down (be careful with the strength setting though - see below).

Yaw tune is disabled in any copter with less than 4 motors (like tricopters).

G-Tune in Horizon or Level mode will just affect Yaw axis (if more than 3 motors...)

You will see the results in the GUI - the tuning results will only be saved if you enable G-Tune mode while the copter is disarmed and G-Tune was used before when armed. You also can save the configuration in an alternative way (like hitting save button in the GUI, casting an eepromwrite with trimming, acc calibration etc.)

TPA and G-Tune: It is not tested and will most likely not result into something good. However G-Tune might be able to replace TPA for you.

A typical use may go in this order:

1. Arm
2. Enable G-tune
3. Lift off slowly, avoid stick inputs (Roll, Pitch / Yaw).
4. Eventually the copter should fly well. Perhaps do a few throttle punch outs and fly around a bit. Take note if each punch out seems to become smoother with less oscillation and the overall flight performance.
5. Disable G-tune
6. Land 
7. Disarm, but don’t power off. 
8. If these are desired results then either a) Connect cleanflight configurator review and save the configuration. or b) Enable G-Tune again to save settings. 
9. Power off. 

If the results are not desired look into changing the parameters as shown below and try again.

Some other notes and instructions can be found here:

> http://www.rcgroups.com/forums/showpost.php?p=31321635&postcount=6160
> http://www.rcgroups.com/forums/showpost.php?p=31525114&postcount=7150

## Parameters and their function:

    gtune_loP_rll        = 10  [0..200] Lower limit of ROLL P during G-Tune.  Note "10" means "1.0" in the GUI.
    gtune_loP_ptch       = 10  [0..200] Lower limit of PITCH P during G-Tune. Note "10" means "1.0" in the GUI.
    gtune_loP_yw         = 10  [0..200] Lower limit of YAW P during G-Tune.   Note "10" means "1.0" in the GUI.
    gtune_hiP_rll        = 100 [0..200] Higher limit of ROLL P during G-Tune. 0 Disables tuning for that axis.  Note "100" means "10.0" in the GUI.
    gtune_hiP_ptch       = 100 [0..200] Higher limit of PITCH P during G-Tune. 0 Disables tuning for that axis. Note "100" means "10.0" in the GUI.
    gtune_hiP_yw         = 100 [0..200] Higher limit of YAW P during G-Tune. 0 Disables tuning for that axis.   Note "100" means "10.0" in the GUI.
    gtune_pwr            = 0   [0..10] Strength of adjustment
    gtune_settle_time    = 450 [200..1000] Settle time in ms
    gtune_average_cycles = 16  [8..128] Number of looptime cycles used for gyro average calcullation

So you have lower and higher limits for each P for every axis. The preset range (GUI: 1.0 - 10.0) is quiet broad to represent most setups.

If you want tighter or more loose ranges change them here. gtune_loP_XXX can be configured lower than "10" that means a P of "1.0" in the GUI. So you can have "Zero P" but you may get sluggish initial control.

If you want to exclude one axis from the tuning you must set gtune_hiP_XXX to zero. Let's say you want to disable yaw tuning write in CLI `set gtune_hiP_yw = 0`. Note: The MultiWii Wiki advises you to trim the yaw axis on your transmitter. If you have done so (yaw not neutral on your RC) yaw tuning will be disabled.

You can adjust the strength of tuning by using `set gtune_pwr = N`. My small copter works fine with 0 and doesn't like a value of "3". My big copter likes "gtune_pwr = 5". It shifts the tuning to higher values and if too high can diminish the wobble blocking! So start with 0 (default). If you feel your resulting P is always too low for you, increase gtune_pwr. You will see it getting a little shaky if value is too high.
