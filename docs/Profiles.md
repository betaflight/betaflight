# Profiles (aka PID Profiles)

A Profile is a set of performance-related configuration settings.  PID values, Filter settings, Anti Gravity, Crash Recovery, and more.  Currently, 3 Profiles are supported. The default Profile is profile 1 (`profile 0` in CLI).

Profiles offer another way to perform PID or Filter tuning (currently only Dterm filters).  Say that Profile 1 can have default settings you can trust to get in the air and not damage your motors.  But you want to try Dterm Bi-quad over PT1, increase P or decrease D values without sacrificing your day of flying.  Profiles are perfect for this, edit any Profile when you're at the bench and select the profile when you're in the field.  Quad not flying perfectly, merely change your profile back and continue flying.


## Changing profiles

Profiles can be selected using the GUI, CLI, OSD or stick combinations.  Once selected, changes to settings can be made and saved to the active Profile.  Also worth mentioning when selecting a Profile, that Profile will remain the active Profile even over power cycles.

Betaflight Configurator (GUI): In the PID Tuning Tab > Select a Profile using the drop-down menu.  Once a profile is chosen, that profile is activated for current use.  Any modifications made are saved when pressing Save button.  

OSD: Access CMS (Left Yaw + Pitch up) > Select `Profiles` > Change `PID Prof`.  

Stick combinations:  When disarmed use the following stick commands to select a Profile.  The status LED on your FC will flicker when changing the Profile.  

| Profile | Throttle | Yaw   | Pitch  | Roll   |
| ------- | -------- | ----- | ------ | ------ |
| 0       | Down     | Left  | Middle | Left   |
| 1       | Down     | Left  | Up     | Middle |
| 2       | Down     | Left  | Middle | Right  |

CLI: the `profile` command can be used to change the profile:
```
profile <index>
```
To save configuration changes:
```
save
```

# Rate Profiles

A Rate Profile is a set of Rate related settings. These profiles store the settings for Rate Type, RC Rate, Super Rate, Expo for Pitch, Roll, & Yaw.  Along with Throttle Expo and TPA settings.  

Pre-BetaFlight 3.2 Profiles and Rate Profiles are coupled together.  Each Profile had its own set of 3 Rate Profiles.  Allowing up to 9 Rate Profiles all together.
Betaflight 3.2 broke this link, giving us independent Profiles and Rate Profiles.  Selecting Rate Profiles are independent of Profiles now.  Allowing only 3 Rate Profiles.
BetaFlight 3.3 > the number of Rate Profiles increased from 3 to 6 Rate Profiles.  

## Changing Rate Profiles
Rate Profiles can be selected using a GUI, CLI, OSD, or AUX channel. Once selected (just like Profiles), changes to settings can be made and saved to the active Rate Profile.  Also, when changing our Rate Profile that profile will remain the active Profile even over power cycles, except when using the AUX channel option.  When BetaFlight boots the active profile is selected by the position of the AUX channel.

Betaflight Configurator (GUI): Click PID Tuning Tab > Select a Rate Profile using the drop-down menu.  Once selected, any changes you make to your settings and click Save will be store to the selected Rate Profile.  The action of selecting a Rate Profile also activates that rate profile for current use.

OSD: Access CMS (Left Yaw + Pitch up) > Select `Profiles` > Change `Rate Prof`.  Make sure to Save + Exit or Save + Reboot to save settings.

Aux Channel (In-flight Adjustments): Adjustments need to be configured first using the GUI.  You must have an available AUX channel configured on your radio.  
Enable Expert Mode in the Configurator > Click Adjustments
Click the slider for If enable.  Select the AUX # channel to use.
For the Range, Select the whole Range from 900 to 2100.  Select Rate Profile Selection from the drop-down menu.  Use Slot 1, or any Slot not used.  Via Channel is the same as the AUX channel used.  

CLI: the `rateprofile` command can be used to change the profile:
```
rateprofile <index>
```
To save configuration changes:
```
save
```

# Backing up Profiles and Rate Profiles
Currently creating a backup from the GUI only backs up the active Profile and Rate Profile.  So the best way to see and backup all Profiles is to use CLI.

## Using Diff and Dump to output only Profiles
Use these commands to view and back up the complete active profiles (including default settings): `dump profile`, & `dump rates`
In order to see all Profiles configured add the `all` variable: `dump all`, & `diff all`
Use these commands to view and back up the changes the active profiles -defaults settings: `diff profile`, & `diff rates`

Examples:

```
# diff rates

# rateprofile
rateprofile 0

set roll_rc_rate = 200
set pitch_rc_rate = 200
set yaw_rc_rate = 200
set roll_expo = 50
set pitch_expo = 50
set yaw_expo = 90

# dump rates

# rateprofile
rateprofile 0

set thr_mid = 50
set thr_expo = 0
set rates_type = BETAFLIGHT
set roll_rc_rate = 200
set pitch_rc_rate = 200
set yaw_rc_rate = 200
set roll_expo = 50
set pitch_expo = 50
set yaw_expo = 90
set roll_srate = 70
set pitch_srate = 70
set yaw_srate = 70
set tpa_rate = 10
set tpa_breakpoint = 1650
set throttle_limit_type = OFF
set throttle_limit_percent = 100

# diff profile

# profile
profile 0

set p_pitch = 40
set d_pitch = 26
set p_roll = 20
set d_roll = 13
set p_yaw = 80

# dump profile

# profile
profile 0

set dterm_lowpass_type = PT1
set dterm_lowpass_hz = 100
set dterm_lowpass2_hz = 200
set dterm_notch_hz = 0
set dterm_notch_cutoff = 160
set vbat_pid_gain = OFF
set pid_at_min_throttle = ON
set anti_gravity_mode = SMOOTH
set anti_gravity_threshold = 250
set anti_gravity_gain = 5000
set feedforward_transition = 0
set acc_limit_yaw = 100
set acc_limit = 0
set crash_dthreshold = 50
set crash_gthreshold = 400
set crash_setpoint_threshold = 350
set crash_time = 500
set crash_delay = 0
set crash_recovery_angle = 10
set crash_recovery_rate = 100
set crash_limit_yaw = 200
set crash_recovery = OFF
set iterm_rotation = ON
set smart_feedforward = OFF
set iterm_relax = OFF
set iterm_relax_type = GYRO
set iterm_relax_cutoff = 11
set iterm_windup = 40
set iterm_limit = 150
set pidsum_limit = 500
set pidsum_limit_yaw = 400
set yaw_lowpass_hz = 0
set throttle_boost = 5
set throttle_boost_cutoff = 15
set acro_trainer_angle_limit = 20
set acro_trainer_lookahead_ms = 50
set acro_trainer_debug_axis = ROLL
set acro_trainer_gain = 75
set p_pitch = 40
set i_pitch = 50
set d_pitch = 26
set f_pitch = 60
set p_roll = 20
set i_roll = 45
set d_roll = 13
set f_roll = 60
set p_yaw = 80
set i_yaw = 45
set d_yaw = 0
set f_yaw = 60
set p_level = 50
set i_level = 50
set d_level = 75
set level_limit = 55
set horizon_tilt_effect = 75
set horizon_tilt_expert_mode = OFF
set abs_control_gain = 0
set abs_control_limit = 90
set abs_control_error_limit = 20
```
```
# profile
profile 0

set dterm_lowpass_hz = 0
set dterm_lowpass2_hz = 0
set dterm_notch_cutoff = 0
set p_pitch = 40
set d_pitch = 26
set p_roll = 20
set d_roll = 13
set p_yaw = 80

# profile
profile 1

set dterm_lowpass_type = BIQUAD
set dterm_notch_cutoff = 0

# profile
profile 2

set dterm_notch_cutoff = 0

# restore original profile selection
profile 2

# rateprofile
rateprofile 0

set roll_rc_rate = 200
set pitch_rc_rate = 200
set yaw_rc_rate = 200
set roll_expo = 50
set pitch_expo = 50
set yaw_expo = 90

# rateprofile
rateprofile 1


# rateprofile
rateprofile 2


# rateprofile
rateprofile 3

set thr_expo = 25
set roll_rc_rate = 155
set tpa_breakpoint = 1500

# rateprofile
rateprofile 4


# rateprofile
rateprofile 5


# restore original rateprofile selection
rateprofile 3
```
