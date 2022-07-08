# Failsafe

If the radio link is lost, or the receiver fails or becomes disconnected, the pilot will have no control over their aircraft.  

Betaflight provides a failsafe system to safely manage this potential hazard.

**_It is vitally important to check that your failsafe system is working properly before flying!_**

There are two types of failsafe:

1. Receiver based failsafe
2. Flight controller based failsafe

__Receiver based failsafe__ is where your transmitter configures the receiver to output specific signals on specific channels if your receiver loses signal. The receiver sends 'normal-looking' data packets to the flight controller, containing the exact values that you configured, in the radio, for throttle and other channels. Normally these are configured so that the aircraft either cuts motors and falls, or descends in a controlled manner.  See your receiver's documentation for this method.  The flight controller will be unaware that anything bad is happening, and you will not see anything in the OSD.  

__Flight controller based failsafe__ is where the flight controller attempts to detect signal loss by analysing the received data packets, then responding according to failsafe settings made in the Configurator, and provides some indication of what is going on in the OSD and in the logs.  There are three phases:

- signal validation
- failsafe stage 1
- failsafe stage 2

After basic signal validation fails, the flight controller holds the last known good values for 300ms.  It then enters __Failsafe Stage 1__, the 'Guard' period, and applies your specified __Channel Fallback Settings__ for the `failsafe_delay` period (1.5s by default) counted from the time of the last valid packet.  If any valid data arrives while in Stage 1, the flight controller will respond to it immediately, and the failsafe process stops.  

If the failsafe duration exceeds the __Failsafe Stage 1__ or "Guard" period, the flight controller enters __Failsafe Stage 2__.  This either immediately disarms and drops, or enters a customisable Landing Mode or a GPS Rescue Mode.  After Stage 2 Failsafe, the signal must be consistently good for a period of time before control is returned to the pilot.

Flight controller based failsafe relies, for its triggering signal, on the absence of valid data from the radio receiver.  If the receiver is configured to keep sending 'normal' packets on signal loss, the flight controller will never enter 'flight controller failsafe'.  Hence, it is essential that the receiver is configured to send __nothing__ (no packets at all) on signal loss.  

Flight controller failsafe will work if your receiver signal wires come loose, break, or your receiver malfunctions in a way the receiver itself cannot detect.  

A transmitter switch may be configured to immediately activate failsafe.  This is useful for field testing the failsafe system and as a **_`PANIC`_** switch when you lose orientation.

## Flight controller failsafe system 

This system continuously monitors the integrity of the radio link, and has two stages that apply sequentially after signal loss is confirmed.

### Signal Loss

__Signal loss__ means:
- __no incoming data packets__, or __failsafe mode__ or __frame dropped__ packets for more than 100ms, or
- __*invalid pulse length*__ data on any flight channel for more than 300ms.

Once signal loss is detected, the values on the bad channels, or on all channels for total packet loss, will be held at their last received value for 300ms from the last known good data, or until signal returns.

`RXLOSS` should be displayed in the warnings field of the OSD 100ms after the last valid packet.  This is an 'early warning' of significant packet loss - an indicator that the link is in a bad way.

When failsafe is initiated by a failsafe switch that has been configured to to enter Stage 1, the flight channels (Roll, Pitch, Yaw and Throttle), but not the auxiliary channels, are immediately set to Stage 1 values, without any delay.

### Stage 1 Failsafe

__Stage 1__ applies fixed values after confirmed signal loss.  

__The default Stage 1 duration__ is one second, with a minimum of 200ms.  The timer for initiation of stage 1 is activated when 100ms has elapsed since the last good packet, or 300ms after any persistently bad flight channel, but it starts timing down to failsafe from the time of the last good packet.  The Stage 1 duration may be customised via the "Guard time for stage 2 activation" parameter in Configurator (`failsafe_delay` in the CLI).  It starts at the time of the last good packet.  The units are tenths of a second.  

__During Stage 1 failsafe__, all stick positions are set to the fixed 'fallback' values as specified in the 'Channel fallback settings' panel in Configurator, after the 300ms 'hold last good values' period expires.  When initiated by a switch, the aux channels remain active, and there is no hold period - the effect is immediate  Default behaviour is to center the sticks, set throttle stick to zero (causing the motors to idle), and hold the existing switch positions.  These values may be be customised in Configurator or with the CLI command `rxfail` (see [rxfail](Rx.md#rx-loss-configuration) section in rx documentation) or v.

The pilot may choose to active Level mode, using an Aux channel, so that it becomes active soon as Stage 1 commences.

The flight mode field in the OSD does *not* show failsafe with `!FS!` during stage 1, but `RXLOSS` will show in the warnings section after the 100ms basic signal validation period, to provide early warning of a bad link or impending failsafe.

__If signal returns during either the hold or Stage 1 period__, control is immediately returned to the pilot, the failsafe system resets.  `RXLOSS` should immediately disappear from the OSD.

> Note: the PID system remains active in Stage 1.  

__Stage 1 can also be activated by a transmitter switch__.  The switch should be configured (using Modes) to enable failsafe, and the `failsafe_switch_mode` should be set to `STAGE1`.

The switch emulates a signal loss, but there are some differences.

The main difference is that channels go immediately to Stage 1 values without waiting for any hold or signal evaluation periods.

While in Stage 1, returning the switch to the OFF position will restore normal control immediately, just like re-gaining signal during Stage 1.  

Note that if the switch is held ON for longer than the `failsafe_delay` period, the flight controller will enter Stage 2 (see below), and, depending on how Stage 2 is configured, may immediately drop and disarm, from which recovery will not be immediate.

With switch-initiated failsafe, the auxiliary channels must remain responsive so that the pilot can exit failsafe with the switch  Hence they are not set 'Channel fallback settings' to the Aux channels.  This is different from a true 'signal lost' type failsafe, where the Aux channels are fixed, according to the Guard period settings.


### Stage 2 Failsafe

__Stage 2 Failsafe__ is entered when your craft is __armed__ and __stage 1__ persists longer then the configured Stage 1, or 'guard' time (`failsafe_delay`), or, by switch.  

During Stage 2, all channels will remain at the fallback settings, unless overruled by the chosen Stage 2 procedure (the settings of the `failsafe_procedure`).  

> During Stage 2,`!FS!` will be shown in the Flight Mode field of the OSD.

Entering Stage 2 is not possible until 5 seconds after the flight controller boots up.  This is to prevent unwanted activation, as in the case of TX/RX gear with long bind procedures, before the RX sends out valid data.

Stage 2 Failsafe can be activated directly, and immediately, with a transmitter failsafe switch where the switch behaviour is set to `STAGE2` (`failsafe_switch_mode` in the CLI).  

Stage 2 will also be activated if the switch is set to Stage 1 and it is held for longer than the Stage 1 time (the Stage 2 'guard time').

When the flight contoller enters Stage 2, it implements one of three (actually, four) possible Stage 2 failsafe procedures::

* __Drop__, the default, causing immediate disarm and motor stop.
* __Landing Mode__, where the sticks are centered, throttle is held at a defined value, and the aux channels are set as configured for Stage 1 (which could include configuring an aux channel to enable Level mode).  These settings will apply for the Landing Time (`failsafe_off_delay` period), which defaults to 1 second, but can be longer.  Landing mode can be hazardous, since the motors and PIDs are active, but you cannot control where the quad goes, and there is no way to make the motors stop if you crash.  If the machine crashes and the props get stuck, they can burn out.
* __GPS Rescue__, where an appropriately configured machine will transfer stick and throttle control to the GPS controller, so that the quad should try to fly towards its starting point. 
* __Just Disarm__, a 'fourth' internal mode, which applies if the throttle has been held low for at least 10 seconds before entering Stage 2 (unless the mode is set to GPS Return).  This can sometimes cause confusion when testing failsafe - always test with throttle up at some point before the test.  Its primary purpose is to force a disarm if the user powers down their radio after landing, but has forgotten to remove the lipo.  This prevents the quad entering landing mode, for example, and spinning the props up by itself

At the end of the stage 2 procedure, the flight controller will disarm.  The word `FAILSAFE` will alternate with `RXLOSS` in the warnings field.  Arming will be blocked until the signal from the receiver is restored for more than the `failsafe_recovery_delay` period.  This period is the same regardless of the Stage 2 mode.

Control will be returned to the pilot:

* in Landing mode, when the RC signal has recovered for longer than the `failsafe_recovery_delay` period, or
* in GPS Rescue mode, when the link has returned for `failsafe_recovery_delay` and the pilot has moved the sticks more than 30 degrees out from centre.
* a transmitter failsafe Stage 2 switch that was set to ON is turned OFF (unless the `failsafe_switch_mode` is _not_ set to `KILL`).

__There is no way to instantly recover from a Drop or Just Disarm outcome__.  The quad will disarm and fall with zero motor power.  The pilot must re-arm, after the `failsafe_recovery_delay` period, to regain control.  The usual arming checks apply; arming switches must be off, throttle must be zero, and if the accelerometer is enabled, the quad must be within the 'small_ _angle' range.

### The default Drop mode

The default signal loss behaviour with one second of stage 1 'guard time' is:
- 300ms holding last values
- the next 1200ms at idle throttle with sticks centred (the default fallback settings)
- followed by disarm and drop at 1s.  

Recovery within the Stage 1 time is immediate, but if Stage 2 completes to disarm, the defaultRecovery time of one second will block arming for one full second.

The defaults are typically used by racers and park fliers.  

The `failsafe_delay`, or guard time, should be long enough that a brief Rx loss will be tolerated without leading to a disarm.  It should reflect the reliability of your link, vs how long it typically takes before you hit the ground.  If you crash within the guard time, the PIDs may cook your motors.  

Shorter guard times will stop the motors more quickly when signal is lost.  In practice the minimum is 200ms.  Any shorter and you are vulnerable to false failsafes from brief signal loss.

The default of 1.5s is a good guard time.

The `failsafe_recovery_delay` is how long the signal must be 'good' for before you're allowed to re-arm after being disarmed by the Drop.  By default this is one second.  For many setups this time can be a lot shorter.  However, some radio links can be erratic when they recover, so don't make it too short without first checking that the quad doesn't go bezerk when the signal recovers.

### Landing mode

This can be used to apply, after the Stage 1 or Guard period expires. a defined set of Aux switch settings and stick values for a set period of time.

Historically, this was used to enable Level mode, and apply sufficient throttle for a gradual fall from a typical flight altitude.  It had a role when people would hover at a consistent altitude and not do much else, potentially minimising the damage from a crash from altitude.

However, it is s a potentially hazardous thing to do, and not generally recommended, because the quad will fall with active PIDs and will drift with the breeze, potentially landing almost anywhere.  The motors may be active when it hits the ground, and could burn them out, and the quad could land on top of people with throttle on and fully active PIDs.

We do not recommend this anymore.

You will regain normal flight control during landing mode after signal restores for more than the `failsafe_recovery_delay` period.

You will regain the ability to re-arm after landing mode terminates and disarms, after the signal has been restored for more than the `failsafe_recovery_delay` period.

__Configuring Landing Mode.__

1.  Enable Landing Mode as opposed to Drop as the failsafe procedure
2.  Set `failsafe_off_delay` to an appropriate value based on how high you fly (how long you think it will take to land at the set throttle value).
3.  Set `failsafe_throttle` to a value that allows the aircraft to descend at approximately one meter per second (default is 1000 which should be throttle off).

The behaviour with default one second of stage 1 'guard time' and a 10s Landing time is:
- 300ms holding last values
- the next 1,200ms at idle throttle with sticks centred (the default fallback settings)
- 10s of landing throttle
- your Stage 1 Aux switch values applied

### GPS Return mode

The full details of GPS return are covered in the wiki, and elsewhere.

You will regain during GPS Return mode only after signal restores for more than the `failsafe_recovery_delay` period AND you move the sticks more than 30 degrees out from centre.  

You will regain the ability to re-arm after GPS Return terminates and disarms, after the signal has been restored for more than the `failsafe_recovery_delay` period.

### "Just Drop" mode

This is an 'invisible' mode that is always present.

It is intended to 'catch' the possibility that the pilot has landed, forgotten to disarm, and powered off their transmitter.  This would result in a failsafe, and if landing mode was active, the motors could spin up.

"Just Drop" looks at the throttle position, and if it has been down for 10s before turning the transmitter off, the failsafe system will immediately disarm the quad, and not enter landing mode.  This protects the pilot, so long as they have throttle low for 10s before switching the radio off.


### General Stage 2 SAFETY Considerations

* The failsafe system will activate and de-activate without regard to the current throttle or stick position. When the failsafe intervention is aborted (RC signal restored/failsafe switch set to OFF) the current throttle and stick positions will be applied to the craft!

* If you land and turn your transmitter off before powering down your quad, it could be on the ground with motors stopped, and those motors and props could spin again.  Take care when using `Landing Mode`, particularly with the `MOTOR_STOP` feature. **The props may spin up without warning**, when failsafe activates in Landing mode!
* In 4.3 re-arming is possible, after a failsafe, without needing to power cycle the quad, so that if you crash after failsafe in an inaccessible area, you can perhaps get close enough to regain signal and fly out.



## Failsafe Settings

Failsafe delays are configured in 0.1 second steps.
1 step = 0.1sec
1 second = 10 steps

### `failsafe_delay`

Guard time, or Failsafe Stage 1 period; the time before for failsafe Stage 2 activation after a lost signal.  This is the amount of time the flight controller waits, after a signal loss, before activating Stage 2 failsafe.

### `failsafe_off_delay`

The time from when Landing Mode activates to when the motors finally turn off.  Throttle will be at 'failsafe_throttle' for this period of time.  If you fly at higher altitudes you may need more time to descend safely.

### `failsafe_throttle`

Throttle level used for landing.  Specify a value that causes the aircraft to descend at about 1M/sec (relatively slowly). Default is set to 1000 which should correspond to throttle off.

### `failsafe_switch_mode`

Configure the RC switched failsafe action. It can be one of:
* `STAGE1` - activates Stage 1 failsafe. RC controls are applied as configured for Stage 1 and the `failsafe_delay` guard time will have to elapse before Stage 2 is activated. This is useful if you want to simulate signal loss failsafe behavior.  Note that there are some differences.  There is no 300ms 'hold' time; the flight channels (RPY&T) go directly to Stage 1 settings the moment the switch goes ON.  Also, the Auxiliary channels remain active, and are not set to the Stage 1 values.  Recovery of signal immediately restores full pilot control.
* `STAGE2` - skips Stage 1 and immediately activates the selected Stage 2 procedure. Useful if you want to assign instant auto-landing, GPS Return, or Drop, to a switch.
* `KILL` - immediately disarms the quad with no delay.  Your craft will crash.  Note that a single glitch on the failsafe channel will immediately crash the quad.  Re-arming is blocked for 1 second after signal is restored.  A similar, but safer effect can be achieved by:
  * setting `failsafe_switch_mode` to `STAGE2`, `failsafe_procedure` to `DROP`, and `failsafe delay` to 2.  This gives a 200ms delay signal validation period, the shortest allowed, so that transient glitches on the failsafe channel will not falsely trigger a disarm.  Drop recovery can be made faster than Kill by configuring a short `failsafe_recovery_delay` time (which can be as short as 200ms).
  * using the arm switch. This does not introduce re-arming locking.

### `failsafe_throttle_low_delay`

Time throttle level must have been below 'min_throttle' to _only disarm_ instead of _full failsafe procedure_.

Use standard RX μs values.  See [Rx documentation](Rx.md).

### `failsafe_procedure`

* `DROP`: Just kill the motors and disarm (crash the craft). Re-arming is locked until RC link is available for at least 3 seconds and the arm switch (if used) is in the OFF position.
* `AUTO-LAND`: Enable an auto-level mode, center the flight sticks and set the throttle to a predefined value (`failsafe_throttle`) for a predefined time (`failsafe_off_delay`). This should allow the craft to come to a safer landing. Re-arming is locked until RC link is available for at least 30 seconds and the arm switch (if used) is in the OFF position.

### `failsafe_recovery_delay`

Time for a recovered signal to be considered valid.  In Failsafe Landing Mode, signal must be 'good' for at least this time for control to be returned to the pilot.  In GPS Return mode, this time is required before the stick inputs will be assessed for the restoration of control.

### `failsafe_stick_threshold`

For GPS Return, the angle in degrees that the sticks must be away from centre in order to return control to the pilot, assuming the signal has already recovered.

The idea is that as the quad flies home, the pilot leaves the sticks centred.  Once they get video back, and see that Rx signal has returned, moving the sticks allows the pilot to regain control at a time that suits them, rather than just immediately signal returns.

### `rx_min_usec`

The lowest channel value considered valid.  e.g. PWM/PPM pulse length 

### `rx_max_usec`

The highest channel value considered valid.  e.g. PWM/PPM pulse length 

The `rx_min_usec` and `rx_max_usec` settings helps detect when your RX stops sending any data, enters failsafe mode or when the RX looses signal.

With a Graupner GR-24 configured for PWM output with failsafe on channels 1-4 set to OFF in the receiver settings then this setting, at its default value, will allow failsafe to be activated.


## Testing failsafe

**Bench test of Stage 1 - _remove the props!_.**

1. Set the Guard, or stage 1 time, to 10s (100)
1. Set the Stage 2 procedure to Drop
1. Arm the craft and throttle up briefly
1. While wiggling the sticks, listen to the motors, and throttle off
1. Confirm that motors hold RPM for 300ms, then drop to idle and stay there
1. Note that RXLOSS appears in the OSD immediately the link is lost
1. Power up the transmitter, wiggle the pitch/roll stick while it re-establishes the link
1. Confirm that the motors start responding about 1s after the link is re-established

**Bench test the failsafe system before flying - _remove props while doing so_.**

1. Configure failsafe
1. Arm the craft and throttle up briefly
1. Either turn the transmitter off, or unplug the RX.
1. Observe motors spin at configured throttle setting for configured duration.
1. Observe motors turn off after configured duration.
1. Power cycle the FC.
1. Arm the craft.
1. Configure a long Stage 2 Landing Time (eg 200 or 20s) and enable Landing mode
1. Turn off transmitter or unplug RX.
1. Observe motors spin at configured Landing throttle setting.
1. Turn on TX or reconnect RX before Landing times out
1. Ensure that your switch positions don't now cause the craft to disarm (otherwise it would fall out of the sky on regained signal).
1. Observe that normal flight behavior is resumed.
1. Disarm.
 
**Field test of Landing Mode.**

1. Perform bench testing first!
1. On a calm day go to an unpopulated area away from buildings or test indoors in a safe controlled environment - e.g. inside a big net.
1. Arm the craft.
1. Hover over something soft (long grass, ferns, heather, foam, etc.).
1. Descend the craft and observe throttle position and record throttle value from your TX channel monitor.  Ideally 1500 should be hover. So your value should be less than 1500.
1. Stop, disarm.
1. Set failsafe throttle to the recorded value.
1. Arm, hover over something soft again.
1. Turn off TX (!)
1. Observe craft descends and motors continue to spin for the configured duration.
1. Observe FC disarms after the configured duration.
1. Remove flight battery.

If craft descends too quickly then increase failsafe throttle setting.

Ensure that the duration is long enough for your craft to land from the altitudes you normally fly at.

Using a configured transmitter switch to activate failsafe mode, instead of switching off your TX, is good primary testing method in addition to the above procedure.
