## Important information when upgrading from Betaflight 3.2 to Betaflight 3.3

- To get optimal support for configuration and tuning of the firmware, please use the latest version of the Betaflight configurator (10.2.0 at the time of this release), available [here](https://github.com/betaflight/betaflight-configurator/releases);
- If you are using OpenTX and the Betaflight lua scripts, please also make sure to use the latest version of these, available [here](https://github.com/betaflight/betaflight-tx-lua-scripts/releases);
- This release is introducing Runaway Takeoff Prevention [#4935](https://github.com/betaflight/betaflight/pull/4935). This feature will prevent uncontrollable acceleration ('tasmanian devil') when the craft is armed with misconfigured motor outputs or props on the wrong way, by disarming the craft if such a configuration is detected. It will also cause disarms when throttling up quickly on the bench with props off. If you get unwanted disarms right after arming, use the parameters (`runaway_takeoff_deactivate_delay`, `runaway_takeoff_deactivate_throttle_percent`) to tune the function to work for you, and please report back your working configuration. See the [wiki article](https://github.com/betaflight/betaflight/wiki/Runaway-Takeoff-Prevention) for details.
- The orientation of the AK8963 magnetometer has been changed, to make it match the orientation it has when it comes integrated in the MPU9250 gyro / accelerometer / magnetometer chip. If you are using an external AK8963 magnetometer, check your orientation to make sure it is still correct. If not, use the `align_mag` to configure the correct orientation;
- More possible reasons for arming to be disabled have been added. Because of this, and because counting the 'arming disabled' beep indications was becoming difficult, the method of how the arming disabled reason is indicated has been changed. See the [wiki article](https://github.com/betaflight/betaflight/wiki/Arming-Sequence-&-Safety#arming-prevention) for details.
- Arming is now disabled when the flight controller is connected to the Betaflight configurator. This was added in order to keep users from accidentally arming their craft when testing their RX / mode switches with a battery connected. In order to enable arming for bench testing, go to the 'Motors' tab and enable the 'Motor test mode' switch (**REMOVE ALL PROPELLERS FIRST**);
- The parameter `sbus_inversion` has been changed into `serialrx_inverted`, and the way it is applied has been changed as follows: It now applies to all RX protocols, not just SBus, and instead of switching the UART to normal when off and inverted when on, 'off' now means that the port is set to whatever the default is for the selected protocol (i.e. inverted for SBus, not inverted for SUMD), and 'on' means that the port is inverted from default (i.e. not inverted for SBus, inverted for SUMD);
- The way that rates are configured has been changed [#4973](https://github.com/betaflight/betaflight/pull/4973). Betaflight now supports independent rates settings for roll / pitch / yaw. When updating from an older version of the firmware, make sure to convert your settings to the new parameters as follows: `rc_rate` becomes `roll_rc_rate` and `pitch_rc_rate`, `rc_rate_yaw` becomes `yaw_rc_rate`, `rc_expo` becomes `roll_expo` and `pitch_expo`, and `rc_expo_yaw` becomes `yaw_expo`. The same change also makes 'RaceFlight' type rate settings available. Set `rates_type = raceflight`. After this, 'rc_rate_<axis>' is RaceFlight 'rate' (scaled down by a factor of 10), `<axis>_expo` is RaceFlight 'expo', and `<axis>_srate` is RaceFlight 'acro+';
- The `DISABLE 3D` mode is now called `DISABLE / SWITCH 3D` [#5179](https://github.com/betaflight/betaflight/pull/5179). In default configuration it works in the same way it used to work before. With `3d_switched_mode = on`, 'switched 3d mode' is enabled: With the 3D switch on, the throttle goes from forward thrust idle (min) to forward thrust full (max), which is the same as with `3d_switched_mode = off`. With the 3D switch off, the throttle goes from reversed thrust idle (min) to reversed thrust full (max). This allows the pilot to fly 3d by using the full throttle range in normal / reversed position, by switching motor directions with the switch when flipping over;
- The `disarm_kill_switch` function that allowed (switch) disarming to be set up to be only possible on low throttle has been removed. There is no use case that requires it, and having it enabled introduces the safety risk of not being able to reliably disarm in an emergency;
- The SmartAudio protocol implementation has been updated to be fully compliant with the SmartAudio specification. First generation AKK / RDQ VTX devices have a bug in their SmartAudio protocol implementation, causing them to fail to work with the SmartAudio protocol as implemented by Betaflight 3.3 (and KISS / RaceFlight). In order to not leave the owners of these VTX devices stranded, a branch off the Betaflight 3.3 maintenance branch has been created with a fix for these devices in it. This branch will be updated for all 3.3 patch releases. If you own one of these first generation AKK / RDQ VTX devices and are affected by this bug, please go to [this website](https://ci.betaflight.tech/job/Betaflight%20Maintenance%203.3%20%28AKK%20-%20RDQ%20VTX%20Patch%29/lastSuccessfulBuild/artifact/obj/) to download a Betaflight 3.3 version that works with your VTX device. (Go back to this page to download the latest version whenever a new version of Betaflight 3.3 has been released.)

## Major features:

- Added support for the FrSky FPort protocol [#4158](https://github.com/betaflight/betaflight/pull/4158);
- Added Spektrum VTX control [#4434](https://github.com/betaflight/betaflight/pull/4434);
- Added CMS configuration over Spektrum telemetry [#4545](https://github.com/betaflight/betaflight/pull/4545);
- Added FrSky X SPI RX protocol [#4683](https://github.com/betaflight/betaflight/pull/4683);
- Added fast Biquad RC+FIR2 filter (optimised version of  Kalman gyro filter in #4890) [#4965](https://github.com/betaflight/betaflight/pull/4965);
- Added Runaway Takeoff Prevention (anti-taz) [#4935](https://github.com/betaflight/betaflight/pull/4935).


## Minor features:

- Added CMS power menu [#3724](https://github.com/betaflight/betaflight/pull/3724);
- Added support for FlySky SPI receiver [#4060](https://github.com/betaflight/betaflight/pull/4060);
- Added use TIM_UP and DMAR for all timer channels with Dshot ([#4073](https://github.com/betaflight/betaflight/pull/4073), [#4843](https://github.com/betaflight/betaflight/pull/4843), [#4852](https://github.com/betaflight/betaflight/pull/4852));
- Added '3D on a switch' mode [#4227](https://github.com/betaflight/betaflight/pull/4227);
- Added Dshot beacon activation to BEEPER_RX_LOST_LANDING [#4231](https://github.com/betaflight/betaflight/pull/4231);
- Added generic RunCam device protocol support [#4251](https://github.com/betaflight/betaflight/pull/4251), see the [wiki article](https://github.com/betaflight/betaflight/wiki/RunCam-Device-Protocol) for details;
- Added a reasonable default OSD layout [#4260](https://github.com/betaflight/betaflight/pull/4260);
- Added handling and display of date and time [#4289](https://github.com/betaflight/betaflight/pull/4289);
- Added MSP command to disable arming [#4320](https://github.com/betaflight/betaflight/pull/4320);
- Added support for Spektrum real RSSI from SRXL Rx and fake RSSI from both int and ext bound satellites [#4347](https://github.com/betaflight/betaflight/pull/4347);
- Changed after flight OSD statistics screen to only show when enabled [#4428](https://github.com/betaflight/betaflight/pull/4428);
- Added remaining time estimate based on flight used mAh rate to OSD ([#4487](https://github.com/betaflight/betaflight/pull/4487), [#4543](https://github.com/betaflight/betaflight/pull/4543), [#4618](https://github.com/betaflight/betaflight/pull/4618));
- Added setting of RSSI value with MSP [#4507](https://github.com/betaflight/betaflight/pull/4507);
- Improved SmartAudio update frequency (make it QuietAudio) [#4532](https://github.com/betaflight/betaflight/pull/4532);
- Updated PID calculations to use actual deltaT [#4556](https://github.com/betaflight/betaflight/pull/4556);
- Added AND logic to modes [#4722](https://github.com/betaflight/betaflight/pull/4722);
- Add TCM support to F7 [#4757](https://github.com/betaflight/betaflight/pull/4757);
- Added Benewake TFmini/TF02 rangefinder support  [#4793](https://github.com/betaflight/betaflight/pull/4793);
- Added selectable RaceFlight rates [#4973](https://github.com/betaflight/betaflight/pull/4973);
- Added KN (NRF24) SPI RX protocol [#4994](https://github.com/betaflight/betaflight/pull/4994);
- Added Spektrum VTX status via telemtry [#5081](https://github.com/betaflight/betaflight/pull/5081);
- Added PINIOBOX BOX to PINIO general purpose pin output mapper [#5110](https://github.com/betaflight/betaflight/pull/5110).
