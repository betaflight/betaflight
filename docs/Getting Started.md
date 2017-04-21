# Getting Started

This is a step-by-step guide that can help a person that has never used INAV before set up a flight controller and the aircraft around it for flight. Basic RC knowledge is required, though. A total beginner should first familiarize themselves with concepts and techniques of RC before using this (e.g. basic controls, soldering, transmitter operation etc). One could use [RCGroups](http://www.rcgroups.com/forums/index.php) and/or [the Youtube show FliteTest](https://www.youtube.com/user/flitetest) for this.

DISCLAIMER: This documents is a work in progress. We cannot guarantee the safety or success of your project. At this point the document is only meant to be a helping guide, not an authoritative checklist of everything you should do to be safe and successful. Always exercise common sense, critical thinking and caution.

Read the [Introduction](Introduction.md) chapter for an overview of INAV and how the community works.

## Hardware

NOTE: Flight Controllers are typically equipped with accelerometers. These devices are sensitive to shocks. When the device is not yet installed  to an aircraft, it has very little mass by itself. If you drop or bump the controller, a big force will be applied on its accelerometers, which could potentially damage them. Bottom line: Handle the board very carefully until it's installed on an aircraft!

For an overview of the hardware INAV (hereby CF) can run on, see [Boards.md](Boards.md). For information about specific boards, see the board specific documentation.

* Assuming that you have a flight controller board (hereby FC) in hand, you should first read through the manual that it came with. You can skip the details about software setup, as we'll cover that here.

* Decide how you'll connect your receiver by reading the [receiver](Rx.md) chapter, and how many pins you need on the outputs (to connect ESCs and servos) by reading about [Mixers](Mixer.md).

* If you're interested in monitoring your flight battery with CF, see [Battery Monitoring](Battery.md).

* You may want audible feedback from your copter so skim through [Buzzer](Buzzer.md) and mark the pins that will be used.

* Do you want your RC Receiver's RSSI to be sent to the board? [The RSSI chapter](Rssi.md) explains how. You may or may not need to make an additional connection from your Receiver to the FC.

* Would you like to try using a GPS unit to get your aircraft to Loiter or Return-To-Launch? Take a look at the [GPS](Gps.md) and [GPS Tested Hardware](Gps.md#hardware) chapters.

* You may also want to read the [Serial](Serial.md) chapter to determine what extra devices (such as Blackbox, OSD, Telemetry) you may want to use, and how they should be connected.

* Now that you know what features you are going to use, and which pins you need, you can go ahead and solder them to your board, if they are not soldered already. Soldering only the pins required for the application may save weight and contribute to a neater looking setup, but if you need to use a new feature later you may have to unmount the board from the craft and solder missing pins, so plan accordingly.  Before soldering your FC please review a how-to-solder tutorial to avoid expensive mistakes, practice soldering on some scrap before soldering your FC.

* If you are going to use [Oneshot125](Oneshot.md), you may need to enable that on your ESCs using a jumper or flashing them with the latest stable firmware and enable Damped Light in their settings, if it's supported. Refer to the ESCs' documentation or online discussions to determine this.

## Software setup

Now that your board has pins on it, you are ready to connect it to your PC and flash it with CF. Install the Chromium browser or Google Chrome to your PC, if you don't have it already, add the [INAV Configurator](https://chrome.google.com/webstore/detail/inav-configurator/fmaidjmgkdkpafmbnmigkpdnpdhopgel) to it, and start it.

Then follow these instructions for [Installation](Installation.md) of the firmware to the FC.

## INAV Configuration

Your FC should now be running CF, and you should be able to connect to it using the Configurator. If that is not the case, please go back to the previous sections and follow the steps carefully.

<!--- This next paragraph should probably contain less info, as this info already exists in Configuration.md -->
Now, there are two ways to [configure CF](Configuration.md); via  the Configurator's tabs (in a "graphical" way, clicking through and selecting/changing values and tickboxes) and using the [Command Line Interface (CLI)](Cli.md). Some settings may only be configurable using the CLI and some settings are best configured using the GUI (particularly the ports settings, which aren't documented for the CLI as they're not human friendly).

* It is now a good time to setup your RC Receiver and Transmitter. Set the Tx so that it outputs at least 4 channels (Aileron, Elevator, Throttle, Rudder) but preferably more. E.g. you can set channels 5 and 6 to be controlled by 3-position switches, to be used later. Maybe set up EXPO on AIL/ELE/RUD, but you should know that it can also be done in CF's software later. If using RSSI over PPM or PWM, it's now time to configure your Rx to output it on a spare channel.

* Connect the Rx to the FC, and the FC to the PC. You may need to power the Rx through a BEC (its 5V rail - observe polarity!).

* On your PC, connect to the Configurator, and go to the first tab. Check that the board animation is moving properly when you move the actual board. Do an accelerometer calibration.

* Configuration tab: Select your aircraft configuration (e.g. Quad X), and go through each option in the tab to check if relevant for you.

  * E.g. you may want to enable ONESHOT125 for Oneshot-capable ESCs.
  * You may need RX_PPM if you're using an RC Receiver with PPM output etc.
  * If planning to use the battery measurement feature of the FC, check VBAT under Battery Voltage.
  * If using analog RSSI, enable that under RSSI. Do not enable this setting if using RSSI injected into the PPM stream.
  * Motors will spin by default when the FC is armed. If you don't like this, enable MOTOR_STOP.
  * Also, adjust the minimum, middle and maximum throttle according to these guidelines:

    * Minimum Throttle - Set this to the minimum throttle level that enables all motors to start reliably. If this is too low, some motors may not start properly after spindowns, which can cause loss of stability and control. A typical value would be 1100.
    * Middle Throttle - The throttle level for middle stick position. Many radios use 1500, but some (e.g. Futaba) may use 1520 or other values.
    * Maximum Throttle - The maximum throttle level that the ESCs should receive. A typical value would be 2000.
    * Minimum Command - This is the "idle" signal level that will be sent to the ESCs when the craft is disarmed, which should not cause the motors to spin. A typical value would be 1000.
  * Finally, click Save and Reboot.

* Receiver tab:
    * Check that the channel inputs move according to your Tx inputs.
    * Check that the Channel map is correct along with the RSSI Channel, if you use that.
    * Verify the range of each channel goes from ~1000 to ~2000.  See also [controls](Controls.md). and `rx_min_usec` and `rx_max_usec`.
    * You can also set EXPO here instead of your Tx.
    * Click Save!
* Modes tab: Setup the desired modes. See the [modes](Modes.md) chapter for what each mode does, but for the beginning you mainly need HORIZON, if any.

* Before finishing this section, you should calibrate the ESCs, install the FC to the frame, and connect the RSSI cable, buzzer and battery if you have chosen to use those.

## Final testing and safety

It's important that you have configured CF properly, so that your aircraft does not fly away, or even worse fly into property and people! This is an important step that you should NOT postpone until after your maiden flight. Please do this now, before you head off to the flying field.

* First, learn how to arm your FC, and about other [controls](Controls.md).
* Next up, setup [Failsafe](Failsafe.md). Take your time, do it properly.
* Now, on the bench, without props, test that failsafe works properly, according to the above doc.
* Additionally, test the effect of AIL/ELE input of your Tx. Is the aircraft responding properly? Do the same for RUD input.
* Test the direction of AIL/ELE auto correction. Raise throttle at 30% (no blades!); when you tilt the aircraft, do the motors try to compensate momentarily? This should simulate random wind forces that the FC should counteract
* Test the direction of AIL/ELE auto correction in HORIZON mode. With throttle at 30%, if you tilt the aircraft so that one motor is lowered towards the ground, does it spin up and stay at high RPM until you level it off again? This tests the auto-leveling direction.

If one of these tests fail, do not attempt to fly, but go back to the configuration phase instead. Some channel may need reversing, or the direction of the board is wrong.


## Using it (AKA: Flying)

Go to the field, turn Tx on, place aircraft on the ground, connect flight battery and wait. Arm and fly. Good luck!

## Advanced Matters

Some advanced configurations and features are documented in the following pages, but have not been touched-upon earlier:

* [Profiles](Profiles.md)
* [PID tuning](PID%20tuning.md)
* [In-flight Adjustments](Inflight%20Adjustments.md)
* [Blackbox logging](Blackbox.md)
* [Using a Sonar](Sonar.md)
* [Spektrum Bind](Spektrum%20bind.md)
* [Telemetry](Telemetry.md)
* [Using a Display](Display.md)
* [Using a LED strip](LedStrip.md)
