# Betaflight 4.4 Release Notes

## 1. Cloud build 

This is predominantly brought to you for convenience, and to ensure we can keep the 512kb flash targets (STM32F411 and STM32F722) alive and well for years to come. The cloud build system will allow you, the flyer, to select the features you want, and a **custom firmware** will be created for you.

Any issues with the cloud build process please check out the #cloud-build-issues Discord channel. Help us to help you, buy taking advantage of the new `Support` button in the Command Line Interface (CLI) tab in Configurator 10.9.0 (RC4 onwards). This will give us valuable information in trying to diagnose your issue.

**NOTE:** If you have something missing from your cloud build that you would normally expect to be present, e.g. a flash chip or barometer, the reason is because the board configuration (in unified targets) has not been updated with this information (either by the community, or the manufacturer).

If you do have something missing then we suggest flashing the `core` version. This will load all the hardware drivers, and then you can submit a `Support` level of detail from the CLI tab. In addition you can use the **custom defines** input box that is shown when "Expert Mode" is enabled. The `classic mode` option will load the all of the hardware drivers (but not all the features), and will allow you to boot the Flight Controller, and run the commands `status`, `flash_info`, `dump hardware` etc, in the CLI tab of configurator to find the information about the hardware you have.

For those missing a barometer: You can try any or all of `BARO_MS5611 BARO_SPI_MS5611 BARO_BMP280 BARO_SPI_BMP280 BARO_BMP388 BARO_SPI_BMP388 BARO_LPS BARO_SPI_LPS BARO_QMP6988 BARO_SPI_QMP6988 BARO_DPS310 BARO_SPI_DPS310 BARO_BMP085 BARO_2SMBP_02B BARO_SPI_2SMBP_02B` in the **custom defines** input box.

For those missing the flash chip: You can try any or all of `USE_FLASH_W25P16 USE_FLASH_W25Q128FV USE_FLASH_W25M02G USE_FLASH_W25N01G USE_FLASH_W25M` in the custom defines input box.

Thank you all for your patience and assistance in working through what boards have what hardware. Unfortunately we need our flyers to help crowd source this information - as there is such diverse hardware out there! 

Thanks to @blckmn for the implementation of the cloud build!

## 2. HD OSD

HD OSD is now supported and adds the following features. Note that not all HD Goggle/VTX combinations support all features, but hopefully will do so in time.

If the `OSD` option alone, or the `OSD (HD)` option is included in the build options, then HD support will be included.

To enable HD support select `VTX (MSP + Displayport)` on the ports tab for the UART to which the VTX is connected. The `MSP` option will automatically be selected.

![image](https://user-images.githubusercontent.com/11480839/212375683-0ac11ca1-9694-451d-a399-db910a72a062.png)

The OSD `HD` preview may be manually selected on the OSD tab along side the `PAL` and `NTSC` options however this is not necessary as the first time the HD Goggle/VTX system is powered the FC detects it and automatically applies the following settings, saves and reboots automatically. This only happens if the setting below are not already applied. This is a new feature, and may not be supported by the your HD system yet, so if the OSD tab does not automatically switch to HD, please enter the commands below manually.

```
set osd_displayport_device = MSP
set vcd_video_system = HD
```

The size of the canvas (visible number of columns/rows) defaults to 53x20 (compared to 30x16 for PAL and 30x13 for NTSC). The VTX is then able to adjust this by sending an MSP command to adjust the canvas size. WTFOS will set this to 60x22 for example. This will be automatic with no user interaction required. Should the goggle vendor decide to support alternate canvas sizes then they would be selected in the goggle menus, and the new canvas size communicated to Betaflight which would then adjust the usable number of OSD rows/columns accordingly.

Regardless of the canvas size the boot logo, armed message, stats, CMS menu etc. will be centred correctly.

If the canvas size is reduced, either by selecting the PAL/NTSC options or by the VTX sending a different canvas size, all OSD elements will be moved onto the available canvas so they can be repositioned using the configurator.

In addition to the usual OSD elements it is now possible, where supported by the Goggle/VTX vendor, to enable/disable and reposition the following OSD elements. If none are explicitly enabled then all those supported will be displayed at their default locations.

1. Goggle battery voltage
2. VTX voltage
3. Bit-rate
4. Delay
5. Distance
6. Video link quality
7. Goggle DVR icon
8. VTX DVR icon
9. VTX warnings
10. VTX temperature
11. Goggle fan speed

To make better use of the colour OSD capabilities of the HD systems, four fonts are now supported. The intent is that these be used to display text and other icons in **white**, **green**, **amber** and **red** corresponding to **normal**, **good**, **marginal**, **critical** conditions respectively. This will allow the  link quality text and icon to be display in red when **critical**, **amber** when **marginal**, and **green** when **good** for example.

To enable this feature set the `displayport_msp_fonts` array to select the font number (0-4) to use for each of **normal**, **good**, **marginal**, **critical** conditions respectively.

To enable all four fonts:

```
set displayport_msp_fonts = 0,1,2,3
```

To only use the the default (predominantly white) font:

```
set displayport_msp_fonts = 0,0,0,0
```

To display critial warnings in red, select the first font for **normal**, **good**, and **marginal** OSD elements and for **critical** OSD elements select the third font:

```
set displayport_msp_fonts = 0,0,0,3
```

Thanks goes to @SteveCEvans for these improvements!


## 3. Preset Favourites

This feature reduces the amount of search the users have to do in the presets tab. Configurator will remember the presets you are using, automatically marking them with the "star". Favorite presets will always appear first in the initial list and the search results. In combination with the fix that preselects the current firmware from the plugged in FC it allows users to completely avoid searching of the commonly used presets, and pick them right away. The UI "stars" are clickable, so users can manually add/remove favorite presets.
![image](https://user-images.githubusercontent.com/2925027/212130300-f67a5d82-dbc2-4726-9c07-b6aae0aa98ae.png)
Favorite presets are being remembered by it's path+name in the repo. So a favorite preset in one repository becomes automatically a favorite in another, if it's sharing the same name and path withing the repos.


## 4. GPS Return to Home enhancements

**GPS "Rescue" has been extensively revised and greatly improved.**  The quad should reliably return at the set speed, descend at an angle, land within a few metres of the home point, and disarm automatically on touch-down.  There are separate PID control elements for altitude and return velocity t home; the defaults work very well for 'typical' quads.  The system should initially be tested with a switch at reasonably close range and low altitude.  Setting up and testing GPS Rescue to provide a reliable return on the event of an RxLoss failsafe is a non-trivial task, but well worth the trouble.  

**We strongly recommend reading the [wiki entry](https://github.com/betaflight/betaflight/wiki/GPS-Rescue-for-4.4) and following the instructions there.**  

Remember that in any true failsafe the quad will always enter Failsafe Stage 1 phase for 1s (user-configurable) before initiating the Rescue.  You MUST set the Stage 1 behaviour NOT to DROP, or it will disarm and drop in Stage 1 and never enter GPS Rescue.  The safest option for Stage 1 is to configure Angle Mode on an aux switch, and set Stage 1 Failsafe to enable Angle Mode at a fixed hover / light climb throttle value with all other sticks forced to center.  When you get signal loss of more than 300ms, you'll enter Angle Mode, and the will start to level out.  That will give you a clear advance warning that you are getting signal breakup.  Alternatively, you can set Failsafe Stage 1 to hold all current values; the quad will then continue on the same path until Stage 1 Failsafe expires and the Rescue starts.

**Please disable the compass/magnetometer** unless:
- it has been fully calibrated and 
- you have confirmed, by logging, that the magnetometer heading values are noise free and reflect the true attitude of the quad.

**In most short flights, using the Baro provides a significant improvement in altitude control.**  Baro data is updated more frequently than GPS data, and often varies less.  The user can control how much trust they place on the Baro vs GPS altitude data.  Over long flights, and with some particular Baro units or installations, Baro drift can be more of a problem, and the GPS data should be trusted more.

There are three new, and very useful, GPS Rescue Debug modes.  One shows how accurately the quad tracks the requested altitude and the requested velocity.  The other two are used for tuning the GPS Rescue PIDs. If Mag is enabled, Mag information is automatically recorded.

Data acquisition from GPS hardware now uses the UBlox protocol, at 10hz, by default. NMEA mode has been improved and in some cases will run at 10hz.  We now can log Dilution Of Precision values for future improvements.

There are extensive changes to sanity checks, and in most cases the quad will attempt to land itself, rather than disarm, if necessary using only the Baro signal.  

**WARNING: ALWAYS CHECK that the Home Arrow points directly back towards home after takeoff!**  Sometimes, if you take off and spin around during arming, or immediately on takeoff, the quad's attitude information can become corrupted, and the Home Arrow can point the wrong way.  It's best to arm cleanly and fly away from Home in a straight line at reasonable speed immediately after takeoff.  Watch the Home Arrow carefully to ensure that it quickly points back to Home. If the Home Arrow points the wrong way when a failsafe occurs, the GPS Rescue will initially fly off in the wrong direction and in some cases you may lose the quad.

thanks to @ctzsnooze, @karatebrot, @haslinghuis
 

## 5. Other OSD improvements

**Option to show 'READY' in the OSD with a mode switch**
This is a niche improvement, intended for racing situations where all pilots video feeds are on one central screen.  
The pilot can flick a switch to indicate that they are ready to fly, and the word `READY` appears on their OSD.  
The race director can then tell if all pilots are ready by looking at the central screen.  
On arming, the `READY` text disappears.
For more info see [PR#11886](https://github.com/betaflight/betaflight/pull/11886) - thanks @jonmahoney15

**Craft and Pilot name now handled correctly**
The user can now configure their OSD to show either Craft or Pilot name, or both.
For more info see [PR#11391](https://github.com/betaflight/betaflight/pull/11391) - thanks @krasiyan

**PID profile and Rate profile names shown in OSD**

Thanks @qvasic


## 6. Support for extended DShot Telemetry

If the ESC supports it, we now can get per-motor temp, current and voltage via DSHot Telemetry.
For more info see [PR11694](https://github.com/betaflight/betaflight/pull/11694) - thanks @damosvil


## 7. Flight improvements

**Antigravity**

AntiGravity has been tweaked, resulting in greater stablility during rapid throttle changes, by:
- not applying AntiGravity P boost to yaw, preventing additional yaw wobbles during rapid throttle changes
- reducing the overall AntiGravity P boost to reduce the chance of unwanted P wobbles
- allows the relative P gain during AntiGravity to be set independently of the I boost, with `anti_gravity_p_gain`.  The default is 100, meaning 'normal'.  A lower number will give proportionally less P boost than default, for machines that get P wobbles readily during throttle boost, and vice versa.
- further optimisations of the timing of the boost effect.  A PT2 filter is used at 6Hz by default.  The value can be tweaked to focus the boost when it is needed most by changing the `anti_gravity_cutoff` value.  Higher values will make the boost a bit stronger but of shorter duration.  Lower values may work better for less responsive builds or where a more prolonged boost is needed.
- applying iTerm windup constraints to antigravity-induced iTerm increases (thanks tbolin)
- removing the old 'step' mode, which wasn't working as intended
The AntiGravity value is now directly related to the amount of iTerm boost.  The default is now 80, meaning 8x iTerm boost on fast throttle cuts.
**Do not increase AntiGravity unless you've made a log and confirm that there is no P wobble.  Use the debug.**
See [PR#11679](https://github.com/betaflight/betaflight/pull/11679) for more information - thanks @ctzsnooze, @tbolin.

**iTermWindup**

iTermWindup is an 'old' way to prevent iTerm growth when the quad cannot achieve the requested target rate.
In 4.4 we are making iTermWindup active by default, at a value of 85.
Whenever the motorMix percentage exceeds the `iterm_windup` limit value of 85%, iTerm growth will be reduced to the extent that motor_mix exceeds the `iterm_windup` value.  There will be zero iTerm growth only when motorMix differential is 100%.  The iTerm inhibiotion will apply on all axes, including yaw.
The default value is appropriate for nearly all quads.  Heavy or very low authority quads with significant iTerm windup problems (slow oscillations on larger moves) may benefit from lower iterm_windup values.
iTermWindup complements iTermRelax, and is especially useful to prevent iterm windup in low authority quads where the machine is not able to meet the target rate for some time after the sticks have stopped moving.  iTermRelax is most effective, and operates more smoothly, when the sticks are moving fast.  iTermWindup also constrains iTerm growth in a variety of impact or failure states.
For more information see [PR#11806](https://github.com/betaflight/betaflight/pull/11806) - thanks @ctzsnooze.

**Smoother mixer behaviour when airmode is active with extreme stick inputs**

There were a number of edge cases where the onset of airmode throttle boost was not smooth, with different outcomes depending on the mixer type.  The mixer is now better behaved.
For more information see [PR#11867](https://github.com/betaflight/betaflight/pull/11867) - thanks @QuickFlash.


## 7. ELRS 3.x support for ELRS SPI boards

NOTE:  ELRS 2.x transmitters will not be able to bind to ELRS SPI Boards flashed with Betaflight 4.x.


## Other changes

- Four PID profiles (was 6), and 4 rate profiles (was 3) - thanks @haslinghuis
- TPA settings inside the PID profile - thanks @haslinghuis
- improved barometer smoothing and calibration, attitude calculation fixes, filter fixes - thanks @karatebrot
- HD OSD support - thanks @SteveCEvans
- VTX device over MSP [PR11705](https://github.com/betaflight/betaflight/pull/11705) - thanks @phobos
- parse and log GPS degree of precision (DOP) values [PR11912](https://github.com/betaflight/betaflight/pull/11912) - thanks @karatebrot
- hardware support for newer gyro chips, improved filtering for BMI160/270, etc - thanks @SteveCEvans and others
- lower minimum of 20Hz for dynamic notch, useful for low RPM setups with very large props
- increased dynamic idle minimum RPM from 100 (10k RPM) to 200 (20k rpm) for quads with very small props
- support for 64 discrete LEDs via cloud build option - PR12064 - thanks @Limonspb
- Fixes to NMEA at 10hz and UBlox comms - thanks @Karatebrot, @krzysztofkuczek
- Winbond W25q80 flash support - thanks @David-OConnor
- many bugfixes, target updates, driver updates and fixes - thanks to too many people to mention individually
