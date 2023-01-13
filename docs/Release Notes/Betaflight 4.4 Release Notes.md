# Betaflight 4.4 Release Notes

## 1. Cloud build 

This is predominantly brought to you for convenience, and to ensure we can keep the 512kb flash targets (STM32F411 and STM32F722) alive and well for years to come. The cloud build system will allow you, the flyer, to select the features you want, and a custom hex will be created for you.

Any issues with the cloud build process please check out the #cloud-build-issues Discord channel. Help us to help you, buy taking advantage of the new "Support" button in the CLI tab in Configurator 10.9.0 (RC4 onwards). This will give us valuable information in trying to diagnose your issue.

**NOTE:** If you have something missing from your cloud build that you would normally expect to be present, e.g. a flash chip or barometer, the reason is because the board configuration (in unified targets) has not been updated with this information (either by the community, or the manufacturer).

If you do have something missing and it works as expected in `classic mode`, please don't worry it is an easy fix, you can use the **custom defines** input box that is shown when "Expert Mode" is enabled. The `classic mode` option will load the all of the hardware drivers (but not all the features), and will allow you to boot the Flight Controller, and run the commands `status`, `flash_info`, `dump hardware` etc, in the CLI tab of configurator to find the information about the hardware you have.

For those missing a barometer: You can try any or all of `BARO_MS5611 BARO_SPI_MS5611 BARO_BMP280 BARO_SPI_BMP280 BARO_BMP388 BARO_SPI_BMP388 BARO_LPS BARO_SPI_LPS BARO_QMP6988 BARO_SPI_QMP6988 BARO_DPS310 BARO_SPI_DPS310 BARO_BMP085 BARO_2SMBP_02B BARO_SPI_2SMBP_02B` in the **custom defines** input box.

For those missing the flash chip: You can try any or all of `USE_FLASH_W25P16 USE_FLASH_W25Q128FV USE_FLASH_W25M02G USE_FLASH_W25N01G USE_FLASH_W25M` in the custom defines input box.

Thank you all for your patience and assistance in working through what boards have what hardware. Unfortunately we need our flyers to help crowd source this information - as there is such diverse hardware out there! 

## 2. HD OSD Canvas

HD OSD is now supported and adds the following features. Note that not all HD Goggle/VTX combinations support all features, but hopefully will do so in time.

If the `OSD` option alone, or the `OSD (HD)` option is included in the build options, then HD support will be included.

To enable HD support select `VTX (MSP + Displayport)` on the ports tab for the UART to which the VTX is connected. The `MSP` option will automatically be selected.

The OSD `HD` preview may be selected on the OSD tab along side the `PAL` and `NTSC` options. The size of the canvas (visible number of columns/rows) defaults to 53x20 (compared to 30x16 for PAL and 30x13 for NTSC). The VTX is then able to adjust this by sending an MSP command to adjust the canvas size. WTFOS will set this to 60x22 for example. This will be automatic with no user interaction required. Should the goggle vendor decide to support alternate canvas sizes then they would be selected in the goggle menus, and the new canvas size communicated to Betaflight which would then adjust the usable number of OSD rows/columns accordingly.

Regardless of the canvas size the boot logo, armed message, CMS menu etc. will be centred correctly.

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

## 3. Preset Favourites

This feature reduces the amount of search the users have to do in the presets tab. Configurator will remember the presets you are using, automatically marking them with the "star". Favorite presets will always appear first in the initial list and the search results. In combination with the fix that preselects the current firmware from the plugged in FC it allows users to completely avoid searching of the commonly used presets, and pick them right away. The UI "stars" are clickable, so users can manually add/remove favorite presets.
![image](https://user-images.githubusercontent.com/2925027/212130300-f67a5d82-dbc2-4726-9c07-b6aae0aa98ae.png)
Favorite presets are being remembered by it's path+name in the repo. So a favorite preset in one repository becomes automatically a favorite in another, if it's sharing the same name and path withing the repos.


## 4. GPS Return to Home enhancements

GPS "Rescue" has been extensively revised and greatly improved.  The quad should reliably return at the set speed, descend at an angle, land within a few metres of the home point, and disarm automatically on touch-down.  There are separate PID control elements for altitude and return velocity t home; the defaults work very well for 'typical' quads.  The system should initially be tested with a switch at reasonably close range and low altitude.  Setting up and testing GPS Rescue to provide a reliable return on the event of an RxLoss failsafe is a non-trivial task, but well worth the trouble.  

**We strongly recommend reading the [wiki entry](https://github.com/betaflight/betaflight/wiki/GPS-Rescue-for-4.4) and following the instructions there.**  

Remember that in any true failsafe the quad will always enter Failsafe Stage 1 phase for 1s (user-configurable) before initiating the Rescue.  You MUST set the Stage 1 behaviour NOT to DROP, or it will disarm and drop in Stage 1 and never enter GPS Rescue.  The safest option for Stage 1 is to configure Angle Mode on an aux switch, and set Stage 1 Failsafe to enable Angle Mode at a fixed hover / light climb throttle value with all other sticks forced to center.  When you get signal loss of more than 300ms, you'll enter Angle Mode, and the will start to level out.  That will give you a clear advance warning that you are getting signal breakup.  Alternatively, you can set Failsafe Stage 1 to hold all current values; the quad will then continue on the same path until Stage 1 Failsafe expires and the Rescue starts.

Please disable the compass/magnetometer unless it has been fully calibrated and you have confirmed, by logging, that the magnetometer heading values are noise free and reflect the true attitude of the quad.

In most short flights, using the Baro provides a significant improvement in altitude control.  Baro data is updated more frequently than GPS data, and often varies less.  The user can control how much trust they place on the Baro vs GPS altitude data.  Over long flights, and with some particular Baro units or installations, Baro drift can be more of a problem, and the GPS data should be trusted more.

There are three new, and very useful, GPS Rescue Debug modes.  One shows how accurately the quad tracks the requested altitude and the requested velocity.  The other two are used for tuning the GPS Rescue PIDs. If Mag is enabled, Mag information is automatically recorded.

Data acquisition from GPS hardware now uses the UBlox protocol, at 10hz, by default. NMEA mode has been improved and in some cases will run at 10hz.  We now can log Dilution Of Precision values for future improvements.

There are extensive changes to sanity checks, and in most cases the quad will attempt to land itself, rather than disarm, if necessary using only the Baro signal.  

WARNING: ALWAYS CHECK that the Home Arrow points directly back towards home when you take off and fly away.  Sometimes, if you take off and spin around during arming, or immediately on takeoff, the quad's attitude information can become corrupted, and the Home Arrow can point the wrong way.  It's best to arm cleanly and fly away from Home in a straight line at reasonable speed immediately after takeoff.  Watch the Home Arrow carefully to ensure that it quickly points back to Home. If the Home Arrow points the wrong way when a failsafe occurs, the GPS Rescue will initially fly off in the wrong direction and in some cases you may lose the quad.
 
