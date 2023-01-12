# Betaflight 4.4 Release Notes

## 1. Cloud build 

This is predominantly brought to you for convenience, and to ensure we can keep the 512kb flash targets (STM32F411 and STM32F722) alive and well for years to come. The cloud build system will allow you, the flyer, to select the features you want, and a custom hex will be created for you.

Any issues with the cloud build process please check out the #cloud-build-issues Discord channel. Help us to help you, buy taking advantage of the new "Support" button in the CLI tab in Configurator 10.9.0 (RC4 onwards). This will give us valuable information in trying to diagnose your issue.

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

