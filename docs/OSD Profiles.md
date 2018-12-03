# OSD Profiles

An OSD Profile is a screen or page of OSD elements.  Three OSD Profiles are supported, i.e. you can configure 3 different OSD Profiles or pages each with their own OSD elements.  Elements may also be on all 3 profiles.  The OSD can still be turned on or off as before via an AUX channel.  Hence users who don't want this feature are not affected by its availability.  With no profiles are configured for elements, all elements are visible, i.e. OSD Profile 1 is the default profile.  Keep in mind that if an element is used, it must be in the same position on all OSD Profiles in which it is visible.  It is not possible to configure an element to be in a different location on a different profile.


## Configuration

Currently (Configurator 10.4.0) OSD Profiles can only be configured via the CLI.  Layout your OSD using the configurator and save.  Elements in the same position but on different profiles will overlap in the configurator at this stage.  OSD Profiles can either be selected via an adjustment range (Adjustments tab, after enabling Expert Mode) or by setting the osd_profile parameter with the set command in the CLI.  The selected OSD Profile can either be controlled from a switch on your radio or you can configure different profiles and select the one you want by using the ```set osd_profile=2``` command in the CLI and saving.


###### OSD Configuration
1. Configure your OSD via the OSD tab in the configurator, i.e. element layout, font etc. as before.


###### OPTION 1: Configure an adjustment range to change OSD Profile from your radio
1. Turn on Expert mode - see top right of configurator screen "Enable Expert Mode".
2. The OSD Profile selection is performed using an adjustment configured via the Adjustments tab.
    - Enable an adjustment. ("If enabled")
    - Select the AUX channel to be used to change OSD Profile. ("when channel")
    - Set the range to cover the entire range of the selected AUX channel. ("is in ranges")
    - For the action select "RC Rate Adjustment". ("then apply")  This will be configured in the CLI since OSD Profiles is not supported by Configurator 10.4.0 and earlier. "RC Rate Adjustment" is only selected to make the configuration in the CLI a little easier below.
    - Select slot 1. ("using slot")  (the slot number does not matter, pick any free number)
    - Select the "via channel" to match the selected AUX channel of above. ("when channel").
    - Save
3. Open the CLI and type ```adjrange``` followed by enter.
4. Copy the adjrange configured in step 2. above and paste it in the command window.  Change the '1' following the range of the channel to '29' and press enter.  Type ```save``` and press enter.  The configured adjrange will now be saved and the FC will reboot.
5. Configure the AUX channel on your radio.  When this channel is changed the selected OSD Profile will change displaying all the elements configured for the selected profile.  Note that elements are assigned to profile 1 by default.  To revert back to not using OSD Profiles, go to the CLI and select and type ```set osd_profile=1``` press enter, followed by ```save``` and enter.  Keep in mind that elements configured to be only visible on profile 2 and 3 will need to reconfigured and will not be visible on profile 1.


###### OPTION 2: Use the CLI to select an OSD Profile (i.e. not selecting the OSD Profile with your radio)
1. Open the CLI.
2. Type ```get osd_profile``` followed by enter to display the currently selected profile.
3. Type ```set osd_profile=x``` where x is the profile number 1,2 or 3 and press enter.
4. Type ```save``` followed by enter to save the selected profile.


###### CONFIGURE OSD PROFILES
The steps below are only required if using The Configurator 10.4.0 or earlier.
1. Open the CLI.
2. Find the name of the element to be configured, all elements can be displayed using the ```dump``` command or ```get osd``` command.  Element names end in _pos.
3. Take the current value of the element, apply the OSD Profile config and set its new value using the ```set``` command.  The OSD Profile config value is described below.

   eg. osd_vbat_pos = 6560
       ```set osd_vbat_pos = 31136```
       ```save```

4. Repeat steps 2 & 3 for every element to be configured.


###### OSD PROFILE CONFIG VALUES
The OSD Profile config is stored in the higher order bits, bits 11, 12 & 13 of an OSD element, eg. osd_vbat_pos.  This means the initial value, after being configured in the OSD tab needs to be adjusted to add the OSD Profile configuration of the element.  To calculate the new value, take the current (initially configured value) of the OSD element and convert it to HEX.  Determine which OSD Profiles this element should be visible on and select the corresponding HEX value from the table below.  Perform an OR operation between the initial OSD element value (in HEX) and the HEX value from the table.  Convert the result to decimal and set the OSD element to this value using the ```set``` command in the CLI.

    Binary Value    HEX Value    Profile Configuration   Description
    13 12 11
     0  0  1        0x0800   -   1                       Visible in OSD Profile 1
     0  1  0        0x1000   -   2                       Visible in OSD Profile 2
     0  1  1        0x1800   -   1 and 2                 Visible in OSD Profile 1 and 2 only
     1  0  0        0x2000   -   3                       Visible in OSD Profile 3 only
     1  0  1        0x2800   -   1 and 3                 Visible in OSD Profile 1 and 3 only
     1  1  0        0x3000   -   2 and 3                 Visible in OSD Profile 2 and 3 only
     1  1  1        0x3800   -   1, 2 and 3              Visible in all OSD Profiles

Example:
1. osd_vbat_pos = 6560
2. Convert 6560 to HEX results in 19A0 (HEX)  (All these operations can be done with the windows calculator in programmer mode)
3. We want this element to be visible on OSD Profile 2 & 3 and hence select the HEX value 0x3000 from the table above.
4. OR the two HEX values together  19A0 OR 3000 = 39A0
5. Convert 39A0 (HEX) to decimal results in 14752.
5. In the CLI, ```set osd_vbat_pos = 14752``` enter and ```save``` enter.


## Note:

1. If using Configurator 10.4.0 or earlier, when changing an adjustment on the Adjustments tab, the "then apply" item (29) for OSD Profile will also be cleared, i.e. set to 0 and would have to be re-configured via the CLI, e.g. ```adjrange 0 0 8 900 2100 29 8 0 0```
2. OSD Profile selection is not active if the CLI is open, in other words if you change the value of osd_profile while the CLI is open, nothing will change, the selected OSD Profile will remain as is.


## Use

After completing the above configuration, you should be able to select the active OSD Profile from your radio or via the CLI.  The profile can be selected/changed at any time while on the ground or in mid flight.


## Useful OSD Profile CLI commands

```get osd_profile {enter}``` >> displays the currently selected OSD Profile in the CLI.

```set osd_profile= {number, 1-3}``` selects the OSD Profile.
