# VTX

Cleanflight supports control of VTX modules.  

## VTX Systems

Current support includes 
1. RTC6705 directly connected to the CPU (maybe via a PCB board interconnect, e.g. SPRACINGF3NEO)
2. IRC Tramp
3. TBS Smart Audio

## VTX Button

If your FC has a button, excluding a BOOT buttons, then it can be used for VTX control.

Some boards like the SPRacingF3NEO have both a VTX module and a button.
Other boards like the SPRacingF3MINI have multiple buttons.

### VTX Button usage

While the VTX button is held the STATUS 2 LED will flash N times per second indicating the action that will be taken when
the button is released. The flashing starts as soon as the button is held. e.g. You press the button, count flashes and
then release as appropriate.

| Duration      | Function                  | Flashes   |
|---------------|---------------------------|-----------|
| 25ms to 1s    | Cycle Channel             | 4         |
| 1s to 3s      | Cycle Band                | 3         |
| 3s to 5s      | Cycle Power and RF Power  | 2         |
| 5s or more    | Save FC settings          | 1         |

Example to cycle VTX power

```
| 0 seconds      | 1 second      | 2 seconds    | 3 seconds     | 4 seconds     | 5 seconds     | 6 seconds or more |
|-HOLD BUTTON-----------------------------------|-RELEASE BUTTON-NOW------------|-RELEASED TOO LATE TO CHANGE POWER-|
| 4 Flashes      | 3 flashes     | 3 flashes    | 2 flashes     | 2 flashes     | 1 flash       | 1 flash           |
```

The VTX button works with ALL VTX systems including onboard RTC6705, Tramp and SmartAudio.


If the VTX can be turned off then POWER 0 will turn off the VTX and POWER 1 will set the VTX into it's lowest power output.
If the VTX cannot be turned off then POWER 0 will set the VTX into it's lowest power output.

## VTX Table

As of Betaflight 4.1.0, band/channel and power level information needed to control videotransmitters is no longer hardcoded,
but stored in a new facility called vtxTable.
  
The contents of the vtxTable need to be setup manually. They need to match the hardware, local laws and regulations, as
well as user preferences.

The contents of the table can be examined by typing the command `vtxtable` into the cli.
Example:
```
# vtxtable
vtxtable bands 5
vtxtable channels 8
vtxtable band 1 BOSCAM_A A FACTORY    0 5845 5825 5805 5785 5765 5745    0
vtxtable band 2 BOSCAM_B B FACTORY    0 5752 5771 5790 5809 5828 5847    0
vtxtable band 3 BOSCAM_E E FACTORY    0    0    0    0    0    0    0    0
vtxtable band 4 MYBAND   M CUSTOM  5745 5769    0 5806    0 5843    0    0
vtxtable band 5 RACEBAND R FACTORY    0    0    0 5769 5806 5843    0    0
vtxtable powerlevels 3
vtxtable powervalues  14 20 26
vtxtable powerlabels 25 100 400
``` 

### Bands and channels

The example above contains 5 bands, **each with a name, a single-letter abbreviation, a factory flag and eight frequencies.**

The factory flag controls how Betaflight communicates with the vtx.

**When the flag is set to `FACTORY`, Betaflight sends the vtx a band and channel number.**
The vtx will then use its built-in frequency table.
In this mode, the actual contents of the vtxtable are **not** send the vtx. They are only used for display in the OSD and similar places.
As such, bands with the flag set to `FACTORY` should be set to match the built-in frequency table of the vtx.

**When the flag is set to `CUSTOM`, Betaflight sends the vtx the frequency it should use.**
This mode utilizes the contents of the table and allows the user to create custom bands with whatever frequencies they like.
**Videotransmitters without a built-in table, such as IRC Tramp or rtc6705 only support `CUSTOM`.**

Entries of the vtxtable can be blocked by setting their frequency to 0. This is especially useful for bands set to `FACTORY`: The spots of
unwanted entries of the videotransmitter's built-in table can be set to 0, effectively disabling them.
In the example above this was used to only allow frequencies between 5725 and 5875 MHz, as a German pilot would want it to comply with
German laws. Additionally, the Fatshark band was replaced with a new custom one.

As a starting point, the following table contains the commonly used frequencies:

```
# This table should not be used as-is, but trimmed down according to local laws and regulations.
vtxtable band 1 BOSCAM_A A FACTORY 5865 5845 5825 5805 5785 5765 5745 5725
vtxtable band 2 BOSCAM_B B FACTORY 5733 5752 5771 5790 5809 5828 5847 5866
vtxtable band 3 BOSCAM_E E FACTORY 5705 5685 5665 5645 5885 5905 5925 5945
vtxtable band 4 FATSHARK F FACTORY 5740 5760 5780 5800 5820 5840 5860 5880
vtxtable band 5 RACEBAND R FACTORY 5658 5695 5732 5769 5806 5843 5880 5917
``` 


### Power levels

In addition the the frequency, videotransmitters also need to know how much power they should use for transmission.
The example shown previously contains three power levels, **each with a value and a label.** The label is shown to the user in the OSD,
while the value is sent to the vtx.

Power levels should be setup to match the hardware in use.

#### IRC Tramp devices should use:
```
vtxtable powerlevels 5
vtxtable powervalues 25 100 200 400 600
vtxtable powerlabels 25 100 200 400 600
```

#### rtc6705 should use:
```
vtxtable powerlevels 2
vtxtable powervalues 1 2
vtxtable powerlabels MIN MAX
```

Please note that turning off rtc6705 devices is not possible using powervalues. Use pitmode instead.

#### SmartAudio V1.0 devices should use:
```
vtxtable powerlevels 4
vtxtable powervalues 7 16 25 40
vtxtable powerlabels 25 200 500 800
```

#### SmartAudio V2.0 devices should use:
```
vtxtable powerlevels 4
vtxtable powervalues 0 1 2 3
vtxtable powerlabels 25 200 500 800
```

#### SmartAudio V2.1 devices vary depending on their model. Check the manufacturers website.
For these devices the `powervalues` are the output power in dBm.

To query the available power levels from a SmartAudio 2.1 VTX enter the `vtx_info` command with no parameters. This will report the available power settings thus:

```
# vtx_info
level 14 dBm, power 25 mW
level 20 dBm, power 100 mW
level 26 dBm, power 400 mW
```

For example the

[TBS Unify Pro32 Nano 5G8](https://www.team-blacksheep.com/products/prod:unifypro32_nano):

```
vtxtable powerlevels 3
vtxtable powervalues 14 20 26
vtxtable powerlabels 25 100 400
```

[TBS Unify Pro 5G8 HV - Race 2 (MMCX)](https://www.team-blacksheep.com/products/prod:unify_pro_hv_race2_m):

```
vtxtable powerlevels 3
vtxtable powervalues 13 20 26
vtxtable powerlabels 25 100 400
```

[TBS Unify Pro32 HV (MMCX)](https://www.team-blacksheep.com/products/prod:unifypro32_hv):

```
vtxtable powerlevels 4
vtxtable powervalues 14 20 26 30
vtxtable powerlabels 25 100 400 1W
```

[TBS Unify EVO](https://www.team-blacksheep.com/products/prod:tbs_unify_evo):

```
vtxtable powerlevels 4
vtxtable powervalues 14 20 26 29
vtxtable powerlabels 25 100 400 800
```

Power levels may be omitted. This is useful for compliance with local laws and regulations.
Additionally, powerlabels (but not values!) can be set to anything three characters long.
For example a TBS Unify EVO will also work the this config:

```
vtxtable powerlevels 2
vtxtable powervalues 20 26
vtxtable powerlabels .1W .4W
```

### Complete Examples

#### IRC Tramp device

```
# This example enables a lot of power levels and channels.
# Almost nobody will be able to legally use this without modification.
# Check your local laws and regulations before use!
vtxtable bands 5
vtxtable channels 8
vtxtable band 1 BOSCAM_A A CUSTOM 5865 5845 5825 5805 5785 5765 5745 5725
vtxtable band 2 BOSCAM_B B CUSTOM 5733 5752 5771 5790 5809 5828 5847 5866
vtxtable band 3 BOSCAM_E E CUSTOM 5705 5685 5665 5645 5885 5905 5925 5945
vtxtable band 4 FATSHARK F CUSTOM 5740 5760 5780 5800 5820 5840 5860 5880
vtxtable band 5 RACEBAND R CUSTOM 5658 5695 5732 5769 5806 5843 5880 5917
vtxtable powerlevels 5
vtxtable powervalues 25 100 200 400 600
vtxtable powerlabels 25 100 200 400 600
```

#### SmartAudio 1.0 device

```
# This example enables a lot of power levels and channels.
# Almost nobody will be able to legally use this without modification.
# Check your local laws and regulations before use!
vtxtable bands 5
vtxtable channels 8
vtxtable band 1 BOSCAM_A A FACTORY 5865 5845 5825 5805 5785 5765 5745 5725
vtxtable band 2 BOSCAM_B B FACTORY 5733 5752 5771 5790 5809 5828 5847 5866
vtxtable band 3 BOSCAM_E E FACTORY 5705 5685 5665 5645 5885 5905 5925 5945
vtxtable band 4 FATSHARK F FACTORY 5740 5760 5780 5800 5820 5840 5860 5880
vtxtable band 5 RACEBAND R FACTORY 5658 5695 5732 5769 5806 5843 5880 5917
vtxtable powerlevels 4
vtxtable powervalues 7 16 25 40
vtxtable powerlabels 25 200 500 800
```

#### SmartAudio 2.0 device

```
# This example enables a lot of power levels and channels.
# Almost nobody will be able to legally use this without modification.
# Check your local laws and regulations before use!
vtxtable bands 5
vtxtable channels 8
vtxtable band 1 BOSCAM_A A FACTORY 5865 5845 5825 5805 5785 5765 5745 5725
vtxtable band 2 BOSCAM_B B FACTORY 5733 5752 5771 5790 5809 5828 5847 5866
vtxtable band 3 BOSCAM_E E FACTORY 5705 5685 5665 5645 5885 5905 5925 5945
vtxtable band 4 FATSHARK F FACTORY 5740 5760 5780 5800 5820 5840 5860 5880
vtxtable band 5 RACEBAND R FACTORY 5658 5695 5732 5769 5806 5843 5880 5917
vtxtable powerlevels 4
vtxtable powervalues 0 1 2 3
vtxtable powerlabels 25 200 500 800
```

#### SmartAudio 2.1 device

```
# This example enables a lot of power levels and channels.
# Almost nobody will be able to legally use this without modification.
# Check your local laws and regulations before use!
vtxtable bands 5
vtxtable channels 8
vtxtable band 1 BOSCAM_A A FACTORY 5865 5845 5825 5805 5785 5765 5745 5725
vtxtable band 2 BOSCAM_B B FACTORY 5733 5752 5771 5790 5809 5828 5847 5866
vtxtable band 3 BOSCAM_E E FACTORY 5705 5685 5665 5645 5885 5905 5925 5945
vtxtable band 4 FATSHARK F FACTORY 5740 5760 5780 5800 5820 5840 5860 5880
vtxtable band 5 RACEBAND R FACTORY 5658 5695 5732 5769 5806 5843 5880 5917
vtxtable powerlevels 4
vtxtable powervalues 14 20 26 30
vtxtable powerlabels 25 100 400 1W
```

#### rtc6705

```
# This example enables a lot of power levels and channels.
# Almost nobody will be able to legally use this without modification.
# Check your local laws and regulations before use!
vtxtable bands 5
vtxtable channels 8
vtxtable band 1 BOSCAM_A A CUSTOM 5865 5845 5825 5805 5785 5765 5745 5725
vtxtable band 2 BOSCAM_B B CUSTOM 5733 5752 5771 5790 5809 5828 5847 5866
vtxtable band 3 BOSCAM_E E CUSTOM 5705 5685 5665 5645 5885 5905 5925 5945
vtxtable band 4 FATSHARK F CUSTOM 5740 5760 5780 5800 5820 5840 5860 5880
vtxtable band 5 RACEBAND R CUSTOM 5658 5695 5732 5769 5806 5843 5880 5917
vtxtable powerlevels 2
vtxtable powervalues 1 2
vtxtable powerlabels MIN MAX
```

### Pitmode
Pitmode is separate from vtxTable. No power level should be created for pitmode.
Pitmode can be controlled in a variety of ways including OSD, AUX switches and lua scripts.

Some videotransmitters have restrictions on its usage. For example, SmartAudio V1.0 and V2.0 devices can only enter pitmode on power-up.
Betaflight can make the these devices leave pitmode, but not enter it.

rtc6705 devices do not support a proper ultra-low power pitmode. Instead, if the board supports it, pitmode turns off rtc6705 devices completely.
