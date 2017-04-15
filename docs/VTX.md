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

While the VTX button is held the STATUS 2 LED will flash N times per second indicating the action that will be taken when the button is released. The flashing starts as soon as the button is held. e.g. You press the button, count flashes and then release as appropriate.

| Duration      | Function                  | Flashes   |
|---------------|---------------------------|-----------|
| 25ms to 1s    | Cycle Channel             | 4         |
| 1s to 3s      | Cycle Band                | 3         |
| 3s to 5s      | Cycle Power and RF Power  | 2         |
| 5s or more    | Save FC settings          | 1         |

Example to cycle VTX power
```
0 seconds       | 1 second      | 2 seconds     | 3 seconds     | 4 seconds     | 5 seconds     | 6 seconds or more |
[-HOLD BUTTON-----------------------------------|-RELEASE BUTTON-NOW------------|-RELEASED TO LATE TO CHANGE POWER -|
| 4 Flashes     | 3 flashes     | 3 flashes     | 2 flashes     | 2 flashes     | 1 flash       | 1 flash           |
```

The VTX button works with ALL VTX systems including onboard RTC6705, Tramp and SmartAudio.


If the VTX can be turned off then POWER 0 will turn off the VTX and POWER 1 will set the VTX into it's lowest power output.
If the VTX cannot be turned off then POWER 0 will set the VTX into it's lowest power output.
