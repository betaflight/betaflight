# VTX

Cleanflight supports control of VTX modules.  Some boards like the SPRacingF3NEO have both a VTX module and a button connected to the CPU to allow the user to change channel, band, rf power, etc.

## VTX Button usage

While the VTX button is held the STATUS 2 LED will flash N times per second indicating the action that will be taken when the button is released. The flashing starts as soon as the button is held. e.g. You press the button, count flashes and then release as appropriate.

* 25ms to 1s - Cycle Channel 1-8 - 5 flashes.
* 1s to 3s - Band 1-5 - 4 flashes.
* 3s to 5s - Cycle RF Power - 3 flashes.
* 5s to 10s - Enable/Disable VTX - 2 flashes.
* 10s or more - Save settings - If VTX is currently ON then it will be ON on next boot. If it's currently OFF then it will be OFF at the next boot. - 1 flash.
