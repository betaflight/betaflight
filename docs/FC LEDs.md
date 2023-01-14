# Betaflight Flight Controller LED usage

Betaflight Flight Controllers typically have two LEDs, possibly three which indicate the conditions below.

FCs implement at least one LED to indicate activity. The second is preferable, and the third optional. 

| LED Number | Colour | Required | Function |
| :-- | :-- | :-- | :-- |
| 0 | Blue | Yes | Flashes 5 times on startup<br>On whilst armed<br>Flashes to indicate warning<br>Flashes during ESC passthough<br>Flashes during USB MSC activity<br>Indicates activity for serial passthrough<br>Indicates Rx activity for serial 4way<br>Flashes during Spekrum binding<br>Flashes during Hard Fault conditions |
| 1 | Green | Preferably | Flashes 5 times on startup<br>Indicates Tx activity for serial 4way<br>Flashes during Hard Fault conditions |
| 2 | Amber | No | Normally on<br>Flashes during Spekrum binding<br>Flashes during Hard Fault conditions |

Error codes indicated by a brief 100 ms flash and then a count of 250 ms flashes as per the table below.

| Error | Flash count |
| :-- | :-- |
| FAILURE\_MISSING\_ACC | 1 |
| FAILURE\_ACC\_INIT | 2 |
| FAILURE\_ACC\_INCOMPATIBLE | 3 |
| FAILURE\_INVALID\_EEPROM\_CONTENTS | 4 |
| FAILURE\_CONFIG\_STORE\_FAILURE | 5 |
| FAILURE\_GYRO\_INIT\_FAILED | 6 |
| FAILURE\_FLASH\_READ\_FAILED | 7 |
| FAILURE\_FLASH\_WRITE\_FAILED | 8 |
| FAILURE\_FLASH\_INIT\_FAILED | 9 |
| FAILURE\_EXTERNAL\_FLASH\_READ\_FAILED | 10 |
| FAILURE\_EXTERNAL\_FLASH\_WRITE\_FAILED | 11 |
| FAILURE\_EXTERNAL\_FLASH\_INIT\_FAILED | 12 |
| FAILURE\_SDCARD\_READ\_FAILED | 13 |
| FAILURE\_SDCARD\_WRITE\_FAILED | 14 |
| FAILURE\_SDCARD\_INITIALISATION\_FAILED | 15 |
| FAILURE\_SDCARD\_REQUIRED | 16 |
