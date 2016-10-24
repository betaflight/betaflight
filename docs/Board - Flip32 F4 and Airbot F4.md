# Board - Flip32 F4 and Airbot F4

## Motors

| Motor     | pin   |   Shared with |
| ----      | ----  |   ----        |
| 1         | PB0   |               |
| 2         | PB1   |               |
| 3         | PA3   |               |
| 4         | PA2   |               |
| 5         | PA1   |               |
| 6         | PA8   |               |

## RSSI ADC

* Connected to pin PA0
* 3.3V tolerant, do not supply 5V

## Current Meter ADC

* Connected to pin PC1
* 3.3V tolerant, do not supply 5V

## Voltage monitoring

* Connected to pin PC2
* Connected to VBAT pins (both are the same) and integrated Voltage Stabilizer (LM7805M)

### Integrated voltage stabilizer

It is integrated with voltage monitoring and always powered when VBAT is conntected to battery.
Because this is **Linear Stabilizer**, it has a tendency to overheat, especially on 4S. Because of that,
avoid powering too many devices directly to 5V pins on the board. RX receiver is (and board itself) is rather all
it can do without overeating (150mA on 4S gives 1.5W of waste heat!). OSD, LED Strip and other devices should powered from separate BEC if voltage monitoring is to be enabled.
