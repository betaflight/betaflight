# Board - Naze32

The Naze32 target supports all Naze hardware revisions.  Revision 4 and 5 are used and
frequently flown by the primary maintainer.  Previous Naze hardware revisions may have issues,
if found please report via the [github issue tracker](https://github.com/iNavFlight/inav/issues).

## Serial Ports

| Value | Identifier   | RX         | TX                 | Notes                                                                                       |
| ----- | ------------ | ---------- | ------------------ | ------------------------------------------------------------------------------------------- |
| 1     | USART1       | RX  / PA10 | TX  / PA9 / TELEM  | TELEM output is always inverted (for FrSky). Internally connected to USB port via CP2102 IC |
| 2     | USART2       | RC4 / PA3  | RC3 / PA2          |                                                                                             |
| 4     | SOFTSERIAL1  | RC5 / PA6  | RC6 / PA7          |                                                                                             |
| 5     | SOFTSERIAL2  | RC7 / PB0  | RC8 / PB1          |                                                                                             |

* You cannot use USART1/TX/TELEM pins at the same time.
* You may encounter flashing problems if you have something connected to the RX/TX pins.  Try disconnecting RX/TX.

## Pinouts

The 10 pin RC I/O connector has the following pinouts when used in RX_PPM/RX_SERIAL mode.

| Pin | Identifier | Function                    | Notes                            |
| --- | ---------- | --------------------------- | -------------------------------- |
| 1   |            | Ground                      |                                  |
| 2   | Circle     | +5V                         |                                  |
| 3   | 1          | RX_PPM                      | Enable `feature RX_PPM`          |
| 4   | 2          | RSSI_ADC                    | Enable `feature RSSI_ADC`.  Connect to the output of a PWM-RSSI conditioner, 0v-3.3v input |
| 5   | 3          | USART2 TX                   |                                  |
| 6   | 4          | USART2 RX                   |                                  |
| 7   | 5          | LED_STRIP                   | Enable `feature LED_STRIP`       |
| 8   | 6          | unused                      |                                  |
| 9   | 7          | HC-SR04 Trigger               |                                  |
| 10  | 8          | HC-SR04 Echo / CURRENT        | Enable `feature CURRENT_METER`  Connect to the output of a current sensor, 0v-3.3v input |

When SOFTSERIAL is enabled, LED_STRIP and CURRENT_METER are unavailable, but two SoftSerial ports are made available to use instead.

| Pin | Identifier | Function       | Notes                            |
| --- | ---------- | -------------- | -------------------------------- |
| 7   | 5          | SOFTSERIAL1 RX | Enable `feature SOFTSERIAL`      |
| 8   | 6          | SOFTSERIAL1 TX |                                  |
| 9   | 7          | SOFTSERIAL2 RX |                                  |
| 10  | 8          | SOFTSERIAL2 TX |                                  |

## Recovery

### Board
+ Short the two pads labelled 'Boot' **taking extra care not to touch the 5V pad**
+ Apply power to the board
+ Remove the short on the board

### INAV configurator
+ Select the correct hardware and the desired release of the INAV firmware
+ Put a check in the "No reboot sequence"
+ Flash firmware

```
/-------------------\
|O                 O|
| []5V              |
| [][]Boot          |
|                   |
|                   |
|                   |
|                   |
|O                 O|
\-------[USB]-------/
```
