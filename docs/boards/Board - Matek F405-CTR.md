# Board - Matek F405-CTR

Full details on the Matek Sys F405-CTR can be found on the Matek Website: [mateksys.com/?portfolio=f405-ctr](http://www.mateksys.com/?portfolio=f405-ctr). Betaflight Target: `MATEKF405`

## Hardware Specs

### FC Specs

* *MCU:* 168MHz STM32F405 with IMU: MPU6000 accelerometer/gyro (SPI)
* *Baro:* BMP280 (I2C)
* *OSD:* BetaFlight OSD w/ AT7456E chip
* *Blackbox:* MicroSD card slot (SD/SDHC)
* 5 UART TX/RX pairs
* Built in inverter for SBUS (`UART2-RX`)
* SoftSerial on `TX2`, `S5` or `S6` optional
* Camera control on `S6` or `DAC` optional
* Smartaudio & Tramp VTX protocol supported
* *WS2812 LED pad:* Yes
* *Beeper pad:* Yes
* *RSSI pad:* Yes

### Integrated PDB Specs

* *Input:* 6-30V (3-6S LiPo) w/TVS protection
* *PDB:* Rated 4x30A per ESC pad set (Max. 4x46A burst)
* *BEC:* 5V 2A continuous load (Max. 3A burst)
* *LDO:* 3.3V: Max load: 300mA for Spektrum RX
* *Current Sensor:* Rated for 184A (*Suggested scale value `179`*)
* *Battery Voltage Sensor:* 1:10 (*Suggested scale value `110`*)

## Pinout

Pads are organised into two large banks of pads on left and right sides of board with a couple sets of pads on the underside of the board and ESC related connections near the board corners.

```
          __________
         /  U    U  \
/-----------------------------\
|oE                         Eo|
|SC                         SC|
|                             |
| P                        P  |
| A                        A  |
| D                        D  |
| S                        S  |
|                             |
|ES                         ES|
|oC                         Co|
\------------[USB]------------/
```

> NOTE: Under normal operation, the use of the Barometer `I2C1` bus (internal) and the WS2812 `LED` signal pad are mutually exclusive due to a pad mapping conflict. However, the `LED_STRIP` feature may be remapped to the `S5` (`PA15`) pad to avoid this conflict.
>
> *I.e.* `resource LED_STRIP 1 A15`

| Pad Silkscreen Label |   Function    |                                                 Notes                                          |
|---------------------:|---------------|:-----------------------------------------------------------------------------------------------|
| `+ / -`              | Battery       | 6-30VDC LiPo Power input                                                                       |
| `S1-S6`              | ESC Out       | (1-4 near ESC power connections, 5-6 on front) Supports PWM, Oneshot, Multishot, DSHOT         |
| `5V, GND, S1-S4`     | ESC Out       | (Rear of board) 4-in-1 ESC Out                                                                 |
| `VBT, GND`           | VBT Out       | Marked for VTX OSD, but could be used for whatever                                             |
| `CURR`               | Current Sense | Current Sensor I/O pin (output from onboard sensor or input from external sensor)              |
| `5V`                 |               | Out from internal 5V BEC (rated 2A continuous, 3A burst)                                       |
| `3V3`                |               | Out from 3V3 regulator (rated 300mA)                                                           |
| `4V5`                |               | Out from 4V4~4V8 regulator (unknown rating, power is also supplied when connected via USB)     |
| `G`                  | GND           |                                                                                                |
| `LED`                | WS2812 Signal |                                                                                                |
| `Bz-, 5V`            | Buzzer        |                                                                                                |
| `Rssi`               | RSSI          | FrSky RSSI input from RX                                                                       |
| `G, 3V3, SDA, SLC`   | I2C           | (Rear of board) I2C connection marked for a magnetometer but could be used for whatever        |
| `VTX, Cam, DAC`      | VTX/CAM       | VTX: Video out, Cam: Video in, DAC: Cam control pin (since BF3.3)                              |
| `RX1, TX1`           | UART1         |                                                                                                |
| `RX2`                | UART2-RX      | RX connection for Spektrum DSMX or DSM2, FlySky iBUS, or PPM                                   |
| `Sbus`               | SBUS (UART2)  | Dedicated pad for SBUS (duplicate pad of RX2 with signal inverter)                             |
| `RX3, TX3`           | UART3         |                                                                                                |
| `RX4, TX4`           | UART4         |                                                                                                |
| `RX5, TX5`           | UART5         |                                                                                                |
| `RX5`                | UART5-RX      | (4x, one near each board corner) Duplicates of RX5 pad for ESC Telemetry                       |
