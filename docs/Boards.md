# Flight controller hardware

The current focus is geared towards flight controller hardware that use the STM32F3, STM32F4, STM32F7 and legacy STM32F1 series processors. The core logic is separated from the hardware drivers, porting to other processors is possible.

### Boards based on F1 CPUs

These boards are not recommended for new setups. They are very limited in memory and don't support many features of the INAV. In general F1 boards are limited to only UBLOX GPS protocol, don't have BLHeli Passthrough, don't have LEDSTRIP, only support a few telemetries (LTM and maybe FrSky) etc.

### Boards based on F3/F4/F7 CPUs

These boards are powerfull and in general support everything INAV is capable of. Limitations are quite rare and are usually caused by hardware design issues.

### Recommended boards

These boards are well tested with INAV and are known to be of good quality and reliability.

| Board name                | CPU Family | Target name(s)            | GPS  | Compass | Barometer      | Telemetry | RX                             | Blackbox             |
|---------------------------|:----------:|:-------------------------:|:----:|:-------:|:--------------:|:---------:|:------------------------------:|:--------------------:|
| [PARIS Sirius™ AIR3](http://www.multiwiicopter.com/products/inav-air3-fixed-wing)                 | F3         | AIRHEROF3, AIRHEROF3_QUAD | All  | All     | All            | All       | All                            | SERIAL               |
| [Airbot OMNIBUS AIO F3](http://shop.myairbot.com/index.php/flight-control/cleanflight-baseflight/omnibusv11.html) | F3         | OMNIBUS                   | All  | All     | All            | All       | All                            | SERIAL, SD           |
| [Airbot OMNIBUS AIO F4](http://shop.myairbot.com/index.php/flight-control/cleanflight-baseflight/omnibusf4v2.html)| F4         | OMNIBUSF4, OMNIBUSF4PRO   | All  | All     | All            | All       | All                            | SERIAL, SD, SPIFLASH |
| [Airbot F4 / Flip F4](http://shop.myairbot.com/index.php/flight-control/apm/airbotf4v1.html)                      | F4         | AIRBOTF4                  | All  | All     | All            | All       | All                            | SERIAL, SPIFLASH     |

### Other supported boards

These boards work with INAV but might be not well tested or of random quality.

| Board name                | CPU Family | Target name(s)            | GPS  | Compass | Barometer      | Telemetry | RX                             | Blackbox             |
|---------------------------|:----------:|:-------------------------:|:----:|:-------:|:--------------:|:---------:|:------------------------------:|:--------------------:|
| TBS Colibri Race          | F3         | COLIBRI_RACE              | All  | All     | All            | All       | All                            | SERIAL               |
| FURY F3                   | F3         | FURYF3, FURYF3_SPIFLASH   | All  | All     | All            | All       | All                            | SERIAL, SD, SPIFLASH |
| RCExplorer F3FC Tricopter | F3         | RCEXPLORERF3              | All  | All     | All            | All       | All                            | SERIAL               |
| Taulabs Sparky            | F3         | SPARKY                    | All  | All     | All            | All       | All                            | SERIAL               |
| SPRacing F3               | F3         | SPRACINGF3                | All  | All     | All            | All       | All                            | SERIAL, SPIFLASH     |
| SPRacing F3 EVO           | F3         | SPRACINGF3EVO             | All  | All     | All            | All       | All                            | SERIAL, SD           |
| SPRacing F3 MINI          | F3         | SPRACINGF3MINI            | All  | All     | All            | All       | All                            | SERIAL, SD           |
| Taulabs Sparky 2          | F4         | SPARKY2, SPARKY2_OPBL     | All  | All     | All            | All       | All                            | SERIAL, SPIFLASH     |
| Taulabs QUANTON           | F4         | QUANTON                   | All  | All     | All            | All       | All                            | SERIAL, SPIFLASH     |
| BlueJay F4                | F4         | BLUEJAYF4                 | All  | All     | All            | All       | All                            | SERIAL, SPIFLASH     |
| F4BY                      | F4         | F4BY                      | All  | All     | All            | All       | All                            | SERIAL, SD           |
| OpenPilot REVO            | F4         | REVO, REVO_OPBL           | All  | All     | All            | All       | All                            | SERIAL, SPIFLASH     |

### Not recommended for new setups

These boards will work with INAV but are either end-of-life, limited on features, rare or hard to get from a reliable source.

| Board name                | CPU Family | Target name(s)            | GPS  | Compass | Barometer      | Telemetry | RX                             | Blackbox             |
|---------------------------|:----------:|:-------------------------:|:----:|:-------:|:--------------:|:---------:|:------------------------------:|:--------------------:|
| PARIS Sirius™ AIR HERO    | F1         | AIRHERO3                  | NMEA | HMC5883 | MS5611, BMP280 | LTM       | PWM, PPM, SBUS, IBUS, SPEKTRUM | SERIAL               |
| OpenPilot CC3D            | F1         | CC3D, CC3D_PPM1           | NMEA | HMC5883 | BMP085, BMP280 | LTM       | PWM, PPM, SBUS, IBUS, SPEKTRUM | no                   |
| AfroFlight NAZE32         | F1         | NAZE                      | NMEA | HMC5883 | MS5611, BMP280 | LTM       | PWM, PPM, SBUS, IBUS, SPEKTRUM | SERIAL, SPIFLASH     |
| RMRC Seriously DODO       | F3         | RMDO                      | All  | All     | All            | All       | All                            | SERIAL               |
| ANYFC                     | F4         | ANYFC                     | All  | All     | All            | All       | All                            | SERIAL               |
| YuPiF4 by RcNet           | F4         | YUPIF4                    | All  | All     | All            | All       | All                            | SERIAL, SD           |
