# Betaflight 4.4 cloud build API

## API

Avoiding EOL on 512K targets we have introduced a cloud build API saving around 25% of firmware flash usage.

Unified targets define hardware drivers to be included in the firmware as described in the [Hardware specification](https://github.com/betaflight/betaflight/blob/master/docs/Manufacturer%20Design%20Guidelines.md#42-definitions-for-unified-targets)

The build log has information about the build in case of failure.


## Usage

For optimal use please select ONLY the appropiate hardware for the flight controller after selecting the right target (using auto-detect button).


### Radio Protocols

    CRSF
    FPORT
    GHOST
    IBUS
    JETIEXBUS
    PPM
    SBUS
    SPECTRUM
    SRXL2
    SUMD
    SUMH
    XBUS

    EXPRESSLRS (SPI)
    CC2500 (SPI)
    SX1280 (SPI)
### Telemetry Protocols

    CRSF
    FRSKY_HUB
    GHOST
    HOTT
    IBUS_EXTENDED
    JETIEXBUS
    LTM
    MAVLINK
    SMARTPORT
    SRXL

### Other Options

    AKK (SA FIX)
    BARO
    FLASH
    GPS
    LED
    MAG
    OSD
    PINIO
    VTX

### Motor Protocols

    BRUSHED
    DSHOT
    MULTISHOT
    ONESHOT
    PROSHOT
    PWM

### Custom Defines

    BATTERY_CONTINUE
    DASHBOARD
    EMFAT_AUTORUN
    EMFAT_ICON
    ESCSERIAL_SIMONK
    GPS_PLUS_CODES
    SERIAL_4WAY_SK_BOOTLOADER

