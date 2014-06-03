# GPS

Two GPS protocols are supported. NMEA text and UBLOX Binary.

## GPS Provider

Set the `gps_provider` appropriately.

| Value | Meaning  |
| ----- | -------- |
| 0     | NMEA     |
| 1     | UBLOX    |

## SBAS

When using a UBLOX GPS the SBAS mode can be configured using `gps_sbas_mode`.

The default is AUTO.

| Value | Meaning  | Region        |
| ----- | -------- | ------------- |
| 0     | AUTO     | Global        |
| 1     | EGNOS    | Europe        |
| 2     | WAAS     | North America |
| 3     | MSAS     | Asia          |
| 4     | GAGAN    | India         |

If you use a regional specific setting you may achieve a faster GPS lock than using AUTO.
