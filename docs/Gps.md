# GPS

Two GPS protocols are supported. NMEA text and UBLOX Binary.

## Configuration

Enable the GPS from the CLI as follows:

1) first enable the `feature GPS`
2) set the `gps_provider`
3) if required, set your GPS baud rate using `gps_baudrate`
4) connect your GPS to a serial port that supports GPS and set an approprate `serial_port_x_scenario` to `2`. where `x` is a serial port number.
5) `save`.

For the last step check the Board documentation for pins and port numbers and check the Serial documentation for details on serial port scenarios where you will also find some example configurations. 

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
