## Serial port functions and scenarios

### Serial port scenarios

```
0   UNUSED
1   MSP, CLI, TELEMETRY, GPS-PASTHROUGH
2   GPS ONLY
3   RX SERIAL ONLY
4   TELEMETRY ONLY
5   MSP, CLI, GPS-PASTHROUGH
6   CLI ONLY
7   GPS-PASSTHROUGH ONLY
8   MSP ONLY
```

### Contraints

* There must always be a port available to use for MSP
* There must always be a port available to use for CLI
* To use a port for a function, the function's corresponding feature must be enabled first.
e.g. to use GPS enable the GPS feature.
* If the configuration is invalid the serial port configuration will reset to it's defaults and features may be disabled.

### Examples

All examples assume default configuration (via cli `defaults` command)

a) GPS and TELEMETRY (when armed)

- TELEMETRY,MSP,CLI,GPS PASSTHROUGH on UART1
- GPS on UART2

```
feature TELEMETRY
feature GPS
save
```

b) RX SERIAL and TELEMETRY (when armed)

- TELEMETRY,MSP,CLI,GPS PASSTHROUGH on UART1
- RX SERIAL on UART2

```
feature -RX_PARALLEL_PWM
feature RX_SERIAL
feature TELEMETRY
set serial_port_2_scenario = 3
save
```

b) RX SERIAL and TELEMETRY via softserial

- MSP,CLI,GPS PASSTHROUGH on UART1
- RX SERIAL on UART2
- TELEMETRY on SOFTSERIAL1

```
feature -RX_PARALLEL_PWM
feature RX_SERIAL
feature TELEMETRY
feature SOFTSERIAL
set serial_port_2_scenario = 3
set serial_port_3_scenario = 4
save
```

c) GPS and TELEMETRY via softserial

- MSP,CLI,GPS PASSTHROUGH on UART1
- GPS on UART2
- TELEMETRY on SOFTSERIAL1

```
feature -RX_PARALLEL_PWM
feature RX_PPM
feature TELEMETRY
feature GPS
feature SOFTSERIAL
set serial_port_3_scenario = 4
save
```
d) RX SERIAL, GPS and TELEMETRY (when armed) MSP/CLI via softserial

- GPS on UART1
- RX SERIAL on UART2
- TELEMETRY,MSP,CLI,GPS PASSTHROUGH on SOFTSERIAL1

```
feature -RX_PARALLEL_PWM
feature TELEMETRY
feature GPS
feature RX_SERIAL
feature SOFTSERIAL
set serial_port_1_scenario = 2
set serial_port_2_scenario = 3
set serial_port_3_scenario = 1
set msp_baudrate = 19200
set cli_baudrate = 19200
set gps_passthrough_baudrate = 19200
save
```

## FrSky telemetry

FrSky telemetry signals are inverted.  To connect a cleanflight capable board to an FrSKy receiver you have some options.

1. A hardware inverter - Built in to some flight controllers.
2. Use software serial and enable frsky_inversion.
3. Use a flight controller that has software configurable hardware inversion (e.g. STM32F30x).

For 1, just connect your inverter to a usart or software serial port.

For 2 and 3 use the cli command as follows:

```
set frsky_inversion = 1
```

## CLI command differences from baseflight

### gps_type
reason: renamed to `gps_provider` for consistency

### serialrx_type
reason: renamed to `serialrx_provider` for consistency

### rssi_aux_channel
reason: improved functionality

Cleanflight supports using any RX channel for rssi.  Baseflight only supports AUX1 to 4.

In Cleanflight a value of 0 disables the feature, a higher value indicates the channel number to read RSSI information from.

Example, to use RSSI on AUX1 in Cleanflight set the value to 5, since 5 is the first AUX channel.

### failsafe_detect_threshold
reason: improved functionality

See failsafe_min_usec and failsafe_max_usec in Failsafe documentation.
