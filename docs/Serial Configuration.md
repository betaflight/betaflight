# Serial Configuration

Cleanflight has enhanced serial port flexibility but configuration is slightly more complex as a result.

Cleanflight has the concept of a function (MSP, GPS, Serial RX, etc) and a port (USARTx, SoftSerial x).  Not all
functions can be used on all ports due to hardware pin mapping, conflicting features and software constraints.

To make configuration easier common usage scenarios are listed below.

## Serial port scenarios

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

## Contraints

* There must always be a port available to use for MSP
* There must always be a port available to use for CLI
* To use a port for a function, the function's corresponding feature must be enabled first.
e.g. to use GPS enable the GPS feature.
* If the configuration is invalid the serial port configuration will reset to it's defaults and features may be disabled.

## Examples

All examples assume default configuration (via cli `defaults` command)

a) GPS and FrSky TELEMETRY (when armed)

- TELEMETRY,MSP,CLI,GPS PASSTHROUGH on UART1
- GPS on UART2

```
feature TELEMETRY
feature GPS
save
```

b) RX SERIAL and FrSky TELEMETRY (when armed)

- TELEMETRY,MSP,CLI,GPS PASSTHROUGH on UART1
- RX SERIAL on UART2

```
feature -RX_PARALLEL_PWM
feature RX_SERIAL
feature TELEMETRY
set serial_port_2_scenario = 3
save
```

b) RX SERIAL and FrSky TELEMETRY via softserial

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

c) GPS and FrSky TELEMETRY via softserial

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

e) HoTT Telemetry via UART2

- MSP,CLI,GPS PASSTHROUGH on UART1
- HoTT telemetry on UART2

```
feature -RX_PARALLEL_PWM
feature RX_PPM
feature TELEMETRY
set serial_port_2_scenario = 4
set telemetry_provider = 1
```

f) GPS, HoTT Telemetry via SoftSerial 1

- MSP,CLI,GPS PASSTHROUGH on UART1
- GPS on UART2
- HoTT telemetry on SOFTSERIAL1

```
feature -RX_PARALLEL_PWM
feature RX_PPM
feature TELEMETRY
feature GPS
feature SOFTSERIAL
set serial_port_3_scenario = 4
set telemetry_provider = 1
save
```

