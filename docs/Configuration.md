
## Serial port functions and scenarios

### Serial port scenarios

```
0   UNUSED
1   MSP, CLI, TELEMETRY, GPS-PASTHROUGH
2   GPS ONLY
3   SERIAL-RX ONLY
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

b) SERIAL_RX and TELEMETRY (when armed)

- TELEMETRY,MSP,CLI,GPS PASSTHROUGH on UART1
- SERIAL_RX on UART2

```
feature TELEMETRY
feature SERIAL_RX
set serial_port_2_scenario = 3
save
```

c) GPS and TELEMETRY via softserial

- TELEMETRY,MSP,CLI,GPS PASSTHROUGH on UART1
- GPS on UART2

```
feature TELEMETRY
feature GPS
feature SOFTSERIAL
set serial_port_3_scenario = 4
save
```
d) SERIAL_RX, GPS and TELEMETRY (when armed) MSP/CLI via softserial

- GPS on UART1
- SERIAL RX on UART2
- TELEMETRY,MSP,CLI,GPS PASSTHROUGH on SOFTSERIAL1
Note: PPM needed to disable parallel PWM which in turn allows SOFTSERIAL to be used.

```
feature PPM
feature TELEMETRY
feature GPS
feature SERIALRX
feature SOFTSERIAL
set serial_port_1_scenario = 2
set serial_port_2_scenario = 3
set serial_port_3_scenario = 1
set msp_baudrate = 19200
set cli_baudrate = 19200
set gps_passthrough_baudrate = 19200
save
```
