# Serial

Cleanflight has enhanced serial port flexibility and configuration is slightly more complex as a result.

Cleanflight has the concept of a function (MSP, GPS, Serial RX, etc) and a port (VCP, UARTx, SoftSerial x).
Not all functions can be used on all ports due to hardware pin mapping, conflicting features, hardware and software
constraints.

## Serial port types

* USB Virtual Com Port (VCP) - USB pins on a USB port connected directly to the processor without requiring
a dedicated USB to UART adapter.  VCP does not 'use' a physical UART port.
* UART - A pair of dedicated hardware transmit and receive pins with signal detection and generation done in hardware.
* SoftSerial - A pair of hardware transmit and receive pins with signal detection and generation done in software.

UART is the most efficent in terms of CPU usage.
SoftSerial is the least efficient and slowest, softserial should only be used for low-bandwith usages, such as telemetry transmission.

UART ports are sometimes exposed via on-board USB to UART converters, such as the CP2102 as found on the Naze and Flip32 boards.
If the flight controller does not have an onboard USB to UART converter and doesn't support VCP then an external USB to UART board is required.
These are sometimes referred to as FTDI boards.  FTDI is just a common manufacter of a chip (the FT232RL) used on USB to UART boards.

When selecting a USB to UART converter choose one that has DTR exposed as well as a selector for 3.3v and 5v since they are more useful.

Examples:
http://www.banggood.com/FT232RL-FTDI-USB-To-TTL-Serial-Converter-Adapter-Module-For-Arduino-p-917226.html
http://www.banggood.com/Wholesale-USB-To-TTL-Or-COM-Converter-Module-Buildin-in-CP2102-New-p-27989.html

Both SoftSerial and UART ports can be connected to your computer via USB to UART converter boards. 

## Serial Configuration

To make configuration easier common usage scenarios are listed below.

### Serial port scenarios

```
0   UNUSED
1   MSP, CLI, TELEMETRY, SMARTPORT TELEMETRY, GPS-PASSTHROUGH
2   GPS ONLY
3   RX SERIAL ONLY
4   TELEMETRY ONLY
5   MSP, CLI, GPS-PASSTHROUGH
6   CLI ONLY
7   GPS-PASSTHROUGH ONLY
8   MSP ONLY
9   SMARTPORT TELEMETRY ONLY
```

### Constraints

* There must always be a port available to use for MSP
* There must always be a port available to use for CLI
* To use a port for a function, the function's corresponding feature must be enabled first.
e.g. to use GPS enable the GPS feature.
* If the configuration is invalid the serial port configuration will reset to it's defaults and features may be disabled.
* If softserial is used be aware that both softserial ports must use the same baudrate.

### Examples

All examples assume default configuration (via cli `defaults` command)

a) Parallel PWM, GPS and FrSky TELEMETRY (when armed)

- TELEMETRY,MSP,CLI,GPS PASSTHROUGH on UART1
- GPS on UART2

```
feature RX_PARALLEL_PWM
feature TELEMETRY
feature GPS
set serial_port_2_scenario = 2
save
```

b) Graupner SumD RX SERIAL and HoTT TELEMETRY via Softserial

- MSP,CLI,GPS PASSTHROUGH on UART1
- RX SERIAL on UART2
- HoTT Telemetry on SOFTSERIAL1
```
feature -RX_PARALLEL_PWM
feature RX_SERIAL
feature TELEMETRY
feature SOFTSERIAL 
set serial_port_1_scenario = 5
set serialrx_provider = 3
set serial_port_2_scenario = 3
set telemetry_provider = 1
set serial_port_3_scenario = 4
save
```

c) PPM RX, GPS and FrSky TELEMETRY via softserial

- MSP,CLI,GPS PASSTHROUGH on UART1
- GPS on UART2
- TELEMETRY on SOFTSERIAL1

```
feature -RX_PARALLEL_PWM
feature RX_PPM
feature TELEMETRY
feature GPS
feature SOFTSERIAL
set serial_port_1_scenario = 5
set serial_port_2_scenario = 2
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

e) PPX RX, HoTT Telemetry via UART2

- MSP,CLI,GPS PASSTHROUGH on UART1
- HoTT telemetry on UART2

```
feature -RX_PARALLEL_PWM
feature RX_PPM
feature TELEMETRY
set serial_port_1_scenario = 5
set serial_port_2_scenario = 4
set telemetry_provider = 1
```

f) PPM RX, GPS, HoTT Telemetry via SoftSerial 1

- MSP,CLI,GPS PASSTHROUGH on UART1
- GPS on UART2
- HoTT telemetry on SOFTSERIAL1

```
feature -RX_PARALLEL_PWM
feature RX_PPM
feature TELEMETRY
feature GPS
feature SOFTSERIAL
set serial_port_2_scenario = 2
set serial_port_3_scenario = 4
set telemetry_provider = 1
save
```

g) SBus RX SERIAL 

- TELEMETRY,MSP,CLI,GPS PASSTHROUGH on UART1
- RX SERIAL on UART2

```
feature -RX_PARALLEL_PWM
feature RX_SERIAL
set serialrx_provider = 2
set serial_port_2_scenario = 3
save
```

h) Spektrum RX SERIAL 

- TELEMETRY,MSP,CLI,GPS PASSTHROUGH on UART1
- RX SERIAL on UART2

```
feature -RX_PARALLEL_PWM
feature RX_SERIAL
set serialrx_provider = 0
set serial_port_2_scenario = 3
save
```