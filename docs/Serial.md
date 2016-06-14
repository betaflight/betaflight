# Serial

INAV has enhanced serial port flexibility but configuration is slightly more complex as a result.

INAV has the concept of a function (MSP, GPS, Serial RX, etc) and a port (VCP, UARTx, SoftSerial x).
Not all functions can be used on all ports due to hardware pin mapping, conflicting features, hardware, and software
constraints.

## Serial port types

* USB Virtual Com Port (VCP) - USB pins on a USB port connected directly to the processor without requiring
a dedicated USB to UART adapter.  VCP does not 'use' a physical UART port.
* UART - A pair of dedicated hardware transmit and receive pins with signal detection and generation done in hardware.
* SoftSerial - A pair of hardware transmit and receive pins with signal detection and generation done in software.

UART is the most efficient in terms of CPU usage.
SoftSerial is the least efficient and slowest, SoftSerial should only be used for low-bandwidth usages, such as telemetry transmission.

UART ports are sometimes exposed via on-board USB to UART converters, such as the CP2102 as found on the Naze and Flip32 boards.
If the flight controller does not have an on-board USB to UART converter and doesn't support VCP then an external USB to UART board is required.
These are sometimes referred to as FTDI boards.  FTDI is just a common manufacturer of a chip (the FT232RL) used on USB to UART boards.

When selecting a USB to UART converter choose one that has DTR exposed as well as a selector for 3.3v and 5v since they are more useful.

Examples:

 * [FT232RL FTDI USB To TTL Serial Converter Adapter](http://www.banggood.com/FT232RL-FTDI-USB-To-TTL-Serial-Converter-Adapter-Module-For-Arduino-p-917226.html)
 * [USB To TTL / COM Converter Module buildin-in CP2102](http://www.banggood.com/Wholesale-USB-To-TTL-Or-COM-Converter-Module-Buildin-in-CP2102-New-p-27989.html)

Both SoftSerial and UART ports can be connected to your computer via USB to UART converter boards.

## Serial Configuration

Serial port configuration is best done via the configurator.

Configure serial ports first, then enable/disable features that use the ports.  To configure SoftSerial ports the SOFTSERIAL feature must be also be enabled.

### Constraints

If the configuration is invalid the serial port configuration will reset to its defaults and features may be disabled.

* There must always be a port available to use for MSP/CLI.
* There is a maximum of 2 MSP ports.
* To use a port for a function, the function's corresponding feature must be also be enabled.
e.g. after configuring a port for GPS enable the GPS feature.
* If SoftSerial is used, then all SoftSerial ports must use the same baudrate.
* Softserial is limited to 19200 buad.
* All telemetry systems except MSP will ignore any attempts to override the baudrate.
* MSP/CLI can be shared with EITHER Blackbox OR telemetry.  In shared mode blackbox or telemetry will be output only when armed.
* Smartport telemetry cannot be shared with MSP.
* No other serial port sharing combinations are valid.
* You can use as many different telemetry systems as you like at the same time.
* You can only use each telemetry system once.  e.g.  FrSky telemetry cannot be used on two port, but MSP Telemetry + FrSky on different ports is fine.

### Configuration via CLI

You can use the CLI for configuration but the commands are reserved for developers and advanced users.

The `serial` CLI command takes 6 arguments.

1. Identifier
1. Function bitmask (see serialPortFunction_e in the source)
1. MSP baud rate
1. GPS baud rate
1. Telemetry baud rate (auto baud allowed)
1. Blackbox baud rate


### Baud Rates

The allowable baud rates are as follows:

| Identifier | Baud rate |
| ---------- | --------- |
| 0          | Auto      |
| 1          | 9600      |
| 2          | 19200     |
| 3          | 38400     |
| 4          | 57600     |
| 5          | 115200    |
| 6          | 230400    |
| 7          | 250000    |
