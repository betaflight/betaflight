# Telemetry

Telemetry allows you to know what is happening on your aircraft while you are flying it.  Among other things you can receive battery voltages and GPS positions on your transmitter.

Telemetry can be either always on, or enabled when armed.  If no serial port is set to be telemetry-only then telemetry will only be enabled when armed.

Telemetry is enabled using the 'TELEMETRY` feature.

```
feature TELEMETRY
```

Three telemetry providers are currently supported, FrSky (the default), Graupner HoTT V4 and MultiWii Serial Protocol (MSP)

Use the `telemetry_provider` cli command to select one.

| Value | Meaning         |
| ----- | --------------- |
| 0     | FrSky (Default) |
| 1     | HoTT            |
| 2     | MSP             |

Example:

```
set telemetry_provider = 1
```

There are further examples in the Configuration section of the documentation.

## FrSky telemetry

FrSky telemetry is transmit only and just requires a single connection from the TX pin of a serial port to the RX pin on an FrSky telemetry receiver.

FrSky telemetry signals are inverted.  To connect a cleanflight capable board to an FrSKy receiver you have some options.

1. A hardware inverter - Built in to some flight controllers.
2. Use software serial and enable frsky_inversion.
3. Use a flight controller that has software configurable hardware inversion (e.g. STM32F30x).

For 1, just connect your inverter to a usart or software serial port.

For 2 and 3 use the cli command as follows:

```
set frsky_inversion = 1
```

## HoTT telemetry

HoTT telemetry can be used when the TX and RX pins of a serial port are connected using a diode and a single wire to the T port on a HoTT receiver.

Only Electric Air Modules and GPS Modules are emulated, remember to enable them on your transmitter - in the Telemetry Menu on the MX-20.
 
Serial ports use two wires but HoTT uses a single wire so some electronics are required so that the signals don't get mixed up.

Connect as follows:
```
HoTT TX/RX -> Serial RX (connect directly)
Serial TX -> 1N4148 Diode -(|  )-> HoTT TX/RX (connect via diode)
```

The diode should be arranged to allow the data signals to flow the right way

```
-(|  )- == Diode, | indicates cathode marker.
```

As noticed by Skrebber the GR-12 (and probably GR-16/24, too) are based on a PIC 24FJ64GA-002, which has 5V tolerant digital pins.

Note: The softserial ports are not listed as 5V tolerant in the STM32F103xx data sheet pinouts and pin description section.  Verify if you require a 5v/3.3v level shifters.

## MultiWii Serial Protocol (MSP)

MSP Telemetry simply transmitts MSP packets in sequence to any MSP device attached to the telemetry port.  It rotates though a fixes sequence of command responses.

It is transmit only, it can work at any supported baud rate.

MSP telemetry is currently only output on serial ports that are set to MSP, NOT telemetry.
This will likely change in the future so that the MSP telemetry uses ports configured as telemetry just like the other providers do.
