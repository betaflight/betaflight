# Telemetry

Telemetry allows you to know what is happening on your aircraft while you are flying it.  Among other things you can receive battery voltages and GPS positions on your transmitter.

Telemetry can be either always on, or enabled when armed.  If no serial port is set to be telemetry-only then telemetry will only be enabled when armed.

Telemetry is enabled using the 'TELEMETRY` feature.

```
feature TELEMETRY
```

Multiple telemetry providers are currently supported, FrSky (the default), Graupner HoTT V4, SmartPort (S.Port) and MultiWii Serial Protocol (MSP)

Use the `telemetry_provider` cli command to select one.

| Value | Meaning         |
| ----- | --------------- |
| 0     | FrSky (Default) |
| 1     | HoTT            |
| 2     | MSP             |
| 3     | SmartPort       |

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

For 2 and 3 use the CLI command as follows:

```
set telemetry_inversion = 1
```

### Notes

RPM shows throttle output when armed.
RPM shows when diarmed.
TEMP2 shows Satellite Signal Quality when GPS is enabled.

RPM requires that the 'blades' setting is set to 12 on your receiver/display - tested with Taranis/OpenTX.

## HoTT telemetry

HoTT telemetry can be used when the TX and RX pins of a serial port are connected using a diode and a single wire to the T port on a HoTT receiver.

Only Electric Air Modules and GPS Modules are emulated, remember to enable them on your transmitter - in the Telemetry Menu on the MX-20.
 
Serial ports use two wires but HoTT uses a single wire so some electronics are required so that the signals don't get mixed up.

Connect as follows:

* HoTT TX/RX `T` -> Serial RX (connect directly)
* HoTT TX/RX `T` -> Diode `-(  |)-` > Serial TX (connect via diode)

The diode should be arranged to allow the data signals to flow the right way

```
-(  |)- == Diode, | indicates cathode marker.
```

1N4148 diodes have been tested and work with the GR-24.

As noticed by Skrebber the GR-12 (and probably GR-16/24, too) are based on a PIC 24FJ64GA-002, which has 5V tolerant digital pins.

Note: The SoftSerial ports are not listed as 5V tolerant in the STM32F103xx data sheet pinouts and pin description section.  Verify if you require a 5v/3.3v level shifters.

## MultiWii Serial Protocol (MSP)

MSP Telemetry simply transmitts MSP packets in sequence to any MSP device attached to the telemetry port.  It rotates though a fixes sequence of command responses.

It is transmit only, it can work at any supported baud rate.

## SmartPort (S.Port)

Smartport is a telemetry system used by newer FrSky transmitters and receivers such as the Taranis/XJR and X8R, X6R and X4R(SB).

Smartport telemetry is currently experimental, more information can be found here: https://github.com/frank26080115/cleanflight/wiki/Using-Smart-Port

In time this documentation will be updated with further details.
