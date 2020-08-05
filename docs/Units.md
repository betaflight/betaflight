# Units

This document describes the current variables used to configure specific elements depending on the measurement unit applied.

### Units
| Unit     | Speed | Distance | Temperature |
| -------- | ----- | -------- | ----------- |
| Imperial | MPH   | Miles    | Fahrenheit  |
| Metric   | KPH   | KM       | Celcius     |
| British  | MPH   | KM       | Celcius     |

### Affected OSD Elements
| OSD Element      | Imperial | Metric | British |
| ---------------- | -------- | ------ | ------- |
| Altitude         | Feet     | Metre  | Metre   |
| GPS Speed        | MPH      | KPH    | MPH     |
| Home Distance    | Feet     | Metre  | Metre   |
| Numerical Vario  | FTPS     | MPS    | MPS     |
| Flight Distance  | Feet     | Metre  | Metre   |
| OSD Efficiency   | Miles    | KM     | KM      |

Note: Configuration is done in cli with `set osd_units = <UNIT>` or in the OSD tab in the Betaflight Configurator. For example `set osd_units = BRITISH`

### Affected FrSky Hub Telemetry Elements
| Element | Imperial   | Metric  | British |
| ------- | ---------- | ------- | ------- |
| HDOP    | Fahrenheit | Celcius | Celcius |

The FrSky hub telemetry setting can be changed in cli by `set frsky_unit = <UNIT>`.
For example `set frsky_unit = METRIC`
