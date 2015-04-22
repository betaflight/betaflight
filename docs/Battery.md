# Battery Monitoring

Cleanflight has a battery monitoring feature.  Battery voltage of the main battery can be measured by the system and used
to trigger a low-battery warning buzzer, on-board status LED flashing and LED strip patterns.

Low battery warnings can:

* help to ensure that you have time to safely land the aircraft.
* help maintain the life and safety of your LiPo/LiFe batteries which should not be discharged below manufacturers recommendations.

Minimum and maximum cell voltages can be set, and these voltages are used to detect the amount of cells you are using.

Per-cell monitoring is not supported, as we only use one ADC to read the battery voltage.
  
## Supported targets

All targets support battery voltage monitoring unless status.

## Connections

When dealing with batteries **ALWAYS CHECK POLARITY!**

Measure expected voltages first and then connect to flight controller, connecting to the flight controller with
incorrect voltage or reversed polarity will likely fry your flight controller. Ensure that your flight controller
has a voltage divider that is capable of measuring your particular battery voltage.

### Naze32

The Naze32 has an on-board battery divider circuit, connect your main battery to the VBAT connector.

**CAUTION:**  When installing connection from main battery to the VBAT connector, be sure to first disconnect the main battery from the frame / power distribution board.  Check the wiring very carefully before connecting battery again.  Incorrect connections can immediately and completely destroy the flight controller and connected peripherals (ESC, GPS, Receiver etc.)

### CC3D

The CC3D has no battery divider, create one that gives you a 3.3v MAXIMUM output when your battery is
fully charged and connect the output from it to S5_IN/PA0/RC5.

S5_IN/PA0/RC5 is Pin 7 on the 8 pin connector, second to last pin, opposite end from the GND/+5/PPM signal input.

Note: When battery monitoring is enabled on the CC3D RC5 can no-longer be used for PWM input.

### Sparky

See Sparky board chapter.

## Configuration

Enable the `VBAT` feature.

Configure min/max cell voltages using the following CLI setting:

`vbat_scale` - adjust this to match battery voltage to reported value.

`vbat_max_cell_voltage` - maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, i.e. 43 = 4.3V

`set vbat_warning_cell_voltage` - warning voltage per cell, this triggers battery out alarms, in 0.1V units, i.e. 34 = 3.4V

`vbat_min_cell_voltage` - minimum voltage per cell, this triggers battery out alarms, in 0.1V units, i.e. 33 = 3.3V

e.g.

```
set vbat_scale = 110
set vbat_max_cell_voltage = 43
set vbat_warning_cell_voltage = 34
set vbat_min_cell_voltage = 33
```

# Current Monitoring

Current monitoring (Amperage) is supported by connecting a current meter to the appropriate current meter ADC input (See Board documentation).

When enabled, Amps, mAh used and capacity remaining are calculated and used by the telemetry and OLED display subsystems.

## Configuration

Enable current monitoring using the CLI command

```
feature CURRENT_METER
```

Configure the current meter type using the `current_meter_type` settings as per the following table.

| Value | Sensor Type            |
| ----- | ---------------------- | 
| 0     | None                   |
| 1     | ADC/hardware sensor    |
| 2     | Virtual sensor         |

Configure capacity using the `battery_capacity` setting, which takes a value in mAh.

If you're using an OSD that expects the multiwii current meter output value, then set `multiwii_current_meter_output` to `1` (this multiplies amperage sent to MSP by 10).

### ADC Sensor
The current meter may need to be configured so that the value read at the ADC input matches actual current draw.  Just like you need a voltmeter to correctly calibrate your voltage reading you also need an ammeter to calibrate your current sensor.

Use the following settings to adjust calibration. 

`current_meter_scale`
`current_meter_offset`

### Virtual Sensor
The virtual sensor uses the throttle position to calculate as estimated current value. This is useful when a real sensor is not available. The following settings adjust the calibration.

| Setting                       | Description                                              |
| ----------------------------- | -------------------------------------------------------- | 
| `current_meter_scale`      | The throttle scaling factor [centiamps, i.e. 1/100th A]  |
| `current_meter_offset`     | The current at zero throttle (while disarmed) [centiamps, i.e. 1/100th A] |

There are two simple methods to tune the parameters depending in whether it is possible to measure current draw for your craft.

#### Tuning Using Battery Charger Measurement
It may be difficult to adjust `current_meter_offset` using this method unless you can measure the actual current draw with the craft disarmed. Adjust `current_meter_scale` until the mAh draw reported by Cleanflight matches the charging data given by your battery charger after the flight (if the mAh draw is lower than reported by your battery charger, increase `current_meter_scale`, and vice-versa).
#### Tuning Using Actual Current Measurements
If you know your crafts current draw while disarmed (Imin), and at maximum throttle while armed (Imax), you can calculate the scaling factors using the following formulas where Tmax is maximum throttle offset (i.e. for `max_throttle` = 1850, Tmax = 1850 - 1000 = 850):
```
current_meter_scale = (Imax - Imin) * 100000 / (Tmax + (Tmax * Tmax / 50))
current_meter_offset = Imin * 100
```
e.g. For a maximum current of 34.2 A and minimum current of 2.8 A with `max_throttle` = 1850
```
current_meter_scale = (Imax - Imin) * 100000 / (Tmax + (Tmax * Tmax / 50))
                    = (34.2 - 2.8) * 100000 / (850 + (850 * 850 / 50))
                    = 205
current_meter_offset = Imin * 100 = 280
```
