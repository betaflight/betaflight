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

### CC3D

The CC3D has no battery divider, create one that gives you a 3.3v MAXIMUM output when your battery is
fully charged and connect the output from it to S5_IN/PA0/RC5.

S5_IN/PA0/RC5 is Pin 7 on the 8 pin connector, second to last pin, opposite end from the GND/+5/PPM signal input.

Note: When battery monitoring is enabled on the CC3D RC5 can no-longer be used for PWM input.
 
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

Configure capacity using the `battery_capacity` setting, which takes a value in mAh.

The current meter may need to be configured so that the value read at the ADC input matches actual current draw.  Just like you need a voltmeter to correctly calibrate your voltage reading you also need an ammeter to calibrate your current sensor.

Use the following settings to adjust calibration. 

`current_meter_scale`
`current_meter_offset`

If you're using an OSD that expects the multiwii current meter output value, then set `multiwii_current_meter_output` to `1`.
