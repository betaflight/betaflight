# Battery Monitoring

Cleanflight has a battery monitoring feature.  The voltage of the main battery can be measured by the system and used to trigger a low-battery warning [buzzer](Buzzer.md), on-board status LED flashing and LED strip patterns.

Low battery warnings can:

* Help ensure you have time to safely land the aircraft
* Help maintain the life and safety of your LiPo/LiFe batteries, which should not be discharged below manufacturer recommendations

Minimum and maximum cell voltages can be set, and these voltages are used to auto-detect the number of cells in the battery when it is first connected.

Per-cell monitoring is not supported, as we only use one ADC to read the battery voltage.
  
## Supported targets

All targets support battery voltage monitoring unless status.

## Connections

When dealing with batteries **ALWAYS CHECK POLARITY!**

Measure expected voltages **first** and then connect to the flight controller.  Powering the flight controller with
incorrect voltage or reversed polarity will likely fry your flight controller. Ensure your flight controller
has a voltage divider capable of measuring your particular battery voltage.

### Naze32

The Naze32 has an on-board battery divider circuit; just connect your main battery to the VBAT connector.

**CAUTION:**  When installing the connection from main battery to the VBAT connector, be sure to first disconnect the main battery from the frame/power distribution board.  Check the wiring very carefully before connecting battery again.  Incorrect connections can immediately and completely destroy the flight controller and connected peripherals (ESC, GPS, Receiver etc.).

### CC3D

The CC3D has no battery divider.  To use voltage monitoring, you must create a divider that gives a 3.3v 
MAXIMUM output when the main battery is fully charged.  Connect the divider output to S5_IN/PA0/RC5.

Notes:

* S5_IN/PA0/RC5 is Pin 7 on the 8 pin connector, second to last pin, on the opposite end from the 
  GND/+5/PPM signal input.

* When battery monitoring is enabled on the CC3D, RC5 can no-longer be used for PWM input.

### Sparky

See the [Sparky board chapter](Board - Sparky.md).

## Configuration

Enable the `VBAT` feature.

Configure min/max cell voltages using the following CLI setting:

`vbat_scale` - Adjust this to match actual measured battery voltage to reported value.

`vbat_max_cell_voltage` - Maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, i.e. 43 = 4.3V

`set vbat_warning_cell_voltage` - Warning voltage per cell; this triggers battery-out alarms, in 0.1V units, i.e. 34 = 3.4V

`vbat_min_cell_voltage` - Minimum voltage per cell; this triggers battery-out alarms, in 0.1V units, i.e. 33 = 3.3V

e.g.

```
set vbat_scale = 110
set vbat_max_cell_voltage = 43
set vbat_warning_cell_voltage = 34
set vbat_min_cell_voltage = 33
```

# Current Monitoring

Current monitoring (amperage) is supported by connecting a current meter to the appropriate current meter ADC input (see the documentation for your particular board).

When enabled, the following values calculated and used by the telemetry and OLED display subsystems:
* Amps
* mAh used
* Capacity remaining

## Configuration

Enable current monitoring using the CLI command:

```
feature CURRENT_METER
```

Configure the current meter type using the `current_meter_type` settings here:

| Value | Sensor Type            |
| ----- | ---------------------- | 
| 0     | None                   |
| 1     | ADC/hardware sensor    |
| 2     | Virtual sensor         |

Configure capacity using the `battery_capacity` setting, in mAh units.

If you're using an OSD that expects the multiwii current meter output value, then set `multiwii_current_meter_output` to `1` (this multiplies amperage sent to MSP by 10).

### ADC Sensor

The current meter may need to be configured so the value read at the ADC input matches actual current draw.  Just like you need a voltmeter to correctly calibrate your voltage reading you also need an ammeter to calibrate the current sensor.

Use the following settings to adjust calibration:

`current_meter_scale`
`current_meter_offset`

### Virtual Sensor

The virtual sensor uses the throttle position to calculate an estimated current value. This is useful when a real sensor is not available. The following settings adjust the virtual sensor calibration:

| Setting                       | Description                                              |
| ----------------------------- | -------------------------------------------------------- | 
| `current_meter_scale`      | The throttle scaling factor [centiamps, i.e. 1/100th A]  |
| `current_meter_offset`     | The current at zero throttle (while disarmed) [centiamps, i.e. 1/100th A] |

There are two simple methods to tune these parameters:  one uses a battery charger and another depends on actual current measurements.

#### Tuning Using Actual Current Measurements
If you know your craft's current draw while disarmed (Imin) and at maximum throttle while armed (Imax), calculate the scaling factors as follows:
```
current_meter_scale = (Imax - Imin) * 100000 / (Tmax + (Tmax * Tmax / 50))
current_meter_offset = Imin * 100
```
Note: Tmax is maximum throttle offset (i.e. for `max_throttle` = 1850, Tmax = 1850 - 1000 = 850)

For example, assuming a maximum current of 34.2A, a minimum current of 2.8A, and a Tmax `max_throttle` = 1850:
```
current_meter_scale = (Imax - Imin) * 100000 / (Tmax + (Tmax * Tmax / 50))
                    = (34.2 - 2.8) * 100000 / (850 + (850 * 850 / 50))
                    = 205
current_meter_offset = Imin * 100 = 280
```
#### Tuning Using Battery Charger Measurement
If you cannot measure current draw directly, you can approximate it indirectly using your battery charger.  
However, note it may be difficult to adjust `current_meter_offset` using this method unless you can 
measure the actual current draw with the craft disarmed.

Note:
+ This method depends on the accuracy of your battery charger; results may vary.
+ If you add or replace equipment that changes the in-flight current draw (e.g. video transmitter, 
  camera, gimbal, motors, prop pitch/sizes, ESCs, etc.), you should recalibrate.

The general method is:

1. Fully charge your flight battery
2. Fly your craft, using >50% of your battery pack capacity (estimated)
3. Note Cleanflight's reported mAh draw
4. Re-charge your flight battery, noting the mAh charging data needed to restore the pack to fully charged
5. Adjust `current_meter_scale` to according to the formula given below
6. Repeat and test

Given (a) the reported mAh draw and the (b) mAh charging data, calculate a new `current_meter_scale` value as follows:
```
current_meter_scale = (charging_data_mAh / reported_draw_mAh) * old_current_meter_scale
```
For example, assuming:
+ A Cleanflight reported current draw of 1260 mAh
+ Charging data to restore full charge of 1158 mAh
+ A existing `current_meter_scale` value of 400 (the default)

Then the updated `current_meter_scale` is:
```
current_meter_scale = (charging_data_mAh / reported_draw_mAh) * old_current_meter_scale
                    = (1158 / 1260) * 400
                    = 368
```





