## Serial port functions and scenarios

### Serial port scenarios

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

### Contraints

* There must always be a port available to use for MSP
* There must always be a port available to use for CLI
* To use a port for a function, the function's corresponding feature must be enabled first.
e.g. to use GPS enable the GPS feature.
* If the configuration is invalid the serial port configuration will reset to it's defaults and features may be disabled.

### Examples

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

## Migrating from baseflight

First ensure your main flight battery is disconnected or your props are off!

Before flashing with cleanflight, dump your configs for each profile via the CLI and save to a text file.

profile 0
dump
profile 1
dump
profile 2
dump

Then after flashing cleanflight paste the output from each dump command into the cli, switching profiles as you go.

You'll note that some commands are not recognised by cleanflight when you do this.  For the commands that are not recognised look
up the new configuration options and choose appropriate values for the settings.  See below for a list of differences.

Once you've done this for the first profile, save the config.  Then verify your config is OK, e.g. features serial ports, etc.
When you've verified the first profile is OK repeat for the other profiles.

It's also advisable to take screenshots of your AUX settings from baseflight configurator and then after re-applying the settings
verify your aux config is correct - some changes were made and some aux settings are not backwards compatible.


## CLI command differences from baseflight

In general all CLI commands use underscore characters to separate words for consistency.  In baseflight the format of CLI commands is somewhat haphazard.

### gps_baudrate
reason: simplify

Cleanflight uses normal baud rate values for gps baudrate, baseflight uses an index.

If an unsupported baud rate value is used the gps code will select 115200 baud.

example: `set gps_baudrate = 115200`


### gps_type
reason: renamed to `gps_provider` for consistency

### serialrx_type
reason: renamed to `serialrx_provider` for consistency

### rssi_aux_channel
reason: improved functionality

Cleanflight supports using any RX channel for rssi.  Baseflight only supports AUX1 to 4.

In Cleanflight a value of 0 disables the feature, a higher value indicates the channel number to read RSSI information from.

Example: to use RSSI on AUX1 in Cleanflight use `set rssi_aux_channel = 5`, since 5 is the first AUX channel.

### failsafe_detect_threshold
reason: improved functionality

See `failsafe_min_usec` and `failsafe_max_usec` in Failsafe documentation.

### emfavoidance
reason: renamed to `emf_avoidance` for consistency

### yawrate
reason: renamed to `yaw_rate` for consistency

### yawdeadband
reason: renamed to `yaw_deadband` for consistency

### midrc
reason: renamed to `midrc` for consistency

### mincheck
reason: renamed to `min_check` for consistency

### maxcheck
reason: renamed to `max_check` for consistency

### minthrottle
reason: renamed to `min_throttle` for consistency

### maxthrottle
reason: renamed to `max_throttle` for consistency

### mincommand
reason: renamed to `min_command` for consistency

### deadband3d_low
reason: renamed to `3d_deadband_low` for consistency

### deadband3d_high
reason: renamed to `3d_deadband_high` for consistency

### deadband3d_throttle
reason: renamed to `3d_deadband_throttle` for consistency

### neutral3d
reason: renamed to `3d_neutral` for consistency

### alt_hold_throttle_neutral
reason: renamed to 'alt_hold_deadband'


