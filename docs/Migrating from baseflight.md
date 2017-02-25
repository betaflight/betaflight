# Migrating from baseflight

## Procedure

**First ensure your main flight battery is disconnected or your props are off!**

Before flashing with cleanflight, dump your configs for each profile via the CLI and save to a text file.

```
profile 0
dump
profile 1
dump
profile 2
dump
```

Then after flashing cleanflight paste the output from each dump command into the cli, switching profiles as you go.

You'll note that some commands are not recognised by cleanflight when you do this.  For the commands that are not recognised look
up the new configuration options and choose appropriate values for the settings.  See below for a list of differences.

Once you've done this for the first profile, save the config.  Then verify your config is OK, e.g. features serial ports, etc.
When you've verified the first profile is OK repeat for the other profiles.

It's also advisable to take screenshots of your AUX settings from baseflight configurator and then after re-applying the settings
verify your aux config is correct - aux settings are not backwards compatible.

## CLI command differences from baseflight

In general all CLI commands use underscore characters to separate words for consistency.  In baseflight the format of CLI commands is somewhat haphazard.

### gps_baudrate
reason: new serial port configuration.

See `serial` command.

### gps_type
reason: renamed to `gps_provider` for consistency

### serialrx_type
reason: renamed to `serialrx_provider` for consistency

### rssi_aux_channel
reason: renamed to `rssi_channel` for improved functionality

Cleanflight supports using any RX channel for rssi.  Baseflight only supports AUX1 to 4.

In Cleanflight a value of 0 disables the feature, a higher value indicates the channel number to read RSSI information from.

Example: to use RSSI on AUX1 in Cleanflight use `set rssi_channel = 5`, since 5 is the first AUX channel (this is equivalent to `set rssi_aux_channel = 1` in Baseflight).

### failsafe_detect_threshold
reason: improved functionality

See `rx_min_usec` and `rx_max_usec` in Failsafe documentation.

### emfavoidance
reason: renamed to `emf_avoidance` for consistency

### yawrate
reason: renamed to `yaw_rate` for consistency

### yawdeadband
reason: renamed to `yaw_deadband` for consistency

### midrc
reason: renamed to `mid_rc` for consistency

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
reason: renamed to `alt_hold_deadband` for consistency

### gimbal_flags
reason: seperation of features.

see `gimbal_mode` and `CHANNEL_FORWARDING` feature
