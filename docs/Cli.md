# Command Line Interface (CLI)

Cleanflight has a command line interface (CLI) that can be used to change settings and configure the FC.

## Accessing the CLI.

The CLI can be accessed via the GUI tool or via a terminal emulator connected to the CLI serial port.

1. Connect your terminal emulator to the CLI serial port (which, by default, is the same as the MSP serial port)
2. Use the baudrate specified by cli_baudrate (115200 by default).
3. Send a `#` character.

To save your settings type in 'save', saving will reboot the flight controller.

To exit the CLI without saving power off the flight controller or type in 'exit'.

To see a list of other commands type in 'help' and press return.

To dump your configuration (including the current profile), use the 'dump' command.

See the other documentation sections for details of the cli commands and settings that are available.

## Backup via CLI

Disconnect main power, connect to cli via USB/FTDI.

dump using cli

`rate profile 0`
`profile 0`
`dump`

dump profiles using cli if you use them

`profile 1`
`dump profile`

`profile 2`
`dump profile`

dump rate profiles using cli if you use them

`rate profile 1`
`dump rates`

`rate profile 2`
`dump rates`

copy screen output to a file and save it.

## Restore via CLI.

Use the cli `defaults` command first.

When restoring from a backup it is a good idea to do a dump of the latest defaults so you know what has changed - if you do this each time a firmware release is created youwill be able to see the cli changes between firmware versions.  For instance, in December 2014 the default GPS navigation PIDs changed.  If you blindly restore your backup you would not benefit from these new defaults.

Use the CLI and send all the output from the saved from the backup commands.

Do not send the file too fast, if you do the FC might not be able to keep up when using USART adapters (including built in ones) since there is no hardware serial flow control.

You may find you have to copy/paste a few lines at a time.

Repeat the backup process again!

Compare the two backups to make sure you are happy with your restored settings.

Re-apply any new defaults as desired.

## CLI Command Reference

| Command        | Description                                    |
|----------------|------------------------------------------------|
| adjrange       | show/set adjustment ranges settings            |
| aux            | show/set aux settings                          |
| cmix           | design custom mixer                            |
| color          | configure colors                               |
| defaults       | reset to defaults and reboot                   |
| dump           | print configurable settings in a pastable form |
| exit           |                                                |
| feature        | list or -val or val                            |
| get            | get variable value                             |
| gpspassthrough | passthrough gps to serial                      |
| help           |                                                |
| led            | configure leds                                 |
| map            | mapping of rc channel order                    |
| mixer          | mixer name or list                             |
| motor          | get/set motor output value                     |
| profile        | index (0 to 2)                                 |
| rateprofile    | index (0 to 2)                                 |
| save           | save and reboot                                |
| set            | name=value or blank or * for list              |
| status         | show system status                             |
| version        |                                                |

## CLI Variable Reference

| Variable                      | Description/Units                   | Min    | Max    | Default       | Type         | Datatype |
|-------------------------------|-------------------------------------|--------|--------|---------------|--------------|----------|
| looptime                      |                                     | 0      | 9000   | 3500          | Master       | UINT16   |
| emf_avoidance                 |                                     | 0      | 1      | 0             | Master       | UINT8    |
| mid_rc                        |                                     | 1200   | 1700   | 1500          | Master       | UINT16   |
| min_check                     |                                     | 0      | 2000   | 1100          | Master       | UINT16   |
| max_check                     |                                     | 0      | 2000   | 1900          | Master       | UINT16   |
| rssi_channel                  |                                     | 0      | 18     | 0             | Master       | INT8     |
| rssi_scale                    |                                     | 1      | 255    | 30            | Master       | UINT8    |
| input_filtering_mode          |                                     | 0      | 1      | 0             | Master       | INT8     |
| min_throttle                  |                                     | 0      | 2000   | 1150          | Master       | UINT16   |
| max_throttle                  |                                     | 0      | 2000   | 1850          | Master       | UINT16   |
| min_command                   |                                     | 0      | 2000   | 1000          | Master       | UINT16   |
| servo_center_pulse            |                                     | 0      | 2000   | 1500          | Master       | UINT16   |
| 3d_deadband_low               |                                     | 0      | 2000   | 1406          | Master       | UINT16   |
| 3d_deadband_high              |                                     | 0      | 2000   | 1514          | Master       | UINT16   |
| 3d_neutral                    |                                     | 0      | 2000   | 1460          | Master       | UINT16   |
| 3d_deadband_throttle          |                                     | 0      | 2000   | 50            | Master       | UINT16   |
| motor_pwm_rate                | Default is 16000 for brushed motors | 50     | 32000  | 400           | Master       | UINT16   |
| servo_pwm_rate                |                                     | 50     | 498    | 50            | Master       | UINT16   |
| retarded_arm                  |                                     | 0      | 1      | 0             | Master       | UINT8    |
| disarm_kill_switch            |                                     | 0      | 1      | 1             | Master       | UINT8    |
| auto_disarm_delay             |                                     | 0      | 60     | 5             | Master       | UINT8    |
| small_angle                   |                                     | 0      | 180    | 25            | Master       | UINT8    |
| flaps_speed                   |                                     | 0      | 100    | 0             | Master       | UINT8    |
| fixedwing_althold_dir         |                                     | -1     | 1      | 1             | Master       | INT8     |
| serial_port_1_scenario        |                                     | 0      | 11     |               | Master       | UINT8    |
| serial_port_2_scenario        |                                     | 0      | 11     |               | Master       | UINT8    |
| serial_port_3_scenario        |                                     | 0      | 11     |               | Master       | UINT8    |
| serial_port_4_scenario        |                                     | 0      | 11     |               | Master       | UINT8    |
| serial_port_5_scenario        |                                     | 0      | 11     |               | Master       | UINT8    |
| reboot_character              |                                     | 48     | 126    | 82            | Master       | UINT8    |
| msp_baudrate                  |                                     | 1200   | 115200 | 115200        | Master       | UINT32   |
| cli_baudrate                  |                                     | 1200   | 115200 | 115200        | Master       | UINT32   |
| gps_baudrate                  |                                     | 0      | 115200 | 115200        | Master       | UINT32   |
| gps_passthrough_baudrate      |                                     | 1200   | 115200 | 115200        | Master       | UINT32   |
| gps_provider                  |                                     | 0      | 1      | 0             | Master       | UINT8    |
| gps_sbas_mode                 |                                     | 0      | 4      | 0             | Master       | UINT8    |
| gps_auto_config               |                                     | 0      | 1      | 1             | Master       | UINT8    |
| gps_auto_baud                 |                                     | 0      | 1      | 0             | Master       | UINT8    |
| gps_pos_p                     |                                     | 0      | 200    | 15            | Profile      | UINT8    |
| gps_pos_i                     |                                     | 0      | 200    | 0             | Profile      | UINT8    |
| gps_pos_d                     |                                     | 0      | 200    | 0             | Profile      | UINT8    |
| gps_posr_p                    |                                     | 0      | 200    | 34            | Profile      | UINT8    |
| gps_posr_i                    |                                     | 0      | 200    | 14            | Profile      | UINT8    |
| gps_posr_d                    |                                     | 0      | 200    | 53            | Profile      | UINT8    |
| gps_nav_p                     |                                     | 0      | 200    | 25            | Profile      | UINT8    |
| gps_nav_i                     |                                     | 0      | 200    | 33            | Profile      | UINT8    |
| gps_nav_d                     |                                     | 0      | 200    | 83            | Profile      | UINT8    |
| gps_wp_radius                 |                                     | 0      | 2000   | 200           | Profile      | UINT16   |
| nav_controls_heading          |                                     | 0      | 1      | 1             | Profile      | UINT8    |
| nav_speed_min                 |                                     | 10     | 2000   | 100           | Profile      | UINT16   |
| nav_speed_max                 |                                     | 10     | 2000   | 300           | Profile      | UINT16   |
| nav_slew_rate                 |                                     | 0      | 100    | 30            | Profile      | UINT8    |
| serialrx_provider             |                                     | 0      | 6      | 0             | Master       | UINT8    |
| spektrum_sat_bind             |                                     | 0      | 10     | 0             | Master       | UINT8    |
| telemetry_provider            |                                     | 0      | 3      | 0             | Master       | UINT8    |
| telemetry_switch              |                                     | 0      | 1      | 0             | Master       | UINT8    |
| telemetry_inversion           |                                     | 0      | 1      | 0             | Master       | UINT8    |
| frsky_default_lattitude       |                                     | -90    | 90     | 0             | Master       | FLOAT    |
| frsky_default_longitude       |                                     | -180   | 180    | 0             | Master       | FLOAT    |
| frsky_coordinates_format      |                                     | 0      | 1      | 0             | Master       | UINT8    |
| frsky_unit                    |                                     | 0      | 1      | 0             | Master       | UINT8    |
| battery_capacity              |                                     | 0      | 20000  | 0             | Master       | UINT16   |
| vbat_scale                    |                                     | 0      | 255    | 110           | Master       | UINT8    |
| vbat_max_cell_voltage         |                                     | 10     | 50     | 43            | Master       | UINT8    |
| vbat_min_cell_voltage         |                                     | 10     | 50     | 33            | Master       | UINT8    |
| vbat_warning_cell_voltage     |                                     | 10     | 50     | 35            | Master       | UINT8    |
| current_meter_scale           |                                     | -10000 | 10000  | 400           | Master       | INT16    |
| current_meter_offset          |                                     | 0      | 3300   | 0             | Master       | UINT16   |
| multiwii_current_meter_output |                                     | 0      | 1      | None defined! | Master       | UINT8    |
| current_meter_type            |                                     | 0      | 2      | 1             | Master       | UINT8    |
| align_gyro                    |                                     | 0      | 8      | 0             | Master       | UINT8    |
| align_acc                     |                                     | 0      | 8      | 0             | Master       | UINT8    |
| align_mag                     |                                     | 0      | 8      | 0             | Master       | UINT8    |
| align_board_roll              |                                     | -180   | 360    | 0             | Master       | INT16    |
| align_board_pitch             |                                     | -180   | 360    | 0             | Master       | INT16    |
| align_board_yaw               |                                     | -180   | 360    | 0             | Master       | INT16    |
| max_angle_inclination         |                                     | 100    | 900    | 500           | Master       | UINT16   |
| gyro_lpf                      |                                     | 0      | 256    | 42            | Master       | UINT16   |
| moron_threshold               |                                     | 0      | 128    | 32            | Master       | UINT8    |
| gyro_cmpf_factor              |                                     | 100    | 1000   | 600           | Master       | UINT16   |
| gyro_cmpfm_factor             |                                     | 100    | 1000   | 250           | Master       | UINT16   |
| alt_hold_deadband             |                                     | 1      | 250    | 40            | Profile      | UINT8    |
| alt_hold_fast_change          |                                     | 0      | 1      | 1             | Profile      | UINT8    |
| deadband                      |                                     | 0      | 32     | 0             | Profile      | UINT8    |
| yaw_deadband                  |                                     | 0      | 100    | 0             | Profile      | UINT8    |
| throttle_correction_value     |                                     | 0      | 150    | 0             | Profile      | UINT8    |
| throttle_correction_angle     |                                     | 1      | 900    | 800           | Profile      | UINT16   |
| yaw_control_direction         |                                     | -1     | 1      | 1             | Master       | INT8     |
| yaw_direction                 |                                     | -1     | 1      | 1             | Profile      | INT8     |
| tri_unarmed_servo             |                                     | 0      | 1      | 1             | Profile      | INT8     |
| default_rate_profile          | Default = profile number            | 0      | 2      |               | Profile      | UINT8    |
| rc_rate                       |                                     | 0      | 250    | 90            | Rate Profile | UINT8    |
| rc_expo                       |                                     | 0      | 100    | 65            | Rate Profile | UINT8    |
| thr_mid                       |                                     | 0      | 100    | 50            | Rate Profile | UINT8    |
| thr_expo                      |                                     | 0      | 100    | 0             | Rate Profile | UINT8    |
| roll_pitch_rate               |                                     | 0      | 100    | 0             | Rate Profile | UINT8    |
| yaw_rate                      |                                     | 0      | 100    | 0             | Rate Profile | UINT8    |
| tpa_rate                      |                                     | 0      | 100    | 0             | Rate Profile | UINT8    |
| tpa_breakpoint                |                                     | 1000   | 2000   | 1500          | Rate Profile | UINT16   |
| failsafe_delay                |                                     | 0      | 200    | 10            | Profile      | UINT8    |
| failsafe_off_delay            |                                     | 0      | 200    | 200           | Profile      | UINT8    |
| failsafe_throttle             |                                     | 1000   | 2000   | 1200          | Profile      | UINT16   |
| failsafe_min_usec             |                                     | 100    | 2000   | 985           | Profile      | UINT16   |
| failsafe_max_usec             |                                     | 100    | 3000   | 2115          | Profile      | UINT16   |
| gimbal_flags                  |                                     | 0      | 255    | 1             | Profile      | UINT8    |
| acc_hardware                  |                                     | 0      | 9      | 0             | Master       | UINT8    |
| acc_lpf_factor                |                                     | 0      | 250    | 4             | Profile      | UINT8    |
| accxy_deadband                |                                     | 0      | 100    | 40            | Profile      | UINT8    |
| accz_deadband                 |                                     | 0      | 100    | 40            | Profile      | UINT8    |
| accz_lpf_cutoff               |                                     | 1      | 20     | 5             | Profile      | FLOAT    |
| acc_unarmedcal                |                                     | 0      | 1      | 1             | Profile      | UINT8    |
| acc_trim_pitch                |                                     | -300   | 300    | 0             | Profile      | INT16    |
| acc_trim_roll                 |                                     | -300   | 300    | 0             | Profile      | INT16    |
| baro_tab_size                 |                                     | 0      | 48     | 21            | Profile      | UINT8    |
| baro_noise_lpf                |                                     | 0      | 1      | 0.6           | Profile      | FLOAT    |
| baro_cf_vel                   |                                     | 0      | 1      | 0.985         | Profile      | FLOAT    |
| baro_cf_alt                   |                                     | 0      | 1      | 0.965         | Profile      | FLOAT    |
| mag_hardware                  |                                     | 0      | 3      | 0             | Master       | UINT8    |
| mag_declination               |                                     | -18000 | 18000  | 0             | Profile      | INT16    |
| pid_controller                |                                     | 0      | 4      | 0             | Profile      | UINT8    |
| p_pitch                       |                                     | 0      | 200    | 40            | Profile      | UINT8    |
| i_pitch                       |                                     | 0      | 200    | 30            | Profile      | UINT8    |
| d_pitch                       |                                     | 0      | 200    | 23            | Profile      | UINT8    |
| p_roll                        |                                     | 0      | 200    | 40            | Profile      | UINT8    |
| i_roll                        |                                     | 0      | 200    | 30            | Profile      | UINT8    |
| d_roll                        |                                     | 0      | 200    | 23            | Profile      | UINT8    |
| p_yaw                         |                                     | 0      | 200    | 85            | Profile      | UINT8    |
| i_yaw                         |                                     | 0      | 200    | 45            | Profile      | UINT8    |
| d_yaw                         |                                     | 0      | 200    | 0             | Profile      | UINT8    |
| p_pitchf                      |                                     | 0      | 100    | 2.5           | Profile      | FLOAT    |
| i_pitchf                      |                                     | 0      | 100    | 0.6           | Profile      | FLOAT    |
| d_pitchf                      |                                     | 0      | 100    | 0.06          | Profile      | FLOAT    |
| p_rollf                       |                                     | 0      | 100    | 2.5           | Profile      | FLOAT    |
| i_rollf                       |                                     | 0      | 100    | 0.6           | Profile      | FLOAT    |
| d_rollf                       |                                     | 0      | 100    | 0.06          | Profile      | FLOAT    |
| p_yawf                        |                                     | 0      | 100    | 8             | Profile      | FLOAT    |
| i_yawf                        |                                     | 0      | 100    | 0.5           | Profile      | FLOAT    |
| d_yawf                        |                                     | 0      | 100    | 0.05          | Profile      | FLOAT    |
| level_horizon                 |                                     | 0      | 10     | 3             | Profile      | FLOAT    |
| level_angle                   |                                     | 0      | 10     | 5             | Profile      | FLOAT    |
| sensitivity_horizon           |                                     | 0      | 250    | 75            | Profile      | UINT8    |
| p_alt                         |                                     | 0      | 200    | 50            | Profile      | UINT8    |
| i_alt                         |                                     | 0      | 200    | 0             | Profile      | UINT8    |
| d_alt                         |                                     | 0      | 200    | 0             | Profile      | UINT8    |
| p_level                       |                                     | 0      | 200    | 90            | Profile      | UINT8    |
| i_level                       |                                     | 0      | 200    | 10            | Profile      | UINT8    |
| d_level                       |                                     | 0      | 200    | 100           | Profile      | UINT8    |
| p_vel                         |                                     | 0      | 200    | 120           | Profile      | UINT8    |
| i_vel                         |                                     | 0      | 200    | 45            | Profile      | UINT8    |
| d_vel                         |                                     | 0      | 200    | 1             | Profile      | UINT8    |
| blackbox_rate_num             |                                     | 1      | 32     | 1             | Master       | UINT8    |
| blackbox_rate_denom           |                                     | 1      | 32     | 1             | Master       | UINT8    |
