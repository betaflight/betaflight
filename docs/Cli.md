# Command Line Interface (CLI)

INAV has a command line interface (CLI) that can be used to change settings and configure the FC.

## Accessing the CLI.

The CLI can be accessed via the GUI tool or via a terminal emulator connected to the CLI serial port.

1. Connect your terminal emulator to the CLI serial port (which, by default, is the same as the MSP serial port)
2. Use the baudrate specified by msp_baudrate (115200 by default).
3. Send a `#` character.

To save your settings type in 'save', saving will reboot the flight controller.

To exit the CLI without saving power off the flight controller or type in 'exit'.

To see a list of other commands type in 'help' and press return.

To dump your configuration (including the current profile), use the 'dump' command.

See the other documentation sections for details of the cli commands and settings that are available.

## Backup via CLI

Disconnect main power, connect to cli via USB/FTDI.

dump using cli

```
profile 0
dump
```

dump profiles using cli if you use them

```
profile 1
dump profile
profile 2
dump profile
```

copy screen output to a file and save it.

## Restore via CLI.

Use the cli `defaults` command first.

When restoring from a backup it is a good idea to do a dump of the latest defaults so you know what has changed - if you do this each time a firmware release is created youwill be able to see the cli changes between firmware versions.  For instance, in December 2014 the default GPS navigation PIDs changed.  If you blindly restore your backup you would not benefit from these new defaults.

Use the CLI and send all the output from the saved backup commands.

Do not send the file too fast, if you do the FC might not be able to keep up when using USART adapters (including built in ones) since there is no hardware serial flow control.

You may find you have to copy/paste a few lines at a time.

Repeat the backup process again!

Compare the two backups to make sure you are happy with your restored settings.

Re-apply any new defaults as desired.

## CLI Command Reference

| `Command`        | Description                                    |
|------------------|------------------------------------------------|
| `1wire <esc>`    | passthrough 1wire to the specified esc         |
| `adjrange`       | show/set adjustment ranges settings            |
| `aux`            | show/set aux settings                          |
| `mmix`           | design custom motor mixer                      |
| `smix`           | design custom servo mixer                      |
| `color`          | configure colors                               |
| `defaults`       | reset to defaults and reboot                   |
| `dump`           | print configurable settings in a pastable form |
| `exit`           |                                                |
| `feature`        | list or -val or val                            |
| `get`            | get variable value                             |
| `gpspassthrough` | passthrough gps to serial                      |
| `help`           |                                                |
| `led`            | configure leds                                 |
| `map`            | mapping of rc channel order                    |
| `mixer`          | mixer name or list                             |
| `motor`          | get/set motor output value                     |
| `play_sound`     | index, or none for next                        |
| `profile`        | index (0 to 2)                                 |
| `rxrange`        | configure rx channel ranges (end-points) |
| `save`           | save and reboot                                |
| `set`            | name=value or blank or * for list              |
| `status`         | show system status                             |
| `version`        |                                                |

## CLI Variable Reference

|  Variable Name | Default Value | Description |
|  ------ | ------ | ------ |
|  looptime  | 2000 | This is the main loop time (in us). Changing this affects PID effect with some PID controllers (see PID section for details). Default of 3500us/285Hz should work for everyone. Setting it to zero does not limit loop time, so it will go as fast as possible. |
|  i2c_speed | 400KHZ | This setting controls the clock speed of I2C bus. 400KHZ is the default that most setups are able to use. Some noise-free setups may be overclocked to 800KHZ. Some sensor chips or setups with long wires may work unreliably at 400KHZ - user can try lowering the clock speed to 200KHZ or even 100KHZ. User need to bear in ming that lower clock speeds might require higher looptimes (lower looptime rate) |
|  cpu_underclock  | OFF | This option is only available on certain architectures (F3 CPUs at the moment). It makes CPU clock lower to reduce interference to long-range RC systems working at 433MHz |
|  gyro_sync  | OFF | This option enables gyro_sync feature. In this case the loop will be synced to gyro refresh rate. Loop will always wait for the newest gyro measurement. Use gyro_lpf and gyro_sync_denom determine the gyro refresh rate. Note that different targets have different limits. Setting too high refresh rate can mean that FC cannot keep up with the gyro and higher gyro_sync_denom is needed, |
|  gyro_sync_denom  | 2 | This option determines the sampling ratio. Denominator of 1 means full gyro sampling rate. Denominator 2 would mean 1/2 samples will be collected. Denominator and gyro_lpf will together determine the control loop speed. |
|  acc_task_frequency  | 500 | Determines accelerometer task frequency in async_mode = ALL. Depending on UAV type this frequency can be lowered to conserve CPU resources as long as vibrations are not a problem. |
|  attitude_task_frequency  | 250 | Determines attitude task frequency when async_mode = ALL |
|  async_mode  | NONE | Enables asynchronous mode processing for gyro/accelerometer and attitude computations. Allowed modes: NONE -> default behavior, all calculations are executed in main PID loop. GYRO -> gyro samling and filtering is detached from main PID loop. PID loop runs based on looptime while gyro sampling uses gyro_sync_denom and gyro_lpf combination to determine its frequency. ALL -> in this mode, gyro, accelerometer and attitude are running as separate tasks. Accelerometer task frequency is determined by acc_task_frequency, attitude task frequency by attitude_task_frequency. In this mode ANGLE and HORIZON, as well GPS assisted flight modes (including PosHold) performance might be lowered. |
|  mid_rc  | 1500 | This is an important number to set in order to avoid trimming receiver/transmitter. Most standard receivers will have this at 1500, however Futaba transmitters will need this set to 1520. A way to find out if this needs to be changed, is to clear all trim/subtrim on transmitter, and connect to GUI. Note the value most channels idle at - this should be the number to choose. Once midrc is set, use subtrim on transmitter to make sure all channels (except throttle of course) are centered at midrc value. |
|  min_check  | 1100 | These are min/max values (in us) which, when a channel is smaller (min) or larger (max) than the value will activate various RC commands, such as arming, or stick configuration. Normally, every RC channel should be set so that min = 1000us, max = 2000us. On most transmitters this usually means 125% endpoints. Default check values are 100us above/below this value. |
|  max_check  | 1900 | These are min/max values (in us) which, when a channel is smaller (min) or larger (max) than the value will activate various RC commands, such as arming, or stick configuration. Normally, every RC channel should be set so that min = 1000us, max = 2000us. On most transmitters this usually means 125% endpoints. Default check values are 100us above/below this value. |
|  rssi_channel  | 0 | RX channel containing the RSSI signal |
|  rssi_scale  | 30 | When using ADC RSSI, the raw ADC value will be divided by rssi_scale in order to get the RSSI percentage. RSSI scale is therefore the ADC raw value for 100% RSSI. |
|  rssi_invert  | OFF |  |
|  rc_smoothing  | ON | Interpolation of Rc data during looptimes when there are no new updates. This gives smoother RC input to PID controller and cleaner PIDsum |
|  input_filtering_mode  | OFF | Filter out noise from OpenLRS Telemetry RX |
|  min_throttle  | 1150 | These are min/max values (in us) that are sent to esc when armed. Defaults of 1150/1850 are OK for everyone, for use with AfroESC, they could be set to 1064/1864. |
|  max_throttle  | 1850 | These are min/max values (in us) that are sent to esc when armed. Defaults of 1150/1850 are OK for everyone, for use with AfroESC, they could be set to 1064/1864. If you have brushed motors, the value should be set to 2000. |
|  min_command  | 1000 | This is the PWM value sent to ESCs when they are not armed. If ESCs beep slowly when powered up, try decreasing this value. It can also be used for calibrating all ESCs at once. |
|  3d_deadband_low  | 1406 | Low value of throttle deadband for 3D mode (when stick is in the 3d_deadband_throttle range, the fixed values of 3d_deadband_low / _high are used instead) |
|  3d_deadband_high  | 1514 | High value of throttle deadband for 3D mode (when stick is in the deadband range, the value in 3d_neutral is used instead) |
|  3d_neutral  | 1460 | Neutral (stop) throttle value for 3D mode |
|  3d_deadband_throttle  | 50 | Throttle signal will be held to a fixed value when throttle is centered with an error margin defined in this parameter. |
|  motor_pwm_rate  | 400 | Output frequency (in Hz) for motor pins. Default is 400Hz for motor with motor_pwm_protocol set to STANDARD. For *SHOT (e.g. ONESHOT125) values of 1000 and 2000 have been tested by the development team and are supported. It may be possible to use higher values. For BRUSHED values of 8000 and above should be used. Setting to 8000 will use brushed mode at 8kHz switching frequency. Up to 32kHz is supported for brushed. Default is 16000 for boards with brushed motors. Note, that in brushed mode, minthrottle is offset to zero. For brushed mode, set max_throttle to 2000. |
|  motor_pwm_protocol  | STANDARD | Protocol that is used to send motor updates to ESCs. Possible values - STANDARD, ONESHOT125, ONESHOT42, MULTISHOT, BRUSHED |
|  fixed_wing_auto_arm  | OFF | Auto-arm fixed wing aircraft on throttle above min_throttle, and disarming with stick commands are disabled, so power cycle is requirred to disarm. Requirres enabled motorstop and no arm switch configured. |
|  disarm_kill_switch  | ON | Disarms the motors independently of throttle value. Setting to OFF reverts to the old behaviour of disarming only when the throttle is low. Only applies when arming and disarming with an AUX channel. |
|  auto_disarm_delay  | 5 | Delay before automatic disarming when using stick arming and MOTOR_STOP. This does not apply when using FIXED_WING |
|  small_angle  | 25 | If the aircraft tilt angle exceed this value the copter will refuse to arm.  |
|  reboot_character  | 82 | Special character used to trigger reboot |
|  gps_provider  | UBLOX | Which GPS protocol to be used |
|  gps_sbas_mode  | NONE | Which SBAS mode to be used |
|  gps_dyn_model  | AIR_1G | GPS navigation model: Pedestrian, Air_1g, Air_4g. Default is AIR_1G. Use pedestrian with caution, can cause flyaways with fast flying. |
|  gps_auto_config  | ON | Enable automatic configuration of UBlox GPS receivers. |
|  gps_auto_baud  | ON | Automatic configuration of GPS baudrate(The spesified baudrate in configured in ports will be used) when used with UBLOX GPS. When used with NAZA/DJI it will automatic detect GPS baudrate and change to it, ignoring the selected baudrate set in ports |
|  gps_min_sats  | 6 | Minimum number of GPS satellites in view to acquire GPS_FIX and consider GPS position valid. Some GPS receivers appeared to be very inaccurate with low satellite count. |
|  inav_auto_mag_decl  | ON | Automatic setting of magnetic declination based on GPS position. When used manual magnetic declination is ignored. |
|  inav_gravity_cal_tolerance  | 5 | Unarmed gravity calibration tolerance level. Won't finish the calibration until estimated gravity error falls below this value. |
|  inav_use_gps_velned  | ON | Defined if iNav should use velocity data provided by GPS module for doing position and speed estimation. If set to OFF iNav will fallback to calculating velocity from GPS coordinates. Using native velocity data may improve performance on some GPS modules. Some GPS modules introduce significant delay and using native velocity may actually result in much worse performance. |
|  inav_gps_delay  | 200 | GPS position and velocity data usually arrive with a delay. This parameter defines this delay. Default (200) should be reasonable for most GPS receivers. |
|  inav_reset_altitude | FIRST_ARM | Defines when relative estimated altitude is reset to zero. Variants - `NEVER` (once reference is acquired it's used regardless); `FIRST_ARM` (keep altitude at zero until firstly armed), `EACH_ARM` (altitude is reset to zero on each arming) |
|  inav_max_surface_altitude  | 200 | Max allowed altitude for surface following mode. [cm] |
|  inav_w_z_baro_p  | 0.350 | Weight of barometer measurements in estimated altitude and climb rate |
|  inav_w_z_gps_p  | 0.200 | Weight of GPS altitude measurements in estimated altitude. Setting is used only of airplanes |
|  inav_w_z_gps_v  | 0.500 | Weight of GPS climb rate measurements in estimated climb rate. Setting is used on both airplanes and multirotors. If GPS doesn't support native climb rate reporting (i.e. NMEA GPS) you may consider setting this to zero |
|  inav_w_xy_gps_p  | 1.000 | Weight of GPS coordinates in estimated UAV position and speed. |
|  inav_w_xy_gps_v  | 2.000 | Weight of GPS velocity data in estimated UAV speed |
|  inav_w_z_res_v  | 0.500 | Decay coefficient for estimated climb rate when baro/GPS reference for altitude is lost |
|  inav_w_xy_res_v  | 0.500 | Decay coefficient for estimated velocity when GPS reference for position is lost |
|  inav_w_acc_bias  | 0.010 | Weight for accelerometer drift estimation |
|  inav_max_eph_epv  | 1000.000 | Maximum uncertainty value until estimated position is considered valid and is used for navigation [cm] |
|  inav_baro_epv  | 100.000 | Uncertainty value for barometric sensor [cm] |
|  nav_disarm_on_landing  | OFF | If set to ON, iNav disarms the FC after landing |
|  nav_use_midthr_for_althold  | OFF | If set to OFF, the FC remembers your throttle stick position when enabling ALTHOLD and treats it as a netraul midpoint for holding altitude |
|  nav_extra_arming_safety  | ON | If set to ON drone won't arm if no GPS fix and any navigation mode like RTH or POSHOLD is configured |
|  nav_user_control_mode  | ATTI | Defines how Pitch/Roll input from RC receiver affects flight in POSHOLD mode: ATTI - right stick controls attitude like in ANGLE mode; CRUISE - right stick controls velocity in forward and right direction. |
|  nav_position_timeout  | 5 | If GPS fails wait for this much seconds before switching to emergency landing mode (0 - disable) |
|  nav_wp_radius  | 100 | Waypoint radius [cm]. Waypoint would be considered reached if machine is within this radius |
|  nav_wp_safe_distance  | 10000 | First waypoint in the mission should be closer than this value [cm] |
|  nav_auto_speed  | 300 | Maximum velocity firmware is allowed in full auto modes (POSHOLD, RTH, WP) [cm/s] [Multirotor only] |
|  nav_auto_climb_rate  | 500 | Maximum climb/descent rate that UAV is allowed to reach during navigation modes. [cm/s] |
|  nav_manual_speed  | 500 | Maximum velocity firmware is allowed when processing pilot input for POSHOLD/CRUISE control mode [cm/s] [Multirotor only] |
|  nav_manual_climb_rate  | 200 | Maximum climb/descent rate firmware is allowed when processing pilot input for ALTHOLD control mode [cm/s] |
|  nav_landing_speed  | 200 | Vertical descent velocity during the RTH landing phase. [cm/s] |
|  nav_land_slowdown_minalt  | 500 | Defines at what altitude the descent velocity should start to be 25% of nav_landing_speed [cm] |
|  nav_land_slowdown_maxalt  | 2000 | Defines at what altitude the descent velocity should start to ramp down from 100% nav_landing_speed to 25% nav_landing_speed. [cm] |
|  nav_emerg_landing_speed  | 500 | Rate of descent UAV will try to maintain when doing emergency descent sequence |
|  nav_min_rth_distance  | 500 | Minimum distance from homepoint when RTH can be activated [cm] |
|  nav_rth_climb_first  | ON | If set to ON drone will climb to nav_rth_altitude first and head home afterwards. If set to OFF drone will head home instantly and climb on the way. |
|  nav_rth_tail_first  | OFF | If set to ON drone will return tail-first. Obviously meaningless for airplanes. |
|  nav_rth_allow_landing  | ON | If set to ON drone will land as a last phase of RTH. |
|  nav_rth_climb_ignore_emerg  | ON | If set to ON, aircraft will execute initial climb regardless of position sensor (GPS) status. |
|  nav_rth_alt_mode  | AT_LEAST | Configure how the aircraft will manage altitude on the way home, se Navigation modes on wiki for more details |
|  nav_rth_altitude  | 1000 | Used in EXTRA, FIXED and AT_LEAST rth alt modes (Default 1000 means 10 meters) |
|  nav_rth_abort_threshold  | 50000 | RTH sanity checking feature will notice if distance to home is increasing during RTH and once amount of increase exceeds the threshold defined by this parameter, instead of continuing RTH machine will enter emergency landing, self-level and go down safely. Default is 500m which is safe enough for both multirotor machines and airplanes. |
|  nav_mc_bank_angle  | 30 | Maximum banking angle (deg) that multicopter navigation is allowed to set. Machine must be able to satisfy this angle without loosing altitude |
|  nav_mc_hover_thr  | 1500 | Multicopter hover throttle hint for altitude controller. Should be set to approximate throttle value when drone is hovering. |
|  nav_mc_auto_disarm_delay  | 2000 |  |
|  nav_fw_cruise_thr  | 1400 | Cruise throttle in GPS assisted modes, this includes RTH. Should be set high enough to avoid stalling. This values gives INAV a base for throttle when flying straight, and it will increase or decrease throttle based on pitch of airplane and the parameters below. In addiotional it will increase throttle if GPS speed gets below 7m/s ( hardcoded )  |
|  nav_fw_min_thr  | 1200 | Minimum throttle for flying wing in GPS assisted modes |
|  nav_fw_max_thr  | 1700 | Maximum throttle for flying wing in GPS assisted modes |
|  nav_fw_bank_angle  | 20 | Max roll angle when rolling / turning in GPS assisted modes, is also restrained by global max_angle_inclination_rll |
|  nav_fw_climb_angle  | 20 | Max pitch angle when climbing in GPS assisted modes, is also restrained by global max_angle_inclination_pit |
|  nav_fw_dive_angle  | 15 | Max negative pitch angle when diving in GPS assisted modes, is also restrained by global max_angle_inclination_pit |
|  nav_fw_pitch2thr  | 10 | Amount of throttle applied related to pitch attitude in GPS assisted modes |
|  nav_fw_loiter_radius  | 5000 | PosHold radius. 3000 to 7500 is a good value (30-75m) [cm] |
|  nav_fw_launch_velocity  | 300 | Forward velocity threshold for swing-launch detection [cm/s] |
|  nav_fw_launch_accel  | 1863 | Forward acceleration threshold for bungee launch of throw launch [cm/s/s], 1G = 981 cm/s/s |
|  nav_fw_launch_max_angle  | 45 | Max tilt angle (pitch/roll combined) to consider launch successful. Set to 180 to disable completely [deg] |
|  nav_fw_launch_detect_time  | 40 | Time for which thresholds have to breached to consider launch happened [ms] |
|  nav_fw_launch_thr  | 1700 | Launch throttle - throttle to be set during launch sequence (pwm units) |
|  nav_fw_launch_idle_thr       | 1000  | Launch idle throttle - throttle to be set before launch sequence is initiated. If set below min_throttle it will force motor stop or at idle throttle (depending if the MOTOR_STOP is enabled). If set above min_throttle it will force throttle to this value (if MOTOR_STOP is enabled it will be handled according to throttle stick position)	|
|  nav_fw_launch_motor_delay    | 500 | Delay between detected launch and launch sequence start and throttling up (ms) |
|  nav_fw_launch_spinup_time    | 100 | Time to bring power from min_throttle to nav_fw_launch_thr - to avoid big stress on ESC and large torque from propeller |
|  nav_fw_launch_timeout  | 5000 | Maximum time for launch sequence to be executed. After this time LAUNCH mode will be turned off and regular flight mode will take over (ms) |
|  nav_fw_launch_climb_angle  | 18 | Climb angle for launch sequence (degrees), is also restrained by global max_angle_inclination_pit |
|  nav_fw_land_dive_angle  | 2 | Dive angle that airplane will use during final landing phase. During dive phase, motor is stopped or IDLE and roll controll is locked to 0 degrees |
|  serialrx_provider  | SPEK1024 | When feature SERIALRX is enabled, this allows connection to several receivers which output data via digital interface resembling serial. See RX section. |
|  serialrx_halfduplex  | OFF | Allow serial receiver to operate on UART TX pin. With some receivers will allow control and telemetry over a single wire |
|  sbus_inversion     | OFF | Standard SBUS (Futaba, FrSKY) uses an inverted signal. Some OpenLRS receivers produce a non-inverted SBUS signal. This setting is to support this type of receivers (including modified FrSKY). This only works on supported hardware (mainly F3 based flight controllers). |
|  spektrum_sat_bind  | 0 | 0 = disabled. Used to bind the spektrum satellite to RX |
|  telemetry_switch  | OFF | Which aux channel to use to change serial output & baud rate (MSP / Telemetry). It disables automatic switching to Telemetry when armed. |
|  telemetry_inversion  | ON | Determines if the telemetry signal is inverted (Futaba, FrSKY). Only suitable on F3 uarts and Softserial on all targets |
|  frsky_default_lattitude  | 0.000 | OpenTX needs a valid set of coordinates to show compass value. A fake value defined in this setting is sent while no fix is acquired. |
|  frsky_default_longitude  | 0.000 | OpenTX needs a valid set of coordinates to show compass value. A fake value defined in this setting is sent while no fix is acquired. |
|  frsky_coordinates_format  | 0 | FRSKY_FORMAT_DMS (default), FRSKY_FORMAT_NMEA |
|  frsky_unit  | METRIC | METRIC , IMPERIAL |
|  frsky_vfas_precision  | 0 | Set to 1 to send raw VBat value in 0.1V resolution for receivers that can handle it, or 0 (default) to use the standard method |
|  frsky_vfas_cell_voltage  | OFF |  |
|  hott_alarm_sound_interval  | 5 | Battery alarm delay in seconds for Hott telemetry |
|  smartport_uart_unidir  | OFF | Turn UART into UNIDIR for smartport telemetry for usage on F1 and F4 target. See Telemertry.md for details |
|  smartport_fuel_percent  | OFF | Set to ON for `Fuel` telemetry to return remaining battery percentage (calculated using `battery_capacity` variable), mAh drawn otherwise. |
|  ibus_telemetry_type  | 0 | Type compatibility ibus telemetry for transmitters. See Telemetry.md label IBUS for details. |
|  ltm_update_rate  | NORMAL | Defines the LTM update rate (use of bandwidth [NORMAL/MEDIUM/SLOW]). See Telemetry.md, LTM section for details. |
|  battery_capacity  | 0 | Battery capacity in mAH. This value is used in conjunction with the current meter to determine remaining battery capacity. |
|  vbat_scale  | 110 | Result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 4095 = 12bit adc, 110 = 11:1 voltage divider (10k:1k) x 10 for 0.1V. Adjust this slightly if reported pack voltage is different from multimeter reading. You can get current voltage by typing "status" in cli. |
|  vbat_max_cell_voltage  | 43 | Maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V) |
|  vbat_min_cell_voltage  | 33 | Minimum voltage per cell, this triggers battery out alarms, in 0.1V units, default is 33 (3.3V) |
|  vbat_warning_cell_voltage  | 35 | Warning voltage per cell, this triggers battery-warning alarms, in 0.1V units, default is 35 (3.5V) |
|  current_meter_scale  | 400 | This sets the output voltage to current scaling for the current sensor in 0.1 mV/A steps. 400 is 40mV/A such as the ACS756 sensor outputs. 183 is the setting for the uberdistro with a 0.25mOhm shunt. |
|  current_meter_offset  | 0 | This sets the output offset voltage of the current sensor in millivolts. |
|  multiwii_current_meter_output  | OFF | Default current output via MSP is in 0.01A steps. Setting this to 1 causes output in default multiwii scaling (1mA steps) |
|  current_meter_type  | ADC | ADC , VIRTUAL, NONE. The virtual current sensor, once calibrated, estimates the current value from throttle position. |
|  align_gyro  | DEFAULT | When running on non-default hardware or adding support for new sensors/sensor boards, these values are used for sensor orientation. When carefully understood, these values can also be used to rotate (in 90deg steps) or flip the board. Possible values are: DEFAULT, CW0_DEG, CW90_DEG, CW180_DEG, CW270_DEG, CW0_DEG_FLIP, CW90_DEG_FLIP, CW180_DEG_FLIP, CW270_DEG_FLIP. |
|  align_acc  | DEFAULT | When running on non-default hardware or adding support for new sensors/sensor boards, these values are used for sensor orientation. When carefully understood, these values can also be used to rotate (in 90deg steps) or flip the board. Possible values are: DEFAULT, CW0_DEG, CW90_DEG, CW180_DEG, CW270_DEG, CW0_DEG_FLIP, CW90_DEG_FLIP, CW180_DEG_FLIP, CW270_DEG_FLIP. |
|  align_mag  | DEFAULT | When running on non-default hardware or adding support for new sensors/sensor boards, these values are used for sensor orientation. When carefully understood, these values can also be used to rotate (in 90deg steps) or flip the board. Possible values are: DEFAULT, CW0_DEG, CW90_DEG, CW180_DEG, CW270_DEG, CW0_DEG_FLIP, CW90_DEG_FLIP, CW180_DEG_FLIP, CW270_DEG_FLIP. |
|  align_board_roll  | 0 | Arbitrary board rotation in degrees, to allow mounting it sideways / upside down / rotated etc |
|  align_board_pitch  | 0 | Arbitrary board rotation in degrees, to allow mounting it sideways / upside down / rotated etc |
|  align_board_yaw  | 0 | Arbitrary board rotation in degrees, to allow mounting it sideways / upside down / rotated etc |
|  gyro_hardware_lpf  | 42HZ | Hardware lowpass filter for gyro. Allowed values depend on the driver - For example MPU6050 allows 10HZ,20HZ,42HZ,98HZ,188HZ,256Hz (8khz mode). If you have to set gyro lpf below 42Hz generally means the frame is vibrating too much, and that should be fixed first. |
|  moron_threshold  | 32 | When powering up, gyro bias is calculated. If the model is shaking/moving during this initial calibration, offsets are calculated incorrectly, and could lead to poor flying performance. This threshold means how much average gyro reading could differ before re-calibration is triggered. |
|  imu_dcm_kp  | 2500 | Inertial Measurement Unit KP Gain for accelerometer measurements |
|  imu_dcm_ki  | 50 | Inertial Measurement Unit KI Gain for accelerometer measurements |
|  imu_dcm_kp_mag  | 10000 | Inertial Measurement Unit KP Gain for compass measurements |
|  imu_dcm_ki_mag  | 0 | Inertial Measurement Unit KI Gain for compass measurements |
|  pos_hold_deadband  | 20 | Stick deadband in [r/c points], applied after r/c deadband and expo |
|  alt_hold_deadband  | 50 | Defines the deadband of throttle during alt_hold [r/c points] |
|  yaw_motor_direction  | 1 | Use if you need to inverse yaw motor direction. |
|  yaw_jump_prevention_limit  | 200 | Prevent yaw jumps during yaw stops and rapid YAW input. To disable set to 500. Adjust this if your aircraft 'skids out'. Higher values increases YAW authority but can cause roll/pitch instability in case of underpowered UAVs. Lower values makes yaw adjustments more gentle but can cause UAV unable to keep heading |
|  tri_unarmed_servo  | ON | On tricopter mix only, if this is set to ON, servo will always be correcting regardless of armed state. to disable this, set it to OFF. |
|  servo_lpf_hz  | 20 | Selects the servo PWM output cutoff frequency. Value is in [Hz] |
|  servo_center_pulse  | 1500 | Servo midpoint |
|  servo_pwm_rate  | 50 | Output frequency (in Hz) servo pins. When using tricopters or gimbal with digital servo, this rate can be increased. Max of 498Hz (for 500Hz pwm period), and min of 50Hz. Most digital servos will support for example 330Hz. |
|  failsafe_delay  | 5 | Time in deciseconds to wait before activating failsafe when signal is lost. See [Failsafe documentation](Failsafe.md#failsafe_delay). |
|  failsafe_recovery_delay  | 5 | Time in deciseconds to wait before aborting failsafe when signal is recovered. See [Failsafe documentation](Failsafe.md#failsafe_recovery_delay). |
|  failsafe_off_delay  | 200 | Time in deciseconds to wait before turning off motors when failsafe is activated. 0 = No timeout. See [Failsafe documentation](Failsafe.md#failsafe_off_delay). |
|  failsafe_throttle  | 1000 | Throttle level used for landing when failsafe is enabled. See [Failsafe documentation](Failsafe.md#failsafe_throttle). |
|  failsafe_throttle_low_delay  | 100 | If failsafe activated when throttle is low for this much time - bypass failsafe and disarm, in 10th of seconds. 0 = No timeout |
|  failsafe_procedure  | SET-THR | What failsafe procedure to initiate in Stage 2. See [Failsafe documentation](Failsafe.md#failsafe_throttle). |
|  failsafe_stick_threshold  | 0 | Threshold for stick motion to consider failsafe condition resolved. If non-zero failsafe won't clear even if RC link is restored - you have to move sticks to exit failsafe. |
|  failsafe_fw_roll_angle  | -200 | Amount of banking when `SET-THR` failsafe is active on a fixed-wing machine. In 1/10 deg (deci-degrees). Negative values = left roll |
|  failsafe_fw_pitch_angle  | 100 | Amount of dive/climb when `SET-THR` failsafe is active on a fixed-wing machine. In 1/10 deg (deci-degrees). Negative values = climb |
|  failsafe_fw_yaw_rate  | -45 | Requested yaw rate to execute when `SET-THR` failsafe is active on a fixed-wing machine. In deg/s. Negative values = left turn |
|  rx_min_usec  | 885 | Defines the shortest pulse width value used when ensuring the channel value is valid. If the receiver gives a pulse value lower than this value then the channel will be marked as bad and will default to the value of mid_rc. |
|  rx_max_usec  | 2115 | Defines the longest pulse width value used when ensuring the channel value is valid. If the receiver gives a pulse value higher than this value then the channel will be marked as bad and will default to the value of mid_rc. |
|  rx_nosignal_throttle  | HOLD | Defines behavior of throttle channel after signal loss is detected and until `failsafe_procedure` kicks in. Possible values - `HOLD` and `DROP`. |
|  acc_hardware  | AUTO | Selection of acc hardware. See Wiki Sensor auto detect and hardware failure detection for more info |
|  baro_use_median_filter  | ON | 3-point median filtering for barometer readouts. No reason to change this setting |
|  baro_hardware  | AUTO | Selection of baro hardware. See Wiki Sensor auto detect and hardware failure detection for more info |
|  mag_hardware  | AUTO | Selection of mag hardware. See Wiki Sensor auto detect and hardware failure detection for more info |
|  blackbox_rate_num  | 1 | Blackbox logging rate numerator. Use num/denom settings to decide if a frame should be logged, allowing control of the portion of logged loop iterations |
|  blackbox_rate_denom  | 1 | Blackbox logging rate denominator. See blackbox_rate_num. |
|  blackbox_device  | SPIFLASH | Selection of where to write blackbox data |
|  sdcard_detect_inverted  | `TARGET dependent` | This setting drives the way SD card is detected in card slot. On some targets (AnyFC F7 clone) different card slot was used and depending of hardware revision ON or OFF setting might be required. If card is not detected, change this value. |
|  ledstrip_visual_beeper  | OFF |  |
|  osd_video_system     | 0     |  |
|  osd_row_shiftdown    | 0     |  |
|  osd_units            | 0     |  |
|  osd_rssi_alarm       | 20    |  |
|  osd_cap_alarm        | 2200  |  |
|  osd_time_alarm       | 10    |  |
|  osd_alt_alarm        | 100   |  |
|  osd_main_voltage_pos | 0     |  |
|  osd_rssi_pos         | 0     |  |
|  osd_flytimer_pos     | 0     |  |
|  osd_ontime_pos       | 0     |  |
|  osd_flymode_pos      | 0     |  |
|  osd_throttle_pos     | 0     |  |
|  osd_vtx_channel_pos  | 0     |  |
|  osd_crosshairs       | 0     |  |
|  osd_artificial_horizon  | 0  |  |
|  osd_current_draw_pos | 0     |  |
|  osd_mah_drawn_pos    | 0     |  |
|  osd_craft_name_pos   | 0     |  |
|  osd_gps_speed_pos    | 0     |  |
|  osd_gps_sats_pos     | 0     |  |
|  osd_altitude_pos     | 0     |  |
|  osd_pid_roll_pos     | 0     |  |
|  osd_pid_pitch_pos    | 0     |  |
|  osd_pid_yaw_pos      | 0     |  |
|  osd_power_pos        | 0     |  |
|  magzero_x  | 0 | Magnetometer calibration X offset. If its 0 none offset has been applied and calibration is failed. |
|  magzero_y  | 0 | Magnetometer calibration Y offset. If its 0 none offset has been applied and calibration is failed. |
|  magzero_z  | 0 | Magnetometer calibration Z offset. If its 0 none offset has been applied and calibration is failed. |
|  acczero_x  | 0 | Calculated value after '6 position avanced calibration'. See Wiki page. |
|  acczero_y  | 0 | Calculated value after '6 position avanced calibration'. See Wiki page. |
|  acczero_z  | 0 | Calculated value after '6 position avanced calibration'. See Wiki page. |
|  accgain_x  | 4096 | Calculated value after '6 position avanced calibration'. Uncalibrated value is 4096. See Wiki page. |
|  accgain_y  | 4096 | Calculated value after '6 position avanced calibration'. Uncalibrated value is 4096. See Wiki page. |
|  accgain_z  | 4096 | Calculated value after '6 position avanced calibration'. Uncalibrated value is 4096. See Wiki page. |
|  nav_mc_pos_z_p  | 50 | P gain of altitude PID controller (Multirotor) |
|  nav_fw_pos_z_p  | 50 | P gain of altitude PID controller (Fixdwing) |
|  nav_mc_pos_z_i  | 0 | I gain of altitude PID controller (Multirotor) |
|  nav_fw_pos_z_i  | 0 | I gain of altitude PID controller (Fixdwing) |
|  nav_mc_pos_z_d  | 0 | D gain of altitude PID controller (Multirotor) |
|  nav_fw_pos_z_d  | 0 | D gain of altitude PID controller (Fixdwing) |
|  nav_mc_vel_z_p  | 100 | P gain of velocity PID controller |
|  nav_mc_vel_z_i  | 50 | I gain of velocity PID controller |
|  nav_mc_vel_z_d  | 10 | D gain of velocity PID controller |
|  nav_mc_pos_xy_p  | 65 | Controls how fast the drone will fly towards the target position. This is a multiplier to convert displacement to target velocity |
|  nav_mc_pos_xy_i  | 120 | Controls deceleration time. Measured in 1/100 sec. Expected hold position is placed at a distance calculated as decelerationTime * currentVelocity |
|  nav_mc_pos_xy_d  | 10 |  |
|  nav_mc_vel_xy_p  | 180 | P gain of Position-Rate (Velocity to Acceleration) PID controller. Higher P means stronger response when position error occurs. Too much P might cause "nervous" behavior and oscillations |
|  nav_mc_vel_xy_i  | 15 | I gain of Position-Rate (Velocity to Acceleration) PID controller. Used for drift compensation (caused by wind for example). Higher I means stronger response to drift. Too much I gain might cause target overshot |
|  nav_mc_vel_xy_d  | 100 | D gain of Position-Rate (Velocity to Acceleration) PID controller. It can damp P and I. Increasing D might help when drone overshoots target. |
|  nav_fw_pos_xy_p  | 10 | P gain of 2D trajectory PID controller. Play with this to get a straigh line between waypoints or a straight RTH |
|  nav_fw_pos_xy_i  | 5 | I gain of 2D trajectory PID controller. Too high and there will be overshoot in trajectory. Better start tunning with zero |
|  nav_fw_pos_xy_d  | 8 | D gain of 2D trajectory PID controller. Too high and there will be overshoot in trajectory. Better start tunning with zero |
|  deadband  | 5 | These are values (in us) by how much RC input can be different before it's considered valid. For transmitters with jitter on outputs, this value can be increased. Defaults are zero, but can be increased up to 10 or so if rc inputs twitch while idle. |
|  yaw_deadband  | 5 | These are values (in us) by how much RC input can be different before it's considered valid. For transmitters with jitter on outputs, this value can be increased. Defaults are zero, but can be increased up to 10 or so if rc inputs twitch while idle. |
|  throttle_tilt_comp_str  | 0 | Can be used in ANGLE and HORIZON mode and will automatically boost throttle when banking. Setting is in percentage, 0=disabled. |
|  flaperon_throw_offset  | 250 | Defines throw range in us for both ailerons that will be passed to servo mixer via input source 14 (`FEATURE FLAPS`) when FLAPERON mode is activated. |
|  gimbal_mode  | NORMAL | When feature SERVO_TILT is enabled, this can be either NORMAL or MIXTILT |
|  fw_iterm_throw_limit  | 165 | Limits max/min I-term value in stabilization PID controller in case of Fixed Wing. It solves the problem of servo saturation before take-off/throwing the airplane into the air. By default, error accumulated in I-term can not exceed 1/3 of servo throw (around 165us). Set 0 to disable completely. |
|  fw_reference_airspeed  | 1000 | Reference airspeed. Set this to airspeed at which PIDs were tuned. Usually should be set to cruise airspeed. Also used for coordinated turn calculation if airspeed sensor is not present. |
|  fw_turn_assist_yaw_gain  | 1 | Gain required to keep the yaw rate consistent with the turn rate for a coordinated turn (in TURN_ASSIST mode). Value significantly different from 1.0 indicates a problem with the airspeed calibration (if present) or value of `fw_reference_airspeed` parameter |
|  mode_range_logic_operator  | OR | Control how Mode selection works in flight modes. If you example have Angle mode configured on two different Aux channels, this controls if you need both activated ( AND ) or if you only need one activated ( OR ) to active angle mode. |
|  default_rate_profile  | 0 | Default = profile number |
|  mag_declination  | 0 | Current location magnetic declination in format. For example, -6deg 37min = -637 for Japan. Leading zero in ddd not required. Get your local magnetic declination here: http://magnetic-declination.com/ . Not in use if inav_auto_mag_decl  is turned on and you aquirre valid GPS fix. |
|  heading_hold_rate_limit  | 90 | This setting limits yaw rotation rate that HEADING_HOLD controller can request from PID inner loop controller. It is independent from manual yaw rate and used only when HEADING_HOLD flight mode is enabled by pilot, RTH or WAYPOINT modes. |
| `mag_calibration_time` | 30 | Adjust how long time the Calibration of mag will last. |
| `mc_p_pitch` | 40 | Multicopter rate stabilisation P-gain for PITCH               |
| `mc_i_pitch` | 30 | Multicopter rate stabilisation I-gain for PITCH               |
| `mc_d_pitch` | 23 | Multicopter rate stabilisation D-gain for PITCH               |
| `mc_p_roll`  | 40 | Multicopter rate stabilisation P-gain for ROLL                |
| `mc_i_roll`  | 30 | Multicopter rate stabilisation I-gain for ROLL                |
| `mc_d_roll`  | 23 | Multicopter rate stabilisation D-gain for ROLL                |
| `mc_p_yaw`   | 85 | Multicopter rate stabilisation P-gain for YAW                 |
| `mc_i_yaw`   | 45 | Multicopter rate stabilisation I-gain for YAW                 |
| `mc_d_yaw`   | 0  | Multicopter rate stabilisation D-gain for YAW                 |
| `mc_p_level` | 20 | Multicopter attitude stabilisation P-gain                     |
| `mc_i_level` | 15 | Multicopter attitude stabilisation low-pass filter cutoff     |
| `mc_d_level` | 75 | Multicopter attitude stabilisation HORIZON transition point   |
| `fw_p_pitch` | 20 | Fixed-wing rate stabilisation P-gain for PITCH                |
| `fw_i_pitch` | 30 | Fixed-wing rate stabilisation I-gain for PITCH                |
| `fw_ff_pitch`| 10 | Fixed-wing rate stabilisation FF-gain for PITCH               |
| `fw_p_roll`  | 25 | Fixed-wing rate stabilisation P-gain for ROLL                 |
| `fw_i_roll`  | 30 | Fixed-wing rate stabilisation I-gain for ROLL                 |
| `fw_ff_roll` | 10 | Fixed-wing rate stabilisation FF-gain for ROLL                |
| `fw_p_yaw`   | 50 | Fixed-wing rate stabilisation P-gain for YAW                  |
| `fw_i_yaw`   | 45 | Fixed-wing rate stabilisation I-gain for YAW                  |
| `fw_ff_yaw`  | 0  | Fixed-wing rate stabilisation FF-gain for YAW                 |
| `fw_p_level` | 20 | Fixed-wing attitude stabilisation P-gain                      |
| `fw_i_level` | 15 | Fixed-wing attitude stabilisation low-pass filter cutoff      |
| `fw_d_level` | 75 | Fixed-wing attitude stabilisation HORIZON transition point    |
|  max_angle_inclination_rll  | 300 | Maximum inclination in level (angle) mode (ROLL axis). 100=10° |
|  max_angle_inclination_pit  | 300 | Maximum inclination in level (angle) mode (PITCH axis). 100=10° |
|  gyro_lpf_hz  | 60 | Software-based filter to remove mechanical vibrations from the gyro signal. Value is cutoff frequency (Hz). For larger frames with bigger props set to lower value. |
|  acc_lpf_hz  | 15 | Software-based filter to remove mechanical vibrations from the accelerometer measurements. Value is cutoff frequency (Hz). For larger frames with bigger props set to lower value. |
|  dterm_lpf_hz  | 40 |  |
|  yaw_lpf_hz  | 30 |  |
|  pidsum_limit  | 500 | A limitation to overall amount of correction Flight PID can request on each axis (Roll/Pitch/Yaw). If when doing a hard maneuver on one axis machine looses orientation on other axis - reducing this parameter may help |
|  yaw_p_limit  | 300 |  |
|  iterm_ignore_threshold  | 200 | Used to prevent Iterm accumulation on ROLL/PITCH axis during stick movements. Iterm is allowed to change fully when sticks are centered. Iterm will not change when requested rotation speed is above iterm_ignore_threshold. Iterm acumulation is scaled lineary between 0 and iterm_ignore_threshold |
|  yaw_iterm_ignore_threshold  | 50 | Used to prevent Iterm accumulation on YAW axis during stick movements. Iterm is allowed to change fully when sticks are centered. Iterm will not change when requested rotation speed is above yaw_iterm_ignore_threshold. Iterm acumulation is scaled lineary between 0 and yaw_iterm_ignore_threshold |
|  rate_accel_limit_roll_pitch  | 0 | Limits acceleration of ROLL/PITCH rotation speed that can be requested by stick input. In degrees-per-second-squared. Small and powerful UAV flies great with high acceleration limit ( > 5000 dps^2 and even > 10000 dps^2). Big and heavy multirotors will benefit from low acceleration limit (~ 360 dps^2). When set correctly, it greatly improves stopping performance. Value of 0 disables limiting.  |
|  rate_accel_limit_yaw  | 10000 | Limits acceleration of YAW rotation speed that can be requested by stick input. In degrees-per-second-squared. Small and powerful UAV flies great with high acceleration limit ( > 10000 dps^2). Big and heavy multirotors will benefit from low acceleration limit (~ 180 dps^2). When set correctly, it greatly improves stopping performance and general stability durig yaw turns. Value of 0 disables limiting. |
|  rc_expo  | 70 | Exposition value for all RC directions |
|  rc_yaw_expo  | 20 | Yaw exposition value |
|  thr_mid  | 50 | Throttle value when the stick is set to mid-position. Used in the throttle curve calculation. |
|  thr_expo  | 0 | Throttle exposition value |
|  roll_rate  | 20 | Defines rotation rate on ROLL axis that UAV will try to archive on max. stick deflection. Rates are defined in tenths of degrees per second [dps/10]. That means, rate 20 represents 200dps rotation speed. Default 20 (200dps) is more less equivalent of old Cleanflight/Baseflight rate 0. Max. 180 (1800dps) is what gyro can measure. |
|  pitch_rate  | 20 | Defines rotation rate on PITCH axis that UAV will try to archive on max. stick deflection. Rates are defined in tenths of degrees per second [dps/10]. That means, rate 20 represents 200dps rotation speed. Default 20 (200dps) is more less equivalent of old Cleanflight/Baseflight rate 0. Max. 180 (1800dps) is what gyro can measure. |
|  yaw_rate  | 20 | Defines rotation rate on YAW axis that UAV will try to archive on max. stick deflection. Rates are defined in tenths of degrees per second [dps/10]. That means, rate 20 represents 200dps rotation speed. Default 20 (200dps) is more less equivalent of old Cleanflight/Baseflight rate 0. Max. 180 (1800dps) is what gyro can measure. |
|  tpa_rate  | 0 | Throttle PID attenuation reduces influence of P on ROLL and PITCH as throttle increases. For every 1% throttle after the TPA breakpoint, P is reduced by the TPA rate. |
|  tpa_breakpoint  | 1500 | See tpa_rate. |
|  fw_autotune_overshoot_time  | 100 | Time [ms] to detect sustained overshoot |
|  fw_autotune_undershoot_time | 200 | Time [ms] to detect sustained undershoot |
|  fw_autotune_threshold       | 50  | Threshold [%] of max rate to consider overshoot/undershoot detection |
|  fw_autotune_ff_to_p_gain    | 10  | FF to P gain (strength relationship) [%] |
|  fw_autotune_ff_to_i_tc      | 600 | FF to I time (defines time for I to reach the same level of response as FF) [ms]  |
|  stats                       | OFF | General switch of the statistics recording feature (a.k.a. odometer) |
|  stats_total_time            |  0  | Total flight time [in seconds]. The value is updated on every disarm when "stats" are enabled. |
|  stats_total_dist            |  0  | Total flight distance [in meters]. The value is updated on every disarm when "stats" are enabled. |
|  vbat_adc_channel            |  -  | ADC channel to use for battery voltage sensor. Defaults to board VBAT input (if available). 0 = disabled |
|  rssi_adc_channel            |  -  | ADC channel to use for analog RSSI input. Defaults to board RSSI input (if available). 0 = disabled |
|  current_adc_channel         |  -  | ADC channel to use for analog current sensor input. Defaults to board CURRENT sensor input (if available). 0 = disabled |
|  airspeed_adc_channel        |  -  | ADC channel to use for analog pitot tube (airspeed) sensor. If board doesn't have a dedicated connector for analog airspeed sensor will default to 0 |

This Markdown table is made by MarkdwonTableMaker addon for google spreadsheet.
Original Spreadsheet used to make this table can be found here https://docs.google.com/spreadsheets/d/1ubjYdMGmZ2aAMUNYkdfe3hhIF7wRfIjcuPOi_ysmp00/edit?usp=sharing
