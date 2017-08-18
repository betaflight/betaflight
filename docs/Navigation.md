# Navigation

Navigation system in INAV is responsible for assisting the pilot allowing altitude and position hold, return-to-home and waypoint flight.

## NAV ALTHOLD mode - altitude hold

Altitude hold requires a valid source of altitude - barometer, GPS or rangefinder. The best source is chosen automatically. GPS is available as an altitude source for airplanes only.
In this mode THROTTLE stick controls climb rate (vertical velocity). When pilot moves stick up - quad goes up, pilot moves stick down - quad descends, you keep stick at neutral position - quad hovers.

### CLI parameters affecting ALTHOLD mode:
* *nav_use_midthr_for_althold* - when set to "0", firmware will remember where your throttle stick was when ALTHOLD was activated - this will be considered neutral position. When set to "1" - 50% throttle will be considered neutral position.

### Related PIDs
PIDs affecting altitude hold: ALT & VEL
PID meaning:
* ALT - translates altitude error to desired climb rate and acceleration. Tune P for altitude-to-velocity regulator and I for velocity-to-acceleration regulator
* VEL - translated Z-acceleration error to throttle adjustment

## Throttle tilt compensation

Throttle tilt compensation attempts to maintain constant vertical thrust when copter is tilted giving additional throttle if tilt angle (pitch/roll) is not zero. Controlled by *throttle_tilt_comp_str* CLI variable.

## NAV POSHOLD mode - position hold

Position hold requires GPS, accelerometer and compass sensors. Flight modes that require a compass (POSHOLD, RTH) are locked until compass is properly calibrated.
When activated, this mode will attempt to keep copter where it is (based on GPS coordinates). Can be activated together with ALTHOLD to get a full 3D position hold. Heading hold in this mode is assumed and activated automatically.

### CLI parameters affecting ALTHOLD mode:
* *nav_user_control_mode* - can be set to "0" (GPS_ATTI) or "1" (GPS_CRUISE), controls how firmware will respond to roll/pitch stick movement. When in GPS_ATTI mode, right stick controls attitude, when it is released, new position is recorded and held. When in GPS_CRUISE mode right stick controls velocity and firmware calculates required attitude on its own.

### Related PIDs
PIDs affecting position hold: POS, POSR
PID meaning:
* POS - translated position error to desired velocity, uses P term only
* POSR - translates velocity error to desired acceleration

## NAV RTH - return to home mode

Home for RTH is position, where copter was armed. RTH requires accelerometer, compass and GPS sensors.

If barometer is NOT present, RTH will fly directly to home, altitude control here is up to pilot.

If barometer is present, RTH will maintain altitude during the return and when home is reached copter will attempt automated landing.
When deciding what altitude to maintain, RTH has 4 different modes of operation (controlled by *nav_rth_alt_mode* and *nav_rth_altitude* cli variables):
* 0 (NAV_RTH_NO_ALT) - keep current altitude during whole RTH sequence (*nav_rth_altitude* is ignored)
* 1 (NAV_RTH_EXTRA_ALT) - climb to current altitude plus extra margin prior to heading home (*nav_rth_altitude* defines the extra altitude (cm))
* 2 (NAV_RTH_CONST_ALT) - climb/descend to predefined altitude before heading home (*nav_rth_altitude* defined altitude above launch point (cm))
* 3 (NAV_RTH_MAX_ALT) - track maximum altitude of the whole flight, climb to that altitude prior to the return (*nav_rth_altitude* is ignored)
* 4 (NAV_RTH_AT_LEAST_ALT) - same as 2 (NAV_RTH_CONST_ALT), but only climb, do not descend
