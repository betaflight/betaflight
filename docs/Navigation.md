# Navigation

Navigation system in Cleanflight is responsible for assisting the pilot allowing altitude and position hold, return-to-home and waypoint flight.

## Altitude hold

Altitude hold requires a valid source of altitude - barometer or sonar. The best source is chosen automatically

PIDs to tune: ALT & VEL
PID meaning:
    ALT - translates altitude error to desired climb rate, only P-term is used, I and D has no meaning
    VEL - translated climb rate error to throttle adjustment, full PID is used

## Throttle tilt compensation

TODO

## Position hold

Position hold required GPS and compass sensors. Flight modes that require a compass (POSHOLD, RTH) are locked until compass is properly calibrated

PIDs to tune: POS, POSR, NAVR
PID meaning:
    POS - translated position error to desired velocity, uses P & I terms
    POSR - translates velocity error to pitch/roll adjustment when in POSHOLD mode, is a full PID-regulator
    NAVR - translates velocity error to pitch/roll adjustment when in RTH/WP mode, is a full PID-regulator

## Wind compensation