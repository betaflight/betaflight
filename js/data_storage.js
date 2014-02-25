var firmware_version_accepted = 2.3;

var CONFIG = {
    version:       0,
    multiType:     0,
    msp_version:   0,
    capability:    0,
    cycleTime:     0,
    i2cError:      0,
    activeSensors: 0,
    mode:          0,
    profile:       0,
    
    uid:           [0, 0, 0],
    accelerometerTrims: [0, 0]
};

var PIDs = new Array(10);
for (var i = 0; i < 10; i++) {
    PIDs[i] = new Array(3);
}

var RC = {
    roll:     0,
    pitch:    0,
    yaw:      0,
    throttle: 0,
    AUX1:     0,
    AUX2:     0,
    AUX3:     0,
    AUX4:     0
};

var RC_tuning = {
    RC_RATE:         0,
    RC_EXPO:         0,
    roll_pitch_rate: 0,
    yaw_rate:        0,
    dynamic_THR_PID: 0,
    throttle_MID:    0,
    throttle_EXPO:   0,
};

var AUX_CONFIG = new Array();
var AUX_CONFIG_values = new Array();

var SERVO_CONFIG = new Array();

var SENSOR_DATA = {
    gyroscope:     [0, 0, 0],
    accelerometer: [0, 0, 0],
    magnetometer:  [0, 0, 0],
    altitude:      0,
    kinematicsX:   0.0,
    kinematicsY:   0.0,
    kinematicsZ:   0.0,
    debug:        [0, 0, 0, 0]
};

var MOTOR_DATA = new Array(8);
var SERVO_DATA = new Array(8);

var GPS_DATA = {
    fix:             0,
    numSat:          0,
    lat:             0,
    lon:             0,
    alt:             0,
    speed:           0,
    ground_course:   0,
    distanceToHome:  0,
    ditectionToHome: 0,
    update:          0,
    
    // baseflight specific gps stuff
    chn:     new Array(),
    svid:    new Array(),
    quality: new Array(),
    cno:     new Array()
};

var ANALOG = {
    voltage:    0,
    pMeterSum:  0,
    rssi:       0,
    amperage:   0
};

var MISC = {
    PowerTrigger1:      0, // intPowerTrigger1 (aka useless trash)
    minthrottle:        0,
    maxthrottle:        0,
    mincommand:         0,
    failsafe_throttle:  0,
    plog0:              0, // plog useless shit
    plog1:              0, // plog useless shit
    mag_declination:    0, // not checked
    vbatscale:          0,
    vbatmincellvoltage: 0,
    vbatmaxcellvoltage: 0,
    empty:              0 // unknown
};