#include "board.h"
#include "mw.h"

static uint8_t numberMotor = 4;
uint8_t useServo = 0;
int16_t motor[8];
int16_t servo[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

void mixerInit(void)
{
    // enable servos for mixes that require them. note, this shifts motor counts.
    if (cfg.mixerConfiguration == MULTITYPE_BI || cfg.mixerConfiguration == MULTITYPE_TRI || cfg.mixerConfiguration == MULTITYPE_GIMBAL || cfg.mixerConfiguration == MULTITYPE_FLYING_WING)
        useServo = 1;
    // if we want camstab/trig, that also enabled servos. this is kinda lame. maybe rework feature bits later.
    if (feature(FEATURE_SERVO_TILT) || feature(FEATURE_CAMTRIG))
        useServo = 1;

    switch (cfg.mixerConfiguration) {
        case MULTITYPE_GIMBAL:
            numberMotor = 0;
            break;
        case MULTITYPE_FLYING_WING:
            numberMotor = 1;
            break;
        case MULTITYPE_BI:
            numberMotor = 2;
            break;
        case MULTITYPE_TRI:
            numberMotor = 3;
            break;

        case MULTITYPE_QUADP:
        case MULTITYPE_QUADX:
        case MULTITYPE_Y4:
        case MULTITYPE_VTAIL4:
            numberMotor = 4;
            break;

        case MULTITYPE_Y6:
        case MULTITYPE_HEX6:
        case MULTITYPE_HEX6X:
            numberMotor = 6;
            break;

        case MULTITYPE_OCTOX8:
        case MULTITYPE_OCTOFLATP:
        case MULTITYPE_OCTOFLATX:
            numberMotor = 8;
            break;
    }
}

void writeServos(void)
{
    if (!useServo)
        return;

    if (cfg.mixerConfiguration == MULTITYPE_TRI || cfg.mixerConfiguration == MULTITYPE_BI) {
        /* One servo on Motor #4 */
        pwmWrite(0, servo[4]);
        if (cfg.mixerConfiguration == MULTITYPE_BI)
            pwmWrite(1, servo[5]);
    } else {
        /* Two servos for camstab or FLYING_WING */
        pwmWrite(0, servo[0]);
        pwmWrite(1, servo[1]);
    }
}

extern uint8_t cliMode;

void writeMotors(void)
{
    uint8_t i;
    uint8_t offset = 0;

    // when servos are enabled, motor outputs 1..2 are for servos only
    if (useServo)
        offset = 2;

    for (i = 0; i < numberMotor; i++)
        pwmWrite(i + offset, motor[i]);
}

void writeAllMotors(int16_t mc)
{
    uint8_t i;

    // Sends commands to all motors
    for (i = 0; i < numberMotor; i++)
        motor[i] = mc;
    writeMotors();
}

#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL] * X + axisPID[PITCH] * Y + cfg.yaw_direction * axisPID[YAW] * Z

void mixTable(void)
{
    int16_t maxMotor;
    uint8_t i;
    static uint8_t camCycle = 0;
    static uint8_t camState = 0;
    static uint32_t camTime = 0;

    if (numberMotor > 3) {
        //prevent "yaw jump" during yaw correction
        axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));
    }

    switch (cfg.mixerConfiguration) {

        case MULTITYPE_BI:
            motor[0] = PIDMIX(+1, 0, 0);        //LEFT
            motor[1] = PIDMIX(-1, 0, 0);        //RIGHT        
            servo[4] = constrain(1500 + (cfg.yaw_direction * axisPID[YAW]) + axisPID[PITCH], 1020, 2000);   //LEFT
            servo[5] = constrain(1500 + (cfg.yaw_direction * axisPID[YAW]) - axisPID[PITCH], 1020, 2000);   //RIGHT
            break;

        case MULTITYPE_TRI:
            motor[0] = PIDMIX(0, +4 / 3, 0);    //REAR
            motor[1] = PIDMIX(-1, -2 / 3, 0);   //RIGHT
            motor[2] = PIDMIX(+1, -2 / 3, 0);   //LEFT
            servo[4] = constrain(cfg.tri_yaw_middle + cfg.yaw_direction * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
            break;

        case MULTITYPE_QUADP:
            motor[0] = PIDMIX(0, +1, -1);       //REAR
            motor[1] = PIDMIX(-1, 0, +1);       //RIGHT
            motor[2] = PIDMIX(+1, 0, +1);       //LEFT
            motor[3] = PIDMIX(0, -1, -1);       //FRONT
            break;

        case MULTITYPE_QUADX:
            motor[0] = PIDMIX(-1, +1, -1);      //REAR_R
            motor[1] = PIDMIX(-1, -1, +1);      //FRONT_R
            motor[2] = PIDMIX(+1, +1, +1);      //REAR_L
            motor[3] = PIDMIX(+1, -1, -1);      //FRONT_L
            break;

        case MULTITYPE_Y4:
            motor[0] = PIDMIX(+0, +1, -1);      //REAR_1 CW
            motor[1] = PIDMIX(-1, -1, 0);       //FRONT_R CCW
            motor[2] = PIDMIX(+0, +1, +1);      //REAR_2 CCW
            motor[3] = PIDMIX(+1, -1, 0);       //FRONT_L CW
            break;

        case MULTITYPE_Y6:
            motor[0] = PIDMIX(+0, +4 / 3, +1);  //REAR
            motor[1] = PIDMIX(-1, -2 / 3, -1);  //RIGHT
            motor[2] = PIDMIX(+1, -2 / 3, -1);  //LEFT
            motor[3] = PIDMIX(+0, +4 / 3, -1);  //UNDER_REAR
            motor[4] = PIDMIX(-1, -2 / 3, +1);  //UNDER_RIGHT
            motor[5] = PIDMIX(+1, -2 / 3, +1);  //UNDER_LEFT    
            break;

        case MULTITYPE_HEX6:
            motor[0] = PIDMIX(-1 / 2, +1 / 2, +1);      //REAR_R
            motor[1] = PIDMIX(-1 / 2, -1 / 2, -1);      //FRONT_R
            motor[2] = PIDMIX(+1 / 2, +1 / 2, +1);      //REAR_L
            motor[3] = PIDMIX(+1 / 2, -1 / 2, -1);      //FRONT_L
            motor[4] = PIDMIX(+0, -1, +1);      //FRONT
            motor[5] = PIDMIX(+0, +1, -1);      //REAR
            break;

        case MULTITYPE_HEX6X:
            motor[0] = PIDMIX(-1 / 2, +1 / 2, +1);      //REAR_R
            motor[1] = PIDMIX(-1 / 2, -1 / 2, +1);      //FRONT_R
            motor[2] = PIDMIX(+1 / 2, +1 / 2, -1);      //REAR_L
            motor[3] = PIDMIX(+1 / 2, -1 / 2, -1);      //FRONT_L
            motor[4] = PIDMIX(-1, +0, -1);      //RIGHT
            motor[5] = PIDMIX(+1, +0, +1);      //LEFT
            break;

        case MULTITYPE_OCTOX8:
            motor[0] = PIDMIX(-1, +1, -1);      //REAR_R
            motor[1] = PIDMIX(-1, -1, +1);      //FRONT_R
            motor[2] = PIDMIX(+1, +1, +1);      //REAR_L
            motor[3] = PIDMIX(+1, -1, -1);      //FRONT_L
            motor[4] = PIDMIX(-1, +1, +1);      //UNDER_REAR_R
            motor[5] = PIDMIX(-1, -1, -1);      //UNDER_FRONT_R
            motor[6] = PIDMIX(+1, +1, -1);      //UNDER_REAR_L
            motor[7] = PIDMIX(+1, -1, +1);      //UNDER_FRONT_L
            break;

        case MULTITYPE_OCTOFLATP:
            motor[0] = PIDMIX(+7 / 10, -7 / 10, +1);    //FRONT_L
            motor[1] = PIDMIX(-7 / 10, -7 / 10, +1);    //FRONT_R
            motor[2] = PIDMIX(-7 / 10, +7 / 10, +1);    //REAR_R
            motor[3] = PIDMIX(+7 / 10, +7 / 10, +1);    //REAR_L
            motor[4] = PIDMIX(+0, -1, -1);      //FRONT
            motor[5] = PIDMIX(-1, +0, -1);      //RIGHT
            motor[6] = PIDMIX(+0, +1, -1);      //REAR
            motor[7] = PIDMIX(+1, +0, -1);      //LEFT 
            break;

        case MULTITYPE_OCTOFLATX:
            motor[0] = PIDMIX(+1, -1 / 2, +1);  //MIDFRONT_L
            motor[1] = PIDMIX(-1 / 2, -1, +1);  //FRONT_R
            motor[2] = PIDMIX(-1, +1 / 2, +1);  //MIDREAR_R
            motor[3] = PIDMIX(+1 / 2, +1, +1);  //REAR_L
            motor[4] = PIDMIX(+1 / 2, -1, -1);  //FRONT_L
            motor[5] = PIDMIX(-1, -1 / 2, -1);  //MIDFRONT_R
            motor[6] = PIDMIX(-1 / 2, +1, -1);  //REAR_R
            motor[7] = PIDMIX(+1, +1 / 2, -1);  //MIDREAR_L 
            break;

        case MULTITYPE_VTAIL4:
            motor[0] = PIDMIX(+0, +1, -1 / 2);      //REAR_R 
            motor[1] = PIDMIX(-1, -1, +2 / 10); //FRONT_R 
            motor[2] = PIDMIX(+0, +1, +1 / 2);      //REAR_L 
            motor[3] = PIDMIX(+1, -1, -2 / 10); //FRONT_L
            break;

        case MULTITYPE_GIMBAL:
            servo[0] = constrain(TILT_PITCH_MIDDLE + cfg.tilt_pitch_prop * angle[PITCH] / 16 + rcCommand[PITCH], TILT_PITCH_MIN, TILT_PITCH_MAX);
            servo[1] = constrain(TILT_ROLL_MIDDLE + cfg.tilt_roll_prop * angle[ROLL] / 16 + rcCommand[ROLL], TILT_ROLL_MIN, TILT_ROLL_MAX);
            break;

        case MULTITYPE_FLYING_WING:
            motor[0] = rcCommand[THROTTLE];
            if (passThruMode) { // do not use sensors for correction, simple 2 channel mixing
                servo[0]  = PITCH_DIRECTION_L * (rcData[PITCH] - cfg.midrc) + ROLL_DIRECTION_L * (rcData[ROLL] - cfg.midrc);
                servo[1]  = PITCH_DIRECTION_R * (rcData[PITCH] - cfg.midrc) + ROLL_DIRECTION_R * (rcData[ROLL] - cfg.midrc);
            } else {                    // use sensors to correct (gyro only or gyro+acc according to aux1/aux2 configuration
                servo[0]  = PITCH_DIRECTION_L * axisPID[PITCH] + ROLL_DIRECTION_L * axisPID[ROLL];
                servo[1]  = PITCH_DIRECTION_R * axisPID[PITCH] + ROLL_DIRECTION_R * axisPID[ROLL];
            }
            servo[0]  = constrain(servo[0] + cfg.wing_left_mid , WING_LEFT_MIN, WING_LEFT_MAX);
            servo[1]  = constrain(servo[1] + cfg.wing_right_mid, WING_RIGHT_MIN, WING_RIGHT_MAX);
            break;
    }

    // do camstab
    if (feature(FEATURE_SERVO_TILT)) {
        servo[0] = TILT_PITCH_MIDDLE + rcData[AUX3] - 1500;
        servo[1] = TILT_ROLL_MIDDLE + rcData[AUX4] - 1500;

        if (rcOptions[BOXCAMSTAB]) {
            servo[0] += cfg.tilt_pitch_prop * angle[PITCH] / 16;
            servo[1] += cfg.tilt_roll_prop * angle[ROLL]  / 16;
        }

        servo[0] = constrain(servo[0], TILT_PITCH_MIN, TILT_PITCH_MAX);
        servo[1] = constrain(servo[1], TILT_ROLL_MIN, TILT_ROLL_MAX);
    }

    // do camtrig (this doesn't actually work)        
    if (feature(FEATURE_CAMTRIG)) {
        if (camCycle == 1) {
            if (camState == 0) {
                servo[2] = CAM_SERVO_HIGH;
                camState = 1;
                camTime = millis();
            } else if (camState == 1) {
                if ((millis() - camTime) > CAM_TIME_HIGH) {
                    servo[2] = CAM_SERVO_LOW;
                    camState = 2;
                    camTime = millis();
                }
            } else {                //camState ==2
                if ((millis() - camTime) > CAM_TIME_LOW) {
                    camState = 0;
                    camCycle = 0;
                }
            }
        }
        if (rcOptions[BOXCAMTRIG])
            camCycle = 1;
    }                

    maxMotor = motor[0];
    for (i = 1; i < numberMotor; i++)
        if (motor[i] > maxMotor)
            maxMotor = motor[i];
    for (i = 0; i < numberMotor; i++) {
        if (maxMotor > cfg.maxthrottle)     // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motor[i] -= maxMotor - cfg.maxthrottle;
        motor[i] = constrain(motor[i], cfg.minthrottle, cfg.maxthrottle);
        if ((rcData[THROTTLE]) < cfg.mincheck) {
            if (!feature(FEATURE_MOTOR_STOP))
                motor[i] = cfg.minthrottle;
            else
                motor[i] = cfg.mincommand;
        }
        if (armed == 0)
            motor[i] = cfg.mincommand;
    }

#if (LOG_VALUES == 2) || defined(POWERMETER_SOFT)
    uint32_t amp;
    /* true cubic function; when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 500 */
    /* Lookup table moved to PROGMEM 11/21/2001 by Danal */
    static uint16_t amperes[64] = {
    0, 2, 6, 15, 30, 52, 82, 123, 175, 240, 320, 415, 528, 659, 811, 984, 1181, 1402, 1648, 1923, 2226, 2559, 2924, 3322, 3755, 4224, 4730, 5276, 5861, 6489, 7160, 7875, 8637, 9446, 10304, 11213, 12173, 13187, 14256, 15381, 16564, 17805, 19108, 20472, 21900, 23392, 24951, 26578, 28274, 30041, 31879, 33792, 35779, 37843, 39984, 42205, 44507, 46890, 49358, 51910, 54549, 57276, 60093, 63000};

    if (vbat) {                 // by all means - must avoid division by zero 
        for (i = 0; i < NUMBER_MOTOR; i++) {
            amp = amperes[((motor[i] - 1000) >> 4)] / vbat;     // range mapped from [1000:2000] => [0:1000]; then break that up into 64 ranges; lookup amp
#if (LOG_VALUES == 2)
            pMeter[i] += amp;   // sum up over time the mapped ESC input 
#endif
#if defined(POWERMETER_SOFT)
            pMeter[PMOTOR_SUM] += amp;  // total sum over all motors
#endif
        }
    }
#endif
}
