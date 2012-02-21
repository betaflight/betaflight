#include "board.h"
#include "mw.h"

#if defined(BI) || defined(TRI) || defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(CAMTRIG)
#define SERVO
#endif

#if defined(GIMBAL)
#define NUMBER_MOTOR 0
#define PRI_SERVO_FROM   1      // use servo from 1 to 2
#define PRI_SERVO_TO     2
#elif defined(FLYING_WING)
#define NUMBER_MOTOR 1
#define PRI_SERVO_FROM   1      // use servo from 1 to 2
#define PRI_SERVO_TO     2
#elif defined(BI)
#define NUMBER_MOTOR 2
#define PRI_SERVO_FROM   5      // use servo from 5 to 6
#define PRI_SERVO_TO     6
#elif defined(TRI)
#define NUMBER_MOTOR 3
#define PRI_SERVO_FROM   6      // use only servo 5
#define PRI_SERVO_TO     6
#elif defined(QUADP) || defined(QUADX) || defined(Y4)
#define NUMBER_MOTOR 4
#elif defined(Y6) || defined(HEX6) || defined(HEX6X)
#define NUMBER_MOTOR 6
#elif defined(OCTOX8) || defined(OCTOFLATP) || defined(OCTOFLATX)
#define NUMBER_MOTOR 8
#endif

void writeServos(void)
{
#if defined(SERVO)

#endif
}

void writeMotors(void)
{
    uint8_t i;
    
    for (i = 0; i < NUMBER_MOTOR; i++)
        pwmWrite(i, motor[i]);
}

void writeAllMotors(int16_t mc)
{
    uint8_t i;
    
    // Sends commands to all motors
    for (i = 0; i < NUMBER_MOTOR; i++)
        motor[i] = mc;
    writeMotors();
}

void mixTable(void)
{
    int16_t maxMotor;
    uint8_t i, axis;
    static uint8_t camCycle = 0;
    static uint8_t camState = 0;
    static uint32_t camTime = 0;

#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL] * X + axisPID[PITCH] * Y + YAW_DIRECTION * axisPID[YAW] * Z

#if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));
#endif
#ifdef BI
    motor[0] = PIDMIX(+1, 0, 0);        //LEFT
    motor[1] = PIDMIX(-1, 0, 0);        //RIGHT        
    servo[4] = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] + axisPID[PITCH]), 1020, 2000);   //LEFT
    servo[5] = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] - axisPID[PITCH]), 1020, 2000);   //RIGHT
#endif
#ifdef TRI
    motor[0] = PIDMIX(0, +4 / 3, 0);    //REAR
    motor[1] = PIDMIX(-1, -2 / 3, 0);   //RIGHT
    motor[2] = PIDMIX(+1, -2 / 3, 0);   //LEFT
    servo[5] = constrain(tri_yaw_middle + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR

#endif
#ifdef QUADP
    motor[0] = PIDMIX(0, +1, -1);       //REAR
    motor[1] = PIDMIX(-1, 0, +1);       //RIGHT
    motor[2] = PIDMIX(+1, 0, +1);       //LEFT
    motor[3] = PIDMIX(0, -1, -1);       //FRONT
#endif
#ifdef QUADX
    motor[0] = PIDMIX(-1, +1, -1);      //REAR_R
    motor[1] = PIDMIX(-1, -1, +1);      //FRONT_R
    motor[2] = PIDMIX(+1, +1, +1);      //REAR_L
    motor[3] = PIDMIX(+1, -1, -1);      //FRONT_L
#endif
#ifdef Y4
    motor[0] = PIDMIX(+0, +1, -1);      //REAR_1 CW
    motor[1] = PIDMIX(-1, -1, 0);       //FRONT_R CCW
    motor[2] = PIDMIX(+0, +1, +1);      //REAR_2 CCW
    motor[3] = PIDMIX(+1, -1, 0);       //FRONT_L CW
#endif
#ifdef Y6
    motor[0] = PIDMIX(+0, +4 / 3, +1);  //REAR
    motor[1] = PIDMIX(-1, -2 / 3, -1);  //RIGHT
    motor[2] = PIDMIX(+1, -2 / 3, -1);  //LEFT
    motor[3] = PIDMIX(+0, +4 / 3, -1);  //UNDER_REAR
    motor[4] = PIDMIX(-1, -2 / 3, +1);  //UNDER_RIGHT
    motor[5] = PIDMIX(+1, -2 / 3, +1);  //UNDER_LEFT    
#endif
#ifdef HEX6
    motor[0] = PIDMIX(-1 / 2, +1 / 2, +1);      //REAR_R
    motor[1] = PIDMIX(-1 / 2, -1 / 2, -1);      //FRONT_R
    motor[2] = PIDMIX(+1 / 2, +1 / 2, +1);      //REAR_L
    motor[3] = PIDMIX(+1 / 2, -1 / 2, -1);      //FRONT_L
    motor[4] = PIDMIX(+0, -1, +1);      //FRONT
    motor[5] = PIDMIX(+0, +1, -1);      //REAR
#endif
#ifdef HEX6X
    motor[0] = PIDMIX(-1 / 2, +1 / 2, +1);      //REAR_R
    motor[1] = PIDMIX(-1 / 2, -1 / 2, +1);      //FRONT_R
    motor[2] = PIDMIX(+1 / 2, +1 / 2, -1);      //REAR_L
    motor[3] = PIDMIX(+1 / 2, -1 / 2, -1);      //FRONT_L
    motor[4] = PIDMIX(-1, +0, -1);      //RIGHT
    motor[5] = PIDMIX(+1, +0, +1);      //LEFT
#endif
#ifdef OCTOX8
    motor[0] = PIDMIX(-1, +1, -1);      //REAR_R
    motor[1] = PIDMIX(-1, -1, +1);      //FRONT_R
    motor[2] = PIDMIX(+1, +1, +1);      //REAR_L
    motor[3] = PIDMIX(+1, -1, -1);      //FRONT_L
    motor[4] = PIDMIX(-1, +1, +1);      //UNDER_REAR_R
    motor[5] = PIDMIX(-1, -1, -1);      //UNDER_FRONT_R
    motor[6] = PIDMIX(+1, +1, -1);      //UNDER_REAR_L
    motor[7] = PIDMIX(+1, -1, +1);      //UNDER_FRONT_L
#endif
#ifdef OCTOFLATP
    motor[0] = PIDMIX(+7 / 10, -7 / 10, +1);    //FRONT_L
    motor[1] = PIDMIX(-7 / 10, -7 / 10, +1);    //FRONT_R
    motor[2] = PIDMIX(-7 / 10, +7 / 10, +1);    //REAR_R
    motor[3] = PIDMIX(+7 / 10, +7 / 10, +1);    //REAR_L
    motor[4] = PIDMIX(+0, -1, -1);      //FRONT
    motor[5] = PIDMIX(-1, +0, -1);      //RIGHT
    motor[6] = PIDMIX(+0, +1, -1);      //REAR
    motor[7] = PIDMIX(+1, +0, -1);      //LEFT 
#endif
#ifdef OCTOFLATX
    motor[0] = PIDMIX(+1, -1 / 2, +1);  //MIDFRONT_L
    motor[1] = PIDMIX(-1 / 2, -1, +1);  //FRONT_R
    motor[2] = PIDMIX(-1, +1 / 2, +1);  //MIDREAR_R
    motor[3] = PIDMIX(+1 / 2, +1, +1);  //REAR_L
    motor[4] = PIDMIX(+1 / 2, -1, -1);  //FRONT_L
    motor[5] = PIDMIX(-1, -1 / 2, -1);  //MIDFRONT_R
    motor[6] = PIDMIX(-1 / 2, +1, -1);  //REAR_R
    motor[7] = PIDMIX(+1, +1 / 2, -1);  //MIDREAR_L 
#endif
#ifdef VTAIL4
    motor[0] = PIDMIX(+0, +1, -1 / 2);      //REAR_R 
    motor[1] = PIDMIX(-1, -1, +2 / 10); //FRONT_R 
    motor[2] = PIDMIX(+0, +1, +1 / 2);      //REAR_L 
    motor[3] = PIDMIX(+1, -1, -2 / 10); //FRONT_L
#endif


#ifdef SERVO_TILT
    servo[0] = TILT_PITCH_MIDDLE + rcData[AUX3] - 1500;
    servo[1] = TILT_ROLL_MIDDLE + rcData[AUX4] - 1500;
    
    if (rcOptions[BOXCAMSTAB]) {
        servo[0] += TILT_PITCH_PROP * angle[PITCH] / 16;
        servo[1] += TILT_ROLL_PROP * angle[ROLL]  / 16;
    }
    
    servo[0] = constrain(servo[0], TILT_PITCH_MIN, TILT_PITCH_MAX);
    servo[1] = constrain(servo[1], TILT_ROLL_MIN, TILT_ROLL_MAX);   
#endif
#ifdef GIMBAL
    servo[0] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] / 16 + rcCommand[PITCH], TILT_PITCH_MIN, TILT_PITCH_MAX);
    servo[1] = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP * angle[ROLL] / 16 + rcCommand[ROLL], TILT_ROLL_MIN, TILT_ROLL_MAX);
#endif
#ifdef FLYING_WING
    motor[0] = rcCommand[THROTTLE];
    if (passThruMode) {// do not use sensors for correction, simple 2 channel mixing
        servo[0]  = PITCH_DIRECTION_L * (rcData[PITCH] - MIDRC) + ROLL_DIRECTION_L * (rcData[ROLL] - MIDRC);
        servo[1]  = PITCH_DIRECTION_R * (rcData[PITCH] - MIDRC) + ROLL_DIRECTION_R * (rcData[ROLL] - MIDRC);
    } else {                    // use sensors to correct (gyro only or gyro+acc according to aux1/aux2 configuration
        servo[0]  = PITCH_DIRECTION_L * axisPID[PITCH] + ROLL_DIRECTION_L * axisPID[ROLL];
        servo[1]  = PITCH_DIRECTION_R * axisPID[PITCH] + ROLL_DIRECTION_R * axisPID[ROLL];
    }
    servo[0]  = constrain(servo[0] + wing_left_mid , WING_LEFT_MIN, WING_LEFT_MAX);
    servo[1]  = constrain(servo[1] + wing_right_mid, WING_RIGHT_MIN, WING_RIGHT_MAX);
#endif
#if defined(CAMTRIG)
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
#endif

    maxMotor = motor[0];
    for (i = 1; i < NUMBER_MOTOR; i++)
        if (motor[i] > maxMotor)
            maxMotor = motor[i];
    for (i = 0; i < NUMBER_MOTOR; i++) {
        if (maxMotor > MAXTHROTTLE)     // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motor[i] -= maxMotor - MAXTHROTTLE;
        motor[i] = constrain(motor[i], MINTHROTTLE, MAXTHROTTLE);
        if ((rcData[THROTTLE]) < MINCHECK)
#ifndef MOTOR_STOP
            motor[i] = MINTHROTTLE;
#else
            motor[i] = MINCOMMAND;
#endif
        if (armed == 0)
            motor[i] = MINCOMMAND;
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
