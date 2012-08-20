#include "board.h"
#include "mw.h"

static uint8_t numberMotor = 4;
uint8_t useServo = 0;

int16_t motor[MAX_MOTORS];
int16_t servo[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

typedef struct {
    uint8_t enabled;        // is this mix channel enabled
    float throttle;         // proportion of throttle affect
    float pitch;
    float roll;
    float yaw;
} mixerPower_t;

static mixerPower_t customMixer[12];

void mixerInit(void)
{
    int i;

    // enable servos for mixes that require them. note, this shifts motor counts.
    if (cfg.mixerConfiguration == MULTITYPE_BI || cfg.mixerConfiguration == MULTITYPE_TRI || cfg.mixerConfiguration == MULTITYPE_FLYING_WING || 
        cfg.mixerConfiguration == MULTITYPE_AIRPLANE || cfg.mixerConfiguration == MULTITYPE_GIMBAL)
        useServo = 1;
    // if we want camstab/trig, that also enabled servos. this is kinda lame. maybe rework feature bits later.
    if (feature(FEATURE_SERVO_TILT))
        useServo = 1;

    switch (cfg.mixerConfiguration) {
        case MULTITYPE_GIMBAL:
            numberMotor = 0;
            break;

        case MULTITYPE_AIRPLANE:
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

        case MULTITYPE_CUSTOM:
            numberMotor = 0;
            for (i = 0; i < MAX_MOTORS; i++) {
                if (customMixer[i].enabled)
                    numberMotor++;
            }
            break;
    }
}

void writeServos(void)
{
    if (!useServo)
        return;

    switch (cfg.mixerConfiguration) {
        case MULTITYPE_BI:
            pwmWriteServo(0, servo[4]);
            pwmWriteServo(1, servo[5]);
            break;

        case MULTITYPE_TRI:
            pwmWriteServo(0, servo[5]);
            break;

        case MULTITYPE_AIRPLANE:

            break;

        case MULTITYPE_FLYING_WING:

            break;

        default:
            // Two servos for SERVO_TILT, if enabled
            if (feature(FEATURE_SERVO_TILT)) {
                pwmWriteServo(0, servo[0]);
                pwmWriteServo(1, servo[1]);
            }
            break;
    }
}

extern uint8_t cliMode;

void writeMotors(void)
{
    uint8_t i;

    for (i = 0; i < numberMotor; i++)
        pwmWriteMotor(i, motor[i]);
}

void writeAllMotors(int16_t mc)
{
    uint8_t i;

    // Sends commands to all motors
    for (i = 0; i < numberMotor; i++)
        motor[i] = mc;
    writeMotors();
}

#define PIDMIX(R, P, Y) rcCommand[THROTTLE] + axisPID[ROLL] * R + axisPID[PITCH] * P + cfg.yaw_direction * axisPID[YAW] * Y

static void airplaneMixer(void)
{
#if 0
    uint16_t servomid[8];
    int16_t flaperons[2] = { 0, 0 };

    for (i = 0; i < 8; i++) {
        servomid[i] = 1500 + cfg.servotrim[i]; // servo center is 1500?
    }

    if (!f.ARMED)
        motor[0] = cfg.mincommand; // Kill throttle when disarmed
    else
        motor[0] = rcData[THROTTLE];

    if (cfg.flaperons) {
        
        
    }

    if (cfg.flaps) {
        int16_t flap = 1500 - constrain(rcData[cfg.flaps], cfg.servoendpoint_low[2], cfg.servoendpoint_high[2]);
        static int16_t slowFlaps = flap;

        if (cfg.flapspeed) {
            if (slowFlaps < flap) {
                slowFlaps += cfg.flapspeed;
            } else if (slowFlaps > flap) {
                slowFlaps -= cfg.flapspeed;
            }
        } else {
            slowFlaps = flap;
        }
        servo[2] = servomid[2] + (slowFlaps * cfg.servoreverse[2]);
    }

    if (f.PASSTHRU_MODE) { // Direct passthru from RX
        servo[3] = servomid[3] + ((rcCommand[ROLL] + flapperons[0]) * cfg.servoreverse[3]);     //   Wing 1
        servo[4] = servomid[4] + ((rcCommand[ROLL] + flapperons[1]) * cfg.servoreverse[4]);     //   Wing 2
        servo[5] = servomid[5] + (rcCommand[YAW] * cfg.servoreverse[5]);                        //   Rudder
        servo[6] = servomid[6] + (rcCommand[PITCH] * cfg.servoreverse[6]);                      //   Elevator
    } else { // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
        servo[3] = (servomid[3] + ((axisPID[ROLL] + flapperons[0]) * cfg.servoreverse[3]));     //   Wing 1
        servo[4] = (servomid[4] + ((axisPID[ROLL] + flapperons[1]) * cfg.servoreverse[4]));     //   Wing 2
        servo[5] = (servomid[5] + (axisPID[YAW] * cfg.servoreverse[5]));                        //   Rudder
        servo[6] = (servomid[6] + (axisPID[PITCH] * cfg.servoreverse[6]));                      //   Elevator
    }
#endif
}

void mixTable(void)
{
    int16_t maxMotor;
    uint32_t i;

    if (numberMotor > 3) {
        // prevent "yaw jump" during yaw correction
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
            servo[5] = constrain(cfg.tri_yaw_middle + cfg.yaw_direction * axisPID[YAW], cfg.tri_yaw_min, cfg.tri_yaw_max); //REAR
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
            motor[0] = PIDMIX(-4/5,+9/10,+1); //REAR_R 
            motor[1] = PIDMIX(-4/5,-9/10,+1); //FRONT_R 
            motor[2] = PIDMIX(+4/5,+9/10,-1); //REAR_L 
            motor[3] = PIDMIX(+4/5,-9/10,-1); //FRONT_L 
            motor[4] = PIDMIX(-4/5 ,+0 ,-1); //RIGHT 
            motor[5] = PIDMIX(+4/5 ,+0 ,+1); //LEFT
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
            motor[0] = PIDMIX(+0, +1, +1);      //REAR_R 
            motor[1] = PIDMIX(-1, -1, +0);      //FRONT_R 
            motor[2] = PIDMIX(+0, +1, -1);      //REAR_L 
            motor[3] = PIDMIX(+1, -1, -0);      //FRONT_L
            break;

        case MULTITYPE_GIMBAL:
            servo[0] = constrain(cfg.gimbal_pitch_mid + cfg.gimbal_pitch_gain * angle[PITCH] / 16 + rcCommand[PITCH], cfg.gimbal_pitch_min, cfg.gimbal_pitch_max);
            servo[1] = constrain(cfg.gimbal_roll_mid + cfg.gimbal_roll_gain * angle[ROLL] / 16 + rcCommand[ROLL], cfg.gimbal_roll_min, cfg.gimbal_roll_max);
            break;
        
        case MULTITYPE_AIRPLANE:
            airplaneMixer();
            break;

        case MULTITYPE_FLYING_WING:
            motor[0] = rcCommand[THROTTLE];
            if (f.PASSTHRU_MODE) { // do not use sensors for correction, simple 2 channel mixing
                int p = 0, r = 0;
                servo[0] = p * (rcData[PITCH] - cfg.midrc) + r * (rcData[ROLL] - cfg.midrc);
                servo[1] = p * (rcData[PITCH] - cfg.midrc) + r * (rcData[ROLL] - cfg.midrc);
            } else { // use sensors to correct (gyro only or gyro+acc)
                int p = 0, r = 0;
                servo[0] = p * axisPID[PITCH] + r * axisPID[ROLL];
                servo[1] = p * axisPID[PITCH] + r * axisPID[ROLL];
            }
            break;
    }

    // do camstab
    if (feature(FEATURE_SERVO_TILT)) {
        uint16_t aux[2] = { 0, 0 };

        if ((cfg.gimbal_flags & GIMBAL_NORMAL) || (cfg.gimbal_flags & GIMBAL_TILTONLY))
            aux[0] = rcData[AUX3] - cfg.midrc;
        if (!(cfg.gimbal_flags & GIMBAL_DISABLEAUX34))
            aux[1] = rcData[AUX4] - cfg.midrc;

        servo[0] = cfg.gimbal_pitch_mid + aux[0];
        servo[1] = cfg.gimbal_roll_mid + aux[1];

        if (rcOptions[BOXCAMSTAB]) {
            servo[0] += cfg.gimbal_pitch_gain * angle[PITCH] / 16;
            servo[1] += cfg.gimbal_roll_gain * angle[ROLL]  / 16;
        }

        servo[0] = constrain(servo[0], cfg.gimbal_pitch_min, cfg.gimbal_pitch_max);
        servo[1] = constrain(servo[1], cfg.gimbal_roll_min, cfg.gimbal_roll_max);
    }

    if (cfg.gimbal_flags & GIMBAL_FORWARDAUX) {
        int offset = 0;
        if (feature(FEATURE_SERVO_TILT))
            offset = 2;
        for (i = 0; i < 4; i++)
            pwmWriteServo(i + offset, rcData[AUX1 + i]);
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
        if (!f.ARMED)
            motor[i] = cfg.mincommand;
    }
}
