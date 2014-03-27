#include "board.h"
#include "mw.h"

static uint8_t numberMotor = 0;
int16_t motor[MAX_MOTORS];
int16_t motor_disarmed[MAX_MOTORS];
int16_t servo[MAX_SERVOS] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

static motorMixer_t currentMixer[MAX_MOTORS];

static const motorMixer_t mixerTri[] = {
    { 1.0f,  0.0f,  1.333333f,  0.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f,  0.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f,  0.0f },     // LEFT
};

static const motorMixer_t mixerQuadP[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR
    { 1.0f, -1.0f,  0.0f,  1.0f },          // RIGHT
    { 1.0f,  1.0f,  0.0f,  1.0f },          // LEFT
    { 1.0f,  0.0f, -1.0f, -1.0f },          // FRONT
};

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};

static const motorMixer_t mixerBi[] = {
    { 1.0f,  1.0f,  0.0f,  0.0f },          // LEFT
    { 1.0f, -1.0f,  0.0f,  0.0f },          // RIGHT
};

static const motorMixer_t mixerY6[] = {
    { 1.0f,  0.0f,  1.333333f,  1.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f, -1.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f, -1.0f },     // LEFT
    { 1.0f,  0.0f,  1.333333f, -1.0f },     // UNDER_REAR
    { 1.0f, -1.0f, -0.666667f,  1.0f },     // UNDER_RIGHT
    { 1.0f,  1.0f, -0.666667f,  1.0f },     // UNDER_LEFT
};

static const motorMixer_t mixerHex6P[] = {
    { 1.0f, -0.866025f,  0.5f,  1.0f },     // REAR_R
    { 1.0f, -0.866025f, -0.5f, -1.0f },     // FRONT_R
    { 1.0f,  0.866025f,  0.5f,  1.0f },     // REAR_L
    { 1.0f,  0.866025f, -0.5f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,      -1.0f,  1.0f },     // FRONT
    { 1.0f,  0.0f,       1.0f, -1.0f },     // REAR
};

static const motorMixer_t mixerY4[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_TOP CW
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R CCW
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_BOTTOM CCW
    { 1.0f,  1.0f, -1.0f,  0.0f },          // FRONT_L CW
};

static const motorMixer_t mixerHex6X[] = {
    { 1.0f, -0.5f,  0.866025f,  1.0f },     // REAR_R
    { 1.0f, -0.5f, -0.866025f,  1.0f },     // FRONT_R
    { 1.0f,  0.5f,  0.866025f, -1.0f },     // REAR_L
    { 1.0f,  0.5f, -0.866025f, -1.0f },     // FRONT_L
    { 1.0f, -1.0f,  0.0f,      -1.0f },     // RIGHT
    { 1.0f,  1.0f,  0.0f,       1.0f },     // LEFT
};

static const motorMixer_t mixerOctoX8[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f,  1.0f,  1.0f },          // UNDER_REAR_R
    { 1.0f, -1.0f, -1.0f, -1.0f },          // UNDER_FRONT_R
    { 1.0f,  1.0f,  1.0f, -1.0f },          // UNDER_REAR_L
    { 1.0f,  1.0f, -1.0f,  1.0f },          // UNDER_FRONT_L
};

static const motorMixer_t mixerOctoFlatP[] = {
    { 1.0f,  0.707107f, -0.707107f,  1.0f },    // FRONT_L
    { 1.0f, -0.707107f, -0.707107f,  1.0f },    // FRONT_R
    { 1.0f, -0.707107f,  0.707107f,  1.0f },    // REAR_R
    { 1.0f,  0.707107f,  0.707107f,  1.0f },    // REAR_L
    { 1.0f,  0.0f, -1.0f, -1.0f },              // FRONT
    { 1.0f, -1.0f,  0.0f, -1.0f },              // RIGHT
    { 1.0f,  0.0f,  1.0f, -1.0f },              // REAR
    { 1.0f,  1.0f,  0.0f, -1.0f },              // LEFT
};

static const motorMixer_t mixerOctoFlatX[] = {
    { 1.0f,  1.0f, -0.5f,  1.0f },          // MIDFRONT_L
    { 1.0f, -0.5f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f, -1.0f,  0.5f,  1.0f },          // MIDREAR_R
    { 1.0f,  0.5f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  0.5f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f, -0.5f, -1.0f },          // MIDFRONT_R
    { 1.0f, -0.5f,  1.0f, -1.0f },          // REAR_R
    { 1.0f,  1.0f,  0.5f, -1.0f },          // MIDREAR_L
};

static const motorMixer_t mixerVtail4[] = {
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -0.0f },          // FRONT_L
};

static const motorMixer_t mixerHex6H[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },     // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },     // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },     // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,  0.0f,  0.0f },     // RIGHT
    { 1.0f,  0.0f,  0.0f,  0.0f },     // LEFT
};

static const motorMixer_t mixerDualcopter[] = {
    { 1.0f,  0.0f,  0.0f, -1.0f },          // LEFT
    { 1.0f,  0.0f,  0.0f,  1.0f },          // RIGHT
};

// Keep this synced with MultiType struct in mw.h!
const mixer_t mixers[] = {
//    Mo Se Mixtable
    { 0, 0, NULL },                // entry 0
    { 3, 1, mixerTri },            // MULTITYPE_TRI
    { 4, 0, mixerQuadP },          // MULTITYPE_QUADP
    { 4, 0, mixerQuadX },          // MULTITYPE_QUADX
    { 2, 1, mixerBi },             // MULTITYPE_BI
    { 0, 1, NULL },                // * MULTITYPE_GIMBAL
    { 6, 0, mixerY6 },             // MULTITYPE_Y6
    { 6, 0, mixerHex6P },          // MULTITYPE_HEX6
    { 1, 1, NULL },                // * MULTITYPE_FLYING_WING
    { 4, 0, mixerY4 },             // MULTITYPE_Y4
    { 6, 0, mixerHex6X },          // MULTITYPE_HEX6X
    { 8, 0, mixerOctoX8 },         // MULTITYPE_OCTOX8
    { 8, 0, mixerOctoFlatP },      // MULTITYPE_OCTOFLATP
    { 8, 0, mixerOctoFlatX },      // MULTITYPE_OCTOFLATX
    { 1, 1, NULL },                // * MULTITYPE_AIRPLANE
    { 0, 1, NULL },                // * MULTITYPE_HELI_120_CCPM
    { 0, 1, NULL },                // * MULTITYPE_HELI_90_DEG
    { 4, 0, mixerVtail4 },         // MULTITYPE_VTAIL4
    { 6, 0, mixerHex6H },          // MULTITYPE_HEX6H
    { 0, 1, NULL },                // * MULTITYPE_PPM_TO_SERVO
    { 2, 1, mixerDualcopter  },    // MULTITYPE_DUALCOPTER
    { 1, 1, NULL },                // MULTITYPE_SINGLECOPTER
    { 0, 0, NULL },                // MULTITYPE_CUSTOM
};

int16_t servoMiddle(int nr)
{
    // Normally, servo.middle is a value between 1000..2000, but for the purposes of stupid, if it's less than
    // the number of RC channels, it means the center value is taken FROM that RC channel (by its index)
    if (cfg.servoConf[nr].middle < RC_CHANS && nr < MAX_SERVOS)
        return rcData[cfg.servoConf[nr].middle];
    else
        return cfg.servoConf[nr].middle;
}

int servoDirection(int nr, int lr)
{
    // servo.rate is overloaded for servos that don't have a rate, but only need direction
    // bit set = negative, clear = positive
    // rate[2] = ???_direction
    // rate[1] = roll_direction
    // rate[0] = pitch_direction
    // servo.rate is also used as gimbal gain multiplier (yeah)
    if (cfg.servoConf[nr].rate & lr)
        return -1;
    else
        return 1;
}

void mixerInit(void)
{
    int i;

    // enable servos for mixes that require them. note, this shifts motor counts.
    core.useServo = mixers[mcfg.mixerConfiguration].useServo;
    // if we want camstab/trig, that also enables servos, even if mixer doesn't
    if (feature(FEATURE_SERVO_TILT))
        core.useServo = 1;

    if (mcfg.mixerConfiguration == MULTITYPE_CUSTOM) {
        // load custom mixer into currentMixer
        for (i = 0; i < MAX_MOTORS; i++) {
            // check if done
            if (mcfg.customMixer[i].throttle == 0.0f)
                break;
            currentMixer[i] = mcfg.customMixer[i];
            numberMotor++;
        }
    } else {
        numberMotor = mixers[mcfg.mixerConfiguration].numberMotor;
        // copy motor-based mixers
        if (mixers[mcfg.mixerConfiguration].motor) {
            for (i = 0; i < numberMotor; i++)
                currentMixer[i] = mixers[mcfg.mixerConfiguration].motor[i];
        }
    }

    // in 3D mode, mixer gain has to be halved
    if (feature(FEATURE_3D)) {
        if (numberMotor > 1) {
            for (i = 0; i < numberMotor; i++) {
                currentMixer[i].pitch *= 0.5f;
                currentMixer[i].roll *= 0.5f;
                currentMixer[i].yaw *= 0.5f;
            }
        }
    }

    // set flag that we're on something with wings
    if (mcfg.mixerConfiguration == MULTITYPE_FLYING_WING ||
        mcfg.mixerConfiguration == MULTITYPE_AIRPLANE)
        f.FIXED_WING = 1;
    else
        f.FIXED_WING = 0;

    mixerResetMotors();
}

void mixerResetMotors(void)
{
    int i;
    // set disarmed motor values
    for (i = 0; i < MAX_MOTORS; i++)
        motor_disarmed[i] = feature(FEATURE_3D) ? mcfg.neutral3d : mcfg.mincommand;
}

void mixerLoadMix(int index)
{
    int i;

    // we're 1-based
    index++;
    // clear existing
    for (i = 0; i < MAX_MOTORS; i++)
        mcfg.customMixer[i].throttle = 0.0f;

    // do we have anything here to begin with?
    if (mixers[index].motor != NULL) {
        for (i = 0; i < mixers[index].numberMotor; i++)
            mcfg.customMixer[i] = mixers[index].motor[i];
    }
}

void writeServos(void)
{
    if (!core.useServo)
        return;

    switch (mcfg.mixerConfiguration) {
        case MULTITYPE_BI:
            pwmWriteServo(0, servo[4]);
            pwmWriteServo(1, servo[5]);
            break;

        case MULTITYPE_TRI:
            if (cfg.tri_unarmed_servo) {
                // if unarmed flag set, we always move servo
                pwmWriteServo(0, servo[5]);
            } else {
                // otherwise, only move servo when copter is armed
                if (f.ARMED)
                    pwmWriteServo(0, servo[5]);
                else
                    pwmWriteServo(0, 0); // kill servo signal completely.
            }
            break;

        case MULTITYPE_FLYING_WING:
            pwmWriteServo(0, servo[3]);
            pwmWriteServo(1, servo[4]);
            break;

        case MULTITYPE_GIMBAL:
            pwmWriteServo(0, servo[0]);
            pwmWriteServo(1, servo[1]);
            break;

        case MULTITYPE_DUALCOPTER:
            pwmWriteServo(0, servo[4]);
            pwmWriteServo(1, servo[5]);
            break;

        case MULTITYPE_AIRPLANE:
        case MULTITYPE_SINGLECOPTER:
            pwmWriteServo(0, servo[3]);
            pwmWriteServo(1, servo[4]);
            pwmWriteServo(2, servo[5]);
            pwmWriteServo(3, servo[6]);
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

static void airplaneMixer(void)
{
    int16_t flapperons[2] = { 0, 0 };
    int i;

    if (!f.ARMED)
        servo[7] = mcfg.mincommand; // Kill throttle when disarmed
    else
        servo[7] = constrain(rcCommand[THROTTLE], mcfg.minthrottle, mcfg.maxthrottle);
    motor[0] = servo[7];

#if 0
    if (cfg.flaperons) {


    }
#endif

    if (mcfg.flaps_speed) {
        // configure SERVO3 middle point in GUI to using an AUX channel for FLAPS control
        // use servo min, servo max and servo rate for proper endpoints adjust
        static int16_t slow_LFlaps;
        int16_t lFlap = servoMiddle(2);

        lFlap = constrain(lFlap, cfg.servoConf[2].min, cfg.servoConf[2].max);
        lFlap = mcfg.midrc - lFlap; // shouldn't this be servoConf[2].middle?
        if (slow_LFlaps < lFlap)
            slow_LFlaps += mcfg.flaps_speed;
        else if (slow_LFlaps > lFlap)
            slow_LFlaps -= mcfg.flaps_speed;

        servo[2] = ((int32_t)cfg.servoConf[2].rate * slow_LFlaps) / 100L;
        servo[2] += mcfg.midrc;
    }

    if (f.PASSTHRU_MODE) {   // Direct passthru from RX
        servo[3] = rcCommand[ROLL] + flapperons[0];     // Wing 1
        servo[4] = rcCommand[ROLL] + flapperons[1];     // Wing 2
        servo[5] = rcCommand[YAW];                      // Rudder
        servo[6] = rcCommand[PITCH];                    // Elevator
    } else {
        // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
        servo[3] = axisPID[ROLL] + flapperons[0];       // Wing 1
        servo[4] = axisPID[ROLL] + flapperons[1];       // Wing 2
        servo[5] = axisPID[YAW];                        // Rudder
        servo[6] = axisPID[PITCH];                      // Elevator
    }
    for (i = 3; i < 7; i++) {
        servo[i] = ((int32_t)cfg.servoConf[i].rate * servo[i]) / 100L; // servo rates
        servo[i] += servoMiddle(i);
    }
}

void mixTable(void)
{
    int16_t maxMotor;
    uint32_t i;

    if (numberMotor > 3) {
        // prevent "yaw jump" during yaw correction
        axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));
    }

    // motors for non-servo mixes
    if (numberMotor > 1)
        for (i = 0; i < numberMotor; i++)
            motor[i] = rcCommand[THROTTLE] * currentMixer[i].throttle + axisPID[PITCH] * currentMixer[i].pitch + axisPID[ROLL] * currentMixer[i].roll + -cfg.yaw_direction * axisPID[YAW] * currentMixer[i].yaw;

    // airplane / servo mixes
    switch (mcfg.mixerConfiguration) {
        case MULTITYPE_BI:
            servo[4] = (servoDirection(4, 2) * axisPID[YAW]) + (servoDirection(4, 1) * axisPID[PITCH]) + servoMiddle(4);     // LEFT
            servo[5] = (servoDirection(5, 2) * axisPID[YAW]) + (servoDirection(5, 1) * axisPID[PITCH]) + servoMiddle(5);     // RIGHT
            break;

        case MULTITYPE_TRI:
            servo[5] = (servoDirection(5, 1) * axisPID[YAW]) + servoMiddle(5); // REAR
            break;

        case MULTITYPE_GIMBAL:
            servo[0] = (((int32_t)cfg.servoConf[0].rate * angle[PITCH]) / 50) + servoMiddle(0);
            servo[1] = (((int32_t)cfg.servoConf[1].rate * angle[ROLL]) / 50) + servoMiddle(1);
            break;

        case MULTITYPE_AIRPLANE:
            airplaneMixer();
            break;

        case MULTITYPE_FLYING_WING:
            if (!f.ARMED)
                servo[7] = mcfg.mincommand;
            else
                servo[7] = constrain(rcCommand[THROTTLE], mcfg.minthrottle, mcfg.maxthrottle);
            motor[0] = servo[7];
            if (f.PASSTHRU_MODE) {
                // do not use sensors for correction, simple 2 channel mixing
                servo[3] = (servoDirection(3, 1) * rcCommand[PITCH]) + (servoDirection(3, 2) * rcCommand[ROLL]);
                servo[4] = (servoDirection(4, 1) * rcCommand[PITCH]) + (servoDirection(4, 2) * rcCommand[ROLL]);
            } else {
                // use sensors to correct (gyro only or gyro + acc)
                servo[3] = (servoDirection(3, 1) * axisPID[PITCH]) + (servoDirection(3, 2) * axisPID[ROLL]);
                servo[4] = (servoDirection(4, 1) * axisPID[PITCH]) + (servoDirection(4, 2) * axisPID[ROLL]);
            }
            servo[3] += servoMiddle(3);
            servo[4] += servoMiddle(4);
            break;

        case MULTITYPE_DUALCOPTER:
            for (i = 4; i < 6; i++ ) {
                servo[i] = axisPID[5 - i] * servoDirection(i, 1); // mix and setup direction
                servo[i] += servoMiddle(i);
            }
            break;

        case MULTITYPE_SINGLECOPTER:
            for (i = 3; i < 7; i++) {
                servo[i] = (axisPID[YAW] * servoDirection(i, 2)) + (axisPID[(6 - i) >> 1] * servoDirection(i, 1)); // mix and setup direction
                servo[i] += servoMiddle(i);
            }
            motor[0] = rcCommand[THROTTLE];
            break;
    }

    // do camstab
    if (feature(FEATURE_SERVO_TILT)) {
        // center at fixed position, or vary either pitch or roll by RC channel
        servo[0] = servoMiddle(0);
        servo[1] = servoMiddle(1);

        if (rcOptions[BOXCAMSTAB]) {
            if (cfg.gimbal_flags & GIMBAL_MIXTILT) {
                servo[0] -= (-(int32_t)cfg.servoConf[0].rate) * angle[PITCH] / 50 - (int32_t)cfg.servoConf[1].rate * angle[ROLL] / 50;
                servo[1] += (-(int32_t)cfg.servoConf[0].rate) * angle[PITCH] / 50 + (int32_t)cfg.servoConf[1].rate * angle[ROLL] / 50;
            } else {
                servo[0] += (int32_t)cfg.servoConf[0].rate * angle[PITCH] / 50;
                servo[1] += (int32_t)cfg.servoConf[1].rate * angle[ROLL]  / 50;
            }
        }
    }

    // constrain servos
    for (i = 0; i < MAX_SERVOS; i++)
        servo[i] = constrain(servo[i], cfg.servoConf[i].min, cfg.servoConf[i].max); // limit the values

    // forward AUX1-4 to servo outputs (not constrained)
    if (cfg.gimbal_flags & GIMBAL_FORWARDAUX) {
        int offset = 0;
        // offset servos based off number already used in mixer types
        // airplane and servo_tilt together can't be used
        if (mcfg.mixerConfiguration == MULTITYPE_AIRPLANE || mcfg.mixerConfiguration == MULTITYPE_FLYING_WING)
            offset = 4;
        else if (mixers[mcfg.mixerConfiguration].useServo)
            offset = 2;
        for (i = 0; i < 4; i++)
            pwmWriteServo(i + offset, rcData[AUX1 + i]);
    }

    maxMotor = motor[0];
    for (i = 1; i < numberMotor; i++)
        if (motor[i] > maxMotor)
            maxMotor = motor[i];
    for (i = 0; i < numberMotor; i++) {
        if (maxMotor > mcfg.maxthrottle)     // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motor[i] -= maxMotor - mcfg.maxthrottle;
        if (feature(FEATURE_3D)) {
            if ((rcData[THROTTLE]) > 1500) {
                motor[i] = constrain(motor[i], mcfg.deadband3d_high, mcfg.maxthrottle);
            } else {
                motor[i] = constrain(motor[i], mcfg.mincommand, mcfg.deadband3d_low);
            }
        } else {
            motor[i] = constrain(motor[i], mcfg.minthrottle, mcfg.maxthrottle);
            if ((rcData[THROTTLE]) < mcfg.mincheck) {
                if (!feature(FEATURE_MOTOR_STOP))
                    motor[i] = mcfg.minthrottle;
                else
                    motor[i] = mcfg.mincommand;
            }
        }
        if (!f.ARMED) {
            motor[i] = motor_disarmed[i];
        }
    }
}
