
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
#define PRI_SERVO_FROM   5      // use only servo 5
#define PRI_SERVO_TO     5
#elif defined(QUADP) || defined(QUADX) || defined(Y4)
#define NUMBER_MOTOR 4
#elif defined(Y6) || defined(HEX6) || defined(HEX6X)
#define NUMBER_MOTOR 6
#elif defined(OCTOX8) || defined(OCTOFLATP) || defined(OCTOFLATX)
#define NUMBER_MOTOR 8
#endif

#if defined(SERVO_TILT) && defined(CAMTRIG)
#define SEC_SERVO_FROM   1      // use servo from 1 to 3
#define SEC_SERVO_TO     3
#else
#if defined(SERVO_TILT)
    // if A0 and A1 is taken by motors, we can use A2 and 12 for Servo tilt
#if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR == 6) && defined(PROMINI)
#define SEC_SERVO_FROM   3      // use servo from 3 to 4
#define SEC_SERVO_TO     4
#else
#define SEC_SERVO_FROM   1      // use servo from 1 to 2
#define SEC_SERVO_TO     2
#endif
#endif
#if defined(CAMTRIG)
#define SEC_SERVO_FROM   3      // use servo 3
#define SEC_SERVO_TO     3
#endif
#endif


uint8_t PWM_PIN[8] = { MOTOR_ORDER };
// so we need a servo pin array
volatile uint8_t atomicServo[8] = { 125, 125, 125, 125, 125, 125, 125, 125 };

//for HEX Y6 and HEX6/HEX6X flat and for promini
volatile uint8_t atomicPWM_PIN5_lowState;
volatile uint8_t atomicPWM_PIN5_highState;
volatile uint8_t atomicPWM_PIN6_lowState;
volatile uint8_t atomicPWM_PIN6_highState;
//for OCTO on promini
volatile uint8_t atomicPWM_PINA2_lowState;
volatile uint8_t atomicPWM_PINA2_highState;
volatile uint8_t atomicPWM_PIN12_lowState;
volatile uint8_t atomicPWM_PIN12_highState;

void writeServos()
{
#if defined(SERVO)
    // write primary servos
#if defined(PRI_SERVO_FROM)
    for (uint8_t i = (PRI_SERVO_FROM - 1); i < PRI_SERVO_TO; i++) {
        atomicServo[i] = (servo[i] - 1000) >> 2;
    }
#endif
    // write secundary servos
#if defined(SEC_SERVO_FROM)
    for (uint8_t i = (SEC_SERVO_FROM - 1); i < SEC_SERVO_TO; i++) {
        atomicServo[i] = (servo[i] - 1000) >> 2;
    }
#endif
#endif
}

void writeMotors()
{                               // [1000;2000] => [125;250]
#if defined(MEGA)
#if (NUMBER_MOTOR > 0)
    OCR3C = motor[0] >> 3;      //  pin 3
#endif
#if (NUMBER_MOTOR > 1)
    OCR3A = motor[1] >> 3;      //  pin 5
#endif
#if (NUMBER_MOTOR > 2)
    OCR4A = motor[2] >> 3;      //  pin 6
#endif
#if (NUMBER_MOTOR > 3)
    OCR3B = motor[3] >> 3;      //  pin 2
#endif
#if (NUMBER_MOTOR > 4)
    OCR4B = motor[4] >> 3;      //  pin 7
    OCR4C = motor[5] >> 3;      //  pin 8
#endif
#if (NUMBER_MOTOR > 6)
    OCR2B = motor[6] >> 3;      //  pin 9
    OCR2A = motor[7] >> 3;      //  pin 10
#endif
#endif
#if defined(PROMINI)
#if (NUMBER_MOTOR > 0)
#ifndef EXT_MOTOR_RANGE
    OCR1A = motor[0] >> 3;      //  pin 9
#else
    OCR1A = ((motor[0] >> 2) - 250) + 2)
#endif
#endif
#if (NUMBER_MOTOR > 1)
#ifndef EXT_MOTOR_RANGE
        OCR1B = motor[1] >> 3;  //  pin 10
#else
        OCR1B = ((motor[1] >> 2) - 250) + 2)
#endif
#endif
#if (NUMBER_MOTOR > 2)
#ifndef EXT_MOTOR_RANGE
        OCR2A = motor[2] >> 3;  //  pin 11
#else
        OCR2A = ((motor[2] >> 2) - 250) + 2)
#endif
#endif
#if (NUMBER_MOTOR > 3)
#ifndef EXT_MOTOR_RANGE
        OCR2B = motor[3] >> 3;  //  pin 3
#else
        OCR2B = ((motor[3] >> 2) - 250) + 2)
#endif
#endif
#if (NUMBER_MOTOR > 4)
        atomicPWM_PIN5_highState = ((motor[5] - 1000) / 4.08) + 5;
    atomicPWM_PIN5_lowState = 255 - atomicPWM_PIN5_highState;
    atomicPWM_PIN6_highState = ((motor[4] - 1000) / 4.08) + 5;
    atomicPWM_PIN6_lowState = 255 - atomicPWM_PIN6_highState;
#endif
#if (NUMBER_MOTOR > 6)
    atomicPWM_PINA2_highState = ((motor[6] - 1000) / 4.08) + 5;
    atomicPWM_PINA2_lowState = 255 - atomicPWM_PINA2_highState;
    atomicPWM_PIN12_highState = ((motor[7] - 1000) / 4.08) + 5;
    atomicPWM_PIN12_lowState = 255 - atomicPWM_PIN12_highState;
#endif
#endif
}

void writeAllMotors(int16_t mc) {       // Sends commands to all motors
    for (uint8_t i = 0; i < NUMBER_MOTOR; i++)
        motor[i] = mc;
    writeMotors();
}

void initOutput() {
#if defined(MEGA)
    for (uint8_t i = 0; i < NUMBER_MOTOR; i++)
        pinMode(PWM_PIN[i], OUTPUT);
#if (NUMBER_MOTOR > 0)
    TCCR3A |= _BV(COM3C1);      // connect pin 3 to timer 3 channel C
#endif
#if (NUMBER_MOTOR > 1)
    TCCR3A |= _BV(COM3A1);      // connect pin 5 to timer 3 channel A
#endif
#if (NUMBER_MOTOR > 2)
    TCCR4A |= _BV(COM4A1);      // connect pin 6 to timer 4 channel A
#endif
#if (NUMBER_MOTOR > 3)
    TCCR3A |= _BV(COM3B1);      // connect pin 2 to timer 3 channel B
#endif
#if (NUMBER_MOTOR > 4)
    TCCR4A |= _BV(COM4B1);      // connect pin 7 to timer 4 channel B
    TCCR4A |= _BV(COM4C1);      // connect pin 8 to timer 4 channel C
#endif
#if (NUMBER_MOTOR > 6)
    TCCR2A |= _BV(COM2B1);      // connect pin 9 to timer 2 channel B
    TCCR2A |= _BV(COM2A1);      // connect pin 10 to timer 2 channel A
#endif
#endif
#if defined(PROMINI)
    for (uint8_t i = 0; i < min(NUMBER_MOTOR, 4); i++)
        pinMode(PWM_PIN[i], OUTPUT);
#if (NUMBER_MOTOR > 0)
    TCCR1A |= _BV(COM1A1);      // connect pin 9 to timer 1 channel A
#endif
#if (NUMBER_MOTOR > 1)
    TCCR1A |= _BV(COM1B1);      // connect pin 10 to timer 1 channel B
#endif
#if (NUMBER_MOTOR > 2)
    TCCR2A |= _BV(COM2A1);      // connect pin 11 to timer 2 channel A
#endif
#if (NUMBER_MOTOR > 3)
    TCCR2A |= _BV(COM2B1);      // connect pin 3 to timer 2 channel B
#endif
#endif

    writeAllMotors(1000);
    delay(300);
#if defined(SERVO)
    initializeServo();
#endif
#if (NUMBER_MOTOR == 6) && defined(PROMINI)
    initializeSoftPWM();
#if defined(A0_A1_PIN_HEX)
    pinMode(5, INPUT);
    pinMode(6, INPUT);          // we reactivate the INPUT affectation for these two PINs
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);
#endif
#elif (NUMBER_MOTOR == 8) && defined(PROMINI)
    initializeSoftPWM();
#if defined(A0_A1_PIN_HEX)
    pinMode(5, INPUT);
    pinMode(6, INPUT);          // we reactivate the INPUT affectation for these two PINs
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);
#endif
#endif
}

#if defined(SERVO)
void initializeServo() {
#if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
    SERVO_1_PINMODE;
#endif
#if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2)
    SERVO_2_PINMODE;
#endif
#if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3)
    SERVO_3_PINMODE;
#endif
#if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4)
    SERVO_4_PINMODE;
#endif
#if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5)
    SERVO_5_PINMODE;
#endif
#if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6)
    SERVO_6_PINMODE;
#endif
#if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7)
    SERVO_7_PINMODE;
#endif
#if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8)
    SERVO_8_PINMODE;
#endif

    TCCR0A = 0;                 // normal counting mode
    TIMSK0 |= (1 << OCIE0A);    // Enable CTC interrupt

    // timer 0B for hex with servo tilt
#if (NUMBER_MOTOR == 6) && defined(PROMINI)
    TIMSK0 |= (1 << OCIE0B);
#endif
}

// ****servo yaw with a 50Hz refresh rate****
// prescaler is set by default to 64 on Timer0
// Duemilanove : 16MHz / 64 => 4 us
// 256 steps = 1 counter cycle = 1024 us
ISR(TIMER0_COMPA_vect) {
    static uint8_t state = 0;
    static uint8_t count;
    if (state == 0) {
        //http://billgrundmann.wordpress.com/2009/03/03/to-use-or-not-use-writedigital/ 

#if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
        SERVO_1_PIN_HIGH;
#endif
        OCR0A += 250;           // 1000 us
        state++;
    } else if (state == 1) {
        OCR0A += atomicServo[0];        // 1000 + [0-1020] us
        state++;
    } else if (state == 2) {
#if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
        SERVO_1_PIN_LOW;
#endif
#if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2)
        SERVO_2_PIN_HIGH;
#endif
        OCR0A += 250;           // 1000 us
        state++;
    } else if (state == 3) {
        OCR0A += atomicServo[1];        // 1000 + [0-1020] us
        state++;
    } else if (state == 4) {
#if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2)
        SERVO_2_PIN_LOW;
#endif
#if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3)
        SERVO_3_PIN_HIGH;
#endif
        state++;
        OCR0A += 250;           // 1000 us
    } else if (state == 5) {
        OCR0A += atomicServo[2];        // 1000 + [0-1020] us
        state++;
    } else if (state == 6) {
#if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3)
        SERVO_3_PIN_LOW;
#endif
#if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4)
        SERVO_4_PIN_HIGH;
#endif
        state++;
        OCR0A += 250;           // 1000 us
    } else if (state == 7) {
        OCR0A += atomicServo[3];        // 1000 + [0-1020] us
        state++;
    } else if (state == 8) {
#if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4)
        SERVO_4_PIN_LOW;
#endif
#if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5)
        SERVO_5_PIN_HIGH;;
#endif
        state++;
        OCR0A += 250;           // 1000 us
    } else if (state == 9) {
        OCR0A += atomicServo[4];        // 1000 + [0-1020] us
        state++;
    } else if (state == 10) {
#if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5)
        SERVO_5_PIN_LOW;
#endif
#if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6)
        SERVO_6_PIN_HIGH;;
#endif
        state++;
        OCR0A += 250;           // 1000 us
    } else if (state == 11) {
        OCR0A += atomicServo[5];        // 1000 + [0-1020] us
        state++;
    } else if (state == 12) {
#if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6)
        SERVO_6_PIN_LOW;
#endif
#if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7)
        SERVO_7_PIN_HIGH;
#endif
        state++;
        OCR0A += 250;           // 1000 us
    } else if (state == 13) {
        OCR0A += atomicServo[6];        // 1000 + [0-1020] us
        state++;
    } else if (state == 14) {
#if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7)
        SERVO_7_PIN_LOW;
#endif
#if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8)
        SERVO_8_PIN_HIGH;
#endif
        state++;
        OCR0A += 250;           // 1000 us
    } else if (state == 15) {
        OCR0A += atomicServo[7];        // 1000 + [0-1020] us
        state++;
    } else if (state == 16) {
#if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8)
        SERVO_8_PIN_LOW;
#endif
        count = 2;
        state++;
        OCR0A += 250;           // 1000 us
    } else if (state == 17) {
        if (count > 0)
            count--;
        else
            state = 0;
        OCR0A += 250;
    }
}
#endif

#if (NUMBER_MOTOR > 4) && defined(PROMINI)
void initializeSoftPWM() {
    // if there are servos its alredy done
#if (NUMBER_MOTOR == 6) && not defined(SERVO)
    TCCR0A = 0;                 // normal counting mode
    TIMSK0 |= (1 << OCIE0B);    // Enable CTC interrupt  
#endif

#if (NUMBER_MOTOR > 6)
    TCCR0A = 0;                 // normal counting mode
    TIMSK0 |= (1 << OCIE0A);    // Enable CTC interrupt
    TIMSK0 |= (1 << OCIE0B);
#endif
}

  // HEXA with just OCR0B 
ISR(TIMER0_COMPB_vect) {
    static uint8_t state = 0;
    if (state == 0) {
#if not defined(A0_A1_PIN_HEX)
        PORTD |= 1 << 5;        //digital PIN 5 high
#else
        PORTC |= 1 << 0;        //PIN A0
#endif
        OCR0B += atomicPWM_PIN5_highState;
        state = 1;
    } else if (state == 1) {
#if not defined(A0_A1_PIN_HEX)
        PORTD &= ~(1 << 6);
#else
        PORTC &= ~(1 << 1);
#endif
        OCR0B += atomicPWM_PIN6_lowState;
        state = 2;
    } else if (state == 2) {
#if not defined(A0_A1_PIN_HEX)
        PORTD |= 1 << 6;
#else
        PORTC |= 1 << 1;        //PIN A1
#endif
        OCR0B += atomicPWM_PIN6_highState;
        state = 3;
    } else if (state == 3) {
#if not defined(A0_A1_PIN_HEX)
        PORTD &= ~(1 << 5);     //digital PIN 5 low
#else
        PORTC &= ~(1 << 0);
#endif
        OCR0B += atomicPWM_PIN5_lowState;
        state = 0;
    }
}

  //the same with digital PIN A2 & 12 OCR0A counter for OCTO
#if (NUMBER_MOTOR > 6)
ISR(TIMER0_COMPA_vect) {
    static uint8_t state = 0;
    if (state == 0) {
        PORTC |= 1 << 2;        //digital PIN A2 high
        OCR0A += atomicPWM_PINA2_highState;
        state = 1;
    } else if (state == 1) {
        PORTB &= ~(1 << 4);     //digital PIN 12 low
        OCR0A += atomicPWM_PIN12_lowState;
        state = 2;
    } else if (state == 2) {
        PORTB |= 1 << 4;        //digital PIN 12 high
        OCR0A += atomicPWM_PIN12_highState;
        state = 3;
    } else if (state == 3) {
        PORTC &= ~(1 << 2);     //digital PIN A2 low
        OCR0A += atomicPWM_PINA2_lowState;
        state = 0;
    }
}
#endif

#endif

void mixTable() {
    int16_t maxMotor;
    uint8_t i, axis;
    static uint8_t camCycle = 0;
    static uint8_t camState = 0;
    static uint32_t camTime = 0;

#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

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
    servo[4] = constrain(TRI_YAW_MIDDLE + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX);        //REAR
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

#ifdef SERVO_TILT
    if ((rcOptions1 & activate1[BOXCAMSTAB])
        || (rcOptions2 & activate2[BOXCAMSTAB])) {
        servo[0] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] / 16 + rcData[AUX3] - 1500, TILT_PITCH_MIN, TILT_PITCH_MAX);
        servo[1] = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP * angle[ROLL] / 16 + rcData[AUX4] - 1500, TILT_ROLL_MIN, TILT_ROLL_MAX);
    } else {
        // to use it with A0_A1_PIN_HEX
#if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR == 6) && defined(PROMINI)
        servo[2] = constrain(TILT_PITCH_MIDDLE + rcData[AUX3] - 1500, TILT_PITCH_MIN, TILT_PITCH_MAX);
        servo[3] = constrain(TILT_ROLL_MIDDLE + rcData[AUX4] - 1500, TILT_ROLL_MIN, TILT_ROLL_MAX);
#else
        servo[0] = constrain(TILT_PITCH_MIDDLE + rcData[AUX3] - 1500, TILT_PITCH_MIN, TILT_PITCH_MAX);
        servo[1] = constrain(TILT_ROLL_MIDDLE + rcData[AUX4] - 1500, TILT_ROLL_MIN, TILT_ROLL_MAX);
#endif
    }
#endif
#ifdef GIMBAL
    servo[0] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] / 16 + rcCommand[PITCH], TILT_PITCH_MIN, TILT_PITCH_MAX);
    servo[1] = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP * angle[ROLL] / 16 + rcCommand[ROLL], TILT_ROLL_MIN, TILT_ROLL_MAX);
#endif
#ifdef FLYING_WING
    motor[0] = rcCommand[THROTTLE];
    if (passThruMode) {         // use raw stick values to drive output 
        servo[0] = constrain(wing_left_mid + PITCH_DIRECTION_L * (rcData[PITCH] - MIDRC) + ROLL_DIRECTION_L * (rcData[ROLL] - MIDRC), WING_LEFT_MIN, WING_LEFT_MAX);    //LEFT
        servo[1] = constrain(wing_right_mid + PITCH_DIRECTION_R * (rcData[PITCH] - MIDRC) + ROLL_DIRECTION_R * (rcData[ROLL] - MIDRC), WING_RIGHT_MIN, WING_RIGHT_MAX); //RIGHT
    } else {                    // use sensors to correct (gyro only or gyro+acc according to aux1/aux2 configuration
        servo[0] = constrain(wing_left_mid + PITCH_DIRECTION_L * axisPID[PITCH] + ROLL_DIRECTION_L * axisPID[ROLL], WING_LEFT_MIN, WING_LEFT_MAX);      //LEFT
        servo[1] = constrain(wing_right_mid + PITCH_DIRECTION_R * axisPID[PITCH] + ROLL_DIRECTION_R * axisPID[ROLL], WING_RIGHT_MIN, WING_RIGHT_MAX);   //RIGHT
    }
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
    if ((rcOptions1 & activate1[BOXCAMTRIG])
        || (rcOptions1 & activate2[BOXCAMTRIG]))
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
        for (uint8_t i = 0; i < NUMBER_MOTOR; i++) {
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
