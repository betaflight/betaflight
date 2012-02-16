#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define PROMINI
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #define MEGA
#endif

#if defined(PROMINI)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);
  #define LEDPIN_TOGGLE              PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
  #define LEDPIN_OFF                 PORTB &= ~(1<<5);
  #define LEDPIN_ON                  PORTB |= (1<<5);
  #define BUZZERPIN_PINMODE          pinMode (8, OUTPUT);
  #define BUZZERPIN_ON               PORTB |= 1;
  #define BUZZERPIN_OFF              PORTB &= ~1;
  #define POWERPIN_PINMODE           pinMode (12, OUTPUT);
  #define POWERPIN_ON                PORTB |= 1<<4;
  #define POWERPIN_OFF               PORTB &= ~(1<<4); //switch OFF WMP, digital PIN 12
  #define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
  #define PINMODE_LCD                pinMode(0, OUTPUT);
  #define LCDPIN_OFF                 PORTD &= ~1;
  #define LCDPIN_ON                  PORTD |= 1;
  #define STABLEPIN_PINMODE          ;
  #define STABLEPIN_ON               ;
  #define STABLEPIN_OFF              ;
  #define DIGITAL_SERVO_TRI_PINMODE  pinMode(3,OUTPUT); //also right servo for BI COPTER
  #define DIGITAL_SERVO_TRI_HIGH     PORTD |= 1<<3;
  #define DIGITAL_SERVO_TRI_LOW      PORTD &= ~(1<<3);
  #define DIGITAL_TILT_PITCH_PINMODE pinMode(A0,OUTPUT);
  #define DIGITAL_TILT_PITCH_HIGH    PORTC |= 1<<0;
  #define DIGITAL_TILT_PITCH_LOW     PORTC &= ~(1<<0);
  #define DIGITAL_TILT_ROLL_PINMODE  pinMode(A1,OUTPUT);
  #define DIGITAL_TILT_ROLL_HIGH     PORTC |= 1<<1;
  #define DIGITAL_TILT_ROLL_LOW      PORTC &= ~(1<<1);
  #define DIGITAL_BI_LEFT_PINMODE    pinMode(11,OUTPUT); 
  #define DIGITAL_BI_LEFT_HIGH       PORTB |= 1<<3;
  #define DIGITAL_BI_LEFT_LOW        PORTB &= ~(1<<3);
  #define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); //PIN 0
  #define SPEK_SERIAL_VECT           USART_RX_vect
  #define SPEK_DATA_REG              UDR0
//  #define MOTOR_ORDER                9,10,11,3,6,5  //for a quad+: rear,right,left,front
  #define DIGITAL_CAM_PINMODE        pinMode(A2,OUTPUT);
  #define DIGITAL_CAM_HIGH           PORTC |= 1<<2;
  #define DIGITAL_CAM_LOW            PORTC &= ~(1<<2);
  //RX PIN assignment inside the port //for PORTD
  #define THROTTLEPIN                2
  #define ROLLPIN                    4
  #define PITCHPIN                   5
  #define YAWPIN                     6
  #define AUX1PIN                    7
  #define AUX2PIN                    0 // optional PIN 8 or PIN 12
  #define AUX3PIN                    1 // unused 
  #define AUX4PIN                    3 // unused 
  #define ISR_UART                   ISR(USART_UDRE_vect)
  #define V_BATPIN                   A3    // Analog PIN 3
  #define PSENSORPIN                 A2    // Analog PIN 2

  //motor order changes because of possible octo
  #define MOTOR_ORDER       9,10,11,3,6,5,A2,12  //for a quad+: rear,right,left,front
  
  // TILT_PITCH
  #define SERVO_1_PINMODE   pinMode(A0,OUTPUT);
  #define SERVO_1_PIN_HIGH  PORTC |= 1<<0;
  #define SERVO_1_PIN_LOW   PORTC &= ~(1<<0);
  
  // TILT_ROLL
  #define SERVO_2_PINMODE   pinMode(A1,OUTPUT);
  #define SERVO_2_PIN_HIGH  PORTC |= 1<<1;
  #define SERVO_2_PIN_LOW   PORTC &= ~(1<<1);
  
  // CAM TRIG
  #define SERVO_3_PINMODE   pinMode(A2,OUTPUT);
  #define SERVO_3_PIN_HIGH  PORTC |= 1<<2;
  #define SERVO_3_PIN_LOW   PORTC &= ~(1<<2);
  
  // new
  #define SERVO_4_PINMODE   pinMode(12,OUTPUT);
  #define SERVO_4_PIN_HIGH  PORTB |= 1<<4;
  #define SERVO_4_PIN_LOW   PORTB &= ~(1<<4);
  
  // BI LEFT
  #define SERVO_5_PINMODE   pinMode(3,OUTPUT);
  #define SERVO_5_PIN_HIGH  PORTD|= 1<<3;
  #define SERVO_5_PIN_LOW   PORTD &= ~(1<<3);
  
  // TRI REAR
  #define SERVO_6_PINMODE   pinMode(11,OUTPUT);
  #define SERVO_6_PIN_HIGH  PORTB |= 1<<3;
  #define SERVO_6_PIN_LOW   PORTB &= ~(1<<3);
  
  // new motor pin 10
  #define SERVO_7_PINMODE   pinMode(10,OUTPUT);
  #define SERVO_7_PIN_HIGH  PORTB |= 1<<2;
  #define SERVO_7_PIN_LOW   PORTB &= ~(1<<2);
  
  //new motor pin 9
  #define SERVO_8_PINMODE   pinMode(9,OUTPUT);
  #define SERVO_8_PIN_HIGH  PORTB |= 1<<1;
  #define SERVO_8_PIN_LOW   PORTB &= ~(1<<1);

#endif
#if defined(MEGA)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);pinMode (30, OUTPUT);
  #define LEDPIN_TOGGLE              PINB  |= (1<<7); PINC  |= (1<<7);
  #define LEDPIN_ON                  PORTB |= (1<<7); PORTC |= (1<<7);
  #define LEDPIN_OFF                 PORTB &= ~(1<<7);PORTC &= ~(1<<7);
  #define BUZZERPIN_PINMODE          pinMode (32, OUTPUT);
  #define BUZZERPIN_ON               PORTC |= 1<<5;
  #define BUZZERPIN_OFF              PORTC &= ~(1<<5);
  #define POWERPIN_PINMODE           pinMode (37, OUTPUT);
  #define POWERPIN_ON                PORTC |= 1<<0;
  #define POWERPIN_OFF               PORTC &= ~(1<<0);
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;       // PIN 20&21 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
  #define PINMODE_LCD                pinMode(0, OUTPUT);
  #define LCDPIN_OFF                 PORTE &= ~1;      //switch OFF digital PIN 0
  #define LCDPIN_ON                  PORTE |= 1;       //switch OFF digital PIN 0
  #define STABLEPIN_PINMODE          pinMode (31, OUTPUT);
  #define STABLEPIN_ON               PORTC |= 1<<6;
  #define STABLEPIN_OFF              PORTC &= ~(1<<6);
  #define DIGITAL_SERVO_TRI_PINMODE  pinMode(2,OUTPUT); //PIN 2 //also right servo for BI COPTER
  #define DIGITAL_SERVO_TRI_HIGH     PORTE |= 1<<4;
  #define DIGITAL_SERVO_TRI_LOW      PORTE &= ~(1<<4);
  #define DIGITAL_TILT_PITCH_PINMODE pinMode(34,OUTPUT);pinMode(44,OUTPUT); // 34 + 44
  #define DIGITAL_TILT_PITCH_HIGH    PORTC |= 1<<3;PORTL |= 1<<5;
  #define DIGITAL_TILT_PITCH_LOW     PORTC &= ~(1<<3);PORTL &= ~(1<<5);
  #define DIGITAL_TILT_ROLL_PINMODE  pinMode(35,OUTPUT);pinMode(45,OUTPUT); // 35 + 45
  #define DIGITAL_TILT_ROLL_HIGH     PORTC |= 1<<2;PORTL |= 1<<4;
  #define DIGITAL_TILT_ROLL_LOW      PORTC &= ~(1<<2);PORTL &= ~(1<<4);
  #define DIGITAL_BI_LEFT_PINMODE    pinMode(6,OUTPUT); 
  #define DIGITAL_BI_LEFT_HIGH       PORTH |= 1<<3;
  #define DIGITAL_BI_LEFT_LOW        PORTH &= ~(1<<3);
  #define PPM_PIN_INTERRUPT          attachInterrupt(4, rxInt, RISING);  //PIN 19, also used for Spektrum satellite option
  #define SPEK_SERIAL_VECT           USART1_RX_vect
  #define SPEK_DATA_REG              UDR1
  #define DIGITAL_CAM_PINMODE        pinMode(33,OUTPUT); pinMode(46,OUTPUT); // 33 + 46
  #define DIGITAL_CAM_HIGH           PORTC |= 1<<4;PORTL |= 1<<3;
  #define DIGITAL_CAM_LOW            PORTC &= ~(1<<4);PORTL &= ~(1<<3);
  //RX PIN assignment inside the port //for PORTK
  #define THROTTLEPIN                0  //PIN 62 =  PIN A8
  #define ROLLPIN                    1  //PIN 63 =  PIN A9
  #define PITCHPIN                   2  //PIN 64 =  PIN A10
  #define YAWPIN                     3  //PIN 65 =  PIN A11
  #define AUX1PIN                    4  //PIN 66 =  PIN A12
  #define AUX2PIN                    5  //PIN 67 =  PIN A13
  #define AUX3PIN                    6  //PIN 68 =  PIN A14
  #define AUX4PIN                    7  //PIN 69 =  PIN A15
  #define ISR_UART                   ISR(USART0_UDRE_vect)
  #define V_BATPIN                   A0    // Analog PIN 0
  #define PSENSORPIN                 A2    // Analog PIN 2

  #define MOTOR_ORDER                3,5,6,2,7,8,9,10   //for a quad+: rear,right,left,front   //+ for y6: 7:under right  8:under left
  
  // TILT_PITCH
  #define SERVO_1_PINMODE   pinMode(34,OUTPUT);pinMode(44,OUTPUT);
  #define SERVO_1_PIN_HIGH  PORTC |= 1<<3;PORTL |= 1<<5;
  #define SERVO_1_PIN_LOW   PORTC &= ~(1<<3);PORTL &= ~(1<<5);
  
  // TILT_ROLL
  #define SERVO_2_PINMODE   pinMode(35,OUTPUT);pinMode(45,OUTPUT);
  #define SERVO_2_PIN_HIGH  PORTC |= 1<<2;PORTL |= 1<<4;
  #define SERVO_2_PIN_LOW   PORTC &= ~(1<<2);PORTL &= ~(1<<4);
  
  // CAM TRIG
  #define SERVO_3_PINMODE   pinMode(33,OUTPUT); pinMode(46,OUTPUT);
  #define SERVO_3_PIN_HIGH  PORTC |= 1<<4;PORTL |= 1<<3;
  #define SERVO_3_PIN_LOW   PORTC &= ~(1<<4);PORTL &= ~(1<<3);
  
  // new ?
  #define SERVO_4_PINMODE   pinMode (37, OUTPUT);
  #define SERVO_4_PIN_HIGH  PORTC |= 1<<0;
  #define SERVO_4_PIN_LOW   PORTC &= ~(1<<0);
  
  // BI LEFT
  #define SERVO_5_PINMODE   pinMode(6,OUTPUT);
  #define SERVO_5_PIN_HIGH  PORTH |= 1<<3;
  #define SERVO_5_PIN_LOW   PORTH &= ~(1<<3);
  
  // TRI REAR
  #define SERVO_6_PINMODE   pinMode(2,OUTPUT);
  #define SERVO_6_PIN_HIGH  PORTE |= 1<<4;
  #define SERVO_6_PIN_LOW   PORTE &= ~(1<<4);
  
  //new motor pin 5 
  #define SERVO_7_PINMODE   pinMode(5,OUTPUT);
  #define SERVO_7_PIN_HIGH  PORTE |= 1<<3;
  #define SERVO_7_PIN_LOW   PORTE &= ~(1<<3);
  
  //new motor pin 3 
  #define SERVO_8_PINMODE   pinMode(3,OUTPUT);
  #define SERVO_8_PIN_HIGH  PORTE |= 1<<5;
  #define SERVO_8_PIN_LOW   PORTE &= ~(1<<5);
#endif


//please submit any correction to this list.
#if defined(FFIMUv1)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5843
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(FFIMUv2)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(FREEIMUv1)
  #define ITG3200
  #define ADXL345
  #define HMC5843
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  -Y; accADC[PITCH]  = X; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X;  gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X;  magADC[PITCH]  = Y; magADC[YAW]  = Z;}
  #define ADXL345_ADDRESS 0xA6
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(FREEIMUv03)
  #define ITG3200
  #define ADXL345 // this is actually an ADXL346 but that's just the same as ADXL345
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  -Y; accADC[PITCH]  = X; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X;  gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -Y;  magADC[PITCH]  = X; magADC[YAW]  = Z;} 
  #define ADXL345_ADDRESS 0xA6
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(FREEIMUv035) || defined(FREEIMUv035_MS) || defined(FREEIMUv035_BMP)
  #define ITG3200
  #define BMA180
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X;  gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -Y;  magADC[PITCH]  = X; magADC[YAW]  = Z;} 
  #undef INTERNAL_I2C_PULLUPS
  #if defined(FREEIMUv035_MS)
    #define MS561101BA
  #elif defined(FREEIMUv035_BMP)
    #define BMP085
  #endif
#endif

#if defined(FREEIMUv04)
  #define MPU6050
  #define HMC5883
  #define MS561101BA
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X;  gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -Y;  magADC[PITCH]  = X; magADC[YAW]  = Z;}
  #define MPU6050_EN_I2C_BYPASS // MAG connected to the AUX I2C bus of MPU6050
  #undef INTERNAL_I2C_PULLUPS
#endif

#if defined(PIPO)
  #define L3G4200D
  #define ADXL345
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  = X; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
  #define ADXL345_ADDRESS 0xA6
#endif

#if defined(QUADRINO)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = X; magADC[YAW]  = Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(QUADRINO_ZOOM)
  #define ITG3200
  #define BMA180
  #define BMP085  // note, can be also #define MS561101BA  on some versions
  //#define MS561101BA
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = X; magADC[YAW]  = Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
  #define STABLEPIN_PINMODE pinMode (A2, OUTPUT);
  #define STABLEPIN_ON PORTC |= (1<<2);
  #define STABLEPIN_OFF PORTC &= ~(1<<2);
#endif

#if defined(ALLINONE)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = X; magADC[YAW]  = Z;}
#endif

#if defined(AEROQUADSHIELDv2) // to confirm
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5843
  #define ACC_ORIENTATION(X, Y, Z) {accADC[ROLL] = -Y; accADC[PITCH] = X; accADC[YAW] = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -Y; gyroADC[PITCH] = X; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z) {magADC[ROLL] = -Y; magADC[PITCH] = X; magADC[YAW] = Z;}
  #define ITG3200_ADDRESS 0XD2
#endif

#if defined(ATAVRSBIN1)
  #define ITG3200
  #define BMA020        //Actually it's a BMA150, but this is a drop in replacement for the discountinued BMA020
  #define AK8975
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  Y; accADC[PITCH]  = -X; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -Y; gyroADC[PITCH] =  X; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -X; magADC[PITCH]  = -Y; magADC[YAW]  = -Z;}
#endif

#if defined(SIRIUS)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y;  magADC[PITCH] = X; magADC[YAW]  = Z;}
  #define BMA180_ADDRESS 0x80
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(SIRIUS600)
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y;  magADC[PITCH] = X; magADC[YAW]  = Z;}
  #define BMA180_ADDRESS 0x80
#endif

#if defined(MINIWII)
  #define ITG3200
  #define BMA180
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = -Y; accADC[YAW]  = -Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -X; gyroADC[PITCH] =  Y; gyroADC[YAW] =  Z;}
#endif

#if defined(CITRUSv1_0)
  #define ITG3200
  #define ADXL345
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(Y, X, Z)  {accADC[ROLL]  =  -X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(Y, X, Z)  {magADC[ROLL]  = Y;  magADC[PITCH] = X; magADC[YAW]  = Z;}
  #define ADXL345_ADDRESS  0xA6
  #define ITG3200_ADDRESS 0XD0
#endif

#if defined(DROTEK_IMU10DOF)
  #define ITG3200
  #define BMA180
  #define BMP085
  #define HMC5883
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = X; magADC[YAW]  = Z;}
  #define ITG3200_ADDRESS 0XD2
#endif

#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK) || defined(ADCACC) || defined(LSM303DLx_ACC) || defined(MPU6050)
  #define ACC 1
#else
  #define ACC 0
#endif

#if defined(HMC5883) || defined(HMC5843) || defined(AK8975)
  #define MAG 1
#else
  #define MAG 0
#endif

#if defined(ITG3200) || defined(L3G4200D) || defined(MPU6050)
  #define GYRO 1
#else
  #define GYRO 0
#endif

#if defined(BMP085) || defined(MS561101BA)
  #define BARO 1
#else
  #define BARO 0
#endif

#if defined(GPS)
  #define GPSPRESENT 1
#else
  #define GPSPRESENT 0
#endif


#if defined(RCAUXPIN8)
  #define BUZZERPIN_PINMODE          ;
  #define BUZZERPIN_ON               ;
  #define BUZZERPIN_OFF              ;
  #define RCAUXPIN
#endif
#if defined(RCAUXPIN12)
  #define POWERPIN_PINMODE           ;
  #define POWERPIN_ON                ;
  #define POWERPIN_OFF               ;
  #define RCAUXPIN
#endif


#if defined(TRI)
  #define MULTITYPE 1
#elif defined(QUADP)
  #define MULTITYPE 2
#elif defined(QUADX)
  #define MULTITYPE 3
#elif defined(BI)
  #define MULTITYPE 4
#elif defined(GIMBAL)
  #define MULTITYPE 5
#elif defined(Y6)
  #define MULTITYPE 6
#elif defined(HEX6)
  #define MULTITYPE 7
#elif defined(FLYING_WING)
  #define MULTITYPE 8
#elif defined(Y4)
  #define MULTITYPE 9
#elif defined(HEX6X)
  #define MULTITYPE 10
#elif defined(OCTOX8)
  #define MULTITYPE 11
#elif defined(OCTOFLATP)
  #define MULTITYPE 11      //the GUI is the same for all 8 motor configs
#elif defined(OCTOFLATX)
  #define MULTITYPE 11      //the GUI is the same for all 8 motor configs
#endif

#if defined(POWERMETER_HARD) || defined(POWERMETER_SOFT)
  #define POWERMETER
#endif

/**************************/
/* Error Checking Section */
/**************************/

#if (defined(LCD_CONF) || defined(LCD_TELEMETRY)) && !(defined(LCD_SERIAL3W) || defined(LCD_TEXTSTAR) || defined(LCD_VT100) || defined(LCD_ETPP) || defined(LCD_LCD03))
  #error "LCD_CONF or LCD_TELEMETRY defined, and choice of LCD not defined.  Uncomment one of LCD_SERIAL3W or LCD_TEXTSTAR or LCD_VT100 or LCD_ETPP or LCD_LCD03"
#endif


#if defined(POWERMETER) && !(defined(VBAT))
  	#error "to use powermeter, you must also define and configure VBAT"
#endif

#if defined(LCD_TELEMETRY_AUTO) && !(defined(LCD_TELEMETRY))
 	#error "to use automatic telemetry, you MUST also define and configure LCD_TELEMETRY"
#endif



