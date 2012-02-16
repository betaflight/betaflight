/*******************************/
/****CONFIGURABLE PARAMETERS****/
/*******************************/

/* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
   This is the minimum value that allow motors to run at a idle speed  */
//#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
//#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
//#define MINTHROTTLE 1220
#define MINTHROTTLE 1150 

/* The type of multicopter */
//#define GIMBAL
//#define BI
//#define TRI
//#define QUADP
#define QUADX
//#define Y4
//#define Y6
//#define HEX6
//#define HEX6X
//#define OCTOX8
//#define OCTOFLATP
//#define OCTOFLATX
//#define FLYING_WING //experimental

#define YAW_DIRECTION 1 // if you want to reverse the yaw correction direction
//#define YAW_DIRECTION -1

#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
//#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

//enable internal I2C pull ups
#define INTERNAL_I2C_PULLUPS


//****** advanced users settings   *************
/* I2C DFRobot LED RING communication */
//#define LED_RING

/* This option should be uncommented if ACC Z is accurate enough when motors are running*/
/* should now be ok with BMA020 and BMA180 ACC */
#define TRUSTED_ACCZ

/* This will activate the ACC-Inflight calibration if unchecked */  
//#define InflightAccCalibration

/* PIN A0 and A1 instead of PIN D5 & D6 for 6 motors config and promini config
   This mod allow the use of a standard receiver on a pro mini
   (no need to use a PPM sum receiver)
*/
//#define A0_A1_PIN_HEX

/* possibility to use PIN8 or PIN12 as the AUX2 RC input
   it deactivates in this case the POWER PIN (pin 12) or the BUZZER PIN (pin 8)
*/
//#define RCAUXPIN8
//#define RCAUXPIN12

/* This option is here if you want to use the old level code from the verison 1.7
   It's just to have some feedback. This will be removed in the future */
//#define STAB_OLD_17

/* GPS
   only available on MEGA boards (this might be possible on 328 based boards in the future)
   if enabled, define here the Arduino Serial port number and the UART speed
   note: only the RX PIN is used, the GPS is not configured by multiwii
   the GPS must be configured to output NMEA sentences (which is generally the default conf for most GPS devices)
*/
//#define GPS
#define GPS_SERIAL 2 // should be 2 for flyduino v2. It's the serial port number on arduino MEGA
#define GPS_BAUD   115200
//#define GPS_BAUD   9600

/* Pseudo-derivative conrtroller for level mode (experimental)
   Additional information: http://www.multiwii.com/forum/viewtopic.php?f=8&t=503 */
//#define LEVEL_PDF

/* introduce a deadband around the stick center
   Must be greater than zero, comment if you dont want a deadband on roll, pitch and yaw */
//#define DEADBAND 6

/* if you use a specific sensor board:
   please submit any correction to this list.
     Note from Alex: I only own some boards
                     for other boards, I'm not sure, the info was gathered via rc forums, be cautious */
//#define FFIMUv1         // first 9DOF+baro board from Jussi, with HMC5843                   <- confirmed by Alex
//#define FFIMUv2         // second version of 9DOF+baro board from Jussi, with HMC5883       <- confirmed by Alex
//#define FREEIMUv1       // v0.1 & v0.2 & v0.3 version of 9DOF board from Fabio
//#define FREEIMUv03      // FreeIMU v0.3 and v0.3.1
//#define FREEIMUv035     // FreeIMU v0.3.5 no baro
//#define FREEIMUv035_MS  // FreeIMU v0.3.5_MS                                                <- confirmed by Alex
//#define FREEIMUv035_BMP // FreeIMU v0.3.5_BMP
//#define FREEIMUv04      // FreeIMU v0.4 with MPU6050, HMC5883L, MS561101BA                  <- confirmed by Alex
//#define PIPO            // 9DOF board from erazz
//#define QUADRINO        // full FC board 9DOF+baro board from witespy  with BMP085 baro     <- confirmed by Alex
//#define QUADRINO_ZOOM   // full FC board 9DOF+baro board from witespy  second edition       <- confirmed by Alex
//#define ALLINONE        // full FC board or standalone 9DOF+baro board from CSG_EU
//#define AEROQUADSHIELDv2
//#define ATAVRSBIN1      // Atmel 9DOF (Contribution by EOSBandi). requires 3.3V power.
//#define SIRIUS          // Sirius Navigator IMU                                             <- confirmed by Alex
//#define SIRIUS600       // Sirius Navigator IMU  using the WMP for the gyro
//#define MINIWII         // Jussi's MiniWii Flight Controller
//#define CITRUSv1_0      // CITRUSv1 from qcrc.ca
//#define DROTEK_IMU10DOF


//if you use independent sensors
//leave it commented it you already checked a specific board above
/* I2C gyroscope */
//#define ITG3200
//#define L3G4200D

/* I2C accelerometer */
//#define ADXL345
//#define BMA020
//#define BMA180
//#define NUNCHACK  // if you want to use the nunckuk as a standalone I2C ACC without WMP
//#define LIS3LV02
//#define LSM303DLx_ACC

/* I2C barometer */
//#define BMP085
//#define MS561101BA

/* I2C magnetometer */
//#define HMC5843
//#define HMC5883
//#define AK8975

/* ADC accelerometer */ // for 5DOF from sparkfun, uses analog PIN A1/A2/A3
//#define ADCACC

/* ITG3200 & ITG3205 Low pass filter setting. In case you cannot eliminate all vibrations to the Gyro, you can try
   to decrease the LPF frequency, only one step per try. As soon as twitching gone, stick with that setting.
   It will not help on feedback wobbles, so change only when copter is randomly twiching and all dampening and
   balancing options ran out. Uncomment only one option!
   IMPORTANT! Change low pass filter setting changes PID behaviour, so retune your PID's after changing LPF.*/
//#define ITG3200_LPF_256HZ     // This is the default setting, no need to uncomment, just for reference
//#define ITG3200_LPF_188HZ
//#define ITG3200_LPF_98HZ
//#define ITG3200_LPF_42HZ
//#define ITG3200_LPF_20HZ
//#define ITG3200_LPF_10HZ      // Use this only in extreme cases, rather change motors and/or props

/* MPU6050 Low pass filter setting. In case you cannot eliminate all vibrations to the Gyro, you can try
   to decrease the LPF frequency, only one step per try. As soon as twitching gone, stick with that setting.
   It will not help on feedback wobbles, so change only when copter is randomly twiching and all dampening and
   balancing options ran out. Uncomment only one option!
   IMPORTANT! Change low pass filter setting changes PID behaviour, so retune your PID's after changing LPF.*/
//#define MPU6050_LPF_256HZ     // This is the default setting, no need to uncomment, just for reference
//#define MPU6050_LPF_188HZ
//#define MPU6050_LPF_98HZ
//#define MPU6050_LPF_42HZ
//#define MPU6050_LPF_20HZ
//#define MPU6050_LPF_10HZ      // Use this only in extreme cases, rather change motors and/or props

/* The following lines apply only for specific receiver with only one PPM sum signal, on digital PIN 2
   IF YOUR RECEIVER IS NOT CONCERNED, DON'T UNCOMMENT ANYTHING. Note this is mandatory for a Y6 setup on a promini
   Select the right line depending on your radio brand. Feel free to modify the order in your PPM order is different */
//#define SERIAL_SUM_PPM         PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4 //For Graupner/Spektrum
//#define SERIAL_SUM_PPM         ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4 //For Robe/Hitec/Futaba
//#define SERIAL_SUM_PPM         PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4 //For some Hitec/Sanwa/Others

/* The following lines apply only for Spektrum Satellite Receiver
   Spektrum Satellites are 3V devices.  DO NOT connect to 5V!
   For MEGA boards, attach sat grey wire to RX1, pin 19. Sat black wire to ground. Sat orange wire to Mega board's 3.3V (or any other 3V to 3.3V source).
   For PROMINI, attach sat grey to RX0.  Attach sat black to ground.  
     There is no 3.3V source on a pro mini; you can either use a different 3V source, or attach orange to 5V with a 3V regulator in-line (such as http://search.digikey.com/scripts/DkSearch/dksus.dll?Detail&name=MCP1700-3002E/TO-ND)
     If you use an inline-regulator, a standard 3-pin servo connector can connect to ground, +5V, and RX0; solder the correct wires (and the 3V regulator!) to a Spektrum baseRX-to-Sat cable that has been cut in half. 
     NOTE: Because there is only one serial port on the Pro Mini, using a Spektrum Satellite implies you CANNOT use the PC based configuration tool. Further, you cannot use on-aircraft serial LCD as the baud rates are incompatible. You can configure by one of two methods:
       1) Use an on-aircraft i2c LCD (such as Eagle Tree or LCD03) for setting gains, reading sensors, etc. 
       2) Available now: Comment out the Spektrum definition, upload, plug in PC, configure; uncomment the Spektrum definition, upload, plug in RX, and fly.  Repeat as required to configure. 
   (Contribution by Danal) */
//#define SPEKTRUM 1024
//#define SPEKTRUM 2048


/* EXPERIMENTAL !!
   contribution from Captain IxI and Zaggo
   cf http://www.multiwii.com/forum/viewtopic.php?f=7&t=289
   The following line apply only for Futaba S-Bus Receiver on MEGA boards at RX1 only (Serial 1).
   You have to invert the S-Bus-Serial Signal e.g. with a Hex-Inverter like IC SN74 LS 04 */
//#define SBUS

/* Failsave settings - added by MIS
   Failsafe check pulse on THROTTLE channel. If the pulse is OFF (on only THROTTLE or on all channels) the failsafe procedure is initiated.
   After FAILSAVE_DELAY time of pulse absence, the level mode is on (if ACC or nunchuk is avaliable), PITCH, ROLL and YAW is centered
   and THROTTLE is set to FAILSAVE_THR0TTLE value. You must set this value to descending about 1m/s or so for best results. 
   This value is depended from your configuration, AUW and some other params. 
   Next, afrer FAILSAVE_OFF_DELAY the copter is disarmed, and motors is stopped.
   If RC pulse coming back before reached FAILSAVE_OFF_DELAY time, after the small quard time the RC control is returned to normal.
   If you use serial sum PPM, the sum converter must completly turn off the PPM SUM pusles for this FailSafe functionality.*/
#define FAILSAFE                                  // Alex: comment this line if you want to deactivate the failsafe function
#define FAILSAVE_DELAY     10                     // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
#define FAILSAVE_OFF_DELAY 200                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
#define FAILSAVE_THR0TTLE  (MINTHROTTLE + 200)    // Throttle level used for landing - may be relative to MINTHROTTLE - as in this case

/* EXPERIMENTAL !!
  contribution from Luis Correia
  see http://www.multiwii.com/forum/viewtopic.php?f=18&t=828
  It uses a Bluetooth Serial module as the input for controlling the device via an Android application
  As with the SPEKTRUM option, is not possible to use the configuration tool on a mini or promini. */
//#define BTSERIAL

/* The following lines apply only for a pitch/roll tilt stabilization system
   On promini board, it is not compatible with config with 6 motors or more
   Uncomment the first line to activate it */
//#define SERVO_TILT
#define TILT_PITCH_MIN    1020    //servo travel min, don't set it below 1020
#define TILT_PITCH_MAX    2000    //servo travel max, max value=2000
#define TILT_PITCH_MIDDLE 1500    //servo neutral value
#define TILT_PITCH_PROP   10      //servo proportional (tied to angle) ; can be negative to invert movement
#define TILT_ROLL_MIN     1020
#define TILT_ROLL_MAX     2000
#define TILT_ROLL_MIDDLE  1500
#define TILT_ROLL_PROP    10

/* interleaving delay in micro seconds between 2 readings WMP/NK in a WMP+NK config
   if the ACC calibration time is very long (20 or 30s), try to increase this delay up to 4000
   it is relevent only for a conf with NK */
#define INTERLEAVING_DELAY 3000

/* for V BAT monitoring
   after the resistor divisor we should get [0V;5V]->[0;1023] on analog V_BATPIN
   with R1=33k and R2=51k
   vbat = [0;1023]*16/VBATSCALE */
#define VBAT              // comment this line to suppress the vbat code
#define VBATSCALE     131 // change this value if readed Battery voltage is different than real voltage
#define VBATLEVEL1_3S 107 // 10,7V
#define VBATLEVEL2_3S 103 // 10,3V
#define VBATLEVEL3_3S 99  // 9.9V
#define NO_VBAT       16 // Avoid beeping without any battery

/* when there is an error on I2C bus, we neutralize the values during a short time. expressed in microseconds
   it is relevent only for a conf with at least a WMP */
#define NEUTRALIZE_DELAY 100000

/* this is the value for the ESCs when they are not armed
   in some cases, this value must be lowered down to 900 for some specific ESCs */
#define MINCOMMAND 1000

/* this is the maximum value for the ESCs at full power
   this value can be increased up to 2000 */
#define MAXTHROTTLE 1850

/* This is the speed of the serial interface. 115200 kbit/s is the best option for a USB connection.*/
#define SERIAL_COM_SPEED 115200

/* In order to save space, it's possibile to desactivate the LCD configuration functions
   comment this line only if you don't plan to used a LCD */
#define LCD_CONF
/* to include setting the aux switches for AUX1 and AUX2 via LCD */
//#define LCD_CONF_AUX_12
/* to include setting the aux switches for AUX1, AUX2, AUX3 and AUX4 via LCD */
//#define LCD_CONF_AUX_1234
/* Use this to trigger LCD configuration without a TX - only for debugging - do NOT fly with this activated */
//#define LCD_CONF_DEBUG

/* choice of LCD attached for configuration and telemetry, see notes below */
#define LCD_SERIAL3W    // Alex' initial variant with 3 wires, using rx-pin for transmission @9600 baud fixed
/* serial (wired or wireless via BT etc.) */
//#define LCD_TEXTSTAR    // Cat's Whisker LCD_TEXTSTAR Module CW-LCD-02 (Which has 4 input keys for selecting menus)
//#define LCD_VT100		  // vt100 compatible terminal emulation (blueterm, putty, etc.)
/* i2c devices */
//#define LCD_ETPP        // Eagle Tree Power Panel LCD, which is i2c (not serial)
//#define LCD_LCD03       // LCD03, which is i2c

/* keys to navigate the LCD menu (preset to LCD_TEXTSTAR key-depress codes)*/
#define LCD_MENU_PREV 'a'
#define LCD_MENU_NEXT 'c'
#define LCD_VALUE_UP 'd'
#define LCD_VALUE_DOWN 'b'

/* To use an LCD03 for configuration:
 http://www.robot-electronics.co.uk/htm/Lcd03tech.htm
 Remove the jumper on its back to set i2c control.
 VCC to +5V VCC (pin1 from top)
 SDA - Pin A4 Mini Pro - Pin 20 Mega (pin2 from top)
 SCL - Pin A5 Mini Pro - Pin 21 Mega (pin3 from top)
 GND to Ground (pin4 from top)
 (by Th0rsten) */

/* To use an Eagle Tree Power Panel LCD for configuration:
 White wire  to Ground
 Red wire    to +5V VCC (or to the WMP power pin, if you prefer to reset everything on the bus when WMP resets)
 Yellow wire to SDA - Pin A4 Mini Pro - Pin 20 Mega
 Brown wire  to SCL - Pin A5 Mini Pro - Pin 21 Mega 
 (Contribution by Danal) */

/* Cat's whisker LCD_TEXTSTAR LCD
   Pleae note this display needs a full 4 wire connection to (+5V, Gnd, RXD, TXD )
   Configure display as follows: 115K baud, and TTL levels for RXD and TXD, terminal mode
   NO rx / tx line reconfiguration, use natural pins */


/* motors will not spin when the throttle command is in low position
   this is an alternative method to stop immediately the motors */
//#define MOTOR_STOP

/* some radios have not a neutral point centered on 1500. can be changed here */
#define MIDRC 1500

/* experimental
   camera trigger function : activated via Rc Options in the GUI, servo output=A2 on promini */
//#define CAMTRIG
#define CAM_SERVO_HIGH 2000  // the position of HIGH state servo
#define CAM_SERVO_LOW 1020   // the position of LOW state servo
#define CAM_TIME_HIGH 1000   // the duration of HIGH state servo expressed in ms
#define CAM_TIME_LOW 1000    // the duration of LOW state servo expressed in ms

/* you can change the tricopter servo travel here */
#define TRI_YAW_CONSTRAINT_MIN 1020
#define TRI_YAW_CONSTRAINT_MAX 2000
#define TRI_YAW_MIDDLE 1500 // tail servo center pos. - use this for initial trim; later trim midpoint via LCD

/* Flying Wing: you can change change servo orientation and servo min/max values here */
/* valid for all flight modes, even passThrough mode */
/* need to setup servo directions here; no need to swap servos amongst channels at rx */ 
#define PITCH_DIRECTION_L 1 // left servo - pitch orientation
#define PITCH_DIRECTION_R -1  // right servo - pitch orientation (opposite sign to PITCH_DIRECTION_L, if servos are mounted in mirrored orientation)
#define ROLL_DIRECTION_L 1 // left servo - roll orientation
#define ROLL_DIRECTION_R 1  // right servo - roll orientation  (same sign as ROLL_DIRECTION_L, if servos are mounted in mirrored orientation)
#define WING_LEFT_MID  1500 // left servo center pos. - use this for initial trim; later trim midpoint via LCD
#define WING_RIGHT_MID 1500 // right servo center pos. - use this for initial trim; later trim midpoint via LCD
#define WING_LEFT_MIN  1020 // limit servo travel range must be inside [1020;2000]
#define WING_LEFT_MAX  2000 // limit servo travel range must be inside [1020;2000]
#define WING_RIGHT_MIN 1020 // limit servo travel range must be inside [1020;2000]
#define WING_RIGHT_MAX 2000 // limit servo travel range must be inside [1020;2000]

/* enable monitoring of the power consumption from battery (think of mAh) */
/* allows to set alarm value in GUI or via LCD */
/* Two options: */
/* 1 - soft: - (good results +-5% for plush and mystery ESCs @ 2S and 3S, not good with SuperSimple ESC */
/*      00. relies on your combo of battery type (Voltage, cpacity), ESC, ESC settings, motors, props and multiwii cycle time */
/*      01. set POWERMETER soft. Uses PLEVELSCALE = 50, PLEVELDIV = PLEVELDIVSOFT = 5000 */
/*      0. output is a value that linearily scales to power (mAh) */
/*      1. get voltage reading right first */
/*      2. start with freshly charged battery */
/*      3. go fly your typical flight (routine and duration) */
/*      4. at end connect to GUI or LCD and read the power value; write it down (example 4711)*/
/*      5. charge battery, write down amount of energy needed (example 722 mAh) */
/*      6. compute alarm value for desired power threshold (example 750 mAh : alarm = 4711 / 722 * 750) */
/*      7. set alarm value in GUI or LCD */
/*      8. enjoy your new battery alarm - possibly repeat steps 2 .. 7 */
/*      9. if you want the numbers to represent your mAh value, you must change PLEVELDIV */
/* 2 - hard: - (uses hardware sensor, after configuration gives reasonable results */
/*      00. uses analog pin 2 to read voltage output from sensor. */
/*      01. set POWERMETER hard. Uses PLEVELSCALE = 50 */
/*      02. install low path filter for 25 Hz to sensor input */
/*      03. check your average cycle time. If not close to 3ms, then you must change PLEVELDIV accordingly */
/*      1. compute PLEVELDIV for your sensor (see below for insturctions) */
/*      2. set PLEVELDIVSOFT to 5000 ( to use LOG_VALUES for individual motor comparison) */
/*      3. attach, set PSENSORNULL and  PINT2mA */
/*      4. configure, compile, upload, set alarm value in GUI or LCD */
/*      3. enjoy true readings of mAh consumed */
/* set POWERMETER to "soft" (1) or "hard" (2) depending on sensor you want to utilize */
//#define POWERMETER_SOFT
//#define POWERMETER_HARD
/* the sum of all powermeters ranges from [0:60000 e4] theoretically. */
/* the alarm level from eeprom is out of [0:255], so we multipy alarm level with PLEVELSCALE and with 1e4 before comparing */
/* PLEVELSCALE is the step size you can use to set alarm */
#define PLEVELSCALE 50 // if you change this value for other granularity, you must search for comments in code to change accordingly 
/* larger PLEVELDIV will get you smaller value for power (mAh equivalent) */
#define PLEVELDIV 5000 // default for soft - if you lower PLEVELDIV, beware of overrun in uint32 pMeter
#define PLEVELDIVSOFT PLEVELDIV // for soft always equal to PLEVELDIV; for hard set to 5000
//#define PLEVELDIV 1361L // to convert the sum into mAh divide by this value
/* amploc 25A sensor has 37mV/A */
/* arduino analog resolution is 4.9mV per unit; units from [0..1023] */
/* sampling rate 20ms, approx 19977 micro seconds */
/* PLEVELDIV = 37 / 4.9  * 10e6 / 18000  * 3600 / 1000  = 1361L */
/* set to analogRead() value for zero current */
#define PSENSORNULL 510 // for I=0A my sensor gives 1/2 Vss; that is approx 2.49Volt
#define PINT2mA 13 // for telemtry display: one integer step on arduino analog translates to mA (example 4.9 / 37 * 100



/* to monitor system values (battery level, loop time etc. with LCD enable this */
/* note: for now you must send single characters 'A', 'B', 'C', 'D' to request 4 different pages */
/* Buttons toggle request for page on/off */
/* The active page on the LCD does get updated automatically */
/* Easy to use with Terminal application or display like LCD - uses the 4 buttons are preconfigured to send 'A', 'B', 'C', 'D' */
//#define LCD_TELEMETRY
/* to enable automatic hopping between a choice of telemetry pages uncomment this. */
/* This may be useful if your LCD has no buttons or the sending is broken */
/* hopping is activated and deactivated in unarmed mode with throttle=low & roll=left & pitch=forward */
/* set it to the sequence of telemetry pages you want to see */
//#define LCD_TELEMETRY_AUTO "12345267" // pages 1 to 7 in ascending order
//#define LCD_TELEMETRY_AUTO  "2122324252627" // strong emphasis on page 2
/* Use this to trigger telemetry without a TX - only for debugging - do NOT fly with this activated */
//#define LCD_TELEMETRY_DEBUG  //This form rolls between all screens, LCD_TELEMETRY_AUTO must also be defined.
//#define LCD_TELEMETRY_DEBUG 6  //This form stays on the screen specified.

/* on telemetry page B it gives a bar graph which shows how much voltage battery has left. Range from 0 to 12 Volt is not very informative */
/* so we try do define a meaningful part. For a 3S battery we define full=12,6V and calculate how much it is above first warning level */
/* Example: 12.6V - VBATLEVEL1_3S  (for me = 126 - 102 = 24) */
#define VBATREF 24 

/* to log values like max loop time and others to come */
/* logging values are visible via LCD config */
/* set to 2, if you want powerconsumption on a per motor basis (this uses the big array and is a memory hog, if POWERMETER <> PM_SOFT) */
//#define LOG_VALUES 1

/* to add debugging code */
/* not needed and not recommended for normal operation */
/* will add extra code that may slow down the main loop or make copter non-flyable */
//#define DEBUG

//****** end of advanced users settings *************

//if you want to change to orientation of individual sensor
//#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  Y; accADC[PITCH]  = -X; accADC[YAW]  = Z;}
//#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -Y; gyroADC[PITCH] =  X; gyroADC[YAW] = Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}

 
/* frequenies for rare cyclic actions in the main loop, depend on cycle time! */
/* time base is main loop cycle time - a value of 6 means to trigger the action every 6th run through the main loop */
/* example: with cycle time of approx 3ms, do action every 6*3ms=18ms */
/* value must be [1; 65535] */
#define LCD_TELEMETRY_FREQ 23       // to send telemetry data over serial 23 <=> 60ms <=> 16Hz (only sending interlaced, so 8Hz update rate)
#define LCD_TELEMETRY_AUTO_FREQ 667 // to step to next telemetry page 667 <=> 2s
#define PSENSORFREQ 6               // to read hardware powermeter sensor 6 <=> 18ms
#define VBATFREQ PSENSORFREQ        // to read battery voltage - keep equal to PSENSORFREQ unless you know what you are doing
/**************************************/
/****END OF CONFIGURABLE PARAMETERS****/
/**************************************/
