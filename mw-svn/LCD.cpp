// ************************************************************************************************************
// LCD & display & monitoring
// ************************************************************************************************************
#if defined(LCD_CONF) || defined(LCD_TELEMETRY)
static char line1[17], line2[17];

#if defined(LCD_ETPP)
  // *********************
  // i2c Eagle Tree Power Panel primitives
  // *********************
void i2c_ETPP_init()
{
    i2c_rep_start(0x76 + 0);    // LCD_ETPP i2c address: 0x3B in 7 bit form. Shift left one bit and concatenate i2c write command bit of zero = 0x76 in 8 bit form.
    i2c_write(0x00);            // LCD_ETPP command register
    i2c_write(0x24);            // Function Set 001D0MSL D : data length for parallel interface only; M: 0 = 1x32 , 1 = 2x16; S: 0 = 1:18 multiplex drive mode, 1x32 or 2x16 character display, 1 = 1:9 multiplex drive mode, 1x16 character display; H: 0 = basic instruction set plus standard instruction set, 1 = basic instruction set plus extended instruction set
    i2c_write(0x0C);            // Display on   00001DCB D : 0 = Display Off, 1 = Display On; C : 0 = Underline Cursor Off, 1 = Underline Cursor On; B : 0 = Blinking Cursor Off, 1 = Blinking Cursor On
    i2c_write(0x06);            // Cursor Move  000001IS I : 0 = DDRAM or CGRAM address decrements by 1, cursor moves to the left, 1 = DDRAM or CGRAM address increments by 1, cursor moves to the right; S : 0 = display does not shift,  1 = display does shifts
    LCDclear();
}

void i2c_ETPP_send_cmd(byte c)
{
    i2c_rep_start(0x76 + 0);    // I2C write direction
    i2c_write(0x00);            // LCD_ETPP command register
    i2c_write(c);
}
void i2c_ETPP_send_char(char c)
{
    if (c > 0x0f)
        c |= 0x80;              // LCD_ETPP uses character set "R", which has A->z mapped same as ascii + high bit; don't mess with custom chars. 
    i2c_rep_start(0x76 + 0);    // I2C write direction
    i2c_write(0x40);            // LCD_ETPP data register
    i2c_write(c);
}

void i2c_ETPP_set_cursor(byte addr)
{
    i2c_ETPP_send_cmd(0x80 | addr);     // High bit is "Set DDRAM" command, remaing bits are addr.  
}

void i2c_ETPP_set_cursor(byte col, byte row)
{
    row = min(row, 1);
    col = min(col, 15);
    byte addr = col + row * 0x40;       // Why 0x40? RAM in this controller has many more bytes than are displayed.  In particular, the start of the second line (line 1 char 0) is 0x40 in DDRAM. The bytes between 0x0F (last char of line 1) and 0x40 are not displayable (unless the display is placed in marquee scroll mode)
    i2c_ETPP_set_cursor(addr);
}

void i2c_ETPP_create_char(byte idx, uint8_t * array)
{
    i2c_ETPP_send_cmd(0x80);    // CGRAM and DDRAM share an address register, but you can't set certain bits with the CGRAM address command.   Use DDRAM address command to be sure high order address bits are zero. 
    i2c_ETPP_send_cmd(0x40 | byte(idx * 8));    // Set CGRAM address 
    i2c_rep_start(0x76 + 0);    // I2C write direction
    i2c_write(0x40);            // LCD_ETPP data register
    for (byte i = 0; i < 8; i++) {
        i2c_write(*array);
        array++;
    }
}

static boolean charsInitialized;        // chars for servo signals are initialized
void ETPP_barGraph(byte num, int val)
{                               // num chars in graph; percent as 1 to 100
    if (!charsInitialized) {
        charsInitialized = true;

        static byte bars[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, };
        static byte less[8] = { 0x00, 0x04, 0x0C, 0x1C, 0x0C, 0x04, 0x00, 0x15, };
        static byte grt[8] = { 0x00, 0x04, 0x06, 0x07, 0x06, 0x04, 0x00, 0x15, };

        byte pattern = 0x10;
        for (int8_t i = 0; i <= 5; i++) {
            for (int8_t j = 0; j < 7; j++) {
                bars[j] = pattern;
            }
            i2c_ETPP_create_char(i, bars);
            pattern >>= 1;
        }
        i2c_ETPP_create_char(6, less);
        i2c_ETPP_create_char(7, grt);
    }

    static char bar[16];
    for (int8_t i = 0; i < num; i++) {
        bar[i] = 5;
    }

    if (val < -100 || val > 100) {
        bar[0] = 6;
        bar[num] = 7;
    }                           // invalid
    else if (val < 0) {
        bar[0] = 6;
    }                           // <...
    else if (val >= 100) {
        bar[3] = 7;
    }                           // ...>
    else {
        bar[val / (100 / num)] = (val % (100 / num)) / 5;
    }                           // ..|.

    for (int8_t i = 0; i < num; i++) {
        i2c_ETPP_send_char(bar[i]);
    }
}
#endif                          //LCD_ETPP

#if defined(LCD_LCD03)          // LCD_LCD03
  // *********************
  // I2C LCD03 primitives
  // *********************
void i2c_LCD03_init()
{
    i2c_rep_start(0xC6);        // The LCD03 is located on the I2C bus at address 0xC6
    i2c_write(0x00);            // Command register
    i2c_write(04);              // Hide cursor
    i2c_write(12);              // Clear screen
    i2c_write(19);              // Backlight on
}

void i2c_LCD03_send_cmd(byte c)
{
    i2c_rep_start(0xC6);
    i2c_write(0x00);
    i2c_write(c);
}
void i2c_LCD03_send_char(char c)
{
    i2c_rep_start(0xC6);
    i2c_write(0x00);
    i2c_write(c);
}

void i2c_LCD03_set_cursor(byte col, byte row)
{
    row = min(row, 1);
    col = min(col, 15);
    i2c_LCD03_send_cmd(03);     // set cursor (row, column)
    i2c_LCD03_send_cmd(row + 1);
    i2c_LCD03_send_cmd(col + 1);
}
#endif                          // LCD_LCD03
/* ------------------------------------------------------------------ */
void LCDprint(uint8_t i)
{
#if defined(LCD_SERIAL3W)
    // 1000000 / 9600  = 104 microseconds at 9600 baud.
    // we set it below to take some margin with the running interrupts
#define BITDELAY 102
    LCDPIN_OFF;
    delayMicroseconds(BITDELAY);
    for (uint8_t mask = 0x01; mask; mask <<= 1) {
        if (i & mask) {
            LCDPIN_ON;
        } else {
            LCDPIN_OFF;
        }                       // choose bit
        delayMicroseconds(BITDELAY);
    }
    LCDPIN_ON                   //switch ON digital PIN 0
        delayMicroseconds(BITDELAY);
#elif defined(LCD_TEXTSTAR) || defined(LCD_VT100)
    SerialWrite(0, i);
#elif defined(LCD_ETPP)
    i2c_ETPP_send_char(i);
#elif defined(LCD_LCD03)
    i2c_LCD03_send_char(i);
#endif
}

void LCDprintChar(const char *s)
{
    while (*s) {
        LCDprint(*s++);
    }
}

void LCDclear()
{
#if defined(LCD_SERIAL3W)
    LCDprint(0xFE);
    LCDprint(0x01);
    delay(10);
    LCDprint(0xFE);
    LCDprint(0x02);
    delay(10);                  // clear screen, cursor line 1, pos 0 for serial LCD Sparkfun - contrib by flyman777
#elif defined(LCD_TEXTSTAR)
    LCDprint(0x0c);
#elif defined(LCD_VT100)
    //LCDprint(0x1b); LCDprint(0x5b); LCDprintChar("2J"); //ED2
    LCDprint(0x1b);
    LCDprint(0x5b);
    LCDprint('K');              //EL0
#elif defined(LCD_ETPP)
    i2c_ETPP_send_cmd(0x01);    // Clear display command, which does NOT clear an Eagle Tree because character set "R" has a '>' at 0x20
    for (byte i = 0; i < 80; i++)
        i2c_ETPP_send_char(' ');        // Blanks for all 80 bytes of RAM in the controller, not just the 2x16 display
#elif defined(LCD_LCD03)
    i2c_LCD03_send_cmd(12);     // clear screen
#endif
}

void LCDsetLine(byte line)
{                               // Line = 1 or 2
#if defined(LCD_SERIAL3W)
    if (line == 1) {
        LCDprint(0xFE);
        LCDprint(128);
    } else {
        LCDprint(0xFE);
        LCDprint(192);
    }
#elif defined(LCD_TEXTSTAR)
    LCDprintChar("\r\n");
    LCDprint(0xfe);
    LCDprint('L');
    LCDprint(line);
#elif defined(LCD_VT100)
    if (line == 1) {
        //LCDprint(0x1b); LCDprint(0x5b); LCDprintChar("f"); //hvhome
        LCDprintChar("\r\n");
        LCDprint(0x1b);
        LCDprint(0x5b);
        LCDprintChar("001;001H");       //pos 1 1
    } else {
        //LCDprint(0x1b); LCDprint('E'); //NEL
        LCDprintChar("\r\n");
        LCDprint(0x1b);
        LCDprint(0x5b);
        LCDprintChar("002;001H");       //pos 2 1
    }
    //LCDprint(0x1b); LCDprint(0x5b); LCDprint('K'); //EL0
#elif defined(LCD_ETPP)
    i2c_ETPP_set_cursor(0, line - 1);
#elif defined(LCD_LCD03)
    i2c_LCD03_set_cursor(0, line - 1);
#endif
}

void initLCD()
{
    blinkLED(20, 30, 1);
#if defined(LCD_SERIAL3W)
    SerialEnd(0);
    //init LCD
    PINMODE_LCD;                //TX PIN for LCD = Arduino RX PIN (more convenient to connect a servo plug on arduino pro mini)
#elif defined(LCD_TEXTSTAR)
    // Cat's Whisker Technologies 'TextStar' Module CW-LCD-02
    // http://cats-whisker.com/resources/documents/cw-lcd-02_datasheet.pdf
    // Modified by Luca Brizzi aka gtrick90 @ RCG
    //LCDprint(0xFE);LCDprint(0x43);LCDprint(0x02); //cursor blink mode 
    LCDprint(0xFE);
    LCDprint('R');              //reset       
#elif defined(LCD_VT100)
    LCDprint(0x1b);
    LCDprint('c');              //RIS
#elif defined(LCD_ETPP)
    // Eagle Tree Power Panel - I2C & Daylight Readable LCD
    // Contributed by Danal
    i2c_ETPP_init();
#elif defined(LCD_LCD03)
    // LCD03 - I2C LCD
    // http://www.robot-electronics.co.uk/htm/Lcd03tech.htm
    // by Th0rsten
    i2c_LCD03_init();
#endif
    LCDclear();
    strcpy_P(line1, PSTR("MultiWii V1.9+"));
    LCDsetLine(1);
    LCDprintChar(line1);
#if defined(LCD_TEXTSTAR) || defined(LCD_VT100)
    delay(2500);
    LCDclear();
#endif
    if (cycleTime == 0) {       //Called from Setup()
        strcpy_P(line1, PSTR("Ready to Fly"));
        LCDsetLine(2);
        LCDprintChar(line1);
    } else {
        strcpy_P(line1, PSTR("Config All Parms"));
        LCDsetLine(2);
        LCDprintChar(line1);
    }
}
#endif                          //Support functions for LCD_CONF and LCD_TELEMETRY


#ifdef LCD_CONF

typedef void (*formatter_func_ptr) (void *, uint8_t, uint8_t);
typedef void (*inc_func_ptr) (void *, int8_t);

typedef struct lcd_type_desc_t {
    formatter_func_ptr fmt;
    inc_func_ptr inc;
};

static lcd_type_desc_t LTU8 = { &__u8Fmt, &__u8Inc };
static lcd_type_desc_t LTU16 = { &__u16Fmt, &__u16Inc };
static lcd_type_desc_t LPMM = { &__upMFmt, &__nullInc };
static lcd_type_desc_t LPMS = { &__upSFmt, &__nullInc };
static lcd_type_desc_t LAUX = { &__uAuxFmt, &__u8Inc };

typedef struct lcd_param_def_t {
    lcd_type_desc_t *type;
    uint8_t decimal;
    uint8_t multiplier;
    uint8_t increment;
};

//typedef struct lcd_param_t{
//  char*  paramText;
//  void * var; 
//  lcd_param_def_t * def;
//};

// ************************************************************************************************************
// LCD Layout definition
// ************************************************************************************************************
// Descriptors
static lcd_param_def_t __P = { &LTU8, 1, 1, 1 };
static lcd_param_def_t __I = { &LTU8, 3, 1, 2 };
static lcd_param_def_t __D = { &LTU8, 0, 1, 1 };
static lcd_param_def_t __RCR = { &LTU8, 2, 2, 2 };
static lcd_param_def_t __RC = { &LTU8, 2, 1, 2 };
static lcd_param_def_t __PM = { &LPMM, 1, 1, 0 };
static lcd_param_def_t __PS = { &LPMS, 1, 1, 0 };
static lcd_param_def_t __PT = { &LTU8, 0, 1, 1 };
static lcd_param_def_t __VB = { &LTU8, 1, 1, 0 };
static lcd_param_def_t __L = { &LTU8, 0, 1, 0 };
static lcd_param_def_t __FS = { &LTU8, 1, 1, 0 };
static lcd_param_def_t __SE = { &LTU16, 0, 1, 10 };
static lcd_param_def_t __AUX = { &LAUX, 0, 1, 1 };

// Program Space Strings - These sit in program flash, not SRAM.
// Rewrite to support PROGMEM strings 21/21/2011 by Danal

PROGMEM prog_char lcd_param_text01[] = "Pitch&Roll P";
PROGMEM prog_char lcd_param_text02[] = "Roll P";
PROGMEM prog_char lcd_param_text03[] = "Roll I";
PROGMEM prog_char lcd_param_text04[] = "Roll D";
PROGMEM prog_char lcd_param_text05[] = "Pitch P";
PROGMEM prog_char lcd_param_text06[] = "Pitch I";
PROGMEM prog_char lcd_param_text07[] = "Pitch D";
PROGMEM prog_char lcd_param_text08[] = "Yaw P";
PROGMEM prog_char lcd_param_text09[] = "Yaw I";
PROGMEM prog_char lcd_param_text10[] = "Yaw D";
#if  BARO
PROGMEM prog_char lcd_param_text11[] = "Alt P";
PROGMEM prog_char lcd_param_text12[] = "Alt I";
PROGMEM prog_char lcd_param_text13[] = "Alt D";
PROGMEM prog_char lcd_param_text14[] = "Vel P";
PROGMEM prog_char lcd_param_text15[] = "Vel I";
PROGMEM prog_char lcd_param_text16[] = "Vel D";
#endif
PROGMEM prog_char lcd_param_text17[] = "Level P";
PROGMEM prog_char lcd_param_text18[] = "Level I";
#if MAG
PROGMEM prog_char lcd_param_text19[] = "Mag P";
#endif
PROGMEM prog_char lcd_param_text20[] = "RC Rate";
PROGMEM prog_char lcd_param_text21[] = "RC Expo";
PROGMEM prog_char lcd_param_text22[] = "Pitch&Roll Rate";
PROGMEM prog_char lcd_param_text23[] = "Yaw Rate";
PROGMEM prog_char lcd_param_text24[] = "Throttle PID";
#ifdef LOG_VALUES
#if (LOG_VALUES == 2)
PROGMEM prog_char lcd_param_text25[] = "pMeter M0";
PROGMEM prog_char lcd_param_text26[] = "pMeter M1";
PROGMEM prog_char lcd_param_text27[] = "pMeter M2";
PROGMEM prog_char lcd_param_text28[] = "pMeter M3";
PROGMEM prog_char lcd_param_text29[] = "pMeter M4";
PROGMEM prog_char lcd_param_text30[] = "pMeter M5";
PROGMEM prog_char lcd_param_text31[] = "pMeter M6";
PROGMEM prog_char lcd_param_text32[] = "pMeter M7";
#endif
#endif
#ifdef POWERMETER
PROGMEM prog_char lcd_param_text33[] = "pMeter Sum";
PROGMEM prog_char lcd_param_text34[] = "pAlarm /50";    // change text to represent PLEVELSCALE value
#endif
#ifdef VBAT
PROGMEM prog_char lcd_param_text35[] = "Batt Volt";
#endif
#ifdef FLYING_WING
PROGMEM prog_char lcd_param_text36[] = "Trim Servo1";
PROGMEM prog_char lcd_param_text37[] = "Trim Servo2";
#endif
#ifdef TRI
PROGMEM prog_char lcd_param_text38[] = "Trim ServoTail";
#endif
#ifdef LOG_VALUES
PROGMEM prog_char lcd_param_text39[] = "Failsafes";
PROGMEM prog_char lcd_param_text40[] = "i2c Errors";
PROGMEM prog_char lcd_param_text41[] = "annex overruns";
#endif
#if defined(LCD_CONF_AUX_12) || defined(LCD_CONF_AUX_1234)
PROGMEM prog_char lcd_param_text42[] = "AUX1/2 level";
PROGMEM prog_char lcd_param_text43[] = "AUX1/2 baro";
PROGMEM prog_char lcd_param_text44[] = "AUX1/2 mag";
PROGMEM prog_char lcd_param_text45[] = "AUX1/2 cam stab";
PROGMEM prog_char lcd_param_text46[] = "AUX1/2 cam trig";
PROGMEM prog_char lcd_param_text47[] = "AUX1/2 arm";
PROGMEM prog_char lcd_param_text48[] = "AUX1/2 gps home";
PROGMEM prog_char lcd_param_text49[] = "AUX1/2 gps hold";
PROGMEM prog_char lcd_param_text50[] = "AUX1/2 passthru";
PROGMEM prog_char lcd_param_text51[] = "AUX1/2 headfree";
PROGMEM prog_char lcd_param_text52[] = "AUX1/2 beeper";
// 53 to 61 reserved
#endif
#if defined(LCD_CONF_AUX_1234)
// aux3/4
PROGMEM prog_char lcd_param_text62[] = "AUX3/4 level";
PROGMEM prog_char lcd_param_text63[] = "AUX3/4 baro";
PROGMEM prog_char lcd_param_text64[] = "AUX3/4 mag";
PROGMEM prog_char lcd_param_text65[] = "AUX3/4 cam stab";
PROGMEM prog_char lcd_param_text66[] = "AUX3/4 cam trig";
PROGMEM prog_char lcd_param_text67[] = "AUX3/4 arm";
PROGMEM prog_char lcd_param_text68[] = "AUX3/4 gps home";
PROGMEM prog_char lcd_param_text69[] = "AUX3/4 gps hold";
PROGMEM prog_char lcd_param_text70[] = "AUX3/4 passthru";
PROGMEM prog_char lcd_param_text71[] = "AUX3/4 headfree";
PROGMEM prog_char lcd_param_text72[] = "AUX3/4 beeper";
// 73 to 81 reserved
#endif
//                                        0123456789.12345

PROGMEM const prog_void *lcd_param_ptr_table[] = {
    &lcd_param_text01, &P8[ROLL], &__P,
    &lcd_param_text02, &P8[ROLL], &__P,
    &lcd_param_text03, &I8[ROLL], &__I,
    &lcd_param_text04, &D8[ROLL], &__D,
    &lcd_param_text05, &P8[PITCH], &__P,
    &lcd_param_text06, &I8[PITCH], &__I,
    &lcd_param_text07, &D8[PITCH], &__D,
    &lcd_param_text08, &P8[YAW], &__P,
    &lcd_param_text09, &I8[YAW], &__I,
    &lcd_param_text10, &D8[YAW], &__D,
#if BARO
    &lcd_param_text11, &P8[PIDALT], &__P,
    &lcd_param_text12, &I8[PIDALT], &__I,
    &lcd_param_text13, &D8[PIDALT], &__D,
    &lcd_param_text14, &P8[PIDVEL], &__P,
    &lcd_param_text15, &I8[PIDVEL], &__I,
    &lcd_param_text16, &D8[PIDVEL], &__D,
#endif
    &lcd_param_text17, &P8[PIDLEVEL], &__P,
    &lcd_param_text18, &I8[PIDLEVEL], &__I,
#if MAG
    &lcd_param_text19, &P8[PIDMAG], &__P,
#endif
    &lcd_param_text20, &rcRate8, &__RCR,
    &lcd_param_text21, &rcExpo8, &__RC,
    &lcd_param_text22, &rollPitchRate, &__RC,
    &lcd_param_text23, &yawRate, &__RC,
    &lcd_param_text24, &dynThrPID, &__RC,
#ifdef LCD_CONF_AUX_12
    &lcd_param_text42, &activate1[BOXACC], &__AUX,
    &lcd_param_text43, &activate1[BOXBARO], &__AUX,
    &lcd_param_text44, &activate1[BOXMAG], &__AUX,
    &lcd_param_text45, &activate1[BOXCAMSTAB], &__AUX,
    &lcd_param_text46, &activate1[BOXCAMTRIG], &__AUX,
    &lcd_param_text47, &activate1[BOXARM], &__AUX,
    &lcd_param_text48, &activate1[BOXGPSHOME], &__AUX,
    &lcd_param_text49, &activate1[BOXGPSHOLD], &__AUX,
    &lcd_param_text50, &activate1[BOXPASSTHRU], &__AUX,
    &lcd_param_text51, &activate1[BOXHEADFREE], &__AUX,
    &lcd_param_text52, &activate1[BOXBEEPERON], &__AUX,
#endif
#ifdef LCD_CONF_AUX_1234
    &lcd_param_text42, &activate1[BOXACC], &__AUX,
    &lcd_param_text62, &activate2[BOXACC], &__AUX,
    &lcd_param_text43, &activate1[BOXBARO], &__AUX,
    &lcd_param_text63, &activate2[BOXBARO], &__AUX,
    &lcd_param_text44, &activate1[BOXMAG], &__AUX,
    &lcd_param_text64, &activate2[BOXMAG], &__AUX,
    &lcd_param_text45, &activate1[BOXCAMSTAB], &__AUX,
    &lcd_param_text65, &activate2[BOXCAMSTAB], &__AUX,
    &lcd_param_text46, &activate1[BOXCAMTRIG], &__AUX,
    &lcd_param_text66, &activate2[BOXCAMTRIG], &__AUX,
    &lcd_param_text47, &activate1[BOXARM], &__AUX,
    &lcd_param_text67, &activate2[BOXARM], &__AUX,
    &lcd_param_text48, &activate1[BOXGPSHOME], &__AUX,
    &lcd_param_text68, &activate2[BOXGPSHOME], &__AUX,
    &lcd_param_text49, &activate1[BOXGPSHOLD], &__AUX,
    &lcd_param_text69, &activate2[BOXGPSHOLD], &__AUX,
    &lcd_param_text50, &activate1[BOXPASSTHRU], &__AUX,
    &lcd_param_text70, &activate2[BOXPASSTHRU], &__AUX,
    &lcd_param_text51, &activate1[BOXHEADFREE], &__AUX,
    &lcd_param_text71, &activate2[BOXHEADFREE], &__AUX,
    &lcd_param_text52, &activate1[BOXBEEPERON], &__AUX,
    &lcd_param_text72, &activate2[BOXBEEPERON], &__AUX,
#endif
#ifdef LOG_VALUES
#if (LOG_VALUES == 2)
    &lcd_param_text25, &pMeter[0], &__PM,
    &lcd_param_text26, &pMeter[1], &__PM,
    &lcd_param_text27, &pMeter[2], &__PM,
    &lcd_param_text28, &pMeter[3], &__PM,
    &lcd_param_text29, &pMeter[4], &__PM,
    &lcd_param_text30, &pMeter[5], &__PM,
    &lcd_param_text31, &pMeter[6], &__PM,
    &lcd_param_text32, &pMeter[7], &__PM,
#endif
#endif
#ifdef POWERMETER
    &lcd_param_text33, &pMeter[PMOTOR_SUM], &__PS,
    &lcd_param_text34, &powerTrigger1, &__PT,
#endif
#ifdef VBAT
    &lcd_param_text35, &vbat, &__VB,
#endif
#ifdef FLYING_WING
    &lcd_param_text36, &wing_left_mid, &__SE,
    &lcd_param_text37, &wing_right_mid, &__SE,
#endif
#ifdef TRI
    &lcd_param_text38, &tri_yaw_middle, &__SE,
#endif
#ifdef LOG_VALUES
    &lcd_param_text39, &failsafeEvents, &__FS,
    &lcd_param_text40, &i2c_errors_count, &__L,
    &lcd_param_text41, &annex650_overrun_count, &__L
#endif
};
#define PARAMMAX (sizeof(lcd_param_ptr_table)/6 - 1)
// ************************************************************************************************************

void __u8Inc(void *var, int8_t inc)
{
    *(uint8_t *) var += inc;
};
void __u16Inc(void *var, int8_t inc)
{
    *(uint16_t *) var += inc;
};
void __nullInc(void *var, int8_t inc)
{
};

void __u8Fmt(void *var, uint8_t mul, uint8_t dec)
{
    uint16_t unit = *(uint8_t *) var;
    unit *= mul;
    char c1 = '0' + unit / 100;
    char c2 = '0' + unit / 10 - (unit / 100) * 10;
    char c3 = '0' + unit - (unit / 10) * 10;
    switch (dec) {
    case 0:
        line2[6] = c1;
        line2[7] = c2;
        line2[8] = c3;
        break;
    case 1:
        line2[5] = c1;
        line2[6] = c2;
        line2[7] = '.';
        line2[8] = c3;
        break;
    case 2:
        line2[5] = c1;
        line2[6] = '.';
        line2[7] = c2;
        line2[8] = c3;
        break;
    case 3:
        line2[4] = '0';
        line2[5] = '.';
        line2[6] = c1;
        line2[7] = c2;
        line2[8] = c3;
        break;
    }
}

void __u16Fmt(void *var, uint8_t mul, uint8_t dec)
{
    uint16_t unit = *(uint16_t *) var;
    unit *= mul;
    line2[4] = '0' + unit / 10000;
    line2[5] = '0' + unit / 1000 - (unit / 10000) * 10;
    line2[6] = '0' + unit / 100 - (unit / 1000) * 10;
    line2[7] = '0' + unit / 10 - (unit / 100) * 10;
    line2[8] = '0' + unit - (unit / 10) * 10;
}

void __uAuxFmt(void *var, uint8_t mul, uint8_t dec)
{
    uint8_t unit = *(uint8_t *) var;
    line2[0] = 'L';
    line2[1] = 'M';
    line2[2] = 'H';
    line2[4] = (unit & 1 << 0 ? 'X' : '.');
    line2[5] = (unit & 1 << 1 ? 'X' : '.');
    line2[6] = (unit & 1 << 2 ? 'X' : '.');
    line2[8] = (unit & 1 << 3 ? 'X' : '.');
    line2[9] = (unit & 1 << 4 ? 'X' : '.');
    line2[10] = (unit & 1 << 5 ? 'X' : '.');
}

void __upMFmt(void *var, uint8_t mul, uint8_t dec)
{
    uint32_t unit = *(uint32_t *) var;
    // pmeter values need special treatment, too many digits to fit standard 8 bit scheme
    unit = unit / PLEVELDIVSOFT;        // [0:1000] * 1000/3 samples per second(loop time) * 60 seconds *5 minutes -> [0:10000 e4] per motor
    // (that is full throttle for 5 minutes sampling with high sampling rate for wmp only)
    // times 6 for a maximum of 6 motors equals [0:60000 e4] for the sum
    // we are only interested in the big picture, so divide by 10.000
    __u16Fmt(&unit, mul, dec);
}

void __upSFmt(void *var, uint8_t mul, uint8_t dec)
{
    uint32_t unit = *(uint32_t *) var;
#if defined(POWERMETER_SOFT)
    unit = unit / PLEVELDIVSOFT;
#elif defined(POWERMETER_HARD)
    unit = unit / PLEVELDIV;
#endif
    __u16Fmt(&unit, mul, dec);
}

static uint8_t lcdStickState[3];
#define IsLow(x)  (lcdStickState[x] & 0x1)
#define IsHigh(x) (lcdStickState[x] & 0x2)
#define IsMid(x)  (!lcdStickState[x])

/* keys to navigate the LCD menu (preset to LCD_TEXTSTAR key-depress codes)*/
#define LCD_MENU_PREV 'a'
#define LCD_MENU_NEXT 'c'
#define LCD_VALUE_UP 'd'
#define LCD_VALUE_DOWN 'b'

void configurationLoop()
{
    uint8_t i, p;
    uint8_t LCD = 1;
    uint8_t refreshLCD = 1;
    uint8_t key = 0;
    initLCD();
    p = 0;
    while (LCD == 1) {
        if (refreshLCD) {
            blinkLED(10, 20, 1);
            strcpy_P(line1, PSTR("                "));
            strcpy(line2, line1);
            strcpy_P(line1, (char *)
                     pgm_read_word(&(lcd_param_ptr_table[p * 3])));
            lcd_param_def_t *deft = (lcd_param_def_t *) pgm_read_word(&(lcd_param_ptr_table[(p * 3) + 2]));
            deft->type->fmt((void *)
                            pgm_read_word(&(lcd_param_ptr_table[(p * 3) + 1])), deft->multiplier, deft->decimal);
            LCDclear();
            LCDsetLine(1);
            LCDprintChar(line1);        //refresh line 1 of LCD
            LCDsetLine(2);
            LCDprintChar(line2);        //refresh line 2 of LCD
            refreshLCD = 0;
        }
#if defined(LCD_TEXTSTAR) || defined(LCD_VT100) // textstar or vt100 can send keys
        key = (SerialAvailable(0) ? SerialRead(0) : 0);
#endif
#ifdef LCD_CONF_DEBUG
        delay(1000);
        if (key == LCD_MENU_NEXT)
            key = LCD_VALUE_UP;
        else
            key = LCD_MENU_NEXT;
#endif
        for (i = ROLL; i < THROTTLE; i++) {
            uint16_t Tmp = readRawRC(i);
            lcdStickState[i] = (Tmp < MINCHECK) | ((Tmp > MAXCHECK) << 1);
        };
        if (IsLow(YAW) && IsHigh(PITCH))
            LCD = 0;            // save and exit
        else if (IsHigh(YAW) && IsHigh(PITCH))
            LCD = 2;            // exit without save: eeprom has only 100.000 write cycles
        else if (key == LCD_MENU_NEXT || (IsLow(PITCH))) {      //switch config param with pitch
            refreshLCD = 1;
            p++;
            if (p > PARAMMAX)
                p = 0;
        } else if (key == LCD_MENU_PREV || (IsHigh(PITCH))) {
            refreshLCD = 1;
            p--;
            if (p == 0xFF)
                p = PARAMMAX;
        } else if (key == LCD_VALUE_DOWN || (IsLow(ROLL))) {    //+ or - param with low and high roll
            refreshLCD = 1;
            lcd_param_def_t *deft = (lcd_param_def_t *) pgm_read_word(&(lcd_param_ptr_table[(p * 3) + 2]));
            deft->type->inc((void *)
                            pgm_read_word(&(lcd_param_ptr_table[(p * 3) + 1])), -deft->increment);
            if (p == 0)
                P8[PITCH] = P8[ROLL];
        } else if (key == LCD_VALUE_UP || (IsHigh(ROLL))) {
            refreshLCD = 1;
            lcd_param_def_t *deft = (lcd_param_def_t *) pgm_read_word(&(lcd_param_ptr_table[(p * 3) + 2]));
            deft->type->inc((void *)
                            pgm_read_word(&(lcd_param_ptr_table[(p * 3) + 1])), +deft->increment);
            if (p == 0)
                P8[PITCH] = P8[ROLL];
        }
    }                           // while (LCD == 1)
    blinkLED(20, 30, 1);

    LCDclear();
    LCDsetLine(1);
    if (LCD == 0) {
        strcpy_P(line1, PSTR("Saving..."));
        LCDprintChar(line1);
    } else {
        strcpy_P(line1, PSTR("Aborting"));
        LCDprintChar(line1);
    }
    if (LCD == 0)
        writeParams();
    LCDsetLine(2);
    strcpy_P(line1, PSTR("Exit"));
    LCDprintChar(line1);
#if defined(LCD_LCD03)
    delay(2000);                // wait for two seconds then clear screen and show initial message
    initLCD();
#endif
#if defined(LCD_SERIAL3W)
    SerialOpen(0, 115200);
#endif
#ifdef LCD_TELEMETRY
    delay(1500);                // keep exit message visible for one and one half seconds even if (auto)telemetry continues writing in main loop
#endif
}
#endif                          // LCD_CONF


// -------------------- telemetry output to LCD over serial/i2c ----------------------------------

#ifdef LCD_TELEMETRY

  // LCDbar(n,v) : draw a bar graph - n number of chars for width, v value in % to display
void LCDbar(uint8_t n, uint8_t v)
{
#if defined(LCD_SERIAL3W)
    for (uint8_t i = 0; i < n; i++)
        LCDprint((i < n * v / 100 ? '=' : '.'));
#elif defined(LCD_TEXTSTAR)
    LCDprint(0xFE);
    LCDprint('b');
    LCDprint(n);
    LCDprint(v);
#elif defined(LCD_VT100)
    for (uint8_t i = 0; i < n; i++)
        LCDprint((i < n * v / 100 ? '=' : '.'));
#elif defined(LCD_ETPP)
    ETPP_barGraph(n, v);
#elif defined(LCD_LCD03)
    for (uint8_t i = 0; i < n; i++)
        LCDprint((i < n * v / 100 ? '=' : '.'));
#endif
}

void lcd_telemetry()
{
    uint16_t intPowerMeterSum;
    static uint8_t linenr = 0;
    switch (telemetry) {        // output telemetry data, if one of four modes is set
        uint16_t unit;
    case 1:                    // button A on Textstar LCD -> angles
    case '1':
        if (linenr++ % 2) {
            strcpy_P(line1, PSTR("Deg ---.-  ---.-"));
            // 0123456789.12345
            if (angle[0] < 0) {
                unit = -angle[0];
                line1[3] = '-';
            } else
                unit = angle[0];
            line1[4] = '0' + unit / 1000;
            line1[5] = '0' + unit / 100 - (unit / 1000) * 10;
            line1[6] = '0' + unit / 10 - (unit / 100) * 10;
            line1[8] = '0' + unit - (unit / 10) * 10;
            if (angle[1] < 0) {
                unit = -angle[1];
                line1[10] = '-';
            } else
                unit = angle[1];
            line1[11] = '0' + unit / 1000;
            line1[12] = '0' + unit / 100 - (unit / 1000) * 10;
            line1[13] = '0' + unit / 10 - (unit / 100) * 10;
            line1[15] = '0' + unit - (unit / 10) * 10;
            LCDsetLine(1);
            LCDprintChar(line1);
        } else {
            strcpy_P(line2, PSTR("---,-A max---,-A"));
#ifdef LOG_VALUES
            unit = powerValue * PINT2mA;
            line2[0] = '0' + unit / 10000;
            line2[1] = '0' + unit / 1000 - (unit / 10000) * 10;
            line2[2] = '0' + unit / 100 - (unit / 1000) * 10;
            line2[4] = '0' + unit / 10 - (unit / 100) * 10;
            unit = powerMax * PINT2mA;
            line2[10] = '0' + unit / 10000;
            line2[11] = '0' + unit / 1000 - (unit / 10000) * 10;
            line2[12] = '0' + unit / 100 - (unit / 1000) * 10;
            line2[14] = '0' + unit / 10 - (unit / 100) * 10;
#endif
            LCDsetLine(2);
            LCDprintChar(line2);
        }
        break;

    case 2:                    // button B on Textstar LCD -> Voltage, PowerSum and power alarm trigger value
    case '2':
        if (linenr++ % 2) {
            strcpy_P(line1, PSTR("--.-V   -----mAh"));  // uint8_t vbat, intPowerMeterSum
            // 0123456789.12345
#ifdef VBAT
            line1[0] = '0' + vbat / 100;
            line1[1] = '0' + vbat / 10 - (vbat / 100) * 10;
            line1[3] = '0' + vbat - (vbat / 10) * 10;
#endif
#ifdef POWERMETER
            intPowerMeterSum = (pMeter[PMOTOR_SUM] / PLEVELDIV);
            line1[8] = '0' + intPowerMeterSum / 10000;
            line1[9] = '0' + intPowerMeterSum / 1000 - (intPowerMeterSum / 10000) * 10;
            line1[10] = '0' + intPowerMeterSum / 100 - (intPowerMeterSum / 1000) * 10;
            line1[11] = '0' + intPowerMeterSum / 10 - (intPowerMeterSum / 100) * 10;
            line1[12] = '0' + intPowerMeterSum - (intPowerMeterSum / 10) * 10;
#endif
            if (buzzerState) {  // buzzer on? then add some blink for attention
                line1[5] = '+';
                line1[6] = '+';
                line1[7] = '+';
            }
            // set mark, if we had i2c errors, failsafes or annex650 overruns
            if (i2c_errors_count || failsafeEvents || annex650_overrun_count)
                line1[6] = '!';
            LCDsetLine(1);
            LCDprintChar(line1);
        } else {
            LCDsetLine(2);
#ifdef VBAT
            LCDbar(7, (((vbat - VBATLEVEL1_3S) * 100) / VBATREF));
            LCDprint(' ');
#else
            LCDprintChar("        ");
#endif
#ifdef POWERMETER
            //     intPowerMeterSum = (pMeter[PMOTOR_SUM]/PLEVELDIV);
            //   pAlarm = (uint32_t) powerTrigger1 * (uint32_t) PLEVELSCALE * (uint32_t) PLEVELDIV; // need to cast before multiplying
            intPowerMeterSum = (pMeter[PMOTOR_SUM] / PLEVELDIV);
            if (powerTrigger1)
                LCDbar(8, (intPowerMeterSum / (uint16_t) powerTrigger1) * 2);   // bar graph powermeter (scale intPowerMeterSum/powerTrigger1 with *100/PLEVELSCALE)
#endif
        }
        break;

    case 3:                    // button C on Textstar LCD -> cycle time
    case '3':
        if (linenr++ % 2) {
            strcpy_P(line1, PSTR("Cycle    -----us"));  //uin16_t cycleTime
            // 0123456789.12345*/
            //strcpy_P(line2,PSTR("(-----, -----)us")); //uin16_t cycleTimeMax
            line1[9] = '0' + cycleTime / 10000;
            line1[10] = '0' + cycleTime / 1000 - (cycleTime / 10000) * 10;
            line1[11] = '0' + cycleTime / 100 - (cycleTime / 1000) * 10;
            line1[12] = '0' + cycleTime / 10 - (cycleTime / 100) * 10;
            line1[13] = '0' + cycleTime - (cycleTime / 10) * 10;
            LCDsetLine(1);
            LCDprintChar(line1);
        } else {
#ifdef LOG_VALUES
            strcpy_P(line2, PSTR("(-----, -----)us"));  //uin16_t cycleTimeMax
            line2[1] = '0' + cycleTimeMin / 10000;
            line2[2] = '0' + cycleTimeMin / 1000 - (cycleTimeMin / 10000) * 10;
            line2[3] = '0' + cycleTimeMin / 100 - (cycleTimeMin / 1000) * 10;
            line2[4] = '0' + cycleTimeMin / 10 - (cycleTimeMin / 100) * 10;
            line2[5] = '0' + cycleTimeMin - (cycleTimeMin / 10) * 10;
            line2[8] = '0' + cycleTimeMax / 10000;
            line2[9] = '0' + cycleTimeMax / 1000 - (cycleTimeMax / 10000) * 10;
            line2[10] = '0' + cycleTimeMax / 100 - (cycleTimeMax / 1000) * 10;
            line2[11] = '0' + cycleTimeMax / 10 - (cycleTimeMax / 100) * 10;
            line2[12] = '0' + cycleTimeMax - (cycleTimeMax / 10) * 10;
            LCDsetLine(2);
            LCDprintChar(line2);
#endif
        }
        break;

    case 4:                    // button D on Textstar LCD -> sensors
    case '4':
#define GYROLIMIT 30            // threshold: for larger values replace bar with dots
#define ACCLIMIT 40             // threshold: for larger values replace bar with dots
        if (linenr++ % 2) {
            LCDsetLine(1);
            LCDprintChar("G "); //refresh line 1 of LCD
            if (abs(gyroData[0]) < GYROLIMIT) {
                LCDbar(4, (GYROLIMIT + gyroData[0]) * 50 / GYROLIMIT);
            } else
                LCDprintChar("....");
            LCDprint(' ');
            if (abs(gyroData[1]) < GYROLIMIT) {
                LCDbar(4, (GYROLIMIT + gyroData[1]) * 50 / GYROLIMIT);
            } else
                LCDprintChar("....");
            LCDprint(' ');
            if (abs(gyroData[2]) < GYROLIMIT) {
                LCDbar(4, (GYROLIMIT + gyroData[2]) * 50 / GYROLIMIT);
            } else
                LCDprintChar("....");
        } else {
            LCDsetLine(2);
            LCDprintChar("A "); //refresh line 2 of LCD
            if (abs(accSmooth[0]) < ACCLIMIT) {
                LCDbar(4, (ACCLIMIT + accSmooth[0]) * 50 / ACCLIMIT);
            } else
                LCDprintChar("....");
            LCDprint(' ');
            if (abs(accSmooth[1]) < ACCLIMIT) {
                LCDbar(4, (ACCLIMIT + accSmooth[1]) * 50 / ACCLIMIT);
            } else
                LCDprintChar("....");
            LCDprint(' ');
            if (abs(accSmooth[2] - acc_1G) < ACCLIMIT) {
                LCDbar(4, (ACCLIMIT + accSmooth[2] - acc_1G) * 50 / ACCLIMIT);
            } else
                LCDprintChar("....");
        }
        break;

    case 5:                    // No button.  Displays with auto telemetry only
    case '5':
        if (linenr++ % 2) {
            strcpy_P(line1, PSTR("Fails i2c t-errs"));
            LCDsetLine(1);
            LCDprintChar(line1);
        } else {
            // 0123456789012345
            strcpy_P(line2, PSTR("----  ----  ----"));
            unit = failsafeEvents;
            line2[0] = '0' + unit / 1000 - (unit / 10000) * 10;
            line2[1] = '0' + unit / 100 - (unit / 1000) * 10;
            line2[2] = '0' + unit / 10 - (unit / 100) * 10;
            line2[3] = '0' + unit - (unit / 10) * 10;
            unit = i2c_errors_count;
            line2[6] = '0' + unit / 1000 - (unit / 10000) * 10;
            line2[7] = '0' + unit / 100 - (unit / 1000) * 10;
            line2[8] = '0' + unit / 10 - (unit / 100) * 10;
            line2[9] = '0' + unit - (unit / 10) * 10;
            unit = annex650_overrun_count;
            line2[12] = '0' + unit / 1000 - (unit / 10000) * 10;
            line2[13] = '0' + unit / 100 - (unit / 1000) * 10;
            line2[14] = '0' + unit / 10 - (unit / 100) * 10;
            line2[15] = '0' + unit - (unit / 10) * 10;
            LCDsetLine(2);
            LCDprintChar(line2);
        }
        break;

    case 6:                    // No button.  Displays with auto telemetry only
    case '6':
        if (linenr++ % 2) {
            strcpy_P(line1, PSTR("Roll Pitch Throt"));
            if (armed)
                line2[14] = 'A';
            else
                line2[14] = 'a';
            if (failsafeCnt > 5)
                line2[15] = 'F';
            else
                line2[15] = 'f';
            LCDsetLine(1);
            LCDprintChar(line1);
        } else {
            // 0123456789012345
            strcpy_P(line2, PSTR("---- ---- ----xx"));
            line2[0] = '0' + rcData[ROLL] / 1000 - (rcData[ROLL] / 10000) * 10;
            line2[1] = '0' + rcData[ROLL] / 100 - (rcData[ROLL] / 1000) * 10;
            line2[2] = '0' + rcData[ROLL] / 10 - (rcData[ROLL] / 100) * 10;
            line2[3] = '0' + rcData[ROLL] - (rcData[ROLL] / 10) * 10;
            line2[5] = '0' + rcData[PITCH] / 1000 - (rcData[PITCH] / 10000) * 10;
            line2[6] = '0' + rcData[PITCH] / 100 - (rcData[PITCH] / 1000) * 10;
            line2[7] = '0' + rcData[PITCH] / 10 - (rcData[PITCH] / 100) * 10;
            line2[8] = '0' + rcData[PITCH] - (rcData[PITCH] / 10) * 10;
            line2[10] = '0' + rcData[THROTTLE] / 1000 - (rcData[THROTTLE] / 10000) * 10;
            line2[11] = '0' + rcData[THROTTLE] / 100 - (rcData[THROTTLE] / 1000) * 10;
            line2[12] = '0' + rcData[THROTTLE] / 10 - (rcData[THROTTLE] / 100) * 10;
            line2[13] = '0' + rcData[THROTTLE] - (rcData[THROTTLE] / 10) * 10;
            LCDsetLine(2);
            LCDprintChar(line2);
        }
        break;

    case 7:                    // No button.  Displays with auto telemetry only
    case '7':                  // contributed by PatrikE
        if (linenr++ % 2) {
            strcpy_P(line1, PSTR("Lat      Lon --"));
            // 0123456789012345
            if (armed)
                line1[14] = 'A';
            else
                line1[14] = 'a';
            if (failsafeCnt > 5)
                line1[15] = 'F';
            else
                line1[15] = 'f';
            LCDsetLine(1);
            LCDprintChar(line1);
        } else {
            strcpy_P(line2, PSTR("-------  -------"));
#if defined(GPS)
            line2[0] = '0' + GPS_latitude / 1000000 - (GPS_latitude / 10000000) * 10;
            line2[1] = '0' + GPS_latitude / 100000 - (GPS_latitude / 1000000) * 10;
            line2[2] = '0' + GPS_latitude / 10000 - (GPS_latitude / 100000) * 10;
            line2[3] = '0' + GPS_latitude / 1000 - (GPS_latitude / 10000) * 10;
            line2[4] = '0' + GPS_latitude / 100 - (GPS_latitude / 1000) * 10;
            line2[5] = '0' + GPS_latitude / 10 - (GPS_latitude / 100) * 10;
            line2[6] = '0' + GPS_latitude - (GPS_latitude / 10) * 10;
            line2[9] = '0' + GPS_longitude / 1000000 - (GPS_longitude / 10000000) * 10;
            line2[10] = '0' + GPS_longitude / 100000 - (GPS_longitude / 1000000) * 10;
            line2[11] = '0' + GPS_longitude / 10000 - (GPS_longitude / 100000) * 10;
            line2[12] = '0' + GPS_longitude / 1000 - (GPS_longitude / 10000) * 10;
            line2[13] = '0' + GPS_longitude / 100 - (GPS_longitude / 1000) * 10;
            line2[14] = '0' + GPS_longitude / 10 - (GPS_longitude / 100) * 10;
            line2[15] = '0' + GPS_longitude - (GPS_longitude / 10) * 10;
#endif
            LCDsetLine(2);
            LCDprintChar(line2);
        }
        break;

#if defined(LOG_VALUES) && defined(DEBUG)
    case 'R':
        //Reset logvalues
#if defined(LOG_VALUES) && defined(DEBUG)
        cycleTimeMax = 0;       // reset min/max on transition on->off
        cycleTimeMin = 65535;
#endif
        telemetry = 0;          // no use to repeat this forever
        break;
#endif                          // case R

#ifdef DEBUG
    case 'F':
        extern unsigned int __bss_end;
        extern unsigned int __heap_start;
        extern void *__brkval;
        int free_memory;
        if ((int) __brkval == 0)
            free_memory = ((int) &free_memory) - ((int) &__bss_end);
        else
            free_memory = ((int) &free_memory) - ((int) __brkval);
        strcpy_P(line1, PSTR(" Free ----"));    // uint8_t free_memory
        line1[6] = '0' + free_memory / 1000 - (free_memory / 10000) * 10;
        line1[7] = '0' + free_memory / 100 - (free_memory / 1000) * 10;
        line1[8] = '0' + free_memory / 10 - (free_memory / 100) * 10;
        line1[9] = '0' + free_memory - (free_memory / 10) * 10;
        LCDsetLine(1);
        LCDprintChar(line1);
        break;
#endif                          // DEBUG

        // WARNING: if you add another case here, you should also add a case: in Serial.pde, so users can access your case via terminal input
    }                           // end switch (telemetry) 
}                               // end function
#endif                          //  LCD_TELEMETRY
