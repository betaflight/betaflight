#include "board.h"

// BMP085, Standard address 0x77
static bool convDone = false;
static uint16_t convOverrun = 0;

#define BARO_OFF                 digitalLo(BARO_GPIO, BARO_PIN);
#define BARO_ON                  digitalHi(BARO_GPIO, BARO_PIN);

// EXTI14 for BMP085 End of Conversion Interrupt
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line14) == SET) {
        EXTI_ClearITPendingBit(EXTI_Line14);
        convDone = true;
    }
}

typedef struct {
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} bmp085_smd500_calibration_param_t;

typedef struct  {
    bmp085_smd500_calibration_param_t cal_param;
    uint8_t mode;
    uint8_t chip_id, ml_version, al_version;
    uint8_t dev_addr;
    uint8_t sensortype;
    int32_t param_b5;
    int16_t oversampling_setting;
    int16_t smd500_t_resolution, smd500_masterclock;
} bmp085_t;

#define BMP085_I2C_ADDR         0x77
#define BMP085_CHIP_ID          0x55
#define BOSCH_PRESSURE_BMP085   85
#define BMP085_CHIP_ID_REG      0xD0
#define BMP085_VERSION_REG      0xD1
#define E_SENSOR_NOT_DETECTED   (char) 0
#define BMP085_PROM_START__ADDR 0xaa
#define BMP085_PROM_DATA__LEN   22
#define BMP085_T_MEASURE        0x2E                // temperature measurent 
#define BMP085_P_MEASURE        0x34                // pressure measurement
#define BMP085_CTRL_MEAS_REG    0xF4
#define BMP085_ADC_OUT_MSB_REG  0xF6
#define BMP085_ADC_OUT_LSB_REG  0xF7
#define BMP085_CHIP_ID__POS     0
#define BMP085_CHIP_ID__MSK     0xFF
#define BMP085_CHIP_ID__LEN     8
#define BMP085_CHIP_ID__REG     BMP085_CHIP_ID_REG

#define BMP085_ML_VERSION__POS      0
#define BMP085_ML_VERSION__LEN      4
#define BMP085_ML_VERSION__MSK      0x0F
#define BMP085_ML_VERSION__REG      BMP085_VERSION_REG



#define BMP085_AL_VERSION__POS      4
#define BMP085_AL_VERSION__LEN      4
#define BMP085_AL_VERSION__MSK      0xF0
#define BMP085_AL_VERSION__REG      BMP085_VERSION_REG

#define BMP085_GET_BITSLICE(regvar, bitname) (regvar & bitname##__MSK) >> bitname##__POS
#define BMP085_SET_BITSLICE(regvar, bitname, val) (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)

#define SMD500_PARAM_MG      3038        //calibration parameter
#define SMD500_PARAM_MH     -7357        //calibration parameter
#define SMD500_PARAM_MI      3791        //calibration parameter

static bmp085_t bmp085 = { { 0, } };
static bmp085_t *p_bmp085 = &bmp085;                      /**< pointer to SMD500 / BMP085 device area */
static bool bmp085InitDone = false;

static void bmp085_get_cal_param(void);

bool bmp085Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef   EXTI_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;
    uint8_t data;

    if (bmp085InitDone)
        return true;

    // PC13, PC14 (Barometer XCLR reset output, EOC input)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    BARO_ON;

    // EXTI interrupt for barometer EOC
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);
    EXTI_InitStructure.EXTI_Line = EXTI_Line14;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Enable and set EXTI10-15 Interrupt to the lowest priority
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    delay(12); // datasheet says 10ms, we'll be careful and do 12.

    p_bmp085->sensortype = E_SENSOR_NOT_DETECTED;
    p_bmp085->dev_addr = BMP085_I2C_ADDR;                   /* preset BMP085 I2C_addr */
    i2cRead(p_bmp085->dev_addr, BMP085_CHIP_ID__REG, 1, &data);  /* read Chip Id */
    p_bmp085->chip_id = BMP085_GET_BITSLICE(data, BMP085_CHIP_ID);
    p_bmp085->oversampling_setting = 3;

    if (p_bmp085->chip_id == BMP085_CHIP_ID) {            /* get bitslice */
        p_bmp085->sensortype = BOSCH_PRESSURE_BMP085;

        i2cRead(p_bmp085->dev_addr, BMP085_VERSION_REG, 1, &data); /* read Version reg */
        p_bmp085->ml_version = BMP085_GET_BITSLICE(data, BMP085_ML_VERSION);        /* get ML Version */
        p_bmp085->al_version = BMP085_GET_BITSLICE(data, BMP085_AL_VERSION);        /* get AL Version */
        bmp085_get_cal_param(); /* readout bmp085 calibparam structure */
        bmp085InitDone = true;
        return true;
    }

    return false;
}

int16_t bmp085_read_temperature(void)
{
    convDone = false;
    bmp085_start_ut();
    if (!convDone)
        convOverrun++;
    return bmp085_get_temperature(bmp085_get_ut());
}

int32_t bmp085_read_pressure(void)
{
    convDone = false;
    bmp085_start_up();
    if (!convDone)
        convOverrun++;
    return bmp085_get_pressure(bmp085_get_up());
}

// #define BMP_TEMP_OSS 4

int16_t bmp085_get_temperature(uint32_t ut)
{
    int16_t temperature;
    int32_t x1, x2;
#ifdef BMP_TEMP_OSS    
    static uint32_t temp;
#endif

    if (p_bmp085->sensortype == BOSCH_PRESSURE_BMP085) {
        x1 = (((int32_t) ut - (int32_t) p_bmp085->cal_param.ac6) * (int32_t) p_bmp085->cal_param.ac5) >> 15;
        x2 = ((int32_t) p_bmp085->cal_param.mc << 11) / (x1 + p_bmp085->cal_param.md);
        p_bmp085->param_b5 = x1 + x2;
    }
    temperature = ((p_bmp085->param_b5 + 8) >> 4);  // temperature in 0.1°C

#ifdef BMP_TEMP_OSS    
    temp *= (1 << BMP_TEMP_OSS) - 1;        // multiply the temperature variable by 3 - we have tau == 1/4
    temp += ((uint32_t)temperature) << 8;   // add on the buffer
    temp >>= BMP_TEMP_OSS;                  // divide by 4
    return (int16_t)temp;
#else
    return temperature;
#endif
}

int32_t bmp085_get_pressure(uint32_t up)
{
    int32_t pressure, x1, x2, x3, b3, b6;
    uint32_t b4, b7;

    b6 = p_bmp085->param_b5 - 4000;
    // *****calculate B3************
    x1 = (b6 * b6) >> 12;
    x1 *= p_bmp085->cal_param.b2;
    x1 >>= 11;

    x2 = (p_bmp085->cal_param.ac2 * b6);
    x2 >>= 11;

    x3 = x1 + x2;

    b3 = (((((int32_t)p_bmp085->cal_param.ac1) * 4 + x3) << p_bmp085->oversampling_setting) + 2) >> 2;

    // *****calculate B4************
    x1 = (p_bmp085->cal_param.ac3 * b6) >> 13;
    x2 = (p_bmp085->cal_param.b1 * ((b6 * b6) >> 12) ) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (p_bmp085->cal_param.ac4 * (uint32_t) (x3 + 32768)) >> 15;
     
    b7 = ((uint32_t)(up - b3) * (50000 >> p_bmp085->oversampling_setting));
    if (b7 < 0x80000000) {
        pressure = (b7 << 1) / b4;
    } else { 
        pressure = (b7 / b4) << 1;
    }

    x1 = pressure >> 8;
    x1 *= x1;
    x1 = (x1 * SMD500_PARAM_MG) >> 16;
    x2 = (pressure * SMD500_PARAM_MH) >> 16;
    pressure += (x1 + x2 + SMD500_PARAM_MI) >> 4;   // pressure in Pa

    return pressure;
}

void bmp085_start_ut(void)
{
    convDone = false;
    i2cWrite(p_bmp085->dev_addr, BMP085_CTRL_MEAS_REG, BMP085_T_MEASURE);
}

uint16_t bmp085_get_ut(void)
{
    uint16_t ut;
    uint8_t data[2];    
    uint16_t timeout = 10000;

    // wait in case of cockup
    if (!convDone)
        convOverrun++;
#if 0
    while (!convDone && timeout-- > 0) {
        __NOP();
    }
#endif
    i2cRead(p_bmp085->dev_addr, BMP085_ADC_OUT_MSB_REG, 2, data);
    ut = (data[0] << 8) | data[1];
    return ut;
}

void bmp085_start_up(void)
{
    uint8_t ctrl_reg_data;

    ctrl_reg_data = BMP085_P_MEASURE + (p_bmp085->oversampling_setting << 6);
    convDone = false;
    i2cWrite(p_bmp085->dev_addr, BMP085_CTRL_MEAS_REG, ctrl_reg_data);
}

/** read out up for pressure conversion
  depending on the oversampling ratio setting up can be 16 to 19 bit
   \return up parameter that represents the uncompensated pressure value
*/
uint32_t bmp085_get_up(void)
{
    uint32_t up = 0;
    uint8_t data[3];
    uint16_t timeout = 10000;
    
    // wait in case of cockup
    if (!convDone)
        convOverrun++;
#if 0
    while (!convDone && timeout-- > 0) {
        __NOP();
    }
#endif
    i2cRead(p_bmp085->dev_addr, BMP085_ADC_OUT_MSB_REG, 3, data);
    up = (((uint32_t) data[0] << 16) | ((uint32_t) data[1] << 8) | (uint32_t) data[2]) >> (8 - p_bmp085->oversampling_setting);

    return up;
}

static void bmp085_get_cal_param(void)
{
    uint8_t data[22];
    i2cRead(p_bmp085->dev_addr, BMP085_PROM_START__ADDR, BMP085_PROM_DATA__LEN, data);

    /*parameters AC1-AC6*/
    p_bmp085->cal_param.ac1 =  (data[0] <<8) | data[1];
    p_bmp085->cal_param.ac2 =  (data[2] <<8) | data[3];
    p_bmp085->cal_param.ac3 =  (data[4] <<8) | data[5];
    p_bmp085->cal_param.ac4 =  (data[6] <<8) | data[7];
    p_bmp085->cal_param.ac5 =  (data[8] <<8) | data[9];
    p_bmp085->cal_param.ac6 = (data[10] <<8) | data[11];

    /*parameters B1,B2*/
    p_bmp085->cal_param.b1 =  (data[12] <<8) | data[13];
    p_bmp085->cal_param.b2 =  (data[14] <<8) | data[15];

    /*parameters MB,MC,MD*/
    p_bmp085->cal_param.mb =  (data[16] <<8) | data[17];
    p_bmp085->cal_param.mc =  (data[18] <<8) | data[19];
    p_bmp085->cal_param.md =  (data[20] <<8) | data[21];
}
