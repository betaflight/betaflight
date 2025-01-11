/*****************************************************************************
 *
 *   Copyright(C) 2011, Embedded Artists AB
 *   All rights reserved.
 *
 ******************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * Embedded Artists AB assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. Embedded Artists AB
 * reserves the right to make changes in the software without
 * notification. Embedded Artists AB also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 *****************************************************************************/

/*
 * NOTE: I2C must have been initialized before calling any functions in this
 * file.
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

//#include "board.h"
#include "chip.h"

#include "pca9532.h"

/******************************************************************************
 * Defines and typedefs
 *****************************************************************************/

#define I2C_PORT (LPC_I2C0)

#define LS_MODE_ON     0x01
#define LS_MODE_BLINK0 0x02
#define LS_MODE_BLINK1 0x03

/******************************************************************************
 * External global variables
 *****************************************************************************/


/******************************************************************************
 * Local variables
 *****************************************************************************/

static uint16_t blink0Shadow = 0;
static uint16_t blink1Shadow = 0;
static uint16_t ledStateShadow = 0;

/******************************************************************************
 * Local Functions
 *****************************************************************************/

static Status I2CWrite(uint32_t addr, uint8_t* buf, uint32_t len)
{
	I2CM_XFER_T i2cData;

	i2cData.slaveAddr = addr;
	i2cData.options = 0;
	i2cData.status = 0;
	i2cData.txBuff = buf;
	i2cData.txSz = len;
	i2cData.rxBuff = NULL;
	i2cData.rxSz = 0;

	if (Chip_I2CM_XferBlocking(LPC_I2C0, &i2cData) == 0) {
		return ERROR;
	}
	return SUCCESS;
}

static Status I2CRead(uint32_t addr, uint8_t* buf, uint32_t len)
{
	I2CM_XFER_T i2cData;

	i2cData.slaveAddr = addr;
	i2cData.options = 0;
	i2cData.status = 0;
	i2cData.txBuff = NULL;
	i2cData.txSz = 0;
	i2cData.rxBuff = buf;
	i2cData.rxSz = len;

	if (Chip_I2CM_XferBlocking(LPC_I2C0, &i2cData) == 0) {
		return ERROR;
	}
	return SUCCESS;
}

static void setLsStates(uint16_t states, uint8_t* ls, uint8_t mode)
{
#define IS_LED_SET(bit, x) ( ( ((x) & (bit)) != 0 ) ? 1 : 0 )

    int i = 0;

    for (i = 0; i < 4; i++) {

        ls[i] |= ( (IS_LED_SET(0x0001, states)*mode << 0)
                | (IS_LED_SET(0x0002, states)*mode << 2)
                | (IS_LED_SET(0x0004, states)*mode << 4)
                | (IS_LED_SET(0x0008, states)*mode << 6) );

        states >>= 4;
    }
}

static void setLeds(void)
{
    uint8_t buf[5];
    uint8_t ls[4] = {0,0,0,0};
    uint16_t states = ledStateShadow;

    /* LEDs in On/Off state */
    setLsStates(states, ls, LS_MODE_ON);

    /* set the LEDs that should blink */
    setLsStates(blink0Shadow, ls, LS_MODE_BLINK0);
    setLsStates(blink1Shadow, ls, LS_MODE_BLINK1);


    buf[0] = PCA9532_LS0 | PCA9532_AUTO_INC;
    buf[1] = ls[0];
    buf[2] = ls[1];
    buf[3] = ls[2];
    buf[4] = ls[3];
    I2CWrite(PCA9532_I2C_ADDR, buf, 5);
}

/******************************************************************************
 * Public Functions
 *****************************************************************************/

/******************************************************************************
 *
 * Description:
 *    Initialize the PCA9532 Device
 *
 *****************************************************************************/
void pca9532_init (void)
{
    /* nothing to initialize */
}

/******************************************************************************
 *
 * Description:
 *    Get the LED states
 *
 * Params:
 *    [in]  shadow  - TRUE if the states should be retrieved from the shadow
 *                    variables. The shadow variable are updated when any
 *                    of setLeds, setBlink0Leds and/or setBlink1Leds are
 *                    called.
 *
 *                    FALSE if the state should be retrieved from the PCA9532
 *                    device. A blinkin LED may be reported as on or off
 *                    depending on the state when calling the function.
 *
 * Returns:
 *      A mask where a 1 indicates that a LED is on (or blinking).
 *
 *****************************************************************************/
uint16_t pca9532_getLedState (uint32_t shadow)
{
    uint8_t buf[2];
    uint16_t ret = 0;

    if (shadow) {
        /* a blink LED is reported as on*/
        ret = (ledStateShadow | blink0Shadow | blink1Shadow);
    }
    else {

        /*
         * A blinking LED may be reported as on or off depending on
         * its state when reading the Input register.
         */

        buf[0] = PCA9532_INPUT0;
        I2CWrite(PCA9532_I2C_ADDR, buf, 1);

        I2CRead(PCA9532_I2C_ADDR, buf, 1);
        ret = buf[0];


        buf[0] = PCA9532_INPUT1;
        I2CWrite(PCA9532_I2C_ADDR, buf, 1);

        I2CRead(PCA9532_I2C_ADDR, buf, 1);
        ret |= (buf[0] << 8);


        /* invert since LEDs are active low */
        ret = ((~ret) & 0xFFFF);
    }

    return (ret & ~PCA9532_NOT_USED);
}


/******************************************************************************
 *
 * Description:
 *    Set LED states (on or off).
 *
 * Params:
 *    [in]  ledOnMask  - The LEDs that should be turned on. This mask has
 *                       priority over ledOffMask
 *    [in]  ledOffMask - The LEDs that should be turned off.
 *
 *****************************************************************************/
void pca9532_setLeds (uint16_t ledOnMask, uint16_t ledOffMask)
{
    /* turn off leds */
    ledStateShadow &= (~(ledOffMask) & 0xffff);

    /* ledOnMask has priority over ledOffMask */
    ledStateShadow |= ledOnMask;

    /* turn off blinking */
    blink0Shadow &= (~(ledOffMask) & 0xffff);
    blink1Shadow &= (~(ledOffMask) & 0xffff);

    setLeds();
}

/******************************************************************************
 *
 * Description:
 *    Set the blink period for PWM0. Valid values are 0 - 255 where 0
 *    means 152 Hz and 255 means 0.59 Hz. A value of 151 means 1 Hz.
 *
 * Params:
 *    [in]  period  - the period for pwm0
 *
 *****************************************************************************/
void pca9532_setBlink0Period(uint8_t period)
{
    uint8_t buf[2];

    buf[0] = PCA9532_PSC0;
    buf[1] = period;
    I2CWrite(PCA9532_I2C_ADDR, buf, 2);
}

/******************************************************************************
 *
 * Description:
 *    Set the duty cycle for PWM0. Valid values are 0 - 100. 25 means the LED
 *    is on 25% of the period.
 *
 * Params:
 *    [in]  duty  - duty cycle
 *
 *****************************************************************************/
void pca9532_setBlink0Duty(uint8_t duty)
{
    uint8_t buf[2];
    uint32_t tmp = duty;
    if (tmp > 100) {
        tmp = 100;
    }

    tmp = (256 * tmp)/100;

    buf[0] = PCA9532_PWM0;
    buf[1] = tmp;
    I2CWrite(PCA9532_I2C_ADDR, buf, 2);
}

/******************************************************************************
 *
 * Description:
 *    Set the LEDs that should blink with rate and duty cycle from PWM0.
 *    Blinking is turned off with pca9532_setLeds.
 *
 * Params:
 *    [in]  ledMask  - LEDs that should blink.
 *
 *****************************************************************************/
void pca9532_setBlink0Leds(uint16_t ledMask)
{
    blink0Shadow |= ledMask;
    setLeds();
}

/******************************************************************************
 *
 * Description:
 *    Set the blink period for PWM1. Valid values are 0 - 255 where 0
 *    means 152 Hz and 255 means 0.59 Hz. A value of 151 means 1 Hz.
 *
 * Params:
 *    [in]  period  - The period for PWM1
 *
 *****************************************************************************/
void pca9532_setBlink1Period(uint8_t period)
{
    uint8_t buf[2];

    buf[0] = PCA9532_PSC1;
    buf[1] = period;
    I2CWrite(PCA9532_I2C_ADDR, buf, 2);
}

/******************************************************************************
 *
 * Description:
 *    Set the duty cycle for PWM1. Valid values are 0 - 100. 25 means the LED
 *    is on 25% of the period.
 *
 * Params:
 *    [in]  duty  - duty cycle.
 *
 *****************************************************************************/
void pca9532_setBlink1Duty(uint8_t duty)
{
    uint8_t buf[2];

    uint32_t tmp = duty;
    if (tmp > 100) {
        tmp = 100;
    }

    tmp = (256 * tmp)/100;

    buf[0] = PCA9532_PWM1;
    buf[1] = tmp;
    I2CWrite(PCA9532_I2C_ADDR, buf, 2);
}

/******************************************************************************
 *
 * Description:
 *    Set the LEDs that should blink with rate and duty cycle from PWM1.
 *    Blinking is turned off with pca9532_setLeds.
 *
 * Params:
 *    [in]  ledMask  - LEDs that should blink.
 *
 *****************************************************************************/
void pca9532_setBlink1Leds(uint16_t ledMask)
{
    blink1Shadow |= ledMask;
    setLeds();
}
