/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"

#include "sensors/gyro.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_imuf9001.h"

#include "common/axis.h"
#include "build/debug.h"
#include "common/maths.h"
#include "drivers/serial.h"
#include "drivers/bus_spi.h"
#include "drivers/dma_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/sensor.h"
#include "drivers/time.h"
#include "fc/config.h"
#include "fc/runtime_config.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "drivers/system.h"


#ifdef USE_GYRO_IMUF9001

volatile uint16_t imufCurrentVersion = IMUF_FIRMWARE_MIN_VERSION;
volatile uint32_t isImufCalibrating = 0;
volatile imuFrame_t imufQuat;
gyroDev_t *imufDev;

inline uint32_t getCrcImuf9001(uint32_t* data, uint32_t size)
{
    CRC_ResetDR(); //reset data register
    for(uint32_t x=0; x<size; x++ )
    {
        CRC_CalcCRC(data[x]);
    }
    return CRC_GetCRC();
}

inline void appendCrcToData(uint32_t* data, uint32_t size)
{
    data[size] = getCrcImuf9001(data, size);;
}

static inline void gpio_write_pin(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, gpioState_t pinState)
{
    if (pinState == GPIO_HI)
    {
        GPIOx->BSRRL = (uint32_t)GPIO_Pin;
    }
    else
    {
        GPIOx->BSRRH = (uint32_t)GPIO_Pin;
    }
}

void resetImuf9001(void)
{
    gpio_write_pin(IMUF_RST_PORT, IMUF_RST_PIN, GPIO_LO);
    //blink
    for(uint32_t x = 0; x<40; x++)
    {
        LED0_TOGGLE;
        delay(20);
    }
    LED0_OFF;
    gpio_write_pin(IMUF_RST_PORT, IMUF_RST_PIN, GPIO_HI);
    delay(100);
}


#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)
#define GPIO_GET_INDEX(__GPIOx__)    (uint8_t)(((__GPIOx__) == (GPIOA))? 0U :\
                                               ((__GPIOx__) == (GPIOB))? 1U :\
                                               ((__GPIOx__) == (GPIOC))? 2U :\
                                               ((__GPIOx__) == (GPIOD))? 3U :\
                                               ((__GPIOx__) == (GPIOE))? 4U :\
                                               ((__GPIOx__) == (GPIOF))? 5U :\
                                               ((__GPIOx__) == (GPIOG))? 6U :\
                                               ((__GPIOx__) == (GPIOH))? 7U : 8U)
#endif

void imufDeinitGpio(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin)
{
  uint32_t position;
  uint32_t ioposition = 0x00U;
  uint32_t iocurrent = 0x00U;
  uint32_t tmp = 0x00U;

  /* Configure the port pins */
  for(position = 0U; position < 16; position++)
  {
    /* Get the IO position */
    ioposition = 0x01U << position;
    /* Get the current IO position */
    iocurrent = (GPIO_Pin) & ioposition;

    if(iocurrent == ioposition)
    {
      /*------------------------- GPIO Mode Configuration --------------------*/
      /* Configure IO Direction in Input Floating Mode */
      GPIOx->MODER &= ~(GPIO_MODER_MODER0 << (position * 2U));

      /* Configure the default Alternate Function in current IO */
      GPIOx->AFR[position >> 3U] &= ~(0xFU << ((uint32_t)(position & 0x07U) * 4U)) ;

      /* Configure the default value for IO Speed */
      GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2U));

      /* Configure the default value IO Output Type */
      GPIOx->OTYPER  &= ~(GPIO_OTYPER_OT_0 << position) ;

      /* Deactivate the Pull-up and Pull-down resistor for the current IO */
      GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));

      /*------------------------- EXTI Mode Configuration --------------------*/
      tmp = SYSCFG->EXTICR[position >> 2U];
      tmp &= (0x0FU << (4U * (position & 0x03U)));
      if(tmp == ((uint32_t)(GPIO_GET_INDEX(GPIOx)) << (4U * (position & 0x03U))))
      {
        /* Configure the External Interrupt or event for the current IO */
        tmp = 0x0FU << (4U * (position & 0x03U));
        SYSCFG->EXTICR[position >> 2U] &= ~tmp;

        /* Clear EXTI line configuration */
        EXTI->IMR &= ~((uint32_t)iocurrent);
        EXTI->EMR &= ~((uint32_t)iocurrent);
        
        /* Clear Rising Falling edge configuration */
        EXTI->RTSR &= ~((uint32_t)iocurrent);
        EXTI->FTSR &= ~((uint32_t)iocurrent);
      }
    }
  }
}

void initImuf9001(void) 
{
    //GPIO manipulation should go into a fast GPIO driver and should be separate from the befhal
    #ifdef USE_HAL_F7_CRC
        HAL_GPIO_DeInit(IMUF_RST_PORT, IMUF_RST_PIN);
        GPIO_InitTypeDef  GPIO_InitStruct;
        GPIO_InitStruct.Pin       = IMUF_RST_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;

        HAL_GPIO_Init(IMUF_RST_PORT, &GPIO_InitStruct);
    #else
        //GPIO_DeInit(IMUF_RST_PORT);
        imufDeinitGpio(IMUF_RST_PORT, IMUF_RST_PIN);
        GPIO_InitTypeDef gpioInitStruct;
        gpioInitStruct.GPIO_Pin   = IMUF_RST_PIN;
        gpioInitStruct.GPIO_Mode  = GPIO_Mode_OUT;
        gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        gpioInitStruct.GPIO_OType = GPIO_OType_OD;
        gpioInitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
        GPIO_Init(IMUF_RST_PORT, &gpioInitStruct);
    #endif

    resetImuf9001();
}

bool imufSendReceiveSpiBlocking(const busDevice_t *bus, uint8_t *dataTx, uint8_t *daRx, uint8_t length)
{
    spiBusTransfer(bus, dataTx, daRx, length);
    return true;
}

static int imuf9001SendReceiveCommand(const gyroDev_t *gyro, gyroCommands_t commandToSend, imufCommand_t *reply, imufCommand_t *data)
{

    imufCommand_t command;
    uint32_t attempt, crcCalc;
    int failCount = 500;

    memset(reply, 0, sizeof(command));

    if (data)
    {
        memcpy(&command, data, sizeof(command));
    }
    else
    {
        memset(&command, 0, sizeof(command));
    }

    command.command = commandToSend;
    command.crc     = getCrcImuf9001((uint32_t *)&command, 11);;


    while (failCount-- > 0)
    {
        delayMicroseconds(1000);
        if( IORead(IOGetByTag(IO_TAG(MPU_INT_EXTI))) ) //IMU is ready to talk
        {
            failCount -= 100;
            if (imufSendReceiveSpiBlocking(&(gyro->bus), (uint8_t *)&command, (uint8_t *)reply, sizeof(imufCommand_t)))
            {
                crcCalc = getCrcImuf9001((uint32_t *)reply, 11);
                //this is the only valid reply we'll get if we're in BL mode
                if(crcCalc == reply->crc && (reply->command == IMUF_COMMAND_LISTENING || reply->command == BL_LISTENING)) //this tells us the IMU was listening for a command, else we need to reset synbc
                {
                    for (attempt = 0; attempt < 100; attempt++)
                    {
                        //reset command, just waiting for reply data now
                        command.command = IMUF_COMMAND_NONE;
                        command.crc     = getCrcImuf9001((uint32_t *)&command, 11);
                        if (commandToSend == BL_ERASE_ALL){
                            delay(600);
                        }
                        if(commandToSend == BL_WRITE_FIRMWARES)
                        {
                            delay(10);
                        }
                        delayMicroseconds(1000); //give pin time to set

                        if( IORead(IOGetByTag(IO_TAG(MPU_INT_EXTI))) ) //IMU is ready to talk
                        {
                            //reset attempts
                            attempt = 100;

                            imufSendReceiveSpiBlocking(&(gyro->bus), (uint8_t *)&command, (uint8_t *)reply, sizeof(imufCommand_t));
                            crcCalc = getCrcImuf9001((uint32_t *)reply, 11);

                            if(crcCalc == reply->crc && reply->command == commandToSend ) //this tells us the IMU understood the last command
                            {
                                return 1;
                            }
                        }
                    }
                }
            }
        }
    }
    return 0;
}

int imufBootloader()
{
    
    imufCommand_t reply;
    imufCommand_t data;
    memset(&data, 0, sizeof(data));

    //config BL pin as output (shared with EXTI, this happens before EXTI init though)
    //config pins
    #ifdef USE_HAL_F7_CRC
        HAL_GPIO_DeInit(IMUF_EXTI_PORT, IMUF_EXTI_PIN);
        GPIO_InitTypeDef  GPIO_InitStruct;
        GPIO_InitStruct.Pin       = IMUF_EXTI_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;

        HAL_GPIO_Init(IMUF_EXTI_PORT, &GPIO_InitStruct);
    #else
        //GPIO_DeInit(IMUF_EXTI_PORT);
        imufDeinitGpio(IMUF_EXTI_PORT, IMUF_EXTI_PIN);
        GPIO_InitTypeDef gpioInitStruct;
        gpioInitStruct.GPIO_Pin   = IMUF_EXTI_PIN;
        gpioInitStruct.GPIO_Mode  = GPIO_Mode_OUT;
        gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        gpioInitStruct.GPIO_OType = GPIO_OType_PP;
        gpioInitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
        GPIO_Init(IMUF_EXTI_PORT, &gpioInitStruct);
    #endif
    //config pins
    delay(200);

    gpio_write_pin(IMUF_EXTI_PORT, IMUF_EXTI_PIN, 1);     //set bl pin Hi
    initImuf9001(); //reset imuf, make three blinks
    resetImuf9001(); //reset imuf, make three blinks
    resetImuf9001(); //reset imuf, make three blinks
    delay(1000);      //delay 100 ms. give IMUF BL time to look for bl init pin
    gpio_write_pin(IMUF_EXTI_PORT, IMUF_EXTI_PIN, 0);    //set bl pin Lo

    //config EXTI as input so we can check imuf status
    //config pins
    #ifdef USE_HAL_F7_CRC
    GPIO_InitStruct.Pin       = IMUF_EXTI_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;

    HAL_GPIO_Init(IMUF_EXTI_PORT, &GPIO_InitStruct);
    #else
    gpioInitStruct.GPIO_Pin   = IMUF_EXTI_PIN;
    gpioInitStruct.GPIO_Mode  = GPIO_Mode_IN;
    gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInitStruct.GPIO_OType = GPIO_OType_PP;
    gpioInitStruct.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(IMUF_EXTI_PORT, &gpioInitStruct);
    #endif
    //config pins
    delay(200);

    if (imuf9001SendReceiveCommand(imufDev, BL_REPORT_INFO, &reply, &data))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int imufUpdate(uint8_t *buff, uint32_t bin_length)
{
    imufCommand_t reply;
    imufCommand_t data;
    memset(&data, 0, sizeof(data));

    //check if BL is active
    if (imuf9001SendReceiveCommand(imufDev, BL_REPORT_INFO, &reply, &data))
    {
        //erase firmware on MCU
        if( imuf9001SendReceiveCommand(imufDev, BL_ERASE_ALL, &reply, &data) )
        {
            //good data
            if( imuf9001SendReceiveCommand(imufDev, BL_PREPARE_PROGRAM, &reply, &data) )
            {

                //blink
                for(uint32_t x = 0; x<10; x++)
                {
                    LED0_TOGGLE;
                    delay(200);
                }
                LED0_OFF;

                volatile uint32_t chunk_start = (uint32_t)buff;
                for(uint32_t x=0;x<(bin_length);x+=32)
                {
                    data.param1 = 0x08002000+x;
                    data.param2 = (*(__IO uint32_t *)(chunk_start+x));
                    data.param3 = (*(__IO uint32_t *)(chunk_start+x+4));
                    data.param4 = (*(__IO uint32_t *)(chunk_start+x+8));
                    data.param5 = (*(__IO uint32_t *)(chunk_start+x+12));
                    data.param6 = (*(__IO uint32_t *)(chunk_start+x+16));
                    data.param7 = (*(__IO uint32_t *)(chunk_start+x+20));
                    data.param8 = (*(__IO uint32_t *)(chunk_start+x+24));
                    data.param9 = (*(__IO uint32_t *)(chunk_start+x+28));

                    if( imuf9001SendReceiveCommand(imufDev, BL_WRITE_FIRMWARES, &reply, &data) )
                    {
                        //continue writing
                        LED0_TOGGLE;
                    }
                    else
                    {
                        //error handler
                        for(uint32_t x = 0; x<40; x++)
                        {
                            LED0_TOGGLE;
                            delay(200);
                        }
                        LED0_OFF;
                        return 0;
                    }
                }

                if( imuf9001SendReceiveCommand(imufDev, BL_END_PROGRAM, &reply, &data) )
                {
                    //blink
                    for(uint32_t x = 0; x<40; x++)
                    {
                        LED0_TOGGLE;
                        delay(20);
                    }
                    LED0_OFF;
                    return 1;
                }
            }
        }
    }
    
    return 0;
}

int imuf9001Whoami(const gyroDev_t *gyro)
{
    imufDev = (gyroDev_t *)gyro;
    uint32_t attempt;
    imufCommand_t reply;

    for (attempt = 0; attempt < 5; attempt++)
    {
        if (imuf9001SendReceiveCommand(gyro, IMUF_COMMAND_REPORT_INFO, &reply, NULL))
        {
            imufCurrentVersion = (*(imufVersion_t *)&(reply.param1)).firmware;
            if (imufCurrentVersion >= IMUF_FIRMWARE_MIN_VERSION) {
                return IMUF_9001_SPI;
            }
        }
    }
    imufCurrentVersion = 9999;
    return 0;
}

uint8_t imuf9001SpiDetect(const gyroDev_t *gyro)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return(0);
    }

    //making pancakes
    //config exti as input, not exti for now
    IOInit(IOGetByTag( IO_TAG(MPU_INT_EXTI) ), OWNER_MPU_EXTI, 0);
    IOConfigGPIO(IOGetByTag( IO_TAG(MPU_INT_EXTI) ), IOCFG_IPD);

    delayMicroseconds(100);

    IOInit(gyro->bus.busdev_u.spi.csnPin, OWNER_MPU_CS, 0);
    IOConfigGPIO(gyro->bus.busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(gyro->bus.busdev_u.spi.csnPin);

    hardwareInitialised = true;

    for (int x=0; x<3; x++)
    {
        int returnCheck;
        if (x)
        {
            initImuf9001();
            delay(200 * x);
        }

        returnCheck = imuf9001Whoami(gyro);
        if(returnCheck)
        {
            return returnCheck;
        }
    }
    return 0;
}

void imufSpiAccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

static gyroToBoardCommMode_t VerifyAllowedCommMode(uint32_t commMode)
{
    switch (commMode)
    {
        case GTBCM_SETUP:
        case GTBCM_GYRO_ONLY_PASSTHRU:
        case GTBCM_GYRO_ACC_PASSTHRU:
        case GTBCM_GYRO_ONLY_FILTER_F:
        case GTBCM_GYRO_ACC_FILTER_F:
        case GTBCM_GYRO_ACC_QUAT_FILTER_F:
            return (gyroToBoardCommMode_t)commMode;
            break;
        default:
            return GTBCM_DEFAULT;
    }
}

uint16_t imufGyroAlignment(void)
{
    if (isBoardAlignmentStandard(boardAlignment()))
    {
        if(gyroConfig()->gyro_align <= 1)
        {
            return 0;
        }
        else
        {
            return (uint16_t)(gyroConfig()->gyro_align - 1);
        }
    }
    else
    {
        return (uint16_t)IMU_CW0;
    }
}

void setupImufParams(imufCommand_t * data)
{
    if (imufCurrentVersion < 107) {
        //backwards compatibility for Caprica
        data->param2 = ( (uint16_t)(gyroConfig()->imuf_rate+1) << 16 );
        data->param3 = ( (uint16_t)gyroConfig()->imuf_pitch_q << 16 )            | (uint16_t)constrain(gyroConfig()->imuf_w, 6, 10);
        data->param4 = ( (uint16_t)gyroConfig()->imuf_roll_q << 16 )             | (uint16_t)constrain(gyroConfig()->imuf_w, 6, 10);
        data->param5 = ( (uint16_t)gyroConfig()->imuf_yaw_q << 16 )              | (uint16_t)constrain(gyroConfig()->imuf_w, 6, 10);
        data->param6 = ( (uint16_t)gyroConfig()->imuf_pitch_lpf_cutoff_hz << 16) | (uint16_t)gyroConfig()->imuf_roll_lpf_cutoff_hz;
        data->param7 = ( (uint16_t)gyroConfig()->imuf_yaw_lpf_cutoff_hz << 16)   | (uint16_t)0;
        data->param8 = ( (int16_t)boardAlignment()->rollDegrees << 16 )          | imufGyroAlignment();
        data->param9 = ( (int16_t)boardAlignment()->yawDegrees << 16 )           | (int16_t)boardAlignment()->pitchDegrees;
    } else {
        //Odin contract.
        data->param2 = ( (uint16_t)(gyroConfig()->imuf_rate+1) << 16)            | (uint16_t)gyroConfig()->imuf_w;
        data->param3 = ( (uint16_t)gyroConfig()->imuf_roll_q << 16)              | (uint16_t)gyroConfig()->imuf_pitch_q;
        data->param4 = ( (uint16_t)gyroConfig()->imuf_yaw_q << 16)               | (uint16_t)gyroConfig()->imuf_roll_lpf_cutoff_hz;
        data->param5 = ( (uint16_t)gyroConfig()->imuf_pitch_lpf_cutoff_hz << 16) | (uint16_t)gyroConfig()->imuf_yaw_lpf_cutoff_hz;
        data->param6 = ( (uint32_t)((gyroConfig()->imuf_roll_af & 1) | 
                                    (gyroConfig()->imuf_pitch_af & 1) << 1 | 
                                    (gyroConfig()->imuf_yaw_af & 1) << 2 ));
        data->param7 = ( (uint16_t)0 << 16)                                      | (uint16_t)0;
        data->param8 = ( (int16_t)boardAlignment()->rollDegrees << 16 )          | imufGyroAlignment();
        data->param9 = ( (int16_t)boardAlignment()->yawDegrees << 16 )           | (int16_t)boardAlignment()->pitchDegrees;
    }
}

void imufSpiGyroInit(gyroDev_t *gyro)
{
    uint32_t attempt = 0;
    imufCommand_t txData;
    imufCommand_t rxData;

    rxData.param1 = VerifyAllowedCommMode(gyroConfig()->imuf_mode);

    setupImufParams(&rxData);

    for (attempt = 0; attempt < 10; attempt++)
    {
        if(attempt)
        {
            resetImuf9001();
            delay(300 * attempt);
        }

        if (imuf9001SendReceiveCommand(gyro, IMUF_COMMAND_SETUP, &txData, &rxData))
        {
            //enable EXTI
            mpuGyroInit(gyro);
            return;
        }
    }
    setArmingDisabled(ARMING_DISABLED_NO_GYRO);
}

bool imufReadAccData(accDev_t *acc) {
    UNUSED(acc);
    return true;
}

bool imufSpiAccDetect(accDev_t *acc)
{
    acc->initFn = imufSpiAccInit;
    acc->readFn = imufReadAccData;

    return true;
}

bool imufSpiGyroDetect(gyroDev_t *gyro)
{
    // MPU6500 is used as a equivalent of other gyros by some flight controllers
    switch (gyro->mpuDetectionResult.sensor) {
    case IMUF_9001_SPI:
        break;
    default:
        return false;
    }

    gyro->initFn = imufSpiGyroInit;
    gyro->scale = 1.0f;
    gyro->mpuConfiguration.resetFn = resetImuf9001;
    return true;
}

void imufStartCalibration(void)
{
    isImufCalibrating = IMUF_IS_CALIBRATING; //reset by EXTI
}

void imufEndCalibration(void)
{
    isImufCalibrating = IMUF_NOT_CALIBRATING; //reset by EXTI
}

#endif