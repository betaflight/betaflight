/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(USE_SOFT_I2C)

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "platform/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

#if I2C_TRAIT_HANDLE
#include "platform/bus_i2c_hal.h"
#endif


static volatile uint16_t i2cErrorCount = 0;

static bool i2cHandleHardwareFailure(i2cDevice_e device)
{
    (void)device;
    i2cErrorCount++;
    // reinit peripheral + clock out garbage
    //i2cInit(device);
    return false;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}


FAST_IRQ_HANDLER void I2C1_IRQHandler(void)
{
    HAL_I2C_EX_IRQHandler(&i2cDevice[I2CDEV_1].halHandle->hal);
}

FAST_IRQ_HANDLER void I2C2_IRQHandler(void)
{
    HAL_I2C_EX_IRQHandler(&i2cDevice[I2CDEV_2].halHandle->hal);
}

FAST_IRQ_HANDLER void I2C3_IRQHandler(void)
{
    HAL_I2C_EX_IRQHandler(&i2cDevice[I2CDEV_3].halHandle->hal);
}

// Blocking write
bool i2cWrite(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

#if I2C_TRAIT_HANDLE
    I2C_HandleTypeDef *pHandle = &i2cDevice[device].halHandle->hal;

    if (!pHandle->Instance) {
        return false;
    }
   
    HAL_StatusTypeDef status;

    if (reg_ == 0xFF)
        status = HAL_I2C_EX_Master_Transmit(pHandle ,addr_, &data, 1, I2C_TIMEOUT_SYS_TICKS);
    else
        status = HAL_I2C_EX_Mem_Write(pHandle ,addr_ , reg_, I2C_EX_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_SYS_TICKS);
    if (status != HAL_OK)
         return i2cHandleHardwareFailure(device);
#endif
 
    return true;
}

// Non-blocking write
bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

#if I2C_TRAIT_HANDLE
    I2C_HandleTypeDef *pHandle = &i2cDevice[device].halHandle->hal;

    if (!pHandle->Instance) {
        return false;
    }
    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }
    if (pHandle->State != HAL_I2C_EX_STATE_READY) {
        return false;
    }
    
    HAL_StatusTypeDef status;

    timeUs_t timeoutStartUs = microsISR();

    /* Disable I2C_EX peripheral */
    __HAL_I2C_EX_DISABLE(pHandle);

    MODIFY_REG(I2Cx->TAR, I2C_EX_TAR_ADDR, (addr_ & 0x7FUL));

   /* Enable I2C_EX peripheral */
	__HAL_I2C_EX_ENABLE(pHandle);	
    
    I2Cx->DATACMD = (reg_ | I2C_EX_CMD_WRITE);	
    while ((I2Cx->RAWISR & I2C_EX_RAW_ITFLAG_TX_EMPTY) == 0) {                          // wait for any stop to finish sending
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device);
        }
    }

    status = HAL_I2C_EX_Master_Transmit_IT(pHandle ,addr_, data, len_);
    
    if (status == HAL_BUSY) {
        return false;
    }

    if (status != HAL_OK)
    {
        return i2cHandleHardwareFailure(device);
    }
#endif

    return true;
}

// Blocking read
bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

#if I2C_TRAIT_HANDLE
    I2C_HandleTypeDef *pHandle = &i2cDevice[device].halHandle->hal;

    if (!pHandle->Instance) {
        return false;
    }

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF)
        status = HAL_I2C_EX_Master_Receive(pHandle ,addr_ , buf, len, I2C_TIMEOUT_SYS_TICKS);
    else
        status = HAL_I2C_EX_Mem_Read(pHandle, addr_ , reg_, I2C_EX_MEMADD_SIZE_8BIT,buf, len, I2C_TIMEOUT_SYS_TICKS);
    if (status != HAL_OK) {
        return i2cHandleHardwareFailure(device);
    }
#endif

    return true;
}

// Non-blocking read
bool i2cReadBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

#if I2C_TRAIT_HANDLE
    I2C_HandleTypeDef *pHandle = &i2cDevice[device].halHandle->hal;

    if (!pHandle->Instance) {
        return false;
    }
    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    if (pHandle->State != HAL_I2C_EX_STATE_READY) {
        return false;
    }

    HAL_StatusTypeDef status;

    timeUs_t timeoutStartUs = microsISR();

    /* Disable I2C_EX peripheral */
    __HAL_I2C_EX_DISABLE(pHandle);

    MODIFY_REG(I2Cx->TAR, I2C_EX_TAR_ADDR, (addr_ & 0x7FUL));

   /* Enable I2C_EX peripheral */
	__HAL_I2C_EX_ENABLE(pHandle);	
    
    I2Cx->DATACMD = (reg_ | I2C_EX_CMD_WRITE);	

     while ((I2Cx->RAWISR & I2C_EX_RAW_ITFLAG_TX_EMPTY) == 0) {                          // wait for any stop to finish sending
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device);
        }
    }
   
    status = HAL_I2C_EX_Master_Receive_IT(pHandle, addr_ , buf, len);
 
    if (status == HAL_BUSY) {
        return false;
    }

    if (status != HAL_OK) {
        return i2cHandleHardwareFailure(device);
    }
#endif

    return true;
}

bool i2cBusy(i2cDevice_e device, bool *error)
{
#if I2C_TRAIT_HANDLE
    I2C_HandleTypeDef *pHandle = &i2cDevice[device].halHandle->hal;

    if (error) {
        *error = pHandle->ErrorCode;
    }

    if (pHandle->State == HAL_I2C_EX_STATE_READY)
    {
        if (__HAL_I2C_EX_GET_FLAG(pHandle, I2C_EX_FLAG_ACTIVITY) == SET)  //?? Need to check.
        {
            return true;
        }

        return false;
    }
#endif

    return true;
}

#endif
