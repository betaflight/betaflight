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

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

#ifdef USE_I2C_DEVICE_1
void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cDevice[I2CDEV_1].handle);
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cDevice[I2CDEV_1].handle);
}
#endif

#ifdef USE_I2C_DEVICE_2
void I2C2_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cDevice[I2CDEV_2].handle);
}

void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cDevice[I2CDEV_2].handle);
}
#endif

#ifdef USE_I2C_DEVICE_3
void I2C3_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cDevice[I2CDEV_3].handle);
}

void I2C3_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cDevice[I2CDEV_3].handle);
}
#endif

#ifdef USE_I2C_DEVICE_4
void I2C4_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cDevice[I2CDEV_4].handle);
}

void I2C4_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cDevice[I2CDEV_4].handle);
}
#endif

static volatile uint16_t i2cErrorCount = 0;

static bool i2cHandleHardwareFailure(I2CDevice device)
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

// Blocking write
bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cDevice[device].handle;

    if (!pHandle->Instance) {
        return false;
    }

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF)
        status = HAL_I2C_Master_Transmit(pHandle ,addr_ << 1, &data, 1, I2C_TIMEOUT_SYS_TICKS);
    else
        status = HAL_I2C_Mem_Write(pHandle ,addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_SYS_TICKS);

    if (status != HAL_OK)
        return i2cHandleHardwareFailure(device);

    return true;
}

// Non-blocking write
bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cDevice[device].handle;

    if (!pHandle->Instance) {
        return false;
    }

    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write_IT(pHandle ,addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT,data, len_);

    if (status == HAL_BUSY) {
        return false;
    }

    if (status != HAL_OK)
    {
        return i2cHandleHardwareFailure(device);
    }

    return true;
}

// Blocking read
bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cDevice[device].handle;

    if (!pHandle->Instance) {
        return false;
    }

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF)
        status = HAL_I2C_Master_Receive(pHandle ,addr_ << 1, buf, len, I2C_TIMEOUT_SYS_TICKS);
    else
        status = HAL_I2C_Mem_Read(pHandle, addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT,buf, len, I2C_TIMEOUT_SYS_TICKS);

    if (status != HAL_OK) {
        return i2cHandleHardwareFailure(device);
    }

    return true;
}

// Non-blocking read
bool i2cReadBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cDevice[device].handle;

    if (!pHandle->Instance) {
        return false;
    }

    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read_IT(pHandle, addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT, buf, len);

    if (status == HAL_BUSY) {
        return false;
    }

    if (status != HAL_OK) {
        return i2cHandleHardwareFailure(device);
    }

    return true;
}

bool i2cBusy(I2CDevice device, bool *error)
{
    I2C_HandleTypeDef *pHandle = &i2cDevice[device].handle;

    if (error) {
        *error = pHandle->ErrorCode;
    }

    if (pHandle->State == HAL_I2C_STATE_READY)
    {
        if (__HAL_I2C_GET_FLAG(pHandle, I2C_FLAG_BUSY) == SET)
        {
            return true;
        }

        return false;
    }

    return true;
}

#endif
