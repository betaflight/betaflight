/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
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
static volatile timeUs_t i2cLastActivityUs[I2CDEV_COUNT];

static bool i2cHandleHardwareFailure(i2cDevice_e device)
{
    (void)device;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    i2cErrorCount++;
    __set_PRIMASK(primask);
    return false;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

static I2C_HandleTypeDef *i2cGetHandle(i2cDevice_e device)
{
    if ((unsigned)device >= I2CDEV_COUNT) {
        return NULL;
    }

    i2cHalHandle_t *handle = i2cDevice[device].halHandle;
    return handle && handle->hal.Instance ? &handle->hal : NULL;
}

static void i2cCompleteTransfer(i2cDevice_e device, I2C_HandleTypeDef *pHandle, uint32_t error)
{
    __HAL_I2C_EX_DISABLE_IT(pHandle, I2C_EX_IT_MASK_ALL);
    __HAL_I2C_EX_DISABLE(pHandle);
    __HAL_I2C_EX_CLEAR_ALL_ITFLAG(pHandle);
    pHandle->ErrorCode = error;
    pHandle->Mode = HAL_I2C_EX_MODE_NONE;
    pHandle->State = HAL_I2C_EX_STATE_READY;
    if (error != HAL_I2C_EX_ERROR_NONE) {
        i2cHandleHardwareFailure(device);
    }
}

static void i2cIrqHandler(i2cDevice_e device)
{
    I2C_HandleTypeDef *pHandle = i2cGetHandle(device);
    if (!pHandle || pHandle->Mode != HAL_I2C_EX_MODE_MEM) {
        return;
    }

    I2C_TypeDef *I2Cx = pHandle->Instance;
    const uint32_t flags = I2Cx->ISR;
    if (flags & I2C_EX_IT_TX_ABRT) {
        // An abort flushes both FIFOs. It must not be reported as a successful STOP.
        i2cCompleteTransfer(device, pHandle, HAL_I2C_EX_ERROR_BERR);
        return;
    }
    if (flags & (I2C_EX_IT_RX_OVER | I2C_EX_IT_RX_UNDER | I2C_EX_IT_TX_OVER)) {
        i2cCompleteTransfer(device, pHandle, HAL_I2C_EX_ERROR_OVR);
        return;
    }

    // Only one read command is outstanding. Read the last byte before handling STOP.
    if ((flags & I2C_EX_IT_RX_FULL) && pHandle->State == HAL_I2C_EX_STATE_BUSY_RX) {
        if (!pHandle->XferCount) {
            i2cCompleteTransfer(device, pHandle, HAL_I2C_EX_ERROR_SIZE);
            return;
        }
        *pHandle->pBuffPtr++ = I2Cx->DATACMD;
        pHandle->XferCount--;
        i2cLastActivityUs[device] = microsISR();
        if (!(flags & I2C_EX_IT_STOP_DET) && pHandle->XferCount) {
            I2Cx->DATACMD = I2C_EX_CMD_READ | (pHandle->XferCount == 1 ? I2C_EX_CMD_STOP : 0);
        }
    }

    if (flags & I2C_EX_IT_STOP_DET) {
        i2cCompleteTransfer(device, pHandle, pHandle->XferCount ? HAL_I2C_EX_ERROR_SIZE : HAL_I2C_EX_ERROR_NONE);
        return;
    }

    if ((flags & I2C_EX_IT_TX_EMPTY) && pHandle->State == HAL_I2C_EX_STATE_BUSY_TX && pHandle->XferCount) {
        I2Cx->DATACMD = *pHandle->pBuffPtr++ | (pHandle->XferCount == 1 ? I2C_EX_CMD_STOP : 0);
        pHandle->XferCount--;
        i2cLastActivityUs[device] = microsISR();
        if (!pHandle->XferCount) {
            __HAL_I2C_EX_DISABLE_IT(pHandle, I2C_EX_IT_TX_EMPTY);
        }
    }
}

FAST_IRQ_HANDLER void I2C1_IRQHandler(void)
{
    i2cIrqHandler(I2CDEV_1);
}

FAST_IRQ_HANDLER void I2C2_IRQHandler(void)
{
    i2cIrqHandler(I2CDEV_2);
}

FAST_IRQ_HANDLER void I2C3_IRQHandler(void)
{
    i2cIrqHandler(I2CDEV_3);
}

// Blocking write
bool i2cWrite(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
#if I2C_TRAIT_HANDLE
    I2C_HandleTypeDef *pHandle = i2cGetHandle(device);

    if (!pHandle) {
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

static bool i2cStartTransfer(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data, bool read)
{
    I2C_HandleTypeDef *pHandle = i2cGetHandle(device);
    if (!pHandle || !data || !len || addr > 0x7F) {
        return false;
    }

    // BASEPRI cannot mask the I2C IRQ at priority zero.
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (pHandle->State != HAL_I2C_EX_STATE_READY || __HAL_I2C_EX_GET_FLAG(pHandle, I2C_EX_FLAG_ACTIVITY) == SET) {
        __set_PRIMASK(primask);
        return false;
    }

    I2C_TypeDef *I2Cx = pHandle->Instance;
    __HAL_I2C_EX_DISABLE_IT(pHandle, I2C_EX_IT_MASK_ALL);
    __HAL_I2C_EX_DISABLE(pHandle);
    __HAL_I2C_EX_CLEAR_ALL_ITFLAG(pHandle);
    MODIFY_REG(I2Cx->TAR, I2C_EX_TAR_ADDR, addr);
    I2Cx->TXTL = 0;
    I2Cx->RXTL = 0;

    pHandle->pBuffPtr = data;
    pHandle->XferSize = len;
    pHandle->XferCount = len;
    pHandle->ErrorCode = HAL_I2C_EX_ERROR_NONE;
    pHandle->Mode = HAL_I2C_EX_MODE_MEM;
    pHandle->State = read ? HAL_I2C_EX_STATE_BUSY_RX : HAL_I2C_EX_STATE_BUSY_TX;
    i2cLastActivityUs[device] = microsISR();

    __HAL_I2C_EX_ENABLE(pHandle);
    // UM324xF RM 26.5.4: keep the controller enabled between register and data.
    // Changing from write to read generates a repeated START with RESTART_EN set.
    I2Cx->DATACMD = reg | I2C_EX_CMD_WRITE;
    if (read) {
        I2Cx->DATACMD = I2C_EX_CMD_READ | (len == 1 ? I2C_EX_CMD_STOP : 0);
    }
    __HAL_I2C_EX_ENABLE_IT(pHandle, I2C_EX_IT_TX_ABRT | I2C_EX_IT_RX_OVER | I2C_EX_IT_RX_UNDER
        | I2C_EX_IT_TX_OVER | I2C_EX_IT_STOP_DET | (read ? I2C_EX_IT_RX_FULL : I2C_EX_IT_TX_EMPTY));
    __set_PRIMASK(primask);

    return true;
}

// Non-blocking register write
bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    return i2cStartTransfer(device, addr_, reg_, len_, data, false);
}

// Blocking read
bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
#if I2C_TRAIT_HANDLE
    I2C_HandleTypeDef *pHandle = i2cGetHandle(device);

    if (!pHandle) {
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
    return i2cStartTransfer(device, addr_, reg_, len, buf, true);
}

bool i2cBusy(i2cDevice_e device, bool *error)
{
#if I2C_TRAIT_HANDLE
    I2C_HandleTypeDef *pHandle = i2cGetHandle(device);
    if (!pHandle) {
        if (error) {
            *error = true;
        }
        return true;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (pHandle->Mode == HAL_I2C_EX_MODE_MEM && pHandle->State != HAL_I2C_EX_STATE_READY
        && cmpTimeUs(microsISR(), i2cLastActivityUs[device]) >= I2C_TIMEOUT_US) {
        i2cCompleteTransfer(device, pHandle, HAL_I2C_EX_ERROR_TIMEOUT);
    }

    if (error) {
        *error = pHandle->ErrorCode != HAL_I2C_EX_ERROR_NONE;
    }

    const bool busy = pHandle->State != HAL_I2C_EX_STATE_READY
        || (pHandle->ErrorCode == HAL_I2C_EX_ERROR_NONE && __HAL_I2C_EX_GET_FLAG(pHandle, I2C_EX_FLAG_ACTIVITY) == SET);
    __set_PRIMASK(primask);
    return busy;
#endif

    return true;
}

#endif
