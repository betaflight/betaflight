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
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(USE_SOFT_I2C)

#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_i2c_timing.h"
#include "drivers/bus_i2c_utils.h"

static void i2c_er_handler(i2cDevice_e device);
static void i2c_ev_handler(i2cDevice_e device);

#define IOCFG_I2C_PU IO_CONFIG(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_LOW, GPIO_PULL_UP)
#define IOCFG_I2C    IO_CONFIG(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_LOW, GPIO_NO_PULL)

#define I2C_CLOCK_DIVIDER 4

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    [I2CDEV_1] = {
        .device = I2CDEV_1,
        .reg = (i2cResource_t *)I2C1,
        .sclPins = {
            I2CPINDEF(PG5, GPIO_AF10),
            I2CPINDEF(PD4, GPIO_AF9),
            I2CPINDEF(PB6, GPIO_AF11),
            I2CPINDEF(PB8, GPIO_AF11),
        },
        .sdaPins = {
            I2CPINDEF(PG4, GPIO_AF9),
            I2CPINDEF(PD5, GPIO_AF8),
            I2CPINDEF(PB7, GPIO_AF10),
            I2CPINDEF(PB9, GPIO_AF10),
        },
        .ev_irq = I2C1_EV_IRQn,
        .er_irq = I2C1_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    [I2CDEV_2] = {
        .device = I2CDEV_2,
        .reg = (i2cResource_t *)I2C2,
        .sclPins = {
            I2CPINDEF(PF1, GPIO_AF11),
            I2CPINDEF(PH4, GPIO_AF9),
            I2CPINDEF(PB10, GPIO_AF10),
        },
        .sdaPins = {
            I2CPINDEF(PF0, GPIO_AF10),
            I2CPINDEF(PH5, GPIO_AF9),
            I2CPINDEF(PB11, GPIO_AF7),
        },
        .ev_irq = I2C2_EV_IRQn,
        .er_irq = I2C2_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    [I2CDEV_3] = {
        .device = I2CDEV_3,
        .reg = (i2cResource_t *)I2C3,
        .sclPins = {
            I2CPINDEF(PA8, GPIO_AF9),
            I2CPINDEF(PH7, GPIO_AF9),
        },
        .sdaPins = {
            I2CPINDEF(PC9, GPIO_AF12),
            I2CPINDEF(PH8, GPIO_AF11),
        },
        .ev_irq = I2C3_EV_IRQn,
        .er_irq = I2C3_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_4
    [I2CDEV_4] = {
        .device = I2CDEV_4,
        .reg = (i2cResource_t *)I2C4,
        .sclPins = {
            I2CPINDEF(PB6, GPIO_AF12),
            I2CPINDEF(PB8, GPIO_AF12),
            I2CPINDEF(PD12, GPIO_AF10),
            I2CPINDEF(PF14, GPIO_AF8),
            I2CPINDEF(PH11, GPIO_AF8),
        },
        .sdaPins = {
            I2CPINDEF(PB7, GPIO_AF11),
            I2CPINDEF(PB9, GPIO_AF11),
            I2CPINDEF(PD13, GPIO_AF8),
            I2CPINDEF(PF15, GPIO_AF9),
            I2CPINDEF(PH12, GPIO_AF10),
        },
        .ev_irq = I2C4_EV_IRQn,
        .er_irq = I2C4_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_5
    [I2CDEV_5] = {
        .device = I2CDEV_5,
        .reg = (i2cResource_t *)I2C5,
        .sclPins = {
            I2CPINDEF(PA8, GPIO_AF10),
            I2CPINDEF(PC11, GPIO_AF9),
            I2CPINDEF(PF1, GPIO_AF12),
        },
        .sdaPins = {
            I2CPINDEF(PC9, GPIO_AF13),
            I2CPINDEF(PC10, GPIO_AF9),
            I2CPINDEF(PF0, GPIO_AF11),
        },
        .ev_irq = I2C5_EV_IRQn,
        .er_irq = I2C5_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_6
    [I2CDEV_6] = {
        .device = I2CDEV_6,
        .reg = (i2cResource_t *)I2C6,
        .sclPins = {
            I2CPINDEF(PE14, GPIO_AF10),
            I2CPINDEF(PF4, GPIO_AF12),
            I2CPINDEF(PF12, GPIO_AF7),
        },
        .sdaPins = {
            I2CPINDEF(PE13, GPIO_AF8),
            I2CPINDEF(PF3, GPIO_AF11),
            I2CPINDEF(PF11, GPIO_AF5),
        },
        .ev_irq = I2C6_EV_IRQn,
        .er_irq = I2C6_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_7
    [I2CDEV_7] = {
        .device = I2CDEV_7,
        .reg = (i2cResource_t *)I2C7,
        .sclPins = {
            I2CPINDEF(PD9, GPIO_AF6),
            I2CPINDEF(PE8, GPIO_AF9),
            I2CPINDEF(PI13, GPIO_AF7),
        },
        .sdaPins = {
            I2CPINDEF(PD8, GPIO_AF7),
            I2CPINDEF(PE7, GPIO_AF8),
            I2CPINDEF(PI12, GPIO_AF7),
        },
        .ev_irq = I2C7_EV_IRQn,
        .er_irq = I2C7_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_8
    [I2CDEV_8] = {
        .device = I2CDEV_8,
        .reg = (i2cResource_t *)I2C8,
        .sclPins = {
            I2CPINDEF(PG5, GPIO_AF11),
            I2CPINDEF(PD15, GPIO_AF9),
            I2CPINDEF(PI3, GPIO_AF10),
            I2CPINDEF(PJ9, GPIO_AF9),
        },
        .sdaPins = {
            I2CPINDEF(PG4, GPIO_AF10),
            I2CPINDEF(PD14, GPIO_AF9),
            I2CPINDEF(PI2, GPIO_AF10),
            I2CPINDEF(PJ8, GPIO_AF9),
        },
        .ev_irq = I2C8_EV_IRQn,
        .er_irq = I2C8_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_9
    [I2CDEV_9] = {
        .device = I2CDEV_9,
        .reg = (i2cResource_t *)I2C9,
        .sclPins = {
            I2CPINDEF(PH15, GPIO_AF9),
            I2CPINDEF(PI11, GPIO_AF7),
            I2CPINDEF(PJ1, GPIO_AF9),
            I2CPINDEF(PJ4, GPIO_AF8),
        },
        .sdaPins = {
            I2CPINDEF(PH14, GPIO_AF10),
            I2CPINDEF(PI10, GPIO_AF9),
            I2CPINDEF(PJ0, GPIO_AF8),
            I2CPINDEF(PJ3, GPIO_AF9),
        },
        .ev_irq = I2C9_EV_IRQn,
        .er_irq = I2C9_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_10
    [I2CDEV_10] = {
        .device = I2CDEV_10,
        .reg = (i2cResource_t *)I2C10,
        .sclPins = {
            I2CPINDEF(PI1, GPIO_AF11),
            I2CPINDEF(PJ14, GPIO_AF4),
            I2CPINDEF(PI6, GPIO_AF9),
        },
        .sdaPins = {
            I2CPINDEF(PI0, GPIO_AF8),
            I2CPINDEF(PJ13, GPIO_AF6),
            I2CPINDEF(PI5, GPIO_AF10),
        },
        .ev_irq = I2C10_EV_IRQn,
        .er_irq = I2C10_ER_IRQn,
    },
#endif
};


i2cDevice_t i2cDevice[I2CDEV_COUNT];

// State used by event handler ISR
typedef struct {
    uint8_t subaddress_sent;    // flag to indicate if subaddess sent
    uint8_t final_stop;         // flag to indicate final bus condition
    int8_t index;               // index is signed -1 == send the subaddress
} i2cEvState_t;
static i2cEvState_t i2c_ev_state[I2CDEV_COUNT];

static volatile uint16_t i2cErrorCount = 0;

#define I2C_IRQ_HANDLERS(instance, device) \
    void I2C ## instance ## _ER_IRQHandler(void) \
    { \
        i2c_er_handler(device); \
    } \
    void I2C ## instance ## _EV_IRQHandler(void) \
    { \
        i2c_ev_handler(device); \
    }

#ifdef USE_I2C_DEVICE_1
I2C_IRQ_HANDLERS(1, I2CDEV_1)
#endif
#ifdef USE_I2C_DEVICE_2
I2C_IRQ_HANDLERS(2, I2CDEV_2)
#endif
#ifdef USE_I2C_DEVICE_3
I2C_IRQ_HANDLERS(3, I2CDEV_3)
#endif
#ifdef USE_I2C_DEVICE_4
I2C_IRQ_HANDLERS(4, I2CDEV_4)
#endif
#ifdef USE_I2C_DEVICE_5
I2C_IRQ_HANDLERS(5, I2CDEV_5)
#endif
#ifdef USE_I2C_DEVICE_6
I2C_IRQ_HANDLERS(6, I2CDEV_6)
#endif
#ifdef USE_I2C_DEVICE_7
I2C_IRQ_HANDLERS(7, I2CDEV_7)
#endif
#ifdef USE_I2C_DEVICE_8
I2C_IRQ_HANDLERS(8, I2CDEV_8)
#endif
#ifdef USE_I2C_DEVICE_9
I2C_IRQ_HANDLERS(9, I2CDEV_9)
#endif
#ifdef USE_I2C_DEVICE_10
I2C_IRQ_HANDLERS(10, I2CDEV_10)
#endif

static void i2cClockEnable(I2C_TypeDef *I2Cx)
{
    if (I2Cx == I2C1) {
        RCC_EnableAPB1PeriphClk4(RCC_APB1_PERIPHEN_M7_I2C1, ENABLE);
        RCC_ConfigI2C1_3_KerSysDivider(RCC_I2CKERCLK_SYSBUSDIV4);
        RCC_ConfigI2C1KerClkSource(RCC_I2CKERCLK_SRC_SYSBUSDIV);
    } else if (I2Cx == I2C2) {
        RCC_EnableAPB1PeriphClk4(RCC_APB1_PERIPHEN_M7_I2C2, ENABLE);
        RCC_ConfigI2C1_3_KerSysDivider(RCC_I2CKERCLK_SYSBUSDIV4);
        RCC_ConfigI2C2KerClkSource(RCC_I2CKERCLK_SRC_SYSBUSDIV);
    } else if (I2Cx == I2C3) {
        RCC_EnableAPB1PeriphClk4(RCC_APB1_PERIPHEN_M7_I2C3, ENABLE);
        RCC_ConfigI2C1_3_KerSysDivider(RCC_I2CKERCLK_SYSBUSDIV4);
        RCC_ConfigI2C3KerClkSource(RCC_I2CKERCLK_SRC_SYSBUSDIV);
    } else if (I2Cx == I2C4) {
        RCC_EnableAPB2PeriphClk2(RCC_APB2_PERIPHEN_M7_I2C4, ENABLE);
        RCC_ConfigI2C4_6_KerSysDivider(RCC_I2CKERCLK_SYSBUSDIV4);
        RCC_ConfigI2C4KerClkSource(RCC_I2CKERCLK_SRC_SYSBUSDIV);
    } else if (I2Cx == I2C5) {
        RCC_EnableAPB2PeriphClk2(RCC_APB2_PERIPHEN_M7_I2C5, ENABLE);
        RCC_ConfigI2C4_6_KerSysDivider(RCC_I2CKERCLK_SYSBUSDIV4);
        RCC_ConfigI2C5KerClkSource(RCC_I2CKERCLK_SRC_SYSBUSDIV);
    } else if (I2Cx == I2C6) {
        RCC_EnableAPB2PeriphClk2(RCC_APB2_PERIPHEN_M7_I2C6, ENABLE);
        RCC_ConfigI2C4_6_KerSysDivider(RCC_I2CKERCLK_SYSBUSDIV4);
        RCC_ConfigI2C6KerClkSource(RCC_I2CKERCLK_SRC_SYSBUSDIV);
    } else if (I2Cx == I2C7) {
        RCC_EnableAPB5PeriphClk2(RCC_APB5_PERIPHEN_M7_I2C7, ENABLE);
        RCC_ConfigI2C7_10_KerSysDivider(RCC_I2CKERCLK_SYSBUSDIV4);
        RCC_ConfigI2C7KerClkSource(RCC_I2CKERCLK_SRC_SYSBUSDIV);
    } else if (I2Cx == I2C8) {
        RCC_EnableAPB5PeriphClk2(RCC_APB5_PERIPHEN_M7_I2C8, ENABLE);
        RCC_ConfigI2C7_10_KerSysDivider(RCC_I2CKERCLK_SYSBUSDIV4);
        RCC_ConfigI2C8KerClkSource(RCC_I2CKERCLK_SRC_SYSBUSDIV);
    } else if (I2Cx == I2C9) {
        RCC_EnableAPB5PeriphClk2(RCC_APB5_PERIPHEN_M7_I2C9, ENABLE);
        RCC_ConfigI2C7_10_KerSysDivider(RCC_I2CKERCLK_SYSBUSDIV4);
        RCC_ConfigI2C9KerClkSource(RCC_I2CKERCLK_SRC_SYSBUSDIV);
    } else if (I2Cx == I2C10) {
        RCC_EnableAPB5PeriphClk2(RCC_APB5_PERIPHEN_M7_I2C10, ENABLE);
        RCC_ConfigI2C7_10_KerSysDivider(RCC_I2CKERCLK_SYSBUSDIV4);
        RCC_ConfigI2C10KerClkSource(RCC_I2CKERCLK_SRC_SYSBUSDIV);
    }
}


static bool i2cHandleHardwareFailure(i2cDevice_e device)
{
    i2cErrorCount++;
    // reinit peripheral + clock out garbage
    i2cInit(device);
    return false;
}

bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;
    if (state->busy) {
        return false;
    }

    timeUs_t timeoutStartUs = microsISR();

    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 1;
    state->reading = 0;
    state->write_p = data;
    state->read_p = data;
    state->bytes = len_;
    state->busy = 1;
    state->error = false;

    i2c_ev_state[device].subaddress_sent = 1;// need tx reg address 
    i2c_ev_state[device].index = 0;
    i2c_ev_state[device].final_stop = 0;


    if (!(I2Cx->CTRL2 & I2C_CTRL2_START)) {                             // ensure sending a start
        while (I2Cx->CTRL2 & I2C_CTRL2_STOP) {                          // wait for any stop to finish sending
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device);
            }
        }
        I2C_ConfigInt(I2Cx, I2C_INT_ERR|I2C_INT_STOP|I2C_INT_RDR|I2C_INT_WDR|I2C_INT_TFC|I2C_INT_NACK, ENABLE);

        I2C_ConfigSendAddress(I2Cx, state->addr, I2C_DIRECTION_SEND);
        /* configure number of bytes to be transferred */
        I2C_SetTransferByteNumber(I2Cx, (state->bytes + 1));
        // while(I2C_GetFlag(I2Cx, I2C_FLAG_BUSY)) 
        // {
        // }
        I2C_GenerateStart(I2Cx, ENABLE);                            // send the start for the new job
    }
    
    return true;
}

bool i2cBusy(i2cDevice_e device, bool *error)
{
    if(device == I2CINVALID || device >= I2CDEV_COUNT) {
        if (error) {
            *error = true;
        }
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;

    if (error) {
        *error = state->error;
    }
    return state->busy;
}

static bool i2cWait(i2cDevice_e device)
{
    i2cState_t *state = &i2cDevice[device].state;
    timeUs_t timeoutStartUs = microsISR();

    while (state->busy) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device) && i2cWait(device);
        }
    }

    return !(state->error);
}

bool i2cWrite(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data) && i2cWait(device);
}

bool i2cReadBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].reg;
    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;
    if (state->busy) {
        return false;
    }

    timeUs_t timeoutStartUs = microsISR();

    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 0;
    state->reading = 1;
    state->read_p = buf;
    state->write_p = buf;
    state->bytes = len;
    state->busy = 1;
    state->error = false;

    i2c_ev_state[device].subaddress_sent = 1;// need tx reg address 
    i2c_ev_state[device].index = 0;
    i2c_ev_state[device].final_stop = 0;

    if (!(I2Cx->CTRL2 & I2C_CTRL2_START)) {                             // ensure sending a start
        while (I2Cx->CTRL2 & I2C_CTRL2_STOP) {                          // wait for any stop to finish sending
            if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                return i2cHandleHardwareFailure(device);
            }
        }
        I2C_ConfigInt(I2Cx, I2C_INT_ERR|I2C_INT_STOP|I2C_INT_RDR|I2C_INT_WDR|I2C_INT_TFC|I2C_INT_NACK, ENABLE);

        I2C_ConfigSendAddress(I2Cx, state->addr, I2C_DIRECTION_SEND);
        /* configure number of bytes to be transferred */
        I2C_SetTransferByteNumber(I2Cx, 1);
        // while(I2C_GetFlag(I2Cx, I2C_FLAG_BUSY)) 
        // {
        // }
        I2C_GenerateStart(I2Cx, ENABLE);                            // send the start for the new job
    }
    return true;
}

bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    return i2cReadBuffer(device, addr_, reg_, len, buf) && i2cWait(device);
}

static void i2c_er_handler(i2cDevice_e device)
{
    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].hardware->reg;

    i2cState_t *state = &i2cDevice[device].state;

    // Read the I2C1 status register
    volatile uint32_t SR1Register = I2Cx->STSINT;

    if (SR1Register & (I2C_FLAG_BSER | I2C_FLAG_ABLO | I2C_FLAG_OVF | I2C_FLAG_CRCERR | I2C_FLAG_TMOUT | I2C_FLAG_ALRT)) // an error
        state->error = true;

    if(I2C_GetFlag(I2Cx, I2C_FLAG_BSER)) 
    {
        I2C_ClrFlag(I2Cx, I2C_FLAG_BSER);
    }

    /* arbitration lost */
    if(I2C_GetFlag(I2Cx, I2C_FLAG_ABLO)) 
    {
        I2C_ClrFlag(I2Cx, I2C_FLAG_ABLO);
    }

    /* over-run or under-run when SCL stretch is disabled */
    if(I2C_GetFlag(I2Cx, I2C_FLAG_OVF)) 
    {
        I2C_ClrFlag(I2Cx, I2C_FLAG_OVF);
    }

    /* PEC error */
    if(I2C_GetFlag(I2Cx, I2C_FLAG_CRCERR)) 
    {
        I2C_ClrFlag(I2Cx, I2C_FLAG_CRCERR);
    }

    /* timeout error */
    if(I2C_GetFlag(I2Cx, I2C_FLAG_TMOUT)) 
    {
        I2C_ClrFlag(I2Cx, I2C_FLAG_TMOUT);
    }

    /* SMBus alert */
    if(I2C_GetFlag(I2Cx, I2C_FLAG_ALRT)) 
    {
        I2C_ClrFlag(I2Cx, I2C_FLAG_ALRT);
    }

    I2Cx->INTCLR |= (I2C_FLAG_BSER | I2C_FLAG_ABLO | I2C_FLAG_OVF | I2C_FLAG_CRCERR | I2C_FLAG_TMOUT | I2C_FLAG_ALRT);     // reset all the error bits to clear the interrupt
    state->busy = 0;
}

static void i2c_ev_handler(i2cDevice_e device)
{
    I2C_TypeDef *I2Cx = (I2C_TypeDef *)i2cDevice[device].hardware->reg;

    i2cEvState_t *ev_state = &i2c_ev_state[device];
    i2cState_t *state = &i2cDevice[device].state;

    uint32_t SReg_1 = I2Cx->STSINT;                                                

    if (SReg_1 & I2C_FLAG_STOPF) 
    {                                  // Byte transmitted EV8 / EV8_1
        /* clear STOP flag */
        I2C_ClrFlag(I2Cx, I2C_FLAG_STOPF);
        state->busy = 0;
    }
    else if (SReg_1 & I2C_FLAG_RDAVL) 
    {                                 // Byte received - EV7
        state->read_p[ev_state->index++] = I2C_RecvData(I2Cx);
        if (ev_state->index == state->bytes) 
        {
            ev_state->final_stop = 1;
        }
    }
    else if (SReg_1 & I2C_FLAG_WRAVL) 
    {                                  
        if (ev_state->subaddress_sent == 1) 
        {  
            I2C_SendData(I2Cx, state->reg);                                            
            if (state->writing == 1)
            {
                ev_state->subaddress_sent = 0;
            }
        }
        else 
        {
            I2C_SendData(I2Cx, state->write_p[ev_state->index++]);
        }
    }
    else if (SReg_1 & I2C_FLAG_TFC) 
    {                                  
        if ((ev_state->subaddress_sent == 1) &&(state->reading == 1))
        {  
            ev_state->subaddress_sent = 0;
            ev_state->final_stop = 0;
            I2C_ConfigSendAddress(I2Cx, state->addr, I2C_DIRECTION_RECV);
            /* configure number of bytes to be transferred */
            I2C_SetTransferByteNumber(I2Cx, state->bytes);
            I2C_GenerateStart(I2Cx, ENABLE); 
        }
        else if((ev_state->final_stop ==1) || (state->writing == 1))
        {
            I2C_GenerateStop(I2Cx,ENABLE);
        }
    }
    else if (SReg_1 & I2C_FLAG_NAKF) 
    {                                  // Byte transmitted EV8 / EV8_1
        /* clear NACK flag */
        I2C_ClrFlag(I2Cx, I2C_FLAG_NAKF);
        state->error = true;
        state->busy = 0;
    }
}

void i2cInit(i2cDevice_e device)
{
    if (device == I2CINVALID)
        return;

    i2cDevice_t *pDev = &i2cDevice[device];
    const i2cHardware_t *hw = pDev->hardware;
    const IO_t scl = pDev->scl;
    const IO_t sda = pDev->sda;

    if (!hw || IOGetOwner(scl) || IOGetOwner(sda)) {
        return;
    }

    I2C_TypeDef *I2Cx = (I2C_TypeDef *)hw->reg;

    memset(&pDev->state, 0, sizeof(pDev->state));

    NVIC_InitTypeDef nvic;
    I2C_InitType I2C_InitStructure;
    RCC_ClocksTypeDef RCC_ClocksTmp;

    RCC_GetClocksFreqValue(&RCC_ClocksTmp);

    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));

    i2cClockEnable(I2Cx);
    
    I2C_ConfigInt(I2Cx, I2C_INT_ERR|I2C_INT_STOP|I2C_INT_RDR|I2C_INT_WDR|I2C_INT_TFC, ENABLE);

    // i2cUnstick(scl, sda);

    // Init pins
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sclAF);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sdaAF);

    
    I2C_InitStruct(&I2C_InitStructure);
    I2C_InitStructure.Timing           = i2cClockTIMINGR(RCC_ClocksTmp.SysBusDivClkFreq / I2C_CLOCK_DIVIDER, pDev->clockSpeed, 0);
    I2C_InitStructure.HSTiming         = 0x0;
    I2C_InitStructure.OwnAddress1      = 0;
    I2C_InitStructure.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    I2C_InitStructure.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2C_InitStructure.OwnAddress2      = 0x0;
    I2C_InitStructure.OwnAddress2Masks = I2C_ADDRESS2MASK_NONE;
    I2C_InitStructure.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2C_InitStructure.NoStretchMode    = I2C_NOSTRCH_DISABLE;

    I2C_Init(I2Cx, &I2C_InitStructure);

    // I2C ER Interrupt
    nvic.NVIC_IRQChannel = hw->er_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER);
    nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER);
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // I2C EV Interrupt
    nvic.NVIC_IRQChannel = hw->ev_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV);
    nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV);
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

#endif
