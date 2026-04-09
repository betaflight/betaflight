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
/*
 * porting for ch32h41x by Temperslee
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(USE_SOFT_I2C)

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "platform/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_i2c_timing.h"
#include "drivers/bus_i2c_utils.h"

#include "pg/pinio.h"


static void i2c_er_handler(i2cDevice_e device);
static void i2c_ev_handler(i2cDevice_e device);



#define IOCFG_I2C_PU IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_OD, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)  //Without this mode, an external pull-up resistor is required
#define IOCFG_I2C    IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_OD, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)



const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = I2C1,
        .sclPins = {
            I2CPINDEF(PB6, GPIO_AF4),
            I2CPINDEF(PB8, GPIO_AF4),
        },
        .sdaPins = {
            I2CPINDEF(PB7, GPIO_AF4),
            I2CPINDEF(PB9, GPIO_AF4),
        },
        .rcc = RCC_HB1(I2C1),
        .ev_irq = I2C1_EV_IRQn,
        .er_irq = I2C1_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = I2C2,
        .sclPins = {
            I2CPINDEF(PB10,  GPIO_AF4),
            I2CPINDEF(PC0,   GPIO_AF9),
        },
        .sdaPins = {
            I2CPINDEF(PB11,  GPIO_AF4),
            I2CPINDEF(PC1,   GPIO_AF9),
        },
        .rcc = RCC_HB1(I2C2),
        .ev_irq = I2C2_EV_IRQn,
        .er_irq = I2C2_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = I2C3,
        .sclPins = {
            I2CPINDEF(PA8,  GPIO_AF4),
            I2CPINDEF(PA14, GPIO_AF7),
        },
        .sdaPins = {
            I2CPINDEF(PC9,  GPIO_AF4),
            I2CPINDEF(PA13, GPIO_AF7),
        },
        .rcc = RCC_HB1(I2C3),
        .ev_irq = I2C3_EV_IRQn,
        .er_irq = I2C3_ER_IRQn,
    },
#endif

#ifdef USE_I2C_DEVICE_4
    {
        .device = I2CDEV_4,
        .reg = I2C4,
        .sclPins = {
            I2CPINDEF(PF12, GPIO_AF2),
            I2CPINDEF(PD12, GPIO_AF4),
            I2CPINDEF(PB6,  GPIO_AF6),
            I2CPINDEF(PB8,  GPIO_AF6),
        },
        .sdaPins = {
            I2CPINDEF(PF13, GPIO_AF2),
            I2CPINDEF(PD13, GPIO_AF4),
            I2CPINDEF(PB7,  GPIO_AF6),
            I2CPINDEF(PB9,  GPIO_AF6),
        },
        .rcc = RCC_HB2(I2C4),
        .ev_irq = I2C4_EV_IRQn,
        .er_irq = I2C4_ER_IRQn,
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


#ifdef USE_I2C_DEVICE_1
__FAST_INTERRUPT
void I2C1_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_1);
}
__FAST_INTERRUPT
void I2C1_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_1);
}
#endif

#ifdef USE_I2C_DEVICE_2
__FAST_INTERRUPT
void I2C2_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_2);
}
__FAST_INTERRUPT
void I2C2_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_2);
}
#endif

#ifdef USE_I2C_DEVICE_3
__FAST_INTERRUPT
void I2C3_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_3);
}
__FAST_INTERRUPT
void I2C3_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_3);
}
#endif

#ifdef USE_I2C_DEVICE_4
__FAST_INTERRUPT
void I2C4_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_4);
}
__FAST_INTERRUPT
void I2C4_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_4);
}
#endif

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

    I2C_TypeDef *I2Cx = i2cDevice[device].reg;

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

    if (!(I2Cx->CTLR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(I2Cx->CTLR1 & I2C_CTLR1_START)) {                             // ensure sending a start
            while (I2Cx->CTLR1 & I2C_CTLR1_STOP) {                          // wait for any stop to finish sending
                if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                    return i2cHandleHardwareFailure(device);
                }
            }
            I2C_GenerateSTART(I2Cx, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    return true;
}

bool i2cBusy(i2cDevice_e device, bool *error)
{
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

    I2C_TypeDef *I2Cx = i2cDevice[device].reg;
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

    if (!(I2Cx->CTLR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(I2Cx->CTLR1 & I2C_CTLR1_START)) {                             // ensure sending a start
            while (I2Cx->CTLR1 & I2C_CTLR1_STOP) {                          // wait for any stop to finish sending
                if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                    return i2cHandleHardwareFailure(device);
                }
            }
            I2C_GenerateSTART(I2Cx, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    return true;
}

bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    return i2cReadBuffer(device, addr_, reg_, len, buf) && i2cWait(device);
}

static void i2c_er_handler(i2cDevice_e device)
{
    I2C_TypeDef *I2Cx = i2cDevice[device].hardware->reg;

    i2cState_t *state = &i2cDevice[device].state;

    // Read the I2C1 status register
    volatile uint32_t SR1Register = I2Cx->STAR1;

    if (SR1Register & (I2C_STAR1_BERR | I2C_STAR1_ARLO | I2C_STAR1_AF | I2C_STAR1_OVR)) // an error
        state->error = true;

    // If AF, BERR or ARLO, abandon the current job and commence new if there are jobs
    if (SR1Register & (I2C_STAR1_BERR | I2C_STAR1_ARLO | I2C_STAR1_AF)) {
        (void)I2Cx->STAR2;                                                        // read second status register to clear ADDR if it is set (note that BTF will not be set after a NACK)
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                                // disable the RXNE/TXE interrupt - prevent the ISR tailchaining onto the ER (hopefully)
        if (!(SR1Register & I2C_STAR1_ARLO) && !(I2Cx->CTLR1 & I2C_CTLR1_STOP)) {     // if we dont have an ARLO error, ensure sending of a stop
            if (I2Cx->CTLR1 & I2C_CTLR1_START) {                                    // We are currently trying to send a start, this is very bad as start, stop will hang the peripheral
                while (I2Cx->CTLR1 & I2C_CTLR1_START) {; }                         // wait for any start to finish sending
                I2C_GenerateSTOP(I2Cx, ENABLE);                                 // send stop to finalise bus transaction
                while (I2Cx->CTLR1 & I2C_CTLR1_STOP) {; }                          // wait for stop to finish sending
                i2cInit(device);                                                // reset and configure the hardware
            }
            else {
                I2C_GenerateSTOP(I2Cx, ENABLE);                                 // stop to free up the bus
                I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);           // Disable EVT and ERR interrupts while bus inactive
            }
        }
    }
    I2Cx->STAR1 &= ~(I2C_STAR1_BERR | I2C_STAR1_ARLO | I2C_STAR1_AF | I2C_STAR1_OVR);     // reset all the error bits to clear the interrupt
    state->busy = 0;
}



void i2c_ev_handler(i2cDevice_e device)
{
    I2C_TypeDef *I2Cx = i2cDevice[device].hardware->reg;

    i2cEvState_t *ev_state = &i2c_ev_state[device];
    i2cState_t *state = &i2cDevice[device].state;

    uint8_t SReg_1 = I2Cx->STAR1;                                                 // read the status register here

    if (SReg_1 & I2C_STAR1_SB) {                                                  // we just sent a start - EV5 in ref manual
        I2Cx->CTLR1 &= ~I2C_CTLR1_POS;                                              // reset the POS bit so ACK/NACK applied to the current byte
        I2C_AcknowledgeConfig(I2Cx, ENABLE);                                    // make sure ACK is on
        ev_state->index = 0;                                                              // reset the index
        if (state->reading && (ev_state->subaddress_sent || 0xFF == state->reg)) {          // we have sent the subaddr
            ev_state->subaddress_sent = 1;                                                // make sure this is set in case of no subaddress, so following code runs correctly
            if (state->bytes == 2)
                I2Cx->CTLR1 |= I2C_CTLR1_POS;                                       // set the POS bit so NACK applied to the final byte in the two byte read
            I2C_Send7bitAddress(I2Cx, state->addr, I2C_Direction_Receiver);      // send the address and set hardware mode
        }
        else {                                                                // direction is Tx, or we havent sent the sub and rep start
            I2C_Send7bitAddress(I2Cx, state->addr, I2C_Direction_Transmitter);   // send the address and set hardware mode
            if (state->reg != 0xFF)                                              // 0xFF as subaddress means it will be ignored, in Tx or Rx mode
                ev_state->index = -1;                                                     // send a subaddress
        }
    }
    else if (SReg_1 & I2C_STAR1_ADDR) {                                         // we just sent the address - EV6 in ref manual
        // Read SR1,2 to clear ADDR
        // __DMB();    
        __asm("fence");                                                            // memory fence to control hardware
        if (state->bytes == 1 && state->reading && ev_state->subaddress_sent) {             // we are receiving 1 byte - EV6_3
            I2C_AcknowledgeConfig(I2Cx, DISABLE);                               // turn off ACK
            // __DMB();
            __asm("fence"); 
            (void)I2Cx->STAR2;                                                    // clear ADDR after ACK is turned off
            I2C_GenerateSTOP(I2Cx, ENABLE);                                     // program the stop
            ev_state->final_stop = 1;
            I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                     // allow us to have an EV7
        }
        else {                                                        // EV6 and EV6_1
            (void)I2Cx->STAR2;                                            // clear the ADDR here
            // __DMB();
            __asm("fence");
            if (state->bytes == 2 && state->reading && ev_state->subaddress_sent) {         // rx 2 bytes - EV6_1
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                           // turn off ACK
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                        // disable TXE to allow the buffer to fill
            }
            else if (state->bytes == 3 && state->reading && ev_state->subaddress_sent)    // rx 3 bytes
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                        // make sure RXNE disabled so we get a BTF in two bytes time
            else                                                                // receiving greater than three bytes, sending subaddress, or transmitting
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
        }
    }
    else if (SReg_1 & I2C_STAR1_BTF) {                                  // Byte transfer finished - EV7_2, EV7_3 or EV8_2
        ev_state->final_stop = 1;
        if (state->reading && ev_state->subaddress_sent) {                         // EV7_2, EV7_3
            if (state->bytes > 2) {                                      // EV7_2
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                   // turn off ACK
                state->read_p[ev_state->index++] = (uint8_t)I2Cx->DATAR;              // read data N-2
                I2C_GenerateSTOP(I2Cx, ENABLE);                         // program the Stop
                ev_state->final_stop = 1;                                         // required to fix hardware
                state->read_p[ev_state->index++] = (uint8_t)I2Cx->DATAR;              // read data N - 1
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                 // enable TXE to allow the final EV7
            }
            else {                                                    // EV7_3
                if (ev_state->final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     // program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    // program a rep start
                state->read_p[ev_state->index++] = (uint8_t)I2Cx->DATAR;                    // read data N - 1
                state->read_p[ev_state->index++] = (uint8_t)I2Cx->DATAR;                    // read data N
                ev_state->index++;                                                // to show job completed
            }
        }
        else {                                                        // EV8_2, which may be due to a subaddress sent or a write completion
            if (ev_state->subaddress_sent || (state->writing)) {
                if (ev_state->final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     // program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    // program a rep start
                ev_state->index++;                                                // to show that the job is complete
            }
            else {                                                    // We need to send a subaddress
                I2C_GenerateSTART(I2Cx, ENABLE);                        // program the repeated Start
                ev_state->subaddress_sent = 1;                                    // this is set back to zero upon completion of the current task
            }
        }
        // we must wait for the start to clear, otherwise we get constant BTF
        while (I2Cx->CTLR1 & I2C_CTLR1_START) {; }
    }
    else if (SReg_1 & I2C_STAR1_RXNE) {                                 // Byte received - EV7
        state->read_p[ev_state->index++] = (uint8_t)I2Cx->DATAR;
        if (state->bytes == (ev_state->index + 3))
            I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                    // disable TXE to allow the buffer to flush so we can get an EV7_2
        if (state->bytes == ev_state->index)                                             // We have completed a final EV7
            ev_state->index++;                                                    // to show job is complete
    }
    else if (SReg_1 & I2C_STAR1_TXE) {                                  // Byte transmitted EV8 / EV8_1
        if (ev_state->index != -1) {                                              // we dont have a subaddress to send
            I2Cx->DATAR = state->write_p[ev_state->index++];
            if (state->bytes == ev_state->index)                                         // we have sent all the data
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush
        }
        else {
            ev_state->index++;
            I2Cx->DATAR = state->reg;                                             // send the subaddress
            if (state->reading || !(state->bytes))                                      // if receiving or sending 0 bytes, flush now
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush
        }
    }
    if (ev_state->index == state->bytes + 1) {                                           // we have completed the current job
        ev_state->subaddress_sent = 0;                                            // reset this here
        if (ev_state->final_stop)                                                 // If there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
            I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);       // Disable EVT and ERR interrupts while bus inactive
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

    I2C_TypeDef *I2Cx = hw->reg;

    memset(&pDev->state, 0, sizeof(pDev->state));

    // NVIC_InitTypeDef nvic;
    I2C_InitTypeDef i2cInit = {0};

    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));

    // Enable RCC
    RCC_ClockCmd(hw->rcc, ENABLE);

    I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

    i2cUnstick(scl, sda);

    // Init pins
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sclAF);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sdaAF);


    I2C_DeInit(I2Cx);
    I2C_StructInit(&i2cInit);

    I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);               // Disable EVT and ERR interrupts - they are enabled by the first request
    i2cInit.I2C_Mode = I2C_Mode_I2C;
    i2cInit.I2C_DutyCycle = I2C_DutyCycle_2;
    i2cInit.I2C_OwnAddress1 = 0;
    i2cInit.I2C_Ack = I2C_Ack_Enable;
    i2cInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2cInit.I2C_ClockSpeed = pDev->clockSpeed * 1000;
    I2C_Init(I2Cx, &i2cInit);
    I2C_Cmd(I2Cx, ENABLE);

    I2C_StretchClockCmd(I2Cx, ENABLE);

    // I2C ER Interrupt
    // nvic.NVIC_IRQChannel = hw->er_irq;
    // nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER);
    // nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER);
    // nvic.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&nvic);

    NVIC_SetPriority(hw->er_irq,NVIC_PRIO_I2C_ER);
    NVIC_EnableIRQ(hw->er_irq);

    // I2C EV Interrupt
    // nvic.NVIC_IRQChannel = hw->ev_irq;
    // nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV);
    // nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV);
    // NVIC_Init(&nvic);

    NVIC_SetPriority(hw->ev_irq,NVIC_PRIO_I2C_EV);
    NVIC_EnableIRQ(hw->ev_irq);
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}






#endif
