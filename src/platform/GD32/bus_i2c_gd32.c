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
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_i2c_utils.h"

static void i2c_er_handler(I2CDevice device);
static void i2c_ev_handler(I2CDevice device);

#if defined(GD32F4)
#define IOCFG_I2C_PU IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_50MHZ, GPIO_OTYPE_OD, GPIO_PUPD_PULLUP)
#define IOCFG_I2C    IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_50MHZ, GPIO_OTYPE_OD, GPIO_PUPD_NONE)
#else // GD32F4
#define IOCFG_I2C   IO_CONFIG(GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ)
#endif

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_0
    {
        .device = I2CDEV_0,
        .reg = (void *)I2C0,
        .sclPins = {
            I2CPINDEF(PB6, GPIO_AF_4),
            I2CPINDEF(PB8, GPIO_AF_4),
        },
         .sdaPins = {
            I2CPINDEF(PB7, GPIO_AF_4),
            I2CPINDEF(PB9, GPIO_AF_4),
         },
        .rcc = RCC_APB1(I2C0),
        .ev_irq = I2C0_EV_IRQn,
        .er_irq = I2C0_ER_IRQn,
    },
#endif
 #ifdef USE_I2C_DEVICE_1
     {
         .device = I2CDEV_1,
         .reg = (void *)I2C1,
         .sclPins = {
             I2CPINDEF(PB10, GPIO_AF_4),
             I2CPINDEF(PF1,  GPIO_AF_4),
         },
         .sdaPins = {
             I2CPINDEF(PB11, GPIO_AF_4),
             I2CPINDEF(PF0,  GPIO_AF_4),
         },
         .rcc = RCC_APB1(I2C1),
         .ev_irq = I2C1_EV_IRQn,
         .er_irq = I2C1_ER_IRQn,
     },
 #endif
 #ifdef USE_I2C_DEVICE_2
     {
         .device = I2CDEV_2,
         .reg = (void *)I2C2,
         .sclPins = {
             I2CPINDEF(PA8, GPIO_AF_4),
         },
         .sdaPins = {
             I2CPINDEF(PC9, GPIO_AF_4),
         },
         .rcc = RCC_APB1(I2C2),
         .ev_irq = I2C2_EV_IRQn,
         .er_irq = I2C2_ER_IRQn,
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

void I2C0_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_0);
}

void I2C0_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_0);
}

void I2C1_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_1);
}

void I2C1_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_1);
}

void I2C2_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_2);
}

void I2C2_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_2);
}

 static bool i2cHandleHardwareFailure(I2CDevice device)
 {
     i2cErrorCount++;
     // reinit peripheral + clock out garbage
     i2cInit(device);
     return false;
 }

bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    uint32_t I2Cx = (uint32_t )(i2cDevice[device].reg);

    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &(i2cDevice[device].state);
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

    if (!(I2C_CTL1(I2Cx) & I2C_CTL1_EVIE)) {                                      // if we are restarting the driver
        if (!(I2C_CTL0(I2Cx) & I2C_CTL0_START)) {                                 // ensure sending a start
            while (I2C_CTL0(I2Cx) & I2C_CTL0_STOP) {                              // wait for any stop to finish sending
                if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                    return i2cHandleHardwareFailure(device);
                }
            }
            i2c_start_on_bus(I2Cx);                                               // send the start for the new job
        }
        i2c_interrupt_enable(I2Cx, I2C_INT_ERR);                                  // allow the interrupts to fire off again
        i2c_interrupt_enable(I2Cx, I2C_INT_EV); 
    }

    return true;
}

bool i2cBusy(I2CDevice device, bool *error)
{
    i2cState_t *state = &i2cDevice[device].state;

    if (error) {
        *error = state->error;
    }
    return state->busy;
}

static bool i2cWait(I2CDevice device)
{
    i2cState_t *state = &(i2cDevice[device].state);
    timeUs_t timeoutStartUs = microsISR();

    while (state->busy) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device) && i2cWait(device);
        }
    }

    return !(state->error);
}

bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data) && i2cWait(device);
}

bool i2cReadBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{

    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    uint32_t I2Cx = (uint32_t )(i2cDevice[device].reg);
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

    if (!(I2C_CTL1(I2Cx) & I2C_CTL1_EVIE)) {                                      // if we are restarting the driver
        if (!(I2C_CTL0(I2Cx) & I2C_CTL0_START)) {                                 // ensure sending a start
            while (I2C_CTL0(I2Cx) & I2C_CTL0_STOP) {                              // wait for any stop to finish sending
                if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                    return i2cHandleHardwareFailure(device);
                }
            }

            /* enable acknowledge */
            i2c_ack_config(I2Cx, I2C_ACK_ENABLE);
            uint8_t wait_count = 0;
            /* wait until I2C bus is idle */
            while(i2c_flag_get(I2Cx, I2C_FLAG_I2CBSY)){
                if(wait_count>10) {
                    break;
                }
                wait_count++;
                delay(5);
            }
            if(2 == len) {
                i2c_ackpos_config(I2Cx, I2C_ACKPOS_NEXT);
            }
            i2c_start_on_bus(I2Cx);                                               // send the start for the new job
        }
        i2c_interrupt_enable(I2Cx, I2C_INT_ERR);                                  // allow the interrupts to fire off again
        i2c_interrupt_enable(I2Cx, I2C_INT_EV);
    }

    return true;
}

bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    return i2cReadBuffer(device, addr_, reg_, len, buf) && i2cWait(device);
}

static void i2c_er_handler(I2CDevice device)
{
    uint32_t I2Cx = (uint32_t )(i2cDevice[device].hardware->reg);

    i2cState_t *state = &i2cDevice[device].state;

    // Read the I2C status register
    volatile uint32_t SR1Register = I2C_STAT0(I2Cx);

    if (SR1Register & (I2C_STAT0_BERR | I2C_STAT0_LOSTARB | I2C_STAT0_AERR | I2C_STAT0_OUERR))   // an error
        state->error = true;

     if (SR1Register & (I2C_STAT0_BERR | I2C_STAT0_LOSTARB | I2C_STAT0_AERR)) {
        // I2C_STAT0(I2Cx);
        I2C_STAT1(I2Cx);                                                                         // read second status register to clear ADDR if it is set (note that BTC will not be set after a NACK)
        i2c_interrupt_disable(I2Cx, I2C_INT_BUF);                                                // disable the RBNE/TBE interrupt - prevent the ISR tailchaining onto the ERR
        if (!(SR1Register & I2C_STAT0_LOSTARB) && !(I2C_CTL0(I2Cx) & I2C_CTL0_STOP)) {
            if (I2C_CTL0(I2Cx) & I2C_CTL0_START) {                                               // We are currently trying to send a start, this is very bad as start, stop will hang the peripheral
                while (I2C_CTL0(I2Cx) & I2C_CTL0_START) {; }                                     // wait for any start to finish sending
                i2c_stop_on_bus(I2Cx);                                                           // send stop to finalise bus transaction
                while (I2C_CTL0(I2Cx) & I2C_CTL0_STOP) {; }                                      // wait for stop to finish sending
                i2cInit(device);                                                                 // reset and configure the hardware
            } else {
                i2c_stop_on_bus(I2Cx);                                                           // stop to free up the bus
                while (I2C_CTL0(I2Cx) & I2C_CTL0_STOP) {; }
                i2c_interrupt_disable(I2Cx, I2C_INT_ERR);                                        // Disable EV and ERR interrupts while bus inactive
                i2c_interrupt_disable(I2Cx, I2C_INT_EV); 
            }
        }
    }
    I2C_STAT0(I2Cx) &= ~(I2C_STAT0_BERR | I2C_STAT0_LOSTARB | I2C_STAT0_AERR | I2C_STAT0_OUERR); // reset all the error bits to clear the interrupt
    state->busy = 0;
}

void i2c_ev_handler(I2CDevice device)
{
    uint32_t I2Cx = (uint32_t)(i2cDevice[device].hardware->reg);

    i2cEvState_t *ev_state = &i2c_ev_state[device];
    i2cState_t *state = &i2cDevice[device].state;

    uint8_t SReg_1 = I2C_STAT0(I2Cx);                                                           // read the status register here

    if (SReg_1 & I2C_STAT0_SBSEND) {                                                            // we just sent a start
        ev_state->index = 0;                                                                    // reset the index
        if (state->reading && (ev_state->subaddress_sent || 0xFF == state->reg)) {              // we have sent the subaddr
            ev_state->subaddress_sent = 1;                                                      // make sure this is set in case of no subaddress, so following code runs correctly
            i2c_master_addressing(I2Cx, state->addr, I2C_RECEIVER);                             // send the address and set hardware mode
        } else {                                                                                // direction is TB, or we havent sent the sub and rep start
            i2c_master_addressing(I2Cx, state->addr, I2C_TRANSMITTER);                          // send the address and set hardware mode
            if (state->reg != 0xFF)                                                             // 0xFF as subaddress means it will be ignored, in TB or RB mode
                ev_state->index = -1;                                                           // send a subaddress
         }
    } else if (SReg_1 & I2C_STAT0_ADDSEND) {                                                    // we just sent the address
        // Read SR1,2 to clear ADDR
        __DMB();                                                                                // memory fence to control hardware
        if (state->bytes == 1 && state->reading && ev_state->subaddress_sent) {                 // we are receiving 1 byte
            i2c_ack_config(I2Cx, I2C_ACK_DISABLE);                                              // turn off ACK
            __DMB();
            I2C_STAT1(I2Cx);                                                                    // clear ADDR after ACK is turned off
            i2c_stop_on_bus(I2Cx);                                                              // program the stop
            while(I2C_CTL0(I2Cx) & I2C_CTL0_STOP){;}

            ev_state->final_stop = 1;
            i2c_interrupt_enable(I2Cx, I2C_INT_BUF);
        } else {   
            I2C_STAT1(I2Cx);                                                                    // clear the ADDR here
            __DMB();
            if (state->bytes == 2 && state->reading && ev_state->subaddress_sent) {             // rx 2 bytes
                i2c_ack_config(I2Cx, I2C_ACK_DISABLE);                                          // turn off ACK
                i2c_interrupt_disable(I2Cx, I2C_INT_BUF);                                       // disable TBE to allow the buffer to fill
            } else if (state->bytes == 3 && state->reading && ev_state->subaddress_sent) {      // rx 3 bytes
                i2c_interrupt_disable(I2Cx, I2C_INT_BUF);                                       // make sure RBNE disabled so we get a BTC in two bytes time
            } else {                                                                            // receiving greater than three bytes, sending subaddress, or transmitting
                i2c_interrupt_enable(I2Cx, I2C_INT_BUF);
            }
        }
    } else if (SReg_1 & I2C_STAT0_BTC) {                                                        // Byte transfer finished
        ev_state->final_stop = 1;
        if (state->reading && ev_state->subaddress_sent) {
            if (state->bytes > 2) {
                i2c_ack_config(I2Cx, I2C_ACK_DISABLE);                                          // turn off ACK
                state->read_p[ev_state->index++] = (uint8_t)I2C_DATA(I2Cx);                     // read data N-2
                i2c_stop_on_bus(I2Cx);                                                          // program the Stop
                while (I2C_CTL0(I2Cx) & I2C_CTL0_STOP) {; }

                ev_state->final_stop = 1;                                                       // required to fix hardware
                state->read_p[ev_state->index++] = (uint8_t)I2C_DATA(I2Cx);                     // read data N - 1
                i2c_interrupt_enable(I2Cx, I2C_INT_BUF);
            } else {
                if (ev_state->final_stop) {
                    i2c_stop_on_bus(I2Cx);                                                      // program the Stop
                    while (I2C_CTL0(I2Cx) & I2C_CTL0_STOP) {; }
                } else {
                    i2c_start_on_bus(I2Cx);                                                     // program a rep start
                }

                state->read_p[ev_state->index++] = (uint8_t)I2C_DATA(I2Cx);                     // read data N - 1
                state->read_p[ev_state->index++] = (uint8_t)I2C_DATA(I2Cx);                     // read data N
                ev_state->index++;
            }
        } else {
            if (ev_state->subaddress_sent || (state->writing)) {
                if (ev_state->final_stop) {
                    i2c_stop_on_bus(I2Cx);                                                      // program the Stop
                    while (I2C_CTL0(I2Cx) & I2C_CTL0_STOP) {; }
                } else {
                    i2c_start_on_bus(I2Cx);                                                     // program a rep start
                }

                ev_state->index++;
            } else {                                                                            // We need to send a subaddress
                i2c_start_on_bus(I2Cx);                                                         // program the repeated Start
                ev_state->subaddress_sent = 1;                                                  // this is set back to zero upon completion of the current task
            }
        }
        // we must wait for the start to clear, otherwise we get constant BTC
        while (I2C_CTL0(I2Cx) & I2C_CTL0_START) {; }
    } else if (SReg_1 & I2C_STAT0_RBNE) {                                                       // Byte received
        state->read_p[ev_state->index++] = (uint8_t)I2C_DATA(I2Cx);
        if (state->bytes == (ev_state->index + 3))
            i2c_interrupt_disable(I2Cx, I2C_INT_BUF);                                           // disable TBE to allow the buffer to flush
        if (state->bytes == ev_state->index)
            ev_state->index++;
    } else if (SReg_1 & I2C_STAT0_TBE) {                                                        // Byte transmitted
        if (ev_state->index != -1) { 
            if (state->bytes == (ev_state->index + 1) )                                         // we have sent all the data
                i2c_interrupt_disable(I2Cx, I2C_INT_BUF);                                       // disable TXE to allow the buffer to flush
            I2C_DATA(I2Cx) = state->write_p[ev_state->index++];
        } else {
            ev_state->index++;
            if (state->reading || !(state->bytes))                                              // if receiving or sending 0 bytes, flush now
                i2c_interrupt_disable(I2Cx, I2C_INT_BUF);                                       // disable TBE to allow the buffer to flush
            I2C_DATA(I2Cx) = state->reg;                                                        // send the subaddress

        }
    }
    if (ev_state->index == state->bytes + 1) {
        ev_state->subaddress_sent = 0;                                                          // reset this here
        if (ev_state->final_stop){
            i2c_interrupt_disable(I2Cx, I2C_INT_ERR); 
            i2c_interrupt_disable(I2Cx, I2C_INT_EV); 
        }                                                                                       // If there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTC

        state->busy = 0;
    }
}

void i2cInit(I2CDevice device)
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

    uint32_t I2Cx = (uint32_t )i2cDevice[device].reg;
    memset(&pDev->state, 0, sizeof(pDev->state));

    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));

    // Enable RCC
    RCC_ClockCmd(hw->rcc, ENABLE);

    i2c_interrupt_disable(I2Cx, I2C_INT_EV);
    i2cUnstick(scl, sda);

     // Init pins
 #ifdef GD32F4
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sclAF);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sdaAF);
 #else
    IOConfigGPIO(scl, IOCFG_I2C);
    IOConfigGPIO(sda, IOCFG_I2C);
 #endif

    i2c_deinit(I2Cx);
    i2c_interrupt_disable(I2Cx, I2C_INT_ERR);
    pDev->clockSpeed = ((pDev->clockSpeed > 400) ? 400 : pDev->clockSpeed);
    i2c_clock_config(I2Cx, pDev->clockSpeed * 1000, I2C_DTCY_2);
    i2c_mode_addr_config(I2Cx, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0);

    i2c_stretch_scl_low_config(I2Cx, I2C_SCLSTRETCH_ENABLE);
    i2c_enable(I2Cx);
    i2c_ack_config(I2Cx, I2C_ACK_ENABLE);
     // I2C Interrupt
    nvic_irq_enable(hw->er_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER));
    nvic_irq_enable(hw->ev_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV));
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

#endif
