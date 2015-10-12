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

#include <platform.h>

#include "build_config.h"

#include "gpio.h"
#include "system.h"

#include "bus_i2c.h"
#include "nvic.h"

#ifndef SOFT_I2C

// I2C2
// SCL  PB10
// SDA  PB11
// I2C1
// SCL  PB6
// SDA  PB7

static void i2c_er_handler(void);
static void i2c_ev_handler(void);
static void i2cUnstick(void);

typedef struct i2cDevice_s {
    I2C_TypeDef *dev;
    GPIO_TypeDef *gpio;
    uint16_t scl;
    uint16_t sda;
    uint8_t ev_irq;
    uint8_t er_irq;
    uint32_t peripheral;
} i2cDevice_t;

static const i2cDevice_t i2cHardwareMap[] = {
    { I2C1, GPIOB, Pin_6, Pin_7, I2C1_EV_IRQn, I2C1_ER_IRQn, RCC_APB1Periph_I2C1 },
    { I2C2, GPIOB, Pin_10, Pin_11, I2C2_EV_IRQn, I2C2_ER_IRQn, RCC_APB1Periph_I2C2 },
};

// Copy of peripheral address for IRQ routines
static I2C_TypeDef *I2Cx = NULL;
// Copy of device index for reinit, etc purposes
static I2CDevice I2Cx_index;

void I2C1_ER_IRQHandler(void)
{
    i2c_er_handler();
}

void I2C1_EV_IRQHandler(void)
{
    i2c_ev_handler();
}

void I2C2_ER_IRQHandler(void)
{
    i2c_er_handler();
}

void I2C2_EV_IRQHandler(void)
{
    i2c_ev_handler();
}

#define I2C_DEFAULT_TIMEOUT 30000
static volatile uint16_t i2cErrorCount = 0;

static volatile bool error = false;
static volatile bool busy;

static volatile uint8_t addr;
static volatile uint8_t reg;
static volatile uint8_t bytes;
static volatile uint8_t writing;
static volatile uint8_t reading;
static volatile uint8_t* write_p;
static volatile uint8_t* read_p;

static bool i2cHandleHardwareFailure(void)
{
    i2cErrorCount++;
    // reinit peripheral + clock out garbage
    i2cInit(I2Cx_index);
    return false;
}

bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    addr = addr_ << 1;
    reg = reg_;
    writing = 1;
    reading = 0;
    write_p = data;
    read_p = data;
    bytes = len_;
    busy = 1;
    error = false;

    if (!I2Cx)
        return false;

    if (!(I2Cx->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(I2Cx->CR1 & 0x0100)) {                                    // ensure sending a start
            while (I2Cx->CR1 & 0x0200 && --timeout > 0) { ; }           // wait for any stop to finish sending
            if (timeout == 0) {
                return i2cHandleHardwareFailure();
            }
            I2C_GenerateSTART(I2Cx, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    timeout = I2C_DEFAULT_TIMEOUT;
    while (busy && --timeout > 0) { ; }
    if (timeout == 0) {
        return i2cHandleHardwareFailure();
    }

    return !error;
}

bool i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(addr_, reg_, 1, &data);
}

bool i2cRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    addr = addr_ << 1;
    reg = reg_;
    writing = 0;
    reading = 1;
    read_p = buf;
    write_p = buf;
    bytes = len;
    busy = 1;
    error = false;

    if (!I2Cx)
        return false;

    if (!(I2Cx->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(I2Cx->CR1 & 0x0100)) {                                    // ensure sending a start
            while (I2Cx->CR1 & 0x0200 && --timeout > 0) { ; }           // wait for any stop to finish sending
            if (timeout == 0)
                return i2cHandleHardwareFailure();
            I2C_GenerateSTART(I2Cx, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    timeout = I2C_DEFAULT_TIMEOUT;
    while (busy && --timeout > 0) { ; }
    if (timeout == 0)
        return i2cHandleHardwareFailure();

    return !error;
}

static void i2c_er_handler(void)
{
    // Read the I2Cx status register
    uint32_t SR1Register = I2Cx->SR1;

    if (SR1Register & 0x0F00) {                                         // an error
        error = true;
    }

    // If AF, BERR or ARLO, abandon the current job and commence new if there are jobs
    if (SR1Register & 0x0700) {
        (void)I2Cx->SR2;                                                // read second status register to clear ADDR if it is set (note that BTF will not be set after a NACK)
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                        // disable the RXNE/TXE interrupt - prevent the ISR tailchaining onto the ER (hopefully)
        if (!(SR1Register & 0x0200) && !(I2Cx->CR1 & 0x0200)) {         // if we dont have an ARLO error, ensure sending of a stop
            if (I2Cx->CR1 & 0x0100) {                                   // We are currently trying to send a start, this is very bad as start, stop will hang the peripheral
                // TODO - busy waiting in highest priority IRQ. Maybe only set flag and handle it from main loop
                while (I2Cx->CR1 & 0x0100) { ; }                        // wait for any start to finish sending
                I2C_GenerateSTOP(I2Cx, ENABLE);                         // send stop to finalise bus transaction
                while (I2Cx->CR1 & 0x0200) { ; }                        // wait for stop to finish sending
                i2cInit(I2Cx_index);                                    // reset and configure the hardware
            } else {
                I2C_GenerateSTOP(I2Cx, ENABLE);                         // stop to free up the bus
                I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);   // Disable EVT and ERR interrupts while bus inactive
            }
        }
    }
    I2Cx->SR1 &= ~0x0F00;                                               // reset all the error bits to clear the interrupt
    busy = 0;
}

void i2c_ev_handler(void)
{
    static uint8_t subaddress_sent, final_stop;                         // flag to indicate if subaddess sent, flag to indicate final bus condition
    static int8_t index;                                                // index is signed -1 == send the subaddress
    uint8_t SReg_1 = I2Cx->SR1;                                         // read the status register here

    if (SReg_1 & 0x0001) {                                              // we just sent a start - EV5 in ref manual
        I2Cx->CR1 &= ~0x0800;                                           // reset the POS bit so ACK/NACK applied to the current byte
        I2C_AcknowledgeConfig(I2Cx, ENABLE);                            // make sure ACK is on
        index = 0;                                                      // reset the index
        if (reading && (subaddress_sent || 0xFF == reg)) {              // we have sent the subaddr
            subaddress_sent = 1;                                        // make sure this is set in case of no subaddress, so following code runs correctly
            if (bytes == 2)
                I2Cx->CR1 |= 0x0800;                                    // set the POS bit so NACK applied to the final byte in the two byte read
            I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Receiver);    // send the address and set hardware mode
        } else {                                                        // direction is Tx, or we havent sent the sub and rep start
            I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Transmitter); // send the address and set hardware mode
            if (reg != 0xFF)                                            // 0xFF as subaddress means it will be ignored, in Tx or Rx mode
                index = -1;                                             // send a subaddress
        }
    } else if (SReg_1 & 0x0002) {                                       // we just sent the address - EV6 in ref manual
        // Read SR1,2 to clear ADDR
        __DMB();                                                        // memory fence to control hardware
        if (bytes == 1 && reading && subaddress_sent) {                 // we are receiving 1 byte - EV6_3
            I2C_AcknowledgeConfig(I2Cx, DISABLE);                       // turn off ACK
            __DMB();
            (void)I2Cx->SR2;                                            // clear ADDR after ACK is turned off
            I2C_GenerateSTOP(I2Cx, ENABLE);                             // program the stop
            final_stop = 1;
            I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                     // allow us to have an EV7
        } else {                                                        // EV6 and EV6_1
            (void)I2Cx->SR2;                                            // clear the ADDR here
            __DMB();
            if (bytes == 2 && reading && subaddress_sent) {             // rx 2 bytes - EV6_1
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                   // turn off ACK
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to fill
            } else if (bytes == 3 && reading && subaddress_sent)        // rx 3 bytes
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // make sure RXNE disabled so we get a BTF in two bytes time
            else                                                        // receiving greater than three bytes, sending subaddress, or transmitting
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
        }
    } else if (SReg_1 & 0x004) {                                        // Byte transfer finished - EV7_2, EV7_3 or EV8_2
        final_stop = 1;
        if (reading && subaddress_sent) {                               // EV7_2, EV7_3
            if (bytes > 2) {                                            // EV7_2
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                   // turn off ACK
                read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N-2
                I2C_GenerateSTOP(I2Cx, ENABLE);                         // program the Stop
                final_stop = 1;                                         // required to fix hardware
                read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N - 1
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                 // enable TXE to allow the final EV7
            } else {                                                    // EV7_3
                if (final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     // program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    // program a rep start
                read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N - 1
                read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N
                index++;                                                // to show job completed
            }
        } else {                                                        // EV8_2, which may be due to a subaddress sent or a write completion
            if (subaddress_sent || (writing)) {
                if (final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     // program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    // program a rep start
                index++;                                                // to show that the job is complete
            } else {                                                    // We need to send a subaddress
                I2C_GenerateSTART(I2Cx, ENABLE);                        // program the repeated Start
                subaddress_sent = 1;                                    // this is set back to zero upon completion of the current task
            }
        }
        // TODO - busy waiting in ISR
        // we must wait for the start to clear, otherwise we get constant BTF
        while (I2Cx->CR1 & 0x0100) { ; }
    } else if (SReg_1 & 0x0040) {                                       // Byte received - EV7
        read_p[index++] = (uint8_t)I2Cx->DR;
        if (bytes == (index + 3))
            I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                    // disable TXE to allow the buffer to flush so we can get an EV7_2
        if (bytes == index)                                             // We have completed a final EV7
            index++;                                                    // to show job is complete
    } else if (SReg_1 & 0x0080) {                                       // Byte transmitted EV8 / EV8_1
        if (index != -1) {                                              // we dont have a subaddress to send
            I2Cx->DR = write_p[index++];
            if (bytes == index)                                         // we have sent all the data
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush
        } else {
            index++;
            I2Cx->DR = reg;                                             // send the subaddress
            if (reading || !bytes)                                      // if receiving or sending 0 bytes, flush now
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush
        }
    }
    if (index == bytes + 1) {                                           // we have completed the current job
        subaddress_sent = 0;                                            // reset this here
        if (final_stop)                                                 // If there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
            I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);       // Disable EVT and ERR interrupts while bus inactive
        busy = 0;
    }
}

void i2cInit(I2CDevice index)
{
    NVIC_InitTypeDef nvic;
    I2C_InitTypeDef i2c;

    if (index > I2CDEV_MAX)
        index = I2CDEV_MAX;

    // Turn on peripheral clock, save device and index
    I2Cx = i2cHardwareMap[index].dev;
    I2Cx_index = index;
    RCC_APB1PeriphClockCmd(i2cHardwareMap[index].peripheral, ENABLE);

    // diable I2C interrrupts first to avoid ER handler triggering
    I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

    // clock out stuff to make sure slaves arent stuck
    // This will also configure GPIO as AF_OD at the end
    i2cUnstick();

    // Init I2C peripheral
    I2C_DeInit(I2Cx);
    I2C_StructInit(&i2c);

    I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);               // Disable EVT and ERR interrupts - they are enabled by the first request
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = 400000;
    I2C_Cmd(I2Cx, ENABLE);
    I2C_Init(I2Cx, &i2c);

    // I2C ER Interrupt
    nvic.NVIC_IRQChannel = i2cHardwareMap[index].er_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER);
    nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER);
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // I2C EV Interrupt
    nvic.NVIC_IRQChannel = i2cHardwareMap[index].ev_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV);
    nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV);
    NVIC_Init(&nvic);
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

static void i2cUnstick(void)
{
    GPIO_TypeDef *gpio;
    gpio_config_t cfg;
    uint16_t scl, sda;
    int i;

    // prepare pins
    gpio = i2cHardwareMap[I2Cx_index].gpio;
    scl = i2cHardwareMap[I2Cx_index].scl;
    sda = i2cHardwareMap[I2Cx_index].sda;

    digitalHi(gpio, scl | sda);

    cfg.pin = scl | sda;
    cfg.speed = Speed_2MHz;
    cfg.mode = Mode_Out_OD;
    gpioInit(gpio, &cfg);

    for (i = 0; i < 8; i++) {
        // Wait for any clock stretching to finish
        while (!digitalIn(gpio, scl))
            delayMicroseconds(10);

        // Pull low
        digitalLo(gpio, scl); // Set bus low
        delayMicroseconds(10);
        // Release high again
        digitalHi(gpio, scl); // Set bus high
        delayMicroseconds(10);
    }

    // Generate a start then stop condition
    // SCL  PB10
    // SDA  PB11
    digitalLo(gpio, sda); // Set bus data low
    delayMicroseconds(10);
    digitalLo(gpio, scl); // Set bus scl low
    delayMicroseconds(10);
    digitalHi(gpio, scl); // Set bus scl high
    delayMicroseconds(10);
    digitalHi(gpio, sda); // Set bus sda high

    // Init pins
    cfg.pin = scl | sda;
    cfg.speed = Speed_2MHz;
    cfg.mode = Mode_AF_OD;
    gpioInit(gpio, &cfg);
}

#endif
