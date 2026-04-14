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
 * Weak stub functions for STM32C5 HAL2 bringup.
 *
 * These provide linker-resolvable symbols for peripheral drivers that
 * have not yet been ported to the HAL2 API. Each stub will be removed
 * as the corresponding HAL2 driver implementation is added.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/adc.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_spi.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/timer.h"
#include "platform/timer.h"
#include "common/color.h"
#include "drivers/light_ws2811strip.h"
#include "dshot_dpwm.h"

#include "pg/adc.h"

#define STUB __attribute__((weak))

/* ---- Timer ---- */

STUB const timerHardware_t fullTimerHardware[1] = {};

STUB void timerInit(void) {}
STUB void timerConfigure(const timerHardware_t *timHw, uint16_t period, uint32_t hz) { (void)timHw; (void)period; (void)hz; }
STUB void timerEnable(const timerHardware_t *timHw) { (void)timHw; }
STUB void timerDisable(const timerHardware_t *timHw) { (void)timHw; }
STUB void timerEnableInterrupt(const timerHardware_t *timHw) { (void)timHw; }
STUB void timerForceOverflow(timerResource_t *tim) { (void)tim; }
STUB void timerSetCounter(const timerHardware_t *timHw, uint32_t val) { (void)timHw; (void)val; }
STUB void timerSetPeriod(const timerHardware_t *timHw, uint32_t val) { (void)timHw; (void)val; }
STUB void timerReconfigureTimeBase(const timerHardware_t *timHw, uint16_t period, uint32_t hz) { (void)timHw; (void)period; (void)hz; }
STUB uint32_t timerClock(const timerHardware_t *timHw) { (void)timHw; return SystemCoreClock; }
STUB uint32_t timerGetPrescaler(const timerHardware_t *timHw) { (void)timHw; return 0; }
STUB int8_t timerGetTIMNumber(const timerHardware_t *timHw) { (void)timHw; return 0; }
STUB int8_t timerGetNumberByIndex(uint8_t index) { (void)index; return -1; }
STUB int8_t timerGetIndexByNumber(uint8_t number) { (void)number; return -1; }
STUB uint8_t timerLookupChannelIndex(const uint16_t channel) { (void)channel; return 0; }
STUB uint8_t timerInputInterrupt(const timerHardware_t *timHw) { (void)timHw; return 0; }
STUB void timerConfigUpdateCallback(const timerHardware_t *timHw, timerOvrHandlerRec_t *cb) { (void)timHw; (void)cb; }
STUB void timerChannelConfigCallbacks(const timerHardware_t *timHw, timerEdgeHandlerRec_t *edgeCb, timerOvrHandlerRec_t *ovrCb) { (void)timHw; (void)edgeCb; (void)ovrCb; }
STUB void timerChannelEdgeHandlerInit(timerEdgeHandlerRec_t *self, timerCCHandlerCallback *fn) { (void)self; (void)fn; }
STUB void timerChannelOverflowHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn) { (void)self; (void)fn; }
STUB volatile timCCR_t* timerChCCR(const timerHardware_t *timHw) { (void)timHw; return NULL; }
STUB void* timerFindTimerHandle(timerResource_t *tim) { (void)tim; return NULL; }

/* ---- UART ---- */

STUB const uartHardware_t uartHardware[UARTDEV_COUNT] = {};
STUB void uartReconfigure(uartPort_t *s) { (void)s; }
STUB void uartTryStartTxDMA(uartPort_t *s) { (void)s; }
STUB void uartTxMonitor(uartPort_t *s) { (void)s; }
STUB void uartDmaIrqHandler(dmaChannelDescriptor_t *descriptor) { (void)descriptor; }
STUB void uartIrqHandler(uartPort_t *s) { (void)s; }
STUB bool checkUsartTxOutput(uartPort_t *s) { (void)s; return false; }

/* ---- SPI ---- */

STUB void spiInitDevice(spiDevice_e device) { (void)device; }
STUB bool spiSequenceStart(const extDevice_t *dev) { (void)dev; return false; }

/* ---- I2C ---- */

STUB const i2cHardware_t i2cHardware[1] = {};
STUB i2cDevice_t i2cDevice[1] = {};
STUB void i2cInit(i2cDevice_e device) { (void)device; }
STUB bool i2cBusy(i2cDevice_e device, bool *error) { (void)device; (void)error; return false; }
STUB bool i2cWrite(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t data) { (void)device; (void)addr; (void)reg; (void)data; return false; }
STUB bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) { (void)device; (void)addr; (void)reg; (void)len; (void)data; return false; }
STUB bool i2cRead(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf) { (void)device; (void)addr; (void)reg; (void)len; (void)buf; return false; }
STUB bool i2cReadBuffer(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf) { (void)device; (void)addr; (void)reg; (void)len; (void)buf; return false; }
STUB uint16_t i2cGetErrorCounter(void) { return 0; }

/* ---- ADC ---- */

STUB void adcInit(const adcConfig_t *config) { (void)config; }
STUB void adcGetChannelValues(void) {}
STUB bool adcInternalIsBusy(void) { return false; }
STUB void adcInternalStartConversion(void) {}
STUB uint16_t adcInternalCompensateVref(uint16_t vrefAdcValue) { (void)vrefAdcValue; return 0; }
STUB int16_t adcInternalComputeTemperature(uint16_t tempAdcValue, uint16_t vrefValue) { (void)tempAdcValue; (void)vrefValue; return 25; }

/* ---- DMA reqmap ---- */

STUB const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e periph, uint8_t index, int8_t opt) { (void)periph; (void)index; (void)opt; return NULL; }
STUB const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(timerResource_t *tim, uint8_t channel, dmaoptValue_t dmaopt) { (void)tim; (void)channel; (void)dmaopt; return NULL; }
STUB dmaoptValue_t dmaoptByTag(ioTag_t ioTag) { (void)ioTag; return -1; }

/* ---- DSHOT ---- */

STUB void dshotEnableChannels(unsigned motorCount) { (void)motorCount; }
STUB bool dshotBitbangDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig) { (void)device; (void)motorConfig; return false; }
STUB dshotBitbangStatus_e dshotBitbangGetStatus(void) { return 0; }
STUB const timerHardware_t *dshotBitbangTimerGetAllocatedByNumberAndChannel(int8_t timerNumber, uint16_t timerChannel) { (void)timerNumber; (void)timerChannel; return NULL; }
STUB const resourceOwner_t *dshotBitbangTimerGetOwner(const timerHardware_t *timer) { (void)timer; return NULL; }

/* ---- PWM/DSHOT output ---- */

STUB void pwmCompleteDshotMotorUpdate(void) {}
STUB bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, motorProtocolTypes_e pwmProtocolType, uint8_t output) { (void)timerHardware; (void)motorIndex; (void)reorderedMotorIndex; (void)pwmProtocolType; (void)output; return false; }
STUB void pwmDshotSetDirectionOutput(motorDmaOutput_t *const motor) { (void)motor; }

/* ---- USB VCP ---- */

STUB serialPort_t *usbVcpOpen(void) { return NULL; }
STUB uint8_t usbVcpIsConnected(void) { return 0; }
STUB void usbVcpInit(void) {}
STUB bool usbCableIsInserted(void) { return false; }

/* ---- LED strip ---- */

STUB bool ws2811LedStripHardwareInit(void) { return false; }
STUB void ws2811LedStripStartTransfer(void) {}
