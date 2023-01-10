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

/*
 * Based on https://github.com/ExpressLRS/ExpressLRS
 * Thanks to AlessandroAU, original creator of the ExpressLRS project.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RX_SX1280

#include "build/atomic.h"
#include "build/debug.h"

#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/rx/rx_sx1280.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/time.h"

#include "rx/rx_spi.h"
#include "rx/expresslrs.h"
#include "rx/expresslrs_common.h"
#include "rx/expresslrs_impl.h"

#define SX1280_MAX_SPI_MHZ 18000000

// The following global variables are accessed from interrupt context to process the sequence of steps in packet processing
// As there is only ever one device, no need to add a device context; globals will do
static volatile dioReason_e irqReason; // Used to pass irq status from sx1280IrqStatusRead() to sx1280ProcessIrq()
static volatile uint8_t packetStats[2];
static volatile uint8_t FIFOaddr; // Used to pass data from sx1280GotFIFOAddr() to sx1280DoReadBuffer()

static IO_t busy;

typedef struct busyIntContext_s {
    extiCallbackRec_t exti;
} busyIntContext_t;

static busyIntContext_t busyIntContext;

static volatile timeUs_t sx1280Processing;

static volatile bool pendingISR = false;
static volatile bool pendingDoFHSS = false;

#define SX1280_BUSY_TIMEOUT_US 1000


bool sx1280IsBusy(void)
{
    return IORead(busy);
}

FAST_CODE static bool sx1280PollBusy(void)
{
    uint32_t startTime = micros();
    while (IORead(busy)) {
        if ((micros() - startTime) > SX1280_BUSY_TIMEOUT_US) {
            return false;
        } else {
            __asm__("nop");
        }
    }
    return true;
}

FAST_CODE static bool sx1280MarkBusy(void)
{
    // Check that there isn't already a sequence of accesses to the SX1280 in progress
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (sx1280Processing) {
            return false;
        }

        sx1280Processing = micros();
    }

    return true;
}

static void sx1280ClearBusyFn(void)
{
    EXTIDisable(busy);
}

// Switch to waiting for busy interrupt
FAST_CODE static bool sx1280EnableBusy(void)
{
    if (!sx1280MarkBusy()) {
        return false;
    }

    /* Ensure BUSY EXTI is enabled
     *
     * This is needed because the BETAFPV F4SX1280 target defines the following resources which cannot be
     * simultaneously used with the EXTI15_10_IRQHandler. Fortunately we can enable  RX_SPI_EXTI until an
     * interrupt is received, then enable RX_SPI_EXPRESSLRS_BUSY with the call below until data transfers
     * are complete and then switch back with a call to sx1280EnableExti().
     *
     * resource RX_SPI_EXTI 1 C13
     * resource RX_SPI_EXPRESSLRS_BUSY 1 A13
     *
     */

    EXTIConfig(busy, &busyIntContext.exti, NVIC_PRIO_RX_BUSY_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_FALLING);

    return true;
}

// waitingFn() must call sx1280ClearBusyFn() to prevent repeated calls

static void sx1280SetBusyFn(extiHandlerCallback *waitingFn)
{
    bool sx1280Busy;

    ATOMIC_BLOCK(NVIC_PRIO_RX_BUSY_EXTI) {
        sx1280Busy = IORead(busy);
        if (sx1280Busy) {
            EXTIHandlerInit(&busyIntContext.exti, waitingFn);
            EXTIEnable(busy);
        } else {
            EXTIDisable(busy);
        }
    }

    if (!sx1280Busy) {
        waitingFn(&busyIntContext.exti);
    }
}

static void sx1280MarkFree(void)
{
    // Mark that current sequence of accesses is concluded
    sx1280Processing = (timeUs_t)0;
}

// Switch to waiting for EXTI interrupt
static void sx1280EnableExti(void)
{
    sx1280MarkFree();
    rxSpiEnableExti();
}

// Unlikely as it is for the code to lock up waiting on a busy SX1280, we can't afford the risk
// If this routine is called twice in succession whilst waiting on the same busy, force the code to advance
// Called from the Tick timer
bool sx1280HandleFromTick(void)
{
    // Grab a copy to prevent a race condition
    timeUs_t startTime = sx1280Processing;

    if (startTime) {
        // No operation should take SX1280_BUSY_TIMEOUT_US us
        if (cmpTimeUs(micros(), startTime) > SX1280_BUSY_TIMEOUT_US) {
            // Brute force abandon the current sequence of operations
            sx1280ClearBusyFn();
            // Renable EXTI
            sx1280EnableExti();

            return true;
        }
    }
    
    return false;
}

bool sx1280Init(IO_t resetPin, IO_t busyPin)
{
    if (!rxSpiExtiConfigured()) {
        return false;
    }

    rxSpiSetNormalSpeedMhz(SX1280_MAX_SPI_MHZ);
    rxSpiNormalSpeed();

    if (resetPin) {
        IOInit(resetPin, OWNER_RX_SPI_EXPRESSLRS_RESET, 0);
        IOConfigGPIO(resetPin, IOCFG_OUT_PP);
    } else {
        resetPin = IO_NONE;
    }

    if (busyPin) {
        IOInit(busyPin, OWNER_RX_SPI_EXPRESSLRS_BUSY, 0);
        IOConfigGPIO(busyPin, IOCFG_IN_FLOATING);
    } else {
        busyPin = IO_NONE;
    }

    busy = busyPin;

    IOLo(resetPin);
    delay(50);
    IOConfigGPIO(resetPin, IOCFG_IN_FLOATING); // leave floating, internal pullup on sx1280 side
    delay(20);

    uint16_t firmwareRev = (((sx1280ReadRegister(REG_LR_FIRMWARE_VERSION_MSB)) << 8) | (sx1280ReadRegister(REG_LR_FIRMWARE_VERSION_MSB + 1)));

    if ((firmwareRev == 0) || (firmwareRev == 65535)) {
        return false;
    }

    // Record the dev pointer for callbacks
    extDevice_t *dev = rxSpiGetDevice();
    dev->callbackArg = (uint32_t)dev;

    return true;
}

void sx1280WriteCommand(const uint8_t address, const uint8_t data)
{
    sx1280PollBusy();
    rxSpiWriteCommand(address, data);
}

void sx1280WriteCommandBurst(const uint8_t address, const uint8_t *data, const uint8_t length)
{
    uint8_t outBuffer[length + 1];

    outBuffer[0] = address;

    memcpy(outBuffer + 1, data, length);

    sx1280PollBusy();
    rxSpiTransferCommandMulti(&outBuffer[0], length + 1);
}

void sx1280ReadCommandBurst(const uint8_t address, uint8_t *data, const uint8_t length)
{
    uint8_t outBuffer[length + 2];

    outBuffer[0] = address;
    outBuffer[1] = 0x00;

    memcpy(outBuffer + 2, data, length);

    sx1280PollBusy();
    rxSpiTransferCommandMulti(&outBuffer[0], length + 2);
    memcpy(data, outBuffer + 2, length);
}

void sx1280WriteRegisterBurst(const uint16_t address, const uint8_t *buffer, const uint8_t size)
{
    uint8_t outBuffer[size + 3];

    outBuffer[0] = (uint8_t) SX1280_RADIO_WRITE_REGISTER;
    outBuffer[1] = ((address & 0xFF00) >> 8);
    outBuffer[2] = (address & 0x00FF);

    memcpy(outBuffer + 3, buffer, size);

    sx1280PollBusy();
    rxSpiTransferCommandMulti(&outBuffer[0], size + 3);
}

void sx1280WriteRegister(const uint16_t address, const uint8_t value)
{
    sx1280WriteRegisterBurst(address, &value, 1);
}

void sx1280ReadRegisterBurst(const uint16_t address, uint8_t *buffer, const uint8_t size)
{
    uint8_t outBuffer[size + 4];

    outBuffer[0] = (uint8_t) SX1280_RADIO_READ_REGISTER;
    outBuffer[1] = ((address & 0xFF00) >> 8);
    outBuffer[2] = (address & 0x00FF);
    outBuffer[3] = 0x00;

    sx1280PollBusy();
    rxSpiTransferCommandMulti(&outBuffer[0], size + 4);
    memcpy(buffer, outBuffer + 4, size);
}

uint8_t sx1280ReadRegister(const uint16_t address)
{
    uint8_t data;
    sx1280ReadRegisterBurst(address, &data, 1);
    return data;
}

void sx1280WriteBuffer(const uint8_t offset, const uint8_t *buffer, const uint8_t size)
{
    uint8_t outBuffer[size + 2];

    outBuffer[0] = (uint8_t) SX1280_RADIO_WRITE_BUFFER;
    outBuffer[1] = offset;

    memcpy(outBuffer + 2, buffer, size);

    sx1280PollBusy();
    rxSpiTransferCommandMulti(&outBuffer[0], size + 2);
}

void sx1280ReadBuffer(const uint8_t offset, uint8_t *buffer, const uint8_t size)
{
    uint8_t outBuffer[size + 3];

    outBuffer[0] = (uint8_t) SX1280_RADIO_READ_BUFFER;
    outBuffer[1] = offset;
    outBuffer[2] = 0x00;

    sx1280PollBusy();
    rxSpiTransferCommandMulti(&outBuffer[0], size + 3);
    memcpy(buffer, outBuffer + 3, size);
}

uint8_t sx1280GetStatus(void)
{
    uint8_t buffer[3] = {(uint8_t) SX1280_RADIO_GET_STATUS, 0, 0};
    sx1280PollBusy();
    rxSpiTransferCommandMulti(&buffer[0], 3);
    return buffer[0];
}

void sx1280ConfigLoraDefaults(void)
{
    sx1280SetMode(SX1280_MODE_STDBY_RC);                                      //step 1 put in STDBY_RC mode
    sx1280WriteCommand(SX1280_RADIO_SET_PACKETTYPE, SX1280_PACKET_TYPE_LORA); //Step 2: set packet type to LoRa
    sx1280ConfigLoraModParams(SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_4_7); //Step 5: Configure Modulation Params
    sx1280WriteCommand(SX1280_RADIO_SET_AUTOFS, 0x01);                        //enable auto FS
    sx1280WriteRegister(0x0891, (sx1280ReadRegister(0x0891) | 0xC0));         //default is low power mode, switch to high sensitivity instead
    sx1280SetPacketParams(12, SX1280_LORA_PACKET_IMPLICIT, 8, SX1280_LORA_CRC_OFF, SX1280_LORA_IQ_NORMAL); //default params
    sx1280SetFrequencyReg(fhssGetInitialFreq(0));                             //Step 3: Set Freq
    sx1280SetFifoAddr(0x00, 0x00);                                            //Step 4: Config FIFO addr
    sx1280SetDioIrqParams(SX1280_IRQ_RADIO_ALL, SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE, SX1280_IRQ_RADIO_NONE, SX1280_IRQ_RADIO_NONE); //set IRQ to both RXdone/TXdone on DIO1
}

void sx1280Config(const sx1280LoraBandwidths_e bw, const sx1280LoraSpreadingFactors_e sf, const sx1280LoraCodingRates_e cr, 
                  const uint32_t freq, const uint8_t preambleLength, const bool iqInverted)
{
    sx1280SetMode(SX1280_MODE_SLEEP);
    sx1280PollBusy();

    sx1280ConfigLoraDefaults();
    sx1280SetOutputPower(13); //default is max power (12.5dBm for SX1280 RX)
    sx1280SetMode(SX1280_MODE_STDBY_RC); 
    sx1280ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    sx1280ConfigLoraModParams(bw, sf, cr);
    sx1280SetPacketParams(preambleLength, SX1280_LORA_PACKET_IMPLICIT, 8, SX1280_LORA_CRC_OFF, (sx1280LoraIqModes_e)((uint8_t)!iqInverted << 6)); // TODO don't make static etc.
    sx1280SetFrequencyReg(freq);
}

void sx1280SetOutputPower(const int8_t power)
{
    uint8_t buf[2];
    buf[0] = power + 18;
    buf[1] = (uint8_t) SX1280_RADIO_RAMP_04_US;
    sx1280WriteCommandBurst(SX1280_RADIO_SET_TXPARAMS, buf, 2);
}

void sx1280SetPacketParams(const uint8_t preambleLength, const sx1280LoraPacketLengthsModes_e headerType, const uint8_t payloadLength, 
                           const sx1280LoraCrcModes_e crc, const sx1280LoraIqModes_e invertIQ)
{
    uint8_t buf[7];

    buf[0] = preambleLength;
    buf[1] = headerType;
    buf[2] = payloadLength;
    buf[3] = crc;
    buf[4] = invertIQ;
    buf[5] = 0x00;
    buf[6] = 0x00;

    sx1280WriteCommandBurst(SX1280_RADIO_SET_PACKETPARAMS, buf, 7);
}

void sx1280SetMode(const sx1280OperatingModes_e opMode)
{
    uint8_t buf[3];

    switch (opMode) {
    case SX1280_MODE_SLEEP:
        sx1280WriteCommand(SX1280_RADIO_SET_SLEEP, 0x01);
        break;
    case SX1280_MODE_CALIBRATION:
        break;
    case SX1280_MODE_STDBY_RC:
        sx1280WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_RC);
        break;
    case SX1280_MODE_STDBY_XOSC:
        sx1280WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_XOSC);
        break;
    case SX1280_MODE_FS:
        sx1280WriteCommand(SX1280_RADIO_SET_FS, 0x00);
        break;
    case SX1280_MODE_RX:
        buf[0] = 0x00; // periodBase = 1ms, page 71 datasheet, set to FF for cont RX
        buf[1] = 0xFF;
        buf[2] = 0xFF;
        sx1280WriteCommandBurst(SX1280_RADIO_SET_RX, buf, 3);
        break;
    case SX1280_MODE_TX:
        //uses timeout Time-out duration = periodBase * periodBaseCount
        buf[0] = 0x00; // periodBase = 1ms, page 71 datasheet
        buf[1] = 0xFF; // no timeout set for now
        buf[2] = 0xFF; // TODO dynamic timeout based on expected onairtime
        sx1280WriteCommandBurst(SX1280_RADIO_SET_TX, buf, 3);
        break;
    case SX1280_MODE_CAD: // not implemented yet
    default:
        break;
    }
}

void sx1280ConfigLoraModParams(const sx1280LoraBandwidths_e bw, const sx1280LoraSpreadingFactors_e sf, const sx1280LoraCodingRates_e cr)
{
    // Care must therefore be taken to ensure that modulation parameters are set using the command
    // SetModulationParam() only after defining the packet type SetPacketType() to be used

    uint8_t rfparams[3] = {0};

    rfparams[0] = (uint8_t)sf;
    rfparams[1] = (uint8_t)bw;
    rfparams[2] = (uint8_t)cr;

    sx1280WriteCommandBurst(SX1280_RADIO_SET_MODULATIONPARAMS, rfparams, 3);

    switch (sf) {
    case SX1280_LORA_SF5:
    case SX1280_LORA_SF6:
        sx1280WriteRegister(0x925, 0x1E); // for SF5 or SF6
        break;
    case SX1280_LORA_SF7:
    case SX1280_LORA_SF8:
        sx1280WriteRegister(0x925, 0x37); // for SF7 or SF8
        break;
    default:
        sx1280WriteRegister(0x925, 0x32); // for SF9, SF10, SF11, SF12
    }
}

void sx1280SetFrequencyReg(const uint32_t freqReg)
{
    uint8_t buf[3] = {0};

    buf[0] = (uint8_t)((freqReg >> 16) & 0xFF);
    buf[1] = (uint8_t)((freqReg >> 8) & 0xFF);
    buf[2] = (uint8_t)(freqReg & 0xFF);

    sx1280WriteCommandBurst(SX1280_RADIO_SET_RFFREQUENCY, buf, 3);
}

void sx1280AdjustFrequency(int32_t offset, const uint32_t freq)
{
    // just a stub to show that frequency adjustment is not used on this chip as opposed to sx127x
    UNUSED(offset);
    UNUSED(freq);
}

void sx1280SetFifoAddr(const uint8_t txBaseAddr, const uint8_t rxBaseAddr)
{
    uint8_t buf[2];

    buf[0] = txBaseAddr;
    buf[1] = rxBaseAddr;
    sx1280WriteCommandBurst(SX1280_RADIO_SET_BUFFERBASEADDRESS, buf, 2);
}

void sx1280SetDioIrqParams(const uint16_t irqMask, const uint16_t dio1Mask, const uint16_t dio2Mask, const uint16_t dio3Mask)
{
    uint8_t buf[8];

    buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irqMask & 0x00FF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1Mask & 0x00FF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2Mask & 0x00FF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3Mask & 0x00FF);

    sx1280WriteCommandBurst(SX1280_RADIO_SET_DIOIRQPARAMS, buf, 8);
}

void sx1280TransmitData(const uint8_t *data, const uint8_t length)
{
    sx1280WriteBuffer(0x00, data, length);
    sx1280SetMode(SX1280_MODE_TX);
}

static uint8_t sx1280GetRxBufferAddr(void)
{
    uint8_t status[2] = {0};
    sx1280ReadCommandBurst(SX1280_RADIO_GET_RXBUFFERSTATUS, status, 2);
    return status[1];
}

void sx1280ReceiveData(uint8_t *data, const uint8_t length)
{
    uint8_t FIFOaddr = sx1280GetRxBufferAddr();
    sx1280ReadBuffer(FIFOaddr, data, length);
}

void sx1280StartReceiving(void)
{
    if (sx1280MarkBusy()) {
        sx1280SetMode(SX1280_MODE_RX);
        sx1280MarkFree();
    }
}

void sx1280GetLastPacketStats(int8_t *rssi, int8_t *snr)
{
    *rssi = -(int8_t)(packetStats[0] / 2);
    *snr = (int8_t) packetStats[1];
    int8_t negOffset = (*snr < 0) ? (*snr / 4) : 0;
    *rssi += negOffset;
}

void sx1280DoFHSS(void)
{
    return;
}

void sx1280ClearIrqStatus(const uint16_t irqMask)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(((uint16_t)irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irqMask & 0x00FF);

    sx1280WriteCommandBurst(SX1280_RADIO_CLR_IRQSTATUS, buf, 2);
}

// Forward Definitions for DMA Chain //
static void sx1280IrqGetStatus(extiCallbackRec_t *cb);
static busStatus_e sx1280IrqStatusRead(uint32_t arg);
static void sx1280IrqClearStatus(extiCallbackRec_t *cb);
static busStatus_e sx1280IrqCmdComplete(uint32_t arg);
static void sx1280ProcessIrq(extiCallbackRec_t *cb);
static busStatus_e sx1280GotFIFOAddr(uint32_t arg);
static void sx1280DoReadBuffer(extiCallbackRec_t *cb);
static busStatus_e sx1280ReadBufferComplete(uint32_t arg);
static void sx1280GetPacketStats(extiCallbackRec_t *cb);
static busStatus_e sx1280GetStatsCmdComplete(uint32_t arg);
static busStatus_e sx1280IsFhssReq(uint32_t arg);
static void sx1280SetFrequency(extiCallbackRec_t *cb);
static busStatus_e sx1280SetFreqComplete(uint32_t arg);
static void sx1280StartReceivingDMA(extiCallbackRec_t *cb);
static busStatus_e sx1280EnableIRQs(uint32_t arg);
static void sx1280SendTelemetryBuffer(extiCallbackRec_t *cb);
static busStatus_e sx1280TelemetryComplete(uint32_t arg);
static void sx1280StartTransmittingDMA(extiCallbackRec_t *cb);

FAST_IRQ_HANDLER void sx1280ISR(void)
{
    // Only attempt to access the SX1280 if it is currently idle to avoid any race condition
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (sx1280EnableBusy()) {
            pendingISR = false;
            sx1280SetBusyFn(sx1280IrqGetStatus);
        } else {
            pendingISR = true;
        }
    }
}

// Next, the reason for the IRQ must be read

FAST_IRQ_HANDLER static void sx1280IrqGetStatus(extiCallbackRec_t *cb)
{
    extDevice_t *dev = rxSpiGetDevice();

    UNUSED(cb);

    sx1280ClearBusyFn();

    STATIC_DMA_DATA_AUTO uint8_t irqStatusCmd[] = {SX1280_RADIO_GET_IRQSTATUS, 0, 0, 0};
    STATIC_DMA_DATA_AUTO uint8_t irqStatus[sizeof(irqStatusCmd)];

    static busSegment_t segments[] = {
            {.u.buffers = {irqStatusCmd, irqStatus}, sizeof(irqStatusCmd), true, sx1280IrqStatusRead},
            {.u.link = {NULL, NULL}, 0, false, NULL},
    };

    spiSequence(dev, segments);
}

// Read the IRQ status, and save it to irqStatus variable

FAST_IRQ_HANDLER static busStatus_e sx1280IrqStatusRead(uint32_t arg)
{
    extDevice_t *dev = (extDevice_t *)arg;

    uint16_t irqStatus = (dev->bus->curSegment->u.buffers.rxData[2] << 8) | dev->bus->curSegment->u.buffers.rxData[3];

    if (irqStatus & SX1280_IRQ_TX_DONE) {
        irqReason = ELRS_DIO_TX_DONE;
    } else if (irqStatus & SX1280_IRQ_RX_DONE) {
        irqReason = ELRS_DIO_RX_DONE;
    } else {
        irqReason = ELRS_DIO_UNKNOWN;
    }

    sx1280SetBusyFn(sx1280IrqClearStatus);
    return BUS_READY;
}

// Clear the IRQ bit in the Radio registers

FAST_IRQ_HANDLER static void sx1280IrqClearStatus(extiCallbackRec_t *cb)
{
    extDevice_t *dev = rxSpiGetDevice();

    UNUSED(cb);

    sx1280ClearBusyFn();

    STATIC_DMA_DATA_AUTO uint8_t irqCmd[] = {SX1280_RADIO_CLR_IRQSTATUS, 0, 0};

    irqCmd[1] = (uint8_t)(((uint16_t)SX1280_IRQ_RADIO_ALL >> 8) & 0x00FF);
    irqCmd[2] = (uint8_t)((uint16_t)SX1280_IRQ_RADIO_ALL & 0x00FF);

    static busSegment_t segments[] = {
            {.u.buffers = {irqCmd, NULL}, sizeof(irqCmd), true, sx1280IrqCmdComplete},
            {.u.link = {NULL, NULL}, 0, false, NULL},
    };

    spiSequence(dev, segments);
}

// Callback follow clear of IRQ status
FAST_IRQ_HANDLER static busStatus_e sx1280IrqCmdComplete(uint32_t arg)
{
    UNUSED(arg);

    sx1280SetBusyFn(sx1280ProcessIrq);

    return BUS_READY;
}

// Process IRQ status
FAST_IRQ_HANDLER static void sx1280ProcessIrq(extiCallbackRec_t *cb)
{
    extDevice_t *dev = rxSpiGetDevice();

    UNUSED(cb);

    sx1280ClearBusyFn();

    if (irqReason == ELRS_DIO_RX_DONE || irqReason == ELRS_DIO_UNKNOWN) {
        // Fire off the chain to read and decode the packet from the radio
        // Get the buffer status to determine the FIFO address
        STATIC_DMA_DATA_AUTO uint8_t cmdBufStatusCmd[] = {SX1280_RADIO_GET_RXBUFFERSTATUS, 0, 0, 0};
        STATIC_DMA_DATA_AUTO uint8_t bufStatus[sizeof(cmdBufStatusCmd)];

        static busSegment_t segments[] = {
            {.u.buffers = {cmdBufStatusCmd, bufStatus}, sizeof(cmdBufStatusCmd), true, sx1280GotFIFOAddr},
            {.u.link = {NULL, NULL}, 0, false, NULL},
        };

        spiSequence(dev, segments);

    } else {
        // return to RX mode immediately, the next packet will be an RX and we won't need to FHSS
        STATIC_DMA_DATA_AUTO uint8_t irqSetRxCmd[] = {SX1280_RADIO_SET_RX, 0, 0xff, 0xff};

        static busSegment_t segments[] = {
            {.u.buffers = {irqSetRxCmd, NULL}, sizeof(irqSetRxCmd), true, sx1280EnableIRQs},
            {.u.link = {NULL, NULL}, 0, false, NULL},
        };

        spiSequence(dev, segments);
    }
}

// First we read from the FIFO address register to determine the FIFO address
static busStatus_e sx1280GotFIFOAddr(uint32_t arg)
{
    extDevice_t *dev = (extDevice_t *)arg;

    FIFOaddr = dev->bus->curSegment->u.buffers.rxData[3];

    // Wait until no longer busy and read the buffer
    sx1280SetBusyFn(sx1280DoReadBuffer);

    return BUS_READY;
}

// Using the addr val stored to the global varable FIFOaddr, read the buffer
static void sx1280DoReadBuffer(extiCallbackRec_t *cb)
{
    extDevice_t *dev = rxSpiGetDevice();

    UNUSED(cb);

    sx1280ClearBusyFn();

    STATIC_DMA_DATA_AUTO uint8_t cmdReadBuf[] = {SX1280_RADIO_READ_BUFFER, 0, 0};

    cmdReadBuf[1] = FIFOaddr;

    static busSegment_t segments[] = {
            {.u.buffers = {cmdReadBuf, NULL}, sizeof(cmdReadBuf), false, NULL},
            {.u.buffers = {NULL, NULL}, ELRS_RX_TX_BUFF_SIZE, true, sx1280ReadBufferComplete},
            {.u.link = {NULL, NULL}, 0, false, NULL},
    };

    segments[1].u.buffers.rxData = (uint8_t *)expressLrsGetRxBuffer();

    spiSequence(dev, segments);
}

// Get the Packet Status and RSSI
static busStatus_e sx1280ReadBufferComplete(uint32_t arg)
{
    UNUSED(arg);

    sx1280SetBusyFn(sx1280GetPacketStats);

    return BUS_READY;
}

// Save the Packet Stats to the global variables
static void sx1280GetPacketStats(extiCallbackRec_t *cb)
{
    UNUSED(cb);

    extDevice_t *dev = rxSpiGetDevice();

    sx1280ClearBusyFn();

    STATIC_DMA_DATA_AUTO uint8_t getStatsCmd[] = {SX1280_RADIO_GET_PACKETSTATUS, 0, 0, 0};
    STATIC_DMA_DATA_AUTO uint8_t stats[sizeof(getStatsCmd)];

    static busSegment_t segments[] = {
            {.u.buffers = {getStatsCmd, stats}, sizeof(getStatsCmd), true, sx1280GetStatsCmdComplete},
            {.u.link = {NULL, NULL}, 0, false, NULL},
    };

    spiSequence(dev, segments);
}

// Process and decode the RF packet 
static busStatus_e sx1280GetStatsCmdComplete(uint32_t arg)
{
    extDevice_t *dev = (extDevice_t *)arg;
    volatile uint8_t *payload = expressLrsGetPayloadBuffer();

    packetStats[0] = dev->bus->curSegment->u.buffers.rxData[2];
    packetStats[1] = dev->bus->curSegment->u.buffers.rxData[3];

    expressLrsSetRfPacketStatus(processRFPacket(payload, rxSpiGetLastExtiTimeUs()));

    return sx1280IsFhssReq(arg);
}

void sx1280HandleFromTock(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (expressLrsIsFhssReq()) {
            if (sx1280EnableBusy()) {
                pendingDoFHSS = false;
                sx1280SetBusyFn(sx1280SetFrequency);
            } else {
                pendingDoFHSS = true;
            }
        }
    }
}

// Next we need to check if we need to FHSS and then do so if needed
static busStatus_e sx1280IsFhssReq(uint32_t arg)
{
    UNUSED(arg);

    if (expressLrsIsFhssReq()) {
        sx1280SetBusyFn(sx1280SetFrequency);   
    } else {
        sx1280SetFreqComplete(arg);
    }

    return BUS_READY;
}

// Set the frequency
static void sx1280SetFrequency(extiCallbackRec_t *cb)
{
    UNUSED(cb);

    extDevice_t *dev = rxSpiGetDevice();
    uint32_t currentFreq = expressLrsGetCurrentFreq();

    sx1280ClearBusyFn();

    STATIC_DMA_DATA_AUTO uint8_t setFreqCmd[] = {SX1280_RADIO_SET_RFFREQUENCY, 0, 0, 0};
    setFreqCmd[1] = (uint8_t)((currentFreq >> 16) & 0xFF);
    setFreqCmd[2] = (uint8_t)((currentFreq >> 8) & 0xFF);
    setFreqCmd[3] = (uint8_t)(currentFreq & 0xFF);

    static busSegment_t segments[] = {
            {.u.buffers = {setFreqCmd, NULL}, sizeof(setFreqCmd), true, sx1280SetFreqComplete},
            {.u.link = {NULL, NULL}, 0, false, NULL},
    };

    spiSequence(dev, segments);
}

// Determine if we need to go back to RX or if we need to send TLM data
static busStatus_e sx1280SetFreqComplete(uint32_t arg)
{
    UNUSED(arg);

    if (expressLrsTelemRespReq()) {
        expressLrsDoTelem();
        // if it's time to do TLM and we have enough to do so
        sx1280SetBusyFn(sx1280SendTelemetryBuffer);
    } else {
        // we don't need to send TLM and we've already FHSS so just hop back into RX mode
        sx1280SetBusyFn(sx1280StartReceivingDMA);
    }

    return BUS_READY;
}

// Go back into RX mode
static void sx1280StartReceivingDMA(extiCallbackRec_t *cb)
{
    UNUSED(cb);
    extDevice_t *dev = rxSpiGetDevice();

    sx1280ClearBusyFn();

    // Issue command to start receiving
    // periodBase = 1ms, page 71 datasheet, set to FF for cont RX
    STATIC_DMA_DATA_AUTO uint8_t irqSetRxCmd[] = {SX1280_RADIO_SET_RX, 0, 0xff, 0xff};

    static busSegment_t segments[] = {
            {.u.buffers = {irqSetRxCmd, NULL}, sizeof(irqSetRxCmd), true, sx1280EnableIRQs},
            {.u.link = {NULL, NULL}, 0, false, NULL},
    };

    spiSequence(dev, segments);
}

static busStatus_e sx1280EnableIRQs(uint32_t arg)
{
    UNUSED(arg);

    // Handle any queued interrupt processing
    if (pendingISR) {
        pendingISR = false;
        sx1280SetBusyFn(sx1280IrqGetStatus);
    } else if (pendingDoFHSS) {
        pendingDoFHSS = false;
        sx1280SetBusyFn(sx1280SetFrequency);
    } else {
        // Switch back to waiting for EXTI interrupt
        sx1280EnableExti();
    }

    return BUS_READY;
}


// Send telemetry response 
static void sx1280SendTelemetryBuffer(extiCallbackRec_t *cb)
{
    UNUSED(cb);
    extDevice_t *dev = rxSpiGetDevice();

    sx1280ClearBusyFn();

    STATIC_DMA_DATA_AUTO uint8_t writeBufferCmd[] = {SX1280_RADIO_WRITE_BUFFER, 0};

    static busSegment_t segments[] = {
            {.u.buffers = {writeBufferCmd, NULL}, sizeof(writeBufferCmd), false, NULL},
            {.u.buffers = {NULL, NULL}, ELRS_RX_TX_BUFF_SIZE, true, sx1280TelemetryComplete},
            {.u.link = {NULL, NULL}, 0, false, NULL},
    };

    segments[1].u.buffers.txData = (uint8_t *)expressLrsGetTelemetryBuffer();

    spiSequence(dev, segments);
}

static busStatus_e sx1280TelemetryComplete(uint32_t arg)
{
    UNUSED(arg);

    sx1280SetBusyFn(sx1280StartTransmittingDMA);

    return BUS_READY;
}

static void sx1280StartTransmittingDMA(extiCallbackRec_t *cb)
{
    UNUSED(cb);
    extDevice_t *dev = rxSpiGetDevice();

    sx1280ClearBusyFn();

    //uses timeout Time-out duration = periodBase * periodBaseCount
    // periodBase = 1ms, page 71 datasheet
    // no timeout set for now
    // TODO dynamic timeout based on expected onairtime
    STATIC_DMA_DATA_AUTO uint8_t irqSetRxCmd[] = {SX1280_RADIO_SET_TX, 0, 0xff, 0xff};

    static busSegment_t segments[] = {
            {.u.buffers = {irqSetRxCmd, NULL}, sizeof(irqSetRxCmd), true, sx1280EnableIRQs},
            {.u.link = {NULL, NULL}, 0, false, NULL},
    };

    spiSequence(dev, segments);
}
#endif /* USE_RX_SX1280 */
