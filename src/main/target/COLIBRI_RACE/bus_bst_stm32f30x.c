/*                                             By Larry Ho Ka Wai @ 23/06/2015*/

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include <platform.h>

#include <build_config.h>

#include "drivers/nvic.h"
#include "bus_bst.h"


#ifdef USE_BST
#define NVIC_PRIO_BST_READ_DATA  NVIC_BUILD_PRIORITY(1, 1)

#define BST_SHORT_TIMEOUT        ((uint32_t)0x1000)
#define BST_LONG_TIMEOUT         ((uint32_t)(10 * BST_SHORT_TIMEOUT))

#if !defined(BST1_SCL_GPIO)
#define BST1_SCL_GPIO            GPIOB
#define BST1_SCL_GPIO_AF         GPIO_AF_4
#define BST1_SCL_PIN             GPIO_Pin_6
#define BST1_SCL_PIN_SOURCE      GPIO_PinSource6
#define BST1_SCL_CLK_SOURCE      RCC_AHBPeriph_GPIOB
#define BST1_SDA_GPIO            GPIOB
#define BST1_SDA_GPIO_AF         GPIO_AF_4
#define BST1_SDA_PIN             GPIO_Pin_7
#define BST1_SDA_PIN_SOURCE      GPIO_PinSource7
#define BST1_SDA_CLK_SOURCE      RCC_AHBPeriph_GPIOB
#endif

#if !defined(BST2_SCL_GPIO)
#define BST2_SCL_GPIO            GPIOF
#define BST2_SCL_GPIO_AF         GPIO_AF_4
#define BST2_SCL_PIN             GPIO_Pin_6
#define BST2_SCL_PIN_SOURCE      GPIO_PinSource6
#define BST2_SCL_CLK_SOURCE      RCC_AHBPeriph_GPIOF
#define BST2_SDA_GPIO            GPIOA
#define BST2_SDA_GPIO_AF         GPIO_AF_4
#define BST2_SDA_PIN             GPIO_Pin_10
#define BST2_SDA_PIN_SOURCE      GPIO_PinSource10
#define BST2_SDA_CLK_SOURCE      RCC_AHBPeriph_GPIOA
#endif

static volatile uint16_t bst1ErrorCount = 0;
static volatile uint16_t bst2ErrorCount = 0;

static I2C_TypeDef *BSTx = NULL;

volatile uint8_t CRC8 = 0;
volatile bool coreProReady = false;

///////////////////////////////////////////////////////////////////////////////
// BST TimeoutUserCallback
///////////////////////////////////////////////////////////////////////////////

uint8_t dataBuffer[DATA_BUFFER_SIZE] = {0};
uint8_t dataBufferPointer = 0;
uint8_t bstWriteDataLen = 0;

uint32_t micros(void);

uint8_t writeData[DATA_BUFFER_SIZE] = {0};
uint8_t currentWriteBufferPointer = 0;
bool receiverAddress = false;

uint8_t readData[DATA_BUFFER_SIZE] = {0};
uint8_t bufferPointer = 0;

bool cleanflight_data_ready = false;
uint8_t interruptCounter = 0;
#define DELAY_SENDING_BYTE    40

void bstProcessInCommand(void);
void I2C_EV_IRQHandler()
{
    if(I2C_GetITStatus(BSTx, I2C_IT_ADDR)) {
        CRC8 = 0;
        if(I2C_GetTransferDirection(BSTx) == I2C_Direction_Receiver) {
            currentWriteBufferPointer = 0;
            receiverAddress = true;
            I2C_SendData(BSTx, (uint8_t) writeData[currentWriteBufferPointer++]);
               I2C_ITConfig(BSTx, I2C_IT_TXI, ENABLE);
        } else {
            readData[0] = I2C_GetAddressMatched(BSTx);
            bufferPointer = 1;
        }
        I2C_ClearITPendingBit(BSTx, I2C_IT_ADDR);
    } else if(I2C_GetITStatus(BSTx, I2C_IT_RXNE)) {
        uint8_t data = I2C_ReceiveData(BSTx);
        readData[bufferPointer] = data;
        if(bufferPointer > 1) {
            if(readData[1]+1 == bufferPointer) {
                crc8Cal(0);
                bstProcessInCommand();
            } else {
                crc8Cal(data);
            }
        }
        bufferPointer++;
        I2C_ClearITPendingBit(BSTx, I2C_IT_RXNE);
    } else if(I2C_GetITStatus(BSTx, I2C_IT_TXIS)) {
        if(receiverAddress) {
            if(currentWriteBufferPointer > 0) {
                if(!cleanflight_data_ready) {
                    I2C_ClearITPendingBit(BSTx, I2C_IT_TXIS);
                    return;
                }
                if(interruptCounter < DELAY_SENDING_BYTE) {
                    interruptCounter++;
                    I2C_ClearITPendingBit(BSTx, I2C_IT_TXIS);
                    return;
                } else {
                    interruptCounter = 0;
                }
                if(writeData[0] == currentWriteBufferPointer) {
                    receiverAddress = false;
                    crc8Cal(0);
                    I2C_SendData(BSTx, (uint8_t) CRC8);
                       I2C_ITConfig(BSTx, I2C_IT_TXI, DISABLE);
                } else {
                    crc8Cal((uint8_t) writeData[currentWriteBufferPointer]);
                    I2C_SendData(BSTx, (uint8_t) writeData[currentWriteBufferPointer++]);
                }
            }
        } else if(bstWriteDataLen) {
            I2C_SendData(BSTx, (uint8_t) dataBuffer[dataBufferPointer]);
            if(bstWriteDataLen > 1)
                dataBufferPointer++;
            if(dataBufferPointer == bstWriteDataLen) {
                I2C_ITConfig(BSTx, I2C_IT_TXI, DISABLE);
                dataBufferPointer = 0;
                bstWriteDataLen = 0;
            }
        } else {
        }
        I2C_ClearITPendingBit(BSTx, I2C_IT_TXIS);
    } else if(I2C_GetITStatus(BSTx, I2C_IT_NACKF)) {
        if(receiverAddress) {
            receiverAddress = false;
            I2C_ITConfig(BSTx, I2C_IT_TXI, DISABLE);
        }
        I2C_ClearITPendingBit(BSTx, I2C_IT_NACKF);
    } else if(I2C_GetITStatus(BSTx, I2C_IT_STOPF)) {
        if(bstWriteDataLen && dataBufferPointer == bstWriteDataLen) {
            dataBufferPointer = 0;
            bstWriteDataLen = 0;
        }
        I2C_ClearITPendingBit(BSTx, I2C_IT_STOPF);
    } else if(I2C_GetITStatus(BSTx, I2C_IT_BERR)
            || I2C_GetITStatus(BSTx, I2C_IT_ARLO)
            || I2C_GetITStatus(BSTx, I2C_IT_OVR)) {
        bstTimeoutUserCallback();
        I2C_ClearITPendingBit(BSTx, I2C_IT_BERR | I2C_IT_ARLO | I2C_IT_OVR);
    }
}

void I2C1_EV_IRQHandler()
{
    I2C_EV_IRQHandler();
}

void I2C2_EV_IRQHandler()
{
    I2C_EV_IRQHandler();
}

uint32_t bstTimeoutUserCallback()
{
    if (BSTx == I2C1) {
        bst1ErrorCount++;
    } else {
        bst2ErrorCount++;
    }
    I2C_GenerateSTOP(BSTx, ENABLE);
    receiverAddress = false;
    dataBufferPointer = 0;
    bstWriteDataLen = 0;
    I2C_ITConfig(BSTx, I2C_IT_TXI, DISABLE);
    I2C_SoftwareResetCmd(BSTx);
    return false;
}

void bstInitPort(I2C_TypeDef *BSTx/*, uint8_t Address*/)
{
    NVIC_InitTypeDef nvic;
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef BST_InitStructure;

    if(BSTx == I2C1) {
        RCC_AHBPeriphClockCmd(BST1_SCL_CLK_SOURCE | BST1_SDA_CLK_SOURCE, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);

        GPIO_PinAFConfig(BST1_SCL_GPIO, BST1_SCL_PIN_SOURCE, BST1_SCL_GPIO_AF);
        GPIO_PinAFConfig(BST1_SDA_GPIO, BST1_SDA_PIN_SOURCE, BST1_SDA_GPIO_AF);

        GPIO_StructInit(&GPIO_InitStructure);
        I2C_StructInit(&BST_InitStructure);

        // Init pins
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

        GPIO_InitStructure.GPIO_Pin = BST1_SCL_PIN;
        GPIO_Init(BST1_SCL_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = BST1_SDA_PIN;
        GPIO_Init(BST1_SDA_GPIO, &GPIO_InitStructure);

        I2C_StructInit(&BST_InitStructure);

        BST_InitStructure.I2C_Mode = I2C_Mode_I2C;
        BST_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
        BST_InitStructure.I2C_DigitalFilter = 0x00;
        BST_InitStructure.I2C_OwnAddress1 = CLEANFLIGHT_FC;
        BST_InitStructure.I2C_Ack = I2C_Ack_Enable;
        BST_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
        BST_InitStructure.I2C_Timing = 0x30E0257A; // 100 Khz, 72Mhz Clock, Analog Filter Delay ON, Rise 100, Fall 10.

        I2C_Init(I2C1, &BST_InitStructure);

        I2C_GeneralCallCmd(I2C1, ENABLE);

        nvic.NVIC_IRQChannel = I2C1_EV_IRQn;
        nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_BST_READ_DATA);
        nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_BST_READ_DATA);
        nvic.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&nvic);

        I2C_ITConfig(I2C1, I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_ERRI, ENABLE);

        I2C_Cmd(I2C1, ENABLE);
    }

    if(BSTx == I2C2) {
        RCC_AHBPeriphClockCmd(BST2_SCL_CLK_SOURCE | BST2_SDA_CLK_SOURCE, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
        RCC_I2CCLKConfig(RCC_I2C2CLK_SYSCLK);

        GPIO_PinAFConfig(BST2_SCL_GPIO, BST2_SCL_PIN_SOURCE, BST2_SCL_GPIO_AF);
        GPIO_PinAFConfig(BST2_SDA_GPIO, BST2_SDA_PIN_SOURCE, BST2_SDA_GPIO_AF);

        GPIO_StructInit(&GPIO_InitStructure);
        I2C_StructInit(&BST_InitStructure);

        // Init pins
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

        GPIO_InitStructure.GPIO_Pin = BST2_SCL_PIN;
        GPIO_Init(BST2_SCL_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = BST2_SDA_PIN;
        GPIO_Init(BST2_SDA_GPIO, &GPIO_InitStructure);

        I2C_StructInit(&BST_InitStructure);

        BST_InitStructure.I2C_Mode = I2C_Mode_I2C;
        BST_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
        BST_InitStructure.I2C_DigitalFilter = 0x00;
        BST_InitStructure.I2C_OwnAddress1 = CLEANFLIGHT_FC;
        BST_InitStructure.I2C_Ack = I2C_Ack_Enable;
        BST_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
        BST_InitStructure.I2C_Timing = 0x30E0257A; // 100 Khz, 72Mhz Clock, Analog Filter Delay ON, Rise 100, Fall 10.

        I2C_Init(I2C2, &BST_InitStructure);

        I2C_GeneralCallCmd(I2C2, ENABLE);

        nvic.NVIC_IRQChannel = I2C2_EV_IRQn;
        nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_BST_READ_DATA);
        nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_BST_READ_DATA);
        nvic.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&nvic);

        I2C_ITConfig(I2C2, I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_ERRI, ENABLE);

        I2C_Cmd(I2C2, ENABLE);
    }
}

void bstInit(BSTDevice index)
{
    if (index == BSTDEV_1) {
        BSTx = I2C1;
    } else {
        BSTx = I2C2;
    }
    bstInitPort(BSTx);
}

uint16_t bstGetErrorCounter(void)
{
    if (BSTx == I2C1) {
        return bst1ErrorCount;
    }

    return bst2ErrorCount;

}

/*************************************************************************************************/

bool bstWriteBusy(void)
{
    if(bstWriteDataLen)
        return true;
    else
        return false;
}

bool bstMasterWrite(uint8_t* data)
{
    if(bstWriteDataLen==0) {
        CRC8 = 0;
        dataBufferPointer = 0;
        dataBuffer[0] = *data;
        dataBuffer[1] = *(data+1);
        bstWriteDataLen = dataBuffer[1] + 2;
        for(uint8_t i=2; i<bstWriteDataLen; i++) {
            if(i==(bstWriteDataLen-1)) {
                crc8Cal(0);
                dataBuffer[i] = CRC8;
            } else {
                dataBuffer[i] = *(data+i);
                crc8Cal((uint8_t)dataBuffer[i]);
            }
        }
        return true;
    }
    return false;
}

void bstMasterWriteLoop(void)
{
    static uint32_t bstMasterWriteTimeout = 0;
    uint32_t currentTime = micros();
    if(bstWriteDataLen && dataBufferPointer==0) {
        bool scl_set = false;
        if(BSTx == I2C1)
            scl_set = BST1_SCL_GPIO->IDR&BST1_SCL_PIN;
        else
            scl_set = BST2_SCL_GPIO->IDR&BST2_SCL_PIN;
        if(I2C_GetFlagStatus(BSTx, I2C_FLAG_BUSY)==RESET && scl_set) {
            I2C_TransferHandling(BSTx, dataBuffer[dataBufferPointer], dataBuffer[dataBufferPointer+1]+1, I2C_AutoEnd_Mode, I2C_Generate_Start_Write);
            I2C_ITConfig(BSTx, I2C_IT_TXI, ENABLE);
            dataBufferPointer = 1;
            bstMasterWriteTimeout = micros();
        }
    } else if(currentTime>bstMasterWriteTimeout+BST_SHORT_TIMEOUT) {
        bstTimeoutUserCallback();
    }
}

/*************************************************************************************************/
void crc8Cal(uint8_t data_in)
{
    /* Polynom = x^8+x^7+x^6+x^4+x^2+1 = x^8+x^7+x^6+x^4+x^2+X^0 */
    uint8_t Polynom = BST_CRC_POLYNOM;
    bool MSB_Flag;

    /* Step through each bit of the BYTE (8-bits) */
    for (uint8_t i = 0; i < 8; i++) {
        /* Clear the Flag */
        MSB_Flag = false;

        /* MSB_Set = 80; */
        if (CRC8 & 0x80) {
            MSB_Flag = true;
        }

        CRC8 <<= 1;

        /* MSB_Set = 80; */
        if (data_in & 0x80) {
            CRC8++;
        }
        data_in <<= 1;

        if (MSB_Flag == true) {
            CRC8 ^= Polynom;
        }
    }
}
#endif
