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
 * Original author: Alain (https://github.com/aroyer-qc)
 * Modified for BF source: Chris Hockuba (https://github.com/conkerkh)
 */

/* Include(s) -------------------------------------------------------------------------------------------------------*/

#include <stdbool.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SDCARD_SDIO

#include "drivers/sdmmc_sdio.h"

#include "pg/sdio.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sdio.h"
#include "x32m7xx_rcc.h"

typedef struct SD_Handle_s
{
    uint32_t          CSD[4];           // SD card specific data table
    uint32_t          CID[4];           // SD card identification number table
    volatile uint32_t RXCplt;          // SD RX Complete is equal 0 when no transfer
    volatile uint32_t TXCplt;          // SD TX Complete is equal 0 when no transfer

    uint32_t RXErrors;
    uint32_t TXErrors;
} SD_Handle_t;

sd_card_t card;

SD_CardInfo_t                      SD_CardInfo;
SD_CardType_t                      SD_CardType;

SD_Handle_t                        SD_Handle;

typedef struct sdioPin_s {
    ioTag_t pin;
    uint8_t af;
} sdioPin_t;

#define SDIO_PIN_D0  0
#define SDIO_PIN_D1  1
#define SDIO_PIN_D2  2
#define SDIO_PIN_D3  3
#define SDIO_PIN_CK  4
#define SDIO_PIN_CMD 5
#define SDIO_PIN_COUNT  6

#define SDIO_MAX_PINDEFS 2

typedef struct sdioHardware_s {
    SDMMC_Module *instance;
    IRQn_Type irqn;
    sdioPin_t sdioPinCK[SDIO_MAX_PINDEFS];
    sdioPin_t sdioPinCMD[SDIO_MAX_PINDEFS];
    sdioPin_t sdioPinD0[SDIO_MAX_PINDEFS];
    sdioPin_t sdioPinD1[SDIO_MAX_PINDEFS];
    sdioPin_t sdioPinD2[SDIO_MAX_PINDEFS];
    sdioPin_t sdioPinD3[SDIO_MAX_PINDEFS];
} sdioHardware_t;

// Possible pin assignments

#define PINDEF(device, pin, afnum) { DEFIO_TAG_E(pin), GPIO_AF ## afnum }

static const sdioHardware_t sdioPinHardware[SDIODEV_COUNT] = {
    {
        .instance = SDMMC1,
        .irqn = SDMMC1_IRQn,
        .sdioPinCK  = { PINDEF(1, PC12, 1) },
        .sdioPinCMD = { PINDEF(1, PD2,  2) },
        .sdioPinD0  = { PINDEF(1, PC8,  0), PINDEF(1, PB13,  2) },
        .sdioPinD1  = { PINDEF(1, PC9,  1) },
        .sdioPinD2  = { PINDEF(1, PC10, 1) },
        .sdioPinD3  = { PINDEF(1, PC11, 1) },
    },
    {
        .instance = SDMMC2,
        .irqn = SDMMC2_IRQn,
        .sdioPinCK  = { PINDEF(2, PC1,   2), PINDEF(2, PD6,  1) },
        .sdioPinCMD = { PINDEF(2, PA0,   1), PINDEF(2, PD7,  1) },
        .sdioPinD0  = { PINDEF(2, PB14,  1), PINDEF(2, PG9,  2) },
        .sdioPinD1  = { PINDEF(2, PB15,  1), PINDEF(2, PG10, 3) },
        .sdioPinD2  = { PINDEF(2, PB3,   0), PINDEF(2, PG11, 3) },
        .sdioPinD3  = { PINDEF(2, PB4,   1), PINDEF(2, PG12, 2) },
    }
};

#undef PINDEF

// Active configuration
static const sdioHardware_t *sdioHardware;
static sdioPin_t sdioPin[SDIO_PIN_COUNT];

static const sdioPin_t *sdioFindPinDef(const sdioPin_t *pindefs, ioTag_t pin)
{
    for (unsigned index = 0; index < SDIO_MAX_PINDEFS; index++) {
        if (pindefs[index].pin == pin) {
            return &pindefs[index];
        }
    }

    return NULL;
}

#define SDIOFINDPIN(pinname)  { \
        const sdioPin_t *pindef;                                                                     \
        pindef = sdioFindPinDef(sdioHardware->sdioPin ## pinname, sdioPinConfig()->pinname ## Pin);  \
        if (pindef) {                                                                                \
            sdioPin[SDIO_PIN_ ## pinname] = *pindef;                                           \
        }                                                                                            \
    } struct dummy

void sdioPinConfigure(void)
{
    SDIODevice device = SDIO_CFG_TO_DEV(sdioConfig()->device);

    if (device == SDIOINVALID) {
        return;
    }

    sdioHardware = &sdioPinHardware[device];

    SDIOFINDPIN(CK);
    SDIOFINDPIN(CMD);
    SDIOFINDPIN(D0);

    if (sdioConfig()->use4BitWidth) {
        SDIOFINDPIN(D1);
        SDIOFINDPIN(D2);
        SDIOFINDPIN(D3);
    }
}

#undef SDIOFINDPIN

#if defined(USE_SDIO_PULLUP)
#define IOCFG_SDMMC       IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SLEW_RATE_FAST, GPIO_PULL_UP, 0x00)
#else
#define IOCFG_SDMMC       IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SLEW_RATE_FAST, GPIO_NO_PULL, 0x00)
#endif

void SDMMC_port_config(void)
{
    if (!sdioHardware) {
        return;
    }

    RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOB | RCC_AHB5_PERIPHEN_M7_GPIOC | RCC_AHB5_PERIPHEN_M7_GPIOD 
        | RCC_AHB5_PERIPHEN_M7_GPIOA | RCC_AHB5_PERIPHEN_M7_GPIOG,ENABLE);
    RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_AFIO,ENABLE);

    uint8_t is4BitWidth = sdioConfig()->use4BitWidth;

    const IO_t clk = IOGetByTag(sdioPin[SDIO_PIN_CK].pin);
    const IO_t cmd = IOGetByTag(sdioPin[SDIO_PIN_CMD].pin);
    const IO_t d0 = IOGetByTag(sdioPin[SDIO_PIN_D0].pin);
    const IO_t d1 = IOGetByTag(sdioPin[SDIO_PIN_D1].pin);
    const IO_t d2 = IOGetByTag(sdioPin[SDIO_PIN_D2].pin);
    const IO_t d3 = IOGetByTag(sdioPin[SDIO_PIN_D3].pin);

    IOConfigGPIOAF(clk, IOCFG_SDMMC, sdioPin[SDIO_PIN_CK].af);
    IOConfigGPIOAF(cmd, IOCFG_SDMMC, sdioPin[SDIO_PIN_CMD].af);
    IOConfigGPIOAF(d0, IOCFG_SDMMC, sdioPin[SDIO_PIN_D0].af);

    if(is4BitWidth) {
        IOConfigGPIOAF(d1, IOCFG_SDMMC, sdioPin[SDIO_PIN_D1].af);
        IOConfigGPIOAF(d2, IOCFG_SDMMC, sdioPin[SDIO_PIN_D2].af);
        IOConfigGPIOAF(d3, IOCFG_SDMMC, sdioPin[SDIO_PIN_D3].af);
    }

    // HAL_NVIC_SetPriority(sdioHardware->irqn, NVIC_PRIORITY_BASE(NVIC_PRIO_SDIO_DMA), NVIC_PRIORITY_SUB(NVIC_PRIO_SDIO_DMA));
    // HAL_NVIC_EnableIRQ(sdioHardware->irqn);
}

void SDIO_GPIO_Init(void)
{
    if (!sdioHardware) {
        return;
    }

    uint8_t is4BitWidth = sdioConfig()->use4BitWidth;

    const IO_t clk = IOGetByTag(sdioPin[SDIO_PIN_CK].pin);
    const IO_t cmd = IOGetByTag(sdioPin[SDIO_PIN_CMD].pin);
    const IO_t d0 = IOGetByTag(sdioPin[SDIO_PIN_D0].pin);
    const IO_t d1 = IOGetByTag(sdioPin[SDIO_PIN_D1].pin);
    const IO_t d2 = IOGetByTag(sdioPin[SDIO_PIN_D2].pin);
    const IO_t d3 = IOGetByTag(sdioPin[SDIO_PIN_D3].pin);

    IOInit(clk, OWNER_SDIO_CK, 0);
    IOInit(cmd, OWNER_SDIO_CMD, 0);
    IOInit(d0, OWNER_SDIO_D0, 0);

    if (is4BitWidth) {
        IOInit(d1, OWNER_SDIO_D1, 0);
        IOInit(d2, OWNER_SDIO_D2, 0);
        IOInit(d3, OWNER_SDIO_D3, 0);
    }

    //
    // Setting all the SDIO pins to high for a short time results in more robust initialisation.
    //
    IOHi(d0);
    IOConfigGPIO(d0, IOCFG_OUT_PP);

    if(is4BitWidth) {
        IOHi(d1);
        IOHi(d2);
        IOHi(d3);
        IOConfigGPIO(d1, IOCFG_OUT_PP);
        IOConfigGPIO(d2, IOCFG_OUT_PP);
        IOConfigGPIO(d3, IOCFG_OUT_PP);
    }

    IOHi(clk);
    IOHi(cmd);
    IOConfigGPIO(clk, IOCFG_OUT_PP);
    IOConfigGPIO(cmd, IOCFG_OUT_PP);
}

bool SD_Initialize_LL(DMA_Stream_TypeDef *dma)
{
    UNUSED(dma);

    return true;
}

bool SD_GetState(void)
{
    Status_card cardState = SD_PollingCardStatusBusy(&card,1000);

    // return (cardState == Status_CardStatusBusy);
    return (cardState == Status_CardStatusIdle);
}

#define SD_XIN_CLK  100000000U  //SDMMC clock source

/**
 *\*\name   SDMMC_Config.
 *\*\fun    Config SDMMC device.
 *\*\param  none
 *\*\return none
 */
static void SDMMC_Config(void)
{
    SDMMC_WrapperType SDMMC_WrapperParamstruct;
    SDMMC_StructWrapperInit(&SDMMC_WrapperParamstruct);
    SDMMC_WrapperParamstruct.MaxBlockLen     = SDMMC_MAXBLOCKLEN_512B;
    SDMMC_WrapperParamstruct.SDBaseCLKFreq   = 25;
    SDMMC_WrapperParamstruct.TimeOutUnit     = SDMMC_TIMEOUTCLKUNIT_KHZ;
    SDMMC_WrapperParamstruct.TuningCNT       = 0x20;
    SDMMC_WrapperParamstruct.WKUPSignalMode  = SDMMC_ASYNCWKUP;
    SDMMC_WrapperParamstruct.SPIBlockMode    = SDMMC_SPIBLOCKMODEUNSUPPORT;
    SDMMC_WrapperParamstruct.SPIMode         = SDMMC_SPIMODEUNSUPPORT;
    SDMMC_WrapperParamstruct.DDR50           = SDMMC_DDR50SUPPORT;
    SDMMC_WrapperParamstruct.SDR104          = SDMMC_SDR104SUPPORT;
    SDMMC_WrapperParamstruct.SDR50           = SDMMC_SDR50SUPPORT;
    SDMMC_WrapperParamstruct.SlotType        = SDMMC_SDTYPE;
    SDMMC_WrapperParamstruct.AsyncInt        = SDMMC_ASYNCINTSUPPORT;
    SDMMC_WrapperParamstruct.Suspend_Resume  = SDMMC_SUSRESUNSUPPORT;
    SDMMC_WrapperParamstruct.SDMA            = SDMMC_SDMASUPPORT;
    SDMMC_WrapperParamstruct.HS              = SDMMC_HSSUPPORT;
    SDMMC_WrapperParamstruct.ADMA2           = SDMMC_ADMA2SUPPORT;
    SDMMC_WrapperParamstruct.Embedded_8bit   = SDMMC_EMBEDDEDUNSUPPORT;
    SDMMC_WrapperParamstruct.UseTuningSDR50  = SDMMC_SDR50TUNING;
    SDMMC_WrapperParamstruct.DSSDCLKFreq     = 0x04;
    SDMMC_WrapperParamstruct.INITSDCLKFreq   = 0x00;
    SDMMC_WrapperParamstruct.SDR12SDCLKFreq  = 0x04;
    SDMMC_WrapperParamstruct.HSSDCLKFreq     = 0x02;
    SDMMC_WrapperParamstruct.SDR50SDCLKFreq  = 0x01;
    SDMMC_WrapperParamstruct.SDR25SDCLKFreq  = 0x02;
    SDMMC_WrapperParamstruct.SDR104SDCLKFreq = 0x00;
    SDMMC_WrapperParamstruct.DDR50SDCLKFreq  = 0x02;
    if(sdioHardware->instance == SDMMC1)
    {
        SDMMC_WrapperConfig(SDMMC1,&SDMMC_WrapperParamstruct);
    }
    else
    {
        SDMMC_WrapperConfig(SDMMC2,&SDMMC_WrapperParamstruct);
    }
}


/**
 *\*\name   SD_DefaultSpeedModeInit.
 *\*\fun    Configure to communicate in Default Speed mode.
 *\*\param  none
 *\*\return none
 */
static Status_card SD_PowerOnInit(sd_card_t* card)
{
    Status_card status_temp = Status_Success;
    uint32_t timeout_value;
    // uint32_t timeout_temp;
    uint32_t persacle_value;
    
    /* SD card clock initialization, configure xin_clk to 100M move to systemInit */
    // RCC_ConfigHse(RCC_HSE_ENABLE);
    // RCC_WaitHseStable();
    // RCC_ConfigPll2(RCC_PLL_SRC_HSI,64000000,500000000,ENABLE);
    // /* configure PLL1A is PLL1 */
    // RCC_ConfigPLL2ADivider(RCC_PLLA_DIV5);
    
    if(sdioHardware->instance == SDMMC1)
    { 
        /* Config  Clock Source for SDMMC , SDCLK is 500/5 = 100MHz */
        RCC_ConfigSDMMC1KerClk(RCC_SDMMC1KERCLK_SRC_PLL2A,RCC_SDMMC1KERCLK_AXIDIV1);
        /* Enable DEVICE clock */
        RCC_EnableAXIPeriphClk1(RCC_AXI_PERIPHEN_M7_SDMMC1,ENABLE);
        /* Reset DEVICE register */
        RCC_EnableAXIPeriphReset1(RCC_AXI_PERIPHRST_SDMMC1 | RCC_AXI_PERIPHRST_SDHOST1);

        AFIO_SDMMCClkSel(SDMMC1_CLKFB,ENABLE);
    } 
    else if(sdioHardware->instance == SDMMC2)
    {
        /* Config  Clock Source for SDMMC , SDCLK is 500/5 = 100MHz */
        RCC_ConfigSDMMC2KerClk(RCC_SDMMC2KERCLK_SRC_PLL2A,RCC_SDMMC2KERCLK_SYSBUSDIV1);
        /* Enable DEVICE clock */
        RCC_EnableAHB1PeriphClk1(RCC_AHB1_PERIPHEN_M7_SDMMC2,ENABLE);
        /* Reset DEVICE register */
        RCC_EnableAHB1PeriphReset1(RCC_AHB1_PERIPHRST_SDMMC2 | RCC_AHB1_PERIPHRST_SDHOST2);

        AFIO_SDMMCClkSel(SDMMC2_CLKFB,ENABLE);
    }

    /* SDMMC IO config */
    SDMMC_port_config();
    
    /* SDMMC Wrapper register config */
    SDMMC_Config();
    
    /* enable all status signal */
    SDMMC_EnableFlagStatus(card->SDHOSTx,SDHOST_AllInterruptFlags,ENABLE);
    
    /* Change the detection of SD card CD pin to TEST mode, default to high */
    SDMMC_ConfigCardDetectSignal(card->SDHOSTx,SDMMC_CARDDETECT_TEST,SDMMC_CARDTESTLEVEL_HIGH);
    
    /* Wait for the card to be inserted for 1 second */
    timeout_value = 0;
    // SysTick_start_time();
    // timeout_temp = systick_timeoutms;
    while(SDMMC_GetPresentFlagStatus(card->SDHOSTx,SDHOST_CardInsertedFlag) != SET)
    {
        /* timeout cnt */
        SDMMC_Delay(1);
        timeout_value++;
        if(timeout_value >= 1000)
        {
            status_temp = Status_Fail;
            break;
        }
    }
    // SysTick_Stop_time();
    
    if(status_temp != Status_Success)
    {
        return status_temp;
    }
    
    /* Enable SD card power and clock */
    SDMMC_EnablePower(card->SDHOSTx,ENABLE);
    if(((SD_XIN_CLK % 400000U) != 0)  || ((SD_XIN_CLK/400000U)%2 != 0))
    {
        if(SD_XIN_CLK <= card->card_workmode.busClock_Hz)
        {
            persacle_value = 0;
        }
        else
        {
            persacle_value = SD_XIN_CLK/400000U/2 + 1;
        }
    }
    else
    {
        persacle_value = SD_XIN_CLK/400000U/2;
    }
    
    SDMMC_SetSdClock(card->SDHOSTx,DISABLE,persacle_value);  //100MHz/250 = 400KHz
     
    /** A series of commands begins to begin the card identification process. **/
    
    /* CMD0 */
    if(SD_NormalCMD_Send(card,SDMMC_GoIdleState,0x00,CARD_ResponseTypeNone) != Status_Success)
    {
        return Status_Fail;
    }
    
    /* CMD8 */
    //arg[19:16] = 0001b 2.7V~3.6V,arg[15:8] = 0xAA check pattern
    if(SD_NormalCMD_Send(card,SD_SendInterfaceCondition,0x1AAU,CARD_ResponseTypeR7) != Status_Success)
    {
        return Status_Fail;
    }
    else
    {
        if((card->command.response[0] & 0xFFU) != 0xAAU)
        {
            return Status_CardNotSupportYet;
        }
        else
        {
            card->sd_card_information.flags |= SD_SupportSdhcFlag;
        }
    }
    
    /* ACMD41 */
    card->sd_card_information.busy = 0x00U;
    timeout_value = 0;
    // SysTick_start_time();
    // timeout_temp = systick_timeoutms;
    /* Waiting for sdcard to be ready,Wait for the card to be ready, timeout exit after 1 second. */
    while((card->sd_card_information.busy != 0x80000000U) && (timeout_value < 1000))
    {
        /* ACMD41 */
        /* arg[31] busy bit 
        arg[30] HCS bit(Host Capacity support): 1b SDHC or SDXC supported , 0b SDSC only Host 
        arg[28] XPC bit(SDXC Power Contorl): 1b Maximum Performance , 0b Power Saving 
        arg[24] S18R bit(Switching to 1.8V request): 1b switch to 1.8V signal voltage , 0b use current signal voltage 
        arg[23:15]: VDD Voltage Window, bit20: 3.2~3.3V
        */
        if(card->card_workmode.operationVoltageflag == SD_OperationVoltage180V)
        {
            if(SD_AutoCMD_Send(card,SD_ApplicationSendOperationCondition,0x41100000,CARD_ResponseTypeR3) != Status_Success)
            {

            }
        }
        else
        {
            if(SD_AutoCMD_Send(card,SD_ApplicationSendOperationCondition,0x40100000,CARD_ResponseTypeR3) != Status_Success)
            {

            }
        }
        
        card->sd_card_information.busy = (card->command.response[0] & 0x80000000U);
        
        /* timeout cnt */
        SDMMC_Delay(1);
        timeout_value++;
    }
    
    if(timeout_value >= 1000)
    {
        return Status_Fail;
    }
    
    // SysTick_Stop_time();
    
    if((card->command.response[0] & 0x40000000U) == 0x40000000U)
    {
        card->sd_card_information.flags |= SD_SupportHighCapacityFlag;
    }
    if((card->command.response[0] & 0x01000000U) == 0x01000000U)
    {
        card->sd_card_information.flags |= SD_SupportVoltage180v;
    }
    card->sd_card_information.ocr = (card->command.response[0] & 0x00FFFF00U) >> 8U;
    
    
    /* cmd11 */
    if(card->card_workmode.operationVoltageflag == SD_OperationVoltage180V)
    {
        if(SD_SwitchVoltage(card) != Status_Success)
        {
            return Status_CardSwitchFailed;
        }
    }
    
    
    /* CMD2 */
    if(SD_NormalCMD_Send(card,SDMMC_AllSendCid,0x00,CARD_ResponseTypeR2) != Status_Success)
    {
        return Status_Fail;
    }
    SD_DecodeCid(card);

    SD_Handle.CID[0] = card->command.response[3U];
    SD_Handle.CID[1] = card->command.response[2U];
    SD_Handle.CID[2] = card->command.response[1U];
    SD_Handle.CID[3] = card->command.response[0U];
    
    
    /* CMD3 */
    if(SD_NormalCMD_Send(card,SD_SendRelativeAddress,0x00,CARD_ResponseTypeR6) != Status_Success)
    {
        return Status_Fail;
    }
    card->sd_card_information.rca = ((card->command.response[0U] & 0xFFFF0000U) >> 16U);
    
    
    /* CMD9 */
    if(SD_NormalCMD_Send(card,SDMMC_SendCsd,card->sd_card_information.rca << 16,CARD_ResponseTypeR2) != Status_Success)
    {
        return Status_Fail;
    }
    SD_DecodeCsd(card);

    SD_Handle.CSD[0] = card->command.response[3U];
    SD_Handle.CSD[1] = card->command.response[2U];
    SD_Handle.CSD[2] = card->command.response[1U];
    SD_Handle.CSD[3] = card->command.response[0U];
    
    /* CMD7 */
    if(SD_NormalCMD_Send(card,SDMMC_SelectCard,card->sd_card_information.rca << 16,CARD_ResponseTypeR1b) != Status_Success)
    {
        return Status_Fail;
    }
    
    /* ACMD51 */
    if(SD_SendSCR(card) != Status_Success)
    {
        return Status_Fail;
    }
    
    /* polling card status idle */
    if (Status_CardStatusIdle != SD_PollingCardStatusBusy(card, SD_CARD_ACCESS_WAIT_IDLE_TIMEOUT))
    {
        return Status_Fail;
    }
    
    /* CMD16 */
    if(SD_NormalCMD_Send(card,SDMMC_SetBlockLength,FSL_SDMMC_DEFAULT_BLOCK_SIZE,CARD_ResponseTypeR1) != Status_Success)
    {
        return Status_Fail;
    }
    
    /* Set to 4-bit data bus mode. */
    if(card->card_workmode.busWidth == SDMMC_BusWdith4Bit)
    {
        if((card->sd_card_information.flags & SD_Support4BitWidthFlag) != SD_Support4BitWidthFlag)
        {
            return Status_CardNotSupportYet;
        }
        /* ACMD6 */
        /* 00b:1bit  10b:4bit */
        else if(SD_AutoCMD_Send(card,SD_ApplicationSetBusWdith,0x00000002,CARD_ResponseTypeR1) != Status_Success)
        {
            return Status_Fail;
        }
        else
        {
            SDMMC_ConfigBusWidth(card->SDHOSTx,SDHOST_DataBusWidth4Bit);
        }
    }

    /* CMD6 */
    if(SD_SelectBusTiming(card) != Status_Success)
    {
        return Status_Fail;
    }
    else
    {
        SDMMC_ConfigWorkMode(card->SDHOSTx,card->card_workmode.mode);
    }
    
    /* Set to sdclk. */
    if(((SD_XIN_CLK % card->card_workmode.busClock_Hz) != 0)  || ((SD_XIN_CLK/card->card_workmode.busClock_Hz)%2 != 0))
    {
        if(SD_XIN_CLK <= card->card_workmode.busClock_Hz)
        {
            persacle_value = 0;
        }
        else
        {
            persacle_value = SD_XIN_CLK/card->card_workmode.busClock_Hz/2 + 1;
        }
    }
    else
    {
        persacle_value = SD_XIN_CLK/card->card_workmode.busClock_Hz/2;
    }
    SDMMC_SetSdClock(card->SDHOSTx,DISABLE,persacle_value);
     
    /* This function is not necessary. Depending on the hardware and card, 
       you can decide whether to enable TX CLK delay and how much delay to use */
    // SDMMC_EnableManualTuningOut(SDMMC1,8,ENABLE);
    
    return status_temp;
}

static SD_Error_t SD_DoInit(void)
{
    Status_card status;

    memset(&card, 0, sizeof(card));

    /* SDMMC Module Power Enable */
    RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_PWR,ENABLE);
    if(sdioHardware->instance == SDMMC1)
    {
        PWR_MoudlePowerEnable(HSC1_SDMMC1_PWRCTRL,ENABLE);
        card.SDHOSTx = SDHOST1;
        card.SDMMCx = SDMMC1;
    }
    else if(sdioHardware->instance == SDMMC2)
    {
        PWR_MoudlePowerEnable(HSC2_SDMMC2_PWRCTRL,ENABLE);
        card.SDHOSTx = SDHOST2;
        card.SDMMCx = SDMMC2;
    }
    else
    {
        return SD_ERROR;
    }

    if (sdioConfig()->use4BitWidth) {
        card.card_workmode.busWidth = SDMMC_BusWdith4Bit;
    } else {
        card.card_workmode.busWidth = SDMMC_BusWdith1Bit; // FIXME untested
    }

    card.card_workmode.mode = SDMMC_DS; 
    card.card_workmode.busClock_Hz = 25000000;
    card.card_workmode.dma = SDMMC_SDMA;
    card.card_workmode.operationVoltageflag = SD_OperationVoltage330V;
    SDMMC_TModeStructInit(&card.TMODE_truct);

    status = SD_PowerOnInit(&card); // Will call HAL_SD_MspInit

    if (status != Status_Success) {
        return SD_ERROR;
    }

    // Fix: Set SD_CardType based on flags detected during ACMD41 response
    // Check if card supports high capacity (SDHC/SDXC)
    if (card.sd_card_information.flags & (SD_SupportSdhcFlag | SD_SupportSdxcFlag)) {
        SD_CardType = SD_HIGH_CAPACITY;
    } else {
        // Standard Capacity card (SDSC)
        // Determine version from CSD structure or other means
        // For now, assume V2.0 for cards that respond to CMD8
        if (card.sd_card_information.flags & SD_SupportSdhcFlag) {
            // Card responded to CMD8, likely V2.0+
            SD_CardType = SD_STD_CAPACITY_V2_0;
        } else {
            // Older card, likely V1.1
            SD_CardType = SD_STD_CAPACITY_V1_1;
        }
    }


    return SD_OK;
}

SD_Error_t SD_GetCardInfo(void)
{
    SD_Error_t ErrorState = SD_OK;

    // fill in SD_CardInfo

    uint32_t Temp = 0;

    // Byte 0
    Temp = (SD_Handle.CSD[0] & 0xFF000000) >> 24;
    SD_CardInfo.SD_csd.CSDStruct      = (uint8_t)((Temp & 0xC0) >> 6);
    SD_CardInfo.SD_csd.SysSpecVersion = (uint8_t)((Temp & 0x3C) >> 2);
    SD_CardInfo.SD_csd.Reserved1      = Temp & 0x03;

    // Fix: Update card.sd_card_information.csd.csdStructure from parsed CSD
    card.sd_card_information.csd.csdStructure = SD_CardInfo.SD_csd.CSDStruct;

    // Byte 1
    Temp = (SD_Handle.CSD[0] & 0x00FF0000) >> 16;
    SD_CardInfo.SD_csd.TAAC = (uint8_t)Temp;

    // Byte 2
    Temp = (SD_Handle.CSD[0] & 0x0000FF00) >> 8;
    SD_CardInfo.SD_csd.NSAC = (uint8_t)Temp;

    // Byte 3
    Temp = SD_Handle.CSD[0] & 0x000000FF;
    SD_CardInfo.SD_csd.MaxBusClkFrec = (uint8_t)Temp;

    // Byte 4
    Temp = (SD_Handle.CSD[1] & 0xFF000000) >> 24;
    SD_CardInfo.SD_csd.CardComdClasses = (uint16_t)(Temp << 4);

    // Byte 5
    Temp = (SD_Handle.CSD[1] & 0x00FF0000) >> 16;
    SD_CardInfo.SD_csd.CardComdClasses |= (uint16_t)((Temp & 0xF0) >> 4);
    SD_CardInfo.SD_csd.RdBlockLen       = (uint8_t)(Temp & 0x0F);

    // Byte 6
    Temp = (SD_Handle.CSD[1] & 0x0000FF00) >> 8;
    SD_CardInfo.SD_csd.PartBlockRead   = (uint8_t)((Temp & 0x80) >> 7);
    SD_CardInfo.SD_csd.WrBlockMisalign = (uint8_t)((Temp & 0x40) >> 6);
    SD_CardInfo.SD_csd.RdBlockMisalign = (uint8_t)((Temp & 0x20) >> 5);
    SD_CardInfo.SD_csd.DSRImpl         = (uint8_t)((Temp & 0x10) >> 4);
    SD_CardInfo.SD_csd.Reserved2       = 0; /*!< Reserved */

    if(card.sd_card_information.csd.csdStructure == 0U) {
        SD_CardInfo.SD_csd.DeviceSize = (Temp & 0x03) << 10;

        // Byte 7
        Temp = (uint8_t)(SD_Handle.CSD[1] & 0x000000FF);
        SD_CardInfo.SD_csd.DeviceSize |= (Temp) << 2;

        // Byte 8
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0xFF000000) >> 24);
        SD_CardInfo.SD_csd.DeviceSize |= (Temp & 0xC0) >> 6;

        SD_CardInfo.SD_csd.MaxRdCurrentVDDMin = (Temp & 0x38) >> 3;
        SD_CardInfo.SD_csd.MaxRdCurrentVDDMax = (Temp & 0x07);

        // Byte 9
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0x00FF0000) >> 16);
        SD_CardInfo.SD_csd.MaxWrCurrentVDDMin = (Temp & 0xE0) >> 5;
        SD_CardInfo.SD_csd.MaxWrCurrentVDDMax = (Temp & 0x1C) >> 2;
        SD_CardInfo.SD_csd.DeviceSizeMul      = (Temp & 0x03) << 1;

        // Byte 10
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0x0000FF00) >> 8);
        SD_CardInfo.SD_csd.DeviceSizeMul |= (Temp & 0x80) >> 7;

        SD_CardInfo.CardCapacity  = (SD_CardInfo.SD_csd.DeviceSize + 1) ;
        SD_CardInfo.CardCapacity *= (1 << (SD_CardInfo.SD_csd.DeviceSizeMul + 2));
        SD_CardInfo.CardBlockSize = 1 << (SD_CardInfo.SD_csd.RdBlockLen);
        SD_CardInfo.CardCapacity = SD_CardInfo.CardCapacity * SD_CardInfo.CardBlockSize / 512; // In 512 byte blocks
    } else if(card.sd_card_information.csd.csdStructure == 1U){
        // Byte 7
        Temp = (uint8_t)(SD_Handle.CSD[1] & 0x000000FF);
        SD_CardInfo.SD_csd.DeviceSize = (Temp & 0x3F) << 16;

        // Byte 8
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0xFF000000) >> 24);

        SD_CardInfo.SD_csd.DeviceSize |= (Temp << 8);

        // Byte 9
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0x00FF0000) >> 16);

        SD_CardInfo.SD_csd.DeviceSize |= (Temp);

        // Byte 10 - CSD[2] bits [47:40] (not used in DeviceSize for SDHC/SDXC)
        // Temp = (uint8_t)((SD_Handle.CSD[2] & 0x0000FF00) >> 8);
        // Note: For SDHC/SDXC (CSD v2.0), DeviceSize is 22 bits [69:48]
        // Bytes 7-9 provide all 22 bits, byte 10 is not part of DeviceSize

        // Calculate card capacity in 512-byte blocks
        // Formula: CardCapacity = (DeviceSize + 1) * 1024 (in 512-byte blocks)
        SD_CardInfo.CardCapacity  = ((uint64_t)SD_CardInfo.SD_csd.DeviceSize + 1) * 1024;
        SD_CardInfo.CardBlockSize = 512;
    } else {
        // Not supported card type
        ErrorState = SD_ERROR;
    }

    SD_CardInfo.SD_csd.EraseGrSize = (Temp & 0x40) >> 6;
    SD_CardInfo.SD_csd.EraseGrMul  = (Temp & 0x3F) << 1;

    // Byte 11
    Temp = (uint8_t)(SD_Handle.CSD[2] & 0x000000FF);
    SD_CardInfo.SD_csd.EraseGrMul     |= (Temp & 0x80) >> 7;
    SD_CardInfo.SD_csd.WrProtectGrSize = (Temp & 0x7F);

    // Byte 12
    Temp = (uint8_t)((SD_Handle.CSD[3] & 0xFF000000) >> 24);
    SD_CardInfo.SD_csd.WrProtectGrEnable = (Temp & 0x80) >> 7;
    SD_CardInfo.SD_csd.ManDeflECC        = (Temp & 0x60) >> 5;
    SD_CardInfo.SD_csd.WrSpeedFact       = (Temp & 0x1C) >> 2;
    SD_CardInfo.SD_csd.MaxWrBlockLen     = (Temp & 0x03) << 2;

    // Byte 13
    Temp = (uint8_t)((SD_Handle.CSD[3] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_csd.MaxWrBlockLen      |= (Temp & 0xC0) >> 6;
    SD_CardInfo.SD_csd.WriteBlockPaPartial = (Temp & 0x20) >> 5;
    SD_CardInfo.SD_csd.Reserved3           = 0;
    SD_CardInfo.SD_csd.ContentProtectAppli = (Temp & 0x01);

    // Byte 14
    Temp = (uint8_t)((SD_Handle.CSD[3] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_csd.FileFormatGrouop = (Temp & 0x80) >> 7;
    SD_CardInfo.SD_csd.CopyFlag         = (Temp & 0x40) >> 6;
    SD_CardInfo.SD_csd.PermWrProtect    = (Temp & 0x20) >> 5;
    SD_CardInfo.SD_csd.TempWrProtect    = (Temp & 0x10) >> 4;
    SD_CardInfo.SD_csd.FileFormat       = (Temp & 0x0C) >> 2;
    SD_CardInfo.SD_csd.ECC              = (Temp & 0x03);

    // Byte 15
    Temp = (uint8_t)(SD_Handle.CSD[3] & 0x000000FF);
    SD_CardInfo.SD_csd.CSD_CRC   = (Temp & 0xFE) >> 1;
    SD_CardInfo.SD_csd.Reserved4 = 1;

    // Byte 0
    Temp = (uint8_t)((SD_Handle.CID[0] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ManufacturerID = Temp;

    // Byte 1
    Temp = (uint8_t)((SD_Handle.CID[0] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.OEM_AppliID = Temp << 8;

    // Byte 2
    Temp = (uint8_t)((SD_Handle.CID[0] & 0x000000FF00) >> 8);
    SD_CardInfo.SD_cid.OEM_AppliID |= Temp;

    // Byte 3
    Temp = (uint8_t)(SD_Handle.CID[0] & 0x000000FF);
    SD_CardInfo.SD_cid.ProdName1 = Temp << 24;

    // Byte 4
    Temp = (uint8_t)((SD_Handle.CID[1] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ProdName1 |= Temp << 16;

    // Byte 5
    Temp = (uint8_t)((SD_Handle.CID[1] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.ProdName1 |= Temp << 8;

    // Byte 6
    Temp = (uint8_t)((SD_Handle.CID[1] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_cid.ProdName1 |= Temp;

    // Byte 7
    Temp = (uint8_t)(SD_Handle.CID[1] & 0x000000FF);
    SD_CardInfo.SD_cid.ProdName2 = Temp;

    // Byte 8
    Temp = (uint8_t)((SD_Handle.CID[2] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ProdRev = Temp;

    // Byte 9
    Temp = (uint8_t)((SD_Handle.CID[2] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.ProdSN = Temp << 24;

    // Byte 10
    Temp = (uint8_t)((SD_Handle.CID[2] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_cid.ProdSN |= Temp << 16;

    // Byte 11
    Temp = (uint8_t)(SD_Handle.CID[2] & 0x000000FF);
    SD_CardInfo.SD_cid.ProdSN |= Temp << 8;

    // Byte 12
    Temp = (uint8_t)((SD_Handle.CID[3] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ProdSN |= Temp;

    // Byte 13
    Temp = (uint8_t)((SD_Handle.CID[3] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.Reserved1   |= (Temp & 0xF0) >> 4;
    SD_CardInfo.SD_cid.ManufactDate = (Temp & 0x0F) << 8;

    // Byte 14
    Temp = (uint8_t)((SD_Handle.CID[3] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_cid.ManufactDate |= Temp;

    // Byte 15
    Temp = (uint8_t)(SD_Handle.CID[3] & 0x000000FF);
    SD_CardInfo.SD_cid.CID_CRC   = (Temp & 0xFE) >> 1;
    SD_CardInfo.SD_cid.Reserved2 = 1;

    return ErrorState;
}

SD_Error_t SD_Init(void)
{
    static bool sdInitAttempted = false;
    static SD_Error_t result = SD_ERROR;

    if (sdInitAttempted) {
        return result;
    }

    sdInitAttempted = true;

    result = SD_DoInit();

    return result;
}

SD_Error_t SD_CheckWrite(void)
{
    if (SD_Handle.TXCplt != 0) return SD_BUSY;
    return SD_OK;
}

SD_Error_t SD_CheckRead(void)
{
    if (SD_Handle.RXCplt != 0) return SD_BUSY;
    return SD_OK;
}

SD_Error_t SD_Erase(uint64_t StartAddress, uint64_t EndAddress)
{
    SD_Error_t ErrorState = SD_OK;
    SD_Handle.TXCplt = 1;

    Status_card status;
    
    uint32_t startBlock = (uint32_t)StartAddress;
    uint32_t blockCount = (uint32_t)(EndAddress - StartAddress + 1);

    if ((status = SD_Erase_Block(&card, startBlock, blockCount)) != Status_Success) {
        ErrorState = SD_ERROR;
    }

    SD_Handle.TXCplt = 0;

    return ErrorState;
}

SD_Error_t SD_WriteBlocks_DMA(uint64_t WriteAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error_t ErrorState = SD_OK;
    SD_Handle.TXCplt = 1;

    if (BlockSize != 512) {
        return SD_ERROR; // unsupported.
    }

    // if ((uint32_t)buffer & 0x1f) {
    //     return SD_ADDR_MISALIGNED;
    // }

    // Ensure the data is flushed to main memory
    //SCB_CleanDCache_by_Addr(buffer, NumberOfBlocks * BlockSize);

    Status_card status;

    card.card_workmode.dma = SDMMC_SDMA;

    if ((status = SD_WriteBlocks(&card, buffer, (uint32_t)WriteAddress, NumberOfBlocks)) != Status_Success) {
        ErrorState = SD_ERROR;
    }

    SD_Handle.TXCplt = 0;

    return ErrorState;
}

typedef struct {
    uint32_t *buffer;
    uint32_t BlockSize;
    uint32_t NumberOfBlocks;
} sdReadParameters_t;

sdReadParameters_t sdReadParameters;

SD_Error_t SD_ReadBlocks_DMA(uint64_t ReadAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error_t ErrorState = SD_OK;

    if (BlockSize != 512) {
        return SD_ERROR; // unsupported.
    }

    // if ((uint32_t)buffer & 0x1f) {
    //     return SD_ADDR_MISALIGNED;
    // }

    SD_Handle.RXCplt = 1;

    sdReadParameters.buffer = buffer;
    sdReadParameters.BlockSize = BlockSize;
    sdReadParameters.NumberOfBlocks = NumberOfBlocks;

    Status_card status;

    card.card_workmode.dma = SDMMC_SDMA;
    if ((status = SD_ReadBlocks(&card, buffer, (uint32_t)ReadAddress, NumberOfBlocks)) != Status_Success) {
        ErrorState = SD_ERROR;
    }

    SD_Handle.RXCplt = 0;

    //uint32_t alignedAddr = (uint32_t)sdReadParameters.buffer &  ~0x1F;
    //SCB_InvalidateDCache_by_Addr((uint32_t*)alignedAddr, sdReadParameters.NumberOfBlocks * sdReadParameters.BlockSize + ((uint32_t)sdReadParameters.buffer - alignedAddr));

    return ErrorState;
}



void SDMMC1_IRQHandler(void)
{
    // HAL_SD_IRQHandler(&card);
}

void SDMMC2_IRQHandler(void)
{
    // HAL_SD_IRQHandler(&card);
}

bool SD_InitialiseHardware(dmaResource_t *dma)
{
    UNUSED(dma);

    return true;
}

bool mscSdioInitDma(void)
{
    return true;
}

#endif
