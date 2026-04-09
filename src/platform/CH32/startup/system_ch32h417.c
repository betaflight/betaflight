/********************************** (C) COPYRIGHT *******************************
* File Name          : system_ch32h417.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : CH32H417 Device Peripheral Access Layer System Source File.
*                      For HSE = 25Mhz
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417.h" 

uint32_t HCLKClock;
uint32_t SystemClock;       
uint32_t SystemCoreClock;

static __I uint8_t PLLMULTB[32] = {4,6,7,8,17,9,19,10,21,11,23,12,25,13,14,15,16,17,18,19,20,22,24,26,28,30,32,34,36,38,40,59};
static __I uint8_t HBPrescTB[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
static __I uint8_t FPRETB[4] = {0, 1, 2, 2};

/*********************************************************************
 * @fn      SystemAndCoreClockUpdate
 *
 * @brief   Update SystemClock and CoreClock variable according to Clock Register Values.
 *
 * @return  none
 */
void SystemAndCoreClockUpdate (void)
{
    uint32_t tmp = 0,tmp1 = 0, tmp2 = 0, tmp3 = 0, pllmull = 0, pllsource = 0, presc = 0, presc1 = 0;

    tmp = RCC->CFGR0 & RCC_SWS;
    tmp2 = RCC->PLLCFGR & RCC_SYSPLL_SEL;

    switch(tmp)
    {
        case 0x00:
            SystemClock = HSI_VALUE;
            break;

        case 0x04:
            SystemClock = HSE_VALUE;
            break;

        case 0x08:
            switch(tmp2)
            { 
                case RCC_SYSPLL_PLL:
                    pllmull = RCC->PLLCFGR & RCC_PLLMUL;
                    pllsource = RCC->PLLCFGR & RCC_PLLSRC;
                    presc = (((RCC->PLLCFGR & RCC_PLL_SRC_DIV) >> 8) + 1);

                    if((pllsource == 0xE0) || (pllsource == 0xA0))
                    {
                        tmp1 = 500000000 / presc;
                    }
                    else if(pllsource == 0x80)
                    {
                        tmp1 = 480000000 / presc;
                    }
                    else if(pllsource == 0xC0)
                    {
                        tmp1 = 125000000 / presc;
                    }
                    else if(pllsource == 0x20)
                    {
                        tmp1 = HSE_VALUE / presc;
                    }
                    else
                    {
                        tmp1 = HSI_VALUE / presc;
                    }

                    if((pllmull == 4) || (pllmull == 6) || (pllmull == 8) || (pllmull == 10) || (pllmull == 12))
                    {
                        SystemClock = (tmp1 * PLLMULTB[pllmull]) >> 1;
                    }
                    else
                    {
                        SystemClock = tmp1 * PLLMULTB[pllmull];
                    }
                    break;

                case RCC_SYSPLL_USBHS:
                    SystemClock = 480000000;
                    break;

                case RCC_SYSPLL_ETH:
                    SystemClock = 500000000;
                    break;

                case RCC_SYSPLL_SERDES:
                    SystemClock = 500000000;
                    break;

                case RCC_SYSPLL_USBSS:
                    SystemClock = 125000000;
                    break;

                default:
                    SystemClock = HSI_VALUE;
                    break;
            }  
            break;

        default:
            SystemClock = HSI_VALUE;
            break;
    }

    tmp = (RCC->CFGR0 & RCC_HPRE) >> 4;
    presc1 = HBPrescTB[tmp];

    tmp3 = SystemClock >> presc1;

    tmp = (RCC->CFGR0 & RCC_FPRE) >> 16;
    presc1 = FPRETB[tmp];
    HCLKClock = tmp3 >> presc1;

    if(NVIC_GetCurrentCoreID() == 0)//V3F
    {
        SystemCoreClock = HCLKClock;
    }
    else 
    {
         SystemCoreClock = tmp3;
    }
}

