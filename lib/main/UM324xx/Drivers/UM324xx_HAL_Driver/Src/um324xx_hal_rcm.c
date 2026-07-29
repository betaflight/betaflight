#if defined(UM324xF)
/**
  ******************************************************************************
  * @file     um324xx_hal_rcm.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-03-21
  * @brief
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup UM324xF_HAL_Examples
  * @{
  */

/** @defgroup RCM_functions
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void HAL_RCM_SetXthFreq(uint32_t Freq);
/* Private functions ---------------------------------------------------------*/

/** @defgroup RCM_Exported_Functions RCM Exported Functions
  * @{
  */

/**
  * @brief  Initializes the RCM Oscillators according to the specified parameters in the
  *         RCM_OscInitTypeDef.
  * @param  RCM_OscInitStruct pointer to an RCM_OscInitTypeDef structure that
  *         contains the configuration information for the RCM Oscillators.
  * @note   The PLL is not disabled when used as system clock.
  * @note   Transition XTH Bypass to XTH On and XTH On to XTH Bypass are not
  *         supported by this API. User should request a transition to XTH Off
  *         first and then XTH On or XTH Bypass.
  * @retval HAL status
  */
__weak HAL_StatusTypeDef HAL_RCM_OscConfig(RCM_OscInitTypeDef  *RCM_OscInitStruct)
{
    uint32_t tickstart, pll_config;
    /* Check Null pointer */
    if(RCM_OscInitStruct == NULL)
    {
        return HAL_ERROR;
    }

    /*Unlock protection register*/
    __HAL_RCM_UNLOCK_REGISTER();

    /*------------------------------- XTH Configuration ------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_XTH) == RCM_OSCILLATORTYPE_XTH)
    {
        /* When the XTH is used as system clock or clock source for PLL in these cases XTH will not disabled */
        if((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_CFGR0_SYS_SWS_XTH) ||\
                ((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_CFGR0_SYS_SWS_PLL0) && ((RCM->CR0 & RCM_CR0_PLLSRC) == RCM_CR0_PLLSRC_XTH)))
        {
            if((__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) != RESET) && (RCM_OscInitStruct->XTHState == RCM_XTH_OFF))
            {
                /*Lock protection register*/
                __HAL_RCM_LOCK_REGISTER();
                return HAL_ERROR;
            }
        }
        else
        {
            /* Set the new XTH configuration ---------------------------------------*/

            HAL_RCM_SetXthFreq(XTH_VALUE);

            __HAL_RCM_XTH_CONFIG(RCM_OscInitStruct->XTHState);

            /* Check the XTH State */
            if((RCM_OscInitStruct->XTHState) != RCM_XTH_OFF)
            {
                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till XTH is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) == RESET)
                {
                    if((HAL_GetTick() - tickstart ) > XTH_TIMEOUT_VALUE)
                    {
                        /*Lock protection register*/
                        __HAL_RCM_LOCK_REGISTER();
                        return HAL_TIMEOUT;
                    }
                }
            }
            else
            {
                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till XTH is bypassed or disabled */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > XTH_TIMEOUT_VALUE)
                    {
                        /*Lock protection register*/
                        __HAL_RCM_LOCK_REGISTER();
                        return HAL_TIMEOUT;
                    }
                }
            }
        }
    }
    /*----------------------------- RCH Configuration --------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_RCH) == RCM_OSCILLATORTYPE_RCH)
    {
        /* Check if RCH is used as system clock or as PLL source when PLL is selected as system clock */
        if((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_CFGR0_SYS_SWS_RCH) ||\
                ((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_CFGR0_SYS_SWS_PLL0) && ((RCM->CR0 & RCM_CR0_PLLSRC) == RCM_CR0_PLLSRC_RCH)))
        {
            /* When RCH is used as system clock it will not disabled */
            if((__HAL_RCM_GET_FLAG(RCM_FLAG_RCH_STB) != RESET) && (RCM_OscInitStruct->RCHState != RCM_RCH_ON))
            {
                /*Lock protection register*/
                __HAL_RCM_LOCK_REGISTER();
                return HAL_ERROR;
            }
        }
        else
        {
            /* Check the RCH State */
            if((RCM_OscInitStruct->RCHState)!= RCM_RCH_OFF)
            {
                /* Enable the Internal High Speed oscillator (RCH). */
                __HAL_RCM_RCH_ENABLE();

                /* Get Start Tick*/
                tickstart = HAL_GetTick();

                /* Wait till RCH is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_RCH_STB) == RESET)
                {
                    if((HAL_GetTick() - tickstart ) > RCH_TIMEOUT_VALUE)
                    {
                        /*Lock protection register*/
                        __HAL_RCM_LOCK_REGISTER();
                        return HAL_TIMEOUT;
                    }
                }
            }
            else
            {
                /* Disable the Internal High Speed oscillator (RCH). */
                __HAL_RCM_RCH_DISABLE();

                /* Get Start Tick*/
                tickstart = HAL_GetTick();

                /* Wait till RCH is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_RCH_STB) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > RCH_TIMEOUT_VALUE)
                    {
                        /*Lock protection register*/
                        __HAL_RCM_LOCK_REGISTER();
                        return HAL_TIMEOUT;
                    }
                }
            }
        }
    }
    /*------------------------------ RCL Configuration -------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_RCL) == RCM_OSCILLATORTYPE_RCL)
    {
        /* Check the RCL State */
        if((RCM_OscInitStruct->RCLState)!= RCM_RCL_OFF)
        {
            /* Enable the Internal Low Speed oscillator (RCL). */
            __HAL_PMU_RCL_ENABLE();
        }
        else
        {
            /* Disable the Internal Low Speed oscillator (RCL). */
            __HAL_PMU_RCL_DISABLE();
        }
    }
    /*------------------------------ XTL Configuration -------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_XTL) == RCM_OSCILLATORTYPE_XTL)
    {
        /* Check the XTL State */
        if((RCM_OscInitStruct->XTLState) != RCM_XTL_OFF)
        {
            /* Enable the External Low Speed oscillator (XTL). */
            __HAL_PMU_XTL_ENABLE();
        }
        else
        {
            /* Disable the External Low Speed oscillator (XTL). */
            __HAL_PMU_XTL_DISABLE();
        }
    }
    /*-------------------------------PLL Configuration -------------------------*/
    if ((RCM_OscInitStruct->PLL.PLLState) != RCM_PLL_NONE)
    {
        /* Check if the PLL is used as system clock or not */
        if(__HAL_RCM_GET_SYSCLK_SOURCE() != RCM_CFGR0_SYS_SWS_PLL0)
        {
            if((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_ON)
            {
                /* Disable the main PLL. */
                __HAL_RCM_PLL_DISABLE();

                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till PLL is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
                    {
                        /*Lock protection register*/
                        __HAL_RCM_LOCK_REGISTER();
                        return HAL_TIMEOUT;
                    }
                }

                /* Configure the PLL0 stable time */
                MODIFY_REG(RCM->PLLTSR, (RCM_PLLTSR_PLL_LT_EN|RCM_PLLTSR_PLL_LT), PLLLT_TIME_VALUE);

                /* Configure the main PLL clock source */
                MODIFY_REG(RCM->CR0, RCM_CR0_PLLSRC, (RCM_OscInitStruct->PLL.PLLSource ));

                /* Configure the main PLL multiplication and division factors. */
                WRITE_REG(RCM->PLL0CFGR0, (RCM_OscInitStruct->PLL.PLLM << RCM_PLL0CFGR0_PLL0_DM_Pos            | \
                                           (RCM_OscInitStruct->PLL.PLLN << RCM_PLL0CFGR0_PLL0_DN_Pos)          | \
                                           (RCM_OscInitStruct->PLL.PLLP<< RCM_PLL0CFGR0_PLL0_DP_Pos)           | \
                                           (0x1<<0)));
                /* Enable the main PLL. */
                __HAL_RCM_PLL_ENABLE();

                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till PLL is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) == RESET)
                {
                    if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
                    {
                        /*Lock protection register*/
                        __HAL_RCM_LOCK_REGISTER();
                        return HAL_TIMEOUT;
                    }
                }
            }
            else
            {
                /* Disable the main PLL. */
                __HAL_RCM_PLL_DISABLE();

                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till PLL is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
                    {
                        /*Lock protection register*/
                        __HAL_RCM_LOCK_REGISTER();
                        return HAL_TIMEOUT;
                    }
                }
            }
        }
        else
        {
            /* Check if there is a request to disable the PLL used as System clock source */
            if((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_OFF)
            {
                /*Lock protection register*/
                __HAL_RCM_LOCK_REGISTER();
                return HAL_ERROR;
            }
            else
            {
                /* Do not return HAL_ERROR if request repeats the current configuration */
                pll_config = RCM->PLL0CFGR0;

                if (((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_OFF) ||
                        (READ_BIT(pll_config, RCM_CR0_PLLSRC) != RCM_OscInitStruct->PLL.PLLSource) ||
                        (READ_BIT(pll_config, RCM_PLL0CFGR0_PLL0_DM) != (RCM_OscInitStruct->PLL.PLLM) << RCM_PLL0CFGR0_PLL0_DM_Pos) ||
                        (READ_BIT(pll_config, RCM_PLL0CFGR0_PLL0_DN) != (RCM_OscInitStruct->PLL.PLLN) << RCM_PLL0CFGR0_PLL0_DN_Pos) ||
                        (READ_BIT(pll_config, RCM_PLL0CFGR0_PLL0_DP) != (RCM_OscInitStruct->PLL.PLLP) << RCM_PLL0CFGR0_PLL0_DP_Pos))
                {
                    /*Lock protection register*/
                    __HAL_RCM_LOCK_REGISTER();
                    return HAL_ERROR;
                }
            }
        }
    }

    /*Lock protection register*/
    __HAL_RCM_LOCK_REGISTER();

    return HAL_OK;
}

/**
  * @brief  Initializes the CPU, AHB and APB busses clocks according to the specified
  *         parameters in the RCM_ClkInitStruct.
  * @param  RCM_ClkInitStruct pointer to an RCM_OscInitTypeDef structure that
  *         contains the configuration information for the RCM peripheral.
  * @param  Rwaitcyc FLASH Read wait cycles, this parameter depend on systemclock
  *
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  *         and updated by HAL_RCM_GetHCLKFreq() function called within this function
  *
  * @note   The RCH is used (enabled by hardware) as system clock source after
  *         startup from Reset, wake-up from STOP and STANDBY mode, or in case
  *         of failure of the XTH used directly or indirectly as system clock
  *         (if the Clock Security System XTH_MEN is enabled).
  *
  * @note   A switch from one clock source to another occurs only if the target
  *         clock source is ready (clock stable after startup delay or PLL locked).
  *         If a clock source which is not yet ready is selected, the switch will
  *         occur when the clock source will be ready.
  *
  * @retval None
  */
HAL_StatusTypeDef HAL_RCM_ClockConfig(RCM_ClkInitTypeDef  *RCM_ClkInitStruct, uint32_t Rwaitcyc)
{
    uint32_t tickstart;
    uint16_t efc_freq;

    /* Check Null pointer */
    if(RCM_ClkInitStruct == NULL)
    {
        return HAL_ERROR;
    }

    /*Unlock protection register*/
    __HAL_RCM_UNLOCK_REGISTER();

    /* To correctly read data from FLASH memory, the number of read wait cycles (RWAITCYC)
     must be correctly programmed according to the frequency of the CPU clock
     (systemclock) and the supply voltage of the device. */

    /* Increasing the number of wait states because of higher CPU frequency */
    if(Rwaitcyc > __HAL_FLASH_GET_RWAITCYC())
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        __HAL_FLASH_SET_RWAITCYC(Rwaitcyc);

        /* Check that the new number of read wait cycles is taken into account to access the Flash
        memory by reading the EFC_TIME register */
        if(__HAL_FLASH_GET_RWAITCYC() != Rwaitcyc)
        {
            /*Lock protection register*/
            __HAL_RCM_LOCK_REGISTER();
            return HAL_ERROR;
        }
    }

    /*-------------------------- RCH Configuration --------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_RCH) == RCM_CLOCKTYPE_RCH)
    {
        MODIFY_REG(RCM->CFGR0, RCM_CFGR0_RCH_DIV, RCM_ClkInitStruct->RCHDivider);
    }
    /*-------------------------- HCLK Configuration --------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_HCLK) == RCM_CLOCKTYPE_HCLK)
    {
        /* Set the highest APBx dividers in order to ensure that we do not go through
           a non-spec phase whatever we decrease or increase HCLK. */
        if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK0) == RCM_CLOCKTYPE_PCLK0)
        {
            MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB0_DIV, RCM_PCLK0_DIV256);
        }
        if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK1) == RCM_CLOCKTYPE_PCLK1)
        {
            MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB1_DIV, RCM_PCLK1_DIV256);
        }
        if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK2) == RCM_CLOCKTYPE_PCLK2)
        {
            MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB2_DIV, RCM_PCLK2_DIV256);
        }
        if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK3) == RCM_CLOCKTYPE_PCLK3)
        {
            MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB3_DIV, RCM_PCLK3_DIV256);
        }

        MODIFY_REG(RCM->CFGR0, RCM_CFGR0_AHB_DIV, RCM_ClkInitStruct->AHBCLKDivider);
    }

    /*------------------------- SYSCLK Configuration ---------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_SYSCLK) == RCM_CLOCKTYPE_SYSCLK)
    {

        /* XTH is selected as System Clock Source */
        if(RCM_ClkInitStruct->SYSCLKSource == RCM_SYSCLKSOURCE_XTH)
        {
            /* Check the XTH ready flag */
            if(__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) == RESET)
            {
                /*Lock protection register*/
                __HAL_RCM_LOCK_REGISTER();
                return HAL_ERROR;
            }
        }
        /* PLL0 is selected as System Clock Source */
        else if(RCM_ClkInitStruct->SYSCLKSource == RCM_SYSCLKSOURCE_PLL0CLK)
        {
            /* Check the PLL ready flag */
            if(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) == RESET)
            {
                /*Lock protection register*/
                __HAL_RCM_LOCK_REGISTER();
                return HAL_ERROR;
            }
        }
        /* RCH is selected as System Clock Source */
        else
        {
            /* Check the RCH ready flag */
            if(__HAL_RCM_GET_FLAG(RCM_FLAG_RCH_STB) == RESET)
            {
                /*Lock protection register*/
                __HAL_RCM_LOCK_REGISTER();
                return HAL_ERROR;
            }
        }

        __HAL_RCM_SYSCLK_CONFIG(RCM_ClkInitStruct->SYSCLKSource);

        /* Get Start Tick */
        tickstart = HAL_GetTick();


        while (__HAL_RCM_GET_SYSCLK_SOURCE() != (RCM_ClkInitStruct->SYSCLKSource << RCM_CFGR0_SYS_SWS_Pos))
        {
            if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
            {
                /*Lock protection register*/
                __HAL_RCM_LOCK_REGISTER();
                return HAL_TIMEOUT;
            }
        }
    }

    /* Decreasing the number of wait states because of lower CPU frequency */
    if(Rwaitcyc < __HAL_FLASH_GET_RWAITCYC())
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        __HAL_FLASH_SET_RWAITCYC(Rwaitcyc);

        /* Check that the new number of read wait cycles is taken into account to access the Flash
        memory by reading the EFC_TIME register */
        if(__HAL_FLASH_GET_RWAITCYC() != Rwaitcyc)
        {
            /*Lock protection register*/
            __HAL_RCM_LOCK_REGISTER();
            return HAL_ERROR;
        }
    }

    /*-------------------------- PCLK0 Configuration ---------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK0) == RCM_CLOCKTYPE_PCLK0)
    {
        MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB0_DIV, RCM_ClkInitStruct->APB0CLKDivider);
    }

    /*-------------------------- PCLK1 Configuration ---------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK1) == RCM_CLOCKTYPE_PCLK1)
    {
        MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB1_DIV, RCM_ClkInitStruct->APB1CLKDivider);
    }

    /*-------------------------- PCLK2 Configuration ---------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK2) == RCM_CLOCKTYPE_PCLK2)
    {
        MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB2_DIV, RCM_ClkInitStruct->APB2CLKDivider);
    }

    /*-------------------------- PCLK3 Configuration ---------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK3) == RCM_CLOCKTYPE_PCLK3)
    {
        MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB3_DIV, RCM_ClkInitStruct->APB3CLKDivider);
    }

    /*-------------------------- USB/SDIO CLK Configuration ---------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_USBSDIO) == RCM_CLOCKTYPE_USBSDIO)
    {
        MODIFY_REG(RCM->CFGR1, RCM_CFGR1_USB_SDIO_DIV, RCM_ClkInitStruct->USBSDIOCLKDivider);
    }

    /* Update the SystemCoreClock global variable */
    SystemCoreClock = HAL_RCM_GetSysClockFreq() >> AHBPrescTable[(RCM->CFGR0 & RCM_CFGR0_AHB_DIV)>> RCM_CFGR0_AHB_DIV_Pos];

    /* Configure the source of time base considering new system clocks settings */
    HAL_InitTick (uwTickPrio);

    /* Set the time scale for FLASH erase and write */
    efc_freq = SystemCoreClock/1000000;
    __HAL_FLASH_SET_FREQ(efc_freq);

    /* Enable reset filtering to enhance anti-interference capability */
    SET_BIT(RCM->EXRSTFER, RCM_EXRSTFER_EXT_FILTER_EN);

    /*Lock protection register*/
    __HAL_RCM_LOCK_REGISTER();

    return HAL_OK;
}

/**
  * @brief  Returns the SYSCLK frequency
  *
  * @note   The system frequency computed by this function is not the real
  *         frequency in the chip. It is calculated based on the predefined
  *         constant and the selected clock source:
  * @note     If SYSCLK source is RCH, function returns values based on RCH_VALUE(*)
  * @note     If SYSCLK source is XTH, function returns values based on XTH_VALUE(**)
  * @note     If SYSCLK source is PLL0, function returns values based on XTH_VALUE(**)
  *           or RCH_VALUE(*) multiplied/divided by the PLL0 factors.
  * @note     (*) RCH_VALUE is a constant defined in um324xF_hal_conf.h file (default value
  *               48 MHz) but the real value may vary depending on the variations
  *               in voltage and temperature.
  * @note     (**) XTH_VALUE is a constant defined in um324xF_hal_conf.h file (default value
  *                12 MHz), user has to ensure that XTH_VALUE is same as the real
  *                frequency of the crystal used. Otherwise, this function may
  *                have wrong result.
  *
  * @note   The result of this function could be not correct when using fractional
  *         value for XTH crystal.
  *
  * @note   This function can be used by the user application to compute the
  *         baudrate for the communication peripherals or configure other parameters.
  *
  * @note   Each time SYSCLK changes, this function must be called to update the
  *         right SYSCLK value. Otherwise, any configuration based on this function will be incorrect.
  *
  *
  * @retval SYSCLK frequency
  */
__weak uint32_t HAL_RCM_GetSysClockFreq(void)
{
    uint32_t pll0_dm = 0U, pll0_vco = 0U, pll0_dp = 0U, pll0cfgr0 = 0U;
    uint32_t Fclk = 0U, sysclockfreq = 0U;

    /* Get SYSCLK source -------------------------------------------------------*/
    switch (RCM->CFGR0 & RCM_CFGR0_SYS_SWS)
    {
    case RCM_CFGR0_SYS_SWS_RCH:  /* RCH used as system clock source */
    {
        sysclockfreq = RCH_VALUE / ((READ_BIT(RCM->CFGR0, RCM_CFGR0_RCH_DIV) >> RCM_CFGR0_RCH_DIV_Pos) + 1);
        break;
    }
    case RCM_CFGR0_SYS_SWS_XTH:  /* XTH used as system clock  source */
    {
        sysclockfreq = XTH_VALUE;
        break;
    }
    case RCM_CFGR0_SYS_SWS_PLL0:  /* PLL0 used as system clock  source */
    {
        /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_DM) * PLL_DN,    PLL clock output is Fclk, Fclk = PLL_VCO / PLL_DP;
          Fclk=(HSE_VALUE or HSI_VALUE *PLL_DN)/(PLL_DP * PLL_DM)
          SYSCLK = Fclk / ahb_div */

        pll0_dm = (RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DM) >> RCM_PLL0CFGR0_PLL0_DM_Pos;
        if(__HAL_RCM_GET_PLL_OSCSOURCE() != RCM_PLLSOURCE_RCH)
        {
            /* XTH used as PLL clock source */
            pll0_vco = (uint32_t) ((((uint64_t) XTH_VALUE * ((uint64_t) ((RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DN) >> RCM_PLL0CFGR0_PLL0_DN_Pos)))) / (uint64_t)pll0_dm);
        }
        else
        {
            pll0cfgr0 = RCM->PLL0CFGR0;
            /* RCH used as PLL clock source */
            pll0_vco = (uint32_t) ((((uint64_t) (RCH_VALUE / ((READ_BIT(RCM->CFGR0, RCM_CFGR0_RCH_DIV) >> RCM_CFGR0_RCH_DIV_Pos) + 1)) \
                                     * ((uint64_t) ((pll0cfgr0 & RCM_PLL0CFGR0_PLL0_DN) >> RCM_PLL0CFGR0_PLL0_DN_Pos)))) / (uint64_t)pll0_dm);
        }
        pll0_dp = ((RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DP) >> RCM_PLL0CFGR0_PLL0_DP_Pos);

        Fclk = pll0_vco/pll0_dp;

        sysclockfreq = Fclk;
        break;
    }
    default:
    {
        sysclockfreq = RCH_VALUE;
        break;
    }
    }
    return sysclockfreq;
}

/**
 * @brief  XTH oscillation frequency selection.
 * @param  Freq     XTH clock
 * @return frequency band.
 */
static void HAL_RCM_SetXthFreq(uint32_t Freq)
{
    uint8_t temp;

    Freq = Freq/1000000;

    if		((Freq>0) && (Freq<=4))		temp = 0;
    else if	((Freq>4) && (Freq<=12))	temp = 1;
    else if	((Freq>12) && (Freq<=24))	temp = 2;
    else								temp = 3;
    /*Set XTH_SF*/
    RCM->CR0 = (RCM->CR0 &(~(RCM_CR0_XTH_SF))) | (temp << RCM_CR0_XTH_SF_Pos);
}

/**
  * @brief  Returns the HCLK frequency
  * @note   Each time HCLK changes, this function must be called to update the
  *         right HCLK value. Otherwise, any configuration based on this function will be incorrect.
  *
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  *         and updated within this function
  * @retval HCLK frequency
  */
uint32_t HAL_RCM_GetHCLKFreq(void)
{
    return SystemCoreClock;
}

/**
  * @brief  Returns the PCLK0 frequency
  * @note   Each time PCLK0 changes, this function must be called to update the
  *         right PCLK0 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK0 frequency
  */
uint32_t HAL_RCM_GetPCLK0Freq(void)
{
    /* Get HCLK source and Compute PCLK0 frequency ---------------------------*/
    return (HAL_RCM_GetHCLKFreq() >> APBPrescTable[(RCM->CFGR0 & RCM_CFGR0_APB0_DIV)>> RCM_CFGR0_APB0_DIV_Pos]);
}

/**
  * @brief  Returns the PCLK1 frequency
  * @note   Each time PCLK1 changes, this function must be called to update the
  *         right PCLK1 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK1 frequency
  */
uint32_t HAL_RCM_GetPCLK1Freq(void)
{
    /* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
    return (HAL_RCM_GetHCLKFreq() >> APBPrescTable[(RCM->CFGR0 & RCM_CFGR0_APB1_DIV)>> RCM_CFGR0_APB1_DIV_Pos]);
}

/**
  * @brief  Returns the PCLK2 frequency
  * @note   Each time PCLK2 changes, this function must be called to update the
  *         right PCLK2 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK2 frequency
  */
uint32_t HAL_RCM_GetPCLK2Freq(void)
{
    /* Get HCLK source and Compute PCLK2 frequency ---------------------------*/
    return (HAL_RCM_GetHCLKFreq()>> APBPrescTable[(RCM->CFGR0 & RCM_CFGR0_APB2_DIV)>> RCM_CFGR0_APB2_DIV_Pos]);
}

/**
  * @brief  Returns the PCLK3 frequency
  * @note   Each time PCLK3 changes, this function must be called to update the
  *         right PCLK3 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK3 frequency
  */
uint32_t HAL_RCM_GetPCLK3Freq(void)
{
    /* Get HCLK source and Compute PCLK2 frequency ---------------------------*/
    return (HAL_RCM_GetHCLKFreq()>> APBPrescTable[(RCM->CFGR0 & RCM_CFGR0_APB3_DIV)>> RCM_CFGR0_APB3_DIV_Pos]);
}

/**
  * @}
  */

/**
  * @}
  */
#endif

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
#if defined(UM32x42x)
/**
  ******************************************************************************
  * @file     um324xx_hal_rcm.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-04-11
  * @brief
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"


/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @defgroup xxx_functions
  * @{
  */

#ifdef HAL_RCM_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup RCM_Private_Constants
  * @{
  */
/* Private macro -------------------------------------------------------------*/
#define __MCO0_CLK_ENABLE()   __HAL_RCM_GPIOA_CLK_ENABLE()
#define MCO0_GPIO_PORT        GPIOA
#define MCO0_PIN              GPIO_PIN_8

#define __MCO1_CLK_ENABLE()   __HAL_RCM_GPIOC_CLK_ENABLE()
#define MCO1_GPIO_PORT         GPIOC
#define MCO1_PIN               GPIO_PIN_9
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup RCC_Private_Variables RCC Private Variables
  * @{
  */
/**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void HAL_RCM_SetXthFreq(uint32_t Freq);

/** @defgroup RCM_Exported_Functions RCM Exported Functions
  * @{
  */


/**
  * @brief  Initializes the RCM Oscillators according to the specified parameters in the
  *         RCM_OscInitTypeDef.
  * @param  RCM_OscInitStruct pointer to an RCM_OscInitTypeDef structure that
  *         contains the configuration information for the RCM Oscillators.
  * @note   The PLL is not disabled when used as system clock.
  * @note   Transition XTH Bypass to XTH On and XTH On to XTH Bypass are not
  *         supported by this API. User should request a transition to XTH Off
  *         first and then XTH On or XTH Bypass.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCM_OscConfig(RCM_OscInitTypeDef  *RCM_OscInitStruct)
{

    uint32_t tickstart, pll_config;
    /* Check Null pointer */
    if(RCM_OscInitStruct == NULL)
    {
        return HAL_ERROR;
    }

    /*Unlock protection register*/
    __HAL_RCM_UNLOCK_REGISTER();

    /*------------------------------- XTH Configuration ------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_XTH) == RCM_OSCILLATORTYPE_XTH)
    {
        /* When the XTH is used as system clock or clock source for PLL in these cases XTH will not disabled */
        if((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_SYSCLK_STATUS_XTH) ||\
                ((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_SYSCLK_STATUS_PLL0CLK) && ((RCM->CR0 & RCM_CR0_PLLSRC) == RCM_PLLSOURCE_XTH)))
        {
            if((__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) != RESET) && (RCM_OscInitStruct->XTHState == RCM_XTH_OFF))
            {
                return HAL_ERROR;
            }
        }
        else
        {
            /* Set the new XTH configuration ---------------------------------------*/
            HAL_RCM_SetXthFreq(XTH_VALUE);

            /* Check the XTH State */
            if((RCM_OscInitStruct->XTHState) != RCM_XTH_OFF)
            {
                __HAL_RCM_XTH_CONFIG(RCM_OscInitStruct->XTHState);

                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till XTH is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) == RESET)
                {
                    if((HAL_GetTick() - tickstart ) > XTH_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
            else
            {
                CLEAR_BIT(RCM->CR0, RCM_CR0_XTH_EN);
                CLEAR_BIT(RCM->CR0, RCM_CR0_XTH_BYP);
                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till XTH is bypassed or disabled */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > XTH_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
        }
    }
    /*----------------------------- RCH Configuration --------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_RCH) == RCM_OSCILLATORTYPE_RCH)
    {
        /* Check if RCH is used as system clock or as PLL source when PLL is selected as system clock */
        if((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_SYSCLK_STATUS_RCH) ||\
                ((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_SYSCLK_STATUS_PLL0CLK) && ((RCM->CR0 & RCM_CR0_PLLSRC) == RCM_PLLSOURCE_RCH)))
        {
            /* When RCH is used as system clock it will not disabled */
            if((__HAL_RCM_GET_FLAG(RCM_FLAG_RCH_STB) != RESET) && (RCM_OscInitStruct->RCHState != RCM_RCH_ON))
            {
                return HAL_ERROR;
            }
        }
        else
        {
            /* Check the RCH State */
            if((RCM_OscInitStruct->RCHState) == RCM_RCH_OFF)
            {
                __HAL_RCM_RCH_DISABLE();
            }
            else
            {
                __HAL_RCM_RCH_ENABLE();
                /* Get Start Tick*/
                tickstart = HAL_GetTick();
                /* Wait till RCH is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_RCH_STB) == RESET)
                {
                    if((HAL_GetTick() - tickstart ) > RCH_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
        }
    }

    /*------------------------------ RCL Configuration -------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_RCL) == RCM_OSCILLATORTYPE_RCL)
    {
        if(__HAL_RCM_GET_SYSCLK_SOURCE() != RCM_SYSCLK_STATUS_LTC)
        {
            /* Check the RCL State */
            if((RCM_OscInitStruct->RCLState) == RCM_RCL_OFF)
            {
                __HAL_PMU_RCL_DISABLE();
            }
            else
            {
                /* Enable the Internal Low Speed oscillator (RCL). */
                __HAL_PMU_RCL_ENABLE();
                /* Select RCL for low speed clock*/
                __HAL_PMU_LSCLK_SEL(PMU_LSCLK_SEL_RCL);

            }
        }
    }
    /*------------------------------ XTL Configuration -------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_XTL) == RCM_OSCILLATORTYPE_XTL)
    {
        if(__HAL_RCM_GET_SYSCLK_SOURCE() != RCM_SYSCLK_STATUS_LTC)
        {
            /* Check the XTL State */
            if((RCM_OscInitStruct->XTLState) == RCM_XTL_OFF)
            {
                __HAL_PMU_XTL_DISABLE();
            }
            else
            {
                /* Enable the Internal Low Speed oscillator (XTL). */
                __HAL_PMU_XTL_ENABLE();
                __HAL_PMU_LSCLK_SEL(PMU_LSCLK_SEL_XTL);

                /* The system clock is configured as LTC*/
                __HAL_RCM_SYSCLK_SEL(RCM_SYSCLKSOURCE_LTC);
            }
        }
    }

    /*-------------------------------PLL Configuration -------------------------*/
    if ((RCM_OscInitStruct->PLL.PLLState) != RCM_PLL_NONE)
    {
        /* Check if the PLL is used as system clock or not */
        if(__HAL_RCM_GET_SYSCLK_SOURCE() != RCM_SYSCLK_STATUS_PLL0CLK)
        {
            if((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_ON)
            {
                /* Disable the main PLL. */
                __HAL_RCM_PLL0_DISABLE();

                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till PLL is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
                RCM->PLLTSR &= ~0x1ffff;
                RCM->PLLTSR |= (0x10000 | 12000);   //PLL0 stable time

                /* Configure the main PLL clock source */
                MODIFY_REG(RCM->CR0, RCM_CR0_PLLSRC, (RCM_OscInitStruct->PLL.PLLSource));

                /* Configure the main PLL multiplication and division factors. */
                WRITE_REG(RCM->PLL0CFGR0, (RCM_OscInitStruct->PLL.PLLM << RCM_PLL0CFGR0_PLL0_DM_Pos            | \
                                           (RCM_OscInitStruct->PLL.PLLN << RCM_PLL0CFGR0_PLL0_DN_Pos)          | \
                                           (RCM_OscInitStruct->PLL.PLLP<< RCM_PLL0CFGR0_PLL0_DP_Pos)           | \
                                           (0x1<<0)));
                /* Enable the main PLL. */
                __HAL_RCM_PLL0_ENABLE();

                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till PLL is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) == RESET)
                {
                    if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
            else
            {
                /* Disable the main PLL. */
                __HAL_RCM_PLL0_DISABLE();

                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till PLL is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
        }
        else
        {
            /* Check if there is a request to disable the PLL used as System clock source */
            if((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_OFF)
            {
                return HAL_ERROR;
            }
            else
            {
                /* Do not return HAL_ERROR if request repeats the current configuration */
                pll_config = RCM->PLL0CFGR0;

                if (((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_OFF) ||
                        (READ_BIT(pll_config, RCM_CR0_PLLSRC) != RCM_OscInitStruct->PLL.PLLSource) ||
                        (READ_BIT(pll_config, RCM_PLL0CFGR0_PLL0_DM) != (RCM_OscInitStruct->PLL.PLLM) << RCM_PLL0CFGR0_PLL0_DM_Pos) ||
                        (READ_BIT(pll_config, RCM_PLL0CFGR0_PLL0_DN) != (RCM_OscInitStruct->PLL.PLLN) << RCM_PLL0CFGR0_PLL0_DN_Pos) ||
                        (READ_BIT(pll_config, RCM_PLL0CFGR0_PLL0_DP) != (RCM_OscInitStruct->PLL.PLLP) << RCM_PLL0CFGR0_PLL0_DP_Pos))
                {
                    return HAL_ERROR;
                }
            }
        }
    }
    return HAL_OK;
}


/**
  * @brief  Initializes the CPU, AHB and APB busses clocks according to the specified
  *         parameters in the RCC_ClkInitStruct.
  * @param  RCM_ClkInitStruct pointer to an RCM_OscInitTypeDef structure that
  *         contains the configuration information for the RCM peripheral.
  * @param  FLatency FLASH Latency, this parameter depend on device selected
  *
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  *         and updated by HAL_RCM_GetHCLKFreq() function called within this function
  *
  * @note   The RCH is used (enabled by hardware) as system clock source after
  *         startup from Reset, wake-up from STOP and STANDBY mode, or in case
  *         of failure of the HSE used directly or indirectly as system clock
  *         (if the Clock Security System CSS is enabled).
  *
  * @note   A switch from one clock source to another occurs only if the target
  *         clock source is ready (clock stable after startup delay or PLL locked).
  *         If a clock source which is not yet ready is selected, the switch will
  *         occur when the clock source will be ready.
  *
  * @retval None
  */
HAL_StatusTypeDef HAL_RCM_ClockConfig(RCM_ClkInitTypeDef  *RCM_ClkInitStruct, uint32_t Rwaitcyc)
{
    uint32_t tickstart;
    uint16_t efc_freq;

    /* Check Null pointer */
    if(RCM_ClkInitStruct == NULL)
    {
        return HAL_ERROR;
    }
    /* To correctly read data from FLASH memory, the number of read wait cycles (RWAITCYC)
     must be correctly programmed according to the frequency of the CPU clock
     (systemclock) and the supply voltage of the device. */

    /* Increasing the number of wait states because of higher CPU frequency */
    if(Rwaitcyc > __HAL_FLASH_GET_RWAITCYC())
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        __HAL_FLASH_SET_RWAITCYC(Rwaitcyc);

        /* Check that the new number of read wait cycles is taken into account to access the Flash
        memory by reading the EFC_TIME register */
        if(__HAL_FLASH_GET_RWAITCYC() != Rwaitcyc)
        {
            return HAL_ERROR;
        }
    }
    __HAL_RCM_UNLOCK_REGISTER();

    /*-------------------------- HCLK Configuration --------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_HCLK) == RCM_CLOCKTYPE_HCLK)
    {
        /* Set the highest APBx dividers in order to ensure that we do not go through
           a non-spec phase whatever we decrease or increase HCLK. */
        if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK0) == RCM_CLOCKTYPE_PCLK0)
        {
            MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB0_DIV, RCM_PCLK0_DIV256);

        }
        if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK1) == RCM_CLOCKTYPE_PCLK1)
        {
            MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB1_DIV, RCM_PCLK1_DIV256);
        }

        MODIFY_REG(RCM->CFGR0, RCM_CFGR0_AHB_DIV, RCM_ClkInitStruct->AHBCLKDivider);
    }

    /*------------------------- SYSCLK Configuration ---------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_SYSCLK) == RCM_CLOCKTYPE_SYSCLK)
    {

        /* XTH is selected as System Clock Source */
        if(RCM_ClkInitStruct->SYSCLKSource == RCM_SYSCLKSOURCE_XTH)
        {
            /* Check the XTH ready flag */
            if(__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) == RESET)
            {
                return HAL_ERROR;
            }
        }
        /* PLL0 is selected as System Clock Source */
        else if(RCM_ClkInitStruct->SYSCLKSource == RCM_SYSCLKSOURCE_PLL0CLK)
        {
            /* Check the PLL ready flag */
            if(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) == RESET)
            {
                return HAL_ERROR;
            }
            /*Configure the system clock input to PLL to be 24M*/
            SET_BIT(RCM->CFGR0,RCM_CFGR0_RCH_DIV_Msk);
        }
        /* RCH is selected as System Clock Source */
        else
        {
            /* Check the RCH ready flag */
            if(__HAL_RCM_GET_FLAG(RCM_FLAG_RCH_STB) == RESET)
            {
                return HAL_ERROR;
            }
            MODIFY_REG(RCM->CFGR0, RCM_CFGR0_RCH_DIV_Msk, RCM_ClkInitStruct->RCHCLKDivider);
        }

        __HAL_RCM_SYSCLK_CONFIG(RCM_ClkInitStruct->SYSCLKSource);

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        while (__HAL_RCM_GET_SYSCLK_SOURCE() != (RCM_ClkInitStruct->SYSCLKSource << RCM_CFGR0_SYS_SWS_Pos))
        {
            if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
            {
                return HAL_TIMEOUT;
            }
        }
    }


    /*-------------------------- PCLK0 Configuration ---------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK0) == RCM_CLOCKTYPE_PCLK0)
    {
        MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB0_DIV, RCM_ClkInitStruct->APB0CLKDivider);
    }

    /*-------------------------- PCLK1 Configuration ---------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK1) == RCM_CLOCKTYPE_PCLK1)
    {
        MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB1_DIV, RCM_ClkInitStruct->APB1CLKDivider);
    }

    /* Update the SystemCoreClock global variable */
    SystemCoreClock = HAL_RCM_GetSysClockFreq() >> AHBPrescTable[(RCM->CFGR0 & RCM_CFGR0_AHB_DIV)>> RCM_CFGR0_AHB_DIV_Pos];

    /* Configure the source of time base considering new system clocks settings */
    HAL_InitTick (uwTickPrio);

    /* Set the time scale for FLASH erase and write */
    efc_freq = SystemCoreClock/1000000;

    __HAL_FLASH_SET_FREQ(efc_freq);

    /* Enable reset filtering to enhance anti-interference capability */
    SET_BIT(RCM->EXRSTCR, RCM_EXRSTCR_EXT_FILTER_EN);

    /* Decreasing the number of wait states because of lower CPU frequency */
    if(efc_freq<50)
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        __HAL_FLASH_SET_RWAITCYC(FLASH_RWAITCYC_0);
    }
    else if((efc_freq>=50)&&(efc_freq<100))
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        __HAL_FLASH_SET_RWAITCYC(FLASH_RWAITCYC_1);
    }
    else if((efc_freq>=100)&&(efc_freq<150))
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        __HAL_FLASH_SET_RWAITCYC(FLASH_RWAITCYC_2);
    }
    else if((efc_freq>=150)&&(efc_freq<200))
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        __HAL_FLASH_SET_RWAITCYC(FLASH_RWAITCYC_3);
    }
    else if((efc_freq>=200)&&(efc_freq<250))
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        __HAL_FLASH_SET_RWAITCYC(FLASH_RWAITCYC_4);
    }
    else if((efc_freq>=250)&&(efc_freq<300))
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        __HAL_FLASH_SET_RWAITCYC(FLASH_RWAITCYC_5);
    }
    else
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        __HAL_FLASH_SET_RWAITCYC(FLASH_RWAITCYC_6);
    }

    return HAL_OK;
}

/**
 * @brief  XTH oscillation frequency selection.
 * @param  Freq     XTH clock
 * @return frequency band.
 */
static void HAL_RCM_SetXthFreq(uint32_t Freq)
{
    uint8_t temp;

    Freq = Freq/1000000;

    if      ((Freq>1) && (Freq<=4))     temp = 0x0;
    else if ((Freq>4) && (Freq<=12))    temp = 0x1;
    else if ((Freq>12) && (Freq<=24))   temp = 0x2;
    else                                temp = 0x3;
    /*Set XTH_SF*/
    RCM->CR0 = (RCM->CR0 &(~(RCM_CR0_XTH_SF))) | (temp << RCM_CR0_XTH_SF_Pos);
}

/**
  * @brief  Returns the SYSCLK frequency
  *
  * @note   The system frequency computed by this function is not the real
  *         frequency in the chip. It is calculated based on the predefined
  *         constant and the selected clock source:
  * @note     If SYSCLK source is RCH, function returns values based on RCH_VALUE(*)
  * @note     If SYSCLK source is XTH, function returns values based on XTH_VALUE(**)
  * @note     If SYSCLK source is PLL0, function returns values based on XTH_VALUE(**)
  *           or RCH_VALUE(*) multiplied/divided by the PLL0 factors.
  * @note     (*) RCH_VALUE is a constant defined in um32x42x_hal_conf.h file (default value
  *               96 MHz) but the real value may vary depending on the variations
  *               in voltage and temperature.
  * @note     (**) XTH_VALUE is a constant defined in um32x42x_hal_conf.h file (default value
  *                12 MHz), user has to ensure that XTH_VALUE is same as the real
  *                frequency of the crystal used. Otherwise, this function may
  *                have wrong result.
  * @note   The result of this function could be not correct when using fractional
  *         value for XTH crystal.
  * @note   This function can be used by the user application to compute the
  *         baudrate for the communication peripherals or configure other parameters.
  * @note   Each time SYSCLK changes, this function must be called to update the
  *         right SYSCLK value. Otherwise, any configuration based on this function will be incorrect.
  * @retval SYSCLK frequency
  */
__weak uint32_t HAL_RCM_GetSysClockFreq(void)
{
    uint32_t pll0_dm = 0U, pll0_vco = 0U, pll0_dp = 0U;
    uint32_t Fclk = 0U;
    uint32_t sysclockfreq = 0U;
    uint32_t rch_div_val;

    /* Get SYSCLK source -------------------------------------------------------*/
    switch (RCM->CFGR0 & RCM_CFGR0_SYS_SWS)
    {
    case RCM_SYSCLK_STATUS_RCH:  /* RCH used as system clock source */
    {
        sysclockfreq = RCH_VALUE / (((RCM->CFGR0 & RCM_CFGR0_RCH_DIV)>>RCM_CFGR0_RCH_DIV_Pos)+1);
        break;
    }
    case RCM_SYSCLK_STATUS_XTH:  /* XTH used as system clock  source */
    {
        sysclockfreq = XTH_VALUE;
        break;
    }
    case RCM_SYSCLK_STATUS_PLL0CLK:  /* PLL0 used as system clock  source */
    {
        /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_DM) * PLL_DN,    PLL clock output is Fclk, Fclk = PLL_VCO / PLL_DP;
        Fclk=(HSE_VALUE or HSI_VALUE *PLL_DN)/(PLL_DP * PLL_DM)
        SYSCLK = Fclk / ahb_div */

        pll0_dm = (RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DM) >> RCM_PLL0CFGR0_PLL0_DM_Pos;
        if(__HAL_RCM_GET_PLL_OSCSOURCE() != RCM_PLLSOURCE_RCH)
        {
            /* XTH used as PLL clock source */
            pll0_vco = (uint32_t) ((((uint64_t) XTH_VALUE * ((uint64_t) ((RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DN) >> RCM_PLL0CFGR0_PLL0_DN_Pos)))) / (uint64_t)pll0_dm);
        }
        else
        {
            rch_div_val = RCH_VALUE / (((RCM->CFGR0 & RCM_CFGR0_RCH_DIV)>>RCM_CFGR0_RCH_DIV_Pos)+1);
            /* RCH used as PLL clock source */
            pll0_vco = (uint32_t) ((((uint64_t) rch_div_val * ((uint64_t) ((RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DN) >> RCM_PLL0CFGR0_PLL0_DN_Pos)))) / (uint64_t)pll0_dm);
        }
        pll0_dp = ((RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DP) >> RCM_PLL0CFGR0_PLL0_DP_Pos);

        Fclk = pll0_vco/pll0_dp;

        sysclockfreq = Fclk;
        break;
    }
    default:
    {
        sysclockfreq = RCH_VALUE;
        break;
    }
    }
    return sysclockfreq;
}


/**
  * @brief  Returns the HCLK frequency
  * @note   Each time HCLK changes, this function must be called to update the
  *         right HCLK value. Otherwise, any configuration based on this function will be incorrect.
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  *         and updated within this function
  * @retval HCLK frequency
  */
uint32_t HAL_RCM_GetHCLKFreq(void)
{
    /* Get HCLK source and Compute AHB frequency ---------------------------*/
    return SystemCoreClock;
}


/**
  * @brief  Returns the PCLK0 frequency
  * @note   Each time PCLK0 changes, this function must be called to update the
  *         right PCLK0 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK0 frequency
  */
uint32_t HAL_RCM_GetPCLK0Freq(void)
{
    /* Get HCLK source and Compute PCLK0 frequency ---------------------------*/
    return (HAL_RCM_GetHCLKFreq() >> APBPrescTable[(RCM->CFGR0 & RCM_CFGR0_APB0_DIV)>> RCM_CFGR0_APB0_DIV_Pos]);

}

/**
  * @brief  Returns the PCLK1 frequency
  * @note   Each time PCLK1 changes, this function must be called to update the
  *         right PCLK1 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK1 frequency
  */
uint32_t HAL_RCM_GetPCLK1Freq(void)
{
    /* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
    return (HAL_RCM_GetHCLKFreq() >> APBPrescTable[(RCM->CFGR0 & RCM_CFGR0_APB1_DIV)>> RCM_CFGR0_APB1_DIV_Pos]);

}

/**
  * @brief  Selects the clock source to output on MCO0 pin(PA8) or on MCO1 pin(PC9).
  * @note   PA8/PC9 should be configured in alternate function mode.
  * @param  RCC_MCOx specifies the output direction for the clock source.
  *          This parameter can be one of the following values:
  *            @arg RCM_MCO0: Clock source to output on MCO0 pin(PA8).
  *            @arg RCM_MCO1: Clock source to output on MCO1 pin(PC9).
  * @param  RCC_MCOSource specifies the clock source to output.
  *          This parameter can be one of the following values:
  *            @arg RCM_MCO_RCH_DIV_CLK: RCH clock output after frequency division
  *            @arg RCM_MCO_XTH_CLK: XTH clock selected as MCO source
  *            @arg RCM_MCO_LTC_CLK: LTC(RCL or XTL) clock selected as MCO source
  *            @arg RCM_MCO_XTL_CLK: XTL clock selected as MCO source
  *            @arg RCM_MCO_PLL0_CLK: PLL0 clock selected as MCO source
  *            @arg RCM_MCO_RCL_CLK: RCL clock selected as MCO source
  *            @arg RCM_MCO_AHB_CLK: AHB clock selected as MCO source
  * @param  RCC_MCODiv specifies the MCOx prescaler.
  *          This parameter can be one of the following values:
  *            @arg RCM_MCODIV_1: no division applied to MCOx clock
  *            @arg RCC_MCODIV_2: division by 2 applied to MCOx clock
  *            @arg RCM_MCODIV_4: division by 4 applied to MCOx clock
  *            @arg RCM_MCODIV_8: division by 8 applied to MCOx clock
  *            @arg RCM_MCODIV_16: division by 16 applied to MCOx clock
  *            @arg RCM_MCODIV_32: division by 32 applied to MCOx clock
  *            @arg RCM_MCODIV_64: division by 64 applied to MCOx clock
  *            @arg RCM_MCODIV_128: division by 128 applied to MCOx clock
  * @note  MCO1 does not support writing to RCM_ MCO_RCL_CLK,If you need to output RCL, set the system clock to
  *        RCL and select RCM_MCO_LTC_CLK
  * @retval None
  */
HAL_StatusTypeDef HAL_RCC_MCOConfig(uint32_t RCM_MCOx, uint32_t RCM_MCOSource, uint32_t RCM_MCODiv)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if(RCM_MCOx == RCM_MCO0)
    {
        /* MCO0 Clock Enable */
        __MCO0_CLK_ENABLE();
        /* Configure the MCO1 pin in alternate function mode */
        GPIO_InitStruct.Pin = MCO0_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
        HAL_GPIO_Init(MCO0_GPIO_PORT, &GPIO_InitStruct);

        __HAL_RCM_UNLOCK_REGISTER();

        RCM->CFGR1 |= (1<<7);
        MODIFY_REG(RCM->CFGR1, RCM_CFGR1_MCO0_Msk | RCM_CFGR1_MCO0_DIV_Msk, RCM_MCODiv | RCM_MCOSource);

        __HAL_RCM_LOCK_REGISTER();
    }
    else
    {
        /* MCO1 does not support writing to RCM_MCO_RCL_CLK*/
        if(RCM_MCOSource == RCM_MCO_RCL_CLK)
        {
            return HAL_ERROR;
        }
        /* MCO1 Clock Enable */
        __MCO1_CLK_ENABLE();
        /* Configure the MCO1 pin in alternate function mode */
        GPIO_InitStruct.Pin = MCO1_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
        HAL_GPIO_Init(MCO1_GPIO_PORT, &GPIO_InitStruct);

        __HAL_RCM_UNLOCK_REGISTER();
        RCM->CFGR1 |= (1<<11);
        MODIFY_REG(RCM->CFGR1, RCM_CFGR1_MCO1_Msk | RCM_CFGR1_MCO1_DIV_Msk, \
                   (RCM_MCOSource<<RCM_CFGR1_MCO1_Pos) | (RCM_MCODiv<<8));

        __HAL_RCM_LOCK_REGISTER();
    }

    return HAL_OK;

}


/**
  * @}
  */

#endif /* HAL_RCM_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/


#endif
#if defined(UM32x41x)
/**
  ******************************************************************************
  * @file     um32x41x_hal_rcm.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-04-11
  * @brief
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"


/** @addtogroup UM32x41x_HAL_Examples
  * @{
  */

/** @defgroup xxx_functions
  * @{
  */

#ifdef HAL_RCM_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup RCM_Private_Constants
  * @{
  */
/* Private macro -------------------------------------------------------------*/
#define __MCO0_CLK_ENABLE()   __HAL_RCM_GPIOA_CLK_ENABLE()
#define MCO0_GPIO_PORT        GPIOA
#define MCO0_PIN              GPIO_PIN_8

#define __MCO1_CLK_ENABLE()   __HAL_RCM_GPIOC_CLK_ENABLE()
#define MCO1_GPIO_PORT         GPIOC
#define MCO1_PIN               GPIO_PIN_9
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup RCC_Private_Variables RCC Private Variables
  * @{
  */
/**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void HAL_RCM_SetXthFreq(uint32_t Freq);

/** @defgroup RCM_Exported_Functions RCM Exported Functions
  * @{
  */


/**
  * @brief  Initializes the RCM Oscillators according to the specified parameters in the
  *         RCM_OscInitTypeDef.
  * @param  RCM_OscInitStruct pointer to an RCM_OscInitTypeDef structure that
  *         contains the configuration information for the RCM Oscillators.
  * @note   The PLL is not disabled when used as system clock.
  * @note   Transition XTH Bypass to XTH On and XTH On to XTH Bypass are not
  *         supported by this API. User should request a transition to XTH Off
  *         first and then XTH On or XTH Bypass.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCM_OscConfig(RCM_OscInitTypeDef  *RCM_OscInitStruct)
{

    uint32_t tickstart, pll_config;
    /* Check Null pointer */
    if(RCM_OscInitStruct == NULL)
    {
        return HAL_ERROR;
    }

    /*Unlock protection register*/
    __HAL_RCM_UNLOCK_REGISTER();

    /*------------------------------- XTH Configuration ------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_XTH) == RCM_OSCILLATORTYPE_XTH)
    {
        /* When the XTH is used as system clock or clock source for PLL in these cases XTH will not disabled */
        if((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_SYSCLK_STATUS_XTH) ||\
                ((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_SYSCLK_STATUS_PLL0CLK) && ((RCM->CR0 & RCM_CR0_PLLSRC) == RCM_PLLSOURCE_XTH)))
        {
            if((__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) != RESET) && (RCM_OscInitStruct->XTHState == RCM_XTH_OFF))
            {
                return HAL_ERROR;
            }
        }
        else
        {
            /* Set the new XTH configuration ---------------------------------------*/
            HAL_RCM_SetXthFreq(XTH_VALUE);

            /* Check the XTH State */
            if((RCM_OscInitStruct->XTHState) != RCM_XTH_OFF)
            {
                __HAL_RCM_XTH_CONFIG(RCM_OscInitStruct->XTHState);

                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till XTH is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) == RESET)
                {
                    if((HAL_GetTick() - tickstart ) > XTH_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
            else
            {
                CLEAR_BIT(RCM->CR0, RCM_CR0_XTH_EN);
                CLEAR_BIT(RCM->CR0, RCM_CR0_XTH_BYP);
                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till XTH is bypassed or disabled */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > XTH_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
        }
    }
    /*----------------------------- RCH Configuration --------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_RCH) == RCM_OSCILLATORTYPE_RCH)
    {
        /* Check if RCH is used as system clock or as PLL source when PLL is selected as system clock */
        if((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_SYSCLK_STATUS_RCH) ||\
                ((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_SYSCLK_STATUS_PLL0CLK) && ((RCM->CR0 & RCM_CR0_PLLSRC) == RCM_PLLSOURCE_RCH)))
        {
            /* When RCH is used as system clock it will not disabled */
            if((__HAL_RCM_GET_FLAG(RCM_FLAG_RCH_STB) != RESET) && (RCM_OscInitStruct->RCHState != RCM_RCH_ON))
            {
                return HAL_ERROR;
            }
        }
        else
        {
            /* Check the RCH State */
            if((RCM_OscInitStruct->RCHState) == RCM_RCH_OFF)
            {
                __HAL_RCM_RCH_DISABLE();
            }
            else
            {
                __HAL_RCM_RCH_ENABLE();
                /* Get Start Tick*/
                tickstart = HAL_GetTick();
                /* Wait till RCH is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_RCH_STB) == RESET)
                {
                    if((HAL_GetTick() - tickstart ) > RCH_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
        }
    }

    /*------------------------------ RCL Configuration -------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_RCL) == RCM_OSCILLATORTYPE_RCL)
    {
        if(__HAL_RCM_GET_SYSCLK_SOURCE() != RCM_SYSCLK_STATUS_LTC)
        {
            /* Check the RCL State */
            if((RCM_OscInitStruct->RCLState) == RCM_RCL_OFF)
            {
                __HAL_PMU_RCL_DISABLE();
            }
            else
            {
                /* Enable the Internal Low Speed oscillator (RCL). */
                __HAL_PMU_RCL_ENABLE();
                /* Select RCL for low speed clock*/
//                __HAL_PMU_LSCLK_SEL(PMU_LSCLK_SEL_RCL);

            }
        }
    }
    /*------------------------------ XTL Configuration -------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_XTL) == RCM_OSCILLATORTYPE_XTL)
    {
        if(__HAL_RCM_GET_SYSCLK_SOURCE() != RCM_SYSCLK_STATUS_LTC)
        {
            /* Check the XTL State */
            if((RCM_OscInitStruct->XTLState) == RCM_XTL_OFF)
            {
//                __HAL_PMU_XTL_DISABLE();
            }
            else
            {
                /* Enable the Internal Low Speed oscillator (XTL). */
//                __HAL_PMU_XTL_ENABLE();
//                __HAL_PMU_LSCLK_SEL(PMU_LSCLK_SEL_XTL);

                /* The system clock is configured as LTC*/
                __HAL_RCM_SYSCLK_SEL(RCM_SYSCLKSOURCE_LTC);
            }
        }
    }

    /*-------------------------------PLL Configuration -------------------------*/
    if ((RCM_OscInitStruct->PLL.PLLState) != RCM_PLL_NONE)
    {
        /* Check if the PLL is used as system clock or not */
        if(__HAL_RCM_GET_SYSCLK_SOURCE() != RCM_SYSCLK_STATUS_PLL0CLK)
        {
            if((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_ON)
            {
                /* Disable the main PLL. */
                __HAL_RCM_PLL0_DISABLE();

                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till PLL is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
                RCM->PLLTSR &= ~0x1ffff;
                RCM->PLLTSR |= (0x10000 | 12000);   //PLL0 stable time

                /* Configure the main PLL clock source */
                MODIFY_REG(RCM->CR0, RCM_CR0_PLLSRC, (RCM_OscInitStruct->PLL.PLLSource));

                /* Configure the main PLL multiplication and division factors. */
                WRITE_REG(RCM->PLL0CFGR0, (RCM_OscInitStruct->PLL.PLLM << RCM_PLL0CFGR0_PLL0_DM_Pos            | \
                                           (RCM_OscInitStruct->PLL.PLLN << RCM_PLL0CFGR0_PLL0_DN_Pos)          | \
                                           (RCM_OscInitStruct->PLL.PLLP<< RCM_PLL0CFGR0_PLL0_DP_Pos)           | \
                                           (0x1<<0)));
                /* Enable the main PLL. */
                __HAL_RCM_PLL0_ENABLE();

                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till PLL is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) == RESET)
                {
                    if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
            else
            {
                /* Disable the main PLL. */
                __HAL_RCM_PLL0_DISABLE();

                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till PLL is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
        }
        else
        {
            /* Check if there is a request to disable the PLL used as System clock source */
            if((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_OFF)
            {
                return HAL_ERROR;
            }
            else
            {
                /* Do not return HAL_ERROR if request repeats the current configuration */
                pll_config = RCM->PLL0CFGR0;

                if (((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_OFF) ||
                        (READ_BIT(pll_config, RCM_CR0_PLLSRC) != RCM_OscInitStruct->PLL.PLLSource) ||
                        (READ_BIT(pll_config, RCM_PLL0CFGR0_PLL0_DM) != (RCM_OscInitStruct->PLL.PLLM) << RCM_PLL0CFGR0_PLL0_DM_Pos) ||
                        (READ_BIT(pll_config, RCM_PLL0CFGR0_PLL0_DN) != (RCM_OscInitStruct->PLL.PLLN) << RCM_PLL0CFGR0_PLL0_DN_Pos) ||
                        (READ_BIT(pll_config, RCM_PLL0CFGR0_PLL0_DP) != (RCM_OscInitStruct->PLL.PLLP) << RCM_PLL0CFGR0_PLL0_DP_Pos))
                {
                    return HAL_ERROR;
                }
            }
        }
    }
    return HAL_OK;
}


/**
  * @brief  Initializes the CPU, AHB and APB busses clocks according to the specified
  *         parameters in the RCC_ClkInitStruct.
  * @param  RCM_ClkInitStruct pointer to an RCM_OscInitTypeDef structure that
  *         contains the configuration information for the RCM peripheral.
  * @param  FLatency FLASH Latency, this parameter depend on device selected
  *
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  *         and updated by HAL_RCM_GetHCLKFreq() function called within this function
  *
  * @note   The RCH is used (enabled by hardware) as system clock source after
  *         startup from Reset, wake-up from STOP and STANDBY mode, or in case
  *         of failure of the HSE used directly or indirectly as system clock
  *         (if the Clock Security System CSS is enabled).
  *
  * @note   A switch from one clock source to another occurs only if the target
  *         clock source is ready (clock stable after startup delay or PLL locked).
  *         If a clock source which is not yet ready is selected, the switch will
  *         occur when the clock source will be ready.
  *
  * @retval None
  */
HAL_StatusTypeDef HAL_RCM_ClockConfig(RCM_ClkInitTypeDef  *RCM_ClkInitStruct, uint32_t Rwaitcyc)
{
    uint32_t tickstart;
    uint16_t efc_freq;

    /* Check Null pointer */
    if(RCM_ClkInitStruct == NULL)
    {
        return HAL_ERROR;
    }
    /* To correctly read data from FLASH memory, the number of read wait cycles (RWAITCYC)
     must be correctly programmed according to the frequency of the CPU clock
     (systemclock) and the supply voltage of the device. */

    /* Increasing the number of wait states because of higher CPU frequency */
    if(Rwaitcyc > __HAL_FLASH_GET_RWAITCYC())
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        __HAL_FLASH_SET_RWAITCYC(Rwaitcyc);

        /* Check that the new number of read wait cycles is taken into account to access the Flash
        memory by reading the EFC_TIME register */
        if(__HAL_FLASH_GET_RWAITCYC() != Rwaitcyc)
        {
            return HAL_ERROR;
        }
    }
    __HAL_RCM_UNLOCK_REGISTER();

    /*-------------------------- HCLK Configuration --------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_HCLK) == RCM_CLOCKTYPE_HCLK)
    {
        /* Set the highest APBx dividers in order to ensure that we do not go through
           a non-spec phase whatever we decrease or increase HCLK. */
        if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK0) == RCM_CLOCKTYPE_PCLK0)
        {
            MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB0_DIV, RCM_PCLK0_DIV256);

        }
        if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK1) == RCM_CLOCKTYPE_PCLK1)
        {
            MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB1_DIV, RCM_PCLK1_DIV256);
        }

        MODIFY_REG(RCM->CFGR0, RCM_CFGR0_AHB_DIV, RCM_ClkInitStruct->AHBCLKDivider);
    }

    /*------------------------- SYSCLK Configuration ---------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_SYSCLK) == RCM_CLOCKTYPE_SYSCLK)
    {

        /* XTH is selected as System Clock Source */
        if(RCM_ClkInitStruct->SYSCLKSource == RCM_SYSCLKSOURCE_XTH)
        {
            /* Check the XTH ready flag */
            if(__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) == RESET)
            {
                return HAL_ERROR;
            }
        }
        /* PLL0 is selected as System Clock Source */
        else if(RCM_ClkInitStruct->SYSCLKSource == RCM_SYSCLKSOURCE_PLL0CLK)
        {
            /* Check the PLL ready flag */
            if(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) == RESET)
            {
                return HAL_ERROR;
            }
            /*Configure the system clock input to PLL to be 24M*/
            SET_BIT(RCM->CFGR0,RCM_CFGR0_RCH_DIV_Msk);
        }
        /* RCH is selected as System Clock Source */
        else
        {
            /* Check the RCH ready flag */
            if(__HAL_RCM_GET_FLAG(RCM_FLAG_RCH_STB) == RESET)
            {
                return HAL_ERROR;
            }
            MODIFY_REG(RCM->CFGR0, RCM_CFGR0_RCH_DIV_Msk, RCM_ClkInitStruct->RCHCLKDivider);
        }

        __HAL_RCM_SYSCLK_CONFIG(RCM_ClkInitStruct->SYSCLKSource);

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        while (__HAL_RCM_GET_SYSCLK_SOURCE() != (RCM_ClkInitStruct->SYSCLKSource << RCM_CFGR0_SYS_SWS_Pos))
        {
            if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
            {
                return HAL_TIMEOUT;
            }
        }
    }

    /* Decreasing the number of wait states because of lower CPU frequency */
    if(Rwaitcyc < __HAL_FLASH_GET_RWAITCYC())
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        __HAL_FLASH_SET_RWAITCYC(Rwaitcyc);

        /* Check that the new number of read wait cycles is taken into account to access the Flash
        memory by reading the EFC_TIME register */
        if(__HAL_FLASH_GET_RWAITCYC() != Rwaitcyc)
        {
            return HAL_ERROR;
        }
    }

    /*-------------------------- PCLK0 Configuration ---------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK0) == RCM_CLOCKTYPE_PCLK0)
    {
        MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB0_DIV, RCM_ClkInitStruct->APB0CLKDivider);
    }

    /*-------------------------- PCLK1 Configuration ---------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK1) == RCM_CLOCKTYPE_PCLK1)
    {
        MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB1_DIV, RCM_ClkInitStruct->APB1CLKDivider);
    }

    /* Update the SystemCoreClock global variable */
    SystemCoreClock = HAL_RCM_GetSysClockFreq() >> AHBPrescTable[(RCM->CFGR0 & RCM_CFGR0_AHB_DIV)>> RCM_CFGR0_AHB_DIV_Pos];

//    SystemCoreClock = 48000000;

    /* Configure the source of time base considering new system clocks settings */
    HAL_InitTick (uwTickPrio);

    /* Set the time scale for FLASH erase and write */
    efc_freq = SystemCoreClock/1000000;

    __HAL_FLASH_SET_FREQ(efc_freq);

    return HAL_OK;
}



/**
 * @brief  XTH oscillation frequency selection.
 * @param  Freq     XTH clock
 * @return frequency band.
 */
static void HAL_RCM_SetXthFreq(uint32_t Freq)
{
    uint8_t temp;

    Freq = Freq/1000000;

    if      ((Freq>1) && (Freq<=4))     temp = 0x0;
    else if ((Freq>4) && (Freq<=12))    temp = 0x1;
    else if ((Freq>12) && (Freq<=24))   temp = 0x2;
    else                                temp = 0x3;
    /*Set XTH_SF*/
    RCM->CR0 = (RCM->CR0 &(~(RCM_CR0_XTH_SF))) | (temp << RCM_CR0_XTH_SF_Pos);
}



/**
  * @brief  Returns the SYSCLK frequency
  *
  * @note   The system frequency computed by this function is not the real
  *         frequency in the chip. It is calculated based on the predefined
  *         constant and the selected clock source:
  * @note     If SYSCLK source is RCH, function returns values based on RCH_VALUE(*)
  * @note     If SYSCLK source is XTH, function returns values based on XTH_VALUE(**)
  * @note     If SYSCLK source is PLL0, function returns values based on XTH_VALUE(**)
  *           or RCH_VALUE(*) multiplied/divided by the PLL0 factors.
  * @note     (*) RCH_VALUE is a constant defined in um32x41x_hal_conf.h file (default value
  *               96 MHz) but the real value may vary depending on the variations
  *               in voltage and temperature.
  * @note     (**) XTH_VALUE is a constant defined in um32x41x_hal_conf.h file (default value
  *                12 MHz), user has to ensure that XTH_VALUE is same as the real
  *                frequency of the crystal used. Otherwise, this function may
  *                have wrong result.
  * @note   The result of this function could be not correct when using fractional
  *         value for XTH crystal.
  * @note   This function can be used by the user application to compute the
  *         baudrate for the communication peripherals or configure other parameters.
  * @note   Each time SYSCLK changes, this function must be called to update the
  *         right SYSCLK value. Otherwise, any configuration based on this function will be incorrect.
  * @retval SYSCLK frequency
  */
__weak uint32_t HAL_RCM_GetSysClockFreq(void)
{
    uint32_t pll0_dm = 0U, pll0_vco = 0U, pll0_dp = 0U;
    uint32_t Fclk = 0U;
    uint32_t sysclockfreq = 0U;
    uint32_t rch_div_val;

    /* Get SYSCLK source -------------------------------------------------------*/
    switch (RCM->CFGR0 & RCM_CFGR0_SYS_SWS)
    {
    case RCM_SYSCLK_STATUS_RCH:  /* RCH used as system clock source */
    {
        sysclockfreq = RCH_VALUE / (((RCM->CFGR0 & RCM_CFGR0_RCH_DIV)>>RCM_CFGR0_RCH_DIV_Pos)+1);
        break;
    }
    case RCM_SYSCLK_STATUS_XTH:  /* XTH used as system clock  source */
    {
        sysclockfreq = XTH_VALUE;
        break;
    }
    case RCM_SYSCLK_STATUS_PLL0CLK:  /* PLL0 used as system clock  source */
    {
        /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_DM) * PLL_DN,    PLL clock output is Fclk, Fclk = PLL_VCO / PLL_DP;
        Fclk=(HSE_VALUE or HSI_VALUE *PLL_DN)/(PLL_DP * PLL_DM)
        SYSCLK = Fclk / ahb_div */

        pll0_dm = (RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DM) >> RCM_PLL0CFGR0_PLL0_DM_Pos;
        if(__HAL_RCM_GET_PLL_OSCSOURCE() != RCM_PLLSOURCE_RCH)
        {
            /* XTH used as PLL clock source */
            pll0_vco = (uint32_t) ((((uint64_t) XTH_VALUE * ((uint64_t) ((RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DN) >> RCM_PLL0CFGR0_PLL0_DN_Pos)))) / (uint64_t)pll0_dm);
        }
        else
        {
            rch_div_val = RCH_VALUE / (((RCM->CFGR0 & RCM_CFGR0_RCH_DIV)>>RCM_CFGR0_RCH_DIV_Pos)+1);
            /* RCH used as PLL clock source */
            pll0_vco = (uint32_t) ((((uint64_t) rch_div_val * ((uint64_t) ((RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DN) >> RCM_PLL0CFGR0_PLL0_DN_Pos)))) / (uint64_t)pll0_dm);
        }
        pll0_dp = ((RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DP) >> RCM_PLL0CFGR0_PLL0_DP_Pos);

        Fclk = pll0_vco/pll0_dp;

        sysclockfreq = Fclk;
        break;
    }
    default:
    {
        sysclockfreq = RCH_VALUE;
        break;
    }
    }
    return sysclockfreq;
}


/**
  * @brief  Returns the HCLK frequency
  * @note   Each time HCLK changes, this function must be called to update the
  *         right HCLK value. Otherwise, any configuration based on this function will be incorrect.
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  *         and updated within this function
  * @retval HCLK frequency
  */
uint32_t HAL_RCM_GetHCLKFreq(void)
{
    /* Get HCLK source and Compute AHB frequency ---------------------------*/
    return SystemCoreClock;
}


/**
  * @brief  Returns the PCLK0 frequency
  * @note   Each time PCLK0 changes, this function must be called to update the
  *         right PCLK0 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK0 frequency
  */
uint32_t HAL_RCM_GetPCLK0Freq(void)
{
    /* Get HCLK source and Compute PCLK0 frequency ---------------------------*/
    return (HAL_RCM_GetHCLKFreq() >> APBPrescTable[(RCM->CFGR0 & RCM_CFGR0_APB0_DIV)>> RCM_CFGR0_APB0_DIV_Pos]);

}

/**
  * @brief  Returns the PCLK1 frequency
  * @note   Each time PCLK1 changes, this function must be called to update the
  *         right PCLK1 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK1 frequency
  */
uint32_t HAL_RCM_GetPCLK1Freq(void)
{
    /* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
    return (HAL_RCM_GetHCLKFreq() >> APBPrescTable[(RCM->CFGR0 & RCM_CFGR0_APB1_DIV)>> RCM_CFGR0_APB1_DIV_Pos]);

}



/**
  * @brief  Selects the clock source to output on MCO0 pin(PA8) or on MCO1 pin(PC9).
  * @note   PA8/PC9 should be configured in alternate function mode.
  * @param  RCC_MCOx specifies the output direction for the clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCO0: Clock source to output on MCO0 pin(PA8).
  *            @arg RCC_MCO1: Clock source to output on MCO1 pin(PC9).
  * @param  RCC_MCOSource specifies the clock source to output.
  *          This parameter can be one of the following values:
  *            @arg RCM_MCO_RCH_DIV_CLK: RCH clock output after frequency division
  *            @arg RCM_MCO_XTH_CLK: XTH clock selected as MCO source
  *            @arg RCM_MCO_LTC_CLK: LTC(RCL or XTL) clock selected as MCO source
  *            @arg RCM_MCO_XTL_CLK: XTL clock selected as MCO source
  *            @arg RCM_MCO_PLL0_CLK: PLL0 clock selected as MCO source
  *            @arg RCM_MCO_RCL_CLK: RCL clock selected as MCO source
  *            @arg RCM_MCO_AHB_CLK: AHB clock selected as MCO source
  * @param  RCC_MCODiv specifies the MCOx prescaler.
  *          This parameter can be one of the following values:
  *            @arg RCM_MCODIV_1: no division applied to MCOx clock
  *            @arg RCC_MCODIV_2: division by 2 applied to MCOx clock
  *            @arg RCM_MCODIV_4: division by 4 applied to MCOx clock
  *            @arg RCM_MCODIV_8: division by 8 applied to MCOx clock
  *            @arg RCM_MCODIV_16: division by 16 applied to MCOx clock
  *            @arg RCM_MCODIV_32: division by 32 applied to MCOx clock
  *            @arg RCM_MCODIV_64: division by 64 applied to MCOx clock
  *            @arg RCM_MCODIV_128: division by 128 applied to MCOx clock
  * @note  MCO1 does not support writing to RCM_ MCO_RCL_CLK,If you need to output RCL, set the system clock to
  *        RCL and select RCM_MCO_LTC_CLK
  * @retval None
  */
HAL_StatusTypeDef HAL_RCC_MCOConfig(uint32_t RCM_MCOx, uint32_t RCM_MCOSource, uint32_t RCM_MCODiv)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if(RCM_MCOx == RCM_MCO0)
    {
        /* MCO0 Clock Enable */
        __MCO0_CLK_ENABLE();
        /* Configure the MCO1 pin in alternate function mode */
        GPIO_InitStruct.Pin = MCO0_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
        HAL_GPIO_Init(MCO0_GPIO_PORT, &GPIO_InitStruct);

        __HAL_RCM_UNLOCK_REGISTER();

        RCM->CFGR1 |= (1<<7);
        MODIFY_REG(RCM->CFGR1, RCM_CFGR1_MCO0_Msk | RCM_CFGR1_MCO0_DIV_Msk, RCM_MCODiv | RCM_MCOSource);

        __HAL_RCM_LOCK_REGISTER();
    }
    else
    {
        /* MCO1 does not support writing to RCM_MCO_RCL_CLK*/
        if(RCM_MCOSource == RCM_MCO_RCL_CLK)
        {
            return HAL_ERROR;
        }
        /* MCO1 Clock Enable */
        __MCO1_CLK_ENABLE();
        /* Configure the MCO1 pin in alternate function mode */
        GPIO_InitStruct.Pin = MCO1_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
        HAL_GPIO_Init(MCO1_GPIO_PORT, &GPIO_InitStruct);

        __HAL_RCM_UNLOCK_REGISTER();
        RCM->CFGR1 |= (1<<11);
//        MODIFY_REG(RCM->CFGR1, RCM_CFGR1_MCO1_Msk | RCM_CFGR1_MCO1_DIV_Msk, (RCM_MCOSource<<RCM_CFGR1_MCO1_Pos) | (RCM_MCODiv<<8));

        __HAL_RCM_LOCK_REGISTER();
    }

    return HAL_OK;


}


/**
  * @}
  */



#endif /* HAL_RCM_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




#endif

#if defined(UM324xH)
/**
  ******************************************************************************
  * @file     um324xH_hal_rcm.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-04-11
  * @brief
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"


/** @addtogroup UM32x42x_HAL_Driver
  * @{
  */

/** @defgroup xxx_functions
  * @{
  */

#ifdef HAL_RCM_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup RCM_Private_Constants
  * @{
  */
/* Private macro -------------------------------------------------------------*/
#define __MCO0_CLK_ENABLE()   __HAL_RCM_GPIOA_CLK_ENABLE()
#define MCO0_GPIO_PORT        GPIOA
#define MCO0_PIN              GPIO_PIN_8

#define __MCO1_CLK_ENABLE()   __HAL_RCM_GPIOC_CLK_ENABLE()
#define MCO1_GPIO_PORT         GPIOC
#define MCO1_PIN               GPIO_PIN_9
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup RCC_Private_Variables RCC Private Variables
  * @{
  */
/**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void HAL_RCM_SetXthFreq(uint32_t Freq);

/** @defgroup RCM_Exported_Functions RCM Exported Functions
  * @{
  */


/**
  * @brief  Initializes the RCM Oscillators according to the specified parameters in the
  *         RCM_OscInitTypeDef.
  * @param  RCM_OscInitStruct pointer to an RCM_OscInitTypeDef structure that
  *         contains the configuration information for the RCM Oscillators.
  * @note   The PLL is not disabled when used as system clock.
  * @note   Transition XTH Bypass to XTH On and XTH On to XTH Bypass are not
  *         supported by this API. User should request a transition to XTH Off
  *         first and then XTH On or XTH Bypass.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCM_OscConfig(RCM_OscInitTypeDef  *RCM_OscInitStruct)
{

    uint32_t tickstart, pll_config;
    /* Check Null pointer */
    if(RCM_OscInitStruct == NULL)
    {
        return HAL_ERROR;
    }

    /*Unlock protection register*/
    __HAL_RCM_UNLOCK_REGISTER();

    /*------------------------------- XTH Configuration ------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_XTH) == RCM_OSCILLATORTYPE_XTH)
    {
        /* When the XTH is used as system clock or clock source for PLL in these cases XTH will not disabled */
        if((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_SYSCLK_STATUS_XTH) ||\
                ((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_SYSCLK_STATUS_PLL0CLK) && ((RCM->CR0 & RCM_CR0_PLLSRC) == RCM_PLLSOURCE_XTH)))
        {
            if((__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) != RESET) && (RCM_OscInitStruct->XTHState == RCM_XTH_OFF))
            {
                return HAL_ERROR;
            }
        }
        else
        {
            /* Set the new XTH configuration ---------------------------------------*/
            HAL_RCM_SetXthFreq(XTH_VALUE);

            /* Check the XTH State */
            if((RCM_OscInitStruct->XTHState) != RCM_XTH_OFF)
            {
                __HAL_RCM_XTH_CONFIG(RCM_OscInitStruct->XTHState);

                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till XTH is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) == RESET)
                {
                    if((HAL_GetTick() - tickstart ) > XTH_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
            else
            {
                CLEAR_BIT(RCM->CR0, RCM_CR0_XTH_EN);
                CLEAR_BIT(RCM->CR0, RCM_CR0_XTH_BYP);
                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till XTH is bypassed or disabled */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > XTH_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
        }
    }
    /*----------------------------- RCH Configuration --------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_RCH) == RCM_OSCILLATORTYPE_RCH)
    {
        /* Check if RCH is used as system clock or as PLL source when PLL is selected as system clock */
        if((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_SYSCLK_STATUS_RCH) ||\
                ((__HAL_RCM_GET_SYSCLK_SOURCE() == RCM_SYSCLK_STATUS_PLL0CLK) && ((RCM->CR0 & RCM_CR0_PLLSRC) == RCM_PLLSOURCE_RCH)))
        {
            /* When RCH is used as system clock it will not disabled */
            if((__HAL_RCM_GET_FLAG(RCM_FLAG_RCH_STB) != RESET) && (RCM_OscInitStruct->RCHState != RCM_RCH_ON))
            {
                return HAL_ERROR;
            }
        }
        else
        {
            /* Check the RCH State */
            if((RCM_OscInitStruct->RCHState) == RCM_RCH_OFF)
            {
                __HAL_RCM_RCH_DISABLE();
            }
            else
            {
                __HAL_RCM_RCH_ENABLE();
                /* Get Start Tick*/
                tickstart = HAL_GetTick();
                /* Wait till RCH is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_RCH_STB) == RESET)
                {
                    if((HAL_GetTick() - tickstart ) > RCH_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
        }
    }

    /*------------------------------ RCL Configuration -------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_RCL) == RCM_OSCILLATORTYPE_RCL)
    {
        if(__HAL_RCM_GET_SYSCLK_SOURCE() != RCM_SYSCLK_STATUS_LTC)
        {
            /* Check the RCL State */
            if((RCM_OscInitStruct->RCLState) == RCM_RCL_OFF)
            {
                __HAL_PMU_RCL_DISABLE();
            }
            else
            {
                /* Enable the Internal Low Speed oscillator (RCL). */
                __HAL_PMU_RCL_ENABLE();
                /* Select RCL for low speed clock*/
                __HAL_PMU_LSCLK_SEL(PMU_LSCLK_SEL_RCL);

            }
        }
    }
    /*------------------------------ XTL Configuration -------------------------*/
    if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_XTL) == RCM_OSCILLATORTYPE_XTL)
    {
        if(__HAL_RCM_GET_SYSCLK_SOURCE() != RCM_SYSCLK_STATUS_LTC)
        {
            /* Check the XTL State */
            if((RCM_OscInitStruct->XTLState) == RCM_XTL_OFF)
            {
                __HAL_PMU_XTL_DISABLE();
            }
            else
            {
                /* Enable the Internal Low Speed oscillator (XTL). */
                __HAL_PMU_XTL_ENABLE();
                __HAL_PMU_LSCLK_SEL(PMU_LSCLK_SEL_XTL);

                /* The system clock is configured as LTC*/
                __HAL_RCM_SYSCLK_SEL(RCM_SYSCLKSOURCE_LTC);
            }
        }
    }

    /*-------------------------------PLL Configuration -------------------------*/
    if ((RCM_OscInitStruct->PLL.PLLState) != RCM_PLL_NONE)
    {
        /* Check if the PLL is used as system clock or not */
        if(__HAL_RCM_GET_SYSCLK_SOURCE() != RCM_SYSCLK_STATUS_PLL0CLK)
        {
            if((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_ON)
            {
                /* Disable the main PLL. */
                __HAL_RCM_PLL0_DISABLE();

                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till PLL is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
                RCM->PLLTSR &= ~0x1ffff;
                RCM->PLLTSR |= (0x10000 | 12000);   //PLL0 stable time

                /* Configure the main PLL clock source */
                MODIFY_REG(RCM->CR0, RCM_CR0_PLLSRC, (RCM_OscInitStruct->PLL.PLLSource));

                /* Configure the main PLL multiplication and division factors. */
                WRITE_REG(RCM->PLL0CFGR0, (RCM_OscInitStruct->PLL.PLLM << RCM_PLL0CFGR0_PLL0_DM_Pos            | \
                                           (RCM_OscInitStruct->PLL.PLLN << RCM_PLL0CFGR0_PLL0_DN_Pos)          | \
                                           (RCM_OscInitStruct->PLL.PLLP<< RCM_PLL0CFGR0_PLL0_DP_Pos)           | \
                                           (0x1<<0)));
                /* Enable the main PLL. */
                __HAL_RCM_PLL0_ENABLE();

                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till PLL is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) == RESET)
                {
                    if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
            else
            {
                /* Disable the main PLL. */
                __HAL_RCM_PLL0_DISABLE();

                /* Get Start Tick */
                tickstart = HAL_GetTick();

                /* Wait till PLL is ready */
                while(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) != RESET)
                {
                    if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
                    {
                        return HAL_TIMEOUT;
                    }
                }
            }
        }
        else
        {
            /* Check if there is a request to disable the PLL used as System clock source */
            if((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_OFF)
            {
                return HAL_ERROR;
            }
            else
            {
                /* Do not return HAL_ERROR if request repeats the current configuration */
                pll_config = RCM->PLL0CFGR0;

                if (((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_OFF) ||
                        (READ_BIT(pll_config, RCM_CR0_PLLSRC) != RCM_OscInitStruct->PLL.PLLSource) ||
                        (READ_BIT(pll_config, RCM_PLL0CFGR0_PLL0_DM) != (RCM_OscInitStruct->PLL.PLLM) << RCM_PLL0CFGR0_PLL0_DM_Pos) ||
                        (READ_BIT(pll_config, RCM_PLL0CFGR0_PLL0_DN) != (RCM_OscInitStruct->PLL.PLLN) << RCM_PLL0CFGR0_PLL0_DN_Pos) ||
                        (READ_BIT(pll_config, RCM_PLL0CFGR0_PLL0_DP) != (RCM_OscInitStruct->PLL.PLLP) << RCM_PLL0CFGR0_PLL0_DP_Pos))
                {
                    return HAL_ERROR;
                }
            }
        }
    }
    return HAL_OK;
}


/**
  * @brief  Initializes the CPU, AHB and APB busses clocks according to the specified
  *         parameters in the RCC_ClkInitStruct.
  * @param  RCM_ClkInitStruct pointer to an RCM_OscInitTypeDef structure that
  *         contains the configuration information for the RCM peripheral.
  * @param  FLatency FLASH Latency, this parameter depend on device selected
  *
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  *         and updated by HAL_RCM_GetHCLKFreq() function called within this function
  *
  * @note   The RCH is used (enabled by hardware) as system clock source after
  *         startup from Reset, wake-up from STOP and STANDBY mode, or in case
  *         of failure of the HSE used directly or indirectly as system clock
  *         (if the Clock Security System CSS is enabled).
  *
  * @note   A switch from one clock source to another occurs only if the target
  *         clock source is ready (clock stable after startup delay or PLL locked).
  *         If a clock source which is not yet ready is selected, the switch will
  *         occur when the clock source will be ready.
  *
  * @retval None
  */

HAL_StatusTypeDef HAL_RCM_ClockConfig(RCM_ClkInitTypeDef  *RCM_ClkInitStruct, uint32_t Rwaitcyc)
{
    uint32_t tickstart;
    uint16_t efc_freq;

    /* Check Null pointer */
    if(RCM_ClkInitStruct == NULL)
    {
        return HAL_ERROR;
    }
    /* To correctly read data from FLASH memory, the number of read wait cycles (RWAITCYC)
     must be correctly programmed according to the frequency of the CPU clock
     (systemclock) and the supply voltage of the device. */

    /* Increasing the number of wait states because of higher CPU frequency */
    if(Rwaitcyc > __HAL_FLASH_GET_RWAITCYC())
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        __HAL_FLASH_SET_RWAITCYC(Rwaitcyc);

        /* Check that the new number of read wait cycles is taken into account to access the Flash
        memory by reading the EFC_TIME register */
        if(__HAL_FLASH_GET_RWAITCYC() != Rwaitcyc)
        {
            return HAL_ERROR;
        }
    }
    __HAL_RCM_UNLOCK_REGISTER();

    /*-------------------------- HCLK Configuration --------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_HCLK) == RCM_CLOCKTYPE_HCLK)
    {
        /* Set the highest APBx dividers in order to ensure that we do not go through
           a non-spec phase whatever we decrease or increase HCLK. */
        if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK0) == RCM_CLOCKTYPE_PCLK0)
        {
            MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB0_DIV, RCM_PCLK0_DIV256);

        }
        if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK1) == RCM_CLOCKTYPE_PCLK1)
        {
            MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB1_DIV, RCM_PCLK1_DIV256);
        }

        MODIFY_REG(RCM->CFGR0, RCM_CFGR0_AHB_DIV, RCM_ClkInitStruct->AHBCLKDivider);
    }

    /*------------------------- SYSCLK Configuration ---------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_SYSCLK) == RCM_CLOCKTYPE_SYSCLK)
    {

        /* XTH is selected as System Clock Source */
        if(RCM_ClkInitStruct->SYSCLKSource == RCM_SYSCLKSOURCE_XTH)
        {
            /* Check the XTH ready flag */
            if(__HAL_RCM_GET_FLAG(RCM_FLAG_XTH_STB) == RESET)
            {
                return HAL_ERROR;
            }
        }
        /* PLL0 is selected as System Clock Source */
        else if(RCM_ClkInitStruct->SYSCLKSource == RCM_SYSCLKSOURCE_PLL0CLK)
        {
            /* Check the PLL ready flag */
            if(__HAL_RCM_GET_FLAG(RCM_FLAG_PLL0STB) == RESET)
            {
                return HAL_ERROR;
            }
            /*Configure the system clock input to PLL to be 24M*/
            SET_BIT(RCM->CFGR0,RCM_CFGR0_RCH_DIV_Msk);
        }
        /* RCH is selected as System Clock Source */
        else
        {
            /* Check the RCH ready flag */
            if(__HAL_RCM_GET_FLAG(RCM_FLAG_RCH_STB) == RESET)
            {
                return HAL_ERROR;
            }
            MODIFY_REG(RCM->CFGR0, RCM_CFGR0_RCH_DIV_Msk, RCM_ClkInitStruct->RCHCLKDivider);
        }

        __HAL_RCM_SYSCLK_CONFIG(RCM_ClkInitStruct->SYSCLKSource);

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        while (__HAL_RCM_GET_SYSCLK_SOURCE() != (RCM_ClkInitStruct->SYSCLKSource << RCM_CFGR0_SYS_SWS_Pos))
        {
            if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
            {
                return HAL_TIMEOUT;
            }
        }
    }

    /* Decreasing the number of wait states because of lower CPU frequency */
    if(Rwaitcyc < __HAL_FLASH_GET_RWAITCYC())
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        __HAL_FLASH_SET_RWAITCYC(Rwaitcyc);

        /* Check that the new number of read wait cycles is taken into account to access the Flash
        memory by reading the EFC_TIME register */
        if(__HAL_FLASH_GET_RWAITCYC() != Rwaitcyc)
        {
            return HAL_ERROR;
        }
    }

    /*-------------------------- PCLK0 Configuration ---------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK0) == RCM_CLOCKTYPE_PCLK0)
    {
        MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB0_DIV, RCM_ClkInitStruct->APB0CLKDivider);
    }

    /*-------------------------- PCLK1 Configuration ---------------------------*/
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK1) == RCM_CLOCKTYPE_PCLK1)
    {
        MODIFY_REG(RCM->CFGR0, RCM_CFGR0_APB1_DIV, RCM_ClkInitStruct->APB1CLKDivider);
    }

    /* Update the SystemCoreClock global variable */
    SystemCoreClock = HAL_RCM_GetSysClockFreq() >> AHBPrescTable[(RCM->CFGR0 & RCM_CFGR0_AHB_DIV)>> RCM_CFGR0_AHB_DIV_Pos];

    /* Configure the source of time base considering new system clocks settings */
    HAL_InitTick (uwTickPrio);

    /* Set the time scale for FLASH erase and write */
    efc_freq = SystemCoreClock/1000000;

    __HAL_FLASH_SET_FREQ(efc_freq);

    /* Enable reset filtering to enhance anti-interference capability */
    SET_BIT(RCM->EXRSTCR, RCM_EXRSTCR_EXT_FILTER_EN);

    return HAL_OK;
}



/**
 * @brief  XTH oscillation frequency selection.
 * @param  Freq     XTH clock
 * @return frequency band.
 */
static void HAL_RCM_SetXthFreq(uint32_t Freq)
{
    uint8_t temp;

    Freq = Freq/1000000;

    if      ((Freq>1) && (Freq<=4))     temp = 0x0;
    else if ((Freq>4) && (Freq<=12))    temp = 0x1;
    else if ((Freq>12) && (Freq<=24))   temp = 0x2;
    else                                temp = 0x3;
    /*Set XTH_SF*/
    RCM->CR0 = (RCM->CR0 &(~(RCM_CR0_XTH_SF))) | (temp << RCM_CR0_XTH_SF_Pos);
}



/**
  * @brief  Returns the SYSCLK frequency
  *
  * @note   The system frequency computed by this function is not the real
  *         frequency in the chip. It is calculated based on the predefined
  *         constant and the selected clock source:
  * @note     If SYSCLK source is RCH, function returns values based on RCH_VALUE(*)
  * @note     If SYSCLK source is XTH, function returns values based on XTH_VALUE(**)
  * @note     If SYSCLK source is PLL0, function returns values based on XTH_VALUE(**)
  *           or RCH_VALUE(*) multiplied/divided by the PLL0 factors.
  * @note     (*) RCH_VALUE is a constant defined in um32x42x_hal_conf.h file (default value
  *               96 MHz) but the real value may vary depending on the variations
  *               in voltage and temperature.
  * @note     (**) XTH_VALUE is a constant defined in um32x42x_hal_conf.h file (default value
  *                12 MHz), user has to ensure that XTH_VALUE is same as the real
  *                frequency of the crystal used. Otherwise, this function may
  *                have wrong result.
  * @note   The result of this function could be not correct when using fractional
  *         value for XTH crystal.
  * @note   This function can be used by the user application to compute the
  *         baudrate for the communication peripherals or configure other parameters.
  * @note   Each time SYSCLK changes, this function must be called to update the
  *         right SYSCLK value. Otherwise, any configuration based on this function will be incorrect.
  * @retval SYSCLK frequency
  */
__weak uint32_t HAL_RCM_GetSysClockFreq(void)
{
    uint32_t pll0_dm = 0U, pll0_vco = 0U, pll0_dp = 0U;
    uint32_t Fclk = 0U;
    uint32_t sysclockfreq = 0U;
    uint32_t rch_div_val;

    /* Get SYSCLK source -------------------------------------------------------*/
    switch (RCM->CFGR0 & RCM_CFGR0_SYS_SWS)
    {
    case RCM_SYSCLK_STATUS_RCH:  /* RCH used as system clock source */
    {
        sysclockfreq = RCH_VALUE / (((RCM->CFGR0 & RCM_CFGR0_RCH_DIV)>>RCM_CFGR0_RCH_DIV_Pos)+1);
        break;
    }
    case RCM_SYSCLK_STATUS_XTH:  /* XTH used as system clock  source */
    {
        sysclockfreq = XTH_VALUE;
        break;
    }
    case RCM_SYSCLK_STATUS_PLL0CLK:  /* PLL0 used as system clock  source */
    {
        /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_DM) * PLL_DN,    PLL clock output is Fclk, Fclk = PLL_VCO / PLL_DP;
        Fclk=(HSE_VALUE or HSI_VALUE *PLL_DN)/(PLL_DP * PLL_DM)
        SYSCLK = Fclk / ahb_div */

        pll0_dm = (RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DM) >> RCM_PLL0CFGR0_PLL0_DM_Pos;
        if(__HAL_RCM_GET_PLL_OSCSOURCE() != RCM_PLLSOURCE_RCH)
        {
            /* XTH used as PLL clock source */
            pll0_vco = (uint32_t) ((((uint64_t) XTH_VALUE * ((uint64_t) ((RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DN) >> RCM_PLL0CFGR0_PLL0_DN_Pos)))) / (uint64_t)pll0_dm);
        }
        else
        {
            rch_div_val = RCH_VALUE / (((RCM->CFGR0 & RCM_CFGR0_RCH_DIV)>>RCM_CFGR0_RCH_DIV_Pos)+1);
            /* RCH used as PLL clock source */
            pll0_vco = (uint32_t) ((((uint64_t) rch_div_val * ((uint64_t) ((RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DN) >> RCM_PLL0CFGR0_PLL0_DN_Pos)))) / (uint64_t)pll0_dm);
        }
        pll0_dp = ((RCM->PLL0CFGR0 & RCM_PLL0CFGR0_PLL0_DP) >> RCM_PLL0CFGR0_PLL0_DP_Pos);

        Fclk = pll0_vco/pll0_dp;

        sysclockfreq = Fclk;
        break;
    }
    default:
    {
        sysclockfreq = RCH_VALUE;
        break;
    }
    }
    return sysclockfreq;
}


/**
  * @brief  Returns the HCLK frequency
  * @note   Each time HCLK changes, this function must be called to update the
  *         right HCLK value. Otherwise, any configuration based on this function will be incorrect.
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  *         and updated within this function
  * @retval HCLK frequency
  */
uint32_t HAL_RCM_GetHCLKFreq(void)
{
    /* Get HCLK source and Compute AHB frequency ---------------------------*/
    return SystemCoreClock;
}


/**
  * @brief  Returns the PCLK0 frequency
  * @note   Each time PCLK0 changes, this function must be called to update the
  *         right PCLK0 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK0 frequency
  */
uint32_t HAL_RCM_GetPCLK0Freq(void)
{
    /* Get HCLK source and Compute PCLK0 frequency ---------------------------*/
    return (HAL_RCM_GetHCLKFreq() >> APBPrescTable[(RCM->CFGR0 & RCM_CFGR0_APB0_DIV)>> RCM_CFGR0_APB0_DIV_Pos]);

}

/**
  * @brief  Returns the PCLK1 frequency
  * @note   Each time PCLK1 changes, this function must be called to update the
  *         right PCLK1 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK1 frequency
  */
uint32_t HAL_RCM_GetPCLK1Freq(void)
{
    /* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
    return (HAL_RCM_GetHCLKFreq() >> APBPrescTable[(RCM->CFGR0 & RCM_CFGR0_APB1_DIV)>> RCM_CFGR0_APB1_DIV_Pos]);

}



/**
  * @brief  Selects the clock source to output on MCO0 pin(PA8) or on MCO1 pin(PC9).
  * @note   PA8/PC9 should be configured in alternate function mode.
  * @param  RCC_MCOx specifies the output direction for the clock source.
  *          This parameter can be one of the following values:
  *            @arg RCM_MCO0: Clock source to output on MCO0 pin(PA8).
  *            @arg RCM_MCO1: Clock source to output on MCO1 pin(PC9).
  * @param  RCC_MCOSource specifies the clock source to output.
  *          This parameter can be one of the following values:
  *            @arg RCM_MCO_RCH_DIV_CLK: RCH clock output after frequency division
  *            @arg RCM_MCO_XTH_CLK: XTH clock selected as MCO source
  *            @arg RCM_MCO_LTC_CLK: LTC(RCL or XTL) clock selected as MCO source
  *            @arg RCM_MCO_XTL_CLK: XTL clock selected as MCO source
  *            @arg RCM_MCO_PLL0_CLK: PLL0 clock selected as MCO source
  *            @arg RCM_MCO_RCL_CLK: RCL clock selected as MCO source
  *            @arg RCM_MCO_AHB_CLK: AHB clock selected as MCO source
  * @param  RCC_MCODiv specifies the MCOx prescaler.
  *          This parameter can be one of the following values:
  *            @arg RCM_MCODIV_1: no division applied to MCOx clock
  *            @arg RCC_MCODIV_2: division by 2 applied to MCOx clock
  *            @arg RCM_MCODIV_4: division by 4 applied to MCOx clock
  *            @arg RCM_MCODIV_8: division by 8 applied to MCOx clock
  *            @arg RCM_MCODIV_16: division by 16 applied to MCOx clock
  *            @arg RCM_MCODIV_32: division by 32 applied to MCOx clock
  *            @arg RCM_MCODIV_64: division by 64 applied to MCOx clock
  *            @arg RCM_MCODIV_128: division by 128 applied to MCOx clock
  * @note  MCO1 does not support writing to RCM_ MCO_RCL_CLK,If you need to output RCL, set the system clock to
  *        RCL and select RCM_MCO_LTC_CLK
  * @retval None
  */
HAL_StatusTypeDef HAL_RCC_MCOConfig(uint32_t RCM_MCOx, uint32_t RCM_MCOSource, uint32_t RCM_MCODiv)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if(RCM_MCOx == RCM_MCO0)
    {
        /* MCO0 Clock Enable */
        __MCO0_CLK_ENABLE();
        /* Configure the MCO1 pin in alternate function mode */
        GPIO_InitStruct.Pin = MCO0_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
        HAL_GPIO_Init(MCO0_GPIO_PORT, &GPIO_InitStruct);

        __HAL_RCM_UNLOCK_REGISTER();

        RCM->CFGR1 |= (1<<7);
        MODIFY_REG(RCM->CFGR1, RCM_CFGR1_MCO0_Msk | RCM_CFGR1_MCO0_DIV_Msk, RCM_MCODiv | RCM_MCOSource);

        __HAL_RCM_LOCK_REGISTER();
    }
    else
    {
        /* MCO1 does not support writing to RCM_MCO_RCL_CLK*/
        if(RCM_MCOSource == RCM_MCO_RCL_CLK)
        {
            return HAL_ERROR;
        }
        /* MCO1 Clock Enable */
        __MCO1_CLK_ENABLE();
        /* Configure the MCO1 pin in alternate function mode */
        GPIO_InitStruct.Pin = MCO1_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
        HAL_GPIO_Init(MCO1_GPIO_PORT, &GPIO_InitStruct);

        __HAL_RCM_UNLOCK_REGISTER();
        RCM->CFGR1 |= (1<<11);
        MODIFY_REG(RCM->CFGR1, RCM_CFGR1_MCO1_Msk | RCM_CFGR1_MCO1_DIV_Msk, \
                   (RCM_MCOSource<<RCM_CFGR1_MCO1_Pos) | (RCM_MCODiv<<8));

        __HAL_RCM_LOCK_REGISTER();
    }

    return HAL_OK;


}


/**
  * @}
  */



#endif /* HAL_RCM_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/


#endif

