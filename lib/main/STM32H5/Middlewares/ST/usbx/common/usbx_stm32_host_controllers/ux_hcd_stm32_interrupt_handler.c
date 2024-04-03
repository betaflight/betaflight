/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** USBX Component                                                        */
/**                                                                       */
/**   STM32 Controller Driver                                             */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE
#define UX_HCD_STM32_SOURCE_CODE

#include "ux_api.h"
#include "ux_hcd_stm32.h"
#include "ux_host_stack.h"

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_hcd_stm32_interrupt_handler                     PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*     This function is the interrupt handler for the USB interrupts.     */
/*     This function calls HAL driver for interrupt handling.             */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    HAL_HCD_IRQHandler                      Interrupt handler           */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    ThreadX Interrupt Handler                                           */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
VOID  _ux_hcd_stm32_interrupt_handler(VOID)
{

UINT                hcd_index;
UX_HCD              *hcd;
UX_HCD_STM32        *hcd_stm32;


    /* We need to parse the controller driver table to find all controllers that
      are registered as STM32.  */
    for (hcd_index = 0; hcd_index < _ux_system_host -> ux_system_host_registered_hcd; hcd_index++)
    {

        /* Check type of controller.  */
        if (_ux_system_host -> ux_system_host_hcd_array[hcd_index].ux_hcd_controller_type == UX_HCD_STM32_CONTROLLER)
        {

            /* Get the pointers to the generic HCD and STM32 specific areas.  */
            hcd =  &_ux_system_host -> ux_system_host_hcd_array[hcd_index];
            hcd_stm32 =  (UX_HCD_STM32 *) hcd -> ux_hcd_controller_hardware;

            /* Call HAL interrupt handler.  */
            HAL_HCD_IRQHandler(hcd_stm32 -> hcd_handle);
        }
    }
}

