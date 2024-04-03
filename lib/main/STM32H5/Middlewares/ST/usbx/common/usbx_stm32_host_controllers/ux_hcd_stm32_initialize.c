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
/*    _ux_hcd_stm32_initialize                            PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function initializes the STM32 HS USB host controller. This    */
/*    is not for the OTG mode. It forces the chip in Host mode only.      */
/*    For OTG support, the filex in the usbx_otg subdirectory must be     */
/*    used.                                                               */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    HCD                                   Pointer to HCD                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_memory_allocate             Allocate memory block       */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Host Stack                                                          */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_stm32_initialize(UX_HCD *hcd)
{

UX_HCD_STM32          *hcd_stm32;


    /* The controller initialized here is of STM32 type.  */
    hcd -> ux_hcd_controller_type =  UX_HCD_STM32_CONTROLLER;

    /* Initialize the max bandwidth for periodic endpoints. On STM32, the spec says
       no more than 90% to be allocated for periodic.  */
#if UX_MAX_DEVICES > 1
    hcd -> ux_hcd_available_bandwidth =  UX_HCD_STM32_AVAILABLE_BANDWIDTH;
#endif

    /* Allocate memory for this STM32 HCD instance.  */
    hcd_stm32 =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HCD_STM32));
    if (hcd_stm32 == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Set the pointer to the STM32 HCD.  */
    hcd -> ux_hcd_controller_hardware =  (VOID *) hcd_stm32;

    /* Set the generic HCD owner for the STM32 HCD.  */
    hcd_stm32 -> ux_hcd_stm32_hcd_owner =  hcd;

    /* Initialize the function collector for this HCD.  */
    hcd -> ux_hcd_entry_function =  _ux_hcd_stm32_entry;

    /* Set the state of the controller to HALTED first.  */
    hcd -> ux_hcd_status =  UX_HCD_STATUS_HALTED;

    /* Initialize the number of channels.  */
    hcd_stm32 -> ux_hcd_stm32_nb_channels =  UX_HCD_STM32_MAX_NB_CHANNELS;

    /* Check if the parameter is null.  */
    if (hcd -> ux_hcd_irq == 0)
    {
        _ux_utility_memory_free(hcd_stm32);
        return(UX_ERROR);
    }

    /* Get HCD handle from parameter.  */
    hcd_stm32 -> hcd_handle = (HCD_HandleTypeDef*)hcd -> ux_hcd_irq;
    hcd_stm32 -> hcd_handle -> pData = hcd;

    /* Allocate the list of eds.   */
    hcd_stm32 -> ux_hcd_stm32_ed_list =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HCD_STM32_ED) *_ux_system_host -> ux_system_host_max_ed);
    if (hcd_stm32 -> ux_hcd_stm32_ed_list == UX_NULL)
    {
        _ux_utility_memory_free(hcd_stm32);
        return(UX_MEMORY_INSUFFICIENT);
    }

    /* Since we know this is a high-speed controller, we can hardwire the version.  */
#if UX_MAX_DEVICES > 1
    hcd -> ux_hcd_version =  0x200;
#endif

    /* The number of ports on the controller is fixed to 1. The number of ports needs to be reflected both
       for the generic HCD container and the local stm32 container.  */
    hcd -> ux_hcd_nb_root_hubs             =  UX_HCD_STM32_NB_ROOT_PORTS;

    /* The root port must now be powered to pick up device insertion.  */
    _ux_hcd_stm32_power_on_port(hcd_stm32, 0);

    /* The asynchronous queues are empty for now.  */
    hcd_stm32 -> ux_hcd_stm32_queue_empty =  UX_TRUE;

    /* The periodic scheduler is not active.  */
    hcd_stm32 -> ux_hcd_stm32_periodic_scheduler_active =  0;

    /* Set the host controller into the operational state.  */
    hcd -> ux_hcd_status =  UX_HCD_STATUS_OPERATIONAL;

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

