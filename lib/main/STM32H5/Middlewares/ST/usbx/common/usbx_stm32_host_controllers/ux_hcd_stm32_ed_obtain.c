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
/*    _ux_hcd_stm32_ed_obtain                             PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function obtains a free ED from the ED list.                   */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_stm32                           Pointer to STM32 controller     */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    UX_HCD_STM32_ED *                   Pointer to ED                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_memory_set                Set memory block              */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 Controller Driver                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
UX_HCD_STM32_ED  *_ux_hcd_stm32_ed_obtain(UX_HCD_STM32 *hcd_stm32)
{

UX_HCD_STM32_ED       *ed;
ULONG                 ed_index;


    /* Start the search from the beginning of the list.  */
    ed =  hcd_stm32 -> ux_hcd_stm32_ed_list;
    for (ed_index = 0; ed_index < _ux_system_host -> ux_system_host_max_ed; ed_index++)
    {

        /* Check the ED status, a free ED is marked with the UNUSED flag.  */
        if (ed -> ux_stm32_ed_status == UX_HCD_STM32_ED_STATUS_FREE)
        {

            /* The ED may have been used, so we reset all fields.  */
            _ux_utility_memory_set(ed, 0, sizeof(UX_HCD_STM32_ED));

            /* This ED is now marked as ALLOCATED.  */
            ed -> ux_stm32_ed_status =  UX_HCD_STM32_ED_STATUS_ALLOCATED;

            /* Reset the channel.  */
            ed -> ux_stm32_ed_channel =  UX_HCD_STM32_NO_CHANNEL_ASSIGNED;

            /* Return ED pointer.  */
            return(ed);
        }

        /* Point to the next ED.  */
        ed++;
    }

    /* There is no available ED in the ED list.  */
    return(UX_NULL);
}

