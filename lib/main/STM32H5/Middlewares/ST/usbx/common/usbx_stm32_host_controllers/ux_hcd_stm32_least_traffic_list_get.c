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
/*    _ux_hcd_stm32_least_traffic_list_get                PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*     This function return a pointer to the first ED in the periodic     */
/*     tree that has the least traffic registered.                        */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_stm32                               Pointer to STM32 controller */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    UX_HCD_STM32_ED *                       Pointer to STM32 ED         */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    None                                                                */
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
UINT  _ux_hcd_stm32_least_traffic_list_get(UX_HCD_STM32 *hcd_stm32)
{

UX_HCD_STM32_ED     *ed;
UINT                list_index;
ULONG               min_bandwidth_used;
ULONG               bandwidth_used;
UINT                min_bandwidth_slot;


    /* Set the min bandwidth used to a arbitrary maximum value.  */
    min_bandwidth_used =  0xffffffff;

    /* The first ED is the list candidate for now.  */
    min_bandwidth_slot =  0;

    /* All list will be scanned.  */
    for (list_index = 0; list_index < 32; list_index++)
    {

        /* Reset the bandwidth for this list.  */
        bandwidth_used =  0;

        /* Get the ED of the beginning of the list we parse now.  */
        ed =  hcd_stm32 -> ux_hcd_stm32_periodic_ed_head;

        /* Parse the eds in the list.  */
        while (ed != UX_NULL)
        {

            if ((list_index & ed -> ux_stm32_ed_interval_mask) == ed -> ux_stm32_ed_interval_position)
            {

                /* Add to the bandwidth used the max packet size pointed by this ED.  */
                bandwidth_used +=  (ULONG) ed -> ux_stm32_ed_endpoint -> ux_endpoint_descriptor.wMaxPacketSize;
            }

            /* Move to next ED.  */
            ed =  ed -> ux_stm32_ed_next_ed;
        }

        /* We have processed a list, check the bandwidth used by this list.
           If this bandwidth is the minimum, we memorize the ED.  */
        if (bandwidth_used < min_bandwidth_used)
        {

            /* We have found a better list with a lower used bandwidth, memorize the bandwidth
               for this list.  */
            min_bandwidth_used =  bandwidth_used;

            /* Memorize the begin ED for this list.  */
            min_bandwidth_slot =  list_index;
        }
    }

    /* Return the ED list with the lowest bandwidth.  */
    return(min_bandwidth_slot);
}

