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
/*    _ux_hcd_stm32_endpoint_destroy                      PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will destroy an endpoint.                             */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_stm32                             Pointer to STM32 controller   */
/*    endpoint                              Pointer to endpoint           */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_virtual_address           Get virtual address           */
/*    _ux_utility_delay_ms                  Delay ms                      */
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_stm32_endpoint_destroy(UX_HCD_STM32 *hcd_stm32, UX_ENDPOINT *endpoint)
{

#if defined(UX_HOST_STANDALONE)
UX_INTERRUPT_SAVE_AREA
#endif /* defined(UX_HOST_STANDALONE) */
UX_HCD_STM32_ED       *ed;
UX_HCD_STM32_ED       *next_ed;
UINT                   endpoint_type;

    /* From the endpoint container fetch the STM32 ED descriptor.  */
    ed =  (UX_HCD_STM32_ED *) endpoint -> ux_endpoint_ed;

    /* Check if this physical endpoint has been initialized properly!  */
    if (ed == UX_NULL)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_ENDPOINT_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_ENDPOINT_HANDLE_UNKNOWN, endpoint, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_ENDPOINT_HANDLE_UNKNOWN);

    }

#if defined(UX_HOST_STANDALONE)

    /* There is no background thread, just remove the ED from processing list.  */
    UX_DISABLE
#else

    /* Wait for the controller to finish the current frame processing.  */
    _ux_utility_delay_ms(1);
#endif /* defined(UX_HOST_STANDALONE) */

    /* We need to free the channel.  */
    hcd_stm32 -> ux_hcd_stm32_channels_ed[ed -> ux_stm32_ed_channel] =  UX_NULL;

    /* Get endpoint type.  */
    endpoint_type = (endpoint -> ux_endpoint_descriptor.bmAttributes) & UX_MASK_ENDPOINT_TYPE;

    /* Check for periodic endpoints.  */
    if ((endpoint_type == UX_INTERRUPT_ENDPOINT) || (endpoint_type == UX_ISOCHRONOUS_ENDPOINT))
    {

        /* Remove the ED from periodic ED list.  */
        if (hcd_stm32 -> ux_hcd_stm32_periodic_ed_head == ed)
        {

            /* The head one in the list, just set the pointer to it's next.  */
            hcd_stm32 -> ux_hcd_stm32_periodic_ed_head = ed -> ux_stm32_ed_next_ed;
        }
        else
        {

            /* Get the first ED in the list.  */
            next_ed = hcd_stm32 -> ux_hcd_stm32_periodic_ed_head;

            /* Search for the ED in the list.  */
            while( (next_ed != UX_NULL) && (next_ed -> ux_stm32_ed_next_ed != ed) )
            {

                /* Move to next ED.  */
                next_ed = next_ed -> ux_stm32_ed_next_ed;
            }

            /* Check if we found the ED.  */
            if (next_ed)
            {

                /* Remove the ED from list.  */
                next_ed -> ux_stm32_ed_next_ed = next_ed -> ux_stm32_ed_next_ed -> ux_stm32_ed_next_ed;
            }
        }

        /* Decrease the periodic active count.  */
        hcd_stm32 -> ux_hcd_stm32_periodic_scheduler_active --;
    }

    /* Now we can safely make the ED free.  */
    ed -> ux_stm32_ed_status =  UX_HCD_STM32_ED_STATUS_FREE;

#if defined (USBH_HAL_HUB_SPLIT_SUPPORTED)
    HAL_HCD_HC_ClearHubInfo(hcd_stm32->hcd_handle, ed -> ux_stm32_ed_channel);
#endif /* USBH_HAL_HUB_SPLIT_SUPPORTED */

    /* Finish current transfer.  */
    _ux_hcd_stm32_request_trans_finish(hcd_stm32, ed);

#if defined(UX_HOST_STANDALONE)

    /* If setup memory is not freed correct, free it.  */
    if (ed -> ux_stm32_ed_setup)
        _ux_utility_memory_free(ed -> ux_stm32_ed_setup);

    UX_RESTORE
#endif /* defined(UX_HOST_STANDALONE) */

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

