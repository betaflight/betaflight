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
/*    _ux_hcd_stm32_transfer_abort                        PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will abort transactions attached to a transfer        */
/*    request.                                                            */
/*                                                                        */
/*    Note since ThreadX delay is used, the function must not be used in  */
/*    interrupts.                                                         */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_stm32                             Pointer to stm32 controller   */
/*    transfer_request                      Pointer to transfer request   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_delay_ms                   Delay                        */
/*    HAL_HCD_HC_Halt                        Halt host channel            */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    stm32 Controller Driver                                             */
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
UINT  _ux_hcd_stm32_transfer_abort(UX_HCD_STM32 *hcd_stm32, UX_TRANSFER *transfer_request)
{

UX_ENDPOINT         *endpoint;
UX_HCD_STM32_ED     *ed;
UX_TRANSFER         *transfer;
UX_INTERRUPT_SAVE_AREA

    /* Get the pointer to the endpoint associated with the transfer request.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

    /* From the endpoint container, get the address of the physical endpoint.  */
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

    UX_DISABLE

    /* Halt the host channel.  */
    HAL_HCD_HC_Halt(hcd_stm32 -> hcd_handle, ed -> ux_stm32_ed_channel);

    /* Save the transfer status in the ED.  */
    ed -> ux_stm32_ed_status = UX_HCD_STM32_ED_STATUS_ABORTED;

    /* Finish current transfer.  */
    _ux_hcd_stm32_request_trans_finish(hcd_stm32, ed);

    /* Update the transfer status in linked transfer requests.  */
    transfer = ed -> ux_stm32_ed_transfer_request;
    while(transfer)
    {

        /* Set transfer status to aborted.  */
        transfer -> ux_transfer_request_status = UX_TRANSFER_STATUS_ABORT;

        /* Get next transfer linked.  */
        transfer = transfer -> ux_transfer_request_next_transfer_request;
    }

    /* No transfer on going.  */
    ed -> ux_stm32_ed_transfer_request = UX_NULL;

    UX_RESTORE

#if !defined(UX_HOST_STANDALONE)

    /* Wait for the controller to finish the current frame processing.  */
    _ux_utility_delay_ms(1);
#else

    /* If setup memory is not freed correct, free it.  */
    if (ed -> ux_stm32_ed_setup)
        _ux_utility_memory_free(ed -> ux_stm32_ed_setup);
#endif /* !defined(UX_HOST_STANDALONE) */

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

