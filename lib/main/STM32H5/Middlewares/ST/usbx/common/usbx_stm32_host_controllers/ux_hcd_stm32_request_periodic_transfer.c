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
/*    _ux_hcd_stm32_request_periodic_transfer             PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*     This function performs an periodic transfer request. An periodic   */
/*     transfer can only be as large as the MaxpacketField in the         */
/*     endpoint descriptor. This was verified at the USB layer and does   */
/*     not need to be reverified here.                                    */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_stm32                             Pointer to STM32 controller   */
/*    transfer_request                      Pointer to transfer request   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    UX_DISABLE                            Disable interrupt             */
/*    UX_RESTORE                            Restore interrupt             */
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
/*                                            refined macros names,       */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_stm32_request_periodic_transfer(UX_HCD_STM32 *hcd_stm32, UX_TRANSFER *transfer_request)
{

UX_ENDPOINT             *endpoint;
UX_HCD_STM32_ED         *ed;
UX_TRANSFER             *transfer;
UX_INTERRUPT_SAVE_AREA


    /* Get the pointer to the Endpoint.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

    /* Now get the physical ED attached to this endpoint.  */
    ed =  endpoint -> ux_endpoint_ed;

    /* Disable interrupt.  */
    UX_DISABLE

#if defined(UX_HOST_STANDALONE)

    /* Check if transfer is still in progress.  */
    if ((ed -> ux_stm32_ed_status & UX_HCD_STM32_ED_STATUS_PENDING_MASK) >
        UX_HCD_STM32_ED_STATUS_ABORTED)
    {

        /* Check done bit.  */
        if ((ed -> ux_stm32_ed_status & UX_HCD_STM32_ED_STATUS_TRANSFER_DONE) == 0)
        {
            UX_RESTORE
            return(UX_STATE_WAIT);
        }

        /* Check status to see if it's first initialize.  */
        if (transfer_request -> ux_transfer_request_status !=
            UX_TRANSFER_STATUS_NOT_PENDING)
        {

            /* Done, modify status and notify state change.  */
            ed -> ux_stm32_ed_status = UX_HCD_STM32_ED_STATUS_ALLOCATED;
            UX_RESTORE
            return(UX_STATE_NEXT);
        }

        /* Maybe transfer completed but state not reported yet.  */
    }
    transfer_request -> ux_transfer_request_status = UX_TRANSFER_STATUS_PENDING;

#endif /* defined(UX_HOST_STANDALONE) */

    /* Save the transfer status in the ED.  */
    ed -> ux_stm32_ed_status = UX_HCD_STM32_ED_STATUS_PERIODIC_TRANSFER;

    /* Isochronous transfer supports transfer list.  */
    if (ed -> ux_stm32_ed_transfer_request == UX_NULL)
    {

        /* Scheduler is needed to start, and kept if interval is more than 1 SOF/uSOF.  */
        ed -> ux_stm32_ed_sch_mode = 1;

    /* Save the pending transfer in the ED.  */
    ed -> ux_stm32_ed_transfer_request = transfer_request;
    }
    else
    {

        /* Link the pending transfer to list tail.  */
        transfer = ed -> ux_stm32_ed_transfer_request;
        while(transfer -> ux_transfer_request_next_transfer_request != UX_NULL)
            transfer = transfer -> ux_transfer_request_next_transfer_request;
        transfer -> ux_transfer_request_next_transfer_request = transfer_request;
    }

    /* Restore interrupt.  */
    UX_RESTORE

#if defined(UX_HOST_STANDALONE)

    /* Background transfer started but not done yet.  */
    return(UX_STATE_WAIT);
#else

    /* There is no need to wake up the stm32 controller on this transfer
       since periodic transactions will be picked up when the interrupt
       tree is scanned.  */
    return(UX_SUCCESS);
#endif /* defined(UX_HOST_STANDALONE) */

}

