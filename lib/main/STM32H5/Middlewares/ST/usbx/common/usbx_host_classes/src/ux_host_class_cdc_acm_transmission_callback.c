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
/**   ACM CDC Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_cdc_acm.h"
#include "ux_host_stack.h"


#if defined(UX_HOST_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_cdc_acm_transmission_callback        PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the callback from the USBX transfer functions,     */
/*    it is called when a full or partial transfer has been done for a    */
/*    bulk in transfer. It calls back the application.                    */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    transfer_request                      Pointer to transfer request   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_stack_transfer_run           Process transfer request      */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_cdc_acm_transmission_callback(UX_TRANSFER *transfer_request)
{

UX_HOST_CLASS_CDC_ACM   *cdc_acm;
ULONG                   request_length;
VOID                    (*write_callback)(struct UX_HOST_CLASS_CDC_ACM_STRUCT *, UINT, ULONG);

    /* Get the class instance for this transfer request.  */
    cdc_acm = (UX_HOST_CLASS_CDC_ACM *) transfer_request -> ux_transfer_request_class_instance;

    /* Check the state of the transfer.  If there is an error, we do not proceed with this report.  */
    if (transfer_request -> ux_transfer_request_completion_code == UX_SUCCESS)
    {

        /* Update write information.  */
        cdc_acm -> ux_host_class_cdc_acm_write_count +=
                        transfer_request -> ux_transfer_request_actual_length;

        /* Check if there is remaining.  */
        request_length = cdc_acm -> ux_host_class_cdc_acm_write_length -
                                 cdc_acm -> ux_host_class_cdc_acm_write_count;
        if (request_length > 0)
        {

            /* Program the maximum authorized length for this transfer_request.  */
            if (request_length > transfer_request -> ux_transfer_request_maximum_length)
                request_length =  transfer_request -> ux_transfer_request_maximum_length;

            /* Update buffer pointer and request length.  */
            transfer_request -> ux_transfer_request_data_pointer +=
                        transfer_request -> ux_transfer_request_actual_length;
            transfer_request -> ux_transfer_request_requested_length = request_length;
            UX_TRANSFER_STATE_RESET(transfer_request);

            /* Arm another transfer.  */
            _ux_host_stack_transfer_run(transfer_request);

            /* There is no status to be reported back to the stack.  */
            return;
        }
    }

    /* There is error, or total things done, set state to idle.  */
    cdc_acm -> ux_host_class_cdc_acm_write_state = UX_STATE_RESET;

    /* Clear callback for blocking mode.  */
    write_callback = cdc_acm -> ux_host_class_cdc_acm_write_callback;
    cdc_acm -> ux_host_class_cdc_acm_write_callback = UX_NULL;

    /* We need to report this transfer to the application.  */
    if (write_callback)
    {
        write_callback(cdc_acm,
                    transfer_request -> ux_transfer_request_completion_code,
                    cdc_acm -> ux_host_class_cdc_acm_write_count);
    }

    /* There is no status to be reported back to the stack.  */
    return;
}
#endif
