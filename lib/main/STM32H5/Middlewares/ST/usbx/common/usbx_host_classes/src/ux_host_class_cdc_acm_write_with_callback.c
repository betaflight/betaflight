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
/**   CDC ACM Class                                                       */
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
/*    _ux_host_class_cdc_acm_write_with_callback          PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function writes to the cdc_acm interface. The call starts      */
/*    background transmission and returns immediately.                    */
/*    A callback is invoked when background transmission is done or there */
/*    is error during transmission.                                       */
/*                                                                        */
/*    Following IOCTL functions can be used with this function:           */
/*    - _WRITE_CALLBACK                     Set the callback function     */
/*    - _GET_WRITE_STATUS                   Return UX_BUSY if transfer is */
/*                                          pending, otherwise report     */
/*                                          actual length of bytes done,  */
/*                                          a pointer to ULONG must be    */
/*                                          passed to fill the actual     */
/*                                          length value, as parameter    */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    cdc_acm                               Pointer to cdc_acm class      */
/*    data_pointer                          Pointer to data to write      */
/*    requested_length                      Length of data to write       */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_stack_transfer_run           Run transfer state machine    */
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
UINT  _ux_host_class_cdc_acm_write_with_callback(UX_HOST_CLASS_CDC_ACM *cdc_acm,
                                    UCHAR *data_pointer, ULONG requested_length)
{

UX_TRANSFER     *transfer_request;
UINT            status;
ULONG           transfer_request_length;


    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_CDC_ACM_WRITE, cdc_acm, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Ensure the instance is valid.  */
    if (cdc_acm -> ux_host_class_cdc_acm_state !=  UX_HOST_CLASS_INSTANCE_LIVE)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, cdc_acm, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* As further protection, we must ensure this instance of the interface is the data interface and not
       the control interface !  */
    if (cdc_acm -> ux_host_class_cdc_acm_interface -> ux_interface_descriptor.bInterfaceClass != UX_HOST_CLASS_CDC_DATA_CLASS)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, cdc_acm, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Check write state.  */
    if (cdc_acm -> ux_host_class_cdc_acm_write_state != UX_STATE_RESET)
        return(UX_BUSY);

    /* Set state to busy.  */
    cdc_acm -> ux_host_class_cdc_acm_write_state = UX_STATE_WAIT;

    /* Reset write status.  */
    cdc_acm -> ux_host_class_cdc_acm_write_length = requested_length;
    cdc_acm -> ux_host_class_cdc_acm_write_count = 0;

    /* Get the pointer to the bulk out endpoint transfer request.  */
    transfer_request =  &cdc_acm -> ux_host_class_cdc_acm_bulk_out_endpoint -> ux_endpoint_transfer_request;

    /* Initialize request for callback mode.  */
    transfer_request -> ux_transfer_request_class_instance      = (VOID *) cdc_acm;
    transfer_request -> ux_transfer_request_completion_function = _ux_host_class_cdc_acm_transmission_callback;

    /* Program the maximum authorized length for this transfer_request.  */
    if (requested_length > transfer_request -> ux_transfer_request_maximum_length)
        transfer_request_length =  transfer_request -> ux_transfer_request_maximum_length;
    else
        transfer_request_length =  requested_length;

    /* Initialize the transfer_request.  */
    transfer_request -> ux_transfer_request_data_pointer = data_pointer;
    transfer_request -> ux_transfer_request_requested_length = transfer_request_length;
    UX_TRANSFER_STATE_RESET(transfer_request);

    /* Start the transfer.  */
    status =  _ux_host_stack_transfer_run(transfer_request);

    /* We get here when all the transfers went through without errors.  */
    return((status == UX_STATE_WAIT) ?
        UX_SUCCESS : transfer_request -> ux_transfer_request_completion_code);
}
#endif
