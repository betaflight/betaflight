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
/**   Generic Serial Host module class                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_gser.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_gser_reception_start                 PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function starts a reception with the generic serial class. This*/ 
/*    allows for non blocking calls based on a packet orientated round    */ 
/*    robbin buffer. When a packet is fully or partially received, an     */
/*    application callback function is invoked and a new transfer request */
/*    is rescheduled.                                                     */
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    gser                               Pointer to gser class            */ 
/*    gser_reception                     Pointer to reception struct      */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_gser_reception_start (UX_HOST_CLASS_GSER *gser, 
                                    UX_HOST_CLASS_GSER_RECEPTION *gser_reception)
{

UX_TRANSFER     *transfer_request;
UINT            status;
ULONG           interface_index;
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_GSER_RECEPTION_START, gser, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Ensure the instance is valid.  */
    if (gser -> ux_host_class_gser_state !=  UX_HOST_CLASS_INSTANCE_LIVE)
    {       

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, gser, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Get the interface index.  */
    interface_index = gser_reception -> ux_host_class_gser_reception_interface_index;

    /* Start by aligning the head and tail of buffers to the same address supplied by the application.  */
    gser_reception -> ux_host_class_gser_reception_data_head =  gser_reception -> ux_host_class_gser_reception_data_buffer;
    gser_reception -> ux_host_class_gser_reception_data_tail =  gser_reception -> ux_host_class_gser_reception_data_buffer;

    /* Get the pointer to the bulk in endpoint in the transfer_request.  */
    transfer_request =  &gser -> ux_host_class_gser_interface_array[interface_index].ux_host_class_gser_bulk_in_endpoint -> ux_endpoint_transfer_request;
    
    /* Save the interface number in the Transfer Request. */
    transfer_request -> ux_transfer_request_user_specific = (VOID *) (ALIGN_TYPE) interface_index;

    /* Initialize the transfer request.  */
    transfer_request -> ux_transfer_request_class_instance      =  (VOID *) gser;
    transfer_request -> ux_transfer_request_data_pointer        =  gser_reception -> ux_host_class_gser_reception_data_head;
    transfer_request -> ux_transfer_request_requested_length    =  gser_reception -> ux_host_class_gser_reception_block_size;
    transfer_request -> ux_transfer_request_completion_function =  _ux_host_class_gser_reception_callback;
    
    /* Save the acm reception structure in the acm structure.  */
    gser -> ux_host_class_gser_interface_array[interface_index].ux_host_class_gser_reception = gser_reception;
    
    /* And declare we have a transfer in progress.  */
    gser_reception -> ux_host_class_gser_reception_state =  UX_HOST_CLASS_GSER_RECEPTION_STATE_STARTED;
                    
    /* Arm a first transfer on the bulk in endpoint. There is a callback to this function so we return to the caller
       right away. */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* We do not know if the first transfer was successful yet.  If the status is not OK, we need to stop the transfer
       in progress flag. */
    if (status != UX_SUCCESS)
        gser_reception -> ux_host_class_gser_reception_state =  UX_HOST_CLASS_GSER_RECEPTION_STATE_STOPPED;
    
    return(status); 
}

