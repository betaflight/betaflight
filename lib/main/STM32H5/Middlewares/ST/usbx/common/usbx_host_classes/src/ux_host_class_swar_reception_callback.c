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
/**   Host Sierra Wireless AR module class                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_swar.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_swar_reception_callback              PORTABLE C      */ 
/*                                                           6.1          */
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
VOID  _ux_host_class_swar_reception_callback (UX_TRANSFER *transfer_request)
{

UX_HOST_CLASS_SWAR               *swar;
UX_HOST_CLASS_SWAR_RECEPTION     *swar_reception;
    
    /* Get the class instance for this transfer request.  */
    swar =  (UX_HOST_CLASS_SWAR *) transfer_request -> ux_transfer_request_class_instance;
    
    /* Get the pointer to the acm reception structure.  */
    swar_reception =  swar -> ux_host_class_swar_reception;

    /* Check the state of the transfer.  If there is an error, we do not proceed with this report.  */
    if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {
        
        /* The reception is stopped.  */
        swar_reception -> ux_host_class_swar_reception_state =  UX_HOST_CLASS_SWAR_RECEPTION_STATE_STOPPED;

        /* We do not proceed.  */
        return;        
        
    }

    /* And move to the next reception buffer.  Check if we are at the end of the application buffer.  */
    if (swar_reception -> ux_host_class_swar_reception_data_head + swar_reception -> ux_host_class_swar_reception_block_size >= 
        swar_reception -> ux_host_class_swar_reception_data_buffer + swar_reception -> ux_host_class_swar_reception_data_buffer_size)
    {

        /* We are at the end of the buffer. Move back to the beginning if we have space available. */
        if (swar_reception -> ux_host_class_swar_reception_data_tail ==  swar_reception -> ux_host_class_swar_reception_data_buffer)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_BUFFER_OVERFLOW);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_BUFFER_OVERFLOW, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

            /* We have an overflow. We cannot continue.  Report to the application.  */
            swar_reception -> ux_host_class_swar_reception_callback(swar, UX_BUFFER_OVERFLOW, UX_NULL, 0); 
            
            /* And stop the transfer in progress flag.  */
            swar_reception -> ux_host_class_swar_reception_state =  UX_HOST_CLASS_SWAR_RECEPTION_STATE_STOPPED;
            
            return;
        }
        else
        
            /* Program the head to be at the beginning of the application buffer.  */
            swar_reception -> ux_host_class_swar_reception_data_head =  swar_reception -> ux_host_class_swar_reception_data_buffer;
                    
    }        
    else

            /* Program the head to be after the current buffer.  */
            swar_reception -> ux_host_class_swar_reception_data_head +=  swar_reception -> ux_host_class_swar_reception_block_size;


    /* We need to report this transfer to the application.  */
    swar_reception -> ux_host_class_swar_reception_callback(swar, 
                                                                    transfer_request -> ux_transfer_request_completion_code,
                                                                    transfer_request -> ux_transfer_request_data_pointer,
                                                                    transfer_request -> ux_transfer_request_actual_length);

    /* Arm another transfer.  */
    _ux_host_stack_transfer_request(transfer_request);

    /* There is no status to be reported back to the stack.  */
    return; 
}

