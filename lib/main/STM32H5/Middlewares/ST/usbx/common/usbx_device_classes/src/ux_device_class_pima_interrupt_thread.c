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
/**   Device PIMA Class                                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_pima.h"
#include "ux_device_stack.h"


#if !defined(UX_DEVICE_STANDALONE)
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_pima_interrupt_thread              PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the thread of the pima interrupt endpoint          */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima_class                               Address of pima class      */ 
/*                                                container               */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_request     Request transfer              */ 
/*    _ux_utility_memory_allocate           Allocate memory               */ 
/*    _ux_device_semaphore_get              Get semaphore                 */
/*    _ux_device_class_pima_event_get       Get PIMA event                */
/*    _ux_utility_long_put                  Put 32-bit value              */
/*    _ux_utility_short_put                 Put 16-bit value              */
/*    _ux_device_thread_suspend             Suspend thread                */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    ThreadX                                                             */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            added transaction ID,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed event message size,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_device_class_pima_interrupt_thread(ULONG pima_class)
{

UX_SLAVE_CLASS_PIMA         *pima;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_TRANSFER           *transfer_request_in;
UX_SLAVE_CLASS_PIMA_EVENT   pima_event;
UINT                        status;
UCHAR                       *buffer;
    
    /* Get the pima instance from the calling parameter.  */
    UX_THREAD_EXTENSION_PTR_GET(pima, UX_SLAVE_CLASS_PIMA, pima_class)

    /* Allocate the event round robin buffer.  */
    pima -> ux_device_class_pima_event_array =  
            _ux_utility_memory_allocate_mulc_safe(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_SLAVE_CLASS_PIMA_EVENT), UX_DEVICE_CLASS_PIMA_MAX_EVENTS_QUEUE);

    /* Check for successful allocation.  */
    if (pima -> ux_device_class_pima_event_array == UX_NULL)
    {
        /* Return, no event management.  */
        return;
    }        

    /* Allocate the head\tail and end of the round robin buffer.  */
    pima -> ux_device_class_pima_event_array_head =  pima -> ux_device_class_pima_event_array;
    pima -> ux_device_class_pima_event_array_tail =  pima -> ux_device_class_pima_event_array;
    pima -> ux_device_class_pima_event_array_end  =  pima -> ux_device_class_pima_event_array + UX_DEVICE_CLASS_PIMA_MAX_EVENTS_QUEUE;

    /* This thread runs forever but can be suspended or resumed.  */
    while(1)
    {

        /* Get the pointer to the device.  */
        device =  &_ux_system_slave -> ux_system_slave_device;
        
        /* All PIMA events are on the interrupt endpoint IN, from the host.  */
        transfer_request_in =  &pima -> ux_device_class_pima_interrupt_endpoint -> ux_slave_endpoint_transfer_request;
    
        /* As long as the device is in the CONFIGURED state.  */
        while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED)
        { 

            /* Wait until something has awaken us.  */
            status =  _ux_device_semaphore_get(&pima -> ux_device_class_pima_interrupt_thread_semaphore, UX_WAIT_FOREVER);
            
            /* Check the completion code. */
            if (status != UX_SUCCESS)

                /* Do not proceed.  */
                return;

            /* Check if we have an event to report.  */
            status = _ux_device_class_pima_event_get(pima, &pima_event);
            
            /* We may have an event to report on the interrupt pipe.  */
            if(status == UX_SUCCESS)
            {

                /* Prepare the event data payload from the pima event structure.  Get a pointer to the buffer area.  */
                buffer =  transfer_request_in -> ux_slave_transfer_request_data_pointer;
                
                /* Put the length of the entire event payload.  */
                _ux_utility_long_put(buffer + UX_DEVICE_CLASS_PIMA_AEI_DATA_LENGTH, UX_DEVICE_CLASS_PIMA_AEI_MAX_LENGTH);
                
                /* Put the type of packet (Event)   */
                _ux_utility_short_put(buffer + UX_DEVICE_CLASS_PIMA_AEI_TYPE, UX_DEVICE_CLASS_PIMA_CT_EVENT_BLOCK);

                /* Put the type of event.   */
                _ux_utility_short_put(buffer + UX_DEVICE_CLASS_PIMA_AEI_EVENT_CODE, (USHORT)pima_event.ux_device_class_pima_event_code);
                
                /* Put the transaction ID.  */
                _ux_utility_long_put(buffer + UX_DEVICE_CLASS_PIMA_AEI_TRANSACTION_ID, pima_event.ux_device_class_pima_event_transaction_id);
                
                /* Put the value of parameter 1.   */
                _ux_utility_long_put(buffer + UX_DEVICE_CLASS_PIMA_AEI_PARAMETER_1, pima_event.ux_device_class_pima_event_parameter_1);
                
                /* Put the value of parameter 2.   */
                _ux_utility_long_put(buffer + UX_DEVICE_CLASS_PIMA_AEI_PARAMETER_2, pima_event.ux_device_class_pima_event_parameter_3);

                /* Put the value of parameter 3.   */
                _ux_utility_long_put(buffer + UX_DEVICE_CLASS_PIMA_AEI_PARAMETER_2, pima_event.ux_device_class_pima_event_parameter_3);
                
                /* Send the request to the device controller.  */
                status =  _ux_device_stack_transfer_request(transfer_request_in, UX_DEVICE_CLASS_PIMA_AEI_MAX_LENGTH, UX_DEVICE_CLASS_PIMA_AEI_MAX_LENGTH);

                /* Check error code. */
                if (status != UX_SUCCESS)

                    /* Error trap. */
                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);

            }                
        }

    /* We need to suspend ourselves. We will be resumed by the device enumeration module.  */
    _ux_device_thread_suspend(&pima -> ux_device_class_pima_interrupt_thread);
    }
}
#endif
