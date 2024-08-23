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


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_pima_event_set                     PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function sends an event to the pima class. It is processed     */ 
/*    asynchronously by the interrupt thread.                             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                     Address of pima class      */ 
/*    event                                    Pointer of the event       */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    status                                   UX_SUCCESS if there is an  */ 
/*                                             event                      */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_semaphore_put                 Put semaphore              */
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_event_set(UX_SLAVE_CLASS_PIMA *pima, 
                                      UX_SLAVE_CLASS_PIMA_EVENT *pima_event)
{

UX_SLAVE_CLASS_PIMA_EVENT       *current_pima_event;
UX_SLAVE_CLASS_PIMA_EVENT       *next_pima_event;
UX_SLAVE_DEVICE                 *device;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_EVENT_SET, pima, pima_event, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;
    
    /* Check the device state.  */
    if (device -> ux_slave_device_state !=  UX_DEVICE_CONFIGURED)
    {

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DEVICE_HANDLE_UNKNOWN, device, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_DEVICE_HANDLE_UNKNOWN);
    }
    
    /* Current position of the head.  */
    current_pima_event =  pima -> ux_device_class_pima_event_array_head;
    
    /* If the pointer is NULL, the round robin buffer has not been activated.  */
    if (current_pima_event == UX_NULL)
        return (UX_ERROR);
    
    /* Calculate the next position.  */
    if ((current_pima_event + 1) == pima -> ux_device_class_pima_event_array_end)

        /* We are at the end, go back to the beginning.  */
        next_pima_event =  pima -> ux_device_class_pima_event_array;
        
    else        
        /* We are not at the end, increment the head position.  */
        next_pima_event = current_pima_event + 1;
    

    /* Any place left for this event ? */
    if (next_pima_event == pima -> ux_device_class_pima_event_array_tail)
        return (UX_ERROR);

    /* Update the head.  */
    pima -> ux_device_class_pima_event_array_head = next_pima_event;

    /* There is an event to report, get the current pointer to the event.  */
    current_pima_event =  pima -> ux_device_class_pima_event_array_tail;
    
    /* fill in the event structure from the user.  */
    current_pima_event -> ux_device_class_pima_event_code           = pima_event -> ux_device_class_pima_event_code;      
    current_pima_event -> ux_device_class_pima_event_transaction_id = pima_event -> ux_device_class_pima_event_transaction_id;
    current_pima_event -> ux_device_class_pima_event_parameter_1    = pima_event -> ux_device_class_pima_event_parameter_1;     
    current_pima_event -> ux_device_class_pima_event_parameter_2    = pima_event -> ux_device_class_pima_event_parameter_2;     
    current_pima_event -> ux_device_class_pima_event_parameter_3    = pima_event -> ux_device_class_pima_event_parameter_3;     
    
    /* Set a semaphore to wake up the interrupt thread.  */
    _ux_device_semaphore_put(&pima -> ux_device_class_pima_interrupt_thread_semaphore);

    /* Return event status to the user.  */
    return(UX_SUCCESS);
}

