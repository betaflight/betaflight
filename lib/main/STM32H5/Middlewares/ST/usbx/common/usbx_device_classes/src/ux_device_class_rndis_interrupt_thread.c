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
/**   Device RNDIS Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_rndis.h"
#include "ux_device_stack.h"


#if !defined(UX_DEVICE_STANDALONE)
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_rndis_interrupt_thread             PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the thread of the rndis interrupt endpoint         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    rndis_class                             Address of rndis class      */ 
/*                                                container               */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_request     Request transfer              */ 
/*    _ux_utility_event_flags_get           Get event flags               */
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
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_device_class_rndis_interrupt_thread(ULONG rndis_class)
{

UX_SLAVE_CLASS                  *class_ptr;
UX_SLAVE_CLASS_RNDIS            *rndis;
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_TRANSFER               *transfer_request;
UINT                            status;
ULONG                           actual_flags;

    /* Cast properly the rndis instance.  */
    UX_THREAD_EXTENSION_PTR_GET(class_ptr, UX_SLAVE_CLASS, rndis_class)
    
    /* Get the rndis instance from this class container.  */
    rndis =  (UX_SLAVE_CLASS_RNDIS *) class_ptr -> ux_slave_class_instance;
    
    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;
    
    /* This thread runs forever but can be suspended or resumed.  */
    while(1)
    {

        /* All RNDIS events are on the interrupt endpoint IN, from the host.  */
        transfer_request =  &rndis -> ux_slave_class_rndis_interrupt_endpoint -> ux_slave_endpoint_transfer_request;

        /* As long as the device is in the CONFIGURED state.  */
        while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED)
        { 


            /* Wait until we have a event sent by the application. We do not treat yet the case where a timeout based
               on the interrupt pipe frequency or a change in the idle state forces us to send an empty report.  */
            status =  _ux_utility_event_flags_get(&rndis -> ux_slave_class_rndis_event_flags_group, (UX_DEVICE_CLASS_RNDIS_NEW_INTERRUPT_EVENT |
                                                                                            UX_DEVICE_CLASS_RNDIS_NEW_DEVICE_STATE_CHANGE_EVENT), 
                                                                                            UX_OR_CLEAR, &actual_flags, UX_WAIT_FOREVER);
                                                                                            
                                                                                            /* If error log is enabled, insert this error message into the log buffer.  */
                  
            /* Check the completion code. */
            if (status == UX_SUCCESS && (actual_flags & UX_DEVICE_CLASS_RNDIS_NEW_DEVICE_STATE_CHANGE_EVENT) == 0)
            {
                
                /* Send the request to the device controller.  */
                status =  _ux_device_stack_transfer_request(transfer_request, UX_DEVICE_CLASS_RNDIS_INTERRUPT_RESPONSE_LENGTH,
                                                                    UX_DEVICE_CLASS_RNDIS_INTERRUPT_RESPONSE_LENGTH);
                
                /* Check error code. */
                if (status != UX_SUCCESS)

                    /* Error trap. */
                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);
            }
        }
         
    /* We need to suspend ourselves. We will be resumed by the device enumeration module.  */
    _ux_device_thread_suspend(&rndis -> ux_slave_class_rndis_interrupt_thread);

    }
}
#endif
