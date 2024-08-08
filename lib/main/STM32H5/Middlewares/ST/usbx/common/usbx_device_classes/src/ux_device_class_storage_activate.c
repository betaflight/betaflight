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
/**   Device Storage Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_storage.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_storage_activate                   PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function activates the USB storage device.                     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Pointer to storage command    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_thread_resume              Resume thread                 */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Device Storage Class                                                */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_storage_activate(UX_SLAVE_CLASS_COMMAND *command)
{
                                          
UINT                                    status = UX_SUCCESS;
UX_SLAVE_INTERFACE                      *interface_ptr;
UX_SLAVE_CLASS                          *class_ptr;
UX_SLAVE_CLASS_STORAGE                  *storage;
#if defined(UX_DEVICE_STANDALONE)
UX_SLAVE_ENDPOINT                       *endpoint;
#endif


    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    storage = (UX_SLAVE_CLASS_STORAGE *)class_ptr -> ux_slave_class_instance;

    /* Get the interface that owns this instance.  */
    interface_ptr =  (UX_SLAVE_INTERFACE  *) command -> ux_slave_class_command_interface;
    
    /* Store the class instance into the interface.  */
    interface_ptr -> ux_slave_interface_class_instance =  (VOID *)storage;
         
    /* Now the opposite, store the interface in the class instance.  */
    storage -> ux_slave_class_storage_interface =  interface_ptr;

#if !defined(UX_DEVICE_STANDALONE)

    /* Resume thread.  */
    _ux_device_thread_resume(&class_ptr -> ux_slave_class_thread); 

#else

    /* Locate the endpoints.  */
    /* Check the first endpoint direction, if IN we have the correct endpoint.  */
    endpoint = interface_ptr -> ux_slave_interface_first_endpoint;
    if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_IN)
    {

        /* Wrong direction, we found the OUT endpoint first.  */
        storage -> ux_device_class_storage_ep_out = endpoint;

        /* So the next endpoint has to be the IN endpoint.  */
        storage -> ux_device_class_storage_ep_in = endpoint -> ux_slave_endpoint_next_endpoint;
    }
    else
    {

        /* We found the IN endpoint first.  */
        storage -> ux_device_class_storage_ep_in = endpoint;
            
        /* So the next endpoint has to be the OUT endpoint.  */
        storage -> ux_device_class_storage_ep_out = endpoint -> ux_slave_endpoint_next_endpoint;
    }

    /* Reset states.  */
    storage -> ux_device_class_storage_buffer[0] = storage -> ux_device_class_storage_ep_out ->
                    ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer;
    storage -> ux_device_class_storage_buffer[1] = storage -> ux_device_class_storage_ep_in ->
                    ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer;
    storage -> ux_device_class_storage_data_buffer = UX_NULL;
    storage -> ux_device_class_storage_state = UX_STATE_RESET;
    storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_IDLE;
    storage -> ux_device_class_storage_buffer_state[0] = UX_DEVICE_CLASS_STORAGE_BUFFER_IDLE;
    storage -> ux_device_class_storage_buffer_state[1] = UX_DEVICE_CLASS_STORAGE_BUFFER_IDLE;
    storage -> ux_device_class_storage_buffer_usb = 0;
    storage -> ux_device_class_storage_buffer_disk = 0;
    UX_SLAVE_TRANSFER_STATE_RESET(&storage -> ux_device_class_storage_ep_out -> ux_slave_endpoint_transfer_request);
    UX_SLAVE_TRANSFER_STATE_RESET(&storage -> ux_device_class_storage_ep_in -> ux_slave_endpoint_transfer_request);

    status = UX_SUCCESS;
#endif

    /* If there is a activate function call it.  */
    if (storage -> ux_slave_class_storage_instance_activate != UX_NULL)
    {        
        /* Invoke the application.  */
        storage -> ux_slave_class_storage_instance_activate(storage);
    }
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_STORAGE_ACTIVATE, storage, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_DEVICE_OBJECT_TYPE_INTERFACE, storage, 0, 0, 0)

    /* Return completion status.  */
    return(status);
}

