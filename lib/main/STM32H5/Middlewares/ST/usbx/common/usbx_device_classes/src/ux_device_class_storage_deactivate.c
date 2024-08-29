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
/*    _ux_device_class_storage_deactivate                 PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function deactivate an instance of the storage class.          */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Pointer to a class command    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_all_request_abort Abort all transfers     */ 
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
UINT  _ux_device_class_storage_deactivate(UX_SLAVE_CLASS_COMMAND *command)
{
                                          
UX_SLAVE_CLASS_STORAGE      *storage;
UX_SLAVE_ENDPOINT           *endpoint_in;
UX_SLAVE_ENDPOINT           *endpoint_out;
UX_SLAVE_CLASS              *class_ptr;

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    storage = (UX_SLAVE_CLASS_STORAGE *)class_ptr -> ux_slave_class_instance;

#if defined(UX_DEVICE_STANDALONE)

    endpoint_in = storage -> ux_device_class_storage_ep_in;
    endpoint_out = storage -> ux_device_class_storage_ep_out;
    _ux_device_stack_transfer_all_request_abort(endpoint_in, UX_TRANSFER_BUS_RESET);
    _ux_device_stack_transfer_all_request_abort(endpoint_out, UX_TRANSFER_BUS_RESET);
    endpoint_out -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer =
                                storage -> ux_device_class_storage_buffer[0];
    endpoint_in -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer =
                                storage -> ux_device_class_storage_buffer[1];
#else

    /* Locate the endpoints.  */
    endpoint_in =  storage -> ux_slave_class_storage_interface -> ux_slave_interface_first_endpoint;
    
    /* Check the endpoint direction, if IN we have the correct endpoint.  */
    if ((endpoint_in -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_IN)
    {

        /* Wrong direction, we found the OUT endpoint first.  */
        endpoint_out =  endpoint_in;
            
        /* So the next endpoint has to be the IN endpoint.  */
        endpoint_in =  endpoint_out -> ux_slave_endpoint_next_endpoint;
    }
    else
    {

        /* We found the endpoint IN first, so next endpoint is OUT.  */
        endpoint_out =  endpoint_in -> ux_slave_endpoint_next_endpoint;
    }
        
    /* Terminate the transactions pending on the endpoints.  */
    _ux_device_stack_transfer_all_request_abort(endpoint_in, UX_TRANSFER_BUS_RESET);
    _ux_device_stack_transfer_all_request_abort(endpoint_out, UX_TRANSFER_BUS_RESET);
#endif

    /* If there is a deactivate function call it.  */
    if (storage -> ux_slave_class_storage_instance_deactivate != UX_NULL)
    {

        /* Invoke the application.  */
        storage -> ux_slave_class_storage_instance_deactivate(storage);
    }
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_STORAGE_DEACTIVATE, storage, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_UNREGISTER(storage);

    /* Return completion status.  */
    return(UX_SUCCESS);
}

