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
/**                                                                       */ 
/** USBX Component                                                        */ 
/**                                                                       */
/**   Device DPUMP Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_dpump.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_dpump_change                       PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function changes the interface of the DPUMP device             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                             Pointer to dpump command        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_set              Set memory                      */
/*    _ux_device_stack_transfer_all_request_abort                         */
/*                                        Abort request                   */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX Source Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
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
UINT  _ux_device_class_dpump_change(UX_SLAVE_CLASS_COMMAND *command)
{
                                          
UX_SLAVE_INTERFACE                      *interface_ptr;            
UX_SLAVE_CLASS                          *class_ptr;
UX_SLAVE_CLASS_DPUMP                    *dpump;
UX_SLAVE_ENDPOINT                       *endpoint;

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    dpump = (UX_SLAVE_CLASS_DPUMP *) class_ptr -> ux_slave_class_instance;

    /* Get the interface that owns this instance.  */
    interface_ptr =  (UX_SLAVE_INTERFACE  *) command -> ux_slave_class_command_interface;
    
    /* Locate the endpoints.  Control and Bulk in/out for data.  */
    endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;
    
    /* Keep the alternate setting in the dpump structure. */
    dpump -> ux_slave_class_dpump_alternate_setting =  interface_ptr -> ux_slave_interface_descriptor.bAlternateSetting;

    /* If the interface to mount has a non zero alternate setting, the class is really active with
       the endpoints active.  If the interface reverts to alternate setting 0, it needs to have
       the pending transactions terminated.  */
    if (interface_ptr -> ux_slave_interface_descriptor.bAlternateSetting != 0)       
    {
    
        /* Parse all endpoints.  */
        while (endpoint != UX_NULL)
        {
        
            /* Check the endpoint direction, and type.  */
            if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN)
            {
    
                /* Look at type.  */
                if ((endpoint -> ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT)
            
                    /* We have found the bulk in endpoint, save it.  */
                    dpump -> ux_slave_class_dpump_bulkin_endpoint =  endpoint;
                    
            }
            else
            {
                /* Look at type for out endpoint.  */
                if ((endpoint -> ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT)
            
                    /* We have found the bulk out endpoint, save it.  */
                    dpump -> ux_slave_class_dpump_bulkout_endpoint =  endpoint;
            }                
    
            /* Next endpoint.  */
            endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
        }


        /* Now check if all endpoints have been found.  */
        if (dpump -> ux_slave_class_dpump_bulkout_endpoint == UX_NULL || dpump -> ux_slave_class_dpump_bulkin_endpoint == UX_NULL)

            /* Not all endpoints have been found. Major error, do not proceed.  */
            return(UX_ERROR);

        /* Reset the endpoint buffers.  */
        _ux_utility_memory_set(dpump -> ux_slave_class_dpump_bulkout_endpoint -> ux_slave_endpoint_transfer_request. 
                                        ux_slave_transfer_request_data_pointer, 0, UX_SLAVE_REQUEST_DATA_MAX_LENGTH); /* Use case of memset is verified. */
        _ux_utility_memory_set(dpump -> ux_slave_class_dpump_bulkin_endpoint -> ux_slave_endpoint_transfer_request. 
                                        ux_slave_transfer_request_data_pointer, 0, UX_SLAVE_REQUEST_DATA_MAX_LENGTH); /* Use case of memset is verified. */

        /* Keep the alternate setting in the dpump structure. */
        dpump -> ux_slave_class_dpump_alternate_setting =  interface_ptr -> ux_slave_interface_descriptor.bAlternateSetting;

#if defined(UX_DEVICE_STANDALONE)

        /* Reset read/write states.  */
        dpump -> ux_device_class_dpump_read_state = 0;
        dpump -> ux_device_class_dpump_write_state = 0;
#endif

        /* If there is an activate function call it.  */
        if (dpump -> ux_slave_class_dpump_parameter.ux_slave_class_dpump_instance_activate != UX_NULL)

            /* Invoke the application.  */
            dpump -> ux_slave_class_dpump_parameter.ux_slave_class_dpump_instance_activate(dpump);
    }                
    else
    {

        /* In this case, we are reverting to the Alternate Setting 0.  We need to terminate the pending transactions.  */
        /* Terminate the transactions pending on the endpoints (bulk in, bulk out).  */
        _ux_device_stack_transfer_all_request_abort(dpump -> ux_slave_class_dpump_bulkin_endpoint, UX_TRANSFER_APPLICATION_RESET);
        _ux_device_stack_transfer_all_request_abort(dpump -> ux_slave_class_dpump_bulkout_endpoint, UX_TRANSFER_APPLICATION_RESET);

    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_DPUMP_CHANGE, dpump, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_DEVICE_OBJECT_TYPE_INTERFACE, dpump, 0, 0, 0)

    /* Return completion status.  */
    return(UX_SUCCESS);
}

