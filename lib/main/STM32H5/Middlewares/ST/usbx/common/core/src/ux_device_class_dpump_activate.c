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
/**   Device Data Pump Class                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_dpump.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_dpump_activate                     PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function activates the USB dpump device.                       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                                 Pointer to dpump command    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Device Data Pump Class                                              */ 
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
UINT  _ux_device_class_dpump_activate(UX_SLAVE_CLASS_COMMAND *command)
{
                                          
UX_SLAVE_INTERFACE                      *interface_ptr;
UX_SLAVE_CLASS_DPUMP                    *dpump;
UX_SLAVE_CLASS                          *class_ptr;
UX_SLAVE_ENDPOINT                       *endpoint;

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Store the class instance in the container.  */
    dpump = (UX_SLAVE_CLASS_DPUMP *) class_ptr -> ux_slave_class_instance;

    /* Get the interface that owns this instance.  */
    interface_ptr =  (UX_SLAVE_INTERFACE  *) command -> ux_slave_class_command_interface;
    
    /* Store the class instance into the interface.  */
    interface_ptr -> ux_slave_interface_class_instance =  (VOID *)dpump;
         
    /* Now the opposite, store the interface in the class instance.  */
    dpump -> ux_slave_class_dpump_interface =  interface_ptr;

    /* Locate the endpoints.  Interrupt for Control and Bulk in/out for Data.  */
    endpoint =  interface_ptr -> ux_slave_interface_first_endpoint;
    
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

#if defined(UX_DEVICE_STANDALONE)

    /* Reset read/write states.  */
    dpump -> ux_device_class_dpump_read_state = 0;
    dpump -> ux_device_class_dpump_write_state = 0;
#endif

    /* If there is a activate function call it.  */
    if (dpump -> ux_slave_class_dpump_parameter.ux_slave_class_dpump_instance_activate != UX_NULL)
    {
    
        /* Invoke the application.  */
        dpump -> ux_slave_class_dpump_parameter.ux_slave_class_dpump_instance_activate(dpump);
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_DPUMP_ACTIVATE, dpump, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)
  
    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_DEVICE_OBJECT_TYPE_INTERFACE, dpump, 0, 0, 0)

    /* Return completion status.  */
    return(UX_SUCCESS);
}

