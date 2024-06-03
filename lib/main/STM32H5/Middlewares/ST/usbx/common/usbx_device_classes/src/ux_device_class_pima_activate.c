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
/**   Device CDC Class                                                    */
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
/*    _ux_device_class_pima_activate                      PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function activates the USB Pima device.                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                              Pointer to pima command        */ 
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
/*    USBX Source Code                                                    */ 
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
/*                                            added variables initialize, */
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
UINT  _ux_device_class_pima_activate(UX_SLAVE_CLASS_COMMAND *command)
{
                                          
UX_SLAVE_INTERFACE                      *interface_ptr;
UX_SLAVE_CLASS                          *class_ptr;
UX_SLAVE_CLASS_PIMA                     *pima;
UX_SLAVE_ENDPOINT                       *endpoint_in;
UX_SLAVE_ENDPOINT                       *endpoint_out;
UX_SLAVE_ENDPOINT                       *endpoint_interrupt;


    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Store the class instance in the container.  */
    pima = (UX_SLAVE_CLASS_PIMA *) class_ptr -> ux_slave_class_instance;

    /* Get the interface that owns this instance.  */
    interface_ptr =  (UX_SLAVE_INTERFACE  *) command -> ux_slave_class_command_interface;
    
    /* Store the class instance into the interface.  */
    interface_ptr -> ux_slave_interface_class_instance =  (VOID *)pima;
         
    /* Now the opposite, store the interface in the class instance.  */
    pima -> ux_slave_class_pima_interface =  interface_ptr;
    
    /* Locate the endpoints.  */
    endpoint_in =  interface_ptr -> ux_slave_interface_first_endpoint;
    
    /* Check the endpoint direction, if IN we have the correct endpoint.  */
    if ((endpoint_in -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_OUT)
    {

        /* Wrong direction, we found the OUT endpoint first.  */
        endpoint_out =  endpoint_in;
            
        /* So the next endpoint has to be the IN endpoint.  */
        endpoint_in =  endpoint_out -> ux_slave_endpoint_next_endpoint;
        
        /* And the endpoint after that interrupt.  */
        endpoint_interrupt =  endpoint_in -> ux_slave_endpoint_next_endpoint;
        
    }
    else
    {

        /* We found the endpoint IN first, so next endpoint is OUT.  */
        endpoint_out =  endpoint_in -> ux_slave_endpoint_next_endpoint;

        /* And the endpoint after that interrupt.  */
        endpoint_interrupt =  endpoint_out -> ux_slave_endpoint_next_endpoint;
    }

    /* Save the endpoints in the pima instance.  */
    pima -> ux_device_class_pima_bulk_in_endpoint           = endpoint_in;
    pima -> ux_device_class_pima_bulk_out_endpoint          = endpoint_out;
    pima -> ux_device_class_pima_interrupt_endpoint         = endpoint_interrupt;

    /* Initialize status code.  */
    pima -> ux_device_class_pima_state = UX_DEVICE_CLASS_PIMA_PHASE_IDLE;
    pima -> ux_device_class_pima_session_id = 0;
    pima -> ux_device_class_pima_device_status = UX_DEVICE_CLASS_PIMA_RC_OK;

    /* Resume thread.  */
    _ux_device_thread_resume(&class_ptr -> ux_slave_class_thread); 
    
    /* If there is a activate function call it.  */
    if (pima -> ux_device_class_pima_instance_activate != UX_NULL)
    {        
        /* Invoke the application.  */
        pima -> ux_device_class_pima_instance_activate(pima);
    }
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_ACTIVATE, pima, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_DEVICE_OBJECT_TYPE_INTERFACE, pima, 0, 0, 0)

    /* Return completion status.  */
    return(UX_SUCCESS);
}

