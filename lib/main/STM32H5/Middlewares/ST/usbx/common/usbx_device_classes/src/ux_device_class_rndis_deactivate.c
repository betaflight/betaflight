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


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_rndis_deactivate                   PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function deactivate an instance of the rndis class.            */ 
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
/*    _ux_device_stack_transfer_all_request_abort                         */
/*                                          Abort all transfers           */
/*    _ux_device_event_flags_set            Set event flags               */
/*    _ux_network_driver_deactivate         Deactivate NetX USB interface */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    RNDIS Class                                                         */
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
/*  08-02-2021     Wen Wang                 Modified comment(s),          */
/*                                            fixed spelling error,       */
/*                                            resulting in version 6.1.8  */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_rndis_deactivate(UX_SLAVE_CLASS_COMMAND *command)
{
                                          
UX_SLAVE_CLASS_RNDIS        *rndis;
UX_SLAVE_INTERFACE          *interface_ptr;            
UX_SLAVE_CLASS              *class_ptr;

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    rndis = (UX_SLAVE_CLASS_RNDIS *) class_ptr -> ux_slave_class_instance;

    /* Get the interface that owns this instance.  Normally the interface can be derived
       from the class instance but since RNDIS has 2 interfaces and we only store the Control
       interface in the class container, we used the class_command pointer to retrieve the
       correct interface which issued the deactivation. */
    interface_ptr =  (UX_SLAVE_INTERFACE  *) command -> ux_slave_class_command_interface;
    
    /* Check if this is the Control or Data interface.  We only need to dismount the link and abort the
       transfer once for the 2 classes.  */
    if (interface_ptr -> ux_slave_interface_descriptor.bInterfaceClass == UX_DEVICE_CLASS_RNDIS_CLASS_COMMUNICATION_CONTROL)
    {
    
        /* Declare the link to be down. That may need to change later to make it dependant on the
           WAN/Wireless modem.  */
        rndis -> ux_slave_class_rndis_link_state = UX_DEVICE_CLASS_RNDIS_LINK_STATE_DOWN;

        /* Terminate the transactions pending on the endpoints (interrupt, bulk in, bulk out).  */
        _ux_device_stack_transfer_all_request_abort(rndis -> ux_slave_class_rndis_interrupt_endpoint, UX_TRANSFER_BUS_RESET);
        _ux_device_stack_transfer_all_request_abort(rndis -> ux_slave_class_rndis_bulkin_endpoint, UX_TRANSFER_BUS_RESET);
        _ux_device_stack_transfer_all_request_abort(rndis -> ux_slave_class_rndis_bulkout_endpoint, UX_TRANSFER_BUS_RESET);

        /* We have 2 threads waiting for an event, interrupt and bulk in. We wake them up with 
           a DEVICE_STATE_CHANGE event. In turn they will release the NetX resources used and suspend.  */
        _ux_device_event_flags_set(&rndis -> ux_slave_class_rndis_event_flags_group, UX_DEVICE_CLASS_RNDIS_NEW_DEVICE_STATE_CHANGE_EVENT, UX_OR);                

        /* If there is a deactivate function call it.  */
        if (rndis -> ux_slave_class_rndis_parameter.ux_slave_class_rndis_instance_deactivate != UX_NULL)
        
            /* Invoke the application.  */
            rndis -> ux_slave_class_rndis_parameter.ux_slave_class_rndis_instance_deactivate(rndis);

        /* Deregister this interface to the NetX USB interface broker.  */
        _ux_network_driver_deactivate((VOID *) rndis, rndis -> ux_slave_class_rndis_network_handle);
        
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_RNDIS_DEACTIVATE, rndis, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_UNREGISTER(rndis);
    
    /* Return completion status.  */
    return(UX_SUCCESS);
}

