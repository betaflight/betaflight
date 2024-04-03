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
/**   Device CDC_ECM Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_cdc_ecm.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_cdc_ecm_deactivate                 PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function deactivate an instance of the cdc_ecm class.          */ 
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
/*    CDC_ECM Class                                                       */
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
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_cdc_ecm_deactivate(UX_SLAVE_CLASS_COMMAND *command)
{
                                          
UX_SLAVE_CLASS_CDC_ECM      *cdc_ecm;
UX_SLAVE_INTERFACE          *interface_ptr;            
UX_SLAVE_CLASS              *class_ptr;

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    cdc_ecm = (UX_SLAVE_CLASS_CDC_ECM *) class_ptr -> ux_slave_class_instance;

    /* Get the interface that owns this instance.  Normally the interface can be derived
       from the class instance but since CDC_ECM has 2 interfaces and we only store the Control
       interface in the class container, we used the class_command pointer to retrieve the
       correct interface which issued the deactivation. */
    interface_ptr =  (UX_SLAVE_INTERFACE  *) command -> ux_slave_class_command_interface;
    
    /* Check if this is the Control or Data interface.  We only need to dismount the link and abort the
       transfer once for the 2 classes.  */
    if (interface_ptr -> ux_slave_interface_descriptor.bInterfaceClass == UX_DEVICE_CLASS_CDC_ECM_CLASS_COMMUNICATION_CONTROL)
    {

        /* Is the link state up?  */
        if (cdc_ecm -> ux_slave_class_cdc_ecm_link_state == UX_DEVICE_CLASS_CDC_ECM_LINK_STATE_UP)
        {

            /* Then we've found the bulk endpoints and started the threads.  */

            /* Abort transfers. Note that since the bulk out thread is most likely waiting for 
               a transfer from the host, this will allow it to resume and suspend itself.  */
            _ux_device_stack_transfer_all_request_abort(cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_endpoint, UX_TRANSFER_BUS_RESET);
            _ux_device_stack_transfer_all_request_abort(cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_endpoint, UX_TRANSFER_BUS_RESET);

            /* Declare the link to be down. That may need to change later to make it dependent on the
               WAN/Wireless modem.  */
            cdc_ecm -> ux_slave_class_cdc_ecm_link_state = UX_DEVICE_CLASS_CDC_ECM_LINK_STATE_DOWN;

            /* Is there an interrupt endpoint?  */
            if (cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_endpoint != UX_NULL)

                /* Abort the transfers on the interrupt endpoint as well.  */
                _ux_device_stack_transfer_all_request_abort(cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_endpoint, UX_TRANSFER_BUS_RESET);

            /* Wake up the bulk in thread so it will release the NetX resources used and suspend.  */
            _ux_device_event_flags_set(&cdc_ecm -> ux_slave_class_cdc_ecm_event_flags_group, UX_DEVICE_CLASS_CDC_ECM_NEW_DEVICE_STATE_CHANGE_EVENT, UX_OR);                

            /* If there is a deactivate function call it.  */
            if (cdc_ecm -> ux_slave_class_cdc_ecm_parameter.ux_slave_class_cdc_ecm_instance_deactivate != UX_NULL)

                /* Invoke the application.  */
                cdc_ecm -> ux_slave_class_cdc_ecm_parameter.ux_slave_class_cdc_ecm_instance_deactivate(cdc_ecm);

            /* Deregister this interface to the NetX USB interface broker.  */
            _ux_network_driver_deactivate((VOID *) cdc_ecm, cdc_ecm -> ux_slave_class_cdc_ecm_network_handle);
        }
        else
        {

            /* The link state is down.  */

            /* Did activation succeed?  */
            if (cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_endpoint != UX_NULL && cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_endpoint != UX_NULL)
            {

                /* The only thing we need to do is deregister this interface to the NetX USB interface broker.  */
                _ux_network_driver_deactivate((VOID *) cdc_ecm, cdc_ecm -> ux_slave_class_cdc_ecm_network_handle);
            }
            else
            {

                /* Activation did not succeed. Nothing to do.  */
            }
        }
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_CDC_ECM_DEACTIVATE, cdc_ecm, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_UNREGISTER(cdc_ecm);

    /* Return completion status.  */
    return(UX_SUCCESS);
}

