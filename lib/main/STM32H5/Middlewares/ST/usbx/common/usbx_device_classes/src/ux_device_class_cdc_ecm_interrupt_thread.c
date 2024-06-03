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


#if !defined(UX_DEVICE_STANDALONE)
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_cdc_ecm_interrupt_thread           PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the thread of the cdc_ecm interrupt endpoint       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    cdc_ecm_class                         Address of cdc_ecm class      */ 
/*                                          container                     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_request     Request transfer              */ 
/*    _ux_utility_event_flags_get           Get event flags               */
/*    _ux_utility_short_put                 Put 16-bit value to buffer    */
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
/*                                                                        */
/**************************************************************************/
VOID  _ux_device_class_cdc_ecm_interrupt_thread(ULONG cdc_ecm_class)
{

UX_SLAVE_CLASS                  *class_ptr;
UX_SLAVE_CLASS_CDC_ECM          *cdc_ecm;
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_TRANSFER               *transfer_request;
UINT                            status;
ULONG                           actual_flags;
UCHAR                           *notification_buffer;

    /* Cast properly the cdc_ecm instance.  */
    UX_THREAD_EXTENSION_PTR_GET(class_ptr, UX_SLAVE_CLASS, cdc_ecm_class)
    
    /* Get the cdc_ecm instance from this class container.  */
    cdc_ecm =  (UX_SLAVE_CLASS_CDC_ECM *) class_ptr -> ux_slave_class_instance;
    
    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;
    
    /* This thread runs forever but can be suspended or resumed.  */
    while(1)
    {

        /* All CDC_ECM events are on the interrupt endpoint IN, from the host.  */
        transfer_request =  &cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_endpoint -> ux_slave_endpoint_transfer_request;

        /* As long as the device is in the CONFIGURED state.  */
        while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED)
        { 


            /* Wait until we have a event sent by the application. We do not treat yet the case where a timeout based
               on the interrupt pipe frequency or a change in the idle state forces us to send an empty report.  */
            _ux_utility_event_flags_get(&cdc_ecm -> ux_slave_class_cdc_ecm_event_flags_group, 
                                        UX_DEVICE_CLASS_CDC_ECM_NETWORK_NOTIFICATION_EVENT, 
                                        UX_OR_CLEAR, &actual_flags, UX_WAIT_FOREVER);

            /* Build the Network Notification response.  */
            notification_buffer = transfer_request -> ux_slave_transfer_request_data_pointer;

            /* Set the request type.  */
            *(notification_buffer + UX_SETUP_REQUEST_TYPE) = UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;

            /* Set the request itself.  */
            *(notification_buffer + UX_SETUP_REQUEST) = 0;
            
            /* Set the value. It is the network link.  */
            _ux_utility_short_put(notification_buffer + UX_SETUP_VALUE, (USHORT)(cdc_ecm -> ux_slave_class_cdc_ecm_link_state));

            /* Set the Index. It is interface.  The interface used is the DATA interface. Here we simply take the interface number of the CONTROL and add 1 to it
                as it is assumed the classes are contiguous in number. */
            _ux_utility_short_put(notification_buffer + UX_SETUP_INDEX, (USHORT)(cdc_ecm -> ux_slave_class_cdc_ecm_interface -> ux_slave_interface_descriptor.bInterfaceNumber + 1));

            /* And the length is zero.  */
            *(notification_buffer + UX_SETUP_LENGTH) = 0;

            /* Send the request to the device controller.  */
            status =  _ux_device_stack_transfer_request(transfer_request, UX_DEVICE_CLASS_CDC_ECM_INTERRUPT_RESPONSE_LENGTH,
                                                                UX_DEVICE_CLASS_CDC_ECM_INTERRUPT_RESPONSE_LENGTH);

            /* Check error code.  */
            if (status != UX_SUCCESS)
            {

                /* Since bus resets are expected, we do not treat it as an error.  */
                if (status != UX_TRANSFER_BUS_RESET)
                {

                    /* Error trap. */
                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);
                }
            }
        }

        /* We need to suspend ourselves. We will be resumed by the device enumeration module.  */
        _ux_device_thread_suspend(&cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_thread);
    }
}
#endif
