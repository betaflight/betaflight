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
/*    _ux_device_class_cdc_ecm_bulkin_thread              PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the thread of the cdc_ecm bulkin endpoint. The bulk*/ 
/*    IN endpoint is used when the device wants to write data to be sent  */ 
/*    to the host.                                                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    cdc_ecm_class                             Address of cdc_ecm class  */ 
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
/*    _ux_device_mutex_on                   Take mutex                    */
/*    _ux_device_mutex_off                  Free mutex                    */
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
/*                                            verified memset and memcpy  */
/*                                            cases, used UX prefix to    */
/*                                            refer to TX symbols instead */
/*                                            of using them directly,     */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used NX API to copy data,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
VOID  _ux_device_class_cdc_ecm_bulkin_thread(ULONG cdc_ecm_class)
{

UX_SLAVE_CLASS                  *class_ptr;
UX_SLAVE_CLASS_CDC_ECM          *cdc_ecm;
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_TRANSFER               *transfer_request;
UINT                            status;
ULONG                           actual_flags;
NX_PACKET                       *current_packet;
ULONG                           transfer_length;
ULONG                           copied;

    /* Cast properly the cdc_ecm instance.  */
    UX_THREAD_EXTENSION_PTR_GET(class_ptr, UX_SLAVE_CLASS, cdc_ecm_class)
    
    /* Get the cdc_ecm instance from this class container.  */
    cdc_ecm =  (UX_SLAVE_CLASS_CDC_ECM *) class_ptr -> ux_slave_class_instance;
    
    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;
    
    /* This thread runs forever but can be suspended or resumed.  */
    while (1)
    {

        /* For as long we are configured.  */
        while (1)
        {
            
            /* Wait until either a new packet has been added to the xmit queue,
               or until there has been a change in the device state (i.e. disconnection).  */
            _ux_utility_event_flags_get(&cdc_ecm -> ux_slave_class_cdc_ecm_event_flags_group, (UX_DEVICE_CLASS_CDC_ECM_NEW_BULKIN_EVENT |
                                                                                               UX_DEVICE_CLASS_CDC_ECM_NEW_DEVICE_STATE_CHANGE_EVENT), 
                                                                                              UX_OR_CLEAR, &actual_flags, UX_WAIT_FOREVER);

            /* Check the completion code and the actual flags returned.  */
            if ((actual_flags & UX_DEVICE_CLASS_CDC_ECM_NEW_DEVICE_STATE_CHANGE_EVENT) == 0)
            {

                /* Get the transfer request for the bulk IN pipe.  */
                transfer_request =  &cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_endpoint -> ux_slave_endpoint_transfer_request;
    
                /* Parse all packets.  */
                while (cdc_ecm -> ux_slave_class_cdc_ecm_xmit_queue != UX_NULL)
                {

                    /* Ensure no other threads are modifying the xmit queue.  */
                    _ux_device_mutex_on(&cdc_ecm -> ux_slave_class_cdc_ecm_mutex);

                    /* Get the current packet in the list.  */
                    current_packet =  cdc_ecm -> ux_slave_class_cdc_ecm_xmit_queue;

                    /* Set the next packet (or a NULL value) as the head of the xmit queue. */
                    cdc_ecm -> ux_slave_class_cdc_ecm_xmit_queue =  current_packet -> nx_packet_queue_next;
                
                    /* Free Mutex resource.  */
                    _ux_device_mutex_off(&cdc_ecm -> ux_slave_class_cdc_ecm_mutex);
                    
                    /* If the link is down no need to rearm a packet. */
                    if (cdc_ecm -> ux_slave_class_cdc_ecm_link_state == UX_DEVICE_CLASS_CDC_ECM_LINK_STATE_UP)
                    {
                
                        /* Can the packet fit in the transfer requests data buffer?  */
                        if (current_packet -> nx_packet_length <= UX_SLAVE_REQUEST_DATA_MAX_LENGTH)
                        {

                            /* Copy the packet in the transfer descriptor buffer.  */
                            status = nx_packet_data_extract_offset(current_packet, 0,
                                    transfer_request -> ux_slave_transfer_request_data_pointer,
                                    current_packet -> nx_packet_length, &copied);
                            if (status == UX_SUCCESS)
                            {

                                /* Calculate the transfer length.  */
                                transfer_length =  current_packet -> nx_packet_length;
                                
                                /* If trace is enabled, insert this event into the trace buffer.  */
                                UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_CDC_ECM_PACKET_TRANSMIT, cdc_ecm, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

                                /* Send the request to the device controller.  */
                                status =  _ux_device_stack_transfer_request(transfer_request, transfer_length, UX_DEVICE_CLASS_CDC_ECM_ETHERNET_PACKET_SIZE + 1);
                            }

                            /* Check error code. */
                            if (status != UX_SUCCESS)
                            {

                                /* Is this not a transfer abort? (this is expected to happen)  */
                                if (status != UX_TRANSFER_BUS_RESET)
                                {

                                    /* Error trap. */
                                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);
                                }
                            }
                        }
                        else
                        {

                            /* Packet is too large.  */

                            /* Report error to application.  */
                            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_BUFFER_OVERFLOW);
                        }
                    }

                    /* Free the packet that was just sent.  First do some housekeeping.  */
                    current_packet -> nx_packet_prepend_ptr =  current_packet -> nx_packet_prepend_ptr + UX_DEVICE_CLASS_CDC_ECM_ETHERNET_SIZE; 
                    current_packet -> nx_packet_length =  current_packet -> nx_packet_length - UX_DEVICE_CLASS_CDC_ECM_ETHERNET_SIZE;
                
                    /* And ask Netx to release it.  */
                    nx_packet_transmit_release(current_packet); 
                }
            }
            else
            {

                /* We need to ensure nobody is adding to the queue, so get the mutex protection. */
                _ux_device_mutex_on(&cdc_ecm -> ux_slave_class_cdc_ecm_mutex);

                /* Since we got the mutex, we know no one is trying to modify the queue; we also know
                   no one can start modifying the queue since the link state is down, so we can just 
                   release the mutex.  */
                _ux_device_mutex_off(&cdc_ecm -> ux_slave_class_cdc_ecm_mutex);

                /* We get here when the link is down. All packets pending must be freed.  */
                while (cdc_ecm -> ux_slave_class_cdc_ecm_xmit_queue != UX_NULL)
                {

                    /* Get the current packet in the list.  */
                    current_packet =  cdc_ecm -> ux_slave_class_cdc_ecm_xmit_queue;
                    
                    /* Set the next packet (or a NULL value) as the head of the xmit queue. */
                    cdc_ecm -> ux_slave_class_cdc_ecm_xmit_queue =  current_packet -> nx_packet_queue_next;

                    /* Free the packet.  */
                    current_packet -> nx_packet_prepend_ptr =  current_packet -> nx_packet_prepend_ptr + UX_DEVICE_CLASS_CDC_ECM_ETHERNET_SIZE; 
                    current_packet -> nx_packet_length =  current_packet -> nx_packet_length - UX_DEVICE_CLASS_CDC_ECM_ETHERNET_SIZE;

                    /* And ask Netx to release it.  */
                    nx_packet_transmit_release(current_packet); 
                }

                /* Was the change in the device state caused by a disconnection?  */
                if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
                {

                    /* Yes. Break out of the loop and suspend ourselves, waiting for the next configuration.  */
                    break;
                }
            }
        }

        /* We need to suspend ourselves. We will be resumed by the device enumeration module or when a change of alternate setting happens.  */
        _ux_device_thread_suspend(&cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_thread);
    }
}
#endif
