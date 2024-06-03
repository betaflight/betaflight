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
/*    _ux_device_class_cdc_ecm_bulkout_thread             PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the thread of the cdc_ecm bulk out endpoint. It    */ 
/*    is waiting for the host to send data on the bulk out endpoint to    */ 
/*    the device.                                                         */ 
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
/*    _ux_utility_memory_copy               Copy memory                   */
/*    nx_packet_allocate                    Allocate NetX packet          */
/*    nx_packet_release                     Free NetX packet              */
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
/*                                            prefixed UX to MS_TO_TICK,  */
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed EP not ready issue,   */
/*                                            used pool from NX IP inst,  */
/*                                            used NX API to copy data,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
VOID  _ux_device_class_cdc_ecm_bulkout_thread(ULONG cdc_ecm_class)
{

UX_SLAVE_CLASS                  *class_ptr;
UX_SLAVE_CLASS_CDC_ECM          *cdc_ecm;
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_TRANSFER               *transfer_request;
UINT                            status;
NX_PACKET                       *packet;
USB_NETWORK_DEVICE_TYPE         *ux_nx_device;

    /* Cast properly the cdc_ecm instance.  */
    UX_THREAD_EXTENSION_PTR_GET(class_ptr, UX_SLAVE_CLASS, cdc_ecm_class)
    
    /* Get the cdc_ecm instance from this class container.  */
    cdc_ecm =  (UX_SLAVE_CLASS_CDC_ECM *) class_ptr -> ux_slave_class_instance;
    
    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;
    
    /* This thread runs forever but can be suspended or resumed.  */
    while (1)
    {

        /* As long as the device is in the CONFIGURED state.  */
        while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED)
        { 

            /* Check if packet pool is ready.  */
            if (cdc_ecm -> ux_slave_class_cdc_ecm_packet_pool == UX_NULL)
            {

                /* Get the network device handle.  */
                ux_nx_device = (USB_NETWORK_DEVICE_TYPE *)(cdc_ecm -> ux_slave_class_cdc_ecm_network_handle);

                /* Get packet pool from IP instance (if available).  */
                if (ux_nx_device -> ux_network_device_ip_instance != UX_NULL)
                {
                    cdc_ecm -> ux_slave_class_cdc_ecm_packet_pool = ux_nx_device -> ux_network_device_ip_instance -> nx_ip_default_packet_pool;
                }
                else
                {

                    /* Error trap.  */
                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CLASS_ETH_PACKET_POOL_ERROR);

                    _ux_utility_delay_ms(UX_DEVICE_CLASS_CDC_ECM_PACKET_POOL_INST_WAIT);
                    continue;
                }
            }

            /* Check if Bulk OUT endpoint is ready.  */
            if (cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_endpoint == UX_NULL)
            {
                _ux_utility_delay_ms(UX_DEVICE_CLASS_CDC_ECM_LINK_CHECK_WAIT);
                continue;
            }

            /* We can accept new reception. Get a NX Packet */
            status =  nx_packet_allocate(cdc_ecm -> ux_slave_class_cdc_ecm_packet_pool, &packet, 
                                         NX_RECEIVE_PACKET, UX_MS_TO_TICK(UX_DEVICE_CLASS_CDC_ECM_PACKET_POOL_WAIT));

            if (status == NX_SUCCESS)
            {

                /* Select the transfer request associated with BULK OUT endpoint.   */
                transfer_request =  &cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_endpoint -> ux_slave_endpoint_transfer_request;

                /* And length.  */
                transfer_request -> ux_slave_transfer_request_requested_length =  UX_DEVICE_CLASS_CDC_ECM_MAX_PACKET_LENGTH;
                transfer_request -> ux_slave_transfer_request_actual_length =     0;
            
                /* Memorize this packet at the beginning of the queue.  */
                cdc_ecm -> ux_slave_class_cdc_ecm_receive_queue = packet;
            
                /* Reset the queue pointer of this packet.  */
                packet -> nx_packet_queue_next = UX_NULL;
            
                /* Send the request to the device controller.  */
                status =  _ux_device_stack_transfer_request(transfer_request, UX_DEVICE_CLASS_CDC_ECM_MAX_PACKET_LENGTH,
                                                                    UX_DEVICE_CLASS_CDC_ECM_MAX_PACKET_LENGTH);

                /* Check the completion code. */
                if (status == UX_SUCCESS)
                {

                    /* We only proceed with packets that are received OK, if error, ignore the packet. */
                    /* If trace is enabled, insert this event into the trace buffer.  */
                    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_CDC_ECM_PACKET_RECEIVE, cdc_ecm, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

                    /* Adjust the prepend pointer to take into account the non 3 bit alignment of the ethernet header.  */
                    packet -> nx_packet_prepend_ptr += sizeof(USHORT);
                    packet -> nx_packet_append_ptr += sizeof(USHORT);

                    /* Copy the received packet in the IP packet data area.  */
                    status = nx_packet_data_append(packet,
                            transfer_request -> ux_slave_transfer_request_data_pointer,
                            transfer_request -> ux_slave_transfer_request_actual_length,
                            cdc_ecm -> ux_slave_class_cdc_ecm_packet_pool,
                            UX_MS_TO_TICK(UX_DEVICE_CLASS_CDC_ECM_PACKET_POOL_WAIT));
                    if (status == NX_SUCCESS)
                    {

                        /* Send that packet to the NetX USB broker.  */
                        _ux_network_driver_packet_received(cdc_ecm -> ux_slave_class_cdc_ecm_network_handle, packet);
                    }
                    else
                    {

                        /* We received a malformed packet. Report to application.  */
                        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CLASS_MALFORMED_PACKET_RECEIVED_ERROR);
                        nx_packet_release(packet);
                    }
                }
                else

                    /* Free the packet that was not successfully received.  */
                    nx_packet_release(packet);
            }
            else
            {

                /* Packet allocation timed out. Note that the timeout value is
                   configurable.  */

                /* Error trap. No need for trace, since NetX does it.  */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);
            }
        }
             
        /* We need to suspend ourselves. We will be resumed by the device enumeration module.  */
        _ux_device_thread_suspend(&cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_thread);
    }
}
#endif
