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
/**   ASIX Class                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_asix.h"
#include "ux_host_stack.h"


#if !defined(UX_HOST_STANDALONE)

static inline UINT _ux_host_class_asix_link_up_controls(UX_HOST_CLASS_ASIX  *asix);

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_asix_thread                          PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This is the Asix thread that monitors the link change flag.         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    asix                                   Asix instance                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request        Transfer request             */
/*    _ux_host_semaphore_get                 Get semaphore                */
/*    _ux_host_semaphore_put                 Put semaphore                */
/*    _ux_utility_memory_allocate            Allocate memory              */
/*    _ux_utility_memory_free                Free memory                  */
/*    _ux_utility_memory_set                 Set memory                   */
/*    _ux_network_driver_activate            Activate NetX USB interface  */
/*    _ux_network_driver_link_down           Set state to link down       */
/*    _ux_network_driver_link_up             Set state to link up         */
/*    nx_packet_allocate                     Allocate NetX packet         */
/*    nx_packet_transmit_release             Release NetX packet          */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Asix class initialization                                           */ 
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
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            removed internal NX pool,   */
/*                                            moved NX driver activate,   */
/*                                            refined control REQ flow,   */
/*                                            refined reception flow,     */
/*                                            refined interrupt flow,     */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_asix_thread(ULONG parameter)
{

UX_HOST_CLASS_ASIX          *asix;
UX_TRANSFER                 *transfer_request;
NX_PACKET                   *packet;
NX_PACKET                   *current_packet;
NX_PACKET                   *next_packet;
UCHAR                       *buffer;
ULONG                       buffer_count;
ULONG                       asix_length;
ULONG                       asix_count;
ULONG                       asix_remain;
ULONG                       packet_discard;
ULONG                       buffer_remain;
ULONG                       copy_length;
UINT                        status;
UX_DEVICE                   *device;
ULONG                       available0, available1;
USB_NETWORK_DEVICE_TYPE     *ux_nx_device;


    /* Cast the parameter passed in the thread into the asix pointer.  */
    UX_THREAD_EXTENSION_PTR_GET(asix, UX_HOST_CLASS_ASIX, parameter)

    /* Loop forever waiting for changes signaled through the semaphore. */
    while (1)
    {

        /* Wait for the semaphore to be put by the asix interrupt event.  */
        status = _ux_host_semaphore_get(&asix -> ux_host_class_asix_interrupt_notification_semaphore, UX_WAIT_FOREVER);

        /* Check for successful completion.  */
        if (status != UX_SUCCESS)
            return;

        /* Protect Thread reentry to this instance.  */
        status =  _ux_host_semaphore_get(&asix -> ux_host_class_asix_semaphore, UX_WAIT_FOREVER);

        /* Check for successful completion.  */
        if (status != UX_SUCCESS)
            return;
    
        /* Check the link state. It is either pending up or down.  */
        if (asix -> ux_host_class_asix_link_state == UX_HOST_CLASS_ASIX_LINK_STATE_PENDING_UP)
        {

            /* Issue a list of control requests to setup link up.  */
            status = _ux_host_class_asix_link_up_controls(asix);
            if (status != UX_SUCCESS)
            {

                /* Unprotect thread reentry to this instance.  */
                _ux_host_semaphore_put(&asix -> ux_host_class_asix_semaphore);
                continue;
            }

            /* Now the link is up.  */
            asix -> ux_host_class_asix_link_state = UX_HOST_CLASS_ASIX_LINK_STATE_UP;

            /* Communicate the state with the network driver.  */
            _ux_network_driver_link_up(asix -> ux_host_class_asix_network_handle);

            /* Reactivate interrupt notification polling.  */
            _ux_host_stack_transfer_request(&asix -> ux_host_class_asix_interrupt_endpoint -> ux_endpoint_transfer_request);

            /* Unprotect thread reentry to this instance.  */
            _ux_host_semaphore_put(&asix -> ux_host_class_asix_semaphore);

            /* Re-check packet pool on link up.  */
            asix -> ux_host_class_asix_packet_pool = UX_NULL;

            /* Keep polling if device is connected and link is up.  */

            /* Initialize variables.  */
            device = asix -> ux_host_class_asix_device;
            packet = UX_NULL;
            buffer = UX_NULL;
            asix_length = 0;
            asix_count = 0;
            buffer_count = 0;
            packet_discard = UX_FALSE;
            asix -> ux_host_class_asix_packet_available_min = 0xFFFFFFFF;

            /* Polling loop.  */
            while(device -> ux_device_state == UX_DEVICE_CONFIGURED &&
                asix -> ux_host_class_asix_link_state == UX_HOST_CLASS_ASIX_LINK_STATE_UP)
            {

                /* Check if packet pool is ready.  */
                if (asix -> ux_host_class_asix_packet_pool == UX_NULL)
                {

                    /* Get the network device handle.  */
                    ux_nx_device = (USB_NETWORK_DEVICE_TYPE *)(asix -> ux_host_class_asix_network_handle);

                    /* Get packet pool from IP instance (if available).  */
                    if (ux_nx_device -> ux_network_device_ip_instance != UX_NULL)
                    {
                        asix -> ux_host_class_asix_packet_pool = ux_nx_device -> ux_network_device_ip_instance -> nx_ip_default_packet_pool;
                    }
                    else
                    {

                        /* Error trap.  */
                        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CLASS_ETH_PACKET_POOL_ERROR);

                        _ux_utility_delay_ms(UX_HOST_CLASS_ASIX_PACKET_POOL_WAIT);
                        continue;
                    }
                }

                /* Get transfer request for bulk in reception. */
                transfer_request =  &asix -> ux_host_class_asix_bulk_in_endpoint -> ux_endpoint_transfer_request;

                /* If there is no buffer ready, read it.  */
                if (buffer == UX_NULL)
                {

                    /* Set the data pointer.  */
                    transfer_request -> ux_transfer_request_data_pointer = asix -> ux_host_class_asix_receive_buffer;

                    /* And length.  */
                    transfer_request -> ux_transfer_request_requested_length =  UX_HOST_CLASS_ASIX_RECEIVE_BUFFER_SIZE;
                    transfer_request -> ux_transfer_request_actual_length =     0;

                    /* Schedule a reception.  */
                    status = _ux_host_stack_transfer_request(transfer_request);

                    /* Check if it's started successfully.  */
                    if (status != UX_SUCCESS)
                    {

                        /* If a packet (chain) is on-going, release it.  */
                        if (packet != UX_NULL)
                        {
                            nx_packet_release(packet);
                            packet = UX_NULL;
                        }
                        continue;
                    }

                    /* Wait for transfer completion.  */
                    _ux_host_semaphore_get_norc(&transfer_request -> ux_transfer_request_semaphore, UX_WAIT_FOREVER);

                    /* Check completion code.  */
                    if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
                    {

                        /* Abort transfer.  */
                        _ux_host_stack_transfer_request_abort(transfer_request);

                        /* If a packet (chain) is on-going, release it.  */
                        if (packet != UX_NULL)
                        {
                            nx_packet_release(packet);
                            packet = UX_NULL;
                        }
                        continue;
                    }

                    /* If it's ZLP, reset buffer and ASIX packet and transfer again.  */
                    if (transfer_request -> ux_transfer_request_actual_length == 0)
                    {

                        /* Packet should have been processed!  */
                        if (packet != UX_NULL)
                        {

                            /* ASIX header corrupt?  */
                            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);
                            nx_packet_release(packet);
                            packet = UX_NULL;
                        }

                        buffer = UX_NULL;
                        buffer_count = 0;
                        asix_length = 0;
                        continue;
                    }

                    /* Buffer is ready.  */
                    buffer = asix -> ux_host_class_asix_receive_buffer;
                }

                /* There is data in buffer, extract the data to packets.  */

                /* If there is no packet, allocate it.  */
                if (packet == UX_NULL)
                {

                    /* Get a free NX Packet.  */
                    available0 = asix -> ux_host_class_asix_packet_pool -> nx_packet_pool_available;
                    status = nx_packet_allocate(asix -> ux_host_class_asix_packet_pool,
                                &packet, NX_RECEIVE_PACKET,
                                UX_MS_TO_TICK(UX_HOST_CLASS_ASIX_PACKET_ALLOCATE_WAIT));
                    available1 = asix -> ux_host_class_asix_packet_pool -> nx_packet_pool_available;
                    if (asix -> ux_host_class_asix_packet_available_min > available1)
                        asix -> ux_host_class_asix_packet_available_min = available1;
                    if (asix -> ux_host_class_asix_packet_available_min > available0)
                        asix -> ux_host_class_asix_packet_available_min = available0;
                    if (status != NX_SUCCESS)
                    {

                        /* Error trap.  */
                        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CLASS_ETH_PACKET_ERROR);
                        continue;
                    }

                    /* Adjust the prepend pointer to take into account the non 3 bit alignment of the ethernet header.  */
                    packet -> nx_packet_prepend_ptr += sizeof(USHORT);
                    packet -> nx_packet_append_ptr = packet -> nx_packet_prepend_ptr;

                    /* Log the packet.  */

                }

                /* Get remaining in buffer.  */
                buffer_remain = transfer_request -> ux_transfer_request_actual_length -
                                buffer_count;

                /* For new packet, get packet length from ASIX header.  */
                if (asix_length == 0)
                {

                    /* If remaining less than 2 bytes in buffer, read buffer again.  */
                    if (buffer_remain < 2)
                    {
                        buffer = UX_NULL;
                        buffer_count = 0;
                        continue;
                    }

                    /* Get ASIX packet length.  */
                    asix_length = _ux_utility_short_get(buffer + buffer_count);
                    asix_length &= UX_HOST_CLASS_ASIX_RX_PACKET_LENGTH_MASK;
                    asix_count = 0;

                    /* If remaining 2-3 bytes, skip 2 of next buffer.  */
                    if (buffer_remain < 4)
                    {
                        buffer = UX_NULL;
                        buffer_count = 2;
                        continue;
                    }

                    /* Skip header in buffer, it's not copied to NX packet.  */
                    buffer_count += 4;
                    buffer_remain -= 4;
                }

                /* Get remaining in ASIX packet.  */
                asix_remain = asix_length - asix_count;

                /* Get actual length to process.  */
                if (asix_remain <= buffer_remain)
                    copy_length = asix_remain;
                else
                    copy_length = buffer_remain;

                /* Check if the data is discarded.  */
                if (packet_discard == UX_FALSE && copy_length)
                {

                    /* Append data to packet.  */
                    status = nx_packet_data_append(packet,
                                        buffer + buffer_count, copy_length,
                                        asix -> ux_host_class_asix_packet_pool,
                                        UX_MS_TO_TICK(UX_HOST_CLASS_ASIX_PACKET_ALLOCATE_WAIT));
                    if (status != NX_SUCCESS)
                    {

                        /* There is error, this ASIX packet is discarded.  */
                        packet_discard = UX_TRUE;
                    }
                }

                /* Update counts.  */
                buffer_count += copy_length;
                asix_count += copy_length;

                /* If packet size is odd, padding one byte is ignored.  */
                if (asix_length & 0x1u)
                    buffer_count ++;

                /* Check if buffer ends.  */
                if (buffer_count >= transfer_request -> ux_transfer_request_actual_length)
                {
                    buffer = UX_NULL;
                    buffer_count = 0;
                }

                /* Check if ASIX packet ends.
                 * - ASIX count achieve its length.
                 * - Buffer achieve its end and it's a short packet.
                 */
                if (asix_count >= asix_length ||
                    (buffer == UX_NULL &&
                     transfer_request -> ux_transfer_request_actual_length <
                     transfer_request -> ux_transfer_request_requested_length))
                {

                    /* Check if packet is discarded.  */
                    if (packet_discard)
                    {

                        packet_discard = UX_FALSE;

                        /* Re-allocate packet.  */
                        nx_packet_release(packet);
                        packet = UX_NULL;
                        continue;
                    }

                    /* Send that packet to the NetX USB broker.  */
                    _ux_network_driver_packet_received(asix -> ux_host_class_asix_network_handle, packet);
                    packet = UX_NULL;

                    /* Reset ASIX packet.  */
                    asix_length = 0;

                    continue;
                }

                /* Buffer ends but packet continues.  */
                /* Buffer already reset to start reception again.  */
            }

            /* Release packet not sent to NX.  */
            if (packet)
                nx_packet_release(packet);
        }
        else
        {

            /* Abort pending transfers.  */
            _ux_host_stack_endpoint_transfer_abort(asix -> ux_host_class_asix_bulk_out_endpoint);

            /* We need to free the packets that will not be sent.  */
            current_packet =  asix -> ux_host_class_asix_xmit_queue;

            /* Get the next packet associated with the first packet.  */
            next_packet = current_packet -> nx_packet_queue_next;

            /* Parse all these packets that were scheduled.  */
            while (current_packet != UX_NULL)
            {

                /* Free the packet that was just sent.  First do some housekeeping.  */
                current_packet -> nx_packet_prepend_ptr =  current_packet -> nx_packet_prepend_ptr + UX_HOST_CLASS_ASIX_ETHERNET_SIZE;
                current_packet -> nx_packet_length =  current_packet -> nx_packet_length - UX_HOST_CLASS_ASIX_ETHERNET_SIZE;

                /* And ask Netx to release it.  */
                nx_packet_transmit_release(current_packet);

                /* Next packet becomes the current one.  */
                current_packet = next_packet;

                /* Next packet now.  */
                if (current_packet != UX_NULL)

                    /* Get the next packet associated with the first packet.  */
                    next_packet = current_packet -> nx_packet_queue_next;
            
            }

            /* Communicate the state with the network driver.  */
            _ux_network_driver_link_down(asix -> ux_host_class_asix_network_handle);

            /* Now the link is down.  */
            asix -> ux_host_class_asix_link_state = UX_HOST_CLASS_ASIX_LINK_STATE_DOWN;

            /* Reactivate interrupt notification polling.  */
            _ux_host_stack_transfer_request(&asix -> ux_host_class_asix_interrupt_endpoint -> ux_endpoint_transfer_request);

            /* Unprotect thread reentry to this instance.  */
            _ux_host_semaphore_put(&asix -> ux_host_class_asix_semaphore);
        }
    }    
}

static inline UINT _ux_host_class_asix_link_up_controls(UX_HOST_CLASS_ASIX  *asix)
{

UCHAR                       *setup_buffer;
UX_ENDPOINT                 *control_endpoint;
UX_TRANSFER                 *transfer_request;
UINT                        status;


    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &asix -> ux_host_class_asix_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Need to allocate memory for the buffer.  */
    setup_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_ASIX_SETUP_BUFFER_SIZE);
    if (setup_buffer == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Request ownership of Serial Management Interface.  */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_OWN_SMI ;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  0;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);
        return(UX_TRANSFER_ERROR);
    }

    /* Get the value of the PHYIDR1 in the PHY register.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  2;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_READ_PHY_REG;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_IN | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  asix -> ux_host_class_asix_primary_phy_id;
    transfer_request -> ux_transfer_request_index               =  UX_HOST_CLASS_ASIX_PHY_REG_ANLPAR;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS ||
        transfer_request -> ux_transfer_request_actual_length != 2)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);
        return(UX_TRANSFER_ERROR);
    }

    /* Release SMI ownership. */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_RELEASE_SMI;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  0;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);
        return(UX_TRANSFER_ERROR);
    }

    /* Check speed.  */
    if (asix -> ux_host_class_asix_speed_selected == UX_HOST_CLASS_ASIX_SPEED_SELECTED_100MPBS)

        /* Set speed at 100MBPS.  */
        transfer_request -> ux_transfer_request_value =  UX_HOST_CLASS_ASIX_MEDIUM_PS;

    /* Write the value of the Medium Mode. */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_WRITE_MEDIUM_MODE;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               |=  (UX_HOST_CLASS_ASIX_MEDIUM_FD | UX_HOST_CLASS_ASIX_MEDIUM_BIT2 | UX_HOST_CLASS_ASIX_MEDIUM_RFC_ENABLED |
                                                                    UX_HOST_CLASS_ASIX_MEDIUM_TFC_ENABLED | UX_HOST_CLASS_ASIX_MEDIUM_RE_ENABLED);
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);
        return(UX_TRANSFER_ERROR);
    }

    /* Set the Rx Control register value.  */
    transfer_request -> ux_transfer_request_data_pointer        =  UX_NULL;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_WRITE_RX_CTL;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  (UX_HOST_CLASS_ASIX_RXCR_AM | UX_HOST_CLASS_ASIX_RXCR_AB |
                                                                    UX_HOST_CLASS_ASIX_RXCR_SO | UX_HOST_CLASS_ASIX_RXCR_MFB_2048);
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);
        return(UX_TRANSFER_ERROR);
    }

    /* Set the multicast value.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  8;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_ASIX_REQ_WRITE_MULTICAST_FILTER;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  0;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Fill in the multicast filter.  */
    _ux_utility_memory_set(setup_buffer, 0, 8); /* Use case of memset is verified. */
    *(setup_buffer +1) = 0x40;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS ||
        transfer_request -> ux_transfer_request_actual_length != 8)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);
        return(UX_TRANSFER_ERROR);
    }

    /* Free all used resources.  */
    _ux_utility_memory_free(setup_buffer);
    return(UX_SUCCESS);
}
#endif
