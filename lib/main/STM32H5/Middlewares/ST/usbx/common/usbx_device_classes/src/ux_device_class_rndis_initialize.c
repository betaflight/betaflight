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
/**   Device RNDIS Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_rndis.h"
#include "ux_device_stack.h"

UX_DEVICE_CLASS_RNDIS_NX_ETHERNET_POOL_ALLOCSIZE_ASSERT

/* Define list of supported OIDs (ends with zero-terminator) */
ULONG ux_device_class_rndis_oid_supported_list[UX_DEVICE_CLASS_RNDIS_OID_SUPPORTED_LIST_LENGTH + 1] =
{
    /* Mandatory general OIDs. */
    UX_DEVICE_CLASS_RNDIS_OID_GEN_SUPPORTED_LIST,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_HARDWARE_STATUS,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_MEDIA_SUPPORTED,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_MEDIA_IN_USE,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_MAXIMUM_FRAME_SIZE,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_LINK_SPEED,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_TRANSMIT_BLOCK_SIZE,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_RECEIVE_BLOCK_SIZE,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_VENDOR_ID,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_VENDOR_DESCRIPTION,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_VENDOR_DRIVER_VERSION,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_CURRENT_PACKET_FILTER,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_MAXIMUM_TOTAL_SIZE,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_MAC_OPTIONS,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_MEDIA_CONNECT_STATUS,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_PHYSICAL_MEDIUM,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_RNDIS_CONFIG_PARAMETER,

    /* Mandatory statistical OIDs. */
    UX_DEVICE_CLASS_RNDIS_OID_GEN_XMIT_OK,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_RCV_OK,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_XMIT_ERROR,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_RCV_ERROR,
    UX_DEVICE_CLASS_RNDIS_OID_GEN_RCV_NO_BUFFER,

    /* Mandatory 802.3 OIDs.  */
    UX_DEVICE_CLASS_RNDIS_OID_802_3_PERMANENT_ADDRESS,
    UX_DEVICE_CLASS_RNDIS_OID_802_3_CURRENT_ADDRESS,
    UX_DEVICE_CLASS_RNDIS_OID_802_3_MULTICAST_LIST,
    UX_DEVICE_CLASS_RNDIS_OID_802_3_MAC_OPTIONS,
    UX_DEVICE_CLASS_RNDIS_OID_802_3_MAXIMUM_LIST_SIZE,

    /* Mandatory 802.3 statistical OIDs. */
    UX_DEVICE_CLASS_RNDIS_OID_802_3_RCV_ERROR_ALIGNMENT,
    UX_DEVICE_CLASS_RNDIS_OID_802_3_XMIT_ONE_COLLISION,
    UX_DEVICE_CLASS_RNDIS_OID_802_3_XMIT_MORE_COLLISIONS,

    0,
};

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_rndis_initialize                   PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function initializes the USB RNDIS device.                     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Pointer to rndis command      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_allocate           Allocate memory               */ 
/*    _ux_utility_memory_copy               Copy memory                   */
/*    _ux_utility_memory_free               Free memory                   */
/*    _ux_utility_event_flags_create        Create Flag group             */ 
/*    _ux_utility_event_flags_delete        Delete Flag group             */ 
/*    _ux_utility_mutex_create              Create mutex                  */
/*    _ux_device_mutex_delete               Delete mutex                  */
/*    _ux_device_semaphore_create           Create semaphore              */
/*    _ux_device_semaphore_delete           Delete semaphore              */
/*    _ux_device_thread_create              Create thread                 */
/*    _ux_device_thread_delete              Delete thread                 */
/*    nx_packet_pool_create                 Create NetX packet pool       */
/*    nx_packet_pool_delete                 Delete NetX packet pool       */
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
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            removed internal NX pool,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_rndis_initialize(UX_SLAVE_CLASS_COMMAND *command)
{
#if defined(UX_DEVICE_STANDALONE)
    UX_PARAMETER_NOT_USED(command);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_SLAVE_CLASS_RNDIS                        *rndis;
UX_SLAVE_CLASS_RNDIS_PARAMETER              *rndis_parameter;
UX_SLAVE_CLASS                              *class_ptr;
UINT                                        status;

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Create an instance of the device rndis class.  */
    rndis =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_SLAVE_CLASS_RNDIS));

    /* Check for successful allocation.  */
    if (rndis == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Save the address of the RNDIS instance inside the RNDIS container.  */
    class_ptr -> ux_slave_class_instance = (VOID *) rndis;

    /* Get the pointer to the application parameters for the rndis class.  */
    rndis_parameter =  command -> ux_slave_class_command_parameter;

    /* Store the start and stop signals if needed by the application.  */
    rndis -> ux_slave_class_rndis_parameter.ux_slave_class_rndis_instance_activate = rndis_parameter -> ux_slave_class_rndis_instance_activate;
    rndis -> ux_slave_class_rndis_parameter.ux_slave_class_rndis_instance_deactivate = rndis_parameter -> ux_slave_class_rndis_instance_deactivate;
    
    /* Save the Netx IP passed by the application.  */
    rndis -> ux_slave_class_rndis_nx_ip = rndis_parameter -> ux_slave_class_rndis_parameter_nx_ip; 
    
    /* Save the Netx IP address passed by the application.  */
    rndis -> ux_slave_class_rndis_nx_ip_address = rndis_parameter -> ux_slave_class_rndis_parameter_nx_ip_address; 
    
    /* Save the Netx IP address network mask passed by the application.  */
    rndis -> ux_slave_class_rndis_nx_ip_network_mask = rndis_parameter -> ux_slave_class_rndis_parameter_nx_ip_network_mask; 
    
    /* Copy the local node ID.  */
    _ux_utility_memory_copy(rndis -> ux_slave_class_rndis_local_node_id, rndis_parameter -> ux_slave_class_rndis_parameter_local_node_id,
                            UX_DEVICE_CLASS_RNDIS_NODE_ID_LENGTH); /* Use case of memcpy is verified. */

    /* Copy the remote node ID.  */
    _ux_utility_memory_copy(rndis -> ux_slave_class_rndis_remote_node_id, rndis_parameter -> ux_slave_class_rndis_parameter_remote_node_id,
                            UX_DEVICE_CLASS_RNDIS_NODE_ID_LENGTH); /* Use case of memcpy is verified. */

    /* Store the rest of the parameters as they are in the local instance.  */
    _ux_utility_memory_copy(&rndis -> ux_slave_class_rndis_parameter, rndis_parameter, sizeof (UX_SLAVE_CLASS_RNDIS_PARAMETER)); /* Use case of memcpy is verified. */

    /* Create a mutex to protect the RNDIS thread and the application messing up the transmit queue.  */
    status =  _ux_utility_mutex_create(&rndis -> ux_slave_class_rndis_mutex, "ux_slave_class_rndis_mutex");
    if (status != UX_SUCCESS)
        status = UX_MUTEX_ERROR;

    /* Allocate some memory for the interrupt thread stack. */
    if (status == UX_SUCCESS)
    {
        rndis -> ux_slave_class_rndis_interrupt_thread_stack =  
                _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_THREAD_STACK_SIZE);
        
        /* Check for successful allocation.  */
        if (rndis -> ux_slave_class_rndis_interrupt_thread_stack  == UX_NULL)

            /* Set status to memory insufficient.  */
            status = UX_MEMORY_INSUFFICIENT;
    }

    /* Interrupt endpoint treatment needs to be running in a different thread. So start
       a new thread. We pass a pointer to the rndis instance to the new thread.  This thread
       does not start until we have a instance of the class. */
    if (status == UX_SUCCESS)
    {
        status =  _ux_device_thread_create(&rndis -> ux_slave_class_rndis_interrupt_thread , "ux_slave_class_rndis_interrupt_thread", 
                    _ux_device_class_rndis_interrupt_thread,
                    (ULONG) (ALIGN_TYPE) class_ptr, (VOID *) rndis -> ux_slave_class_rndis_interrupt_thread_stack ,
                    UX_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                    UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);
                    
        /* Check the creation of this thread.  */
        if (status != UX_SUCCESS)
            status = UX_THREAD_ERROR;
    }

    UX_THREAD_EXTENSION_PTR_SET(&(rndis -> ux_slave_class_rndis_interrupt_thread), class_ptr)

    /* Allocate some memory for the bulk out thread stack. */
    if (status == UX_SUCCESS)
    {
        rndis -> ux_slave_class_rndis_bulkout_thread_stack =  
                _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_THREAD_STACK_SIZE);
        
        /* Check for successful allocation.  */
        if (rndis -> ux_slave_class_rndis_bulkout_thread_stack  == UX_NULL)
            status = UX_MEMORY_INSUFFICIENT;
    }

    /* Bulk endpoint treatment needs to be running in a different thread. So start
       a new thread. We pass a pointer to the rndis instance to the new thread.  This thread
       does not start until we have a instance of the class. */
    if (status == UX_SUCCESS)
    {
        status =  _ux_device_thread_create(&rndis -> ux_slave_class_rndis_bulkout_thread , "ux_slave_class_rndis_bulkout_thread", 
                    _ux_device_class_rndis_bulkout_thread,
                    (ULONG) (ALIGN_TYPE) class_ptr, (VOID *) rndis -> ux_slave_class_rndis_bulkout_thread_stack ,
                    UX_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                    UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);
                    
        /* Check the creation of this thread.  */
        if (status != UX_SUCCESS)
            status = UX_THREAD_ERROR;
    }

    UX_THREAD_EXTENSION_PTR_SET(&(rndis -> ux_slave_class_rndis_bulkout_thread), class_ptr)

    /* Allocate some memory for the bulk in thread stack. */
    if (status == UX_SUCCESS)
    {
        rndis -> ux_slave_class_rndis_bulkin_thread_stack =  
                _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_THREAD_STACK_SIZE);
        
        /* Check for successful allocation.  */
        if (rndis -> ux_slave_class_rndis_bulkin_thread_stack  == UX_NULL)
            status = UX_MEMORY_INSUFFICIENT;
    }

    /* Bulk endpoint treatment needs to be running in a different thread. So start
       a new thread. We pass a pointer to the rndis instance to the new thread.  This thread
       does not start until we have a instance of the class. */
    if (status == UX_SUCCESS)
    {
        status =  _ux_device_thread_create(&rndis -> ux_slave_class_rndis_bulkin_thread , "ux_slave_class_rndis_bulkin_thread", 
                    _ux_device_class_rndis_bulkin_thread,
                    (ULONG) (ALIGN_TYPE) class_ptr, (VOID *) rndis -> ux_slave_class_rndis_bulkin_thread_stack ,
                    UX_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                    UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);

        /* Check the creation of this thread.  */
        if (status != UX_SUCCESS)
            status = UX_THREAD_ERROR;
    }

    UX_THREAD_EXTENSION_PTR_SET(&(rndis -> ux_slave_class_rndis_bulkin_thread), class_ptr)

    /* Create a event flag group for the rndis class to synchronize with the event interrupt thread.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_utility_event_flags_create(&rndis -> ux_slave_class_rndis_event_flags_group, "ux_device_class_rndis_event_flag");

        /* Check status.  */
        if (status != UX_SUCCESS)
            status = UX_EVENT_ERROR;
    }

    /* Create a semaphore for protecting the driver entry.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_device_semaphore_create(&rndis -> ux_slave_class_rndis_semaphore, "ux_device_class_rndis_semaphore", 1);
        if (status == UX_SUCCESS)

            /* Return success.  */
            return(UX_SUCCESS);

        /* Semaphore creation error.  */
        status = UX_SEMAPHORE_ERROR;
    }

    /* Failed! Free up resources. */

    /* Delete semaphore for protecting the driver entry.  */
    if (rndis -> ux_slave_class_rndis_semaphore.tx_semaphore_id != 0)
        _ux_device_semaphore_delete(&rndis -> ux_slave_class_rndis_semaphore);

    /* Delete rndis -> ux_slave_class_rndis_event_flags_group.  */
    if (rndis -> ux_slave_class_rndis_event_flags_group.tx_event_flags_group_id != 0)
        _ux_utility_event_flags_delete(&rndis -> ux_slave_class_rndis_event_flags_group);

    /* Delete rndis -> ux_slave_class_rndis_bulkin_thread.  */
    if (rndis -> ux_slave_class_rndis_bulkin_thread.tx_thread_id != 0)
        _ux_device_thread_delete(&rndis -> ux_slave_class_rndis_bulkin_thread);

    /* Free rndis -> ux_slave_class_rndis_bulkin_thread_stack.  */
    if (rndis -> ux_slave_class_rndis_bulkin_thread_stack)
        _ux_utility_memory_free(rndis -> ux_slave_class_rndis_bulkin_thread_stack);

    /* Delete rndis -> ux_slave_class_rndis_bulkout_thread.  */
    if (rndis -> ux_slave_class_rndis_bulkout_thread.tx_thread_id != 0)
        _ux_device_thread_delete(&rndis -> ux_slave_class_rndis_bulkout_thread);

    /* Free rndis -> ux_slave_class_rndis_bulkout_thread_stack.  */
    if (rndis -> ux_slave_class_rndis_bulkout_thread_stack)
        _ux_utility_memory_free(rndis -> ux_slave_class_rndis_bulkout_thread_stack);
    
    /* Delete rndis -> ux_slave_class_rndis_interrupt_thread.  */
    if (rndis -> ux_slave_class_rndis_interrupt_thread.tx_thread_id != 0)
        _ux_device_thread_delete(&rndis -> ux_slave_class_rndis_interrupt_thread);
    
    /* Free rndis -> ux_slave_class_rndis_interrupt_thread_stack.  */
    if (rndis -> ux_slave_class_rndis_interrupt_thread_stack)
        _ux_utility_memory_free(rndis -> ux_slave_class_rndis_interrupt_thread_stack);

    /* Delete rndis -> ux_slave_class_rndis_mutex.  */
    if (rndis -> ux_slave_class_rndis_mutex.tx_mutex_id != 0)
        _ux_device_mutex_delete(&rndis -> ux_slave_class_rndis_mutex);

    /* Free memory for rndis instance.  */
    _ux_utility_memory_free(rndis);

    /* Return completion status.  */
    return(status);
#endif
}
