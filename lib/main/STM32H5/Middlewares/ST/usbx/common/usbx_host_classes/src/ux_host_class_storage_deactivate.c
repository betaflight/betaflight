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
/**   Storage Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_storage.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_storage_deactivate                   PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called when this instance of the storage device    */ 
/*    has been removed from the bus either directly or indirectly. The    */ 
/*    bulk in\out pipes will be destroyed and the instanced removed.      */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Pointer to class command      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    ux_media_close                        Close media                   */ 
/*    _ux_host_stack_endpoint_transfer_abort Abort transfer request       */ 
/*    _ux_host_stack_class_instance_destroy Destroy class instance        */ 
/*    _ux_utility_memory_free               Free memory block             */ 
/*    _ux_host_semaphore_get                Get protection semaphore      */ 
/*    _ux_host_semaphore_delete             Delete protection semaphore   */ 
/*    _ux_utility_thread_schedule_other     Schedule other threads        */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Storage Class                                                       */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added option to disable FX  */
/*                                            media integration, used UX_ */
/*                                            things instead of FX_       */
/*                                            things directly, used host  */
/*                                            class extension pointer for */
/*                                            class specific structured   */
/*                                            data,                       */
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile issues with   */
/*                                            some macro options,         */
/*                                            resulting in version 6.1.6  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved media insert/eject */
/*                                            management without FX,      */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_deactivate(UX_HOST_CLASS_COMMAND *command)
{

UINT                            status;
UX_HOST_CLASS_STORAGE           *storage;
UX_HOST_CLASS_STORAGE_MEDIA     *storage_media;
UX_HOST_CLASS                   *class_inst;
UINT                            inst_index;
#if !defined(UX_HOST_CLASS_STORAGE_NO_FILEX)
UX_MEDIA                        *media;
VOID                            *memory;
#endif


    /* Get the instance for this class.  */
    storage =  (UX_HOST_CLASS_STORAGE *) command -> ux_host_class_command_instance;
    
    /* We need the class container.  */
    class_inst =  storage -> ux_host_class_storage_class;

    /* Point the media structure to the first media in the container.  */
    storage_media =  (UX_HOST_CLASS_STORAGE_MEDIA *) class_inst -> ux_host_class_media;

    /* The storage device is being shut down.  */
    storage -> ux_host_class_storage_state =  UX_HOST_CLASS_INSTANCE_SHUTDOWN;

    /* We come to this point when the device has been extracted. So there may have been a transaction
       being scheduled. We make sure the transaction has been completed by the controller driver.
       When the device is extracted, the controller tries multiple times the transaction and retires it
       with a DEVICE_NOT_RESPONDING error code.  
       
       First we take care of endpoint OUT.  */

    /* We need to abort transactions on the bulk pipes.  */
    if (storage -> ux_host_class_storage_bulk_out_endpoint != UX_NULL)
        _ux_host_stack_endpoint_transfer_abort(storage -> ux_host_class_storage_bulk_out_endpoint);
    
    /* Then endpoint IN.  */       
    if (storage -> ux_host_class_storage_bulk_in_endpoint != UX_NULL)
        _ux_host_stack_endpoint_transfer_abort(storage -> ux_host_class_storage_bulk_in_endpoint);
       
#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    /* Was the protocol CBI ? */
    if (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceProtocol == UX_HOST_CLASS_STORAGE_PROTOCOL_CBI)
    {

        /* Was there an interrupt endpoint?  */
        if (storage -> ux_host_class_storage_interrupt_endpoint != UX_NULL)
        {

            /* Abort the data transfer on the interrupt endpoint.  */
            _ux_host_stack_endpoint_transfer_abort(storage -> ux_host_class_storage_interrupt_endpoint);

            /* Free the memory that was used by the interrupt endpoint.  */
            if (storage -> ux_host_class_storage_interrupt_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer != UX_NULL)
                _ux_utility_memory_free(storage -> ux_host_class_storage_interrupt_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer);
        }
    }
#endif

    /* The enumeration thread needs to sleep a while to allow the application or the class that may be using
       endpoints to exit properly.  */
    _ux_host_thread_schedule_other(UX_THREAD_PRIORITY_ENUM); 


    /* Inform UX_MEDIA (default FileX) of the deactivation of all Media attached to this instance.  */
    for (inst_index = 0; inst_index < UX_HOST_CLASS_STORAGE_MAX_MEDIA; inst_index++)
    {

#if !defined(UX_HOST_CLASS_STORAGE_NO_FILEX)
        /* Get the UX_MEDIA (default FileX) attached to this media.  */
        media = &storage_media -> ux_host_class_storage_media;

        /* Check if the media belongs to the device being removed.  */
        if (((UX_HOST_CLASS_STORAGE *) ux_media_driver_info_get(media)) == storage)
        {

            /* Check if the media was properly opened.  */
            if (storage_media -> ux_host_class_storage_media_status == UX_HOST_CLASS_STORAGE_MEDIA_MOUNTED)
            {
            
                /* We preserve the memory used by this media.  */
                memory =  storage_media -> ux_host_class_storage_media_memory;

                /* Ask UX_MEDIA (default FileX) to unmount the partition.  */
                ux_media_close(media);

                /* This device is now unmounted.  */
                storage_media -> ux_host_class_storage_media_status =  UX_HOST_CLASS_STORAGE_MEDIA_UNMOUNTED;
            
                /* Reset the media ID.  */
                ux_media_id_set(media, 0);
                                
                /* Free the memory block used for data transfer on behalf of UX_MEDIA (default FileX).  */
                _ux_utility_memory_free(memory);
            }                
        }
#else

        /* Check if the media is for this storage.  */
        if (storage_media -> ux_host_class_storage_media_status == UX_USED &&
            storage_media -> ux_host_class_storage_media_storage == storage)
        {

            /* Free the storage media.  */
            storage_media -> ux_host_class_storage_media_status = UX_UNUSED;

            /* Invoke callback for media removal.  */
            if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
            {

                /* Call system change function.  */
                _ux_system_host ->  ux_system_host_change_function(UX_STORAGE_MEDIA_REMOVAL,
                                    storage -> ux_host_class_storage_class, (VOID *) storage_media);
            }
        }
#endif

        /* Move to next entry in the media array.  */
        storage_media++;
    }

    /* Protect thread reentry to this instance.  */
    status = _ux_host_semaphore_get(&storage -> ux_host_class_storage_semaphore, UX_WAIT_FOREVER);
    UX_PARAMETER_NOT_USED(status);

    /* Destroy the instance.  */
    _ux_host_stack_class_instance_destroy(storage -> ux_host_class_storage_class, (VOID *) storage);

    /* Destroy the protection semaphore.  */
    _ux_host_semaphore_delete(&storage -> ux_host_class_storage_semaphore);

    /* Before we free the device resources, we need to inform the application
        that the device is removed.  */
    if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
    {
        
        /* Inform the application the device is removed.  */
        _ux_system_host -> ux_system_host_change_function(UX_DEVICE_REMOVAL, storage -> ux_host_class_storage_class, (VOID *) storage);
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_STORAGE_DEACTIVATE, storage, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_UNREGISTER(storage);

    /* Free the storage instance memory.  */
    _ux_utility_memory_free(storage);

    /* Return successful completion.  */
    return(UX_SUCCESS);         
}

