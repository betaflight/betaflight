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
/*    _ux_host_class_storage_activate                     PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function activates an instance of the storage class.           */
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
/*    _ux_host_class_storage_configure      Configure storage device      */
/*    _ux_host_class_storage_device_initialize                            */
/*                                          Initialize storage device     */
/*    _ux_host_stack_class_instance_create  Create class instance         */
/*    _ux_host_stack_class_instance_destroy Destroy class instance        */
/*    _ux_utility_memory_allocate           Allocate memory block         */
/*    _ux_utility_memory_free               Free memory block             */
/*    _ux_host_semaphore_create             Create semaphore              */
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
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support.   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_activate(UX_HOST_CLASS_COMMAND *command)
{

UX_INTERFACE            *interface_ptr;
UX_HOST_CLASS_STORAGE   *storage;
UINT                    status;


    /* The storage is always activated by the interface descriptor and not the
       device descriptor.  */
    interface_ptr =  (UX_INTERFACE *) command -> ux_host_class_command_container;

    /* Obtain memory for this class instance.  The memory used MUST BE allocated from a CACHE SAFE memory
       since the buffer for the CSW is an array contained within each storage instance. */
    storage =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, sizeof(UX_HOST_CLASS_STORAGE));
    if (storage == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Store the class container into this instance */
    storage -> ux_host_class_storage_class =  command -> ux_host_class_command_class_ptr;

    /* Store the interface container into the storage class instance.  */
    storage -> ux_host_class_storage_interface =  interface_ptr;

    /* Store the device container into the storage class instance.  */
    storage -> ux_host_class_storage_device =  interface_ptr -> ux_interface_configuration -> ux_configuration_device;

    /* Create this class instance.  */
    _ux_host_stack_class_instance_create(command -> ux_host_class_command_class_ptr, (VOID *) storage);

    /* This instance of the device must also be stored in the interface container.  */
    interface_ptr -> ux_interface_class_instance =  (VOID *) storage;

#if defined(UX_HOST_STANDALONE)

    /* Check class,sub class, protocol.  */
    status =  _ux_host_class_storage_device_support_check(storage);
    if (status != UX_SUCCESS)
    {
        _ux_utility_memory_free(storage);
        return(status);
    }

    /* Search all the endpoints for the storage interface (Bulk Out, Bulk in,
       and optional Interrupt endpoint).  */
    status =  _ux_host_class_storage_endpoints_get(storage);
    if (status != UX_SUCCESS)
    {
        _ux_utility_memory_free(storage);
        return(status);
    }

    /* Activate storage class task function.  */
    storage -> ux_host_class_storage_class -> ux_host_class_task_function = _ux_host_class_storage_tasks_run;

    /* Mark the storage as live now (media mounts in task).  */
    storage -> ux_host_class_storage_state = UX_HOST_CLASS_INSTANCE_MOUNTING;

    /* Keep storage locked before it's initialized.  */
    storage -> ux_host_class_storage_flags |= UX_HOST_CLASS_STORAGE_FLAG_LOCK;
#else

    /* Configure the USB storage device.  */
    status =  _ux_host_class_storage_configure(storage);

    /* Create the semaphore to protect multiple threads from accessing the same storage instance.  */
    if (status == UX_SUCCESS)
    {
        status = _ux_host_semaphore_create(&storage -> ux_host_class_storage_semaphore, "ux_host_class_storage_semaphore", 1);
        if (status != UX_SUCCESS)
            status = UX_SEMAPHORE_ERROR;
    }

    /* Error case, free resources.  */
    if (status != UX_SUCCESS)
    {

        /* Last one, semaphore not created or created error, no need to free.  */

        /* Error, destroy the class and return an error.  */
        _ux_host_stack_class_instance_destroy(storage -> ux_host_class_storage_class, (VOID *) storage);

        /* This instance of the device must also be removed in the interface container.  */
        interface_ptr -> ux_interface_class_instance =  (VOID *) UX_NULL;

        /* Free memory for class instance.  */
        _ux_utility_memory_free(storage);

        return(status);
    }

    /* Mark the storage as mounting now.  */
    storage -> ux_host_class_storage_state =  UX_HOST_CLASS_INSTANCE_MOUNTING;

    /* Initialize the USB storage device.  We do not check the status at this stage. We let the instance of this
       class live even if there was a failure. Because the storage class has many media instance, we will let the
       disconnection signal clean the instance at a later stage.  */
    _ux_host_class_storage_device_initialize(storage);

    /* Mark the storage as live now.  */
    storage -> ux_host_class_storage_state =  UX_HOST_CLASS_INSTANCE_LIVE;

#endif

    /* If all is fine and the device is mounted, we may need to inform the application
       if a function has been programmed in the system structure.  */
    if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
    {

        /* Call system change function.  */
        _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, storage -> ux_host_class_storage_class, (VOID *) storage);
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_STORAGE_ACTIVATE, storage, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, storage, 0, 0, 0)

    /* Return completion status. Force it to success. */
    return(UX_SUCCESS);
}

