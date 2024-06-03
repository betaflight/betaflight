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
/**   Pictbridge Application                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_pictbridge.h"
#include "ux_device_stack.h"
#include "ux_device_class_pima.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_dpsclient_start                      PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function starts the DPS client (usually a camera or phone)     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                   Pima instance associated     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    user application                                                    */ 
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
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used macros for RTOS calls, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_dpsclient_start(UX_PICTBRIDGE *pictbridge)
{
UINT                                status;
UX_SLAVE_CLASS_PIMA_OBJECT          *object_info;

    /* Allocate 1 object for the host.  */
    pictbridge -> ux_pictbridge_object_host = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, sizeof(UX_SLAVE_CLASS_PIMA_OBJECT));

    /* Check status. */
    if (pictbridge -> ux_pictbridge_object_host  == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);    

    /* Initialize status to success for things going on.  */
    status = UX_SUCCESS;

    /* Allocate 1 object for the client.  */
    pictbridge -> ux_pictbridge_object_client = (VOID *)_ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, sizeof(UX_SLAVE_CLASS_PIMA_OBJECT));

    /* Check status. */
    if (pictbridge -> ux_pictbridge_object_client  == UX_NULL)
        status = (UX_MEMORY_INSUFFICIENT);    

    /* Allocate 1 object for the job.  */
    if (status == UX_SUCCESS)
    {
        pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_object = (VOID *)_ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, sizeof(UX_SLAVE_CLASS_PIMA_OBJECT));

        /* Check status. */
        if (pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_object  == UX_NULL)
            status = (UX_MEMORY_INSUFFICIENT);    
    }

    /* Allocate some memory for the XML objects.  For the client script. */
    if (status == UX_SUCCESS)
    {
        object_info = (UX_SLAVE_CLASS_PIMA_OBJECT *) pictbridge -> ux_pictbridge_object_client;
        object_info -> ux_device_class_pima_object_buffer = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER);

        /* Check status. */
        if (object_info -> ux_device_class_pima_object_buffer == UX_NULL)
            status = (UX_MEMORY_INSUFFICIENT);    
    }

    /* Allocate some memory for the XML objects.  For the host script. */
    if (status == UX_SUCCESS)
    {
        object_info = (UX_SLAVE_CLASS_PIMA_OBJECT *)  pictbridge -> ux_pictbridge_object_host;
        object_info -> ux_device_class_pima_object_buffer = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER);

        /* Check status. */
        if (object_info -> ux_device_class_pima_object_buffer == UX_NULL)
            return(UX_MEMORY_INSUFFICIENT);    
    }
        
    /* Create a event flag group for the client to communicate with the application.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_system_event_flags_create(&pictbridge -> ux_pictbridge_event_flags_group, "ux_pictbridge_client_event_flag");

        /* Check status.  */
        if (status != UX_SUCCESS)

            /* Do not proceed if error.  */
            status = (UX_EVENT_ERROR);
    }

    /* Create the semaphore to wake up the thread.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_system_semaphore_create(&pictbridge -> ux_pictbridge_notification_semaphore, "ux_pictbridge_client_semaphore", 0);
        if (status != UX_SUCCESS)

            /* Do not proceed if error.  */
            status = (UX_SEMAPHORE_ERROR);
    }

    /* Allocate a Thread stack.  */
    if (status == UX_SUCCESS)
    {
        pictbridge -> ux_pictbridge_thread_stack =  
                    _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_PICTBRIDGE_THREAD_STACK_SIZE);

        /* Check the completion status.  */
        if (pictbridge -> ux_pictbridge_thread_stack == UX_NULL)

            /* Do not proceed if error.  */
            return(UX_MEMORY_INSUFFICIENT);
    }

    /* Create the pictbridge class thread.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_system_thread_create(&pictbridge -> ux_pictbridge_thread,
                                "ux_pictbridge_thread", _ux_pictbridge_dpsclient_thread,
                                (ULONG)(ALIGN_TYPE) pictbridge, 
                                pictbridge -> ux_pictbridge_thread_stack,
                                UX_PICTBRIDGE_THREAD_STACK_SIZE, 
                                UX_PICTBRIDGE_THREAD_PRIORITY_CLASS,
                                UX_PICTBRIDGE_THREAD_PRIORITY_CLASS,
                                UX_NO_TIME_SLICE, UX_AUTO_START);
                    
        /* Check the completion status.  */
        if (status != UX_SUCCESS)

            /* Do not proceed if error.  */
            status = (UX_THREAD_ERROR);

        UX_THREAD_EXTENSION_PTR_SET(&(pictbridge -> ux_pictbridge_thread), pictbridge)
    }

    /* Go on to next step.  */
    if (status == UX_SUCCESS)
    {
        /* Initialize the pictbridge request/response state to request expected.  */
        pictbridge -> ux_pictbridge_request_response = UX_PICTBRIDGE_REQUEST;

        /* Set the host/client cycle to idle.  */    
        pictbridge -> ux_pictbridge_host_client_state_machine = UX_PICTBRIDGE_STATE_MACHINE_IDLE;
        
        /* Initialize the first XML object valid in the pictbridge instance.  Initialize the handle, type and file name. 
        The storage handle and the object handle have a fixed value of 1 in our implementation. */
        object_info = pictbridge -> ux_pictbridge_object_client;
        object_info -> ux_device_class_pima_object_format = UX_DEVICE_CLASS_PIMA_OFC_SCRIPT;
        object_info -> ux_device_class_pima_object_storage_id = 1;
        object_info -> ux_device_class_pima_object_handle_id = 2;
        _ux_utility_string_to_unicode(_ux_pictbridge_ddiscovery_name, object_info -> ux_device_class_pima_object_filename);

        /* Initialize the head and tail of the notification round robin buffers. 
        At first, the head and tail are pointing to the beginning of the array.  */
        pictbridge -> ux_pictbridge_event_array_head =  pictbridge -> ux_pictbridge_event_array;
        pictbridge -> ux_pictbridge_event_array_tail =  pictbridge -> ux_pictbridge_event_array;
        pictbridge -> ux_pictbridge_event_array_end  =  pictbridge -> ux_pictbridge_event_array + UX_PICTBRIDGE_MAX_EVENT_NUMBER;

        /* Initialize the pima device parameter.  */
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_manufacturer                  = pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_vendor_name;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_model                         = pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_product_name;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_serial_number                 = pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_serial_no;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_storage_id                    = 1;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_storage_type                  = UX_DEVICE_CLASS_PIMA_STC_FIXED_RAM;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_storage_file_system_type      = UX_DEVICE_CLASS_PIMA_FSTC_GENERIC_FLAT;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_storage_access_capability     = UX_DEVICE_CLASS_PIMA_AC_READ_WRITE;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_storage_max_capacity_low      = pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_storage_size;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_storage_max_capacity_high     = 0;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_storage_free_space_low        = pictbridge -> ux_pictbridge_dpslocal.ux_pictbridge_devinfo_storage_size;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_storage_free_space_high       = 0;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_storage_free_space_image      = 0;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_storage_description           = _ux_pictbridge_volume_description;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_storage_volume_label          = _ux_pictbridge_volume_label;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_object_number_get             = _ux_pictbridge_dpsclient_object_number_get;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_object_handles_get            = _ux_pictbridge_dpsclient_object_handles_get;
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_object_info_get               = _ux_pictbridge_dpsclient_object_info_get;   
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_object_data_get               = _ux_pictbridge_dpsclient_object_data_get;   
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_object_info_send              = _ux_pictbridge_dpsclient_object_info_send;  
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_object_data_send              = _ux_pictbridge_dpsclient_object_data_send;  
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_object_delete                 = _ux_pictbridge_dpsclient_object_delete;

        /* Store the instance owner.  */
        pictbridge -> ux_pictbridge_pima_parameter.ux_device_class_pima_parameter_application                   = (VOID *) pictbridge;
        
        /* Initialize the device pima class. The class is connected with interface 0 */
        status = _ux_device_stack_class_register(_ux_system_slave_class_pima_name, _ux_device_class_pima_entry, 
                                                    1, 0, (VOID *)&pictbridge -> ux_pictbridge_pima_parameter);
    }

    /* Check status.  */
    if (status != UX_SUCCESS)
    {
            
        /* Free resources allocated so far.  */
        if (_ux_system_thread_created(&pictbridge -> ux_pictbridge_thread))
            _ux_system_thread_delete(&pictbridge -> ux_pictbridge_thread);
        if (pictbridge -> ux_pictbridge_thread_stack)
            _ux_utility_memory_free(pictbridge -> ux_pictbridge_thread_stack);
        if (_ux_system_semaphore_created(&pictbridge -> ux_pictbridge_notification_semaphore))
            _ux_system_semaphore_delete(&pictbridge -> ux_pictbridge_notification_semaphore);
        if (_ux_system_event_flags_created(&pictbridge -> ux_pictbridge_event_flags_group))
            _ux_system_event_flags_delete(&pictbridge -> ux_pictbridge_event_flags_group);
        if (pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_object)
        {
            _ux_utility_memory_free(pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_object);
        }
        if (pictbridge -> ux_pictbridge_object_client)
        {
            object_info = (UX_SLAVE_CLASS_PIMA_OBJECT *) pictbridge -> ux_pictbridge_object_client;
            if (object_info -> ux_device_class_pima_object_buffer)
                _ux_utility_memory_free(object_info -> ux_device_class_pima_object_buffer);
            _ux_utility_memory_free(pictbridge -> ux_pictbridge_object_client);
        }
        object_info = (UX_SLAVE_CLASS_PIMA_OBJECT *)  pictbridge -> ux_pictbridge_object_host;
        if (object_info -> ux_device_class_pima_object_buffer)
            _ux_utility_memory_free(object_info -> ux_device_class_pima_object_buffer);
        _ux_utility_memory_free(pictbridge -> ux_pictbridge_object_host);

        /* Do not proceed if error.  */
        return(status);
    }        

    /* We are done.  */
    return(UX_SUCCESS);
}

