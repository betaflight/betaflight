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
#include "ux_host_class_pima.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_dpshost_start                        PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function starts the DPS host (usually a printer).              */ 
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
/*  04-25-2022     Yajun Xia                Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed string length check,  */
/*                                            fixed possible overflow,    */
/*                                            used define instead of num, */
/*                                            used macros for RTOS calls, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_dpshost_start(UX_PICTBRIDGE *pictbridge, UX_HOST_CLASS_PIMA *pima)
{
UINT                                status;
ULONG                               object_index;
UCHAR                               string_discovery_name[UX_PICTBRIDGE_MAX_FILE_NAME_SIZE + 1]; /* +1 for null-terminator */
UX_HOST_CLASS_PIMA_OBJECT           *pima_object;
UX_HOST_CLASS_PIMA_SESSION          *pima_session;
UINT                                length, length1;

    /* Store the pima instance in the pictbridge container.  */
    pictbridge -> ux_pictbridge_pima = (VOID *) pima;

    /* We need to allocate memory for the pima structures contained in the pictbridge instance : Storage.  */
    pictbridge -> ux_pictbridge_storage = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS_PIMA_STORAGE));

    /* Check the completion status.  */
    if (pictbridge -> ux_pictbridge_storage == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Initialize status to success for things going on.  */
    status = UX_SUCCESS;

    /* We need to allocate memory for the pima structures contained in the pictbridge instance : host object.  */
    pictbridge -> ux_pictbridge_object_host = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS_PIMA_OBJECT));

    /* Get the address of the object container.  */
    pima_object =  (UX_HOST_CLASS_PIMA_OBJECT *) pictbridge -> ux_pictbridge_object_host;

    /* Check the completion status.  */
    if (pictbridge -> ux_pictbridge_object_host == UX_NULL)
        status = (UX_MEMORY_INSUFFICIENT);
    else
    {

        /* Allocate some memory for the script object.  */
        pima_object -> ux_host_class_pima_object_buffer =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER);
        if (pima_object -> ux_host_class_pima_object_buffer == UX_NULL)
            status = (UX_MEMORY_INSUFFICIENT);
    }

    /* We need to allocate memory for the pima structures contained in the pictbridge instance : session.  */
    if (status == UX_SUCCESS)
    {
        pictbridge -> ux_pictbridge_session = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS_PIMA_SESSION));

        /* Check the completion status.  */
        if (pictbridge -> ux_pictbridge_session == UX_NULL)
            status = (UX_MEMORY_INSUFFICIENT);
    }

    /* Go on if no error.  */
    if (status == UX_SUCCESS)
    {

        /* We need to allocate memory for the pima structures contained in the pictbridge instance : device.  */
        pictbridge -> ux_pictbridge_device = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS_PIMA_DEVICE));

        /* Check the completion status.  */
        if (pictbridge -> ux_pictbridge_device == UX_NULL)
            status = (UX_MEMORY_INSUFFICIENT);
    }

    /* Go on if no error.  */
    if (status == UX_SUCCESS)
    {

        /* Store the Pictbridge instance in the pima application. This is used for the Pima callback
        into the Pictbridge (or other application) layer.  */
        pima -> ux_host_class_pima_application = (VOID *) pictbridge;
        
        /* State machine for the dpshost is idle.  */
        pictbridge -> ux_pictbridge_host_client_state_machine = UX_PICTBRIDGE_STATE_MACHINE_IDLE;
        
        /* Create the semaphore to wake up the thread.  */
        status =  _ux_system_semaphore_create(&pictbridge -> ux_pictbridge_notification_semaphore, "ux_pictbridge_notification_semaphore", 0);
        if (status != UX_SUCCESS)
            status = (UX_SEMAPHORE_ERROR);
    }

    /* Go on if no error.  */
    if (status == UX_SUCCESS)
    {

        /* Allocate a Thread stack.  */
        pictbridge -> ux_pictbridge_thread_stack =  
                    _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_PICTBRIDGE_THREAD_STACK_SIZE);

        /* Check the completion status.  */
        if (pictbridge -> ux_pictbridge_thread_stack == UX_NULL)
            status = (UX_MEMORY_INSUFFICIENT);
    }

    /* Go on if no error.  */
    if (status == UX_SUCCESS)
    {

        /* Create the pictbridge class thread.  */
        status =  _ux_system_thread_create(&pictbridge -> ux_pictbridge_thread,
                                "ux_pictbridge_thread", _ux_pictbridge_dpshost_thread,
                                (ULONG)(ALIGN_TYPE) pictbridge, 
                                pictbridge -> ux_pictbridge_thread_stack,
                                UX_PICTBRIDGE_THREAD_STACK_SIZE, 
                                UX_PICTBRIDGE_THREAD_PRIORITY_CLASS,
                                UX_PICTBRIDGE_THREAD_PRIORITY_CLASS,
                                UX_NO_TIME_SLICE, UX_AUTO_START);
                    
        /* Check the completion status.  */
        if (status != UX_SUCCESS)
            status = (UX_THREAD_ERROR);

        UX_THREAD_EXTENSION_PTR_SET(&(pictbridge -> ux_pictbridge_thread), pictbridge)
    }

    /* Free up resources if there is error.  */
    if (status != UX_SUCCESS)
    {

        /* Check and free pictbridge -> ux_pictbridge_thread.  */
        if (_ux_system_thread_created(&pictbridge -> ux_pictbridge_thread))
            _ux_system_thread_delete(&pictbridge -> ux_pictbridge_thread);

        /* Check and free pictbridge -> ux_pictbridge_thread_stack.  */
        if (pictbridge -> ux_pictbridge_thread_stack)
            _ux_utility_memory_free(pictbridge -> ux_pictbridge_thread_stack);

        /* Check and free pictbridge -> ux_pictbridge_notification_semaphore.  */
        if (_ux_system_semaphore_created(&pictbridge -> ux_pictbridge_notification_semaphore))
            _ux_system_semaphore_delete(&pictbridge -> ux_pictbridge_notification_semaphore);

        /* Check and free pictbridge -> ux_pictbridge_device.  */
        if (pictbridge -> ux_pictbridge_device)
            _ux_utility_memory_free(pictbridge -> ux_pictbridge_device);

        /* Check and free pictbridge -> ux_pictbridge_session.  */
        if (pictbridge -> ux_pictbridge_session)
            _ux_utility_memory_free(pictbridge -> ux_pictbridge_session);

        /* Check and free pictbridge -> ux_pictbridge_object_host (pima_object).  */
        if (pima_object)
        {
            if (pima_object -> ux_host_class_pima_object_buffer)
                _ux_utility_memory_free(pima_object -> ux_host_class_pima_object_buffer);
            _ux_utility_memory_free(pima_object);
        }

        /* Free pictbridge -> ux_pictbridge_storage.  */
        _ux_utility_memory_free(pictbridge -> ux_pictbridge_storage);

        return(status);
    }

    /* Save the session in a properly casted format.  */
    pima_session = (UX_HOST_CLASS_PIMA_SESSION *)pictbridge->ux_pictbridge_session;

    /* Initialize the pictbridge state machine to request expected.  */
    pictbridge -> ux_pictbridge_request_response = UX_PICTBRIDGE_REQUEST;

    /* Initialize the head and tail of the notification round robin buffers. 
       At first, the head and tail are pointing to the beginning of the array.  */
    pictbridge -> ux_pictbridge_event_array_head =  pictbridge -> ux_pictbridge_event_array;
    pictbridge -> ux_pictbridge_event_array_tail =  pictbridge -> ux_pictbridge_event_array;
    pictbridge -> ux_pictbridge_event_array_end  =  pictbridge -> ux_pictbridge_event_array + UX_PICTBRIDGE_MAX_EVENT_NUMBER;

    /* Get the device info .  */
    status = _ux_host_class_pima_device_info_get(pima, (UX_HOST_CLASS_PIMA_DEVICE *)pictbridge -> ux_pictbridge_device);
    if (status != UX_SUCCESS)
        return(UX_PICTBRIDGE_ERROR_GENERAL_ERROR);

    /* Set a callback for the pima session notification.  */
    pima_session -> ux_host_class_pima_session_event_callback =  _ux_pictbridge_dpshost_notification_callback;
    
    /* Open a pima session.  */
    status = _ux_host_class_pima_session_open(pima, pima_session);
    if (status != UX_SUCCESS)
        return(UX_PICTBRIDGE_ERROR_SESSION_NOT_OPEN);

    /* Get the number of storage IDs.  */
    status = _ux_host_class_pima_storage_ids_get(pima, pima_session, 
                                                 pictbridge -> ux_pictbridge_storage_ids,
                                                 UX_PICTBRIDGE_MAX_NUMBER_STORAGE_IDS);
    if (status != UX_SUCCESS)
    {
        /* Close the pima session.  */
        _ux_host_class_pima_session_close(pima, pima_session);

        return(UX_PICTBRIDGE_ERROR_STORE_NOT_AVAILABLE);
    }        

    /* Get the first storage ID info container.    */
    status = _ux_host_class_pima_storage_info_get(pima, pima_session, 
                                                 pictbridge -> ux_pictbridge_storage_ids[0],
                                                 (UX_HOST_CLASS_PIMA_STORAGE *)pictbridge -> ux_pictbridge_storage);
    if (status != UX_SUCCESS)
    {
        /* Close the pima session.  */
        _ux_host_class_pima_session_close(pictbridge -> ux_pictbridge_pima, pima_session);

        return(UX_PICTBRIDGE_ERROR_STORE_NOT_AVAILABLE);
    }        

    /* Get the number of objects on the container.  */
    status = _ux_host_class_pima_num_objects_get(pima, pima_session, 
                                                 UX_PICTBRIDGE_ALL_CONTAINERS, UX_PICTBRIDGE_OBJECT_SCRIPT);
    if (status != UX_SUCCESS)
    {
        /* Close the pima session.  */
        _ux_host_class_pima_session_close(pima, pima_session);

        return(UX_PICTBRIDGE_ERROR_STORE_NOT_AVAILABLE);
    }        

    /* Get the array of objects handles on the container.  */
    if (pima_session -> ux_host_class_pima_session_nb_objects > UX_PICTBRIDGE_MAX_NUMBER_OBJECT_HANDLES)
        length = UX_PICTBRIDGE_MAX_NUMBER_OBJECT_HANDLES;
    else
        length = pima_session -> ux_host_class_pima_session_nb_objects;
    status = _ux_host_class_pima_object_handles_get(pima, pima_session, 
                                                 pictbridge -> ux_pictbridge_object_handles_array, 
                                                 length,
                                                 UX_PICTBRIDGE_ALL_CONTAINERS, UX_PICTBRIDGE_OBJECT_SCRIPT, 0);
    if (status != UX_SUCCESS)
    {
        /* Close the pima session.  */
        _ux_host_class_pima_session_close(pima, pima_session);

        return(UX_PICTBRIDGE_ERROR_STORE_NOT_AVAILABLE);
    }        

    /* We search for an object that is a picture or a script.  */
    object_index =  0;
    while (object_index < pima_session -> ux_host_class_pima_session_nb_objects)
    {

        /* Get the object info structure.  */
        status = _ux_host_class_pima_object_info_get(pima, pima_session, 
                                                 pictbridge -> ux_pictbridge_object_handles_array[object_index], pima_object);
        if (status != UX_SUCCESS)
        {
            /* Close the pima session.  */
            _ux_host_class_pima_session_close(pima, pima_session);

            return(UX_PICTBRIDGE_ERROR_INVALID_OBJECT_HANDLE );
        }        

        /* Check if this is a script.  */        
        if (pima_object -> ux_host_class_pima_object_format == UX_HOST_CLASS_PIMA_OFC_SCRIPT)
        {

            /* Yes this is a script. We need to search for the DDISCVRY.DPS file name.
               Get the file name length (with null-terminator).  */
            length1 = (UINT)(*pima_object -> ux_host_class_pima_object_filename);

            /* Now, compare it to the DDISCVRY.DPS file name.  Check length first (excluding null-terminator).  */
            if (length1 <= UX_PICTBRIDGE_MAX_FILE_NAME_SIZE)
            {

                /* Invalidate length, on error it's untouched.  */
                length = UX_PICTBRIDGE_MAX_FILE_NAME_SIZE + 1;
                _ux_utility_string_length_check(_ux_pictbridge_ddiscovery_name, &length, UX_PICTBRIDGE_MAX_FILE_NAME_SIZE);
                if ((length + 1) == length1)
                {

                    /* Get the file name in a ascii format (with null-terminator).  */
                    _ux_utility_unicode_to_string(pima_object -> ux_host_class_pima_object_filename, string_discovery_name);

                    /* So far, the length of name of the files are the same.
                    Compare names now (since length is same just compare without null-terminator). */
                    if (_ux_utility_memory_compare(_ux_pictbridge_ddiscovery_name, string_discovery_name,
                                                    length) ==  UX_SUCCESS)
                    {

                        /* We have found a script with the DDISCVRY.DPS file name. Prepare a reply with a HDISCVRY. DPS name.  
                        We use the same object container. Just change the name of the file.  */
                        _ux_utility_string_to_unicode(_ux_pictbridge_hdiscovery_name, pima_object -> ux_host_class_pima_object_filename);                    
                        
                        /* Send the script info.  */
                        status = _ux_host_class_pima_object_info_send(pima, pima_session, 0,0,
                                                            pima_object);
                        
                        /* Check the status of this operation.  */
                        if (status != UX_SUCCESS)
                        {

                            /* Close the pima session.  */
                            _ux_host_class_pima_session_close(pima, pima_session);

                            return(UX_PICTBRIDGE_ERROR_INVALID_OBJECT_HANDLE);
                        }        
                        else
            
                            /* We return to the application with a success status. We leave the session opened. */         
                            return(UX_SUCCESS);
                        
                    }
                    
                }
            }
        }

        /* Next object index.  */
        object_index++;
    
    }
    
    /* We come here when we have not found any script or the script does not have the DDISCVRY.DPS file. Close the pima session.  */
    _ux_host_class_pima_session_close(pima, pima_session);
    return(UX_PICTBRIDGE_ERROR_NO_DISCOVERY_SCRIPT);
}

