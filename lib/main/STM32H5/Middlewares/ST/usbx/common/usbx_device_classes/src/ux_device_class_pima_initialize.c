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
/**   Device CDC Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_pima.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_pima_initialize                    PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function initializes the USB Pima device class                 */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                              Pointer to pima command        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_allocate           Allocate memory               */ 
/*    _ux_utility_memory_free               Free memory                   */
/*    _ux_device_thread_create              Create thread                 */
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
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            added cancel callback,      */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_initialize(UX_SLAVE_CLASS_COMMAND *command)
{
#if defined(UX_DEVICE_STANDALONE)
    UX_PARAMETER_NOT_USED(command);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else
UINT                                    status;
UX_SLAVE_CLASS_PIMA                     *pima;
UX_SLAVE_CLASS_PIMA_PARAMETER           *pima_parameter;
UX_SLAVE_CLASS                          *class_ptr;

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Create an instance of the device pima class.  */
    pima =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_SLAVE_CLASS_PIMA));

    /* Check for successful allocation.  */
    if (pima == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Save the address of the PIMA instance inside the PIMA container.  */
    class_ptr -> ux_slave_class_instance = (VOID *) pima;

    /* Allocate some memory for the thread stack. */
    class_ptr -> ux_slave_class_thread_stack =  
            _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_THREAD_STACK_SIZE);
    
    /* Check for successful allocation.  */
    if (class_ptr -> ux_slave_class_thread_stack == UX_NULL)
        status = UX_MEMORY_INSUFFICIENT;
    else
        status = UX_SUCCESS;

    /* This instance needs to be running in a different thread. So start
       a new thread. We pass a pointer to the class to the new thread.  This thread
       does not start until we have a instance of the class. */
    if (status == UX_SUCCESS)
    {
        status =  _ux_device_thread_create(&class_ptr -> ux_slave_class_thread, "ux_slave_class_thread", 
                    _ux_device_class_pima_thread,
                    (ULONG) (ALIGN_TYPE) class_ptr, (VOID *) class_ptr -> ux_slave_class_thread_stack,
                    UX_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                    UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);
                    
        /* Check the creation of this thread.  */
        if (status != UX_SUCCESS)
            status = UX_THREAD_ERROR;
    }

    UX_THREAD_EXTENSION_PTR_SET(&(class_ptr -> ux_slave_class_thread), class_ptr)

    /* There is error, free resources and return error.  */
    if (status != UX_SUCCESS)
    {

        /* The last resource, thread is not created or created error,
           no need to free.  */

        if (class_ptr -> ux_slave_class_thread_stack)
            _ux_utility_memory_free(class_ptr -> ux_slave_class_thread_stack);

        /* Detach instance and free memory.  */
        class_ptr -> ux_slave_class_instance = UX_NULL;
        _ux_utility_memory_free(pima);

        /* Return completion status.  */
        return(status);
    }

    /* Success, complete remaining settings.  */

    /* Get the pointer to the application parameters for the pima class.  */
    pima_parameter =  command -> ux_slave_class_command_parameter;

    /* Store all the application parameter information about the media.  */
    pima -> ux_device_class_pima_manufacturer                   = pima_parameter -> ux_device_class_pima_parameter_manufacturer;  
    pima -> ux_device_class_pima_model                          = pima_parameter -> ux_device_class_pima_parameter_model;         
    pima -> ux_device_class_pima_device_version                 = pima_parameter -> ux_device_class_pima_parameter_device_version;
    pima -> ux_device_class_pima_serial_number                  = pima_parameter -> ux_device_class_pima_parameter_serial_number; 

    /* Store all the application parameter information about the storage.  */
    pima -> ux_device_class_pima_storage_id                     = pima_parameter -> ux_device_class_pima_parameter_storage_id;  
    pima -> ux_device_class_pima_storage_type                   = pima_parameter -> ux_device_class_pima_parameter_storage_type;  
    pima -> ux_device_class_pima_storage_file_system_type       = pima_parameter -> ux_device_class_pima_parameter_storage_file_system_type;  
    pima -> ux_device_class_pima_storage_access_capability      = pima_parameter -> ux_device_class_pima_parameter_storage_access_capability;  
    pima -> ux_device_class_pima_storage_max_capacity_low       = pima_parameter -> ux_device_class_pima_parameter_storage_max_capacity_low;  
    pima -> ux_device_class_pima_storage_max_capacity_high      = pima_parameter -> ux_device_class_pima_parameter_storage_max_capacity_high;  
    pima -> ux_device_class_pima_storage_free_space_low         = pima_parameter -> ux_device_class_pima_parameter_storage_free_space_low;  
    pima -> ux_device_class_pima_storage_free_space_high        = pima_parameter -> ux_device_class_pima_parameter_storage_free_space_high;  
    pima -> ux_device_class_pima_storage_free_space_image       = pima_parameter -> ux_device_class_pima_parameter_storage_free_space_image;  
    pima -> ux_device_class_pima_storage_description            = pima_parameter -> ux_device_class_pima_parameter_storage_description;  
    pima -> ux_device_class_pima_storage_volume_label           = pima_parameter -> ux_device_class_pima_parameter_storage_volume_label;  
    
    /* Update device properties supported. */
    pima -> ux_device_class_pima_device_properties_list         = pima_parameter -> ux_device_class_pima_parameter_device_properties_list;

    /* Update the capture formats supported list. */
    pima -> ux_device_class_pima_supported_capture_formats_list = pima_parameter -> ux_device_class_pima_parameter_supported_capture_formats_list;
        
    /* Update the image formats supported list. */
    pima -> ux_device_class_pima_supported_image_formats_list   = pima_parameter -> ux_device_class_pima_parameter_supported_image_formats_list;
        
#ifdef UX_PIMA_WITH_MTP_SUPPORT
    /* Update the internal pima structure with the object properties.  */
    pima -> ux_device_class_pima_object_properties_list         = pima_parameter -> ux_device_class_pima_parameter_object_properties_list;

#endif

    /* Store the callback functions for request.  */
    pima -> ux_device_class_pima_cancel                         = pima_parameter -> ux_device_class_pima_parameter_cancel;

    /* Store the callback functions for device. */
    pima -> ux_device_class_pima_device_reset                   = pima_parameter -> ux_device_class_pima_parameter_device_reset;
    pima -> ux_device_class_pima_device_prop_desc_get           = pima_parameter -> ux_device_class_pima_parameter_device_prop_desc_get;
    pima -> ux_device_class_pima_device_prop_value_get          = pima_parameter -> ux_device_class_pima_parameter_device_prop_value_get;
    pima -> ux_device_class_pima_device_prop_value_set          = pima_parameter -> ux_device_class_pima_parameter_device_prop_value_set;

    /* Store the callback functions for storage. */
    pima -> ux_device_class_pima_storage_format                 = pima_parameter -> ux_device_class_pima_parameter_storage_format;
    pima -> ux_device_class_pima_storage_info_get               = pima_parameter -> ux_device_class_pima_parameter_storage_info_get;

    /* Store the callback functions for objects.  */
    pima -> ux_device_class_pima_object_number_get              = pima_parameter -> ux_device_class_pima_parameter_object_number_get; 
    pima -> ux_device_class_pima_object_handles_get             = pima_parameter -> ux_device_class_pima_parameter_object_handles_get;
    pima -> ux_device_class_pima_object_info_get                = pima_parameter -> ux_device_class_pima_parameter_object_info_get;   
    pima -> ux_device_class_pima_object_data_get                = pima_parameter -> ux_device_class_pima_parameter_object_data_get;   
    pima -> ux_device_class_pima_object_info_send               = pima_parameter -> ux_device_class_pima_parameter_object_info_send;  
    pima -> ux_device_class_pima_object_data_send               = pima_parameter -> ux_device_class_pima_parameter_object_data_send;  
    pima -> ux_device_class_pima_object_delete                  = pima_parameter -> ux_device_class_pima_parameter_object_delete;


#ifdef UX_PIMA_WITH_MTP_SUPPORT
    /* Add the MTP specific callback functions.  */
    pima -> ux_device_class_pima_object_prop_desc_get           = pima_parameter -> ux_device_class_pima_parameter_object_prop_desc_get;
    pima -> ux_device_class_pima_object_prop_value_get          = pima_parameter -> ux_device_class_pima_parameter_object_prop_value_get;
    pima -> ux_device_class_pima_object_prop_value_set          = pima_parameter -> ux_device_class_pima_parameter_object_prop_value_set;
    pima -> ux_device_class_pima_object_references_get          = pima_parameter -> ux_device_class_pima_parameter_object_references_get;
    pima -> ux_device_class_pima_object_references_set          = pima_parameter -> ux_device_class_pima_parameter_object_references_set;
#endif

    /* Store the application owner. */
    pima -> ux_device_class_pima_application                    = pima_parameter -> ux_device_class_pima_parameter_application;

    /* Store the start and stop signals if needed by the application.  */
    pima -> ux_device_class_pima_instance_activate = pima_parameter -> ux_device_class_pima_instance_activate;
    pima -> ux_device_class_pima_instance_deactivate = pima_parameter -> ux_device_class_pima_instance_deactivate;

    /* Return completion status.  */
    return(status);
#endif
}

