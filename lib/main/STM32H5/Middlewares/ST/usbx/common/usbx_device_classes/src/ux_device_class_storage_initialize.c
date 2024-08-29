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
/**   Device Storage Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_storage.h"
#include "ux_device_stack.h"

/* Define the Slave Storage Class Inquiry data : DO NOT CHANGE THE LENGTH OF THESE ITEMS */

UCHAR _ux_system_slave_class_storage_vendor_id[] =                          "AzureRTO";
UCHAR _ux_system_slave_class_storage_product_id[] =                         "USBX storage dev";
UCHAR _ux_system_slave_class_storage_product_rev[] =                        "2000";
UCHAR _ux_system_slave_class_storage_product_serial[] =                     "12345678901234567890";

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_storage_initialize                 PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function initializes the USB storage device.                   */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                               Pointer to storage command    */
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
/*    _ux_device_thread_delete              Delete thread                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Device Storage Class                                                */
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
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_storage_initialize(UX_SLAVE_CLASS_COMMAND *command)
{

UINT                                    status = UX_SUCCESS;
UX_SLAVE_CLASS_STORAGE                  *storage;
UX_SLAVE_CLASS_STORAGE_PARAMETER        *storage_parameter;
UX_SLAVE_CLASS                          *class_inst;
ULONG                                   lun_index;


    /* Get the pointer to the application parameters for the storage class.  */
    storage_parameter =  command -> ux_slave_class_command_parameter;

    /* Ensure the number of LUN declared by the caller does not exceed the
       max number allowed for LUN storage.  */
    if (storage_parameter -> ux_slave_class_storage_parameter_number_lun > UX_MAX_SLAVE_LUN)
        return UX_ERROR;

    /* Get the class container.  */
    class_inst =  command -> ux_slave_class_command_class_ptr;

    /* Create an instance of the device storage class.  */
    storage =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_SLAVE_CLASS_STORAGE));

    /* Check for successful allocation.  */
    if (storage == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

#if !defined(UX_DEVICE_STANDALONE)

    /* Allocate some memory for the thread stack. */
    class_inst -> ux_slave_class_thread_stack = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_THREAD_STACK_SIZE);

    /* If it's OK, create thread.  */
    if (class_inst -> ux_slave_class_thread_stack != UX_NULL)

        /* This instance needs to be running in a different thread. So start
           a new thread. We pass a pointer to the class to the new thread.  This thread
           does not start until we have a instance of the class. */
        status =  _ux_device_thread_create(&class_inst -> ux_slave_class_thread, "ux_slave_storage_thread",
                    _ux_device_class_storage_thread,
                    (ULONG) (ALIGN_TYPE) class_inst, (VOID *) class_inst -> ux_slave_class_thread_stack,
                    UX_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                    UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);
    else
        status = UX_MEMORY_INSUFFICIENT;
#else

    /* Save tasks run entry.  */
    class_inst -> ux_slave_class_task_function = _ux_device_class_storage_tasks_run;

    status = UX_SUCCESS;
#endif

    /* If thread resources allocated, go on.  */
    if (status == UX_SUCCESS)
    {

        UX_THREAD_EXTENSION_PTR_SET(&(class_inst -> ux_slave_class_thread), class_inst)

        /* Store the number of LUN declared.  */
        storage -> ux_slave_class_storage_number_lun = storage_parameter -> ux_slave_class_storage_parameter_number_lun;

        /* Copy each individual LUN parameters.  */
        for (lun_index = 0; lun_index < storage -> ux_slave_class_storage_number_lun; lun_index++)
        {

            /* Check block length size. */
            if (storage_parameter -> ux_slave_class_storage_parameter_lun[lun_index].ux_slave_class_storage_media_block_length > UX_SLAVE_CLASS_STORAGE_BUFFER_SIZE)
            {
                /* Cannot proceed.  */
                status = (UX_MEMORY_INSUFFICIENT);
                break;
            }

            /* Store all the application parameter information about the media.  */
            storage -> ux_slave_class_storage_lun[lun_index].ux_slave_class_storage_media_last_lba       = storage_parameter -> ux_slave_class_storage_parameter_lun[lun_index].ux_slave_class_storage_media_last_lba;
            storage -> ux_slave_class_storage_lun[lun_index].ux_slave_class_storage_media_block_length   = storage_parameter -> ux_slave_class_storage_parameter_lun[lun_index].ux_slave_class_storage_media_block_length;
            storage -> ux_slave_class_storage_lun[lun_index].ux_slave_class_storage_media_type           = storage_parameter -> ux_slave_class_storage_parameter_lun[lun_index].ux_slave_class_storage_media_type;
            storage -> ux_slave_class_storage_lun[lun_index].ux_slave_class_storage_media_removable_flag = storage_parameter -> ux_slave_class_storage_parameter_lun[lun_index].ux_slave_class_storage_media_removable_flag;
            storage -> ux_slave_class_storage_lun[lun_index].ux_slave_class_storage_media_read_only_flag = storage_parameter -> ux_slave_class_storage_parameter_lun[lun_index].ux_slave_class_storage_media_read_only_flag;
            storage -> ux_slave_class_storage_lun[lun_index].ux_slave_class_storage_media_read           = storage_parameter -> ux_slave_class_storage_parameter_lun[lun_index].ux_slave_class_storage_media_read;
            storage -> ux_slave_class_storage_lun[lun_index].ux_slave_class_storage_media_flush          = storage_parameter -> ux_slave_class_storage_parameter_lun[lun_index].ux_slave_class_storage_media_flush;
            storage -> ux_slave_class_storage_lun[lun_index].ux_slave_class_storage_media_write          = storage_parameter -> ux_slave_class_storage_parameter_lun[lun_index].ux_slave_class_storage_media_write;
            storage -> ux_slave_class_storage_lun[lun_index].ux_slave_class_storage_media_status         = storage_parameter -> ux_slave_class_storage_parameter_lun[lun_index].ux_slave_class_storage_media_status;
            storage -> ux_slave_class_storage_lun[lun_index].ux_slave_class_storage_media_notification   = storage_parameter -> ux_slave_class_storage_parameter_lun[lun_index].ux_slave_class_storage_media_notification;
        }

        /* If it's OK, complete it.  */
        if (status == UX_SUCCESS)
        {

            /* Store the start and stop signals if needed by the application.  */
            storage -> ux_slave_class_storage_instance_activate = storage_parameter -> ux_slave_class_storage_instance_activate;
            storage -> ux_slave_class_storage_instance_deactivate = storage_parameter -> ux_slave_class_storage_instance_deactivate;

            /* Store the vendor id, product id, product revision and product serial.  */
            if (storage_parameter -> ux_slave_class_storage_parameter_vendor_id)
                storage -> ux_slave_class_storage_vendor_id = storage_parameter -> ux_slave_class_storage_parameter_vendor_id;
            else
                storage -> ux_slave_class_storage_vendor_id = _ux_system_slave_class_storage_vendor_id;

            if (storage_parameter -> ux_slave_class_storage_parameter_product_id)
                storage -> ux_slave_class_storage_product_id = storage_parameter -> ux_slave_class_storage_parameter_product_id;
            else
                storage -> ux_slave_class_storage_product_id = _ux_system_slave_class_storage_product_id;

            if (storage_parameter -> ux_slave_class_storage_parameter_product_rev)
                storage -> ux_slave_class_storage_product_rev = storage_parameter -> ux_slave_class_storage_parameter_product_rev;
            else
                storage -> ux_slave_class_storage_product_rev = _ux_system_slave_class_storage_product_rev;

            if (storage_parameter -> ux_slave_class_storage_parameter_product_serial)
                storage -> ux_slave_class_storage_product_serial = storage_parameter -> ux_slave_class_storage_parameter_product_serial;
            else
                storage -> ux_slave_class_storage_product_serial = _ux_system_slave_class_storage_product_serial;

            /* Save the address of the STORAGE instance inside the STORAGE container.  */
            class_inst -> ux_slave_class_instance = (VOID *) storage;

            return(UX_SUCCESS);
        }

        /* Free thread resources.  */
        _ux_device_thread_delete(&class_inst -> ux_slave_class_thread);
    }

#if !defined(UX_DEVICE_STANDALONE)
    if (class_inst -> ux_slave_class_thread_stack != UX_NULL)
        _ux_utility_memory_free(&class_inst -> ux_slave_class_thread_stack);
#endif

    /* Free instance.  */
    _ux_utility_memory_free(storage);

    /* Return completion status.  */
    return(status);
}

