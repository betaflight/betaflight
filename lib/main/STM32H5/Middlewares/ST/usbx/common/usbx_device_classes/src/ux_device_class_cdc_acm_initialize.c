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
#include "ux_device_class_cdc_acm.h"
#include "ux_device_stack.h"


#if UX_OVERFLOW_CHECK_MULC_ULONG(UX_THREAD_STACK_SIZE, 2)
#error UX_THREAD_STACK_SIZE too large
#endif

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_cdc_acm_initialize                 PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function initializes the USB CDC device.                       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Pointer to cdc_acm command    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_allocate           Allocate memory               */ 
/*    _ux_utility_memory_free               Free memory                   */ 
/*    _ux_utility_mutex_create              Create mutex                  */ 
/*    _ux_device_mutex_delete               Delete mutex                  */ 
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
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added macro to disable      */
/*                                            transmission support,       */
/*                                            moved transmission resource */
/*                                            allocate to here (init),    */
/*                                            resulting in version 6.1.6  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_cdc_acm_initialize(UX_SLAVE_CLASS_COMMAND *command)
{
                                          
UX_SLAVE_CLASS_CDC_ACM                  *cdc_acm;
UX_SLAVE_CLASS_CDC_ACM_PARAMETER        *cdc_acm_parameter;
UX_SLAVE_CLASS                          *class_ptr;
#if !defined(UX_DEVICE_STANDALONE)
UINT                                    status;
#endif

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Create an instance of the device cdc_acm class.  */
    cdc_acm =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_SLAVE_CLASS_CDC_ACM));

    /* Check for successful allocation.  */
    if (cdc_acm == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Save the address of the CDC instance inside the CDC container.  */
    class_ptr -> ux_slave_class_instance = (VOID *) cdc_acm;

    /* Get the pointer to the application parameters for the cdc_acm class.  */
    cdc_acm_parameter =  command -> ux_slave_class_command_parameter;

    /* Store the start and stop signals if needed by the application.  */
    cdc_acm -> ux_slave_class_cdc_acm_parameter.ux_slave_class_cdc_acm_instance_activate = cdc_acm_parameter -> ux_slave_class_cdc_acm_instance_activate;
    cdc_acm -> ux_slave_class_cdc_acm_parameter.ux_slave_class_cdc_acm_instance_deactivate = cdc_acm_parameter -> ux_slave_class_cdc_acm_instance_deactivate;
    cdc_acm -> ux_slave_class_cdc_acm_parameter.ux_slave_class_cdc_acm_parameter_change = cdc_acm_parameter -> ux_slave_class_cdc_acm_parameter_change;

#if !defined(UX_DEVICE_STANDALONE)

    /* Create the Mutex for each endpoint as multiple threads cannot access each pipe at the same time.  */
    status =  _ux_utility_mutex_create(&cdc_acm -> ux_slave_class_cdc_acm_endpoint_in_mutex, "ux_slave_class_cdc_acm_in_mutex");

    /* Check Mutex creation error.  */
    if(status != UX_SUCCESS)
    {

        /* Free the resources.  */
        _ux_utility_memory_free(cdc_acm);
        
        /* Return fatal error.  */
        return(UX_MUTEX_ERROR);
    }        

    /* Out Mutex. */
    status =  _ux_utility_mutex_create(&cdc_acm -> ux_slave_class_cdc_acm_endpoint_out_mutex, "ux_slave_class_cdc_acm_out_mutex");

    /* Check Mutex creation error.  */
    if(status != UX_SUCCESS)
    {

        /* Delete the endpoint IN mutex.  */
        _ux_device_mutex_delete(&cdc_acm -> ux_slave_class_cdc_acm_endpoint_in_mutex);

        /* Free the resources.  */
        _ux_utility_memory_free(cdc_acm);
        
        /* Return fatal error.  */
        return(UX_MUTEX_ERROR);
    }        

#endif

    /* Update the line coding fields with default values.  */
    cdc_acm -> ux_slave_class_cdc_acm_baudrate  =  UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_BAUDRATE;
    cdc_acm -> ux_slave_class_cdc_acm_stop_bit  =  UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_STOP_BIT;
    cdc_acm -> ux_slave_class_cdc_acm_parity    =  UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARITY;
    cdc_acm -> ux_slave_class_cdc_acm_data_bit  =  UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_DATA_BIT;

#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE

#if defined(UX_DEVICE_STANDALONE)

    /* Set task function.  */
    class_ptr -> ux_slave_class_task_function = _ux_device_class_cdc_acm_tasks_run;
#else

    /* We need to prepare the 2 threads for sending and receiving.  */
    /* Allocate some memory for the bulk out and in thread stack. */
    cdc_acm -> ux_slave_class_cdc_acm_bulkout_thread_stack =  
            _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_THREAD_STACK_SIZE * 2);

    /* Check for successful allocation.  */
    if (cdc_acm -> ux_slave_class_cdc_acm_bulkout_thread_stack == UX_NULL)

        /* Return the status to the caller.  */
        status = (UX_MEMORY_INSUFFICIENT);

    /* If success, go on to create event flags.  */
    if (status == UX_SUCCESS)
    {

        /* Allocate some memory for the bulk in thread stack. */
        cdc_acm -> ux_slave_class_cdc_acm_bulkin_thread_stack =
            cdc_acm -> ux_slave_class_cdc_acm_bulkout_thread_stack + UX_THREAD_STACK_SIZE;

        /* Create a event flag group for the cdc_acm class to synchronize with the application writing event .  */
        status =  _ux_utility_event_flags_create(
                        &cdc_acm -> ux_slave_class_cdc_acm_event_flags_group,
                        "ux_device_class_cdc_acm_event_flag");

        /* Check status.  */
        if (status != UX_SUCCESS)
        {
            status = (UX_EVENT_ERROR);
        }
    }

    /* If success, go on to create bulkin thread.  */
    if (status == UX_SUCCESS)
    {

        /* Bulk endpoint treatment needs to be running in a different thread. So start
            a new thread. We pass a pointer to the cdc_acm instance to the new thread.  This thread
            does not start until we have a instance of the class. */
        status =  _ux_utility_thread_create(
                    &cdc_acm -> ux_slave_class_cdc_acm_bulkin_thread,
                    "ux_slave_class_cdc_acm_bulkin_thread", 
                    _ux_device_class_cdc_acm_bulkin_thread,
                    (ULONG) (ALIGN_TYPE) cdc_acm,
                    (VOID *) cdc_acm -> ux_slave_class_cdc_acm_bulkin_thread_stack,
                    UX_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                    UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);

        /* Check the creation of this thread.  */
        if (status != UX_SUCCESS)
        {
            status = (UX_THREAD_ERROR);
        }
        else
        {
            UX_THREAD_EXTENSION_PTR_SET(
                &(cdc_acm -> ux_slave_class_cdc_acm_bulkin_thread), cdc_acm)
        }
    }

    /* If success, go on to create bulkout thread.  */
    if (status == UX_SUCCESS)
    {

        /* Bulk endpoint treatment needs to be running in a different thread. So start
            a new thread. We pass a pointer to the cdc_acm instance to the new thread.  This thread
            does not start until we have a instance of the class. */
        status =  _ux_utility_thread_create(
                    &cdc_acm -> ux_slave_class_cdc_acm_bulkout_thread,
                    "ux_slave_class_cdc_acm_bulkout_thread", 
                    _ux_device_class_cdc_acm_bulkout_thread,
                    (ULONG) (ALIGN_TYPE) cdc_acm,
                    (VOID *) cdc_acm -> ux_slave_class_cdc_acm_bulkout_thread_stack,
                    UX_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                    UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);

        /* Check the creation of this thread.  */
        if (status != UX_SUCCESS)
        {
            status = (UX_THREAD_ERROR);
        }
        else
        {
            UX_THREAD_EXTENSION_PTR_SET(
                &(cdc_acm -> ux_slave_class_cdc_acm_bulkout_thread), cdc_acm)
        }
    }

    /* Check error.  */
    if (status != UX_SUCCESS)
    {

        /* Free resources and return error.  */
        if (cdc_acm -> ux_slave_class_cdc_acm_bulkin_thread.tx_thread_id)
            _ux_utility_thread_delete(&cdc_acm -> ux_slave_class_cdc_acm_bulkin_thread);
        if (cdc_acm -> ux_slave_class_cdc_acm_event_flags_group.tx_event_flags_group_id)
            _ux_utility_event_flags_delete(&cdc_acm -> ux_slave_class_cdc_acm_event_flags_group);
        if (cdc_acm -> ux_slave_class_cdc_acm_bulkout_thread_stack)
            _ux_utility_memory_free(cdc_acm -> ux_slave_class_cdc_acm_bulkout_thread_stack);
        _ux_device_mutex_delete(&cdc_acm -> ux_slave_class_cdc_acm_endpoint_in_mutex);
        _ux_device_mutex_delete(&cdc_acm -> ux_slave_class_cdc_acm_endpoint_out_mutex);
        _ux_utility_memory_free(cdc_acm);
        return(status);
    }

#endif
#endif

    /* Return completion status.  */
    return(UX_SUCCESS);
}

