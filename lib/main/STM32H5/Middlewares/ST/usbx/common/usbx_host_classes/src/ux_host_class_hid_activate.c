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
/**   HID Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_activate                         PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs the enumeration of the HID class.            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Pointer to command            */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_hid_client_search      HID client search             */ 
/*    _ux_host_class_hid_configure          Configure HID                 */ 
/*    _ux_host_class_hid_descriptor_parse   Parse descriptor              */ 
/*    _ux_host_class_hid_interrupt_endpoint_search  Search endpoint       */ 
/*    _ux_host_class_hid_instance_clean     Clean up instance resources   */
/*    _ux_host_stack_class_instance_create  Create class instance         */ 
/*    _ux_host_stack_class_instance_destroy Destroy class instance        */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Free memory                   */
/*    _ux_host_semaphore_create             Create semaphore              */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HID Class                                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_activate(UX_HOST_CLASS_COMMAND  *command)
{

UX_INTERFACE        *interface_ptr;
UX_HOST_CLASS_HID   *hid;
UINT                status;


    /* The HID is always activated by the interface descriptor and not the
       device descriptor.  */
    interface_ptr =  (UX_INTERFACE *) command -> ux_host_class_command_container;
    
    /* Instantiate this HID class */
    hid =  _ux_utility_memory_allocate(UX_NO_ALIGN,  UX_REGULAR_MEMORY,sizeof(UX_HOST_CLASS_HID));
    if (hid == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);
        
    /* Store the class container into this instance.  */
    hid -> ux_host_class_hid_class =  command -> ux_host_class_command_class_ptr;

    /* Store the interface container into the HID class instance.  */
    hid -> ux_host_class_hid_interface =  interface_ptr;

    /* Store the device container into the HID class instance.  */
    hid -> ux_host_class_hid_device =  interface_ptr -> ux_interface_configuration -> ux_configuration_device;

    /* This instance of the device must also be stored in the interface container.  */
    interface_ptr -> ux_interface_class_instance =  (VOID *) hid;

    /* Create this class instance.  */
    _ux_host_stack_class_instance_create(command -> ux_host_class_command_class_ptr, (VOID *) hid);

#if defined(UX_HOST_STANDALONE)

    /* Set class tasks function.  */
    hid -> ux_host_class_hid_class -> ux_host_class_task_function = _ux_host_class_hid_tasks_run;

    /* Set activate state to first step.  */
    hid -> ux_host_class_hid_enum_state = UX_STATE_WAIT;

    status = UX_SUCCESS;
    return(status);
#else

    /* Configure the HID.  */
    status =  _ux_host_class_hid_configure(hid);

    /* If configure is done success, goes on. */
    if (status == UX_SUCCESS)
    {

        /* Get the HID descriptor and parse it.  */
        status =  _ux_host_class_hid_descriptor_parse(hid);
    }

    /* If HID descriptor parse is done success, goes on. */
    if (status == UX_SUCCESS)
    {

        /* Search the HID interrupt endpoint but do not start it.  */
        status =  _ux_host_class_hid_interrupt_endpoint_search(hid);     
    }

    /* If HID interrupt endpoint is found, goes on. */
    if (status == UX_SUCCESS)
    {

        /* Create the semaphore to protect multiple threads from accessing the same
        storage instance.  */
        status =  _ux_host_semaphore_create(&hid -> ux_host_class_hid_semaphore, "ux_host_class_hid_semaphore", 1);

        if (status == UX_SUCCESS)
        {

            /* If all is fine, try to locate the HID client for this HID device.  */
            _ux_host_class_hid_client_search(hid);

            /* Mark the HID class as live now.  */
            hid -> ux_host_class_hid_state =  UX_HOST_CLASS_INSTANCE_LIVE;

            /* We may need to inform the application if a function has been programmed in the system structure.  */
            if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
            {

                /* Call system change function.  */
                _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, hid -> ux_host_class_hid_class, (VOID *) hid);
            }

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HID_ACTIVATE, hid, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

            /* If trace is enabled, register this object.  */
            UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, hid, 0, 0, 0)

            /* Return completion code.  */
            return(status);    
        }
        else
        {
            status = UX_SEMAPHORE_ERROR;
        }
    }

    /* Clean interrupt endpoint.  */
    if (hid -> ux_host_class_hid_interrupt_endpoint &&
        hid -> ux_host_class_hid_interrupt_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer)
        _ux_utility_memory_free(hid -> ux_host_class_hid_interrupt_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer);

    /* Clean instance. */
    _ux_host_class_hid_instance_clean(hid);

    /* Error, destroy the class instance and return error code. */
    _ux_host_stack_class_instance_destroy(hid -> ux_host_class_hid_class, (VOID *) hid);

    /* Unmount instance. */
    interface_ptr -> ux_interface_class_instance = UX_NULL;

    /* Free instance. */
    _ux_utility_memory_free(hid);

    /* Return error code. */
    return(status);

#endif
}

