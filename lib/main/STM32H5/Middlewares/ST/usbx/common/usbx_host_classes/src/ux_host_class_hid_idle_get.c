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
/*    _ux_host_class_hid_idle_get                         PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs a GET_IDLE to the HID device.                */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid                                   Pointer to HID class          */ 
/*    idle_time                             Idle time                     */ 
/*    report_id                             Report ID                     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_class_instance_verify  Verify instance is valid      */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_host_semaphore_get                Get protection semaphore      */ 
/*    _ux_host_semaphore_put                Release protection semaphore  */ 
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
/*                                            protected default control   */
/*                                            endpoint before using it,   */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_idle_get(UX_HOST_CLASS_HID *hid, USHORT *idle_time, USHORT report_id)
{
#if defined(UX_HOST_STANDALONE)
UX_INTERRUPT_SAVE_AREA
#endif
UX_ENDPOINT     *control_endpoint;
UX_TRANSFER     *transfer_request;
UCHAR           *idle_byte;
UINT            status;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HID_IDLE_GET, hid, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_hid_name, (VOID *) hid) != UX_SUCCESS)
    {        

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &hid -> ux_host_class_hid_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Protect thread reentry to this instance.  */
#if defined(UX_HOST_STANDALONE)
    UX_DISABLE
    if (hid -> ux_host_class_hid_flags & UX_HOST_CLASS_HID_FLAG_LOCK)
    {
        UX_RESTORE
        return(UX_BUSY);
    }
    hid -> ux_host_class_hid_flags &= ~UX_HOST_CLASS_HID_FLAG_LOCK;
    UX_RESTORE
#else

    status =  _ux_host_semaphore_get(&hid -> ux_host_class_hid_semaphore, UX_WAIT_FOREVER);
    if (status != UX_SUCCESS)
    
        /* Return error.  */
        return(status);
#endif

    /* Need to allocate memory for the idle byte.  */
    idle_byte =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 1);
    if (idle_byte == UX_NULL)
    {

        /* Unprotect thread reentry to this instance.  */
        _ux_host_class_hid_unlock(hid);

        /* Return error status.  */
        return(UX_MEMORY_INSUFFICIENT);
    }

    /* Protect the control endpoint semaphore here.  It will be unprotected in the 
       transfer request function.  */
#if defined(UX_HOST_STANDALONE)
    UX_DISABLE
    if (hid -> ux_host_class_hid_device -> ux_device_flags & UX_DEVICE_FLAG_LOCK)
    {

        /* Free allocated return busy.  */
        _ux_utility_memory_free(idle_byte);
        hid -> ux_host_class_hid_flags &= ~UX_HOST_CLASS_HID_FLAG_LOCK;
        UX_RESTORE
        return(UX_BUSY);
    }
    hid -> ux_host_class_hid_device -> ux_device_flags |= UX_DEVICE_FLAG_LOCK;
    transfer_request -> ux_transfer_request_flags |= UX_TRANSFER_FLAG_AUTO_DEVICE_UNLOCK;
    UX_TRANSFER_STATE_RESET(transfer_request);
    UX_RESTORE
#else
    status =  _ux_host_semaphore_get(&hid -> ux_host_class_hid_device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);

    /* Check for status.  */
    if (status != UX_SUCCESS)
    {

        /* Something went wrong. */

        /* Free allocated memory.  */
        _ux_utility_memory_free(idle_byte);

        /* Unprotect thread reentry to this instance.  */
        _ux_host_semaphore_put(&hid -> ux_host_class_hid_semaphore);

        return(status);
    }
#endif

    /* Create a transfer request for the GET_IDLE request.  */
    transfer_request -> ux_transfer_request_data_pointer =      idle_byte;
    transfer_request -> ux_transfer_request_requested_length =  1;
    transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_HID_GET_IDLE;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
    transfer_request -> ux_transfer_request_value =             report_id;
    transfer_request -> ux_transfer_request_index =             hid -> ux_host_class_hid_interface -> ux_interface_descriptor.bInterfaceNumber;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

#if defined(UX_HOST_STANDALONE)
    if (!(transfer_request -> ux_transfer_request_flags & UX_TRANSFER_FLAG_AUTO_WAIT))
    {
        hid -> ux_host_class_hid_allocated = idle_byte;
        return(status);
    }
#endif

    /* Check for correct transfer and for the entire descriptor returned.  */
    if ((status == UX_SUCCESS) && (transfer_request -> ux_transfer_request_actual_length == 1))
        *idle_time =  (USHORT) *idle_byte;    

    /* Free used resources.  */
    _ux_utility_memory_free(idle_byte);

    /* Unprotect thread reentry to this instance.  */
#if defined(UX_HOST_STANDALONE)
    _ux_host_class_hid_unlock(hid);
#else
    _ux_host_semaphore_put(&hid -> ux_host_class_hid_semaphore);
#endif

    /* Return the completion status.  */
    return(status);
}
