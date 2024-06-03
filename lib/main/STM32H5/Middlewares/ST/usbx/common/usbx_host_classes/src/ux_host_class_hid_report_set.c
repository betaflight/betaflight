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


#if !defined(UX_HOST_STANDALONE)

/* Inline the function to let compiler optimize.  */
static inline
#else

/* Prototype the function to remove warnings.  */
UINT ux_host_class_hid_report_set_buffer_allocate(
        UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_CLIENT_REPORT *client_report,
        UCHAR **allocated);
#endif
UINT ux_host_class_hid_report_set_buffer_allocate(
        UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_CLIENT_REPORT *client_report,
        UCHAR **allocated)
{

UX_HOST_CLASS_HID_REPORT    *hid_report;
UCHAR                       *report_buffer;
UCHAR                       *current_report_buffer;

    /* Get the report pointer from the caller.  */
    hid_report =  client_report -> ux_host_class_hid_client_report;

    /* Ensure this is NOT an INPUT report.  */
    if (hid_report -> ux_host_class_hid_report_type == UX_HOST_CLASS_HID_REPORT_TYPE_INPUT)
    {

        /* Return error code.  */
        return(UX_HOST_CLASS_HID_REPORT_ERROR);
    }

    /* Get some memory for sending the report.  */
    if (hid_report -> ux_host_class_hid_report_id == 0)
        report_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, hid_report -> ux_host_class_hid_report_byte_length);
    else
        report_buffer =  _ux_utility_memory_allocate_add_safe(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, hid_report -> ux_host_class_hid_report_byte_length, 1);
    if (report_buffer == UX_NULL)
    {

        /* Return error code.  */
        return(UX_MEMORY_INSUFFICIENT);
    }

    /* Memorize the start of the real report buffer.  */
    current_report_buffer =  report_buffer;

    /* Check if there is a report ID to be inserted in front of the buffer.  */
    if (hid_report -> ux_host_class_hid_report_id != 0)
        *current_report_buffer++ =  (UCHAR)(hid_report -> ux_host_class_hid_report_id);

    /* The report is in the client's buffer. It may be raw or or decompressed. If decompressed,
       we need to create the report.  */
    if (client_report -> ux_host_class_hid_client_report_flags & UX_HOST_CLASS_HID_REPORT_RAW)
    {

        /* Ensure the user is not trying to overflow the report buffer.  */
        if (hid_report -> ux_host_class_hid_report_byte_length >= client_report -> ux_host_class_hid_client_report_length)
        {

            /* We have enough memory to store the raw buffer.  */
            _ux_utility_memory_copy(current_report_buffer, client_report -> ux_host_class_hid_client_report_buffer, hid_report -> ux_host_class_hid_report_byte_length); /* Use case of memcpy is verified. */
        }
        else
        {

            /* Free allocated buffer.  */
            _ux_utility_memory_free(report_buffer);

            /* Return error code.  */
            return(UX_HOST_CLASS_HID_REPORT_OVERFLOW);
        }
    }
    else
    {

        /* The report buffer has to be compressed.  */
        _ux_host_class_hid_report_compress(hid, client_report, current_report_buffer, client_report -> ux_host_class_hid_client_report_length);
    }

    /* Buffer is allocated and report data copied.  */
    *allocated = report_buffer;
    return(UX_SUCCESS);
}

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_report_set                       PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function sets a report (input/output/feature) to the device.   */
/*    The report can be either decompressed by the stack or raw.          */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid                                   Pointer to HID class          */ 
/*    client_report                         Pointer to client report      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_hid_report_compress    Compress HID report           */ 
/*    _ux_host_stack_class_instance_verify  Verify the instance is valid  */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_copy               Copy memory block             */ 
/*    _ux_utility_memory_free               Release memory block          */ 
/*    _ux_host_semaphore_get                Get protection semaphore      */ 
/*    _ux_host_semaphore_put                Release protection semaphore  */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*    HID Class                                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Xiuwen Cai, CQ Xiao      Modified comment(s),          */
/*                                            added interrupt OUT support,*/
/*                                            added standalone support,   */
/*                                            refined code sequence,      */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_report_set(UX_HOST_CLASS_HID *hid, UX_HOST_CLASS_HID_CLIENT_REPORT *client_report)
{
#if defined(UX_HOST_STANDALONE)
UINT                    status;
    do
    {
        status = _ux_host_class_hid_report_set_run(hid, client_report);
    } while(status == UX_STATE_WAIT || status == UX_STATE_LOCK);
    return(hid -> ux_host_class_hid_status);
#else

UX_ENDPOINT                 *control_endpoint;
UX_TRANSFER                 *transfer_request;
UCHAR                       *report_buffer;
UX_HOST_CLASS_HID_REPORT    *hid_report;
UINT                        status;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HID_REPORT_SET, hid, client_report, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_hid_name, (VOID *) hid) != UX_SUCCESS)
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Protect thread reentry to this instance.  */
    _ux_host_class_hid_lock_fail_return(hid);

    /* Get the report pointer from the caller.  */
    hid_report =  client_report -> ux_host_class_hid_client_report;
#if defined(UX_HOST_CLASS_HID_INTERRUPT_OUT_SUPPORT)

    /* Check if there is an interrupt OUT endpoint.  */
    if (hid -> ux_host_class_hid_interrupt_out_endpoint != UX_NULL)
    {

        /* Transfer the report by using the interrupt OUT endpoint.  */
        transfer_request = &hid -> ux_host_class_hid_interrupt_out_endpoint -> ux_endpoint_transfer_request;
    }
    else
    {
#endif
        /* We need to get the default control endpoint transfer request pointer.  */
        control_endpoint =  &hid -> ux_host_class_hid_device -> ux_device_control_endpoint;
        transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;
    
        /* Protect the control endpoint semaphore here.  It will be unprotected in the 
           transfer request function.  */
        status =  _ux_host_semaphore_get(&hid -> ux_host_class_hid_device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);
    
        /* Check for status.  */
        if (status != UX_SUCCESS)
        {
    
            /* Something went wrong. */
            /* Unprotect thread reentry to this instance.  */
            _ux_host_class_hid_unlock(hid);
            return(status);
        }
#if defined(UX_HOST_CLASS_HID_INTERRUPT_OUT_SUPPORT)
    }
#endif
    /* Build the report buffer for SET_REPORT request.  */
    status = ux_host_class_hid_report_set_buffer_allocate(hid, client_report, &report_buffer);
    if (status != UX_SUCCESS)
    {

        /* Unlock device control endpoint.  */
        _ux_host_semaphore_put(&hid -> ux_host_class_hid_device -> ux_device_protection_semaphore);

        /* Unprotect thread reentry to this instance.  */
        _ux_host_class_hid_unlock(hid);

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, status, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(status);
    }

    /* Create a transfer request for the SET_REPORT request.  */
    transfer_request -> ux_transfer_request_data_pointer =      report_buffer;
    transfer_request -> ux_transfer_request_requested_length =  hid_report -> ux_host_class_hid_report_byte_length;
    transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_HID_SET_REPORT;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_OUT | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
    transfer_request -> ux_transfer_request_value =             (UINT)((USHORT) hid_report -> ux_host_class_hid_report_id | (USHORT) hid_report -> ux_host_class_hid_report_type << 8);
    transfer_request -> ux_transfer_request_index =             hid -> ux_host_class_hid_interface -> ux_interface_descriptor.bInterfaceNumber;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);
#if defined(UX_HOST_CLASS_HID_INTERRUPT_OUT_SUPPORT)
    
    /* Check if interrupt OUT endpoint is used.  */
    if ((hid -> ux_host_class_hid_interrupt_out_endpoint != UX_NULL) && (status == UX_SUCCESS))
    {

        /* We need to wait for the completion of the transfer request.  */
        status =  _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore, UX_MS_TO_TICK(UX_HOST_CLASS_HID_REPORT_TRANSFER_TIMEOUT));
    }
#endif

    /* Check for correct transfer and the actual transfer length.  */
    if ((status != UX_SUCCESS) || (transfer_request -> ux_transfer_request_actual_length != hid_report -> ux_host_class_hid_report_byte_length))
    {
#if defined(UX_HOST_CLASS_HID_INTERRUPT_OUT_SUPPORT)

        /* Check if interrupt OUT endpoint is used.  */
        if ((hid -> ux_host_class_hid_interrupt_out_endpoint != UX_NULL) && (status != UX_SUCCESS))
        {

            /* We need to abort the transfer request.  */
            _ux_host_stack_transfer_request_abort(transfer_request);
        }
#endif

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_REPORT_ERROR);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_REPORT_ERROR, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

        status =  UX_HOST_CLASS_HID_REPORT_ERROR;
    }
    
    /* Free all resources.  */
    _ux_utility_memory_free(report_buffer);

    /* Unprotect thread reentry to this instance.  */
    _ux_host_class_hid_unlock(hid);
    
    /* Return the function status */
    return(status);
#endif
}

