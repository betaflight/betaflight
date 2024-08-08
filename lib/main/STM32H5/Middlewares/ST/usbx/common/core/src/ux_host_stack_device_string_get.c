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
/**   Host Stack                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_stack_device_string_get                    PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function obtains the device string descriptor.                 */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    device                                Pointer to device instance    */
/*    descriptor_buffer                     Pointer to a buffer to fill   */
/*                                          LANGID or STRING descriptor   */
/*    length                                Length of buffer              */
/*    language_id                           0 to obtain LANGID descriptor */
/*                                          valid language ID to obtain   */
/*                                          string descriptor             */
/*    string_index                          Index of the string           */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_stack_transfer_request       Process transfer request      */
/*    _ux_utility_semaphore_get             Get Semaphore                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  02-02-2021     Chaoqiong Xiao           Initial Version 6.1.4         */
/*  06-02-2021     Chaoqiong Xiao           Modified comment(s), and      */
/*                                            removed unnecessary header, */
/*                                            resulting in version 6.1.7  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_device_string_get(UX_DEVICE *device, UCHAR *descriptor_buffer, ULONG length, ULONG language_id, ULONG string_index)
{
#if defined(UX_HOST_STANDALONE)
UX_INTERRUPT_SAVE_AREA
#endif
UX_ENDPOINT     *control_endpoint;
UX_TRANSFER     *transfer_request;
UINT            status;


    /* Do a sanity check on the device handle.  */
    if (device -> ux_device_handle != (ULONG) (ALIGN_TYPE) device)
    {
        
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_DEVICE_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DEVICE_HANDLE_UNKNOWN, device, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_DEVICE_HANDLE_UNKNOWN);
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_DEVICE_STRING_GET, device, descriptor_buffer, length, (language_id << 16) | string_index, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

#if defined(UX_HOST_STANDALONE)
    UX_DISABLE
    if ((device -> ux_device_flags & UX_DEVICE_FLAG_LOCK) ||
        (transfer_request -> ux_transfer_request_flags & UX_TRANSFER_FLAG_LOCK))
    {
        UX_RESTORE
        return(UX_BUSY);
    }
    device -> ux_device_flags |= UX_DEVICE_FLAG_LOCK;
    transfer_request -> ux_transfer_request_flags |=
                (UX_TRANSFER_FLAG_LOCK | UX_TRANSFER_FLAG_AUTO_DEVICE_UNLOCK);
    UX_RESTORE
#else

    /* Protect the control endpoint semaphore here.  It will be unprotected in the
       transfer request function.  */
    status =  _ux_host_semaphore_get(&device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);

    /* Check for status.  */
    if (status != UX_SUCCESS)
        return(UX_SEMAPHORE_ERROR);
#endif

    /* Create a transfer request for the GET_DEVICE_ID request.  */
    transfer_request -> ux_transfer_request_data_pointer =      descriptor_buffer;
    transfer_request -> ux_transfer_request_requested_length =  length;
    transfer_request -> ux_transfer_request_function =          UX_GET_DESCRIPTOR;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value =             (UX_STRING_DESCRIPTOR_ITEM << 8) | string_index;
    transfer_request -> ux_transfer_request_index =             (language_id);

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Return completion status.  */
    return(status);
}
