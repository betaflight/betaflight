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
/**   Printer Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_printer.h"
#include "ux_host_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_printer_device_id_get                PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function obtains the printer IEEE 1284 device ID string        */
/*    (including length in the first two bytes in big endian format).     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    printer                               Pointer to printer class      */
/*    descriptor_buffer                     Pointer to a buffer to fill   */
/*                                          IEEE 1284 device ID string    */
/*                                          (including length in the      */
/*                                          first two bytes in BE format) */
/*    length                                Length of buffer in bytes     */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_stack_transfer_request       Process transfer request      */
/*    _ux_utility_memory_allocate           Allocate memory block         */
/*    _ux_utility_memory_compare            Compare memory block          */
/*    _ux_utility_memory_copy               Copy memory block             */
/*    _ux_utility_memory_free               Free memory block             */
/*    _ux_utility_short_get_big_endian      Get 16-bit value              */
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_printer_device_id_get(UX_HOST_CLASS_PRINTER *printer, UCHAR *descriptor_buffer, ULONG length)
{
#if defined(UX_HOST_STANDALONE)
UX_INTERRUPT_SAVE_AREA
#endif
UX_INTERFACE         *interface_ptr;
UX_ENDPOINT          *control_endpoint;
UX_TRANSFER          *transfer_request;
UINT                 status;


    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PRINTER_DEVICE_ID_GET, printer, descriptor_buffer, length, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Protect thread reentry to this instance.  */
#if defined(UX_HOST_STANDALONE)
    UX_DISABLE
    if ((printer -> ux_host_class_printer_flags & UX_HOST_CLASS_PRINTER_FLAG_LOCK) ||
        (printer -> ux_host_class_printer_device -> ux_device_flags & UX_DEVICE_FLAG_LOCK))
    {
        UX_RESTORE
        return(UX_BUSY);
    }
    printer -> ux_host_class_printer_flags |= UX_HOST_CLASS_PRINTER_FLAG_LOCK;
    printer -> ux_host_class_printer_device -> ux_device_flags |= UX_DEVICE_FLAG_LOCK;
    UX_RESTORE
#else

    status =  _ux_host_semaphore_get(&printer -> ux_host_class_printer_semaphore, UX_WAIT_FOREVER);
    if (status != UX_SUCCESS)
        return(status);

    /* Protect the control endpoint semaphore here.  It will be unprotected in the
       transfer request function.  */
    status =  _ux_host_semaphore_get(&printer -> ux_host_class_printer_device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);

    /* Check for status.  */
    if (status != UX_SUCCESS)
    {
        _ux_host_semaphore_put(&printer -> ux_host_class_printer_semaphore);
        return(status);
    }
#endif

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &printer -> ux_host_class_printer_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Need interface for wIndex.  */
    interface_ptr = printer -> ux_host_class_printer_interface;

    /* Create a transfer request for the GET_DEVICE_ID request.  */
    transfer_request -> ux_transfer_request_data_pointer =      descriptor_buffer;
    transfer_request -> ux_transfer_request_requested_length =  length;
    transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_PRINTER_GET_DEVICE_ID;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
    transfer_request -> ux_transfer_request_value =             0; /* Do not support multiple configuration for now.  */
    transfer_request -> ux_transfer_request_index =             (interface_ptr -> ux_interface_descriptor.bInterfaceNumber  << 8) |
                                                                (interface_ptr -> ux_interface_descriptor.bAlternateSetting     );

#if defined(UX_HOST_STANDALONE)

    /* Enable auto unlock device.  */
    transfer_request -> ux_transfer_request_flags |= UX_TRANSFER_FLAG_AUTO_DEVICE_UNLOCK;
#endif

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Unprotect thread reentry to this instance.  */
    _ux_host_class_printer_unlock(printer);

    /* Return completion status.  */
    return(status);
}
