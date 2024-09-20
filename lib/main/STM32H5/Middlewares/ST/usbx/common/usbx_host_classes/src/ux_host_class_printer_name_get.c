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


#if !defined(UX_HOST_STANDALONE)
static inline
#else
VOID _ux_host_class_printer_name_parse(UX_HOST_CLASS_PRINTER *printer, UCHAR *descriptor_buffer, ULONG descriptor_length);
#endif
VOID _ux_host_class_printer_name_parse(UX_HOST_CLASS_PRINTER *printer, UCHAR *descriptor_buffer, ULONG descriptor_length)
{
ULONG           length;
UCHAR *         printer_name_buffer;
UCHAR *         printer_name;
UINT            printer_name_length;

    /* Cannot use descriptor buffer, so copy it to 1284 buffer.  */
    printer_name_buffer =  descriptor_buffer;

    /* Retrieve the printer name which is a USHORT at the beginning of the returned buffer.  */
    length =  (USHORT)_ux_utility_short_get_big_endian(printer_name_buffer);
    if (length < descriptor_length)
        descriptor_length = length;

    /* Point the name buffer after the length.  */
    printer_name_buffer +=  2;

    /* Parse the name for a tag which is in the form DES: or DESCRIPTOR:  */
    while (descriptor_length != 0)
    {

        if (descriptor_length > 12)
        {

            /* Compare the current pointer position with the DESCRIPTOR: tag.  */
            if (_ux_utility_memory_compare(printer_name_buffer, UX_HOST_CLASS_PRINTER_TAG_DESCRIPTION, 12) == UX_SUCCESS)
            {

                printer_name_buffer +=  12;
                descriptor_length =  (USHORT)(descriptor_length - 12);
                break;
            }
        }

        if (descriptor_length > 4)
        {

            /* Compare the current pointer position with the DES: tag.  */
            if (_ux_utility_memory_compare(printer_name_buffer, UX_HOST_CLASS_PRINTER_TAG_DES, 4) == UX_SUCCESS)
            {

                printer_name_buffer +=  4;
                descriptor_length = (USHORT)(descriptor_length - 4);
                break;
            }
        }
        
        /* And reduce the remaining length by 1.  */
        descriptor_length--;

        /* Increment the descriptor pointer.  */
        printer_name_buffer++;
    }

    /* If the length remaining is 0, we have not found the descriptor tag we wanted.  */
    if (descriptor_length == 0)
    {

        /* Use the generic USB printer name.  */
        _ux_utility_memory_copy(printer -> ux_host_class_printer_name, UX_HOST_CLASS_PRINTER_GENERIC_NAME, 11); /* Use case of memcpy is verified. */
    }
    else
    {

        /* We have found a tag and the name is right after delimited by ; or a 0.  */
        printer_name =  printer -> ux_host_class_printer_name;
        printer_name_length =  UX_HOST_CLASS_PRINTER_NAME_LENGTH;

        /* Parse the name and only copy the name until the max length is reached
            or the delimiter is reached.  */
        while ((descriptor_length--) && (printer_name_length--))
        {

            /* Check for the delimiter.  */
            if((*printer_name_buffer == 0) || (*printer_name_buffer == ';'))
                break;
            else
                /* Copy the name of the printer to the printer instance character
                    by character.  */
                *printer_name++ =  *printer_name_buffer++;
        }
    }
}

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_printer_name_get                     PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function obtains the printer name. The name is used by the     */
/*    application layer to identify the printer and loads its handler.    */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    printer                               Pointer to printer class      */ 
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
/*    Printer Class                                                       */ 
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
/*  02-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            supported interface other   */
/*                                            than number zero,           */
/*                                            resulting in version 6.1.4  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_printer_name_get(UX_HOST_CLASS_PRINTER *printer)
{

UCHAR *         descriptor_buffer;
UX_INTERFACE    *interface_ptr;
UX_ENDPOINT     *control_endpoint;
UX_TRANSFER     *transfer_request;
#if !defined(UX_HOST_STANDALONE)
UINT            status;
#endif

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PRINTER_NAME_GET, printer, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &printer -> ux_host_class_printer_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Need to allocate memory for the 1284 descriptor.  */
    descriptor_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_PRINTER_DESCRIPTOR_LENGTH);
    if(descriptor_buffer == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Need interface for wIndex.  */
    interface_ptr = printer -> ux_host_class_printer_interface;

    /* Create a transfer request for the GET_DEVICE_ID request.  */
    transfer_request -> ux_transfer_request_data_pointer =      descriptor_buffer;
    transfer_request -> ux_transfer_request_requested_length =  UX_HOST_CLASS_PRINTER_DESCRIPTOR_LENGTH;
    transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_PRINTER_GET_DEVICE_ID;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
    transfer_request -> ux_transfer_request_value =             0; /* Do not support multiple configuration for now.  */
    transfer_request -> ux_transfer_request_index =             (interface_ptr -> ux_interface_descriptor.bInterfaceNumber  << 8) |
                                                                (interface_ptr -> ux_interface_descriptor.bAlternateSetting     );

#if defined(UX_HOST_STANDALONE)
    printer -> ux_host_class_printer_allocated = descriptor_buffer;
    UX_TRANSFER_STATE_RESET(transfer_request);
    return(UX_SUCCESS);
#else

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check for correct transfer. We do not check for entire length as it may vary.  */
    if(status == UX_SUCCESS)
    {

        _ux_host_class_printer_name_parse(printer, descriptor_buffer,
                                    UX_HOST_CLASS_PRINTER_DESCRIPTOR_LENGTH);
    }

    /* Free all used resources.  */
    _ux_utility_memory_free(descriptor_buffer);

    /* Return completion status.  */
    return(status);

#endif
}

