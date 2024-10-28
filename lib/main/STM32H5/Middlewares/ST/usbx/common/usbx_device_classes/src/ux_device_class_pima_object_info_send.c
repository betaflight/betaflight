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
/**   Device Pima Class                                                   */
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
/*    _ux_device_class_pima_object_info_send              PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function receives an object info structure from the host       */ 
/*    before receiving the actual data.                                   */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                  Pointer to pima class         */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_request     Transfer request              */ 
/*    _ux_device_stack_endpoint_stall       Stall endpoint                */
/*    _ux_utility_long_put                  Put 32-bit value              */ 
/*    _ux_utility_short_put                 Put 32-bit value              */ 
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_copy               Copy memory                   */ 
/*    _ux_utility_memory_free               Free memory                   */ 
/*    _ux_utility_descriptor_parse          Parse descriptor              */
/*    _ux_device_class_pima_response_send   Send PIMA response            */
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
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            updated status handling,    */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_object_info_send(UX_SLAVE_CLASS_PIMA *pima, ULONG storage_id, ULONG parent_object_handle)
{

UINT                        status;
UX_SLAVE_TRANSFER           *transfer_request;
UX_SLAVE_CLASS_PIMA_OBJECT  *object;
UCHAR                       *object_info;
UCHAR                       *object_info_pointer;
ULONG                       unicode_string_length;
ULONG                       object_handle;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_OBJECT_INFO_SEND, pima, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &pima -> ux_device_class_pima_bulk_out_endpoint -> ux_slave_endpoint_transfer_request;
    
    /* Obtain memory for this object info. Use the transfer request pre-allocated memory.  */
    object_info =  transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Get the data payload.  */
    status =  _ux_device_stack_transfer_request(transfer_request, UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH, 
                                                    UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH);

    /* Check if there was an error. If so, stall the endpoint.  */
    if (status != UX_SUCCESS)
    {
    
        /* Stall the endpoint.  */
        _ux_device_stack_endpoint_stall(pima -> ux_device_class_pima_bulk_out_endpoint);
        
        /* Return the status.  */
        return(status);
        
    }

    /* Allocate some memory for the object.  */
    object =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_SLAVE_CLASS_PIMA_OBJECT));

    /* Check for successful allocation.  */
    if (object == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Allocate the device info pointer to the beginning of the dynamic object info field.  */
    object_info_pointer = object_info + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;

    /* Uncompress the object descriptor, at least the fixed part.  */
    _ux_utility_descriptor_parse(object_info_pointer,
                        _ux_system_class_pima_object_structure,
                        UX_DEVICE_CLASS_PIMA_OBJECT_ENTRIES,
                        (UCHAR *) object);

    /* Copy the object filename  field.  Point to the beginning of the object description string.  */
    object_info_pointer =  object_info_pointer + UX_DEVICE_CLASS_PIMA_OBJECT_VARIABLE_OFFSET;
    
    /* Get the unicode string length.  */
    unicode_string_length =  ((ULONG) *object_info_pointer * 2) + 1;

    /* Ensure there's enough space for this string.  */
    if (unicode_string_length > UX_DEVICE_CLASS_PIMA_UNICODE_MAX_LENGTH)

        /* Return overflow error.  */
        status =  UX_MEMORY_INSUFFICIENT;

    /* Is there enough space?  */
    if (status == UX_SUCCESS)
    {
        
        /* Copy that string into the object description field.  */
        _ux_utility_memory_copy(object -> ux_device_class_pima_object_filename, object_info_pointer, unicode_string_length); /* Use case of memcpy is verified. */

        /* Point to the next field.  */
        object_info_pointer += unicode_string_length;
        
        /* Get the unicode string length.  */
        unicode_string_length =  ((ULONG) *object_info_pointer  * 2) + 1;

        /* Ensure there's enough space for this string.  */
        if (unicode_string_length > UX_DEVICE_CLASS_PIMA_DATE_TIME_STRING_MAX_LENGTH)

            /* Return overflow error.  */
            status =  UX_MEMORY_INSUFFICIENT;
    }

    /* Is there enough space?  */
    if (status == UX_SUCCESS)
    {

        /* Copy that string into the capture date field.  */
        _ux_utility_memory_copy(object -> ux_device_class_pima_object_capture_date, object_info_pointer, unicode_string_length); /* Use case of memcpy is verified. */

        /* Point to the next field.  */
        object_info_pointer += unicode_string_length;
        
        /* Get the unicode string length.  */
        unicode_string_length =  ((ULONG) *object_info_pointer  * 2) + 1;

        /* Ensure there's enough space for this string.  */
        if (unicode_string_length > UX_DEVICE_CLASS_PIMA_DATE_TIME_STRING_MAX_LENGTH)

            /* Return overflow error.  */
            status =  UX_MEMORY_INSUFFICIENT;
    }

    /* Is there enough space?  */
    if (status == UX_SUCCESS)
    {

        /* Copy that string into the modification date field.  */
        _ux_utility_memory_copy(object -> ux_device_class_pima_object_modification_date, object_info_pointer, unicode_string_length); /* Use case of memcpy is verified. */

        /* Point to the next field.  */
        object_info_pointer += unicode_string_length;
        
        /* Get the unicode string length.  */
        unicode_string_length =  ((ULONG) *object_info_pointer  * 2) + 1;

        /* Ensure there's enough space for this string.  */
        if (unicode_string_length > UX_DEVICE_CLASS_PIMA_DATE_TIME_STRING_MAX_LENGTH)

            /* Return overflow error.  */
            status =  UX_MEMORY_INSUFFICIENT;
    }

    /* Is there enough space?  */
    if (status == UX_SUCCESS)
    {

        /* Copy that string into the keywords field.  */
        _ux_utility_memory_copy(object -> ux_device_class_pima_object_keywords, object_info_pointer, unicode_string_length); /* Use case of memcpy is verified. */
        
        /* Reset the rest of the other parameters.  */
        object -> ux_device_class_pima_object_state            =  0;
        object -> ux_device_class_pima_object_offset           =  0;
        object -> ux_device_class_pima_object_transfer_status  =  0;
        object -> ux_device_class_pima_object_handle_id        =  0;
        object -> ux_device_class_pima_object_length           =  0;
        
        /* Send the object to the application.  */
        status = pima -> ux_device_class_pima_object_info_send(pima, object, storage_id, parent_object_handle, &object_handle);
        
        /* Now we return a response with success.  */
        status = (status == UX_SUCCESS) ? UX_DEVICE_CLASS_PIMA_RC_OK : status;
        _ux_device_class_pima_response_send(pima, status, 3, pima -> ux_device_class_pima_storage_id, 
                                            object -> ux_device_class_pima_object_parent_object, object_handle);

        /* Store the object handle. It will be used for the OBJECT_SEND command.  */
        pima -> ux_device_class_pima_current_object_handle =  object_handle;
    }
    else
    {

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_MEMORY_INSUFFICIENT, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* We return an error.  */
        _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_GENERAL_ERROR, 0, 0, 0, 0);
    }

    /* Free the resources. */
    _ux_utility_memory_free(object);
    
    /* Return completion status.  */
    return(status);
}


