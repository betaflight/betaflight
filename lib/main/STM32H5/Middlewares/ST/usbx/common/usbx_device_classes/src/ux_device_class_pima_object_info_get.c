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
/*    _ux_device_class_pima_object_info_get               PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function returns the object info structure to the host.        */ 
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
/*    _ux_utility_long_put                  Put 32-bit value              */ 
/*    _ux_utility_short_put                 Put 32-bit value              */ 
/*    _ux_utility_memory_copy               Copy memory                   */ 
/*    _ux_utility_descriptor_pack           Pack descriptor               */
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
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_object_info_get(UX_SLAVE_CLASS_PIMA *pima, ULONG object_handle)
{

UINT                        status;
UX_SLAVE_TRANSFER           *transfer_request;
UX_SLAVE_CLASS_PIMA_OBJECT  *object;
ULONG                       object_info_length;
UCHAR                       *object_info;
UCHAR                       *object_info_pointer;
ULONG                       file_name_length;
ULONG                       capture_date_length;
ULONG                       modification_date_length;
ULONG                       keywords_length;


    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_OBJECT_INFO_GET, pima, object_handle, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the object info from the application.  */
    status = pima -> ux_device_class_pima_object_info_get(pima, object_handle, &object);
    
    /* Check for error.  */
    if (status != UX_SUCCESS)

        /* We return an error.  */
        _ux_device_class_pima_response_send(pima, status, 0, 0, 0, 0);
    
    else
    {    
    
        /* Length calculation and overflow check.  */
        file_name_length = ((ULONG) *object -> ux_device_class_pima_object_filename * 2 ) + 1;
        capture_date_length = ((ULONG) *object -> ux_device_class_pima_object_capture_date *2 ) + 1;
        modification_date_length = ((ULONG) *object -> ux_device_class_pima_object_modification_date * 2 ) + 1;
        keywords_length = ((ULONG) *object -> ux_device_class_pima_object_keywords * 2 ) +1;
        object_info_length = UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE +
                            UX_DEVICE_CLASS_PIMA_OBJECT_VARIABLE_OFFSET +
                            file_name_length +
                            capture_date_length +
                            modification_date_length +
                            keywords_length;

        /* Ensure the object info data can fit in the endpoint's data buffer.  */
        if (object_info_length > UX_SLAVE_REQUEST_DATA_MAX_LENGTH)
        {

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_MEMORY_INSUFFICIENT, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

            /* We return an error.  */
            _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_GENERAL_ERROR, 0, 0, 0, 0);

            /* Return overflow error.  */
            return(UX_MEMORY_INSUFFICIENT);
        }

        /* Obtain the pointer to the transfer request.  */
        transfer_request =  &pima -> ux_device_class_pima_bulk_in_endpoint -> ux_slave_endpoint_transfer_request;

        /* Obtain memory for this object info. Use the transfer request pre-allocated memory.  */
        object_info =  transfer_request -> ux_slave_transfer_request_data_pointer;

        /* Fill in the data container type.  */
        _ux_utility_short_put(object_info + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TYPE,
                                UX_DEVICE_CLASS_PIMA_CT_DATA_BLOCK);
        
        /* Fill in the data code.  */
        _ux_utility_short_put(object_info + UX_DEVICE_CLASS_PIMA_DATA_HEADER_CODE,
                                UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_INFO);
        
        /* Fill in the Transaction ID.  */
        _ux_utility_long_put(object_info + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TRANSACTION_ID, 
                                pima -> ux_device_class_pima_transaction_id);
            
        /* Allocate the device info pointer to the beginning of the dynamic object info field.  */
        object_info_pointer = object_info + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;

        /* The object info structure coming from the application needs to be packed. */
        _ux_utility_descriptor_pack((UCHAR *) object, 
                            _ux_system_class_pima_object_structure,
                            UX_DEVICE_CLASS_PIMA_OBJECT_ENTRIES,
                            object_info_pointer);

        /* Copy the object filename  field.  Point to the beginning of the object description string.  */
        object_info_pointer += UX_DEVICE_CLASS_PIMA_OBJECT_VARIABLE_OFFSET;
        
        /* Copy that string into the object description field.  */
        _ux_utility_memory_copy(object_info_pointer, object -> ux_device_class_pima_object_filename, file_name_length); /* Use case of memcpy is verified. */
    
        /* Point to the next field.  */
        object_info_pointer += file_name_length;
        
        /* Copy that string into the capture date field.  */
        _ux_utility_memory_copy(object_info_pointer, object -> ux_device_class_pima_object_capture_date, capture_date_length); /* Use case of memcpy is verified. */
    
        /* Point to the next field.  */
        object_info_pointer += capture_date_length;
        
        /* Copy that string into the modification date field.  */
        _ux_utility_memory_copy(object_info_pointer, object -> ux_device_class_pima_object_modification_date, modification_date_length); /* Use case of memcpy is verified. */
    
        /* Point to the next field.  */
        object_info_pointer += modification_date_length;
        
        /* Copy that string into the keywords field.  */
        _ux_utility_memory_copy(object_info_pointer, object -> ux_device_class_pima_object_keywords, keywords_length); /* Use case of memcpy is verified. */
        
        /* Fill in the size of the response header.  */
        _ux_utility_long_put(object_info + UX_DEVICE_CLASS_PIMA_DATA_HEADER_LENGTH, 
                                object_info_length);
        
        /* Send a data payload with the object info data set.  */
        status =  _ux_device_stack_transfer_request(transfer_request, object_info_length, 0);
        
        /* Now we return a response with success.  */
        _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_OK, 0, 0, 0, 0);
    }

    /* Return completion status.  */
    return(status);
}
