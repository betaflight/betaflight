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
/*    _ux_device_class_pima_object_prop_value_get         PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*   Return an Object Property Value. The value is fetched by calling the */ 
/*   application which initialized a call back pointer. The application   */
/*   will copy the value at the specified address. The length of the      */ 
/*   value of the object will be returned to compute the length of the    */ 
/*   payload packet.                                                      */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                  Pointer to pima class         */ 
/*    object_handle                         Object Handle                 */
/*    object_property_code                  Object Property code for      */ 
/*                                          which the value is obtained.  */ 
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
/*    _ux_device_class_pima_response_send   Send PIMA response            */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Device Pima Class                                                   */ 
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
/*                                            passed max length to app,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_object_prop_value_get(UX_SLAVE_CLASS_PIMA *pima,
                                                    ULONG object_handle,
                                                    ULONG object_property_code)
{

UINT                    status;
UX_SLAVE_TRANSFER       *transfer_request;
UCHAR                   *pima_data_buffer;
UCHAR                   *pima_data_buffer_end;
UCHAR                   *object_property_value;
ULONG                   object_property_value_length;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_GET_OBJECT_PROP_VALUE, pima, object_handle, object_property_code, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &pima -> ux_device_class_pima_bulk_in_endpoint -> ux_slave_endpoint_transfer_request;
    
    /* Obtain memory for this object info. Use the transfer request pre-allocated memory.  */
    pima_data_buffer =  transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Save the end of the buffer.  */
    pima_data_buffer_end =  pima_data_buffer + UX_SLAVE_REQUEST_DATA_MAX_LENGTH;

    /* Fill in the data container type.  */
    _ux_utility_short_put(pima_data_buffer + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TYPE,
                            UX_DEVICE_CLASS_PIMA_CT_DATA_BLOCK);
    
    /* Fill in the data code.  */
    _ux_utility_short_put(pima_data_buffer + UX_DEVICE_CLASS_PIMA_DATA_HEADER_CODE,
                            UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_PROP_VALUE);
    
    /* Fill in the Transaction ID.  */
    _ux_utility_long_put(pima_data_buffer + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TRANSACTION_ID, 
                            pima -> ux_device_class_pima_transaction_id);

    /* Ask the application to retrieve for us the object prop value.  */
    object_property_value_length = UX_DEVICE_CLASS_PIMA_MAX_PAYLOAD;
    status = pima -> ux_device_class_pima_object_prop_value_get(pima, object_handle, object_property_code, &object_property_value, &object_property_value_length);
    
    /* Result should always be OK, but to be sure .... */
    if (status != UX_SUCCESS)

        /* We return an error.  */
        _ux_device_class_pima_response_send(pima, status, 0, 0, 0, 0);

    else
    {

        /* Check if the prop value will fit.  */
        if (pima_data_buffer + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + object_property_value_length <= pima_data_buffer_end)
        {
            
            /* Copy the property dataset into the local buffer.  */
            _ux_utility_memory_copy(pima_data_buffer + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE, object_property_value, object_property_value_length); /* Use case of memcpy is verified. */

            /* Add the header size to the payload.  */
            object_property_value_length += UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;
            
            /* Fill in the size of the response header.  */
            _ux_utility_long_put(pima_data_buffer + UX_DEVICE_CLASS_PIMA_DATA_HEADER_LENGTH, 
                            object_property_value_length);
        
            /* Send a data payload with the object props array.  */
            status =  _ux_device_stack_transfer_request(transfer_request, object_property_value_length, 0);
            
            /* Now we return a response with success.  */
            _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_OK, 0, 0, 0, 0);
        }
        else
        {

            /* The prop value doesn't fit.  */

            /* Report the error to the application.  */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);

            /* Return response code to host.  */
            _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_GENERAL_ERROR, 0, 0, 0, 0);

            /* Return error.  */
            status =  UX_MEMORY_INSUFFICIENT;
        }
    }
    
    /* Return completion status.  */
    return(status);
}


