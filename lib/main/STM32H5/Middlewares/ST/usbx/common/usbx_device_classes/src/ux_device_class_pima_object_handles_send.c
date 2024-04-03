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
/*    _ux_device_class_pima_object_handles_send           PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function returns the object handle array to the host.          */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                  Pointer to pima class         */ 
/*    storage_id                            StorageID                     */ 
/*    object_format_code                    Format code filter            */ 
/*    object_association                    Object Handle Association     */ 
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
/*    _ux_utility_memory_set                Set memory                    */ 
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
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            updated status handling,    */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_object_handles_send(UX_SLAVE_CLASS_PIMA *pima, 
                                                    ULONG storage_id,
                                                    ULONG object_format_code,
                                                    ULONG object_association)
{

UINT                    status;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                   object_handles_array_length;
UCHAR                   *object_handles_array;

    UX_PARAMETER_NOT_USED(storage_id);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_OBJECT_HANDLES_SEND, pima, storage_id, object_format_code, object_association, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &pima -> ux_device_class_pima_bulk_in_endpoint -> ux_slave_endpoint_transfer_request;
    
    /* Obtain memory for this object info. Use the transfer request pre-allocated memory.  */
    object_handles_array =  transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Fill in the data container type.  */
    _ux_utility_short_put(object_handles_array + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TYPE,
                            UX_DEVICE_CLASS_PIMA_CT_DATA_BLOCK);
    
    /* Fill in the data code.  */
    _ux_utility_short_put(object_handles_array + UX_DEVICE_CLASS_PIMA_DATA_HEADER_CODE,
                            UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_HANDLES);
    
    /* Fill in the Transaction ID.  */
    _ux_utility_long_put(object_handles_array + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TRANSACTION_ID, 
                            pima -> ux_device_class_pima_transaction_id);

    /* Get the array from the application.  */
    status = pima -> ux_device_class_pima_object_handles_get(pima, object_format_code,
                                                            object_association, (ULONG *) (object_handles_array + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE),
                                                            (UX_DEVICE_CLASS_PIMA_ARRAY_BUFFER_SIZE / sizeof(ULONG)) -1);
    
    /* Result should always be OK, but to be sure .... */
    if (status != UX_SUCCESS)

        /* We return an error.  */
        _ux_device_class_pima_response_send(pima, status, 0, 0, 0, 0);

    else
    {
        
        /* Compute the overall length of the handle arrays.  */
        object_handles_array_length = (_ux_utility_long_get(object_handles_array + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE) +1) * (ULONG)sizeof(ULONG);
        
        /* Add the header size.  */
        object_handles_array_length += UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;
        
        /* Fill in the size of the response header.  */
        _ux_utility_long_put(object_handles_array + UX_DEVICE_CLASS_PIMA_DATA_HEADER_LENGTH, 
                                object_handles_array_length);
        
        /* Send a data payload with the object handles array.  */
        status =  _ux_device_stack_transfer_request(transfer_request, object_handles_array_length, 0);
        
        /* Now we return a response with success.  */
        _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_OK, 0, 0, 0, 0);

    }
    
    /* Return completion status.  */
    return(status);
}


