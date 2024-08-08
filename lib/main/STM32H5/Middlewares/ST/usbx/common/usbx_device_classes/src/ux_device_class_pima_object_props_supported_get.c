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
/*    _ux_device_class_pima_object_props_supported_get    PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*   Return an Object Property Code array of supported object properties  */ 
/*   in the first parameter.                                              */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                  Pointer to pima class         */ 
/*    object_format_code                    Object format for which the   */ 
/*                                          properties supported are.     */ 
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
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved sanity checks,     */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_object_props_supported_get(UX_SLAVE_CLASS_PIMA *pima,
                                                    ULONG object_format_code)
{

UINT                    status;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                   object_props_list_items;
ULONG                   object_props_list_length;
UCHAR                   *object_props_list;
USHORT                  *object_props_list_pointer;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_OBJECTS_PROPS_SUPPORTED_GET, pima, object_format_code, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &pima -> ux_device_class_pima_bulk_in_endpoint -> ux_slave_endpoint_transfer_request;
    
    /* Obtain memory for this object info. Use the transfer request pre-allocated memory.  */
    object_props_list =  transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Fill in the data container type.  */
    _ux_utility_short_put(object_props_list + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TYPE,
                            UX_DEVICE_CLASS_PIMA_CT_DATA_BLOCK);
    
    /* Fill in the data code.  */
    _ux_utility_short_put(object_props_list + UX_DEVICE_CLASS_PIMA_DATA_HEADER_CODE,
                            UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_PROPS_SUPPORTED);
    
    /* Fill in the Transaction ID.  */
    _ux_utility_long_put(object_props_list + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TRANSACTION_ID, 
                            pima -> ux_device_class_pima_transaction_id);

    /* Get the pointer to the list of arrays.  */
    object_props_list_pointer = pima -> ux_device_class_pima_object_properties_list;
    
    /* Find the object format within the object props arrays.  */
    while (*object_props_list_pointer != 0)
    {
        /* See what the format code is.  */
        if (*object_props_list_pointer == (USHORT) object_format_code)
        {
           
            /* We have found the format code requested, retrieve the array of props.  */
            /* Retrieve the number of properties for this format code.  */
            object_props_list_items = (ULONG) *(object_props_list_pointer + 1);

            /* Compute the overall length of the object properties array = number of properties + size of array in USHORT.  */
            object_props_list_length = (object_props_list_items * (ULONG)sizeof(USHORT)) + (ULONG)sizeof(ULONG);

            /* Add the header size.  */
            object_props_list_length += UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;

            /* Sanity check of buffer length.  */
            UX_ASSERT(object_props_list_length <= UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH);

            /* Fill in the size of the response header.  */
            _ux_utility_long_put(object_props_list + UX_DEVICE_CLASS_PIMA_DATA_HEADER_LENGTH, 
                                   object_props_list_length);
    
            /* Update the destination pointer.  */
            object_props_list += UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;
            
            /* Insert the number of elements in the array.   */
            _ux_utility_long_put(object_props_list, object_props_list_items);
            
            /* Add the length of the array.  */
            object_props_list += sizeof(ULONG);
            
            /* Point the object_props_list_pointer to the props supported.  Skip the object format and the number
               of properties.  */
            object_props_list_pointer += 2;
            
            /* Create the array in little endian format for transfer to host.  */
            while(object_props_list_items--)
            {
            
                /* Insert the object property in little endian.  */
                _ux_utility_short_put(object_props_list, *object_props_list_pointer);
                                
                /* Update target pointer.  */
                object_props_list += sizeof(SHORT);
                                                
                /* Update source pointer.  */
                object_props_list_pointer++;
    
            }
            
            /* Send a data payload with the object props array.  */
            status =  _ux_device_stack_transfer_request(transfer_request, object_props_list_length, 0);
            
            /* Now we return a response with success.  */
            _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_OK, 0, 0, 0, 0);

            /* Return status.  */
            return(status);
        }
        else
        {

            /* We need to skip this object format and point to the next.  */
            /* Retrieve the number of properties for this format code.  */
            object_props_list_items = *(object_props_list_pointer + 1);
            
            /* Add the number of items to skip to the current list pointer.  */
            object_props_list_pointer += object_props_list_items + 2;

        }
    }
    
    /* We get here when we did not find the object format code.  */        
    status = _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_INVALID_OBJECT_FORMAT_CODE, 0, 0, 0, 0);

    /* Return completion status.  */
    return(status);
}


