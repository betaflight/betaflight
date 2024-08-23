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
/*    _ux_device_class_pima_object_prop_desc_get          PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*   Return an Object Property Description dataset for a format code and  */ 
/*   a specific object property.                                          */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                  Pointer to pima class         */ 
/*    object_property                       Object Property               */
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
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            updated status handling,    */
/*                                            passed max length to app,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_object_prop_desc_get(UX_SLAVE_CLASS_PIMA *pima,
                                                    ULONG object_property,
                                                    ULONG object_format_code)
{

UINT                    status;
UX_SLAVE_TRANSFER       *transfer_request;
UCHAR                   *object_prop_dataset;
ULONG                   object_prop_dataset_length;
UCHAR                   *object_props_desc;
UCHAR                   *object_props_desc_end;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_GET_OBJECT_PROP_DESC, pima, object_property, object_format_code, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &pima -> ux_device_class_pima_bulk_in_endpoint -> ux_slave_endpoint_transfer_request;
    
    /* Obtain memory for this object info. Use the transfer request pre-allocated memory.  */
    object_props_desc =  transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Save the end of the object info.  */
    object_props_desc_end =  object_props_desc + UX_SLAVE_REQUEST_DATA_MAX_LENGTH;

    /* Fill in the data container type.  */
    _ux_utility_short_put(object_props_desc + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TYPE,
                            UX_DEVICE_CLASS_PIMA_CT_DATA_BLOCK);
    
    /* Fill in the data code.  */
    _ux_utility_short_put(object_props_desc + UX_DEVICE_CLASS_PIMA_DATA_HEADER_CODE,
                            UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_PROP_DESC);
    
    /* Fill in the Transaction ID.  */
    _ux_utility_long_put(object_props_desc + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TRANSACTION_ID, 
                            pima -> ux_device_class_pima_transaction_id);

    /* Call the application to retrieve the property description dataset.  */
    object_prop_dataset_length = UX_DEVICE_CLASS_PIMA_MAX_PAYLOAD;
    status = pima -> ux_device_class_pima_object_prop_desc_get(pima, object_property, object_format_code, &object_prop_dataset, &object_prop_dataset_length);

    /* See if we have the right format code and object property.  */
    if (status == UX_SUCCESS)
    {
       
        /* We have found the object property for the format code requested, retrieve the dataset.  */

        /* Do we have enough space?  */
        if (object_props_desc + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + object_prop_dataset_length <= object_props_desc_end)
        {

            /* Copy the object property array.  */
            _ux_utility_memory_copy(object_props_desc +  UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE,
                                    object_prop_dataset, object_prop_dataset_length); /* Use case of memcpy is verified. */
            
            /* Add the header size.  */
            object_prop_dataset_length += UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;
            
            /* Fill in the size of the response header.  */
            _ux_utility_long_put(object_props_desc + UX_DEVICE_CLASS_PIMA_DATA_HEADER_LENGTH, 
                            object_prop_dataset_length);
        
            /* Send a data payload with the object props array.  */
            status =  _ux_device_stack_transfer_request(transfer_request, object_prop_dataset_length, 0);
            
            /* Now we return a response with success.  */
            _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_OK, 0, 0, 0, 0);

            /* Return status.  */
            return(status);
        }
        else
        {

            /* The dataset is too large for our buffer.  */

            /* Report the error to the application.  */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);

            /* Report the error.  */
            status =  UX_DEVICE_CLASS_PIMA_RC_GENERAL_ERROR;
        }
    }
    
    /* We get here when we did not find the object format code or the dataset was too large.  */        

    /* Now we return a response with error code.  */
    _ux_device_class_pima_response_send(pima, status, 0, 0, 0, 0);

    /* Return completion status.  */
    return(status);
}


