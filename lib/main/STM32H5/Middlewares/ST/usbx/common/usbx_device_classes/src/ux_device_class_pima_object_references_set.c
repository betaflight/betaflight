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
/*    _ux_device_class_pima_object_references_set         PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*   Set object references associated with an object handle.              */ 
/*   The pima class will call the application to set the array of         */ 
/*   references.                                                          */ 
/*                                                                        */ 
/*                                                                        */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                  Pointer to pima class         */ 
/*    object_handle                         Object Handle                 */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_request     Transfer request              */
/*    _ux_device_stack_endpoint_stall       Stall endpoint                */
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
UINT  _ux_device_class_pima_object_references_set(UX_SLAVE_CLASS_PIMA *pima,
                                                    ULONG object_handle)
{

UINT                    status;
UX_SLAVE_TRANSFER       *transfer_request;
UCHAR                   *pima_data_buffer;
UCHAR                   *object_references;
ULONG                   object_references_length;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_SET_OBJECT_REFERENCES, pima, object_handle, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &pima -> ux_device_class_pima_bulk_out_endpoint -> ux_slave_endpoint_transfer_request;
    
    /* Obtain memory for this object info. Use the transfer request pre-allocated memory.  */
    pima_data_buffer =  transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Get the data payload.  */
    status =  _ux_device_stack_transfer_request(transfer_request, UX_DEVICE_CLASS_PIMA_OBJECT_PROP_VALUE_BUFFER_SIZE, 
                                                    UX_DEVICE_CLASS_PIMA_OBJECT_PROP_VALUE_BUFFER_SIZE);

    /* Check if there was an error. If so, stall the endpoint.  */
    if (status != UX_SUCCESS)
    {
    
        /* Stall the endpoint.  */
        _ux_device_stack_endpoint_stall(pima -> ux_device_class_pima_bulk_out_endpoint);
        
        /* Return the status.  */
        return(status);
        
    }
        
    /* Allocate the device info pointer to the beginning of the dynamic object info field.  */
    object_references = pima_data_buffer + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;

    /* Obtain the length of the data payload.  */
    object_references_length = transfer_request -> ux_slave_transfer_request_actual_length;

    /* Ensure there is some data payload.  */
    if (object_references_length > UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE)
    {

        /* Take out the header.  */
        object_references_length -= UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;
    
        /* Send the object references to the application.  */
        status = pima -> ux_device_class_pima_object_references_set(pima, object_handle, object_references, object_references_length);
    
        /* Now we return a response with success.  */
        status = (status == UX_SUCCESS) ? UX_DEVICE_CLASS_PIMA_RC_OK : status;
        _ux_device_class_pima_response_send(pima, status, 0, 0, 0, 0);

    }
    else
    {    
        /* We return an error.  */
        _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_INVALID_PARAMETER, 0, 0, 0, 0);

        /* Status error.  */
        status = UX_ERROR;

    }
    /* Return completion status.  */
    return(status);
}


