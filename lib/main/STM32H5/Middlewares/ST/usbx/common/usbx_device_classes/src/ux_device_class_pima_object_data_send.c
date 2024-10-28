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
/*    _ux_device_class_pima_object_data_send              PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function accepts an object data from the host.                 */ 
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
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            updated status handling,    */
/*                                            improved cancel flow,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_object_data_send(UX_SLAVE_CLASS_PIMA *pima)
{

UINT                        status;
UX_SLAVE_TRANSFER           *transfer_request;
ULONG                       transfer_length;
UX_SLAVE_CLASS_PIMA_OBJECT  *object;
ULONG                       object_handle;
UCHAR                       *object_data;
ULONG                       object_offset;
ULONG                       object_length;
ULONG                       total_length;

    /* Get the last object handle.  The handle used is the last handle used when he host performed a OBJECT_INFO_SEND.  */
    object_handle =  pima -> ux_device_class_pima_current_object_handle;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_OBJECT_DATA_SEND, pima, object_handle, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the object info from the application.  */
    status = pima -> ux_device_class_pima_object_info_get(pima, object_handle, &object);
    
    /* Check for error.  */
    if (status == UX_SUCCESS)
    {    

        /* Data phase (Bulk OUT).  */
        pima -> ux_device_class_pima_state = UX_DEVICE_CLASS_PIMA_PHASE_DATA_OUT;

        /* Set the object length.  */
        object_length =  object -> ux_device_class_pima_object_compressed_size;   
        
        /* Set the total length to be received.  */
        total_length = object_length + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;
        
        /* Reset the offset. */
        object_offset =  0;        

        /* Obtain the pointer to the transfer request of the bulk out endpoint.  */
        transfer_request =  &pima -> ux_device_class_pima_bulk_out_endpoint -> ux_slave_endpoint_transfer_request;
        
        /* Obtain memory for this object info. Use the transfer request pre-allocated memory.  */
        object_data =  transfer_request -> ux_slave_transfer_request_data_pointer;

        /* Assume the host will send all the data.  */
        while (total_length != 0)
        {
        
            /* Get a data payload.  */
            status =  _ux_device_stack_transfer_request(transfer_request, UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH, UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH);

            /* It's canceled, do not proceed.  */
            if (pima -> ux_device_class_pima_state == UX_DEVICE_CLASS_PIMA_PHASE_IDLE)
            {
                pima -> ux_device_class_pima_device_status = UX_DEVICE_CLASS_PIMA_RC_OK;
                return(UX_ERROR);
            }

            /* Check for the status. We may have had a request to cancel the transaction from the host.  */
            if (status != UX_SUCCESS)
            {
    
                /* Check the completion code for transfer abort from the host.  */
                if (transfer_request -> ux_slave_transfer_request_status ==  UX_TRANSFER_STATUS_ABORT)
                {
                    
                    /* Do not proceed.  */
                    return(UX_ERROR);
    
                }                    

            else
                {
                    /* We have a transmission error. Do not proceed.  */
                    status = UX_ERROR;
                    break;

                }
            
            }
                    
            /* Obtain the length of the transaction.  */
            transfer_length =  transfer_request -> ux_slave_transfer_request_actual_length;
            
            /* If this is the first packet, we have to take into account the
               header.  */
            if (object_offset == 0)
            {

                /* Do some sanity check.  */
                if (object_length < (transfer_length - UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE))
                {
                    /* We have an overflow. Do not proceed.  */
                    status = UX_ERROR;
                    break;

                }

                /* Send the object data to the application.  */
                status = pima -> ux_device_class_pima_object_data_send(pima, object_handle, UX_DEVICE_CLASS_PIMA_OBJECT_TRANSFER_PHASE_ACTIVE, 
                                                                        object_data + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE, 
                                                                        object_offset,
                                                                        (transfer_length - UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE));
                                                                        
                /* Check status, if we have a problem, we abort.  */
                if (status != UX_SUCCESS)
                {
    
                    /* We need to inform the host of an error.  */
                    status =  UX_ERROR;
                    break;
                }
                else
                {    

                    /* Adjust the remaining length of the object.  */
                    total_length -= transfer_length;

                    /* Adjust the length to be sent.  */
                    object_offset += (transfer_length - UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE);
                    
                }                        
                                                                                        
            }
            else
            {        

                /* Do some sanity check.  */
                if (object_length < transfer_length)
                {

                    /* We have an overflow. Do not proceed.  */
                    status = UX_ERROR;
                    break;

                }

                /* This is not the first packet, send the object data to the application.  */
                status = pima -> ux_device_class_pima_object_data_send(pima, object_handle, UX_DEVICE_CLASS_PIMA_OBJECT_TRANSFER_PHASE_ACTIVE, 
                                                                        object_data, 
                                                                        object_offset,
                                                                        transfer_length);
                                                                        
                /* Check status, if we have a problem, we abort.  */
                if (status != UX_SUCCESS)
                {
    
                    /* We need to inform the host of an error.  */
                    status =  UX_ERROR;
                    break;
                }
                else
                {    

                    /* Adjust the remaining length of the total object.  */
                    total_length -= transfer_length;

                    /* Adjust the length to be sent.  */
                    object_offset += transfer_length;
                    
                }                        

            }

            /* It's canceled, do not proceed.  */
            if (pima -> ux_device_class_pima_state == UX_DEVICE_CLASS_PIMA_PHASE_IDLE)
            {
                pima -> ux_device_class_pima_device_status = UX_DEVICE_CLASS_PIMA_RC_OK;
                return(UX_ERROR);
            }
        }
    }
    
    /* Check for status.  */
    if (status == UX_SUCCESS)
    {
    
        /* Now we return a response with success.  */
        _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_OK, 0, 0, 0, 0);

        /* Make the object transfer completed.  */
        status = pima -> ux_device_class_pima_object_data_send(pima, object_handle, UX_DEVICE_CLASS_PIMA_OBJECT_TRANSFER_PHASE_COMPLETED, 
                                                                                UX_NULL, 0, 0);
    }    
    else
    {
        
        /* We need to stall the bulk out pipe.  This is the method used by Pima devices to 
           cancel a transaction.  */
        _ux_device_stack_endpoint_stall(pima -> ux_device_class_pima_bulk_in_endpoint);

        /* Make the object transfer non completed.  */
        status = pima -> ux_device_class_pima_object_data_send(pima, object_handle, UX_DEVICE_CLASS_PIMA_OBJECT_TRANSFER_PHASE_COMPLETED_ERROR, 
                                                                                UX_NULL, 0, 0);

    }
    
    /* Return completion status.  */
    return(status);
}


