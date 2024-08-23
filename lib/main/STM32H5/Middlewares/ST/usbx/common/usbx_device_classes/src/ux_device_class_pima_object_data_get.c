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
/*    _ux_device_class_pima_object_data_get               PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function returns the object data to the host.                  */
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
/*    _ux_device_class_pima_response_send   Send PIMA response            */
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
/*                                            improved sanity checks,     */
/*                                            improved cancel flow,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_object_data_get(UX_SLAVE_CLASS_PIMA *pima, ULONG object_handle)
{

UINT                        status;
UX_SLAVE_TRANSFER           *transfer_request;
UX_SLAVE_CLASS_PIMA_OBJECT  *object;
UCHAR                       *object_data;
ULONG                       object_offset;
ULONG                       object_length;
ULONG                       total_length;
ULONG                       object_length_demanded;
ULONG                       object_length_received;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_OBJECT_DATA_GET, pima, object_handle, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the object info from the application.  */
    status = pima -> ux_device_class_pima_object_info_get(pima, object_handle, &object);

    /* Check for error.  */
    if (status == UX_SUCCESS)
    {

        /* Data phase (Bulk IN).  */
        pima -> ux_device_class_pima_state = UX_DEVICE_CLASS_PIMA_PHASE_DATA_IN;

        /* Set the object length.  */
        object_length =  object -> ux_device_class_pima_object_compressed_size;

        /* Set the total length to be sent.  */
        total_length = object_length + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;

        /* Reset the offset. */
        object_offset =  0;

        /* Obtain the pointer to the transfer request of the bulk in endpoint.  */
        transfer_request =  &pima -> ux_device_class_pima_bulk_in_endpoint -> ux_slave_endpoint_transfer_request;

        /* Obtain memory for this object info. Use the transfer request pre-allocated memory.  */
        object_data =  transfer_request -> ux_slave_transfer_request_data_pointer;

        /* Fill in the total length to be sent (header + payload.   */
        _ux_utility_long_put(object_data + UX_DEVICE_CLASS_PIMA_DATA_HEADER_LENGTH,
                                total_length);

        /* Fill in the data container type.  */
        _ux_utility_short_put(object_data + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TYPE,
                                UX_DEVICE_CLASS_PIMA_CT_DATA_BLOCK);

        /* Fill in the data code.  */
        _ux_utility_short_put(object_data + UX_DEVICE_CLASS_PIMA_DATA_HEADER_CODE,
                                UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT);

        /* Fill in the Transaction ID.  */
        _ux_utility_long_put(object_data + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TRANSACTION_ID,
                                pima -> ux_device_class_pima_transaction_id);

        /* Assuming the host will ask for the entire object.  */
        while (object_length != 0)
        {

            /* If this is the first packet, we have to take into account the
               header.  */
            if (object_offset == 0)
            {

                /* Calculate the maximum length for the first packet.  */
                object_length_demanded = UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH -
                                            UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;

                /* Can we get that much from the application ?  */
                if (object_length_demanded > object_length)

                    /* We ask too much.  */
                    object_length_demanded =  object_length;

                /* Obtain some data from the application.  */
                status = pima -> ux_device_class_pima_object_data_get(pima, object_handle,
                                            object_data + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE,
                                            object_offset,
                                            object_length_demanded,
                                            &object_length_received);

                /* Check status, if we have a problem, we abort.  */
                if (status != UX_SUCCESS)
                {

                    /* We need to inform the host of an error.  */
                    break;
                }
                else
                {

                    /* Do some sanity check.  */
                    if (object_length < object_length_received)
                    {

                        /* We have an overflow. Do not proceed.  */
                        status = UX_DEVICE_CLASS_PIMA_RC_GENERAL_ERROR;
                        break;
                    }

                    /* Adjust the length of the object.  */
                    object_length -= object_length_received;

                    /* Adjust the offset within the object data.  */
                    object_offset += object_length_received;

                    /* Adjust the length to be sent.  */
                    object_length_received +=  UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;
                }
            }
            else
            {

                /* Calculate the maximum length for the first packet.  */
                object_length_demanded = UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH;

                /* Can we get that much from the application ?  */
                if (object_length_demanded > object_length)

                    /* We ask too much.  */
                    object_length_demanded =  object_length;

                /* Obtain some data from the application.  */
                status = pima -> ux_device_class_pima_object_data_get(pima, object_handle, object_data, object_offset,
                                                                        object_length_demanded,
                                                                        &object_length_received);

                /* Check status, if we have a problem, we abort.  */
                if (status != UX_SUCCESS)
                {

                    /* We need to inform the host of an error.  */
                    break;
                }
                else
                {

                    /* Do some sanity check.  */
                    if (object_length < object_length_received)
                    {

                        /* We have an overflow. Do not proceed.  */
                        status = UX_DEVICE_CLASS_PIMA_RC_GENERAL_ERROR;
                        break;
                    }

                    /* Adjust the length of the object.  */
                    object_length -= object_length_received;

                    /* Adjust the offset within the object data.  */
                    object_offset += object_length_received;
                }
            }

            /* It's canceled, do not proceed.  */
            if (pima -> ux_device_class_pima_state == UX_DEVICE_CLASS_PIMA_PHASE_IDLE)
            {
                pima -> ux_device_class_pima_device_status = UX_DEVICE_CLASS_PIMA_RC_OK;
                return(UX_ERROR);
            }

            /* Not do transfer, just send the object data to the host.  */
            status =  _ux_device_stack_transfer_request(transfer_request,
                                object_length_received, UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH);

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

                    /* We need to inform the host of an error.  */
                    status = UX_DEVICE_CLASS_PIMA_RC_GENERAL_ERROR;
                    break;
                }
            }
            else
            {

                /* Update the total length to be sent.  */
                total_length -= object_length_received;
            }
        }

        /* Now we return a response.  */
        if (status == UX_SUCCESS)
            _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_OK, 0, 0, 0, 0);
    }

    /* Check if status is OK.  */
    if (status != UX_SUCCESS)
    {

        /* We need to stall the bulk in pipe.  This is the method used by Pima devices to
           cancel a transaction.  */
        _ux_device_stack_endpoint_stall(pima -> ux_device_class_pima_bulk_in_endpoint);
    }

    /* Return completion status.  */
    return(status);
}
