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
/**   Storage Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_storage.h"
#include "ux_host_stack.h"


#if defined(UX_HOST_STANDALONE)


#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
#error Storage legacy protocols not supported in standlone mode
#endif

#define UX_HOST_CLASS_STORAGE_PROTOCOL_GET(s)                                   \
    ((s) -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceProtocol)
#define UX_HOST_CLASS_STORAGE_PROTOCOL_IS_BO(s)                                 \
    (UX_HOST_CLASS_STORAGE_PROTOCOL_GET(s) == UX_HOST_CLASS_STORAGE_PROTOCOL_BO)
#define UX_HOST_CLASS_STORAGE_PROTOCOL_IS_CB(s)                                 \
    (UX_HOST_CLASS_STORAGE_PROTOCOL_GET(s) == UX_HOST_CLASS_STORAGE_PROTOCOL_CB)
#define UX_HOST_CLASS_STORAGE_PROTOCOL_IS_CBI(s)                                \
    (UX_HOST_CLASS_STORAGE_PROTOCOL_GET(s) == UX_HOST_CLASS_STORAGE_PROTOCOL_CBI)

static inline VOID _ux_host_class_storage_transport_cbw(UX_HOST_CLASS_STORAGE *storage);
static inline VOID _ux_host_class_storage_transport_csw(UX_HOST_CLASS_STORAGE *storage);
static inline VOID _ux_host_class_storage_transport_trans_exit(UX_HOST_CLASS_STORAGE *storage);
static inline VOID _ux_host_class_storage_transport_trans_error(UX_HOST_CLASS_STORAGE *storage);
static inline VOID _ux_host_class_storage_transport_trans_next(UX_HOST_CLASS_STORAGE *storage);
static inline VOID _ux_host_class_storage_transport_trans_in_next(UX_HOST_CLASS_STORAGE *storage);
static inline VOID _ux_host_class_storage_transport_trans_out_next(UX_HOST_CLASS_STORAGE *storage);
static inline VOID _ux_host_class_storage_transport_status(UX_HOST_CLASS_STORAGE *storage);
static inline VOID _ux_host_class_storage_transport_ms_reset(UX_HOST_CLASS_STORAGE *storage);
static inline VOID _ux_host_class_storage_transport_ep_reset(UX_HOST_CLASS_STORAGE *storage);


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_storage_transport_run                PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the transport layer for all protocols. It perform  */
/*    the error recovery and retries if needed.                           */
/*                                                                        */
/*    It's valid only with standalone mode.                               */
/*    It's non-blocking.                                                  */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    storage                               Pointer to storage class      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    (ux_host_class_storage_transport)     Class storage transport       */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Storage Class                                                       */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved internal logic,    */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_transport_run(UX_HOST_CLASS_STORAGE *storage)
{
UINT                    status;
UCHAR                   state;
INT                     immediate_state = UX_TRUE;

    while(immediate_state)
    {
        state = storage -> ux_host_class_storage_trans_state;
        switch(state)
        {
        case UX_STATE_RESET:
            storage -> ux_host_class_storage_trans_retry = UX_HOST_CLASS_STORAGE_REQUEST_SENSE_RETRY;
            storage -> ux_host_class_storage_sense_code = 0;
            storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_CBW;

            /* Start the command immediately.  */
            /* Fall through.  */
        case UX_HOST_CLASS_STORAGE_TRANS_CBW:
            _ux_host_class_storage_transport_cbw(storage);
            continue;

        case UX_HOST_CLASS_STORAGE_TRANS_CSW:
            _ux_host_class_storage_transport_csw(storage);
            continue;

        case UX_HOST_CLASS_STORAGE_TRANS_REQ_SENSE:
            status = _ux_host_class_storage_request_sense(storage);
            if (status == UX_SUCCESS)
                continue;

            /* Failed to issue request sense.  */
            storage -> ux_host_class_storage_status = status;
            storage -> ux_host_class_storage_trans_state = UX_STATE_IDLE;
            return(UX_STATE_ERROR);

        case UX_HOST_CLASS_STORAGE_TRANS_IN_NEXT:
            _ux_host_class_storage_transport_trans_in_next(storage);
            continue;

        case UX_HOST_CLASS_STORAGE_TRANS_OUT_NEXT:
            _ux_host_class_storage_transport_trans_out_next(storage);
            continue;

        case UX_HOST_CLASS_STORAGE_TRANS_WAIT:
            status = _ux_host_stack_transfer_run(storage -> ux_host_class_storage_trans);

            /* Next cases.  */
            if (status == UX_STATE_NEXT || status == UX_STATE_IDLE)
            {
                _ux_host_class_storage_transport_trans_next(storage);
                continue;
            }

            /* Exit cases.  */
            if (status < UX_STATE_IDLE)
            {
                _ux_host_class_storage_transport_trans_exit(storage);
                return(UX_STATE_EXIT);
            }

            /* Error cases.  */
            if (status < UX_STATE_NEXT)
            {
                _ux_host_class_storage_transport_trans_error(storage);
                continue;
            }

            /* Wait.  */
            return(UX_STATE_WAIT);

        case UX_HOST_CLASS_STORAGE_TRANS_STATUS:
            _ux_host_class_storage_transport_status(storage);
            continue;

        case UX_HOST_CLASS_STORAGE_TRANS_MS_RESET:
            _ux_host_class_storage_transport_ms_reset(storage);
            continue;

        case UX_HOST_CLASS_STORAGE_TRANS_EP_RESET:
            _ux_host_class_storage_transport_ep_reset(storage);
            continue;

        case UX_STATE_IDLE:
            return(UX_STATE_NEXT);
        case UX_STATE_EXIT:
            return(UX_STATE_EXIT);

        default:
            break;
        }

        /* Invalid unhandled state, break the loop.  */
        immediate_state = UX_FALSE;
    }

    /* Unexpected state, fatal error.  */
    storage -> ux_host_class_storage_status = UX_ERROR;
    storage -> ux_host_class_storage_trans_state = UX_STATE_EXIT;
    return(UX_STATE_EXIT);
}
static inline VOID _ux_host_class_storage_transport_cbw(UX_HOST_CLASS_STORAGE *storage)
{
UX_ENDPOINT         *endpoint;
UX_TRANSFER         *trans;

    /* Prepare BulkOUT CBW transfer.  */
    endpoint = storage -> ux_host_class_storage_bulk_out_endpoint;
    trans = &endpoint -> ux_endpoint_transfer_request;
    trans -> ux_transfer_request_data_pointer = storage -> ux_host_class_storage_cbw;
    trans -> ux_transfer_request_requested_length = UX_HOST_CLASS_STORAGE_CBW_LENGTH;

    /* Next : wait transfer done.  */
    UX_TRANSFER_STATE_RESET(trans);
    storage -> ux_host_class_storage_trans = trans;
    storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_WAIT;
    storage -> ux_host_class_storage_trans_stage = UX_HOST_CLASS_STORAGE_STAGE_CBW;
}
static inline VOID _ux_host_class_storage_transport_csw(UX_HOST_CLASS_STORAGE *storage)
{
UX_ENDPOINT         *endpoint;
UX_TRANSFER         *trans;

    /* Prepare BulkIN CSW transfer.  */
    endpoint = storage -> ux_host_class_storage_bulk_in_endpoint;
    trans = &endpoint -> ux_endpoint_transfer_request;
    trans -> ux_transfer_request_data_pointer = storage -> ux_host_class_storage_csw;
    trans -> ux_transfer_request_requested_length = UX_HOST_CLASS_STORAGE_CSW_LENGTH;

    UX_TRANSFER_STATE_RESET(trans);
    storage -> ux_host_class_storage_trans = trans;
    storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_WAIT;
    storage -> ux_host_class_storage_trans_stage = UX_HOST_CLASS_STORAGE_STAGE_CSW;
}
static inline VOID _ux_host_class_storage_transport_trans_exit(UX_HOST_CLASS_STORAGE *storage)
{
UCHAR                   *cbw = (UCHAR *) storage -> ux_host_class_storage_cbw;
UCHAR                   *cb = cbw + UX_HOST_CLASS_STORAGE_CBW_CB;

    /* If request sense in progress, response buffer should be freed.  */
    if (*(cb + UX_HOST_CLASS_STORAGE_REQUEST_SENSE_OPERATION) ==
        UX_HOST_CLASS_STORAGE_SCSI_REQUEST_SENSE)
    {

        /* Free sense buffer and restore data pointer.  */
        _ux_utility_memory_free(storage -> ux_host_class_storage_trans_data);
        storage -> ux_host_class_storage_trans_data =
                                storage -> ux_host_class_storage_trans_data_bak;

        /* Restore CBW.  */
        _ux_utility_memory_copy(storage -> ux_host_class_storage_cbw,
                                storage -> ux_host_class_storage_saved_cbw,
                                UX_HOST_CLASS_STORAGE_CBW_LENGTH); /* Use case of memcpy is verified. */
    }

    /* It's exit.  */
    storage -> ux_host_class_storage_trans_state = UX_STATE_EXIT;
}
static inline VOID _ux_host_class_storage_transport_trans_error(UX_HOST_CLASS_STORAGE *storage)
{
UX_TRANSFER             *trans = storage -> ux_host_class_storage_trans;
UCHAR                   stage = storage -> ux_host_class_storage_trans_stage;
    if (trans -> ux_transfer_request_completion_code == UX_TRANSFER_STALLED)
    {

        /* In case CBW stall, send MS Reset.  */
        if (stage & UX_HOST_CLASS_STORAGE_STAGE_CBW)
        {
            storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_MS_RESET;
            return;
        }
        /* In case DATA/CSW stall, reset endpoint and try again.  */
        if (stage <= UX_HOST_CLASS_STORAGE_STAGE_CSW)
        {

            /* Check retry.  */
            if (storage -> ux_host_class_storage_trans_retry)
            {
                storage -> ux_host_class_storage_trans_retry --;
                storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_EP_RESET;
            }
            else
            {
                storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_MS_RESET;
            }
            return;
        }

        /* In other case, control transfer error.  */
        storage -> ux_host_class_storage_status = UX_ERROR;
        storage -> ux_host_class_storage_trans_state = UX_STATE_EXIT;
        return;
    }
    else
    {

        /* Perform MS Reset.  */
        storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_MS_RESET;
    }
}
static inline VOID _ux_host_class_storage_transport_trans_next(UX_HOST_CLASS_STORAGE *storage)
{
UCHAR                   stage = storage -> ux_host_class_storage_trans_stage;
UCHAR                   *cbw = storage -> ux_host_class_storage_cbw;
ULONG                   len = _ux_utility_long_get(cbw + UX_HOST_CLASS_STORAGE_CBW_DATA_LENGTH);
UCHAR                   flags = *(cbw + UX_HOST_CLASS_STORAGE_CBW_FLAGS);
UX_ENDPOINT             *endpoint;
UX_TRANSFER             *trans;
UX_HCD                  *hcd;

    /* Check transfer completion code.  */
    trans = storage -> ux_host_class_storage_trans;
    if (UX_SUCCESS != trans -> ux_transfer_request_completion_code)
    {
        _ux_host_class_storage_transport_trans_error(storage);
        return;
    }

    if (stage == UX_HOST_CLASS_STORAGE_STAGE_CBW)
    {

        if (len == 0)
        {

            /* There is no data, start CSW.  */
            storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_CSW;
            return;
        }

        /* Reset transfer count.  */
        storage -> ux_host_class_storage_data_phase_length = 0;
        if (flags & UX_HOST_CLASS_STORAGE_DATA_IN)
        {

            /* Data IN.  */
            endpoint = storage -> ux_host_class_storage_bulk_in_endpoint;
            storage -> ux_host_class_storage_trans = &endpoint -> ux_endpoint_transfer_request;
            storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_IN_NEXT;
            return;
        }

        /* Data OUT.  */
        endpoint = storage -> ux_host_class_storage_bulk_out_endpoint;
        storage -> ux_host_class_storage_trans = &endpoint -> ux_endpoint_transfer_request;
        storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_OUT_NEXT;
        return;
    }
    if (stage == UX_HOST_CLASS_STORAGE_STAGE_DATA)
    {
        if (flags & UX_HOST_CLASS_STORAGE_DATA_IN)
            storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_IN_NEXT;
        else
            storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_OUT_NEXT;
        return;
    }
    if (stage == UX_HOST_CLASS_STORAGE_STAGE_CSW)
    {
        storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_STATUS;
        return;
    }
    if (stage == UX_HOST_CLASS_STORAGE_STAGE_MS_RESET)
    {

        /* Issue EPReset(OUT) then EPReset(IN) after MSReset().  */
        endpoint = storage -> ux_host_class_storage_bulk_out_endpoint;
        storage -> ux_host_class_storage_trans = &endpoint -> ux_endpoint_transfer_request;
        storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_EP_RESET;
        return;
    }
    if (stage & UX_HOST_CLASS_STORAGE_STAGE_EP_RESET)
    {

        /* Reset endpoint in HCD level.  */
        /* Pickup HCD pointer.  */
        hcd = UX_DEVICE_HCD_GET(storage -> ux_host_class_storage_device);

        /* Get endpoint instance.  */
        trans = storage -> ux_host_class_storage_trans;
        endpoint = (trans -> ux_transfer_request_index & UX_ENDPOINT_DIRECTION) ?
                            storage -> ux_host_class_storage_bulk_in_endpoint :
                            storage -> ux_host_class_storage_bulk_out_endpoint;

        /* Call HCD entry function.  */
        hcd -> ux_hcd_entry_function(hcd, UX_HCD_RESET_ENDPOINT, endpoint);

        /* Check if it's part of MSReset() - OUT - IN.  */
        if (stage & UX_HOST_CLASS_STORAGE_STAGE_MS_RESET)
        {

            /* If it's last step, transport is done with error.  */
            if (trans -> ux_transfer_request_index & UX_ENDPOINT_DIRECTION)
            {

                storage -> ux_host_class_storage_trans_state = UX_STATE_IDLE;
            }
            else
            {

                /* BulkIN endpoint reset.  */
                endpoint = storage -> ux_host_class_storage_bulk_in_endpoint;
                storage -> ux_host_class_storage_trans = &endpoint -> ux_endpoint_transfer_request;
                storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_EP_RESET;
            }
        }
        else
        {

            storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_CSW;
        }
        return;
    }
}
static inline VOID _ux_host_class_storage_transport_trans_in_next(UX_HOST_CLASS_STORAGE *storage)
{
UCHAR               *cbw = storage -> ux_host_class_storage_cbw;
UX_TRANSFER         *trans = storage -> ux_host_class_storage_trans;
ULONG               requested_length;

    /* Get total data length.  */
    requested_length = _ux_utility_long_get(cbw + UX_HOST_CLASS_STORAGE_CBW_DATA_LENGTH);

    /* Check if it's start of DATA phase.  */
    if (storage -> ux_host_class_storage_trans_stage == UX_HOST_CLASS_STORAGE_STAGE_DATA)
    {

        /* Update transferred data length.  */
        storage -> ux_host_class_storage_data_phase_length +=
                                    trans -> ux_transfer_request_actual_length;

        /* Check if all data is done.  */
        if ((storage -> ux_host_class_storage_data_phase_length >= requested_length) ||
            (trans -> ux_transfer_request_actual_length < trans -> ux_transfer_request_requested_length))
        {

            /* Start CSW.  */
            storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_CSW;
            return;
        }
    }
    else
    {

        /* Initialize the request for data phase.  */
        storage -> ux_host_class_storage_trans_stage = UX_HOST_CLASS_STORAGE_STAGE_DATA;
    }


    /* There is data remains, continue transfer.  */
    requested_length -= storage -> ux_host_class_storage_data_phase_length;

    /* Limit max transfer size.  */
    if (requested_length > UX_HOST_CLASS_STORAGE_MAX_TRANSFER_SIZE)
        requested_length = UX_HOST_CLASS_STORAGE_MAX_TRANSFER_SIZE;

    /* Update transfer.  */
    UX_TRANSFER_STATE_RESET(trans);
    trans -> ux_transfer_request_requested_length = requested_length;
    trans -> ux_transfer_request_data_pointer =
                            storage -> ux_host_class_storage_trans_data +
                            storage -> ux_host_class_storage_data_phase_length;
    storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_WAIT;
}
static inline VOID _ux_host_class_storage_transport_trans_out_next(UX_HOST_CLASS_STORAGE *storage)
{
UCHAR               *cbw = storage -> ux_host_class_storage_cbw;
UX_TRANSFER         *trans = storage -> ux_host_class_storage_trans;
ULONG               requested_length;

    /* Get total data length.  */
    requested_length = _ux_utility_long_get(cbw + UX_HOST_CLASS_STORAGE_CBW_DATA_LENGTH);

    /* Check if it's start of DATA phase.  */
    if (storage -> ux_host_class_storage_trans_stage == UX_HOST_CLASS_STORAGE_STAGE_DATA)
    {
        /* Update data count.  */
        storage -> ux_host_class_storage_data_phase_length += 
                                trans -> ux_transfer_request_requested_length;

        /* Check if all data is done.  */
        if (storage -> ux_host_class_storage_data_phase_length >= requested_length)
        {

            /* Start CSW.  */
            storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_CSW;
            return;
        }
    }
    else
    {

        /* Initialize the request for data phase.  */
        storage -> ux_host_class_storage_trans_stage = UX_HOST_CLASS_STORAGE_STAGE_DATA;
    }

    /* There is data remains, continue transfer.  */
    requested_length -= storage -> ux_host_class_storage_data_phase_length;

    /* Limit max transfer size.  */
    if (requested_length > UX_HOST_CLASS_STORAGE_MAX_TRANSFER_SIZE)
        requested_length = UX_HOST_CLASS_STORAGE_MAX_TRANSFER_SIZE;

    /* Update transfer.  */
    UX_TRANSFER_STATE_RESET(trans);
    trans -> ux_transfer_request_requested_length = requested_length;
    trans -> ux_transfer_request_data_pointer =
                            storage -> ux_host_class_storage_trans_data +
                            storage -> ux_host_class_storage_data_phase_length;
    storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_WAIT;
}
static inline VOID _ux_host_class_storage_transport_status(UX_HOST_CLASS_STORAGE *storage)
{
UCHAR                   *cbw = storage -> ux_host_class_storage_cbw;
UCHAR                   *cb = cbw + UX_HOST_CLASS_STORAGE_CBW_CB;
UCHAR                   *resp = storage -> ux_host_class_storage_trans_data;
ULONG                   sense_code;
UCHAR                   csw_status;

    /* Get CSW status code.  */
    csw_status = storage -> ux_host_class_storage_csw[UX_HOST_CLASS_STORAGE_CSW_STATUS];

    /* Check if it's request sense.  */
    if (*(cb + UX_HOST_CLASS_STORAGE_REQUEST_SENSE_OPERATION) == UX_HOST_CLASS_STORAGE_SCSI_REQUEST_SENSE)
    {

        /* Restore command data pointer.  */
        storage -> ux_host_class_storage_trans_data =
                                storage -> ux_host_class_storage_trans_data_bak;

        /* Get sense code from response buffer.  */ 
        sense_code = UX_HOST_CLASS_STORAGE_SENSE_STATUS(
                (ULONG) *(resp + UX_HOST_CLASS_STORAGE_REQUEST_SENSE_RESPONSE_SENSE_KEY),
                (ULONG) *(resp + UX_HOST_CLASS_STORAGE_REQUEST_SENSE_RESPONSE_CODE),
                (ULONG) *(resp + UX_HOST_CLASS_STORAGE_REQUEST_SENSE_RESPONSE_CODE_QUALIFIER));

        /* Free buffer allocated for sense response.  */
        _ux_utility_memory_free(resp);

        /* Passed, keep previous status and done.  */
        if (csw_status == UX_HOST_CLASS_STORAGE_CSW_PASSED)
        {

            /* Store the sense code in the storage instance.  */
            storage -> ux_host_class_storage_sense_code =  sense_code;

            /* Now we are done.  */
            storage -> ux_host_class_storage_trans_state = UX_STATE_IDLE;
            return;
        }

        /* There is issue, do MSReset() and transport done.  */
        storage -> ux_host_class_storage_trans_status = UX_HOST_CLASS_STORAGE_CSW_PHASE_ERROR;
        storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_MS_RESET;
        return;
    }

    /* Save status.  */
    storage -> ux_host_class_storage_trans_status = csw_status;

    /* Success.  */
    if (csw_status == UX_HOST_CLASS_STORAGE_CSW_PASSED)
    {
        storage -> ux_host_class_storage_status = UX_SUCCESS;
        storage -> ux_host_class_storage_trans_state = UX_STATE_IDLE;
        return;
    }

    /* There is error.  */
    storage -> ux_host_class_storage_status = UX_ERROR;

    /* Fail.  */
    if (csw_status == UX_HOST_CLASS_STORAGE_CSW_FAILED)
    {
        storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_REQ_SENSE;
        return;
    }

    /* Fatal/phase error.  */
    storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_MS_RESET;
    return;
}
static inline VOID _ux_host_class_storage_transport_ms_reset(UX_HOST_CLASS_STORAGE *storage)
{
UX_INTERFACE            *interface_ptr;
UX_ENDPOINT             *endpoint;
UX_TRANSFER             *trans;

    /* Prepare MSReset()  */
    interface_ptr = storage -> ux_host_class_storage_interface;
    endpoint = &storage -> ux_host_class_storage_device -> ux_device_control_endpoint;
    trans = &endpoint -> ux_endpoint_transfer_request;

    /* Create a transfer_request for the RESET request.  */
    trans -> ux_transfer_request_data_pointer =     UX_NULL;
    trans -> ux_transfer_request_requested_length = 0;
    trans -> ux_transfer_request_function =         UX_HOST_CLASS_STORAGE_RESET;
    trans -> ux_transfer_request_type =             UX_REQUEST_OUT | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
    trans -> ux_transfer_request_value =            0;
    trans -> ux_transfer_request_index  =           interface_ptr -> ux_interface_descriptor.bInterfaceNumber;

    UX_TRANSFER_STATE_RESET(trans);
    storage -> ux_host_class_storage_trans = trans;
    storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_WAIT;
    storage -> ux_host_class_storage_trans_stage = UX_HOST_CLASS_STORAGE_STAGE_MS_RESET;
}
static inline VOID _ux_host_class_storage_transport_ep_reset(UX_HOST_CLASS_STORAGE *storage)
{
UX_ENDPOINT             *endpoint;
UX_TRANSFER             *trans;
ULONG                   endpoint_address;

    /* Clear halt for current transfer.  */

    /* Get endpoint address.  */
    endpoint = storage -> ux_host_class_storage_trans -> ux_transfer_request_endpoint;
    endpoint_address = endpoint -> ux_endpoint_descriptor.bEndpointAddress;

    /* Send ClearEndpointFeature(EP, Halt).  */
    endpoint = &storage -> ux_host_class_storage_device -> ux_device_control_endpoint;
    trans = &endpoint -> ux_endpoint_transfer_request;

    /* Create a transfer_request for the CLEAR_FEATURE request.  */
    trans -> ux_transfer_request_data_pointer =     UX_NULL;
    trans -> ux_transfer_request_requested_length = 0;
    trans -> ux_transfer_request_function =         UX_CLEAR_FEATURE;
    trans -> ux_transfer_request_type =             UX_REQUEST_OUT | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_ENDPOINT;
    trans -> ux_transfer_request_value =            UX_ENDPOINT_HALT;
    trans -> ux_transfer_request_index =            endpoint_address;

    UX_TRANSFER_STATE_RESET(trans);
    storage -> ux_host_class_storage_trans = trans;
    storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_WAIT;
    storage -> ux_host_class_storage_trans_stage |= UX_HOST_CLASS_STORAGE_STAGE_EP_RESET;
}
#endif
