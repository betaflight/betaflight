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
/**   Device Storage Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_storage.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_storage_get_status_notification    PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs a GET_STATUS_NOTIFICATION command.           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    storage                               Pointer to storage class      */ 
/*    endpoint_in                           Pointer to IN endpoint        */
/*    endpoint_out                          Pointer to OUT endpoint       */
/*    cbwcb                                 Pointer to CBWCB              */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_class_storage_csw_send     Send CSW                      */ 
/*    _ux_device_stack_transfer_request     Transfer request              */ 
/*    _ux_device_stack_endpoint_stall       Stall endpoint                */
/*    _ux_utility_short_put_big_endian      Put 16-bit big endian         */
/*    _ux_utility_memory_copy               Copy memory                   */ 
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
/*                                            optimized command logic,    */
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_storage_get_status_notification(UX_SLAVE_CLASS_STORAGE *storage, ULONG lun,
                                            UX_SLAVE_ENDPOINT *endpoint_in,
                                            UX_SLAVE_ENDPOINT *endpoint_out, UCHAR * cbwcb)
{

UINT                    status;
UX_SLAVE_TRANSFER       *transfer_request;
UCHAR                   *media_notification;
ULONG                   media_notification_length;
ULONG                   notification_class;

    UX_PARAMETER_NOT_USED(endpoint_out);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_STORAGE_READ_CAPACITY, storage, lun, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0);

    /* Default CSW to failed.  */
    storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_FAILED;

    /* Ensure the callback has been initialized.  */
    if (storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_notification == UX_NULL)
    {

#if !defined(UX_DEVICE_STANDALONE)

        /* We need to STALL the IN endpoint.  The endpoint will be reset by the host.  */
        _ux_device_stack_endpoint_stall(endpoint_in);
#endif

        /* And update the REQUEST_SENSE codes.  */
        storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_request_sense_status =
                                               UX_DEVICE_CLASS_STORAGE_SENSE_STATUS(0x05,0x26,0x01);

        /* Return error.  */
        return(UX_FUNCTION_NOT_SUPPORTED);
    }

    /* Extract the notification from the cbwcb.  */
    notification_class = (ULONG) *(cbwcb + UX_SLAVE_CLASS_STORAGE_EVENT_NOTIFICATION_CLASS_REQUEST);
    
    /* Obtain the notification of the device.  */
    status =  storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_notification(storage, lun, 
                                storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_id,
                                notification_class, 
                                &media_notification, 
                                &media_notification_length);

    /* Check the notification length.  */
    if (media_notification_length > UX_SLAVE_REQUEST_DATA_MAX_LENGTH - sizeof(USHORT))
    {

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_MEMORY_INSUFFICIENT, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Error callback.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);

        /* Set status code.  */
        status = UX_MEMORY_INSUFFICIENT;
    }

    /* Check the status for error.  */
    if (status != UX_SUCCESS)
    {
        
#if !defined(UX_DEVICE_STANDALONE)

        /* We need to STALL the IN endpoint.  The endpoint will be reset by the host.  */
        _ux_device_stack_endpoint_stall(endpoint_in);
#endif
    }    
    else
    {

        /* Obtain the pointer to the transfer request.  */
        transfer_request =  &endpoint_in -> ux_slave_endpoint_transfer_request;
        
        /* Put the length of the notification length in the buffer.  */
        _ux_utility_short_put_big_endian(transfer_request -> ux_slave_transfer_request_data_pointer, (USHORT)media_notification_length);

        /* Copy the CSW into the transfer request memory.  */
        _ux_utility_memory_copy(transfer_request -> ux_slave_transfer_request_data_pointer + sizeof (USHORT), 
                                            media_notification, 
                                            media_notification_length); /* Use case of memcpy is verified. */
        
        /* Update the notification length. */
        media_notification_length += (ULONG)sizeof (USHORT);

#if !defined(UX_DEVICE_STANDALONE)
        /* Send a data payload with the notification buffer.  */
        _ux_device_stack_transfer_request(transfer_request, 
                                  media_notification_length,
                                  media_notification_length);
#else
        /* Next: Transfer (DATA).  */
        storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_TRANS_START;
        storage -> ux_device_class_storage_cmd_state = UX_DEVICE_CLASS_STORAGE_CMD_READ;

        storage -> ux_device_class_storage_transfer = transfer_request;
        storage -> ux_device_class_storage_device_length = media_notification_length;
        storage -> ux_device_class_storage_data_length = media_notification_length;
        storage -> ux_device_class_storage_data_count = 0;
        UX_SLAVE_TRANSFER_STATE_RESET(storage -> ux_device_class_storage_transfer);
#endif

        /* Now we set the CSW with success.  */
        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PASSED;
        status = UX_SUCCESS;
    }
        
    /* Return completion status.  */
    return(status);
}

