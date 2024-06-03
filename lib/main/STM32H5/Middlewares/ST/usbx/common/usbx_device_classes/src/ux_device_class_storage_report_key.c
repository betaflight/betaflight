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


#if UX_SLAVE_REQUEST_DATA_MAX_LENGTH < UX_SLAVE_CLASS_STORAGE_REPORT_KEY_ANSWER_LENGTH
#error UX_SLAVE_REQUEST_DATA_MAX_LENGTH too small, please check
#endif

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_storage_report_key                 PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs a REPORT_KEY SCSI command.                   */ 
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
/*    _ux_device_stack_transfer_request     Transfer request              */ 
/*    _ux_device_class_storage_csw_send     Send CSW                      */ 
/*    _ux_utility_memory_set                Set memory                    */
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
/*  12-31-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile warning,      */
/*                                            resulting in version 6.1.3  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_storage_report_key(UX_SLAVE_CLASS_STORAGE *storage, 
                      ULONG               lun, 
                      UX_SLAVE_ENDPOINT   *endpoint_in,
                      UX_SLAVE_ENDPOINT   *endpoint_out, 
                      UCHAR               *cbwcb)
{

UINT                    status = UX_SUCCESS;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                   allocation_length;
ULONG                   key_format;


    UX_PARAMETER_NOT_USED(lun);
    UX_PARAMETER_NOT_USED(endpoint_out);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_STORAGE_OTHER, storage, lun, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &endpoint_in -> ux_slave_endpoint_transfer_request;

    /* Get the report key.  */
    key_format =  (ULONG) *(cbwcb + UX_SLAVE_CLASS_STORAGE_REPORT_KEY_FORMAT);
    
    /* Extract the length to be returned by the cbwcb.  */
    allocation_length =  _ux_utility_short_get_big_endian(cbwcb + UX_SLAVE_CLASS_STORAGE_REPORT_KEY_ALLOCATION_LENGTH);

    /* Ensure memory buffer cleaned.  */
    _ux_utility_memory_set(transfer_request -> ux_slave_transfer_request_data_pointer, 0, UX_SLAVE_CLASS_STORAGE_REPORT_KEY_ANSWER_LENGTH); /* Use case of memset is verified. */
    
    /* Filter page code. This is necessary to isolate the CD-ROM mode sense response.  */
    switch (key_format)
    {

        case UX_SLAVE_CLASS_STORAGE_REPORT_KEY_FORMAT_RPC :


            /* Put the length to be returned. */
            _ux_utility_short_put_big_endian(transfer_request -> ux_slave_transfer_request_data_pointer, 
                                            UX_SLAVE_CLASS_STORAGE_REPORT_KEY_ANSWER_PAYLOAD);

            /* Put the reset field. */
            *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_STORAGE_REPORT_KEY_ANSWER_RESET_FIELD) =  0x25;

            /* Put the region mask. */
            *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_STORAGE_REPORT_KEY_ANSWER_REGION_MASK) =  0xFF;

            /* No region code. */
            *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_STORAGE_REPORT_KEY_ANSWER_RPC_SCHEME)  =  0x00;

            /* Compute the payload to return. Depends on the request.  */
            if (allocation_length >  UX_SLAVE_CLASS_STORAGE_REPORT_KEY_ANSWER_LENGTH)
            
                /* Adjust the reply.  */
                allocation_length = UX_SLAVE_CLASS_STORAGE_REPORT_KEY_ANSWER_LENGTH;                

            break;
          
        default :
            allocation_length = 0;
            break;
                
    }

#if defined(UX_DEVICE_STANDALONE)

    /* Next: Transfer (DATA).  */
    storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_TRANS_START;
    storage -> ux_device_class_storage_cmd_state = UX_DEVICE_CLASS_STORAGE_CMD_READ;

    storage -> ux_device_class_storage_transfer = transfer_request;
    storage -> ux_device_class_storage_device_length = allocation_length;
    storage -> ux_device_class_storage_data_length = allocation_length;
    storage -> ux_device_class_storage_data_count = 0;
    UX_SLAVE_TRANSFER_STATE_RESET(storage -> ux_device_class_storage_transfer);

#else

    /* Send a data payload with the read_capacity response buffer.  */
    status = _ux_device_stack_transfer_request(transfer_request, allocation_length, allocation_length);  /* Use case of memset is verified. */
#endif

    /* Now we set the CSW with success.  */
    storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PASSED;

    /* Return completion status.  */
    return(status);
}
    
