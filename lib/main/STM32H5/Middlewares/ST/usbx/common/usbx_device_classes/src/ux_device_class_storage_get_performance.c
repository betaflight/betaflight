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

#define USBX_DEVICE_CLASS_STORAGE_GET_PERFORMANCE_0_LENGTH                    16
#if UX_SLAVE_REQUEST_DATA_MAX_LENGTH < (USBX_DEVICE_CLASS_STORAGE_GET_PERFORMANCE_0_LENGTH + 8)
#error UX_SLAVE_REQUEST_DATA_MAX_LENGTH too small, please check
#endif
UCHAR usbx_device_class_storage_performance[] = { 

    0x08, 0x00, 0x00, 0x00, 0x00, 0x23, 0x12, 0x80, 
    0x00, 0x00, 0x10, 0x89, 0x00, 0x00, 0x10, 0x89
};

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_storage_get_performance            PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs a GET_PERFORMANCE SCSI command.              */ 
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
/*    _ux_utility_memory_copy               Copy memory                   */
/*    _ux_utility_long_put_big_endian       Put 32-bit big endian         */
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
UINT  _ux_device_class_storage_get_performance(UX_SLAVE_CLASS_STORAGE *storage, 
                                            ULONG               lun, 
                                            UX_SLAVE_ENDPOINT   *endpoint_in,
                                            UX_SLAVE_ENDPOINT   *endpoint_out, 
                                            UCHAR               *cbwcb)
{

UINT                    status;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                   performance_page;
ULONG                   data_length = 0;


    UX_PARAMETER_NOT_USED(lun);
    UX_PARAMETER_NOT_USED(endpoint_out);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_STORAGE_OTHER, storage, lun, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &endpoint_in -> ux_slave_endpoint_transfer_request;

    /* Ensure memory buffer cleaned.  */
    _ux_utility_memory_set(transfer_request -> ux_slave_transfer_request_data_pointer, 0, 64); /* Use case of memset is verified. */
    
    /* Get the performance page code.  */
    performance_page =  (ULONG) *(cbwcb + UX_SLAVE_CLASS_STORAGE_GET_PERFORMANCE_PAGE);

    /* Filter it as the response depends on it.  */
    switch (performance_page)
    {    
            
        case UX_SLAVE_CLASS_STORAGE_GET_PERFORMANCE_PAGE_14  :            

            /* Put the length to be returned. */
            _ux_utility_long_put_big_endian(transfer_request -> ux_slave_transfer_request_data_pointer, 
                                            UX_SLAVE_CLASS_STORAGE_GET_PERFORMANCE_PAYLOAD_LENGTH);

            /* Put the payload to be returned. */
            _ux_utility_long_put_big_endian(transfer_request -> ux_slave_transfer_request_data_pointer + 4, 
                                            UX_SLAVE_CLASS_STORAGE_GET_PERFORMANCE_PAYLOAD);

            data_length = UX_SLAVE_CLASS_STORAGE_GET_PERFORMANCE_RESPONSE_LENGTH;
            break;
            
        case UX_SLAVE_CLASS_STORAGE_GET_PERFORMANCE_PAGE_0 :

            /* Put the length to be returned. */
            _ux_utility_long_put_big_endian(transfer_request -> ux_slave_transfer_request_data_pointer, 
                                            USBX_DEVICE_CLASS_STORAGE_GET_PERFORMANCE_0_LENGTH + 8);

            /* Copy the CSW into the transfer request memory.  */
            _ux_utility_memory_copy(transfer_request -> ux_slave_transfer_request_data_pointer + 8, 
                                        usbx_device_class_storage_performance, USBX_DEVICE_CLASS_STORAGE_GET_PERFORMANCE_0_LENGTH); /* Use case of memcpy is verified. */

            data_length = USBX_DEVICE_CLASS_STORAGE_GET_PERFORMANCE_0_LENGTH + 8;
            break;

        default:
            data_length = 0;
    }

#if defined(UX_DEVICE_STANDALONE)

    /* Next: Transfer (DATA).  */
    storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_TRANS_START;
    storage -> ux_device_class_storage_cmd_state = UX_DEVICE_CLASS_STORAGE_CMD_READ;

    storage -> ux_device_class_storage_transfer = transfer_request;
    storage -> ux_device_class_storage_device_length = data_length;
    storage -> ux_device_class_storage_data_length = data_length;
    storage -> ux_device_class_storage_data_count = 0;
    UX_SLAVE_TRANSFER_STATE_RESET(storage -> ux_device_class_storage_transfer);

#else

    /* Send a data payload with the read_capacity response buffer.  */
    _ux_device_stack_transfer_request(transfer_request, data_length, data_length); 
#endif

    /* Now we set the CSW with success.  */
    storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PASSED;
    status = UX_SUCCESS;

    /* Return completion status.  */
    return(status);
}
    
