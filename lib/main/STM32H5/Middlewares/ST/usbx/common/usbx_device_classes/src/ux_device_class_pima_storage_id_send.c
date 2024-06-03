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


#if UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH < UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE +   \
                                                   4 +                                      \
                                                  (4 * UX_DEVICE_CLASS_PIMA_MAX_STORAGE_IDS)
#error UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH too small
#endif

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_pima_storage_id_send               PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function returns the storage id array to the host.             */ 
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
UINT  _ux_device_class_pima_storage_id_send(UX_SLAVE_CLASS_PIMA *pima)
{

UINT                    status;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                   storage_id_length;
UCHAR                   *storage_id;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_STORAGE_ID_SEND, pima, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &pima -> ux_device_class_pima_bulk_in_endpoint -> ux_slave_endpoint_transfer_request;

    /* Obtain memory for this object. Use the transfer request pre-allocated memory.  */
    storage_id =  transfer_request -> ux_slave_transfer_request_data_pointer;
    
    /* Fill in the data container type.  */
    _ux_utility_short_put(storage_id + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TYPE,
                            UX_DEVICE_CLASS_PIMA_CT_DATA_BLOCK);
    
    /* Fill in the data code.  */
    _ux_utility_short_put(storage_id + UX_DEVICE_CLASS_PIMA_DATA_HEADER_CODE,
                            UX_DEVICE_CLASS_PIMA_OC_GET_STORAGE_IDS);
    
    /* Fill in the Transaction ID.  */
    _ux_utility_long_put(storage_id + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TRANSACTION_ID, 
                            pima -> ux_device_class_pima_transaction_id);
        
    /* We have one element only to report.  */
    _ux_utility_long_put(storage_id + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE, UX_DEVICE_CLASS_PIMA_MAX_STORAGE_IDS);

    /* Insert the element ID.  */
    _ux_utility_long_put(storage_id + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + sizeof(ULONG), pima -> ux_device_class_pima_storage_id);

    /* Compute the overall length of the device info structure.  */
    storage_id_length = UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + sizeof(ULONG) + (sizeof(ULONG) * UX_DEVICE_CLASS_PIMA_MAX_STORAGE_IDS);
    
    /* Fill in the size of the response header.  */
    _ux_utility_long_put(storage_id + UX_DEVICE_CLASS_PIMA_DATA_HEADER_LENGTH, 
                            storage_id_length);
    
    /* Send a data payload with the storage id data set.  */
    status =  _ux_device_stack_transfer_request(transfer_request, storage_id_length, 0);
    
    /* Now we return a response with success.  */
    _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_OK, 0, 0, 0, 0);

    /* Return completion status.  */
    return(status);
}
