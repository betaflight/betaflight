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

#if UX_SLAVE_REQUEST_DATA_MAX_LENGTH < UX_SLAVE_CLASS_STORAGE_CSW_LENGTH
#error UX_SLAVE_REQUEST_DATA_MAX_LENGTH too small, please check
#endif

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_storage_csw_send                   PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function sends the status phase of SCSI transaction. Note that */ 
/*    dCSWDataResidue is always set to 0 because we either transfer all   */ 
/*    the data, or report command failure.                                */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    storage                               Pointer to storage class      */ 
/*    endpoint_in                           Pointer to IN endpoint        */ 
/*    status                                Status of CSW                 */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_request     Transfer request              */ 
/*    _ux_utility_long_put                  Put long word                 */ 
/*    _ux_utility_memory_copy               Copy memory                   */ 
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
/*                                            fixed USB CV test issues,   */
/*                                            resulting in version 6.1.3  */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved TAG management,    */
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_storage_csw_send(UX_SLAVE_CLASS_STORAGE *storage, ULONG lun, 
                                UX_SLAVE_ENDPOINT *endpoint_in, UCHAR csw_status)
{

UINT                    status = UX_SUCCESS;
UX_SLAVE_TRANSFER       *transfer_request;
UCHAR                   *csw_buffer;


    UX_PARAMETER_NOT_USED(csw_status);
    UX_PARAMETER_NOT_USED(lun);

#if defined(UX_DEVICE_STANDALONE)

    /* Reset transfer request buffer pointers.  */
    storage -> ux_device_class_storage_ep_out -> ux_slave_endpoint_transfer_request.
            ux_slave_transfer_request_data_pointer = storage -> ux_device_class_storage_buffer[0];
    storage -> ux_device_class_storage_ep_in -> ux_slave_endpoint_transfer_request.
            ux_slave_transfer_request_data_pointer = storage -> ux_device_class_storage_buffer[1];
#endif

    /* If CSW skipped, just return.  */
    if (UX_DEVICE_CLASS_STORAGE_CSW_SKIP(&storage -> ux_slave_class_storage_csw_status))
        return(UX_SUCCESS);

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &endpoint_in -> ux_slave_endpoint_transfer_request;

    /* Get CSW buffer pointer.  */
    csw_buffer = transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Ensure it is cleaned.  */
    _ux_utility_memory_set(csw_buffer, 0, UX_SLAVE_CLASS_STORAGE_CSW_LENGTH); /* Use case of memset is verified. */

    /* Store the signature of the CSW.  */
    _ux_utility_long_put(&csw_buffer[UX_SLAVE_CLASS_STORAGE_CSW_SIGNATURE], UX_SLAVE_CLASS_STORAGE_CSW_SIGNATURE_MASK);

    /* Store the SCSI tag from the CBW.  */
    _ux_utility_long_put(&csw_buffer[UX_SLAVE_CLASS_STORAGE_CSW_TAG], storage -> ux_slave_class_storage_scsi_tag);

    /* Store the dCSWDataResidue.  */
    _ux_utility_long_put(&csw_buffer[UX_SLAVE_CLASS_STORAGE_CSW_DATA_RESIDUE], storage -> ux_slave_class_storage_csw_residue);

    /* Store the status of the previous operation.  */
    csw_buffer[UX_SLAVE_CLASS_STORAGE_CSW_STATUS] = (UCHAR)storage -> ux_slave_class_storage_csw_status;

#if defined(UX_DEVICE_STANDALONE)

    /* Next: Transfer (CSW).  */
    storage -> ux_device_class_storage_cmd_state = UX_DEVICE_CLASS_STORAGE_CMD_CSW;
    storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_TRANS_START;
    storage -> ux_device_class_storage_transfer = transfer_request;

    storage -> ux_device_class_storage_device_length = UX_SLAVE_CLASS_STORAGE_CSW_LENGTH;
    storage -> ux_device_class_storage_data_length = UX_SLAVE_CLASS_STORAGE_CSW_LENGTH;
    storage -> ux_device_class_storage_data_count = 0;

#else

    /* We may be in a special state machine condition where the endpoint is stalled waiting for
       a CLEAR_FEATURE.  We will wait until the host clears the endpoint.  
       The transfer_request function does that.  */
    /* Send the CSW back to the host.  */
    status =  _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_STORAGE_CSW_LENGTH, 
                                    UX_SLAVE_CLASS_STORAGE_CSW_LENGTH);
#endif

    /* Return completion status.  */
    return(status);
}

