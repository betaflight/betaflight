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


#if UX_SLAVE_REQUEST_DATA_MAX_LENGTH < 24
#error UX_SLAVE_REQUEST_DATA_MAX_LENGTH is too small, please check
#endif

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_storage_inquiry                    PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs a INQUIRY command.                           */ 
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
/*    _ux_utility_memory_copy               Copy memory                   */ 
/*    _ux_utility_memory_set                Set memory                    */ 
/*    _ux_utility_short_put_big_endian      Put 16-bit big endian         */
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            updated dCSWDataResidue,    */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_storage_inquiry(UX_SLAVE_CLASS_STORAGE *storage, ULONG lun, UX_SLAVE_ENDPOINT *endpoint_in,
                                            UX_SLAVE_ENDPOINT *endpoint_out, UCHAR * cbwcb)
{

UINT                    status = UX_SUCCESS;
UX_SLAVE_TRANSFER       *transfer_request;
UCHAR                   inquiry_page_code;
ULONG                   inquiry_length;
UCHAR                   *inquiry_buffer;

    UX_PARAMETER_NOT_USED(endpoint_out);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_STORAGE_INQUIRY, storage, lun, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

#if !defined(UX_DEVICE_STANDALONE)

    /* Check direction.  */
    if (storage -> ux_slave_class_storage_host_length &&
        (storage -> ux_slave_class_storage_cbw_flags & 0x80) == 0)
    {
        _ux_device_stack_endpoint_stall(endpoint_out);
        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR;
        return(UX_ERROR);
    }
#endif

    /* From the SCSI Inquiry payload, get the page code.  */
    inquiry_page_code =  *(cbwcb + UX_SLAVE_CLASS_STORAGE_INQUIRY_PAGE_CODE);
    
    /* And the length to be returned. */
    inquiry_length =  storage -> ux_slave_class_storage_host_length;

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &endpoint_in -> ux_slave_endpoint_transfer_request;

    /* Obtain inquiry buffer pointer.  */
    inquiry_buffer = transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Ensure the data buffer is cleaned.  */
    _ux_utility_memory_set(inquiry_buffer, 0, UX_SLAVE_CLASS_STORAGE_INQUIRY_RESPONSE_LENGTH); /* Use case of memset is verified. */

    /* Check for the maximum length to be returned. */
    if (inquiry_length > UX_SLAVE_CLASS_STORAGE_INQUIRY_RESPONSE_LENGTH)
        inquiry_length = UX_SLAVE_CLASS_STORAGE_INQUIRY_RESPONSE_LENGTH;

    /* Default CSW to passed.  */
    storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PASSED;

    /* Ensure we know about the page code.  */
    switch (inquiry_page_code)
    {

    case UX_SLAVE_CLASS_STORAGE_INQUIRY_PAGE_CODE_STANDARD:
            
        /* Store the product type.  */
        inquiry_buffer[UX_SLAVE_CLASS_STORAGE_INQUIRY_RESPONSE_PERIPHERAL_TYPE] =  (UCHAR)storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_type;

        /* Store the Media Removable bit.  */
        inquiry_buffer[UX_SLAVE_CLASS_STORAGE_INQUIRY_RESPONSE_REMOVABLE_MEDIA] =  (UCHAR)storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_removable_flag;

        /* Store the Data Format bit.  */
        if (storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_type == UX_SLAVE_CLASS_STORAGE_MEDIA_CDROM)
            inquiry_buffer[UX_SLAVE_CLASS_STORAGE_INQUIRY_RESPONSE_DATA_FORMAT] =  0x32;
        else
            inquiry_buffer[UX_SLAVE_CLASS_STORAGE_INQUIRY_RESPONSE_DATA_FORMAT] =  0x00;

        /* Store the length of the response.  There is a hack here. For CD-ROM, the data lg is fixed to 0x5B !  */
        if (storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_type != UX_SLAVE_CLASS_STORAGE_MEDIA_CDROM)
            inquiry_buffer[UX_SLAVE_CLASS_STORAGE_INQUIRY_RESPONSE_ADDITIONAL_LENGTH] =  UX_SLAVE_CLASS_STORAGE_INQUIRY_RESPONSE_LENGTH;
        else            
            inquiry_buffer[UX_SLAVE_CLASS_STORAGE_INQUIRY_RESPONSE_ADDITIONAL_LENGTH] =  UX_SLAVE_CLASS_STORAGE_INQUIRY_RESPONSE_LENGTH_CD_ROM;

        /* Fill in the storage vendor ID.  */
        _ux_utility_memory_copy(inquiry_buffer + UX_SLAVE_CLASS_STORAGE_INQUIRY_RESPONSE_VENDOR_INFORMATION,
                                                                    storage -> ux_slave_class_storage_vendor_id, 8); /* Use case of memcpy is verified. */

        /* Fill in the product vendor ID.  */
        _ux_utility_memory_copy(inquiry_buffer + UX_SLAVE_CLASS_STORAGE_INQUIRY_RESPONSE_PRODUCT_ID,
                                                                    storage -> ux_slave_class_storage_product_id, 16); /* Use case of memcpy is verified. */

        /* Fill in the product revision number.  */
        _ux_utility_memory_copy(inquiry_buffer + UX_SLAVE_CLASS_STORAGE_INQUIRY_RESPONSE_PRODUCT_REVISION,
                                                                    storage -> ux_slave_class_storage_product_rev, 4); /* Use case of memcpy is verified. */

        break;

    case UX_SLAVE_CLASS_STORAGE_INQUIRY_PAGE_CODE_SERIAL:

        /* Initialize the page code in response buffer.  */
        _ux_utility_short_put_big_endian(transfer_request -> ux_slave_transfer_request_data_pointer, UX_SLAVE_CLASS_STORAGE_INQUIRY_PAGE_CODE_SERIAL);

        /* Initialize the length of the serial number in response buffer.  */
        _ux_utility_short_put_big_endian(transfer_request -> ux_slave_transfer_request_data_pointer + 2, 20);

        /* Copy the serial number buffer into the transfer request memory.  */
        _ux_utility_memory_copy(transfer_request -> ux_slave_transfer_request_data_pointer + 4, storage -> ux_slave_class_storage_product_serial, 20); /* Use case of memcpy is verified. */

        /* Send a data payload with the inquiry response buffer.  */
        if (inquiry_length > 24)
            inquiry_length = 24;
    
        break;

    default:

#if !defined(UX_DEVICE_STANDALONE)
        /* The page code is not supported.  */
        _ux_device_stack_endpoint_stall(endpoint_in);
#endif

        /* And update the REQUEST_SENSE codes.  */
        storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_request_sense_status =
                                               UX_DEVICE_CLASS_STORAGE_SENSE_STATUS(0x05,0x26,0x01);

        /* Now we set the CSW with failure.  */
        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_FAILED;

        /* Return error.  */
        status =  UX_ERROR;

        break;            
    }    

    /* Error cases.  */
    if (status != UX_SUCCESS)
        return(status);

#if defined(UX_DEVICE_STANDALONE)

    /* Next: Transfer (DATA).  */
    storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_TRANS_START;
    storage -> ux_device_class_storage_cmd_state = UX_DEVICE_CLASS_STORAGE_CMD_READ;

    storage -> ux_device_class_storage_transfer = transfer_request;
    storage -> ux_device_class_storage_device_length = inquiry_length;
    storage -> ux_device_class_storage_data_length = inquiry_length;
    storage -> ux_device_class_storage_data_count = 0;

#else

    /* Send a data payload with the inquiry response buffer.  */
    if (inquiry_length)
        _ux_device_stack_transfer_request(transfer_request, inquiry_length, inquiry_length);

    /* Check length.  */
    if (storage -> ux_slave_class_storage_host_length != inquiry_length)
    {
        storage -> ux_slave_class_storage_csw_residue = storage -> ux_slave_class_storage_host_length - inquiry_length;
        _ux_device_stack_endpoint_stall(endpoint_in);
    }
#endif

    /* Return completion status.  */
    return(status);
}

