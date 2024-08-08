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

#ifdef UX_SLAVE_CLASS_STORAGE_INCLUDE_MMC
#define USBX_DEVICE_CLASS_STORAGE_MODE_SENSE_PAGE_CDROM_LENGTH (0x42 + 2)
UCHAR usbx_device_class_storage_mode_sense_page_cdrom[USBX_DEVICE_CLASS_STORAGE_MODE_SENSE_PAGE_CDROM_LENGTH] = { 

    0x2A, 0x42, 0x3F, 0x37, 0xF1, 0x77, 0x29, 0x23,
    0x10, 0x89, 0x01, 0x00, 0x02, 0x00, 0x05, 0x84, 
    0x00, 0x10, 0x10, 0x89, 0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00

};
#else
#define USBX_DEVICE_CLASS_STORAGE_MODE_SENSE_PAGE_CDROM_LENGTH (0)
#endif
#define USBX_DEVICE_CLASS_STORAGE_MODE_SENSE_PAGE_CACHE_LENGTH (UX_SLAVE_CLASS_STORAGE_CACHING_MODE_PAGE_PAGE_LENGTH + 2)
#define USBX_DEVICE_CLASS_STORAGE_MODE_SENSE_PAGE_IEC_LENGTH   (UX_SLAVE_CLASS_STORAGE_IEC_MODE_PAGE_PAGE_LENGTH + 2)

/* Ensure sense pages can fit in the bulk in endpoint's transfer buffer.  */
#define USBX_DEVICE_CLASS_STORAGE_MODE_SENSE_PAGE_ALL_RESPONSE_LENGTH (    \
    UX_SLAVE_CLASS_STORAGE_MODE_SENSE_PARAMETER_HEADER_LENGTH_10 +         \
    USBX_DEVICE_CLASS_STORAGE_MODE_SENSE_PAGE_CDROM_LENGTH +               \
    USBX_DEVICE_CLASS_STORAGE_MODE_SENSE_PAGE_CACHE_LENGTH +               \
    USBX_DEVICE_CLASS_STORAGE_MODE_SENSE_PAGE_IEC_LENGTH)
#if USBX_DEVICE_CLASS_STORAGE_MODE_SENSE_PAGE_ALL_RESPONSE_LENGTH > UX_SLAVE_REQUEST_DATA_MAX_LENGTH
#error "The maximum-sized MODE_SENSE response cannot fit inside the bulk in endpoint's data buffer."
#endif

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_storage_mode_sense                 PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs a MODE_SENSE SCSI command. It supports       */ 
/*    the standard page for the CD-ROM.                                   */ 
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
/*    _ux_utility_short_get_big_endian      Get 16-bit big endian         */
/*    _ux_utility_short_put_big_endian      Put 16-bit big endian         */
/*    _ux_utility_memory_set                Set memory                    */
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
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_storage_mode_sense(UX_SLAVE_CLASS_STORAGE *storage, 
                      ULONG               lun, 
                      UX_SLAVE_ENDPOINT   *endpoint_in,
                      UX_SLAVE_ENDPOINT   *endpoint_out, 
                      UCHAR               *cbwcb)
{

UINT                    status;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                   mode_sense_reply_length;
ULONG                   page_code;
ULONG                   mode_sense_command;
UCHAR                   read_only_flag;
ULONG                   response_header_length;
ULONG                   flags_index;
ULONG                   mode_data_length;
UCHAR                   *page_pointer;
ULONG                   page_length;


    UX_PARAMETER_NOT_USED(endpoint_out);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_STORAGE_MODE_SENSE, storage, lun, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &endpoint_in -> ux_slave_endpoint_transfer_request;

    /* Get the command format : we have 1a and 5a.  */
    mode_sense_command =  (ULONG) *(cbwcb + UX_SLAVE_CLASS_STORAGE_MODE_SENSE_OPERATION);
    
    /* Extract the notification from the cbwcb.  */
    page_code =  (ULONG) *(cbwcb + UX_SLAVE_CLASS_STORAGE_MODE_SENSE_PC_PAGE_CODE);

    /* Check the command.  */
    if (mode_sense_command == UX_SLAVE_CLASS_STORAGE_SCSI_MODE_SENSE_SHORT)
    {

        /* Extract the length to be returned by the cbwcb.  */
        mode_sense_reply_length =  (ULONG) *(cbwcb + UX_SLAVE_CLASS_STORAGE_MODE_SENSE_ALLOCATION_LENGTH_6);
        flags_index = UX_SLAVE_CLASS_STORAGE_MODE_SENSE_PARAMETER_FLAGS_6;
        response_header_length = UX_SLAVE_CLASS_STORAGE_MODE_SENSE_PARAMETER_HEADER_LENGTH_6;
    }

    else
    {

        /* Extract the length to be returned by the cbwcb.  */
        mode_sense_reply_length =  _ux_utility_short_get_big_endian(cbwcb + UX_SLAVE_CLASS_STORAGE_MODE_SENSE_ALLOCATION_LENGTH_10);
        flags_index = UX_SLAVE_CLASS_STORAGE_MODE_SENSE_PARAMETER_FLAGS_10;
        response_header_length = UX_SLAVE_CLASS_STORAGE_MODE_SENSE_PARAMETER_HEADER_LENGTH_10;
    }

    /* Ensure reply not exceed storage buffer.  */
    if (mode_sense_reply_length > UX_SLAVE_CLASS_STORAGE_BUFFER_SIZE)
        mode_sense_reply_length = UX_SLAVE_CLASS_STORAGE_BUFFER_SIZE;

    /* Ensure memory buffer cleaned.  */
    _ux_utility_memory_set(transfer_request -> ux_slave_transfer_request_data_pointer, 0, mode_sense_reply_length); /* Use case of memset is verified. */

    /* Establish READ ONLY flag.  */
    if (storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_read_only_flag == UX_TRUE)
    
        /* This device is Read Only.  */
        read_only_flag = UX_SLAVE_CLASS_STORAGE_MODE_SENSE_PARAMETER_FLAG_WP;
    
    else
    
        /* This device can be written to.  */
        read_only_flag = 0;        

    /* Build response based on expected page codes.  */

    /* Initialize length and page pointer.  */
    mode_data_length = response_header_length;
    page_pointer = transfer_request -> ux_slave_transfer_request_data_pointer + response_header_length;

#ifdef UX_SLAVE_CLASS_STORAGE_INCLUDE_MMC
    /* CD Capabilities and Mechanical Status mode page.  */
    if(page_code == UX_SLAVE_CLASS_STORAGE_MMC2_PAGE_CODE_CDROM ||
        page_code == UX_SLAVE_CLASS_STORAGE_PAGE_CODE_ALL)
    {
        page_length = USBX_DEVICE_CLASS_STORAGE_MODE_SENSE_PAGE_CDROM_LENGTH;

        /* Copy page data.  */
        _ux_utility_memory_copy(page_pointer, usbx_device_class_storage_mode_sense_page_cdrom, page_length); /* Use case of memcpy is verified. */

        /* Update pointer and length.  */
        mode_data_length += page_length;
        page_pointer += page_length;
    }
#endif

    /* Caching mode page is returned if cache flush callback implemented.  */
    if (storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_flush != UX_NULL &&
        (page_code == UX_SLAVE_CLASS_STORAGE_PAGE_CODE_CACHE ||
        page_code == UX_SLAVE_CLASS_STORAGE_PAGE_CODE_ALL))
    {
        page_length = USBX_DEVICE_CLASS_STORAGE_MODE_SENSE_PAGE_CACHE_LENGTH;

        /* Store page code.  */
        *(page_pointer) = UX_SLAVE_CLASS_STORAGE_PAGE_CODE_CACHE;

        /* Store the length of the page data.  */
        *(page_pointer + UX_SLAVE_CLASS_STORAGE_CACHING_MODE_PAGE_LENGTH) =
                            UX_SLAVE_CLASS_STORAGE_CACHING_MODE_PAGE_PAGE_LENGTH;

        /* Set the Write Cache Enabled (WCE) bit.  */
        *(page_pointer + UX_SLAVE_CLASS_STORAGE_CACHING_MODE_PAGE_FLAGS) |=
                            UX_SLAVE_CLASS_STORAGE_CACHING_MODE_PAGE_FLAG_WCE;

        mode_data_length += page_length;
        page_pointer += page_length;
    }

    /* Informational Exceptions Control mode page.  */
    if (page_code == UX_SLAVE_CLASS_STORAGE_PAGE_CODE_IEC ||
        page_code == UX_SLAVE_CLASS_STORAGE_PAGE_CODE_ALL)
    {
        page_length = USBX_DEVICE_CLASS_STORAGE_MODE_SENSE_PAGE_IEC_LENGTH;

        /* Store page code.  */
        *(page_pointer) = UX_SLAVE_CLASS_STORAGE_PAGE_CODE_IEC;

        /* Store the length of the page data.  */
        *(page_pointer + 1) = UX_SLAVE_CLASS_STORAGE_IEC_MODE_PAGE_PAGE_LENGTH;

        mode_data_length += page_length;
    }

    /* Put the payload length in the header.  */
    if (mode_sense_command == UX_SLAVE_CLASS_STORAGE_SCSI_MODE_SENSE_SHORT)
        * transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR)(mode_data_length);
    else
        _ux_utility_short_put_big_endian(transfer_request -> ux_slave_transfer_request_data_pointer, (USHORT)mode_data_length);

    /* Store the write protection flag.  */
    *(transfer_request -> ux_slave_transfer_request_data_pointer + flags_index) = read_only_flag;

#if defined(UX_DEVICE_STANDALONE)

    /* Next: Transfer (DATA).  */
    storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_TRANS_START;
    storage -> ux_device_class_storage_cmd_state = UX_DEVICE_CLASS_STORAGE_CMD_READ;

    storage -> ux_device_class_storage_transfer = transfer_request;
    storage -> ux_device_class_storage_device_length = mode_data_length;
    storage -> ux_device_class_storage_data_length = mode_data_length;
    storage -> ux_device_class_storage_data_count = 0;

#else

    /* Send a payload with the response buffer.  */
    _ux_device_stack_transfer_request(transfer_request, mode_sense_reply_length, mode_sense_reply_length); 
#endif

    /* Now we set the CSW with success.  */
    storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PASSED;
    status = UX_SUCCESS;

    /* Return completion status.  */
    return(status);
}
    
