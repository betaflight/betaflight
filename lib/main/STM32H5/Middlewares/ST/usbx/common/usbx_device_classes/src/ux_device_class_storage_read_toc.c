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


#if UX_SLAVE_CLASS_STORAGE_BUFFER_SIZE < 20
#error UX_SLAVE_CLASS_STORAGE_BUFFER_SIZE too small, please check
#endif

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_storage_read_toc                   PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs a READ_TOC SCSI command. This is only for    */ 
/*    supporting CD-ROM emulation and is hardwired to what Windows wants. */ 
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
/*    _ux_utility_short_get_big_endian      Get 16-bit big endian         */
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
UINT  _ux_device_class_storage_read_toc(UX_SLAVE_CLASS_STORAGE *storage, ULONG lun, 
                                            UX_SLAVE_ENDPOINT *endpoint_in,
                                            UX_SLAVE_ENDPOINT *endpoint_out, UCHAR * cbwcb)
{

UINT                    status;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                   allocation_length;
ULONG                   toc_length;
UCHAR                   *toc_buffer;


    UX_PARAMETER_NOT_USED(lun);
    UX_PARAMETER_NOT_USED(endpoint_out);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_STORAGE_READ_TOC, storage, lun, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &endpoint_in -> ux_slave_endpoint_transfer_request;

    /* Obtain TOC buffer.  */
    toc_buffer = transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Reset this buffer.  */
    _ux_utility_memory_set(toc_buffer,0,20); /* Use case of memset is verified. */

    /* Get the allocation length.  */
    allocation_length =  _ux_utility_short_get_big_endian(cbwcb + UX_SLAVE_CLASS_STORAGE_READ_TOC_ALLOCATION_LENGTH);

    /* Insert the fist and last tack number.  */
    toc_buffer[2] =  0x01;
    toc_buffer[3] =  0x01;
    
    /* Set TOC length by default.  */
    toc_length = 20;
    
    /* Insert the ADR and control values.  */
    toc_buffer[5] =  0x14;
    
    /* Insert the TOC tack number.  */
    toc_buffer[6] =  0x01;
        
    /* Check if the request is for the TOC or time stamp.  */
    switch (*(cbwcb + UX_SLAVE_CLASS_STORAGE_READ_TOC_FORMAT))
    {

        case  0x02 :

            /* Set the toc buffer length.  */
            toc_length  = 20;
        
            /* Insert the TOC buffer length.  */
            toc_buffer[1] =  0x12;
            
            /* Insert some time values.  */
            toc_buffer[10] =  0x02;
            toc_buffer[13] =  0x17;
            toc_buffer[14] =  0xAA;
            toc_buffer[18] =  0x04;
            toc_buffer[19] =  0x1a;
            
            break;

        case 0x01 :

            /* Set the toc buffer length.  */
            toc_length  = 19;
        
            /* Insert the TOC buffer length.  */
            toc_buffer[1] =  0x12;
            
            toc_buffer[13] =  0x17;
            toc_buffer[14] =  0xAA;
            toc_buffer[19] =  0xb0;
            
            break;

        case 0x00 :
        
            /* Set the toc buffer length.  */
            toc_length  = 20;
        
            /* Insert the TOC buffer length.  */
            toc_buffer[1] =  0x12;
            
            /* Insert some time values.  */
            toc_buffer[10] =  0x02;
            toc_buffer[13] =  0x17;
            toc_buffer[14] =  0xAA;
            toc_buffer[18] =  0x04;
            toc_buffer[19] =  0x1a;

            break;
    }
            
    /* Check how much we can send back.  */
    if (allocation_length > toc_length)
    
        /* We return less than demanded.  */
        allocation_length = toc_length;
    
#if defined(UX_DEVICE_STANDALONE)

    /* Next: Transfer (DATA).  */
    storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_TRANS_START;
    storage -> ux_device_class_storage_cmd_state = UX_DEVICE_CLASS_STORAGE_CMD_READ;

    storage -> ux_device_class_storage_transfer = transfer_request;
    storage -> ux_device_class_storage_device_length = allocation_length;
    storage -> ux_device_class_storage_data_length = allocation_length;
    storage -> ux_device_class_storage_data_count = 0;

#else

    /* Send a data payload with the TOC response buffer.  */
    _ux_device_stack_transfer_request(transfer_request, allocation_length, allocation_length);
#endif

    /* Now we set the CSW with success.  */
    storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PASSED;
    status = UX_SUCCESS;
    
    /* Return completion status.  */
    return(status);
    
}
    
