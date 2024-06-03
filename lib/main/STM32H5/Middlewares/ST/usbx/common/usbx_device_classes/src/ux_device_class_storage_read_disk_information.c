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

#define USBX_DEVICE_CLASS_STORAGE_DISK_INFORMATION_LENGTH 34
#if UX_SLAVE_REQUEST_DATA_MAX_LENGTH < USBX_DEVICE_CLASS_STORAGE_DISK_INFORMATION_LENGTH
#error UX_SLAVE_REQUEST_DATA_MAX_LENGTH is too small, please check
#endif
UCHAR usbx_device_class_storage_disk_information[] = { 

        0x00, 0x00,                     /* Entire length of disk_information             */
        0x0e,                           /* Erasable/state of last session/disk status ...*/
        0x01,                           /* Number of first track on disk.                */             
        0x01,                           /* Number of sessions.                           */
        0x01,                           /* First track number in last session.           */                           
        0x01,                           /* Last track number in last session.            */                           

        0x00,                           /* DID_V, DBC_V, URU ....                        */
        0x00,                           /* Disk type                                     */
        0x00,                           /* Number of sessions                            */  
        0x00,                           /* First Track Number in last session            */                           
        0x00,                           /* Last Track Number in last session             */                           
        
        0x00, 0x00, 0x00, 0x00,         /* Disk Identification                           */
        0xff, 0xff, 0xff, 0xff,         /* Last session lead in start time               */
        0xff, 0xff, 0xff, 0xff,         /* Last possible start time for SOLO             */
        0x00, 0x00, 0x00, 0x00,         /* Disk Bar Code                                 */
        0x00,                           /* Reserved                                      */
        0x00,                           /* Number of OPC table entries.                  */
        0x00, 0x00, 0x00, 0x00,         /* Some padding ?                                */
        0x00, 0x00                      /*                                               */
        
    };

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_storage_read_disk_information      PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs a READ_DISK_INFORMATION command.             */ 
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
UINT  _ux_device_class_storage_read_disk_information(UX_SLAVE_CLASS_STORAGE *storage, ULONG lun,
                                            UX_SLAVE_ENDPOINT *endpoint_in,
                                            UX_SLAVE_ENDPOINT *endpoint_out, UCHAR * cbwcb)
{

UINT                    status;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                   allocation_length;

    UX_PARAMETER_NOT_USED(endpoint_out);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_STORAGE_GET_CONFIGURATION, storage, lun, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)
    
    /* Initialize the length of the disk information field.  */
    _ux_utility_short_put_big_endian(usbx_device_class_storage_disk_information, (USBX_DEVICE_CLASS_STORAGE_DISK_INFORMATION_LENGTH - 2));
    
    /* Clean the disk status.  */
    usbx_device_class_storage_disk_information[2] &= (UCHAR)~3;

    /* Update the disk status.  */
    usbx_device_class_storage_disk_information[2] = (UCHAR)(usbx_device_class_storage_disk_information[2] | (storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_disk_status & 3));

    /* Clean the last session state.  */
    usbx_device_class_storage_disk_information[2] &= (UCHAR)~0x0c;

    /* Update the last session state.  */
    usbx_device_class_storage_disk_information[2] = (UCHAR)(usbx_device_class_storage_disk_information[2] | ((storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_last_session_state << 2) & 0x0c));

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &endpoint_in -> ux_slave_endpoint_transfer_request;
    
    /* Get the allocation length.  */
    allocation_length =  _ux_utility_short_get_big_endian(cbwcb + UX_SLAVE_CLASS_STORAGE_READ_DISK_INFORMATION_ALLOCATION_LENGTH);

    /* Can we send all the disk information ? */
    if (allocation_length > USBX_DEVICE_CLASS_STORAGE_DISK_INFORMATION_LENGTH)
    
        /* Yes, so send only the disk information profile.  */
        allocation_length = USBX_DEVICE_CLASS_STORAGE_DISK_INFORMATION_LENGTH;
    
    /* Copy the CSW into the transfer request memory.  */
    _ux_utility_memory_copy(transfer_request -> ux_slave_transfer_request_data_pointer, 
                                        usbx_device_class_storage_disk_information, 
                                        allocation_length); /* Use case of memcpy is verified. */
    
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
    _ux_device_stack_transfer_request(transfer_request, 
                              allocation_length,
                              allocation_length);
#endif

    /* Now we set the CSW with success.  */
    storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PASSED;
    status = UX_SUCCESS;
    
    /* Return completion status.  */
    return(status);
}

