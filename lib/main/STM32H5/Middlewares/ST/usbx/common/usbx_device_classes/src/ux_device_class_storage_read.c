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


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_device_class_storage.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_storage_read                       PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs a READ command in 32 or 16 bits.             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    storage                               Pointer to storage class      */ 
/*    endpoint_in                           Pointer to IN endpoint        */
/*    endpoint_out                          Pointer to OUT endpoint       */
/*    cbwcb                                 Pointer to CBWCB              */ 
/*    scsi_command                          SCSI command                  */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    (ux_slave_class_storage_media_read)   Read from media               */ 
/*    (ux_slave_class_storage_media_status) Get media status              */ 
/*    _ux_device_stack_endpoint_stall       Stall endpoint                */ 
/*    _ux_device_stack_transfer_request     Transfer request              */ 
/*    _ux_utility_long_get_big_endian       Get 32-bit big endian         */ 
/*    _ux_utility_short_get_big_endian      Get 16-bit big endian         */ 
/*    _ux_device_class_storage_csw_send     Send CSW                      */
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
/*                                            resulting in version 6.1    */
/*  12-31-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed USB CV test issues,   */
/*                                            resulting in version 6.1.3  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_storage_read(UX_SLAVE_CLASS_STORAGE *storage, ULONG lun, 
                                            UX_SLAVE_ENDPOINT *endpoint_in,
                                            UX_SLAVE_ENDPOINT *endpoint_out, UCHAR * cbwcb, UCHAR scsi_command)
{

UINT                    status;
ULONG                   lba;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                   total_number_blocks; 
ULONG                   media_status;
ULONG                   total_length;

#if !defined(UX_DEVICE_STANDALONE)
ULONG                   number_blocks; 
ULONG                   transfer_length;
ULONG                   done_length;
#endif


    UX_PARAMETER_NOT_USED(endpoint_out);

    /* Get the LBA from the CBWCB.  */
    lba =  _ux_utility_long_get_big_endian(cbwcb + UX_SLAVE_CLASS_STORAGE_READ_LBA);

    /* The type of commands will tell us the width of the field containing the number
       of sectors to read.  */
    if (scsi_command == UX_SLAVE_CLASS_STORAGE_SCSI_READ16)

        /* Get the number of blocks from the CBWCB in 16 bits.  */
        total_number_blocks =  _ux_utility_short_get_big_endian(cbwcb + UX_SLAVE_CLASS_STORAGE_READ_TRANSFER_LENGTH_16);

    else        

        /* Get the number of blocks from the CBWCB in 32 bits.  */
        total_number_blocks =  _ux_utility_long_get_big_endian(cbwcb + UX_SLAVE_CLASS_STORAGE_READ_TRANSFER_LENGTH_32);

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &endpoint_in -> ux_slave_endpoint_transfer_request;

    /* Compute the total length to transfer and how much remains.  */
    total_length =  total_number_blocks * storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_block_length;

    /* Default CSW to failed.  */
    storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_FAILED;

#if defined(UX_DEVICE_STANDALONE)

    /* Obtain the status of the device.  */
    status =  storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_status(storage, lun, 
                                storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_id, &media_status);

    /* Update the request sense.  */
    storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_request_sense_status = media_status;

    /* Update the request to use.  */
    storage -> ux_device_class_storage_transfer = transfer_request;

    /* If there is a problem, return a failed command.  */
    if (status != UX_SUCCESS)
    {

        /* Update residue.  */
        storage -> ux_slave_class_storage_csw_residue = storage -> ux_slave_class_storage_host_length;

        /* Return an error.  */
        return(UX_ERROR);
    }

    /* Next: Disk read -> Transfer (DATA).  */
    storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_DISK_WAIT;
    storage -> ux_device_class_storage_cmd_state = UX_DEVICE_CLASS_STORAGE_CMD_READ;
    storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_OP_START;
    storage -> ux_device_class_storage_buffer_state[0] = UX_DEVICE_CLASS_STORAGE_BUFFER_EMPTY;
    storage -> ux_device_class_storage_buffer_state[1] = UX_DEVICE_CLASS_STORAGE_BUFFER_EMPTY;
    storage -> ux_device_class_storage_buffer_usb = 1;
    storage -> ux_device_class_storage_buffer_disk = 1;

    storage -> ux_device_class_storage_device_length = total_length;
    storage -> ux_device_class_storage_data_length =
        UX_MIN(total_length , storage -> ux_slave_class_storage_host_length);
    storage -> ux_device_class_storage_data_count = 0;

    storage -> ux_device_class_storage_cmd_lba = lba;
    storage -> ux_device_class_storage_cmd_n_lb = total_number_blocks;

#else

    /* Check transfer length.  */

    /* Case (7).  Host length < device length.  */
    if (total_length > storage -> ux_slave_class_storage_host_length)
    {
        _ux_device_stack_endpoint_stall(endpoint_in);
        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR;
        return(UX_ERROR);
    }

    /* Case (8). Hi <> Do.  */
    if ((storage -> ux_slave_class_storage_cbw_flags & 0x80) == 0)
    {
        _ux_device_stack_endpoint_stall(endpoint_out);
        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR;
        return(UX_ERROR);
    }

    /* It may take several transfers to send the requested data.  */
    done_length = 0;
    while (total_number_blocks)
    {

        /* Obtain the status of the device.  */
        status =  storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_status(storage, lun, 
                                    storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_id, &media_status);
    
        /* Update the request sense.  */
        storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_request_sense_status = media_status;
    
        /* If there is a problem, return a failed command.  */
        if (status != UX_SUCCESS)
        {
    
            /* We have a problem, media status error. Return a bad completion and wait for the
               REQUEST_SENSE command.  */
            _ux_device_stack_endpoint_stall(endpoint_in);

            /* Update residue.  */
            storage -> ux_slave_class_storage_csw_residue = storage -> ux_slave_class_storage_host_length - done_length;
    
            /* Return an error.  */
            return(UX_ERROR);
        }

        /* How much can we send in this transfer?  */
        if (total_length > UX_SLAVE_CLASS_STORAGE_BUFFER_SIZE)

            /* Compute the transfer length based on the maximum allowed.  */
            transfer_length =  UX_SLAVE_CLASS_STORAGE_BUFFER_SIZE;
            
        else

            /* Compute the transfer length based on what is left to transfer.  */
            transfer_length =  total_length;

        /* Compute the number of blocks to transfer.  */
        number_blocks = transfer_length / storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_block_length;

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_STORAGE_READ, storage, lun, transfer_request -> ux_slave_transfer_request_data_pointer, 
                                number_blocks, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

        /* Execute the read command from the local media.  */
        status =  storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_read(storage, lun, 
                                                    transfer_request -> ux_slave_transfer_request_data_pointer, number_blocks, lba, &media_status); 

        /* If there is a problem, return a failed command.  */
        if (status != UX_SUCCESS)
        {
    
            /* We have a problem, request error. Return a bad completion and wait for the
               REQUEST_SENSE command.  */
            _ux_device_stack_endpoint_stall(endpoint_in);
    
            /* Update residue.  */
            storage -> ux_slave_class_storage_csw_residue = storage -> ux_slave_class_storage_host_length - done_length;

            /* And update the REQUEST_SENSE codes.  */
            storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_request_sense_status = media_status;
    
            /* Return an error.  */
            return(UX_ERROR);
        }

        /* Sends the data payload back to the caller.  */
        status =  _ux_device_stack_transfer_request(transfer_request, transfer_length, transfer_length);

        /* Check the status.  */
        if(status != UX_SUCCESS)
        {

            /* We have a problem, request error. Return a bad completion and wait for the
               REQUEST_SENSE command.  */
            _ux_device_stack_endpoint_stall(endpoint_in);
    
            /* Update residue.  */
            storage -> ux_slave_class_storage_csw_residue = storage -> ux_slave_class_storage_host_length - done_length;

            /* Update the REQUEST_SENSE codes.  */
            storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_request_sense_status =
                                                UX_DEVICE_CLASS_STORAGE_SENSE_STATUS(0x02,0x54,0x00);

            /* Return an error.  */
            return(UX_ERROR);

        }

        /* Update the LBA address.  */
        lba += number_blocks;
        
        /* Update the length to remain.  */
        total_length -= transfer_length;        
        done_length += transfer_length;
        
        /* Update the number of blocks to read.  */
        total_number_blocks -= number_blocks;
    }

    /* Case (4), (5). Host length too large.  */
    if (storage -> ux_slave_class_storage_host_length > done_length)
    {

        /* Stall Bulk-In.  */
        _ux_device_stack_endpoint_stall(endpoint_in);

        /* Update residure.  */
        storage -> ux_slave_class_storage_csw_residue = storage -> ux_slave_class_storage_host_length - done_length;
    }

#endif /* else defined(UX_DEVICE_STANDALONE) */

    /* Now we set the CSW with success.  */
    storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PASSED;

    /* Return completion status.  */
    return(UX_SUCCESS);
}
