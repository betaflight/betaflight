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
/*    _ux_device_class_storage_synchronize_cache          PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs a SYNCHRONIZE_CACHE command in 32 or 16      */ 
/*    bits.                                                               */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    storage                               Pointer to storage class      */ 
/*    endpoint_in                           Pointer to IN endpoint        */
/*    endpoint_out                          Pointer to OUT endpoint       */
/*    cbwcb                                 Pointer to the CBWCB          */ 
/*    scsi_command                          SCSI command                  */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    (ux_slave_class_storage_media_status) Get media status              */ 
/*    (ux_slave_class_storage_media_flush)  Flush media                   */ 
/*    _ux_device_class_storage_csw_send     Send CSW                      */ 
/*    _ux_device_stack_endpoint_stall       Stall endpoint                */ 
/*    _ux_utility_long_get_big_endian       Get 32-bit big endian         */ 
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
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_storage_synchronize_cache(UX_SLAVE_CLASS_STORAGE *storage, ULONG lun, 
                                                 UX_SLAVE_ENDPOINT *endpoint_in,
                                                 UX_SLAVE_ENDPOINT *endpoint_out, UCHAR *cbwcb, UCHAR scsi_command)
{

UINT                    status;
ULONG                   lba;
USHORT                  number_blocks;
ULONG                   media_status;

#if !defined(UX_DEVICE_STANDALONE)
UCHAR                   flags;
#endif


    UX_PARAMETER_NOT_USED(endpoint_out);
    UX_PARAMETER_NOT_USED(scsi_command);

    /* By default status is passed.  */
    storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PASSED;

    /* Is there not an implementation?  */
    if (storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_flush == UX_NULL)
    {

        /* This means the application is not using a cache.  */

        /* Return success.  */
        return(UX_SUCCESS);
    }

    /* Get the LBA and number of blocks from the CBWCB in 16 bits.  */
    lba           =         _ux_utility_long_get_big_endian(cbwcb + UX_SLAVE_CLASS_STORAGE_SYNCHRONIZE_CACHE_LBA);
    number_blocks = (USHORT)_ux_utility_short_get_big_endian(cbwcb + UX_SLAVE_CLASS_STORAGE_SYNCHRONIZE_CACHE_NUMBER_OF_BLOCKS);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_STORAGE_SYNCHRONIZE_CACHE, storage, lun, lba, number_blocks, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the status of the device.  */
    status =  storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_status(storage, 
                            lun, storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_id, &media_status);

    /* Update the request sense.  */
    storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_request_sense_status = media_status;

    /* If there is a problem, return a failed command.  */
    if (status != UX_SUCCESS)
    {

        /* We have a problem, media status error. Return a bad completion and wait for the
           REQUEST_SENSE command.  */
#if !defined(UX_DEVICE_STANDALONE)
        _ux_device_stack_endpoint_stall(endpoint_in);
#else
        UX_PARAMETER_NOT_USED(endpoint_in);
        storage -> ux_device_class_storage_cmd_state = UX_DEVICE_CLASS_STORAGE_CMD_ERR;
#endif

        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_FAILED;

        /* We are done here.  */
        return(UX_ERROR);
    }

    /* Now it's OK to perform synchronize cache.  */

#if defined(UX_DEVICE_STANDALONE)

    /* Next: Disk (SYNC).  */
    storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_DISK_WAIT;
    storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_OP_START;
    storage -> ux_device_class_storage_cmd_state = UX_DEVICE_CLASS_STORAGE_CMD_DISK_OP;

    storage -> ux_device_class_storage_cmd_lba = lba;
    storage -> ux_device_class_storage_cmd_n_lb = number_blocks;

#else

    /* Get the flags.  */
    flags =  *(cbwcb + UX_SLAVE_CLASS_STORAGE_SYNCHRONIZE_CACHE_FLAGS);

    /* If the immediate bit is set, we return a CSW before flush.  */
    if ((flags & UX_SLAVE_CLASS_STORAGE_SYNCHRONIZE_CACHE_FLAGS_IMMED) != 0)
        _ux_device_class_storage_csw_send(storage, lun, endpoint_in, UX_SLAVE_CLASS_STORAGE_CSW_PASSED);

    /* Send the flush command to the local media.  */
    status =  storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_media_flush(storage, lun, number_blocks, lba, &media_status);

    /* Update the request sense.  */
    storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_request_sense_status = media_status;

    /* If the immediate bit is set, we are already done, no matter what local operation status is.  */
    if ((flags & UX_SLAVE_CLASS_STORAGE_SYNCHRONIZE_CACHE_FLAGS_IMMED) != 0)
    {

        /* CSW skipped since already sent in this function.  */
        UX_DEVICE_CLASS_STORAGE_CSW_SKIP(&storage -> ux_slave_class_storage_csw_status) = UX_TRUE;
        return(status);
    }

    /* If there is a problem, return a failed command.  */
    if (status != UX_SUCCESS)
    {

        /* We have a problem, request error. Return a bad completion and wait for the
           REQUEST_SENSE command.  */
        _ux_device_stack_endpoint_stall(endpoint_in);

        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_FAILED;

        /* Return an error.  */
        return(UX_ERROR);
    }
#endif

    /* Return completion status.  */
    return(status);
}
