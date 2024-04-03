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
/**   Storage Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_storage.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_storage_cbw_initialize               PORTABLE C      */ 
/*                                                           6.1.3        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will initialize the Command Block Wrapper (CBW) that  */ 
/*    encapsulate the SCSI request to be sent to the storage device.      */
/*    The CBW is normally used only for the BO protocol.                  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    storage                               Pointer to storage class      */ 
/*    flags                                 Flags for transfer            */ 
/*    data_transfer_length                  Length of data transfer       */ 
/*    command_length                        Length of command             */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_long_put                  Write a 32-bit value          */ 
/*    _ux_utility_memory_set                Set memory to a value         */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Storage Class                                                       */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  12-31-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.3  */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_storage_cbw_initialize(UX_HOST_CLASS_STORAGE *storage, UINT flags,
                                            ULONG data_transfer_length, UINT command_length)
{

UCHAR   *cbw;
    

    /* Use a pointer for the cbw, easier to manipulate.  */
    cbw =  (UCHAR *) storage -> ux_host_class_storage_cbw;

    /* Store the signature of the CBW.  */
    _ux_utility_long_put(cbw, UX_HOST_CLASS_STORAGE_CBW_SIGNATURE_MASK);
    
    /* Set the Tag, this value is simply an arbitrary number that is echoed by 
       the device in the CSW.  */
    _ux_utility_long_put(cbw + UX_HOST_CLASS_STORAGE_CBW_TAG, UX_HOST_CLASS_STORAGE_CBW_TAG_MASK);

    /* Store the Data Transfer Length expected for the data payload.  */
    _ux_utility_long_put(cbw + UX_HOST_CLASS_STORAGE_CBW_DATA_LENGTH, data_transfer_length);

    /* Store the CBW Flag field that contains the transfer flags.  */
    *(cbw + UX_HOST_CLASS_STORAGE_CBW_FLAGS) =  (UCHAR)flags;
    
    /* Store the LUN value.  */
    *(cbw + UX_HOST_CLASS_STORAGE_CBW_LUN) =  (UCHAR)storage -> ux_host_class_storage_lun;

    /* Store the size of the SCSI command block that follows.  */
    *(cbw + UX_HOST_CLASS_STORAGE_CBW_CB_LENGTH) =  (UCHAR)command_length;

    /* Reset the SCSI command block.  */
    _ux_utility_memory_set(cbw + UX_HOST_CLASS_STORAGE_CBW_CB, 0, (ULONG) command_length); /* Use case of memset is verified. */

    /* Return to caller.  */
    return;
}

