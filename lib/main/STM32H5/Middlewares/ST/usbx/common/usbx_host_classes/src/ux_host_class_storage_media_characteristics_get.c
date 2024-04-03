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
/*    _ux_host_class_storage_media_characteristics_get    PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will send a INQUIRY command to get the type of        */
/*    device/media we are dealing with.                                   */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    storage                               Pointer to storage class      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_storage_cbw_initialize Initialize CBW                */ 
/*    _ux_host_class_storage_transport      Send command                  */ 
/*    _ux_host_class_storage_media_capacity_get                           */
/*                                          Get media capacity            */
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Release memory block          */ 
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
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_media_characteristics_get(UX_HOST_CLASS_STORAGE *storage)
{

UINT            status;
UCHAR           *cbw;
UCHAR           *inquiry_response;
UINT            command_length;

    /* Use a pointer for the cbw, easier to manipulate.  */
    cbw =  (UCHAR *) storage -> ux_host_class_storage_cbw;

    /* Get the Write Command Length.  */
#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    if (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_STORAGE_SUBCLASS_UFI)
        command_length =  UX_HOST_CLASS_STORAGE_INQUIRY_COMMAND_LENGTH_UFI;
    else
        command_length =  UX_HOST_CLASS_STORAGE_INQUIRY_COMMAND_LENGTH_SBC;
#else
    command_length =  UX_HOST_CLASS_STORAGE_INQUIRY_COMMAND_LENGTH_SBC;
#endif

    /* Initialize the CBW for this command.  */
    _ux_host_class_storage_cbw_initialize(storage, UX_HOST_CLASS_STORAGE_DATA_IN, UX_HOST_CLASS_STORAGE_INQUIRY_RESPONSE_LENGTH, command_length);
    
    /* Prepare the INQUIRY command block.  */
    *(cbw + UX_HOST_CLASS_STORAGE_CBW_CB + UX_HOST_CLASS_STORAGE_INQUIRY_OPERATION) =  UX_HOST_CLASS_STORAGE_SCSI_INQUIRY;
    
    /* Store the length of the Inquiry Response.  */
    *(cbw + UX_HOST_CLASS_STORAGE_CBW_CB + UX_HOST_CLASS_STORAGE_INQUIRY_ALLOCATION_LENGTH) =  UX_HOST_CLASS_STORAGE_INQUIRY_RESPONSE_LENGTH;

    /* Obtain a block of memory for the answer.  */
    inquiry_response =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_STORAGE_INQUIRY_RESPONSE_LENGTH);
    if (inquiry_response == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

#if defined(UX_HOST_STANDALONE)
    UX_HOST_CLASS_STORAGE_TRANS_STATE_RESET(storage);
    storage -> ux_host_class_storage_memory = inquiry_response;
    storage -> ux_host_class_storage_state_state = UX_HOST_CLASS_STORAGE_STATE_TRANSPORT;
    storage -> ux_host_class_storage_state_next = UX_HOST_CLASS_STORAGE_STATE_INQUIRY_SAVE;
    storage -> ux_host_class_storage_trans_data = inquiry_response;
    status = UX_SUCCESS;
    return(status);
#else
    /* Send the command to transport layer.  */
    status =  _ux_host_class_storage_transport(storage, inquiry_response);

    /* If we have a transport error, there is not much we can do, simply return the
       error.  */
    if (status == UX_SUCCESS)
    {

        /* The Inquiry response contains the type of device/media and a media removable flag.  */
        storage -> ux_host_class_storage_media_type =                                                       *(inquiry_response + UX_HOST_CLASS_STORAGE_INQUIRY_RESPONSE_PERIPHERAL_TYPE);
        storage -> ux_host_class_storage_lun_removable_media_flags[storage -> ux_host_class_storage_lun] =  *(inquiry_response + UX_HOST_CLASS_STORAGE_INQUIRY_RESPONSE_REMOVABLE_MEDIA);

        /* Attempt to read the device capacity in order to retrieve the Sector Size. If the command fails,
           we will default to 512 bytes for a regular drive and 2048 bytes for a CD-ROM or optical drive.  */
        status =  _ux_host_class_storage_media_capacity_get(storage);
    }       

    /* Free the memory resource used for the command response.  */
    _ux_utility_memory_free(inquiry_response);
    
    /* Return completion status.  */
    return(status);                                            
#endif
}

