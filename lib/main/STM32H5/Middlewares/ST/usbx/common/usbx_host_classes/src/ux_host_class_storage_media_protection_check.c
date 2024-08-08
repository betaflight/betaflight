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
/*    _ux_host_class_storage_media_protection_check       PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will send a MODE_SENSE command to retrieve the medium */
/*    and device parameters.                                              */ 
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
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Release memory block          */ 
/*    _ux_utility_short_get_big_endian      Get short value               */ 
/*    _ux_utility_short_put_big_endian      Put short value               */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_media_protection_check(UX_HOST_CLASS_STORAGE *storage)
{

UINT            status;
UCHAR             *cbw;
UCHAR             *mode_sense_response;
UINT            command_length;
ULONG           wp_parameter_location;


    /* Use a pointer for the cbw, easier to manipulate.  */
    cbw =  (UCHAR *) storage -> ux_host_class_storage_cbw;

    /* Get the Write Command Length.  */
#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    if (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_STORAGE_SUBCLASS_UFI)
        command_length =  UX_HOST_CLASS_STORAGE_MODE_SENSE_COMMAND_LENGTH_UFI;
    else
        command_length =  UX_HOST_CLASS_STORAGE_MODE_SENSE_COMMAND_LENGTH_SBC;
#else
    command_length =  UX_HOST_CLASS_STORAGE_MODE_SENSE_COMMAND_LENGTH_SBC;
#endif

    /* Initialize the CBW for this command.  */
    _ux_host_class_storage_cbw_initialize(storage, UX_HOST_CLASS_STORAGE_DATA_IN, UX_HOST_CLASS_STORAGE_MODE_SENSE_ALL_PAGE_LENGTH, command_length);
    
    /* Prepare the MODE_SENSE command block.  Distinguish between SUBCLASSES. */
    switch (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceSubClass)
    {
    
        case UX_HOST_CLASS_STORAGE_SUBCLASS_RBC         :
#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
        case UX_HOST_CLASS_STORAGE_SUBCLASS_UFI         :
#endif
            *(cbw + UX_HOST_CLASS_STORAGE_CBW_CB + UX_HOST_CLASS_STORAGE_MODE_SENSE_OPERATION) =  UX_HOST_CLASS_STORAGE_SCSI_MODE_SENSE;
            wp_parameter_location =  UX_HOST_CLASS_STORAGE_MODE_SENSE_RESPONSE_ATTRIBUTES;
            break;

        default                                         :            
            *(cbw + UX_HOST_CLASS_STORAGE_CBW_CB + UX_HOST_CLASS_STORAGE_MODE_SENSE_OPERATION) =  UX_HOST_CLASS_STORAGE_SCSI_MODE_SENSE_SHORT;
            wp_parameter_location =  UX_HOST_CLASS_STORAGE_MODE_SENSE_RESPONSE_ATTRIBUTES_SHORT;
            break;
    }
    
    /* We ask for all pages.  */
    *(cbw + UX_HOST_CLASS_STORAGE_CBW_CB + UX_HOST_CLASS_STORAGE_MODE_SENSE_PC_PAGE_CODE) = UX_HOST_CLASS_STORAGE_MODE_SENSE_ALL_PAGE;
    
    /* Store the length of the Inquiry Response.  */
    _ux_utility_short_put_big_endian(cbw + UX_HOST_CLASS_STORAGE_CBW_CB + UX_HOST_CLASS_STORAGE_MODE_SENSE_PARAMETER_LIST_LENGTH, UX_HOST_CLASS_STORAGE_MODE_SENSE_ALL_PAGE_LENGTH);

    /* Obtain a block of memory for the answer.  */
    mode_sense_response =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_STORAGE_MODE_SENSE_ALL_PAGE_LENGTH);
    if (mode_sense_response == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);
    
    /* Send the command to transport layer.  */
    status =  _ux_host_class_storage_transport(storage, mode_sense_response);
    
    /* Reset the Write Protected flag. */
    storage -> ux_host_class_storage_write_protected_media =  UX_FALSE;

    /* If we have a transport error, there is not much we can do, simply return the
       error. The default will be non protected disk. */
    if (status == UX_SUCCESS)
    {
        /* Check to see that we have at least the header of the MODE_SENSE response, if not, ignore the data payload.
           Some devices do not stall this command but rather return 0 byte length.  */
        if(_ux_utility_short_get_big_endian(mode_sense_response + UX_HOST_CLASS_STORAGE_MODE_SENSE_RESPONSE_MODE_DATA_LENGTH) >= UX_HOST_CLASS_STORAGE_MODE_SENSE_HEADER_PAGE_LENGTH)
        {

            /* The Mode Sense response tells us if the media is Write protected or not. */
            if (*(mode_sense_response + wp_parameter_location) & UX_HOST_CLASS_STORAGE_MODE_SENSE_RESPONSE_ATTRIBUTES_WP)
            
                /* The Mode Sense response tells us if the media is Write protected or not. */
                storage -> ux_host_class_storage_write_protected_media =  UX_TRUE;
        }            
    }       

    /* Free the memory resource used for the command response.  */
    _ux_utility_memory_free(mode_sense_response);
    
    /* Return completion status.  */
    return(status);                                            
}

