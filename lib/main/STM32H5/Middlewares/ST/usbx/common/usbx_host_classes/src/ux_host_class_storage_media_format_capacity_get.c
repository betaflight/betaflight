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
/*    _ux_host_class_storage_media_format_capacity_get    PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will send a READ_FORMAT_CAPACITY command to the       */
/*    device. Some USB storage disk require this function for the sanity  */ 
/*    of their state machine.                                             */ 
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
/*    _ux_utility_short_put_big_endian      Put 32-bit big endian         */
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
UINT  _ux_host_class_storage_media_format_capacity_get(UX_HOST_CLASS_STORAGE *storage)
{

UCHAR           *cbw;
UCHAR           *read_format_capacity_response;
UINT            command_length;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_STORAGE_MEDIA_FORMAT_CAPACITY_GET, storage, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Use a pointer for the cbw, easier to manipulate.  */
    cbw =  (UCHAR *) storage -> ux_host_class_storage_cbw;

    /* Get the Write Command Length.  */
#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    if (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_STORAGE_SUBCLASS_UFI)
        command_length =  UX_HOST_CLASS_STORAGE_READ_FORMAT_COMMAND_LENGTH_UFI;
    else
        command_length =  UX_HOST_CLASS_STORAGE_READ_FORMAT_COMMAND_LENGTH_SBC;
#else
    command_length =  UX_HOST_CLASS_STORAGE_READ_FORMAT_COMMAND_LENGTH_SBC;
#endif

    /* Initialize the CBW for this command.  */
    _ux_host_class_storage_cbw_initialize(storage, UX_HOST_CLASS_STORAGE_DATA_IN, UX_HOST_CLASS_STORAGE_READ_FORMAT_RESPONSE_LENGTH, command_length);
    
    /* Prepare the READ FORMAT CAPACITY command block.  */
    *(cbw + UX_HOST_CLASS_STORAGE_CBW_CB + UX_HOST_CLASS_STORAGE_READ_FORMAT_OPERATION) =  UX_HOST_CLASS_STORAGE_SCSI_READ_FORMAT_CAPACITY;
    
    /* Store the number of sectors to read.  */
    _ux_utility_short_put_big_endian(cbw + UX_HOST_CLASS_STORAGE_CBW_CB + UX_HOST_CLASS_STORAGE_READ_FORMAT_PARAMETER_LIST_LENGTH, UX_HOST_CLASS_STORAGE_READ_FORMAT_RESPONSE_LENGTH);

    /* Obtain a block of memory for the answer.  */
    read_format_capacity_response =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_STORAGE_READ_FORMAT_RESPONSE_LENGTH);
    if (read_format_capacity_response == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

#if defined(UX_HOST_STANDALONE)

    /* Initialize main states for GetFormatCapacity().  */
    UX_HOST_CLASS_STORAGE_TRANS_STATE_RESET(storage);
    storage -> ux_host_class_storage_memory = read_format_capacity_response;
    storage -> ux_host_class_storage_state_state = UX_HOST_CLASS_STORAGE_STATE_TRANSPORT;
    storage -> ux_host_class_storage_state_next = UX_HOST_CLASS_STORAGE_STATE_FORMAT_CAP_SAVE;
    storage -> ux_host_class_storage_trans_data = read_format_capacity_response;
    return(UX_SUCCESS);
#else

    /* Send the command to transport layer.  */
    _ux_host_class_storage_transport(storage, read_format_capacity_response);

    /* Free the memory resource used for the command response.  */
    _ux_utility_memory_free(read_format_capacity_response);

    /* If we have a transport error, there is not much we can do, we do not fail.  */
    return(UX_SUCCESS);                                            
#endif
}

