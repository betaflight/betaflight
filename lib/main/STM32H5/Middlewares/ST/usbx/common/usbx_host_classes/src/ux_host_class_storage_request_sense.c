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
/*    _ux_host_class_storage_request_sense                PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will send a request sense to the device to see what   */ 
/*    error happened during the last command. Request sense commands      */ 
/*    cannot be nested.                                                   */ 
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
/*    _ux_utility_memory_copy               Copy memory block             */ 
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
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_request_sense(UX_HOST_CLASS_STORAGE *storage)
{

UCHAR           *cbw;
UCHAR           *request_sense_response;
UINT            command_length;
#if !defined(UX_HOST_STANDALONE)
UINT            status;
ULONG           sense_code;
#endif

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_STORAGE_REQUEST_SENSE, storage, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Clear the former sense code value.  */
    storage -> ux_host_class_storage_sense_code =  0;
    
    /* Use a pointer for the cbw, easier to manipulate.  */
    cbw =  (UCHAR *) storage -> ux_host_class_storage_cbw;

    /* Get the Request Sense Command Length.  */
#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    if (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_STORAGE_SUBCLASS_UFI)
        command_length =  UX_HOST_CLASS_STORAGE_REQUEST_SENSE_COMMAND_LENGTH_UFI;
    else
        command_length =  UX_HOST_CLASS_STORAGE_REQUEST_SENSE_COMMAND_LENGTH_SBC;
#else
    command_length =  UX_HOST_CLASS_STORAGE_REQUEST_SENSE_COMMAND_LENGTH_SBC;
#endif

    /* Check of we are reentering a REQUEST SENSE command which is illegal.  */
    if (*(cbw + UX_HOST_CLASS_STORAGE_CBW_CB + UX_HOST_CLASS_STORAGE_REQUEST_SENSE_OPERATION) == UX_HOST_CLASS_STORAGE_SCSI_REQUEST_SENSE)
        return(UX_ERROR);
                                                
    /* Save the current command so that we can re-initiate it after the 
       REQUEST_SENSE command.  */
    _ux_utility_memory_copy(storage -> ux_host_class_storage_saved_cbw, storage -> ux_host_class_storage_cbw, UX_HOST_CLASS_STORAGE_CBW_LENGTH); /* Use case of memcpy is verified. */

    /* Initialize the CBW for this command.  */
    _ux_host_class_storage_cbw_initialize(storage, UX_HOST_CLASS_STORAGE_DATA_IN, UX_HOST_CLASS_STORAGE_REQUEST_SENSE_RESPONSE_LENGTH, command_length);
    
    /* Prepare the REQUEST SENSE command block.  */
    *(cbw + UX_HOST_CLASS_STORAGE_CBW_CB + UX_HOST_CLASS_STORAGE_REQUEST_SENSE_OPERATION) =  UX_HOST_CLASS_STORAGE_SCSI_REQUEST_SENSE;
    
    /* Store the length of the Request Sense Response.  */
    *(cbw + UX_HOST_CLASS_STORAGE_CBW_CB + UX_HOST_CLASS_STORAGE_REQUEST_SENSE_ALLOCATION_LENGTH) =  UX_HOST_CLASS_STORAGE_REQUEST_SENSE_RESPONSE_LENGTH;

    /* Obtain a block of memory for the answer.  */
    request_sense_response =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_STORAGE_REQUEST_SENSE_RESPONSE_LENGTH);
    if (request_sense_response == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

#if defined(UX_HOST_STANDALONE)

    /* Prepare data buffer for request sense.  */
    storage -> ux_host_class_storage_trans_data_bak = storage -> ux_host_class_storage_trans_data;
    storage -> ux_host_class_storage_trans_data = request_sense_response;

    /* Perform CBW-DATA-CSW transport.  */
    storage -> ux_host_class_storage_trans_state = UX_HOST_CLASS_STORAGE_TRANS_CBW;

    return(UX_SUCCESS);
#else

    /* Send the command to transport layer.  */
    status =  _ux_host_class_storage_transport(storage, request_sense_response);

    /* If we have a transport error, there is not much we can do, simply return the error.  */
    if (status == UX_SUCCESS)
    {

        /* We have a successful transaction, even though the sense code could reflect an error. The sense code 
           will be assembled and store in the device instance.  */ 
        sense_code = UX_HOST_CLASS_STORAGE_SENSE_STATUS(
            (ULONG) *(request_sense_response +
                      UX_HOST_CLASS_STORAGE_REQUEST_SENSE_RESPONSE_SENSE_KEY),
            (ULONG) *(request_sense_response +
                      UX_HOST_CLASS_STORAGE_REQUEST_SENSE_RESPONSE_CODE),
            (ULONG) *(request_sense_response +
                      UX_HOST_CLASS_STORAGE_REQUEST_SENSE_RESPONSE_CODE_QUALIFIER));

        /* Store the sense code in the storage instance.  */
        storage -> ux_host_class_storage_sense_code =  sense_code;
    }       

    /* Free the memory resource used for the command response.  */
    _ux_utility_memory_free(request_sense_response);

    /* Restore the current CBW command.  */
    _ux_utility_memory_copy(storage -> ux_host_class_storage_cbw, storage -> ux_host_class_storage_saved_cbw, UX_HOST_CLASS_STORAGE_CBW_LENGTH); /* Use case of memcpy is verified. */

    /* Return completion code.  */    
    return(status);                                            
#endif
}

