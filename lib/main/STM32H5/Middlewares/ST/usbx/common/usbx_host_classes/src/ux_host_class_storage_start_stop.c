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
/*    _ux_host_class_storage_start_stop                   PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function starts or stops the UFI device.                       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    storage                               Pointer to storage class      */ 
/*    start_stop_flag                       1 or 0 if start/stop          */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_storage_cbw_initialize Initialize the CBW            */ 
/*    _ux_host_class_storage_transport      Send transport layer command  */ 
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
UINT  _ux_host_class_storage_start_stop(UX_HOST_CLASS_STORAGE *storage, 
                                            ULONG start_stop_signal)
{

UINT            status;
UCHAR             *cbw;
UINT            command_length;
ULONG           command_retry;
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_STORAGE_START_STOP, storage, start_stop_signal, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Use a pointer for the CBW, easier to manipulate.  */
    cbw =  (UCHAR *) storage -> ux_host_class_storage_cbw;

    /* Get the START_STOP Command Length.  */
#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    if (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_STORAGE_SUBCLASS_UFI)
        command_length =  UX_HOST_CLASS_STORAGE_START_STOP_COMMAND_LENGTH_UFI;
    else
        command_length =  UX_HOST_CLASS_STORAGE_START_STOP_COMMAND_LENGTH_SBC;
#else
    command_length =  UX_HOST_CLASS_STORAGE_START_STOP_COMMAND_LENGTH_SBC;
#endif

    /* Initialize the CBW for this command.  */
    _ux_host_class_storage_cbw_initialize(storage, 0, 0, command_length);
    
    /* Prepare the START STOP command block.  */
    *(cbw + UX_HOST_CLASS_STORAGE_CBW_CB + UX_HOST_CLASS_STORAGE_START_STOP_OPERATION) =  UX_HOST_CLASS_STORAGE_SCSI_START_STOP;

    /* Set the required signal.  */
    *(cbw + UX_HOST_CLASS_STORAGE_CBW_CB + UX_HOST_CLASS_STORAGE_START_STOP_START_BIT) =  (UCHAR) start_stop_signal;

    /* On floppies, this operation tends to fail a few times. So we try harder.  */
    for (command_retry = 0; command_retry < 50; command_retry++)
    {
        
        /* Send the command to transport layer.  */
        status =  _ux_host_class_storage_transport(storage, UX_NULL);

        /* If we have a transport error give up. */
        if(status != UX_SUCCESS)    

            /* Return completion status.  */
            return(status);                                            

        /* Check the CSW. We may learn something there about the state of the device.  */
        if (storage -> ux_host_class_storage_sense_code == 0) 
            return(UX_SUCCESS);
    }

    /* The start/Stop did not work.  */
    return(UX_ERROR);
}

