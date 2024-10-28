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
/*    _ux_host_class_storage_unit_ready_test              PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will verify that a SCSI unit is ready for data        */ 
/*    transfer. This command is used when the device does not mount       */ 
/*    when power is supplied.                                             */ 
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_unit_ready_test(UX_HOST_CLASS_STORAGE *storage)
{

UINT            status;
UCHAR           *cbw;
UINT            command_length;
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_STORAGE_UNIT_READY_TEST, storage, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Use a pointer for the CBW, easier to manipulate.  */
    cbw =  (UCHAR *) storage -> ux_host_class_storage_cbw;

    /* Get the Unit Ready Test Command Length.  */
#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    if (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_STORAGE_SUBCLASS_UFI)
        command_length =  UX_HOST_CLASS_STORAGE_TEST_READY_COMMAND_LENGTH_UFI;
    else
        command_length =  UX_HOST_CLASS_STORAGE_TEST_READY_COMMAND_LENGTH_SBC;
#else
    command_length =  UX_HOST_CLASS_STORAGE_TEST_READY_COMMAND_LENGTH_SBC;
#endif

    /* Initialize the CBW for this command.  */
    _ux_host_class_storage_cbw_initialize(storage, 0, 0, command_length);
    
    /* Prepare the TEST UNIT READY command block.  */
    *(cbw + UX_HOST_CLASS_STORAGE_CBW_CB + UX_HOST_CLASS_STORAGE_TEST_READY_OPERATION) =  UX_HOST_CLASS_STORAGE_SCSI_TEST_READY;

#if defined(UX_HOST_STANDALONE)

    /* Prepare states.  */
    UX_HOST_CLASS_STORAGE_TRANS_STATE_RESET(storage);
    storage -> ux_host_class_storage_state_state = UX_HOST_CLASS_STORAGE_STATE_TRANSPORT;
    storage -> ux_host_class_storage_state_next = UX_HOST_CLASS_STORAGE_STATE_TEST_CHECK;
    status = UX_SUCCESS;
#else

    /* Send the command to transport layer.  */
    status =  _ux_host_class_storage_transport(storage, UX_NULL);
#endif

    /* Return completion status.  */
    return(status);                                            
}
