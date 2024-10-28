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


#if defined(UX_HOST_STANDALONE) && defined(UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT)
#error Only Mass Storage BulkOnly (BO) is supported in standalone mode right now.
#endif
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_storage_device_support_check         PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function verifies the support for the subclass and protocol    */
/*    of the USB device. If the device is supported it will update the    */
/*    storage class instances with the transport layer functions.         */
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
/*    None                                                                */
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
UINT  _ux_host_class_storage_device_support_check(UX_HOST_CLASS_STORAGE *storage)
{

    /* Check for the protocol type (BO/CB/CBI) and update the transport functions.  */
    switch(storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceProtocol)
    {

    case UX_HOST_CLASS_STORAGE_PROTOCOL_BO:

#if !defined(UX_HOST_STANDALONE)
        storage -> ux_host_class_storage_transport =  _ux_host_class_storage_transport_bo;
#endif
        break;

#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    case UX_HOST_CLASS_STORAGE_PROTOCOL_CB:

#if !defined(UX_HOST_STANDALONE)
        storage -> ux_host_class_storage_transport =  _ux_host_class_storage_transport_cb;
#endif
        break;


    case UX_HOST_CLASS_STORAGE_PROTOCOL_CBI:

#if !defined(UX_HOST_STANDALONE)

        /* In case of CBI, the subclass must be UFI, if not, default back to CB transport.  */
        if (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_STORAGE_SUBCLASS_UFI)
            storage -> ux_host_class_storage_transport =  _ux_host_class_storage_transport_cbi;
        else
            storage -> ux_host_class_storage_transport =  _ux_host_class_storage_transport_cb;
#endif
        break;

#endif

    default:

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_PROTOCOL_ERROR);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_PROTOCOL_ERROR, storage, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_PROTOCOL_ERROR);
    }

    /* Check for the sub class and make sure we support it.  */
    switch (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceSubClass)
    {

#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    case UX_HOST_CLASS_STORAGE_SUBCLASS_UFI:
#endif
    case UX_HOST_CLASS_STORAGE_SUBCLASS_RBC:
    case UX_HOST_CLASS_STORAGE_SUBCLASS_SCSI:
    case UX_HOST_CLASS_STORAGE_SUBCLASS_SFF8020:
    case UX_HOST_CLASS_STORAGE_SUBCLASS_SFF8070:

        return(UX_SUCCESS);

    default:

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_PROTOCOL_ERROR);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_PROTOCOL_ERROR, storage, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_PROTOCOL_ERROR);
    }
}
