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
/**   Device RNDIS Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_rndis.h"
#include "ux_device_stack.h"



/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_rndis_msg_query                    PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function analyzes and replies to the MSG QUERY                 */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    rndis                           Pointer to rndis class              */ 
/*    transfer_request                Pointer to the transfer request     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_long_get            Get 32-bit value                    */
/*    _ux_utility_long_put            Put 32-bit value                    */
/*    _ux_utility_string_length_check Check and return C string length    */
/*    _ux_utility_memory_copy         Copy memory                         */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    RNDIS Class                                                         */
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
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_rndis_msg_query(UX_SLAVE_CLASS_RNDIS *rndis, UX_SLAVE_TRANSFER *transfer_request)
{

UCHAR            *rndis_msg;
UCHAR            *rndis_response;
ULONG            rndis_oid;
ULONG            rndis_response_length;
UINT             rndis_response_string_length =  0;
ULONG            oid_index;
ULONG            status;

    /* Get the pointer to the RNDIS message.  */
    rndis_msg = transfer_request -> ux_slave_transfer_request_data_pointer;
    
    /* Get the request ID and keep it for the response.  */
    rndis -> ux_slave_class_rndis_request_id =  _ux_utility_long_get(rndis_msg + UX_DEVICE_CLASS_RNDIS_MSG_QUERY_REQUEST_ID);
    
    /* Get the OID.  */
    rndis_oid = _ux_utility_long_get(rndis_msg + UX_DEVICE_CLASS_RNDIS_MSG_QUERY_OID);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_RNDIS_MSG_QUERY, rndis, rndis_oid, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Now prepare the response.  */
    rndis_response = rndis -> ux_slave_class_rndis_response;
    
    /* First store the command. */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_MESSAGE_TYPE, UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY);
    
    /* Store the request ID.  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_REQUEST_ID, rndis -> ux_slave_class_rndis_request_id);

    /* By default the function will succeed. */
    status = UX_DEVICE_CLASS_RNDIS_STATUS_SUCCESS;

    /* What OID are we dealing here ? */
    switch (rndis_oid)
    {
 
         case UX_DEVICE_CLASS_RNDIS_OID_GEN_SUPPORTED_LIST        :
 
             /* Supported List of OIDs.  Parse each OID until the end and store it in the response buffer. */
              oid_index = 0;
            while (ux_device_class_rndis_oid_supported_list[oid_index] != 0)
            {

                /* We have found a valid OID to return.  */
                _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER + (oid_index * sizeof(ULONG)), 
                                        ux_device_class_rndis_oid_supported_list[oid_index]);

                /* Next OID index.  */
                oid_index++;

            }     
            
            /* Set the total response length.  */
            rndis_response_length = oid_index * (ULONG)sizeof(ULONG);
        
            break; 

        case UX_DEVICE_CLASS_RNDIS_OID_GEN_MAXIMUM_FRAME_SIZE    :

            /* Set the maximum frame size.  */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, UX_DEVICE_CLASS_RNDIS_MAX_FRAME_SIZE);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);
            
            break;
            
        case UX_DEVICE_CLASS_RNDIS_OID_GEN_MEDIA_SUPPORTED         :

            /* Set the media supported.  */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, UX_DEVICE_CLASS_RNDIS_MEDIA_802_3);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);
        
            break;
            
        case UX_DEVICE_CLASS_RNDIS_OID_GEN_MEDIA_IN_USE            :

            /* Set the media in use.  */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, UX_DEVICE_CLASS_RNDIS_MEDIA_802_3);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);
        
            break;

        case UX_DEVICE_CLASS_RNDIS_OID_GEN_HARDWARE_STATUS            :

            /* Set the hardware status.  */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, UX_DEVICE_CLASS_RNDIS_OID_HW_STATUS_READY);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);
        
            break;

        case UX_DEVICE_CLASS_RNDIS_OID_GEN_PHYSICAL_MEDIUM            :

            /* Set the physical medium.  */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, 0);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);
        
            break;

        case UX_DEVICE_CLASS_RNDIS_OID_GEN_MAXIMUM_TOTAL_SIZE            :

            /* Set the physical medium.  */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, UX_DEVICE_CLASS_RNDIS_MAX_PACKET_LENGTH);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);
        
            break;


        case UX_DEVICE_CLASS_RNDIS_OID_GEN_LINK_SPEED            :

            /* Set the link speed.  For now we assume a full speed device.  */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, UX_DEVICE_CLASS_RNDIS_LINK_SPEED_FS);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);
        
            break;

        case UX_DEVICE_CLASS_RNDIS_OID_GEN_MEDIA_CONNECT_STATUS            :

            /* Set the media connection status.  */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, UX_DEVICE_CLASS_RNDIS_MEDIA_CONNECTED);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);
        
            break;


        case UX_DEVICE_CLASS_RNDIS_OID_GEN_MAC_OPTIONS                    :

            /* Set the MAC options.  */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, UX_DEVICE_CLASS_RNDIS_MAC_OPTIONS);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);
        
            break;

        case UX_DEVICE_CLASS_RNDIS_OID_GEN_VENDOR_ID                     :

            /* Set the vendor ID.  */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, rndis -> ux_slave_class_rndis_parameter.ux_slave_class_rndis_parameter_vendor_id);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);

            break;
        
        case UX_DEVICE_CLASS_RNDIS_OID_GEN_DRIVER_VERSION                  :

            /* Set the driver version.  */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, rndis -> ux_slave_class_rndis_parameter.ux_slave_class_rndis_parameter_driver_version);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);

            break;
        
        
        case UX_DEVICE_CLASS_RNDIS_OID_GEN_VENDOR_DESCRIPTION            :

            /* Get the string length for the vendor description.  */
            status = _ux_utility_string_length_check(rndis -> ux_slave_class_rndis_parameter.ux_slave_class_rndis_parameter_vendor_description, &rndis_response_string_length, UX_DEVICE_CLASS_RNDIS_VENDOR_DESCRIPTION_MAX_LENGTH);
            if (status)
                return(status);

            /* Set the total response length.  */
            rndis_response_length =  (ULONG) rndis_response_string_length;

            /* Copy the vendor description.  */
            _ux_utility_memory_copy(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, rndis -> ux_slave_class_rndis_parameter.ux_slave_class_rndis_parameter_vendor_description, rndis_response_length); /* Use case of memcpy is verified. */

            break;
        
        case UX_DEVICE_CLASS_RNDIS_OID_GEN_XMIT_OK                        :

            /* Set the appropriate statistic value. */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, rndis -> ux_slave_class_rndis_statistics_xmit_ok);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);
        
            break;

        case UX_DEVICE_CLASS_RNDIS_OID_GEN_RCV_OK                        :

            /* Set the appropriate statistic value. */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, rndis -> ux_slave_class_rndis_statistics_rcv_ok);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);
        
            break;

        case UX_DEVICE_CLASS_RNDIS_OID_GEN_XMIT_ERROR                    :

            /* Set the appropriate statistic value. */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, rndis -> ux_slave_class_rndis_statistics_xmit_error);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);
        
            break;

        case UX_DEVICE_CLASS_RNDIS_OID_GEN_RCV_ERROR                    :

            /* Set the appropriate statistic value. */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, rndis -> ux_slave_class_rndis_statistics_rcv_error);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);
        
            break;

        case UX_DEVICE_CLASS_RNDIS_OID_GEN_RCV_NO_BUFFER                :

            /* Set the appropriate statistic value. */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, rndis -> ux_slave_class_rndis_statistics_rcv_no_buffer);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);
        
            break;

         case UX_DEVICE_CLASS_RNDIS_OID_802_3_CURRENT_ADDRESS            :
 
            
            /* Save the Hardware address in the return message.   */
            _ux_utility_memory_copy(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, 
                                    rndis -> ux_slave_class_rndis_remote_node_id, UX_DEVICE_CLASS_RNDIS_NODE_ID_LENGTH); /* Use case of memcpy is verified. */

            /* Set the total response length.  */
            rndis_response_length = UX_DEVICE_CLASS_RNDIS_NODE_ID_LENGTH;
        
            break; 


        default                                                            :

            /* Just return zero ULONG field. */
            _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER, 0);
        
            /* Set the total response length.  */
            rndis_response_length = sizeof(ULONG);
        
            break;

    }
        
    /* Set the status field.  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_STATUS, status);

    /* Set the buffer offset value.  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER_OFFSET, 
                            (UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER - UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_REQUEST_ID));
    
    /* Store the length of the buffer.  */
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER_LENGTH, rndis_response_length); 

    /* Update the response length to add the header.  */
    rndis_response_length += UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER;
            
    /* Set the response length.  */
    rndis -> ux_slave_class_rndis_response_length =   rndis_response_length;
    _ux_utility_long_put(rndis_response + UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_MESSAGE_LENGTH, rndis_response_length);

    /* We are done. Return UX_SUCCESS.  */
    return(status);        
}

