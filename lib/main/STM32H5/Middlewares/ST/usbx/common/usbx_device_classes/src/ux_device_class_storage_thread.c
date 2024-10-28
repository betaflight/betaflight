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

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_storage.h"
#include "ux_device_stack.h"


#if !defined(UX_DEVICE_STANDALONE)
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_storage_thread                     PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the thread of the storage class.                   */ 
/*                                                                        */
/*    It's for RTOS mode.                                                 */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    class                                 Address of storage class      */ 
/*                                            container                   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_class_storage_format       Storage class format          */ 
/*    _ux_device_class_storage_inquiry      Storage class inquiry         */ 
/*    _ux_device_class_storage_mode_select  Mode select                   */ 
/*    _ux_device_class_storage_mode_sense   Mode sense                    */ 
/*    _ux_device_class_storage_prevent_allow_media_removal                */ 
/*                                          Prevent media removal         */ 
/*    _ux_device_class_storage_read         Read                          */ 
/*    _ux_device_class_storage_read_capacity                              */
/*                                          Read capacity                 */ 
/*    _ux_device_class_storage_read_format_capacity                       */ 
/*                                          Read format capacity          */ 
/*    _ux_device_class_storage_request_sense                              */
/*                                          Sense request                 */ 
/*    _ux_device_class_storage_start_stop   Start/Stop                    */
/*    _ux_device_class_storage_synchronize_cache                          */ 
/*                                          Synchronize cache             */
/*    _ux_device_class_storage_test_ready   Ready test                    */ 
/*    _ux_device_class_storage_verify       Verify                        */ 
/*    _ux_device_class_storage_write        Write                         */
/*    _ux_device_stack_endpoint_stall       Endpoint stall                */ 
/*    _ux_device_stack_interface_delete     Interface delete              */ 
/*    _ux_device_stack_transfer_request     Transfer request              */ 
/*    _ux_utility_long_get                  Get 32-bit value              */ 
/*    _ux_utility_memory_allocate           Allocate memory               */ 
/*    _ux_device_semaphore_create           Create semaphore              */ 
/*    _ux_utility_delay_ms                  Sleep thread for several ms   */
/*    _ux_device_thread_suspend             Suspend thread                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    ThreadX                                                             */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used sleep instead of       */
/*                                            relinquish on error,        */
/*                                            optimized command logic,    */
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  12-31-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed USB CV test issues,   */
/*                                            resulting in version 6.1.3  */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile issues with   */
/*                                            some macro options,         */
/*                                            resulting in version 6.1.6  */
/*  06-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            get interface and endpoints */
/*                                            from configured device,     */
/*                                            resulting in version 6.1.7  */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved TAG management,    */
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_device_class_storage_thread(ULONG storage_class)
{

UX_SLAVE_CLASS              *class_ptr;
UX_SLAVE_CLASS_STORAGE      *storage;
UX_SLAVE_TRANSFER           *transfer_request;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_INTERFACE          *interface_ptr;
UX_SLAVE_ENDPOINT           *endpoint_in;
UX_SLAVE_ENDPOINT           *endpoint_out;
UINT                        status;
ULONG                       length;
ULONG                       cbwcb_length;
ULONG                       lun;
UCHAR                       *scsi_command;
UCHAR                       *cbw_cb;


    /* This thread runs forever but can be suspended or resumed.  */
    while(1)
    {

        /* Cast properly the storage instance.  */
        UX_THREAD_EXTENSION_PTR_GET(class_ptr, UX_SLAVE_CLASS, storage_class)
        
        /* Get the storage instance from this class container.  */
        storage =  (UX_SLAVE_CLASS_STORAGE *) class_ptr -> ux_slave_class_instance;
    
        /* Get the pointer to the device.  */
        device =  &_ux_system_slave -> ux_system_slave_device;
        
        /* As long as the device is in the CONFIGURED state.  */
        while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED)
        { 

            /* We are activated. We need the interface to the class.  */
            interface_ptr =  storage -> ux_slave_class_storage_interface;

            /* We assume the worst situation.  */
            status =  UX_ERROR;

            /* Locate the endpoints.  */
            endpoint_in =  interface_ptr -> ux_slave_interface_first_endpoint;

            /* Check the endpoint direction, if IN we have the correct endpoint.  */
            if ((endpoint_in -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_IN)
            {

                /* Wrong direction, we found the OUT endpoint first.  */
                endpoint_out =  endpoint_in;

                /* So the next endpoint has to be the IN endpoint.  */
                endpoint_in =  endpoint_out -> ux_slave_endpoint_next_endpoint;
            }
            else
            {

                /* We found the endpoint IN first, so next endpoint is OUT.  */
                endpoint_out =  endpoint_in -> ux_slave_endpoint_next_endpoint;
            }

            /* All SCSI commands are on the endpoint OUT, from the host.  */
            transfer_request =  &endpoint_out -> ux_slave_endpoint_transfer_request;

            /* Check state, they must be both RESET.  */
            if (endpoint_out -> ux_slave_endpoint_state == UX_ENDPOINT_RESET &&
                (UCHAR)storage -> ux_slave_class_storage_csw_status != UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR)
            {

                /* Send the request to the device controller.  */
                status =  _ux_device_stack_transfer_request(transfer_request, 64, 64);

            }                
    
            /* Check the status. Our status is UX_ERROR if one of the endpoint was STALLED. We must wait for the host
               to clear the mess.   */    
            if (status == UX_SUCCESS)
            {

                /* Obtain the length of the transaction.  */
                length =  transfer_request -> ux_slave_transfer_request_actual_length;
                
                /* Obtain the buffer address containing the SCSI command.  */
                scsi_command =  transfer_request -> ux_slave_transfer_request_data_pointer;
                
                /* Obtain the lun from the CBW.  */
                lun =  (ULONG) *(scsi_command + UX_SLAVE_CLASS_STORAGE_CBW_LUN);
                storage -> ux_slave_class_storage_cbw_lun = (UCHAR)lun;
                
                /* We have to memorize the SCSI command tag for the CSW phase.  */
                storage -> ux_slave_class_storage_scsi_tag =  _ux_utility_long_get(scsi_command + UX_SLAVE_CLASS_STORAGE_CBW_TAG);

                /* Get dCBWDataTransferLength: number of bytes to transfer.  */
                storage -> ux_slave_class_storage_host_length = _ux_utility_long_get(scsi_command + UX_SLAVE_CLASS_STORAGE_CBW_DATA_LENGTH);

                /* Save bmCBWFlags.  */
                storage -> ux_slave_class_storage_cbw_flags = *(scsi_command + UX_SLAVE_CLASS_STORAGE_CBW_FLAGS);

                /* Reset CSW status.  */
                storage -> ux_slave_class_storage_csw_residue = 0;
                storage -> ux_slave_class_storage_csw_status = 0;

                /* Ensure the LUN number is within our declared values and check the command 
                   content and format. First we make sure we have a complete CBW.  */
                if ((lun < storage -> ux_slave_class_storage_number_lun) && (length == UX_SLAVE_CLASS_STORAGE_CBW_LENGTH))
                {

                    /* The length of the CBW is correct, analyze the header.  */
                    if (_ux_utility_long_get(scsi_command) == UX_SLAVE_CLASS_STORAGE_CBW_SIGNATURE_MASK)
                    {

                        /* Get the length of the CBWCB.  */
                        cbwcb_length =  (ULONG) *(scsi_command + UX_SLAVE_CLASS_STORAGE_CBW_CB_LENGTH);
    
                        /* Check the length of the CBWCB to ensure there is at least a command.  */
                        if (cbwcb_length != 0)
                        {

                            /* Analyze the command stored in the CBWCB.  */
                            cbw_cb = scsi_command + UX_SLAVE_CLASS_STORAGE_CBW_CB;
                            switch (*(cbw_cb))
                            {

                            case UX_SLAVE_CLASS_STORAGE_SCSI_TEST_READY:

                                _ux_device_class_storage_test_ready(storage, lun, endpoint_in, endpoint_out, cbw_cb);
                                break;
                                    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_REQUEST_SENSE:

                                _ux_device_class_storage_request_sense(storage, lun, endpoint_in, endpoint_out, cbw_cb);
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_FORMAT:

                                _ux_device_class_storage_format(storage, lun, endpoint_in, endpoint_out, cbw_cb);
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_INQUIRY:

                                _ux_device_class_storage_inquiry(storage, lun, endpoint_in, endpoint_out, cbw_cb);
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_START_STOP:

                                _ux_device_class_storage_start_stop(storage, lun, endpoint_in, endpoint_out, cbw_cb);
                                break;
                                    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_PREVENT_ALLOW_MEDIA_REMOVAL:

                                _ux_device_class_storage_prevent_allow_media_removal(storage, lun, endpoint_in, endpoint_out, cbw_cb);
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_READ_FORMAT_CAPACITY:

                                _ux_device_class_storage_read_format_capacity(storage, lun, endpoint_in, endpoint_out, cbw_cb);
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_READ_CAPACITY:

                                _ux_device_class_storage_read_capacity(storage, lun, endpoint_in, endpoint_out, cbw_cb);
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_VERIFY:

                                _ux_device_class_storage_verify(storage, lun, endpoint_in, endpoint_out, cbw_cb);
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_MODE_SELECT:

                                _ux_device_class_storage_mode_select(storage, lun, endpoint_in, endpoint_out, cbw_cb);
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_MODE_SENSE_SHORT:
                            case UX_SLAVE_CLASS_STORAGE_SCSI_MODE_SENSE:

                                _ux_device_class_storage_mode_sense(storage, lun, endpoint_in, endpoint_out, cbw_cb);
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_READ32:

                                _ux_device_class_storage_read(storage, lun, endpoint_in, endpoint_out, cbw_cb, 
                                                                UX_SLAVE_CLASS_STORAGE_SCSI_READ32);
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_READ16:

                                _ux_device_class_storage_read(storage, lun, endpoint_in, endpoint_out, cbw_cb, 
                                                                UX_SLAVE_CLASS_STORAGE_SCSI_READ16);
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_WRITE32:

                                _ux_device_class_storage_write(storage, lun, endpoint_in, endpoint_out, cbw_cb,
                                                                UX_SLAVE_CLASS_STORAGE_SCSI_WRITE32);
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_WRITE16:

                                _ux_device_class_storage_write(storage, lun, endpoint_in, endpoint_out, cbw_cb, 
                                                                UX_SLAVE_CLASS_STORAGE_SCSI_WRITE16);
                                break;

                            case UX_SLAVE_CLASS_STORAGE_SCSI_SYNCHRONIZE_CACHE:

                                _ux_device_class_storage_synchronize_cache(storage, lun, endpoint_in, endpoint_out, cbw_cb, *(cbw_cb));
                                break;

#ifdef UX_SLAVE_CLASS_STORAGE_INCLUDE_MMC
                            case UX_SLAVE_CLASS_STORAGE_SCSI_GET_STATUS_NOTIFICATION:

                                _ux_device_class_storage_get_status_notification(storage, lun, endpoint_in, endpoint_out, cbw_cb); 
                                break;

                            case UX_SLAVE_CLASS_STORAGE_SCSI_GET_CONFIGURATION:

                                _ux_device_class_storage_get_configuration(storage, lun, endpoint_in, endpoint_out, cbw_cb); 
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_READ_DISK_INFORMATION:

                                _ux_device_class_storage_read_disk_information(storage, lun, endpoint_in, endpoint_out, cbw_cb); 
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_REPORT_KEY:

                                _ux_device_class_storage_report_key(storage, lun, endpoint_in, endpoint_out, cbw_cb); 
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_GET_PERFORMANCE:

                                _ux_device_class_storage_get_performance(storage, lun, endpoint_in, endpoint_out, cbw_cb); 
                                break;
    
                            case UX_SLAVE_CLASS_STORAGE_SCSI_READ_DVD_STRUCTURE:

                                _ux_device_class_storage_read_dvd_structure(storage, lun, endpoint_in, endpoint_out, cbw_cb); 
                                break;

                            case UX_SLAVE_CLASS_STORAGE_SCSI_READ_TOC:

                                status = _ux_device_class_storage_read_toc(storage, lun, endpoint_in, endpoint_out, cbw_cb); 

                                /* Special treatment of TOC command. If error, default to Stall endpoint.  */
                                if (status == UX_SUCCESS)
                                    break;
#endif

                            /* fall through */
                            default:
    
                                /* The command is unknown or unsupported, so we stall the endpoint.  */

                                if (storage -> ux_slave_class_storage_host_length > 0 &&
                                    ((storage -> ux_slave_class_storage_cbw_flags & 0x80) == 0))

                                    /* Data-Out from host to device, stall OUT.  */
                                    _ux_device_stack_endpoint_stall(endpoint_out);
                                else

                                    /* Data-In from device to host, stall IN.  */
                                    _ux_device_stack_endpoint_stall(endpoint_in);
                                
                                /* Initialize the request sense keys.  */
                                storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_request_sense_status =
                                    UX_DEVICE_CLASS_STORAGE_SENSE_STATUS(UX_SLAVE_CLASS_STORAGE_SENSE_KEY_ILLEGAL_REQUEST,
                                                                         UX_SLAVE_CLASS_STORAGE_ASC_KEY_INVALID_COMMAND,0);

                                /* This is the tricky part of the SCSI state machine. We must send the CSW BUT need to wait
                                   for the endpoint_in to be reset by the host.  */
                                while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED)
                                { 

                                    /* Check the endpoint state.  */
                                    if (endpoint_in -> ux_slave_endpoint_state == UX_ENDPOINT_RESET)
                                    {

                                        /* Now we set the CSW with failure.  */
                                        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_FAILED;
                                        break;
                                    }                                        

                                    else

                                        /* We must therefore wait a while.  */
                                        _ux_device_thread_relinquish();
                                }
                                break;
                            }

                            /* Send CSW if not SYNC_CACHE.  */
                            status = _ux_device_class_storage_csw_send(storage, lun, endpoint_in, 0 /* Don't care */);

                            /* Check error code. */
                            if (status != UX_SUCCESS)

                                /* Error trap. */
                                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);
                        }
                        else

                            /* Phase error!  */
                            storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR;
                    }
                    
                    else

                        /* Phase error!  */
                        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR;
                }
                else

                    /* Phase error!  */
                    storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR;
            }
            else
            {

                if ((UCHAR)storage -> ux_slave_class_storage_csw_status == UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR)
                {

                    /* We should keep the endpoints stalled.  */
                    _ux_device_stack_endpoint_stall(endpoint_out);
                    _ux_device_stack_endpoint_stall(endpoint_in);
                }

                /* We must therefore wait a while.  */
                _ux_utility_delay_ms(2);
            }
        }

        /* We need to suspend ourselves. We will be resumed by the 
           device enumeration module.  */
        _ux_device_thread_suspend(&class_ptr -> ux_slave_class_thread);
    }
}
#endif
