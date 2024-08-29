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

#if defined(UX_DEVICE_STANDALONE)

/* Internal static inline implements.  */


static inline UINT _ux_device_class_storage_task_usb(UX_SLAVE_CLASS_STORAGE *storage);

static inline UINT _ux_device_class_storage_reset_wait(UX_SLAVE_CLASS_STORAGE *storage);
static inline VOID _ux_device_class_storage_cbw_receive(UX_SLAVE_CLASS_STORAGE *storage);
static inline VOID _ux_device_class_storage_cbw_process(UX_SLAVE_CLASS_STORAGE *storage);
static inline VOID _ux_device_class_storage_cmd_process(UX_SLAVE_CLASS_STORAGE *storage, UCHAR *cbw);
static inline VOID _ux_device_class_storage_data_cases_check(UX_SLAVE_CLASS_STORAGE *storage);
static inline VOID _ux_device_class_storage_trans_start(UX_SLAVE_CLASS_STORAGE *storage);
static inline UINT _ux_device_class_storage_trans_wait(UX_SLAVE_CLASS_STORAGE *storage);
static inline VOID _ux_device_class_storage_trans_error(UX_SLAVE_CLASS_STORAGE *storage);
static inline UINT _ux_device_class_storage_data_next(UX_SLAVE_CLASS_STORAGE *storage);

static inline VOID _ux_device_class_storage_halt_out(UX_SLAVE_CLASS_STORAGE *storage);
static inline VOID _ux_device_class_storage_halt_in(UX_SLAVE_CLASS_STORAGE *storage);
static inline VOID _ux_device_class_storage_halt_trans(UX_SLAVE_CLASS_STORAGE *storage);

static inline VOID _ux_device_class_storage_task_disk(UX_SLAVE_CLASS_STORAGE *storage);

static inline VOID _ux_device_class_storage_disk_start(UX_SLAVE_CLASS_STORAGE *storage);
static inline UINT _ux_device_class_storage_disk_wait(UX_SLAVE_CLASS_STORAGE *storage);
static inline VOID _ux_device_class_storage_disk_next(UX_SLAVE_CLASS_STORAGE *storage);
static inline VOID _ux_device_class_storage_disk_read_next(UX_SLAVE_CLASS_STORAGE *storage);
static inline VOID _ux_device_class_storage_disk_write_next(UX_SLAVE_CLASS_STORAGE *storage);
static inline VOID _ux_device_class_storage_disk_error(UX_SLAVE_CLASS_STORAGE *storage);


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_storage_tasks_run                  PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function runs tasks of the storage class.                      */
/*    E.g., CBW-DATA-CSW state machine.                                   */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    instance                              Address of storage instance   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    UX_STATE_RESET                        Tasks suspended               */
/*    UX_STATE_IDLE                         Activated but no task ran     */
/*    (others > UX_STATE_IDLE)              Tasks running                 */
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
/*    _ux_utility_semaphore_create          Create semaphore              */
/*    _ux_utility_thread_suspend            Suspend thread                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Device Stack                                                        */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved internal logic,    */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_storage_tasks_run(VOID *instance)
{

UX_SLAVE_CLASS_STORAGE      *storage;
UINT                        status;


    /* Get storage instance.  */
    storage = (UX_SLAVE_CLASS_STORAGE *) instance;

    /* Run USB and disk tasks.  */
    status = _ux_device_class_storage_task_usb(storage);
    _ux_device_class_storage_task_disk(storage);
    return(status);
}

static inline UINT _ux_device_class_storage_task_usb(UX_SLAVE_CLASS_STORAGE *storage)
{
UX_SLAVE_DEVICE             *device;
UCHAR                       state;
UINT                        status;
INT                         immediate_state = UX_TRUE;


    /* Get pointer to the device.  */
    device = &_ux_system_slave -> ux_system_slave_device;

    /* Run states once.  */
    while(immediate_state)
    {

        /* General check for MSC ready.  */
        if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED ||
            storage -> ux_device_class_storage_ep_in == UX_NULL ||
            storage -> ux_device_class_storage_ep_out == UX_NULL)
        {
            storage -> ux_device_class_storage_state = UX_STATE_RESET;
            return(UX_STATE_EXIT);
        }

        /* Get current state.  */
        state = storage -> ux_device_class_storage_state;

        /* Handle different state.  */
        switch(state)
        {

        case UX_STATE_RESET: /* Initial, reset.  */
            _ux_device_class_storage_cbw_receive(storage);

            /* Roll back to next state directly.  */
            continue;

        case UX_DEVICE_CLASS_STORAGE_STATE_RESET:
            _ux_device_class_storage_halt_in(storage);
            _ux_device_class_storage_halt_out(storage);
            storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_RESET_WAIT;

            /* Fall through.  */
        case UX_DEVICE_CLASS_STORAGE_STATE_RESET_WAIT: /* Wait reset recovery.  */
            return _ux_device_class_storage_reset_wait(storage);

        case UX_DEVICE_CLASS_STORAGE_STATE_TRANS_START:
            _ux_device_class_storage_trans_start(storage);

            /* Fall through.  */
        case UX_DEVICE_CLASS_STORAGE_STATE_TRANS_WAIT:
            status = _ux_device_class_storage_trans_wait(storage);

            /* Fatal case.  */
            if (status < UX_STATE_ERROR)
            {

                /* USB idle.  */
                storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_IDLE;

                /* Disk notified with USB error.  */
                if (storage -> ux_device_class_storage_disk_state !=
                    UX_DEVICE_CLASS_STORAGE_DISK_IDLE)
                {
                    storage -> ux_device_class_storage_disk_state =
                                        UX_DEVICE_CLASS_STORAGE_DISK_USB_ERROR;
                }
                return(UX_STATE_WAIT);
            }

            /* Error case.  */
            if (status == UX_STATE_ERROR)
            {

                /* Stall.  */
                _ux_device_class_storage_halt_trans(storage);

                /* Disk notified with USB error.  */
                if (storage -> ux_device_class_storage_disk_state !=
                    UX_DEVICE_CLASS_STORAGE_DISK_IDLE)
                {
                    storage -> ux_device_class_storage_disk_state =
                                        UX_DEVICE_CLASS_STORAGE_DISK_USB_ERROR;
                }

                /* Update residue.  */
                storage -> ux_slave_class_storage_csw_residue =
                        storage -> ux_slave_class_storage_host_length -
                        storage -> ux_device_class_storage_data_count;

                /* Update the REQUEST_SENSE codes.  */
                storage -> ux_slave_class_storage_lun[
                    storage -> ux_slave_class_storage_cbw_lun].
                        ux_slave_class_storage_request_sense_status =
                            UX_DEVICE_CLASS_STORAGE_SENSE_STATUS(0x02,0x54,0x00);

                /* Issue CSW.  */
                _ux_device_class_storage_csw_send(storage,
                                    storage -> ux_slave_class_storage_cbw_lun,
                                    storage -> ux_device_class_storage_ep_in, 0);

                return(UX_STATE_WAIT);
            }

            /* Success case.  */
            if (status == UX_STATE_NEXT)
            {

                /* Update data count.  */
                storage -> ux_device_class_storage_data_count +=
                        storage -> ux_device_class_storage_transfer ->
                                ux_slave_transfer_request_actual_length;

                storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_TRANS_NEXT;
            }

            /* Other cases, keep waiting.  */
            return(UX_STATE_WAIT);

        case UX_DEVICE_CLASS_STORAGE_STATE_TRANS_NEXT:

            /* CBW received: -> CBW handle.  */
            if (storage -> ux_device_class_storage_cmd_state == UX_DEVICE_CLASS_STORAGE_CMD_CBW)
            {
                _ux_device_class_storage_cbw_process(storage);

                /* Apply command in next call anyway.  */
                return(UX_STATE_WAIT);
            }

            /* CSW sent: -> CSW done - CBW start.  */
            if (storage -> ux_device_class_storage_cmd_state == UX_DEVICE_CLASS_STORAGE_CMD_CSW)
            {
                if (storage -> ux_slave_class_storage_csw_status == UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR)
                {
                    storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_RESET;
                }
                else
                {
                    _ux_device_class_storage_cbw_receive(storage);
                }
                continue;
            }

            /* DATA done: -> process data.  */
            return _ux_device_class_storage_data_next(storage);

        case UX_DEVICE_CLASS_STORAGE_STATE_DISK_ERROR:
            _ux_device_class_storage_trans_error(storage);
            continue;

        case UX_DEVICE_CLASS_STORAGE_STATE_IDLE: /* Nothing to do, fall through.  */

        case UX_DEVICE_CLASS_STORAGE_STATE_DISK_WAIT: /* Nothing to do, fall through.  */

        default: /* Do nothing.  */
            break;
        }

        /* Unhandled, just break the loop and do again by app call.  */
        immediate_state = UX_FALSE;
    }

    /* Unhandled state.  */
    return(UX_STATE_EXIT);
}

static inline VOID _ux_device_class_storage_cbw_receive(UX_SLAVE_CLASS_STORAGE *storage)
{
UX_SLAVE_ENDPOINT   *endpoint;
UX_SLAVE_TRANSFER   *transfer;
ULONG               max_packet_size;


    /* Command state: CBW.  */
    storage -> ux_device_class_storage_cmd_state = UX_DEVICE_CLASS_STORAGE_CMD_CBW;
    storage -> ux_device_class_storage_data_buffer = UX_NULL;

    /* Transfer state: START: OUT 31.  */
    storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_TRANS_START;

    endpoint = storage -> ux_device_class_storage_ep_out;
    max_packet_size = endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize;
    transfer = &endpoint -> ux_slave_endpoint_transfer_request;

    storage -> ux_device_class_storage_transfer = transfer;
    storage -> ux_device_class_storage_data_length = max_packet_size;
    storage -> ux_device_class_storage_data_count = 0;
}

static inline VOID _ux_device_class_storage_cbw_process(UX_SLAVE_CLASS_STORAGE *storage)
{
UX_SLAVE_TRANSFER   *cbw_trans;
ULONG               cbw_length;
UCHAR               *cbw;


    /* Get transfer.  */
    cbw_trans = storage -> ux_device_class_storage_transfer;

    /* If CBW is stalled, just retry.  */
    if (cbw_trans -> ux_slave_transfer_request_completion_code == UX_TRANSFER_STALLED)
    {
        _ux_device_class_storage_cbw_receive(storage);
        return;
    }

    /* Get CBW and length.  */
    cbw_length = cbw_trans -> ux_slave_transfer_request_actual_length;
    cbw = cbw_trans -> ux_slave_transfer_request_data_pointer;
    if (cbw_length != UX_SLAVE_CLASS_STORAGE_CBW_LENGTH)
    {
        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR;
    }
    else
    {
        _ux_device_class_storage_cmd_process(storage, cbw);
    }

    /* If still in CBW phase, there must be CBW structure error, wait reset.  */
    if (storage -> ux_device_class_storage_cmd_state == UX_DEVICE_CLASS_STORAGE_CMD_CBW)
    {
        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR;
        storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_RESET;
        return;
    }

    /* If no error, error cases need check.  */
    if (storage -> ux_slave_class_storage_csw_status == UX_SLAVE_CLASS_STORAGE_CSW_PASSED)
    {
        _ux_device_class_storage_data_cases_check(storage);
    }

    /* Error, stall and send CSW.  */
    if (storage -> ux_slave_class_storage_csw_status)
    {

        /* There will not be any disk operation in this case.  */
        storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_IDLE;

        if (storage -> ux_slave_class_storage_host_length &&
            (storage -> ux_slave_class_storage_cbw_flags &
                UX_DEVICE_CLASS_STORAGE_CBW_FLAG_DIR) == 0)
        {
            _ux_device_class_storage_halt_out(storage);
        }
        else
        {
            _ux_device_class_storage_halt_in(storage);
        }

        /* Still need CSW phase.  */
        if (storage -> ux_device_class_storage_cmd_state != UX_DEVICE_CLASS_STORAGE_CMD_CBW)
            storage -> ux_device_class_storage_cmd_state = UX_DEVICE_CLASS_STORAGE_CMD_ERR;
    }

    /* No error, state not changed.  */

    /* Start CSW if there is no data.  */
    if (storage -> ux_device_class_storage_cmd_state < UX_DEVICE_CLASS_STORAGE_CMD_WRITE)
    {
        _ux_device_class_storage_csw_send(storage,
                                storage -> ux_slave_class_storage_cbw_lun,
                                storage -> ux_device_class_storage_ep_in, 0);
    }
}
static inline VOID _ux_device_class_storage_cmd_process(UX_SLAVE_CLASS_STORAGE *storage, UCHAR *cbw)
{

UX_SLAVE_ENDPOINT   *endpoint_in, *endpoint_out;
ULONG               cbwcb_length;
UCHAR               *cbwcb;
UCHAR               lun;


    /* Get bCBWLUN.  */
    lun = *(cbw + UX_SLAVE_CLASS_STORAGE_CBW_LUN);
    storage -> ux_slave_class_storage_cbw_lun = lun;

    /* Get bmCBWFlags.  */
    storage -> ux_slave_class_storage_cbw_flags = *(cbw + UX_SLAVE_CLASS_STORAGE_CBW_FLAGS);

    /* Get dCBWTag.  */
    storage -> ux_slave_class_storage_scsi_tag =
                    _ux_utility_long_get(cbw + UX_SLAVE_CLASS_STORAGE_CBW_TAG);

    /* Get dCBWDataTransferLength: number of bytes to transfer.  */
    storage -> ux_slave_class_storage_host_length = _ux_utility_long_get(cbw + UX_SLAVE_CLASS_STORAGE_CBW_DATA_LENGTH);

    /* Reset CSW status.  */
    storage -> ux_slave_class_storage_csw_residue = 0;
    storage -> ux_slave_class_storage_csw_status = 0;

    /* Check LUN error.  */
    if (lun >= storage -> ux_slave_class_storage_number_lun)
    {

        /* Phase error!  */
        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR;
        return;
    }

    /* Check Signature error.  */
    if (_ux_utility_long_get(cbw) != UX_SLAVE_CLASS_STORAGE_CBW_SIGNATURE_MASK)
    {

        /* Phase error!  */
        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR;
        return;
    }

    /* Get CBWCB length.  */
    cbwcb_length = (ULONG)*(cbw + UX_SLAVE_CLASS_STORAGE_CBW_CB_LENGTH);

    /* Check CBWCB length.  */
    if (cbwcb_length == 0)
    {

        /* Phase error!  */
        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR;
        return;
    }

    /* Get endpoints.  */
    endpoint_in = storage -> ux_device_class_storage_ep_in;
    endpoint_out = storage -> ux_device_class_storage_ep_out;

    /* By default set next command state to CSW (no DATA).  */
    storage -> ux_device_class_storage_cmd_state = UX_DEVICE_CLASS_STORAGE_CMD_CSW;
    storage -> ux_device_class_storage_device_length = 0;
    storage -> ux_device_class_storage_data_length = 0;
    storage -> ux_device_class_storage_data_count = 0;

    /* Reset disk access state.  */
    storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_IDLE;

    /* Analyze the CBWCB command.  */
    cbwcb = cbw + UX_SLAVE_CLASS_STORAGE_CBW_CB;
    storage -> ux_device_class_storage_cmd = *cbwcb;
    switch(storage -> ux_device_class_storage_cmd)
    {

    case UX_SLAVE_CLASS_STORAGE_SCSI_TEST_READY:

        _ux_device_class_storage_test_ready(storage, lun, endpoint_in, endpoint_out, cbwcb);
        break;
            
    case UX_SLAVE_CLASS_STORAGE_SCSI_REQUEST_SENSE:

        _ux_device_class_storage_request_sense(storage, lun, endpoint_in, endpoint_out, cbwcb);
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_FORMAT:

        _ux_device_class_storage_format(storage, lun, endpoint_in, endpoint_out, cbwcb);
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_INQUIRY:

        _ux_device_class_storage_inquiry(storage, lun, endpoint_in, endpoint_out, cbwcb);
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_START_STOP:

        _ux_device_class_storage_start_stop(storage, lun, endpoint_in, endpoint_out, cbwcb);
        break;
            
    case UX_SLAVE_CLASS_STORAGE_SCSI_PREVENT_ALLOW_MEDIA_REMOVAL:

        _ux_device_class_storage_prevent_allow_media_removal(storage, lun, endpoint_in, endpoint_out, cbwcb);
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_READ_FORMAT_CAPACITY:

        _ux_device_class_storage_read_format_capacity(storage, lun, endpoint_in, endpoint_out, cbwcb);
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_READ_CAPACITY:

        _ux_device_class_storage_read_capacity(storage, lun, endpoint_in, endpoint_out, cbwcb);
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_VERIFY:

        _ux_device_class_storage_verify(storage, lun, endpoint_in, endpoint_out, cbwcb);
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_MODE_SELECT:

        _ux_device_class_storage_mode_select(storage, lun, endpoint_in, endpoint_out, cbwcb);
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_MODE_SENSE_SHORT:
    case UX_SLAVE_CLASS_STORAGE_SCSI_MODE_SENSE:

        _ux_device_class_storage_mode_sense(storage, lun, endpoint_in, endpoint_out, cbwcb);
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_READ32:

        _ux_device_class_storage_read(storage, lun, endpoint_in, endpoint_out, cbwcb, 
                                        UX_SLAVE_CLASS_STORAGE_SCSI_READ32);
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_READ16:

        _ux_device_class_storage_read(storage, lun, endpoint_in, endpoint_out, cbwcb, 
                                        UX_SLAVE_CLASS_STORAGE_SCSI_READ16);
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_WRITE32:

        _ux_device_class_storage_write(storage, lun, endpoint_in, endpoint_out, cbwcb,
                                        UX_SLAVE_CLASS_STORAGE_SCSI_WRITE32);
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_WRITE16:

        _ux_device_class_storage_write(storage, lun, endpoint_in, endpoint_out, cbwcb, 
                                        UX_SLAVE_CLASS_STORAGE_SCSI_WRITE16);
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_SYNCHRONIZE_CACHE:

        _ux_device_class_storage_synchronize_cache(storage, lun, endpoint_in, endpoint_out, cbwcb, *(cbwcb));
        break;

#ifdef UX_SLAVE_CLASS_STORAGE_INCLUDE_MMC
    case UX_SLAVE_CLASS_STORAGE_SCSI_GET_STATUS_NOTIFICATION:

        _ux_device_class_storage_get_status_notification(storage, lun, endpoint_in, endpoint_out, cbwcb); 
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_GET_CONFIGURATION:

        _ux_device_class_storage_get_configuration(storage, lun, endpoint_in, endpoint_out, cbwcb); 
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_READ_DISK_INFORMATION:

        _ux_device_class_storage_read_disk_information(storage, lun, endpoint_in, endpoint_out, cbwcb); 
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_REPORT_KEY:

        _ux_device_class_storage_report_key(storage, lun, endpoint_in, endpoint_out, cbwcb); 
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_GET_PERFORMANCE:

        _ux_device_class_storage_get_performance(storage, lun, endpoint_in, endpoint_out, cbwcb); 
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_READ_DVD_STRUCTURE:

        _ux_device_class_storage_read_dvd_structure(storage, lun, endpoint_in, endpoint_out, cbwcb); 
        break;

    case UX_SLAVE_CLASS_STORAGE_SCSI_READ_TOC:

        _ux_device_class_storage_read_toc(storage, lun, endpoint_in, endpoint_out, cbwcb); 
        break;

#endif

    default:

        /* The command is unknown or unsupported, fail.  */
        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_FAILED;

        /* Initialize the request sense keys.  */
        storage -> ux_slave_class_storage_lun[lun].ux_slave_class_storage_request_sense_status =
            UX_DEVICE_CLASS_STORAGE_SENSE_STATUS(UX_SLAVE_CLASS_STORAGE_SENSE_KEY_ILLEGAL_REQUEST,
                                                UX_SLAVE_CLASS_STORAGE_ASC_KEY_INVALID_COMMAND,0);
    }
}
static inline VOID _ux_device_class_storage_data_cases_check(UX_SLAVE_CLASS_STORAGE *storage)
{

    /* ============ 13 cases check.  */
    /* (1)Hn=Dn, (2)Hn<Di, (3)Hn<Do                          : CBW   */
    /* (4)Hi>Dn, (5)Hi>Di, (6)Hi=Di, (7)Hi<Di, (8)Hi<>Do     : DATA  */
    /* (9)Ho>Dn, (10)Ho<>Di, (11)Ho>Do, (12)Ho=Do, (13)Ho<Do : DATA  */
    /* Hn -----------------------------  */
    /* Case (1)      : Success/Error  */
    /* Case (2)(3)   : STALL(IN/OUT), PhaseError.  */
    /* Hi -------------------------------------------------  */
    /* Case (6)      : Send DeviceLength, Success/Error.  */
    /* Case (4)(5)   : Send DeviceLength, STALL(IN), Success/Error, dCSWDataResidue.  */
    /* Case (7)(8)   : (Send HostLength), STALL(IN), PhaseError.  */
    /* Ho -----------------------------------------------------------  */
    /* Case (12)     : Receive, Success/Error.  */
    /* Case (9)(11)  : STALL(OUT), Success/Error, dCSWDataResidue.  */
    /* Case (10)(13) : STALL(OUT), PhaseError.  */
    if (storage -> ux_slave_class_storage_cbw_flags & UX_DEVICE_CLASS_STORAGE_CBW_FLAG_DIR)
    {

        /* Hi.  */
        /* Case (2), (8).  */
        if (storage -> ux_device_class_storage_cmd_state == UX_DEVICE_CLASS_STORAGE_CMD_WRITE ||
            storage -> ux_slave_class_storage_host_length == 0)
        {

            /* Phase error.  */
            storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR;
            return;
        }

        /* Case (7).  */
        if (storage -> ux_slave_class_storage_host_length <
            storage -> ux_device_class_storage_data_length)
        {

            /* Part of data will be sent.  */
            storage -> ux_device_class_storage_data_length =
                storage -> ux_slave_class_storage_host_length;
        }

        /* Case (4), (5), (6), (7).  */
        /* No touch for prepared transfer.  */
    }
    else
    {

        /* Ho.  */
        /* Case (3), (10), (13).  */
        if (storage -> ux_slave_class_storage_host_length <
                storage -> ux_device_class_storage_device_length ||
            storage -> ux_device_class_storage_cmd_state ==
                UX_DEVICE_CLASS_STORAGE_CMD_READ)
        {

            /* Phase error.  */
            storage -> ux_slave_class_storage_csw_status =
                                    UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR;
            return;
        }

        /* Case (9), (11).  */
        if (storage -> ux_slave_class_storage_host_length !=
            storage -> ux_device_class_storage_device_length)
        {

            /* Update dCSWDataResidue.  */
            storage -> ux_slave_class_storage_csw_residue =
                    storage -> ux_slave_class_storage_host_length -
                        storage -> ux_device_class_storage_device_length;

            /* Failed.  */
            storage -> ux_slave_class_storage_csw_status =
                                        UX_SLAVE_CLASS_STORAGE_CSW_FAILED;
            return;
        }
    }
}
static inline UINT _ux_device_class_storage_reset_wait(UX_SLAVE_CLASS_STORAGE *storage)
{

    /* Check PhaseError.  */
    if ((UCHAR)storage -> ux_slave_class_storage_csw_status !=
                                        UX_SLAVE_CLASS_STORAGE_CSW_PHASE_ERROR)
    {

        /* Reset states.  */
        storage -> ux_device_class_storage_state = UX_STATE_RESET;
    }
    else
    {

        /* Keep CBW endpoints halted until status is reset.  */
        storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_RESET;
    }

    return(UX_STATE_WAIT);
}
static inline UINT _ux_device_class_storage_data_next(UX_SLAVE_CLASS_STORAGE *storage)
{
    switch(storage -> ux_device_class_storage_cmd)
    {

    /* Disk read.  */
    case UX_SLAVE_CLASS_STORAGE_SCSI_READ16:
        /* Fall through.  */
    case UX_SLAVE_CLASS_STORAGE_SCSI_READ32:

        /* Check if all data is done.  */
        if (storage -> ux_device_class_storage_data_count >=
            storage -> ux_device_class_storage_data_length)
        {

            /* Stall if host expects more data.  */
            if (storage -> ux_slave_class_storage_host_length >
                storage -> ux_device_class_storage_data_length)
            {
                _ux_device_class_storage_halt_in(storage);

                /* Update dCSWDataResidue.  */
                storage -> ux_slave_class_storage_csw_residue =
                    storage -> ux_slave_class_storage_host_length -
                    storage -> ux_device_class_storage_data_length;
            }

            /* Set to fail if device expects more data.  */
            if (storage -> ux_device_class_storage_device_length >
                storage -> ux_device_class_storage_data_count)
            {
                
                /* Update bCSWStatus.  */
                storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_FAILED;
            }

            /* Issue CSW.  */
            _ux_device_class_storage_csw_send(storage,
                        storage -> ux_slave_class_storage_cbw_lun,
                        storage -> ux_device_class_storage_ep_in,
                        0 /* Not used.  */);
        }
        else
        {

            /* Buffer sent, update buffer states.  */
            storage -> ux_device_class_storage_buffer_state[storage -> ux_device_class_storage_buffer_usb] =
                    UX_DEVICE_CLASS_STORAGE_BUFFER_EMPTY;
            storage -> ux_device_class_storage_buffer_usb = !storage -> ux_device_class_storage_buffer_usb;
            storage -> ux_device_class_storage_transfer -> ux_slave_transfer_request_data_pointer =
                    storage -> ux_device_class_storage_buffer[storage -> ux_device_class_storage_buffer_usb];

            /* If disk read not started (waiting free buffer), start it.  */
            if (storage -> ux_device_class_storage_disk_state == UX_DEVICE_CLASS_STORAGE_DISK_USB_WAIT)
            {
                storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_OP_START;
            }

            /* If there is buffer ready, send next.  */
            if (storage -> ux_device_class_storage_buffer_state[storage->ux_device_class_storage_buffer_usb] ==
                UX_DEVICE_CLASS_STORAGE_BUFFER_FULL)
            {
                storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_TRANS_START;
            }
            else
            {

                /* Wait disk operation to fill buffer.  */
                storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_DISK_WAIT;
            }
        }
        break;

    /* Disk write.  */
    case UX_SLAVE_CLASS_STORAGE_SCSI_WRITE16:
        /* Fall through.  */
    case UX_SLAVE_CLASS_STORAGE_SCSI_WRITE32:

        /* Buffer received, update buffer state.  */
        storage -> ux_device_class_storage_buffer_state[
                        storage -> ux_device_class_storage_buffer_usb] =
                                            UX_DEVICE_CLASS_STORAGE_BUFFER_FULL;

        /* If disk waiting data, Start disk write.  */
        if (storage -> ux_device_class_storage_disk_state ==
            UX_DEVICE_CLASS_STORAGE_DISK_USB_WAIT)
        {
            storage -> ux_device_class_storage_disk_state =
                                    UX_DEVICE_CLASS_STORAGE_DISK_OP_START;
        }

        /* Check if all data is done.  */
        if (storage -> ux_device_class_storage_data_count >=
            storage -> ux_device_class_storage_data_length)
        {

            /* Wait disk operation done.  */
            storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_DISK_WAIT;
        }
        else
        {

            /* Buffer received, update buffer states.  */
            storage -> ux_device_class_storage_buffer_usb =
                                !storage -> ux_device_class_storage_buffer_usb;
            storage -> ux_device_class_storage_transfer ->
                ux_slave_transfer_request_data_pointer =
                            storage -> ux_device_class_storage_buffer[
                                storage -> ux_device_class_storage_buffer_usb];

            /* If there is buffer empty, start it.  */
            if (storage -> ux_device_class_storage_buffer_state[
                    storage -> ux_device_class_storage_buffer_usb] ==
                UX_DEVICE_CLASS_STORAGE_BUFFER_EMPTY)
            {
                storage -> ux_device_class_storage_state =
                                    UX_DEVICE_CLASS_STORAGE_STATE_TRANS_START;
            }
            else
            {

                /* No buffer available, wait disk operation done.  */
                storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_DISK_WAIT;
            }
        }
        break;

    /* No further data to send.  */
    default:

        /* Stall if host expects more data.  */
        if (storage -> ux_slave_class_storage_host_length >
            storage -> ux_device_class_storage_data_count)
        {
            _ux_device_class_storage_halt_trans(storage);

            /* Update dCSWDataResidue.  */
            storage -> ux_slave_class_storage_csw_residue =
                storage -> ux_slave_class_storage_host_length -
                storage -> ux_device_class_storage_data_length;
        }

        /* Set to fail if device expects more data.  */
        if (storage -> ux_device_class_storage_device_length >
            storage -> ux_device_class_storage_data_count)
        {
            
            /* Update bCSWStatus.  */
            storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_FAILED;
        }

        /* Issue CSW.  */
        _ux_device_class_storage_csw_send(storage,
                    storage -> ux_slave_class_storage_cbw_lun,
                    storage -> ux_device_class_storage_ep_in,
                    0 /* Not used.  */);
    }

    /* Next task state.  */
    return(UX_STATE_NEXT);
}
static inline VOID _ux_device_class_storage_trans_start(UX_SLAVE_CLASS_STORAGE *storage)
{
ULONG   remaining, host_length, device_length;


    /* Get remaining transfer length.  */
    remaining = storage -> ux_device_class_storage_data_length - storage -> ux_device_class_storage_data_count;

    /* Check if data exceeds buffer length.  */
    if (remaining > UX_SLAVE_CLASS_STORAGE_BUFFER_SIZE)
    {

        /* Send full packets, without ZLP.  */
        host_length = UX_SLAVE_CLASS_STORAGE_BUFFER_SIZE;
        device_length = UX_SLAVE_CLASS_STORAGE_BUFFER_SIZE;
    }
    else
    {

        /* Send packets sliced based on host length and device length.  */
        device_length = remaining;

        /* USB CV test expecting stall but not ZLP, so host length is same, but not remaining.  */
        host_length = remaining;
    }

    /* Prepare data if necessary.  */
    if (storage -> ux_device_class_storage_data_buffer && device_length &&
        storage -> ux_device_class_storage_cmd_state == UX_DEVICE_CLASS_STORAGE_CMD_READ)
    {
        _ux_utility_memory_copy(storage -> ux_device_class_storage_transfer ->
                                    ux_slave_transfer_request_data_pointer,
                                storage -> ux_device_class_storage_data_buffer +
                                    storage -> ux_device_class_storage_data_count,
                                device_length); /* Use case of memcpy is verified. */
    }

    /* Save host length and device length for task states.  */
    storage -> ux_device_class_storage_trans_device_length = device_length;
    storage -> ux_device_class_storage_trans_host_length = host_length;

    /* To TRANS_WAIT.  */
    storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_TRANS_WAIT;

    /* Reset transfer state.  */
    UX_SLAVE_TRANSFER_STATE_RESET(storage -> ux_device_class_storage_transfer);
}
static inline UINT _ux_device_class_storage_trans_wait(UX_SLAVE_CLASS_STORAGE *storage)
{
    return _ux_device_stack_transfer_run(storage -> ux_device_class_storage_transfer,
                                        storage -> ux_device_class_storage_trans_device_length,
                                        storage -> ux_device_class_storage_trans_host_length);
}
static inline VOID _ux_device_class_storage_trans_error(UX_SLAVE_CLASS_STORAGE *storage)
{

    /* Abort USB operation if transfer not done.  */
    if (storage -> ux_device_class_storage_data_count < storage -> ux_device_class_storage_data_length)
    {
        if (storage -> ux_slave_class_storage_cbw_flags & UX_DEVICE_CLASS_STORAGE_CBW_FLAG_IN)
        {
            _ux_device_class_storage_halt_in(storage);
        }
        else
        {
            _ux_device_class_storage_halt_out(storage);
        }

        /* Update dCSWDataResidue.  */
        storage -> ux_slave_class_storage_csw_residue =
            storage -> ux_slave_class_storage_host_length -
            storage -> ux_device_class_storage_data_count;
    }
    else
    {
        _ux_device_class_storage_halt_in(storage);
    }

    /* Issue CSW.  */
    _ux_device_class_storage_csw_send(storage,
            storage -> ux_slave_class_storage_cbw_lun,
            storage -> ux_device_class_storage_ep_in,
            0 /* Not used.  */);
}
static inline VOID _ux_device_class_storage_halt_out(UX_SLAVE_CLASS_STORAGE *storage)
{
    _ux_device_stack_endpoint_stall(storage -> ux_device_class_storage_ep_out);
}
static inline VOID _ux_device_class_storage_halt_in(UX_SLAVE_CLASS_STORAGE *storage)
{
    _ux_device_stack_endpoint_stall(storage -> ux_device_class_storage_ep_in);
}
static inline VOID _ux_device_class_storage_halt_trans(UX_SLAVE_CLASS_STORAGE *storage)
{
UX_SLAVE_TRANSFER *trans = storage -> ux_device_class_storage_transfer;
UX_SLAVE_ENDPOINT *endp = trans -> ux_slave_transfer_request_endpoint;
    _ux_device_stack_endpoint_stall(endp);
}


static inline VOID _ux_device_class_storage_task_disk(UX_SLAVE_CLASS_STORAGE *storage)
{
UCHAR                   state = storage -> ux_device_class_storage_disk_state;
UINT                    status;
INT                     immediate_state = UX_TRUE;

    /* Run states once.  */
    while(immediate_state)
    {

        /* Update state.  */
        state = storage -> ux_device_class_storage_disk_state;
        switch(state)
        {

        case UX_DEVICE_CLASS_STORAGE_DISK_OP_START:
            _ux_device_class_storage_disk_start(storage);
            storage -> ux_device_class_storage_disk_state =
                                        UX_DEVICE_CLASS_STORAGE_DISK_OP_WAIT;

            /* Fall through.  */
        case UX_DEVICE_CLASS_STORAGE_DISK_OP_WAIT:
            status = _ux_device_class_storage_disk_wait(storage);

            /* Error case.  */
            if (status < UX_STATE_NEXT)
            {

                /* Disk idle.  */
                storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_IDLE;

                /* USB notified with disk error.  */
                storage -> ux_device_class_storage_state = UX_DEVICE_CLASS_STORAGE_STATE_DISK_ERROR;
            }

            /* Success case.  */
            if (status == UX_STATE_NEXT)
            {

                storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_OP_NEXT;
            }

            /* Other cases, keep waiting.  */
            return;

        case UX_DEVICE_CLASS_STORAGE_DISK_OP_NEXT:
            _ux_device_class_storage_disk_next(storage);
            return;

        case UX_DEVICE_CLASS_STORAGE_DISK_USB_ERROR:
            _ux_device_class_storage_disk_error(storage);
            break;

        case UX_DEVICE_CLASS_STORAGE_DISK_USB_WAIT: /* Nothing to do, fall through.  */

        case UX_DEVICE_CLASS_STORAGE_DISK_IDLE: /* Nothing to do, fall through.  */

        default: /* Nothing to do.  */
            break;
        }

        /* Task run once, break the loop.  */
        immediate_state = UX_FALSE;
    }
}
static inline VOID _ux_device_class_storage_disk_start(UX_SLAVE_CLASS_STORAGE *storage)
{
ULONG block_size;
ULONG max_n_blocks;


    if (storage -> ux_device_class_storage_cmd == UX_SLAVE_CLASS_STORAGE_SCSI_SYNCHRONIZE_CACHE)
    {

        /* All things sync in one call.  */
        storage -> ux_device_class_storage_disk_n_lb = storage -> ux_device_class_storage_cmd_n_lb;
        return;
    }

    /* Read/write, split the operation by buffer sizes.  */

    /* Max blocks for one buffer.  */
    block_size = storage -> ux_slave_class_storage_lun[storage -> ux_slave_class_storage_cbw_lun].
                                                        ux_slave_class_storage_media_block_length;
    if (block_size == 0)
        UX_ASSERT(UX_FALSE);
    max_n_blocks = UX_SLAVE_CLASS_STORAGE_BUFFER_SIZE / block_size;

    /* Prepare next disk read.  */
    if (storage -> ux_device_class_storage_cmd_n_lb > max_n_blocks)
    {
        storage -> ux_device_class_storage_disk_n_lb = max_n_blocks;
    }
    else
    {
        storage -> ux_device_class_storage_disk_n_lb = storage -> ux_device_class_storage_cmd_n_lb;
    }
}
static inline UINT _ux_device_class_storage_disk_wait(UX_SLAVE_CLASS_STORAGE *storage)
{
    switch (storage -> ux_device_class_storage_cmd)
    {
    case UX_SLAVE_CLASS_STORAGE_SCSI_READ16:
    case UX_SLAVE_CLASS_STORAGE_SCSI_READ32:
        return storage -> ux_slave_class_storage_lun[storage -> ux_slave_class_storage_cbw_lun].
                        ux_slave_class_storage_media_read(storage,
                            storage -> ux_slave_class_storage_cbw_lun,
                            storage -> ux_device_class_storage_buffer[
                                storage -> ux_device_class_storage_buffer_disk],
                            storage -> ux_device_class_storage_disk_n_lb,
                            storage -> ux_device_class_storage_cmd_lba,
                            &storage -> ux_device_class_storage_media_status);

    case UX_SLAVE_CLASS_STORAGE_SCSI_WRITE16:
    case UX_SLAVE_CLASS_STORAGE_SCSI_WRITE32:
        return storage -> ux_slave_class_storage_lun[storage -> ux_slave_class_storage_cbw_lun].
                        ux_slave_class_storage_media_write(storage,
                            storage -> ux_slave_class_storage_cbw_lun,
                            storage -> ux_device_class_storage_buffer[
                                storage -> ux_device_class_storage_buffer_disk],
                            storage -> ux_device_class_storage_disk_n_lb,
                            storage -> ux_device_class_storage_cmd_lba,
                            &storage -> ux_device_class_storage_media_status);

    case UX_SLAVE_CLASS_STORAGE_SCSI_SYNCHRONIZE_CACHE:
        return storage -> ux_slave_class_storage_lun[storage -> ux_slave_class_storage_cbw_lun].
                        ux_slave_class_storage_media_flush(storage,
                            storage -> ux_slave_class_storage_cbw_lun,
                            storage -> ux_device_class_storage_disk_n_lb,
                            storage -> ux_device_class_storage_cmd_lba,
                            &storage -> ux_device_class_storage_media_status);

    case UX_SLAVE_CLASS_STORAGE_SCSI_VERIFY: /* No nothing for now.  */
    default:
        break;
    }
    return(UX_STATE_NEXT);
}
static inline VOID _ux_device_class_storage_disk_next(UX_SLAVE_CLASS_STORAGE *storage)
{

    /* Update disk operation status.  */
    storage -> ux_device_class_storage_cmd_lba += storage -> ux_device_class_storage_disk_n_lb;
    storage -> ux_device_class_storage_cmd_n_lb -= storage -> ux_device_class_storage_disk_n_lb;

    /* Next check is different for read/write.  */
    switch (storage -> ux_device_class_storage_cmd)
    {
    case UX_SLAVE_CLASS_STORAGE_SCSI_READ16:
    case UX_SLAVE_CLASS_STORAGE_SCSI_READ32:
        _ux_device_class_storage_disk_read_next(storage);
        return;

    case UX_SLAVE_CLASS_STORAGE_SCSI_WRITE16:
    case UX_SLAVE_CLASS_STORAGE_SCSI_WRITE32:
        _ux_device_class_storage_disk_write_next(storage);
        return;

    case UX_SLAVE_CLASS_STORAGE_SCSI_SYNCHRONIZE_CACHE:

        /* Disk is idle now.  */
        storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_IDLE;

        /* Send CSW if not sent yet.  */
        if (storage -> ux_device_class_storage_state == UX_DEVICE_CLASS_STORAGE_STATE_DISK_WAIT)
        {
            _ux_device_class_storage_csw_send(storage,
                    storage -> ux_slave_class_storage_cbw_lun,
                    storage -> ux_device_class_storage_ep_in,
                    0 /* Not used.  */);
        }
        return;

    default:
        storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_IDLE;
        break;
    }
}
static inline VOID _ux_device_class_storage_disk_read_next(UX_SLAVE_CLASS_STORAGE *storage)
{

    /* Update buffer state : full.  */
    storage -> ux_device_class_storage_buffer_state[
        storage -> ux_device_class_storage_buffer_disk] =
                                    UX_DEVICE_CLASS_STORAGE_BUFFER_FULL;

    /* Check if all disk operation is done.  */
    if (storage -> ux_device_class_storage_cmd_n_lb == 0)
    {
        storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_IDLE;
    }
    else
    {

        /* Update buffer index.  */
        storage -> ux_device_class_storage_buffer_disk =
                            !storage -> ux_device_class_storage_buffer_disk;

        /* If buffer is free, start next read.  */
        if (UX_DEVICE_CLASS_STORAGE_BUFFER_EMPTY ==
            storage -> ux_device_class_storage_buffer_state[
                            storage -> ux_device_class_storage_buffer_disk])
        {

            /* Start next read.  */
            storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_OP_START;
        }
        else
        {

            /* Wait until buffer sent by USB.  */
            storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_USB_WAIT;
        }
    }

    /* Start USB transfer.  */
    if (storage -> ux_device_class_storage_state ==
            UX_DEVICE_CLASS_STORAGE_STATE_DISK_WAIT &&
        UX_DEVICE_CLASS_STORAGE_BUFFER_FULL ==
        storage -> ux_device_class_storage_buffer_state[
            storage -> ux_device_class_storage_buffer_usb])
    {
        storage -> ux_device_class_storage_state =
                                UX_DEVICE_CLASS_STORAGE_STATE_TRANS_START;
    }
}
static inline VOID _ux_device_class_storage_disk_write_next(UX_SLAVE_CLASS_STORAGE *storage)
{

    /* Update buffer state : empty.  */
    storage -> ux_device_class_storage_buffer_state[
            storage -> ux_device_class_storage_buffer_disk] =
                                    UX_DEVICE_CLASS_STORAGE_BUFFER_EMPTY;

    /* Check if all disk operation is done.  */
    if (storage -> ux_device_class_storage_cmd_n_lb == 0)
    {

        /* Disk is idle now.  */
        storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_IDLE;

        /* Issue CSW.  */
        _ux_device_class_storage_csw_send(storage,
                    storage -> ux_slave_class_storage_cbw_lun,
                    storage -> ux_device_class_storage_ep_in,
                    0 /* Not used.  */);
    }
    else
    {

        /* Update buffer index.  */
        storage -> ux_device_class_storage_buffer_disk =
                             !storage -> ux_device_class_storage_buffer_disk;

        /* If buffer is full, start next write.  */
        if (UX_DEVICE_CLASS_STORAGE_BUFFER_FULL ==
            storage -> ux_device_class_storage_buffer_state[
                storage -> ux_device_class_storage_buffer_disk])
        {

            /* Start next write.  */
            storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_OP_START;
        }
        else
        {

            /* Wait until buffer filled by USB.  */
            storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_USB_WAIT;
        }

        /* Start USB transfer.  */
        if (storage -> ux_device_class_storage_state ==
                UX_DEVICE_CLASS_STORAGE_STATE_DISK_WAIT &&
            UX_DEVICE_CLASS_STORAGE_BUFFER_EMPTY ==
            storage -> ux_device_class_storage_buffer_state[
                storage -> ux_device_class_storage_buffer_usb])
        {
            storage -> ux_device_class_storage_state =
                                    UX_DEVICE_CLASS_STORAGE_STATE_TRANS_START;
        }
    }
}
static inline VOID _ux_device_class_storage_disk_error(UX_SLAVE_CLASS_STORAGE *storage)
{
    /* Abort disk operation: read or write with NULL!  */
    switch (storage -> ux_device_class_storage_cmd)
    {
    case UX_SLAVE_CLASS_STORAGE_SCSI_READ16:
    case UX_SLAVE_CLASS_STORAGE_SCSI_READ32:
        storage -> ux_slave_class_storage_lun[storage -> ux_slave_class_storage_cbw_lun].
                ux_slave_class_storage_media_read(storage,
                        storage -> ux_slave_class_storage_cbw_lun, UX_NULL, 0, 0, UX_NULL);
        break;
    case UX_SLAVE_CLASS_STORAGE_SCSI_WRITE16:
    case UX_SLAVE_CLASS_STORAGE_SCSI_WRITE32:
        storage -> ux_slave_class_storage_lun[storage -> ux_slave_class_storage_cbw_lun].
                ux_slave_class_storage_media_write(storage,
                        storage -> ux_slave_class_storage_cbw_lun, UX_NULL, 0, 0, UX_NULL);
        break;
    default:
        break;
    }

    /* Change disk state to IDLE.  */
    storage -> ux_device_class_storage_disk_state = UX_DEVICE_CLASS_STORAGE_DISK_IDLE;
}

#endif
