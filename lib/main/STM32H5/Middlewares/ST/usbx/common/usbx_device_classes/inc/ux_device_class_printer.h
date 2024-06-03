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
/**   Device Printer Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

/**************************************************************************/
/*                                                                        */
/*  COMPONENT DEFINITION                                   RELEASE        */
/*                                                                        */
/*    ux_device_class_printer.h                           PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This file defines the equivalences for the USBX Device Class        */
/*    Printer component.                                                  */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.12 */
/*  10-31-2022     Yajun xia                Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/

#ifndef UX_DEVICE_CLASS_PRINTER_H
#define UX_DEVICE_CLASS_PRINTER_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard
   C is used to process the API information.  */

#ifdef   __cplusplus

/* Yes, C++ compiler is present.  Use standard C.  */
extern   "C" {

#endif

/* Defined, _write is pending ZLP automatically (complete transfer) after buffer is sent.  */

/* #define UX_DEVICE_CLASS_PRINTER_WRITE_AUTO_ZLP  */


/* Define Printer Class USB Class constants.  */
#define UX_DEVICE_CLASS_PRINTER_CLASS                                    7

#define UX_DEVICE_CLASS_PRINTER_SUBCLASS                                 1

#define UX_DEVICE_CLASS_PRINTER_PROTOCOL_UNIDIRECTIONAL                  1
#define UX_DEVICE_CLASS_PRINTER_PROTOCOL_BIDIRECTIONAL                   2
#define UX_DEVICE_CLASS_PRINTER_PROTOCOL_1284_4_COMPATIBLE_BIDIR         3


/* Device Printer Requests */
#define UX_DEVICE_CLASS_PRINTER_GET_DEVICE_ID                            0x00
#define UX_DEVICE_CLASS_PRINTER_GET_PORT_STATUS                          0x01
#define UX_DEVICE_CLASS_PRINTER_SOFT_RESET                               0x02


/* Printer Port Status.  */
#define UX_DEVICE_CLASS_PRINTER_PAPER_EMPTY                             (1u << 5)
#define UX_DEVICE_CLASS_PRINTER_SELECT                                  (1u << 4)
#define UX_DEVICE_CLASS_PRINTER_NOT_ERROR                               (1u << 3)


/* Printer IOCTL code.  */
#define UX_DEVICE_CLASS_PRINTER_IOCTL_PORT_STATUS_SET                   1
#define UX_DEVICE_CLASS_PRINTER_IOCTL_READ_TIMEOUT_SET                  2
#define UX_DEVICE_CLASS_PRINTER_IOCTL_WRITE_TIMEOUT_SET                 3

#if defined(UX_DEVICE_STANDALONE)

/* Printer read state machine states.  */
#define UX_DEVICE_CLASS_PRINTER_READ_START                              (UX_STATE_STEP + 1)
#define UX_DEVICE_CLASS_PRINTER_READ_WAIT                               (UX_STATE_STEP + 2)

/* Printer write state machine states.  */
#define UX_DEVICE_CLASS_PRINTER_WRITE_START                             (UX_STATE_STEP + 1)
#define UX_DEVICE_CLASS_PRINTER_WRITE_WAIT                              (UX_STATE_STEP + 2)
#endif

/* Define Device Printer Class Calling Parameter structure */

typedef struct UX_DEVICE_CLASS_PRINTER_PARAMETER_STRUCT
{
    UCHAR                   *ux_device_class_printer_device_id; /* IEEE 1284 string, first 2 big endian length.  */
    VOID                    (*ux_device_class_printer_instance_activate)(VOID *);
    VOID                    (*ux_device_class_printer_instance_deactivate)(VOID *);
    VOID                    (*ux_device_class_printer_soft_reset)(VOID *);
} UX_DEVICE_CLASS_PRINTER_PARAMETER;


/* Define Printer Class structure.  */

typedef struct UX_DEVICE_CLASS_PRINTER_STRUCT
{
    UX_SLAVE_INTERFACE      *ux_device_class_printer_interface;
    UX_SLAVE_ENDPOINT       *ux_device_class_printer_endpoint_out;
    UX_SLAVE_ENDPOINT       *ux_device_class_printer_endpoint_in;
    ULONG                   ux_device_class_printer_port_status;
    UX_DEVICE_CLASS_PRINTER_PARAMETER
                            ux_device_class_printer_parameter;
#if !defined(UX_DEVICE_STANDALONE)
    UX_MUTEX                ux_device_class_printer_endpoint_out_mutex;
    UX_MUTEX                ux_device_class_printer_endpoint_in_mutex;
#else
    UCHAR                  *ux_device_class_printer_read_buffer;
    ULONG                   ux_device_class_printer_read_requested_length;
    ULONG                   ux_device_class_printer_read_transfer_length;
    ULONG                   ux_device_class_printer_read_actual_length;
    UINT                    ux_device_class_printer_read_status;
    UINT                    ux_device_class_printer_read_state;

    UCHAR                  *ux_device_class_printer_write_buffer;
    ULONG                   ux_device_class_printer_write_transfer_length;
    ULONG                   ux_device_class_printer_write_host_length;
    ULONG                   ux_device_class_printer_write_requested_length;
    ULONG                   ux_device_class_printer_write_actual_length;
    UINT                    ux_device_class_printer_write_status;
    UINT                    ux_device_class_printer_write_state;
#endif
} UX_DEVICE_CLASS_PRINTER;


/* Define Device Printer Class prototypes.  */

UINT  _ux_device_class_printer_activate(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_printer_control_request(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_printer_deactivate(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_printer_entry(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_printer_initialize(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_printer_uninitialize(UX_SLAVE_CLASS_COMMAND *command);

VOID  _ux_device_class_printer_soft_reset(UX_DEVICE_CLASS_PRINTER *printer);

UINT  _ux_device_class_printer_write(UX_DEVICE_CLASS_PRINTER *printer, UCHAR *buffer,
                                ULONG requested_length, ULONG *actual_length);
UINT  _ux_device_class_printer_read(UX_DEVICE_CLASS_PRINTER *printer, UCHAR *buffer,
                                ULONG requested_length, ULONG *actual_length);

UINT  _ux_device_class_printer_ioctl(UX_DEVICE_CLASS_PRINTER *printer, ULONG ioctl_function,
                                    VOID *parameter);
#if defined(UX_DEVICE_STANDALONE)
UINT  _ux_device_class_printer_write_run(UX_DEVICE_CLASS_PRINTER *printer, UCHAR *buffer,
                                ULONG requested_length, ULONG *actual_length);
UINT  _ux_device_class_printer_read_run(UX_DEVICE_CLASS_PRINTER *printer, UCHAR *buffer,
                                ULONG requested_length, ULONG *actual_length);
#endif

/* Define Device Printer Class API prototypes.  */

#define ux_device_class_printer_entry               _ux_device_class_printer_entry
#define ux_device_class_printer_read                _ux_device_class_printer_read
#define ux_device_class_printer_write               _ux_device_class_printer_write
#define ux_device_class_printer_ioctl               _ux_device_class_printer_ioctl

#if defined(UX_DEVICE_STANDALONE)
#define ux_device_class_printer_read_run            _ux_device_class_printer_read_run
#define ux_device_class_printer_write_run           _ux_device_class_printer_write_run
#endif

/* Determine if a C++ compiler is being used.  If so, complete the standard
   C conditional started above.  */
#ifdef __cplusplus
}
#endif

#endif /* UX_DEVICE_CLASS_PRINTER_H */
