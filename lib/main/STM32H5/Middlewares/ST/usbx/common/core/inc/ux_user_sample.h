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
/**   User Specific                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  PORT SPECIFIC C INFORMATION                            RELEASE        */ 
/*                                                                        */ 
/*    ux_user.h                                           PORTABLE C      */ 
/*                                                           6.2.0        */
/*                                                                        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains user defines for configuring USBX in specific    */ 
/*    ways. This file will have an effect only if the application and     */ 
/*    USBX library are built with UX_INCLUDE_USER_DEFINE_FILE defined.    */ 
/*    Note that all the defines in this file may also be made on the      */ 
/*    command line when building USBX library and application objects.    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  02-02-2021     Xiuwen Cai               Modified comment(s), added    */
/*                                            compile option for using    */
/*                                            packet pool from NetX,      */
/*                                            resulting in version 6.1.4  */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added DFU_UPLOAD option,    */
/*                                            added macro to enable       */
/*                                            device bi-dir-endpoints,    */
/*                                            added macro to disable CDC- */
/*                                            ACM transmission support,   */
/*                                            resulting in version 6.1.6  */
/*  06-02-2021     Xiuwen Cai               Modified comment(s), added    */
/*                                            transfer timeout value      */
/*                                            options,                    */
/*                                            resulting in version 6.1.7  */
/*  08-02-2021     Wen Wang                 Modified comment(s),          */
/*                                            fixed spelling error,       */
/*                                            resulting in version 6.1.8  */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added option for assert,    */
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            added option for device     */
/*                                            audio feedback endpoint,    */
/*                                            added option for MTP,       */
/*                                            added options for HID       */
/*                                            interrupt OUT support,      */
/*                                            added option to validate    */
/*                                            class code in enumeration,  */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added audio class features, */
/*                                            added device CDC_ACM and    */
/*                                            printer write auto ZLP,     */
/*                                            resulting in version 6.1.12 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            deprecated ECM pool option, */
/*                                            added align minimal config, */
/*                                            added host stack instance   */
/*                                            creation strategy control,  */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/

#ifndef UX_USER_H
#define UX_USER_H


/* Define various build options for the USBX port.  The application should either make changes
   here by commenting or un-commenting the conditional compilation defined OR supply the defines 
   though the compiler's equivalent of the -D option.  */

/* Define USBX Generic Thread Stack Size.  */
/* #define UX_THREAD_STACK_SIZE                                (2 * 1024) */

/* Define USBX Host Enum Thread Stack Size. The default is to use UX_THREAD_STACK_SIZE */
/* 
#define UX_HOST_ENUM_THREAD_STACK_SIZE                      UX_THREAD_STACK_SIZE 
*/


/* Define USBX Host HCD Thread Stack Size.  The default is to use UX_THREAD_STACK_SIZE */
/*
#define UX_HOST_HCD_THREAD_STACK_SIZE                       UX_THREAD_STACK_SIZE
*/

/* Define USBX Host HNP Polling Thread Stack Size. The default is to use UX_THREAD_STACK_SIZE */
/*
#define UX_HOST_HNP_POLLING_THREAD_STACK                    UX_THREAD_STACK_SIZE
*/

/* Override various options with default values already assigned in ux_api.h or ux_port.h. Please 
   also refer to ux_port.h for descriptions on each of these options.  */

/* Defined, this value represents minimal allocated memory alignment in number of bytes.
   The default is UX_ALIGN_16 (0x0f) to align allocated memory to 16 bytes.  */
/* #define UX_ALIGN_MIN UX_ALIGN_16  */

/* Defined, this value represents how many ticks per seconds for a specific hardware platform. 
   The default is 1000 indicating 1 tick per millisecond.  */

/* #define UX_PERIODIC_RATE 1000
*/
#define UX_PERIODIC_RATE (TX_TIMER_TICKS_PER_SECOND)

/* Define control transfer timeout value in millisecond.
   The default is 10000 milliseconds.  */
/*
#define UX_CONTROL_TRANSFER_TIMEOUT                         10000
*/

/* Define non control transfer timeout value in millisecond.
   The default is 50000 milliseconds.  */
/*
#define UX_NON_CONTROL_TRANSFER_TIMEOUT                     50000
*/


/* Defined, this value is the maximum number of classes that can be loaded by USBX. This value
   represents the class container and not the number of instances of a class. For instance, if a
   particular implementation of USBX needs the hub class, the printer class, and the storage
   class, then the UX_MAX_CLASSES value can be set to 3 regardless of the number of devices 
   that belong to these classes.  */

/* #define UX_MAX_CLASSES  3
*/


/* Defined, this value is the maximum number of classes in the device stack that can be loaded by
   USBX.  */

/* #define UX_MAX_SLAVE_CLASS_DRIVER    1
*/

/* Defined, this value is the maximum number of interfaces in the device framework.  */

/* #define UX_MAX_SLAVE_INTERFACES    16
*/

/* Defined, this value represents the number of different host controllers available in the system. 
   For USB 1.1 support, this value will usually be 1. For USB 2.0 support, this value can be more 
   than 1. This value represents the number of concurrent host controllers running at the same time. 
   If for instance there are two instances of OHCI running, or one EHCI and one OHCI controller
   running, the UX_MAX_HCD should be set to 2.  */

/* #define UX_MAX_HCD  1
*/


/* Defined, this value represents the maximum number of devices that can be attached to the USB.
   Normally, the theoretical maximum number on a single USB is 127 devices. This value can be 
   scaled down to conserve memory. Note that this value represents the total number of devices 
   regardless of the number of USB buses in the system.  */

/* #define UX_MAX_DEVICES  127
*/


/* Defined, this value represents the current number of SCSI logical units represented in the device
   storage class driver.  */

/* #define UX_MAX_SLAVE_LUN    1
*/


/* Defined, this value represents the maximum number of SCSI logical units represented in the
   host storage class driver.  */
   
/* #define UX_MAX_HOST_LUN 1
*/


/* Defined, this value represents the maximum number of bytes received on a control endpoint in
   the device stack. The default is 256 bytes but can be reduced in memory constrained environments.  */

/* #define UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH 256
*/


/* Defined, this value represents the maximum number of bytes that can be received or transmitted
   on any endpoint. This value cannot be less than the maximum packet size of any endpoint. The default 
   is 4096 bytes but can be reduced in memory constrained environments. For cd-rom support in the storage 
   class, this value cannot be less than 2048.  */

#define UX_SLAVE_REQUEST_DATA_MAX_LENGTH    (1024 * 2)


/* Defined, this value includes code to handle storage Multi-Media Commands (MMC). E.g., DVD-ROM.
*/

/* #define UX_SLAVE_CLASS_STORAGE_INCLUDE_MMC   */


/* Defined, this value represents the maximum number of bytes that a storage payload can send/receive.
   The default is 8K bytes but can be reduced in memory constrained environments.  */
#define UX_HOST_CLASS_STORAGE_MEMORY_BUFFER_SIZE            (1024 * 8)

/* Define USBX Mass Storage Thread Stack Size. The default is to use UX_THREAD_STACK_SIZE. */

/* #define UX_HOST_CLASS_STORAGE_THREAD_STACK_SIZE             UX_THREAD_STACK_SIZE 
 */

/* Defined, this value represents the maximum number of Ed, regular TDs and Isochronous TDs. These values
   depend on the type of host controller and can be reduced in memory constrained environments.  */

#define UX_MAX_ED                                           80
#define UX_MAX_TD                                           128
#define UX_MAX_ISO_TD                                       1

/* Defined, this value represents the maximum size of the HID decompressed buffer. This cannot be determined
   in advance so we allocate a big block, usually 4K but for simple HID devices like keyboard and mouse
   it can be reduced a lot. */

#define UX_HOST_CLASS_HID_DECOMPRESSION_BUFFER              4096

/* Defined, this value represents the maximum number of HID usages for a HID device. 
   Default is 2048 but for simple HID devices like keyboard and mouse it can be reduced a lot. */

#define UX_HOST_CLASS_HID_USAGES                            2048


/* By default, each key in each HID report from the device is reported by ux_host_class_hid_keyboard_key_get 
   (a HID report from the device is received whenever there is a change in a key state i.e. when a key is pressed
   or released. The report contains every key that is down). There are limitations to this method such as not being
   able to determine when a key has been released.

   Defined, this value causes ux_host_class_hid_keyboard_key_get to only report key changes i.e. key presses
   and key releases. */

/* #define UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE */

/* Works when UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE is defined.

   Defined, this value causes ux_host_class_hid_keyboard_key_get to only report key pressed/down changes;
   key released/up changes are not reported.
 */

/* #define UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE_REPORT_KEY_DOWN_ONLY */

/* Works when UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE is defined.

   Defined, this value causes ux_host_class_hid_keyboard_key_get to report lock key (CapsLock/NumLock/ScrollLock) changes.
 */

/* #define UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE_REPORT_LOCK_KEYS */

/* Works when UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE is defined.

   Defined, this value causes ux_host_class_hid_keyboard_key_get to report modifier key (Ctrl/Alt/Shift/GUI) changes.
 */

/* #define UX_HOST_CLASS_HID_KEYBOARD_EVENTS_KEY_CHANGES_MODE_REPORT_MODIFIER_KEYS */


/* Defined, this value represents the maximum number of media for the host storage class. 
   Default is 8 but for memory constrained resource systems this can ne reduced to 1. */

#define UX_HOST_CLASS_STORAGE_MAX_MEDIA                     2

/* Defined, this value includes code to handle storage devices that use the CB
   or CBI protocol (such as floppy disks). It is off by default because these 
   protocols are obsolete, being superseded by the Bulk Only Transport (BOT) protocol
   which virtually all modern storage devices use.
*/

/* #define UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT */

/* Defined, this value forces the memory allocation scheme to enforce alignment
   of memory with the UX_SAFE_ALIGN field.
*/

/* #define UX_ENFORCE_SAFE_ALIGNMENT   */

/* Defined, this value represents the number of packets in the CDC_ECM device class.
   The default is 16.
*/

#define UX_DEVICE_CLASS_CDC_ECM_NX_PKPOOL_ENTRIES           4

/* Defined, this value represents the number of packets in the CDC_ECM host class.
   The default is 16.
*/

/* #define UX_HOST_CLASS_CDC_ECM_NX_PKPOOL_ENTRIES             16 */

/* Defined, this value represents the number of milliseconds to wait for packet
   allocation until invoking the application's error callback and retrying.
   The default is 1000 milliseconds.
*/

/* #define UX_HOST_CLASS_CDC_ECM_PACKET_POOL_WAIT           10 */

/* Defined, this value represents the number of milliseconds to wait for packet
   pool availability checking loop.
   The default is 100 milliseconds.
*/

/* #define UX_HOST_CLASS_CDC_ECM_PACKET_POOL_INSTANCE_WAIT  10 */

/* Defined, this enables CDC ECM class to use the packet pool from NetX instance.
   It's deprecated, packet pool from NetX instance is always used now.
 */

/* #define UX_HOST_CLASS_CDC_ECM_USE_PACKET_POOL_FROM_NETX */

/* Defined, this value represents the number of milliseconds to wait for packet
   allocation until invoking the application's error callback and retrying.
*/

/* #define UX_DEVICE_CLASS_CDC_ECM_PACKET_POOL_WAIT         10 */

/* Defined, this value represents the the maximum length of HID reports on the
   device.
 */

/* #define UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH          64 */

/* Defined, this value represents the the maximum number of HID events/reports 
   that can be queued at once.                   
 */

/* #define UX_DEVICE_CLASS_HID_MAX_EVENTS_QUEUE             8  */


/* Defined, this macro will disable DFU_UPLOAD support.  */

/* #define UX_DEVICE_CLASS_DFU_UPLOAD_DISABLE  */

/* Defined, this macro will enable DFU_GETSTATUS and DFU_GETSTATE in dfuERROR.  */

/* #define UX_DEVICE_CLASS_DFU_ERROR_GET_ENABLE  */

/* Defined, this macro will change status mode.
   0 - simple mode,
       status is queried from application in dfuDNLOAD-SYNC and dfuMANIFEST-SYNC state,
       no bwPollTimeout.
   1 - status is queried from application once requested,
       b0-3 : media status
       b4-7 : bStatus
       b8-31: bwPollTimeout
       bwPollTimeout supported.
*/

/* #define UX_DEVICE_CLASS_DFU_STATUS_MODE                  (1)  */

/* Defined, this value represents the default DFU status bwPollTimeout.
   The value is 3 bytes long (max 0xFFFFFFu).
   By default the bwPollTimeout is 1 (means 1ms).
 */

/* #define UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT           (1)  */

/* Defined, this macro will enable custom request process callback.  */

/* #define UX_DEVICE_CLASS_DFU_CUSTOM_REQUEST_ENABLE   */

/* Defined, this macro disables CDC ACM non-blocking transmission support.  */

/* #define UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE  */

/* Defined, device HID interrupt OUT transfer is supported.  */

/* #define UX_DEVICE_CLASS_HID_INTERRUPT_OUT_SUPPORT  */

/* defined, this macro enables device audio feedback endpoint support.  */

/* #define UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT  */

/* Defined, class _write is pending ZLP automatically (complete transfer) after buffer is sent.  */

/* #define UX_DEVICE_CLASS_CDC_ACM_WRITE_AUTO_ZLP  */
/* #define UX_DEVICE_CLASS_PRINTER_WRITE_AUTO_ZLP  */

/* defined, this macro enables device audio interrupt endpoint support.  */

/* define UX_DEVICE_CLASS_AUDIO_INTERRUPT_SUPPORT  */

/* Defined, this macro enables device bi-directional-endpoint support.  */

/* #define UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT  */

/* Defined, this macro enables device/host PIMA MTP support.  */

/* #define UX_PIMA_WITH_MTP_SUPPORT  */

/* Defined, this macro enables host device class code validation.
   Only following USB-IF allowed device class code is allowed:
   0x00, 0x02 (CDC Control), 0x09 (Hub), 0x11 (Billboard), 0xDC (Diagnostic), 0xEF (MISC), 0xFF (Vendor)
   Refer to https://www.usb.org/defined-class-codes for more details.
 */

/* #define UX_HOST_DEVICE_CLASS_CODE_VALIDATION_ENABLE  */

/* Defined, host HID interrupt OUT transfer is supported.  */

/* #define UX_HOST_CLASS_HID_INTERRUPT_OUT_SUPPORT  */

/* Define HID report transfer timeout value in millisecond.
   The default is 10000 milliseconds.  */

/* #define UX_HOST_CLASS_HID_REPORT_TRANSFER_TIMEOUT               10000 */

/* Defined, host audio UAC 2.0 is supported.  */
/* #define UX_HOST_CLASS_AUDIO_2_SUPPORT  */

/* Defined, host audio optional feedback endpoint is supported.  */
/* #define UX_HOST_CLASS_AUDIO_FEEDBACK_SUPPORT  */

/* Defined, host audio optional interrupt endpoint is support.  */
/* #define UX_HOST_CLASS_AUDIO_INTERRUPT_SUPPORT  */

/* Defined, this value controls host configuration instance creation, include all
   interfaces and endpoints physical resources.
   Possible settings:
    UX_HOST_STACK_CONFIGURATION_INSTANCE_CREATE_ALL (0) - The default, create all inside configuration.
    UX_HOST_STACK_CONFIGURATION_INSTANCE_CREATE_OWNED (1) - Create things owned by class driver.
   Not defined, default setting is applied.
 */
/* #define UX_HOST_STACK_CONFIGURATION_INSTANCE_CREATE_CONTROL UX_HOST_STACK_CONFIGURATION_INSTANCE_CREATE_OWNED */

/* Defined, this value will only enable the host side of usbx.  */
/* #define UX_HOST_SIDE_ONLY   */

/* Defined, this value will only enable the device side of usbx.  */
/* #define UX_DEVICE_SIDE_ONLY   */

/* Defined, this value will include the OTG polling thread. OTG can only be active if both host/device are present.
*/

#ifndef UX_HOST_SIDE_ONLY 
#ifndef UX_DEVICE_SIDE_ONLY 

/* #define UX_OTG_SUPPORT */

#endif 
#endif 

/* Defined, this macro will enable the standalone mode of usbx.  */
/* #define UX_STANDALONE  */

/* Defined, this macro will remove the FileX dependency of host storage.
   In this mode, sector access is offered instead of directly FileX FX_MEDIA support.
   Use following APIs for media obtain and access:
   - ux_host_class_storage_media_get : get instance of UX_HOST_CLASS_STORAGE_MEDIA
   - ux_host_class_storage_media_lock : lock specific media for further read/write
   - ux_host_class_storage_media_read : read sectors on locked media
   - ux_host_class_storage_media_write : write sectors on locked media
   - ux_host_class_storage_media_unlock : unlock media
   Note it's forced defined/enabled in standalone mode of usbx.
*/
/* #define UX_HOST_CLASS_STORAGE_NO_FILEX  */

/* Defined, this value represents the maximum size of single transfers for the SCSI data phase.
   By default it's 1024.
*/

#define UX_HOST_CLASS_STORAGE_MAX_TRANSFER_SIZE             (1024 * 1)

/* Defined, this value represents the size of the log pool.
*/
#define UX_DEBUG_LOG_SIZE                                   (1024 * 16)

/* Defined, this macro represents the non-blocking function to return time tick.
   This macro is used only in standalone mode.
   The tick rate is defined by UX_PERIODIC_RATE.
   If it's not defined, or TX is not included, a external function must be
   implement in application:
      extern  ULONG       _ux_utility_time_get(VOID);
*/
/* #define _ux_utility_time_get() tx_time_get()  */

/* Defined, this macro represents the non-blocking function to disable interrupts
   and return old interrupt setting flags.
   If it's not defined, or TX is not included, a external function must be
   implement in application:
      extern ALIGN_TYPE   _ux_utility_interrupt_disable(VOID);
*/
/* #define _ux_utility_interrupt_disable() _tx_thread_interrupt_disable()  */

/* Defined, this macro represents the non-blocking function to restore interrupts.
   If it's not defined, or TX is not included, a external function must be
   implement in application:
      extern VOID         _ux_utility_interrupt_restore(ALIGN_TYPE);
*/
/* #define _ux_utility_interrupt_restore(flags) _tx_thread_interrupt_restore(flags)  */

/* Defined, this enables the assert checks inside usbx.  */
#define UX_ENABLE_ASSERT

/* Defined, this defines the assert action taken when failure detected. By default
   it halts without any output.  */
/* #define UX_ASSERT_FAIL  for (;;) {tx_thread_sleep(UX_WAIT_FOREVER); }  */


/* DEBUG includes and macros for a specific platform go here.  */
#ifdef UX_INCLUDE_USER_DEFINE_BSP
#include "usb_bsp.h"
#include "usbh_hcs.h"
#include "usbh_stdreq.h"
#include "usbh_core.h"
#endif 

#endif 

