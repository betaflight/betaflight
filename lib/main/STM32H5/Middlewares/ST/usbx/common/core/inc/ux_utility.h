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
/**   Utility                                                             */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_utility.h                                        PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX components that utilize utility functions.                     */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added timer delete, used UX */
/*                                            prefix to refer to TX       */
/*                                            symbols instead of using    */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added macros for RTOS calls,*/
/*                                            fixed OHCI PRSC issue,      */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/

#ifndef UX_UTILITY_H
#define UX_UTILITY_H


/* Define Utility component function prototypes.  */

VOID             _ux_utility_descriptor_parse(UCHAR * raw_descriptor, UCHAR * descriptor_structure,
                             UINT descriptor_entries, UCHAR * descriptor);
VOID             _ux_utility_descriptor_pack(UCHAR * descriptor, UCHAR * descriptor_structure,
                             UINT descriptor_entries, UCHAR * raw_descriptor);
ULONG            _ux_utility_long_get(UCHAR * address);
VOID             _ux_utility_long_put(UCHAR * address, ULONG value);
VOID             _ux_utility_long_put_big_endian(UCHAR * address, ULONG value);
ULONG            _ux_utility_long_get_big_endian(UCHAR * address);
VOID            *_ux_utility_memory_allocate(ULONG memory_alignment,ULONG memory_cache_flag, ULONG memory_size_requested);
UINT             _ux_utility_memory_compare(VOID *memory_source, VOID *memory_destination, ULONG length);
VOID             _ux_utility_memory_copy(VOID *memory_destination, VOID *memory_source, ULONG length);
VOID             _ux_utility_memory_free(VOID *memory);
ULONG            _ux_utility_string_length_get(UCHAR *string);
UINT             _ux_utility_string_length_check(UCHAR *input_string, UINT *string_length_ptr, UINT max_string_length);
UX_MEMORY_BLOCK *_ux_utility_memory_free_block_best_get(ULONG memory_cache_flag, ULONG memory_size_requested);
VOID             _ux_utility_memory_set(VOID *destination, UCHAR value, ULONG length);
ULONG            _ux_utility_pci_class_scan(ULONG pci_class, ULONG bus_number, ULONG device_number, 
                            ULONG function_number, ULONG *current_bus_number,
                            ULONG *current_device_number, ULONG *current_function_number);
ULONG            _ux_utility_pci_read(ULONG bus_number, ULONG device_number, ULONG function_number,
                             ULONG offset, UINT read_size);
VOID             _ux_utility_pci_write(ULONG bus_number, ULONG device_number, ULONG function_number,
                             ULONG offset, ULONG value, UINT write_size);
VOID            *_ux_utility_physical_address(VOID *virtual_address);
VOID             _ux_utility_set_interrupt_handler(UINT irq, VOID (*interrupt_handler)(VOID));
ULONG            _ux_utility_short_get(UCHAR * address);
ULONG            _ux_utility_short_get_big_endian(UCHAR * address);
VOID             _ux_utility_short_put(UCHAR * address, USHORT value);
VOID             _ux_utility_short_put_big_endian(UCHAR * address, USHORT value);
VOID            *_ux_utility_virtual_address(VOID *physical_address);
VOID             _ux_utility_unicode_to_string(UCHAR *source, UCHAR *destination);
VOID             _ux_utility_string_to_unicode(UCHAR *source, UCHAR *destination);
VOID             _ux_utility_debug_callback_register(VOID (*debug_callback)(UCHAR *, ULONG));
VOID             _ux_utility_delay_ms(ULONG ms_wait);

#if !defined(UX_STANDALONE)
UINT             _ux_utility_mutex_create(UX_MUTEX *mutex, CHAR *mutex_name);
UINT             _ux_utility_mutex_delete(UX_MUTEX *mutex);
VOID             _ux_utility_mutex_off(UX_MUTEX *mutex);
VOID             _ux_utility_mutex_on(UX_MUTEX *mutex);
UINT             _ux_utility_semaphore_create(UX_SEMAPHORE *semaphore, CHAR *semaphore_name, UINT initial_count);
UINT             _ux_utility_semaphore_delete(UX_SEMAPHORE *semaphore);
UINT             _ux_utility_semaphore_get(UX_SEMAPHORE *semaphore, ULONG semaphore_signal);
UINT             _ux_utility_semaphore_put(UX_SEMAPHORE *semaphore);
UINT             _ux_utility_thread_create(UX_THREAD *thread_ptr, CHAR *name, 
                             VOID (*entry_function)(ULONG), ULONG entry_input,
                             VOID *stack_start, ULONG stack_size, 
                             UINT priority, UINT preempt_threshold,
                             ULONG time_slice, UINT auto_start);
UINT             _ux_utility_thread_delete(UX_THREAD *thread_ptr);
VOID             _ux_utility_thread_relinquish(VOID);
UINT             _ux_utility_thread_schedule_other(UINT caller_priority);
UINT             _ux_utility_thread_resume(UX_THREAD *thread_ptr);
UINT             _ux_utility_thread_sleep(ULONG ticks);
UINT             _ux_utility_thread_suspend(UX_THREAD *thread_ptr);
UX_THREAD       *_ux_utility_thread_identify(VOID);
UINT             _ux_utility_timer_create(UX_TIMER *timer, CHAR *timer_name, VOID (*expiration_function) (ULONG),
                             ULONG expiration_input, ULONG initial_ticks, ULONG reschedule_ticks, 
                             UINT activation_flag);
UINT             _ux_utility_timer_delete(UX_TIMER *timer);
UINT             _ux_utility_event_flags_create(UX_EVENT_FLAGS_GROUP*group_ptr, CHAR *name);
UINT             _ux_utility_event_flags_delete(UX_EVENT_FLAGS_GROUP*group_ptr);
UINT             _ux_utility_event_flags_get(UX_EVENT_FLAGS_GROUP*group_ptr, ULONG requested_flags, 
                                                 UINT get_option, ULONG *actual_flags_ptr, ULONG wait_option);
UINT             _ux_utility_event_flags_set(UX_EVENT_FLAGS_GROUP*group_ptr, ULONG flags_to_set,
                                                 UINT set_option);
#endif

#ifndef             _ux_utility_interrupt_disable
#ifdef TX_API_H
#define             _ux_utility_interrupt_disable()             _tx_thread_interrupt_disable()
#else
extern ALIGN_TYPE   _ux_utility_interrupt_disable(VOID);
#endif
#else
extern ALIGN_TYPE   _ux_utility_interrupt_disable(VOID);
#endif

#ifndef             _ux_utility_interrupt_restore
#ifdef TX_API_H
#define             _ux_utility_interrupt_restore(flags)        _tx_thread_interrupt_restore(flags)
#else
extern VOID         _ux_utility_interrupt_restore(ALIGN_TYPE);
#endif
#else
extern VOID         _ux_utility_interrupt_restore(ALIGN_TYPE);
#endif

#ifndef             _ux_utility_time_get
#ifdef TX_API_H
#define             _ux_utility_time_get()                      tx_time_get()
#else
extern  ULONG       _ux_utility_time_get(VOID);
#endif
#else
extern  ULONG       _ux_utility_time_get(VOID);
#endif

#ifndef             _ux_utility_time_elapsed
#define             _ux_utility_time_elapsed(a,b)          (((b)>=(a)) ? ((b)-(a)) : (0xFFFFFFFFul-(b)+(a)+1))
#else
extern  ALIGN_TYPE  _ux_utility_time_elapsed(ALIGN_TYPE, ALIGN_TYPE);
#endif

#if !defined(UX_STANDALONE)
#define _ux_system_semaphore_create                             _ux_utility_semaphore_create
#define _ux_system_semaphore_create_norc                        _ux_utility_semaphore_create
#define _ux_system_semaphore_created(sem)                       ((sem)->tx_semaphore_id != UX_EMPTY)
#define _ux_system_semaphore_get                                _ux_utility_semaphore_get
#define _ux_system_semaphore_get_norc                           _ux_utility_semaphore_get
#define _ux_system_semaphore_waiting(sem)                       ((sem)->tx_semaphore_count != 0)
#define _ux_system_semaphore_delete                             _ux_utility_semaphore_delete
#define _ux_system_semaphore_put                                _ux_utility_semaphore_put
#define _ux_system_thread_create                                _ux_utility_thread_create
#define _ux_system_thread_create_norc                           _ux_utility_thread_create
#define _ux_system_thread_created(t)                            ((t)->tx_thread_id != UX_EMPTY)
#define _ux_system_thread_delete                                _ux_utility_thread_delete
#define _ux_system_mutex_create                                 _ux_utility_mutex_create
#define _ux_system_mutex_delete                                 _ux_utility_mutex_delete
#define _ux_system_mutex_off                                    _ux_utility_mutex_off
#define _ux_system_mutex_on                                     _ux_utility_mutex_on
#define _ux_system_event_flags_create                           _ux_utility_event_flags_create
#define _ux_system_event_flags_created(e)                       ((e)->tx_event_flags_group_id != UX_EMPTY)
#define _ux_system_event_flags_delete                           _ux_utility_event_flags_delete
#define _ux_system_event_flags_get                              _ux_utility_event_flags_get
#define _ux_system_event_flags_set                              _ux_utility_event_flags_set
#define _ux_system_event_flags_set_rc                           _ux_utility_event_flags_set
#else
#define _ux_system_semaphore_create(sem,name,cnt)               (UX_SUCCESS)
#define _ux_system_semaphore_create_norc(sem,name,cnt)          do{}while(0)
#define _ux_system_semaphore_created(sem)                       (UX_FALSE)
#define _ux_system_semaphore_get(sem,opt)                       (UX_SUCCESS)
#define _ux_system_semaphore_get_norc(sem,opt)                  do{}while(0)
#define _ux_system_semaphore_waiting(sem)                       (UX_FALSE)
#define _ux_system_semaphore_delete(sem)                        do{}while(0)
#define _ux_system_semaphore_put(sem)                           do{}while(0)
#define _ux_system_thread_create(t,name,entry,entry_param,stack,stack_size,priority,preempt_threshold,time_slice,auto_start) (UX_SUCCESS)
#define _ux_system_thread_create_norc(t,name,entry,entry_param,stack,stack_size,priority,preempt_threshold,time_slice,auto_start) do{}while(0)
#define _ux_system_thread_created(t)                            (UX_FALSE)
#define _ux_system_thread_delete(t)                             do{}while(0)
#define _ux_system_mutex_create(mutex,name)                     do{}while(0)
#define _ux_system_mutex_delete(mutex)                          do{}while(0)
#define _ux_system_mutex_off(mutex)                             do{}while(0)
#define _ux_system_mutex_on(mutex)                              do{}while(0)
#define _ux_system_event_flags_create(g,name)                   (UX_SUCCESS)
#define _ux_system_event_flags_created(e)                       (UX_FALSE)
#define _ux_system_event_flags_delete(g)                        do{}while(0)
#define _ux_system_event_flags_get(g,req,gopt,actual,wopt)      (*actual = 0)
#define _ux_system_event_flags_set(g,flags,option)              do{(void)flags;}while(0)
#define _ux_system_event_flags_set_rc(g,flags,option)           (UX_SUCCESS)
#endif

#if !defined(UX_DEVICE_STANDALONE)
#define _ux_device_thread_create                                _ux_utility_thread_create
#define _ux_device_thread_delete                                _ux_utility_thread_delete
#define _ux_device_thread_entry(t)                              ((t)->tx_thread_entry)
#define _ux_device_thread_suspend                               _ux_utility_thread_suspend
#define _ux_device_thread_resume                                _ux_utility_thread_resume
#define _ux_device_thread_relinquish                            _ux_utility_thread_relinquish
#define _ux_device_semaphore_create                             _ux_utility_semaphore_create
#define _ux_device_semaphore_created(sem)                       ((sem)->tx_semaphore_id != 0)
#define _ux_device_semaphore_waiting(sem)                       ((sem)->tx_semaphore_count != 0)
#define _ux_device_semaphore_delete                             _ux_utility_semaphore_delete
#define _ux_device_semaphore_get                                _ux_utility_semaphore_get
#define _ux_device_semaphore_put                                _ux_utility_semaphore_put
#define _ux_device_mutex_create                                 _ux_utility_mutex_create
#define _ux_device_mutex_delete                                 _ux_utility_mutex_delete
#define _ux_device_mutex_off                                    _ux_utility_mutex_off
#define _ux_device_mutex_on                                     _ux_utility_mutex_on
#define _ux_device_event_flags_create                           _ux_utility_event_flags_create
#define _ux_device_event_flags_delete                           _ux_utility_event_flags_delete
#define _ux_device_event_flags_get                              _ux_utility_event_flags_get
#define _ux_device_event_flags_set                              _ux_utility_event_flags_set
#else
#define _ux_device_thread_create(t,name,entry,entry_param,stack,stack_size,priority,preempt_threshold,time_slice,auto_start) (UX_SUCCESS)
#define _ux_device_thread_delete(t)                             do{}while(0)
#define _ux_device_thread_entry(t)                              ((UX_THREAD_ENTRY)t)
#define _ux_device_thread_suspend(t)                            do{}while(0)
#define _ux_device_thread_resume(t)                             do{}while(0)
#define _ux_device_thread_relinquish(t)                         do{}while(0)
#define _ux_device_semaphore_create(sem,name,cnt)               (UX_SUCCESS)
#define _ux_device_semaphore_created(sem)                       (UX_FALSE)
#define _ux_device_semaphore_waiting(sem)                       (UX_FALSE)
#define _ux_device_semaphore_delete(sem)                        do{}while(0)
#define _ux_device_semaphore_get(sem,t)                         (UX_SUCCESS)
#define _ux_device_semaphore_put(sem)                           do{}while(0)
#define _ux_device_mutex_create(mutex,name)                     do{}while(0)
#define _ux_device_mutex_delete(mutex)                          do{}while(0)
#define _ux_device_mutex_off(mutex)                             do{}while(0)
#define _ux_device_mutex_on(mutex)                              do{}while(0)
#define _ux_device_event_flags_create(g,name)                   do{}while(0)
#define _ux_device_event_flags_delete(g)                        do{}while(0)
#define _ux_device_event_flags_get(g,req,gopt,actual,wopt)      do{}while(0)
#define _ux_device_event_flags_set(g,flags,option)              do{}while(0)
#endif


#if !defined(UX_HOST_STANDALONE)
#define _ux_host_thread_create                                  _ux_utility_thread_create
#define _ux_host_thread_created(thr)                            ((thr)->tx_thread_id != 0)
#define _ux_host_thread_delete                                  _ux_utility_thread_delete
#define _ux_host_thread_entry(thr)                              ((thr)->tx_thread_entry)
#define _ux_host_thread_resume                                  _ux_utility_thread_resume
#define _ux_host_thread_sleep                                   _ux_utility_thread_sleep
#define _ux_host_thread_schedule_other                          _ux_utility_thread_schedule_other
#define _ux_host_semaphore_create                               _ux_utility_semaphore_create
#define _ux_host_semaphore_created(sem)                         ((sem)->tx_semaphore_id != 0)
#define _ux_host_semaphore_waiting(sem)                         ((sem)->tx_semaphore_count != 0)
#define _ux_host_semaphore_delete                               _ux_utility_semaphore_delete
#define _ux_host_semaphore_get                                  _ux_utility_semaphore_get
#define _ux_host_semaphore_get_norc                             _ux_utility_semaphore_get
#define _ux_host_semaphore_put                                  _ux_utility_semaphore_put
#define _ux_host_semaphore_put_rc                               _ux_utility_semaphore_put
#define _ux_host_mutex_create                                   _ux_utility_mutex_create
#define _ux_host_mutex_delete                                   _ux_utility_mutex_delete
#define _ux_host_mutex_off                                      _ux_utility_mutex_off
#define _ux_host_mutex_on                                       _ux_utility_mutex_on
#define _ux_host_event_flags_create                             _ux_utility_event_flags_create
#define _ux_host_event_flags_delete                             _ux_utility_event_flags_delete
#define _ux_host_event_flags_get                                _ux_utility_event_flags_get
#define _ux_host_event_flags_set                                _ux_utility_event_flags_set
#define _ux_host_timer_create                                   _ux_utility_timer_create
#define _ux_host_timer_delete                                   _ux_utility_timer_delete
#else
#define _ux_host_thread_create(t,name,entry,entry_param,stack,stack_size,priority,preempt_threshold,time_slice,auto_start) (UX_SUCCESS)
#define _ux_host_thread_created(t)                              (UX_FALSE)
#define _ux_host_thread_delete(t)                               do{}while(0)
#define _ux_host_thread_entry(thr)                              (UX_NULL)
#define _ux_host_thread_resume(t)                               do{}while(0)
#define _ux_host_thread_sleep(t)                                do{}while(0)
#define _ux_host_thread_schedule_other(t)                       do{}while(0)
#define _ux_host_semaphore_create(sem,name,cnt)                 (UX_SUCCESS)
#define _ux_host_semaphore_created(sem)                         (UX_FALSE)
#define _ux_host_semaphore_waiting(sem)                         (UX_FALSE)
#define _ux_host_semaphore_delete(sem)                          do{}while(0)
#define _ux_host_semaphore_get(sem,t)                           (UX_SUCCESS)
#define _ux_host_semaphore_get_norc(sem,t)                      do{}while(0)
#define _ux_host_semaphore_put(sem)                             do{}while(0)
#define _ux_host_semaphore_put_rc(sem)                          (UX_SUCCESS)
#define _ux_host_mutex_create(mutex,name)                       (UX_SUCCESS)
#define _ux_host_mutex_delete(mutex)                            do{}while(0)
#define _ux_host_mutex_off(mutex)                               do{}while(0)
#define _ux_host_mutex_on(mutex)                                do{}while(0)
#define _ux_host_event_flags_create(g,name)                     (UX_SUCCESS)
#define _ux_host_event_flags_delete(g)                          (UX_SUCCESS)
#define _ux_host_event_flags_get(g,req,gopt,actual,wopt)        (UX_SUCCESS)
#define _ux_host_event_flags_set(g,flags,option)                do{}while(0)
#define _ux_host_timer_create(t,name,func,arg,tick0,tick1,flag) (UX_SUCCESS)
#define _ux_host_timer_delete(t)                                do{}while(0)
#endif


#ifdef UX_DISABLE_ERROR_HANDLER
#define          _ux_system_error_handler(system_level, system_context, error_code) do {} while(0)
#define          _ux_utility_error_callback_register(error_callback)                do {} while(0)
#else
VOID             _ux_system_error_handler(UINT system_level, UINT system_context, UINT error_code);
VOID             _ux_utility_error_callback_register(VOID (*error_callback)(UINT system_level, UINT system_context, UINT error_code));
#endif

#define          UX_UTILITY_ADD_SAFE(add_a, add_b, result, status) do {     \
        if (UX_OVERFLOW_CHECK_ADD_ULONG(add_a, add_b))                      \
            status = UX_ERROR;                                              \
        else                                                                \
            result = (add_a) + (add_b);                                     \
    } while(0)

#define          UX_UTILITY_MULC_SAFE(mul_v, mul_c, result, status) do {    \
        if (UX_OVERFLOW_CHECK_MULC_ULONG(mul_v, mul_c))                     \
            status = UX_ERROR;                                              \
        else                                                                \
            result = (mul_v) * (mul_c);                                     \
    } while(0)

#define          UX_UTILITY_MULV_SAFE(mul_v0, mul_v1, result, status) do {  \
        if (UX_OVERFLOW_CHECK_MULC_ULONG(mul_v0, mul_v1))                   \
            status = UX_ERROR;                                              \
        else                                                                \
            result = (mul_v0) * (mul_v1);                                   \
    } while(0)

#define          UX_UTILITY_MEMORY_ALLOCATE_MULC_SAFE(align,cache,size_mul_v,size_mul_c)       \
    (UX_OVERFLOW_CHECK_MULC_ULONG(size_mul_v, size_mul_c) ? UX_NULL : _ux_utility_memory_allocate((align), (cache), (size_mul_v)*(size_mul_c)))
#define          UX_UTILITY_MEMORY_ALLOCATE_MULV_SAFE(align,cache,size_mul_v0,size_mul_v1)     \
    (UX_OVERFLOW_CHECK_MULV_ULONG(size_mul_v0, size_mul_v1) ? UX_NULL : _ux_utility_memory_allocate((align), (cache), (size_mul_v0)*(size_mul_v1)))
#define          UX_UTILITY_MEMORY_ALLOCATE_ADD_SAFE(align,cache,size_add_a,size_add_b)        \
    (UX_OVERFLOW_CHECK_ADD_ULONG(size_add_a, size_add_b) ? UX_NULL : _ux_utility_memory_allocate((align), (cache), (size_add_a)+(size_add_b)))

#ifdef UX_DISABLE_ARITHMETIC_CHECK

/* No arithmetic check, calculate directly.  */

#define          _ux_utility_memory_allocate_mulc_safe(align,cache,size_mul_v,size_mul_c)       _ux_utility_memory_allocate((align), (cache), (size_mul_v)*(size_mul_c))
#define          _ux_utility_memory_allocate_mulv_safe(align,cache,size_mul_v0,size_mul_v1)     _ux_utility_memory_allocate((align), (cache), (size_mul_v0)*(size_mul_v1))
#define          _ux_utility_memory_allocate_add_safe(align,cache,size_add_a,size_add_b)        _ux_utility_memory_allocate((align), (cache), (size_add_a)+(size_add_b))

#else /* UX_DISABLE_ARITHMETIC_CHECK */

#ifdef UX_ENABLE_MEMORY_ARITHMETIC_OPTIMIZE

/* Uses macro to enable code optimization on compiling.  */

#define          _ux_utility_memory_allocate_mulc_safe(align,cache,size_mul_v,size_mul_c)       UX_UTILITY_MEMORY_ALLOCATE_MULC_SAFE(align,cache,size_mul_v,size_mul_c)
#define          _ux_utility_memory_allocate_mulv_safe(align,cache,size_mul_v0,size_mul_v1)     UX_UTILITY_MEMORY_ALLOCATE_MULV_SAFE(align,cache,size_mul_v0,size_mul_v1)
#define          _ux_utility_memory_allocate_add_safe(align,cache,size_add_a,size_add_b)        UX_UTILITY_MEMORY_ALLOCATE_ADD_SAFE(align,cache,size_add_a,size_add_b)

#else /* UX_ENABLE_MEMORY_ARITHMETIC_OPTIMIZE */

/* Uses functions to be most flexible.  */

VOID*            _ux_utility_memory_allocate_mulc_safe(ULONG align,ULONG cache,ULONG size_mul_v,ULONG size_mul_c);
VOID*            _ux_utility_memory_allocate_mulv_safe(ULONG align,ULONG cache,ULONG size_mul_v0,ULONG size_mul_v1);
VOID*            _ux_utility_memory_allocate_add_safe(ULONG align,ULONG cache,ULONG size_add_a,ULONG size_add_b);

#endif /* UX_ENABLE_MEMORY_ARITHMETIC_OPTIMIZE */

#endif /* UX_DISABLE_ARITHMETIC_CHECK */


#if defined(UX_NAME_REFERENCED_BY_POINTER)
#define ux_utility_name_match(n0,n1,l) ((n0) == (n1))
#else
#define ux_utility_name_match(n0,n1,l) (_ux_utility_memory_compare(n0,n1,l) == UX_SUCCESS)
#endif


/* Define the system API mappings.
   Note: this section is only applicable to 
   application source code, hence the conditional that turns off this
   stuff when the include file is processed by the ThreadX source. */

#ifndef  UX_SOURCE_CODE


#define ux_utility_descriptor_parse                    _ux_utility_descriptor_parse
#define ux_utility_descriptor_pack                     _ux_utility_descriptor_pack
#define ux_utility_long_get                            _ux_utility_long_get
#define ux_utility_long_put                            _ux_utility_long_put
#define ux_utility_long_put_big_endian                 _ux_utility_long_put_big_endian
#define ux_utility_long_get_big_endian                 _ux_utility_long_get_big_endian
#define ux_utility_memory_allocate                     _ux_utility_memory_allocate
#define ux_utility_memory_compare                      _ux_utility_memory_compare
#define ux_utility_memory_copy                         _ux_utility_memory_copy
#define ux_utility_memory_free                         _ux_utility_memory_free
#define ux_utility_string_length_get                   _ux_utility_string_length_get
#define ux_utility_string_length_check                 _ux_utility_string_length_check
#define ux_utility_memory_set                          _ux_utility_memory_set
#define ux_utility_mutex_create                        _ux_utility_mutex_create
#define ux_utility_mutex_delete                        _ux_utility_mutex_delete
#define ux_utility_mutex_off                           _ux_utility_mutex_off
#define ux_utility_mutex_on                            _ux_utility_mutex_on
#define ux_utility_pci_class_scan                      _ux_utility_pci_class_scan
#define ux_utility_pci_read                            _ux_utility_pci_read
#define ux_utility_pci_write                           _ux_utility_pci_write
#define ux_utility_physical_address                    _ux_utility_physical_address
#define ux_utility_semaphore_create                    _ux_utility_semaphore_create
#define ux_utility_semaphore_delete                    _ux_utility_semaphore_delete
#define ux_utility_semaphore_get                       _ux_utility_semaphore_get
#define ux_utility_semaphore_put                       _ux_utility_semaphore_put
#define ux_utility_set_interrupt_handler               _ux_utility_set_interrupt_handler
#define ux_utility_short_get                           _ux_utility_short_get
#define ux_utility_short_get_big_endian                _ux_utility_short_get_big_endian
#define ux_utility_short_put                           _ux_utility_short_put
#define ux_utility_short_put_big_endian                _ux_utility_short_put_big_endian
#define ux_utility_thread_create                       _ux_utility_thread_create
#define ux_utility_thread_delete                       _ux_utility_thread_delete
#define ux_utility_thread_relinquish                   _ux_utility_thread_relinquish
#define ux_utility_thread_resume                       _ux_utility_thread_resume
#define ux_utility_thread_sleep                        _ux_utility_thread_sleep
#define ux_utility_thread_suspend                      _ux_utility_thread_suspend
#define ux_utility_thread_identify                     _ux_utility_thread_identify
#define ux_utility_timer_create                        _ux_utility_timer_create
#define ux_utility_event_flags_create                  _ux_utility_event_flags_create
#define ux_utility_event_flags_delete                  _ux_utility_event_flags_delete
#define ux_utility_event_flags_get                     _ux_utility_event_flags_get
#define ux_utility_event_flags_set                     _ux_utility_event_flags_set
#define ux_utility_unicode_to_string                   _ux_utility_unicode_to_string
#define ux_utility_string_to_unicode                   _ux_utility_string_to_unicode
#define ux_utility_delay_ms                            _ux_utility_delay_ms 
#define ux_utility_error_callback_register             _ux_utility_error_callback_register 
#define ux_system_error_handler                        _ux_system_error_handler

#define ux_utility_time_get                            _ux_utility_time_get
#define ux_utility_time_elapsed                        _ux_utility_time_elapsed
#endif

#endif
