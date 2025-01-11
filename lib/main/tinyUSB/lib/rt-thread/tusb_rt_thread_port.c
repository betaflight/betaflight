/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */
#ifdef __RTTHREAD__
#include <rtthread.h>

#define  DBG_TAG  "TinyUSB"
#define  DBG_LVL  DBG_INFO
#include <rtdbg.h>
#include <tusb.h>

#ifndef RT_USING_HEAP
/* if there is not enable heap, we should use static thread and stack. */
static rt_uint8_t tusb_stack[PKG_TINYUSB_STACK_SIZE];
static struct rt_thread tusb_thread;
#endif /* RT_USING_HEAP */

extern int tusb_board_init(void);

static void tusb_thread_entry(void *parameter)
{
    (void) parameter;
    while (1)
    {
#if CFG_TUH_ENABLED
        tuh_task();
#endif
#if CFG_TUD_ENABLED
        tud_task();
#endif
    }
}

static int init_tinyusb(void)
{
    rt_thread_t tid;

    tusb_board_init();
    tusb_init();

#ifdef RT_USING_HEAP
    tid = rt_thread_create("tusb", tusb_thread_entry, RT_NULL,
                           PKG_TINYUSB_STACK_SIZE,
                           PKG_TINYUSB_THREAD_PRIORITY, 10);
    if (tid == RT_NULL)
#else
    rt_err_t result;

    tid = &tusb_thread;
    result = rt_thread_init(tid, "tusb", tusb_thread_entry, RT_NULL,
                            tusb_stack, sizeof(tusb_stack), 4, 10);
    if (result != RT_EOK)
#endif /* RT_USING_HEAP */
    {
        LOG_E("Fail to create TinyUSB thread");
        return -1;
    }

    rt_thread_startup(tid);

    return 0;
}
INIT_APP_EXPORT(init_tinyusb);
#endif /*__RTTHREAD__*/
