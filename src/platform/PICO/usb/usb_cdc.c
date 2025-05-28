/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// TODO replace with stdio_usb from pico-sdk, with a few wrappers

#include "platform.h"

#include "tusb_config.h"
#include "tusb.h"
#include "usb_cdc.h"

#include "pico/binary_info.h"
#include "pico/time.h"
#include "pico/mutex.h"
#include "pico/critical_section.h"
#include "hardware/irq.h"

#ifndef CDC_USB_TASK_INTERVAL_US
#define CDC_USB_TASK_INTERVAL_US 1000
#endif

#ifndef CDC_USB_WRITE_TIMEOUT_US
#define CDC_USB_WRITE_TIMEOUT_US 1000
#endif

#ifndef CDC_DEADLOCK_TIMEOUT_MS
#define CDC_DEADLOCK_TIMEOUT_MS 1000
#endif

#ifndef CDC_USB_BAUD_RATE
#define CDC_USD_BAUD_RATE 115200
#endif

static bool configured = false;
static mutex_t cdc_usb_mutex;

// if this crit_sec is initialized, we are not in periodic timer mode, and must make sure
// we don't either create multiple one shot timers, or miss creating one. this crit_sec
// is used to protect the one_shot_timer_pending flag
static critical_section_t one_shot_timer_crit_sec;
static volatile bool one_shot_timer_pending;
static uint8_t low_priority_irq_num;

static int64_t timer_task(alarm_id_t id, void *user_data)
{
    UNUSED(id);
    UNUSED(user_data);

    int64_t repeat_time;
    if (critical_section_is_initialized(&one_shot_timer_crit_sec)) {
        critical_section_enter_blocking(&one_shot_timer_crit_sec);
        one_shot_timer_pending = false;
        critical_section_exit(&one_shot_timer_crit_sec);
        repeat_time = 0; // don't repeat
    } else {
        repeat_time = CDC_USB_TASK_INTERVAL_US;
    }
    if (irq_is_enabled(low_priority_irq_num)) {
        irq_set_pending(low_priority_irq_num);
        return repeat_time;
    } else {
        return 0; // don't repeat
    }
}

static void low_priority_worker_irq(void)
{
    if (mutex_try_enter(&cdc_usb_mutex, NULL)) {
        tud_task();
        mutex_exit(&cdc_usb_mutex);
    } else {
        // if the mutex is already owned, then we are in non IRQ code in this file.
        //
        // it would seem simplest to just let that code call tud_task() at the end, however this
        // code might run during the call to tud_task() and we might miss a necessary tud_task() call
        //
        // if we are using a periodic timer (crit_sec is not initialized in this case),
        // then we are happy just to wait until the next tick, however when we are not using a periodic timer,
        // we must kick off a one-shot timer to make sure the tud_task() DOES run (this method
        // will be called again as a result, and will try the mutex_try_enter again, and if that fails
        // create another one shot timer again, and so on).
        if (critical_section_is_initialized(&one_shot_timer_crit_sec)) {
            bool need_timer;
            critical_section_enter_blocking(&one_shot_timer_crit_sec);
            need_timer = !one_shot_timer_pending;
            one_shot_timer_pending = true;
            critical_section_exit(&one_shot_timer_crit_sec);
            if (need_timer) {
                add_alarm_in_us(CDC_USB_TASK_INTERVAL_US, timer_task, NULL, true);
            }
        }
    }
}

static void usb_irq(void)
{
    irq_set_pending(low_priority_irq_num);
}

int cdc_usb_write(const uint8_t *buf, unsigned length)
{
    static uint64_t last_avail_time;
    int written = 0;

    if (!mutex_try_enter_block_until(&cdc_usb_mutex, make_timeout_time_ms(CDC_DEADLOCK_TIMEOUT_MS))) {
        return -1;
    }

    if (cdc_usb_connected()) {
        for (unsigned i = 0; i < length;) {
            unsigned n = length - i;
            uint32_t avail = tud_cdc_write_available();
            if (n > avail) n = avail;
            if (n) {
                uint32_t n2 = tud_cdc_write(buf + i, n);
                tud_task();
                tud_cdc_write_flush();
                i += n2;
                written = i;
                last_avail_time = time_us_64();
            } else {
                tud_task();
                tud_cdc_write_flush();
                if (!cdc_usb_connected() || (!tud_cdc_write_available() && time_us_64() > last_avail_time + CDC_USB_WRITE_TIMEOUT_US)) {
                    break;
                }
            }
        }
    } else {
        // reset our timeout
        last_avail_time = 0;
    }
    mutex_exit(&cdc_usb_mutex);
    return written;
}

void cdc_usb_write_flush(void)
{
    if (!mutex_try_enter_block_until(&cdc_usb_mutex, make_timeout_time_ms(CDC_DEADLOCK_TIMEOUT_MS))) {
        return;
    }
    do {
        tud_task();
    } while (tud_cdc_write_flush());
    mutex_exit(&cdc_usb_mutex);
}

int cdc_usb_read(uint8_t *buf, unsigned length)
{
    // note we perform this check outside the lock, to try and prevent possible deadlock conditions
    // with printf in IRQs (which we will escape through timeouts elsewhere, but that would be less graceful).
    //
    // these are just checks of state, so we can call them while not holding the lock.
    // they may be wrong, but only if we are in the middle of a tud_task call, in which case at worst
    // we will mistakenly think we have data available when we do not (we will check again), or
    // tud_task will complete running and we will check the right values the next time.
    //
    int rc = PICO_ERROR_NO_DATA;
    if (cdc_usb_connected() && tud_cdc_available()) {
        if (!mutex_try_enter_block_until(&cdc_usb_mutex, make_timeout_time_ms(CDC_DEADLOCK_TIMEOUT_MS))) {
            return PICO_ERROR_NO_DATA; // would deadlock otherwise
        }
        if (cdc_usb_connected() && tud_cdc_available()) {
            uint32_t count = tud_cdc_read(buf, length);
            rc = count ? (int)count : PICO_ERROR_NO_DATA;
        } else {
            // because our mutex use may starve out the background task, run tud_task here (we own the mutex)
            tud_task();
        }
        mutex_exit(&cdc_usb_mutex);
    }
    return rc;
}

bool cdc_usb_init(void)
{
    if (get_core_num() != alarm_pool_core_num(alarm_pool_get_default())) {
        // included an assertion here rather than just returning false, as this is likely
        // a coding bug, rather than anything else.
        assert(false);
        return false;
    }

    // initialize TinyUSB, as user hasn't explicitly linked it
    tusb_init();

    if (!mutex_is_initialized(&cdc_usb_mutex)) {
        mutex_init(&cdc_usb_mutex);
    }
    bool rc = true;
    low_priority_irq_num = (uint8_t)user_irq_claim_unused(true);

    irq_set_exclusive_handler(low_priority_irq_num, low_priority_worker_irq);
    irq_set_enabled(low_priority_irq_num, true);

    if (irq_has_shared_handler(USBCTRL_IRQ)) {
        critical_section_init_with_lock_num(&one_shot_timer_crit_sec, spin_lock_claim_unused(true));
        // we can use a shared handler to notice when there may be work to do
        irq_add_shared_handler(USBCTRL_IRQ, usb_irq, PICO_SHARED_IRQ_HANDLER_LOWEST_ORDER_PRIORITY);
    } else {
        // we use initialization state of the one_shot_timer_critsec as a flag
        memset(&one_shot_timer_crit_sec, 0, sizeof(one_shot_timer_crit_sec));
        rc = add_alarm_in_us(CDC_USB_TASK_INTERVAL_US, timer_task, NULL, true) >= 0;
    }

    configured = rc;
    return rc;
}

bool cdc_usb_deinit(void)
{
    if (get_core_num() != alarm_pool_core_num(alarm_pool_get_default())) {
        // included an assertion here rather than just returning false, as this is likely
        // a coding bug, rather than anything else.
        assert(false);
        return false;
    }

    assert(tud_inited()); // we expect the caller to have initialized when calling sdio_usb_init

    if (irq_has_shared_handler(USBCTRL_IRQ)) {
        spin_lock_unclaim(spin_lock_get_num(one_shot_timer_crit_sec.spin_lock));
        critical_section_deinit(&one_shot_timer_crit_sec);
        // we can use a shared handler to notice when there may be work to do
        irq_remove_handler(USBCTRL_IRQ, usb_irq);
    } else {
        // timer is disabled by disabling the irq
    }

    irq_set_enabled(low_priority_irq_num, false);
    user_irq_unclaim(low_priority_irq_num);

    configured = false;
    return true;
}

bool cdc_usb_configured(void)
{
    return configured;
}

bool cdc_usb_connected(void)
{
    return tud_cdc_connected();
}

bool cdc_usb_bytes_available(void)
{
    return tud_cdc_available();
}

uint32_t cdc_usb_baud_rate(void)
{
    return CDC_USD_BAUD_RATE;
}

uint32_t cdc_usb_tx_bytes_free(void)
{
    return tud_cdc_write_available();
}
