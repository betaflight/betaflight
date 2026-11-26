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

// No Betaflight headers here: <windows.h> macros clash with Betaflight names

// Vista, for CreateWaitableTimerExW
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0600
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <winsock2.h>
#include <windows.h>

#include "sitl_win32.h"

#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

#define SITL_WIN_LOOP_SLEEP_US 400

void sitlWinInit(void)
{
    // Windows has no line-buffered stdout
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        fprintf(stderr, "[SITL] WSAStartup failed\n");
        exit(1);
    }
}

static void timerSleepUs(uint32_t us)
{
    static __thread HANDLE timer = NULL;

    if (timer == NULL) {
        timer = CreateWaitableTimerExW(NULL, NULL, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
    }
    if (timer == NULL) {
        timer = CreateWaitableTimerW(NULL, TRUE, NULL);
    }

    LARGE_INTEGER due;
    due.QuadPart = -(LONGLONG)us * 10;
    if (timer != NULL && SetWaitableTimer(timer, &due, 0, NULL, NULL, FALSE)) {
        WaitForSingleObject(timer, INFINITE);
    } else {
        Sleep((us + 999) / 1000);
    }
}

void sitlWinSleepUs(uint32_t us)
{
    if (us >= 1000) {
        timerSleepUs(us);
    } else {
        LARGE_INTEGER frequency;
        LARGE_INTEGER now;

        QueryPerformanceFrequency(&frequency);
        QueryPerformanceCounter(&now);
        const LONGLONG end = now.QuadPart + (LONGLONG)us * frequency.QuadPart / 1000000;
        for (; now.QuadPart < end; QueryPerformanceCounter(&now)) {
            SwitchToThread();
        }
    }
}

// Windows cannot sleep 50 us and a busy wait costs a core: sleep once the delays add up
void sitlWinLoopDelayUs(uint32_t us)
{
    static uint32_t requestedUs = 0;

    requestedUs += us;
    if (requestedUs >= SITL_WIN_LOOP_SLEEP_US) {
        requestedUs = 0;
        timerSleepUs(us);
    }
}

char *strsep(char **stringp, const char *delim)
{
    char *token = *stringp;

    if (token != NULL) {
        char *end = token + strcspn(token, delim);
        if (*end != '\0') {
            *end = '\0';
            *stringp = end + 1;
        } else {
            *stringp = NULL;
        }
    }
    return token;
}

struct tm *gmtime_r(const time_t *timep, struct tm *result)
{
    return (gmtime_s(result, timep) == 0) ? result : NULL;
}
