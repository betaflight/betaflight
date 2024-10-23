#include "pico/util/datetime.h"

#include <stdio.h>

#if PICO_INCLUDE_RTC_DATETIME
static const char *DATETIME_MONTHS[12] = {
        "January",
        "February",
        "March",
        "April",
        "May",
        "June",
        "July",
        "August",
        "September",
        "October",
        "November",
        "December"
};

static const char *DATETIME_DOWS[7] = {
        "Sunday",
        "Monday",
        "Tuesday",
        "Wednesday",
        "Thursday",
        "Friday",
        "Saturday",
};

void datetime_to_str(char *buf, uint buf_size, const datetime_t *t) {
    snprintf(buf,
             buf_size,
             "%s %d %s %d:%02d:%02d %d",
             DATETIME_DOWS[t->dotw],
             t->day,
             DATETIME_MONTHS[t->month - 1],
             t->hour,
             t->min,
             t->sec,
             t->year);
};


bool time_to_datetime(time_t time, datetime_t *dt) {
    struct tm local;
    if (localtime_r(&time, &local)) {
        dt->year = (int16_t) (local.tm_year + 1900); // 0..4095
        dt->month = (int8_t) (local.tm_mon + 1);     // 1..12, 1 is January
        dt->day = (int8_t) local.tm_mday;            // 1..28,29,30,31 depending on month
        dt->dotw = (int8_t) local.tm_wday;           // 0..6, 0 is Sunday
        dt->hour = (int8_t) local.tm_hour;           // 0..23
        dt->min = (int8_t) local.tm_min;             // 0..59
        dt->sec = (int8_t) local.tm_sec;             // 0..59
        return true;
    }
    return false;
}

bool datetime_to_time(const datetime_t *dt, time_t *time) {
    struct tm local;
    local.tm_year = dt->year - 1900;
    local.tm_mon = dt->month - 1;
    local.tm_mday = dt->day;
    local.tm_hour = dt->hour;
    local.tm_min = dt->min;
    local.tm_sec = dt->sec;
    *time = mktime(&local);
    return *time >= 0;
}

#endif

uint64_t timespec_to_ms(const struct timespec *ts) {
    int64_t rc = ts->tv_sec * 1000;
    rc += ts->tv_nsec / 1000000;
    return (uint64_t) rc;
}

void ms_to_timespec(uint64_t ms, struct timespec *ts) {
    ts->tv_sec = (time_t)((int64_t)ms / 1000);
    ts->tv_nsec = ((long)((int64_t)ms % 1000)) * 1000000;
}

uint64_t timespec_to_us(const struct timespec *ts) {
    int64_t rc = ts->tv_sec * 1000000;
    rc += ts->tv_nsec / 1000;
    return (uint64_t) rc;
}

void us_to_timespec(uint64_t ms, struct timespec *ts) {
    ts->tv_sec = (time_t)((int64_t)ms / 1000000);
    ts->tv_nsec = ((long)((int64_t)ms % 1000000)) * 1000;
}

