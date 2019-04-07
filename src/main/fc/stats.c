
#include "platform.h"

#ifdef USE_PERSISTENT_STATS

#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/stats.h"

#include "io/beeper.h"
#include "io/gps.h"

#include "pg/stats.h"


#define MIN_FLIGHT_TIME_TO_RECORD_STATS_S 10    //prevent recording stats for that short "flights" [s]
#define STATS_SAVE_DELAY_MS              500    //let disarming complete and save stats after this time

static timeMs_t arm_millis;
static uint32_t arm_distance_cm;
static timeMs_t save_pending_millis;  // 0 = none

#ifdef USE_GPS
    #define DISTANCE_FLOWN_CM (GPS_distanceFlownInCm)
#else
    #define DISTANCE_FLOWN_CM (0)
#endif

void statsOnArm(void)
{
    arm_millis      = millis();
    arm_distance_cm = DISTANCE_FLOWN_CM;
}

void statsOnDisarm(void)
{
    if (statsConfig()->stats_enabled) {
        uint32_t dt = (millis() - arm_millis) / 1000;
        if (dt >= MIN_FLIGHT_TIME_TO_RECORD_STATS_S) {
            statsConfigMutable()->stats_total_flights += 1;    //arm/flight counter
            statsConfigMutable()->stats_total_time_s += dt;   //[s]
            statsConfigMutable()->stats_total_dist_m += (DISTANCE_FLOWN_CM - arm_distance_cm) / 100;   //[m]
            /* signal that stats need to be saved but don't execute time consuming flash operation
               now - let the disarming process complete and then execute the actual save */
            save_pending_millis = millis();
        }
    }
}

void statsOnLoop(void)
{
    /* check for pending flash write */
    if (save_pending_millis && millis()-save_pending_millis > STATS_SAVE_DELAY_MS) {
        if (ARMING_FLAG(ARMED)) {
            /* re-armed - don't save! */
        }
        else {
            if (isConfigDirty()) {
                /* There are some adjustments made in the configuration and we don't want
                   to implicitly save it... We can't currently save part of the configuration,
                   so we simply don't execute the stats save operation at all. This will result
                   in missing stats update *if* rc-adjustments were made during the flight. */
            }
            else {
                writeEEPROM();
                /* repeat disarming beep indicating the stats save is complete */
                beeper(BEEPER_DISARMING);
            }
        }
        save_pending_millis = 0;
    }
}

#endif
