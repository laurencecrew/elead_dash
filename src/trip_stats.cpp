#include "trip_stats.h"

Trip_stats_t trip_stats;

void TRIP_STATS_init ()
{
    trip_stats.trip_time.begin();
}

void TRIP_STATS_reset ()
{
    trip_stats.avg_speed_x10 = 0;
    trip_stats.distance_mm = 0;
    trip_stats.watt_s_x100 = 0;
    trip_stats.trip_time.setHours(0);
    trip_stats.trip_time.setMinutes(0);
    trip_stats.trip_time.setSeconds(0);
}

