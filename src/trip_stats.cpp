#include "trip_stats.h"

Trip_stats_t trip_stats;

void TRIP_STATS_init ()
{
    trip_stats.trip_time.begin();
}

void TRIP_STATS_reset ()
{
    trip_stats.distance_mm = 0;
    trip_stats.watt_s_x100 = 0;
    trip_stats.trip_time.setHours(0);
    trip_stats.trip_time.setMinutes(0);
    trip_stats.trip_time.setSeconds(0);
}

// return average speed for the trip
// in km/h x 10
uint16_t TRIP_STATS_get_avg_speed ()
{
    // get the elapsed trip time, in seconds
    int trip_time = trip_stats.trip_time.getSeconds() + (trip_stats.trip_time.getMinutes() * 60) + (trip_stats.trip_time.getHours() * 3600);
    // convert mm/s to km/h * 10
    return ((uint16_t)(trip_stats.distance_mm / 1000 / trip_time * 36));
}

// return watt hours used per km travelled
uint16_t TRIP_STATS_get_wh_per_km ()
{
    return ((trip_stats.distance_mm == 0) ? 0 : trip_stats.watt_s_x100 / trip_stats.distance_mm * 100 / 36); // note, avoid div/0!
}

