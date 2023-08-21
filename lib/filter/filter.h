#ifndef HF_PILOT_FILTER
#define HF_PILOT_FILTER

#include "../madgwick/madgwick.h"
#include "../sensors/Sensors.h"

#define FILTER_RATE 100.f

typedef struct
{
    float roll, pitch, yaw;
} Filter_data;

typedef struct
{
    Filter_data data;
    MPU_data *mpu_d;
    Compass_data *compass_d;
    GPS_data *gps_d;
} Filter_state;

void filter_init(Filter_state *state, MPU_data *mpu_d, Compass_data *compass_d, GPS_data *gps_d);
void filter_handler(void *state);

#endif