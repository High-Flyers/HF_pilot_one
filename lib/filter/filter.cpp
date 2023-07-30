#include "filter.h"
#include "Arduino.h"

void filter_init(Filter_state *state, MPU_data *mpu_d, Compass_data *compass_d, GPS_data *gps_d)
{
    state->mad_state = Madgwick();
    state->mad_state.begin(FILTER_RATE);
    state->mpu_d = mpu_d;
    state->compass_d = compass_d;
    state->gps_d = gps_d;
    state->data.pitch = 0.f;
    state->data.roll = 0.f;
    state->data.yaw = 0.f;
}

void filter_handler(void *param)
{
    Filter_state *state = (Filter_state *)param;

    u_int32_t rate = (u_int32_t)(1000.0f * (1.0f / FILTER_RATE));

    while (true)
    {
        u_int32_t start = millis();

        state->mad_state.update(
            state->mpu_d->gyro_x,
            state->mpu_d->gyro_y,
            state->mpu_d->gyro_z,
            state->mpu_d->acc_x,
            state->mpu_d->acc_y,
            state->mpu_d->acc_z,
            state->compass_d->mag_x,
            state->compass_d->mag_y,
            state->compass_d->mag_z);

        // mpu only
        // state->mad_state.updateIMU(
        //     state->mpu_d->gyro_x,
        //     state->mpu_d->gyro_y,
        //     state->mpu_d->gyro_z,
        //     state->mpu_d->acc_x,
        //     state->mpu_d->acc_y,
        //     state->mpu_d->acc_z);

        state->data.pitch = state->mad_state.getPitch();
        state->data.roll = state->mad_state.getRoll();
        state->data.yaw = state->mad_state.getYaw();

        u_int32_t end = millis() - start;
        if (end < rate)
            delay(rate - end);
    }
}