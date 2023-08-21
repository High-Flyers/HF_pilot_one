#include "filter.h"
#include "Arduino.h"
#include <Functions.h> // CCcontrol library

static uint8_t L = 3;                                 /* How many states we have -> roll pitch yaw */
static float r = 1.f;                                 /* Tuning factor for noise */
static float q = 1.f;                                 /* Tuning factor for disturbance */
static float alpha = 0.1f;                            /* Alpha value - A small number like 0.01 -> 1.0 */
static float beta = 2.0f;                             /* Beta value - Normally 2 for gaussian noise */
static float Rv[3 * 3] = {q, 0, 0, 0, q, 0, 0, 0, q}; /* Initial disturbance covariance matrix - Recommended to use identity matrix */
static float Rn[3 * 3] = {r, 0, 0, 0, r, 0, 0, 0, r}; /* Initial noise covariance matrix - Recommended to use identity matrix */
static float S[3 * 3] = {1, 0, 0, 0, 1, 0, 0, 0, 1};  /* Initial covariance matrix - Recommended to use identity matrix */
static float xhat[3] = {0, 0, 1};                     /* Estimated state vector */
static float y[3] = {0, 0, 0};                        /* This is our measurement, acc and mag fused */
static float u[3] = {0, 0, 0};                        /* u is gyro reading */
static float x[3] = {0, 0, 0};                        /* State vector for the system (unknown in reality) */
static float dt_ms = 0.0f;

static float rot_mat[3 * 3] = {
    0.7071, 0.7071, -0.0000, 0.7071, -0.7071, -0.0000,
    -0.0000, -0.0000, -1.0000};

#define RAD_TO_DEG 57.29577951308232

#define GYRO_OFFSET_PITCH -3.2743889
#define GYRO_OFFSET_ROLL -1.2538174
#define GYRO_OFFSET_YAW -1.9277307

void filter_init(Filter_state *state, MPU_data *mpu_d, Compass_data *compass_d, GPS_data *gps_d)
{
    state->mpu_d = mpu_d;
    state->compass_d = compass_d;
    state->gps_d = gps_d;
    state->data.pitch = 0.f;
    state->data.roll = 0.f;
    state->data.yaw = 0.f;
}

/* y = mat * x , mat is 3x3 */
void mat_mul(float *y, float *mat, float *x)
{
    y[0] = mat[0] * x[0] + mat[1] * x[1] + mat[2] * x[2];
    y[1] = mat[3] * x[0] + mat[4] * x[1] + mat[5] * x[2];
    y[2] = mat[6] * x[0] + mat[7] * x[1] + mat[8] * x[2];
}

/* transition function */
void F_trans(float dx[], float x[], float u[])
{
    dx[0] = u[0];
    dx[1] = u[1];
    dx[2] = u[2];
}

void filter_handler(void *param)
{
    Filter_state *state = (Filter_state *)param;

    u_int32_t rate = (u_int32_t)(1000.0f * (1.0f / FILTER_RATE));
    dt_ms = rate;

    // wait for data to come from sensors
    while (true)
    {
        bool cond1 = state->mpu_d->acc_x == 0.f && state->mpu_d->acc_y == 0.f && state->mpu_d->acc_z == 0.f;
        bool cond2 = state->mpu_d->gyro_x == 0.f && state->mpu_d->gyro_y == 0.f && state->mpu_d->gyro_z == 0.f;
        bool cond3 = state->compass_d->mag_x == 0.f && state->compass_d->mag_y == 0.f && state->compass_d->mag_z == 0.f;
        if (!(cond1 || cond2 || cond3))
            break;
    }

    float temp_calc1[3], temp_calc2[3];
    while (true)
    {
        u_int32_t start = millis();

        // translate gyro reading, correct by offset and multiply by dt
        temp_calc1[0] = state->mpu_d->gyro_x;
        temp_calc1[1] = state->mpu_d->gyro_y;
        temp_calc1[2] = state->mpu_d->gyro_z;
        mat_mul(u, rot_mat, temp_calc1);
        u[0] = (RAD_TO_DEG * u[0] + GYRO_OFFSET_ROLL) * dt_ms;
        u[1] = (RAD_TO_DEG * u[1] + GYRO_OFFSET_PITCH) * dt_ms;
        u[2] = (RAD_TO_DEG * u[2] + GYRO_OFFSET_YAW) * dt_ms;

        // get compass and acc measurment, translate and change to roll pitch yaw
        temp_calc1[0] = state->mpu_d->acc_x;
        temp_calc1[1] = state->mpu_d->acc_y;
        temp_calc1[2] = state->mpu_d->acc_z;
        mat_mul(temp_calc2, rot_mat, temp_calc1);
        float roll_acc = RAD_TO_DEG * atan2f(temp_calc2[1], temp_calc2[2]);
        float pitch_acc = RAD_TO_DEG * asinf(temp_calc2[0] / sqrtf(temp_calc2[0] * temp_calc2[0] + temp_calc2[1] * temp_calc2[1] + temp_calc2[2] * temp_calc2[2]));
        if (temp_calc2[2] < 0.f)
        {
            if (temp_calc2[0] >= 0.f)
                pitch_acc = 180.f - pitch_acc;
            else
                pitch_acc = -180.f - pitch_acc;
        }

        temp_calc1[0] = state->compass_d->mag_x;
        temp_calc1[1] = state->compass_d->mag_y;
        temp_calc1[2] = state->compass_d->mag_z;
        mat_mul(temp_calc2, rot_mat, temp_calc1);
        float yaw_mag = RAD_TO_DEG * atan2f(temp_calc2[1], temp_calc2[0]);
        if (yaw_mag < 0.f)
            yaw_mag += 360.f;

        y[0] = roll_acc;
        y[1] = pitch_acc;
        y[2] = yaw_mag;

        // // use this data to pass into filter
        // sr_ukf_state_estimation(y, xhat, Rn, Rv, u, F_trans, S, alpha, beta, L);

        // // move data to output struct
        // state->data.roll = xhat[0];
        // state->data.pitch = xhat[1];
        // state->data.yaw = xhat[2];

        // state->data.roll = 0.3 * y[0] + 0.7 * (state->data.roll + u[0]);
        // state->data.pitch = 0.3 * y[1] + 0.7 * (state->data.pitch + u[1]);
        // state->data.yaw = 0.3 * y[2] + 0.7 * (state->data.yaw + u[2]);

        // state->data.roll += u[0];
        // state->data.pitch += u[1];
        // state->data.yaw += u[2];

        state->data.roll = y[0];
        state->data.pitch = y[1];
        state->data.yaw = y[2];

        u_int32_t end = millis() - start;
        if (end < rate)
        {
            dt_ms = (float)rate / 1000.f;
            delay(rate - end);
        }
        else
        {
            dt_ms = (float)end / 1000.f;
        }
    }
}