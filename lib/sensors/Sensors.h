#ifndef HF_PILOT_SENSORS
#define HF_PILOT_SENSORS
#include <cstdint>

#define GPS_RATE 5.0f
#define MPU_RATE 500.0f
#define COMPASS_RATE 20.0f

#define GPS_SERIAL_RX 16
#define GPS_SERIAL_TX 17
#define GPS_SERIAL_BAUD_RATE 38400
#define GPS_SERIAL_MODE SERIAL_8N1

#define COMPASS_CORRECTION_X -8.8181801
#define COMPASS_CORRECTION_Y -17.2272739
#define COMPASS_CORRECTION_Z 8.7244892

#define MPU_ADRESS (u_int8_t)0x69

typedef struct
{
    double lat, lon;
    float alt;
    u_int16_t avalaible;
    u_int16_t sat_amount;
} GPS_data;

void gps_init(GPS_data *data);
void gps_handler(void *data);

typedef struct
{
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
} MPU_data;

void mpu_init(MPU_data *data);
void mpu_handler(void *data);

typedef struct
{
    float mag_x, mag_y, mag_z;
} Compass_data;

void compass_init(Compass_data *data);
void compass_handler(void *data);

#endif