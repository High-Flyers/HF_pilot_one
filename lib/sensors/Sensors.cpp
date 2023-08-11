#include "Arduino.h"
#include "Sensors.h"
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>

void gps_init(GPS_data *data)
{
    data->avalaible = false;
    data->lat = 0.0;
    data->lon = 0.0;
    data->alt = 0.0;
    data->sat_amount = 0;
}

void gps_handler(void *param)
{
    GPS_data *data = (GPS_data *)param;
    Serial2.begin(GPS_SERIAL_BAUD_RATE, GPS_SERIAL_MODE, GPS_SERIAL_RX, GPS_SERIAL_TX);
    TinyGPSPlus gps;
    u_int32_t rate = (u_int32_t)(1000.0f * (1.0f / GPS_RATE));

    while (true)
    {
        u_int32_t start = millis();

        if (Serial2.available() > 0)
        {
            if (!gps.encode(Serial2.read()))
                continue;
            if (gps.location.isValid() && gps.satellites.isValid())
            {
                data->lat = gps.location.lat();
                data->lon = gps.location.lng();
                data->sat_amount = gps.satellites.value();
                data->avalaible = data->sat_amount >= 4;
            }
            else
                data->avalaible = false;

            if (gps.altitude.isValid())
                data->alt = (float)gps.altitude.meters();
        }

        u_int32_t end = millis() - start;
        if (end < rate)
            delay(rate - end);
    }
}

void mpu_init(MPU_data *data)
{
    data->acc_x = 0.0f;
    data->acc_y = 0.0f;
    data->acc_z = 0.0f;
    data->gyro_x = 0.0f;
    data->gyro_y = 0.0f;
    data->gyro_z = 0.0f;
}

void mpu_handler(void *param)
{
    MPU_data *data = (MPU_data *)param;

    Adafruit_MPU6050 mpu;
    while (!mpu.begin(MPU_ADRESS))
    {
        Serial.println("Failed to find MPU6050 chip");
        data->acc_x = -1.0;
        data->acc_x = -1.0;
        data->acc_x = -1.0;
        delay(500);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_1000_DEG);

    u_int32_t rate = (u_int32_t)(1000.0f * (1.0f / MPU_RATE));

    while (true)
    {
        u_int32_t start = millis();

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        data->acc_x = a.acceleration.x;
        data->acc_y = a.acceleration.y;
        data->acc_z = a.acceleration.z;

        data->gyro_x = g.gyro.x;
        data->gyro_y = g.gyro.y;
        data->gyro_z = g.gyro.z;

        u_int32_t end = millis() - start;
        if (end < rate)
            delay(rate - end);
    }
}

void compass_init(Compass_data *data)
{
    data->mag_x = 0.0f;
    data->mag_y = 0.0f;
    data->mag_z = 0.0f;
}

void compass_handler(void *param)
{
    Compass_data *data = (Compass_data *)param;
    Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
    if (!mag.begin())
    {
        while (1)
        {
            Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
            delay(500);
        }
    }

    u_int32_t rate = (u_int32_t)(1000.0f * (1.0f / COMPASS_RATE));

    while (true)
    {
        u_int32_t start = millis();

        sensors_event_t mag_event;
        mag.getEvent(&mag_event);
        data->mag_x = -(mag_event.magnetic.x - COMPASS_CORRECTION_X);
        data->mag_y = -(mag_event.magnetic.y - COMPASS_CORRECTION_Y);
        data->mag_z = mag_event.magnetic.z - COMPASS_CORRECTION_Z;

        u_int32_t end = millis() - start;
        if (end < rate)
            delay(rate - end);
    }
}