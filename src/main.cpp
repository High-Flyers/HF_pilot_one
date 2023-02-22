#include "Arduino.h"
#include "Sensors.h"

TaskHandle_t mpu_task, compass_task, gps_task;
MPU_data mpu_d;
Compass_data compass_d;
GPS_data gps_d;

void displayInfo()
{
  Serial.print("ACC: ");
  Serial.print(mpu_d.acc_x);
  Serial.print(" ");
  Serial.print(mpu_d.acc_y);
  Serial.print(" ");
  Serial.print(mpu_d.acc_z);
  Serial.print(" ");
  Serial.print("GYRO: ");
  Serial.print(mpu_d.gyro_x);
  Serial.print(" ");
  Serial.print(mpu_d.gyro_y);
  Serial.print(" ");
  Serial.print(mpu_d.gyro_z);
  Serial.print(" ");
  Serial.print("COMPASS: ");
  Serial.print(compass_d.mag_x);
  Serial.print(" ");
  Serial.print(compass_d.mag_y);
  Serial.print(" ");
  Serial.print(compass_d.mag_z);
  Serial.print(" ");
  if (gps_d.avalaible)
  {
    Serial.print("GPS: ");
    Serial.print(gps_d.lat);
    Serial.print(" ");
    Serial.print(gps_d.lon);
    Serial.print(" ");
  }

  Serial.println();
}

void start_sensors()
{
  mpu_init(&mpu_d);
  gps_init(&gps_d);
  compass_init(&compass_d);

  xTaskCreate(mpu_handler, "mpu_handler", 2000, &mpu_d, 1, &mpu_task);
  xTaskCreate(compass_handler, "compass_handler", 2000, &compass_d, 1, &compass_task);
  xTaskCreate(gps_handler, "gps_handler", 2000, &gps_d, 1, &gps_task);
}

void setup()
{
  Serial.begin(9600);
  start_sensors();
  delay(1000);
}

void loop()
{
  displayInfo();
  delay(400);
}
