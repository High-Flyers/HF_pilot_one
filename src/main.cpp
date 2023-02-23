#include "Arduino.h"
#include "Sensors.h"
#include "lookup.h"
#include "mqtt.h"

TaskHandle_t mpu_task, compass_task, gps_task;
MPU_data mpu_d;
Compass_data compass_d;
GPS_data gps_d;
mqtt_conn_data mqtt_conn;
char mqtt_data[2 * (sizeof(MPU_data) + sizeof(Compass_data) + sizeof(GPS_data)) + 1] = {0};

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

void mqtt_pack_and_send()
{
  int i;
  for (i = 0; i < 2 * sizeof(MPU_data); i += 2)
  {
    char raw = ((char *)&mpu_d)[i];
    mqtt_data[i] = ascii_lookup[raw & 0x0F];
    mqtt_data[i + 1] = ascii_lookup[(raw & 0xF0) >> 4];
  }
  for (i = 0; i < 2 * sizeof(Compass_data); i += 2)
  {
    char raw = ((char *)&compass_d)[i];
    mqtt_data[2 * sizeof(MPU_data) + i] = ascii_lookup[raw & 0x0F];
    mqtt_data[2 * sizeof(MPU_data) + i + 1] = ascii_lookup[(raw & 0xF0) >> 4];
  }
  for (i = 0; i < 2 * sizeof(GPS_data); i += 2)
  {
    char raw = ((char *)&gps_d)[i];
    mqtt_data[2 * (sizeof(MPU_data) + sizeof(Compass_data)) + i] = ascii_lookup[raw & 0x0F];
    mqtt_data[2 * (sizeof(MPU_data) + sizeof(Compass_data)) + i + 1] = ascii_lookup[(raw & 0xF0) >> 4];
  }
  mqtt_data[2 * (sizeof(MPU_data) + sizeof(Compass_data) + sizeof(GPS_data))] = 0;
  mqtt_conn.mqtt_client->publish("HP_PILOT_ONE/sensors", mqtt_data);
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
  wifi_mqtt_init(&mqtt_conn);

  delay(1000);
}

void loop()
{
  mqtt_loop(&mqtt_conn);
  // displayInfo();
  mqtt_pack_and_send();
  // delay(400);
}
