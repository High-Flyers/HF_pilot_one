#include "Arduino.h"
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>

#define GPS_SERIAL_RX 16
#define GPS_SERIAL_TX 17
#define GPS_SERIAL_BAUD_RATE 38400
#define GPS_SERIAL_MODE SERIAL_8N1

#define MPU_ADRESS (u_int8_t)0x69

TinyGPSPlus gps;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_MPU6050 mpu;

void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(", "));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.satellites.isValid())
  {
    Serial.print(F("Satellites: "));
    Serial.print(gps.satellites.value());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  sensors_event_t mag_event;
  mag.getEvent(&mag_event);
  {
    Serial.print(" Compass X: ");
    Serial.print(mag_event.magnetic.x);
    Serial.print(" Y: ");
    Serial.print(mag_event.magnetic.y);
    Serial.print(" Z: ");
    Serial.print(mag_event.magnetic.z);
  }

  {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values */
    Serial.print(" ");
    Serial.print("AccelX:");
    Serial.print(a.acceleration.x);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(a.acceleration.y);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(a.acceleration.z);
    Serial.print(", ");
    Serial.print("GyroX:");
    Serial.print(g.gyro.x);
    Serial.print(",");
    Serial.print("GyroY:");
    Serial.print(g.gyro.y);
    Serial.print(",");
    Serial.print("GyroZ:");
    Serial.print(g.gyro.z);
  }

  Serial.println();
}

void setup()
{
  Serial.begin(9600);
  Serial2.begin(GPS_SERIAL_BAUD_RATE, GPS_SERIAL_MODE, GPS_SERIAL_RX, GPS_SERIAL_TX);

  if (!mag.begin())
  {
    while (1)
    {
      Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
      delay(500);
    }
  }

  if (!mpu.begin(MPU_ADRESS))
  {
    while (1)
    {
      Serial.println("Failed to find MPU6050 chip");
      delay(500);
    }
  }
  Serial.println("MPU6050 Found!");

  // setupt motion detection
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
}

void loop()
{
  while (Serial2.available() > 0)
  {
    if (gps.encode(Serial2.read()))
      displayInfo();
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
  }
}
