#include "Arduino.h"
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

#define GPS_SERIAL_RX 16
#define GPS_SERIAL_TX 17
#define GPS_SERIAL_BAUD_RATE 38400
#define GPS_SERIAL_MODE SERIAL_8N1



TinyGPSPlus gps;

void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
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

  Serial.println();
}

void setup()
{
  Serial.begin(9600);
  Serial2.begin(GPS_SERIAL_BAUD_RATE, GPS_SERIAL_MODE, GPS_SERIAL_RX, GPS_SERIAL_TX);
}

void loop()
{
  while (Serial2.available() > 0){
    if (gps.encode(Serial2.read()))
      displayInfo();
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
  }
}

