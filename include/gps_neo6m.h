#pragma once

#include <Arduino.h>

enum GPSStatus {
  GPS_NO_SERIAL_DATA = 0,
  GPS_RECEIVING_DATA_NO_FIX,
  GPS_VALID_FIX
};

struct GPSData {
  GPSStatus status;

  bool valid;
  double latitude;
  double longitude;
  double altitudeMeters;
  double speedKmph;
  int satellites;

  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;

  unsigned long charsProcessed;
};

bool gpsInit(HardwareSerial &serialPort, int rxPin, int txPin = -1, uint32_t baud = 9600);
void gpsUpdate();
bool gpsRead(GPSData &data);
GPSStatus gpsGetStatus();
void gpsPrint(const GPSData &data);
const char* gpsStatusToString(GPSStatus status);