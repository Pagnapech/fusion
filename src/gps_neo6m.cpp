#include "gps_neo6m.h"
#include <TinyGPS++.h>

static TinyGPSPlus gps;
static HardwareSerial* gpsSerial = nullptr;

static unsigned long lastByteTime = 0;
static bool hasSeenSerialData = false;

bool gpsInit(HardwareSerial &serialPort, int rxPin, int txPin, uint32_t baud) {
  gpsSerial = &serialPort;

  if (txPin < 0) {
    gpsSerial->begin(baud, SERIAL_8N1, rxPin, -1);
  } else {
    gpsSerial->begin(baud, SERIAL_8N1, rxPin, txPin);
  }

  lastByteTime = 0;
  hasSeenSerialData = false;
  return true;
}

void gpsUpdate() {
  if (gpsSerial == nullptr) {
    return;
  }

  while (gpsSerial->available()) {
    char c = gpsSerial->read();
    gps.encode(c);
    hasSeenSerialData = true;
    lastByteTime = millis();
  }
}

GPSStatus gpsGetStatus() {
  if (!hasSeenSerialData) {
    return GPS_NO_SERIAL_DATA;
  }

  if (gps.location.isValid()) {
    return GPS_VALID_FIX;
  }

  return GPS_RECEIVING_DATA_NO_FIX;
}

bool gpsRead(GPSData &data) {
  data.status = gpsGetStatus();
  data.valid = gps.location.isValid();

  data.latitude = gps.location.isValid() ? gps.location.lat() : 0.0;
  data.longitude = gps.location.isValid() ? gps.location.lng() : 0.0;
  data.altitudeMeters = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  data.speedKmph = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  data.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;

  data.year = gps.date.isValid() ? gps.date.year() : 0;
  data.month = gps.date.isValid() ? gps.date.month() : 0;
  data.day = gps.date.isValid() ? gps.date.day() : 0;

  data.hour = gps.time.isValid() ? gps.time.hour() : 0;
  data.minute = gps.time.isValid() ? gps.time.minute() : 0;
  data.second = gps.time.isValid() ? gps.time.second() : 0;

  data.charsProcessed = gps.charsProcessed();

  return data.valid;
}

const char* gpsStatusToString(GPSStatus status) {
  switch (status) {
    case GPS_NO_SERIAL_DATA:
      return "NO_SERIAL_DATA";
    case GPS_RECEIVING_DATA_NO_FIX:
      return "RECEIVING_DATA_NO_FIX";
    case GPS_VALID_FIX:
      return "VALID_FIX";
    default:
      return "UNKNOWN";
  }
}

void gpsPrint(const GPSData &data) {
  Serial.println("----- GPS -----");

  Serial.print("Status: ");
  Serial.println(gpsStatusToString(data.status));

  Serial.print("Chars processed: ");
  Serial.println(data.charsProcessed);

  Serial.print("Satellites: ");
  Serial.println(data.satellites);

  if (data.status == GPS_NO_SERIAL_DATA) {
    Serial.println("No GPS serial data received.");
  } else if (data.status == GPS_RECEIVING_DATA_NO_FIX) {
    Serial.println("GPS data received, but no valid fix yet.");
  } else if (data.status == GPS_VALID_FIX) {
    Serial.println("GPS fix acquired.");
    Serial.print("Latitude: ");
    Serial.println(data.latitude, 6);

    Serial.print("Longitude: ");
    Serial.println(data.longitude, 6);

    Serial.print("Altitude (m): ");
    Serial.println(data.altitudeMeters, 2);

    Serial.print("Speed (km/h): ");
    Serial.println(data.speedKmph, 2);

    Serial.print("Date (UTC): ");
    Serial.print(data.year);
    Serial.print("-");
    Serial.print(data.month);
    Serial.print("-");
    Serial.println(data.day);

    Serial.print("Time (UTC): ");
    Serial.print(data.hour);
    Serial.print(":");
    Serial.print(data.minute);
    Serial.print(":");
    Serial.println(data.second);
  }

  Serial.println("----------------");
}