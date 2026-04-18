/**
 * APRS transmit helpers (PTT + payload formatting).
 *
 * markqvist/LibAPRS is wired for AVR (Timer1/ADC/DAC/ISR modem). It does not build or
 * run on ESP32-S3. To use that library, target an ATmega board or vendor an ESP32 HAL.
 * See platformio.ini for the upstream URL if you add a compatible environment.
 */

#include <Arduino.h>
#include <math.h>
#include <string.h>

#define PTT_PIN 4

const char *MY_CALLSIGN = "KE2DJD";
const int MY_SSID = 1;
const char *DEST_CALLSIGN = "APRS";
const char *PATH1 = "WIDE1";
const char *PATH2 = "WIDE2";

TaskHandle_t aprsTxTaskHandle = NULL;

struct APRSData {
  float latitude;
  float longitude;
  String comment;
};

void formatAPRSPosition(float lat, float lon, String comment, char *buffer) {
  float alat = fabsf(lat);
  float alon = fabsf(lon);
  int latDeg = (int)alat;
  float latMin = (alat - latDeg) * 60.0f;
  char latDir = (lat >= 0) ? 'N' : 'S';

  int lonDeg = (int)alon;
  float lonMin = (alon - lonDeg) * 60.0f;
  char lonDir = (lon >= 0) ? 'E' : 'W';

  sprintf(buffer, "!%02d%05.2f%c/%03d%05.2f%c-%s", latDeg, latMin, latDir, lonDeg, lonMin, lonDir,
          comment.c_str());
}

void aprsTxTask(void *parameter) {
  APRSData *data = (APRSData *)parameter;
  char payload[100];

  formatAPRSPosition(data->latitude, data->longitude, data->comment, payload);

  digitalWrite(PTT_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(400));

  // No AFSK modem on ESP32 yet — log what would be sent (AX.25/APRS stack TBD).
  Serial.print(F("[APRS] would TX: "));
  Serial.println(payload);

  vTaskDelay(pdMS_TO_TICKS(150));
  digitalWrite(PTT_PIN, LOW);

  delete data;
  vTaskDelete(NULL);
}

void transmitAPRSPacket(float lat, float lon, String comment) {
  APRSData *txData = new APRSData;
  txData->latitude = lat;
  txData->longitude = lon;
  txData->comment = comment;

  xTaskCreatePinnedToCore(aprsTxTask, "APRS_TX", 4096, (void *)txData, 1, &aprsTxTaskHandle, 1);
}

void aprs_radio_begin() {
  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);
  Serial.println(F("[APRS] radio begin (PTT only; LibAPRS not used on ESP32-S3)"));
}
