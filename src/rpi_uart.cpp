#include "rpi_uart.h"
#include <Arduino.h>
#include <HardwareSerial.h>

// UART2 on ESP32-S3 (avoid GPIO 9 if used elsewhere on your board).
// kRpiUartTxPin -> Raspberry Pi GPIO15 (header pin 10) RX. RX pin optional for send-only.
static constexpr int kRpiUartRxPin = 16;
static constexpr int kRpiUartTxPin = 17;
static constexpr uint32_t kRpiUartBaud = 115200;

static HardwareSerial sRpiSerial(2);

void rpiUartInit() {
  sRpiSerial.begin(kRpiUartBaud, SERIAL_8N1, kRpiUartRxPin, kRpiUartTxPin);
}

void rpiUartSendPosition(double latitude_deg, double longitude_deg) {
  sRpiSerial.printf("%.7f,%.7f\r\n", latitude_deg, longitude_deg);
}
