#include "rpi_uart.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>

// UART2 on ESP32-S3 (avoid GPIO 9 if used elsewhere on your board).
// kRpiUartTxPin -> Raspberry Pi GPIO15 (header pin 10) RX. RX pin optional for send-only.
static constexpr int kRpiUartRxPin = 16;
static constexpr int kRpiUartTxPin = 17;
static constexpr uint32_t kRpiUartBaud = 115200;

static HardwareSerial sRpiSerial(2);

void rpiUartInit() {
  sRpiSerial.begin(kRpiUartBaud, SERIAL_8N1, kRpiUartRxPin, kRpiUartTxPin);
  sRpiSerial.setTxBufferSize(256);
}

void rpiUartSendPosition(double latitude_deg, double longitude_deg) {
  /* Skip invalid samples (EKF/GPS not ready can yield NaN; printing them looks like "garbage"). */
  if (!isfinite(latitude_deg) || !isfinite(longitude_deg)) {
    return;
  }
  /* One snprintf + single write + flush avoids split packets and printf quirks on UART. */
  char buf[56];
  const int n = snprintf(buf, sizeof(buf), "%.7f,%.7f\r\n", latitude_deg, longitude_deg);
  if (n <= 0 || n >= (int)sizeof(buf)) {
    return;
  }
  sRpiSerial.write(reinterpret_cast<const uint8_t *>(buf), static_cast<size_t>(n));
  sRpiSerial.flush();
}

