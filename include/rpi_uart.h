#pragma once

/**
 * UART link to Raspberry Pi 4B (3.3 V TTL).
 * Pi side: RX is BCM GPIO 15 (40-pin header pin 10) for /dev/serial0 when UART is enabled.
 * Wire: ESP32 GND–Pi GND; ESP32 TX (see rpi_uart.cpp)–Pi GPIO15 (pin 10). Leave ESP RX unconnected if
 * the ESP only transmits. Both sides are 3.3 V logic.
 *
 * ESP32-S3: UART2 on GPIO16 (RX) / GPIO17 (TX) by default; edit rpi_uart.cpp if those clash.
 */
void rpiUartInit();

/**
 * One line per call: "latitude_deg,longitude_deg\\r\\n" (7 decimal places).
 * Invalid (non-finite) coordinates are not sent.
 *
 * If the Pi still sees occasional garbage: use a solid GND, short wires, 115200 8N1 on the Pi,
 * and avoid routing UART next to switching power/noise sources.
 */
void rpiUartSendPosition(double latitude_deg, double longitude_deg);
