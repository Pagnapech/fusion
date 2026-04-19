#pragma once

/**
 * ESP-NOW peer (controller / ground station). Replace with the peer's Wi-Fi STA MAC
 * (often printed at boot on the other ESP32). Broadcast FF:FF:FF:FF:FF:FF works for
 * lab tests; unicast is preferred for production.
 */
#define ESPNOW_PEER_MAC_BYTE0 0xFF
#define ESPNOW_PEER_MAC_BYTE1 0xFF
#define ESPNOW_PEER_MAC_BYTE2 0xFF
#define ESPNOW_PEER_MAC_BYTE3 0xFF
#define ESPNOW_PEER_MAC_BYTE4 0xFF
#define ESPNOW_PEER_MAC_BYTE5 0xFF

/** Wi-Fi channel (1–13). Both ESP-NOW peers must match for reliable links. */
#define ESPNOW_WIFI_CHANNEL 1
