#pragma once

#include <Arduino.h>

/**
 * Same text protocol as former BLE RX path (one command per packet, UTF-8, no embedded NUL):
 * CLEAR, WP,<idx>,<lat>,<lon>, START, STOP, STATUS, SAVE
 *
 * Status replies are sent via ESP-NOW unicast/broadcast to ESPNOW_PEER_MAC_* in espnow_config.h
 */
void missionEspNowInit();

/** Send one NUL-terminated status line (truncated to ESP-NOW max payload). */
void missionNotify(const char *msg);
