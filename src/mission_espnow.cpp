#include "mission_espnow.h"
#include "espnow_config.h"
#include "waypoint_nav.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

static uint8_t s_peerMac[6] = {ESPNOW_PEER_MAC_BYTE0, ESPNOW_PEER_MAC_BYTE1, ESPNOW_PEER_MAC_BYTE2,
                               ESPNOW_PEER_MAC_BYTE3, ESPNOW_PEER_MAC_BYTE4, ESPNOW_PEER_MAC_BYTE5};

static void onEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  (void)mac_addr;
  if (data == nullptr || len <= 0) {
    return;
  }
  char line[256];
  const int copyLen = (len < (int)sizeof(line) - 1) ? len : (int)sizeof(line) - 1;
  memcpy(line, data, (size_t)copyLen);
  line[copyLen] = '\0';

  for (int i = (int)strlen(line) - 1; i >= 0; --i) {
    if (line[i] == '\r' || line[i] == '\n' || line[i] == ' ') {
      line[i] = '\0';
    } else {
      break;
    }
  }

  waypointNavProcessCommand(line);
}

void missionNotify(const char *msg) {
  if (msg == nullptr) {
    return;
  }
  uint8_t buf[250];
  const size_t maxLen = sizeof(buf) - 1;
  size_t n = strnlen(msg, maxLen + 1);
  if (n > maxLen) {
    n = maxLen;
  }
  memcpy(buf, msg, n);

  esp_err_t err = esp_now_send(s_peerMac, buf, n);
  if (err != ESP_OK) {
    /* Optional: Serial.printf("esp_now_send failed: %d\n", (int)err); */
    (void)err;
  }
}

void missionEspNowInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  WiFi.setSleep(false);

  esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    return;
  }

  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, s_peerMac, 6);
  peer.channel = ESPNOW_WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    return;
  }

  waypointNavInit();
}
