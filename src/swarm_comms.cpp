#include "swarm_comms.h"
#include <WiFi.h>

namespace SwarmComms {

    // Internal State Variables (Hidden from main.cpp)
    SwarmRole currentRole;
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peerInfo;
    SwarmCommand_t swarmCmd;
    
    // FreeRTOS Queue Handle (Used only by Follower)
    QueueHandle_t commandQueue = NULL;

    // --- INTERNAL CALLBACKS ---
    void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
        // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "TX Success" : "TX Fail");
    }

    void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
        // Ignore if we aren't a follower, or if queue failed to create
        if (currentRole != ROLE_FOLLOWER || commandQueue == NULL) return;

        if (len == sizeof(SwarmCommand_t)) {
            SwarmCommand_t incomingCmd;
            memcpy(&incomingCmd, incomingData, sizeof(incomingCmd));
            
            // Network ID filter check
            if (incomingCmd.network_id != SWARM_NETWORK_ID) return;
            
            // Push to queue
            xQueueSendFromISR(commandQueue, &incomingCmd, NULL);
        }
    }

    // --- API FUNCTIONS ---
    bool init(SwarmRole role) {
        currentRole = role;

        WiFi.mode(WIFI_STA);

        if (esp_now_init() != ESP_OK) {
            Serial.println("FATAL: ESP-NOW Init Failed");
            return false;
        }

        if (role == ROLE_PRIMARY) {
            // Configure as Broadcaster
            esp_now_register_send_cb(OnDataSent);
            memcpy(peerInfo.peer_addr, broadcastAddress, 6);
            peerInfo.channel = SWARM_WIFI_CHANNEL;      
            peerInfo.encrypt = false;  

            if (esp_now_add_peer(&peerInfo) != ESP_OK) {
                Serial.println("FATAL: Failed to add broadcast peer");
                return false;
            }
            Serial.println("Primary Radio Initialized.");

        } else if (role == ROLE_FOLLOWER) {
            // Configure as Listener
            commandQueue = xQueueCreate(5, sizeof(SwarmCommand_t));
            esp_now_register_recv_cb(OnDataRecv);
            Serial.println("Follower Radio Initialized.");
        }

        return true;
    }

    bool broadcast(float heading, float speed, uint8_t command_type, uint8_t flags) {
        if (currentRole != ROLE_PRIMARY) return false; // Safety check

        swarmCmd.network_id = SWARM_NETWORK_ID; 
        swarmCmd.node_id = 0xFF; 
        swarmCmd.command_type = command_type;
        swarmCmd.target_heading = heading;
        swarmCmd.speed = speed;
        swarmCmd.config_flags = flags;
        
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &swarmCmd, sizeof(swarmCmd));
        return (result == ESP_OK);
    }

    bool receive(SwarmCommand_t* outputCmd, uint32_t timeout_ms) {
        if (currentRole != ROLE_FOLLOWER || commandQueue == NULL) return false;

        // Block and wait for a command up to the timeout limit
        if (xQueueReceive(commandQueue, outputCmd, pdMS_TO_TICKS(timeout_ms)) == pdPASS) {
            return true; // We got a new command
        }
        return false; // Watchdog timer expired
    }
}