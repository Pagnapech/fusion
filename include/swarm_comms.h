#ifndef SWARM_COMMS_H
#define SWARM_COMMS_H

#include <Arduino.h>
#include <esp_now.h>

// ==========================================
// SWARM CONFIGURATION ZONE
// ==========================================
#define SWARM_WIFI_CHANNEL 4     // All nodes must match
#define SWARM_NETWORK_ID   0x4A  // Swarm identifier

// ==========================================
// ROLE DEFINITIONS
// ==========================================
enum SwarmRole {
    ROLE_PRIMARY,
    ROLE_FOLLOWER
};

// ==========================================
// DATA PAYLOAD
// ==========================================
typedef struct __attribute__((packed)) {
    uint8_t  network_id;     // Filters out other swarms
    uint8_t  node_id;        // 0xFF for broadcast
    uint8_t  command_type;   // 0x01: Drive, etc.
    float    target_heading; // Desired vector
    float    speed;          // 0.0 to 1.0 (percentage)
    uint8_t  config_flags;   // Bitmask
    uint16_t checksum;       // Data integrity
} SwarmCommand_t;

// ==========================================
// COMMUNICATION API
// ==========================================
namespace SwarmComms {

    // Initializes the radio based on the assigned role
    bool init(SwarmRole role);

    // [PRIMARY ONLY] Broadcasts a command to the swarm
    bool broadcast(float heading, float speed, uint8_t command_type, uint8_t flags);

    // [FOLLOWER ONLY] Pulls the latest command from the radio queue.
    // Returns 'true' if data was received, 'false' if the timeout was reached.
    bool receive(SwarmCommand_t* outputCmd, uint32_t timeout_ms);
}

#endif