#include <Arduino.h>
#include <LibAPRS.h>

// --- Hardware Pins ---
#define PTT_PIN 4       // Base of NPN transistor
#define AUDIO_PIN 5     // PWM to RC Low-Pass Filter (Note: LibAPRS pin mapping is usually configured in its library header)

// --- AX.25 Headers ---
const char* MY_CALLSIGN = "KE2DJD"; 
const int MY_SSID = 1;
const char* DEST_CALLSIGN = "APRS";
const char* PATH1 = "WIDE1";
const char* PATH2 = "WIDE2";

// --- FreeRTOS ---
TaskHandle_t aprsTxTaskHandle = NULL;

struct APRSData {
    float latitude;
    float longitude;
    String comment;
};

// ==============================================================================
// 1. Position Formatting
// ==============================================================================
void formatAPRSPosition(float lat, float lon, String comment, char* buffer) {
    int latDeg = (int)abs(lat);
    float latMin = (abs(lat) - latDeg) * 60.0;
    char latDir = (lat >= 0) ? 'N' : 'S';

    int lonDeg = (int)abs(lon);
    float lonMin = (abs(lon) - lonDeg) * 60.0;
    char lonDir = (lon >= 0) ? 'E' : 'W';

    // Output: !DDMM.mmN/DDDMM.mmW-[Comment]
    sprintf(buffer, "!%02d%05.2f%c/%03d%05.2f%c-%s", 
            latDeg, latMin, latDir, 
            lonDeg, lonMin, lonDir, 
            comment.c_str());
}

// ==============================================================================
// 2. FreeRTOS TX Task
// ==============================================================================
void aprsTxTask(void *parameter) {
    APRSData* data = (APRSData*)parameter;
    char payload[100];
    
    formatAPRSPosition(data->latitude, data->longitude, data->comment, payload);

    // Key radio and wait for relays/squelch
    digitalWrite(PTT_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(400)); 

    // Generate AFSK tones
    APRS_sendFrame((char*)payload, strlen(payload));

    // Wait for the final audio buffer to clear before dropping PTT
    vTaskDelay(pdMS_TO_TICKS(150)); 
    digitalWrite(PTT_PIN, LOW);

    delete data;
    vTaskDelete(NULL);
}

// ==============================================================================
// 3. Main TX Function Trigger
// ==============================================================================
void transmitAPRSPacket(float lat, float lon, String comment) {
    APRSData* txData = new APRSData;
    txData->latitude = lat;
    txData->longitude = lon;
    txData->comment = comment;

    // Isolate AFSK generation to Core 1
    xTaskCreatePinnedToCore(
        aprsTxTask, "APRS_TX", 4096, (void*)txData, 1, &aprsTxTaskHandle, 1
    );
}

// ==============================================================================
// Setup & Loop
// ==============================================================================
void setup() {
    Serial.begin(115200);

    pinMode(PTT_PIN, OUTPUT);
    digitalWrite(PTT_PIN, LOW);

    // Initialize LibAPRS Headers
    APRS_init(ADC_REF_3V3, 0); 
    APRS_setCallsign((char*)MY_CALLSIGN, MY_SSID);
    APRS_setDestination((char*)DEST_CALLSIGN, 0);
    
    // Set WIDE1-1, WIDE2-1 Path
    APRS_setPath1((char*)PATH1, 1);
    APRS_setPath2((char*)PATH2, 1);

    // Fire test packet on boot (approx. West Lafayette coordinates)
    transmitAPRSPacket(40.4259, -86.9081, "Testing Node");
}

void loop() {
    // Main loop stays empty/handles I2C sensors on Core 0
    vTaskDelay(pdMS_TO_TICKS(1000));
}