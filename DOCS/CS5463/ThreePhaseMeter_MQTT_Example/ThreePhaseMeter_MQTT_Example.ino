// ThreePhaseMeter_MQTT_Example.ino
// Example sketch demonstrating the use of the ThreePhaseMeter library
// Reads data from three CS5463 sensors, checks alarms, and publishes
// data for each phase to separate MQTT topics in JSON format.

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> // Required by the library's getJsonPayload
#include "ThreePhaseMeter.h" // Include the library header

// --- Configuration ---

// Wi-Fi Credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT Broker Configuration
const char* mqtt_server = "YOUR_MQTT_BROKER_IP"; // Replace with your MQTT broker IP or hostname
const int mqtt_port = 1883;
const char* mqtt_user = "YOUR_MQTT_USER"; // Optional
const char* mqtt_password = "YOUR_MQTT_PASSWORD"; // Optional
const char* mqtt_base_topic = "powermeter/trifasico"; // Base topic for publishing
const char* mqtt_client_id = "esp32-powermeter-1";

// ESP32 Pin Configuration for CS5463 Chip Selects
const uint8_t CS_PIN_A = 15; // GPIO15 for Phase A CS
const uint8_t CS_PIN_B = 4;  // GPIO4 for Phase B CS
const uint8_t CS_PIN_C = 5;  // GPIO5 for Phase C CS

// Measurement & Publishing Interval
const unsigned long publishInterval = 1000; // Publish data every 1000 ms (1 second)
unsigned long lastPublishTime = 0;

// Alarm Configuration
const float ALARM_V_MIN = 105.0; // Volts
const float ALARM_V_MAX = 135.0; // Volts
const float ALARM_I_MAX = 70.0;  // Amps
const float ALARM_V_LOSS = 50.0; // Volts (Threshold for phase loss)
const unsigned long ALARM_DURATION = 5000; // Milliseconds (Must persist for 5s to trigger)

// --- Global Objects ---

WiFiClient espClient;
PubSubClient mqttClient(espClient);
ThreePhaseMeter meter(CS_PIN_A, CS_PIN_B, CS_PIN_C); // Instantiate the meter

// Buffer for JSON payload
char jsonBuffer[512];

// --- Function Declarations ---
void setup_wifi();
void reconnect_mqtt();
void mqtt_callback(char* topic, byte* payload, unsigned int length);

// --- Arduino Setup ---
void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10); // Wait for serial connection
    }
    Serial.println("\nStarting Three Phase Power Meter Example...");

    // Initialize SPI and the ThreePhaseMeter library
    // Assumes default SPI pins for VSPI on ESP32 (MOSI=23, MISO=19, SCLK=18)
    // You might need to explicitly begin SPI if using non-default pins or HSPI
    // SPI.begin(SCK, MISO, MOSI, CS); // Example if needed
    Serial.println("Initializing ThreePhaseMeter library...");
    if (!meter.begin()) { // Uses default SPI settings
        Serial.println("Failed to initialize one or more CS5463 sensors!");
        Serial.println("Check wiring and SPI configuration.");
        while (1) delay(1000); // Halt execution
    }
    Serial.println("CS5463 Sensors initialized successfully.");

    // Configure Alarms
    meter.configureAlarms(ALARM_V_MIN, ALARM_V_MAX, ALARM_I_MAX, ALARM_V_LOSS, ALARM_DURATION);
    Serial.println("Alarms configured.");

    // Optional: Apply system-level calibration factors if known
    // Example: meter.configurePhaseCalibration(PHASE_A, 1.01, 0.99, 1.0, 0.0);

    // Optional: Set CS5463 internal settings (e.g., PGA gain if using 50x)
    // meter.setGainPGAAll(true); // Set all sensors to 50x gain

    // Connect to Wi-Fi
    setup_wifi();

    // Configure MQTT
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqtt_callback); // Set callback function for incoming messages

    Serial.println("Setup complete.");
}

// --- Arduino Loop ---
void loop() {
    // Maintain MQTT connection
    if (!mqttClient.connected()) {
        reconnect_mqtt();
    }
    mqttClient.loop(); // Allow the MQTT client to process incoming messages and maintain connection

    // Check if it's time to read and publish data
    unsigned long currentTime = millis();
    if (currentTime - lastPublishTime >= publishInterval) {
        lastPublishTime = currentTime;

        Serial.println("\nReading sensor data...");
        if (!meter.readAllPhases()) {
            Serial.println("Error reading sensor data!");
            return; // Skip this cycle if reading failed
        }

        Serial.println("Checking alarms...");
        meter.checkAlarms();

        // Publish data for each phase
        for (int i = 0; i < 3; ++i) {
            Phase currentPhase = (Phase)i;
            String phaseStr = "";
            switch (currentPhase) {
                case PHASE_A: phaseStr = "phaseA"; break;
                case PHASE_B: phaseStr = "phaseB"; break;
                case PHASE_C: phaseStr = "phaseC"; break;
            }

            // Construct topic
            String topic = String(mqtt_base_topic) + "/" + phaseStr;

            // Generate JSON payload
            size_t payloadSize = meter.getJsonPayload(currentPhase, jsonBuffer, sizeof(jsonBuffer));

            if (payloadSize > 0) {
                Serial.printf("Publishing to %s: %s\n", topic.c_str(), jsonBuffer);
                // Publish data
                if (!mqttClient.publish(topic.c_str(), jsonBuffer, payloadSize)) {
                    Serial.println("MQTT Publish failed!");
                }
            } else {
                Serial.printf("Failed to generate JSON payload for %s\n", phaseStr.c_str());
            }
        }
    }
}

// --- Helper Functions ---

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        attempts++;
        if (attempts > 20) { // Timeout after 10 seconds
            Serial.println("\nFailed to connect to WiFi!");
            // Handle failure (e.g., restart, enter config mode)
            ESP.restart();
        }
    }

    randomSeed(micros());
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void reconnect_mqtt() {
    // Loop until we're reconnected
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        // Provide credentials if configured
        bool connected;
        if (strlen(mqtt_user) > 0) {
             connected = mqttClient.connect(mqtt_client_id, mqtt_user, mqtt_password);
        } else {
             connected = mqttClient.connect(mqtt_client_id);
        }

        if (connected) {
            Serial.println("connected");
            // Once connected, publish an announcement...
            // mqttClient.publish("powermeter/status", "connected");
            // ... and resubscribe to any topics if needed
            // mqttClient.subscribe("some/topic");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

// MQTT Callback function (handles incoming messages)
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    payload[length] = '\0'; // Null-terminate the payload
    Serial.println((char*)payload);

    // Add code here to handle incoming commands if needed
    // Example: if (strcmp(topic, "powermeter/command") == 0) { ... }
}

