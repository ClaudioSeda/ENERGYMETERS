// ThreePhaseMeter.h - Arduino library header for managing a three-phase energy meter using CS5463 sensors

#ifndef ThreePhaseMeter_h
#define ThreePhaseMeter_h

#include "Arduino.h"
#include "CS5463_Sensor.h"

// Define phase identifiers
enum Phase { PHASE_A = 0, PHASE_B = 1, PHASE_C = 2 };

// Structure to hold alarm status for one phase
struct PhaseAlarmStatus {
    bool voltageLow = false;
    bool voltageHigh = false;
    bool currentHigh = false;
    bool phaseLoss = false;
    unsigned long lastVoltageOkTime = 0;
    unsigned long lastCurrentOkTime = 0;
    unsigned long lastPhaseOkTime = 0;
};

// Structure to hold complete data for one phase (measurements + alarms)
struct PhaseData {
    CS5463_Measurements measurements;
    PhaseAlarmStatus alarms;
    float frequency = 0.0f; // Calculated frequency
};

class ThreePhaseMeter {
public:
    ThreePhaseMeter(uint8_t cs_pin_A, uint8_t cs_pin_B, uint8_t cs_pin_C);

    bool begin(SPIClass &spi_bus = SPI, uint32_t spi_freq = 1000000, uint8_t spi_mode = SPI_MODE0);

    // Configuration
    void configureAlarms(float v_min, float v_max, float i_max, float v_loss_threshold, unsigned long min_duration_ms = 5000);
    void configurePhaseCalibration(Phase phase, float v_gain = 1.0, float i_gain = 1.0, float p_gain = 1.0, float phase_corr = 0.0);
    // Note: Gains here are scaling factors applied *after* reading scaled values from CS5463.
    // Phase correction might involve adjusting CS5463 phase compensation or applying correction in software.
    // For simplicity, we might just use CS5463's internal calibration registers via CS5463_Sensor methods.
    void setCycleCountAll(uint16_t n_cycles);
    void setGainPGAAll(bool gain50x);

    // Core operations
    bool readAllPhases();
    void checkAlarms();
    float calculateFrequency(Phase phase); // Basic implementation placeholder

    // Data Access
    CS5463_Measurements getMeasurements(Phase phase);
    PhaseAlarmStatus getAlarmStatus(Phase phase);
    PhaseData getPhaseData(Phase phase);
    float getFrequency(Phase phase);

    // Get individual sensor objects for advanced configuration/calibration
    CS5463_Sensor* getSensor(Phase phase);

    // MQTT Payload Generation (Helper - requires ArduinoJson in sketch)
    // Returns estimated size needed for JSON buffer
    size_t getJsonPayload(Phase phase, char* buffer, size_t bufferSize);
    // Example JSON structure:
    // {"v":120.1, "i":15.2, "p":1800.5, "q":300.1, "s":1825.0, "pf":0.98,
    //  "freq":60.0, "temp":25.5, "vpeak":170.0, "ipeak":25.0,
    //  "alm_v_low":false, "alm_v_high":false, "alm_i_high":false, "alm_p_loss":false}

private:
    CS5463_Sensor _sensorA;
    CS5463_Sensor _sensorB;
    CS5463_Sensor _sensorC;

    // Alarm settings
    float _alarm_v_min;
    float _alarm_v_max;
    float _alarm_i_max;
    float _alarm_v_loss_threshold;
    unsigned long _alarm_min_duration_ms;

    // Phase-specific calibration/scaling factors (applied in software)
    float _cal_v_gain[3];
    float _cal_i_gain[3];
    float _cal_p_gain[3]; // For active power scaling
    // Add more if needed (e.g., Q gain, S gain, phase correction)

    // Internal state
    PhaseData _phaseData[3];
    unsigned long _last_read_time;

    // Frequency calculation helpers
    unsigned long _last_zero_cross_time[3];
    uint8_t _zero_cross_count[3];
    uint32_t _last_status[3];

    // Helper to get sensor reference by phase
    CS5463_Sensor* sensorForPhase(Phase phase);
};

#endif

