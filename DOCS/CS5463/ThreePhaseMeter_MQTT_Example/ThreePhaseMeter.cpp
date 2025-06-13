// ThreePhaseMeter.cpp - Arduino library implementation for managing a three-phase energy meter

#include "ThreePhaseMeter.h"
#include <ArduinoJson.h> // Required for getJsonPayload

// Constructor
ThreePhaseMeter::ThreePhaseMeter(uint8_t cs_pin_A, uint8_t cs_pin_B, uint8_t cs_pin_C)
    : _sensorA(cs_pin_A),
      _sensorB(cs_pin_B),
      _sensorC(cs_pin_C),
      _alarm_v_min(105.0f),
      _alarm_v_max(135.0f),
      _alarm_i_max(70.0f),
      _alarm_v_loss_threshold(50.0f),
      _alarm_min_duration_ms(5000),
      _last_read_time(0)
{
    // Initialize calibration gains to 1.0 (no scaling)
    for (int i = 0; i < 3; ++i) {
        _cal_v_gain[i] = 1.0f;
        _cal_i_gain[i] = 1.0f;
        _cal_p_gain[i] = 1.0f;
        _last_zero_cross_time[i] = 0;
        _zero_cross_count[i] = 0;
        _last_status[i] = 0;
    }
}

// Initialization
bool ThreePhaseMeter::begin(SPIClass &spi_bus, uint32_t spi_freq, uint8_t spi_mode) {
    bool success = true;
    success &= _sensorA.begin(spi_bus, spi_freq, spi_mode);
    success &= _sensorB.begin(spi_bus, spi_freq, spi_mode);
    success &= _sensorC.begin(spi_bus, spi_freq, spi_mode);
    _last_read_time = millis();
    // Initialize alarm timestamps
    for (int i = 0; i < 3; ++i) {
        _phaseData[i].alarms.lastVoltageOkTime = _last_read_time;
        _phaseData[i].alarms.lastCurrentOkTime = _last_read_time;
        _phaseData[i].alarms.lastPhaseOkTime = _last_read_time;
    }
    return success;
}

// --- Configuration Methods ---

void ThreePhaseMeter::configureAlarms(float v_min, float v_max, float i_max, float v_loss_threshold, unsigned long min_duration_ms) {
    _alarm_v_min = v_min;
    _alarm_v_max = v_max;
    _alarm_i_max = i_max;
    _alarm_v_loss_threshold = v_loss_threshold;
    _alarm_min_duration_ms = min_duration_ms;
}

// Configure software scaling factors applied AFTER reading from CS5463
// Assumes CS5463 internal calibration handles fine-tuning, these are for system-level scaling (e.g., sensor ratios)
void ThreePhaseMeter::configurePhaseCalibration(Phase phase, float v_gain, float i_gain, float p_gain, float phase_corr) {
    if (phase >= PHASE_A && phase <= PHASE_C) {
        _cal_v_gain[phase] = v_gain;
        _cal_i_gain[phase] = i_gain;
        _cal_p_gain[phase] = p_gain;
        // phase_corr needs specific implementation - potentially using CS5463 phase comp register
        // For now, use the sensor object directly:
        // sensorForPhase(phase)->setPhaseCompensation(value);
    }
}

void ThreePhaseMeter::setCycleCountAll(uint16_t n_cycles) {
    _sensorA.setCycleCount(n_cycles);
    _sensorB.setCycleCount(n_cycles);
    _sensorC.setCycleCount(n_cycles);
}

void ThreePhaseMeter::setGainPGAAll(bool gain50x) {
    _sensorA.setGainPGA(gain50x);
    _sensorB.setGainPGA(gain50x);
    _sensorC.setGainPGA(gain50x);
}

// --- Core Operations ---

bool ThreePhaseMeter::readAllPhases() {
    bool success = true;
    success &= _sensorA.readMeasurements(_phaseData[PHASE_A].measurements);
    success &= _sensorB.readMeasurements(_phaseData[PHASE_B].measurements);
    success &= _sensorC.readMeasurements(_phaseData[PHASE_C].measurements);
    _last_read_time = millis();

    // Apply software scaling factors
    for (int i = 0; i < 3; ++i) {
        _phaseData[i].measurements.rmsVoltage *= _cal_v_gain[i];
        _phaseData[i].measurements.rmsCurrent *= _cal_i_gain[i];
        _phaseData[i].measurements.activePower *= _cal_p_gain[i];
        // Scale other power values if needed (Apparent, Reactive etc.)
        // Note: Scaling P, V, I might require recalculating S and PF if not scaled proportionally
        // For simplicity, assume P gain scales P, Q, S similarly or use dedicated gains.
        _phaseData[i].measurements.apparentPower *= _cal_v_gain[i] * _cal_i_gain[i]; // S = V*I
        _phaseData[i].measurements.reactivePowerAvg *= _cal_p_gain[i]; // Assuming similar scaling needed
        _phaseData[i].measurements.reactivePowerTrig *= _cal_p_gain[i];

        // Recalculate PF based on scaled values if necessary, or scale PF directly if appropriate
        if (_phaseData[i].measurements.apparentPower > 1e-6) { // Avoid division by zero
             _phaseData[i].measurements.powerFactor = _phaseData[i].measurements.activePower / _phaseData[i].measurements.apparentPower;
        } else {
             _phaseData[i].measurements.powerFactor = 1.0; // Or 0.0, depending on convention for zero power
        }
        // Clamp PF
        if (_phaseData[i].measurements.powerFactor > 1.0) _phaseData[i].measurements.powerFactor = 1.0;
        if (_phaseData[i].measurements.powerFactor < -1.0) _phaseData[i].measurements.powerFactor = -1.0;

        // Update frequency calculation (placeholder)
        _phaseData[i].frequency = calculateFrequency((Phase)i);
    }

    return success;
}

void ThreePhaseMeter::checkAlarms() {
    unsigned long now = _last_read_time; // Use the time the readings were taken

    for (int i = 0; i < 3; ++i) {
        Phase phase = (Phase)i;
        float vrms = _phaseData[phase].measurements.rmsVoltage;
        float irms = _phaseData[phase].measurements.rmsCurrent;

        // --- Phase Loss Check ---
        if (vrms >= _alarm_v_loss_threshold) {
            _phaseData[phase].alarms.lastPhaseOkTime = now;
            _phaseData[phase].alarms.phaseLoss = false;
        } else {
            if (now - _phaseData[phase].alarms.lastPhaseOkTime >= _alarm_min_duration_ms) {
                _phaseData[phase].alarms.phaseLoss = true;
            }
            // If phase loss is active, other voltage/current alarms might be irrelevant
        }

        // --- Voltage Range Check (only if phase is not lost) ---
        if (!_phaseData[phase].alarms.phaseLoss) {
            if (vrms >= _alarm_v_min && vrms <= _alarm_v_max) {
                _phaseData[phase].alarms.lastVoltageOkTime = now;
                _phaseData[phase].alarms.voltageLow = false;
                _phaseData[phase].alarms.voltageHigh = false;
            } else {
                if (now - _phaseData[phase].alarms.lastVoltageOkTime >= _alarm_min_duration_ms) {
                    if (vrms < _alarm_v_min) {
                        _phaseData[phase].alarms.voltageLow = true;
                        _phaseData[phase].alarms.voltageHigh = false;
                    } else { // vrms > _alarm_v_max
                        _phaseData[phase].alarms.voltageLow = false;
                        _phaseData[phase].alarms.voltageHigh = true;
                    }
                }
            }
        } else {
             // Reset voltage alarms if phase is lost
             _phaseData[phase].alarms.voltageLow = false;
             _phaseData[phase].alarms.voltageHigh = false;
             _phaseData[phase].alarms.lastVoltageOkTime = now; // Reset timer
        }

        // --- Current High Check (only if phase is not lost) ---
         if (!_phaseData[phase].alarms.phaseLoss) {
            if (irms <= _alarm_i_max) {
                _phaseData[phase].alarms.lastCurrentOkTime = now;
                _phaseData[phase].alarms.currentHigh = false;
            } else {
                if (now - _phaseData[phase].alarms.lastCurrentOkTime >= _alarm_min_duration_ms) {
                    _phaseData[phase].alarms.currentHigh = true;
                }
            }
        } else {
            // Reset current alarm if phase is lost
            _phaseData[phase].alarms.currentHigh = false;
            _phaseData[phase].alarms.lastCurrentOkTime = now; // Reset timer
        }
    }
}

// Basic frequency calculation placeholder - Needs refinement
// Uses zero crossings from status register. Requires careful handling.
float ThreePhaseMeter::calculateFrequency(Phase phase) {
    CS5463_Sensor* sensor = sensorForPhase(phase);
    if (!sensor) return 0.0f;

    uint32_t current_status = _phaseData[phase].measurements.statusRegister;
    bool crossed_zero = false;

    // Check for positive or negative voltage zero crossing bits
    if ((current_status & CS5463_STATUS_VPZC) && !(_last_status[phase] & CS5463_STATUS_VPZC)) {
        crossed_zero = true;
    }
    // Add check for VNZC if needed for higher resolution
    // else if ((current_status & CS5463_STATUS_VNZC) && !(_last_status[phase] & CS5463_STATUS_VNZC)) {
    //    crossed_zero = true;
    // }

    _last_status[phase] = current_status;

    if (crossed_zero) {
        unsigned long now = _last_read_time;
        if (_last_zero_cross_time[phase] != 0) {
            unsigned long period_micros = now - _last_zero_cross_time[phase];
            // This detects half-cycles if only using VPZC. Need to accumulate full cycles.
            // A more robust method would count N crossings and measure total time.
            // Simple placeholder: assume period_micros is roughly one cycle time for now.
            if (period_micros > 10000 && period_micros < 25000) { // Plausible range for 40-100 Hz
                 _phaseData[phase].frequency = 1000000.0f / (float)period_micros;
            }
        }
        _last_zero_cross_time[phase] = now;
    }
    // Return the last calculated frequency
    return _phaseData[phase].frequency;
}

// --- Data Access ---

CS5463_Measurements ThreePhaseMeter::getMeasurements(Phase phase) {
    if (phase >= PHASE_A && phase <= PHASE_C) {
        return _phaseData[phase].measurements;
    }
    return CS5463_Measurements(); // Return default/empty struct on error
}

PhaseAlarmStatus ThreePhaseMeter::getAlarmStatus(Phase phase) {
    if (phase >= PHASE_A && phase <= PHASE_C) {
        return _phaseData[phase].alarms;
    }
    return PhaseAlarmStatus(); // Return default/empty struct on error
}

PhaseData ThreePhaseMeter::getPhaseData(Phase phase) {
     if (phase >= PHASE_A && phase <= PHASE_C) {
        return _phaseData[phase];
    }
    return PhaseData(); // Return default/empty struct on error
}

float ThreePhaseMeter::getFrequency(Phase phase) {
    if (phase >= PHASE_A && phase <= PHASE_C) {
        return _phaseData[phase].frequency;
    }
    return 0.0f;
}

CS5463_Sensor* ThreePhaseMeter::getSensor(Phase phase) {
    return sensorForPhase(phase);
}

// --- MQTT Payload Generation ---

// Populates the provided buffer with a JSON string.
// Requires ArduinoJson library to be included in the main sketch.
size_t ThreePhaseMeter::getJsonPayload(Phase phase, char* buffer, size_t bufferSize) {
    if (phase < PHASE_A || phase > PHASE_C) {
        return 0; // Invalid phase
    }

    PhaseData data = _phaseData[phase];

    // Use StaticJsonDocument for predictable memory usage on ESP32
    // Adjust size as needed based on floating point precision and field names
    StaticJsonDocument<512> doc;

    // Add measurements with controlled precision
    doc["v"] = serialized(data.measurements.rmsVoltage, 1); // 1 decimal place
    doc["i"] = serialized(data.measurements.rmsCurrent, 2); // 2 decimal places
    doc["p"] = serialized(data.measurements.activePower, 1);
    doc["q"] = serialized(data.measurements.reactivePowerAvg, 1); // Using Avg Q
    doc["s"] = serialized(data.measurements.apparentPower, 1);
    doc["pf"] = serialized(data.measurements.powerFactor, 2);
    doc["freq"] = serialized(data.frequency, 1);
    doc["temp"] = serialized(data.measurements.temperature, 1);
    doc["vpeak"] = serialized(data.measurements.peakVoltage, 1);
    doc["ipeak"] = serialized(data.measurements.peakCurrent, 1);

    // Add alarm statuses
    doc["alm_v_low"] = data.alarms.voltageLow;
    doc["alm_v_high"] = data.alarms.voltageHigh;
    doc["alm_i_high"] = data.alarms.currentHigh;
    doc["alm_p_loss"] = data.alarms.phaseLoss;

    // Serialize JSON to buffer
    size_t jsonSize = serializeJson(doc, buffer, bufferSize);

    return (jsonSize > 0 && jsonSize < bufferSize) ? jsonSize : 0;
}

// --- Private Helper Functions ---

CS5463_Sensor* ThreePhaseMeter::sensorForPhase(Phase phase) {
    switch (phase) {
        case PHASE_A: return &_sensorA;
        case PHASE_B: return &_sensorB;
        case PHASE_C: return &_sensorC;
        default: return nullptr;
    }
}

