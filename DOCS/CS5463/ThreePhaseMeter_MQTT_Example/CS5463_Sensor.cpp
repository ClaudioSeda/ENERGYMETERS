// CS5463_Sensor.cpp - Arduino library implementation for interfacing with the CS5463 Power/Energy IC

#include "CS5463_Sensor.h"

// Constructor
CS5463_Sensor::CS5463_Sensor(uint8_t cs_pin) : _cs_pin(cs_pin), _spi(nullptr), _current_page(0), _gain_pga_50x(false) {
    // Initialize calibration registers to default/reset values (mostly 0 or specific defaults)
    _reg_idcoff = 0x000000;
    _reg_vdcoff = 0x000000;
    _reg_iacoff = 0x000000;
    _reg_vacoff = 0x000000;
    _reg_poff = 0x000000;
    _reg_ign = 0x400000; // Default gain = 1.0
    _reg_vgn = 0x400000; // Default gain = 1.0
    _reg_tgain = 0x11E24A; // Default T Gain
    _reg_toff = 0xD1AE14; // Default T Offset
}

// Initialization
bool CS5463_Sensor::begin(SPIClass &spi_bus, uint32_t spi_freq, uint8_t spi_mode) {
    _spi = &spi_bus;
    _spi_settings = SPISettings(spi_freq, MSBFIRST, spi_mode);

    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH); // Deselect chip initially

    _spi->begin(); // Initialize SPI bus if not already done

    reset(); // Reset the chip
    delay(10); // Allow time for reset and stabilization

    // Basic configuration after reset
    selectPage(0);
    // Enable Write Any Register temporarily to allow config changes
    uint32_t config = readRegister(CS5463_REG_CONFIG);
    writeRegister(CS5463_REG_CONFIG, config | CS5463_CONFIG_EWA);
    delay(1);

    // Set default cycle count (e.g., 4000 for ~1Hz update at 4.096MHz MCLK, K=1)
    setCycleCount(4000);
    // Set default mode (HPFs enabled, IIR enabled)
    setFilters(true, true, true);
    // Set default PGA gain (10x)
    setGainPGA(false);

    // Disable Write Any Register
    config = readRegister(CS5463_REG_CONFIG);
    writeRegister(CS5463_REG_CONFIG, config & ~CS5463_CONFIG_EWA);
    delay(1);

    startConversions(); // Start continuous conversions

    return isConnected(); // Check if chip responds after setup
}

// Reset the chip
void CS5463_Sensor::reset() {
    sendCommand(CS5463_CMD_SOFT_RESET);
    delay(2); // Datasheet recommends >1ms delay after reset
    _current_page = 0; // Page resets to 0
}

// Check if chip is connected (basic check by reading a known register)
bool CS5463_Sensor::isConnected() {
    selectPage(0);
    uint32_t config = readRegister(CS5463_REG_CONFIG);
    // Check for a plausible (non-zero, non-0xFFFFFF) value or specific revision bits
    // A more robust check might involve writing/reading a scratch register if available
    // or checking default values after reset.
    // For now, just check if it's not all 0s or all 1s.
    return (config != 0x000000 && config != 0xFFFFFF);
}

// --- Configuration Methods ---

void CS5463_Sensor::setGainPGA(bool gain50x) {
    selectPage(0);
    uint32_t config = readRegister(CS5463_REG_CONFIG);
    writeRegister(CS5463_REG_CONFIG, config | CS5463_CONFIG_EWA); // Enable write
    if (gain50x) {
        config |= CS5463_CONFIG_IGAIN;
        _gain_pga_50x = true;
    } else {
        config &= ~CS5463_CONFIG_IGAIN;
        _gain_pga_50x = false;
    }
    writeRegister(CS5463_REG_CONFIG, config); // Write new gain setting
    writeRegister(CS5463_REG_CONFIG, config & ~CS5463_CONFIG_EWA); // Disable write
}

void CS5463_Sensor::setClockDivider(uint8_t k_val) {
    if (k_val > 7) k_val = 7; // Limit K value
    selectPage(0);
    uint32_t config = readRegister(CS5463_REG_CONFIG);
    writeRegister(CS5463_REG_CONFIG, config | CS5463_CONFIG_EWA);
    config &= ~CS5463_CONFIG_K_MASK; // Clear existing K bits
    config |= (k_val & 0x07); // Set new K bits
    writeRegister(CS5463_REG_CONFIG, config);
    writeRegister(CS5463_REG_CONFIG, config & ~CS5463_CONFIG_EWA);
}

void CS5463_Sensor::setCycleCount(uint16_t n_cycles) {
    selectPage(0);
    // Cycle count is N-1 according to some interpretations, but datasheet example uses N.
    // Let's assume the register takes N directly.
    writeRegister(CS5463_REG_CYCLECOUNT, (uint32_t)n_cycles);
}

void CS5463_Sensor::setFilters(bool enableIIR, bool enableVHPF, bool enableIHPF) {
    selectPage(0);
    uint32_t mode = readRegister(CS5463_REG_MODE);
    if (enableIIR) mode |= CS5463_MODE_IIR; else mode &= ~CS5463_MODE_IIR;
    if (enableVHPF) mode |= CS5463_MODE_VHPF; else mode &= ~CS5463_MODE_VHPF;
    if (enableIHPF) mode |= CS5463_MODE_IHPF; else mode &= ~CS5463_MODE_IHPF;
    writeRegister(CS5463_REG_MODE, mode);
}

void CS5463_Sensor::setPhaseCompensation(int8_t pc_value) {
    // pc_value range is -64 to +63, maps to register bits 6:0
    // The register field PC[6:0] is bits 13:7 of the Config register
    uint8_t reg_val = (uint8_t)pc_value & 0x7F; // Ensure it fits in 7 bits
    selectPage(0);
    uint32_t config = readRegister(CS5463_REG_CONFIG);
    writeRegister(CS5463_REG_CONFIG, config | CS5463_CONFIG_EWA);
    config &= ~CS5463_CONFIG_PC_MASK; // Clear existing PC bits
    config |= ((uint32_t)reg_val << 7); // Set new PC bits
    writeRegister(CS5463_REG_CONFIG, config);
    writeRegister(CS5463_REG_CONFIG, config & ~CS5463_CONFIG_EWA);
}

void CS5463_Sensor::setEpsilon(float epsilon) {
    // Epsilon is used for Q, PF calculations. Range 0 to <1.
    // Register format is unsigned fractional.
    selectPage(0);
    writeRegister(CS5463_REG_EPSILON, convertFloatToRegister(epsilon, false));
}

void CS5463_Sensor::setPulseRateE(uint32_t rate) {
    selectPage(0);
    writeRegister(CS5463_REG_PULSERATEE, rate);
}

void CS5463_Sensor::setNoLoadThreshold(uint32_t threshold) {
    selectPage(1);
    writeRegister(CS5463_REG_LOADMIN, threshold);
    selectPage(0);
}

// --- Calibration Methods ---

void CS5463_Sensor::setCalibrationOffsets(float i_dc_offset, float v_dc_offset, float i_ac_offset, float v_ac_offset, float p_offset) {
    selectPage(0);
    _reg_idcoff = convertFloatToRegister(i_dc_offset);
    _reg_vdcoff = convertFloatToRegister(v_dc_offset);
    _reg_iacoff = convertFloatToRegister(i_ac_offset);
    _reg_vacoff = convertFloatToRegister(v_ac_offset);
    _reg_poff = convertFloatToRegister(p_offset);

    writeRegister(CS5463_REG_IDCOFF, _reg_idcoff);
    writeRegister(CS5463_REG_VDCoff, _reg_vdcoff);
    writeRegister(CS5463_REG_IACOFF, _reg_iacoff);
    writeRegister(CS5463_REG_VACOFF, _reg_vacoff);
    writeRegister(CS5463_REG_POFF, _reg_poff);
}

void CS5463_Sensor::setCalibrationGains(float i_gain, float v_gain) {
    selectPage(0);
    // Gain registers are typically 1.0 + delta, format 0.xxxxxx
    // Datasheet: Gain = 1 + RegVal / 2^23 (for positive RegVal)
    // Or Gain = 1 + RegVal / 2^22 (for negative RegVal - two's complement)
    // Let's assume we are setting the register directly based on a desired gain factor.
    // The register value represents (Gain - 1.0) scaled.
    // For Gain = 1.0, register = 0x400000 (which is 0.5 in unsigned fractional)
    // This needs careful verification with the datasheet examples.
    // Assuming Gain = RegValue / 0x400000 for now (simple scaling)
    _reg_ign = convertFloatToRegister(i_gain / 2.0, false); // Needs verification
    _reg_vgn = convertFloatToRegister(v_gain / 2.0, false); // Needs verification
    // Let's use the default reset value interpretation: Gain = 1.0 -> 0x400000
    // So, RegValue = Gain * 0x400000
    _reg_ign = (uint32_t)(i_gain * 0x400000);
    _reg_vgn = (uint32_t)(v_gain * 0x400000);

    writeRegister(CS5463_REG_IGN, _reg_ign);
    writeRegister(CS5463_REG_VGN, _reg_vgn);
}

void CS5463_Sensor::setTemperatureCalibration(float t_gain, float t_offset) {
    selectPage(1);
    _reg_tgain = convertFloatToRegister(t_gain);
    _reg_toff = convertFloatToRegister(t_offset);
    writeRegister(CS5463_REG_TGAIN, _reg_tgain);
    writeRegister(CS5463_REG_TOFF, _reg_toff);
    selectPage(0);
}

bool CS5463_Sensor::performCalibration(uint8_t cal_command) {
    // Ensure conversions are running for AC calibration
    // Ensure specific inputs (zero or full-scale) are applied externally
    selectPage(0);
    uint32_t status = readRegister(CS5463_REG_STATUS);
    // Clear DRDY bit if set, by reading result registers (or writing 1 to clear? check datasheet)
    // Datasheet: Status bits are cleared by reading the Status Register

    sendCommand(cal_command);

    // Wait for DRDY bit to indicate calibration completion
    unsigned long startTime = millis();
    while (!(readRegister(CS5463_REG_STATUS) & CS5463_STATUS_DRDY)) {
        if (millis() - startTime > 5000) { // Timeout after 5 seconds
            return false; // Calibration timed out
        }
        delay(10);
    }
    // Calibration done, new offset/gain values are in the registers
    // Optionally read them back into the _reg_* variables
    return true;
}

// --- Reading Measurements ---

bool CS5463_Sensor::readMeasurements(CS5463_Measurements &meas) {
    selectPage(0);
    uint32_t status = readRegister(CS5463_REG_STATUS);

    // Check if data is ready (optional, depends on application timing)
    // if (!(status & CS5463_STATUS_DRDY)) {
    //     return false; // Data not ready yet
    // }

    // Read all relevant registers in one go (if possible/efficient)
    // For simplicity here, read one by one
    meas.statusRegister = status; // Store the status read at the start
    meas.instantaneousCurrent = getInstantaneousCurrent();
    meas.instantaneousVoltage = getInstantaneousVoltage();
    meas.instantaneousPower = getInstantaneousPower();
    meas.activePower = getActivePower();
    meas.rmsCurrent = getIRMS();
    meas.rmsVoltage = getVRMS();
    meas.apparentPower = getApparentPower();
    meas.reactivePowerAvg = getReactivePowerAvg();
    meas.reactivePowerTrig = getReactivePowerTrig();
    meas.powerFactor = getPowerFactor();
    meas.peakCurrent = getPeakCurrent();
    meas.peakVoltage = getPeakVoltage();
    meas.temperature = getTemperature();
    meas.fundamentalActivePower = getFundamentalActivePower();
    meas.fundamentalReactivePower = getFundamentalReactivePower();
    meas.harmonicActivePower = getHarmonicActivePower();

    // Store locally as well
    _last_measurement = meas;

    // DRDY should be cleared by reading the status register, which we did.
    // If not, clear manually if the datasheet specifies writing 1.

    return true;
}

// Individual measurement getters (read register and convert)
float CS5463_Sensor::getInstantaneousCurrent() { return convertRegisterToFloat(readRegister(CS5463_REG_I)); }
float CS5463_Sensor::getInstantaneousVoltage() { return convertRegisterToFloat(readRegister(CS5463_REG_V)); }
float CS5463_Sensor::getInstantaneousPower() { return convertRegisterToFloat(readRegister(CS5463_REG_P)); }
float CS5463_Sensor::getVRMS() { return convertRegisterToFloat(readRegister(CS5463_REG_VRMS), false); } // Unsigned
float CS5463_Sensor::getIRMS() { return convertRegisterToFloat(readRegister(CS5463_REG_IRMS), false); } // Unsigned
float CS5463_Sensor::getActivePower() { return convertRegisterToFloat(readRegister(CS5463_REG_PACTIVE)); }
float CS5463_Sensor::getReactivePowerAvg() { return convertRegisterToFloat(readRegister(CS5463_REG_QAVG)); }
float CS5463_Sensor::getReactivePowerTrig() { return convertRegisterToFloat(readRegister(CS5463_REG_QTRIG)); }
float CS5463_Sensor::getApparentPower() { return convertRegisterToFloat(readRegister(CS5463_REG_S), false); } // Unsigned
float CS5463_Sensor::getPowerFactor() { return convertRegisterToFloat(readRegister(CS5463_REG_PF)); }
float CS5463_Sensor::getPeakVoltage() { return convertRegisterToFloat(readRegister(CS5463_REG_VPEAK), false); } // Unsigned magnitude
float CS5463_Sensor::getPeakCurrent() { return convertRegisterToFloat(readRegister(CS5463_REG_IPEAK), false); } // Unsigned magnitude
float CS5463_Sensor::getTemperature() { return convertRegisterToFloat(readRegister(CS5463_REG_TEMP)); }
float CS5463_Sensor::getFundamentalActivePower() { return convertRegisterToFloat(readRegister(CS5463_REG_PFUNDAMENTAL)); }
float CS5463_Sensor::getFundamentalReactivePower() { return convertRegisterToFloat(readRegister(CS5463_REG_QFUNDAMENTAL)); }
float CS5463_Sensor::getHarmonicActivePower() { return convertRegisterToFloat(readRegister(CS5463_REG_PHARMONIC)); }

uint32_t CS5463_Sensor::getStatusRegister() {
    return readRegister(CS5463_REG_STATUS);
}

// --- Low-level Access ---

void CS5463_Sensor::selectPage(uint8_t page) {
    if (page == _current_page) return;
    writeRegister(CS5463_REG_PAGE, (uint32_t)page);
    _current_page = page;
}

uint32_t CS5463_Sensor::readRegister(uint8_t address) {
    uint8_t packet[4];
    packet[0] = (address & 0x3F); // Read command (0x00) ORed with address (6 bits)

    digitalWrite(_cs_pin, LOW);
    _spi->beginTransaction(_spi_settings);
    _spi->transfer(packet[0]); // Send read command + address
    // Read 3 bytes (24 bits)
    packet[1] = _spi->transfer(0xFF); // Send dummy byte to clock in data
    packet[2] = _spi->transfer(0xFF);
    packet[3] = _spi->transfer(0xFF);
    _spi->endTransaction();
    digitalWrite(_cs_pin, HIGH);

    uint32_t result = ((uint32_t)packet[1] << 16) | ((uint32_t)packet[2] << 8) | packet[3];
    return result;
}

void CS5463_Sensor::writeRegister(uint8_t address, uint32_t value) {
    uint8_t packet[4];
    packet[0] = CS5463_CMD_WRITE_REG | (address & 0x3F); // Write command (0x40) ORed with address
    packet[1] = (value >> 16) & 0xFF; // High byte
    packet[2] = (value >> 8) & 0xFF;  // Mid byte
    packet[3] = value & 0xFF;         // Low byte

    spiTransaction(packet, 4);
}

void CS5463_Sensor::sendCommand(uint8_t command) {
    uint8_t packet[1];
    packet[0] = command;
    spiTransaction(packet, 1);

    // Some commands require SYNC1 afterwards (e.g., calibration)
    if (command >= 0xC0 && command <= 0xDF) { // Calibration commands
       // sendCommand(CS5463_CMD_SYNC1); // Send SYNC1 (0xFF) - Check if needed implicitly
       // Datasheet implies SYNC1 might be needed *before* some commands, or implicitly handled.
       // Let's assume it's not needed after every command for now.
    }
}

// --- Private Helper Functions ---

void CS5463_Sensor::spiTransaction(uint8_t* data, uint8_t len) {
    digitalWrite(_cs_pin, LOW);
    _spi->beginTransaction(_spi_settings);
    for (uint8_t i = 0; i < len; ++i) {
        _spi->transfer(data[i]);
    }
    _spi->endTransaction();
    digitalWrite(_cs_pin, HIGH);
    delayMicroseconds(5); // Add small delay between transactions if needed
}

// Convert 24-bit register value (fractional two's complement or unsigned) to float (-1.0 to <1.0 or 0 to <1.0)
float CS5463_Sensor::convertRegisterToFloat(uint32_t regValue, bool signedValue) {
    regValue &= 0xFFFFFF; // Ensure only 24 bits

    if (signedValue) {
        // Check if negative (MSB is 1)
        if (regValue & 0x800000) {
            // Negative number: perform two's complement conversion
            // Extend sign to 32 bits first
            int32_t signed_val = regValue | 0xFF000000;
            return (float)signed_val / 8388608.0f; // 2^23
        } else {
            // Positive number
            return (float)regValue / 8388608.0f; // 2^23
        }
    } else {
        // Unsigned number
        return (float)regValue / 16777216.0f; // 2^24
    }
}

// Convert float (-1.0 to <1.0 or 0 to <1.0) to 24-bit register value
uint32_t CS5463_Sensor::convertFloatToRegister(float value, bool signedValue) {
    int32_t regVal;
    if (signedValue) {
        // Clamp value to slightly less than 1.0 and >= -1.0
        if (value >= 1.0) value = 1.0 - 1e-7;
        if (value < -1.0) value = -1.0;
        regVal = (int32_t)(value * 8388608.0f); // 2^23
    } else {
        // Clamp value to slightly less than 1.0 and >= 0
        if (value >= 1.0) value = 1.0 - 1e-7;
        if (value < 0.0) value = 0.0;
        regVal = (int32_t)(value * 16777216.0f); // 2^24
    }
    return (uint32_t)regVal & 0xFFFFFF; // Return lower 24 bits
}

void CS5463_Sensor::startConversions(bool continuous) {
    sendCommand(continuous ? CS5463_CMD_START_CONV_CONT : CS5463_CMD_START_CONV_SINGLE);
}

void CS5463_Sensor::haltConversions() {
    sendCommand(CS5463_CMD_POWERUP_HALT); // Halts conversions but keeps chip powered
}

