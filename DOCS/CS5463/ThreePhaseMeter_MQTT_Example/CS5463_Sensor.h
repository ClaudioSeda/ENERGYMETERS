// CS5463_Sensor.h - Arduino library header for interfacing with the CS5463 Power/Energy IC
// Based on datasheet DS678F4 (Cirrus Logic, Nov 2021)

#ifndef CS5463_Sensor_h
#define CS5463_Sensor_h

#include "Arduino.h"
#include <SPI.h>

// CS5463 Register Addresses (Page 0 by default)
#define CS5463_REG_CONFIG       0x00 // Configuration
#define CS5463_REG_IDCOFF       0x01 // Current DC Offset
#define CS5463_REG_IGN          0x02 // Current Gain
#define CS5463_REG_VDCoff       0x03 // Voltage DC Offset
#define CS5463_REG_VGN          0x04 // Voltage Gain
#define CS5463_REG_CYCLECOUNT   0x05 // Cycle Count
#define CS5463_REG_PULSERATEE   0x06 // Real Power Pulse Rate
#define CS5463_REG_I            0x07 // Instantaneous Current
#define CS5463_REG_V            0x08 // Instantaneous Voltage
#define CS5463_REG_P            0x09 // Instantaneous Power
#define CS5463_REG_PACTIVE      0x0A // Active Power
#define CS5463_REG_IRMS         0x0B // RMS Current
#define CS5463_REG_VRMS         0x0C // RMS Voltage
#define CS5463_REG_EPSILON      0x0D // Line Frequency Epsilon
#define CS5463_REG_POFF         0x0E // Power Offset
#define CS5463_REG_STATUS       0x0F // Status Register
#define CS5463_REG_IACOFF       0x10 // Current AC Offset
#define CS5463_REG_VACOFF       0x11 // Voltage AC Offset
#define CS5463_REG_MODE         0x12 // Operational Mode
#define CS5463_REG_TEMP         0x13 // Temperature
#define CS5463_REG_QAVG         0x14 // Average Reactive Power
#define CS5463_REG_IPEAK        0x15 // Peak Current
#define CS5463_REG_VPEAK        0x16 // Peak Voltage
#define CS5463_REG_QTRIG        0x17 // Reactive Power (Trigonometric)
#define CS5463_REG_PF           0x18 // Power Factor
#define CS5463_REG_S            0x19 // Apparent Power
#define CS5463_REG_CTRL         0x1A // Control Register
#define CS5463_REG_PHARMONIC    0x1B // Harmonic Active Power
#define CS5463_REG_PFUNDAMENTAL 0x1C // Fundamental Active Power
#define CS5463_REG_QFUNDAMENTAL 0x1D // Fundamental Reactive Power
#define CS5463_REG_PAGE         0x1F // Page Select

// Page 1 Registers
#define CS5463_REG_PULSEWIDTH   0x01 // Energy Pulse Output Width
#define CS5463_REG_LOADMIN      0x02 // No Load Threshold
#define CS5463_REG_TGAIN        0x03 // Temperature Gain
#define CS5463_REG_TOFF         0x04 // Temperature Offset

// Page 3 Registers
#define CS5463_REG_VSAG_DUR     0x01 // Voltage Sag Duration
#define CS5463_REG_VSAG_LVL     0x02 // Voltage Sag Level
#define CS5463_REG_IFAULT_DUR   0x03 // Current Fault Duration
#define CS5463_REG_IFAULT_LVL   0x04 // Current Fault Level

// Commands
#define CS5463_CMD_START_CONV_CONT  0xE8 // Start Continuous Conversions
#define CS5463_CMD_START_CONV_SINGLE 0xE0 // Start Single Conversion
#define CS5463_CMD_SYNC0            0xFE // Sync 0
#define CS5463_CMD_SYNC1            0xFF // Sync 1
#define CS5463_CMD_POWERUP_HALT     0xA0 // Power Up / Halt
#define CS5463_CMD_POWERDOWN_SLEEP  0x80 // Power Down (Sleep)
#define CS5463_CMD_POWERDOWN_STANDBY 0x90 // Power Down (Standby)
#define CS5463_CMD_SOFT_RESET       0x80 // Software Reset (same as Sleep)
#define CS5463_CMD_CAL_DC_OFFSET_I  0xC8 // Calibrate Current DC Offset
#define CS5463_CMD_CAL_AC_OFFSET_I  0xD0 // Calibrate Current AC Offset
#define CS5463_CMD_CAL_GAIN_I       0xD8 // Calibrate Current Gain
#define CS5463_CMD_CAL_DC_OFFSET_V  0xC9 // Calibrate Voltage DC Offset
#define CS5463_CMD_CAL_AC_OFFSET_V  0xD1 // Calibrate Voltage AC Offset
#define CS5463_CMD_CAL_GAIN_V       0xD9 // Calibrate Voltage Gain

// Register Read/Write Commands (Address ORed with command)
#define CS5463_CMD_READ_REG         0x00 // Read Register (address bits 5-0)
#define CS5463_CMD_WRITE_REG        0x40 // Write Register (address bits 5-0)

// Status Register Bits (Mask)
#define CS5463_STATUS_DRDY      0x800000 // Data Ready
#define CS5463_STATUS_CRDY      0x400000 // Conversion Ready
#define CS5463_STATUS_EOUT      0x200000 // Energy Out of Range
#define CS5463_STATUS_EOUTV     0x100000 // Voltage Energy Out of Range
#define CS5463_STATUS_EOUTI     0x080000 // Current Energy Out of Range
#define CS5463_STATUS_EOL       0x040000 // End of Line Cycle
#define CS5463_STATUS_EOC       0x020000 // End of Conversion
#define CS5463_STATUS_PF        0x010000 // Power Fail
#define CS5463_STATUS_TUP       0x008000 // Temperature Update
#define CS5463_STATUS_TORD      0x004000 // Temperature Out of Range
#define CS5463_STATUS_VSAG      0x002000 // Voltage Sag
#define CS5463_STATUS_IFAULT    0x001000 // Current Fault
#define CS5463_STATUS_VPZC      0x000800 // Voltage Positive Zero Crossing
#define CS5463_STATUS_IPZC      0x000400 // Current Positive Zero Crossing
#define CS5463_STATUS_VNZC      0x000200 // Voltage Negative Zero Crossing
#define CS5463_STATUS_INZC      0x000100 // Current Negative Zero Crossing
#define CS5463_STATUS_IC        0x000080 // Invalid Command
#define CS5463_STATUS_LSD       0x000040 // Low Supply Detect
#define CS5463_STATUS_IRQ0      0x000020 // Interrupt Request 0
#define CS5463_STATUS_IRQ1      0x000010 // Interrupt Request 1
#define CS5463_STATUS_DRDY_MASK 0x000008 // Data Ready Mask
#define CS5463_STATUS_EOUT_MASK 0x000004 // Energy Out Mask
#define CS5463_STATUS_EOC_MASK  0x000002 // End of Conversion Mask
#define CS5463_STATUS_EOL_MASK  0x000001 // End of Line Cycle Mask

// Mode Register Bits
#define CS5463_MODE_IHPF        0x000001 // Enable Current HPF
#define CS5463_MODE_VHPF        0x000002 // Enable Voltage HPF
#define CS5463_MODE_IIR         0x000004 // Enable IIR Filters
#define CS5463_MODE_E2MODE      0x000008 // E2 Pin Mode (0=Sign, 1=Apparent Energy)
#define CS5463_MODE_E3MODE_MASK 0x000030 // E3 Pin Mode Mask
#define CS5463_MODE_E3_REACTIVE 0x000000 // E3 = Reactive Energy
#define CS5463_MODE_E3_PFMON    0x000010 // E3 = PFMON Output
#define CS5463_MODE_E3_VSIGN    0x000020 // E3 = Voltage Sign
#define CS5463_MODE_E3_APPARENT 0x000030 // E3 = Apparent Energy

// Configuration Register Bits
#define CS5463_CONFIG_K_MASK    0x00000007 // Clock Divider K Mask
#define CS5463_CONFIG_K_1       0x00000000 // K=1 (MCLK/1)
#define CS5463_CONFIG_K_2       0x00000001 // K=2 (MCLK/2)
#define CS5463_CONFIG_K_4       0x00000002 // K=4 (MCLK/4)
#define CS5463_CONFIG_K_8       0x00000003 // K=8 (MCLK/8)
#define CS5463_CONFIG_K_16      0x00000004 // K=16 (MCLK/16)
#define CS5463_CONFIG_K_32      0x00000005 // K=32 (MCLK/32)
#define CS5463_CONFIG_K_64      0x00000006 // K=64 (MCLK/64)
#define CS5463_CONFIG_K_128     0x00000007 // K=128 (MCLK/128)
#define CS5463_CONFIG_EWA       0x00000008 // Enable Write Any Register
#define CS5463_CONFIG_PC_MASK   0x00003F80 // Phase Compensation Mask
#define CS5463_CONFIG_IGAIN     0x00004000 // Current Channel PGA Gain (0=10x, 1=50x)
#define CS5463_CONFIG_EINT      0x00008000 // Enable Interrupt Pin
#define CS5463_CONFIG_ECLK      0x00010000 // Enable CPUCLK Output
#define CS5463_CONFIG_STRIP     0x00020000 // Stripe ID (Read Only)
#define CS5463_CONFIG_REVID_MASK 0xE0000000 // Revision ID Mask (Read Only)

// Structure to hold all measurements from one read cycle
struct CS5463_Measurements {
    float instantaneousCurrent;
    float instantaneousVoltage;
    float instantaneousPower;
    float activePower;
    float rmsCurrent;
    float rmsVoltage;
    float apparentPower;
    float reactivePowerAvg;
    float reactivePowerTrig;
    float powerFactor;
    float peakCurrent;
    float peakVoltage;
    float temperature;
    float fundamentalActivePower;
    float fundamentalReactivePower;
    float harmonicActivePower;
    uint32_t statusRegister;
    // Add calibration values if needed later
};

class CS5463_Sensor {
public:
    CS5463_Sensor(uint8_t cs_pin);

    bool begin(SPIClass &spi_bus = SPI, uint32_t spi_freq = 1000000, uint8_t spi_mode = SPI_MODE0);
    void reset();
    bool isConnected();

    // Configuration
    void setGainPGA(bool gain50x); // true for 50x, false for 10x (default)
    void setClockDivider(uint8_t k_val); // 0=1, 1=2, ..., 7=128
    void setCycleCount(uint16_t n_cycles); // N value for RMS/Power calculations
    void setFilters(bool enableIIR, bool enableVHPF, bool enableIHPF);
    void setPhaseCompensation(int8_t pc_value); // -64 to +63
    void setEpsilon(float epsilon); // For line frequency adjustment in Q, PF, etc.
    void setPulseRateE(uint32_t rate); // For E1 pulse output
    void setNoLoadThreshold(uint32_t threshold);

    // Calibration - Setting values (obtained externally or via calibration commands)
    void setCalibrationOffsets(float i_dc_offset, float v_dc_offset, float i_ac_offset, float v_ac_offset, float p_offset);
    void setCalibrationGains(float i_gain, float v_gain);
    void setTemperatureCalibration(float t_gain, float t_offset);

    // Calibration - Performing hardware calibration
    bool performCalibration(uint8_t cal_command); // Use CS5463_CMD_CAL_* constants

    // Reading Measurements
    bool readMeasurements(CS5463_Measurements &meas);
    float getInstantaneousCurrent();
    float getInstantaneousVoltage();
    float getInstantaneousPower();
    float getVRMS();
    float getIRMS();
    float getActivePower();
    float getReactivePowerAvg();
    float getReactivePowerTrig();
    float getApparentPower();
    float getPowerFactor();
    float getPeakVoltage();
    float getPeakCurrent();
    float getTemperature();
    float getFundamentalActivePower();
    float getFundamentalReactivePower();
    float getHarmonicActivePower();
    uint32_t getStatusRegister();

    // Low-level access
    uint32_t readRegister(uint8_t address);
    void writeRegister(uint8_t address, uint32_t value);
    void sendCommand(uint8_t command);
    void selectPage(uint8_t page);

private:
    uint8_t _cs_pin;
    SPIClass* _spi;
    SPISettings _spi_settings;
    uint8_t _current_page;

    // Calibration constants (internal representation, maybe raw register values)
    uint32_t _reg_idcoff, _reg_vdcoff, _reg_iacoff, _reg_vacoff, _reg_poff;
    uint32_t _reg_ign, _reg_vgn;
    uint32_t _reg_tgain, _reg_toff;

    // Internal state
    CS5463_Measurements _last_measurement;
    bool _gain_pga_50x;

    // Helper functions
    void spiTransaction(uint8_t* data, uint8_t len);
    float convertRegisterToFloat(uint32_t regValue, bool signedValue = true);
    uint32_t convertFloatToRegister(float value, bool signedValue = true);
    void startConversions(bool continuous = true);
    void haltConversions();

};

#endif

