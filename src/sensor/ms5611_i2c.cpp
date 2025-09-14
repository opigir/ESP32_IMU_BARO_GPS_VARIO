#include "common.h"
#ifdef ATOM_LITE_BUILD
#include "config_atom.h"
#else
#include "config.h"
#endif
#include "drv/cct.h"
#include "sensor/ms5611_i2c.h"
#include <Wire.h>

float ZCmAvg_MS5611_I2C;
float ZCmSample_MS5611_I2C;
float PaSample_MS5611_I2C;
int   CelsiusSample_MS5611_I2C;

static uint8_t  Prom_[16];
static uint16_t Cal_[6];
static int64_t  Tref_;
static int64_t  OffT1_;
static int64_t  SensT1_;
static int32_t  TempCx100_;
static uint32_t D1_;
static uint32_t D2_;
static int64_t  DT_;
static int      SensorState_;

static const char* TAG = "ms5611_i2c";

#define MS5611_ADDRESS  0x77

// I2C helper functions
static void i2cWriteByte(uint8_t data) {
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(data);
    Wire.endTransmission();
}

static uint8_t i2cReadByte() {
    Wire.requestFrom((uint8_t)MS5611_ADDRESS, (uint8_t)1);
    return Wire.read();
}

static void i2cReadBytes(uint8_t count, uint8_t* dest) {
    Wire.requestFrom((uint8_t)MS5611_ADDRESS, count);
    for (uint8_t i = 0; i < count; i++) {
        dest[i] = Wire.read();
    }
}

static uint16_t i2cReadPROMWord(uint8_t promAddr) {
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(0xA0 + (promAddr * 2));
    Wire.endTransmission();
    
    Wire.requestFrom((uint8_t)MS5611_ADDRESS, (uint8_t)2);
    uint16_t value = Wire.read() << 8;
    value |= Wire.read();
    return value;
}

void ms5611_i2c_reset(void) {
    i2cWriteByte(MS5611_CMD_RESET);
    delayMicroseconds(100000); // 100ms delay after reset
}

int ms5611_i2c_readPROM(void) {
    // Read calibration coefficients C1-C6 from PROM
    for (int i = 1; i <= 6; i++) {
        Cal_[i-1] = i2cReadPROMWord(i);
        ESP_LOGI(TAG, "C%d = 0x%04X = %u", i, Cal_[i-1], Cal_[i-1]);
    }
    
    // Verify coefficients are valid (not all zeros)
    bool valid = false;
    for (int i = 0; i < 6; i++) {
        if (Cal_[i] != 0) {
            valid = true;
            break;
        }
    }
    
    if (!valid) {
        ESP_LOGE(TAG, "Invalid PROM data - all coefficients are zero");
        return 0;
    }
    
    ESP_LOGI(TAG, "PROM read successfully");
    return 1;
}

void ms5611_i2c_getCalibrationParameters(void) {
    Tref_ = (int64_t)Cal_[4] << 8;
    OffT1_ = (int64_t)Cal_[1] << 16;
    SensT1_ = (int64_t)Cal_[0] << 15;
}

int ms5611_i2c_config(void) {
    PaSample_MS5611_I2C = 0.0f;
    ZCmSample_MS5611_I2C = 0.0f;
    CelsiusSample_MS5611_I2C = 0;
    ZCmAvg_MS5611_I2C = 0.0f;
    
    ms5611_i2c_reset();
    
    if (!ms5611_i2c_readPROM()) {
        ESP_LOGE(TAG, "Error reading calibration PROM");
        return -1;
    }
    
    ms5611_i2c_getCalibrationParameters();
    
    ESP_LOGI(TAG, "MS5611 I2C configured successfully");
    return 0;
}

void ms5611_i2c_initializeSampleStateMachine(void) {
    ms5611_i2c_triggerTemperatureSample();
    SensorState_ = MS5611_READ_TEMPERATURE;
}

void ms5611_i2c_triggerTemperatureSample(void) {
    i2cWriteByte(MS5611_CMD_CONVERT_D2 + MS5611_CMD_ADC_4096);
}

void ms5611_i2c_triggerPressureSample(void) {
    i2cWriteByte(MS5611_CMD_CONVERT_D1 + MS5611_CMD_ADC_4096);
}

uint32_t ms5611_i2c_readSample(void) {
    i2cWriteByte(MS5611_CMD_ADC_READ);
    
    uint8_t data[3];
    i2cReadBytes(3, data);
    
    uint32_t result = (uint32_t)data[0] << 16;
    result |= (uint32_t)data[1] << 8;
    result |= data[2];
    
    return result;
}

void ms5611_i2c_calculateTemperatureC(void) {
    DT_ = D2_ - Tref_;
    TempCx100_ = 2000 + ((DT_ * (int64_t)Cal_[5]) >> 23);
    CelsiusSample_MS5611_I2C = TempCx100_;
}

float ms5611_i2c_calculatePressurePa(void) {
    int64_t OFF = OffT1_ + (((int64_t)Cal_[3] * DT_) >> 7);
    int64_t SENS = SensT1_ + (((int64_t)Cal_[2] * DT_) >> 8);
    
    // Second order temperature compensation
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    if (TempCx100_ < 2000) {
        T2 = (DT_ * DT_) >> 31;
        OFF2 = (5 * (TempCx100_ - 2000) * (TempCx100_ - 2000)) >> 1;
        SENS2 = OFF2 >> 1;
        
        if (TempCx100_ < -1500) {
            OFF2 += 7 * (TempCx100_ + 1500) * (TempCx100_ + 1500);
            SENS2 += (11 * (TempCx100_ + 1500) * (TempCx100_ + 1500)) >> 1;
        }
    }
    
    TempCx100_ -= T2;
    OFF -= OFF2;
    SENS -= SENS2;
    
    int32_t P = (((D1_ * SENS) >> 21) - OFF) >> 15;
    
    PaSample_MS5611_I2C = (float)P;
    return PaSample_MS5611_I2C;
}

float ms5611_i2c_pa2Cm(float pa) {
    // Standard atmosphere conversion
    return 44330.0f * (1.0f - pow(pa / 101325.0f, 0.1903f));
}

int ms5611_i2c_sampleStateMachine(void) {
    static uint32_t lastTime = 0;
    uint32_t nowMs = millis();
    
    if (nowMs - lastTime >= MS5611_SAMPLE_PERIOD_MS) {
        lastTime = nowMs;
        
        if (SensorState_ == MS5611_READ_TEMPERATURE) {
            D2_ = ms5611_i2c_readSample();
            ms5611_i2c_calculateTemperatureC();
            ms5611_i2c_triggerPressureSample();
            SensorState_ = MS5611_READ_PRESSURE;
        }
        else {
            D1_ = ms5611_i2c_readSample();
            float pa = ms5611_i2c_calculatePressurePa();
            ZCmSample_MS5611_I2C = ms5611_i2c_pa2Cm(pa);
            ms5611_i2c_triggerTemperatureSample();
            SensorState_ = MS5611_READ_TEMPERATURE;
            return 1; // New altitude sample available
        }
    }
    return 0;
}

void ms5611_i2c_averagedSample(int numSamples) {
    // Implementation would go here if needed
}

void ms5611_i2c_calculateSensorNoisePa(void) {
    // Implementation would go here if needed
}

void ms5611_i2c_measure_noise() {
    // Implementation would go here if needed
}

uint8_t ms5611_i2c_CRC4(uint8_t prom[]) {
    // CRC implementation would go here if needed
    return 0;
}