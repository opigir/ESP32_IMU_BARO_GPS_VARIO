#include "common.h"
#ifdef ATOM_LITE_BUILD
#include "config_atom.h"
#else
#include "config.h"
#endif
#include "drv/cct.h"
#include "sensor/mpu6050.h"
#include <Wire.h>

static const char* TAG = "mpu6050";

// MPU6050 I2C address and registers
#define MPU6050_ADDRESS     0x68
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_SMPRT_DIV   0x19
#define MPU6050_CONFIG      0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_INT_ENABLE  0x38
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_WHO_AM_I    0x75

// Calibration parameters
static float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
static float accelBiasX = 0.0f, accelBiasY = 0.0f, accelBiasZ = 0.0f;

// Scale factors
static float gyroScale = 131.0f;  // LSB/°/s for ±250°/s range
static float accelScale = 16384.0f; // LSB/g for ±2g range

// I2C helper functions
static void i2cWriteByte(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

static uint8_t i2cReadByte(uint8_t reg) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)1);
    return Wire.read();
}

static void i2cReadBytes(uint8_t reg, uint8_t count, uint8_t* dest) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, count);
    for (uint8_t i = 0; i < count; i++) {
        dest[i] = Wire.read();
    }
}

void mpu6050_initCalibrationParams(void) {
    gyroBiasX = gyroBiasY = gyroBiasZ = 0.0f;
    accelBiasX = accelBiasY = accelBiasZ = 0.0f;
}

int mpu6050_config() {
    // Wake up MPU6050 (disable sleep mode)
    i2cWriteByte(MPU6050_PWR_MGMT_1, 0x00);
    delayMicroseconds(100000); // 100ms delay
    
    // Verify communication
    uint8_t whoami = i2cReadByte(MPU6050_WHO_AM_I);
    if (whoami != 0x68) {
        ESP_LOGE(TAG, "MPU6050 not found, WHO_AM_I = 0x%02X", whoami);
        return -1;
    }
    
    // Configure sample rate (1kHz / (1 + 1) = 500Hz)
    i2cWriteByte(MPU6050_SMPRT_DIV, 0x01);
    
    // Configure DLPF (44Hz bandwidth for 1kHz sample rate)
    i2cWriteByte(MPU6050_CONFIG, 0x03);
    
    // Configure gyro range: ±250°/s (most sensitive)
    i2cWriteByte(MPU6050_GYRO_CONFIG, 0x00);
    gyroScale = 131.0f;
    
    // Configure accel range: ±2g (most sensitive)
    i2cWriteByte(MPU6050_ACCEL_CONFIG, 0x00);
    accelScale = 16384.0f;
    
    mpu6050_initCalibrationParams();
    
    ESP_LOGI(TAG, "MPU6050 configured successfully");
    return 0;
}

int mpu6050_setSrd(uint8_t srd) {
    i2cWriteByte(MPU6050_SMPRT_DIV, srd);
    return 0;
}

int mpu6050_enableDataReadyInterrupt() {
    i2cWriteByte(MPU6050_INT_ENABLE, 0x01);
    return 0;
}

int mpu6050_disableDataReadyInterrupt() {
    i2cWriteByte(MPU6050_INT_ENABLE, 0x00);
    return 0;
}

int mpu6050_getVector(uint8_t startAddr, int16_t* px, int16_t* py, int16_t* pz) {
    uint8_t rawData[6];
    i2cReadBytes(startAddr, 6, rawData);
    
    *px = (rawData[0] << 8) | rawData[1];
    *py = (rawData[2] << 8) | rawData[3];
    *pz = (rawData[4] << 8) | rawData[5];
    
    return 0;
}

int mpu6050_calibrateGyro() {
    ESP_LOGI(TAG, "Calibrating gyroscope...");
    
    const int numSamples = 1000;
    float sumX = 0, sumY = 0, sumZ = 0;
    
    for (int i = 0; i < numSamples; i++) {
        int16_t gx, gy, gz;
        mpu6050_getVector(0x43, &gx, &gy, &gz); // Gyro registers start at 0x43
        
        sumX += gx;
        sumY += gy;
        sumZ += gz;
        
        delayMicroseconds(2000); // 2ms delay for 500Hz sampling
    }
    
    gyroBiasX = sumX / numSamples;
    gyroBiasY = sumY / numSamples;
    gyroBiasZ = sumZ / numSamples;
    
    ESP_LOGI(TAG, "Gyro bias: X=%.2f, Y=%.2f, Z=%.2f", gyroBiasX, gyroBiasY, gyroBiasZ);
    return 0;
}

int mpu6050_calibrateAccel() {
    ESP_LOGI(TAG, "Calibrating accelerometer...");
    
    const int numSamples = 1000;
    float sumX = 0, sumY = 0, sumZ = 0;
    
    for (int i = 0; i < numSamples; i++) {
        int16_t ax, ay, az;
        mpu6050_getVector(0x3B, &ax, &ay, &az); // Accel registers start at 0x3B
        
        sumX += ax;
        sumY += ay;
        sumZ += az;
        
        delayMicroseconds(2000); // 2ms delay for 500Hz sampling
    }
    
    accelBiasX = sumX / numSamples;
    accelBiasY = sumY / numSamples;
    accelBiasZ = (sumZ / numSamples) - accelScale; // Remove 1g gravity
    
    ESP_LOGI(TAG, "Accel bias: X=%.2f, Y=%.2f, Z=%.2f", accelBiasX, accelBiasY, accelBiasZ);
    return 0;
}

int mpu6050_getGyroAccelData(float* pgx, float* pgy, float* pgz, float* pax, float* pay, float* paz) {
    uint8_t rawData[14];
    i2cReadBytes(MPU6050_ACCEL_XOUT_H, 14, rawData);
    
    // Parse accelerometer data (first 6 bytes)
    int16_t ax = (rawData[0] << 8) | rawData[1];
    int16_t ay = (rawData[2] << 8) | rawData[3];
    int16_t az = (rawData[4] << 8) | rawData[5];
    
    // Skip temperature data (bytes 6-7)
    
    // Parse gyroscope data (last 6 bytes)
    int16_t gx = (rawData[8] << 8) | rawData[9];
    int16_t gy = (rawData[10] << 8) | rawData[11];
    int16_t gz = (rawData[12] << 8) | rawData[13];
    
    // Apply calibration and convert to physical units
    *pgx = (gx - gyroBiasX) / gyroScale;  // °/s
    *pgy = (gy - gyroBiasY) / gyroScale;
    *pgz = (gz - gyroBiasZ) / gyroScale;
    
    *pax = (ax - accelBiasX) / accelScale; // g
    *pay = (ay - accelBiasY) / accelScale;
    *paz = (az - accelBiasZ) / accelScale;
    
    return 0;
}