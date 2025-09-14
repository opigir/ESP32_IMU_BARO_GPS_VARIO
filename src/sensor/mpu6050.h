#ifndef MPU6050_H_
#define MPU6050_H_

// MPU6050 adapter for GY-86 board
// Provides same interface as MPU9250 but for MPU6050 chip

typedef enum GyroRange_
{
GYRO_RANGE_250DPS,
GYRO_RANGE_500DPS,
GYRO_RANGE_1000DPS,
GYRO_RANGE_2000DPS
} GyroRange;

typedef enum AccelRange_
{
ACCEL_RANGE_2G,
ACCEL_RANGE_4G,
ACCEL_RANGE_8G,
ACCEL_RANGE_16G    
} AccelRange;

typedef enum DlpfBandwidth_
{
DLPF_BANDWIDTH_184HZ,
DLPF_BANDWIDTH_92HZ,
DLPF_BANDWIDTH_41HZ,
DLPF_BANDWIDTH_20HZ,
DLPF_BANDWIDTH_10HZ,
DLPF_BANDWIDTH_5HZ
} DlpfBandwidth;

// Function prototypes matching the MPU9250 interface
int mpu6050_config();
int mpu6050_setSrd(uint8_t srd);
int mpu6050_enableDataReadyInterrupt();
int mpu6050_disableDataReadyInterrupt();

int mpu6050_calibrateGyro();
int mpu6050_calibrateAccel();

// Main data reading function - no magnetometer on MPU6050
int mpu6050_getGyroAccelData(float* pgx, float* pgy, float* pgz, float* pax, float* pay, float* paz);

void mpu6050_initCalibrationParams(void);
int mpu6050_getVector(uint8_t startAddr, int16_t* px, int16_t* py, int16_t* pz);

#endif