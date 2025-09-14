#ifndef MS5611_I2C_H_
#define MS5611_I2C_H_

// MS5611 I2C adapter for GY-86 board
// Provides same interface as original MS5611 but using I2C instead of SPI

#define MS5611_SAMPLE_PERIOD_MS         10

#define MS5611_READ_TEMPERATURE         11
#define MS5611_READ_PRESSURE            22

#define MS5611_CMD_RESET                0x1E
#define MS5611_CMD_CONVERT_D1           0x40
#define MS5611_CMD_CONVERT_D2           0x50
#define MS5611_CMD_ADC_READ             0x00
#define MS5611_CMD_ADC_4096             0x08

void     ms5611_i2c_triggerPressureSample(void);
void     ms5611_i2c_triggerTemperatureSample(void);
uint32_t ms5611_i2c_readSample(void);
void     ms5611_i2c_averagedSample(int numSamples);
void     ms5611_i2c_calculateTemperatureC(void);
float    ms5611_i2c_calculatePressurePa(void);
void     ms5611_i2c_calculateSensorNoisePa(void);
int      ms5611_i2c_config(void);
int      ms5611_i2c_sampleStateMachine(void);
void     ms5611_i2c_initializeSampleStateMachine(void);
float    ms5611_i2c_pa2Cm(float pa);
void     ms5611_i2c_measure_noise();
uint8_t  ms5611_i2c_CRC4(uint8_t prom[]);
int      ms5611_i2c_readPROM(void);
void     ms5611_i2c_getCalibrationParameters(void);
void     ms5611_i2c_reset(void);

extern float ZCmAvg_MS5611_I2C;
extern float ZCmSample_MS5611_I2C;
extern float PaSample_MS5611_I2C;
extern int   CelsiusSample_MS5611_I2C;

#endif // MS5611_I2C_H_