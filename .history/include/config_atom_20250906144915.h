#ifndef CONFIG_ATOM_H_
#define CONFIG_ATOM_H_

#include "sdkconfig.h"
#include "driver/gpio.h"

// M5Stack Atom Lite + GY-86 Configuration
// This is a stripped-down config for bluetooth pressure sensor + GPS

// No barometric sensor selection needed - using MS5611 on GY-86 via I2C
#define USE_MS5611_I2C true

// I2C configuration for GY-86 sensor board
// GY-86 contains: MPU6050, HMC5883L (via MPU6050 bypass), MS5611
#define I2C_SDA_PIN     21  // M5Stack Atom Lite Grove port
#define I2C_SCL_PIN     22  // M5Stack Atom Lite Grove port
#define I2C_FREQ_HZ     100000

// GPS UART configuration (connect to available GPIO pins)
#define pinGpsTXD       19  // Connect to GPS RX
#define pinGpsRXD       23  // Connect to GPS TX
#define GPS_UART_NUM    UART_NUM_1
#define UART_RX_BUFFER_SIZE   256

// Button configuration (built-in button on Atom Lite)
#define pinBtn0         39  // Built-in button (input only, no pullup)
#define BTN0()          (!digitalRead(pinBtn0))  // Button is active low

// LED configuration (built-in RGB LED on Atom Lite)
#define pinLED          27  // Built-in NeoPixel LED
#define LED_ON()        // Will implement via FastLED or similar
#define LED_OFF()       

// Voltage monitoring (if using external voltage divider)
#define pinADC          33  // Available GPIO for voltage monitoring

// Remove all LCD, audio, and other unnecessary hardware definitions

////////////////////////////////////////////////////////////////////
// USER-CONFIGURABLE PARAMETER DEFAULTS AND LIMITS

// UTC offset in minutes
#define UTC_OFFSET_MINS_MIN        (-720)
#define UTC_OFFSET_MINS_DEFAULT     0      // Set to your timezone
#define UTC_OFFSET_MINS_MAX         720

// GPS position degree-of-precision required for valid fix
#define GPS_STABLE_DOP_MIN             3
#define GPS_STABLE_DOP_DEFAULT         8
#define GPS_STABLE_DOP_MAX             15

// Bluetooth message configuration
#define BT_MSG_LK8EX1	0
#define BT_MSG_XCTRC	1

#define BT_MSG_FREQ_HZ_MIN	    1
#define BT_MSG_FREQ_HZ_MAX	    10
#define BT_MSG_FREQ_HZ_DEFAULT	5    // 5Hz for phone apps

// Kalman filter configuration for pressure altitude smoothing
#define KF_ACCEL_VARIANCE_DEFAULT            100
#define KF_ACCEL_VARIANCE_MIN                50
#define KF_ACCEL_VARIANCE_MAX                150

// Vario thresholds in cm/sec for Bluetooth transmission
#define VARIO_CLIMB_THRESHOLD_CPS_DEFAULT  	50
#define VARIO_ZERO_THRESHOLD_CPS_DEFAULT  	5
#define VARIO_SINK_THRESHOLD_CPS_DEFAULT  	(-250)

// MS5611 altitude noise variance (measured offline)
#define KF_Z_MEAS_VARIANCE            200

// Sensor I2C addresses for GY-86
#define MPU6050_I2C_ADDR    0x68
#define MS5611_I2C_ADDR     0x77
#define HMC5883L_I2C_ADDR   0x1E  // Accessed via MPU6050 bypass

// Debug configuration - comment out for release
#define MAIN_DEBUG
#define GPS_DEBUG
#define SENSOR_DEBUG
#define BLUETOOTH_DEBUG

// Minimal build - remove complex features
#define MINIMAL_BUILD

#endif