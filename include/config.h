#ifndef CONFIG_H_
#define CONFIG_H_

#include "sdkconfig.h"
#include "driver/gpio.h"

// M5Stack Atom Lite + GY-86 Configuration
// This is a stripped-down config for bluetooth pressure sensor + GPS

// No barometric sensor selection needed - using MS5611 on GY-86 via I2C
#define USE_MS5611_I2C true

// I2C configuration for GY-86 sensor board
// GY-86 contains: MPU6050, HMC5883L (via MPU6050 bypass), MS5611
#define I2C_SDA_PIN     25  // M5Stack Atom Lite protoboard pin
#define I2C_SCL_PIN     21  // M5Stack Atom Lite protoboard pin
#define I2C_FREQ_HZ     100000

// GPS UART configuration for BZ-121 GPS (M10 chip)
// BZ-121 specs: 5V power, 115200 baud, 10Hz default, multi-constellation
#define pinGpsTXD       19  // Connect to GPS RX
#define pinGpsRXD       23  // Connect to GPS TX
#define pinGpsRTS       UART_PIN_NO_CHANGE  // Not used
#define pinGpsCTS       UART_PIN_NO_CHANGE  // Not used
#define GPS_UART_NUM    UART_NUM_1
#define GPS_BAUD_RATE   115200  // BZ-121 uses 115200 baud (not 9600)
#define UART_RX_BUFFER_SIZE   512  // Larger buffer for 10Hz multi-constellation data

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
#define VARIO_CLIMB_THRESHOLD_CPS_MIN       10
#define VARIO_CLIMB_THRESHOLD_CPS_DEFAULT  	50
#define VARIO_CLIMB_THRESHOLD_CPS_MAX       100

#define VARIO_ZERO_THRESHOLD_CPS_MIN        (-20)
#define VARIO_ZERO_THRESHOLD_CPS_DEFAULT  	5
#define VARIO_ZERO_THRESHOLD_CPS_MAX        10

#define VARIO_SINK_THRESHOLD_CPS_MIN        (-400)
#define VARIO_SINK_THRESHOLD_CPS_DEFAULT  	(-250)
#define VARIO_SINK_THRESHOLD_CPS_MAX        (-10)

#define VARIO_CROSSOVER_CPS_MIN             300
#define VARIO_CROSSOVER_CPS_DEFAULT         400
#define VARIO_CROSSOVER_CPS_MAX             800

#define VARIO_DISPLAY_IIR_MIN               90
#define VARIO_DISPLAY_IIR_DEFAULT           95
#define VARIO_DISPLAY_IIR_MAX               99

// LCD backlight timing
#define BACKLIT_SECS_MIN                    5
#define BACKLIT_SECS_DEFAULT                30
#define BACKLIT_SECS_MAX                    60

// Track logging
#define TRACK_START_THRESHOLD_M_MIN         0
#define TRACK_START_THRESHOLD_M_DEFAULT     20
#define TRACK_START_THRESHOLD_M_MAX         100

#define TRACK_INTERVAL_SECS_MIN             1
#define TRACK_INTERVAL_SECS_DEFAULT         3
#define TRACK_INTERVAL_SECS_MAX             60

// Navigation and sensor settings
#define GLIDE_RATIO_IIR_MIN                 80
#define GLIDE_RATIO_IIR_DEFAULT             90
#define GLIDE_RATIO_IIR_MAX                 99

#define GYRO_OFFSET_LIMIT_1000DPS_MIN       25
#define GYRO_OFFSET_LIMIT_1000DPS_DEFAULT   150
#define GYRO_OFFSET_LIMIT_1000DPS_MAX       200

#define MAG_DECLINATION_DEG_MIN             (-60)
#define MAG_DECLINATION_DEG_DEFAULT         0
#define MAG_DECLINATION_DEG_MAX             60

#define SPEAKER_VOLUME_MIN                  0
#define SPEAKER_VOLUME_DEFAULT              2
#define SPEAKER_VOLUME_MAX                  3

#define WAYPOINT_RADIUS_M_MIN               5
#define WAYPOINT_RADIUS_M_DEFAULT           50
#define WAYPOINT_RADIUS_M_MAX               20000

// Altitude display options
#define ALTITUDE_DISPLAY_GPS    0
#define ALTITUDE_DISPLAY_BARO   1

// LCD contrast settings
#define LCD_CONTRAST_MIN        1
#define LCD_CONTRAST_MAX        10
#define LCD_CONTRAST_DEFAULT    4

// Barometer-only data transmission option
#define USE_BARO_ONLY_MIN       0
#define USE_BARO_ONLY_DEFAULT   0
#define USE_BARO_ONLY_MAX       1

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

// Flash logging types (for GPS module compatibility)
#define LOGTYPE_NONE    0
#define LOGTYPE_IBG     1
#define LOGTYPE_GPS     2

// Stub options structure for GPS module compatibility
typedef struct {
    struct {
        int logType;
        int trackIntervalSecs;
        int utcOffsetMins;
    } misc;
} OPTIONS;

// Global variables stubs for minimal build
extern OPTIONS opt;
extern SemaphoreHandle_t FlashLogMutex;
extern bool IsGpsTrackActive;
extern bool IsLoggingIBG;

// Flash log structures (stubs for GPS compatibility)
typedef struct {
    struct {
        int magic;
        int gpsFlags;
        int baroFlags;
    } hdr;
    struct {
        int timeOfWeekmS;
        int heightMSLmm;
        int vertAccuracymm;
        int velNorthmmps;
        int velEastmmps;
        int velDownmmps;
        int velAccuracymmps;
        int lonDeg7;
        int latDeg7;
    } gps;
    struct {
        int heightMSLcm;
    } baro;
} FLASHLOG_IBG_RECORD;

typedef struct {
    struct {
        int magic;
        int fixType;
        int numSV;
    } hdr;
    struct {
        int posDOP;
        int utcYear, utcMonth, utcDay;
        int utcHour, utcMinute, utcSecond;
        int nanoSeconds;
        int heightMSLmm;
        int lonDeg7, latDeg7;
    } trkpt;
} FLASHLOG_GPS_RECORD;

extern FLASHLOG_IBG_RECORD FlashLogIBGRecord;
extern FLASHLOG_GPS_RECORD FlashLogGPSRecord;

// Flash logging function stubs
int flashlog_writeIBGRecord(FLASHLOG_IBG_RECORD* record);
void flashlog_writeGPSRecord(FLASHLOG_GPS_RECORD* record);

// Additional constants for GPS module
#define FLASHLOG_IBG_MAGIC    0x12345678
#define FLASHLOG_GPS_MAGIC    0x87654321

#endif