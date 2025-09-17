#include <Arduino.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <FastLED.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// M5Stack Atom Lite + GY-86 Bluetooth Pressure Sensor + GPS
// Advanced version with Kalman filter sensor fusion for zero-lag variometer

// Configuration
#ifdef ATOM_LITE_BUILD
#include "config_atom.h"
#else
#include "config.h"
#endif

// Sensor modules from original codebase
#include "sensor/mpu6050.h"
#include "sensor/ms5611_i2c.h"
#include "sensor/imu.h"
#include "sensor/kalmanfilter4d.h"
#include "sensor/gps.h"
#include "sensor/ringbuf.h"

// Minimal debug logging to save space
#define DEBUG_LOG(fmt, ...) // Disabled for space
#define FUSION_LOG(fmt, ...) // Disabled for space

// Sensor fusion configuration
#define IMU_SAMPLE_RATE_HZ      500   // High-frequency IMU sampling
#define BARO_SAMPLE_RATE_HZ     50    // Pressure sensor sampling

// Runtime tunable parameters - can be changed via serial commands
float kalmanAccelVariance = 25.0f;  // Start more aggressive (was 50.0f)
float kalmanAdaptFactor = 3.0f;     // More aggressive adaptation (was 2.0f)
float accelMeasVariance = 2.0f;     // Trust accelerometer much more (was 5.0f)
float accelBiasVariance = 0.02f;    // Allow faster bias adaptation (was 0.01f)

// Helper macro for degree to radian conversion
#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * PI / 180.0f)
#endif

// Runtime debug control
bool showFusionDebug = false;       // Toggle fusion debug output

// Global quaternion variables for IMU orientation (from imu.h)
extern float q0, q1, q2, q3;

// MS5611 I2C sensor variables (matching ms5611_i2c.h interface)
extern float ZCmSample_MS5611_I2C;
extern float PaSample_MS5611_I2C;
extern int CelsiusSample_MS5611_I2C;

// Using existing ringbuf module from sensor/ folder
// Ring buffer functions are declared in sensor/ringbuf.h

// RGB LED configuration for M5Stack Atom Lite
#define NUM_LEDS    1
#define LED_PIN     27
CRGB leds[NUM_LEDS];

// LED timing variables
unsigned long ledOffTime = 0;
bool ledActive = false;

// Global variables
BluetoothSerial SerialBT;
WebServer webServer(80);
Preferences preferences;
unsigned long lastValidGps = 0;

// WiFi Configuration
const char* ssid = "ESP32-Vario-Config";  // AP mode SSID
const char* password = "vario123";        // AP mode password

// GPS module compatibility stubs (minimal build)
OPTIONS opt = { .misc = { .logType = LOGTYPE_NONE, .trackIntervalSecs = 1, .utcOffsetMins = 0, .useBaroOnly = 0 } };
SemaphoreHandle_t FlashLogMutex = NULL;
bool IsGpsTrackActive = false;
bool IsLoggingIBG = false;
FLASHLOG_IBG_RECORD FlashLogIBGRecord = {0};
FLASHLOG_GPS_RECORD FlashLogGPSRecord = {0};

// Stub functions for flash logging
int flashlog_writeIBGRecord(FLASHLOG_IBG_RECORD* record) { return 0; }
void flashlog_writeGPSRecord(FLASHLOG_GPS_RECORD* record) {}

// Simple options initialization for minimal build
void opt_setDefaults() {
    opt.misc.logType = LOGTYPE_NONE;
    opt.misc.trackIntervalSecs = 1;
    opt.misc.utcOffsetMins = 0;
    opt.misc.useBaroOnly = 0;  // Default to fusion mode
}

// Forward declarations
void setLedMode(uint8_t mode, uint32_t duration_ms);
void updateLed();
void generateNMEAFromUBX();
void setupWebServer();
void handleRoot();
void handleGetStatus();
void handleSetParams();
String getWebPageHTML();

// Sensor data
struct SensorData {
    float pressurePa;
    float altitudeM;
    float temperatureC;
    float climbRateCmS;
    float batteryV;
    bool gpsValid;
    float latitude;
    float longitude;
    float gpsAltitudeM;
    float speedKmH;
    float courseHeadingDeg;
    uint16_t year;
    uint8_t month, day, hour, minute, second;
    uint8_t hdop;
} sensorData;

// Configuration (moved up for use in save/load functions)
uint8_t btMsgType = BT_MSG_XCTRC;  // Default to XCTRC for full GPS integration
uint8_t btMsgFreqHz = 5;           // 5Hz transmission rate
unsigned long BT_INTERVAL_MS = 200;            // Dynamic Bluetooth transmission interval

// Timing variables
unsigned long lastBtTransmit = 0;
unsigned long lastSensorRead = 0;
unsigned long lastGpsRead = 0;
const unsigned long SENSOR_INTERVAL_MS = 100; // 10Hz sensor reading
const unsigned long GPS_INTERVAL_MS = 100;    // Check GPS at 10Hz

// High-frequency sensor fusion variables
float fusedAltitudeM = 0.0f;
float fusedClimbRateCmS = 0.0f;
bool sensorFusionInitialized = false;

// Save web configuration settings to NVS
void saveWebConfig() {
    preferences.begin("vario_config", false);
    preferences.putFloat("kalmanAccelVar", kalmanAccelVariance);
    preferences.putFloat("kalmanAdaptFac", kalmanAdaptFactor);
    preferences.putFloat("accelMeasVar", accelMeasVariance);
    preferences.putFloat("accelBiasVar", accelBiasVariance);
    preferences.putBool("showFusionDbg", showFusionDebug);
    preferences.putUChar("btMsgType", btMsgType);
    preferences.putUChar("btMsgFreqHz", btMsgFreqHz);
    preferences.putBool("useBaroOnly", opt.misc.useBaroOnly);
    preferences.end();
    Serial.println("Web configuration saved to NVS");
}

// Load web configuration settings from NVS
void loadWebConfig() {
    preferences.begin("vario_config", true); // Read-only mode
    kalmanAccelVariance = preferences.getFloat("kalmanAccelVar", 25.0f);
    kalmanAdaptFactor = preferences.getFloat("kalmanAdaptFac", 3.0f);
    accelMeasVariance = preferences.getFloat("accelMeasVar", 2.0f);
    accelBiasVariance = preferences.getFloat("accelBiasVar", 0.02f);
    showFusionDebug = preferences.getBool("showFusionDbg", false);
    btMsgType = preferences.getUChar("btMsgType", BT_MSG_XCTRC);
    btMsgFreqHz = preferences.getUChar("btMsgFreqHz", 5);
    opt.misc.useBaroOnly = preferences.getBool("useBaroOnly", false);
    preferences.end();

    // Update Bluetooth interval based on loaded frequency
    BT_INTERVAL_MS = 1000 / btMsgFreqHz;

    Serial.printf("Web configuration loaded from NVS:\n");
    Serial.printf("  Kalman Accel Variance: %.1f\n", kalmanAccelVariance);
    Serial.printf("  Kalman Adapt Factor: %.1f\n", kalmanAdaptFactor);
    Serial.printf("  Accel Meas Variance: %.1f\n", accelMeasVariance);
    Serial.printf("  Accel Bias Variance: %.3f\n", accelBiasVariance);
    Serial.printf("  BT Message Type: %s\n", (btMsgType == BT_MSG_LK8EX1) ? "LK8EX1" : "XCTRC");
    Serial.printf("  BT Frequency: %d Hz\n", btMsgFreqHz);
    Serial.printf("  Barometer Only: %s\n", opt.misc.useBaroOnly ? "ON" : "OFF");
}

// Timing for sensor fusion
unsigned long lastImuUpdate = 0;
unsigned long lastBaroUpdate = 0;
const unsigned long IMU_INTERVAL_US = 1000000 / IMU_SAMPLE_RATE_HZ;  // 2000us for 500Hz
const unsigned long BARO_INTERVAL_US = 1000000 / BARO_SAMPLE_RATE_HZ; // 20000us for 50Hz

// Sensor data buffers
float gx, gy, gz; // Gyroscope data (deg/s)
float ax, ay, az; // Accelerometer data (g)
float pressureAltM = 0.0f; // Barometric altitude (m)
bool newBaroSample = false;

// Initialize sensor fusion system
bool initializeSensorFusion() {
    Serial.println("Initializing advanced sensor fusion...");
    
    // Configure MPU6050
    if (mpu6050_config() != 0) {
        Serial.println("Failed to initialize MPU6050!");
        return false;
    }
    
    // MPU6050 configured - calibration now done manually via long button press
    
    // Configure MS5611 I2C
    if (ms5611_i2c_config() != 0) {
        Serial.println("Failed to initialize MS5611!");
        return false;
    }
    
    // Initialize barometric sampling
    ms5611_i2c_initializeSampleStateMachine();
    
    // Initialize Kalman filter with initial altitude estimate
    Serial.println("Calibrating initial altitude (2 seconds)...");
    delay(1000); // Let sensors settle properly
    
    // Take multiple readings for better initial calibration
    float altSum = 0.0f;
    int validReadings = 0;
    for (int i = 0; i < 20; i++) {
        if (ms5611_i2c_sampleStateMachine()) {
            altSum += ZCmSample_MS5611_I2C / 100.0f;
            validReadings++;
        }
        delay(50);
    }
    
    float initialAltitude = (validReadings > 0) ? (altSum / validReadings) : 0.0f;
    Serial.printf("Initial altitude: %.2f m (from %d readings)\n", initialAltitude, validReadings);
    
    // Initialize ring buffer for accelerometer averaging
    ringbuf_init();
    Serial.println("Ring buffer initialized");
    
    // Configure Kalman filter with runtime tunable values
    kalmanFilter4d_configure(kalmanAccelVariance, kalmanAdaptFactor, 
                            initialAltitude * 100.0f, 0.0f, 0.0f); // Convert to cm like original
    
    sensorFusionInitialized = true;
    Serial.println("Sensor fusion initialized successfully");
    return true;
}

// High-frequency sensor fusion loop
void updateSensorFusion() {
    if (!sensorFusionInitialized) return;
    
    unsigned long nowUs = micros();
    
    // High-frequency IMU update (500Hz)
    if (nowUs - lastImuUpdate >= IMU_INTERVAL_US) {
        lastImuUpdate = nowUs;
        
        // Read MPU6050 data
        if (mpu6050_getGyroAccelData(&gx, &gy, &gz, &ax, &ay, &az) == 0) {
            // Apply NED coordinate transformation for flat-mounted MPU6050 (labeled side up)
            // MPU6050 coordinate system: X=left-to-right, Y=back-to-front, Z=upward
            // NED coordinate system: X=north, Y=east, Z=down
            // When flat on table: MPU reads X≈0, Y≈0, Z≈+1g
            float axNEDmG = -ay * 1000.0f;  // Convert g to milli-G
            float ayNEDmG = -ax * 1000.0f;
            float azNEDmG = az * 1000.0f;   // Positive: MPU Z+ (up) → NED Z+ (down)
            float gxNEDdps = gy;
            float gyNEDdps = gx;
            float gzNEDdps = -gz;
            
            // Use accelerometer data for determining the orientation quaternion only when accel 
            // vector magnitude is in [0.75g, 1.25g] window (from original lines 320-321)
            float asqd = axNEDmG*axNEDmG + ayNEDmG*ayNEDmG + azNEDmG*azNEDmG;
            int useAccel = ((asqd > 562500.0f) && (asqd < 1562500.0f)) ? 1 : 0;
            
            // Update IMU orientation filter with NED coordinates
            float dt = IMU_INTERVAL_US / 1000000.0f;
            imu_mahonyAHRSupdate6DOF(useAccel, dt, 
                                   DEG2RAD(gxNEDdps), DEG2RAD(gyNEDdps), DEG2RAD(gzNEDdps), 
                                   axNEDmG, ayNEDmG, azNEDmG);
            
            // Calculate gravity-compensated acceleration using NED coordinates
            float accelZ = imu_gravityCompensatedAccel(axNEDmG, ayNEDmG, azNEDmG, q0, q1, q2, q3);
            
            // Add to ring buffer for averaging (like original)
            ringbuf_addSample(accelZ);
            
            // Kalman filter prediction step
            kalmanFilter4d_predict(dt);
            
            // If we have a new barometric sample, do update step
            if (newBaroSample) {
#ifdef USE_SIMPLE_FUSION
                // Simple complementary filter approach
                static float lastPressureAlt = pressureAltM;
                static unsigned long lastTime = millis();
                unsigned long now = millis();
                float dt_slow = (now - lastTime) / 1000.0f;
                
                if (dt_slow > 0.1f && lastTime > 0) {
                    // Pressure-based climb rate (slow but accurate)
                    float pressureClimbRate = ((pressureAltM - lastPressureAlt) / dt_slow) * 100.0f;
                    
                    // Accelerometer-based climb rate (fast but drifty)
                    static float accelVelocity = 0.0f;
                    accelVelocity += accelZ * dt; // Integrate acceleration to get velocity
                    float accelClimbRate = accelVelocity * 100.0f;
                    
                    // Complementary filter: 80% pressure, 20% accelerometer for responsiveness
                    fusedClimbRateCmS = 0.8f * pressureClimbRate + 0.2f * accelClimbRate;
                    fusedAltitudeM = pressureAltM; // Use pressure altitude
                    
                    // High-pass filter on accelerometer to prevent drift
                    accelVelocity *= 0.999f; // Slowly decay to prevent drift
                }
                lastPressureAlt = pressureAltM;
                lastTime = now;
#else
                // Full Kalman filter (original approach)
                // Use averaged acceleration from ring buffer like original
                float zAccelAverage = ringbuf_averageNewestSamples(10);
                kalmanFilter4d_update(pressureAltM * 100.0f, zAccelAverage, &fusedAltitudeM, &fusedClimbRateCmS);
                fusedAltitudeM /= 100.0f; // Convert back to meters for display
                // fusedClimbRateCmS is already in cm/s from Kalman filter
#endif
                newBaroSample = false;
            }
        }
    }
    
    // Lower-frequency barometric update (50Hz)
    if (nowUs - lastBaroUpdate >= BARO_INTERVAL_US) {
        lastBaroUpdate = nowUs;
        
        // Check for new barometric sample
        if (ms5611_i2c_sampleStateMachine()) {
            pressureAltM = ZCmSample_MS5611_I2C / 100.0f; // Convert cm to m
            newBaroSample = true;
            
            // Update sensor data structure for Bluetooth transmission
            sensorData.altitudeM = fusedAltitudeM;
            sensorData.climbRateCmS = fusedClimbRateCmS;
            sensorData.pressurePa = PaSample_MS5611_I2C;
            sensorData.temperatureC = CelsiusSample_MS5611_I2C / 100.0f;
            
            // Simple fallback for extreme values
            if (abs(sensorData.climbRateCmS) > 2000) {
                sensorData.climbRateCmS = 0; // Reset extreme values
            }
        }
    }
}

// Legacy sensor reading function - now uses sensor fusion results
void readSensors() {
    // Sensor data is now updated in updateSensorFusion()
    // This function just ensures compatibility with existing code
    
    // Read battery voltage (if connected to ADC pin)
    int adcValue = analogRead(pinADC);
    sensorData.batteryV = (adcValue * 3.3f / 4095.0f) * 2.0f; // Assuming voltage divider
    
    // Clamp climb rate to reasonable values
    if (sensorData.climbRateCmS > 2000.0f) sensorData.climbRateCmS = 2000.0f;
    if (sensorData.climbRateCmS < -2000.0f) sensorData.climbRateCmS = -2000.0f;
}

// I2C Scanner
void scanI2C() {
    Serial.println("Scanning I2C bus...");
    byte error, address;
    int nDevices = 0;
    
    for(address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.printf("I2C device found at address 0x%02X\n", address);
            nDevices++;
        }
    }
    
    if (nDevices == 0) {
        Serial.println("No I2C devices found");
    } else {
        Serial.printf("Found %d I2C devices\n", nDevices);
    }
}

// LED control functions
void setLedMode(uint8_t mode, uint32_t duration_ms) {
    if (mode == BT_MSG_XCTRC) {
        // XCTRC mode: Green
        leds[0] = CRGB::Green;
        Serial.println("LED: Green (XCTRC mode)");
    } else {
        // LK8EX1 mode: White  
        leds[0] = CRGB::White;
        Serial.println("LED: White (LK8EX1 mode)");
    }
    
    FastLED.show();
    ledActive = true;
    ledOffTime = millis() + duration_ms;
}

void updateLed() {
    if (ledActive && millis() >= ledOffTime) {
        // Turn off LED
        leds[0] = CRGB::Black;
        FastLED.show();
        ledActive = false;
        Serial.println("LED: Off");
    }
}

void performCalibration() {
    Serial.println("Starting MPU6050 calibration sequence...");

    // LED sequence: Red -> Yellow -> Green -> Blue (flashing during calibration)
    Serial.println("Calibration countdown...");

    // Red phase - 1 second
    leds[0] = CRGB::Red;
    FastLED.show();
    delay(1000);

    // Yellow phase - 1 second
    leds[0] = CRGB::Yellow;
    FastLED.show();
    delay(1000);

    // Green phase - 1 second
    leds[0] = CRGB::Green;
    FastLED.show();
    delay(1000);

    // Blue flashing during calibration
    Serial.println("Keep device stationary - calibrating...");

    // Flash blue slowly during calibration
    for (int i = 0; i < 10; i++) {
        leds[0] = CRGB::Blue;
        FastLED.show();
        delay(200);
        leds[0] = CRGB::Black;
        FastLED.show();
        delay(200);
    }

    // Perform actual calibration
    mpu6050_calibrateGyro();
    mpu6050_calibrateAccel();

    // Success indication - solid green for 2 seconds
    leds[0] = CRGB::Green;
    FastLED.show();
    delay(2000);

    // Return to normal operation
    leds[0] = CRGB::Black;
    FastLED.show();

    Serial.println("MPU6050 calibration completed!");
}

void setup() {
    Serial.begin(115200);
    delay(2000); // Give time for serial monitor to connect
    Serial.println("\n=== M5Stack Atom Lite + GY-86 Bluetooth Vario ===");
    Serial.println("Starting initialization...");
    
    // Initialize I2C
    Serial.printf("Initializing I2C: SDA=%d, SCL=%d, Freq=%d Hz\n", I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_FREQ_HZ);
    delay(100);
    
    // Scan I2C bus to see what's connected
    scanI2C();
    
    // Initialize button
    pinMode(pinBtn0, INPUT);
    Serial.println("Button initialized");
    
    // Initialize ADC for battery monitoring
    analogReadResolution(12);
    Serial.println("ADC initialized");
    
    // Initialize RGB LED
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50); // Set moderate brightness
    leds[0] = CRGB::Black; // Start with LED off
    FastLED.show();
    Serial.println("RGB LED initialized");
    
    // Initialize Bluetooth
    if (!SerialBT.begin("ESP32-BT-Vario-Atom")) {
        Serial.println("Bluetooth initialization failed!");
        while(1) delay(1000);
    }
    Serial.println("Bluetooth initialized: ESP32-BT-Vario-Atom");
    
    // Initialize advanced sensor fusion system
    if (!initializeSensorFusion()) {
        Serial.println("Failed to initialize sensor fusion!");
        while(1) delay(1000);
    }
    Serial.println("Advanced sensor fusion initialized");

    // Configure Kalman filter with loaded settings
    kalmanFilter4d_configure(kalmanAccelVariance, kalmanAdaptFactor, 0.0f, 0.0f, 0.0f);
    Serial.println("Kalman filter configured with saved settings");
    
    // Initialize GPS using existing sensor module
    Serial.printf("Initializing GPS: RX=%d, TX=%d, Baud=%d\n", pinGpsRXD, pinGpsTXD, GPS_BAUD_RATE);
    if (!gps_config()) {
        Serial.println("GPS initialization failed!");
        while(1) delay(1000);
    }
    Serial.println("GPS initialized with UBX configuration");
    
    // Initialize sensor data
    memset(&sensorData, 0, sizeof(sensorData));
    sensorData.batteryV = 4.0f; // Default value
    
    // Initialize options system with defaults
    opt_setDefaults();
    Serial.println("Configuration system initialized with defaults");

    // Load saved web configuration from NVS
    loadWebConfig();

    // Initialize WiFi Access Point and Web Server
    setupWebServer();
    
    Serial.println("Setup complete - ready for Bluetooth and web connections");
    delay(1000);
}


void readGPS() {
    // Use existing GPS state machine from sensor/gps.cpp
    gps_stateMachine();
    
    // Check for new GPS data and update sensor structure
    if (IsGpsNavUpdated) {
        IsGpsNavUpdated = false;
        lastValidGps = millis();
        
        // Update sensorData from UBX NavPvt structure
        sensorData.gpsValid = (NavPvt.nav.fixType >= 2); // 2D or 3D fix
        sensorData.latitude = NavPvt.nav.latDeg7 / 10000000.0f;
        sensorData.longitude = NavPvt.nav.lonDeg7 / 10000000.0f;
        sensorData.gpsAltitudeM = NavPvt.nav.heightMSLmm / 1000.0f;
        sensorData.year = NavPvt.nav.utcYear;
        sensorData.month = NavPvt.nav.utcMonth;
        sensorData.day = NavPvt.nav.utcDay;
        sensorData.hour = NavPvt.nav.utcHour;
        sensorData.minute = NavPvt.nav.utcMinute;
        sensorData.second = NavPvt.nav.utcSecond;
        
        // Calculate speed and course from velocity components
        float velNorth = NavPvt.nav.velNorthmmps / 1000.0f; // Convert mm/s to m/s
        float velEast = NavPvt.nav.velEastmmps / 1000.0f;
        sensorData.speedKmH = sqrt(velNorth*velNorth + velEast*velEast) * 3.6f; // Convert m/s to km/h
        
        if (sensorData.speedKmH > 1.0f) { // Only calculate course if moving
            sensorData.courseHeadingDeg = atan2(velEast, velNorth) * 180.0f / PI;
            if (sensorData.courseHeadingDeg < 0) sensorData.courseHeadingDeg += 360.0f;
        }
        
        // GPS debug removed to save space
    }
    
    // Report GPS status periodically
    static unsigned long lastGpsReport = 0;
    if (millis() - lastGpsReport > 10000) { // Every 10 seconds
        Serial.printf("GPS: Fix=%d, Satellites=%d, Valid=%s\n", 
                     NavPvt.nav.fixType, NavPvt.nav.numSV, 
                     sensorData.gpsValid ? "YES" : "NO");
        lastGpsReport = millis();
    }
}

// GPS parsing is now handled by the existing sensor/gps.cpp module
// which processes UBX binary data directly

void processSerialCommands() {
    // Removed to save flash space - use web interface instead
}

// Web server setup and handlers
void setupWebServer() {
    // Set up WiFi as Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.printf("Web interface available at: http://%s\n", IP.toString().c_str());
    
    // Define web server routes
    webServer.on("/", handleRoot);
    webServer.on("/status", HTTP_GET, handleGetStatus);
    webServer.on("/setparams", HTTP_POST, handleSetParams);
    
    // Enable CORS for all responses
    webServer.enableCORS(true);
    
    webServer.begin();
    Serial.println("Web server started");
}

void handleRoot() {
    webServer.send(200, "text/html", getWebPageHTML());
}

void handleGetStatus() {
    DynamicJsonDocument doc(1024);
    
    // Current sensor readings
    doc["altitude"] = sensorData.altitudeM;
    doc["climbRate"] = sensorData.climbRateCmS;
    doc["pressure"] = sensorData.pressurePa / 100.0f; // Convert to hPa
    doc["temperature"] = sensorData.temperatureC;
    doc["battery"] = sensorData.batteryV;
    doc["gpsValid"] = sensorData.gpsValid;
    doc["satellites"] = NavPvt.nav.numSV;
    doc["fixType"] = NavPvt.nav.fixType;
    
    // Current tuning parameters
    doc["kalmanAccelVariance"] = kalmanAccelVariance;
    doc["kalmanAdaptFactor"] = kalmanAdaptFactor;
    doc["accelMeasVariance"] = accelMeasVariance;
    doc["accelBiasVariance"] = accelBiasVariance;
    doc["showFusionDebug"] = showFusionDebug;
    doc["btMsgType"] = btMsgType;
    doc["btMsgFreqHz"] = btMsgFreqHz;
    doc["useBaroOnly"] = opt.misc.useBaroOnly;
    
    String response;
    serializeJson(doc, response);
    webServer.send(200, "application/json", response);
}

void handleSetParams() {
    if (webServer.hasArg("plain")) {
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, webServer.arg("plain"));
        
        // Update parameters if provided
        if (doc.containsKey("kalmanAccelVariance")) {
            kalmanAccelVariance = doc["kalmanAccelVariance"];
            Serial.printf("Web: Accel Variance set to: %.1f\n", kalmanAccelVariance);
        }
        if (doc.containsKey("kalmanAdaptFactor")) {
            kalmanAdaptFactor = doc["kalmanAdaptFactor"];
            Serial.printf("Web: Adapt Factor set to: %.1f\n", kalmanAdaptFactor);
        }
        if (doc.containsKey("accelMeasVariance")) {
            accelMeasVariance = doc["accelMeasVariance"];
            Serial.printf("Web: Accel Meas Variance set to: %.1f\n", accelMeasVariance);
        }
        if (doc.containsKey("accelBiasVariance")) {
            accelBiasVariance = doc["accelBiasVariance"];
            Serial.printf("Web: Accel Bias Variance set to: %.3f\n", accelBiasVariance);
        }
        if (doc.containsKey("showFusionDebug")) {
            showFusionDebug = doc["showFusionDebug"];
            Serial.printf("Web: Fusion debug: %s\n", showFusionDebug ? "ON" : "OFF");
        }
        if (doc.containsKey("btMsgType")) {
            btMsgType = doc["btMsgType"];
            Serial.printf("Web: Bluetooth message type: %s\n", (btMsgType == BT_MSG_LK8EX1) ? "LK8EX1" : "XCTRC");
            setLedMode(btMsgType, 3000); // Show LED for 3 seconds
        }
        if (doc.containsKey("btMsgFreqHz")) {
            btMsgFreqHz = doc["btMsgFreqHz"];
            BT_INTERVAL_MS = 1000 / btMsgFreqHz; // Update interval based on frequency
            Serial.printf("Web: Bluetooth frequency: %d Hz (interval: %lu ms)\n", btMsgFreqHz, BT_INTERVAL_MS);
        }
        if (doc.containsKey("useBaroOnly")) {
            opt.misc.useBaroOnly = doc["useBaroOnly"];
            Serial.printf("Web: Barometer-only mode: %s\n", opt.misc.useBaroOnly ? "ON" : "OFF");
        }
        
        // Reconfigure Kalman filter with new parameters if they were changed
        if (doc.containsKey("kalmanAccelVariance") || doc.containsKey("kalmanAdaptFactor")) {
            kalmanFilter4d_configure(kalmanAccelVariance, kalmanAdaptFactor,
                                    fusedAltitudeM * 100.0f, fusedClimbRateCmS, 0.0f);
            Serial.println("Web: Kalman filter reconfigured");
        }

        // Save all settings to persistent storage
        saveWebConfig();

        webServer.send(200, "application/json", "{\"status\":\"ok\"}");
    } else {
        webServer.send(400, "application/json", "{\"error\":\"No data provided\"}");
    }
}

String getWebPageHTML() {
    return R"HTML(<!DOCTYPE html><html><head><title>Vario Config</title><meta name="viewport" content="width=device-width,initial-scale=1"><style>body{font-family:Arial;margin:10px;background:#f0f0f0}h1{text-align:center}div{margin:10px 0}label{display:inline-block;width:150px;font-weight:bold}input,select{margin:5px}button{padding:8px 15px;margin:5px;cursor:pointer}.status{background:#f8f9fa;padding:10px;border-radius:5px;margin:5px}</style></head><body><h1>Vario Config</h1><div id="status">Connecting...</div><h3>Status</h3><div class="status">Alt: <span id="altitude">--</span>m | Climb: <span id="climbRate">--</span>cm/s | GPS: <span id="gps">--</span></div><h3>Tuning</h3><p style="font-size:14px;color:#666;margin:5px 0;">Quick presets for different flying conditions:</p><button onclick="applyPreset('thermals')">Thermals</button><button onclick="applyPreset('ridge')">Ridge</button><button onclick="applyPreset('smooth')">Smooth</button><p style="font-size:12px;color:#666;margin:5px 0;"><strong>Thermals:</strong> Fast response, sensitive | <strong>Ridge:</strong> Balanced | <strong>Smooth:</strong> Stable, less sensitive to movement</p><div><label>Accel Variance:</label><input type="range" id="kalmanAccelVariance" min="5" max="100" step="1" oninput="updateNumberInput(this)"><input type="number" id="kalmanAccelVarianceNum" min="5" max="100" step="1" style="width:60px"><br><small style="color:#666;margin-left:155px;">< More Accelerometer Trust | Less Accelerometer Trust ></small></div><div><label>Adapt Factor:</label><input type="range" id="kalmanAdaptFactor" min="0.5" max="10" step="0.1" oninput="updateNumberInput(this)"><input type="number" id="kalmanAdaptFactorNum" min="0.5" max="10" step="0.1" style="width:60px"><br><small style="color:#666;margin-left:155px;">< Slower Response | Faster Response ></small></div><div><label>Accel Meas Var:</label><input type="range" id="accelMeasVariance" min="0.5" max="10" step="0.1" oninput="updateNumberInput(this)"><input type="number" id="accelMeasVarianceNum" min="0.5" max="10" step="0.1" style="width:60px"><br><small style="color:#666;margin-left:155px;">< Trust Accelerometer | Trust Barometer ></small></div><div><label>Accel Bias Var:</label><input type="range" id="accelBiasVariance" min="0.001" max="0.1" step="0.001" oninput="updateNumberInput(this)"><input type="number" id="accelBiasVarianceNum" min="0.001" max="0.1" step="0.001" style="width:60px"><br><small style="color:#666;margin-left:155px;">< Slow Bias Correction | Fast Bias Correction ></small></div><h3>Bluetooth</h3><div><label>Message Type:</label><select id="btMsgType"><option value="0">LK8EX1</option><option value="1">XCTRC</option></select></div><div><label>Frequency:</label><input type="range" id="btMsgFreqHz" min="1" max="10" step="1" oninput="updateNumberInput(this)"><input type="number" id="btMsgFreqHzNum" min="1" max="10" step="1" style="width:60px">Hz</div><div><label>Barometer Only:</label><input type="checkbox" id="useBaroOnly"> Send raw baro data (let phone app calculate climb rate)<br><small style="color:#666;margin-left:155px;">✓ Recommended for handheld testing and stable flight data</small></div><div><label>Debug:</label><input type="checkbox" id="showFusionDebug"> Show fusion debug messages in serial monitor</div><div style="text-align:center"><button onclick="applySettings()" id="applyBtn">Apply</button><button onclick="refreshStatus()">Refresh</button></div><script>function updateNumberInput(s){document.getElementById(s.id+'Num').value=s.value}function applyPreset(p){var v={thermals:{av:10,af:5,am:1,ab:0.02},ridge:{av:25,af:3,am:2,ab:0.02},smooth:{av:50,af:1,am:5,ab:0.01}}[p];document.getElementById('kalmanAccelVariance').value=v.av;document.getElementById('kalmanAccelVarianceNum').value=v.av;document.getElementById('kalmanAdaptFactor').value=v.af;document.getElementById('kalmanAdaptFactorNum').value=v.af;document.getElementById('accelMeasVariance').value=v.am;document.getElementById('accelMeasVarianceNum').value=v.am;document.getElementById('accelBiasVariance').value=v.ab;document.getElementById('accelBiasVarianceNum').value=v.ab}function refreshStatus(){fetch('/status').then(function(r){return r.json()}).then(function(d){document.getElementById('altitude').textContent=d.altitude.toFixed(1);document.getElementById('climbRate').textContent=d.climbRate.toFixed(0);document.getElementById('gps').textContent=d.gpsValid?'Fix('+d.fixType+')':'No Fix';document.getElementById('status').textContent='Connected'}).catch(function(){document.getElementById('status').textContent='Error'})}function loadAllSettings(){fetch('/status').then(function(r){return r.json()}).then(function(d){document.getElementById('altitude').textContent=d.altitude.toFixed(1);document.getElementById('climbRate').textContent=d.climbRate.toFixed(0);document.getElementById('gps').textContent=d.gpsValid?'Fix('+d.fixType+')':'No Fix';document.getElementById('kalmanAccelVariance').value=d.kalmanAccelVariance;document.getElementById('kalmanAccelVarianceNum').value=d.kalmanAccelVariance;document.getElementById('kalmanAdaptFactor').value=d.kalmanAdaptFactor;document.getElementById('kalmanAdaptFactorNum').value=d.kalmanAdaptFactor;document.getElementById('accelMeasVariance').value=d.accelMeasVariance;document.getElementById('accelMeasVarianceNum').value=d.accelMeasVariance;document.getElementById('accelBiasVariance').value=d.accelBiasVariance;document.getElementById('accelBiasVarianceNum').value=d.accelBiasVariance;document.getElementById('showFusionDebug').checked=d.showFusionDebug;document.getElementById('btMsgType').value=d.btMsgType;document.getElementById('btMsgFreqHz').value=d.btMsgFreqHz;document.getElementById('btMsgFreqHzNum').value=d.btMsgFreqHz;document.getElementById('useBaroOnly').checked=d.useBaroOnly;document.getElementById('status').textContent='Connected'}).catch(function(){document.getElementById('status').textContent='Error'})}function applySettings(){var s={kalmanAccelVariance:parseFloat(document.getElementById('kalmanAccelVariance').value),kalmanAdaptFactor:parseFloat(document.getElementById('kalmanAdaptFactor').value),accelMeasVariance:parseFloat(document.getElementById('accelMeasVariance').value),accelBiasVariance:parseFloat(document.getElementById('accelBiasVariance').value),showFusionDebug:document.getElementById('showFusionDebug').checked,btMsgType:parseInt(document.getElementById('btMsgType').value),btMsgFreqHz:parseInt(document.getElementById('btMsgFreqHz').value),useBaroOnly:document.getElementById('useBaroOnly').checked};document.getElementById('applyBtn').textContent='Applying...';fetch('/setparams',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)}).then(function(){document.getElementById('applyBtn').textContent='Apply'}).catch(function(){document.getElementById('applyBtn').textContent='Apply'})}setInterval(refreshStatus,2000);loadAllSettings()</script></body></html>)HTML";
}

void transmitBluetooth() {
    char message[300];
    
    if (btMsgType == BT_MSG_LK8EX1) {
        // LK8EX1 format: pressure, altitude, vario, temperature, battery
        int32_t altM = (int32_t)sensorData.altitudeM;
        int32_t cps = (int32_t)sensorData.climbRateCmS;
        
        sprintf(message, "$LK8EX1,999999,%d,%d,%.0f,%.1f*", 
                altM, cps, sensorData.temperatureC, sensorData.batteryV);
        
        // Calculate and append checksum
        uint8_t checksum = 0;
        for (int i = 1; message[i] != '*'; i++) {
            checksum ^= message[i];
        }
        char checksumStr[10];
        sprintf(checksumStr, "%02X\r\n", checksum);
        strcat(message, checksumStr);
        
        SerialBT.print(message);
        DEBUG_LOG("BT: %s", message);
        
    } else {
        // Send multiple NMEA sentences for better XCTrack compatibility
        
        // 1. Send GGA sentence (position and altitude)
        if (sensorData.gpsValid) {
            // Convert decimal degrees to DDMM.MMMM format
            float latDeg = abs(sensorData.latitude);
            float lonDeg = abs(sensorData.longitude);
            int latD = (int)latDeg;
            int lonD = (int)lonDeg;
            float latM = (latDeg - latD) * 60.0f;
            float lonM = (lonDeg - lonD) * 60.0f;
            
            sprintf(message, "$GNGGA,%02d%02d%02d,%02d%07.4f,%c,%03d%07.4f,%c,1,8,1.0,%.1f,M,0.0,M,,*",
                    sensorData.hour, sensorData.minute, sensorData.second,
                    latD, latM, (sensorData.latitude >= 0) ? 'N' : 'S',
                    lonD, lonM, (sensorData.longitude >= 0) ? 'E' : 'W',
                    sensorData.gpsAltitudeM);
                    
            uint8_t checksum = 0;
            for (int i = 1; message[i] != '*'; i++) {
                checksum ^= message[i];
            }
            char checksumStr[10];
            sprintf(checksumStr, "%02X\r\n", checksum);
            strcat(message, checksumStr);
            SerialBT.print(message);
            //Serial Monitor Output of Bluetooth messages for debugging
            // Serial.print("BT: ");
            // Serial.print(message);
            
            // 2. Send RMC sentence (time, date, speed, course)
            sprintf(message, "$GNRMC,%02d%02d%02d,A,%02d%07.4f,%c,%03d%07.4f,%c,%.2f,%.1f,%02d%02d%02d,,,A*",
                    sensorData.hour, sensorData.minute, sensorData.second,
                    latD, latM, (sensorData.latitude >= 0) ? 'N' : 'S',
                    lonD, lonM, (sensorData.longitude >= 0) ? 'E' : 'W',
                    sensorData.speedKmH * 0.539957f, // Convert km/h to knots
                    sensorData.courseHeadingDeg,
                    sensorData.day, sensorData.month, sensorData.year % 100);
                    
            checksum = 0;
            for (int i = 1; message[i] != '*'; i++) {
                checksum ^= message[i];
            }
            sprintf(checksumStr, "%02X\r\n", checksum);
            strcat(message, checksumStr);
            SerialBT.print(message);
            //Serial Monitor Output of Bluetooth messages for debugging
            // Serial.print("BT: ");
            // Serial.print(message);
        }
        
        // 3. Always send LK8EX1 for vario data (XCTrack loves this)
        int32_t altM = (int32_t)sensorData.altitudeM;
        int32_t cps = (int32_t)sensorData.climbRateCmS;
        int32_t batteryPercent = (int32_t)((sensorData.batteryV / 5.0f) * 100.0f);
        if (batteryPercent > 100) batteryPercent = 100;
        if (batteryPercent < 0) batteryPercent = 0;
        
        sprintf(message, "$LK8EX1,%.0f,%d,%d,%.0f,%d*", 
                sensorData.pressurePa, altM, cps, sensorData.temperatureC, batteryPercent);
        
        // Debug output removed to save space
                
        uint8_t checksum = 0;
        for (int i = 1; message[i] != '*'; i++) {
            checksum ^= message[i];
        }
        char checksumStr[10];
        sprintf(checksumStr, "%02X\r\n", checksum);
        strcat(message, checksumStr);
        SerialBT.print(message);
        //Serial Monitor Output of Bluetooth messages for debugging
        // Serial.print("BT: ");
        // Serial.print(message);
    }
}

void loop() {
    unsigned long now = millis();
    
    // High-frequency sensor fusion (runs as fast as possible)
    updateSensorFusion();
    
    // Read sensors at 10Hz (for battery monitoring)
    if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
        readSensors();
        lastSensorRead = now;
    }
    
    // Read GPS at 10Hz
    if (now - lastGpsRead >= GPS_INTERVAL_MS) {
        readGPS();
        lastGpsRead = now;
    }
    
    // Transmit Bluetooth at configured rate
    if (now - lastBtTransmit >= BT_INTERVAL_MS) {
        transmitBluetooth();
        lastBtTransmit = now;
    }
    
    // Process serial commands for real-time tuning
    processSerialCommands();
    
    // Handle web server requests
    webServer.handleClient();
    
    // Check button for calibration (long press) or mode switching (short press)
    static unsigned long buttonPressStart = 0;
    static bool buttonWasPressed = false;

    if (BTN0()) {
        if (!buttonWasPressed) {
            buttonPressStart = millis();
            buttonWasPressed = true;
        } else {
            // Check for long press (4 seconds)
            if (millis() - buttonPressStart >= 4000) {
                Serial.println("Long press detected - starting calibration");
                performCalibration();
                // Wait for button release
                while (BTN0()) delay(10);
                buttonWasPressed = false;
                return; // Skip short press handling
            }
        }
    } else {
        // Button released
        if (buttonWasPressed) {
            unsigned long pressDuration = millis() - buttonPressStart;
            if (pressDuration >= 50 && pressDuration < 4000) {
                // Short press - switch BT message type
                btMsgType = (btMsgType == BT_MSG_LK8EX1) ? BT_MSG_XCTRC : BT_MSG_LK8EX1;
                Serial.print("Switched to ");
                Serial.println((btMsgType == BT_MSG_LK8EX1) ? "LK8EX1" : "XCTRC");

                // Show LED indication for 5 seconds
                setLedMode(btMsgType, 5000);
            }
            buttonWasPressed = false;
        }
    }
    
    // Update LED state
    updateLed();
    
    // Print status every 5 seconds
    static unsigned long lastStatus = 0;
    if (now - lastStatus >= 5000) {
        Serial.printf("Alt: %.1fm, ClimbRate: %.1fcm/s, Press: %.1fhPa, GPS: %s, Fusion: %s\n",
                     sensorData.altitudeM, sensorData.climbRateCmS, 
                     sensorData.pressurePa / 100.0f, 
                     sensorData.gpsValid ? "Valid" : "Invalid",
                     sensorFusionInitialized ? "Active" : "Disabled");
        lastStatus = now;
    }
    
    // No delay - let sensor fusion run at maximum frequency
}