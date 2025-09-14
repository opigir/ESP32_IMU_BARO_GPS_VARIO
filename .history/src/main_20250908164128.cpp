#include <Arduino.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <FastLED.h>

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

// Debug logging macros - now runtime controlled
#ifdef DEBUG_FUSION_ONLY
  #define DEBUG_LOG(fmt, ...) // Disable regular debug logs
  #define FUSION_LOG(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#elif defined(DEBUG_ALL_LOGS)
  #define DEBUG_LOG(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
  #define FUSION_LOG(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
  #define DEBUG_LOG(fmt, ...) // Disable regular debug logs by default
  #define FUSION_LOG(fmt, ...) if(showFusionDebug) Serial.printf(fmt, ##__VA_ARGS__)
#endif

// Sensor fusion configuration
#define IMU_SAMPLE_RATE_HZ      500   // High-frequency IMU sampling
#define BARO_SAMPLE_RATE_HZ     50    // Pressure sensor sampling

// Runtime tunable parameters - can be changed via serial commands
float kalmanAccelVariance = 25.0f;  // Start more aggressive (was 50.0f)
float kalmanAdaptFactor = 3.0f;     // More aggressive adaptation (was 2.0f)
float accelMeasVariance = 2.0f;     // Trust accelerometer much more (was 5.0f)
float accelBiasVariance = 0.02f;    // Allow faster bias adaptation (was 0.01f)

// Runtime debug control
bool showFusionDebug = false;       // Toggle fusion debug output

// RGB LED configuration for M5Stack Atom Lite
#define NUM_LEDS    1
#define LED_PIN     27
CRGB leds[NUM_LEDS];

// LED timing variables
unsigned long ledOffTime = 0;
bool ledActive = false;

// Global variables
BluetoothSerial SerialBT;

// Forward declarations
void parseGGA(String sentence);
void parseRMC(String sentence);
float parseCoordinate(String coord, String dir);
void setLedMode(uint8_t mode, uint32_t duration_ms);
void updateLed();

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

// Timing variables
unsigned long lastBtTransmit = 0;
unsigned long lastSensorRead = 0;
unsigned long lastGpsRead = 0;
const unsigned long BT_INTERVAL_MS = 200;      // 5Hz Bluetooth transmission
const unsigned long SENSOR_INTERVAL_MS = 100; // 10Hz sensor reading
const unsigned long GPS_INTERVAL_MS = 100;    // Check GPS at 10Hz

// Configuration
uint8_t btMsgType = BT_MSG_XCTRC;  // Default to XCTRC for full GPS integration
uint8_t btMsgFreqHz = 5;           // 5Hz transmission rate

// High-frequency sensor fusion variables
float fusedAltitudeM = 0.0f;
float fusedClimbRateCmS = 0.0f;
bool sensorFusionInitialized = false;

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
    
    // Calibrate sensors
    Serial.println("Calibrating MPU6050 (keep device still)...");
    mpu6050_calibrateGyro();
    mpu6050_calibrateAccel();
    
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
    
    // Configure Kalman filter with runtime tunable values
    kalmanFilter4d_configure(kalmanAccelVariance, kalmanAdaptFactor, 
                            initialAltitude, 0.0f, 0.0f);
    
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
            // Update IMU orientation filter
            float dt = IMU_INTERVAL_US / 1000000.0f;
            imu_mahonyAHRSupdate6DOF(1, dt, gx, gy, gz, ax, ay, az);
            
            // Calculate gravity-compensated acceleration (INVERT the result for correct orientation)
            float accelZ = -imu_gravityCompensatedAccel(ax, ay, az, q0, q1, q2, q3);
            
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
                kalmanFilter4d_update(pressureAltM, accelZ, &fusedAltitudeM, &fusedClimbRateCmS);
                fusedClimbRateCmS *= 100.0f; // Convert m/s to cm/s
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
            
            // TEMPORARY DEBUG: Check if fusion is giving reasonable values
            if (abs(sensorData.climbRateCmS) > 2000) {
                // Fusion giving extreme values, fall back to simple pressure-only calculation
                static float lastPressureAlt = pressureAltM;
                static unsigned long lastPressureTime = millis();
                unsigned long now = millis();
                float dt = (now - lastPressureTime) / 1000.0f;
                if (dt > 0.1f && lastPressureTime > 0) {
                    float simpleClimbRate = ((pressureAltM - lastPressureAlt) / dt) * 100.0f; // cm/s
                    FUSION_LOG("[FALLBACK] Fusion=%.0fcm/s, Simple=%.0fcm/s, using simple\n", 
                              sensorData.climbRateCmS, simpleClimbRate);
                    sensorData.climbRateCmS = simpleClimbRate;
                    sensorData.altitudeM = pressureAltM;
                }
                lastPressureAlt = pressureAltM;
                lastPressureTime = now;
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
    
    // Initialize GPS UART for BZ-121 GPS
    Serial.printf("Initializing GPS UART: RX=%d, TX=%d, Baud=%d\n", pinGpsRXD, pinGpsTXD, GPS_BAUD_RATE);
    Serial1.begin(GPS_BAUD_RATE, SERIAL_8N1, pinGpsRXD, pinGpsTXD);
    Serial.println("BZ-121 GPS UART initialized");
    
    // Initialize sensor data
    memset(&sensorData, 0, sizeof(sensorData));
    sensorData.batteryV = 4.0f; // Default value
    
    Serial.println("Setup complete - ready for Bluetooth connection");
    delay(1000);
}


void readGPS() {
    static String gpsBuffer = "";
    static unsigned long lastGpsData = 0;
    static int gpsCharCount = 0;
    
    while (Serial1.available()) {
        char c = Serial1.read();
        gpsBuffer += c;
        gpsCharCount++;
        lastGpsData = millis();
        
        if (c == '\n') {
            // Show all GPS data for debugging
            DEBUG_LOG("GPS: %s", gpsBuffer.c_str());
            
            // Process complete NMEA sentence
            // BZ-121 supports multi-constellation: GP (GPS), GL (GLONASS), BD (BeiDou), GA (Galileo), GN (combined)
            if (gpsBuffer.startsWith("$GPGGA") || gpsBuffer.startsWith("$GNGGA") || 
                gpsBuffer.startsWith("$GLGGA") || gpsBuffer.startsWith("$BDGGA") || gpsBuffer.startsWith("$GAGGA")) {
                parseGGA(gpsBuffer);
            } else if (gpsBuffer.startsWith("$GPRMC") || gpsBuffer.startsWith("$GNRMC") || 
                       gpsBuffer.startsWith("$GLRMC") || gpsBuffer.startsWith("$BDRMC") || gpsBuffer.startsWith("$GARMC")) {
                parseRMC(gpsBuffer);
            }
            gpsBuffer = "";
        }
        
        // Prevent buffer overflow
        if (gpsBuffer.length() > 200) {
            Serial.println("GPS buffer overflow, clearing");
            gpsBuffer = "";
        }
    }
    
    // Report GPS status periodically
    static unsigned long lastGpsReport = 0;
    if (millis() - lastGpsReport > 30000) { // Every 30 seconds
        if (gpsCharCount > 0) {
            Serial.printf("GPS: Received %d characters, last data %lu ms ago\n", 
                         gpsCharCount, millis() - lastGpsData);
            gpsCharCount = 0;
        } else {
            Serial.println("GPS: No data received - check wiring");
        }
        lastGpsReport = millis();
    }
}

void parseGGA(String sentence) {
    // Simple GGA parser for position and altitude
    int commaPos[15];
    int commaCount = 0;
    
    for (int i = 0; i < sentence.length() && commaCount < 15; i++) {
        if (sentence[i] == ',') {
            commaPos[commaCount++] = i;
        }
    }
    
    if (commaCount >= 9) {
        // Extract latitude
        String latStr = sentence.substring(commaPos[1] + 1, commaPos[2]);
        String latDir = sentence.substring(commaPos[2] + 1, commaPos[3]);
        
        // Extract longitude
        String lonStr = sentence.substring(commaPos[3] + 1, commaPos[4]);
        String lonDir = sentence.substring(commaPos[4] + 1, commaPos[5]);
        
        // Extract fix quality
        String fixQuality = sentence.substring(commaPos[5] + 1, commaPos[6]);
        
        // Extract altitude
        String altStr = sentence.substring(commaPos[8] + 1, commaPos[9]);
        
        if (latStr.length() > 0 && lonStr.length() > 0 && fixQuality.toInt() > 0) {
            sensorData.latitude = parseCoordinate(latStr, latDir);
            sensorData.longitude = parseCoordinate(lonStr, lonDir);
            sensorData.gpsAltitudeM = altStr.toFloat();
            sensorData.gpsValid = true;
        }
    }
}

void parseRMC(String sentence) {
    // Simple RMC parser for time, date, speed, and course
    int commaPos[15];
    int commaCount = 0;
    
    for (int i = 0; i < sentence.length() && commaCount < 15; i++) {
        if (sentence[i] == ',') {
            commaPos[commaCount++] = i;
        }
    }
    
    if (commaCount >= 9) {
        // Extract time
        String timeStr = sentence.substring(commaPos[0] + 1, commaPos[1]);
        if (timeStr.length() >= 6) {
            sensorData.hour = timeStr.substring(0, 2).toInt();
            sensorData.minute = timeStr.substring(2, 4).toInt();
            sensorData.second = timeStr.substring(4, 6).toInt();
        }
        
        // Extract date
        String dateStr = sentence.substring(commaPos[8] + 1, commaPos[9]);
        if (dateStr.length() >= 6) {
            sensorData.day = dateStr.substring(0, 2).toInt();
            sensorData.month = dateStr.substring(2, 4).toInt();
            sensorData.year = 2000 + dateStr.substring(4, 6).toInt();
        }
        
        // Extract speed
        String speedStr = sentence.substring(commaPos[6] + 1, commaPos[7]);
        if (speedStr.length() > 0) {
            sensorData.speedKmH = speedStr.toFloat() * 1.852f; // Convert knots to km/h
        }
        
        // Extract course
        String courseStr = sentence.substring(commaPos[7] + 1, commaPos[8]);
        if (courseStr.length() > 0) {
            sensorData.courseHeadingDeg = courseStr.toFloat();
        }
    }
}

float parseCoordinate(String coord, String dir) {
    if (coord.length() < 4) return 0.0f;
    
    float degrees = coord.substring(0, coord.indexOf('.') - 2).toFloat();
    float minutes = coord.substring(coord.indexOf('.') - 2).toFloat();
    
    float result = degrees + minutes / 60.0f;
    
    if (dir == "S" || dir == "W") {
        result = -result;
    }
    
    return result;
}

void processSerialCommands() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd.startsWith("av ")) {
            // Accel Variance: av 25.0
            kalmanAccelVariance = cmd.substring(3).toFloat();
            Serial.printf("Accel Variance set to: %.1f\n", kalmanAccelVariance);
        }
        else if (cmd.startsWith("af ")) {
            // Adapt Factor: af 3.0
            kalmanAdaptFactor = cmd.substring(3).toFloat();
            Serial.printf("Adapt Factor set to: %.1f\n", kalmanAdaptFactor);
        }
        else if (cmd.startsWith("am ")) {
            // Accel Meas variance: am 2.0
            accelMeasVariance = cmd.substring(3).toFloat();
            Serial.printf("Accel Meas Variance set to: %.1f\n", accelMeasVariance);
        }
        else if (cmd.startsWith("ab ")) {
            // Accel Bias variance: ab 0.02
            accelBiasVariance = cmd.substring(3).toFloat();
            Serial.printf("Accel Bias Variance set to: %.3f\n", accelBiasVariance);
        }
        else if (cmd == "debug") {
            // Toggle debug output
            showFusionDebug = !showFusionDebug;
            Serial.printf("Fusion debug output: %s\n", showFusionDebug ? "ON" : "OFF");
        }
        else if (cmd == "status") {
            // Show current climb rate without all the debug noise
            Serial.printf("Current ClimbRate: %.0f cm/s, Alt: %.1f m\n", 
                         sensorData.climbRateCmS, sensorData.altitudeM);
        }
        else if (cmd == "show") {
            // Show current settings
            Serial.printf("\n=== CURRENT SETTINGS ===\n");
            Serial.printf("  Accel Variance (av): %.1f\n", kalmanAccelVariance);
            Serial.printf("  Adapt Factor (af): %.1f\n", kalmanAdaptFactor);
            Serial.printf("  Accel Meas Var (am): %.1f\n", accelMeasVariance);
            Serial.printf("  Accel Bias Var (ab): %.3f\n", accelBiasVariance);
            Serial.printf("  Debug Output: %s\n", showFusionDebug ? "ON" : "OFF");
            Serial.printf("=======================\n\n");
        }
        else if (cmd == "help") {
            Serial.println("\n=== VARIO TUNING COMMANDS ===");
            Serial.println("Tuning Parameters:");
            Serial.println("  av X.X  - Accel variance (lower = more responsive, try 5-50)");
            Serial.println("  af X.X  - Adapt factor (higher = faster, try 1-10)");
            Serial.println("  am X.X  - Accel meas variance (lower = trust accel more, try 0.5-10)");
            Serial.println("  ab X.XX - Accel bias variance (higher = adapt faster, try 0.01-0.1)");
            Serial.println();
            Serial.println("Utility Commands:");
            Serial.println("  show    - Show current settings");
            Serial.println("  status  - Show current climb rate (clean output)");
            Serial.println("  debug   - Toggle debug output on/off");
            Serial.println("  help    - Show this help");
            Serial.println();
            Serial.println("Quick Presets:");
            Serial.println("  SUPER FAST:  av 5 af 8 am 0.5");
            Serial.println("  FAST:        av 10 af 5 am 1");
            Serial.println("  BALANCED:    av 25 af 3 am 2");
            Serial.println("  SMOOTH:      av 50 af 1 am 5");
            Serial.println("=============================\n");
        }
        else if (cmd != "") {
            Serial.println("Unknown command. Type 'help' for commands.");
        }
    }
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
            Serial.print("BT: ");
            Serial.print(message);
            
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
            Serial.print("BT: ");
            Serial.print(message);
        }
        
        // 3. Always send LK8EX1 for vario data (XCTrack loves this)
        int32_t altM = (int32_t)sensorData.altitudeM;
        int32_t cps = (int32_t)sensorData.climbRateCmS;
        int32_t batteryPercent = (int32_t)((sensorData.batteryV / 5.0f) * 100.0f);
        if (batteryPercent > 100) batteryPercent = 100;
        if (batteryPercent < 0) batteryPercent = 0;
        
        sprintf(message, "$LK8EX1,%.0f,%d,%d,%.0f,%d*", 
                sensorData.pressurePa, altM, cps, sensorData.temperatureC, batteryPercent);
        
        // Debug output to confirm fusion data  
        FUSION_LOG("[FUSION] Alt=%.1fm, ClimbRate=%dcm/s, Pressure=%.0fPa\n", 
                   sensorData.altitudeM, cps, sensorData.pressurePa);
        
        // Additional debug - show raw sensor values
        static int debugCounter = 0;
        if (debugCounter++ % 25 == 0) { // Every 5 seconds
            FUSION_LOG("[DEBUG] PressureAlt=%.1fm, FusedAlt=%.1fm, IMU_active=%d\n", 
                       pressureAltM, fusedAltitudeM, sensorFusionInitialized);
        }
                
        uint8_t checksum = 0;
        for (int i = 1; message[i] != '*'; i++) {
            checksum ^= message[i];
        }
        char checksumStr[10];
        sprintf(checksumStr, "%02X\r\n", checksum);
        strcat(message, checksumStr);
        SerialBT.print(message);
        Serial.print("BT: ");
        Serial.print(message);
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
    
    // Check button for mode switching
    if (BTN0()) {
        delay(50); // Debounce
        if (BTN0()) {
            btMsgType = (btMsgType == BT_MSG_LK8EX1) ? BT_MSG_XCTRC : BT_MSG_LK8EX1;
            Serial.print("Switched to ");
            Serial.println((btMsgType == BT_MSG_LK8EX1) ? "LK8EX1" : "XCTRC");
            
            // Show LED indication for 5 seconds
            setLedMode(btMsgType, 5000);
            
            while (BTN0()) delay(10); // Wait for release
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