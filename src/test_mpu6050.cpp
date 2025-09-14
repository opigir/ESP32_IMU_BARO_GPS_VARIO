#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>

#ifdef ATOM_LITE_BUILD
#include "config_atom.h"
#else
#include "config.h"
#endif

#include "sensor/mpu6050.h"

// RGB LED for M5Stack Atom Lite
#define NUM_LEDS    1
#define LED_PIN     27
CRGB leds[NUM_LEDS];

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

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== MPU6050 Test Program ===");
    
    // Initialize I2C
    Serial.printf("Initializing I2C: SDA=%d, SCL=%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_FREQ_HZ);
    delay(100);
    
    // Scan for devices
    scanI2C();
    
    // Initialize LED
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
    leds[0] = CRGB::Blue;
    FastLED.show();
    
    // Initialize MPU6050
    Serial.println("\nInitializing MPU6050...");
    if (mpu6050_config() != 0) {
        Serial.println("ERROR: MPU6050 failed to initialize!");
        leds[0] = CRGB::Red;
        FastLED.show();
        while(1) {
            Serial.println("Check wiring: SDA=25, SCL=21, VCC=3.3V, GND=GND");
            delay(2000);
        }
    }
    
    Serial.println("MPU6050 initialized successfully!");
    
    // Calibrate
    Serial.println("Calibrating - keep device still...");
    leds[0] = CRGB::Yellow;
    FastLED.show();
    
    mpu6050_calibrateGyro();
    mpu6050_calibrateAccel();
    
    Serial.println("Calibration complete!");
    leds[0] = CRGB::Green;
    FastLED.show();
    delay(1000);
    leds[0] = CRGB::Black;
    FastLED.show();
    
    Serial.println("\n=== Commands ===");
    Serial.println("'test' - show test instructions");
    Serial.println("'calib' - recalibrate");
    Serial.println("================\n");
}

void loop() {
    static unsigned long lastRead = 0;
    unsigned long now = millis();
    
    // Read MPU6050 every 100ms
    if (now - lastRead >= 100) {
        float gx, gy, gz, ax, ay, az;
        
        if (mpu6050_getGyroAccelData(&gx, &gy, &gz, &ax, &ay, &az) == 0) {
            Serial.printf("Gyro(°/s): X=%6.2f Y=%6.2f Z=%6.2f | Accel(g): X=%6.3f Y=%6.3f Z=%6.3f",
                         gx, gy, gz, ax, ay, az);
            
            // Calculate total acceleration magnitude
            float accelMag = sqrt(ax*ax + ay*ay + az*az);
            Serial.printf(" | Mag=%.3fg", accelMag);
            
            // Simple movement detection
            if (abs(gx) > 20 || abs(gy) > 20 || abs(gz) > 20) {
                Serial.print(" [ROTATION]");
            }
            if (accelMag > 1.2f || accelMag < 0.8f) {
                Serial.print(" [MOVEMENT]");
            }
            
            Serial.println();
        } else {
            Serial.println("Failed to read MPU6050!");
        }
        
        lastRead = now;
    }
    
    // Handle commands
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd == "test") {
            Serial.println("\n=== MPU6050 Test Instructions ===");
            Serial.println("Watch the values change as you:");
            Serial.println("1. Tilt device left/right (X accel changes)");
            Serial.println("2. Tilt device forward/back (Y accel changes)");
            Serial.println("3. Rotate around vertical axis (Z gyro changes)");
            Serial.println("4. When still, accel magnitude should be ~1.0g");
            Serial.println("==================================\n");
        }
        else if (cmd == "calib") {
            Serial.println("Recalibrating - keep still...");
            mpu6050_calibrateGyro();
            mpu6050_calibrateAccel();
            Serial.println("Calibration done!");
        }
    }
    
    delay(10);
}