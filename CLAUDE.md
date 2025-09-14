# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ESP32-based GPS Altimeter Variometer for aviation/gliding applications. It combines IMU (accelerometer, gyroscope, magnetometer), barometric pressure sensor, and GPS data using a Kalman filter to provide zero-lag variometer functionality with audio feedback.

## Build System and Commands

### PlatformIO Build Environment
- Uses Visual Studio Code + PlatformIO plugin with ESP32 Arduino framework
- Platform: `espressif32` with `esp32dev` board configuration
- Build configuration in `platformio.ini`

### Essential Build Commands
```bash
# Clean build
pio run -t clean

# Build firmware
pio run

# Build filesystem image (from /data directory)
pio run -t buildfs

# Upload filesystem image to LittleFS partition
pio run -t uploadfs

# Upload firmware and monitor serial output
pio run -t upload -t monitor

# Erase flash (required before first flash)
pio run -t erase
```

### First-Time Setup Sequence
1. Run **Erase Flash** first
2. Run **Build Filesystem Image** 
3. Run **Upload Filesystem Image**
4. Run **Clean** then **Build**
5. Run **Upload and Monitor**

## Hardware Architecture

### Sensor Configuration
- **IMU**: MPU9250 (accelerometer, gyroscope, magnetometer) @ 500Hz
- **Barometric**: MS5611 or BMP388 pressure sensor @ 50Hz
- **GPS**: Ublox M8N @ 10Hz with UBX binary protocol
- **Display**: 128x64 LCD with ST7565 controller
- **Storage**: W25Q128FVSG 128Mbit SPI flash
- **Audio**: ESP32 DAC + NS8002 amplifier with sine wave tones

### SPI Bus Configuration
- **VSPI**: IMU, barometric sensor, and flash storage
- **HSPI**: LCD display (write-only)
- Pin mappings defined in `include/config.h`

### Key GPIO Pin Assignments
- Buttons: GPIO 0, 34, 36, 39
- GPS UART: GPIO 21/22 (TX/RX)
- Audio DAC: GPIO 25
- Audio Amp Enable: GPIO 32
- LED: GPIO 2

## Software Architecture

### Core Components

#### Main Application (`src/main.cpp`)
- Initialization and task coordination
- FreeRTOS task management for sensor fusion and UI

#### Sensor Fusion (`src/sensor/`)
- **Kalman Filter**: `kalmanfilter4d.cpp` - KF4d algorithm fusing IMU and barometric data
- **IMU Processing**: `imu.cpp`, `mpu9250.cpp` - sensor calibration and data acquisition
- **GPS**: `gps.cpp` - UBlox configuration and NMEA parsing
- **Barometric**: `ms5611.cpp`/`bmp388.cpp` - pressure sensor drivers

#### User Interface (`src/ui/`)
- **Display**: `lcd7565.cpp`, `ui.cpp` - LCD driver and display rendering
- **Audio**: `vario_audio.cpp` - variometer tone generation
- **Navigation**: `route.cpp` - waypoint navigation

#### Data Management (`src/nv/`)
- **Configuration**: `options.cpp` - user settings persistence  
- **Calibration**: `calib.cpp` - sensor calibration data
- **Logging**: `flashlog.cpp` - high-speed data logging to SPI flash

#### Connectivity
- **WiFi Server**: `src/wifi/async_server.cpp` - configuration web interface and file downloads
- **Bluetooth**: `src/bt/btmsg.cpp` - NMEA sentence transmission

### Data Flow
1. IMU data (500Hz) + Barometric data (50Hz) → Kalman Filter
2. Kalman Filter output → Variometer audio tones + LCD display
3. GPS data (10Hz) → Navigation calculations + track logging
4. All sensor data → Optional high-speed logging to flash

## Configuration

### Hardware Selection
- Set barometric sensor type in `include/config.h`:
  - `#define USE_MS5611 true` or `#define USE_BMP388 true`

### User Options
- Runtime configuration via LCD interface or `options.txt` file
- Upload `options.txt` via WiFi web interface
- Default values defined in `include/config.h` under `USER-CONFIGURABLE PARAMETER DEFAULTS`

### Debug Configuration
- Debug flags in `include/config.h` - comment out runtime loop debug flags before deployment
- Serial monitor at 115200 baud

## Calibration Requirements

### Critical Calibration Steps
1. **Gyroscope**: Auto-calibrated on startup (device must be stationary)
2. **Accelerometer**: Place on flat horizontal surface during calibration
3. **Magnetometer**: Perform 3D figure-8 motion while rotating 360° away from metal objects

Force recalibration by deleting `calib.txt` file or pressing btn0 during gyro calibration countdown.

## File System Structure

### LittleFS Partition (`/data/` directory)
- `index.html`, `style.css` - web interface files (required)
- `options.txt` - user configuration (optional)
- `calib.txt` - sensor calibration data
- `*.wpt` - route files in FormatGEO format (max 7 files, 20 char names)
- `flightlog.txt` - flight summaries
- Binary data logs from high-speed logging

### Web Interface Access
1. Press btn0 on startup for server mode
2. Connect to WiFi AP "Esp32GpsVario" 
3. Browse to http://esp32.local (admin/admin)
4. Upload/download files, OTA firmware updates

## Development Notes

### Code Organization
- Modular design with clear separation between drivers, sensors, UI, and connectivity
- FreeRTOS tasks for real-time sensor processing
- Extensive configuration options via compile-time and runtime parameters

### Performance Considerations
- Critical timing for 500Hz IMU sampling and 50Hz Kalman filter updates
- Audio tone generation requires precise DAC control
- Flash logging can fill 128Mb in ~13 minutes at high-speed rates