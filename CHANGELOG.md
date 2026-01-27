# Changelog

## 2026-01-26
### ESP32 Firmware - WS2812 LED Integration
- Added FastLED library support for built-in WS2812 LED on Xiao ESP32 S3 (GPIO 48)
- Boot animation: blue → green with beeps, followed by RGB flash sequence
- Detection alerts: LED flashes red in sync with buzzer beeps
- Active detection state: solid red LED while surveillance device is being detected
- Heartbeat mode: solid purple LED when no new detections for 5+ seconds
- Heartbeat pulses: LED briefly turns off with each 10-second heartbeat beep
- Out of range: LED turns off after 30 seconds with no detections
- Added FastLED library dependency to both xiao_esp32s3 and xiao_esp32c3 environments

## 2026-01-23
### Web API - gpsd Integration & Environment Config
- Added gpsd daemon support via GPSDSerialAdapter for multi-app GPS sharing
- Environment variables for GPS config (GPS_SOURCE, GPSD_HOST, GPSD_PORT)
- Device VID/PID now configurable via .env (FLOCK_DEVICE_VID, FLOCK_DEVICE_PID, GPS_DEVICE_VID, GPS_DEVICE_PID)
- GPS freshness checking to reject stale cached data
- Added python-dotenv and pygpsd dependencies

### Deployment & Infrastructure
- Enhanced deploy.sh: syncs .env file, auto-installs dependencies on Pi, prompts to restart service
- Extended .gitignore with Python, Flask data, and environment file patterns
- Pi LED controller pattern improvements

## 2026-01-22
- NeoPixel LED controller with boot, detection, heartbeat, and WiFi status patterns.
- Auto-connect for Flock sniffer and GPS devices by USB VID/PID.
- Deployment helper script and systemd service for Raspberry Pi hosting.
- Web UI auto-selects connected Flock/GPS ports on status refresh.
