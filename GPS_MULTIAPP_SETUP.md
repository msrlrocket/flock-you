# GPS Multi-App Setup (gpsd Integration)

Instructions for sharing a single GPS dongle across multiple applications using gpsd.

## The Problem

A physical GPS serial device (`/dev/ttyACM0`) can only be opened by one application at a time. If you want to use GPS in both Flock You and another app (like GPSD clients, mapping software, etc.), you need gpsd as a multiplexer.

## The Solution: gpsd Daemon

gpsd sits between the GPS hardware and applications. Multiple apps connect to gpsd instead of the raw serial port.

```
GPS Dongle (/dev/ttyACM0) --> gpsd (port 2947) --> Flock You
                                               --> Other apps (gpsmon, cgps, etc.)
```

---

## System Setup (Raspberry Pi / Linux)

### 1. Install gpsd

```bash
sudo apt-get update
sudo apt-get install -y gpsd gpsd-clients python3-gps
```

### 2. Configure gpsd

Edit `/etc/default/gpsd`:
```bash
START_DAEMON="true"
USBAUTO="true"
DEVICES="/dev/ttyACM0"  # Your GPS device
GPSD_OPTIONS="-n"
```

### 3. Enable services

```bash
sudo systemctl enable gpsd
sudo systemctl start gpsd
```

### 4. Verify gpsd is working

```bash
cgps  # or gpsmon
```

You should see GPS data streaming.

---

## Python Setup (venv with python3-gps)

The PyPI `gps` package (3.19) is broken with Python 3.13. Use the system python3-gps instead:

```bash
# Find system python3-gps location
SYSTEM_GPS=$(python3 -c "import gps; print(gps.__file__)" 2>/dev/null | xargs dirname)

# Symlink into your venv
ln -sf "$SYSTEM_GPS" /path/to/your/venv/lib/python3.x/site-packages/
```

---

## Flock You Integration

### Files Needed

Create a `local/` directory with these files:

#### `local/__init__.py`
```python
# Empty or import hooks
```

#### `local/config.py`
```python
"""Configuration loaded from .env"""
import os
from pathlib import Path
from dotenv import load_dotenv

# Load .env from project root
env_path = Path(__file__).parent.parent / '.env'
load_dotenv(env_path)

# GPS Configuration
GPS_SOURCE = os.getenv('GPS_SOURCE', 'serial')
GPSD_HOST = os.getenv('GPSD_HOST', 'localhost')
GPSD_PORT = int(os.getenv('GPSD_PORT', '2947'))

# Device IDs for auto-connect (VID, PID tuples)
FLOCK_DEVICE_IDS = [(
    int(os.getenv('FLOCK_DEVICE_VID', '12346')),
    int(os.getenv('FLOCK_DEVICE_PID', '4097'))
)]
GPS_DEVICE_IDS = [(
    int(os.getenv('GPS_DEVICE_VID', '5446')),
    int(os.getenv('GPS_DEVICE_PID', '423'))
)]
```

#### `local/gps_adapter.py`
```python
"""
gpsd adapter that mimics a serial port interface.
Passes through raw NMEA sentences so flockyou.py works unchanged.

Uses the system python3-gps library (3.25+) which is compatible with Python 3.13.
Note: Requires system package python3-gps and symlink into venv.
"""
from gps import gps, WATCH_ENABLE, WATCH_NMEA
from local.config import GPSD_HOST, GPSD_PORT


class GPSDSerialAdapter:
    """Makes gpsd look like a serial port - passes through raw NMEA.

    Uses the official gpsd Python bindings for maximum compatibility.
    """

    def __init__(self):
        self.session = gps(host=GPSD_HOST, port=GPSD_PORT)
        self.session.stream(WATCH_ENABLE | WATCH_NMEA)
        self._is_open = True

        # Skip initial JSON responses (VERSION, DEVICES, WATCH)
        for _ in range(3):
            try:
                self.session.read()
            except:
                pass

    def readline(self):
        """Return raw NMEA sentence, just like serial would"""
        try:
            while self._is_open:
                result = self.session.read()
                if result == -1:
                    self._is_open = False
                    return b''

                # Get the raw response - NMEA sentences start with $
                response = getattr(self.session, 'response', None)
                if response and response.startswith('$'):
                    return (response + '\n').encode('utf-8')
                # Skip JSON responses, keep reading for NMEA

        except StopIteration:
            self._is_open = False
            return b''
        except Exception as e:
            print(f"gpsd read error: {e}")
            self._is_open = False
            return b''
        return b''

    @property
    def is_open(self):
        return self._is_open

    @property
    def port(self):
        return f"gpsd://{GPSD_HOST}:{GPSD_PORT}"

    @property
    def in_waiting(self):
        return 0

    def close(self):
        self._is_open = False
        try:
            self.session.close()
        except:
            pass
```

#### `local/hooks.py`
```python
"""
Hook functions that integrate local customizations with flockyou.py
These are called from flockyou.py at specific points.
"""
import sys
from pathlib import Path

# Add project root to path for local imports
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from local.config import GPS_SOURCE, GPSD_HOST, GPSD_PORT, FLOCK_DEVICE_IDS, GPS_DEVICE_IDS


def get_gps_connection(port=None, baudrate=9600, timeout=1):
    """Factory function - returns serial or gpsd adapter based on config"""
    if GPS_SOURCE == 'gpsd':
        try:
            from local.gps_adapter import GPSDSerialAdapter
            print("GPS: Using gpsd adapter")
            return GPSDSerialAdapter()
        except Exception as e:
            print(f"GPS: gpsd adapter failed ({e}), falling back to serial")

    if port:
        import serial
        return serial.Serial(port, baudrate, timeout=timeout)
    return None


def get_gpsd_status():
    """Check if gpsd is available and return port info for UI display"""
    if GPS_SOURCE != 'gpsd':
        return None

    import socket
    try:
        # Quick check if gpsd is responding
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        sock.connect((GPSD_HOST, GPSD_PORT))
        sock.close()

        return {
            'device': f'gpsd://{GPSD_HOST}:{GPSD_PORT}',
            'description': 'GPS Daemon (gpsd) - Shared GPS',
            'manufacturer': 'gpsd',
            'product': 'GPS Multiplexer',
            'vid': None,
            'pid': None,
            'is_gpsd': True
        }
    except Exception:
        return None


def on_startup():
    """Called when flockyou.py starts"""
    pass  # Add LED controller or other startup tasks here


def on_shutdown():
    """Called when flockyou.py shuts down"""
    pass  # Add cleanup tasks here


def on_new_detection(data):
    """Called when a new device is detected"""
    pass  # Add LED alerts or other detection handlers


def on_redetection(existing_detection):
    """Called when a known device is re-detected"""
    pass  # Add re-detection handlers


def auto_connect_devices(flock_connect_fn, gps_connect_fn):
    """Auto-detect and connect to Flock sniffer and GPS on startup"""
    import serial.tools.list_ports

    flock_port = None
    gps_port = None

    # Scan for devices
    for port in serial.tools.list_ports.comports():
        if (port.vid, port.pid) in FLOCK_DEVICE_IDS:
            flock_port = port.device
        elif (port.vid, port.pid) in GPS_DEVICE_IDS:
            gps_port = port.device

    # Connect Flock first
    if flock_port:
        flock_connect_fn(flock_port)
    else:
        print("Flock sniffer not detected")

    # Connect GPS second (will use gpsd if configured)
    if gps_port or GPS_SOURCE == 'gpsd':
        gps_connect_fn(gps_port)
    else:
        print("GPS dongle not detected")
```

---

## Hook Points in flockyou.py

Add these modifications to `api/flockyou.py`:

### 1. After imports (top of file)

```python
import sys
from pathlib import Path
# ... other imports ...

# Add project root to path for local imports
_project_root = Path(__file__).parent.parent
sys.path.insert(0, str(_project_root))

# Local customizations (optional)
try:
    from local import hooks as local_hooks
    LOCAL_ENABLED = True
except ImportError:
    local_hooks = None
    LOCAL_ENABLED = False
```

### 2. Add internal connect functions (before routes section)

```python
def _connect_flock_internal(port):
    """Internal function to connect to Flock device (used by auto-connect)"""
    global flock_device_connected, flock_device_port, flock_serial_connection
    try:
        flock_serial_connection = serial.Serial(port, 115200, timeout=1)
        with connection_lock:
            flock_device_connected = True
        flock_device_port = port
        flock_thread = threading.Thread(target=flock_reader, daemon=True)
        flock_thread.start()
        print(f"Auto-connected to Flock sniffer on {port}")
        time.sleep(1)
    except Exception as e:
        print(f"Failed to auto-connect Flock: {e}")


def _connect_gps_internal(port):
    """Internal function to connect to GPS device (used by auto-connect)"""
    global serial_connection, gps_enabled
    try:
        if LOCAL_ENABLED:
            serial_connection = local_hooks.get_gps_connection(port, GPS_BAUDRATE, GPS_TIMEOUT)
        else:
            serial_connection = serial.Serial(port, GPS_BAUDRATE, timeout=GPS_TIMEOUT)
        with connection_lock:
            gps_enabled = True
        gps_thread = threading.Thread(target=gps_reader, daemon=True)
        gps_thread.start()
        print(f"Auto-connected to GPS on {port}")
    except Exception as e:
        print(f"Failed to auto-connect GPS: {e}")
```

### 3. In get_gps_ports() - add gpsd option to UI

```python
@app.route('/api/gps/ports', methods=['GET'])
def get_gps_ports():
    """Get available serial ports for GPS"""
    ports = []

    # Add gpsd as virtual port option if available
    if LOCAL_ENABLED:
        gpsd_info = local_hooks.get_gpsd_status()
        if gpsd_info:
            ports.append(gpsd_info)

    for port in serial.tools.list_ports.comports():
        # ... existing code ...
```

### 4. In add_detection_from_serial() - add hook calls

After the re-detection emit:
```python
        safe_socket_emit('detection_updated', existing_detection)
        # ... existing code ...

        # Trigger visual alert for re-detection
        if LOCAL_ENABLED:
            local_hooks.on_redetection(existing_detection)
```

After the new detection emit:
```python
        safe_socket_emit('new_detection', data)
        # ... existing code ...

        # Trigger visual alert for new detection
        if LOCAL_ENABLED:
            local_hooks.on_new_detection(data)
```

### 5. In main block - startup and shutdown hooks

```python
if __name__ == '__main__':
    load_oui_database()
    load_cumulative_detections()
    load_settings()

    # Local customizations startup
    if LOCAL_ENABLED:
        local_hooks.on_startup()
        local_hooks.auto_connect_devices(
            flock_connect_fn=_connect_flock_internal,
            gps_connect_fn=_connect_gps_internal
        )

    # Start connection monitor thread
    # ... existing code ...

    try:
        socketio.run(app, ...)
    except KeyboardInterrupt:
        print("\nShutting down server...")
        # Clean up local customizations
        if LOCAL_ENABLED:
            local_hooks.on_shutdown()
        # ... existing cleanup ...
```

---

## Configuration (.env file)

Create `.env` in project root:

```bash
# GPS Configuration
GPS_SOURCE=gpsd              # "gpsd" (multi-app) or "serial" (direct)
GPSD_HOST=localhost
GPSD_PORT=2947

# Auto-connect Device IDs (VID, PID)
# Find these with: lsusb or the deploy.sh --devices command
FLOCK_DEVICE_VID=12346
FLOCK_DEVICE_PID=4097
GPS_DEVICE_VID=5446
GPS_DEVICE_PID=423
```

---

## Finding Device VID/PID

List connected USB devices:

```bash
lsusb
```

Or with more detail:
```bash
python3 -c "import serial.tools.list_ports; [print(f'{p.device}: VID={p.vid} PID={p.pid} - {p.description}') for p in serial.tools.list_ports.comports()]"
```

---

## Troubleshooting

### gpsd not receiving data

```bash
# Check gpsd status
sudo systemctl status gpsd

# Check if device exists
ls -la /dev/ttyACM*

# Test gpsd directly
gpsmon
```

### Python can't find gps module

Make sure system python3-gps is symlinked into your venv:
```bash
ls -la venv/lib/python3.*/site-packages/gps
```

### Permission denied on serial port

Add user to dialout group:
```bash
sudo usermod -a -G dialout $USER
# Then log out and back in
```

Or create udev rules for your GPS device.

---

## Known Bugs / TODO

### BUG: Stale GPS Data When Satellite Fix Lost

**Problem:** When GPS loses satellite fix (indoors, tunnels, parking garages, metal roofs), gpsd continues serving the last known good position. The current temporal matching code checks `time_diff` (system clock difference) but NOT the actual GPS NMEA timestamp. This causes detections to be tagged with old coordinates from a previous location.

**Example from field testing (2026-01-23):**
- Detection occurred at 09:33 PST at Location B
- GPS NMEA timestamp was 09:19 PST (14 minutes stale!)
- GPS coordinates were from Location A (4-5 km away)
- `time_diff` showed 0.96s (misleading - measures system buffer time, not GPS freshness)

**Root cause:** The `gps_history` buffer stores entries with `system_timestamp` (when added to buffer) but the GPS coordinates and NMEA timestamp can be stale if gpsd is serving cached data.

**Fix needed in `add_detection_from_serial()`:**

1. Parse the NMEA timestamp (HHMMSS.SS format in UTC) from `gps_entry['timestamp']`
2. Compare to current system time (converted to UTC)
3. If difference > threshold (e.g., 60 seconds), reject the GPS data as stale
4. Mark detection with `gps_stale: true` or `gps: None`

**Pseudocode:**
```python
def is_gps_fresh(gps_entry, max_age_seconds=60):
    """Check if GPS NMEA timestamp is recent enough"""
    nmea_ts = gps_entry.get('timestamp', '')
    if not nmea_ts:
        return False

    try:
        # Parse HHMMSS.SS format
        t = float(nmea_ts)
        nmea_hour = int(t // 10000)
        nmea_min = int((t % 10000) // 100)
        nmea_sec = t % 100

        # Get current UTC time
        now_utc = datetime.utcnow()

        # Build today's datetime with NMEA time (UTC)
        nmea_time = now_utc.replace(hour=nmea_hour, minute=nmea_min,
                                     second=int(nmea_sec), microsecond=0)

        # Handle midnight wraparound
        diff = abs((now_utc - nmea_time).total_seconds())
        if diff > 43200:  # More than 12 hours, probably day boundary
            diff = 86400 - diff

        return diff <= max_age_seconds
    except:
        return False
```

**Additional improvement:** After N seconds of stale GPS, emit a `gps_stale` event to the UI so user knows GPS is not updating.

**Optional:** Consider resetting gpsd connection or power-cycling USB if GPS stays stale for extended period (though usually the issue is physical - no satellite visibility).

---

## LED Notifications (NeoPixel)

Visual alerts using 2x NeoPixel LEDs on GPIO18. Mirrors the ESP32 firmware's audio alert patterns.

### Hardware Setup

- 2x WS2812B NeoPixel LEDs wired to GPIO18 (directly or via level shifter)
- Requires `rpi_ws281x` and `adafruit-circuitpython-neopixel` libraries

### Install Dependencies

```bash
sudo pip3 install rpi_ws281x adafruit-circuitpython-neopixel
# Also need wireless tools for WiFi monitoring
sudo apt-get install -y iw wireless-tools
```

### LED Patterns

| Event | Pattern | Color |
|-------|---------|-------|
| Boot | Fade in, pause, off | Yellow |
| Scanning (idle) | Dim pulse every 10s | Green (1% brightness) |
| WiFi connected | Sequential L→R→L blink x3 | Blue |
| WiFi disconnected | Sequential L→R→L blink x3 | Amber |
| New detection | Long flash x5 (0.4s on) | Red |
| Re-detection | Fast double-flash x3 | Red |
| WiFi device type | Double flash x2 | Left=Blue, Right=Orange |
| BLE device type | Double flash x2 | Left=Green, Right=Orange |
| Raven (gunshot detector) | Strobe x4 | Red + White alternating |
| In-range heartbeat | Single pulse every 10s | Orange |
| Out of range (30s timeout) | Triple blink, then off | Yellow |

### Configuration Constants

In `local/led_controller.py`:
```python
LED_PIN = 18              # GPIO pin for NeoPixels
LED_COUNT = 2             # Number of LEDs
HEARTBEAT_INTERVAL = 10   # Seconds between heartbeat pulses
RANGE_TIMEOUT = 30        # Seconds until "out of range"
WIFI_CHECK_INTERVAL = 5   # Seconds between WiFi state checks
```

### led_controller.py

```python
"""
NeoPixel LED Controller for Flock You
Visual alerts mirroring ESP32 firmware audio patterns
"""

import os
import time
import threading
import socket
import subprocess
import shutil
import math

# Configuration
LED_PIN = 18
LED_COUNT = 2
HEARTBEAT_INTERVAL = 10
RANGE_TIMEOUT = 30
WIFI_CHECK_INTERVAL = 5
WIFI_BLINK_COUNT = 3
WIFI_BLINK_ON = 0.2
WIFI_BLINK_OFF = 0.15

# Colors (RGB format)
COLOR_OFF = (0, 0, 0)
COLOR_RED = (255, 0, 0)
COLOR_GREEN = (0, 255, 0)
COLOR_BLUE = (0, 0, 255)
COLOR_PURPLE = (128, 0, 128)
COLOR_YELLOW = (255, 255, 0)
COLOR_AMBER = (255, 191, 0)
COLOR_ORANGE = (255, 165, 0)
COLOR_WHITE = (255, 255, 255)

# State
last_detection_time = 0
_heartbeat_thread = None
_heartbeat_running = False
_wifi_monitor_thread = None
_wifi_monitor_running = False
_scanning_thread = None
_scanning_running = False
_pixels = None
_is_raspberry_pi = False
_out_of_range_signaled = False
_last_wifi_connected = None
_last_wifi_ssid = None
_wifi_iface = None
_alert_lock = threading.Lock()


def _is_running_on_pi():
    """Check if running on a Raspberry Pi"""
    try:
        with open('/sys/firmware/devicetree/base/model', 'r') as f:
            return 'Raspberry Pi' in f.read()
    except:
        return False


def _set_all_pixels(color):
    """Set all pixels to the same color"""
    global _pixels
    if _pixels is None:
        return
    try:
        for i in range(LED_COUNT):
            _pixels[i] = color
        _pixels.show()
    except Exception as e:
        print(f"LED set color error: {e}")


def _blink(color, count=1, on_time=0.2, off_time=0.2):
    if _pixels is None:
        return
    for i in range(count):
        _set_all_pixels(color)
        time.sleep(on_time)
        _set_all_pixels(COLOR_OFF)
        if i < count - 1:
            time.sleep(off_time)


def init_leds():
    """Initialize NeoPixel LEDs"""
    global _pixels, _is_raspberry_pi, _heartbeat_running, _heartbeat_thread
    global _wifi_monitor_running, _wifi_monitor_thread
    global _scanning_running, _scanning_thread

    _is_raspberry_pi = _is_running_on_pi()

    if not _is_raspberry_pi:
        print("LED Controller: Not on Pi, using stub mode")
        return True

    try:
        import board
        import neopixel

        _pixels = neopixel.NeoPixel(
            board.D18,
            LED_COUNT,
            brightness=0.5,
            auto_write=False,
            pixel_order=neopixel.GRB
        )
        print("LED Controller: Initialized on GPIO18")

        # Boot sequence: Yellow fade-in
        # ... (fade transition code)

        # Start background threads for scanning pulse, heartbeat, WiFi monitor
        # ... (thread startup code)

        return True
    except ImportError as e:
        print(f"LED Controller: NeoPixel not available ({e})")
        return True


def detection_alert(protocol=None, detection_method=None, is_new=True):
    """Visual alert for detection"""
    global last_detection_time, _out_of_range_signaled

    last_detection_time = time.time()
    _out_of_range_signaled = False

    if not _is_raspberry_pi or _pixels is None:
        return

    def flash_worker():
        with _alert_lock:
            # Raven = red/white strobe
            if detection_method == 'raven_service_uuid':
                _raven_alert()
                return

            if is_new:
                # New: Long red flashes x5
                _blink(COLOR_RED, count=5, on_time=0.4, off_time=0.2)
            else:
                # Re-detection: Fast double-flash x3
                for _ in range(3):
                    _blink(COLOR_RED, count=2, on_time=0.1, off_time=0.1)
                    time.sleep(0.15)

            time.sleep(0.3)
            _device_type_indication(protocol, detection_method)

    threading.Thread(target=flash_worker, daemon=True).start()


def cleanup():
    """Clean up on shutdown"""
    global _heartbeat_running, _wifi_monitor_running, _scanning_running, _pixels

    _heartbeat_running = False
    _wifi_monitor_running = False
    _scanning_running = False

    if _is_raspberry_pi and _pixels:
        _set_all_pixels(COLOR_OFF)
        _pixels.deinit()
    _pixels = None
```

### hooks.py Integration

Update `local/hooks.py` to call the LED controller:

```python
from local import led_controller

def on_startup():
    """Called when flockyou.py starts"""
    led_controller.init_leds()
    led_controller.wifi_connected()

def on_shutdown():
    """Called when flockyou.py shuts down"""
    led_controller.cleanup()

def on_new_detection(data):
    """Called when a new device is detected"""
    led_controller.detection_alert(
        protocol=data.get('protocol'),
        detection_method=data.get('detection_method'),
        is_new=True
    )

def on_redetection(existing_detection):
    """Called when a known device is re-detected"""
    led_controller.detection_alert(
        protocol=existing_detection.get('protocol'),
        detection_method=existing_detection.get('detection_method'),
        is_new=False
    )
```

### Background Threads

The LED controller runs 3 background threads:

1. **Scanning pulse** - Dim green glow every 10s when idle (no recent detections)
2. **Heartbeat** - Orange pulse every 10s while a device is "in range" (detected within last 30s)
3. **WiFi monitor** - Checks WiFi state every 5s, blinks on connect/disconnect/SSID change

### Stub Mode

When not running on a Raspberry Pi (e.g., development on Mac), the controller runs in "stub mode" - all functions log to console but don't try to access GPIO.
