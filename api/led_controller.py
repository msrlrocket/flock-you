"""
NeoPixel LED Controller for Flock You
Visual alerts with device-type differentiation and detection patterns
"""

import os
import time
import threading
import socket
import subprocess
import shutil

# Configuration
LED_PIN = 18
LED_COUNT = 2
HEARTBEAT_INTERVAL = 10   # seconds between pulses (changed from 5)
RANGE_TIMEOUT = 30        # seconds until "out of range"
WIFI_CHECK_INTERVAL = 5   # seconds between WiFi checks
WIFI_BLINK_COUNT = 3
WIFI_BLINK_ON = 0.2
WIFI_BLINK_OFF = 0.2
SCANNING_PULSE_INTERVAL = 10  # seconds between scanning pulses

# Colors (RGB format - NeoPixel library handles GRB conversion via pixel_order)
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
_alert_lock = threading.Lock()  # Prevent overlapping LED patterns


def _is_running_on_pi():
    """Check if running on a Raspberry Pi"""
    try:
        with open('/sys/firmware/devicetree/base/model', 'r') as f:
            model = f.read()
            return 'Raspberry Pi' in model
    except:
        return False


def _check_network():
    """Check if the Pi has network connectivity"""
    try:
        socket.create_connection(("8.8.8.8", 53), timeout=3)
        return True
    except OSError:
        return False


def _command_available(cmd):
    return shutil.which(cmd) is not None


def _wifi_tools_available():
    return _command_available("iwgetid") or _command_available("iw")


def _get_wifi_link_state(iface):
    if not iface:
        return False

    try:
        carrier_path = os.path.join("/sys/class/net", iface, "carrier")
        with open(carrier_path, "r") as f:
            return f.read().strip() == "1"
    except Exception:
        pass

    try:
        with open("/proc/net/wireless", "r") as f:
            for line in f.readlines()[2:]:
                if line.strip().startswith(f"{iface}:"):
                    parts = line.split()
                    if len(parts) > 2:
                        return float(parts[2].strip(".")) > 0
    except Exception:
        pass

    return False


def _detect_wifi_interface():
    env_iface = os.environ.get("FLOCK_WIFI_IFACE")
    if env_iface:
        return env_iface

    try:
        candidates = []
        for iface in os.listdir("/sys/class/net"):
            if iface == "lo":
                continue
            if os.path.isdir(os.path.join("/sys/class/net", iface, "wireless")):
                candidates.append(iface)
        if not candidates:
            return None
        if "wlan0" in candidates:
            return "wlan0"
        return candidates[0]
    except Exception:
        return None


def _get_wifi_ssid(iface):
    if not iface:
        return None

    try:
        if _command_available("iwgetid"):
            output = subprocess.check_output(
                ["iwgetid", "-r", iface],
                stderr=subprocess.DEVNULL,
                text=True
            ).strip()
            if output:
                return output
    except Exception:
        pass

    try:
        if _command_available("iw"):
            output = subprocess.check_output(
                ["iw", "dev", iface, "link"],
                stderr=subprocess.DEVNULL,
                text=True
            )
            for line in output.splitlines():
                line = line.strip()
                if line.startswith("SSID:"):
                    ssid = line.split("SSID:", 1)[1].strip()
                    return ssid or None
    except Exception:
        pass

    return None


def _set_pixel(index, color):
    """Set a single pixel to a color"""
    global _pixels
    if _pixels is None or index >= LED_COUNT:
        return
    try:
        _pixels[index] = color
        _pixels.show()
    except Exception as e:
        print(f"LED set pixel error: {e}")


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
    try:
        for i in range(count):
            _set_all_pixels(color)
            time.sleep(on_time)
            _set_all_pixels(COLOR_OFF)
            if i < count - 1:
                time.sleep(off_time)
    except Exception as e:
        print(f"LED blink error: {e}")


def _sequential_blink(color, count=1, on_time=0.15, off_time=0.1):
    """Sequential L->R->L blink pattern for WiFi events"""
    if _pixels is None:
        return
    try:
        for _ in range(count):
            # Left LED
            _set_pixel(0, color)
            time.sleep(on_time)
            _set_pixel(0, COLOR_OFF)
            time.sleep(off_time)
            # Right LED
            _set_pixel(1, color)
            time.sleep(on_time)
            _set_pixel(1, COLOR_OFF)
            time.sleep(off_time)
    except Exception as e:
        print(f"LED sequential blink error: {e}")


def _fade_transition(from_color, to_color, steps=20, delay=0.05):
    """Smooth fade transition between two colors"""
    global _pixels
    if _pixels is None:
        return

    try:
        for step in range(steps + 1):
            ratio = step / steps
            r = int(from_color[0] + (to_color[0] - from_color[0]) * ratio)
            g = int(from_color[1] + (to_color[1] - from_color[1]) * ratio)
            b = int(from_color[2] + (to_color[2] - from_color[2]) * ratio)
            _set_all_pixels((r, g, b))
            time.sleep(delay)
    except Exception as e:
        print(f"LED fade error: {e}")


def _fade_in(color, steps=20, delay=0.05):
    """Fade in from off to color"""
    _fade_transition(COLOR_OFF, color, steps, delay)


def _fade_out(color, steps=20, delay=0.05):
    """Fade out from color to off"""
    _fade_transition(color, COLOR_OFF, steps, delay)


def _device_type_indication(protocol, detection_method):
    """Show device type indication after main alert"""
    if _pixels is None:
        return

    try:
        time.sleep(0.3)  # Brief pause after main alert

        if protocol == 'wifi':
            # WiFi: Left=Blue, Right=Orange
            _set_pixel(0, COLOR_BLUE)
            _set_pixel(1, COLOR_ORANGE)
        elif protocol in ['bluetooth_le', 'bluetooth_classic']:
            # BLE: Left=Green, Right=Orange
            _set_pixel(0, COLOR_GREEN)
            _set_pixel(1, COLOR_ORANGE)
        else:
            # Unknown: Both amber
            _set_all_pixels(COLOR_AMBER)

        time.sleep(0.5)
        _set_all_pixels(COLOR_OFF)
    except Exception as e:
        print(f"LED device type indication error: {e}")


def _raven_alert():
    """Special alert for Raven gunshot detectors - red/white strobe"""
    if _pixels is None:
        return

    try:
        for _ in range(4):
            _set_all_pixels(COLOR_RED)
            time.sleep(0.1)
            _set_all_pixels(COLOR_WHITE)
            time.sleep(0.1)
        _set_all_pixels(COLOR_OFF)
    except Exception as e:
        print(f"LED raven alert error: {e}")


def _scanning_pulse():
    """Dim green pulse to indicate active scanning"""
    if _pixels is None:
        return

    try:
        # Dim green glow
        dim_green = (0, 50, 0)  # Very dim green
        _fade_in(dim_green, steps=10, delay=0.03)
        time.sleep(0.2)
        _fade_out(dim_green, steps=10, delay=0.03)
    except Exception as e:
        print(f"LED scanning pulse error: {e}")


def init_leds():
    """Initialize NeoPixel LEDs and run boot sequence (yellow fade-in)"""
    global _pixels, _is_raspberry_pi, _heartbeat_running, _heartbeat_thread
    global _wifi_monitor_running, _wifi_monitor_thread, _wifi_iface
    global _scanning_running, _scanning_thread

    _is_raspberry_pi = _is_running_on_pi()

    if not _is_raspberry_pi:
        print("LED Controller: Not running on Raspberry Pi, using stub mode")
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
        print("LED Controller: Running boot sequence (yellow fade-in)")
        _fade_in(COLOR_YELLOW, steps=30, delay=0.03)
        time.sleep(0.5)
        _fade_out(COLOR_YELLOW, steps=20, delay=0.03)

        # Start heartbeat thread
        _heartbeat_running = True
        _heartbeat_thread = threading.Thread(target=_heartbeat_worker, daemon=True)
        _heartbeat_thread.start()

        # Start scanning pulse thread
        _scanning_running = True
        _scanning_thread = threading.Thread(target=_scanning_worker, daemon=True)
        _scanning_thread.start()

        _wifi_monitor_running = True
        _wifi_iface = _detect_wifi_interface()
        if not _wifi_tools_available():
            print("LED Controller: Missing WiFi tools (iw/iwgetid). Install with: sudo apt-get install -y iw wireless-tools")
        _wifi_monitor_thread = threading.Thread(target=_wifi_monitor_worker, daemon=True)
        _wifi_monitor_thread.start()

        print("LED Controller: Boot sequence complete")
        return True

    except ImportError as e:
        print(f"LED Controller: NeoPixel libraries not available ({e}), using stub mode")
        _is_raspberry_pi = False
        return True
    except Exception as e:
        print(f"LED Controller: Initialization failed ({e}), using stub mode")
        _is_raspberry_pi = False
        return True


def wifi_connected():
    """Blue blink when WiFi is connected"""
    global _pixels, _is_raspberry_pi

    if not _is_raspberry_pi or _pixels is None:
        print("LED Controller: WiFi connected (stub)")
        return

    print("LED Controller: WiFi connected - blue blink")

    def worker():
        with _alert_lock:
            _sequential_blink(COLOR_BLUE, count=WIFI_BLINK_COUNT, on_time=WIFI_BLINK_ON, off_time=WIFI_BLINK_OFF)

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()


def wifi_disconnected():
    """Amber triple blink when WiFi disconnects"""
    if not _is_raspberry_pi or _pixels is None:
        print("LED Controller: WiFi disconnected (stub)")
        return

    print("LED Controller: WiFi disconnected - amber blink")

    def worker():
        with _alert_lock:
            _blink(COLOR_AMBER, count=WIFI_BLINK_COUNT, on_time=WIFI_BLINK_ON, off_time=WIFI_BLINK_OFF)

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()


def detection_alert(protocol=None, detection_method=None, is_new=True):
    """
    Visual alert for detection events.

    Args:
        protocol: 'wifi', 'bluetooth_le', 'bluetooth_classic', etc.
        detection_method: 'probe_request', 'beacon', 'raven_service_uuid', etc.
        is_new: True for new detection, False for re-detection
    """
    global _pixels, _is_raspberry_pi, last_detection_time, _out_of_range_signaled

    # Update last detection time for heartbeat tracking
    last_detection_time = time.time()
    _out_of_range_signaled = False

    if not _is_raspberry_pi or _pixels is None:
        print(f"LED Controller: Detection alert (stub) - protocol={protocol}, method={detection_method}, is_new={is_new}")
        return

    print(f"LED Controller: Detection alert - protocol={protocol}, method={detection_method}, is_new={is_new}")

    def flash_worker():
        with _alert_lock:
            try:
                # Special handling for Raven gunshot detectors
                if detection_method == 'raven_service_uuid':
                    _raven_alert()
                elif is_new:
                    # New detection: Long red flashes x5 (0.4s on)
                    _blink(COLOR_RED, count=5, on_time=0.4, off_time=0.2)
                else:
                    # Re-detection: Fast double-flash x3
                    for _ in range(3):
                        _blink(COLOR_RED, count=2, on_time=0.1, off_time=0.05)
                        time.sleep(0.2)

                # Show device type indication after main alert
                if protocol:
                    _device_type_indication(protocol, detection_method)

            except Exception as e:
                print(f"LED Controller: Detection flash error: {e}")

    thread = threading.Thread(target=flash_worker, daemon=True)
    thread.start()


def heartbeat_pulse():
    """Orange double pulse for in-range heartbeat"""
    global _pixels, _is_raspberry_pi

    if not _is_raspberry_pi or _pixels is None:
        return

    try:
        with _alert_lock:
            _blink(COLOR_ORANGE, count=2, on_time=0.2, off_time=0.15)
    except Exception as e:
        print(f"LED Controller: Heartbeat pulse error: {e}")


def out_of_range_alert():
    """Yellow triple blink when device goes out of range"""
    if not _is_raspberry_pi or _pixels is None:
        print("LED Controller: Out of range (stub)")
        return

    print("LED Controller: Out of range - yellow blink")
    with _alert_lock:
        _blink(COLOR_YELLOW, count=3, on_time=0.2, off_time=0.15)


def _heartbeat_worker():
    """Background thread for heartbeat management"""
    global last_detection_time, _heartbeat_running, _out_of_range_signaled

    last_pulse_time = 0

    while _heartbeat_running:
        try:
            now = time.time()
            time_since_detection = now - last_detection_time

            # Only pulse if we have a detection within RANGE_TIMEOUT
            if last_detection_time > 0 and time_since_detection < RANGE_TIMEOUT:
                # Pulse every HEARTBEAT_INTERVAL seconds
                if now - last_pulse_time >= HEARTBEAT_INTERVAL:
                    print(f"LED Controller: Heartbeat pulse ({time_since_detection:.0f}s since last detection)")
                    heartbeat_pulse()
                    last_pulse_time = now
            elif last_detection_time > 0 and time_since_detection >= RANGE_TIMEOUT:
                # Out of range - LEDs off
                if not _out_of_range_signaled:
                    out_of_range_alert()
                    _out_of_range_signaled = True
                clear_leds()

            time.sleep(1)  # Check every second

        except Exception as e:
            print(f"LED Controller: Heartbeat worker error: {e}")
            time.sleep(1)


def _scanning_worker():
    """Background thread for scanning pulse when idle"""
    global _scanning_running, last_detection_time

    while _scanning_running:
        try:
            now = time.time()
            time_since_detection = now - last_detection_time

            # Only show scanning pulse when NOT in range (no recent detection)
            # Skip if we're within RANGE_TIMEOUT of a detection (heartbeat is active)
            if last_detection_time == 0 or time_since_detection >= RANGE_TIMEOUT:
                # Only pulse if we can acquire the lock (don't block other alerts)
                if _alert_lock.acquire(blocking=False):
                    try:
                        _scanning_pulse()
                    finally:
                        _alert_lock.release()

            time.sleep(SCANNING_PULSE_INTERVAL)

        except Exception as e:
            print(f"LED Controller: Scanning worker error: {e}")
            time.sleep(SCANNING_PULSE_INTERVAL)


def _wifi_monitor_worker():
    """Background thread for WiFi state monitoring"""
    global _last_wifi_connected, _last_wifi_ssid, _wifi_iface, _wifi_monitor_running

    while _wifi_monitor_running:
        try:
            if not _wifi_iface:
                _wifi_iface = _detect_wifi_interface()

            ssid = _get_wifi_ssid(_wifi_iface)
            connected = bool(ssid) or _get_wifi_link_state(_wifi_iface)

            if _last_wifi_connected is None:
                _last_wifi_connected = connected
                _last_wifi_ssid = ssid
            else:
                if connected != _last_wifi_connected:
                    if connected:
                        print(f"LED Controller: WiFi connected on {_wifi_iface} (SSID: {ssid})")
                        wifi_connected()
                    else:
                        print(f"LED Controller: WiFi disconnected on {_wifi_iface}")
                        wifi_disconnected()
                    _last_wifi_connected = connected
                    _last_wifi_ssid = ssid
                elif connected and ssid and ssid != _last_wifi_ssid:
                    print(f"LED Controller: WiFi SSID changed on {_wifi_iface} ({_last_wifi_ssid} -> {ssid})")
                    wifi_connected()
                    _last_wifi_ssid = ssid

            time.sleep(WIFI_CHECK_INTERVAL)
        except Exception as e:
            print(f"LED Controller: WiFi monitor error: {e}")
            time.sleep(WIFI_CHECK_INTERVAL)


def clear_leds():
    """Turn off all LEDs"""
    global _pixels, _is_raspberry_pi

    if not _is_raspberry_pi or _pixels is None:
        return

    try:
        _set_all_pixels(COLOR_OFF)
    except Exception as e:
        print(f"LED Controller: Clear error: {e}")


def cleanup():
    """Clean up LED resources on shutdown"""
    global _heartbeat_running, _wifi_monitor_running, _scanning_running, _pixels, _is_raspberry_pi

    print("LED Controller: Cleaning up")

    _heartbeat_running = False
    _wifi_monitor_running = False
    _scanning_running = False

    if _is_raspberry_pi and _pixels is not None:
        try:
            _set_all_pixels(COLOR_OFF)
            _pixels.deinit()
        except Exception as e:
            print(f"LED Controller: Cleanup error: {e}")

    _pixels = None
    print("LED Controller: Cleanup complete")


def update_last_detection_time():
    """Update the last detection time (called from flockyou.py)"""
    global last_detection_time, _out_of_range_signaled
    last_detection_time = time.time()
    _out_of_range_signaled = False
