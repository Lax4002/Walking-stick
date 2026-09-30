# Walking Stick — Vision Module

Real-time YOLO object detection with a **live camera window** using the Raspberry Pi CSI camera. This module is one component of an assistive walking stick that will also integrate GPS tracking and MPU-6050 fall detection.

---

## Features

| Feature | Detail |
|---|---|
| **Picamera2 + libcamera** | Native CSI camera support (ov5647, IMX219, etc.) — works on modern Raspberry Pi OS. |
| **Live camera window** | Bounding boxes, labels, and FPS overlay drawn on the video feed. |
| **Terminal output** | Detections also printed to the console with coordinates and confidence. |
| **Frame-skipping** | Only every *N*th frame is sent through inference (`--skip`). |
| **Frame resizing** | Frames resized to 320×320 before inference (`--size`) to reduce CPU load. |
| **Graceful shutdown** | Press `q` in the camera window or `Ctrl-C` in the terminal. |

---

## Prerequisites

- **Raspberry Pi** (Pi 4 / Pi 5) with a desktop session (VNC, Remote Desktop, or local display).
- **Raspberry Pi OS** (Bookworm or later) — the libcamera stack must be active.
- **Python 3.9+**
- **CSI camera** connected (ov5647, IMX219, etc.) — verify with `rpicam-hello --list-cameras`.
- **Your trained YOLO model** (`yolo11_obstacles_best.pt`) in the project folder.

## Hardware Inventory

| Component | Interface | Role | Runtime option |
|---|---|---|---|
| CSI camera | Picamera2/libcamera | Captures frames for YOLO | Always enabled |
| YOLO ONNX model | CPU/ONNX Runtime | Identifies objects and obstacles | `--model`, `--conf`, `--skip` |
| MPU-6050 | I2C bus 1, address `0x68` | Fall detection and cumulative step count | `--no-mpu`, `--mpu-hz` |
| GPS receiver | UART `/dev/serial0` | Latest latitude/longitude fix | `--no-gps`, `--gps-port` |
| HC-SR04 | GPIO 23 trigger, GPIO 24 echo | Near-obstacle distance | `--no-us` |
| GSM modem | UART `/dev/ttyAMA1` | SMS alerts to every configured number | `--no-gsm`, `--gsm-port` |
| Buzzer | GPIO 17 | Local proximity/vision alert | `--no-alerts` |
| Vibration motor | GPIO 27 | Local proximity/vision alert | `--no-alerts` |
| Audio output | `pyttsx3`/espeak-ng | Spoken priority alerts | `--no-tts` |
| Wakatech MQTT | Network | Signed device events and settings commands | Environment configuration |
| Legacy HTTP backend | Network | Optional compatibility telemetry and alerts | `--backend-url` |

The GSM modem is part of the alert path: every rate-limited fall, priority
YOLO object, and ultrasonic alert is queued for SMS delivery to every number
in `WALKING_STICK_PHONE_NUMBERS` (or the comma-separated `--phone` value).
The modem must have a working SIM, antenna, power supply, network registration,
and its UART must not be shared with the GPS receiver.

---

## Quick Start

### 1. Install the system camera package

`picamera2` is a **system package** (it wraps libcamera) — it can't be installed via pip:

```bash
sudo apt install -y python3-picamera2
```

For spoken alerts, also install the offline speech backend:

```bash
sudo apt install -y espeak-ng libespeak1
```

### 2. Create a virtual environment WITH system packages

This is critical — the venv must inherit the system-level `picamera2`:

```bash
cd ~/walking_stick

python3 -m venv --system-site-packages venv
source venv/bin/activate
```

> **Already have a venv without `--system-site-packages`?** Delete and recreate it:
> ```bash
> rm -rf venv
> python3 -m venv --system-site-packages venv
> source venv/bin/activate
> ```

### 3. Install pip dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

### 4. Ensure the model file is present

```bash
ls *.pt
# Expected: yolo11_obstacles_best.pt
```

### 5. Run the detector

```bash
python main.py
```

The walking stick speaks detected YOLO objects, nearby ultrasonic obstacles,
and fall alerts. Speech runs in its own worker so camera and sensor processing
continues while audio is playing. Repeated events are rate-limited. Use
`--no-tts` to disable speech, or adjust `--tts-rate` and `--tts-volume`.
The program is headless by default and does not require an X/Wayland display.
Use `--display` when a live OpenCV camera window is available. Only priority
objects are spoken by default: people, vehicles, traffic controls, and the
configured walking-stick obstacle classes. Override them with
`--tts-objects "person,car,stairs"`.

### Wakatech MQTT integration

When `MQTT_HOST` is configured, `main.py` embeds the contract implementation in
`wakatech.py`; do not run both programs as long-lived services. The integrated
process publishes signed heartbeats, activity checkpoints, completed walking
sessions, falls, and settings acknowledgements at QoS 1. It subscribes to the
device settings topic and keeps its offline outbox in SQLite.

Copy `walking-stick.env.example` to `/etc/wakatech/device.env`, then edit that
file locally. Never commit or print the resulting file. For a broker on another
computer, set `MQTT_HOST` to that computer's LAN address rather than
`localhost`. Local port/TLS values must match the backend team's broker; the
template's `1883`/`false` values are conventional local-development defaults,
not a statement about an unavailable backend configuration.

The Pi has no native battery gauge. `WAKATECH_BATTERY_PERCENT` is therefore an
explicit test fallback unless a Linux-supported UPS/fuel gauge provides a
`/sys/class/power_supply/*/capacity` reading.

### Legacy HTTP backend events

Set an HTTP POST URL to enable backend delivery. The URL may be a Supabase
REST insert endpoint, a Firebase HTTPS function, or your own API:

```bash
export WALKING_STICK_API_URL="https://your-api.example.com/walking-stick/events"
export WALKING_STICK_API_TOKEN="your-token"
export WALKING_STICK_DEVICE_ID="stick-01"
export WALKING_STICK_PHONE_NUMBERS="+15551234567,+15557654321"
python main.py
```

The same values can be passed with `--backend-url`, `--backend-token`, and
`--device-id`. Every request contains `event_id`, `device_id`, `sequence`,
`event_type`, `priority`, `occurred_at`, and `payload`. `telemetry` events are
sent every five minutes and contain the latest GPS fix plus cumulative
`steps_since_boot`. `alert` events are sent for falls, priority YOLO objects,
and ultrasonic obstacles. Failed requests are retried and saved in
`backend_events.jsonl` for later delivery; change this path with
`--backend-spool`.

When the GPS receiver stops producing fixes, the worker also performs a
non-blocking `GET` against the same backend URL with `device_id`,
`event_type=telemetry`, and `latest=1`. The endpoint should return either
`{"latitude": ..., "longitude": ..., "fix_at": ...}`,
`{"location": {...}}`, `{"gps": {...}}`, or a one-item list containing one
of those objects. A fresh GPS fix always takes precedence; otherwise the
stored fix is used and marked as `location_source: backend`.

### Start automatically at boot

The repository includes `walking-stick.service`, configured for this project
path and headless operation. Install the environment file and service once:

```bash
sudo install -d -m 700 /etc/wakatech
sudo install -m 600 walking-stick.env.example /etc/wakatech/device.env
sudoedit /etc/wakatech/device.env
sudo cp walking-stick.service /etc/systemd/system/walking-stick.service
sudo systemctl daemon-reload
sudo systemctl enable --now walking-stick.service
systemctl status walking-stick.service
journalctl -u walking-stick.service -f
```

Edit `/etc/walking-stick.env` to add all phone numbers and backend settings.
The service starts at boot, restarts after crashes, and runs without a desktop
display. Stop it with `sudo systemctl disable --now walking-stick.service`.

`main.py` will check for picamera2, **auto-install pip dependencies** if missing,
**locate your `.pt` model**, and run the integrated system without requiring a
display.

With custom options:

```bash
python main.py \
    --model yolo11_obstacles_best.pt \
    --size 320 \
    --skip 2 \
    --conf 0.45
```

In headless mode, detections and alerts are printed in the terminal. With
`--display`, a camera window also shows the live feed with bounding boxes. The
terminal will display:

```
── Frame      2  |  FPS:   3.2 ──
  pothole              conf=0.87  bbox=(112, 45, 398, 470)
  curb                 conf=0.62  bbox=(420, 230, 590, 460)
```

Press **`q`** in the camera window when using `--display`, or **Ctrl-C** in the
terminal, to stop.

---

## CLI Reference

| Flag | Default | Description |
|---|---|---|
| `--model` | `yolo11_obstacles_best.pt` | Path to the YOLO `.pt` model file |
| `--size` | `320` | Model input resolution (pixels). Lower = faster |
| `--skip` | `2` | Process every Nth frame. `1` = no skipping |
| `--conf` | `0.45` | Minimum confidence to keep a detection |
| `--width` | `640` | Camera capture width |
| `--height` | `480` | Camera capture height |
| `--display` | off | Enable the live OpenCV window |
| `--no-tts` | off | Disable spoken announcements |
| `--tts-objects` | priority classes | Comma-separated YOLO classes to speak |

---

## Troubleshooting

### "picamera2 is not available"
```bash
sudo apt install -y python3-picamera2
# Then recreate the venv:
rm -rf venv
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r requirements.txt
```

### Camera not detected
```bash
# Check if the camera is visible to libcamera:
rpicam-hello --list-cameras

# If no cameras listed, check the ribbon cable and enable the camera in raspi-config:
sudo raspi-config
# → Interface Options → Camera → Enable
```

---

## Performance Tuning

| Knob | Effect |
|---|---|
| `--size 256` | Smaller input → faster inference, slightly lower accuracy |
| `--skip 3` | Process only 1 in 3 frames → more CPU headroom for GPS / IMU |
| `--width 320 --height 240` | Reduce camera capture resolution → less memory bandwidth |

---

## Project Structure

```
walking_stick/
├── main.py                       # ← Run this — single entry point
├── vision_detector.py            # Core detection engine (Picamera2 + YOLO)
├── requirements.txt              # pip dependencies
├── README.md                     # This file
└── yolo11_obstacles_best.pt      # Your trained YOLO model
```

---

## Licence

Internal project — assistive walking stick prototype.
# Wakatech-hardware
# Wakatech-hardware
