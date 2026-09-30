"""
main.py â€” Walking Stick: YOLO + MPU-6050 + GPS + Ultrasonic + GSM + Alerts

Single entry point that runs all systems together:
  â€¢ YOLO object detection via Picamera2 + Ultralytics ONNX  (main thread)
  â€¢ MPU-6050 fall detection via I2C                         (background thread)
  â€¢ GPS coordinate tracking via UART serial                  (background thread)
  â€¢ HC-SR04 ultrasonic distance sensing                     (background thread)
  â€¢ GSM SMS emergency dispatch via UART serial               (background thread)

The sensor threads use non-blocking queues to communicate with the main
thread, so YOLO inference is never bottlenecked by sensor I/O.

Usage
â”€â”€â”€â”€â”€
    python main.py --phone "+1234567890"
    python main.py --model yolo11n --conf 0.45
    python main.py --skip 3 --gps-port /dev/serial0 --gsm-port /dev/ttyAMA1
"""

from __future__ import annotations
import faulthandler
faulthandler.enable()

import argparse
import ast
import importlib
import os
import queue
import signal
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
from queue import Queue
import RPi.GPIO as GPIO

import cv2
import numpy as np

from audio import SpeechManager
from backend import BackendClient
from mqtt_bridge import (
    WakatechMqttBridge,
    current_battery_percent,
    current_wifi_rssi,
    mqtt_location,
    utc_timestamp,
)
from steps import StepCounter

PROJECT_DIR = Path(__file__).resolve().parent
REQUIREMENTS_FILE = PROJECT_DIR / "requirements.txt"
DEFAULT_MODEL = "yolo11n"  # Resolves to local yolo11n.onnx; no remote download.
DEFAULT_TTS_OBJECTS = {
    "person", "bicycle", "car", "motorcycle", "bus", "truck",
    "traffic light", "stop sign", "pothole", "bollard", "stairs",
    "car_placeholder",
}


def parse_phone_numbers(value: str) -> list[str]:
    """Return normalized non-empty GSM recipients from comma-separated input."""
    return [phone.strip() for phone in value.split(",") if phone.strip()]

# MPU-6050 Constants
MPU6050_ADDR        = 0x68
# The MPU bus is detected automatically; use --i2c-bus to override it.
I2C_BUS             = 1
REG_PWR_MGMT_1      = 0x6B
REG_WHO_AM_I        = 0x75
REG_ACCEL_XOUT_H    = 0x3B
ACCEL_SCALE         = 16384.0   # Â±2 g
GYRO_SCALE          = 131.0     # Â±250 Â°/s
FALL_ACCEL_THRESHOLD = 2.5      # g â€” tune for real-world testing

# UART Defaults
DEFAULT_GPS_PORT = "/dev/serial0"
DEFAULT_GPS_BAUD = 9600
DEFAULT_GSM_PORT = "/dev/ttyAMA1"
DEFAULT_GSM_BAUD = 9600

# GPIO Pins (BCM Mode)
TRIG_PIN = 23
ECHO_PIN = 24
BUZZER_PIN = 17
MOTOR_PIN = 27

_running = True

def _signal_handler(sig, frame):
    global _running
    _running = False

signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


# â”€â”€ FastPiCamera (threaded BGR888 capture) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class FastPiCamera:
    """Threaded Picamera2 wrapper for low-latency BGR frame capture."""

    def __init__(self, width: int = 640, height: int = 480):
        try:
            from picamera2 import Picamera2
        except ImportError:
            sys.exit(
                "[ERROR] picamera2 is not installed.\n"
                "        Install it with:\n"
                "          sudo apt install -y python3-picamera2\n"
                "        Then recreate your venv:\n"
                "          python3 -m venv --system-site-packages venv"
            )

        self.camera = Picamera2()
        config = self.camera.create_preview_configuration(
            main={"size": (width, height), "format": "BGR888"}
        )
        self.camera.configure(config)
        self.frame_queue: Queue = Queue(maxsize=2)
        self.running = False

    def start(self):
        self.camera.start()
        time.sleep(1.0)
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        return self

    def _update(self):
        while self.running:
            if not self.frame_queue.full():
                frame = self.camera.capture_array()
                self.frame_queue.put(frame)

    def get_frame(self):
        if not self.frame_queue.empty():
            return self.frame_queue.get()
        return None

    def stop(self):
        self.running = False
        self.thread.join(timeout=2.0)
        self.camera.stop()


# â”€â”€ Environment checks â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

def _in_virtualenv() -> bool:
    return hasattr(sys, "real_prefix") or (sys.base_prefix != sys.prefix)

def _ensure_virtualenv() -> None:
    if not _in_virtualenv():
        print(
            "Warning: No virtual environment detected.\n"
            "         Recommended to run within venv."
        )

# def _install_dependencies() -> None:
#     missing = []
#     for pkg, imp in [
#         ("opencv-python", "cv2"),
#         ("onnxruntime", "onnxruntime"),
#         ("smbus2", "smbus2"),
#         ("pyserial", "serial"),
#         ("RPi.GPIO", "RPi.GPIO"),
#         ("pyttsx3", "pyttsx3"),
#         ("paho-mqtt>=2,<3", "paho.mqtt.client"),
#     ]:
#         try:
#             importlib.import_module(imp)
#         except ImportError:
#             missing.append(pkg)

#     if not missing:
#         return

#     subprocess.check_call(
#         [sys.executable, "-m", "pip", "install", "--quiet"] + missing
#     )


# â”€â”€ YOLO ONNX Runtime model loading â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class _OrtBox:
    """Small compatibility object matching the attributes used in main()."""

    def __init__(self, xyxy, confidence: float, class_id: int):
        self.xyxy = np.asarray([xyxy], dtype=np.float32)
        self.conf = np.asarray([confidence], dtype=np.float32)
        self.cls = np.asarray([class_id], dtype=np.float32)


class _OrtResult:
    """Ultralytics-like result wrapper so the rest of main.py stays unchanged."""

    def __init__(self, frame, detections, names):
        self._frame = frame
        self._detections = detections
        self.names = names
        self.boxes = [
            _OrtBox(det["xyxy"], det["confidence"], det["class_id"])
            for det in detections
        ]

    def plot(self):
        annotated = self._frame.copy()
        for det in self._detections:
            x1, y1, x2, y2 = [int(v) for v in det["xyxy"]]
            class_id = det["class_id"]
            confidence = det["confidence"]
            label = self.names.get(class_id, f"class_{class_id}")

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                f"{label} {confidence:.2f}",
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        return annotated


class OnnxYoloDetector:
    """Run Ultralytics-exported YOLO detection ONNX models without ultralytics."""

    def __init__(self, model_path: str):
        try:
            import onnxruntime as ort
        except ImportError:
            sys.exit(
                "[ERROR] onnxruntime is not installed.\n"
                "        Install it with: pip install onnxruntime"
            )

        self.model_path = str(model_path)
        self.session = ort.InferenceSession(
            self.model_path,
            providers=["CPUExecutionProvider"],
        )

        input_info = self.session.get_inputs()[0]
        self.input_name = input_info.name
        self.input_height, self.input_width = self._resolve_input_size(input_info.shape)
        self.names = self._load_names()

        print(
            f"[INFO] ONNX Runtime loaded {Path(self.model_path).name} "
            f"with input {self.input_width}x{self.input_height}."
        )
        print(f"[INFO] ONNX Runtime providers: {self.session.get_providers()}")
        if self.names:
            print(f"[INFO] Model classes: {self.names}")
        else:
            print(
                "[WARN] No class-name metadata found in the ONNX model. "
                "Detections will use class_0, class_1, ..."
            )

    def _metadata(self) -> dict:
        try:
            return dict(self.session.get_modelmeta().custom_metadata_map or {})
        except Exception:
            return {}

    def _load_names(self) -> dict[int, str]:
        raw = self._metadata().get("names", "")
        if not raw:
            return {}

        try:
            parsed = ast.literal_eval(raw)
        except Exception:
            return {}

        if isinstance(parsed, dict):
            result = {}
            for key, value in parsed.items():
                try:
                    result[int(key)] = str(value)
                except (TypeError, ValueError):
                    continue
            return result

        if isinstance(parsed, (list, tuple)):
            return {i: str(value) for i, value in enumerate(parsed)}

        return {}

    def _resolve_input_size(self, input_shape) -> tuple[int, int]:
        # Normal YOLO ONNX input is [batch, channels, height, width].
        if len(input_shape) >= 4:
            height, width = input_shape[-2], input_shape[-1]
            if isinstance(height, int) and isinstance(width, int):
                return int(height), int(width)

        # Dynamic exports often keep imgsz in Ultralytics ONNX metadata.
        raw_imgsz = self._metadata().get("imgsz", "")
        if raw_imgsz:
            try:
                parsed = ast.literal_eval(raw_imgsz)
                if isinstance(parsed, int):
                    return parsed, parsed
                if isinstance(parsed, (list, tuple)) and len(parsed) >= 2:
                    return int(parsed[0]), int(parsed[1])
            except Exception:
                pass

        print("[WARN] Dynamic ONNX input size detected; defaulting to 640x640.")
        return 640, 640

    def _letterbox(self, frame):
        original_h, original_w = frame.shape[:2]
        scale = min(
            self.input_width / original_w,
            self.input_height / original_h,
        )

        resized_w = int(round(original_w * scale))
        resized_h = int(round(original_h * scale))
        resized = cv2.resize(frame, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)

        pad_w = self.input_width - resized_w
        pad_h = self.input_height - resized_h
        left = pad_w // 2
        right = pad_w - left
        top = pad_h // 2
        bottom = pad_h - top

        padded = cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )

        # Picamera/OpenCV supplies BGR. YOLO expects RGB NCHW float32 in [0, 1].
        image = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        image = image.transpose(2, 0, 1)
        image = np.ascontiguousarray(image, dtype=np.float32) / 255.0
        image = np.expand_dims(image, axis=0)

        return image, scale, left, top

    @staticmethod
    def _looks_like_end_to_end_nms(predictions: np.ndarray) -> bool:
        """Detect [x1,y1,x2,y2,score,class] outputs from exports with NMS."""
        if predictions.ndim != 2 or predictions.shape[1] != 6:
            return False
        if predictions.shape[0] > 1000:
            return False
        if predictions.shape[0] == 0:
            return True

        scores = predictions[:, 4]
        classes = predictions[:, 5]
        valid_scores = np.all((scores >= -1e-4) & (scores <= 1.0001))
        integer_classes = np.mean(np.abs(classes - np.round(classes)) < 1e-3) > 0.90
        return bool(valid_scores and integer_classes)

    def _scale_xyxy(self, boxes: np.ndarray, scale: float, left: int, top: int, frame_shape):
        boxes = boxes.astype(np.float32, copy=True)
        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - left) / scale
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - top) / scale

        frame_h, frame_w = frame_shape[:2]
        boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, frame_w - 1)
        boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, frame_h - 1)
        return boxes

    def _decode(self, output, conf: float, iou: float, scale: float, left: int, top: int, frame_shape):
        prediction = np.asarray(output)

        # Remove batch dimensions such as [1, 84, 8400].
        while prediction.ndim > 2 and prediction.shape[0] == 1:
            prediction = prediction[0]

        if prediction.ndim != 2:
            raise RuntimeError(
                f"Unsupported YOLO ONNX output shape {np.asarray(output).shape}. "
                "Expected a 2-D detection tensor after removing the batch dimension."
            )

        # Ultralytics YOLOv8/YOLO11 commonly exports [4+classes, anchors].
        if prediction.shape[0] < prediction.shape[1] and prediction.shape[0] <= 512:
            prediction = prediction.T

        if prediction.size == 0:
            return []

        # Some ONNX exports include NMS and output [x1,y1,x2,y2,score,class].
        if self._looks_like_end_to_end_nms(prediction):
            mask = prediction[:, 4] >= conf
            prediction = prediction[mask]
            if len(prediction) == 0:
                return []

            boxes = self._scale_xyxy(
                prediction[:, :4], scale, left, top, frame_shape
            )
            return [
                {
                    "xyxy": boxes[i].tolist(),
                    "confidence": float(prediction[i, 4]),
                    "class_id": int(round(float(prediction[i, 5]))),
                }
                for i in range(len(prediction))
            ]

        columns = prediction.shape[1]
        class_count = len(self.names)

        # YOLOv8/YOLO11 raw detection output: [cx, cy, w, h, class_scores...]
        if class_count and columns == 4 + class_count:
            class_scores = prediction[:, 4:]
            class_ids = np.argmax(class_scores, axis=1)
            confidences = class_scores[np.arange(len(prediction)), class_ids]

        # Older YOLO export: [cx, cy, w, h, objectness, class_scores...]
        elif class_count and columns == 5 + class_count:
            objectness = prediction[:, 4]
            class_scores = prediction[:, 5:]
            class_ids = np.argmax(class_scores, axis=1)
            confidences = objectness * class_scores[np.arange(len(prediction)), class_ids]

        else:
            # For YOLOv8/YOLO11 models whose class-name metadata was stripped.
            if columns <= 4:
                raise RuntimeError(
                    f"Unsupported raw ONNX output with {columns} columns."
                )
            class_scores = prediction[:, 4:]
            class_ids = np.argmax(class_scores, axis=1)
            confidences = class_scores[np.arange(len(prediction)), class_ids]

            if not self.names:
                inferred_classes = class_scores.shape[1]
                self.names = {i: f"class_{i}" for i in range(inferred_classes)}

        keep = confidences >= conf
        prediction = prediction[keep]
        confidences = confidences[keep]
        class_ids = class_ids[keep]

        if len(prediction) == 0:
            return []

        # Convert model-space center-width-height boxes to xyxy.
        cx = prediction[:, 0]
        cy = prediction[:, 1]
        width = prediction[:, 2]
        height = prediction[:, 3]
        boxes = np.stack(
            [
                cx - width / 2,
                cy - height / 2,
                cx + width / 2,
                cy + height / 2,
            ],
            axis=1,
        )
        boxes = self._scale_xyxy(boxes, scale, left, top, frame_shape)

        # OpenCV NMS expects [x, y, width, height].
        nms_boxes = []
        for x1, y1, x2, y2 in boxes:
            nms_boxes.append(
                [float(x1), float(y1), float(x2 - x1), float(y2 - y1)]
            )

        indexes = cv2.dnn.NMSBoxes(
            nms_boxes,
            confidences.astype(float).tolist(),
            float(conf),
            float(iou),
        )

        if indexes is None or len(indexes) == 0:
            return []

        indexes = np.asarray(indexes).reshape(-1)
        detections = []
        for index in indexes:
            index = int(index)
            detections.append(
                {
                    "xyxy": boxes[index].tolist(),
                    "confidence": float(confidences[index]),
                    "class_id": int(class_ids[index]),
                }
            )
        return detections

    def predict(self, frame, conf: float = 0.45, verbose: bool = False, iou: float = 0.45):
        del verbose  # Accepted only for compatibility with the old call site.
        tensor, scale, left, top = self._letterbox(frame)
        outputs = self.session.run(None, {self.input_name: tensor})

        if not outputs:
            raise RuntimeError("ONNX Runtime returned no outputs from the model.")

        detections = self._decode(
            outputs[0],
            conf=float(conf),
            iou=float(iou),
            scale=scale,
            left=left,
            top=top,
            frame_shape=frame.shape,
        )
        return [_OrtResult(frame, detections, self.names)]


def _resolve_model_path(model_path: str) -> str:
    path = Path(model_path)

    if path.exists():
        if path.suffix.lower() != ".onnx":
            sys.exit(
                f"[ERROR] This Raspberry Pi runtime is ONNX-only. "
                f"Expected a .onnx model, got: {path.name}"
            )
        return str(path)

    normalized = model_path.lower().replace(".pt", "").replace(".onnx", "")
    candidate = PROJECT_DIR / f"{normalized}.onnx"
    if candidate.exists():
        return str(candidate)

    onnx_files = sorted(PROJECT_DIR.glob("*.onnx"))
    if len(onnx_files) == 1:
        print(f"[INFO] Using the only ONNX model found: {onnx_files[0].name}")
        return str(onnx_files[0])

    if onnx_files:
        available = ", ".join(path.name for path in onnx_files)
        sys.exit(
            f"[ERROR] Could not find requested model '{model_path}'. "
            f"Available ONNX models: {available}. "
            "Choose one with --model filename.onnx"
        )

    sys.exit(
        f"[ERROR] No ONNX model found in {PROJECT_DIR}. "
        "Copy your exported .onnx model into the project directory."
    )


def load_model(model_path: str):
    """Load a YOLO .onnx model directly with ONNX Runtime."""
    path = _resolve_model_path(model_path)
    return OnnxYoloDetector(path)


def _find_model(cli_args: list[str]) -> str:
    model_path = DEFAULT_MODEL
    for i, arg in enumerate(cli_args):
        if arg == "--model" and i + 1 < len(cli_args):
            model_path = cli_args[i + 1]
            break
    return model_path


def _open_mpu_bus(bus_setting: str):
    """Open the configured bus, or find the bus whose MPU identifies correctly."""
    import smbus2

    if bus_setting.lower() != "auto":
        bus_number = int(bus_setting)
        return smbus2.SMBus(bus_number), bus_number

    candidates = sorted(
        int(path.name.rsplit("-", 1)[1])
        for path in Path("/dev").glob("i2c-*")
        if path.name.rsplit("-", 1)[-1].isdigit()
    )
    for bus_number in candidates:
        bus = None
        found = False
        try:
            bus = smbus2.SMBus(bus_number)
            if bus.read_byte_data(MPU6050_ADDR, REG_WHO_AM_I) in (0x68, 0x69):
                found = True
                return bus, bus_number
        except Exception:
            pass
        finally:
            if bus is not None and not found:
                bus.close()
    raise FileNotFoundError(
        "MPU-6050 was not found on any I2C adapter. "
        f"Checked: {', '.join(f'/dev/i2c-{number}' for number in candidates) or 'none'}. "
        "Enable I2C and check wiring; use --i2c-bus N to select a bus explicitly."
    )


# â”€â”€ Sensor Threads â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

def _mpu_thread(
    fall_queue: queue.Queue,
    step_queue: queue.Queue,
    poll_hz: float = 10.0,
    bus_setting: str = "auto",
    fall_threshold_provider=None,
) -> None:
    try:
        import smbus2
    except ImportError:
        return

    try:
        bus, bus_number = _open_mpu_bus(bus_setting)
        bus.write_byte_data(MPU6050_ADDR, REG_PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        print(f"[INFO] MPU-6050 using /dev/i2c-{bus_number}.")
    except Exception as exc:
        print(f"[WARN] MPU-6050 unavailable: {exc}")
        return

    interval = 1.0 / poll_hz
    step_counter = StepCounter()
    while _running:
        try:
            data = bus.read_i2c_block_data(MPU6050_ADDR, REG_ACCEL_XOUT_H, 14)
            raw = struct.unpack(">hhhhhhh", bytes(data))

            ax, ay, az = raw[0]/ACCEL_SCALE, raw[1]/ACCEL_SCALE, raw[2]/ACCEL_SCALE
            accel_mag = (ax**2 + ay**2 + az**2) ** 0.5

            if step_counter.update(accel_mag, time.monotonic()):
                try:
                    step_queue.put_nowait({"steps": step_counter.count, "timestamp": time.time()})
                except queue.Full:
                    pass

            threshold = (
                float(fall_threshold_provider())
                if fall_threshold_provider is not None
                else FALL_ACCEL_THRESHOLD
            )
            if accel_mag > threshold:
                try:
                    fall_queue.put_nowait({"type": "fall", "accel_mag": accel_mag, "timestamp": time.time()})
                except queue.Full:
                    pass
        except Exception as exc:
            pass
        time.sleep(interval)
    bus.close()

def _gps_thread(gps_queue: queue.Queue, port: str, baud: int) -> None:
    try:
        import serial as pyserial
    except ImportError:
        print("[WARN] GPS unavailable: pyserial is not installed.", flush=True)
        return
    try:
        ser = pyserial.Serial(port, baud, timeout=1.0)
    except Exception as exc:
        print(f"[WARN] GPS unavailable on {port} at {baud} baud: {exc}", flush=True)
        return

    print(f"[INFO] GPS listening on {port} at {baud} baud.", flush=True)
    nmea_sentences = 0
    while _running:
        try:
            raw_line = ser.readline()
            if not raw_line: continue
            line = raw_line.decode("ascii", errors="ignore").strip()
            if not line.startswith("$"): continue

            nmea_sentences += 1
            fields = line.split(",")
            msg_type = fields[0]
            if msg_type in ("$GPGGA", "$GNGGA") and len(fields) >= 10:
                if int(fields[6] or "0") > 0:
                    try:
                        lat_deg = float(fields[2][:2]) + float(fields[2][2:])/60.0
                        lon_deg = float(fields[4][:3]) + float(fields[4][3:])/60.0
                        if fields[3] in ('S', 'W'): lat_deg = -lat_deg
                        if fields[5] in ('S', 'W'): lon_deg = -lon_deg
                        gps_queue.put_nowait({
                            "latitude": lat_deg,
                            "longitude": lon_deg,
                            "fix_at": time.time(),
                        })
                    except (ValueError, IndexError, queue.Full):
                        pass
                elif nmea_sentences % 20 == 0:
                    print(
                        f"[INFO] GPS data received on {port}, but there is no satellite fix yet.",
                        flush=True,
                    )
        except Exception as exc:
            print(f"[WARN] GPS read error on {port}: {exc}", flush=True)
            time.sleep(1.0)
    ser.close()

def _ultrasonic_thread(us_queue: queue.Queue) -> None:
    """Polls the HC-SR04 in a background thread."""
    import RPi.GPIO as GPIO
    while _running:
        try:
            GPIO.output(TRIG_PIN, True)
            time.sleep(0.00001)
            GPIO.output(TRIG_PIN, False)

            start = time.time()
            while GPIO.input(ECHO_PIN) == 0 and _running:
                if time.time() - start > 0.1:
                    break
            pulse_start = time.time()

            while GPIO.input(ECHO_PIN) == 1 and _running:
                if time.time() - start > 0.1:
                    break
            pulse_end = time.time()

            if pulse_end > pulse_start:
                distance = (pulse_end - pulse_start) * 17150
                try:
                    if us_queue.full():
                        us_queue.get_nowait()
                    us_queue.put_nowait(distance)
                except queue.Full:
                    pass
        except Exception:
            pass
        time.sleep(0.2)  # 5Hz poll rate; sensor thread only

def _gsm_thread(sms_queue: queue.Queue, port: str, baud: int, phones: list[str]) -> None:
    """Send each queued alert to every configured number with reconnects."""
    try:
        import serial as pyserial
    except ImportError:
        return
        
    while _running:
        try:
            msg = sms_queue.get(timeout=1.0)
            for phone in phones:
                for attempt in range(2):
                    ser = None
                    try:
                        ser = pyserial.Serial(port, baud, timeout=2.0, write_timeout=2.0)
                        ser.write(b"AT\r\n")
                        ser.write(b"AT+CMGF=1\r\n")
                        time.sleep(1)
                        ser.write((f'AT+CMGS="{phone}"\r\n').encode("ascii"))
                        time.sleep(1)
                        ser.write((msg + "\x1a").encode("ascii", errors="replace"))
                        time.sleep(3)
                        print(f"SMS alert sent to {phone}.")
                        break
                    except Exception as exc:
                        print(f"[WARN] GSM send failed for {phone}: {exc}")
                        if attempt == 1:
                            print(f"[WARN] Giving up on this SMS for {phone}.")
                    finally:
                        if ser:
                            ser.close()
                if not _running:
                    break
        except queue.Empty:
            pass
        except Exception as exc:
            pass


# â”€â”€ Main loop â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

def main() -> None:
    global _running
    parser = argparse.ArgumentParser(description="Walking Stick Integrated System")
    parser.add_argument("--model", type=str, default=DEFAULT_MODEL)
    parser.add_argument("--skip", type=int, default=2)
    parser.add_argument("--conf", type=float, default=0.45)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument(
        "--display",
        action="store_true",
        help="Enable the live OpenCV window (headless by default)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without a display window (default)",
    )
    parser.add_argument("--no-mpu", action="store_true")
    parser.add_argument("--no-gps", action="store_true")
    parser.add_argument("--no-us", action="store_true")
    parser.add_argument("--no-gsm", action="store_true")
    parser.add_argument("--no-alerts", action="store_true")
    parser.add_argument("--no-tts", action="store_true", help="Disable spoken announcements")
    parser.add_argument("--tts-rate", type=int, default=165, help="Speech rate (words per minute)")
    parser.add_argument("--tts-volume", type=float, default=1.0, help="Speech volume from 0.0 to 1.0")
    parser.add_argument(
        "--audio-device",
        default=os.getenv("WALKING_STICK_AUDIO_DEVICE"),
        help="Optional PipeWire sink ID or ALSA device; otherwise detect automatically",
    )
    parser.add_argument(
        "--tts-objects",
        type=str,
        default=",".join(sorted(DEFAULT_TTS_OBJECTS)),
        help="Comma-separated YOLO classes to announce",
    )
    parser.add_argument(
        "--gps-port",
        type=str,
        default=os.getenv("WALKING_STICK_GPS_PORT", DEFAULT_GPS_PORT),
    )
    parser.add_argument(
        "--gps-baud",
        type=int,
        default=int(os.getenv("WALKING_STICK_GPS_BAUD", str(DEFAULT_GPS_BAUD))),
    )
    parser.add_argument("--gsm-port", type=str, default=DEFAULT_GSM_PORT)
    parser.add_argument("--gsm-baud", type=int, default=DEFAULT_GSM_BAUD)
    parser.add_argument("--mpu-hz", type=float, default=10.0)
    parser.add_argument(
        "--i2c-bus",
        default=os.getenv("WALKING_STICK_I2C_BUS", "auto"),
        help="MPU I2C bus number or auto (default: auto)",
    )
    parser.add_argument(
        "--phone",
        type=str,
        default=os.getenv("WALKING_STICK_PHONE_NUMBERS", ""),
        help="Comma-separated GSM recipients, e.g. +15551234567,+15557654321",
    )
    parser.add_argument(
        "--backend-url",
        type=str,
        default=os.getenv("WALKING_STICK_API_URL", ""),
        help="HTTP POST endpoint for events (Supabase, Firebase function, or custom API)",
    )
    parser.add_argument(
        "--backend-token",
        type=str,
        default=os.getenv("WALKING_STICK_API_TOKEN", ""),
        help="Optional bearer/API token for the backend",
    )
    parser.add_argument(
        "--device-id",
        type=str,
        default=os.getenv("WALKING_STICK_DEVICE_ID", "walking-stick"),
    )
    parser.add_argument(
        "--backend-spool",
        type=str,
        default=str(PROJECT_DIR / "backend_events.jsonl"),
        help="File for events that cannot be delivered yet",
    )
    parser.add_argument("--gps-interval", type=float, default=300.0, help="GPS upload interval in seconds")
    parser.add_argument(
        "--fallback-latitude",
        type=float,
        default=float(os.getenv("WALKING_STICK_FALLBACK_LATITUDE", "9.53101")),
        help="Startup latitude used until the GPS obtains its first valid fix",
    )
    parser.add_argument(
        "--fallback-longitude",
        type=float,
        default=float(os.getenv("WALKING_STICK_FALLBACK_LONGITUDE", "6.4469")),
        help="Startup longitude used until the GPS obtains its first valid fix",
    )
    args = parser.parse_args()
    if args.skip < 1:
        parser.error("--skip must be at least 1")
    if args.gps_interval <= 0:
        parser.error("--gps-interval must be greater than 0")
    fallback_location_configured = (
        -90.0 <= args.fallback_latitude <= 90.0
        and -180.0 <= args.fallback_longitude <= 180.0
    )
    display_enabled = args.display and not args.headless
    tts_objects = {
        object_name.strip().lower()
        for object_name in args.tts_objects.split(",")
        if object_name.strip()
    }
    phones = parse_phone_numbers(args.phone)

    _ensure_virtualenv()
    # _install_dependencies()

    try:
        mqtt_bridge = WakatechMqttBridge().start()
    except Exception as exc:
        sys.exit(f"[ERROR] MQTT configuration could not be loaded: {exc}")

    sensitivity_thresholds = {
        "low": float(os.getenv("WAKATECH_FALL_THRESHOLD_LOW", "3.0")),
        "medium": float(os.getenv("WAKATECH_FALL_THRESHOLD_MEDIUM", str(FALL_ACCEL_THRESHOLD))),
        "high": float(os.getenv("WAKATECH_FALL_THRESHOLD_HIGH", "2.0")),
    }

    def active_fall_threshold() -> float:
        sensitivity = mqtt_bridge.settings()["fallSensitivity"]
        return sensitivity_thresholds.get(sensitivity, FALL_ACCEL_THRESHOLD)

    try:
        import RPi.GPIO as GPIO
    except ImportError:
        sys.exit("RPi.GPIO required for alerts/ultrasonic. Run: pip install RPi.GPIO")

    # Setup GPIO for Alerts and Ultrasonic
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    if not args.no_alerts:
        GPIO.setup(BUZZER_PIN, GPIO.OUT)
        GPIO.setup(MOTOR_PIN, GPIO.OUT)
        GPIO.output(BUZZER_PIN, False)
        GPIO.output(MOTOR_PIN, False)
    if not args.no_us:
        GPIO.setup(TRIG_PIN, GPIO.OUT)
        GPIO.setup(ECHO_PIN, GPIO.IN)
        GPIO.output(TRIG_PIN, False)

    model_path = _find_model(sys.argv[1:])
    model = load_model(model_path)

    camera = FastPiCamera(args.width, args.height).start()

    fall_queue = queue.Queue(maxsize=16)
    gps_queue  = queue.Queue(maxsize=16)
    backend_location_queue = queue.Queue(maxsize=2)
    us_queue   = queue.Queue(maxsize=2)
    sms_queue  = queue.Queue(maxsize=4)
    speech = SpeechManager(
        enabled=not args.no_tts,
        rate=args.tts_rate,
        volume=args.tts_volume,
        device=args.audio_device,
    ).start()
    backend = BackendClient(
        url=args.backend_url,
        token=args.backend_token,
        device_id=args.device_id,
        spool_path=args.backend_spool,
        location_queue=backend_location_queue,
    ).start()
    backend.request_last_location()
    last_backend_alert: dict[str, float] = {}

    def publish_alert(alert_type: str, payload: dict, cooldown: float = 5.0, priority: str = "normal") -> bool:
        now = time.monotonic()
        if now - last_backend_alert.get(alert_type, -float("inf")) < cooldown:
            return False
        last_backend_alert[alert_type] = now
        backend.publish("alert", {"alert_type": alert_type, **payload}, priority=priority)
        return True

    def queue_sms(message: str) -> None:
        if not phones:
            return
        try:
            sms_queue.put_nowait(message)
        except queue.Full:
            print("[WARN] GSM alert queue full; SMS alert was not queued.")

    threads = []

    if not args.no_mpu:
        step_queue = queue.Queue(maxsize=4)
        t_mpu = threading.Thread(
            target=_mpu_thread,
            args=(fall_queue, step_queue, args.mpu_hz, args.i2c_bus, active_fall_threshold),
            daemon=True,
        )
        t_mpu.start()
        threads.append(t_mpu)

    if not args.no_gps:
        t_gps = threading.Thread(target=_gps_thread, args=(gps_queue, args.gps_port, args.gps_baud), daemon=True)
        t_gps.start()
        threads.append(t_gps)

    if not args.no_us:
        t_us = threading.Thread(target=_ultrasonic_thread, args=(us_queue,), daemon=True)
        t_us.start()
        threads.append(t_us)

    if not args.no_gsm and phones:
        print(f"GSM alerts enabled for {len(phones)} recipient(s).")
        t_gsm = threading.Thread(target=_gsm_thread, args=(sms_queue, args.gsm_port, args.gsm_baud, phones), daemon=True)
        t_gsm.start()
        threads.append(t_gsm)

    window_name = "Walking Stick Vision"
    frame_id = 0

    fall_active = False
    fall_clear_time = 0.0
    
    alert_active = False
    alert_clear_time = 0.0
    
    # Track states to prevent TTS spam
    us_alert_spoken = False
    last_lat = args.fallback_latitude if fallback_location_configured else 0.0
    last_lon = args.fallback_longitude if fallback_location_configured else 0.0
    us_distance = 999.0
    steps_taken = 0
    heartbeat_interval = float(os.getenv("WAKATECH_HEARTBEAT_INTERVAL", "60"))
    activity_interval = float(os.getenv("WAKATECH_ACTIVITY_INTERVAL", "300"))
    walk_idle_seconds = float(os.getenv("WAKATECH_WALK_IDLE_SECONDS", "120"))
    next_mqtt_heartbeat = time.monotonic()
    next_mqtt_activity = time.monotonic() + activity_interval
    pending_fall: dict | None = None
    walk_started_at: str | None = None
    walk_start_steps = 0
    last_step_at: float | None = None
    # Send startup/fallback telemetry immediately instead of waiting one interval.
    next_gps_upload = time.monotonic()
    last_gps_at = time.time() if fallback_location_configured else None
    last_location_source = "raspberry_pi_fallback" if fallback_location_configured else None
    gps_locked = False
    if fallback_location_configured:
        print(
            f"Using Raspberry Pi fallback location: latitude {last_lat:.6f}, "
            f"longitude {last_lon:.6f}."
        )
    
    OBSTACLE_CLASSES = {'pothole', 'bollard', 'stairs', 'car_placeholder'} 

    if display_enabled:
        print("System active. Press 'q' in window to exit.")
    else:
        print("System active in headless mode. Press Ctrl-C to exit.")

    try:
        while _running:
            frame = camera.get_frame()
            if frame is not None:
                frame_id += 1

            # MQTT and sensor processing must continue even if the camera has
            # not produced a frame yet.
            now_monotonic = time.monotonic()

            # â”€â”€ Drain GPS â”€â”€
            while not gps_queue.empty():
                loc = gps_queue.get_nowait()
                coordinates_changed = (
                    abs(loc["latitude"] - last_lat) > 0.00001
                    or abs(loc["longitude"] - last_lon) > 0.00001
                )
                last_lat, last_lon = loc["latitude"], loc["longitude"]
                last_gps_at = loc["fix_at"]
                last_location_source = "gps"
                if not gps_locked:
                    print("GPS lock acquired; GPS location now has priority.")
                gps_locked = True
                next_gps_upload = time.monotonic()
                if coordinates_changed:
                    print(f"Location updated to latitude {last_lat:.6f} and longitude {last_lon:.6f}.")

            while not backend_location_queue.empty():
                loc = backend_location_queue.get_nowait()
                if not gps_locked:
                    last_lat, last_lon = loc["latitude"], loc["longitude"]
                    last_gps_at = loc.get("fix_at")
                    last_location_source = "backend"
                    next_gps_upload = time.monotonic()
                    print(
                        f"Using last known location from backend: "
                        f"latitude {last_lat:.6f}, longitude {last_lon:.6f}."
                    )

            if not args.no_mpu:
                while not step_queue.empty():
                    new_step_count = step_queue.get_nowait()["steps"]
                    if new_step_count != steps_taken:
                        if walk_started_at is None:
                            walk_started_at = utc_timestamp()
                            walk_start_steps = steps_taken
                        last_step_at = time.monotonic()
                        steps_taken = new_step_count
                        print(f"Steps: {steps_taken}", flush=True)

            if mqtt_bridge.enabled and now_monotonic >= next_mqtt_heartbeat:
                location_payload = None
                if last_gps_at is not None:
                    location_payload = mqtt_location(last_lat, last_lon, last_gps_at)
                mqtt_bridge.heartbeat(
                    current_battery_percent(),
                    current_wifi_rssi(),
                    location_payload,
                )
                next_mqtt_heartbeat = now_monotonic + heartbeat_interval

            if mqtt_bridge.enabled and now_monotonic >= next_mqtt_activity:
                mqtt_bridge.activity(steps_taken)
                next_mqtt_activity = now_monotonic + activity_interval

            if (
                mqtt_bridge.enabled
                and walk_started_at is not None
                and last_step_at is not None
                and now_monotonic - last_step_at >= walk_idle_seconds
            ):
                mqtt_bridge.walk(
                    walk_started_at,
                    utc_timestamp(),
                    steps_taken - walk_start_steps,
                )
                walk_started_at = None
                last_step_at = None

            if now_monotonic >= next_gps_upload:
                backend.request_last_location()
                backend.publish(
                    "telemetry",
                    {
                        "gps": {
                            "latitude": last_lat if last_gps_at is not None else None,
                            "longitude": last_lon if last_gps_at is not None else None,
                            "fix_at": last_gps_at,
                        },
                        "steps_since_boot": steps_taken,
                        "location_source": last_location_source or "none",
                    },
                )
                next_gps_upload = now_monotonic + args.gps_interval

            # â”€â”€ Drain Ultrasonic â”€â”€
            while not us_queue.empty():
                us_distance = us_queue.get_nowait()

            # â”€â”€ Check Fall â”€â”€
            while not fall_queue.empty():
                alert = fall_queue.get_nowait()
                if pending_fall is not None:
                    continue
                fall_active = True
                fall_clear_time = time.time() + 3.0
                grace_seconds = 0
                pending_fall = {
                    "publish_at": time.monotonic(),
                    "accel_mag": alert.get("accel_mag"),
                }
                
                print("Warning: Fall detected. Initiating emergency protocols.")
                speech.announce(
                    "Fall detected. Emergency assistance is being contacted.",
                    key="fall",
                    cooldown=5.0,
                    priority=0,
                )
                publish_alert(
                    "fall",
                    {"accel_mag": alert.get("accel_mag"), "steps_since_boot": steps_taken},
                    cooldown=5.0,
                    priority="critical",
                )
                
                queue_sms(f"EMERGENCY! Fall detected. Lat: {last_lat:.6f}, Lon: {last_lon:.6f}")

            if (
                mqtt_bridge.enabled
                and pending_fall is not None
                and now_monotonic >= pending_fall["publish_at"]
            ):
                if last_gps_at is None:
                    print("[WARN] MQTT fall event is waiting for a valid GPS/fallback location.")
                    pending_fall["publish_at"] = now_monotonic + 5.0
                else:
                    mqtt_bridge.fall(
                        "critical",
                        current_battery_percent(),
                        mqtt_location(last_lat, last_lon, last_gps_at),
                    )
                    pending_fall = None

            if fall_active and time.time() > fall_clear_time:
                fall_active = False

            if frame is None:
                time.sleep(0.01)
                continue

            # â”€â”€ Skip frames for performance â”€â”€
            if frame_id % args.skip != 0:
                if display_enabled:
                    cv2.putText(
                        frame,
                        f"Steps: {steps_taken}",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.imshow(window_name, frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                continue

            # â”€â”€ Run YOLO â”€â”€
            results = model.predict(frame, conf=args.conf, verbose=False)
            annotated_frame = results[0].plot() if display_enabled else None

            # â”€â”€ Vision output and obstacle detection â”€â”€
            yolo_obstacle_detected = False
            if results[0].boxes is not None and len(results[0].boxes) > 0:
                unique_classes = set()
                for box in results[0].boxes:
                    cls_id = int(box.cls[0])
                    label = results[0].names[cls_id]
                    unique_classes.add(label)
                    if label.lower() in OBSTACLE_CLASSES:
                        yolo_obstacle_detected = True

                for cls_name in unique_classes:
                    print(f"Object detected: {cls_name}.")
                    if cls_name.lower() in tts_objects:
                        speech.announce(
                            f"{cls_name} detected.",
                            key=f"vision:{cls_name.lower()}",
                            cooldown=8.0,
                            priority=10,
                        )
                    alert_queued = publish_alert(
                        f"vision_object:{cls_name.lower()}",
                        {"object": cls_name, "steps_since_boot": steps_taken},
                        cooldown=8.0,
                    )
                    if alert_queued and cls_name.lower() in tts_objects:
                        queue_sms(f"Walking stick alert: {cls_name} detected.")

            # â”€â”€ Ultrasonic Alert â”€â”€
            if us_distance < 50.0:
                if not us_alert_spoken:
                    print(f"Obstacle detected {int(us_distance)} centimeters ahead.")
                    speech.announce(
                        f"Obstacle detected {int(us_distance)} centimeters ahead.",
                        key="ultrasonic-obstacle",
                        cooldown=5.0,
                        priority=5,
                    )
                    publish_alert(
                        "ultrasonic_obstacle",
                        {"distance_cm": round(us_distance, 1), "steps_since_boot": steps_taken},
                        cooldown=5.0,
                    )
                    queue_sms(f"Walking stick alert: obstacle {int(us_distance)} cm ahead.")
                    us_alert_spoken = True
            else:
                us_alert_spoken = False

            # â”€â”€ Trigger Hardware Alerts (Buzzer/Motor) â”€â”€
            if us_distance < 50.0 or yolo_obstacle_detected:
                alert_active = True
                alert_clear_time = time.time() + 0.5
            if alert_active and time.time() > alert_clear_time:
                alert_active = False

            if not args.no_alerts:
                GPIO.output(BUZZER_PIN, alert_active)
                GPIO.output(MOTOR_PIN, alert_active)

            # â”€â”€ Display â”€â”€
            if display_enabled:
                cv2.putText(
                    annotated_frame,
                    f"Steps: {steps_taken}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow(window_name, annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        _running = False
        camera.stop()
        if display_enabled:
            cv2.destroyAllWindows()
        
        if not args.no_alerts or not args.no_us:
            GPIO.cleanup()

        for t in threads:
            t.join(timeout=2.0)
        speech.stop()
        backend.stop()
        if mqtt_bridge.enabled and walk_started_at is not None and steps_taken > walk_start_steps:
            mqtt_bridge.walk(walk_started_at, utc_timestamp(), steps_taken - walk_start_steps)
        mqtt_bridge.stop()

if __name__ == "__main__":
    main()
